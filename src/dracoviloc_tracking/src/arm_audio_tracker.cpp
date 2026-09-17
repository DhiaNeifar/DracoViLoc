#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "odas_ros_msgs/msg/odas_sst_array_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ruckig/ruckig.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace fs = std::filesystem;

// =============================================================================
// arm_audio_tracker  -  audio servo driven by the EKF fused bearing
//
// WHAT CHANGED AND WHY
// ====================
// Previously this node consumed /audio/target_direction, a unit vector in the
// world frame published by audio_target_tracker.py from raw /sst peaks. That
// path had no classification gate, no motion model and no outlier rejection:
// every acoustic peak drove the arm, reflections included.
//
// It now consumes /fused_target_pose from the EKF, which is gated on the AST
// and GRE classifiers, smoothed by a constant angular velocity model, and
// protected by chi-squared rejection of bearings inconsistent with the track.
// When the vision team publishes /camera/yolo_detection, the EKF folds it in
// and this node benefits without any change here.
//
// THREE CONVERSIONS THIS NODE NOW OWNS
// ====================================
// The old topic carried a unit vector already expressed in `world`. The EKF
// publishes something different, so the work audio_target_tracker.py used to
// do has moved in here:
//
//   1. ANGLES TO VECTOR. /fused_target_pose is a PointStamped where x is
//      azimuth and y is elevation IN RADIANS, and z is a constant 1.0 marking
//      a unit direction. It is NOT a Cartesian point. Reading point.x/y/z as
//      coordinates yields a vector pointing nowhere near the target.
//
//   2. FRAME ROTATION. The angles are in the EKF's tracking frame (the
//      microphone frame), not in `world`. A TF lookup rotates the direction.
//      This replaces the old `frame_id != "world"` guard, which silently
//      dropped anything not already in world.
//
//   3. VALIDITY BY TIMEOUT. The EKF has no /audio/target_valid equivalent; it
//      simply stops publishing when it has nothing. Freshness of the last
//      message now stands in for that flag. The legacy Bool subscription is
//      kept so an external supervisor can still force a stop.
//
// The exponential smoothing is deliberately kept even though the EKF already
// filters. It runs on the WORLD-frame vector, so it also absorbs jitter
// introduced by the TF lookup itself, which the EKF cannot see. Set
// smoothing_alpha to 1.0 to disable it and follow the EKF exactly.
// =============================================================================

class ArmAudioTracker : public rclcpp::Node {
public:
  ArmAudioTracker() : Node("arm_audio_tracker") {
    target_timeout_ = declare_parameter("target_timeout", 0.75);
    smoothing_alpha_ = declare_parameter("smoothing_alpha", 0.20);
    angular_deadband_ = declare_parameter("angular_deadband", 0.08);
    angular_deadband_exit_ = declare_parameter("angular_deadband_exit", 0.04);
    motion_penalty_ = declare_parameter("motion_penalty", 0.015);
    // command_horizon is retained as a declared parameter so older launch
    // commands remain valid. Tracking no longer sends short trajectories.
    command_horizon_ = declare_parameter("command_horizon", 0.20);
    command_rate_hz_ = declare_parameter("command_rate_hz", 100.0);
    max_velocity_ = declare_parameter("max_velocity", 0.60);
    max_acceleration_ = declare_parameter("max_acceleration", 0.80);
    max_jerk_ = declare_parameter("max_jerk", 4.0);
    max_tracking_error_ = declare_parameter("max_tracking_error", 0.35);
    world_frame_ = declare_parameter("world_frame", std::string("world"));
    // Fallback only. The frame actually used is the one stamped on each
    // incoming message, so this matters only if the EKF ships an empty
    // frame_id.
    tracking_frame_ = declare_parameter(
      "tracking_frame", std::string("table_mic_link"));
    ekf_enabled_ = declare_parameter("ekf_enabled", true);
    direct_classifier_source_ = declare_parameter(
      "direct_classifier_source", std::string("gre"));
    direct_min_activity_ = declare_parameter("direct_min_activity", 0.10);
    direct_class_timeout_ = declare_parameter("direct_class_timeout", 5.0);
    ekf_direction_log_path_ = declare_parameter(
      "ekf_direction_log_path", std::string(""));
    yolo_direction_log_path_ = declare_parameter(
      "yolo_direction_log_path", std::string(""));
    require_home_ = declare_parameter("require_home_before_tracking", true);
    home_duration_s_ = declare_parameter("home_duration_s", 12.0);
    home_tolerance_rad_ = declare_parameter("home_tolerance_rad", 0.035);
    if (angular_deadband_exit_ < 0.0 ||
        angular_deadband_exit_ >= angular_deadband_) {
      throw std::invalid_argument(
        "angular_deadband_exit must be non-negative and smaller than angular_deadband");
    }
    if (command_horizon_ <= 0.0 || max_velocity_ <= 0.0 ||
        max_acceleration_ <= 0.0 || max_jerk_ <= 0.0 || max_tracking_error_ <= 0.0) {
      throw std::invalid_argument(
        "command_horizon, max_velocity, max_acceleration, max_jerk, and "
        "max_tracking_error must be positive");
    }
    if (command_rate_hz_ < 20.0 || command_rate_hz_ > 200.0) {
      throw std::invalid_argument("command_rate_hz must be between 20 and 200");
    }
    if (direct_classifier_source_ != "gre" &&
        direct_classifier_source_ != "ast" &&
        direct_classifier_source_ != "either" &&
        direct_classifier_source_ != "yolo") {
      throw std::invalid_argument(
        "direct_classifier_source must be gre, ast, either, or yolo");
    }
    open_ekf_direction_log();
    open_yolo_direction_log();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
    servo_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_tracking_controller/commands", rclcpp::SensorDataQoS());
    switch_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller", rmw_qos_profile_services_default,
      switch_callback_group_);
    otg_ = std::make_unique<ruckig::Ruckig<6>>(1.0 / command_rate_hz_);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {joint_callback(*msg);});

    if (ekf_enabled_) {
      fused_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/ekf/direction", 10,
        [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
          direction_callback(*msg);
        });
      if (yolo_direction_log_.is_open()) {
        yolo_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
          "/yolo/direction", 10,
          [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
            log_yolo_direction(*msg);
          });
      }
    } else {
      const auto subscribe = [this](const std::string & topic, auto & sub) {
        sub = create_subscription<geometry_msgs::msg::Vector3Stamped>(topic, 10,
          [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {direction_callback(*msg);});
      };
      if (direct_classifier_source_ == "yolo") subscribe("/yolo/direction", yolo_sub_);
      if (direct_classifier_source_ == "ast" || direct_classifier_source_ == "either") subscribe("/ast/direction", ast_sub_);
      if (direct_classifier_source_ == "gre" || direct_classifier_source_ == "either") subscribe("/gre/direction", gre_sub_);
    }

    // Kept for compatibility: an external supervisor can still veto motion.
    // Nothing in the EKF chain publishes it, so it defaults to permitting.
    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/audio/target_valid", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        external_veto_ = !msg->data;
      });
    home_service_ = create_service<std_srvs::srv::Trigger>(
      "/demo/home",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        request_home(*response);
      });
    tracking_service_ = create_service<std_srvs::srv::SetBool>(
      "/demo/tracking",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        set_tracking(request->data, *response);
      });
    const auto command_period = std::chrono::milliseconds(
      static_cast<int64_t>(std::lround(1000.0 / command_rate_hz_)));
    timer_ = create_wall_timer(command_period, [this]() {update();});
    RCLCPP_INFO(get_logger(), "Jerk-limited position servo at %.1f Hz in %s mode%s; "
      "tracking starts disabled. Use /demo/home, then /demo/tracking true. "
      "joint1/joint4 track; joint2/joint3/joint5/joint6 remain locked",
      command_rate_hz_,
      ekf_enabled_ ? "EKF" : "direct",
      ekf_enabled_ ? "" : (" (source=" + direct_classifier_source_ + ")").c_str());
  }

private:
  enum class ControlState {Trajectory, Tracking, Stopping};
  struct WristSample {double q4; Eigen::Vector3d normal;};
  struct Verdict {
    bool is_drone;
    double confidence;
    rclcpp::Time received;
  };

  static double angle(const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
    return std::acos(std::clamp(a.dot(b), -1.0, 1.0));
  }
  static Eigen::AngleAxisd rz(double value) {
    return Eigen::AngleAxisd(value, Eigen::Vector3d::UnitZ());
  }

  void open_ekf_direction_log()
  {
    if (ekf_direction_log_path_.empty()) {
      return;
    }
    const fs::path path(ekf_direction_log_path_);
    if (!path.parent_path().empty()) {
      fs::create_directories(path.parent_path());
    }
    const bool write_header = !fs::exists(path) || fs::file_size(path) == 0;
    ekf_direction_log_.open(path, std::ios::app);
    if (!ekf_direction_log_) {
      throw std::runtime_error("cannot open EKF direction log: " + path.string());
    }
    if (write_header) {
      ekf_direction_log_
        << "receipt_sec,receipt_nanosec,message_sec,message_nanosec,frame_id,"
        << "raw_x,raw_y,raw_z,world_x,world_y,world_z,"
        << "filtered_x,filtered_y,filtered_z,input_jump_deg,world_jump_deg,filtered_step_deg,"
        << "have_joints,joint1,joint4\n";
    }
    ekf_direction_log_ << std::setprecision(17);
    ekf_direction_log_.flush();
    RCLCPP_INFO(get_logger(), "Logging every EKF direction sample to %s",
      path.c_str());
  }

  void write_ekf_direction_log(
    const geometry_msgs::msg::Vector3Stamped & message,
    const rclcpp::Time & receipt,
    const Eigen::Vector3d & input_direction,
    const Eigen::Vector3d & previous_input_direction,
    const Eigen::Vector3d & world_direction,
    const Eigen::Vector3d & previous_world_direction,
    const Eigen::Vector3d & previous_filtered,
    const Eigen::Vector3d & filtered_direction,
    bool had_previous_ekf_direction,
    bool had_previous_direction,
    bool have_joints,
    const std::array<double, 6> & joints)
  {
    if (!ekf_direction_log_.is_open()) {
      return;
    }
    const double input_jump = had_previous_ekf_direction ?
      angle(previous_input_direction, input_direction) * 180.0 / M_PI : 0.0;
    const double world_jump = had_previous_ekf_direction ?
      angle(previous_world_direction, world_direction) * 180.0 / M_PI : 0.0;
    const double filtered_step = had_previous_direction ?
      angle(previous_filtered, filtered_direction) * 180.0 / M_PI : 0.0;
    const int64_t receipt_nanoseconds = receipt.nanoseconds();
    ekf_direction_log_
      << receipt_nanoseconds / 1000000000LL << ','
      << receipt_nanoseconds % 1000000000LL << ','
      << message.header.stamp.sec << ',' << message.header.stamp.nanosec << ','
      << message.header.frame_id << ','
      << message.vector.x << ',' << message.vector.y << ',' << message.vector.z << ','
      << world_direction.x() << ',' << world_direction.y() << ',' << world_direction.z() << ','
      << filtered_direction.x() << ',' << filtered_direction.y() << ','
      << filtered_direction.z() << ',' << input_jump << ',' << world_jump << ','
      << filtered_step << ','
      << (have_joints ? 1 : 0) << ',' << joints[0] << ',' << joints[3] << '\n';
    ekf_direction_log_.flush();
  }

  void open_yolo_direction_log()
  {
    if (yolo_direction_log_path_.empty()) {
      return;
    }
    const fs::path path(yolo_direction_log_path_);
    if (!path.parent_path().empty()) {
      fs::create_directories(path.parent_path());
    }
    const bool write_header = !fs::exists(path) || fs::file_size(path) == 0;
    yolo_direction_log_.open(path, std::ios::app);
    if (!yolo_direction_log_) {
      throw std::runtime_error("cannot open YOLO direction log: " + path.string());
    }
    if (write_header) {
      yolo_direction_log_
        << "receipt_sec,receipt_nanosec,message_sec,message_nanosec,frame_id,"
        << "x,y,z,jump_deg\n";
    }
    yolo_direction_log_ << std::setprecision(17);
    yolo_direction_log_.flush();
    RCLCPP_INFO(get_logger(), "Logging every YOLO direction sample to %s",
      path.c_str());
  }

  void log_yolo_direction(const geometry_msgs::msg::Vector3Stamped & message)
  {
    Eigen::Vector3d direction(message.vector.x, message.vector.y, message.vector.z);
    if (direction.norm() < 1e-6 || !yolo_direction_log_.is_open()) {
      return;
    }
    direction.normalize();
    const auto receipt = now();
    const double jump = have_previous_yolo_direction_ ?
      angle(previous_yolo_direction_, direction) * 180.0 / M_PI : 0.0;
    previous_yolo_direction_ = direction;
    have_previous_yolo_direction_ = true;
    const int64_t receipt_nanoseconds = receipt.nanoseconds();
    yolo_direction_log_
      << receipt_nanoseconds / 1000000000LL << ','
      << receipt_nanoseconds % 1000000000LL << ','
      << message.header.stamp.sec << ',' << message.header.stamp.nanosec << ','
      << message.header.frame_id << ','
      << message.vector.x << ',' << message.vector.y << ',' << message.vector.z << ','
      << jump << '\n';
    yolo_direction_log_.flush();
  }

  bool publish_trajectory(
    const std::array<double, 6> & target, double duration_s,
    const std::array<double, 6> & endpoint_velocity = {})
  {
    if (command_pub_->get_subscription_count() == 0) {
      RCLCPP_ERROR(get_logger(), "arm_controller command topic has no subscriber");
      return false;
    }
    trajectory_msgs::msg::JointTrajectory command;
    command.joint_names.assign(names_.begin(), names_.end());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(target.begin(), target.end());
    point.velocities.assign(endpoint_velocity.begin(), endpoint_velocity.end());
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    command.points.push_back(point);
    command_pub_->publish(command);
    return true;
  }

  void publish_servo_command(const std::array<double, 6> & positions)
  {
    std_msgs::msg::Float64MultiArray command;
    command.data.assign(positions.begin(), positions.end());
    servo_pub_->publish(command);
  }

  bool switch_controllers(
    const std::vector<std::string> & activate,
    const std::vector<std::string> & deactivate,
    std::string & error)
  {
    if (!switch_client_->wait_for_service(std::chrono::seconds(3))) {
      error = "/controller_manager/switch_controller is unavailable";
      return false;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 5;
    auto future = switch_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
      error = "controller switch timed out";
      return false;
    }
    if (!future.get()->ok) {
      error = "controller manager rejected the switch";
      return false;
    }
    return true;
  }

  void initialize_servo(const std::array<double, 6> & position)
  {
    configure_fixed_pose(position);
    servo_input_.current_position = position;
    servo_input_.current_velocity.fill(0.0);
    servo_input_.current_acceleration.fill(0.0);
    servo_input_.target_position = position;
    servo_input_.target_velocity.fill(0.0);
    servo_input_.target_acceleration.fill(0.0);
    servo_input_.max_velocity.fill(max_velocity_);
    servo_input_.max_acceleration.fill(max_acceleration_);
    servo_input_.max_jerk.fill(max_jerk_);
    servo_input_.enabled = std::array<bool, 6>{true, false, false, true, false, false};
    servo_input_.synchronization = ruckig::Synchronization::Time;
    servo_command_ = position;
    otg_->reset();
  }

  void request_home(std_srvs::srv::Trigger::Response & response)
  {
    std::array<double, 6> current;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_joints_) {
        response.success = false;
        response.message = "Cannot home: waiting for /joint_states";
        return;
      }
      if (control_state_ != ControlState::Trajectory) {
        response.success = false;
        response.message =
          "Cannot home while tracking controller is active; disable /demo/tracking first";
        return;
      }
      have_direction_ = false;
      motion_latched_ = false;
      home_requested_ = true;
      home_reached_ = false;
      current = current_;
    }
    if (!publish_trajectory(home_, home_duration_s_)) {
      std::lock_guard<std::mutex> lock(mutex_);
      home_requested_ = false;
      response.success = false;
      response.message = "Cannot home: arm_controller is unavailable";
      return;
    }
    response.success = true;
    response.message = "Home trajectory sent; wait for the tracker to report home reached";
    RCLCPP_WARN(get_logger(), "Home requested from q=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      current[0], current[1], current[2], current[3], current[4], current[5]);
  }

  void set_tracking(bool enable, std_srvs::srv::SetBool::Response & response)
  {
    std::array<double, 6> seed {};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (enable) {
        if (control_state_ == ControlState::Tracking) {
          response.success = true;
          response.message = "Tracking is already active";
          return;
        }
        if (control_state_ == ControlState::Stopping) {
          response.success = false;
          response.message = "Tracking is still stopping; retry after arm_controller is active";
          return;
        }
        if (!have_joints_) {
          response.success = false;
          response.message = "Cannot start tracking: waiting for /joint_states";
          return;
        }
        if (require_home_ && !home_reached_) {
          response.success = false;
          response.message = "Cannot start tracking: call /demo/home and wait until it reaches home";
          return;
        }
        if (std::abs(current_[3]) > M_PI_2) {
          response.success = false;
          response.message = "Cannot start tracking: joint4 is outside the tracker range";
          return;
        }
        have_direction_ = false;
        motion_latched_ = false;
        seed = current_;
      } else {
        if (control_state_ == ControlState::Trajectory) {
          response.success = true;
          response.message = "Tracking is already disabled";
          return;
        }
        if (control_state_ == ControlState::Stopping) {
          response.success = true;
          response.message = "Tracking is already decelerating to a stop";
          return;
        }
        control_state_ = ControlState::Stopping;
        have_direction_ = false;
        motion_latched_ = false;
        set_stopping_target();
        response.success = true;
        response.message = "Tracking is decelerating; arm_controller will reactivate automatically";
        RCLCPP_WARN(get_logger(), "Tracking stop requested; decelerating before controller switch");
        return;
      }
    }

    initialize_servo(seed);
    std::string error;
    if (!switch_controllers(
        {"arm_tracking_controller"}, {"arm_controller"}, error)) {
      response.success = false;
      response.message = "Cannot start tracking: " + error;
      RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
      return;
    }
    publish_servo_command(seed);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      control_state_ = ControlState::Tracking;
    }
    response.success = true;
    response.message = "Tracking armed; waiting for a fresh direction";
    RCLCPP_WARN(get_logger(),
      "Tracking enabled with arm_tracking_controller; stale directions were discarded");
  }

  void yolo_callback(const geometry_msgs::msg::PoseArray & msg) {
    if (msg.poses.empty()) {return;}
    const auto & point = msg.poses.front().position;
    const Eigen::Vector3d local(point.x, point.y, point.z);
    if (local.norm() < 1e-6) {return;}
    accept_local_direction(local.normalized(), msg.header.frame_id);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "direct YOLO target ray=(%.3f, %.3f, %.3f)",
      point.x, point.y, point.z);
  }

  void direction_callback(const geometry_msgs::msg::Vector3Stamped & msg) {
    Eigen::Vector3d local(msg.vector.x, msg.vector.y, msg.vector.z);
    if (local.norm() < 1e-6) return;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Received tracking direction on %s: (%.3f, %.3f, %.3f)",
      msg.header.frame_id.c_str(), msg.vector.x, msg.vector.y, msg.vector.z);
    accept_local_direction(
      local.normalized(), msg.header.frame_id, ekf_enabled_ ? &msg : nullptr);
  }

  void classifier_callback(
    const geometry_msgs::msg::Vector3Stamped & msg,
    Verdict & verdict, bool & have_verdict)
  {
    verdict = Verdict{msg.vector.y >= 0.5, msg.vector.z, now()};
    have_verdict = true;
  }

  const Verdict * live_verdict(const rclcpp::Time & stamp) {
    const Verdict * best = nullptr;
    const auto consider = [&](const Verdict & verdict, bool have_verdict) {
      if (!have_verdict) {return;}
      if (!verdict.is_drone ||
          (stamp - verdict.received).seconds() > direct_class_timeout_) {return;}
      if (best == nullptr || verdict.confidence > best->confidence) {best = &verdict;}
    };
    if (direct_classifier_source_ == "ast" || direct_classifier_source_ == "either") {
      consider(ast_verdict_, have_ast_verdict_);
    }
    if (direct_classifier_source_ == "gre" || direct_classifier_source_ == "either") {
      consider(gre_verdict_, have_gre_verdict_);
    }
    return best;
  }

  void sst_callback(const odas_ros_msgs::msg::OdasSstArrayStamped & msg) {
    const auto stamp = now();
    const odas_ros_msgs::msg::OdasSst * selected = nullptr;
    const auto * verdict = live_verdict(stamp);
    for (const auto & source : msg.sources) {
      if (source.activity < direct_min_activity_) {continue;}
      if (verdict != nullptr &&
          (selected == nullptr || source.activity > selected->activity)) {
        selected = &source;
      }
    }
    if (selected == nullptr) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
        "direct mode waiting for %s detection and an active ODAS direction",
        direct_classifier_source_.c_str());
      return;
    }
    const double norm = std::sqrt(
      selected->x * selected->x + selected->y * selected->y + selected->z * selected->z);
    if (norm < 1e-6) {return;}
    geometry_msgs::msg::PointStamped target;
    target.header = msg.header;
    target.point.x = std::atan2(selected->y, selected->x);
    target.point.y = std::asin(std::clamp(selected->z / norm, -1.0, 1.0));
    target.point.z = 1.0;
    fused_callback(target);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "direct target confidence=%.2f activity=%.2f az=%.1fdeg el=%.1fdeg",
      verdict->confidence, selected->activity,
      target.point.x * 180.0 / M_PI, target.point.y * 180.0 / M_PI);
  }

  // Conversions 1 and 2: azimuth/elevation in the tracking frame become a
  // unit vector in the world frame.
  void fused_callback(const geometry_msgs::msg::PointStamped & msg) {
    const double az = msg.point.x;
    const double el = msg.point.y;
    // point.z is always 1.0 and marks a unit direction. It is NOT a range,
    // and must not enter the geometry.
    const double ce = std::cos(el);
    const Eigen::Vector3d local(ce * std::cos(az), ce * std::sin(az), std::sin(el));

    accept_local_direction(local, msg.header.frame_id);
  }

  void accept_local_direction(
    const Eigen::Vector3d & local, const std::string & frame_id,
    const geometry_msgs::msg::Vector3Stamped * ekf_message = nullptr)
  {

    const std::string source =
      frame_id.empty() ? tracking_frame_ : frame_id;

    Eigen::Vector3d world_dir;
    try {
      // Latest available transform rather than the message stamp: the arm is
      // moving, and with use_sim_time the exact stamp is frequently just
      // outside the buffer. A few milliseconds of TF staleness costs far less
      // than dropping the measurement entirely.
      const auto tf = tf_buffer_->lookupTransform(
        world_frame_, source, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
      world_dir = (rotation * local).normalized();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "no %s -> %s transform: %s", source.c_str(), world_frame_.c_str(), e.what());
      return;
    }

    if (world_dir.norm() < 1e-6) {return;}

    const auto receipt = now();
    Eigen::Vector3d previous_input_direction;
    Eigen::Vector3d previous_world_direction;
    Eigen::Vector3d previous_filtered;
    Eigen::Vector3d filtered;
    std::array<double, 6> joints {};
    bool had_previous_ekf_direction = false;
    bool had_previous_direction = false;
    bool have_joints = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      had_previous_direction = have_direction_;
      previous_filtered = had_previous_direction ? filtered_direction_ : world_dir;
      if (ekf_message != nullptr) {
        had_previous_ekf_direction = have_previous_ekf_world_direction_;
        previous_input_direction = had_previous_ekf_direction ?
          previous_ekf_input_direction_ : local;
        previous_world_direction = had_previous_ekf_direction ?
          previous_ekf_world_direction_ : world_dir;
        previous_ekf_input_direction_ = local;
        previous_ekf_world_direction_ = world_dir;
        have_previous_ekf_world_direction_ = true;
      }
      if (!have_direction_) {
        filtered_direction_ = world_dir;
        have_direction_ = true;
      } else {
        // Smoothing on the world-frame vector absorbs TF jitter, which the EKF
        // cannot see. alpha = 1.0 disables it.
        filtered_direction_ = (
          (1.0 - smoothing_alpha_) * filtered_direction_ +
          smoothing_alpha_ * world_dir).normalized();
      }
      filtered = filtered_direction_;
      joints = current_;
      have_joints = have_joints_;
      last_target_ = receipt;
    }
    if (ekf_message != nullptr) {
      write_ekf_direction_log(
        *ekf_message, receipt, local, previous_input_direction,
        world_dir, previous_world_direction,
        previous_filtered, filtered, had_previous_ekf_direction,
        had_previous_direction, have_joints, joints);
    }
  }

  Eigen::Vector3d microphone_normal(const std::array<double, 6> & q) const {
    Eigen::Matrix3d r = rz(q[0]).toRotationMatrix();
    r *= Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    r *= rz(q[1]).toRotationMatrix();
    r *= rz(q[2]).toRotationMatrix();
    r *= rz(q[3]).toRotationMatrix();
    r *= Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    r *= rz(q[4]).toRotationMatrix();
    r *= Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    r *= rz(q[5]).toRotationMatrix();
    return (r * Eigen::Vector3d::UnitZ()).normalized();
  }

  void joint_callback(const sensor_msgs::msg::JointState & msg) {
    std::array<double, 6> ordered {};
    for (std::size_t i = 0; i < names_.size(); ++i) {
      const auto found = std::find(msg.name.begin(), msg.name.end(), names_[i]);
      if (found == msg.name.end()) {return;}
      ordered[i] = msg.position[std::distance(msg.name.begin(), found)];
    }
    std::lock_guard<std::mutex> lock(mutex_); current_ = ordered;
    if (!have_joints_) {
      configure_fixed_pose(ordered);
    }
    if (home_requested_ && at_home(ordered)) {
      home_requested_ = false;
      home_reached_ = true;
      configure_fixed_pose(ordered);
      RCLCPP_WARN(get_logger(), "Home reached; use /demo/tracking with data: true to arm tracking");
    }
    have_joints_ = true;
  }

  bool at_home(const std::array<double, 6> & joints) const
  {
    for (std::size_t index = 0; index < joints.size(); ++index) {
      if (std::abs(std::remainder(joints[index] - home_[index], 2.0 * M_PI)) >
          home_tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  void configure_fixed_pose(const std::array<double, 6> & joints)
  {
    fixed_ = joints;
    motion_latched_ = false;
    wrist_lookup_.clear();
    wrist_lookup_.reserve(160);
    for (double q4 = -M_PI_2; q4 <= M_PI_2; q4 += 0.02) {
      auto trial = fixed_;
      trial[0] = 0.0;
      trial[3] = q4;
      wrist_lookup_.push_back({q4, microphone_normal(trial)});
    }
    RCLCPP_INFO(get_logger(), "Locked q2=%.3f q3=%.3f q5=%.3f q6=%.3f",
      fixed_[1], fixed_[2], fixed_[4], fixed_[5]);
  }

  std::array<double, 6> solve(
    const std::array<double, 6> & current, const Eigen::Vector3d & desired) const
  {
    auto best = current;
    double best_cost = std::numeric_limits<double>::infinity();
    const double desired_yaw = std::atan2(desired.y(), desired.x());
    const auto evaluate = [&](double q4) {
      auto trial = fixed_;
      trial[0] = 0.0;
      trial[3] = q4;
      const auto base_normal = microphone_normal(trial);
      const double raw_q1 = desired_yaw - std::atan2(base_normal.y(), base_normal.x());
      // Keep the equivalent yaw nearest the measured joint position. Using
      // the principal -pi..pi value directly creates a +/-2pi command jump
      // when a target crosses the yaw wrap boundary.
      double q1 = current[0] + std::remainder(raw_q1 - current[0], 2.0 * M_PI);
      q1 = std::clamp(q1, -3.0543, 3.0543);
      const Eigen::Vector3d normal = rz(q1) * base_normal;
      const double error = angle(normal, desired);
      const double d1 = q1 - current[0];
      const double d4 = q4 - current[3];
      const double cost = error * error + motion_penalty_ * (d1 * d1 + d4 * d4);
      auto candidate = fixed_;
      candidate[0] = q1;
      candidate[3] = q4;
      return std::make_pair(cost, candidate);
    };

    double best_q4 = current[3];
    for (const auto & sample : wrist_lookup_) {
      const auto [cost, candidate] = evaluate(sample.q4);
      if (cost < best_cost) {
        best_cost = cost;
        best = candidate;
        best_q4 = sample.q4;
      }
    }

    // Refine the 0.02-rad lookup result. Without this step q4 targets jump in
    // 1.15-degree increments even when the incoming bearing changes smoothly.
    double low = std::max(-M_PI_2, best_q4 - 0.025);
    double high = std::min(M_PI_2, best_q4 + 0.025);
    for (int iteration = 0; iteration < 16; ++iteration) {
      const double left = low + (high - low) / 3.0;
      const double right = high - (high - low) / 3.0;
      if (evaluate(left).first < evaluate(right).first) {
        high = right;
      } else {
        low = left;
      }
    }
    const auto refined = evaluate(0.5 * (low + high));
    if (refined.first < best_cost) {
      best = refined.second;
    }
    return best;
  }

  void set_stopping_target()
  {
    servo_input_.target_position = servo_input_.current_position;
    for (const std::size_t index : active_joints_) {
      const double velocity = servo_input_.current_velocity[index];
      const double distance = velocity * std::abs(velocity) /
        (2.0 * max_acceleration_);
      servo_input_.target_position[index] = std::clamp(
        servo_input_.current_position[index] + distance,
        index == 0 ? -3.0543 : -M_PI_2,
        index == 0 ? 3.0543 : M_PI_2);
    }
    servo_input_.target_velocity.fill(0.0);
    servo_input_.target_acceleration.fill(0.0);
    stopping_target_set_ = true;
  }

  bool finish_tracking_controller_switch()
  {
    std::string error;
    if (!switch_controllers(
        {"arm_controller"}, {"arm_tracking_controller"}, error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Could not restore arm_controller after stopping: %s", error.c_str());
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    control_state_ = ControlState::Trajectory;
    stopping_target_set_ = false;
    RCLCPP_WARN(get_logger(),
      "Tracking disabled cleanly; arm_controller is active for MoveIt and home");
    return true;
  }

  void update() {
    Eigen::Vector3d desired;
    std::array<double, 6> actual;
    bool target_available = false;
    ControlState control_state = ControlState::Trajectory;
    bool external_veto = false;
    bool have_direction = false;
    double target_age = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(mutex_); const auto stamp = now();
      if (!have_joints_) {return;}
      // Conversion 3: freshness replaces /audio/target_valid. The EKF stops
      // publishing when it has nothing to report, so an old message is the
      // only signal that the target is gone.
      external_veto = external_veto_;
      control_state = control_state_;
      have_direction = have_direction_;
      if (have_direction_) {
        target_age = (stamp - last_target_).seconds();
      }
      target_available = control_state == ControlState::Tracking &&
        !external_veto && have_direction &&
        target_age <= target_timeout_;
      desired = filtered_direction_;
      actual = current_;
    }
    if (control_state == ControlState::Trajectory) {return;}

    const double q1_tracking_error = std::abs(std::remainder(
      actual[0] - servo_input_.current_position[0], 2.0 * M_PI));
    const double q4_tracking_error = std::abs(
      actual[3] - servo_input_.current_position[3]);
    if (control_state == ControlState::Tracking &&
        std::max(q1_tracking_error, q4_tracking_error) > max_tracking_error_) {
      RCLCPP_ERROR(get_logger(),
        "Measured arm fell behind servo by %.3f rad; stopping tracking safely",
        std::max(q1_tracking_error, q4_tracking_error));
      control_state_ = ControlState::Stopping;
      control_state = ControlState::Stopping;
      motion_latched_ = false;
      set_stopping_target();
    }

    auto target = servo_input_.current_position;
    const double pointing_error = have_direction ?
      angle(microphone_normal(actual), desired) : 0.0;

    // Use separate enter/exit thresholds so a noisy bearing near the
    // deadband cannot toggle the arm between chase and hold every cycle.
    if (control_state != ControlState::Tracking || !target_available) {
      motion_latched_ = false;
    } else if (motion_latched_) {
      if (pointing_error <= angular_deadband_exit_) {
        motion_latched_ = false;
      }
    } else if (pointing_error >= angular_deadband_) {
      motion_latched_ = true;
    }

    if (motion_latched_) {
      target = solve(servo_input_.current_position, desired);
      servo_input_.target_position = target;
      servo_input_.target_velocity.fill(0.0);
      servo_input_.target_acceleration.fill(0.0);
      stopping_target_set_ = false;
    } else if (!stopping_target_set_) {
      set_stopping_target();
      target = servo_input_.target_position;
    } else {
      target = servo_input_.target_position;
    }
    const double solution_error = have_direction ?
      angle(microphone_normal(target), desired) : 0.0;
    const auto result = otg_->update(servo_input_, servo_output_);
    if (result < ruckig::Result::Working) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ruckig rejected the servo state (error %d); holding last command",
        static_cast<int>(result));
      return;
    }
    servo_command_ = servo_output_.new_position;
    servo_command_[1] = fixed_[1];
    servo_command_[2] = fixed_[2];
    servo_command_[4] = fixed_[4];
    servo_command_[5] = fixed_[5];
    publish_servo_command(servo_command_);
    servo_output_.pass_to_input(servo_input_);

    if (control_state == ControlState::Stopping && result == ruckig::Result::Finished) {
      finish_tracking_controller_switch();
      return;
    }
    const char * state = target_available ?
      (motion_latched_ ? "tracking" : "reached/deadband") :
      (external_veto ? "hold/veto" :
       (!have_direction ? "hold/no-target" : "hold/stale"));
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "servo q1=%.3f q4=%.3f state=%s age=%.2fs "
      "error=%.1fdeg solution_error=%.1fdeg target_q1=%.3f target_q4=%.3f "
      "desired_world=(%.2f,%.2f,%.2f); q2/q3/q5/q6 fixed",
      servo_command_[0], servo_command_[3], state, target_age,
      pointing_error * 180.0 / M_PI, solution_error * 180.0 / M_PI,
      target[0], target[3], desired.x(), desired.y(), desired.z());
  }

  const std::array<std::string, 6> names_{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  const std::array<std::size_t, 2> active_joints_{0, 3};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr fused_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yolo_sub_;
  rclcpp::Subscription<odas_ros_msgs::msg::OdasSstArrayStamped>::SharedPtr sst_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr ast_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gre_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tracking_service_;
  rclcpp::CallbackGroup::SharedPtr switch_callback_group_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_; mutable std::mutex mutex_;
  std::array<double, 6> current_ {}, fixed_ {}, servo_command_ {};
  ruckig::InputParameter<6> servo_input_;
  ruckig::OutputParameter<6> servo_output_;
  std::unique_ptr<ruckig::Ruckig<6>> otg_;
  std::vector<WristSample> wrist_lookup_;
  Eigen::Vector3d filtered_direction_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d previous_ekf_input_direction_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d previous_ekf_world_direction_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d previous_yolo_direction_{Eigen::Vector3d::UnitX()};
  rclcpp::Time last_target_{0, 0, RCL_ROS_TIME};
  bool have_joints_{false}, external_veto_{false}, have_direction_{false};
  bool have_previous_ekf_world_direction_{false};
  bool have_previous_yolo_direction_{false};
  ControlState control_state_{ControlState::Trajectory};
  bool home_requested_{false}, home_reached_{false};
  bool motion_latched_{false}, stopping_target_set_{false};
  bool ekf_enabled_{true};
  Verdict ast_verdict_{false, 0.0, rclcpp::Time(0, 0, RCL_ROS_TIME)};
  Verdict gre_verdict_{false, 0.0, rclcpp::Time(0, 0, RCL_ROS_TIME)};
  bool have_ast_verdict_{false}, have_gre_verdict_{false};
  double target_timeout_, smoothing_alpha_, angular_deadband_, angular_deadband_exit_;
  double motion_penalty_;
  double command_horizon_, command_rate_hz_, max_velocity_, max_acceleration_;
  double max_jerk_, max_tracking_error_;
  double direct_min_activity_, direct_class_timeout_;
  bool require_home_;
  double home_duration_s_, home_tolerance_rad_;
  std::string world_frame_, tracking_frame_;
  std::string direct_classifier_source_;
  std::string ekf_direction_log_path_;
  std::string yolo_direction_log_path_;
  std::ofstream ekf_direction_log_;
  std::ofstream yolo_direction_log_;
  const std::array<double, 6> home_{
    M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, M_PI / 2.0, 0.0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmAudioTracker>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
