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
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace fs = std::filesystem;

// =============================================================================
// arm_tracker  -  Single and Dual-Arm Coordinated Drone Tracking Controller
//
// Dual-arm coordination:
// - Arm 1 (Left / Primary) carries the microphone array and camera sensors.
//   It resolves target direction in world frame and servos joint1/joint4.
// - Arm 2 (Right / Follower) receives target info from Arm 1.
//   It computes parallax gaze convergence based on the -75 cm baseline (left of Arm 1)
//   and target_nominal_distance (default 3.0 m).
// - Co-tracking policy:
//   "first_arm_lead" (default): Arm 1 turns first to lock on the drone within
//   lead_lock_tolerance_rad (default 0.10 rad); once locked, Arm 2 slews to co-track.
//   "simultaneous": Both arms slew immediately upon detection.
// =============================================================================

class ArmTracker : public rclcpp::Node {
public:
  ArmTracker() : Node("arm_tracker") {
    dual_arm_ = declare_parameter("dual_arm", false);
    second_arm_x_ = declare_parameter("second_arm_x", -0.75);
    second_arm_y_ = declare_parameter("second_arm_y", 0.0);
    second_arm_z_ = declare_parameter("second_arm_z", 0.0);
    target_nominal_distance_ = declare_parameter("target_nominal_distance", 0.0);
    dual_tracking_policy_ = declare_parameter("dual_tracking_policy", std::string("first_arm_lead"));
    lead_lock_tolerance_rad_ = declare_parameter("lead_lock_tolerance_rad", 0.10);

    target_timeout_ = declare_parameter("target_timeout", 0.75);
    smoothing_alpha_ = declare_parameter("smoothing_alpha", 0.50);
    angular_deadband_ = declare_parameter("angular_deadband", 0.003);
    motion_penalty_ = declare_parameter("motion_penalty", 0.015);
    servo_log_path_ = declare_parameter("servo_log_path", std::string(""));
    command_rate_hz_ = declare_parameter("command_rate_hz", 100.0);
    max_velocity_ = declare_parameter("max_velocity", 2.50);
    max_acceleration_ = declare_parameter("max_acceleration", 12.0);
    max_jerk_ = declare_parameter("max_jerk", 80.0);
    max_tracking_error_ = declare_parameter("max_tracking_error", 0.50);
    world_frame_ = declare_parameter("world_frame", std::string("world"));
    tracking_frame_ = declare_parameter("tracking_frame", std::string("table_mic_link"));
    ekf_enabled_ = declare_parameter("ekf_enabled", true);
    direct_classifier_source_ = declare_parameter("direct_classifier_source", std::string("gre"));
    direct_min_activity_ = declare_parameter("direct_min_activity", 0.10);
    direct_class_timeout_ = declare_parameter("direct_class_timeout", 5.0);
    ekf_direction_log_path_ = declare_parameter("ekf_direction_log_path", std::string(""));
    yolo_direction_log_path_ = declare_parameter("yolo_direction_log_path", std::string(""));
    require_home_ = declare_parameter("require_home_before_tracking", true);
    home_duration_s_ = declare_parameter("home_duration_s", 12.0);
    home_tolerance_rad_ = declare_parameter("home_tolerance_rad", 0.035);

    if (angular_deadband_ < 0.0) {
      throw std::invalid_argument("angular_deadband must be non-negative");
    }
    if (max_velocity_ <= 0.0 || max_acceleration_ <= 0.0 ||
        max_jerk_ <= 0.0 || max_tracking_error_ <= 0.0) {
      throw std::invalid_argument(
        "max_velocity, max_acceleration, max_jerk, and "
        "max_tracking_error must be positive");
    }
    if (command_rate_hz_ < 20.0 || command_rate_hz_ > 200.0) {
      throw std::invalid_argument("command_rate_hz must be between 20 and 200");
    }
    if (dual_tracking_policy_ != "first_arm_lead" && dual_tracking_policy_ != "simultaneous") {
      throw std::invalid_argument("dual_tracking_policy must be 'first_arm_lead' or 'simultaneous'");
    }

    open_ekf_direction_log();
    open_yolo_direction_log();
    open_servo_log();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    events_pub_ = create_publisher<std_msgs::msg::String>("/tracking/events", 20);

    const std::string trajectory_topic = dual_arm_ ?
      "/dual_arm_controller/joint_trajectory" : "/arm_controller/joint_trajectory";
    const std::string servo_topic = dual_arm_ ?
      "/dual_arm_tracking_controller/commands" : "/arm_tracking_controller/commands";

    command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(trajectory_topic, 10);
    servo_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(servo_topic, rclcpp::SensorDataQoS());

    switch_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller", rmw_qos_profile_services_default,
      switch_callback_group_);

    otg_ = std::make_unique<ruckig::Ruckig<6>>(1.0 / command_rate_hz_);
    if (dual_arm_) {
      otg_right_ = std::make_unique<ruckig::Ruckig<6>>(1.0 / command_rate_hz_);
    }

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_callback(*msg); });

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
          [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { direction_callback(*msg); });
      };
      if (direct_classifier_source_ == "yolo") subscribe("/yolo/direction", yolo_sub_);
      if (direct_classifier_source_ == "mobilenetv2") subscribe("/mobilenetv2/direction", mobilenetv2_sub_);
      if (direct_classifier_source_ == "ast" || direct_classifier_source_ == "either") subscribe("/ast/direction", ast_sub_);
      if (direct_classifier_source_ == "gre" || direct_classifier_source_ == "either") subscribe("/gre/direction", gre_sub_);
    }

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
    timer_ = create_wall_timer(command_period, [this]() { update(); });

    RCLCPP_INFO(get_logger(),
      "ArmTracker initialized [%s mode]. Mode: %s. Policy: %s (nom_dist: %.1fm). Command rate: %.1f Hz.",
      dual_arm_ ? "DUAL-ARM" : "SINGLE-ARM",
      ekf_enabled_ ? "EKF" : "direct",
      dual_arm_ ? dual_tracking_policy_.c_str() : "N/A",
      target_nominal_distance_,
      command_rate_hz_);
  }

private:
  enum class ControlState { Trajectory, Tracking, Stopping };
  struct WristSample { double q4; Eigen::Vector3d normal; };
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

  void open_ekf_direction_log() {
    if (ekf_direction_log_path_.empty()) return;
    const fs::path path(ekf_direction_log_path_);
    if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
    const bool write_header = !fs::exists(path) || fs::file_size(path) == 0;
    ekf_direction_log_.open(path, std::ios::app);
    if (!ekf_direction_log_) throw std::runtime_error("cannot open EKF direction log: " + path.string());
    if (write_header) {
      ekf_direction_log_ << "receipt_sec,receipt_nanosec,message_sec,message_nanosec,frame_id,x,y,z\n";
    }
  }

  void open_yolo_direction_log() {
    if (yolo_direction_log_path_.empty()) return;
    const fs::path path(yolo_direction_log_path_);
    if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
    const bool write_header = !fs::exists(path) || fs::file_size(path) == 0;
    yolo_direction_log_.open(path, std::ios::app);
    if (!yolo_direction_log_) throw std::runtime_error("cannot open YOLO direction log: " + path.string());
    if (write_header) {
      yolo_direction_log_ << "receipt_sec,receipt_nanosec,message_sec,message_nanosec,frame_id,x,y,z,jump_deg\n";
    }
  }

  void log_yolo_direction(const geometry_msgs::msg::Vector3Stamped & message) {
    Eigen::Vector3d direction(message.vector.x, message.vector.y, message.vector.z);
    if (direction.norm() < 1e-6 || !yolo_direction_log_.is_open()) return;
    direction.normalize();
    const auto receipt = now();
    const double jump = have_previous_yolo_direction_ ?
      angle(previous_yolo_direction_, direction) * 180.0 / M_PI : 0.0;
    previous_yolo_direction_ = direction;
    have_previous_yolo_direction_ = true;
    const int64_t receipt_nanoseconds = receipt.nanoseconds();
    yolo_direction_log_
      << receipt_nanoseconds / 1000000000LL << ',' << receipt_nanoseconds % 1000000000LL << ','
      << message.header.stamp.sec << ',' << message.header.stamp.nanosec << ','
      << message.header.frame_id << ','
      << message.vector.x << ',' << message.vector.y << ',' << message.vector.z << ','
      << jump << '\n';
    yolo_direction_log_.flush();
  }

  void open_servo_log() {
    if (servo_log_path_.empty()) return;
    const fs::path path(servo_log_path_);
    if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
    const bool write_header = !fs::exists(path) || fs::file_size(path) == 0;
    servo_log_.open(path, std::ios::app);
    if (!servo_log_) throw std::runtime_error("cannot open servo log: " + path.string());
    if (write_header) {
      servo_log_
        << "time_sec,control_state,target_available,"
        << "target_q1,target_q4,actual_q1,actual_q4,"
        << "command_q1,command_q4,pointing_error_deg\n";
    }
  }

  std::array<double, 6> compute_jerk(
    const std::array<double, 6> & acceleration,
    std::array<double, 6> & prev_acc, bool & have_prev)
  {
    std::array<double, 6> jerk {};
    if (have_prev) {
      const double dt = 1.0 / command_rate_hz_;
      for (std::size_t i = 0; i < jerk.size(); ++i) {
        jerk[i] = (acceleration[i] - prev_acc[i]) / dt;
      }
    }
    prev_acc = acceleration;
    have_prev = true;
    return jerk;
  }

  void write_servo_log(
    ControlState control_state, bool target_available,
    const std::array<double, 6> & target, const std::array<double, 6> & actual,
    double pointing_error)
  {
    if (!servo_log_.is_open()) return;
    const char * state =
      control_state == ControlState::Tracking ? "tracking" :
      (control_state == ControlState::Stopping ? "stopping" : "trajectory");
    servo_log_
      << now().seconds() << ',' << state << ','
      << (target_available ? 1 : 0) << ','
      << target[0] << ',' << target[3] << ','
      << actual[0] << ',' << actual[3] << ','
      << servo_command_[0] << ',' << servo_command_[3] << ','
      << pointing_error * 180.0 / M_PI << '\n';
    servo_log_.flush();
  }

  bool publish_trajectory(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & target_positions,
    double duration_s)
  {
    if (command_pub_->get_subscription_count() == 0) {
      RCLCPP_ERROR(get_logger(), "Trajectory controller command topic has no subscriber");
      return false;
    }
    trajectory_msgs::msg::JointTrajectory command;
    command.joint_names = joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target_positions;
    point.velocities.assign(target_positions.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    command.points.push_back(point);
    command_pub_->publish(command);
    return true;
  }

  void publish_servo_command(const std::vector<double> & positions) {
    std_msgs::msg::Float64MultiArray command;
    command.data = positions;
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

  void initialize_servo_arm(
    ruckig::InputParameter<6> & input,
    std::array<double, 6> & command,
    ruckig::Ruckig<6> & otg,
    const std::array<double, 6> & position)
  {
    input.control_interface = ruckig::ControlInterface::Position;
    input.current_position = position;
    input.current_velocity.fill(0.0);
    input.current_acceleration.fill(0.0);
    input.target_position = position;
    input.target_velocity.fill(0.0);
    input.target_acceleration.fill(0.0);
    input.max_velocity.fill(max_velocity_);
    input.max_acceleration.fill(max_acceleration_);
    input.max_jerk.fill(max_jerk_);
    input.enabled = std::array<bool, 6>{true, false, false, true, false, false};
    input.synchronization = ruckig::Synchronization::Time;
    command = position;
    otg.reset();
  }

  void initialize_servo(
    const std::array<double, 6> & position_left,
    const std::array<double, 6> & position_right = {})
  {
    configure_fixed_pose(position_left, fixed_, wrist_lookup_);
    initialize_servo_arm(servo_input_, servo_command_, *otg_, position_left);
    if (dual_arm_) {
      configure_fixed_pose(position_right, fixed_right_, wrist_lookup_right_);
      initialize_servo_arm(servo_input_right_, servo_command_right_, *otg_right_, position_right);
    }
  }

  void request_home(std_srvs::srv::Trigger::Response & response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_joints_) {
        response.success = false;
        response.message = "Cannot home: waiting for /joint_states";
        return;
      }
      if (control_state_ != ControlState::Trajectory) {
        response.success = false;
        response.message = "Cannot home while tracking controller is active; disable /demo/tracking first";
        return;
      }
      have_direction_ = false;
      home_requested_ = true;
      home_reached_ = false;
      arm1_locked_ = false;
    }

    bool published = false;
    if (!dual_arm_) {
      published = publish_trajectory(names_single_, {home_.begin(), home_.end()}, home_duration_s_);
    } else {
      std::vector<std::string> dual_names;
      dual_names.insert(dual_names.end(), names_left_.begin(), names_left_.end());
      dual_names.insert(dual_names.end(), names_right_.begin(), names_right_.end());
      std::vector<double> dual_home;
      dual_home.insert(dual_home.end(), home_.begin(), home_.end());
      dual_home.insert(dual_home.end(), home_.begin(), home_.end());
      published = publish_trajectory(dual_names, dual_home, home_duration_s_);
    }

    if (!published) {
      std::lock_guard<std::mutex> lock(mutex_);
      home_requested_ = false;
      response.success = false;
      response.message = "Cannot home: trajectory controller is unavailable";
      return;
    }
    response.success = true;
    response.message = "Home trajectory sent; waiting for arms to reach home pose";
    RCLCPP_INFO(get_logger(), "Home trajectory commanded for %s.", dual_arm_ ? "dual arms" : "arm");
  }

  void set_tracking(bool enable, std_srvs::srv::SetBool::Response & response) {
    std::array<double, 6> seed_left {};
    std::array<double, 6> seed_right {};
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
          response.message = "Tracking is still stopping; retry after trajectory controller is active";
          return;
        }
        if (!have_joints_) {
          response.success = false;
          response.message = "Cannot start tracking: waiting for /joint_states";
          return;
        }
        if (require_home_ && !home_reached_) {
          response.success = false;
          response.message = "Cannot start tracking: call /demo/home and wait until home is reached";
          return;
        }
        if (std::abs(current_[3]) > M_PI_2 || (dual_arm_ && std::abs(current_right_[3]) > M_PI_2)) {
          response.success = false;
          response.message = "Cannot start tracking: joint4 is outside the tracker range";
          return;
        }
        have_direction_ = false;
        arm1_locked_ = false;
        seed_left = current_;
        if (dual_arm_) seed_right = current_right_;
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
        arm1_locked_ = false;
        begin_velocity_stop(servo_input_);
        if (dual_arm_) begin_velocity_stop(servo_input_right_);
        response.success = true;
        response.message = "Tracking is decelerating; trajectory controller will reactivate automatically";
        RCLCPP_WARN(get_logger(), "Tracking stop requested; decelerating before controller switch");
        return;
      }
    }

    initialize_servo(seed_left, seed_right);
    std::string error;
    const std::string tracking_ctrl = dual_arm_ ? "dual_arm_tracking_controller" : "arm_tracking_controller";
    const std::string traj_ctrl = dual_arm_ ? "dual_arm_controller" : "arm_controller";

    if (!switch_controllers({tracking_ctrl}, {traj_ctrl}, error)) {
      response.success = false;
      response.message = "Cannot start tracking: " + error;
      RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
      return;
    }

    if (!dual_arm_) {
      publish_servo_command({seed_left.begin(), seed_left.end()});
    } else {
      std::vector<double> dual_seed;
      dual_seed.insert(dual_seed.end(), seed_left.begin(), seed_left.end());
      dual_seed.insert(dual_seed.end(), seed_right.begin(), seed_right.end());
      publish_servo_command(dual_seed);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      control_state_ = ControlState::Tracking;
    }
    response.success = true;
    response.message = "Tracking armed; waiting for a fresh direction";
    RCLCPP_INFO(get_logger(), "Tracking enabled with %s", tracking_ctrl.c_str());
  }

  void direction_callback(const geometry_msgs::msg::Vector3Stamped & msg) {
    Eigen::Vector3d local(msg.vector.x, msg.vector.y, msg.vector.z);
    if (local.norm() < 1e-6) return;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Received tracking direction on %s: (%.3f, %.3f, %.3f)",
      msg.header.frame_id.c_str(), msg.vector.x, msg.vector.y, msg.vector.z);
    accept_local_direction(local.normalized(), msg.header.frame_id);
  }

  void accept_local_direction(const Eigen::Vector3d & local, const std::string & frame_id) {
    const std::string source = frame_id.empty() ? tracking_frame_ : frame_id;
    Eigen::Vector3d world_dir;
    try {
      const auto tf = tf_buffer_->lookupTransform(world_frame_, source, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
      world_dir = (rotation * local).normalized();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "no %s -> %s transform: %s", source.c_str(), world_frame_.c_str(), e.what());
      return;
    }

    if (world_dir.norm() < 1e-6) return;

    const auto receipt = now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_direction_) {
      filtered_direction_ = world_dir;
      have_direction_ = true;
    } else {
      filtered_direction_ = (
        (1.0 - smoothing_alpha_) * filtered_direction_ +
        smoothing_alpha_ * world_dir).normalized();
    }
    last_target_ = receipt;
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
    if (!dual_arm_) {
      std::array<double, 6> ordered {};
      for (std::size_t i = 0; i < names_single_.size(); ++i) {
        const auto found = std::find(msg.name.begin(), msg.name.end(), names_single_[i]);
        if (found == msg.name.end()) return;
        ordered[i] = msg.position[std::distance(msg.name.begin(), found)];
      }
      std::lock_guard<std::mutex> lock(mutex_);
      current_ = ordered;
      if (!have_joints_) {
        configure_fixed_pose(ordered, fixed_, wrist_lookup_);
      }
      if (home_requested_ && at_home(ordered)) {
        home_requested_ = false;
        home_reached_ = true;
        configure_fixed_pose(ordered, fixed_, wrist_lookup_);
        RCLCPP_WARN(get_logger(), "Home reached; use /demo/tracking with data: true to arm tracking");
      }
      have_joints_ = true;
    } else {
      std::array<double, 6> left_ordered {};
      std::array<double, 6> right_ordered {};
      for (std::size_t i = 0; i < names_left_.size(); ++i) {
        const auto found = std::find(msg.name.begin(), msg.name.end(), names_left_[i]);
        if (found == msg.name.end()) return;
        left_ordered[i] = msg.position[std::distance(msg.name.begin(), found)];
      }
      for (std::size_t i = 0; i < names_right_.size(); ++i) {
        const auto found = std::find(msg.name.begin(), msg.name.end(), names_right_[i]);
        if (found == msg.name.end()) return;
        right_ordered[i] = msg.position[std::distance(msg.name.begin(), found)];
      }
      std::lock_guard<std::mutex> lock(mutex_);
      current_ = left_ordered;
      current_right_ = right_ordered;
      if (!have_joints_) {
        configure_fixed_pose(left_ordered, fixed_, wrist_lookup_);
        configure_fixed_pose(right_ordered, fixed_right_, wrist_lookup_right_);
      }
      if (home_requested_ && at_home(left_ordered) && at_home(right_ordered)) {
        home_requested_ = false;
        home_reached_ = true;
        configure_fixed_pose(left_ordered, fixed_, wrist_lookup_);
        configure_fixed_pose(right_ordered, fixed_right_, wrist_lookup_right_);
        RCLCPP_WARN(get_logger(), "Dual arms reached home; use /demo/tracking with data: true to arm tracking");
      }
      have_joints_ = true;
    }
  }

  bool at_home(const std::array<double, 6> & joints) const {
    for (std::size_t index = 0; index < joints.size(); ++index) {
      if (std::abs(std::remainder(joints[index] - home_[index], 2.0 * M_PI)) > home_tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  void configure_fixed_pose(
    const std::array<double, 6> & joints,
    std::array<double, 6> & fixed_out,
    std::vector<WristSample> & lookup_out)
  {
    fixed_out = joints;
    lookup_out.clear();
    lookup_out.reserve(160);
    for (double q4 = -M_PI_2; q4 <= M_PI_2; q4 += 0.02) {
      auto trial = fixed_out;
      trial[0] = 0.0;
      trial[3] = q4;
      lookup_out.push_back({q4, microphone_normal(trial)});
    }
  }

  std::array<double, 6> solve(
    const std::array<double, 6> & current,
    const Eigen::Vector3d & desired,
    const std::array<double, 6> & fixed,
    const std::vector<WristSample> & lookup) const
  {
    auto best = current;
    double best_cost = std::numeric_limits<double>::infinity();
    const double desired_yaw = std::atan2(desired.y(), desired.x());
    const auto evaluate = [&](double q4) {
      auto trial = fixed;
      trial[0] = 0.0;
      trial[3] = q4;
      const auto base_normal = microphone_normal(trial);
      const double raw_q1 = desired_yaw - std::atan2(base_normal.y(), base_normal.x());
      double q1 = current[0] + std::remainder(raw_q1 - current[0], 2.0 * M_PI);
      q1 = std::clamp(q1, -3.0543, 3.0543);
      const Eigen::Vector3d normal = rz(q1) * base_normal;
      const double error = angle(normal, desired);
      const double d1 = q1 - current[0];
      const double d4 = q4 - current[3];
      const double cost = error * error + motion_penalty_ * (d1 * d1 + d4 * d4);
      auto candidate = fixed;
      candidate[0] = q1;
      candidate[3] = q4;
      return std::make_pair(cost, candidate);
    };

    double best_q4 = current[3];
    for (const auto & sample : lookup) {
      const auto [cost, candidate] = evaluate(sample.q4);
      if (cost < best_cost) {
        best_cost = cost;
        best = candidate;
        best_q4 = sample.q4;
      }
    }

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

  void begin_velocity_stop(ruckig::InputParameter<6> & input) {
    input.control_interface = ruckig::ControlInterface::Velocity;
    input.target_velocity.fill(0.0);
    input.target_acceleration.fill(0.0);
  }

  bool finish_tracking_controller_switch() {
    std::string error;
    const std::string traj_ctrl = dual_arm_ ? "dual_arm_controller" : "arm_controller";
    const std::string tracking_ctrl = dual_arm_ ? "dual_arm_tracking_controller" : "arm_tracking_controller";

    if (!switch_controllers({traj_ctrl}, {tracking_ctrl}, error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Could not restore trajectory controller after stopping: %s", error.c_str());
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    control_state_ = ControlState::Trajectory;
    arm1_locked_ = false;
    RCLCPP_INFO(get_logger(), "Tracking disabled cleanly; %s is active", traj_ctrl.c_str());
    return true;
  }

  void update() {
    Eigen::Vector3d desired;
    std::array<double, 6> actual {};
    std::array<double, 6> actual_right {};
    bool target_available = false;
    ControlState control_state = ControlState::Trajectory;
    bool external_veto = false;
    bool have_direction = false;
    double target_age = std::numeric_limits<double>::infinity();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto stamp = now();
      if (!have_joints_) return;
      external_veto = external_veto_;
      control_state = control_state_;
      have_direction = have_direction_;
      if (have_direction_) target_age = (stamp - last_target_).seconds();
      target_available = control_state == ControlState::Tracking &&
        !external_veto && have_direction &&
        target_age <= target_timeout_;
      desired = filtered_direction_;
      actual = current_;
      if (dual_arm_) actual_right = current_right_;
    }
    if (control_state == ControlState::Trajectory) return;

    // Safety check on tracking lag
    const double q1_err = std::abs(std::remainder(actual[0] - servo_input_.current_position[0], 2.0 * M_PI));
    const double q4_err = std::abs(actual[3] - servo_input_.current_position[3]);
    if (control_state == ControlState::Tracking && std::max(q1_err, q4_err) > max_tracking_error_) {
      RCLCPP_ERROR(get_logger(), "Arm 1 fell behind servo by %.3f rad; stopping tracking safely",
        std::max(q1_err, q4_err));
      std::lock_guard<std::mutex> lock(mutex_);
      control_state_ = ControlState::Stopping;
      control_state = ControlState::Stopping;
    }

    if (dual_arm_ && control_state == ControlState::Tracking) {
      const double q1_err_r = std::abs(std::remainder(actual_right[0] - servo_input_right_.current_position[0], 2.0 * M_PI));
      const double q4_err_r = std::abs(actual_right[3] - servo_input_right_.current_position[3]);
      if (std::max(q1_err_r, q4_err_r) > max_tracking_error_) {
        RCLCPP_ERROR(get_logger(), "Arm 2 fell behind servo by %.3f rad; stopping tracking safely",
          std::max(q1_err_r, q4_err_r));
        std::lock_guard<std::mutex> lock(mutex_);
        control_state_ = ControlState::Stopping;
        control_state = ControlState::Stopping;
      }
    }

    const double pointing_error = have_direction ? angle(microphone_normal(actual), desired) : 0.0;
    const bool should_track = (control_state == ControlState::Tracking) && target_available;

    if (should_track) {
      // 1. Solve Arm 1 target
      const auto candidate = solve(servo_input_.current_position, desired, fixed_, wrist_lookup_);
      const double update_error = angle(
        microphone_normal(candidate), microphone_normal(servo_input_.target_position));
      if (update_error >= angular_deadband_) {
        servo_input_.target_position = candidate;
        std_msgs::msg::String event;
        event.data = "target_updated";
        events_pub_->publish(event);
      }
      servo_input_.control_interface = ruckig::ControlInterface::Position;
      servo_input_.target_velocity.fill(0.0);
      servo_input_.target_acceleration.fill(0.0);

      // 2. Dual-arm coordination
      if (dual_arm_) {
        bool arm2_should_track = false;
        if (dual_tracking_policy_ == "simultaneous") {
          arm2_should_track = true;
        } else { // "first_arm_lead"
          if (pointing_error <= lead_lock_tolerance_rad_) {
            arm1_locked_ = true;
          }
          arm2_should_track = arm1_locked_;
        }

        if (arm2_should_track) {
          // Binocular convergence vector for Arm 2:
          // Target 3D point in world = target_nominal_distance * desired
          // Arm 2 base = (second_arm_x, second_arm_y, second_arm_z)
          Eigen::Vector3d desired_arm2;
          if (target_nominal_distance_ > 0.0) {
            const Eigen::Vector3d p_target = target_nominal_distance_ * desired;
            const Eigen::Vector3d p_arm2(second_arm_x_, second_arm_y_, second_arm_z_);
            desired_arm2 = (p_target - p_arm2).normalized();
          } else {
            desired_arm2 = desired; // Pure parallel gaze
          }

          const auto candidate_right = solve(
            servo_input_right_.current_position, desired_arm2, fixed_right_, wrist_lookup_right_);
          const double update_error_right = angle(
            microphone_normal(candidate_right), microphone_normal(servo_input_right_.target_position));
          if (update_error_right >= angular_deadband_) {
            servo_input_right_.target_position = candidate_right;
          }
          servo_input_right_.control_interface = ruckig::ControlInterface::Position;
          servo_input_right_.target_velocity.fill(0.0);
          servo_input_right_.target_acceleration.fill(0.0);
        } else {
          begin_velocity_stop(servo_input_right_);
        }
      }
    } else {
      begin_velocity_stop(servo_input_);
      if (dual_arm_) {
        begin_velocity_stop(servo_input_right_);
        arm1_locked_ = false;
      }
    }

    // Update Ruckig for Arm 1
    const auto result = otg_->update(servo_input_, servo_output_);
    if (result < ruckig::Result::Working) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ruckig rejected Arm 1 servo state (%d); holding last command", static_cast<int>(result));
      return;
    }
    servo_command_ = servo_output_.new_position;
    servo_command_[1] = fixed_[1];
    servo_command_[2] = fixed_[2];
    servo_command_[4] = fixed_[4];
    servo_command_[5] = fixed_[5];
    servo_output_.pass_to_input(servo_input_);

    // Update Ruckig for Arm 2 (if dual arm)
    ruckig::Result result_right = ruckig::Result::Finished;
    if (dual_arm_) {
      result_right = otg_right_->update(servo_input_right_, servo_output_right_);
      if (result_right < ruckig::Result::Working) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "Ruckig rejected Arm 2 servo state (%d); holding last command", static_cast<int>(result_right));
        return;
      }
      servo_command_right_ = servo_output_right_.new_position;
      servo_command_right_[1] = fixed_right_[1];
      servo_command_right_[2] = fixed_right_[2];
      servo_command_right_[4] = fixed_right_[4];
      servo_command_right_[5] = fixed_right_[5];
      servo_output_right_.pass_to_input(servo_input_right_);
    }

    // Publish servo command
    if (!dual_arm_) {
      publish_servo_command({servo_command_.begin(), servo_command_.end()});
    } else {
      std::vector<double> dual_cmd;
      dual_cmd.insert(dual_cmd.end(), servo_command_.begin(), servo_command_.end());
      dual_cmd.insert(dual_cmd.end(), servo_command_right_.begin(), servo_command_right_.end());
      publish_servo_command(dual_cmd);
    }

    write_servo_log(control_state, target_available, servo_input_.target_position, actual, pointing_error);

    if (control_state == ControlState::Stopping) {
      bool all_finished = (result == ruckig::Result::Finished);
      if (dual_arm_) all_finished = all_finished && (result_right == ruckig::Result::Finished);
      if (all_finished) {
        finish_tracking_controller_switch();
        return;
      }
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "servo status: state=%s age=%.2fs arm1_err=%.1fdeg locked=%d",
      should_track ? "tracking" : "holding",
      target_age,
      pointing_error * 180.0 / M_PI,
      arm1_locked_ ? 1 : 0);
  }

  // Configuration and state
  bool dual_arm_{false};
  double second_arm_x_{-0.75}, second_arm_y_{0.0}, second_arm_z_{0.0};
  double target_nominal_distance_{0.0};
  std::string dual_tracking_policy_{"first_arm_lead"};
  double lead_lock_tolerance_rad_{0.10};
  bool arm1_locked_{false};

  const std::vector<std::string> names_single_{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  const std::vector<std::string> names_left_{
    "left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6"};
  const std::vector<std::string> names_right_{
    "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6"};

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr fused_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yolo_sub_;
  rclcpp::Subscription<odas_ros_msgs::msg::OdasSstArrayStamped>::SharedPtr sst_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr ast_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gre_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr mobilenetv2_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tracking_service_;
  rclcpp::CallbackGroup::SharedPtr switch_callback_group_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex mutex_;

  // Arm 1 (or single arm) Ruckig & state
  std::array<double, 6> current_ {}, fixed_ {}, servo_command_ {};
  ruckig::InputParameter<6> servo_input_;
  ruckig::OutputParameter<6> servo_output_;
  std::array<double, 6> previous_output_acceleration_ {};
  bool have_previous_output_acceleration_{false};
  std::unique_ptr<ruckig::Ruckig<6>> otg_;
  std::vector<WristSample> wrist_lookup_;

  // Arm 2 (Right Arm in dual-arm mode) Ruckig & state
  std::array<double, 6> current_right_ {}, fixed_right_ {}, servo_command_right_ {};
  ruckig::InputParameter<6> servo_input_right_;
  ruckig::OutputParameter<6> servo_output_right_;
  std::array<double, 6> previous_output_acceleration_right_ {};
  bool have_previous_output_acceleration_right_{false};
  std::unique_ptr<ruckig::Ruckig<6>> otg_right_;
  std::vector<WristSample> wrist_lookup_right_;

  Eigen::Vector3d filtered_direction_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d previous_yolo_direction_{Eigen::Vector3d::UnitX()};
  rclcpp::Time last_target_{0, 0, RCL_ROS_TIME};
  bool have_joints_{false}, external_veto_{false}, have_direction_{false};
  bool have_previous_yolo_direction_{false};
  ControlState control_state_{ControlState::Trajectory};
  bool home_requested_{false}, home_reached_{false};
  bool ekf_enabled_{true};

  double target_timeout_, smoothing_alpha_, angular_deadband_;
  double motion_penalty_;
  double command_rate_hz_, max_velocity_, max_acceleration_;
  double max_jerk_, max_tracking_error_;
  double direct_min_activity_, direct_class_timeout_;
  bool require_home_;
  double home_duration_s_, home_tolerance_rad_;
  std::string world_frame_, tracking_frame_;
  std::string direct_classifier_source_;
  std::string ekf_direction_log_path_;
  std::string yolo_direction_log_path_;
  std::string servo_log_path_;
  std::ofstream ekf_direction_log_;
  std::ofstream yolo_direction_log_;
  std::ofstream servo_log_;

  const std::array<double, 6> home_{
    M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, M_PI / 2.0, 0.0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmTracker>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
