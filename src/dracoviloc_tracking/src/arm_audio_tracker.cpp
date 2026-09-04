#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "odas_ros_msgs/msg/odas_sst_array_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

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
    angular_deadband_ = declare_parameter("angular_deadband", 0.04);
    motion_penalty_ = declare_parameter("motion_penalty", 0.015);
    command_horizon_ = declare_parameter("command_horizon", 0.25);
    max_velocity_ = declare_parameter("max_velocity", 0.60);
    max_acceleration_ = declare_parameter("max_acceleration", 0.80);
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
    if (direct_classifier_source_ != "gre" &&
        direct_classifier_source_ != "ast" &&
        direct_classifier_source_ != "either" &&
        direct_classifier_source_ != "yolo") {
      throw std::invalid_argument(
        "direct_classifier_source must be gre, ast, either, or yolo");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {joint_callback(*msg);});

    if (ekf_enabled_) {
      fused_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/ekf/direction", 10,
        [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
          direction_callback(*msg);
        });
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
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {update();});
    RCLCPP_INFO(get_logger(), "Servo at 20 Hz in %s mode%s; only joint1 and joint4 move",
      ekf_enabled_ ? "EKF" : "direct",
      ekf_enabled_ ? "" : (" (source=" + direct_classifier_source_ + ")").c_str());
  }

private:
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
    accept_local_direction(local.normalized(), msg.header.frame_id);
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
    const Eigen::Vector3d & local, const std::string & frame_id)
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

    const auto stamp = now();
    std::lock_guard<std::mutex> lock(mutex_);
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
    last_target_ = stamp;
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
      fixed_ = ordered;
      commanded_ = ordered;
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
    have_joints_ = true;
  }

  std::array<double, 6> solve(
    const std::array<double, 6> & current, const Eigen::Vector3d & desired) const
  {
    auto best = current;
    double best_cost = std::numeric_limits<double>::infinity();
    const double desired_yaw = std::atan2(desired.y(), desired.x());
    for (const auto & sample : wrist_lookup_) {
      double q1 = std::remainder(
        desired_yaw - std::atan2(sample.normal.y(), sample.normal.x()),
        2.0 * M_PI);
      q1 = std::clamp(q1, -3.0543, 3.0543);
      const Eigen::Vector3d normal = rz(q1) * sample.normal;
      const double error = angle(normal, desired);
      const double d1 = q1 - current[0];
      const double d4 = sample.q4 - current[3];
      const double cost = error * error + motion_penalty_ * (d1 * d1 + d4 * d4);
      if (cost < best_cost) {
        best_cost = cost;
        best = fixed_;
        best[0] = q1;
        best[3] = sample.q4;
      }
    }
    return best;
  }

  void update() {
    const auto steady_now = std::chrono::steady_clock::now();
    const double dt = std::clamp(
      std::chrono::duration<double>(steady_now - last_update_).count(), 0.001, 0.15);
    last_update_ = steady_now;
    Eigen::Vector3d desired; std::array<double, 6> reference;
    bool target_available = false;
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
      have_direction = have_direction_;
      if (have_direction_) {
        target_age = (stamp - last_target_).seconds();
      }
      target_available = !external_veto && have_direction && target_age <= target_timeout_;
      desired = filtered_direction_; reference = commanded_;
    }
    if (command_pub_->get_subscription_count() == 0) {return;}
    auto target = reference;
    const double pointing_error = have_direction ?
      angle(microphone_normal(reference), desired) : 0.0;
    if (target_available && pointing_error >= angular_deadband_) {
      target = solve(reference, desired);
    }
    const double solution_error = have_direction ?
      angle(microphone_normal(target), desired) : 0.0;
    for (const std::size_t index : {std::size_t(0), std::size_t(3)}) {
      double error = target[index] - commanded_[index];
      const double requested_velocity = target_available ?
        std::clamp(error / command_horizon_, -max_velocity_, max_velocity_) : 0.0;
      const double velocity_change = std::clamp(
        requested_velocity - velocities_[index],
        -max_acceleration_ * dt, max_acceleration_ * dt);
      velocities_[index] += velocity_change;
      commanded_[index] += velocities_[index] * dt;
    }
    commanded_[0] = std::clamp(commanded_[0], -3.0543, 3.0543);
    commanded_[3] = std::clamp(commanded_[3], -M_PI_2, M_PI_2);
    commanded_[1] = fixed_[1]; commanded_[2] = fixed_[2];
    commanded_[4] = fixed_[4]; commanded_[5] = fixed_[5];
    trajectory_msgs::msg::JointTrajectory command;
    command.joint_names.assign(names_.begin(), names_.end());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(commanded_.begin(), commanded_.end());
    point.time_from_start = rclcpp::Duration::from_seconds(command_horizon_);
    command.points.push_back(point); command_pub_->publish(command);
    const char * state = target_available ?
      (pointing_error < angular_deadband_ ? "reached/deadband" : "tracking") :
      (external_veto ? "hold/veto" :
       (!have_direction ? "hold/no-target" : "hold/stale"));
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "servo q1=%.3f q4=%.3f state=%s age=%.2fs "
      "error=%.1fdeg solution_error=%.1fdeg target_q1=%.3f target_q4=%.3f "
      "desired_world=(%.2f,%.2f,%.2f); q2/q3/q5/q6 fixed",
      commanded_[0], commanded_[3], state, target_age,
      pointing_error * 180.0 / M_PI, solution_error * 180.0 / M_PI,
      target[0], target[3], desired.x(), desired.y(), desired.z());
  }

  const std::array<std::string, 6> names_{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr fused_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yolo_sub_;
  rclcpp::Subscription<odas_ros_msgs::msg::OdasSstArrayStamped>::SharedPtr sst_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr ast_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gre_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_; mutable std::mutex mutex_;
  std::array<double, 6> current_ {}, fixed_ {}, commanded_ {}, velocities_ {};
  std::vector<WristSample> wrist_lookup_;
  std::chrono::steady_clock::time_point last_update_{std::chrono::steady_clock::now()};
  Eigen::Vector3d filtered_direction_{Eigen::Vector3d::UnitX()};
  rclcpp::Time last_target_{0, 0, RCL_ROS_TIME};
  bool have_joints_{false}, external_veto_{false}, have_direction_{false};
  bool ekf_enabled_{true};
  Verdict ast_verdict_{false, 0.0, rclcpp::Time(0, 0, RCL_ROS_TIME)};
  Verdict gre_verdict_{false, 0.0, rclcpp::Time(0, 0, RCL_ROS_TIME)};
  bool have_ast_verdict_{false}, have_gre_verdict_{false};
  double target_timeout_, smoothing_alpha_, angular_deadband_, motion_penalty_;
  double command_horizon_, max_velocity_, max_acceleration_;
  double direct_min_activity_, direct_class_timeout_;
  std::string world_frame_, tracking_frame_;
  std::string direct_classifier_source_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<ArmAudioTracker>());
  rclcpp::shutdown(); return 0;
}
