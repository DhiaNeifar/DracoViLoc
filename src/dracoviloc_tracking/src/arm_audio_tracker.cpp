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
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

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
    command_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {joint_callback(*msg);});
    direction_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/audio/target_direction", 10,
      [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        if (msg->header.frame_id != "world") {return;}
        Eigen::Vector3d direction(msg->vector.x, msg->vector.y, msg->vector.z);
        if (direction.norm() < 1e-6) {return;}
        direction.normalize();
        const auto stamp = now();
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_direction_) {filtered_direction_ = direction; have_direction_ = true;}
        else {filtered_direction_ = (
            (1.0 - smoothing_alpha_) * filtered_direction_ +
            smoothing_alpha_ * direction).normalized();}
        last_target_ = stamp;
      });
    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/audio/target_valid", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        valid_ = msg->data;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {update();});
    RCLCPP_INFO(get_logger(),
      "Continuous audio servo at 20 Hz: only joint1 and joint4 may move");
  }
private:
  struct WristSample {double q4; Eigen::Vector3d normal;};
  static double angle(const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
    return std::acos(std::clamp(a.dot(b), -1.0, 1.0));
  }
  static Eigen::AngleAxisd rz(double value) {
    return Eigen::AngleAxisd(value, Eigen::Vector3d::UnitZ());
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
      wrist_lookup_.reserve(630);
      for (double q4 = -2.0 * M_PI; q4 <= 2.0 * M_PI; q4 += 0.02) {
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
      const double d4 = std::remainder(sample.q4 - current[3], 2.0 * M_PI);
      const double cost = error * error + motion_penalty_ * (d1 * d1 + d4 * d4);
      if (cost < best_cost) {
        best_cost = cost;
        best = fixed_;
        best[0] = q1;
        best[3] = current[3] + d4;
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
    {
      std::lock_guard<std::mutex> lock(mutex_); const auto stamp = now();
      if (!have_joints_) {return;}
      target_available = valid_ && have_direction_ &&
        (stamp - last_target_).seconds() <= target_timeout_;
      desired = filtered_direction_; reference = commanded_;
    }
    if (command_pub_->get_subscription_count() == 0) {return;}
    auto target = reference;
    if (target_available && angle(microphone_normal(reference), desired) >= angular_deadband_) {
      target = solve(reference, desired);
    }
    for (const std::size_t index : {std::size_t(0), std::size_t(3)}) {
      double error = target[index] - commanded_[index];
      if (index == 3) {error = std::remainder(error, 2.0 * M_PI);}
      const double requested_velocity = target_available ?
        std::clamp(error / command_horizon_, -max_velocity_, max_velocity_) : 0.0;
      const double velocity_change = std::clamp(
        requested_velocity - velocities_[index],
        -max_acceleration_ * dt, max_acceleration_ * dt);
      velocities_[index] += velocity_change;
      commanded_[index] += velocities_[index] * dt;
    }
    commanded_[0] = std::clamp(commanded_[0], -3.0543, 3.0543);
    commanded_[1] = fixed_[1]; commanded_[2] = fixed_[2];
    commanded_[4] = fixed_[4]; commanded_[5] = fixed_[5];
    trajectory_msgs::msg::JointTrajectory command;
    command.joint_names.assign(names_.begin(), names_.end());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(commanded_.begin(), commanded_.end());
    point.time_from_start = rclcpp::Duration::from_seconds(command_horizon_);
    command.points.push_back(point); command_pub_->publish(command);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "servo q1=%.3f q4=%.3f target=%s; q2/q3/q5/q6 fixed",
      commanded_[0], commanded_[3], target_available ? "audio" : "hold");
  }
  const std::array<std::string, 6> names_{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr direction_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::TimerBase::SharedPtr timer_; mutable std::mutex mutex_;
  std::array<double, 6> current_ {}, fixed_ {}, commanded_ {}, velocities_ {};
  std::vector<WristSample> wrist_lookup_;
  std::chrono::steady_clock::time_point last_update_{std::chrono::steady_clock::now()};
  Eigen::Vector3d filtered_direction_{Eigen::Vector3d::UnitX()};
  rclcpp::Time last_target_{0, 0, RCL_ROS_TIME};
  bool have_joints_{false}, valid_{false}, have_direction_{false};
  double target_timeout_, smoothing_alpha_, angular_deadband_, motion_penalty_;
  double command_horizon_, max_velocity_, max_acceleration_;
};
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<ArmAudioTracker>());
  rclcpp::shutdown(); return 0;
}
