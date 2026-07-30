#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Geometry>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

class ArmDroneTracker
{
public:
  explicit ArmDroneTracker(const rclcpp::Node::SharedPtr & node)
  : node_(node), buffer_(node_->get_clock()), listener_(buffer_),
    move_group_(node_, node_->declare_parameter("planning_group", "arm"))
  {
    enabled_ = node_->declare_parameter("tracking_enabled", true);
    rate_ = node_->declare_parameter("tracking_rate", 5.0);
    deadband_ = node_->declare_parameter("angular_deadband", 0.06);
    position_deadband_ = node_->declare_parameter("drone_position_deadband", 0.01);
    max_speed_ = node_->declare_parameter("max_angular_speed", 0.8);
    planning_frame_ = node_->declare_parameter("planning_frame", "world");
    camera_frame_ = node_->declare_parameter("camera_frame", "d435i_link");
    drone_frame_ = node_->declare_parameter("drone_frame", "drone_base_link");
    const auto marker_qos = rclcpp::QoS(1).reliable().transient_local();
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/tracking/desired_ray", marker_qos);
    joint_state_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions joint_state_options;
    joint_state_options.callback_group = joint_state_group_;
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (std::size_t index = 0;
          index < message->name.size() && index < message->position.size(); ++index)
        {
          joint_positions_[message->name[index]] = message->position[index];
        }
      },
      joint_state_options);
    move_group_.setPlanningTime(0.25);
    move_group_.setNumPlanningAttempts(1);
    const double velocity_scaling =
      std::clamp(max_speed_ / 3.2, 0.02, 0.25);
    move_group_.setMaxVelocityScalingFactor(velocity_scaling);
    move_group_.setMaxAccelerationScalingFactor(0.25);
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.2, rate_));
    timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {update();});
    RCLCPP_INFO(
      node_->get_logger(), "Tracking %s +Z toward %s using MoveIt group %s",
      camera_frame_.c_str(), drone_frame_.c_str(), move_group_.getName().c_str());
  }

private:
  void update()
  {
    if (!enabled_ || busy_) {return;}
    geometry_msgs::msg::TransformStamped camera_tf;
    geometry_msgs::msg::TransformStamped drone_tf;
    try {
      camera_tf = buffer_.lookupTransform(planning_frame_, camera_frame_, tf2::TimePointZero);
      drone_tf = buffer_.lookupTransform(planning_frame_, drone_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "%s", error.what());
      return;
    }

    const Eigen::Vector3d camera(
      camera_tf.transform.translation.x, camera_tf.transform.translation.y,
      camera_tf.transform.translation.z);
    const Eigen::Vector3d drone(
      drone_tf.transform.translation.x, drone_tf.transform.translation.y,
      drone_tf.transform.translation.z);
    Eigen::Vector3d direction = drone - camera;
    if (direction.norm() < 1e-4) {return;}
    direction.normalize();

    Eigen::Quaterniond current(
      camera_tf.transform.rotation.w, camera_tf.transform.rotation.x,
      camera_tf.transform.rotation.y, camera_tf.transform.rotation.z);
    const Eigen::Vector3d current_forward = current * Eigen::Vector3d::UnitZ();
    const double error = std::acos(std::clamp(current_forward.dot(direction), -1.0, 1.0));
    publish_marker(camera, drone);
    // Drone orientation is deliberately irrelevant. Do not replan for yaw-only
    // updates, or repeatedly solve the same stationary target.
    if (have_last_target_ && (drone - last_target_position_).norm() < position_deadband_) {
      return;
    }
    if (error < deadband_) {return;}

    // Apply only the shortest rotation needed to align camera +Z with the
    // target. Do not impose a world-up roll constraint: optical-axis roll is
    // irrelevant for pointing and would cause unnecessary joint motion.
    const Eigen::Quaterniond pointing_delta =
      Eigen::Quaterniond::FromTwoVectors(current_forward, direction);
    Eigen::Quaterniond target_q = pointing_delta * current;
    target_q.normalize();

    geometry_msgs::msg::Pose target;
    target.position.x = camera.x();
    target.position.y = camera.y();
    target.position.z = camera.z();
    target.orientation.x = target_q.x();
    target.orientation.y = target_q.y();
    target.orientation.z = target_q.z();
    target.orientation.w = target_q.w();

    busy_ = true;
    moveit::core::RobotState current_state(move_group_.getRobotModel());
    current_state.setToDefaultValues();
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      for (const auto & joint_name : move_group_.getJointNames()) {
        const auto position = joint_positions_.find(joint_name);
        if (position == joint_positions_.end()) {
          RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 3000,
            "Waiting for all arm joints on /joint_states before tracking");
          busy_ = false;
          return;
        }
        current_state.setVariablePosition(joint_name, position->second);
      }
    }
    current_state.update();
    try {
      moveit::core::RobotState goal_state(current_state);
      const auto * joint_group =
        goal_state.getJointModelGroup(move_group_.getName());
      if (!joint_group ||
        !goal_state.setFromIK(joint_group, target, camera_frame_, 0.05))
      {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 3000,
          "No nearby IK solution for the requested look-at pose");
        busy_ = false;
        return;
      }
      move_group_.setStartState(current_state);
      move_group_.setJointValueTarget(goal_state);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned =
        move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      if (planned) {
        const auto result = move_group_.execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
          last_target_position_ = drone;
          have_last_target_ = true;
        } else {
          RCLCPP_WARN(node_->get_logger(), "Tracking execution failed");
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 3000,
          "No reachable look-at solution (angular error %.2f rad)", error);
      }
      move_group_.clearPoseTargets();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        node_->get_logger(), "MoveIt tracking update failed safely: %s", exception.what());
    }
    busy_ = false;
  }

  void publish_marker(const Eigen::Vector3d & from, const Eigen::Vector3d & to)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = node_->now();
    marker.ns = "drone_tracking";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point p;
    p.x = from.x(); p.y = from.y(); p.z = from.z(); marker.points.push_back(p);
    p.x = to.x(); p.y = to.y(); p.z = to.z(); marker.points.push_back(p);
    marker.scale.x = 0.015; marker.scale.y = 0.035; marker.scale.z = 0.05;
    marker.color.r = 1.0; marker.color.g = 0.25; marker.color.b = 0.05; marker.color.a = 1.0;
    marker_pub_->publish(marker);
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::CallbackGroup::SharedPtr joint_state_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex joint_state_mutex_;
  std::unordered_map<std::string, double> joint_positions_;
  std::string planning_frame_, camera_frame_, drone_frame_;
  bool enabled_{true};
  bool have_last_target_{false};
  Eigen::Vector3d last_target_position_{Eigen::Vector3d::Zero()};
  double rate_{5.0}, deadband_{0.06}, position_deadband_{0.01}, max_speed_{0.8};
  std::atomic_bool busy_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("arm_drone_tracker");
  auto tracker = std::make_shared<ArmDroneTracker>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  tracker.reset();
  rclcpp::shutdown();
  return 0;
}
