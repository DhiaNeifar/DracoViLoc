#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ignition/msgs/boolean.pb.h"
#include "ignition/msgs/pose.pb.h"
#include "ignition/transport/Node.hh"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class DronePoseController : public rclcpp::Node
{
public:
  DronePoseController()
  : Node("drone_pose_controller"), broadcaster_(*this)
  {
    initial_x_ = declare_parameter("initial_x", 0.0);
    initial_y_ = declare_parameter("initial_y", 0.0);
    initial_z_ = declare_parameter("initial_z", 1.2);
    min_x_ = declare_parameter("min_x", -2.0);
    max_x_ = declare_parameter("max_x", 3.0);
    min_y_ = declare_parameter("min_y", -2.0);
    max_y_ = declare_parameter("max_y", 2.0);
    min_z_ = declare_parameter("min_z", 0.4);
    max_z_ = declare_parameter("max_z", 2.5);
    pose_.position.x = initial_x_;
    pose_.position.y = initial_y_;
    pose_.position.z = initial_z_;
    world_name_ = declare_parameter("world_name", "empty_no_ground");
    pose_.orientation.w = 1.0;
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/drone/pose", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/drone/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {cmd_ = *msg;});
    set_pose_service_ = "/world/" + world_name_ + "/set_pose";
    last_update_ = std::chrono::steady_clock::now();
    startup_time_ = last_update_;
    timer_ = create_wall_timer(50ms, [this]() {update();});
  }

private:
  void update()
  {
    const auto stamp = now();
    const auto steady_now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(steady_now - last_update_).count();
    last_update_ = steady_now;
    if (dt <= 0.0 || dt > 0.25) {dt = 0.05;}

    if (cmd_.angular.x > 0.5) {
      pose_.position.x = initial_x_;
      pose_.position.y = initial_y_;
      pose_.position.z = initial_z_;
      yaw_ = 0.0;
      cmd_ = geometry_msgs::msg::Twist();
    }
    yaw_ += cmd_.angular.z * dt;
    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    pose_.position.x += (c * cmd_.linear.x - s * cmd_.linear.y) * dt;
    pose_.position.y += (s * cmd_.linear.x + c * cmd_.linear.y) * dt;
    pose_.position.z += cmd_.linear.z * dt;
    pose_.position.x = std::clamp(pose_.position.x, min_x_, max_x_);
    pose_.position.y = std::clamp(pose_.position.y, min_y_, max_y_);
    pose_.position.z = std::clamp(pose_.position.z, min_z_, max_z_);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    pose_.orientation.x = q.x(); pose_.orientation.y = q.y();
    pose_.orientation.z = q.z(); pose_.orientation.w = q.w();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "world";
    pose_msg.pose = pose_;
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose_msg.header;
    transform.child_frame_id = "drone_base_link";
    transform.transform.translation.x = pose_.position.x;
    transform.transform.translation.y = pose_.position.y;
    transform.transform.translation.z = pose_.position.z;
    transform.transform.rotation = pose_.orientation;
    broadcaster_.sendTransform(transform);

    if (std::chrono::duration<double>(steady_now - startup_time_).count() >= 1.0) {
      ignition::msgs::Pose request;
      request.set_name("drone");
      request.mutable_position()->set_x(pose_.position.x);
      request.mutable_position()->set_y(pose_.position.y);
      request.mutable_position()->set_z(pose_.position.z);
      request.mutable_orientation()->set_x(pose_.orientation.x);
      request.mutable_orientation()->set_y(pose_.orientation.y);
      request.mutable_orientation()->set_z(pose_.orientation.z);
      request.mutable_orientation()->set_w(pose_.orientation.w);
      ignition::msgs::Boolean response;
      bool result = false;
      gz_node_.Request(set_pose_service_, request, 20u, response, result);
    }
  }

  std::string world_name_;
  double initial_x_{0.0}, initial_y_{0.0}, initial_z_{1.2};
  double min_x_{-2.0}, max_x_{3.0};
  double min_y_{-2.0}, max_y_{2.0};
  double min_z_{0.4}, max_z_{2.5};
  geometry_msgs::msg::Pose pose_;
  geometry_msgs::msg::Twist cmd_;
  double yaw_{0.0};
  std::chrono::steady_clock::time_point last_update_;
  std::chrono::steady_clock::time_point startup_time_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::string set_pose_service_;
  ignition::transport::Node gz_node_;
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DronePoseController>());
  rclcpp::shutdown();
  return 0;
}
