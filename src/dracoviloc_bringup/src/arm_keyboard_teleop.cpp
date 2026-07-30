#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class TerminalGuard
{
public:
  TerminalGuard()
  {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error("keyboard teleop must run in an interactive terminal");
    }
    tcgetattr(STDIN_FILENO, &original_);
    auto raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~TerminalGuard() {tcsetattr(STDIN_FILENO, TCSANOW, &original_);}

private:
  struct termios original_ {};
};

class ArmKeyboardTeleop : public rclcpp::Node
{
public:
  ArmKeyboardTeleop()
  : Node("arm_keyboard_teleop")
  {
    command_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {update_state(*msg);});
    timer_ = create_wall_timer(50ms, [this]() {read_key();});
    RCLCPP_INFO(
      get_logger(),
      "1..6 select joint | a/- decrease | d/+ increase | [/] change radian step | "
      "p print state | h home | q quit");
  }

private:
  void update_state(const sensor_msgs::msg::JointState & msg)
  {
    std::array<double, 6> ordered {};
    for (size_t joint = 0; joint < joint_names_.size(); ++joint) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), joint_names_[joint]);
      if (it == msg.name.end()) {
        return;
      }
      ordered[joint] = msg.position[std::distance(msg.name.begin(), it)];
    }
    positions_ = ordered;
    if (!have_state_) {
      target_ = ordered;
      RCLCPP_INFO(get_logger(), "Joint states received; teleop is ready (step %.3f rad)", step_);
    }
    have_state_ = true;
  }

  void send(const std::array<double, 6> & target)
  {
    if (command_publisher_->get_subscription_count() == 0) {
      RCLCPP_ERROR(get_logger(), "arm_controller command topic has no subscriber");
      return;
    }
    trajectory_msgs::msg::JointTrajectory command;
    command.joint_names.assign(joint_names_.begin(), joint_names_.end());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(target.begin(), target.end());
    point.time_from_start = rclcpp::Duration::from_seconds(command_duration_);
    command.points.push_back(point);
    RCLCPP_INFO(
      get_logger(), "joint%zu target %.4f rad",
      selected_ + 1, target[selected_]);
    command_publisher_->publish(command);
    target_ = target;
  }

  void read_key()
  {
    char key;
    if (::read(STDIN_FILENO, &key, 1) != 1) {
      return;
    }
    if (key >= '1' && key <= '6') {
      selected_ = static_cast<size_t>(key - '1');
      RCLCPP_INFO(get_logger(), "Selected joint%zu", selected_ + 1);
    } else if (key == 'q' || key == 'Q') {
      rclcpp::shutdown();
    } else if (key == 'h' || key == 'H') {
      send(home_);
    } else if (key == '[') {
      step_ = std::max(0.001, step_ - 0.01);
      RCLCPP_INFO(get_logger(), "Step: %.3f rad", step_);
    } else if (key == ']') {
      step_ += 0.01;
      RCLCPP_INFO(get_logger(), "Step: %.3f rad", step_);
    } else if (key == 'p' || key == 'P') {
      if (!have_state_) {
        RCLCPP_WARN(get_logger(), "No /joint_states message received yet");
      } else {
        RCLCPP_INFO(
          get_logger(), "q = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad",
          positions_[0], positions_[1], positions_[2],
          positions_[3], positions_[4], positions_[5]);
      }
    } else if (
      key == '+' || key == '=' || key == 'd' || key == 'D' ||
      key == '-' || key == '_' || key == 'a' || key == 'A')
    {
      if (!have_state_) {
        RCLCPP_WARN(get_logger(), "Cannot move: no /joint_states message received yet");
        return;
      }
      auto target = target_;
      const bool increase = key == '+' || key == '=' || key == 'd' || key == 'D';
      target[selected_] += increase ? step_ : -step_;
      send(target);
    }
  }

  const std::array<std::string, 6> joint_names_{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  const std::array<double, 6> home_{
    M_PI / 2.0, -M_PI / 2.0, -M_PI / 2.0, 0.0, M_PI / 2.0, 0.0};
  std::array<double, 6> positions_ {};
  std::array<double, 6> target_ {};
  bool have_state_{false};
  size_t selected_{0};
  double step_{0.05};
  double command_duration_{0.2};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  try {
    TerminalGuard terminal;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmKeyboardTeleop>());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  } catch (const std::exception & error) {
    std::fprintf(stderr, "arm_keyboard_teleop: %s\n", error.what());
    return 1;
  }
  return 0;
}
