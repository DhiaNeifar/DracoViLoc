#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TerminalGuard {
public:
  TerminalGuard() {
    if (!isatty(STDIN_FILENO)) {throw std::runtime_error("interactive terminal required");}
    tcgetattr(STDIN_FILENO, &old_);
    auto raw = old_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  ~TerminalGuard() {tcsetattr(STDIN_FILENO, TCSANOW, &old_);}
private:
  termios old_{};
};

class DroneKeyboardTeleop : public rclcpp::Node {
public:
  DroneKeyboardTeleop() : Node("drone_keyboard_teleop") {
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/drone/cmd_vel", 10);
    timer_ = create_wall_timer(50ms, [this]() {read_key();});
    RCLCPP_INFO(get_logger(), "w/s forward | a/d left/right | r/f up/down | q/e yaw | space stop | x reset | Esc exit");
  }
private:
  void read_key() {
    char key;
    if (::read(STDIN_FILENO, &key, 1) != 1) {return;}
    geometry_msgs::msg::Twist cmd;
    constexpr double linear = 0.35;
    constexpr double yaw = 0.7;
    if (key == 'w') cmd.linear.x = linear;
    else if (key == 's') cmd.linear.x = -linear;
    else if (key == 'a') cmd.linear.y = linear;
    else if (key == 'd') cmd.linear.y = -linear;
    else if (key == 'r') cmd.linear.z = linear;
    else if (key == 'f') cmd.linear.z = -linear;
    else if (key == 'q') cmd.angular.z = yaw;
    else if (key == 'e') cmd.angular.z = -yaw;
    else if (key == 'x') {
      cmd.angular.x = 1.0;  // reset sentinel
    } else if (key == 27) {
      rclcpp::shutdown(); return;
    }
    pub_->publish(cmd);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  try {
    TerminalGuard terminal;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneKeyboardTeleop>());
    if (rclcpp::ok()) rclcpp::shutdown();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "drone_keyboard_teleop: %s\n", e.what());
    return 1;
  }
  return 0;
}
