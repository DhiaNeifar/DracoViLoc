#include "fairino_hardware/joystick_controller.hpp"

#include <chrono>
#include <cmath>

using std::placeholders::_1;

JoystickController::JoystickController()
: Node("fairino_joystick_controller"),
  active_robot_(1),
  max_command_scale_(0.3),
  threshold_(0.1),
  last_was_jogging_(false),
  last_button1_state_(false)
{
    this->declare_parameter<double>("max_vel_percent", 30.0);
    this->declare_parameter<double>("threshold", 0.1);
    this->declare_parameter<double>("control_rate_hz", 50.0);

    max_command_scale_ = this->get_parameter("max_vel_percent").as_double() / 100.0;
    threshold_ = this->get_parameter("threshold").as_double();
    const double control_rate_hz = this->get_parameter("control_rate_hz").as_double();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        10,
        std::bind(&JoystickController::joyCallback, this, _1));

    twist_pub_left_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node_left/delta_twist_cmds",
        10);

    twist_pub_right_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node_right/delta_twist_cmds",
        10);

    const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
        std::bind(&JoystickController::controlTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Fairino joystick controller using MoveIt Servo");
    RCLCPP_INFO(this->get_logger(), "Publishes to /servo_node_left/right/delta_twist_cmds");
    RCLCPP_INFO(this->get_logger(), "Max command scale: %.2f", max_command_scale_);
    RCLCPP_INFO(this->get_logger(), "Press X/button 1 to switch active robot");
}

JoystickController::~JoystickController()
{
    sendStopTwist();
}

void JoystickController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joy_lock_);
    last_joy_msg_ = msg;
}

void JoystickController::controlTimerCallback()
{
    sensor_msgs::msg::Joy::SharedPtr joy_msg;
    {
        std::lock_guard<std::mutex> lock(joy_lock_);
        joy_msg = last_joy_msg_;
    }

    if (!joy_msg || joy_msg->axes.size() < 5 || joy_msg->buttons.size() < 8) {
        if (last_was_jogging_) {
            sendStopTwist();
            last_was_jogging_ = false;
        }
        return;
    }

    const bool button1_pressed = joy_msg->buttons[1] != 0;
    if (button1_pressed && !last_button1_state_) {
        std::lock_guard<std::mutex> lock(robot_lock_);
        active_robot_ = (active_robot_ == 1) ? 2 : 1;
        RCLCPP_INFO(
            this->get_logger(),
            "Now controlling robot %d (%s arm)",
            active_robot_,
            active_robot_ == 1 ? "left" : "right");
    }
    last_button1_state_ = button1_pressed;

    const double axis_x = joy_msg->axes[0];
    const double axis_y = joy_msg->axes[1];
    const double axis_z = joy_msg->axes[4];
    const int btn_square = joy_msg->buttons[0];
    const int btn_circle = joy_msg->buttons[2];
    const int btn_l1 = joy_msg->buttons[4];
    const int btn_r1 = joy_msg->buttons[5];
    const int btn_l2 = joy_msg->buttons[6];
    const int btn_r2 = joy_msg->buttons[7];

    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;
    std::string desc;

    if (std::abs(axis_x) > threshold_) {
        vx = axis_x * max_command_scale_;
        desc += axis_x > 0.0 ? "+X " : "-X ";
    }
    if (std::abs(axis_y) > threshold_) {
        vy = axis_y * max_command_scale_;
        desc += axis_y > 0.0 ? "+Y " : "-Y ";
    }
    if (std::abs(axis_z) > threshold_) {
        vz = axis_z * max_command_scale_;
        desc += axis_z > 0.0 ? "+Z " : "-Z ";
    }

    if (btn_r1) {
        wx = max_command_scale_;
        desc += "+A ";
    } else if (btn_r2) {
        wx = -max_command_scale_;
        desc += "-A ";
    }

    if (btn_l1) {
        wy = max_command_scale_;
        desc += "+B ";
    } else if (btn_l2) {
        wy = -max_command_scale_;
        desc += "-B ";
    }

    if (btn_circle) {
        wz = max_command_scale_;
        desc += "+C ";
    } else if (btn_square) {
        wz = -max_command_scale_;
        desc += "-C ";
    }

    const bool jogging_active = !desc.empty();
    if (jogging_active) {
        sendTwist(vx, vy, vz, wx, wy, wz, desc);
    } else if (last_was_jogging_) {
        sendStopTwist();
    }

    last_was_jogging_ = jogging_active;
}

void JoystickController::sendTwist(
    double vx,
    double vy,
    double vz,
    double wx,
    double wy,
    double wz,
    const std::string & desc)
{
    auto twist = geometry_msgs::msg::TwistStamped();
    twist.header.stamp = this->now();

    int robot_num = 1;
    {
        std::lock_guard<std::mutex> lock(robot_lock_);
        robot_num = active_robot_;
        twist.header.frame_id = (active_robot_ == 1) ? "left_base_link" : "right_base_link";
    }

    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = vz;
    twist.twist.angular.x = wx;
    twist.twist.angular.y = wy;
    twist.twist.angular.z = wz;

    if (robot_num == 1) {
        twist_pub_left_->publish(twist);
    } else {
        twist_pub_right_->publish(twist);
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[Robot %d] Servo command: %s",
        robot_num,
        desc.c_str());
}

void JoystickController::sendStopTwist()
{
    sendTwist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "STOP");
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
