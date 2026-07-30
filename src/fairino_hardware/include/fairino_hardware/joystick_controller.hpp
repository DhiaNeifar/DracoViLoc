#ifndef JOYSTICK_CONTROLLER_HPP_
#define JOYSTICK_CONTROLLER_HPP_

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <memory>
#include <mutex>
#include <string>

class JoystickController : public rclcpp::Node
{
public:
    JoystickController();
    ~JoystickController();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void controlTimerCallback();
    void sendTwist(
        double vx,
        double vy,
        double vz,
        double wx,
        double wy,
        double wz,
        const std::string & desc);
    void sendStopTwist();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_left_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_right_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;
    std::mutex joy_lock_;

    int active_robot_;
    std::mutex robot_lock_;

    double max_command_scale_;
    double threshold_;
    bool last_was_jogging_;
    bool last_button1_state_;
};

#endif  // JOYSTICK_CONTROLLER_HPP_
