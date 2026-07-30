#ifndef FAIRINO_HARDWARE__DUAL_ARMS_JOYSTICK_CONTROLLER_HPP_
#define FAIRINO_HARDWARE__DUAL_ARMS_JOYSTICK_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <fairino_msgs/srv/remote_cmd_interface.hpp>
#include <memory>

class DualArmsJoystickController : public rclcpp::Node
{
public:
    DualArmsJoystickController();
    ~DualArmsJoystickController();

private:
    enum class ControlMode
    {
        BOTH,
        LEFT_ONLY,
        RIGHT_ONLY
    };

    // Callbacks
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void controlTimerCallback();
    
    // Robot control functions
    void initializeRobots();
    void sendStartJog(
        rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr client,
        int axis_num, int direction, double vel_percent);
    void sendJogForMode(
        int axis_num, int left_direction, int right_direction, double vel_percent);
    void stopBothRobots();
    void setControlMode(ControlMode mode);
    void sendStopJog(
        rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr client);
    
    // ROS2 communication
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr left_client_;
    rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr right_client_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Parameters
    double max_vel_percent_;
    double threshold_;
    bool use_analog_velocity_;
    
    // State tracking
    sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;
    bool last_was_jogging_;
    bool last_reset_button_state_;
    bool last_mode_button_state_;
    ControlMode control_mode_;
};

#endif  // FAIRINO_HARDWARE__DUAL_ARMS_JOYSTICK_CONTROLLER_HPP_
