#include "fairino_hardware/dual_arms_joystick_controller.hpp"
#include <chrono>
#include <sstream>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

DualArmsJoystickController::DualArmsJoystickController()
    : Node("dual_arms_joystick_controller"),
      max_vel_percent_(30.0),
      threshold_(0.1),
      use_analog_velocity_(false),
      last_was_jogging_(false),
      last_reset_button_state_(false),
      last_mode_button_state_(false),
      control_mode_(ControlMode::BOTH)
{
    // Declare parameters
    this->declare_parameter<double>("max_vel_percent", 30.0);
    this->declare_parameter<double>("threshold", 0.1);
    this->declare_parameter<double>("control_rate_hz", 50.0);
    this->declare_parameter<bool>("use_analog_velocity", false);
    
    // Get parameters
    max_vel_percent_ = this->get_parameter("max_vel_percent").as_double();
    threshold_ = this->get_parameter("threshold").as_double();
    double control_rate_hz = this->get_parameter("control_rate_hz").as_double();
    use_analog_velocity_ = this->get_parameter("use_analog_velocity").as_bool();
    
    // Initialize last joystick message
    last_joy_msg_ = nullptr;
    
    // Create subscribers
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&DualArmsJoystickController::joyCallback, this, _1));
    
    // Create robot command clients for both robots
    left_client_ = this->create_client<fairino_msgs::srv::RemoteCmdInterface>(
        "/fairino_robot1_command_service");
    
    right_client_ = this->create_client<fairino_msgs::srv::RemoteCmdInterface>(
        "/fairino_robot2_command_service");
    
    // Create control timer
    auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
        std::bind(&DualArmsJoystickController::controlTimerCallback, this));
    
    // Wait for services
    RCLCPP_INFO(this->get_logger(), "Waiting for robot command services...");
    while ((!left_client_->wait_for_service(std::chrono::seconds(1)) || 
            !right_client_->wait_for_service(std::chrono::seconds(1))) && rclcpp::ok()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for services.");
            return;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Dual-Arm Mirrored Joystick Controller");
    RCLCPP_INFO(this->get_logger(), "  BASE/WORLD COORDINATE SYSTEM (ref=0)");
    RCLCPP_INFO(this->get_logger(), "  MIRRORED SYMMETRIC MOTION");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "Left robot:  192.168.58.2");
    RCLCPP_INFO(this->get_logger(), "Right robot: 192.168.58.3");
    RCLCPP_INFO(this->get_logger(), "Max velocity: %.1f%%", max_vel_percent_);
    RCLCPP_INFO(this->get_logger(), "Analog velocity scaling: %s", use_analog_velocity_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "Control rate: %.1f Hz", control_rate_hz);
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "✓ Both robots ready for mirrored control");
    RCLCPP_INFO(this->get_logger(), "Control mode: BOTH");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // Initialize both robots
    initializeRobots();
}

void DualArmsJoystickController::initializeRobots()
{
    // Reset errors and setup both robots
    auto reset_req = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
    reset_req->cmd_str = "ResetAllError()";
    left_client_->async_send_request(reset_req);
    right_client_->async_send_request(reset_req);
    
    std::this_thread::sleep_for(100ms);
    
    // Enable robots
    auto enable_req = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
    enable_req->cmd_str = "RobotEnable(1)";
    left_client_->async_send_request(enable_req);
    right_client_->async_send_request(enable_req);
    
    std::this_thread::sleep_for(100ms);
    

}

void DualArmsJoystickController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    last_joy_msg_ = msg;
}

void DualArmsJoystickController::controlTimerCallback()
{
    if (!last_joy_msg_) {
        return;  // No joystick input yet
    }
    
    auto joy = last_joy_msg_;
    if (joy->axes.size() < 5 || joy->buttons.size() < 8) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Ignoring /joy message with too few axes/buttons: axes=%zu buttons=%zu",
            joy->axes.size(),
            joy->buttons.size());
        if (last_was_jogging_) {
            sendStopJog(left_client_);
            sendStopJog(right_client_);
            last_was_jogging_ = false;
        }
        return;
    }
    
    // Check for reset button (Triangle = button 3)
    const bool reset_pressed = joy->buttons[3] == 1;
    if (reset_pressed && !last_reset_button_state_) {
        auto reset_req = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
        reset_req->cmd_str = "ResetAllError()";
        left_client_->async_send_request(reset_req);
        right_client_->async_send_request(reset_req);
        RCLCPP_INFO(this->get_logger(), "🔧 Resetting errors on both robots");
    }
    last_reset_button_state_ = reset_pressed;

    const bool mode_button_pressed = joy->buttons[1] == 1;
    if (mode_button_pressed && !last_mode_button_state_) {
        if (control_mode_ == ControlMode::BOTH) {
            setControlMode(ControlMode::LEFT_ONLY);
        } else if (control_mode_ == ControlMode::LEFT_ONLY) {
            setControlMode(ControlMode::RIGHT_ONLY);
        } else {
            setControlMode(ControlMode::BOTH);
        }
    }
    last_mode_button_state_ = mode_button_pressed;
    
    bool is_jogging = false;
    
    // Process axes and buttons to determine motion
    // Axes: [left_x, left_y, left_trigger, right_x, right_y, right_trigger, dpad_x, dpad_y]
    // Left stick X (axis 0) -> X-axis translation (MIRRORED)
    if (std::abs(joy->axes[0]) > threshold_) {
        const double vel_percent = use_analog_velocity_ ?
            std::abs(joy->axes[0]) * max_vel_percent_ : max_vel_percent_;
        // Mirrored: left arm +X when right arm -X for symmetric motion
        sendJogForMode(1, joy->axes[0] < 0 ? 0 : 1, joy->axes[0] < 0 ? 1 : 0, vel_percent);
        is_jogging = true;
    }
    
    // Left stick Y (axis 1) -> Y-axis translation (MIRRORED)
    else if (std::abs(joy->axes[1]) > threshold_) {
        const double vel_percent = use_analog_velocity_ ?
            std::abs(joy->axes[1]) * max_vel_percent_ : max_vel_percent_;
        // Mirrored: both arms move toward/away from center together
        sendJogForMode(2, joy->axes[1] < 0 ? 0 : 1, joy->axes[1] < 0 ? 1 : 0, vel_percent);
        is_jogging = true;
    }
    
    // Right stick Y (axis 4) -> Z-axis translation (MIRRORED)
    else if (std::abs(joy->axes[4]) > threshold_) {
        const double vel_percent = use_analog_velocity_ ?
            std::abs(joy->axes[4]) * max_vel_percent_ : max_vel_percent_;
        // Mirrored: both arms move toward/away from center together
        sendJogForMode(3, joy->axes[4] < 0 ? 0 : 1, joy->axes[4] < 0 ? 1 : 0, vel_percent);
        is_jogging = true;
    }
    
    // Rotation buttons (Cartesian)
    // R1/R2 (buttons 5/7) -> A rotation/Roll (MIRRORED)
    else if (joy->buttons[5] == 1) {  // R1 - positive roll
        sendJogForMode(4, 1, 0, max_vel_percent_);
        is_jogging = true;
    }
    else if (joy->buttons[7] == 1) {  // R2 - negative roll
        sendJogForMode(4, 0, 1, max_vel_percent_);
        is_jogging = true;
    }
    
    // L1/L2 (buttons 4/6) -> B rotation/Pitch (MIRRORED)
    else if (joy->buttons[4] == 1) {  // L1 - positive pitch
        sendJogForMode(5, 1, 0, max_vel_percent_);
        is_jogging = true;
    }
    else if (joy->buttons[6] == 1) {  // L2 - negative pitch
        sendJogForMode(5, 0, 1, max_vel_percent_);
        is_jogging = true;
    }
    
    // Circle/Square (buttons 2/0) -> C rotation/Yaw (SAME)
    else if (joy->buttons[2] == 1) {  // Circle - positive yaw
        sendJogForMode(6, 1, 1, max_vel_percent_);
        is_jogging = true;
    }
    else if (joy->buttons[0] == 1) {  // Square - negative yaw
        sendJogForMode(6, 0, 0, max_vel_percent_);
        is_jogging = true;
    }
    
    // Handle stop
    if (!is_jogging && last_was_jogging_) {
        stopBothRobots();
    }
    
    last_was_jogging_ = is_jogging;
}

void DualArmsJoystickController::sendStartJog(
    rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr client,
    int axis_num, int direction, double vel_percent)
{
    // Format: StartJOG(ref, axis, dir, vel, acc, max_dist)
    // ref=0: Base/World coordinate system (matches joystick_controller.cpp)
    // acc=30.0, max_dist=1000 (matches joystick_controller.cpp)
    std::stringstream ss;
    ss << "StartJOG(0," << axis_num << "," 
       << direction << "," << vel_percent << ",30.0,1000)";
    
    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
    request->cmd_str = ss.str();
    
    // Determine which robot this is
    std::string robot_name = (client == left_client_) ? "LEFT" : "RIGHT";
    
    // Log the command being sent
    RCLCPP_INFO(this->get_logger(), "[%s] Sending: %s", 
                robot_name.c_str(), ss.str().c_str());
    
    client->async_send_request(request);
}

void DualArmsJoystickController::sendJogForMode(
    int axis_num, int left_direction, int right_direction, double vel_percent)
{
    if (control_mode_ == ControlMode::BOTH || control_mode_ == ControlMode::LEFT_ONLY) {
        sendStartJog(left_client_, axis_num, left_direction, vel_percent);
    }
    if (control_mode_ == ControlMode::BOTH || control_mode_ == ControlMode::RIGHT_ONLY) {
        sendStartJog(right_client_, axis_num, right_direction, vel_percent);
    }
}

void DualArmsJoystickController::sendStopJog(
    rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr client)
{
    auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
    request->cmd_str = "StopJOG(1)";  // Matches joystick_controller.cpp
    
    // Determine which robot this is
    std::string robot_name = (client == left_client_) ? "LEFT" : "RIGHT";
    
    // Log the stop command
    RCLCPP_INFO(this->get_logger(), "[%s] Sending: StopJOG(1)", robot_name.c_str());
    
    client->async_send_request(request);
}

void DualArmsJoystickController::stopBothRobots()
{
    sendStopJog(left_client_);
    sendStopJog(right_client_);
}

void DualArmsJoystickController::setControlMode(ControlMode mode)
{
    if (control_mode_ == mode) {
        return;
    }

    stopBothRobots();
    control_mode_ = mode;
    last_was_jogging_ = false;

    const char * mode_name = "BOTH";
    if (control_mode_ == ControlMode::LEFT_ONLY) {
        mode_name = "LEFT_ONLY";
    } else if (control_mode_ == ControlMode::RIGHT_ONLY) {
        mode_name = "RIGHT_ONLY";
    }

    RCLCPP_INFO(this->get_logger(), "Control mode switched to %s", mode_name);
}

DualArmsJoystickController::~DualArmsJoystickController()
{
    // Stop both robots on shutdown
    stopBothRobots();
    RCLCPP_INFO(this->get_logger(), "Dual-arm controller shutting down...");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmsJoystickController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
