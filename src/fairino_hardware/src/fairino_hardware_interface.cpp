#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware{

namespace {
constexpr const char * RECOVERY_SERVICE_NAME = "/fairino_hardware/recover";
constexpr int RECOVERY_RESET_ATTEMPTS = 3;
constexpr auto RECOVERY_SETTLE_TIME = std::chrono::seconds(1);

class RecoveryPauseGuard
{
public:
    explicit RecoveryPauseGuard(std::atomic_bool & recovery_in_progress)
    : recovery_in_progress_(recovery_in_progress)
    {
    }

    ~RecoveryPauseGuard()
    {
        recovery_in_progress_.store(false);
        RCLCPP_INFO(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "ServoJ streaming resumed after hardware recovery");
    }

private:
    std::atomic_bool & recovery_in_progress_;
};
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;

    // Check if this is single-arm or dual-arm configuration
    auto single_ip_it = info_.hardware_parameters.find("robot_ip");
    auto left_ip_it = info_.hardware_parameters.find("left_robot_ip");
    auto right_ip_it = info_.hardware_parameters.find("right_robot_ip");
    
    bool is_dual_arm = (left_ip_it != info_.hardware_parameters.end() && 
                        right_ip_it != info_.hardware_parameters.end());
    bool is_single_arm = (single_ip_it != info_.hardware_parameters.end());
    
    if (!is_dual_arm && !is_single_arm) {
        RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), 
                    "CRITICAL: No robot IP parameters found! Expected 'robot_ip' (single-arm) or 'left_robot_ip'+'right_robot_ip' (dual-arm)");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (is_dual_arm) {
        // Dual-arm mode
        _left_robot_ip = left_ip_it->second;
        _right_robot_ip = right_ip_it->second;
        
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "DUAL-ARM MODE");
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Left robot IP: %s", _left_robot_ip.c_str());
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Right robot IP: %s", _right_robot_ip.c_str());
        
        // Map joints to left/right robots based on joint name prefix
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const std::string& joint_name = info_.joints[i].name;
            if (joint_name.find("left_") == 0) {
                _left_joint_indices.push_back(i);
            } else if (joint_name.find("right_") == 0) {
                _right_joint_indices.push_back(i);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), 
                           "Joint '%s' does not have 'left_' or 'right_' prefix", joint_name.c_str());
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Left arm joints: %zu", _left_joint_indices.size());
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Right arm joints: %zu", _right_joint_indices.size());
    } else {
        // Single-arm mode - use left robot for compatibility
        _left_robot_ip = single_ip_it->second;
        _right_robot_ip = "";  // No right robot
        
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "SINGLE-ARM MODE");
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Robot IP: %s", _left_robot_ip.c_str());
        
        // All joints belong to the single (left) robot
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            _left_joint_indices.push_back(i);
        }
        
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Single arm joints: %zu", _left_joint_indices.size());
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        //指令部分
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        //关节状态部分
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"), "Joint '%s' has %zu state interface. 1 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init



std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_jnt_velocity_state.at(i)));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_state.at(i)));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));

//     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
//         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    using namespace std::chrono_literals;
    bool is_dual_arm = !_right_robot_ip.empty();
    
    if (is_dual_arm) {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting dual-arm system ...please wait...");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Starting single-arm system ...please wait...");
    }
    
    // Initialize all 12 joints (6 left + 6 right)
    for(int i=0;i<12;i++){
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
        _last_position_command[i] = 0;
    }
    _has_last_position_command = false;
    _control_mode = 0; // Position control mode
    
    // Create and connect to left robot (or single robot)
    _left_robot = std::make_unique<FRRobot>();
    _left_robot->SetReConnectParam(true, 30000, 500); // Enable reconnection with 30s timeout
    
    const char* robot_label = is_dual_arm ? "left robot" : "robot";
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Connecting to %s at %s...", robot_label, _left_robot_ip.c_str());
    
    errno_t returncode_left = _left_robot->RPC(_left_robot_ip.c_str());
    rclcpp::sleep_for(200ms);
    
    if(returncode_left != 0){
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to connect to %s at %s! Error code: %d", robot_label, _left_robot_ip.c_str(), returncode_left);
        return hardware_interface::CallbackReturn::ERROR;
    } else{
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Successfully connected to %s!", robot_label);
        
        // Note: Robot mode is not changed during hardware interface activation
        // The robot will remain in its current mode (manual/automatic)
        // This prevents unwanted mode switches during RViz launch
        
        errno_t enable_ret = _left_robot->RobotEnable(1);
        if(enable_ret != 0){
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "%s: Failed to enable robot, error: %d", robot_label, enable_ret);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "%s: Robot enabled", robot_label);
        }
    }
    
    // Only connect to right robot if in dual-arm mode
    if (is_dual_arm) {
        _right_robot = std::make_unique<FRRobot>();
        _right_robot->SetReConnectParam(true, 30000, 500); // Enable reconnection with 30s timeout
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Connecting to right robot at %s...", _right_robot_ip.c_str());
        
        errno_t returncode_right = _right_robot->RPC(_right_robot_ip.c_str());
        rclcpp::sleep_for(200ms);
        
        if(returncode_right != 0){
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to connect to right robot at %s! Error code: %d", _right_robot_ip.c_str(), returncode_right);
            return hardware_interface::CallbackReturn::ERROR;
        } else{
            RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Successfully connected to right robot!");
            
            // Note: Robot mode is not changed during hardware interface activation
            // The robot will remain in its current mode (manual/automatic)
            // This prevents unwanted mode switches during RViz launch
            
            errno_t enable_ret = _right_robot->RobotEnable(1);
            if(enable_ret != 0){
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Right robot: Failed to enable robot, error: %d", enable_ret);
            } else {
                RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Right robot: Robot enabled");
            }
        }
    }
    
    // Read initial joint positions from left robot
    JointPos left_jntpos;
    errno_t left_read = _left_robot->GetActualJointPosDegree(0, &left_jntpos);
    if(left_read != 0){
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read initial positions from %s!", robot_label);
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // Map initial positions from left robot to command arrays
    for(size_t i = 0; i < _left_joint_indices.size(); i++){
        size_t joint_idx = _left_joint_indices[i];
        _jnt_position_command[joint_idx] = left_jntpos.jPos[i] / 180.0 * M_PI;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Initial %s positions (rad): %f,%f,%f,%f,%f,%f",
        robot_label,
        left_jntpos.jPos[0]/180.0*M_PI, left_jntpos.jPos[1]/180.0*M_PI, left_jntpos.jPos[2]/180.0*M_PI,
        left_jntpos.jPos[3]/180.0*M_PI, left_jntpos.jPos[4]/180.0*M_PI, left_jntpos.jPos[5]/180.0*M_PI);
    
    // Read initial joint positions from right robot if in dual-arm mode
    if (is_dual_arm) {
        JointPos right_jntpos;
        errno_t right_read = _right_robot->GetActualJointPosDegree(0, &right_jntpos);
        if(right_read != 0){
            RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read initial positions from right robot!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        for(size_t i = 0; i < _right_joint_indices.size(); i++){
            size_t joint_idx = _right_joint_indices[i];
            _jnt_position_command[joint_idx] = right_jntpos.jPos[i] / 180.0 * M_PI;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Initial right arm positions (rad): %f,%f,%f,%f,%f,%f",
            right_jntpos.jPos[0]/180.0*M_PI, right_jntpos.jPos[1]/180.0*M_PI, right_jntpos.jPos[2]/180.0*M_PI,
            right_jntpos.jPos[3]/180.0*M_PI, right_jntpos.jPos[4]/180.0*M_PI, right_jntpos.jPos[5]/180.0*M_PI);
    }
    
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Hardware started successfully!");
    start_recovery_service();
    
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Stopping dual-arm system ...please wait...");
    stop_recovery_service();
    std::lock_guard<std::mutex> lock(_robot_sdk_mutex);
    
    if (_left_robot) {
        _left_robot->StopMotion();
        _left_robot->CloseRPC();
        _left_robot.release();
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Left robot stopped!");
    }
    
    if (_right_robot) {
        _right_robot->StopMotion();
        _right_robot->CloseRPC();
        _right_robot.release();
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Right robot stopped!");
    }
    
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "Dual-arm system successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if (_recovery_in_progress.load()) {
        return hardware_interface::return_type::OK;
    }

    std::lock_guard<std::mutex> lock(_robot_sdk_mutex);

    // Read from left robot (or single robot)
    JointPos left_state_data;
    error_t left_returncode = _left_robot->GetActualJointPosDegree(1, &left_state_data);
    if(left_returncode != 0){
        RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read from robot!");
        return hardware_interface::return_type::ERROR;
    }
    
    // Map left robot joint positions to state array
    for(size_t i = 0; i < _left_joint_indices.size(); i++){
        size_t joint_idx = _left_joint_indices[i];
        _jnt_position_state[joint_idx] = left_state_data.jPos[i] / 180.0 * M_PI;
    }
    
    // Read from right robot if in dual-arm mode
    if (_right_robot) {
        JointPos right_state_data;
        error_t right_returncode = _right_robot->GetActualJointPosDegree(1, &right_state_data);
        if(right_returncode != 0){
            RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Failed to read from right robot!");
            return hardware_interface::return_type::ERROR;
        }
        
        // Map right robot joint positions to state array
        for(size_t i = 0; i < _right_joint_indices.size(); i++){
            size_t joint_idx = _right_joint_indices[i];
            _jnt_position_state[joint_idx] = right_state_data.jPos[i] / 180.0 * M_PI;
        }
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& time,const rclcpp::Duration& period)
{
    if (_recovery_in_progress.load()) {
        return hardware_interface::return_type::OK;
    }

    std::lock_guard<std::mutex> lock(_robot_sdk_mutex);

    if(_control_mode == 0){ // Position control mode
        // Check for valid commands
        if (std::any_of(_jnt_position_command, _jnt_position_command + 12,
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }

        bool command_changed = !_has_last_position_command;
        for (size_t i = 0; i < 12 && !command_changed; ++i) {
            command_changed =
                std::abs(_jnt_position_command[i] - _last_position_command[i]) >
                _command_change_threshold;
        }

        // When controllers are idle they keep the same hold-position command.
        // Avoid streaming identical ServoJ commands so direct JOG command services can run.
        if (!command_changed) {
            return hardware_interface::return_type::OK;
        }
        
        // Prepare commands for left robot (or single robot)
        JointPos left_cmd;
        ExaxisPos left_extcmd{0,0,0,0};
        for(size_t i = 0; i < _left_joint_indices.size(); i++){
            size_t joint_idx = _left_joint_indices[i];
            left_cmd.jPos[i] = _jnt_position_command[joint_idx] / M_PI * 180.0; // Convert to degrees
        }
        
        // Send command to left robot
        // Using 0.040s (40ms) to match 25Hz controller update rate
        int left_returncode = _left_robot->ServoJ(&left_cmd, &left_extcmd, 0, 0, 0.040, 0, 0);
        if(left_returncode != 0){
            if(left_returncode == 14){
                RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Left robot ServoJ failed with error code: %d", left_returncode);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Left robot ServoJ failed with error code: %d", left_returncode);
            }
        }
        
        // Send command to right robot if in dual-arm mode
        if (_right_robot) {
            JointPos right_cmd;
            ExaxisPos right_extcmd{0,0,0,0};
            for(size_t i = 0; i < _right_joint_indices.size(); i++){
                size_t joint_idx = _right_joint_indices[i];
                right_cmd.jPos[i] = _jnt_position_command[joint_idx] / M_PI * 180.0; // Convert to degrees
            }
            
            // Send command to right robot
            // Using 0.040s (40ms) to match 25Hz controller update rate
            int right_returncode = _right_robot->ServoJ(&right_cmd, &right_extcmd, 0, 0, 0.040, 0, 0);
            if(right_returncode != 0){
                if(right_returncode == 14){
                    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Right robot ServoJ failed with error code: %d", right_returncode);
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"), "Right robot ServoJ failed with error code: %d", right_returncode);
                }
            }
        }

        for (size_t i = 0; i < 12; ++i) {
            _last_position_command[i] = _jnt_position_command[i];
        }
        _has_last_position_command = true;
        
    } else if(_control_mode == 1){ // Torque control mode (not implemented for dual-arm)
        if (std::any_of(_jnt_torque_command, _jnt_torque_command + 12,
            [](double c) { return not std::isfinite(c); })) {
            return hardware_interface::return_type::ERROR;
        }
        // TODO: Implement torque control for dual-arm
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Unknown control mode!");
        return hardware_interface::return_type::ERROR;
    }
 
    return hardware_interface::return_type::OK;
}

void FairinoHardwareInterface::start_recovery_service()
{
    if (_recovery_node) {
        return;
    }

    _recovery_node = std::make_shared<rclcpp::Node>("fairino_hardware_recovery");
    _recovery_service = _recovery_node->create_service<fairino_msgs::srv::RemoteCmdInterface>(
        RECOVERY_SERVICE_NAME,
        std::bind(
            &FairinoHardwareInterface::handle_recovery_request,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    _recovery_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _recovery_executor->add_node(_recovery_node);
    _recovery_thread = std::thread([this]() {
        _recovery_executor->spin();
    });

    RCLCPP_INFO(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Hardware recovery service available at %s",
        RECOVERY_SERVICE_NAME);
}

void FairinoHardwareInterface::stop_recovery_service()
{
    if (_recovery_executor) {
        _recovery_executor->cancel();
    }

    if (_recovery_thread.joinable()) {
        _recovery_thread.join();
    }

    if (_recovery_executor && _recovery_node) {
        _recovery_executor->remove_node(_recovery_node);
    }

    _recovery_service.reset();
    _recovery_node.reset();
    _recovery_executor.reset();
}

void FairinoHardwareInterface::handle_recovery_request(
    const std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Request> request,
    std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Response> response)
{
    const std::string cmd = request ? request->cmd_str : "";
    std::string detail;
    bool success = false;

    if (cmd.empty() || cmd == "recover" || cmd == "Recover" || cmd == "ResetAllError()") {
        success = recover_robot_state(detail);
    } else {
        detail = "Unsupported hardware recovery command: " + cmd;
    }

    response->cmd_res = success ? "0" : "-1:" + detail;
}

bool FairinoHardwareInterface::sync_joint_positions_from_hardware(
    bool sync_commands,
    std::string & detail)
{
    if (!_left_robot) {
        detail = "left robot is not connected";
        return false;
    }

    JointPos left_state_data;
    error_t left_returncode = _left_robot->GetActualJointPosDegree(1, &left_state_data);
    if (left_returncode != 0) {
        detail = "left GetActualJointPosDegree returned " + std::to_string(left_returncode);
        return false;
    }

    for (size_t i = 0; i < _left_joint_indices.size(); i++) {
        size_t joint_idx = _left_joint_indices[i];
        const double position_rad = left_state_data.jPos[i] / 180.0 * M_PI;
        _jnt_position_state[joint_idx] = position_rad;
        if (sync_commands) {
            _jnt_position_command[joint_idx] = position_rad;
        }
    }

    if (_right_robot) {
        JointPos right_state_data;
        error_t right_returncode = _right_robot->GetActualJointPosDegree(1, &right_state_data);
        if (right_returncode != 0) {
            detail = "right GetActualJointPosDegree returned " + std::to_string(right_returncode);
            return false;
        }

        for (size_t i = 0; i < _right_joint_indices.size(); i++) {
            size_t joint_idx = _right_joint_indices[i];
            const double position_rad = right_state_data.jPos[i] / 180.0 * M_PI;
            _jnt_position_state[joint_idx] = position_rad;
            if (sync_commands) {
                _jnt_position_command[joint_idx] = position_rad;
            }
        }
    }

    return true;
}

bool FairinoHardwareInterface::recover_robot_state(std::string & detail)
{
    if (!_left_robot) {
        detail = "left robot is not connected";
        return false;
    }

    bool expected = false;
    if (!_recovery_in_progress.compare_exchange_strong(expected, true)) {
        detail = "recovery already in progress";
        return false;
    }

    RecoveryPauseGuard recovery_pause(_recovery_in_progress);
    std::lock_guard<std::mutex> lock(_robot_sdk_mutex);
    bool success = true;
    std::string errors;

    auto append_error = [&errors](const std::string & message, int code) {
        if (!errors.empty()) {
            errors += "; ";
        }
        errors += message + " returned " + std::to_string(code);
    };

    auto call_both_required = [&](const std::string & label, auto fn) {
        int left_ret = fn(_left_robot.get());
        if (left_ret != 0) {
            append_error("left " + label, left_ret);
            success = false;
        }

        if (_right_robot) {
            int right_ret = fn(_right_robot.get());
            if (right_ret != 0) {
                append_error("right " + label, right_ret);
                success = false;
            }
        }
    };

    RCLCPP_WARN(
        rclcpp::get_logger("FairinoHardwareInterface"),
        "Starting direct hardware recovery; ServoJ writes are paused");

    if (_left_robot) {
        int left_stop = _left_robot->StopMotion();
        if (left_stop != 0) {
            RCLCPP_WARN(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "Left StopMotion returned %d during recovery", left_stop);
        }
    }
    if (_right_robot) {
        int right_stop = _right_robot->StopMotion();
        if (right_stop != 0) {
            RCLCPP_WARN(
                rclcpp::get_logger("FairinoHardwareInterface"),
                "Right StopMotion returned %d during recovery", right_stop);
        }
    }

    rclcpp::sleep_for(RECOVERY_SETTLE_TIME);

    call_both_required("RobotEnable(0)", [](FRRobot * robot) {
        return robot->RobotEnable(0);
    });

    rclcpp::sleep_for(RECOVERY_SETTLE_TIME);

    bool reset_success = false;
    for (int attempt = 1; attempt <= RECOVERY_RESET_ATTEMPTS; ++attempt) {
        int left_reset = _left_robot->ResetAllError();
        int right_reset = _right_robot ? _right_robot->ResetAllError() : 0;

        if (left_reset == 0 && right_reset == 0) {
            reset_success = true;
            break;
        }

        RCLCPP_WARN(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "ResetAllError attempt %d/%d returned left=%d right=%d",
            attempt,
            RECOVERY_RESET_ATTEMPTS,
            left_reset,
            right_reset);

        rclcpp::sleep_for(RECOVERY_SETTLE_TIME);
    }

    if (!reset_success) {
        success = false;
        append_error("ResetAllError", -1);
    }

    rclcpp::sleep_for(RECOVERY_SETTLE_TIME);

    call_both_required("RobotEnable(1)", [](FRRobot * robot) {
        return robot->RobotEnable(1);
    });

    rclcpp::sleep_for(RECOVERY_SETTLE_TIME);

    std::string sync_detail;
    if (!sync_joint_positions_from_hardware(true, sync_detail)) {
        success = false;
        if (!errors.empty()) {
            errors += "; ";
        }
        errors += sync_detail;
        RCLCPP_ERROR(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "Failed to sync joint positions before resuming ServoJ: %s",
            sync_detail.c_str());
    } else {
        RCLCPP_INFO(
            rclcpp::get_logger("FairinoHardwareInterface"),
            "Joint command buffers synced to actual robot positions before ServoJ resume");
    }

    if (success) {
        detail = "hardware recovery completed";
        RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "%s", detail.c_str());
        return true;
    }

    detail = errors.empty() ? "hardware recovery failed" : errors;
    RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "%s", detail.c_str());
    return false;
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
