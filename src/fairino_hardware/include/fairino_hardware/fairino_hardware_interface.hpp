#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "fairino_msgs/srv/remote_cmd_interface.hpp"
#include "visibility_control.h"
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include "libfairino/include/robot.h"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
	  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
	  
	private:
	  void start_recovery_service();
	  void stop_recovery_service();
		  void handle_recovery_request(
		    const std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Request> request,
		    std::shared_ptr<fairino_msgs::srv::RemoteCmdInterface::Response> response);
		  bool recover_robot_state(std::string & detail);
		  bool sync_joint_positions_from_hardware(bool sync_commands, std::string & detail);

		  // Arrays for 12 joints (6 left + 6 right)
	  double _jnt_position_command[12];
  double _jnt_velocity_command[12];
  double _jnt_torque_command[12];
  double _jnt_position_state[12];
  double _jnt_velocity_state[12];
  double _jnt_torque_state[12];
  double _last_position_command[12];
  bool _has_last_position_command{false};
  double _command_change_threshold{1e-6};
  int _control_mode;
  
  // Dual robot support
  std::string _left_robot_ip;
  std::string _right_robot_ip;
  std::unique_ptr<FRRobot> _left_robot;
  std::unique_ptr<FRRobot> _right_robot;
  
	  // Joint index mapping (which joints belong to which robot)
	  std::vector<size_t> _left_joint_indices;
	  std::vector<size_t> _right_joint_indices;

	  std::mutex _robot_sdk_mutex;
	  std::atomic_bool _recovery_in_progress{false};
	  rclcpp::Node::SharedPtr _recovery_node;
	  rclcpp::Service<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr _recovery_service;
	  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _recovery_executor;
	  std::thread _recovery_thread;
};

} //end namespace


#endif
