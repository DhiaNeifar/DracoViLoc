#include "fairino_hardware/command_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "libfairino/include/robot.h"
#include <string>

int main(int argc, char *argv[]){
    // Extract robot IP from first argument (before ROS args)
    std::string robot_ip = "";
    if (argc > 1) {
        // Check if first argument is not a ROS argument
        std::string first_arg(argv[1]);
        if (first_arg.find("--ros-args") == std::string::npos) {
            robot_ip = first_arg;
        }
    }
    
    //该main函数用于创建简化指令客户端的app
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor mulexecutor;
    //创建用户指令节点
    auto command_server_node = std::make_shared<robot_command_thread>("fr_command_server", robot_ip);
    mulexecutor.add_node(command_server_node);
    //创建非实时状态反馈获取节点
    auto robot_state_node = std::make_shared<robot_recv_thread>("fr_state_brodcast", robot_ip);
    mulexecutor.add_node(robot_state_node);//状态反馈节点加入执行器
    mulexecutor.spin();
    rclcpp::shutdown();
    return 0;
}
