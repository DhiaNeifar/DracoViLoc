import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    dual_arm = LaunchConfiguration("dual_arm")
    use_rviz = LaunchConfiguration("use_rviz")
    use_moveit = LaunchConfiguration("use_moveit")
    robot_ip = LaunchConfiguration("robot_ip")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    second_arm_x = LaunchConfiguration("second_arm_x")
    second_arm_y = LaunchConfiguration("second_arm_y")
    second_arm_z = LaunchConfiguration("second_arm_z")
    hardware_mode = LaunchConfiguration("hardware_mode")

    description_share = get_package_share_directory("dracoviloc_description")
    bringup_share = get_package_share_directory("dracoviloc_bringup")
    moveit_share = get_package_share_directory("dracoviloc_moveit_config")

    xacro_file = os.path.join(description_share, "urdf", "dracoviloc.urdf.xacro")
    controllers_file = os.path.join(bringup_share, "config", "ros2_controllers.yaml")

    mappings = {
        "dual_arm": dual_arm,
        "hardware_mode": hardware_mode,
        "robot_ip": robot_ip,
        "left_robot_ip": left_robot_ip,
        "right_robot_ip": right_robot_ip,
        "second_arm_x": second_arm_x,
        "second_arm_y": second_arm_y,
        "second_arm_z": second_arm_z,
        "ros2_controllers_file": controllers_file,
    }

    robot_description = {"robot_description": ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file,
                 " dual_arm:=", dual_arm,
                 " hardware_mode:=", hardware_mode,
                 " robot_ip:=", robot_ip,
                 " left_robot_ip:=", left_robot_ip,
                 " right_robot_ip:=", right_robot_ip,
                 " second_arm_x:=", second_arm_x,
                 " second_arm_y:=", second_arm_y,
                 " second_arm_z:=", second_arm_z,
                 " ros2_controllers_file:=", controllers_file]), value_type=str)}

    moveit_config = (MoveItConfigsBuilder("dracoviloc", package_name="dracoviloc_moveit_config")
        .robot_description(file_path=xacro_file, mappings=mappings)
        .robot_description_semantic(file_path="config/dracoviloc.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"]).to_moveit_configs())

    common = {"use_sim_time": False}

    return LaunchDescription([
        DeclareLaunchArgument("dual_arm", default_value="false",
                              description="Configure and launch dual cobot arms."),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("hardware_mode", default_value="mock",
                              choices=["mock", "real"],
                              description="ROS 2 Control backend."),
        DeclareLaunchArgument("robot_ip", default_value="192.168.58.2",
                              description="IP for single-arm mode."),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.58.2",
                              description="IP for Arm 1 (left) in dual-arm mode."),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.58.3",
                              description="IP for Arm 2 (right) in dual-arm mode."),
        DeclareLaunchArgument("second_arm_x", default_value="-0.75",
                              description="Arm 2 separation distance along X axis in metres (-0.75 for 75cm to the left of Arm 1)."),
        DeclareLaunchArgument("second_arm_y", default_value="0.0",
                              description="Arm 2 separation distance along Y axis in metres."),
        DeclareLaunchArgument("second_arm_z", default_value="0.0",
                              description="Arm 2 separation distance along Z axis in metres."),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[robot_description, common], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[robot_description, controllers_file, common], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "120"], parameters=[common], output="screen"),

        # Single arm controllers
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_controller", "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "120"],
             parameters=[common], output="screen",
             condition=UnlessCondition(dual_arm)),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_tracking_controller", "--inactive",
                        "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "120"],
             parameters=[common], output="screen",
             condition=UnlessCondition(dual_arm)),

        # Dual arm controllers
        Node(package="controller_manager", executable="spawner",
             arguments=["dual_arm_controller", "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "120"],
             parameters=[common], output="screen",
             condition=IfCondition(dual_arm)),
        Node(package="controller_manager", executable="spawner",
             arguments=["dual_arm_tracking_controller", "--inactive",
                        "--controller-manager", "/controller_manager",
                        "--controller-manager-timeout", "120"],
             parameters=[common], output="screen",
             condition=IfCondition(dual_arm)),

        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[moveit_config.to_dict(), common], condition=IfCondition(use_moveit)),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
             parameters=[moveit_config.to_dict(), common], output="screen",
             condition=IfCondition(use_rviz)),
    ])
