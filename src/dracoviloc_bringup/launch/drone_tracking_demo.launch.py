import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")
    tracking_enabled = LaunchConfiguration("tracking_enabled")
    bringup_share = get_package_share_directory("dracoviloc_bringup")
    drone_share = get_package_share_directory("dracoviloc_drone")
    tracking_share = get_package_share_directory("dracoviloc_tracking")

    arm_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "demo.launch.py")),
        launch_arguments={
            "sim": "true",
            "use_gui": use_gui,
            "use_rviz": use_rviz,
            "use_moveit": "true",
        }.items())

    drone = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drone_share, "launch", "drone.launch.py")),
            launch_arguments={
                "start_gazebo": "false",
                "use_rviz": "false",
                "world_name": "empty_no_ground",
            }.items())])

    moveit_config = (
        MoveItConfigsBuilder("dracoviloc", package_name="dracoviloc_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("dracoviloc_description"),
                "urdf", "dracoviloc.urdf.xacro"),
            mappings={
                "sim": "true",
                "ros2_controllers_file": os.path.join(
                    bringup_share, "config", "ros2_controllers.yaml"),
            })
        .robot_description_semantic(file_path="config/dracoviloc.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs())

    tracker = Node(
        package="dracoviloc_tracking",
        executable="arm_drone_tracker",
        parameters=[
            moveit_config.to_dict(),
            os.path.join(tracking_share, "config", "tracking.yaml"),
            {"tracking_enabled": tracking_enabled, "use_sim_time": True},
        ],
        output="screen",
        condition=IfCondition(tracking_enabled))

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument("tracking_enabled", default_value="true"),
        LogInfo(msg="Drone keyboard: ros2 run dracoviloc_drone drone_keyboard_teleop"),
        arm_demo,
        drone,
        tracker,
    ])
