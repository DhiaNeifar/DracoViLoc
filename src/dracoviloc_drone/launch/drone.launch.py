import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_gazebo = LaunchConfiguration("start_gazebo")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")
    world_name = LaunchConfiguration("world_name")
    drone_share = get_package_share_directory("dracoviloc_drone")
    xacro_file = os.path.join(
        drone_share, "urdf", "simple_drone.urdf.xacro")
    description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]),
            value_type=str)
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_gazebo", default_value="true",
            description="Start the standalone Gazebo instance and its sole /clock bridge."),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument(
            "world_name", default_value="empty",
            description="Gazebo world name used by the set-pose controller."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": ["-r empty.sdf"],
                "on_exit_shutdown": "true",
            }.items(),
            condition=IfCondition(PythonExpression([
                "'", start_gazebo, "'.lower() == 'true' and '",
                use_gui, "'.lower() == 'true'"
            ]))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch", "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": ["-r -s empty.sdf"],
                "on_exit_shutdown": "true",
            }.items(),
            condition=IfCondition(PythonExpression([
                "'", start_gazebo, "'.lower() == 'true' and '",
                use_gui, "'.lower() != 'true'"
            ]))),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            parameters=[{"use_sim_time": True}],
            output="screen",
            condition=IfCondition(start_gazebo)),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="drone",
            parameters=[description, {"use_sim_time": True}],
            output="screen"),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "drone", "-topic", "/drone/robot_description",
                "-x", "0.0", "-y", "0.0", "-z", "1.2"],
            output="screen"),
        Node(
            package="dracoviloc_drone",
            executable="drone_pose_controller",
            parameters=[{"use_sim_time": True, "world_name": world_name}],
            output="screen"),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(drone_share, "config", "drone.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(use_rviz),
            output="screen"),
    ])
