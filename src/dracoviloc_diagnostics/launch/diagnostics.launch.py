from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "ee_frame", default_value="wrist3_link",
            description="TF frame treated as the end effector for position/"
                        "velocity/acceleration/jerk (e.g. wrist3_link, "
                        "micarray_camera_link, or odas_link)."),
        DeclareLaunchArgument(
            "smoothing_alpha", default_value="0.3",
            description="Exponential smoothing applied to the "
                        "velocity/acceleration/jerk derived from TF position; "
                        "1.0 disables it (raw finite differences)."),
        DeclareLaunchArgument("events_log_path", default_value=""),
        DeclareLaunchArgument("ee_kinematics_log_path", default_value=""),
        DeclareLaunchArgument(
            "live_plot", default_value="false",
            description="Also open the live speed/acceleration/events plot "
                        "(requires a display)."),
        Node(
            package="dracoviloc_diagnostics",
            executable="event_logger_node",
            output="screen",
            parameters=[{
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_frame": LaunchConfiguration("ee_frame"),
                "smoothing_alpha": ParameterValue(
                    LaunchConfiguration("smoothing_alpha"), value_type=float),
                "events_log_path": LaunchConfiguration("events_log_path"),
                "ee_kinematics_log_path": LaunchConfiguration("ee_kinematics_log_path"),
            }]),
        Node(
            package="dracoviloc_diagnostics",
            executable="live_plot.py",
            output="screen",
            condition=IfCondition(LaunchConfiguration("live_plot"))),
    ])
