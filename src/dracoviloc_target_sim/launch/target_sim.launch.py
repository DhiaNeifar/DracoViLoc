from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("tracking_frame", default_value="table_mic_link"),
        DeclareLaunchArgument(
            "path_type", default_value="sweep",
            choices=["sweep", "circle", "step", "random_walk"]),
        DeclareLaunchArgument("center_azimuth_deg", default_value="0.0"),
        DeclareLaunchArgument("center_elevation_deg", default_value="10.0"),
        DeclareLaunchArgument("azimuth_amplitude_deg", default_value="50.0"),
        DeclareLaunchArgument("elevation_amplitude_deg", default_value="0.0"),
        DeclareLaunchArgument("period_s", default_value="6.0"),
        DeclareLaunchArgument("step_interval_s", default_value="3.0"),
        DeclareLaunchArgument("random_walk_step_deg", default_value="5.0"),
        DeclareLaunchArgument("noise_stddev_deg", default_value="0.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("target_log_path", default_value=""),
        Node(
            package="dracoviloc_target_sim",
            executable="target_sim_node",
            output="screen",
            parameters=[{
                "tracking_frame": LaunchConfiguration("tracking_frame"),
                "path_type": LaunchConfiguration("path_type"),
                "center_azimuth_deg": ParameterValue(
                    LaunchConfiguration("center_azimuth_deg"), value_type=float),
                "center_elevation_deg": ParameterValue(
                    LaunchConfiguration("center_elevation_deg"), value_type=float),
                "azimuth_amplitude_deg": ParameterValue(
                    LaunchConfiguration("azimuth_amplitude_deg"), value_type=float),
                "elevation_amplitude_deg": ParameterValue(
                    LaunchConfiguration("elevation_amplitude_deg"), value_type=float),
                "period_s": ParameterValue(
                    LaunchConfiguration("period_s"), value_type=float),
                "step_interval_s": ParameterValue(
                    LaunchConfiguration("step_interval_s"), value_type=float),
                "random_walk_step_deg": ParameterValue(
                    LaunchConfiguration("random_walk_step_deg"), value_type=float),
                "noise_stddev_deg": ParameterValue(
                    LaunchConfiguration("noise_stddev_deg"), value_type=float),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float),
                "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                "target_log_path": LaunchConfiguration("target_log_path"),
            }]),
    ])
