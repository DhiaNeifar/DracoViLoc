import json
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Digital-twin harness for the arm_tracking control loop: mock ros2_control
# arm + dracoviloc_ekf + arm_tracker + a synthetic moving bearing
# (dracoviloc_target_sim) standing in for YOLO/audio detectors. No camera,
# microphone, Gazebo, or recording pipeline is involved -- this exists to
# iterate on arm_tracking itself. Every run's logs and the parameters that
# produced them land in one timestamped, self-contained folder under
# artifacts_root so experiments can be compared later.


def _build(context, bringup_share, ekf_share):
    artifacts_root = LaunchConfiguration("artifacts_root").perform(context)
    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    experiment_dir = os.path.join(artifacts_root, timestamp)
    os.makedirs(experiment_dir, exist_ok=True)

    tunable_args = (
        "path_type", "center_azimuth_deg", "center_elevation_deg",
        "azimuth_amplitude_deg", "elevation_amplitude_deg", "period_s",
        "step_interval_s", "random_walk_step_deg", "noise_stddev_deg",
        "publish_rate_hz", "seed", "smoothing_alpha", "angular_deadband",
        "max_velocity", "max_acceleration", "max_jerk", "command_rate_hz",
        "ekf_process_noise", "ekf_measurement_noise", "ekf_average_window",
    )
    params = {name: LaunchConfiguration(name).perform(context) for name in tunable_args}
    params["timestamp"] = timestamp
    with open(os.path.join(experiment_dir, "params.json"), "w") as handle:
        json.dump(params, handle, indent=2, sort_keys=True)

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "demo.launch.py")),
        launch_arguments={
            "hardware_mode": "mock",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_moveit": "true",
        }.items())

    table_mic_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_microphone_broadcaster",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.75",
            "--yaw", "3.1415926535897", "--pitch", "0.0", "--roll", "1.57079632679",
            "--frame-id", "world", "--child-frame-id", "table_mic_link",
        ],
        parameters=[{"use_sim_time": False}])

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ekf_share, "launch", "ekf.launch.py")),
        launch_arguments={
            "yolo_enabled": "true",
            "process_noise": LaunchConfiguration("ekf_process_noise"),
            "measurement_noise": LaunchConfiguration("ekf_measurement_noise"),
            "output_average_window": LaunchConfiguration("ekf_average_window"),
        }.items())

    tracker = Node(
        package="dracoviloc_tracking",
        executable="arm_tracker",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "ekf_enabled": True,
            "require_home_before_tracking": True,
            "smoothing_alpha": ParameterValue(
                LaunchConfiguration("smoothing_alpha"), value_type=float),
            "angular_deadband": ParameterValue(
                LaunchConfiguration("angular_deadband"), value_type=float),
            "max_velocity": ParameterValue(
                LaunchConfiguration("max_velocity"), value_type=float),
            "max_acceleration": ParameterValue(
                LaunchConfiguration("max_acceleration"), value_type=float),
            "max_jerk": ParameterValue(LaunchConfiguration("max_jerk"), value_type=float),
            "command_rate_hz": ParameterValue(
                LaunchConfiguration("command_rate_hz"), value_type=float),
            "servo_log_path": os.path.join(experiment_dir, "servo_log.csv"),
            "ekf_direction_log_path": os.path.join(experiment_dir, "ekf_log.csv"),
            "yolo_direction_log_path": os.path.join(experiment_dir, "yolo_log.csv"),
        }])

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("dracoviloc_diagnostics"),
            "launch", "diagnostics.launch.py")),
        launch_arguments={
            "ee_frame": LaunchConfiguration("diagnostics_ee_frame"),
            "smoothing_alpha": LaunchConfiguration("diagnostics_smoothing_alpha"),
            "events_log_path": os.path.join(experiment_dir, "events.csv"),
            "ee_kinematics_log_path": os.path.join(experiment_dir, "ee_kinematics.csv"),
            "live_plot": LaunchConfiguration("diagnostics_live_plot"),
        }.items())

    target_sim = Node(
        package="dracoviloc_target_sim",
        executable="target_sim_node",
        output="screen",
        parameters=[{
            "tracking_frame": "table_mic_link",
            "path_type": LaunchConfiguration("path_type"),
            "center_azimuth_deg": ParameterValue(
                LaunchConfiguration("center_azimuth_deg"), value_type=float),
            "center_elevation_deg": ParameterValue(
                LaunchConfiguration("center_elevation_deg"), value_type=float),
            "azimuth_amplitude_deg": ParameterValue(
                LaunchConfiguration("azimuth_amplitude_deg"), value_type=float),
            "elevation_amplitude_deg": ParameterValue(
                LaunchConfiguration("elevation_amplitude_deg"), value_type=float),
            "period_s": ParameterValue(LaunchConfiguration("period_s"), value_type=float),
            "step_interval_s": ParameterValue(
                LaunchConfiguration("step_interval_s"), value_type=float),
            "random_walk_step_deg": ParameterValue(
                LaunchConfiguration("random_walk_step_deg"), value_type=float),
            "noise_stddev_deg": ParameterValue(
                LaunchConfiguration("noise_stddev_deg"), value_type=float),
            "publish_rate_hz": ParameterValue(
                LaunchConfiguration("publish_rate_hz"), value_type=float),
            "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
            "target_log_path": os.path.join(experiment_dir, "target_log.csv"),
        }])

    return [demo, table_mic_tf, ekf, tracker, target_sim, diagnostics]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "artifacts_root", default_value="artifacts",
            description="Root directory for timestamped experiment folders. "
                        "A relative path resolves against the directory "
                        "ros2 launch was run from."),
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
        DeclareLaunchArgument("smoothing_alpha", default_value="0.50"),
        DeclareLaunchArgument("angular_deadband", default_value="0.003"),
        DeclareLaunchArgument("max_velocity", default_value="2.50"),
        DeclareLaunchArgument("max_acceleration", default_value="12.0"),
        DeclareLaunchArgument("max_jerk", default_value="80.0"),
        DeclareLaunchArgument("command_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("ekf_process_noise", default_value="0.05"),
        DeclareLaunchArgument("ekf_measurement_noise", default_value="0.02"),
        DeclareLaunchArgument("ekf_average_window", default_value="5"),
        DeclareLaunchArgument("diagnostics_ee_frame", default_value="wrist3_link"),
        DeclareLaunchArgument("diagnostics_smoothing_alpha", default_value="0.3"),
        DeclareLaunchArgument("diagnostics_live_plot", default_value="false"),
        OpaqueFunction(function=lambda context: _build(
            context,
            get_package_share_directory("dracoviloc_bringup"),
            get_package_share_directory("dracoviloc_ekf"))),
    ])
