"""Monitoring for the control center: video previews, telemetry, and a ROS bag
of every diagnostics/state topic (no images) when log_dir is set."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Recorded at full rate for offline analysis. Camera images are excluded: the
# raw video is recorded by dracoviloc_recording.
BAG_TOPICS = [
    "/dracoviloc/telemetry", "/ekf/diagnostics", "/ekf/direction",
    "/mobilenetv2/diagnostics", "/mobilenetv2/direction", "/ast/direction", "/gre/direction",
    "/yolo/direction", "/detections_output", "/joint_states", "/sst",
    "/first_arrival/candidates", "/tracking/events", "/diagnostics/events",
    "/ee/speed", "/ee/acceleration", "/ee/jerk", "/ee/position", "/audio/target_valid",
    "/tf", "/tf_static",
]


def _configure(context):
    log_dir = LaunchConfiguration("log_dir").perform(context)
    actions = [
        Node(package="dracoviloc_monitor", executable="preview_node",
             name="dracoviloc_preview", output="screen",
             parameters=[{
                 "max_fps": ParameterValue(LaunchConfiguration("preview_fps"), value_type=float),
                 "max_width": ParameterValue(LaunchConfiguration("preview_width"), value_type=int),
             }]),
        Node(package="dracoviloc_monitor", executable="telemetry_node",
             name="dracoviloc_telemetry", output="screen",
             parameters=[{
                 "log_path": os.path.join(log_dir, "telemetry.jsonl") if log_dir else "",
             }]),
    ]
    if log_dir:
        os.makedirs(log_dir, exist_ok=True)
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "--output", os.path.join(log_dir, "bag"),
                 "--include-hidden-topics", *BAG_TOPICS],
            output="screen", sigterm_timeout="15", sigkill_timeout="20"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("log_dir", default_value="",
                              description="Run log folder: telemetry.jsonl and bag/ are written here."),
        DeclareLaunchArgument("preview_fps", default_value="15.0"),
        DeclareLaunchArgument("preview_width", default_value="960"),
        OpaqueFunction(function=_configure),
    ])
