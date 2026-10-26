from __future__ import annotations

import os
import shlex
from pathlib import Path

from .host_platform import resolve_yolo_image
from .jobs import container_user_args
from .models import RunProfile


BOOL = lambda value: "true" if value else "false"


def runtime_environment(base: dict[str, str] | None = None) -> dict[str, str]:
    env = dict(base or os.environ)
    env.update({
        "ROS_DOMAIN_ID": "42",
        "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4",
        "CUDA_MODULE_LOADING": "LAZY",
        "PYTHONUNBUFFERED": "1",
    })
    return env


def _arg(name: str, value: object) -> str:
    if isinstance(value, bool):
        value = BOOL(value)
    return f"{name}:={value}"


def host_launch_arguments(profile: RunProfile) -> list[str]:
    values: dict[str, object] = {
        "hardware_mode": profile.hardware_mode,
        "robot_ip": profile.robot_ip,
        "dual_arm": profile.dual_arm,
        "left_robot_ip": profile.left_robot_ip,
        "right_robot_ip": profile.right_robot_ip,
        "second_arm_x": profile.second_arm_x,
        "second_arm_y": profile.second_arm_y,
        "second_arm_z": profile.second_arm_z,
        "dual_tracking_policy": profile.dual_tracking_policy,
        "lead_lock_tolerance_rad": profile.lead_lock_tolerance_rad,
        "audio_enabled": profile.audio_enabled,
        "yolo_enabled": profile.yolo_enabled,
        "ast_enabled": profile.ast_enabled,
        "gre_enabled": profile.gre_enabled,
        "mobilenetv2_enabled": profile.mobilenetv2_enabled,
        "mobilenetv2_ekf_enabled": profile.mobilenetv2_ekf_enabled,
        "mobilenetv2_engine_path": Path(profile.mobilenetv2_engine_path).expanduser(),
        "mobilenetv2_venv_python": Path(profile.mobilenetv2_venv_python).expanduser(),
        "mobilenetv2_threshold": profile.mobilenetv2_threshold,
        "mobilenetv2_votes_required": profile.mobilenetv2_votes_required,
        "mobilenetv2_vote_window": profile.mobilenetv2_vote_window,
        "fusion_enabled": profile.fusion_enabled,
        "tracking_mode": profile.tracking_mode,
        "use_rviz": profile.use_rviz,
        "use_moveit": profile.use_moveit,
        "recording_enabled": profile.recording_enabled,
        "recording_root": Path(profile.recording_root).expanduser(),
        "recording_video_topic": profile.recording_video_topic,
        "diagnostics_enabled": profile.diagnostics_enabled,
        "first_arrival_mode": profile.first_arrival_mode,
        "min_activity": profile.min_activity,
        "ekf_process_noise": profile.ekf_process_noise,
        "ekf_measurement_noise": profile.ekf_measurement_noise,
        "ekf_yolo_measurement_noise": profile.ekf_yolo_measurement_noise,
        "ekf_mobilenetv2_measurement_noise": profile.ekf_mobilenetv2_measurement_noise,
        "ekf_innovation_gate": profile.ekf_innovation_gate,
        "ekf_average_window": profile.ekf_average_window,
        "smoothing_alpha": profile.smoothing_alpha,
        "angular_deadband": profile.angular_deadband,
        "max_velocity": profile.max_velocity,
        "max_acceleration": profile.max_acceleration,
        "max_jerk": profile.max_jerk,
        "max_tracking_error": profile.max_tracking_error,
        "command_rate_hz": profile.command_rate_hz,
        "require_home_before_tracking": profile.require_home_before_tracking,
        "home_duration_s": profile.home_duration_s,
    }
    values.update(profile.expert_arguments)
    return [_arg(name, value) for name, value in values.items()]


def sourced_host_command(project_root: Path, profile: RunProfile) -> list[str]:
    script = (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(project_root / 'install/setup.bash'))} && exec \"$@\""
    )
    return [
        "/bin/bash", "-lc", script, "dracoviloc-gui",
        "ros2", "launch", "dracoviloc_bringup", "arm_demo.launch.py",
        *host_launch_arguments(profile),
    ]


def yolo_docker_command(project_root: Path, profile: RunProfile) -> list[str]:
    del project_root  # The Isaac workspace is an external runtime workspace.
    ros_command = [
        "ros2", "launch", "isaac_ros_yolo_direction", "yolo_camera.launch.py",
        _arg("camera", profile.camera),
        _arg("model_path", profile.yolo_model_path),
        _arg("engine_path", profile.yolo_engine_path),
        "width:=640", "height:=480", "num_blocks:=4",
        "camera_fps:=15", "publish_rate:=15.0",
        "direction_frame:=table_mic_link", _arg("use_viewer", profile.use_yolo_viewer),
    ]
    inner = " ".join([
        "export CUDA_MODULE_LOADING=LAZY;",
        "source /opt/ros/humble/setup.bash;",
        "source /workspaces/isaac_ros-dev/install/setup.bash;",
        "exec", shlex.join(ros_command),
    ])
    return [
        "docker", "run", "--rm", "--privileged", "--network", "host",
        "--ipc", "host", "--runtime", "nvidia",
        "--name", "isaac_yolo", *container_user_args(),
        "-e", "ROS_DOMAIN_ID=42", "-e", "FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
        "-e", "CUDA_MODULE_LOADING=LAZY",
        "-v", f"{Path.home() / 'workspaces/isaac_ros-dev'}:/workspaces/isaac_ros-dev",
        "--workdir", "/workspaces/isaac_ros-dev",
        resolve_yolo_image(profile.yolo_image), "/bin/bash", "-lc", inner,
    ]

