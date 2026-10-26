from pathlib import Path

from dracoviloc_gui.commands import (
    host_launch_arguments, runtime_environment, sourced_host_command,
    yolo_docker_command,
)
from dracoviloc_gui.models import RunProfile


def test_host_uses_real_launch_name_and_explicit_recording_default():
    args = sourced_host_command(Path("/tmp/DracoViLoc"), RunProfile())
    assert "arm_demo.launch.py" in args
    assert "arm_audio_demo.launch.py" not in args
    assert "recording_enabled:=false" in args


def test_all_arguments_are_individual_process_tokens():
    profile = RunProfile(recording_root="/tmp/a directory/runs")
    args = host_launch_arguments(profile)
    assert "recording_root:=/tmp/a directory/runs" in args
    assert any(arg.startswith("mobilenetv2_venv_python:=") for arg in args)


def test_mobilenet_runtime_is_chosen_per_machine_by_default():
    args = host_launch_arguments(RunProfile())
    assert "mobilenetv2_engine_path:=auto" in args
    assert "mobilenetv2_venv_python:=auto" in args


def test_explicit_mobilenet_runtime_is_passed_through():
    args = host_launch_arguments(RunProfile(
        mobilenetv2_engine_path="/models/custom.engine",
        mobilenetv2_venv_python="/opt/python3"))
    assert "mobilenetv2_engine_path:=/models/custom.engine" in args
    assert "mobilenetv2_venv_python:=/opt/python3" in args


def test_required_dds_environment_is_forced():
    env = runtime_environment({"ROS_DOMAIN_ID": "0"})
    assert env["ROS_DOMAIN_ID"] == "42"
    assert env["FASTDDS_BUILTIN_TRANSPORTS"] == "UDPv4"
    assert env["CUDA_MODULE_LOADING"] == "LAZY"


def test_docker_command_is_named_and_uses_the_profile_image():
    args = yolo_docker_command(
        Path("/tmp/repo"), RunProfile(yolo_enabled=True, yolo_image="my/isaac:tag"))
    assert "isaac_yolo" in args
    assert "my/isaac:tag" in args
    assert any("yolo_camera.launch.py" in token for token in args)
    assert "num_blocks:=4" in args[-1]
    assert "camera_fps:=15" in args[-1]


def test_yolo_values_are_shell_quoted_inside_container():
    profile = RunProfile(camera="/dev/video 4")
    command = yolo_docker_command(Path("/tmp/repo"), profile)[-1]
    assert "'camera:=/dev/video 4'" in command


def test_dual_cobot_launch_arguments_are_complete():
    profile = RunProfile(
        dual_arm=True, left_robot_ip="192.168.58.2", right_robot_ip="192.168.58.3",
        second_arm_x=-0.8, second_arm_y=0.1, second_arm_z=0.2,
        dual_tracking_policy="simultaneous",
        lead_lock_tolerance_rad=0.08)
    args = host_launch_arguments(profile)
    for expected in (
        "dual_arm:=true", "left_robot_ip:=192.168.58.2",
        "right_robot_ip:=192.168.58.3", "second_arm_x:=-0.8",
        "second_arm_y:=0.1", "second_arm_z:=0.2",
        "dual_tracking_policy:=simultaneous",
        "lead_lock_tolerance_rad:=0.08",
    ):
        assert expected in args

