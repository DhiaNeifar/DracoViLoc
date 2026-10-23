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


def test_required_dds_environment_is_forced():
    env = runtime_environment({"ROS_DOMAIN_ID": "0"})
    assert env["ROS_DOMAIN_ID"] == "42"
    assert env["FASTDDS_BUILTIN_TRANSPORTS"] == "UDPv4"
    assert env["CUDA_MODULE_LOADING"] == "LAZY"


def test_docker_command_is_named_and_uses_t2000_image():
    args = yolo_docker_command(Path("/tmp/repo"), RunProfile(yolo_enabled=True))
    assert "isaac_yolo" in args
    assert "isaac_ros_yolo:draco-t2000" in args
    assert any("yolo_camera.launch.py" in token for token in args)


def test_yolo_values_are_shell_quoted_inside_container():
    profile = RunProfile(camera="/dev/video 4")
    command = yolo_docker_command(Path("/tmp/repo"), profile)[-1]
    assert "'camera:=/dev/video 4'" in command

