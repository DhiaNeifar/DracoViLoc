from __future__ import annotations

import os
import re
import shutil
import socket
import subprocess
from pathlib import Path

from .models import CheckResult, RunProfile
from .validation import validate_profile


def _exists(key: str, label: str, path: Path, blocking: bool = True) -> CheckResult:
    return CheckResult(key, label, path.exists(), str(path), blocking)


def _port_available(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind(("127.0.0.1", port))
            return True
        except OSError:
            return False


def _run_ok(command: list[str], timeout: float = 3.0) -> tuple[bool, str]:
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        output = (result.stdout or result.stderr).strip()
        return result.returncode == 0, output
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)


def run_preflight(project_root: Path, profile: RunProfile) -> list[CheckResult]:
    results: list[CheckResult] = []
    errors = validate_profile(profile)
    results.append(CheckResult(
        "profile", "Profile consistency", not errors,
        "Ready" if not errors else " ".join(errors)))
    results.extend([
        _exists("ros", "ROS Humble", Path("/opt/ros/humble/setup.bash")),
        _exists("workspace", "Workspace install", project_root / "install/setup.bash"),
        _exists("launch", "Host launch", project_root / "src/dracoviloc_bringup/launch/arm_demo.launch.py"),
        _exists("bringup_package", "Bringup package", project_root / "install/dracoviloc_bringup"),
        _exists("tracking_package", "Tracking package", project_root / "install/dracoviloc_tracking"),
    ])
    disk_path = Path(profile.recording_root).expanduser()
    while not disk_path.exists() and disk_path != disk_path.parent:
        disk_path = disk_path.parent
    free = shutil.disk_usage(disk_path).free
    results.append(CheckResult(
        "disk", "Recording disk space", free >= 2 * 1024**3,
        f"{free / 1024**3:.1f} GiB free", blocking=profile.recording_enabled))
    for port in (9000, 9001, 10000):
        available = _port_available(port)
        results.append(CheckResult(
            f"port_{port}", f"ODAS port {port}", available,
            "Available" if available else "Already in use",
            blocking=profile.audio_enabled))
    if profile.audio_enabled:
        sound_cards = Path("/proc/asound/cards").read_text(errors="ignore") if Path("/proc/asound/cards").exists() else ""
        match = re.search(r"^\s*(\d+)\s+\[[^]]+\].*(?:UMA16|miniDSP)", sound_cards, re.MULTILINE | re.IGNORECASE)
        has_uma = match is not None
        results.append(CheckResult(
            "uma16", "UMA16 microphone", has_uma,
            f"Detected as ALSA card {match.group(1)}" if match else "Not detected", True))
        if match:
            configuration = (project_root / "src/odas_ros/config/configuration.cfg").read_text(encoding="utf-8")
            configured = re.search(r"card\s*=\s*(\d+)\s*;", configuration)
            card_matches = configured is not None and configured.group(1) == match.group(1)
            results.append(CheckResult(
                "alsa_config", "ODAS ALSA card", card_matches,
                f"Configuration uses card {configured.group(1) if configured else 'unknown'}; device is card {match.group(1)}"))
    if profile.mobilenetv2_enabled:
        results.append(_exists(
            "mobilenet", "MobileNetV2 TensorRT engine",
            Path(profile.mobilenetv2_engine_path).expanduser()))
    if profile.ast_enabled:
        results.append(_exists("ast_python", "AST environment", project_root / "trt_env/bin/python3"))
        results.append(_exists("ast_engine", "AST engine", project_root / "models/ast/drone_ast.engine"))
    if profile.gre_enabled:
        results.append(_exists("gre_python", "GRE environment", project_root / "gre_env/bin/python3"))
        results.append(_exists("gre_engine", "GRE engine", project_root / "models/gre/model_logmel.engine"))
    if profile.yolo_enabled:
        docker_ok, docker_detail = _run_ok(["docker", "info", "--format", "{{.ServerVersion}}"])
        results.append(CheckResult("docker", "Docker", docker_ok, docker_detail or "Unavailable"))
        container_ok, container_state = _run_ok([
            "docker", "container", "inspect", "isaac_yolo",
            "--format", "{{.State.Running}}"])
        container_running = container_ok and container_state.strip().lower() == "true"
        results.append(CheckResult(
            "yolo_container", "YOLO container name", not container_running,
            "Available" if not container_ok else (
                "Already running" if container_running else
                "Stopped stale container will be removed before launch"),
            blocking=container_running))
        image_ok, image_detail = _run_ok([
            "docker", "image", "inspect", "isaac_ros_yolo:draco-t2000", "--format", "{{.Id}}"])
        results.append(CheckResult("yolo_image", "YOLO container image", image_ok, image_detail or "Missing"))
        results.append(_exists(
            "isaac_workspace", "Isaac ROS workspace",
            Path.home() / "workspaces/isaac_ros-dev/install/setup.bash"))
        isaac_root = Path.home() / "workspaces/isaac_ros-dev"
        def host_isaac_path(container_path: str) -> Path:
            prefix = "/workspaces/isaac_ros-dev/"
            return isaac_root / container_path.removeprefix(prefix)
        results.append(_exists(
            "yolo_model", "YOLO ONNX model", host_isaac_path(profile.yolo_model_path)))
        results.append(_exists(
            "yolo_engine", "YOLO TensorRT plan", host_isaac_path(profile.yolo_engine_path)))
        camera_found = any(
            "icspring" in name.read_text(errors="ignore").lower()
            for name in Path("/sys/class/video4linux").glob("video*/name")
        )
        results.append(CheckResult(
            "camera", "icSpring camera", camera_found,
            "Detected" if camera_found else "Not detected"))
    if profile.yolo_enabled or profile.mobilenetv2_enabled or profile.ast_enabled or profile.gre_enabled:
        gpu_ok, gpu_detail = _run_ok([
            "nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], 3.0)
        t2000 = gpu_ok and "T2000" in gpu_detail
        results.append(CheckResult(
            "gpu", "Quadro T2000", t2000, gpu_detail or "NVIDIA GPU unavailable"))
    if profile.hardware_mode == "real":
        ping_ok, detail = _run_ok(["ping", "-c", "1", "-W", "1", profile.robot_ip], 2.0)
        results.append(CheckResult("robot", "FAIRINO controller", ping_ok, detail or "Unreachable"))
        results.append(_exists(
            "fairino_sdk", "FAIRINO SDK",
            project_root / "src/fairino_hardware/libfairino/lib/libfairino.so"))
    ffmpeg = shutil.which("ffmpeg")
    results.append(CheckResult(
        "ffmpeg", "FFmpeg", bool(ffmpeg), ffmpeg or "Not installed",
        blocking=profile.recording_enabled and profile.yolo_enabled))
    existing_ok, existing = _run_ok([
        "pgrep", "-af", "ros2 launch dracoviloc_bringup|yolo_camera.launch.py"], 2.0)
    results.append(CheckResult(
        "existing_processes", "Existing DracoViLoc runs", not existing_ok,
        "None detected" if not existing_ok else existing,
        blocking=True))
    return results


def blocking_failures(results: list[CheckResult]) -> list[CheckResult]:
    return [result for result in results if result.blocking and not result.ok]

