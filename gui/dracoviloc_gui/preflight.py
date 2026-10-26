from __future__ import annotations

import os
import re
import shutil
import socket
import subprocess
from pathlib import Path

from .host_platform import (
    AUTO, gpu_name, isaac_root, resolve_mobilenet_runtime, resolve_yolo_image)
from .models import CheckResult, RunProfile
from .yolo_models import DETECTOR_NAMES, resolve_detector
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


def _drive_sync_check() -> CheckResult:
    """Recordings are uploaded by the sync-runs-to-drive timer (rclone)."""
    label = "Google Drive upload of recordings"
    if not shutil.which("rclone"):
        return CheckResult("drive_sync", label, False,
                           "rclone is not installed; see docs/DRIVE_SYNC.md", blocking=False)
    timer_ok, timer_state = _run_ok(
        ["systemctl", "--user", "is-active", "sync-runs-to-drive.timer"])
    if not timer_ok:
        return CheckResult("drive_sync", label, False,
                           f"Sync timer is {timer_state or 'not installed'}; "
                           "run scripts/install_drive_sync.sh", blocking=False)
    return CheckResult("drive_sync", label, True, "Sync timer active")


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
        # ODAS is pointed at the detected card at launch (uma16_card:=auto).
        results.append(CheckResult(
            "uma16", "UMA16 microphone", has_uma,
            f"Detected as ALSA card {match.group(1)}" if match else "Not detected", True))
    if profile.mobilenetv2_enabled:
        engine_setting = profile.mobilenetv2_engine_path
        python_setting = profile.mobilenetv2_venv_python
        if AUTO in (engine_setting, python_setting):
            selected = resolve_mobilenet_runtime(project_root)
            results.append(CheckResult(
                "mobilenet_runtime", "MobileNetV2 runtime for this machine",
                "error" not in selected, selected.get("error") or
                f"{selected['gpu']}: {Path(selected['engine']).name}"))
            if "error" in selected:
                selected = {"engine": "", "python": ""}
            if engine_setting == AUTO:
                engine_setting = selected["engine"]
            if python_setting == AUTO:
                python_setting = selected["python"]
        engine = Path(engine_setting).expanduser()
        python = Path(python_setting).expanduser()
        results.append(CheckResult(
            "mobilenet_python", "MobileNetV2 Python runtime",
            python.is_file() and os.access(python, os.X_OK), str(python)))
        results.append(_exists(
            "mobilenet", "MobileNetV2 TensorRT engine", engine))
        if python.is_file() and engine.is_file():
            deserialize = (
                "import pathlib,sys,tensorrt as trt; "
                "p=pathlib.Path(sys.argv[1]); "
                "r=trt.Runtime(trt.Logger(trt.Logger.ERROR)); "
                "e=r.deserialize_cuda_engine(p.read_bytes()); "
                "raise SystemExit(0 if e else 2)")
            compatible, detail = _run_ok(
                [str(python), "-c", deserialize, str(engine)], 10.0)
            results.append(CheckResult(
                "mobilenet_deserialize", "MobileNetV2 engine compatibility",
                compatible, "Engine loaded successfully" if compatible else
                (detail or "TensorRT deserialization failed")))
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
        image = resolve_yolo_image(profile.yolo_image)
        image_ok, image_detail = _run_ok([
            "docker", "image", "inspect", image, "--format", "{{.Id}}"]) if image else (
                False, "No known Isaac ROS image for this CPU architecture")
        results.append(CheckResult(
            "yolo_image", "YOLO container image", image_ok,
            image if image_ok else f"Missing: {image or image_detail}"))
        results.append(_exists(
            "isaac_workspace", "Isaac ROS workspace", isaac_root() / "install/setup.bash"))
        try:
            name = DETECTOR_NAMES.get(profile.visual_detector, profile.visual_detector)
            choice = profile.visual_model_choice()
            _, model = resolve_detector(project_root, profile.visual_detector, choice)
            results.append(CheckResult(
                "yolo_model", f"{name} model", True,
                f"{model.id} · {model.plan_detail}" + (" (auto)" if choice == AUTO else "")))
        except LookupError as error:
            results.append(CheckResult("yolo_model", f"{name} model", False, str(error)))
        camera_found = any(
            "icspring" in name.read_text(errors="ignore").lower()
            for name in Path("/sys/class/video4linux").glob("video*/name")
        )
        results.append(CheckResult(
            "camera", "icSpring camera", camera_found,
            "Detected" if camera_found else "Not detected"))
    if profile.yolo_enabled or profile.mobilenetv2_enabled or profile.ast_enabled or profile.gre_enabled:
        gpu = gpu_name()
        results.append(CheckResult(
            "gpu", "NVIDIA GPU (TensorRT)", bool(gpu),
            gpu or "Not found; YOLO and the audio classifiers need an NVIDIA GPU"))
    if profile.hardware_mode == "real":
        controllers = (
            (("left_robot", "Left FAIRINO cobot", profile.left_robot_ip),
             ("right_robot", "Right FAIRINO cobot", profile.right_robot_ip))
            if profile.dual_arm else
            (("robot", "FAIRINO controller", profile.robot_ip),)
        )
        for key, label, address in controllers:
            ping_ok, detail = _run_ok(["ping", "-c", "1", "-W", "1", address], 2.0)
            results.append(CheckResult(
                key, label, ping_ok,
                f"{address} reachable" if ping_ok else f"{address}: {detail or 'unreachable'}"))
        results.append(_exists(
            "fairino_sdk", "FAIRINO SDK",
            project_root / "src/fairino_hardware/libfairino/lib/libfairino.so"))
    if profile.recording_enabled or profile.yolo_enabled:
        results.append(_drive_sync_check())
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

