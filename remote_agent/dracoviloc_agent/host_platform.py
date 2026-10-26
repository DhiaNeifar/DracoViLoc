"""Machine-specific choices resolved at run time, so one checkout works on
every host (Jetson Orin, x86 laptop or server, with or without NVIDIA).

A copy of this file is deployed with the remote agent as
remote_agent/dracoviloc_agent/host_platform.py; keep the two identical.
"""
from __future__ import annotations

import hashlib
import json
import os
import platform
import subprocess
from pathlib import Path

AUTO = "auto"

# The DracoViLoc project image (isaac_ros/docker/Dockerfile) is preferred; it
# is built on top of the first available Isaac ROS dev image for the CPU.
PROJECT_IMAGE_REPOSITORY = "dracoviloc/isaac_ros"
BASE_IMAGES = {
    "aarch64": ("isaac_ros_dev-aarch64:latest",),
    "x86_64": ("isaac_ros_dev-x86_64:latest", "isaac_ros_yolo:draco-t2000"),
}
PROJECT_DOCKER_CONTEXT = "isaac_ros/docker"


def machine_name(machine: str | None = None) -> str:
    machine = (machine or platform.machine()).lower()
    return {"arm64": "aarch64", "amd64": "x86_64"}.get(machine, machine)


def isaac_root() -> Path:
    """Host path of the Isaac ROS workspace mounted into the container."""
    return Path(os.environ.get(
        "DRACOVILOC_ISAAC_ROOT", str(Path.home() / "workspaces/isaac_ros-dev"))).expanduser()


def project_image_version(project_root: Path) -> str:
    """Content hash of the project image's build context."""
    digest = hashlib.sha256()
    context = Path(project_root) / PROJECT_DOCKER_CONTEXT
    for path in sorted(context.rglob("*")):
        if path.is_file():
            digest.update(str(path.relative_to(context)).encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()[:12]


def project_image_tags(project_root: Path, machine: str | None = None) -> tuple[str, str]:
    """(versioned tag, latest tag) of the project image for this CPU."""
    arch = machine_name(machine)
    return (f"{PROJECT_IMAGE_REPOSITORY}:{arch}-{project_image_version(project_root)}",
            f"{PROJECT_IMAGE_REPOSITORY}:{arch}-latest")


def base_image_candidates(machine: str | None = None) -> tuple[str, ...]:
    return BASE_IMAGES.get(machine_name(machine), ())


MOBILENET_SELECTOR = "src/dracoviloc_mobilenetv2/mobilenetv2/runtime_selection.py"


def _image_exists(image: str) -> bool:
    try:
        return subprocess.run(
            ["docker", "image", "inspect", image, "--format", "{{.Id}}"],
            capture_output=True, timeout=5.0).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def yolo_image_candidates(machine: str | None = None) -> tuple[str, ...]:
    arch = machine_name(machine)
    if arch not in BASE_IMAGES:
        return ()
    return (f"{PROJECT_IMAGE_REPOSITORY}:{arch}-latest", *BASE_IMAGES[arch])


def resolve_yolo_image(requested: str, machine: str | None = None,
                       image_exists=_image_exists) -> str:
    """Return the requested image, or for 'auto' the first one installed here."""
    if requested and requested != AUTO:
        return requested
    candidates = yolo_image_candidates(machine)
    return next((image for image in candidates if image_exists(image)),
                candidates[0] if candidates else "")


def resolve_mobilenet_runtime(project_root: Path) -> dict:
    """Ask the MobileNetV2 package which engine/interpreter fit this machine.

    Returns {'gpu', 'engine', 'python'} or {'error': message}.
    """
    project_root = Path(project_root).expanduser()
    selector = project_root / MOBILENET_SELECTOR
    try:
        result = subprocess.run(
            ["python3", str(selector), "--model-dir",
             str(project_root / "models/mobilenetv2")],
            capture_output=True, text=True, timeout=90.0)
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (OSError, subprocess.TimeoutExpired, ValueError, IndexError) as error:
        return {"error": f"MobileNetV2 runtime selection failed: {error}"}


def gpu_name() -> str:
    """Return the NVIDIA GPU name, or '' on machines without one."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=5.0)
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip().splitlines()[0].strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    try:
        model = Path("/proc/device-tree/model").read_text(errors="ignore").strip("\x00 \n")
        if "jetson" in model.lower() or "orin" in model.lower():
            return model
    except OSError:
        pass
    return ""
