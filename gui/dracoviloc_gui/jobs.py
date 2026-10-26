"""Setup & build jobs: what each one checks, and the command that fixes it.

A job is a status check plus one command. The command is always a script or
tool that also works from a terminal; the GUI runs it as a systemd unit
(``dracoviloc-job-<id>.service``) so its full output is logged and shown live.
Checks are cheap, read-only and never need the network.
"""
from __future__ import annotations

import os
import shlex
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable

from . import docker_control
from .host_platform import (
    base_image_candidates, isaac_root, machine_name, project_image_tags,
    project_image_version, resolve_yolo_image)

OK, STALE, MISSING, BLOCKED, UNKNOWN = "ok", "stale", "missing", "blocked", "unknown"
ISAAC_PACKAGES = ("isaac_ros_yolo_bringup", "isaac_ros_yolo_direction", "yolo_video_publisher")
IGNORED_PARTS = {"__pycache__", ".pytest_cache", ".git"}
ELF_MACHINES = {0x3E: "x86_64", 0xB7: "aarch64"}
AUTO_MODEL = "auto"


@dataclass
class JobStatus:
    state: str
    summary: str
    details: list[str] = field(default_factory=list)


@dataclass
class Job:
    id: str
    title: str
    description: str
    action: str
    check: Callable[[Path], JobStatus]
    command: Callable[[Path], list[str]]
    in_update_all: bool = True
    stop_timeout_s: float = 30.0


def container_user_args() -> list[str]:
    """Make the Isaac entrypoint run commands as the host user (file ownership)."""
    return ["-e", "USERNAME=admin", "-e", f"HOST_USER_UID={os.getuid()}",
            "-e", f"HOST_USER_GID={os.getgid()}"]


def _newest_mtime(directory: Path) -> tuple[float, str]:
    newest, newest_path = 0.0, ""
    for root, dirs, files in os.walk(directory):
        dirs[:] = [d for d in dirs if d not in IGNORED_PARTS]
        for name in files:
            if name.endswith(".pyc"):
                continue
            path = os.path.join(root, name)
            try:
                mtime = os.lstat(path).st_mtime
            except OSError:
                continue
            if mtime > newest:
                newest, newest_path = mtime, path
    return newest, newest_path


def _build_stamp(build_root: Path, package: str) -> float | None:
    """mtime of a successful colcon build of the package, or None."""
    stamp = build_root / package / "colcon_build.rc"
    try:
        if stamp.read_text().strip() != "0":
            return None
        return stamp.stat().st_mtime
    except OSError:
        return None


def _stale_packages(packages: dict[str, Path], build_root: Path) -> list[str]:
    stale = []
    for package, source in sorted(packages.items()):
        stamp = _build_stamp(build_root, package)
        if stamp is None:
            stale.append(f"{package}: never built successfully")
            continue
        newest, path = _newest_mtime(source)
        if newest > stamp:
            stale.append(f"{package}: {Path(path).relative_to(source)} changed")
    return stale


# ---------------------------------------------------------------- FAIRINO SDK
def sdk_library(project_root: Path) -> Path:
    return project_root / "src/fairino_hardware/libfairino/lib/libfairino.so"


def check_fairino_sdk(project_root: Path) -> JobStatus:
    library = sdk_library(project_root)
    try:
        header = library.read_bytes()[:20]
    except OSError:
        return JobStatus(MISSING, "Not built on this machine",
                         [f"{library} is missing (it is not stored in git)."])
    if header[:4] != b"\x7fELF":
        return JobStatus(STALE, "Library is not a valid ELF file", [str(library)])
    built_for = ELF_MACHINES.get(int.from_bytes(header[18:20], "little"), "unknown")
    if built_for != machine_name():
        return JobStatus(STALE, f"Built for {built_for}, this machine is {machine_name()}",
                         [str(library)])
    return JobStatus(OK, f"Built for {built_for}", [str(library)])


def build_fairino_sdk(project_root: Path) -> list[str]:
    return [str(project_root / "scripts/build_fairino_sdk.sh")]


# ------------------------------------------------------------- host workspace
def host_packages(project_root: Path) -> dict[str, Path]:
    return {path.parent.name: path.parent
            for path in (project_root / "src").glob("*/package.xml")}


def check_host_workspace(project_root: Path) -> JobStatus:
    stale = _stale_packages(host_packages(project_root), project_root / "build")
    if not (project_root / "install/setup.bash").exists():
        return JobStatus(MISSING, "Workspace has never been built", stale)
    if stale:
        return JobStatus(STALE, f"{len(stale)} package(s) changed since the last build", stale)
    return JobStatus(OK, f"{len(host_packages(project_root))} packages up to date")


def build_host_workspace(project_root: Path) -> list[str]:
    script = (
        "set -e; set +u; source /opt/ros/humble/setup.bash; set -u; "
        f"cd {shlex.quote(str(project_root))}; "
        # src/ only: isaac_ros/ is built inside the container, yolo-alt/ is not ours.
        "colcon build --symlink-install --base-paths src "
        "--event-handlers console_cohesion+ summary+")
    return ["bash", "-c", script]


# --------------------------------------------------------------- project image
def check_project_image(project_root: Path) -> JobStatus:
    versioned, latest = project_image_tags(project_root)
    if not base_image_candidates():
        return JobStatus(BLOCKED, f"No Isaac ROS image is known for {machine_name()}")
    try:
        if docker_control.image_exists(versioned):
            return JobStatus(OK, versioned)
        bases = [image for image in base_image_candidates() if docker_control.image_exists(image)]
        if not bases:
            return JobStatus(BLOCKED, "No Isaac ROS base image on this machine",
                             ["Expected one of: " + ", ".join(base_image_candidates())])
        if docker_control.image_exists(latest):
            return JobStatus(STALE, "The Dockerfile changed since the image was built",
                             [f"Current: {latest}", f"Will build: {versioned}",
                              f"Base image: {bases[0]}"])
        return JobStatus(MISSING, "Project image not built yet",
                         [f"Will build: {versioned}", f"Base image: {bases[0]}"])
    except docker_control.DockerException as error:
        return JobStatus(UNKNOWN, f"Docker unavailable: {error}")


def build_project_image(project_root: Path) -> list[str]:
    versioned, latest = project_image_tags(project_root)
    base = next((image for image in base_image_candidates()
                 if docker_control.image_exists(image)), "")
    return ["docker", "build", "--progress=plain",
            "--build-arg", f"BASE_IMAGE={base}",
            "--build-arg", f"DRACOVILOC_IMAGE_VERSION={project_image_version(project_root)}",
            "-t", versioned, "-t", latest,
            str(project_root / "isaac_ros/docker")]


# ------------------------------------------------------- Isaac ROS workspace
def _deploy_changes(project_root: Path) -> list[str]:
    result = subprocess.run(
        [str(project_root / "scripts/deploy_isaac_ros.sh"), "--dry-run", str(isaac_root())],
        capture_output=True, text=True, timeout=60.0)
    if result.returncode != 0:
        raise RuntimeError((result.stderr or result.stdout).strip())
    changes, prefix = [], ""
    for line in result.stdout.splitlines():
        if line.startswith("## "):
            prefix = line[3:].strip()
        elif line.startswith("*deleting") or (len(line) > 12 and line[0] in "<>ch" and line[1] in "fdL"):
            item = (line.split(" ", 1)[1] if " " in line else line).strip()
            if not item.endswith("/"):
                changes.append(("deleted: " if line.startswith("*deleting") else "") + prefix + item)
    return changes


def check_isaac_sync(project_root: Path) -> JobStatus:
    if not (isaac_root() / "src/isaac_ros_common").is_dir():
        return JobStatus(BLOCKED, f"Isaac ROS workspace not found at {isaac_root()}")
    try:
        changes = _deploy_changes(project_root)
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        return JobStatus(UNKNOWN, f"Could not compare: {error}")
    if changes:
        return JobStatus(STALE, f"{len(changes)} file(s) differ from this repository", changes[:200])
    return JobStatus(OK, "Isaac workspace matches this repository")


def sync_isaac(project_root: Path) -> list[str]:
    return [str(project_root / "scripts/deploy_isaac_ros.sh"), str(isaac_root())]


def check_isaac_build(project_root: Path) -> JobStatus:
    workspace = isaac_root()
    if not (workspace / "src/isaac_ros_common").is_dir():
        return JobStatus(BLOCKED, f"Isaac ROS workspace not found at {workspace}")
    packages = {name: workspace / "src" / name for name in ISAAC_PACKAGES
                if (workspace / "src" / name).is_dir()}
    if len(packages) != len(ISAAC_PACKAGES):
        return JobStatus(BLOCKED, "DracoViLoc packages are not in the Isaac workspace yet",
                         ["Run 'Sync Isaac ROS workspace' first."])
    stale = _stale_packages(packages, workspace / "build")
    if stale:
        return JobStatus(STALE, f"{len(stale)} package(s) need a container build", stale)
    return JobStatus(OK, "YOLO packages built in the container")


def build_isaac(project_root: Path) -> list[str]:
    workspace = isaac_root()
    return ["docker", "run", "--rm", "--network", "host", "--runtime", "nvidia",
            "--name", "dracoviloc_isaac_build", *container_user_args(),
            "-v", f"{workspace}:/workspaces/isaac_ros-dev",
            "--workdir", "/workspaces/isaac_ros-dev",
            resolve_yolo_image("auto"),
            "/workspaces/isaac_ros-dev/scripts/build_dracoviloc_yolo.sh"]


# --------------------------------------------------------------- git update
def check_git(project_root: Path) -> JobStatus:
    def git(*args: str) -> str:
        return subprocess.run(["git", "-C", str(project_root), *args],
                              capture_output=True, text=True, timeout=10.0).stdout.strip()
    try:
        branch = git("rev-parse", "--abbrev-ref", "HEAD")
        commit = git("log", "-1", "--format=%h %s (%cr)")
        dirty = git("status", "--porcelain", "--untracked-files=no").splitlines()
        upstream = git("rev-parse", "--abbrev-ref", "@{upstream}")
        behind = git("rev-list", "--count", "HEAD..@{upstream}") if upstream else ""
    except (OSError, subprocess.TimeoutExpired) as error:
        return JobStatus(UNKNOWN, f"git unavailable: {error}")
    details = [f"Branch {branch}" + (f" tracking {upstream}" if upstream else " (no upstream)"),
               f"Commit {commit}"]
    if dirty:
        details.append(f"{len(dirty)} file(s) modified locally: pull is disabled")
        return JobStatus(BLOCKED, f"{branch}: local changes", details + dirty[:50])
    if behind and behind != "0":
        return JobStatus(STALE, f"{branch}: {behind} commit(s) behind the last fetch", details)
    return JobStatus(OK, f"{branch} at {commit.split(' ')[0]}", details)


def git_pull(project_root: Path) -> list[str]:
    return ["git", "-C", str(project_root), "pull", "--ff-only"]


# ------------------------------------------------------------------ YOLO model
def check_yolo_model(project_root: Path) -> JobStatus:
    from .yolo_models import Registry, newest_needs_work
    registry = Registry(project_root)
    if not registry.gpu:
        return JobStatus(BLOCKED, "No NVIDIA GPU: TensorRT plans cannot be built here")
    pending = newest_needs_work(registry)
    if pending is None:
        try:
            model = registry.resolve(AUTO_MODEL)
        except LookupError as error:
            return JobStatus(MISSING, str(error))
        return JobStatus(OK, f"Newest model ready: {model.id}", [model.plan_detail])
    return JobStatus(MISSING if pending.plan_state == "missing" else STALE,
                     f"Newest model {pending.id} is not ready on {registry.gpu}",
                     [f"ONNX: {pending.onnx_detail}", f"Plan: {pending.plan_detail}",
                      "Building a TensorRT plan takes about 10 minutes on a Jetson Orin."])


def prepare_yolo_model(project_root: Path) -> list[str]:
    return yolo_models_command("prepare", AUTO_MODEL)


def yolo_models_command(*args: str) -> list[str]:
    return [sys.executable, "-m", "dracoviloc_gui.yolo_models", *args]


JOBS: tuple[Job, ...] = (
    Job("git", "Repository", "Fetch and fast-forward this checkout from GitHub.",
        "Pull latest", check_git, git_pull, in_update_all=False),
    Job("fairino_sdk", "FAIRINO SDK",
        "Native libfairino for this CPU (not stored in git; each machine builds its own).",
        "Build SDK", check_fairino_sdk, build_fairino_sdk),
    Job("host_build", "DracoViLoc workspace",
        "colcon build of src/ on the host.",
        "Build workspace", check_host_workspace, build_host_workspace, stop_timeout_s=60.0),
    Job("project_image", "Project Docker image",
        "Isaac ROS image with every DracoViLoc dependency baked in (isaac_ros/docker).",
        "Build image", check_project_image, build_project_image, stop_timeout_s=60.0),
    Job("isaac_sync", "Isaac ROS workspace sources",
        "Copy isaac_ros/ packages, models and tools into the Isaac workspace.",
        "Sync", check_isaac_sync, sync_isaac),
    Job("isaac_build", "Isaac ROS build",
        "colcon build of the YOLO packages inside the project container.",
        "Build in container", check_isaac_build, build_isaac, stop_timeout_s=60.0),
    Job("yolo_model", "YOLO model for this GPU",
        "Export the newest checkpoint to ONNX (if needed) and build its TensorRT plan for this GPU.",
        "Prepare newest model", check_yolo_model, prepare_yolo_model, stop_timeout_s=60.0),
)
JOBS_BY_ID = {job.id: job for job in JOBS}


def check_all(project_root: Path) -> dict[str, JobStatus]:
    statuses = {}
    for job in JOBS:
        try:
            statuses[job.id] = job.check(project_root)
        except Exception as error:  # a broken check must not hide the others
            statuses[job.id] = JobStatus(UNKNOWN, f"Check failed: {error}")
    return statuses


def describe(job: Job, status: JobStatus) -> dict:
    return {"id": job.id, "title": job.title, "description": job.description,
            "action": job.action, "in_update_all": job.in_update_all, **asdict(status)}


__all__ = ["JOBS", "JOBS_BY_ID", "Job", "JobStatus", "check_all", "container_user_args",
           "describe"]
