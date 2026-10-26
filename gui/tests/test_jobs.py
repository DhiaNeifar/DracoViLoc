import os
import time
from pathlib import Path

from dracoviloc_gui import jobs


def elf(machine_code: int) -> bytes:
    return b"\x7fELF" + bytes(14) + machine_code.to_bytes(2, "little") + bytes(40)


def test_sdk_check_detects_wrong_architecture(tmp_path, monkeypatch):
    library = jobs.sdk_library(tmp_path)
    assert jobs.check_fairino_sdk(tmp_path).state == jobs.MISSING
    library.parent.mkdir(parents=True)
    library.write_bytes(elf(0x3E))
    monkeypatch.setattr(jobs, "machine_name", lambda: "aarch64")
    status = jobs.check_fairino_sdk(tmp_path)
    assert status.state == jobs.STALE and "x86_64" in status.summary
    library.write_bytes(elf(0xB7))
    assert jobs.check_fairino_sdk(tmp_path).state == jobs.OK


def test_host_workspace_staleness_follows_sources(tmp_path):
    package = tmp_path / "src/pkg"
    package.mkdir(parents=True)
    (package / "package.xml").write_text("<package/>")
    (tmp_path / "install").mkdir()
    (tmp_path / "install/setup.bash").write_text("")
    assert jobs.check_host_workspace(tmp_path).state == jobs.STALE  # never built
    stamp = tmp_path / "build/pkg/colcon_build.rc"
    stamp.parent.mkdir(parents=True)
    stamp.write_text("0")
    os.utime(package / "package.xml", (time.time() - 100,) * 2)
    os.utime(stamp, (time.time() - 50,) * 2)
    assert jobs.check_host_workspace(tmp_path).state == jobs.OK
    (package / "node.py").write_text("changed")
    status = jobs.check_host_workspace(tmp_path)
    assert status.state == jobs.STALE and "node.py" in status.details[0]
    stamp.write_text("1")  # last build failed
    assert "never built successfully" in jobs.check_host_workspace(tmp_path).details[0]


def test_deploy_dry_run_is_parsed_with_destinations(tmp_path, monkeypatch):
    output = "\n".join([
        "Previewing deployment from DracoViLoc to: /ws",
        "## src/isaac_ros_yolo_direction/",
        ">f.st...... launch/yolo_camera.launch.py",
        ".d..t...... launch/",
        "*deleting   old.py",
        "## scripts/",
        ">f+++++++++ build_isaac_ros.sh",
        "Dry run complete; no files were changed.",
    ])

    class Result:
        returncode, stdout, stderr = 0, output, ""
    monkeypatch.setattr(jobs.subprocess, "run", lambda *a, **k: Result())
    assert jobs._deploy_changes(tmp_path) == [
        "src/isaac_ros_yolo_direction/launch/yolo_camera.launch.py",
        "deleted: src/isaac_ros_yolo_direction/old.py",
        "scripts/build_isaac_ros.sh",
    ]


def test_every_job_has_a_runnable_command():
    root = Path(__file__).resolve().parents[2]
    for job in jobs.JOBS:
        if job.id in {"project_image", "isaac_build"}:
            continue  # need Docker to resolve images
        command = job.command(root)
        assert command and all(isinstance(part, str) for part in command)
