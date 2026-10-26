from pathlib import Path

from dracoviloc_gui.host_platform import resolve_yolo_image, yolo_image_candidates

ROOT = Path(__file__).resolve().parents[2]


def test_yolo_image_follows_cpu_architecture():
    installed = {"isaac_ros_dev-aarch64:latest", "isaac_ros_dev-x86_64:latest"}
    exists = installed.__contains__
    assert resolve_yolo_image("auto", "aarch64", exists) == "isaac_ros_dev-aarch64:latest"
    assert resolve_yolo_image("auto", "x86_64", exists) == "isaac_ros_dev-x86_64:latest"
    assert resolve_yolo_image("auto", "x86_64", lambda image: True) == "dracoviloc/isaac_ros:x86_64-latest"
    assert resolve_yolo_image(
        "auto", "x86_64", {"isaac_ros_yolo:draco-t2000"}.__contains__) == "isaac_ros_yolo:draco-t2000"


def test_explicit_yolo_image_wins():
    assert resolve_yolo_image("custom:1", "aarch64", lambda image: False) == "custom:1"


def test_unknown_architecture_has_no_image():
    assert yolo_image_candidates("riscv64") == ()
    assert resolve_yolo_image("auto", "riscv64", lambda image: True) == ""


def test_agent_copy_is_identical():
    gui_copy = ROOT / "gui/dracoviloc_gui/host_platform.py"
    agent_copy = ROOT / "remote_agent/dracoviloc_agent/host_platform.py"
    assert gui_copy.read_text() == agent_copy.read_text()


def test_project_image_version_tracks_the_dockerfile(tmp_path):
    from dracoviloc_gui.host_platform import project_image_tags
    context = tmp_path / "isaac_ros/docker"
    context.mkdir(parents=True)
    (context / "Dockerfile").write_text("FROM base\n")
    first, latest = project_image_tags(tmp_path, "aarch64")
    assert latest == "dracoviloc/isaac_ros:aarch64-latest"
    (context / "Dockerfile").write_text("FROM base\nRUN true\n")
    assert project_image_tags(tmp_path, "aarch64")[0] != first
