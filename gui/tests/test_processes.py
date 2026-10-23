import sys
import time
from pathlib import Path

from dracoviloc_gui.processes import ProcessSupervisor


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_captures_output_and_exit(tmp_path):
    logs = []
    exits = []
    supervisor = ProcessSupervisor(
        lambda *row: logs.append(row), lambda *row: exits.append(row))
    supervisor.start(
        "test", [sys.executable, "-c", "print('ready')"],
        Path(tmp_path), {})
    assert wait_until(lambda: exits)
    assert any("ready" in row[2] for row in logs)
    assert exits == [("test", 0)]


def test_stop_terminates_process_group(tmp_path):
    supervisor = ProcessSupervisor(lambda *_: None, lambda *_: None)
    supervisor.start(
        "test", [sys.executable, "-c", "import time; time.sleep(30)"],
        Path(tmp_path), {})
    assert supervisor.is_running("test")
    supervisor.stop("test", grace_seconds=0.2)
    assert not supervisor.is_running("test")

