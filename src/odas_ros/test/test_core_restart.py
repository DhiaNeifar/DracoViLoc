"""An ODAS core that exits (e.g. after an invalidated SSS/SST session) is
restarted; only a crash loop ends the bridge."""

import subprocess
from types import SimpleNamespace

from stub_ros import lib_odas_server_node

Node = lib_odas_server_node.OdasServerNode


def make_node():
    node = Node.__new__(Node)
    node._stopping = False
    node._core_failure_reported = False
    node._started_monotonic = 0.0
    node._diagnostic_log = SimpleNamespace(path="diagnostics.jsonl")
    node.events, node.starts = [], 0
    node.record_diagnostic_event = lambda kind, **data: node.events.append(kind)
    node._describe_returncode = lambda code: f"exit code {code}"
    logger = SimpleNamespace(warning=lambda *a: None, fatal=lambda *a: None)
    node.get_logger = lambda: logger

    def start():
        node.starts += 1
        node._core_process = subprocess.Popen(["true"])
        node._core_process.wait()
    node._start_core_process = start
    node._core_process = subprocess.Popen(["false"])
    node._core_process.wait()
    return node


def test_exited_core_is_restarted(monkeypatch):
    monkeypatch.setattr(lib_odas_server_node.rclpy, "ok", lambda: False, raising=False)
    node = make_node()
    node._check_core_process()
    assert node.starts == 1 and node.events == ["core_restarted"]
    assert not node._core_failure_reported


def test_crash_loop_gives_up(monkeypatch):
    monkeypatch.setattr(lib_odas_server_node.rclpy, "ok", lambda: False, raising=False)
    node = make_node()
    for _ in range(lib_odas_server_node.CORE_RESTART_LIMIT + 1):
        node._check_core_process()
    assert node.starts == lib_odas_server_node.CORE_RESTART_LIMIT
    assert node._core_failure_reported and node.events[-1] == "core_failed"
    assert node._shutdown_requested  # run() then closes the sockets and exits
