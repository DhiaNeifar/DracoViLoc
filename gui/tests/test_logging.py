import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

from dracoviloc_gui.session_log import SessionLog
from dracoviloc_gui.terminal import ansi_to_styled, severity_of, split_line, strip_ansi

WRAPPER = Path(__file__).resolve().parents[1] / "dracoviloc_gui/logged_exec.py"


def run_wrapped(tmp_path, script, **kwargs):
    log, status = tmp_path / "p.log", tmp_path / "p.status.json"
    process = subprocess.Popen(
        [sys.executable, str(WRAPPER), "--log", str(log), "--status", str(status),
         "--name", "demo", "--", "bash", "-c", script], **kwargs)
    return process, log, status


def test_wrapper_records_terminal_output_and_exit_code(tmp_path):
    process, log, status = run_wrapped(
        tmp_path, 'printf "\\033[32mgreen\\033[0m\\n"; [ -t 1 ] && echo tty; exit 3')
    assert process.wait(10) == 3
    lines = log.read_text().splitlines()
    assert lines[0].split(" ", 1)[1].startswith("--- start demo")
    assert any("\x1b[32mgreen" in line for line in lines)
    assert any(line.endswith(" tty") for line in lines), "command must see a terminal"
    assert "exit demo: code 3" in lines[-1]
    assert json.loads(status.read_text())["exit_code"] == 3


def test_wrapper_forwards_ctrl_c_and_keeps_logging(tmp_path):
    process, log, status = run_wrapped(
        tmp_path, 'trap "echo cleaning up; exit 0" INT; echo ready; sleep 30 & wait')
    deadline = time.monotonic() + 5
    while "ready" not in (log.read_text() if log.exists() else "") and time.monotonic() < deadline:
        time.sleep(0.05)
    process.send_signal(signal.SIGINT)
    assert process.wait(10) == 0
    text = log.read_text()
    assert "received SIGINT" in text and "cleaning up" in text
    assert json.loads(status.read_text())["stop_requested_at"]


def test_session_log_is_durable_jsonl(tmp_path):
    session = SessionLog(tmp_path / "runs", tmp_path)
    session.event("action", "operator", "info", "startRun", {"args": [1]})
    log, status = session.process_log_paths("host")
    assert log.parent == session.directory / "processes"
    session.close()
    rows = [json.loads(line) for line in (session.directory / "events.jsonl").read_text().splitlines()]
    assert [row["message"] for row in rows] == ["Session started", "startRun", "Session ended"]
    assert rows[1]["data"] == {"args": [1]}
    metadata = json.loads((session.directory / "session.json").read_text())
    assert metadata["ended_at"] and metadata["machine"]


def test_ansi_conversion_and_severity():
    styled = ansi_to_styled("\x1b[31mbad <tag>\x1b[0m ok")
    assert styled == '<font color="#FF6B76">bad &lt;tag&gt;</font> ok'
    assert ansi_to_styled("10%\r50%\r100%") == "100%"
    assert strip_ansi("\x1b[1;33mwarn\x1b[0m") == "warn"
    assert severity_of("[ERROR] [node]: boom") == "error"
    assert severity_of("[WARN] [node]: hmm") == "warning"
    assert split_line("2026-10-25T19:38:07.963-07:00 hello") == ("19:38:07.963", "hello")


def _systemd_user_available():
    try:
        from dracoviloc_gui.units import SystemdUser
        SystemdUser().active_units()
        return True
    except Exception:
        return False


@pytest.mark.skipif(not _systemd_user_available(), reason="needs a systemd user session")
def test_unit_supervisor_runs_logs_and_stops(tmp_path):
    from dracoviloc_gui.units import UnitSupervisor
    session = SessionLog(tmp_path / "runs", tmp_path)
    lines, exits = [], []
    supervisor = UnitSupervisor(session, lambda name, rows: lines.extend(rows),
                                lambda name, code: exits.append((name, code)), lambda: None,
                                registry=tmp_path / "registry")
    name = f"pytest-{os.getpid()}"
    supervisor.start(name, ["bash", "-c", "echo hello-unit; trap 'echo bye; exit 0' INT; sleep 60 & wait"],
                     tmp_path, dict(os.environ), stop_timeout_s=5)
    deadline = time.monotonic() + 10
    while not any("hello-unit" in line for line in lines) and time.monotonic() < deadline:
        time.sleep(0.1)
    assert supervisor.is_running(name)
    assert supervisor.stop(name, grace_seconds=10)
    deadline = time.monotonic() + 5
    while not exits and time.monotonic() < deadline:
        time.sleep(0.1)
    supervisor.close()
    assert exits == [(name, 0)]
    assert any("bye" in line for line in lines)
    events = (session.directory / "events.jsonl").read_text()
    assert "Started dracoviloc-" in events and "exited with code 0" in events
