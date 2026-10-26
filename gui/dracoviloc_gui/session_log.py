"""Durable logging for the control center.

Two places, nothing nested:

* ``runs/<YYYY-MM-DD_HH-MM-SS>/`` - one folder per run, holding everything
  that run produced:

      run.json         profile, exact commands, machine, git revision,
                       start/end time and outcome
      events.jsonl     operator actions, state changes and process events
                       during the run
      processes/       full terminal output of host, YOLO, ... (+ .status.json)
      video/ audio/    recordings (uploaded to Google Drive)
      telemetry.jsonl, bag/, diagnostics/   written by the ROS side

* ``logs/`` - activity outside runs (no experiment data):

      <YYYY-MM-DD>.jsonl   every event of every GUI session that day
      jobs/                terminal output of setup & build jobs

Every event is flushed to the OS immediately. Actions, state changes,
process events and errors are also fsync'ed, so they survive a crash or a
power cut; routine info lines are fsync'ed at least once per second.
"""
from __future__ import annotations

import getpass
import json
import os
import platform
import socket
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from . import __version__

DURABLE_KINDS = {"action", "state", "process", "run", "session"}
FSYNC_INTERVAL_S = 1.0
RUN_NAME_FORMAT = "%Y-%m-%d_%H-%M-%S"


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def _git(project_root: Path, *args: str) -> str:
    try:
        result = subprocess.run(
            ["git", "-C", str(project_root), *args],
            capture_output=True, text=True, timeout=5.0)
        return result.stdout.strip() if result.returncode == 0 else ""
    except (OSError, subprocess.TimeoutExpired):
        return ""


def _unique(path: Path) -> Path:
    """path, or path_2.ext, path_3.ext, ... if it exists."""
    candidate, suffix = path, 1
    while candidate.exists():
        suffix += 1
        candidate = path.with_name(f"{path.stem}_{suffix}{path.suffix}")
    return candidate


class _JsonlFile:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.stream = open(path, "a", encoding="utf-8")
        self.last_fsync = time.monotonic()

    def write(self, line: str, durable: bool) -> None:
        self.stream.write(line + "\n")
        self.stream.flush()
        if durable or time.monotonic() - self.last_fsync >= FSYNC_INTERVAL_S:
            os.fsync(self.stream.fileno())
            self.last_fsync = time.monotonic()

    def close(self) -> None:
        if not self.stream.closed:
            os.fsync(self.stream.fileno())
            self.stream.close()


class SessionLog:
    def __init__(self, logs_root: Path, project_root: Path) -> None:
        self.directory = Path(logs_root).expanduser()
        self.jobs_directory = self.directory / "jobs"
        self.jobs_directory.mkdir(parents=True, exist_ok=True)
        self.project_root = project_root
        self.run_directory: Path | None = None
        self._lock = threading.Lock()
        self._daily: _JsonlFile | None = None
        self._run_events: _JsonlFile | None = None
        self.machine = {
            "host": socket.gethostname(),
            "user": getpass.getuser(),
            "machine": platform.machine(),
            "kernel": platform.release(),
            "python": platform.python_version(),
            "gui_version": __version__,
            "project_root": str(project_root),
        }
        self.event("session", "gui", "info", "Session started", {
            **self.machine, "git": self.git_state()})

    def git_state(self) -> dict[str, Any]:
        return {
            "commit": _git(self.project_root, "rev-parse", "HEAD"),
            "branch": _git(self.project_root, "rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(_git(self.project_root, "status", "--porcelain", "--untracked-files=no")),
        }

    def _daily_file(self) -> _JsonlFile:
        path = self.directory / f"{datetime.now():%Y-%m-%d}.jsonl"
        if self._daily is None or self._daily.path != path:  # new day: new file
            if self._daily is not None:
                self._daily.close()
            self._daily = _JsonlFile(path)
        return self._daily

    def event(self, kind: str, subsystem: str, severity: str, message: str,
              data: dict[str, Any] | None = None) -> dict[str, Any]:
        record = {
            "ts": now_iso(), "kind": kind, "subsystem": subsystem,
            "severity": severity, "message": message,
        }
        if data:
            record["data"] = data
        line = json.dumps(record, default=str)
        durable = kind in DURABLE_KINDS or severity in {"error", "warning"}
        with self._lock:
            self._daily_file().write(line, durable)
            if self._run_events is not None:
                self._run_events.write(line, durable)
        return record

    # -------------------------------------------------------------------- runs
    def begin_run(self, runs_root: Path) -> Path:
        """Create runs/<YYYY-MM-DD_HH-MM-SS>/; events are copied there until end_run."""
        directory = _unique(Path(runs_root).expanduser() / datetime.now().strftime(RUN_NAME_FORMAT))
        (directory / "processes").mkdir(parents=True)
        with self._lock:
            self.run_directory = directory
            self._run_events = _JsonlFile(directory / "events.jsonl")
        self.event("run", "gui", "info", "Run folder created", {"run": str(directory)})
        return directory

    def record_run(self, profile: dict[str, Any], commands: dict[str, list[str]],
                   extra: dict[str, Any] | None = None) -> Path:
        path = self.run_directory / "run.json"
        self._write_json(path, {
            "started_at": now_iso(), "ended_at": None, "outcome": None,
            "machine": self.machine, "git": self.git_state(),
            "profile": profile, "commands": commands, **(extra or {}),
        })
        return path

    def end_run(self, outcome: str) -> None:
        if self.run_directory is None:
            return
        path = self.run_directory / "run.json"
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            data = {}
        data.update(ended_at=now_iso(), outcome=outcome)
        self._write_json(path, data)
        self.event("run", "gui", "info", f"Run ended: {outcome}", {"run": str(self.run_directory)})
        with self._lock:
            if self._run_events is not None:
                self._run_events.close()
            self._run_events = None
            self.run_directory = None

    @staticmethod
    def _write_json(path: Path, data: dict[str, Any]) -> None:
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, indent=2, default=str), encoding="utf-8")
        temporary.replace(path)

    def process_log_paths(self, name: str) -> tuple[Path, Path]:
        """Run subsystems log into the run folder; setup jobs into logs/jobs/."""
        if self.run_directory is not None and not name.startswith("job-"):
            log = _unique(self.run_directory / "processes" / f"{name}.log")
        else:
            log = _unique(self.jobs_directory / f"{datetime.now():{RUN_NAME_FORMAT}}_{name}.log")
        return log, log.with_suffix(".status.json")

    def close(self) -> None:
        if self.run_directory is not None:
            self.end_run("gui closed during the run")
        self.event("session", "gui", "info", "Session ended")
        with self._lock:
            if self._daily is not None:
                self._daily.close()
                self._daily = None
