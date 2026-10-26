"""Durable session log for the control center.

Each GUI session writes to <runs>/logs/<YYYY-MM-DD_HH-MM-SS>/:

    session.json        machine, git revision, GUI version, start/end time
    events.jsonl        one JSON object per line: operator actions, state
                        changes, process lifecycle, messages and errors
    processes/          full terminal output of every process and job
                        (<name>_<HH-MM-SS>.log + .status.json)
    runs/               profile and launch command of every run started

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


class SessionLog:
    def __init__(self, runs_root: Path, project_root: Path) -> None:
        started = datetime.now()
        self.directory = Path(runs_root).expanduser() / "logs" / started.strftime("%Y-%m-%d_%H-%M-%S")
        suffix = 1
        while self.directory.exists():
            suffix += 1
            self.directory = self.directory.with_name(
                started.strftime("%Y-%m-%d_%H-%M-%S") + f"_{suffix}")
        self.processes_directory = self.directory / "processes"
        self.runs_directory = self.directory / "runs"
        self.processes_directory.mkdir(parents=True)
        self.runs_directory.mkdir()
        self._lock = threading.Lock()
        self._events = open(self.directory / "events.jsonl", "a", encoding="utf-8")
        self._last_fsync = time.monotonic()
        self._metadata = {
            "started_at": now_iso(),
            "ended_at": None,
            "host": socket.gethostname(),
            "user": getpass.getuser(),
            "machine": platform.machine(),
            "kernel": platform.release(),
            "python": platform.python_version(),
            "gui_version": __version__,
            "project_root": str(project_root),
            "git_commit": _git(project_root, "rev-parse", "HEAD"),
            "git_branch": _git(project_root, "rev-parse", "--abbrev-ref", "HEAD"),
            "git_dirty": bool(_git(project_root, "status", "--porcelain", "--untracked-files=no")),
        }
        self._write_metadata()
        self.event("session", "gui", "info", "Session started", self._metadata)

    def _write_metadata(self) -> None:
        path = self.directory / "session.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self._metadata, indent=2), encoding="utf-8")
        temporary.replace(path)

    def event(self, kind: str, subsystem: str, severity: str, message: str,
              data: dict[str, Any] | None = None) -> dict[str, Any]:
        record = {
            "ts": now_iso(), "kind": kind, "subsystem": subsystem,
            "severity": severity, "message": message,
        }
        if data:
            record["data"] = data
        line = json.dumps(record, default=str)
        with self._lock:
            if self._events.closed:
                return record
            self._events.write(line + "\n")
            self._events.flush()
            durable = kind in DURABLE_KINDS or severity in {"error", "warning"}
            if durable or time.monotonic() - self._last_fsync >= FSYNC_INTERVAL_S:
                os.fsync(self._events.fileno())
                self._last_fsync = time.monotonic()
        return record

    def process_log_paths(self, name: str) -> tuple[Path, Path]:
        stem = f"{name}_{datetime.now().strftime('%H-%M-%S')}"
        log = self.processes_directory / f"{stem}.log"
        suffix = 1
        while log.exists():
            suffix += 1
            log = self.processes_directory / f"{stem}_{suffix}.log"
        return log, log.with_suffix(".status.json")

    def record_run(self, profile: dict[str, Any], commands: dict[str, list[str]]) -> Path:
        path = self.runs_directory / f"run_{datetime.now().strftime('%H-%M-%S')}.json"
        path.write_text(json.dumps({
            "started_at": now_iso(), "profile": profile, "commands": commands,
        }, indent=2), encoding="utf-8")
        return path

    def close(self) -> None:
        self.event("session", "gui", "info", "Session ended")
        with self._lock:
            os.fsync(self._events.fileno())
            self._events.close()
        self._metadata["ended_at"] = now_iso()
        self._write_metadata()
