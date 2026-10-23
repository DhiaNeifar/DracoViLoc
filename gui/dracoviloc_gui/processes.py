from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


LogCallback = Callable[[str, str, str], None]
ExitCallback = Callable[[str, int], None]


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen[str]


class ProcessSupervisor:
    """Own child process groups and stream their output without blocking Qt."""

    def __init__(self, log_callback: LogCallback, exit_callback: ExitCallback) -> None:
        self.log_callback = log_callback
        self.exit_callback = exit_callback
        self._processes: dict[str, ManagedProcess] = {}
        self._lock = threading.RLock()

    def start(
        self, name: str, command: list[str], cwd: Path, env: dict[str, str]
    ) -> None:
        with self._lock:
            if self.is_running(name):
                raise RuntimeError(f"{name} is already running")
            process = subprocess.Popen(
                command, cwd=cwd, env=env, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                bufsize=1, start_new_session=True)
            self._processes[name] = ManagedProcess(name, process)
        threading.Thread(target=self._read_output, args=(name,), daemon=True).start()
        threading.Thread(target=self._wait, args=(name,), daemon=True).start()

    def _read_output(self, name: str) -> None:
        with self._lock:
            managed = self._processes.get(name)
        if managed is None or managed.process.stdout is None:
            return
        for line in managed.process.stdout:
            message = line.rstrip()
            severity = "error" if "error" in message.lower() or "fatal" in message.lower() else "info"
            self.log_callback(name, severity, message)

    def _wait(self, name: str) -> None:
        with self._lock:
            managed = self._processes.get(name)
        if managed is None:
            return
        code = managed.process.wait()
        self.exit_callback(name, code)

    def is_running(self, name: str) -> bool:
        managed = self._processes.get(name)
        return managed is not None and managed.process.poll() is None

    def stop(self, name: str, grace_seconds: float = 10.0) -> None:
        with self._lock:
            managed = self._processes.get(name)
        if managed is None or managed.process.poll() is not None:
            return
        process = managed.process
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + grace_seconds
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    def stop_all(self) -> None:
        # Stop host first so recorders receive SIGINT and finalize before YOLO.
        self.stop("host")
        if self.is_running("yolo"):
            try:
                subprocess.run(
                    ["docker", "stop", "--time", "10", "isaac_yolo"],
                    capture_output=True, text=True, timeout=15.0)
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.stop("yolo", grace_seconds=2.0)

    def running(self) -> list[str]:
        with self._lock:
            return [name for name in self._processes if self.is_running(name)]

