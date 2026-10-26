#!/usr/bin/env python3
"""Run one command under a pseudo-terminal and record everything it prints.

systemd starts this wrapper as the main process of each DracoViLoc unit:

    logged_exec.py --log FILE --status FILE -- command [args...]

* The command sees a real terminal, so colcon, docker and ROS print exactly
  what they print in a shell (colors, progress lines).
* Every output line is appended to the log file with a wall-clock timestamp
  prefix and flushed immediately, so nothing is lost if the GUI closes or
  crashes. The GUI tails this file for its terminal panels.
* SIGINT/SIGTERM from systemd are forwarded to the command's process group,
  exactly like Ctrl+C in a terminal, and the wrapper keeps recording until the
  command has exited.
* The status file (JSON) records the command, PIDs, start/end time and exit code.

Standard library only: it must start even if the GUI environment is broken.
"""
from __future__ import annotations

import argparse
import errno
import fcntl
import json
import os
import pty
import select
import signal
import struct
import sys
import termios
import time
from datetime import datetime

TERMINAL_COLUMNS = 200
TERMINAL_ROWS = 50
PARTIAL_FLUSH_SECONDS = 0.5
PARTIAL_FLUSH_BYTES = 4096


def timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def write_json_atomic(path: str, payload: dict) -> None:
    temporary = f"{path}.tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class LineRecorder:
    """Append timestamped lines to the log, flushing each write to disk."""

    def __init__(self, path: str) -> None:
        self._stream = open(path, "ab", buffering=0)
        self._pending = b""
        self._pending_since = 0.0

    def marker(self, text: str) -> None:
        self._write_line(f"--- {text} ---".encode())

    def feed(self, data: bytes) -> None:
        if not self._pending:
            self._pending_since = time.monotonic()
        self._pending += data
        *lines, self._pending = self._pending.split(b"\n")
        for line in lines:
            self._write_line(line)
        if self._pending and len(self._pending) >= PARTIAL_FLUSH_BYTES:
            self.flush_partial()

    def tick(self) -> None:
        if self._pending and time.monotonic() - self._pending_since >= PARTIAL_FLUSH_SECONDS:
            self.flush_partial()

    def flush_partial(self) -> None:
        if self._pending:
            self._write_line(self._pending)
            self._pending = b""

    def _write_line(self, line: bytes) -> None:
        self._stream.write(timestamp().encode() + b" " + line.rstrip(b"\r") + b"\n")

    def close(self) -> None:
        self.flush_partial()
        os.fsync(self._stream.fileno())
        self._stream.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True)
    parser.add_argument("--status", required=True)
    parser.add_argument("--name", default="")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a command is required after --")

    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    recorder = LineRecorder(args.log)
    status = {
        "name": args.name, "command": command, "cwd": os.getcwd(),
        "wrapper_pid": os.getpid(), "started_at": timestamp(),
        "state": "running", "exit_code": None, "signal": None,
        "stop_requested_at": None, "ended_at": None,
    }
    recorder.marker(f"start {args.name}: {' '.join(command)}")

    pid, master = pty.fork()
    if pid == 0:
        try:
            os.execvp(command[0], command)
        except OSError as error:
            os.write(2, f"cannot execute {command[0]}: {error}\n".encode())
        os._exit(127)

    fcntl.ioctl(master, termios.TIOCSWINSZ,
                struct.pack("HHHH", TERMINAL_ROWS, TERMINAL_COLUMNS, 0, 0))
    status["pid"] = pid
    write_json_atomic(args.status, status)

    def forward(signum, _frame):
        if status["stop_requested_at"] is None:
            status["stop_requested_at"] = timestamp()
            write_json_atomic(args.status, status)
        recorder.marker(f"received {signal.Signals(signum).name}; forwarding to the command")
        try:
            # pty.fork made the child a session leader, so its pid is its
            # process group: this matches Ctrl+C in a terminal.
            os.killpg(pid, signum)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    signal.signal(signal.SIGHUP, forward)

    while True:
        try:
            ready, _, _ = select.select([master], [], [], 0.2)
        except InterruptedError:
            continue
        if ready:
            try:
                data = os.read(master, 65536)
            except OSError as error:
                if error.errno == errno.EIO:  # every writer has closed the terminal
                    break
                raise
            if not data:
                break
            recorder.feed(data)
        recorder.tick()

    while True:
        try:
            _, wait_status = os.waitpid(pid, 0)
            break
        except InterruptedError:
            continue
    if os.WIFSIGNALED(wait_status):
        status["signal"] = signal.Signals(os.WTERMSIG(wait_status)).name
        code = 128 + os.WTERMSIG(wait_status)
    else:
        code = os.WEXITSTATUS(wait_status)
    status.update(state="exited", exit_code=code, ended_at=timestamp())
    recorder.marker(f"exit {args.name}: code {code}"
                    + (f" ({status['signal']})" if status["signal"] else ""))
    recorder.close()
    write_json_atomic(args.status, status)
    return code


if __name__ == "__main__":
    sys.exit(main())
