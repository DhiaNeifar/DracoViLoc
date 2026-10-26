"""Run DracoViLoc subsystems and jobs as transient systemd user units.

Each process (YOLO container, host stack, builds, ...) runs as
``dracoviloc-<name>.service`` started over D-Bus. systemd supervises it, so:

* it keeps running, and keeps logging, if the GUI is closed or crashes; the
  GUI reattaches to running units when it starts again;
* stopping sends SIGINT (Ctrl+C) and, after ``stop_timeout`` seconds, SIGKILL
  to everything left in the unit, so no process is ever orphaned;
* the unit's main process is ``logged_exec.py``, which records the full
  terminal output in the session's processes/ folder.
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable

import dbus

from .session_log import SessionLog, now_iso

UNIT_PREFIX = "dracoviloc-"
WRAPPER = Path(__file__).resolve().with_name("logged_exec.py")
REGISTRY = Path.home() / ".local/state/dracoviloc/units"
TAIL_INTERVAL_S = 0.1
STATE_INTERVAL_S = 0.5
REATTACH_TAIL_BYTES = 256 * 1024

LinesCallback = Callable[[str, list[str]], None]
ExitCallback = Callable[[str, int], None]
ChangeCallback = Callable[[], None]


def unit_name(name: str) -> str:
    return f"{UNIT_PREFIX}{name}.service"


@dataclass
class ManagedUnit:
    name: str
    unit: str
    log: str
    status: str
    command: list[str]
    started_at: str
    state: str = "activating"
    pid: int = 0
    exit_code: int | None = None
    _offset: int = field(default=0, repr=False)

    def public(self) -> dict:
        data = asdict(self)
        data.pop("_offset")
        return data


class SystemdUser:
    """Thin wrapper over the systemd user manager's D-Bus API."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        bus = dbus.SessionBus(private=True)
        self._bus = bus
        self._manager = dbus.Interface(
            bus.get_object("org.freedesktop.systemd1", "/org/freedesktop/systemd1"),
            "org.freedesktop.systemd1.Manager")

    def start_transient(self, unit: str, argv: list[str], cwd: str,
                        environment: dict[str, str], stop_timeout_s: float,
                        description: str) -> None:
        env = [f"{key}={value}" for key, value in environment.items()
               if key and "=" not in key and "%" not in key and "\n" not in value]
        properties = dbus.Array([
            ("Description", dbus.String(description, variant_level=1)),
            ("ExecStart", dbus.Array([dbus.Struct(
                (argv[0], dbus.Array(argv, signature="s"), dbus.Boolean(False)))],
                signature="(sasb)", variant_level=1)),
            ("WorkingDirectory", dbus.String(cwd, variant_level=1)),
            ("Environment", dbus.Array(env, signature="s", variant_level=1)),
            # SIGINT to the wrapper, which forwards it to the command's process
            # group like Ctrl+C; anything left after the timeout gets SIGKILL.
            ("KillMode", dbus.String("mixed", variant_level=1)),
            ("KillSignal", dbus.Int32(2, variant_level=1)),
            ("TimeoutStopUSec", dbus.UInt64(int(stop_timeout_s * 1e6), variant_level=1)),
            ("CollectMode", dbus.String("inactive-or-failed", variant_level=1)),
        ], signature="(sv)")
        with self._lock:
            self._manager.StartTransientUnit(unit, "fail", properties, dbus.Array([], signature="(sa(sv))"))

    def stop(self, unit: str) -> None:
        with self._lock:
            try:
                self._manager.StopUnit(unit, "replace")
            except dbus.DBusException as error:
                if "NoSuchUnit" not in error.get_dbus_name():
                    raise

    def kill(self, unit: str, signum: int) -> None:
        with self._lock:
            try:
                self._manager.KillUnit(unit, "all", signum)
            except dbus.DBusException as error:
                if "NoSuchUnit" not in error.get_dbus_name():
                    raise

    def state(self, unit: str) -> tuple[str, int]:
        """Return (ActiveState, MainPID); ('inactive', 0) once collected."""
        with self._lock:
            try:
                path = self._manager.GetUnit(unit)
            except dbus.DBusException as error:
                if "NoSuchUnit" in error.get_dbus_name():
                    return "inactive", 0
                raise
            properties = dbus.Interface(
                self._bus.get_object("org.freedesktop.systemd1", path),
                "org.freedesktop.DBus.Properties")
            active = str(properties.Get("org.freedesktop.systemd1.Unit", "ActiveState"))
            try:
                pid = int(properties.Get("org.freedesktop.systemd1.Service", "MainPID"))
            except dbus.DBusException:
                pid = 0
            return active, pid

    def unit_processes(self, unit: str) -> list[int]:
        """PIDs currently in the unit's cgroup (empty once it has stopped)."""
        with self._lock:
            try:
                processes = self._manager.GetUnitProcesses(unit)
            except dbus.DBusException as error:
                if "NoSuchUnit" in error.get_dbus_name():
                    return []
                raise
        return [int(entry[1]) for entry in processes]

    def active_units(self) -> list[str]:
        with self._lock:
            units = self._manager.ListUnitsByPatterns(
                dbus.Array(["active", "activating", "deactivating", "reloading"], signature="s"),
                dbus.Array([f"{UNIT_PREFIX}*.service"], signature="s"))
        return [str(unit[0]) for unit in units]


class UnitSupervisor:
    def __init__(self, session: SessionLog, lines_callback: LinesCallback,
                 exit_callback: ExitCallback, change_callback: ChangeCallback,
                 systemd: SystemdUser | None = None, registry: Path = REGISTRY) -> None:
        self.session = session
        self.lines_callback = lines_callback
        self.exit_callback = exit_callback
        self.change_callback = change_callback
        self.systemd = systemd or SystemdUser()
        self.registry = registry
        self.registry.mkdir(parents=True, exist_ok=True)
        self._units: dict[str, ManagedUnit] = {}
        self._lock = threading.RLock()
        self._stopped = threading.Event()
        self._reattach()
        threading.Thread(target=self._tail_loop, name="unit-tail", daemon=True).start()
        threading.Thread(target=self._state_loop, name="unit-state", daemon=True).start()

    # ----------------------------------------------------------------- control
    def start(self, name: str, command: list[str], cwd: Path, env: dict[str, str],
              stop_timeout_s: float = 20.0) -> ManagedUnit:
        with self._lock:
            if self.is_running(name):
                raise RuntimeError(f"{name} is already running")
            unit = unit_name(name)
            state, _ = self.systemd.state(unit)
            if state in {"active", "activating", "deactivating"}:
                raise RuntimeError(f"{unit} is already {state} outside this session")
            log, status = self.session.process_log_paths(name)
            argv = [sys.executable, str(WRAPPER), "--log", str(log),
                    "--status", str(status), "--name", name, "--", *command]
            managed = ManagedUnit(name, unit, str(log), str(status), list(command), now_iso())
            self.systemd.start_transient(
                unit, argv, str(cwd), env, stop_timeout_s, f"DracoViLoc {name}")
            self._units[name] = managed
            self._save_registry(managed)
        self.session.event("process", name, "info", f"Started {unit}", {
            "command": command, "cwd": str(cwd), "log": str(log),
            "stop_timeout_s": stop_timeout_s})
        self.change_callback()
        return managed

    def stop(self, name: str, grace_seconds: float = 30.0) -> bool:
        """Ctrl+C the unit and wait until it has exited. Returns True if stopped."""
        with self._lock:
            managed = self._units.get(name)
        unit = managed.unit if managed else unit_name(name)
        self.session.event("process", name, "info", f"Stop requested for {unit}")
        self.systemd.stop(unit)
        deadline = time.monotonic() + grace_seconds
        while time.monotonic() < deadline:
            if self.systemd.state(unit)[0] == "inactive":
                return True
            time.sleep(0.1)
        self.session.event("process", name, "error", f"{unit} did not stop within {grace_seconds}s")
        return False

    def kill(self, name: str) -> None:
        unit = unit_name(name)
        self.session.event("process", name, "warning", f"SIGKILL sent to {unit}")
        self.systemd.kill(unit, 9)

    def is_running(self, name: str) -> bool:
        with self._lock:
            managed = self._units.get(name)
            return managed is not None and managed.state in {"activating", "active", "deactivating"}

    def running(self) -> list[str]:
        with self._lock:
            return [name for name in self._units if self.is_running(name)]

    def snapshot(self) -> list[dict]:
        with self._lock:
            return [unit.public() for unit in self._units.values()]

    def log_path(self, name: str) -> str | None:
        with self._lock:
            managed = self._units.get(name)
            return managed.log if managed else None

    def close(self) -> None:
        """Stop watching; units keep running under systemd."""
        self._stopped.set()

    # ---------------------------------------------------------------- internal
    def _save_registry(self, managed: ManagedUnit) -> None:
        path = self.registry / f"{managed.name}.json"
        path.write_text(json.dumps(managed.public(), indent=2), encoding="utf-8")

    def _reattach(self) -> None:
        for unit in self.systemd.active_units():
            name = unit[len(UNIT_PREFIX):-len(".service")]
            path = self.registry / f"{name}.json"
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                data = {"log": "", "status": "", "command": [], "started_at": ""}
            managed = ManagedUnit(name, unit, data.get("log", ""), data.get("status", ""),
                                  data.get("command", []), data.get("started_at", ""),
                                  state="active")
            try:
                managed._offset = max(0, os.path.getsize(managed.log) - REATTACH_TAIL_BYTES)
            except OSError:
                pass
            self._units[name] = managed
            self.session.event("process", name, "info",
                               f"Reattached to running {unit}", {"log": managed.log})

    def _tail_loop(self) -> None:
        while not self._stopped.is_set():
            with self._lock:
                units = list(self._units.values())
            for managed in units:
                self._read_new_lines(managed)
            time.sleep(TAIL_INTERVAL_S)

    def _read_new_lines(self, managed: ManagedUnit) -> None:
        if not managed.log:
            return
        try:
            with open(managed.log, "rb") as stream:
                stream.seek(managed._offset)
                data = stream.read()
        except OSError:
            return
        complete = data.rfind(b"\n") + 1
        if complete == 0:
            return
        managed._offset += complete
        lines = data[:complete].decode("utf-8", errors="replace").splitlines()
        if lines:
            self.lines_callback(managed.name, lines)

    def _state_loop(self) -> None:
        while not self._stopped.is_set():
            changed = False
            with self._lock:
                units = list(self._units.values())
            for managed in units:
                if managed.state in {"inactive", "failed"}:
                    continue
                try:
                    state, pid = self.systemd.state(managed.unit)
                except dbus.DBusException as error:
                    self.session.event("process", managed.name, "error",
                                       f"systemd query failed: {error}")
                    continue
                if state != managed.state or pid != managed.pid:
                    changed = True
                    managed.pid = pid
                    if state in {"inactive", "failed"}:
                        self._read_new_lines(managed)
                        managed.exit_code = self._exit_code(managed)
                        managed.state = state
                        self.session.event(
                            "process", managed.name,
                            "info" if managed.exit_code == 0 else "error",
                            f"{managed.unit} exited with code {managed.exit_code}",
                            {"log": managed.log})
                        self.exit_callback(managed.name, managed.exit_code)
                    else:
                        managed.state = state
            if changed:
                self.change_callback()
            time.sleep(STATE_INTERVAL_S)

    @staticmethod
    def _exit_code(managed: ManagedUnit) -> int:
        try:
            status = json.loads(Path(managed.status).read_text(encoding="utf-8"))
            code = status.get("exit_code")
            return int(code) if code is not None else -1
        except (OSError, ValueError, TypeError):
            return -1
