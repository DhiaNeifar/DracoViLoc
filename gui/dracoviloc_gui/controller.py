from __future__ import annotations

import functools
import json
import shlex
import re
import threading
import time
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QObject, Property, Qt, QTimer, Signal, Slot
from PySide6.QtGui import QDesktopServices
from PySide6.QtCore import QUrl

from . import docker_control, jobs, process_control
from .commands import runtime_environment, sourced_host_command, yolo_docker_command
from .models import AppState, LogEvent, RunProfile
from .preflight import blocking_failures, run_preflight
from .profiles import ProfileStore
from .presets import built_in_profiles
from .recordings import discover_recordings, drive_sync_status, trigger_drive_sync
from .remote_client import RemoteJetsonClient
from .ros_bridge import RosBridge
from .session_log import SessionLog
from .terminal import TerminalHub, TerminalModel, severity_of, split_line, strip_ansi
from .units import UnitSupervisor
from .validation import validate_profile
from .yolo_models import Registry


def audited(slot):
    """Record every operator action (QML -> controller call) before running it."""
    @functools.wraps(slot)
    def wrapper(self, *args):
        self.session.event("action", "operator", "info", slot.__name__,
                           {"args": list(args)} if args else None)
        return slot(self, *args)
    return wrapper


class AppController(QObject):
    stateChanged = Signal()
    profileChanged = Signal()
    profilesChanged = Signal()
    messageChanged = Signal()
    statusChanged = Signal(str)
    preflightFinished = Signal(str, bool)
    logAdded = Signal(str)
    recordingsChanged = Signal(str)
    cameraFrame = Signal(object)
    telemetryUpdated = Signal(str)
    quitReady = Signal()
    remoteChanged = Signal()
    processesChanged = Signal()
    setupChanged = Signal()
    processListChanged = Signal()
    robotCheckChanged = Signal()
    _jobQueueReady = Signal()

    def __init__(self, project_root: Path) -> None:
        super().__init__()
        self.project_root = project_root
        self._state = AppState.IDLE
        self._message = "Select a profile and run preflight."
        self._profiles = ProfileStore().load()
        self._profile = self._profiles["Mock Observation"]
        launch_source = project_root / "src/dracoviloc_bringup/launch/arm_demo.launch.py"
        self._launch_argument_names = sorted(set(re.findall(
            r'DeclareLaunchArgument\(\s*["\']([^"\']+)["\']',
            launch_source.read_text(encoding="utf-8"))))
        self._status: dict = {}
        self._remote_state: dict = {"state": "disconnected", "has_control": False}
        self._shutting_down_for_quit = False
        self.session = SessionLog(project_root / "logs", project_root)
        self._last_logged_message = ""
        self.messageChanged.connect(self._log_message)
        self.terminals = TerminalHub(self)
        self.terminals.terminalsChanged.connect(self.processesChanged)
        self.processes = UnitSupervisor(
            self.session, self._process_lines, self._process_exited, self.processesChanged.emit)
        self.processesChanged.connect(self.stateChanged)
        for name in self.processes.running():
            self.terminals.model(name)
        if self.processes.is_running("host"):
            self._state = AppState.READY
            self._message = ("Reattached to a run that was already active. "
                             "Motion state is unknown: use STOP MOTION if unsure.")
            self.session.event("state", "gui", "warning", self._message,
                               {"state": self._state.value})
        self._operator_stopped: set[str] = set()
        self._process_list: dict = {"items": [], "report": [], "scanning": False}
        self._robot_check: dict = {}
        self._setup: dict[str, dict] = {}
        self._models: dict = {"models": [], "gpu": "", "auto": "", "auto_error": ""}
        self._job_queue: list[str] = []
        self._current_job = next(
            (name[len("job-"):] for name in self.processes.running() if name.startswith("job-")), "")
        self._checking_setup = False
        # Jobs are started on the GUI thread even when a worker thread decides
        # the next one should run (terminal models belong to this thread).
        self._jobQueueReady.connect(self._start_next_job, Qt.QueuedConnection)
        self.ros = RosBridge(self._ros_status, self._log, self._camera_frame,
                             self.telemetryUpdated.emit)
        self.ros.start()
        self.remote = RemoteJetsonClient(
            self._remote_connection_state, self._remote_event, self._log)

    @Property(str, notify=stateChanged)
    def state(self) -> str:
        return self._state.value

    @Property(str, notify=messageChanged)
    def message(self) -> str:
        return self._message

    @Property(str, notify=profileChanged)
    def profileJson(self) -> str:
        return json.dumps(self._profile.to_dict())

    @Property("QStringList", notify=profilesChanged)
    def profileNames(self) -> list[str]:
        return list(self._profiles)

    @Property("QStringList", constant=True)
    def launchArgumentNames(self) -> list[str]:
        return self._launch_argument_names

    @Property(str, notify=remoteChanged)
    def remoteJson(self) -> str:
        return json.dumps(self._remote_state)

    @Property(bool, notify=remoteChanged)
    def remoteControllable(self) -> bool:
        return bool(self.remote.connected and self.remote.has_control)

    @Property(str, notify=processesChanged)
    def processesJson(self) -> str:
        units = {unit["name"]: unit for unit in self.processes.snapshot()}
        for name in self.terminals.names():
            if self.terminals.model(name).rowCount():
                units.setdefault(name, {"name": name, "state": "inactive"})
        return json.dumps(list(units.values()))

    @Property(str, constant=True)
    def sessionLogDirectory(self) -> str:
        return str(self.session.directory)

    @Property(str, notify=stateChanged)
    def runDirectory(self) -> str:
        return str(self.session.run_directory or "")

    @Slot(str, result=QObject)
    def terminalModel(self, name: str) -> TerminalModel:
        return self.terminals.model(name)

    @Property(str, notify=setupChanged)
    def setupJson(self) -> str:
        return json.dumps({
            "jobs": [self._setup.get(job.id, jobs.describe(job, jobs.JobStatus(jobs.UNKNOWN, "Not checked yet")))
                     for job in jobs.JOBS],
            "current": self._current_job, "queue": self._job_queue,
            "checking": self._checking_setup,
            "images_directory": str(self.project_root / "images"),
            "saved_images": sorted(path.name for path in (self.project_root / "images").glob("*.tar.gz")),
            **self._models,
        })

    @Property(bool, notify=setupChanged)
    def jobActive(self) -> bool:
        return bool(self._current_job)

    TOOL_UNITS = {"rviz"}  # helpers that may run with or without a run

    def _run_processes(self) -> list[str]:
        """Running subsystem units; setup jobs and tools are not part of a run."""
        return [name for name in self.processes.running()
                if not name.startswith("job-") and name not in self.TOOL_UNITS]

    @Property(bool, notify=stateChanged)
    def runActive(self) -> bool:
        if self._profile.execution_target == "remote":
            return bool(self._status.get("run_active"))
        return bool(self._run_processes()) or self._state not in {
            AppState.IDLE, AppState.FAULT,
        }

    def _log_message(self) -> None:
        # Messages that arrive with a state change are already in that event.
        if self._message != self._last_logged_message:
            self._last_logged_message = self._message
            self.session.event("message", "gui", "info", self._message)

    def _set_state(self, state: AppState, message: str | None = None) -> None:
        previous = self._state
        self._state = state
        if message is not None:
            self._last_logged_message = message
        self.session.event(
            "state", "gui", "error" if state == AppState.FAULT else "info",
            message or state.value, {"from": previous.value, "to": state.value})
        self.stateChanged.emit()
        if message is not None:
            self._message = message
            self.messageChanged.emit()

    def _log(self, subsystem: str, severity: str, message: str) -> None:
        self.session.event("log", subsystem, severity, message)
        event = LogEvent(datetime.now().isoformat(timespec="seconds"), subsystem, severity, message)
        self.logAdded.emit(json.dumps(event.to_dict()))

    def _process_lines(self, name: str, lines: list[str]) -> None:
        """Called from the tail thread; the full output is already on disk."""
        self.terminals.linesArrived.emit(name, lines)
        for line in lines:
            text = strip_ansi(split_line(line)[1])
            severity = severity_of(text)
            if severity != "info":
                event = LogEvent(datetime.now().isoformat(timespec="seconds"), name, severity, text)
                self.logAdded.emit(json.dumps(event.to_dict()))

    def _process_exited(self, name: str, code: int) -> None:
        if name.startswith("job-"):
            self._job_finished(name[len("job-"):], code)
        event = LogEvent(datetime.now().isoformat(timespec="seconds"), name,
                         "info" if code == 0 else "error", f"Process exited with code {code}")
        self.logAdded.emit(json.dumps(event.to_dict()))
        if name == "host" and self._state not in {AppState.SHUTTING_DOWN, AppState.IDLE}:
            reason = ("The host stack was stopped from the Processes page."
                      if name in self._operator_stopped else "The host launch exited unexpectedly.")
            self._set_state(AppState.FAULT, reason)
        self._operator_stopped.discard(name)
        if not self._run_processes() and self.session.run_directory is not None and \
                self._state not in {AppState.SHUTTING_DOWN, AppState.STARTING_YOLO, AppState.STARTING_HOST}:
            self.session.end_run(f"{name} exited (code {code})")

    def _ros_status(self, status: dict) -> None:
        self._status = status
        self.statusChanged.emit(json.dumps(status))
        if self._state == AppState.HOMING and status.get("at_home"):
            self._set_state(AppState.HOME_READY, "Home reached. Tracking may now be armed.")

    def _camera_frame(self, frame: object) -> None:
        self.cameraFrame.emit(frame)

    def _remote_connection_state(self, state: dict) -> None:
        self._remote_state.update(state)
        self.remoteChanged.emit()
        if state.get("state") == "connected":
            mode = "controller" if state.get("has_control") else "read-only"
            self._message = f"Connected to {state.get('alias', 'Jetson')} ({mode})."
        elif state.get("state") == "error":
            self._message = f"Jetson connection failed: {state.get('error')}"
        elif state.get("state") == "disconnected" and state.get("error"):
            self._message = str(state["error"])
        else:
            self._message = f"Jetson: {state.get('state', 'disconnected')}"
        self.messageChanged.emit()

    def _remote_event(self, event: str, data: dict, body: bytes) -> None:
        if event == "status":
            self._ros_status(data)
            if data.get("tracking"):
                self._set_state(AppState.TRACKING)
            elif data.get("run_active") and self._state not in {AppState.HOMING, AppState.STOPPING_MOTION}:
                self._set_state(AppState.HOME_READY if data.get("at_home") else AppState.READY)
            elif not data.get("run_active") and self._state not in {AppState.PREFLIGHT, AppState.STARTING_HOST}:
                self._set_state(AppState.IDLE)
            self.stateChanged.emit()
        elif event == "log":
            self._log(data.get("subsystem", "jetson"), data.get("severity", "info"),
                      data.get("message", ""))
        elif event == "camera":
            frame = dict(data)
            frame["data"] = body
            frame["stream"] = "overlay"  # the agent streams the annotated image
            self.cameraFrame.emit(frame)

    def _is_remote(self) -> bool:
        return self._profile.execution_target == "remote"

    def _estop_instruction(self) -> str:
        return ("Engage both physical emergency stops before forcing shutdown."
                if self._profile.dual_arm else
                "Engage the physical emergency stop before forcing shutdown.")

    def _remote_call(self, command: str, args: dict | None, callback=None) -> None:
        def complete(ok: bool, result) -> None:
            if ok:
                message = result.get("message", "Request complete") if isinstance(result, dict) else "Request complete"
                self._log("remote", "info", message)
            else:
                message = str(result)
                self._log("remote", "error", message)
            if callback:
                callback(ok, result)
            elif not ok:
                self._set_state(AppState.FAULT, message)
        self.remote.call(command, args or {}, complete)

    @Slot()
    @audited
    def connectRemote(self) -> None:
        self.remote.connect(self._profile.ssh_alias, self._profile.remote_agent_port)

    @Slot()
    @audited
    def disconnectRemote(self) -> None:
        self.remote.disconnect(release=True)
        self._remote_state = {"state": "disconnected", "has_control": False}
        self.remoteChanged.emit()

    @Slot()
    @audited
    def acquireRemoteControl(self) -> None:
        self.remote.acquire_control(lambda ok, result: self._log(
            "remote", "info" if ok else "error",
            "Control acquired" if ok else str(result)))

    @Slot()
    @audited
    def deployRemoteAgent(self) -> None:
        if self.runActive:
            self._message = "Stop the active run before updating the Jetson agent."
            self.messageChanged.emit()
            return
        config = {
            "project_root": self._profile.jetson_project_root,
            "isaac_root": self._profile.jetson_isaac_root,
            "recordings_root": self._profile.remote_recordings_root,
            "listen_host": "127.0.0.1", "listen_port": self._profile.remote_agent_port,
            "yolo_image": "auto",
            "camera_topic": "/yolov8_processed_image", "camera_fps": 10.0,
        }
        self._message = "Deploying the Jetson agent over SSH…"
        self.messageChanged.emit()
        source = self.project_root / "remote_agent"
        def complete(ok: bool, message: str) -> None:
            self._log("deploy", "info" if ok else "error", message)
            self._message = message
            self.messageChanged.emit()
            if ok:
                self.connectRemote()
        self.remote.deploy(self._profile.ssh_alias, source, config, complete)

    @Slot(str)
    @audited
    def selectProfile(self, name: str) -> None:
        if self.runActive or name not in self._profiles:
            return
        self._profile = RunProfile.from_dict(self._profiles[name].to_dict())
        self.profileChanged.emit()
        self._message = f"{name} selected."
        self.messageChanged.emit()

    @Slot(str)
    @audited
    def saveProfile(self, name: str) -> None:
        name = name.strip()
        if self.runActive or not name:
            return
        saved = RunProfile.from_dict(self._profile.to_dict())
        saved.name = name
        self._profiles[name] = saved
        built_ins = set(built_in_profiles())
        ProfileStore().save_custom([
            profile for profile_name, profile in self._profiles.items()
            if profile_name not in built_ins
        ])
        self._profile = RunProfile.from_dict(saved.to_dict())
        self.profilesChanged.emit()
        self.profileChanged.emit()
        self._message = f"Profile saved as {name}."
        self.messageChanged.emit()

    @Slot(str, "QVariant")
    def setOption(self, name: str, value) -> None:
        if self.runActive or name not in RunProfile.__dataclass_fields__:
            return
        current = getattr(self._profile, name)
        try:
            if isinstance(current, bool):
                converted = bool(value)
            elif isinstance(current, int):
                converted = int(value)
            elif isinstance(current, float):
                converted = float(value)
            else:
                converted = str(value)
            if name == "recording_enabled" and not converted and self._profile.yolo_enabled:
                self._message = "Raw camera recording is always on while the camera is enabled."
                self.messageChanged.emit()
                self.profileChanged.emit()
                return
            previous = getattr(self._profile, name)
            setattr(self._profile, name, converted)
            self._profile.apply_rules()
            self.session.event("action", "operator", "info", f"Setting changed: {name}",
                               {"name": name, "from": previous, "to": converted})
            self.profileChanged.emit()
        except (TypeError, ValueError):
            self._message = f"Invalid value for {name}."
            self.messageChanged.emit()

    @Slot(str)
    @audited
    def removeExpertOption(self, name: str) -> None:
        if self.runActive or name not in self._profile.expert_arguments:
            return
        del self._profile.expert_arguments[name]
        self.profileChanged.emit()
        self._set_message(f"Removed advanced setting {name}.")

    @Slot(str, str)
    @audited
    def setExpertOption(self, name: str, value: str) -> None:
        if self.runActive or name not in self._launch_argument_names:
            self._message = "Choose a valid declared launch argument."
            self.messageChanged.emit()
            return
        self._profile.expert_arguments[name] = value
        self.profileChanged.emit()
        self._message = f"Expert override applied: {name}:={value}"
        self.messageChanged.emit()

    @Slot()
    @audited
    def runPreflight(self) -> None:
        if self.runActive:
            return
        self._set_state(AppState.PREFLIGHT, "Checking this run configuration…")

        if self._is_remote():
            def complete(ok: bool, result) -> None:
                if not ok:
                    self._set_state(AppState.FAULT, str(result))
                    return
                failures = [row for row in result if row.get("blocking") and not row.get("ok")]
                self.preflightFinished.emit(json.dumps(result), not failures)
                self._set_state(AppState.FAULT if failures else AppState.IDLE,
                                f"Preflight found {len(failures)} blocking issue(s)." if failures
                                else "Jetson preflight passed. Ready to start.")
            self._remote_call("preflight", {"profile": self._profile.to_dict()}, complete)
            return

        def work():
            results = run_preflight(self.project_root, self._profile)
            failures = blocking_failures(results)
            self.preflightFinished.emit(
                json.dumps([result.to_dict() for result in results]), not failures)
            if failures:
                self._set_state(AppState.FAULT, f"Preflight found {len(failures)} blocking issue(s).")
            else:
                self._set_state(AppState.IDLE, "Preflight passed. Ready to start.")
        threading.Thread(target=work, daemon=True).start()

    @Slot()
    @audited
    def startRun(self) -> None:
        if self.runActive:
            return
        if self._current_job:
            self._set_message(f"Wait for the '{self._current_job}' setup job to finish first.")
            return
        errors = validate_profile(self._profile)
        if errors:
            self._set_state(AppState.FAULT, " ".join(errors))
            return
        if self._is_remote():
            if not self.remote.has_control:
                self._set_state(AppState.FAULT, "Connect to the Jetson and acquire control first.")
                return
            self._set_state(AppState.PREFLIGHT, "Running Jetson preflight…")
            def preflight_done(ok: bool, result) -> None:
                if not ok:
                    self._set_state(AppState.FAULT, str(result))
                    return
                failures = [row for row in result if row.get("blocking") and not row.get("ok")]
                self.preflightFinished.emit(json.dumps(result), not failures)
                if failures:
                    self._set_state(AppState.FAULT, "Start blocked by Jetson preflight.")
                    return
                self._set_state(AppState.STARTING_HOST, "Starting the run on Jetson…")
                self._remote_call("start", {"profile": self._profile.to_dict()},
                                  lambda started, response: self._set_state(
                                      AppState.READY if started else AppState.FAULT,
                                      response.get("message", "Remote run started") if started else str(response)))
            self._remote_call("preflight", {"profile": self._profile.to_dict()}, preflight_done)
            return
        self._set_state(AppState.PREFLIGHT, "Validating devices and runtime dependencies…")
        threading.Thread(target=self._preflight_and_start, daemon=True).start()

    def _preflight_and_start(self) -> None:
        results = run_preflight(self.project_root, self._profile)
        failures = blocking_failures(results)
        self.preflightFinished.emit(json.dumps([item.to_dict() for item in results]), not failures)
        if failures:
            self._set_state(AppState.FAULT, "Start blocked by preflight.")
            return
        self._start_pipeline()

    def _start_pipeline(self) -> None:
        env = runtime_environment()
        model = None
        if self._profile.yolo_enabled:
            try:
                registry = Registry(self.project_root)
                model = registry.resolve(self._profile.yolo_model)
            except LookupError as exc:
                self._set_state(AppState.FAULT, str(exc))
                return
        run_directory = self.session.begin_run(Path(self._profile.recording_root).expanduser())
        commands = {"host": sourced_host_command(self.project_root, self._profile, run_directory)}
        if model is not None:
            paths = registry.container_paths(model.id, model.legacy)
            self._log("yolo", "info", f"YOLO model: {model.id} ({model.plan_detail})")
            commands["yolo"] = yolo_docker_command(self.project_root, self._profile, paths)
        self.session.record_run(self._profile.to_dict(), commands,
                                {"yolo_model": model.id if model else None})
        self.session.event("run", "gui", "info", "Run starting",
                           {"profile": self._profile.name, "run": str(run_directory)})
        try:
            if self._profile.yolo_enabled:
                self._set_state(AppState.STARTING_YOLO, "Starting the YOLO container…")
                if docker_control.remove_if_stopped():
                    self._log("yolo", "info", "Removed stopped stale isaac_yolo container")
                self.processes.start("yolo", commands["yolo"], self.project_root, env)
                deadline = time.monotonic() + 45.0
                while time.monotonic() < deadline:
                    if not self.processes.is_running("yolo"):
                        raise RuntimeError("YOLO exited before becoming ready.")
                    if self._status.get("freshness", {}).get("/image", 999.0) < 2.0:
                        break
                    time.sleep(0.25)
                else:
                    self.processes.stop("yolo")
                    raise RuntimeError("YOLO did not publish /image within 45 seconds.")
            self._set_state(AppState.STARTING_HOST, "Starting the DracoViLoc host stack…")
            self.processes.start("host", commands["host"], self.project_root, env)
            self._set_state(AppState.READY, "Run active. Motion remains disabled.")
        except Exception as exc:
            self._log("supervisor", "error", str(exc))
            self._set_state(AppState.FAULT, str(exc))
            if not self._run_processes():
                self.session.end_run(f"failed to start: {exc}")

    @Slot()
    @audited
    def requestHome(self) -> None:
        if self._state not in {AppState.READY, AppState.HOME_READY}:
            return
        self._set_state(AppState.HOMING, "Home trajectory requested…")

        if self._is_remote():
            self._remote_call("home", {}, lambda ok, result: self._set_state(
                AppState.HOMING if ok else AppState.READY,
                result.get("message", "Home requested") if ok else str(result)))
            return

        def complete(ok: bool, message: str):
            self._log("motion", "info" if ok else "error", message)
            if not ok:
                self._set_state(AppState.READY, message)
            else:
                self._message = message
                self.messageChanged.emit()
        self.ros.request_home(complete)

    @Slot()
    @audited
    def enableTracking(self) -> None:
        if self._state not in {AppState.HOME_READY, AppState.READY}:
            return

        if self._is_remote():
            self._remote_call("set_tracking", {"enabled": True}, lambda ok, result: self._set_state(
                AppState.TRACKING if ok else AppState.READY,
                result.get("message", "Tracking enabled") if ok else str(result)))
            return

        def complete(ok: bool, message: str):
            self._log("motion", "info" if ok else "error", message)
            self._set_state(AppState.TRACKING if ok else AppState.READY, message)
        self.ros.set_tracking(True, complete)

    @Slot()
    @audited
    def stopMotion(self) -> None:
        if self._state in {AppState.IDLE, AppState.PREFLIGHT, AppState.SHUTTING_DOWN}:
            return
        self._set_state(AppState.STOPPING_MOTION, "Requesting a controlled motion stop…")

        if self._is_remote():
            self._remote_call("set_tracking", {"enabled": False}, lambda ok, result: self._set_state(
                AppState.READY if ok else AppState.FAULT,
                result.get("message", "Motion stopped") if ok else str(result)))
            return

        def complete(ok: bool, message: str):
            self._log("motion", "info" if ok else "error", message)
            if not ok:
                self._set_state(AppState.FAULT, message)
                return
            threading.Thread(target=self._complete_motion_stop, daemon=True).start()
        self.ros.set_tracking(False, complete)

    def _trajectory_controller_active(self) -> bool:
        controllers = self._status.get("controllers", {})
        expected = "dual_arm_controller" if self._profile.dual_arm else "arm_controller"
        tracking = "dual_arm_tracking_controller" if self._profile.dual_arm else "arm_tracking_controller"
        return controllers.get(expected) == "active" and controllers.get(tracking) != "active"

    def _wait_for_trajectory_controller(self, timeout: float = 15.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._trajectory_controller_active():
                return True
            time.sleep(0.2)
        return False

    def _complete_motion_stop(self) -> None:
        if self._wait_for_trajectory_controller():
            self._set_state(AppState.READY, "Motion stopped; trajectory controller is active.")
        else:
            self._set_state(
                AppState.FAULT,
                "Motion stop was requested, but controller restoration was not confirmed.")

    @Slot()
    @audited
    def stopAll(self) -> None:
        if self._state == AppState.SHUTTING_DOWN:
            return
        self._set_state(AppState.SHUTTING_DOWN, "Stopping motion and finalizing the run…")

        if self._is_remote():
            self._remote_call("stop_all", {}, lambda ok, result: self._set_state(
                AppState.IDLE if ok else AppState.FAULT,
                result.get("message", "Remote run stopped") if ok else str(result)))
            return

        def after_motion(ok: bool, message: str):
            self._log("motion", "info" if ok else "error", message)
            if not ok and self._profile.hardware_mode == "real":
                self._set_state(
                    AppState.FAULT,
                    "Safe hold could not be confirmed. " + self._estop_instruction())
                return
            def verify_and_finish():
                if self._wait_for_trajectory_controller():
                    self._finish_shutdown()
                elif self._profile.hardware_mode == "real":
                    self._set_state(
                        AppState.FAULT,
                        "Trajectory controller did not return. " + self._estop_instruction())
                else:
                    self._finish_shutdown()
            threading.Thread(target=verify_and_finish, daemon=True).start()
        if self._status.get("tracking_service"):
            self.ros.set_tracking(False, after_motion)
        elif self._profile.hardware_mode == "real" and self.processes.is_running("host"):
            self._set_state(
                AppState.FAULT,
                "Tracking service is unavailable. " + self._estop_instruction())
        else:
            threading.Thread(target=self._finish_shutdown, daemon=True).start()

    @Slot()
    @audited
    def forceStopAfterEstop(self) -> None:
        """Operator attests that the physical E-stop has made termination safe."""
        self._set_state(AppState.SHUTTING_DOWN, "Physical E-stop confirmed; stopping processes…")
        if self._is_remote():
            self._remote_call("force_stop_all", {}, lambda ok, result: self._set_state(
                AppState.IDLE if ok else AppState.FAULT,
                result.get("message", "Remote run stopped") if ok else str(result)))
            return
        threading.Thread(target=self._finish_shutdown, daemon=True).start()

    def _stop_all_processes(self) -> bool:
        # Host first, so the recorders receive Ctrl+C and finalize their files
        # while the camera is still publishing.
        stopped = True
        for name in ["host", *[n for n in self._run_processes() if n not in {"host", "yolo"}], "yolo"]:
            if self.processes.is_running(name):
                stopped = self.processes.stop(name) and stopped
        try:
            if docker_control.stop():
                self._log("yolo", "warning", "isaac_yolo was still running after its unit stopped; stopped it")
        except docker_control.DockerException as exc:
            self._log("yolo", "error", f"Could not verify the YOLO container: {exc}")
            stopped = False
        return stopped

    def _sweep_leftovers(self) -> list[process_control.StopReport]:
        """After a run is stopped, stop anything DracoViLoc-related that is
        still running (strays started from a terminal, containers, RViz), so
        nothing has to be killed by hand. Setup jobs are left alone."""
        reports = []
        if self.processes.is_running("rviz"):
            self._operator_stopped.add("rviz")
            stopped = self.processes.stop("rviz")
            reports.append(process_control.StopReport(
                "rviz", stopped, ["Ctrl+C"], [] if stopped else ["rviz did not stop"]))
        try:
            items = process_control.scan(self.project_root, self.processes.snapshot(),
                                         self.processes.systemd.unit_processes)
        except Exception as exc:
            self._log("processes", "error", f"Leftover scan failed: {exc}")
            return reports
        for item in items:
            if item.kind == "process":
                reports.append(process_control.stop_processes([item.pid], f"{item.name} ({item.pid})"))
            elif item.kind == "container" and not item.name.startswith("dracoviloc_"):
                reports.append(process_control.stop_container(item.name))
        for report in reports:
            self.session.event("process", report.target, "info" if report.stopped else "error",
                               "Leftover after Stop all: " + report.summary(), report.to_dict())
            self._log("processes", "info" if report.stopped else "error", report.summary())
        return reports

    def _finish_shutdown(self) -> None:
        stopped = self._stop_all_processes()
        leftovers = self._sweep_leftovers()
        stuck = [report for report in leftovers if not report.stopped]
        if stopped and not stuck:
            extra = f" Also stopped {len(leftovers)} leftover item(s)." if leftovers else ""
            self._set_state(AppState.IDLE, "All processes stopped." + extra)
            self.session.end_run("stopped")
        else:
            details = "; ".join(report.summary() for report in stuck)
            self._set_state(AppState.FAULT, "Some processes did not stop. " + (details or "See Logs."))
            self.session.end_run("stopped with processes still running")
        if self._shutting_down_for_quit:
            self._quit()

    def _quit(self) -> None:
        self.ros.stop()
        self.processes.close()
        self.session.close()
        self.quitReady.emit()

    @Slot()
    @audited
    def requestQuit(self) -> None:
        if self._is_remote():
            # Closing the dashboard intentionally does not alter the Jetson run.
            self.remote.disconnect(release=True)
            self._quit()
            return
        if self._run_processes():
            self._shutting_down_for_quit = True
            self.stopAll()
        else:
            self._quit()

    # ------------------------------------------------------------ setup & builds
    def _set_message(self, message: str) -> None:
        self._message = message
        self.messageChanged.emit()

    @Slot()
    @audited
    def refreshSetup(self) -> None:
        self._refresh_setup()

    def _refresh_setup(self) -> None:
        if self._checking_setup:
            return
        self._checking_setup = True
        self.setupChanged.emit()

        def work():
            statuses = jobs.check_all(self.project_root)
            self._setup = {job.id: jobs.describe(job, statuses[job.id]) for job in jobs.JOBS}
            self._models = self._scan_models()
            self._checking_setup = False
            self.session.event("setup", "setup", "info", "Setup status checked",
                               {job_id: status.state for job_id, status in statuses.items()})
            self.setupChanged.emit()
            if self._job_queue and not self._current_job:
                self._jobQueueReady.emit()
        threading.Thread(target=work, daemon=True).start()

    def _scan_models(self) -> dict:
        try:
            registry = Registry(self.project_root)
            models = registry.list()
        except Exception as exc:
            return {"models": [], "gpu": "", "auto": "", "auto_error": f"Model scan failed: {exc}"}
        try:
            auto, auto_error = registry.resolve("auto", models).id, ""
        except LookupError as exc:
            auto, auto_error = "", str(exc)
        return {"models": [model.to_dict() for model in models], "gpu": registry.gpu,
                "auto": auto, "auto_error": auto_error}

    @Slot(str)
    @audited
    def exportModel(self, model_id: str) -> None:
        self._job_queue = []
        self._start_job("yolo_export", f"ONNX export of {model_id}",
                        jobs.yolo_models_command("export", model_id), 60.0)

    @Slot(str)
    @audited
    def buildModelPlan(self, model_id: str) -> None:
        self._job_queue = []
        self._start_job("yolo_plan", f"TensorRT plan for {model_id}",
                        jobs.yolo_models_command("build-plan", model_id), 60.0)

    def _start_job(self, job_id: str, title: str, command: list[str],
                   stop_timeout_s: float = 30.0) -> bool:
        if self.runActive:
            self._set_message("Stop the active run before running setup jobs.")
            return False
        if self._current_job:
            self._set_message(f"'{self._current_job}' is still running.")
            return False
        env = runtime_environment()
        env["BUILDKIT_PROGRESS"] = "plain"
        try:
            self.processes.start(f"job-{job_id}", command, self.project_root, env, stop_timeout_s)
        except Exception as exc:
            self._log("setup", "error", f"{title}: {exc}")
            self._set_message(f"{title} could not start: {exc}")
            return False
        self._current_job = job_id
        self._current_job_title = title
        self.terminals.model(f"job-{job_id}")
        self._set_message(f"{title} running… (output below and in Logs)")
        self.setupChanged.emit()
        return True

    @Slot(str)
    @audited
    def runJob(self, job_id: str) -> None:
        job = jobs.JOBS_BY_ID.get(job_id)
        if job is None:
            return
        self._job_queue = []
        self._start_job(job.id, job.title, job.command(self.project_root), job.stop_timeout_s)

    @Slot()
    @audited
    def updateAll(self) -> None:
        """Run every job whose status is not OK, in dependency order."""
        if self._current_job or self.runActive:
            self._set_message("Wait for the active run or job to finish first.")
            return
        if not any(self._needs_run(job.id) for job in jobs.JOBS if job.in_update_all):
            self._set_message("Everything is up to date (run 'Check again' to re-scan).")
            return
        # Every step stays queued in dependency order; each one is re-checked
        # right before it would run, because earlier steps can make later ones
        # stale (e.g. syncing sources makes the container build outdated).
        self._job_queue = [job.id for job in jobs.JOBS if job.in_update_all]
        self.session.event("setup", "setup", "info", "Update started", {"steps": self._job_queue})
        self._start_next_job()

    def _needs_run(self, job_id: str) -> bool:
        return self._setup.get(job_id, {}).get("state") in {jobs.STALE, jobs.MISSING}

    def _start_next_job(self) -> None:
        while self._job_queue:
            job = jobs.JOBS_BY_ID[self._job_queue.pop(0)]
            if not self._needs_run(job.id):
                continue
            if self._start_job(job.id, job.title, job.command(self.project_root), job.stop_timeout_s):
                self.setupChanged.emit()
                return
            self._job_queue = []
        self.setupChanged.emit()

    def _job_finished(self, job_id: str, code: int) -> None:
        title = getattr(self, "_current_job_title", "") or job_id
        self._current_job = ""
        if code == 0:
            self._set_message(f"{title} finished successfully.")
        else:
            if self._job_queue:
                self.session.event("setup", "setup", "error", "Update stopped",
                                   {"failed": job_id, "skipped": self._job_queue})
            self._job_queue = []
            self._set_message(f"{title} failed (exit code {code}). See its output below.")
        # Re-check before the next queued job: later jobs depend on earlier ones.
        self.setupChanged.emit()
        self._refresh_setup()

    @Slot()
    @audited
    def cancelJob(self) -> None:
        if not self._current_job:
            return
        self._job_queue = []
        name = f"job-{self._current_job}"
        threading.Thread(target=lambda: self.processes.stop(name), daemon=True).start()

    @Slot()
    @audited
    def saveProjectImage(self) -> None:
        versioned, latest = jobs.project_image_tags(self.project_root)
        directory = self.project_root / "images"
        directory.mkdir(exist_ok=True)
        target = directory / f"{versioned.replace('/', '_').replace(':', '_')}.tar.gz"
        part = shlex.quote(f"{target}.part")
        final = shlex.quote(str(target))
        script = (f"set -euo pipefail; docker save {versioned} {latest} | gzip -1 > {part}"
                  f" && mv {part} {final} && ls -lh {final}")
        self._job_queue = []
        self._start_job("image_save", "Save project image", ["bash", "-c", script], 120.0)

    @Slot(str)
    @audited
    def loadProjectImage(self, file_name: str) -> None:
        path = self.project_root / "images" / Path(file_name).name
        if not path.is_file():
            self._set_message(f"{path} not found.")
            return
        self._job_queue = []
        self._start_job("image_load", "Load project image",
                        ["docker", "load", "-i", str(path)], 120.0)

    # ------------------------------------------------------------- processes
    @Property(str, notify=processListChanged)
    def processListJson(self) -> str:
        return json.dumps(self._process_list)

    @Slot()
    def refreshProcesses(self) -> None:
        """Not audited: the Processes page refreshes this every few seconds."""
        if self._process_list.get("scanning"):
            return
        self._process_list["scanning"] = True

        def work():
            started = time.monotonic()
            try:
                items = process_control.scan(
                    self.project_root, self.processes.snapshot(),
                    self.processes.systemd.unit_processes)
                restartable = {name for name, unit in self._restart_commands().items() if unit}
                rows = []
                for item in items:
                    row = item.to_dict()
                    row["restartable"] = item.kind == "unit" and item.name in restartable
                    rows.append(row)
                # Stopped units that can be started again (e.g. YOLO, RViz).
                running = {item.name for item in items if item.kind == "unit"}
                for name in sorted(restartable - running):
                    rows.append({"id": f"unit:{name}", "kind": "unit", "name": name, "state": "stopped",
                                 "managed": True, "restartable": True, "children": [], "pid": 0,
                                 "command": "", "cpu_percent": 0, "memory_mb": 0, "started": 0, "unit": ""})
                self._process_list["items"] = rows
            except Exception as exc:
                self._process_list["report"] = [f"Scan failed: {exc}"]
            finally:
                self._process_list["scanning"] = False
                self._process_list["scan_ms"] = round((time.monotonic() - started) * 1000)
                self.processListChanged.emit()
        threading.Thread(target=work, daemon=True).start()

    RESTARTABLE = {"yolo", "rviz"}  # the host stack restarts as a new run

    def _restart_commands(self) -> dict[str, list[str]]:
        return {unit["name"]: unit.get("command") for unit in self.processes.snapshot()
                if unit["name"] in self.RESTARTABLE}

    def _report(self, reports: list[process_control.StopReport]) -> None:
        lines = [report.summary() for report in reports]
        for report in reports:
            self.session.event("process", report.target,
                               "info" if report.stopped else "error", report.summary(),
                               report.to_dict())
            self._log("processes", "info" if report.stopped else "error", report.summary())
        self._process_list["report"] = lines
        self._set_message(lines[-1] if len(lines) == 1 else
                          f"{sum(r.stopped for r in reports)}/{len(reports)} stopped; see Processes")
        self.refreshProcesses()

    @Slot(str, bool)
    @audited
    def stopProcessItem(self, item_id: str, force: bool) -> None:
        """Stop (Ctrl+C first) or kill (SIGKILL) a unit, process tree or container."""
        kind, _, key = item_id.partition(":")

        def work():
            if kind == "unit":
                self._operator_stopped.add(key)
                if force:
                    self.processes.kill(key)
                    time.sleep(1.0)
                    stopped = not self.processes.is_running(key)
                    report = process_control.StopReport(
                        key, stopped, ["SIGKILL to the whole unit"],
                        [] if stopped else [f"{key} is still running"])
                else:
                    stopped = self.processes.stop(key)
                    report = process_control.StopReport(
                        key, stopped, ["Ctrl+C, SIGKILL after the stop timeout"],
                        [] if stopped else [f"{key} did not stop"])
                if key == "yolo":
                    process_control.stop_container(docker_control.YOLO_CONTAINER)
                self._report([report])
            elif kind == "pid":
                pid = int(key)
                if force:
                    self._report([process_control.stop_processes_now([pid], f"process {pid}")])
                else:
                    self._report([process_control.stop_processes([pid], f"process {pid}")])
            elif kind == "container":
                self._report([process_control.stop_container(key)])
        threading.Thread(target=work, daemon=True).start()

    @Slot(str)
    @audited
    def restartUnit(self, name: str) -> None:
        command = self._restart_commands().get(name)
        if not command:
            self._set_message(f"No earlier command to restart {name}.")
            return

        def work():
            if self.processes.is_running(name):
                self._operator_stopped.add(name)
                self.processes.stop(name)
            if name == "yolo":
                # Killing the unit stops the docker client; the container itself
                # may still be shutting down. Wait until it is really gone.
                report = process_control.stop_container(docker_control.YOLO_CONTAINER)
                if not report.stopped:
                    self._report([report])
                    return
                docker_control.remove_if_stopped()
            try:
                self.processes.start(name, command, self.project_root, runtime_environment())
                self._set_message(f"{name} restarted.")
            except Exception as exc:
                self._set_message(f"{name} could not restart: {exc}")
            self.refreshProcesses()
        threading.Thread(target=work, daemon=True).start()

    @Slot()
    @audited
    def openRviz(self) -> None:
        if self.processes.is_running("rviz"):
            self._set_message("RViz is already open.")
            return
        script = ("source /opt/ros/humble/setup.bash && "
                  f"source {shlex.quote(str(self.project_root / 'install/setup.bash'))} && exec \"$@\"")
        command = ["/bin/bash", "-lc", script, "dracoviloc-gui", "ros2", "launch",
                   "dracoviloc_bringup", "rviz.launch.py",
                   f"dual_arm:={'true' if self._profile.dual_arm else 'false'}"]
        try:
            self.processes.start("rviz", command, self.project_root, runtime_environment(), 10.0)
            self._set_message("RViz started.")
        except Exception as exc:
            self._set_message(f"RViz could not start: {exc}")
        self.refreshProcesses()

    @Slot()
    @audited
    def cleanUpEverything(self) -> None:
        """Stop every DracoViLoc unit, stray process and container, then verify."""
        def work():
            reports = []
            for name in self.processes.running():
                self._operator_stopped.add(name)
                stopped = self.processes.stop(name)
                reports.append(process_control.StopReport(
                    name, stopped, ["Ctrl+C, SIGKILL after the stop timeout"],
                    [] if stopped else [f"{name} did not stop"]))
            items = process_control.scan(self.project_root, self.processes.snapshot(),
                                         self.processes.systemd.unit_processes)
            for item in items:
                if item.kind == "process":
                    reports.append(process_control.stop_processes([item.pid], f"{item.name} ({item.pid})"))
                elif item.kind == "container":
                    reports.append(process_control.stop_container(item.name))
            if not reports:
                reports.append(process_control.StopReport("DracoViLoc", True, ["nothing was running"], []))
            if self.session.run_directory is not None:
                self.session.end_run("cleaned up from the Processes page")
            if self._state not in {AppState.IDLE}:
                self._set_state(AppState.IDLE, "Everything was stopped from the Processes page.")
            self._report(reports)
        threading.Thread(target=work, daemon=True).start()

    # ------------------------------------------------------------ robot check
    @Property(str, notify=robotCheckChanged)
    def robotCheckJson(self) -> str:
        return json.dumps(self._robot_check)

    @Slot()
    @audited
    def checkRobots(self) -> None:
        """Ping each configured FAIRINO controller and open its XML-RPC port."""
        addresses = ({"left": self._profile.left_robot_ip, "right": self._profile.right_robot_ip}
                     if self._profile.dual_arm else {"robot": self._profile.robot_ip})
        self._robot_check = {key: {"ip": ip, "state": "checking"} for key, ip in addresses.items()}
        self.robotCheckChanged.emit()

        def work():
            for key, ip in addresses.items():
                result = process_control.check_robot(ip)
                self._robot_check[key] = {"ip": ip, **result}
                self.session.event("action", "robots", "info" if result["ok"] else "warning",
                                   f"Connection check {ip}: {result['detail']}")
                self.robotCheckChanged.emit()
        threading.Thread(target=work, daemon=True).start()

    @Slot(str, str)
    def uiEvent(self, action: str, detail: str) -> None:
        """Operator interactions handled purely in QML (navigation, dialogs)."""
        self.session.event("action", "operator", "info", action,
                           {"detail": detail} if detail else None)

    @Slot()
    @audited
    def openSessionFolder(self) -> None:
        folder = self.session.run_directory or self.session.directory
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))

    @Slot(str)
    @audited
    def openProcessLog(self, name: str) -> None:
        path = self.processes.log_path(name)
        if path:
            QDesktopServices.openUrl(QUrl.fromLocalFile(path))

    @Slot()
    @audited
    def refreshRecordings(self) -> None:
        if self._is_remote():
            self._remote_call("list_recordings", {}, lambda ok, result:
                self.recordingsChanged.emit(json.dumps({
                    "items": result if ok else [], "sync_active": False,
                    "sync_state": "Stored on Jetson" if ok else str(result),
                })))
            return
        root = Path(self._profile.recording_root).expanduser()
        recordings = [item.to_dict() for item in discover_recordings(root)]
        sync_ok, sync_state = drive_sync_status()
        self.recordingsChanged.emit(json.dumps({
            "items": recordings, "sync_active": sync_ok, "sync_state": sync_state,
        }))

    @Slot()
    @audited
    def syncNow(self) -> None:
        if self._is_remote():
            self._message = "Drive sync runs locally on the Jetson; use its configured timer."
            self.messageChanged.emit()
            return
        def work():
            ok, message = trigger_drive_sync()
            self._log("drive", "info" if ok else "error", message or "Sync request complete")
            self.refreshRecordings()
        threading.Thread(target=work, daemon=True).start()

    @Slot(str)
    @audited
    def openPath(self, path: str) -> None:
        if self._is_remote():
            destination = self.project_root / "remote_downloads"
            self._message = f"Downloading {Path(path).name} from Jetson…"
            self.messageChanged.emit()
            def complete(ok: bool, result: str) -> None:
                self._message = f"Downloaded to {result}" if ok else f"Download failed: {result}"
                self.messageChanged.emit()
                self._log("download", "info" if ok else "error", self._message)
                if ok:
                    QDesktopServices.openUrl(QUrl.fromLocalFile(result))
            self.remote.download_recording(path, destination, complete)
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(path))

