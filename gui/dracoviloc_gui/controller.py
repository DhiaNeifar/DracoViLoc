from __future__ import annotations

import json
import re
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot
from PySide6.QtGui import QDesktopServices
from PySide6.QtCore import QUrl

from .commands import runtime_environment, sourced_host_command, yolo_docker_command
from .models import AppState, LogEvent, RunProfile
from .preflight import blocking_failures, run_preflight
from .processes import ProcessSupervisor
from .profiles import ProfileStore
from .presets import built_in_profiles
from .recordings import discover_recordings, drive_sync_status, trigger_drive_sync
from .ros_bridge import RosBridge
from .validation import validate_profile


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
    quitReady = Signal()

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
        self._shutting_down_for_quit = False
        self.processes = ProcessSupervisor(self._log, self._process_exited)
        self.ros = RosBridge(self._ros_status, self._log, self._camera_frame)
        self.ros.start()

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

    @Property(bool, notify=stateChanged)
    def runActive(self) -> bool:
        return bool(self.processes.running()) or self._state not in {
            AppState.IDLE, AppState.FAULT,
        }

    def _set_state(self, state: AppState, message: str | None = None) -> None:
        self._state = state
        self.stateChanged.emit()
        if message is not None:
            self._message = message
            self.messageChanged.emit()

    def _log(self, subsystem: str, severity: str, message: str) -> None:
        event = LogEvent(datetime.now().isoformat(timespec="seconds"), subsystem, severity, message)
        self.logAdded.emit(json.dumps(event.to_dict()))

    def _process_exited(self, name: str, code: int) -> None:
        self._log(name, "info" if code == 0 else "error", f"Process exited with code {code}")
        if name == "host" and self._state not in {AppState.SHUTTING_DOWN, AppState.IDLE}:
            self._set_state(AppState.FAULT, "The host launch exited unexpectedly.")

    def _ros_status(self, status: dict) -> None:
        self._status = status
        self.statusChanged.emit(json.dumps(status))
        if self._state == AppState.HOMING and status.get("at_home"):
            self._set_state(AppState.HOME_READY, "Home reached. Tracking may now be armed.")

    def _camera_frame(self, frame: object) -> None:
        self.cameraFrame.emit(frame)

    @Slot(str)
    def selectProfile(self, name: str) -> None:
        if self.runActive or name not in self._profiles:
            return
        self._profile = RunProfile.from_dict(self._profiles[name].to_dict())
        self.profileChanged.emit()
        self._message = f"{name} selected."
        self.messageChanged.emit()

    @Slot(str)
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
            setattr(self._profile, name, converted)
            self.profileChanged.emit()
        except (TypeError, ValueError):
            self._message = f"Invalid value for {name}."
            self.messageChanged.emit()

    @Slot(str, str)
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
    def runPreflight(self) -> None:
        if self.runActive:
            return
        self._set_state(AppState.PREFLIGHT, "Checking this run configuration…")

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
    def startRun(self) -> None:
        if self.runActive:
            return
        errors = validate_profile(self._profile)
        if errors:
            self._set_state(AppState.FAULT, " ".join(errors))
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
        try:
            if self._profile.yolo_enabled:
                self._set_state(AppState.STARTING_YOLO, "Starting the YOLO container…")
                stale = subprocess.run(
                    ["docker", "container", "inspect", "isaac_yolo",
                     "--format", "{{.State.Running}}"],
                    capture_output=True, text=True, timeout=3.0)
                if stale.returncode == 0 and stale.stdout.strip().lower() == "false":
                    removed = subprocess.run(
                        ["docker", "rm", "isaac_yolo"],
                        capture_output=True, text=True, timeout=5.0)
                    if removed.returncode != 0:
                        raise RuntimeError(
                            "Could not remove the stopped isaac_yolo container: " +
                            (removed.stderr.strip() or "unknown Docker error"))
                    self._log("yolo", "info", "Removed stopped stale isaac_yolo container")
                self.processes.start(
                    "yolo", yolo_docker_command(self.project_root, self._profile),
                    self.project_root, env)
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
            self.processes.start(
                "host", sourced_host_command(self.project_root, self._profile),
                self.project_root, env)
            self._set_state(AppState.READY, "Run active. Motion remains disabled.")
        except Exception as exc:
            self._log("supervisor", "error", str(exc))
            self._set_state(AppState.FAULT, str(exc))

    @Slot()
    def requestHome(self) -> None:
        if self._state not in {AppState.READY, AppState.HOME_READY}:
            return
        self._set_state(AppState.HOMING, "Home trajectory requested…")

        def complete(ok: bool, message: str):
            self._log("motion", "info" if ok else "error", message)
            if not ok:
                self._set_state(AppState.READY, message)
            else:
                self._message = message
                self.messageChanged.emit()
        self.ros.request_home(complete)

    @Slot()
    def enableTracking(self) -> None:
        if self._state not in {AppState.HOME_READY, AppState.READY}:
            return

        def complete(ok: bool, message: str):
            self._log("motion", "info" if ok else "error", message)
            self._set_state(AppState.TRACKING if ok else AppState.READY, message)
        self.ros.set_tracking(True, complete)

    @Slot()
    def stopMotion(self) -> None:
        if self._state in {AppState.IDLE, AppState.PREFLIGHT, AppState.SHUTTING_DOWN}:
            return
        self._set_state(AppState.STOPPING_MOTION, "Requesting a controlled motion stop…")

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
    def stopAll(self) -> None:
        if self._state == AppState.SHUTTING_DOWN:
            return
        self._set_state(AppState.SHUTTING_DOWN, "Stopping motion and finalizing the run…")

        def after_motion(ok: bool, message: str):
            self._log("motion", "info" if ok else "error", message)
            if not ok and self._profile.hardware_mode == "real":
                self._set_state(
                    AppState.FAULT,
                    "Safe hold could not be confirmed. Engage the physical emergency stop before forcing shutdown.")
                return
            def verify_and_finish():
                if self._wait_for_trajectory_controller():
                    self._finish_shutdown()
                elif self._profile.hardware_mode == "real":
                    self._set_state(
                        AppState.FAULT,
                        "Trajectory controller did not return. Engage the physical emergency stop before forcing shutdown.")
                else:
                    self._finish_shutdown()
            threading.Thread(target=verify_and_finish, daemon=True).start()
        if self._status.get("tracking_service"):
            self.ros.set_tracking(False, after_motion)
        elif self._profile.hardware_mode == "real" and self.processes.is_running("host"):
            self._set_state(
                AppState.FAULT,
                "Tracking service is unavailable. Engage the physical emergency stop before forcing shutdown.")
        else:
            threading.Thread(target=self._finish_shutdown, daemon=True).start()

    @Slot()
    def forceStopAfterEstop(self) -> None:
        """Operator attests that the physical E-stop has made termination safe."""
        self._set_state(AppState.SHUTTING_DOWN, "Physical E-stop confirmed; stopping processes…")
        threading.Thread(target=self._finish_shutdown, daemon=True).start()

    def _finish_shutdown(self) -> None:
        self.processes.stop_all()
        self._set_state(AppState.IDLE, "All managed processes stopped.")
        if self._shutting_down_for_quit:
            self.ros.stop()
            self.quitReady.emit()

    @Slot()
    def requestQuit(self) -> None:
        if self.processes.running():
            self._shutting_down_for_quit = True
            self.stopAll()
        else:
            self.ros.stop()
            self.quitReady.emit()

    @Slot()
    def refreshRecordings(self) -> None:
        root = Path(self._profile.recording_root).expanduser()
        recordings = [item.to_dict() for item in discover_recordings(root)]
        sync_ok, sync_state = drive_sync_status()
        self.recordingsChanged.emit(json.dumps({
            "items": recordings, "sync_active": sync_ok, "sync_state": sync_state,
        }))

    @Slot()
    def syncNow(self) -> None:
        def work():
            ok, message = trigger_drive_sync()
            self._log("drive", "info" if ok else "error", message or "Sync request complete")
            self.refreshRecordings()
        threading.Thread(target=work, daemon=True).start()

    @Slot(str)
    def openPath(self, path: str) -> None:
        QDesktopServices.openUrl(QUrl.fromLocalFile(path))

