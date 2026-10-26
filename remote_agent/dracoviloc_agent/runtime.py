from __future__ import annotations

import json
import math
import re
import shlex
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable

from .config import AgentConfig
from .host_platform import AUTO, resolve_mobilenet_runtime, resolve_yolo_image


class AgentRuntime:
    def __init__(
        self,
        config: AgentConfig,
        status_callback: Callable[[dict[str, Any]], None],
        log_callback: Callable[[str, str, str], None],
        camera_callback: Callable[[dict[str, Any], bytes], None],
    ) -> None:
        self.config = config.expanded()
        self.status_callback = status_callback
        self.log_callback = log_callback
        self.camera_callback = camera_callback
        self.status: dict[str, Any] = {
            "state": "Idle", "freshness": {}, "controllers": {},
            "directions": {}, "tracking_service": False, "at_home": False,
            "run_active": False, "tracking": False,
        }
        self._stop = threading.Event()
        self._ros_thread: threading.Thread | None = None
        self._last_seen: dict[str, float] = {}
        self._last_camera = 0.0
        self._node = None
        self._executor = None
        self._tracking_client = None
        self._home_client = None

    @staticmethod
    def _run(command: list[str], timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
        return subprocess.run(command, capture_output=True, text=True, timeout=timeout)

    def start(self) -> None:
        if self._ros_thread and self._ros_thread.is_alive():
            return
        self._ros_thread = threading.Thread(target=self._run_ros, daemon=True)
        self._ros_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._ros_thread:
            self._ros_thread.join(timeout=3.0)

    def _run_ros(self) -> None:
        try:
            import numpy as np
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from audio_utils_msgs.msg import AudioFrame
            from controller_manager_msgs.srv import ListControllers
            from geometry_msgs.msg import Vector3Stamped
            from odas_ros_msgs.msg import OdasSstArrayStamped
            from sensor_msgs.msg import Image, JointState
            from std_msgs.msg import String
            from std_srvs.srv import SetBool, Trigger
            from vision_msgs.msg import Detection2DArray
        except ImportError as exc:
            self.log_callback("agent", "error", f"ROS dependency unavailable: {exc}")
            return
        try:
            import cv2
        except ImportError:
            cv2 = None
            self.log_callback(
                "camera", "warning", "OpenCV is unavailable; remote video preview is disabled")

        # systemd owns process signals; ROS runs in a worker thread and must not
        # intercept SIGTERM needed for reliable agent updates.
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("dracoviloc_remote_agent")
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        self._node, self._executor = node, executor

        def seen(topic: str):
            def callback(message):
                self._last_seen[topic] = time.monotonic()
                if topic == "/tracking/events":
                    self.status["tracking_event"] = message.data
                    if message.data.startswith("arm2_started:"):
                        self.status["arm2_started_event"] = message.data
                        self._last_seen["/tracking/arm2_started"] = time.monotonic()
            return callback

        def direction(topic: str):
            def callback(message):
                self._last_seen[topic] = time.monotonic()
                self.status["directions"][topic] = {
                    "x": message.vector.x, "y": message.vector.y, "z": message.vector.z,
                }
            return callback

        def joints(message):
            self._last_seen["/joint_states"] = time.monotonic()
            observed = dict(zip(message.name, message.position))
            home = {
                "joint1": math.pi / 2, "joint2": -math.pi / 2,
                "joint3": -math.pi / 2, "joint4": 0.0,
                "joint5": math.pi / 2, "joint6": 0.0,
            }
            def matches(prefix=""):
                return all(prefix + key in observed and abs(math.remainder(
                    observed[prefix + key] - value, 2 * math.pi)) <= 0.035
                    for key, value in home.items())
            self.status["at_home"] = matches() or (matches("left_") and matches("right_"))
            self.status["joint_count"] = len(message.name)
            names = set(message.name)
            left = all("left_" + key in names for key in home)
            right = all("right_" + key in names for key in home)
            single = all(key in names for key in home)
            if left or right:
                self.status["arms"] = {
                    "left": {"joint_count": sum(name.startswith("left_joint") for name in names),
                             "at_home": left and matches("left_")},
                    "right": {"joint_count": sum(name.startswith("right_joint") for name in names),
                              "at_home": right and matches("right_")},
                }
                self.status["dual_arm_detected"] = left and right
            else:
                self.status["arms"] = {"primary": {
                    "joint_count": sum(name in home for name in names),
                    "at_home": single and matches(),
                }}
                self.status["dual_arm_detected"] = False

        def camera(message):
            now = time.monotonic()
            self._last_seen[self.config.camera_topic] = now
            self._last_seen["/image"] = now
            if cv2 is None:
                return
            if now - self._last_camera < 1.0 / max(1.0, self.config.camera_fps):
                return
            if message.encoding not in {"rgb8", "bgr8"}:
                return
            array = np.frombuffer(message.data, dtype=np.uint8).reshape(
                message.height, message.step)[:, :message.width * 3].reshape(
                message.height, message.width, 3)
            if message.encoding == "rgb8":
                array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            ok, encoded = cv2.imencode(
                ".jpg", array, [cv2.IMWRITE_JPEG_QUALITY, self.config.camera_jpeg_quality])
            if ok:
                self._last_camera = now
                self.camera_callback({
                    "width": message.width, "height": message.height,
                    "encoding": "jpeg", "timestamp": time.time(),
                }, encoded.tobytes())

        node.create_subscription(
            JointState, "/joint_states", joints, rclpy.qos.qos_profile_sensor_data)
        node.create_subscription(String, "/tracking/events", seen("/tracking/events"), 20)
        node.create_subscription(AudioFrame, "/sss", seen("/sss"), 10)
        node.create_subscription(OdasSstArrayStamped, "/sst", seen("/sst"), 10)
        node.create_subscription(Detection2DArray, "/detections_output", seen("/detections_output"), 10)
        for topic in ("/yolo/direction", "/mobilenetv2/direction", "/ast/direction", "/gre/direction", "/ekf/direction"):
            node.create_subscription(Vector3Stamped, topic, direction(topic), 10)
        node.create_subscription(
            Image, self.config.camera_topic, camera, rclpy.qos.qos_profile_sensor_data)
        # Create demo service clients lazily when their server appears. On the
        # Jetson's Fast DDS build, clients created before the host process can
        # remain permanently unmatched even though the service later exists.
        self._home_client = None
        self._tracking_client = None
        controllers_client = node.create_client(ListControllers, "/controller_manager/list_controllers")
        controller_pending = False
        last_controller_query = 0.0
        last_status = 0.0
        try:
            while rclpy.ok() and not self._stop.is_set():
                executor.spin_once(timeout_sec=0.1)
                now = time.monotonic()
                if now - last_controller_query >= 1.0 and controllers_client.service_is_ready() and not controller_pending:
                    controller_pending = True
                    future = controllers_client.call_async(ListControllers.Request())
                    def complete(done):
                        nonlocal controller_pending
                        try:
                            response = done.result()
                            self.status["controllers"] = {item.name: item.state for item in response.controller}
                            self.status["tracking"] = any(
                                "tracking_controller" in name and state == "active"
                                for name, state in self.status["controllers"].items())
                        finally:
                            controller_pending = False
                    future.add_done_callback(complete)
                    last_controller_query = now
                if now - last_status >= 0.5:
                    self.status["freshness"] = {
                        topic: max(0.0, now - stamp) for topic, stamp in self._last_seen.items()
                    }
                    service_names = {
                        name for name, _types in node.get_service_names_and_types()}
                    if "/demo/home" in service_names and self._home_client is None:
                        self._home_client = node.create_client(Trigger, "/demo/home")
                    elif "/demo/home" not in service_names and self._home_client is not None:
                        node.destroy_client(self._home_client)
                        self._home_client = None
                    if "/demo/tracking" in service_names and self._tracking_client is None:
                        self._tracking_client = node.create_client(SetBool, "/demo/tracking")
                    elif ("/demo/tracking" not in service_names and
                          self._tracking_client is not None):
                        node.destroy_client(self._tracking_client)
                        self._tracking_client = None
                    node_names = set(node.get_node_names())
                    tracker_present = "arm_tracker" in node_names
                    self.status["mobilenetv2_node"] = (
                        "mobilenetv2_classifier_node" in node_names)
                    # The Jetson Fast DDS graph cache may not add services
                    # created after this long-lived participant. The arm
                    # tracker is launched alongside these controllers; calls
                    # use a clean transient ROS process below.
                    self.status["home_service"] = (
                        "/demo/home" in service_names or
                        (self.run_active() and tracker_present))
                    self.status["tracking_service"] = (
                        "/demo/tracking" in service_names or
                        (self.run_active() and tracker_present))
                    self.status["run_active"] = self.run_active()
                    self.status_callback(dict(self.status))
                    last_status = now
        except Exception as exc:
            if rclpy.ok() and not self._stop.is_set():
                self.log_callback("agent", "error", f"ROS monitor stopped: {exc}")
        finally:
            try:
                executor.remove_node(node)
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    def _call_service(self, client, request, timeout=5.0) -> tuple[bool, str]:
        if client is None:
            return False, "Service is unavailable"
        try:
            future = client.call_async(request)
        except Exception as exc:
            return False, str(exc)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, "Service request timed out"
        try:
            response = future.result()
            return bool(response.success), str(response.message)
        except Exception as exc:
            return False, str(exc)

    def home(self) -> tuple[bool, str]:
        return self._call_service_cli(
            "/demo/home", "std_srvs/srv/Trigger", "{}", timeout=20.0)

    def set_tracking(self, enabled: bool) -> tuple[bool, str]:
        value = "true" if enabled else "false"
        return self._call_service_cli(
            "/demo/tracking", "std_srvs/srv/SetBool", f"{{data: {value}}}",
            timeout=20.0)

    def _call_service_cli(
        self, name: str, service_type: str, request: str, timeout: float = 8.0,
    ) -> tuple[bool, str]:
        try:
            root = Path(self.config.project_root)
            shell = (
                "source /opt/ros/humble/setup.bash; "
                f"source {shlex.quote(str(root / 'install/setup.bash'))}; "
                "export ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4; exec " +
                shlex.join(["ros2", "service", "call", name, service_type, request]))
            unit = f"dracoviloc-service-call-{time.time_ns()}"
            result = self._run([
                "systemd-run", "--user", "--wait", "--pipe", "--collect",
                f"--unit={unit}", "/bin/bash", "-lc", shell,
            ], timeout)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, str(exc)
        output = "\n".join(filter(None, (result.stdout.strip(), result.stderr.strip())))
        success = result.returncode == 0 and (
            "success=True" in output or "success: true" in output.lower())
        message_match = re.search(r"message=['\"]([^'\"]*)", output)
        message = message_match.group(1) if message_match else output
        return success, message or ("Service succeeded" if success else "Service failed")

    def preflight(self, profile: dict[str, Any]) -> list[dict[str, Any]]:
        checks: list[dict[str, Any]] = []
        def check(key, label, ok, detail, blocking=True):
            checks.append({"key": key, "label": label, "ok": bool(ok),
                           "detail": str(detail), "blocking": blocking})
        root, isaac = Path(self.config.project_root), Path(self.config.isaac_root)
        check("workspace", "Jetson workspace", (root / "install/setup.bash").exists(), root)
        check("isaac", "Isaac workspace", (isaac / "install/setup.bash").exists(), isaac,
              profile.get("yolo_enabled", False))
        sound = Path("/proc/asound/cards").read_text(errors="ignore") if Path("/proc/asound/cards").exists() else ""
        check("uma16", "UMA16 microphone", "UMA16" in sound or "miniDSP" in sound,
              "Detected" if "UMA16" in sound or "miniDSP" in sound else "Not detected",
              profile.get("audio_enabled", False))
        cameras = " ".join(path.read_text(errors="ignore") for path in Path("/sys/class/video4linux").glob("video*/name"))
        check("camera", "icSpring camera", "icspring" in cameras.lower(),
              "Detected" if "icspring" in cameras.lower() else "Not detected",
              profile.get("yolo_enabled", False))
        docker = self._run(["docker", "info", "--format", "{{.ServerVersion}}"], 4.0)
        check("docker", "Docker", docker.returncode == 0, docker.stdout.strip() or docker.stderr.strip(),
              profile.get("yolo_enabled", False))
        if profile.get("yolo_enabled"):
            yolo_image = resolve_yolo_image(self.config.yolo_image)
            image = self._run([
                "docker", "image", "inspect", yolo_image or "<none>",
                "--format", "{{.Id}}"], 4.0)
            check("yolo_image", "Isaac ROS image", image.returncode == 0,
                  yolo_image if image.returncode == 0 else
                  f"Missing: {yolo_image or 'no known image for this CPU architecture'}")
        free = shutil.disk_usage(Path(self.config.recordings_root).parent).free
        check("disk", "Recording disk", free >= 2 * 1024 ** 3, f"{free / 1024**3:.1f} GiB free",
              profile.get("recording_enabled", False))
        check("duplicate", "Existing managed run", not self.run_active(),
              "None" if not self.run_active() else "A run is already active")
        engine_checks = []
        if profile.get("mobilenetv2_enabled"):
            selected = resolve_mobilenet_runtime(root)
            if "error" in selected:
                check("mobilenetv2_engine", "MobileNetV2 TensorRT engine", False, selected["error"])
            else:
                engine_checks.append((
                    "mobilenetv2_engine", "MobileNetV2 TensorRT engine",
                    Path(selected["engine"]), Path(selected["python"])))
        if profile.get("ast_enabled"):
            engine_checks.append((
                "ast_engine", "AST TensorRT engine",
                root / "models/ast/drone_ast.engine", root / "trt_env/bin/python3"))
        if profile.get("gre_enabled"):
            engine_checks.append((
                "gre_engine", "GRE TensorRT engine",
                root / "models/gre/model_logmel.engine", root / "gre_env/bin/python3"))
        deserialize = (
            "import pathlib,sys,tensorrt as trt; "
            "p=pathlib.Path(sys.argv[1]); "
            "r=trt.Runtime(trt.Logger(trt.Logger.ERROR)); "
            "e=r.deserialize_cuda_engine(p.read_bytes()); "
            "raise SystemExit(0 if e else 2)")
        for key, label, engine, python in engine_checks:
            if not engine.exists() or not python.exists():
                check(key, label, False, f"Missing: {engine}")
                continue
            try:
                loaded = self._run([str(python), "-c", deserialize, str(engine)], 10.0)
                check(key, label, loaded.returncode == 0,
                      str(engine) if loaded.returncode == 0 else
                      (loaded.stderr.strip() or "TensorRT deserialization failed"))
            except (OSError, subprocess.TimeoutExpired) as exc:
                check(key, label, False, exc)
        if profile.get("hardware_mode") == "real":
            robots = ([
                ("left_robot", "Left FAIRINO cobot", profile.get("left_robot_ip", "")),
                ("right_robot", "Right FAIRINO cobot", profile.get("right_robot_ip", "")),
            ] if profile.get("dual_arm") else [
                ("robot", "FAIRINO controller", profile.get("robot_ip", "")),
            ])
            for key, label, address in robots:
                try:
                    result = self._run(["ping", "-c", "1", "-W", "1", str(address)], 2.0)
                    reachable = result.returncode == 0
                    detail = f"{address} reachable" if reachable else f"{address} unreachable"
                except (OSError, subprocess.TimeoutExpired) as exc:
                    reachable, detail = False, f"{address}: {exc}"
                check(key, label, reachable, detail)
        return checks

    def _host_args(self, profile: dict[str, Any]) -> list[str]:
        skip = {"name", "execution_target", "ssh_alias", "jetson_project_root",
                "jetson_isaac_root", "remote_recordings_root", "remote_agent_port",
                "expert_arguments", "camera", "yolo_model_path", "yolo_engine_path",
                "use_yolo_viewer"}
        values = {key: value for key, value in profile.items() if key not in skip}
        if values.get("mobilenetv2_engine_path"):
            values["mobilenetv2_engine_path"] = str(
                Path(str(values["mobilenetv2_engine_path"])).expanduser())
        values.update(profile.get("expert_arguments", {}))
        # Remote mode is dashboard-only, and recordings must stay on the Jetson.
        values["use_rviz"] = False
        values["recording_root"] = self.config.recordings_root
        # The engine must match the agent machine's GPU, not the GUI's.
        values["mobilenetv2_engine_path"] = AUTO
        values["mobilenetv2_venv_python"] = AUTO
        def render(value):
            if isinstance(value, bool):
                return "true" if value else "false"
            return str(value)
        return [f"{key}:={render(value)}" for key, value in values.items()]

    def start_run(self, profile: dict[str, Any]) -> tuple[bool, str]:
        if self.run_active():
            return False, "A managed run is already active"
        root = Path(self.config.project_root)
        self._last_seen.pop("/joint_states", None)
        self.status["joint_count"] = 0
        self.status["at_home"] = False
        self.status["arms"] = {}
        self.status["dual_arm_detected"] = False
        if profile.get("yolo_enabled"):
            self._last_seen.pop(self.config.camera_topic, None)
            self._last_seen.pop("/image", None)
            stale = self._run(["docker", "inspect", "isaac_yolo", "--format", "{{.State.Running}}"], 3.0)
            if stale.returncode == 0 and stale.stdout.strip() == "false":
                self._run(["docker", "rm", "isaac_yolo"], 5.0)
            isaac = Path(self.config.isaac_root)
            yolo = [
                "docker", "run", "-d", "--rm", "--privileged", "--network", "host",
                "--ipc", "host", "--runtime", "nvidia", "--name", "isaac_yolo",
                "-e", "ROS_DOMAIN_ID=42", "-e", "FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
                "-e", "CUDA_MODULE_LOADING=LAZY",
                "-v", f"{isaac}:/workspaces/isaac_ros-dev", "--workdir", "/workspaces/isaac_ros-dev",
            ]
            if self.config.yolo_entrypoint:
                yolo += ["--entrypoint", self.config.yolo_entrypoint]
            inner = shlex.join([
                "ros2", "launch", "isaac_ros_yolo_direction", "yolo_camera.launch.py",
                f"camera:={profile.get('camera', 'auto')}",
                f"model_path:={profile.get('yolo_model_path')}",
                f"engine_path:={profile.get('yolo_engine_path')}",
                "width:=640", "height:=480", "num_blocks:=4",
                "camera_fps:=15", "publish_rate:=15.0",
                "direction_frame:=table_mic_link", "use_viewer:=false",
            ])
            shell = ("source /opt/ros/humble/setup.bash; "
                     "source /workspaces/isaac_ros-dev/install/setup.bash; exec " + inner)
            yolo += [resolve_yolo_image(self.config.yolo_image), "/bin/bash", "-lc", shell]
            result = self._run(yolo, 20.0)
            if result.returncode != 0:
                return False, result.stderr.strip() or "YOLO container failed"
            deadline = time.monotonic() + 45.0
            while time.monotonic() < deadline:
                container = self._run([
                    "docker", "inspect", "isaac_yolo", "--format",
                    "{{.State.Running}}"], 3.0)
                if container.returncode != 0 or container.stdout.strip() != "true":
                    logs = self._run(["docker", "logs", "--tail", "30", "isaac_yolo"], 5.0)
                    return False, logs.stderr.strip() or logs.stdout.strip() or "YOLO exited during startup"
                freshness = time.monotonic() - self._last_seen.get(
                    self.config.camera_topic, 0.0)
                if freshness < 2.0:
                    break
                time.sleep(0.5)
            else:
                logs = self._run(["docker", "logs", "--tail", "30", "isaac_yolo"], 5.0)
                self._run(["docker", "kill", "isaac_yolo"], 5.0)
                detail = logs.stderr.strip() or logs.stdout.strip()
                return False, "YOLO produced no annotated image within 45 seconds" + (
                    f": {detail}" if detail else "")
        command = [
            "ros2", "launch", "dracoviloc_bringup", "arm_demo.launch.py",
            *self._host_args(profile),
        ]
        shell = (
            f"source /opt/ros/humble/setup.bash; source {shlex.quote(str(root / 'install/setup.bash'))}; "
            f"export ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 CUDA_MODULE_LOADING=LAZY; "
            "exec " + shlex.join(command))
        result = self._run([
            "systemd-run", "--user", "--unit=dracoviloc-host", "--collect",
            "--property=KillMode=control-group", "/bin/bash", "-lc", shell,
        ], 10.0)
        if result.returncode != 0:
            self._run(["docker", "stop", "--time", "10", "isaac_yolo"], 15.0)
            return False, result.stderr.strip() or "Host unit failed"
        expected_controller = (
            "dual_arm_controller" if profile.get("dual_arm") else "arm_controller")
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            if not self.run_active():
                self._stop_managed_processes()
                return False, "Host launch exited before the controllers became ready"
            controller_ready = (
                self.status.get("controllers", {}).get(expected_controller) == "active")
            tracking_ready = (
                profile.get("tracking_mode", "off") == "off" or
                self.status.get("tracking_service"))
            expected_joints = 12 if profile.get("dual_arm") else 6
            joints_ready = (
                self.status.get("joint_count", 0) >= expected_joints and
                self.status.get("freshness", {}).get("/joint_states", 999.0) < 2.0)
            classifier_ready = (
                not profile.get("mobilenetv2_enabled") or
                self.status.get("mobilenetv2_node", False))
            if controller_ready and tracking_ready and joints_ready and classifier_ready:
                break
            time.sleep(0.5)
        else:
            controllers = self.status.get("controllers", {})
            self._stop_managed_processes()
            return False, (
                f"{expected_controller}, joint feedback, or enabled classifier did not become ready; "
                f"observed controllers: {controllers}, "
                f"joints: {self.status.get('joint_count', 0)}/{expected_joints}, "
                f"MobileNetV2: {self.status.get('mobilenetv2_node', False)}")
        self.status["state"] = "Ready"
        return True, "Remote run started; motion remains disabled"

    def run_active(self) -> bool:
        result = self._run([
            "systemctl", "--user", "is-active", "dracoviloc-host.service"], 2.0)
        return result.returncode == 0

    def stop_all(self) -> tuple[bool, str]:
        if self.run_active() and not self.status.get("tracking_service"):
            return False, (
                "Tracking service is unavailable; engage the physical emergency stop(s) "
                "before forcing shutdown")
        if self.status.get("tracking_service"):
            ok, message = self.set_tracking(False)
            if not ok:
                return False, message
            restored_by_service = "already disabled" in message.lower()
            next_service_check = time.monotonic() + 1.0
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                controllers = self.status.get("controllers", {})
                restored_by_cache = any(
                    "arm_controller" in key and "tracking" not in key and value == "active"
                    for key, value in controllers.items())
                if restored_by_cache or restored_by_service:
                    break
                if time.monotonic() >= next_service_check:
                    check_ok, check_message = self.set_tracking(False)
                    restored_by_service = (
                        check_ok and "already disabled" in check_message.lower())
                    next_service_check = time.monotonic() + 1.0
                time.sleep(0.2)
            else:
                return False, "Trajectory controller restoration was not confirmed"
        self._stop_managed_processes()
        self.status["state"] = "Idle"
        return True, "Remote run stopped"

    def _stop_managed_processes(self, force: bool = False) -> None:
        if not force:
            try:
                self._run(["systemctl", "--user", "stop", "dracoviloc-host.service"], 15.0)
            except subprocess.TimeoutExpired:
                pass
        if self.run_active():
            self._run([
                "systemctl", "--user", "kill", "--kill-who=all", "--signal=SIGKILL",
                "dracoviloc-host.service"], 5.0)
        if not force:
            try:
                self._run(["docker", "stop", "--time", "5", "isaac_yolo"], 10.0)
            except subprocess.TimeoutExpired:
                pass
        running = self._run([
            "docker", "inspect", "isaac_yolo", "--format", "{{.State.Running}}"], 3.0)
        if running.returncode == 0 and running.stdout.strip() == "true":
            self._run(["docker", "kill", "isaac_yolo"], 5.0)

    def force_stop_all(self) -> tuple[bool, str]:
        """Terminate only after the operator confirms physical E-stop protection."""
        self._stop_managed_processes(force=True)
        self.status["state"] = "Idle"
        return True, "Remote run stopped"

    def list_recordings(self) -> list[dict[str, Any]]:
        # Same layout rules as the GUI: runs/<time>/{video,audio}/ plus the
        # earlier runs/{video,audio}/<time>/ layout.
        from .recordings import discover_recordings
        return [row.to_dict() for row in discover_recordings(Path(self.config.recordings_root))]
