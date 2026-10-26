from __future__ import annotations

import atexit
import json
import math
import threading
import time
from typing import Callable


class RosBridge:
    """Small rclpy observer/service layer kept independent of Qt."""

    def __init__(
        self,
        status_callback: Callable[[dict], None],
        log_callback: Callable[[str, str, str], None],
        frame_callback: Callable[[object], None] | None = None,
        telemetry_callback: Callable[[str], None] | None = None,
    ) -> None:
        self.status_callback = status_callback
        self.log_callback = log_callback
        self.frame_callback = frame_callback
        self.telemetry_callback = telemetry_callback
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._node = None
        self._executor = None
        self._last_seen: dict[str, float] = {}
        self._joint_count = 0
        self._at_home = False
        self._tracking_event = ""
        self._controllers: dict[str, str] = {}
        self._directions: dict[str, dict[str, float]] = {}
        self._arms: dict[str, dict[str, object]] = {}
        self._controller_query_pending = False

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        # Any exit path must shut the executor down: a spinning rclpy
        # executor during interpreter teardown aborts the process.
        atexit.register(self.stop)

    def _run(self) -> None:
        try:
            import rclpy
            from controller_manager_msgs.srv import ListControllers
            from geometry_msgs.msg import Vector3Stamped
            from sensor_msgs.msg import CameraInfo, CompressedImage
            from std_msgs.msg import String
            from std_srvs.srv import SetBool, Trigger
        except ImportError as exc:
            self.log_callback("ros", "error", f"ROS Python dependency unavailable: {exc}")
            return

        rclpy.init(args=None)
        self._node = rclpy.create_node("dracoviloc_gui")
        self._executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)

        def seen(topic: str):
            def callback(message):
                self._last_seen[topic] = time.monotonic()
                if topic == "/tracking/events":
                    self._tracking_event = message.data
            return callback

        self._node.create_subscription(String, "/tracking/events", seen("/tracking/events"), 20)
        self._node.create_subscription(String, "/diagnostics/events", seen("/diagnostics/events"), 20)
        def direction_seen(topic: str):
            def callback(message):
                self._last_seen[topic] = time.monotonic()
                self._directions[topic] = {
                    "x": message.vector.x, "y": message.vector.y, "z": message.vector.z,
                }
            return callback
        for topic in ("/yolo/direction", "/mobilenetv2/direction", "/ast/direction", "/gre/direction", "/ekf/direction"):
            self._node.create_subscription(Vector3Stamped, topic, direction_seen(topic), 10)
        # The camera publishes one small CameraInfo per frame: liveness of the
        # raw stream without moving pixels through Python (kept as "/image").
        def camera_info(_message):
            self._last_seen["/image"] = time.monotonic()
        self._node.create_subscription(
            CameraInfo, "/camera_info", camera_info, rclpy.qos.qos_profile_sensor_data)
        if self.frame_callback is not None:
            # JPEG previews from dracoviloc_monitor/preview_node; Qt decodes them.
            def preview(stream: str):
                def callback(message):
                    self._last_seen[f"/dracoviloc/preview/{stream}"] = time.monotonic()
                    self.frame_callback({"stream": stream, "encoding": "jpeg",
                                         "data": bytes(message.data)})
                return callback
            for stream in ("raw", "overlay"):
                self._node.create_subscription(
                    CompressedImage, f"/dracoviloc/preview/{stream}/compressed",
                    preview(stream), rclpy.qos.QoSProfile(
                        depth=2, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        # High-rate topics (joint states ~100 Hz, ODAS tracks ~85 Hz,
        # detections) are summarised by the C++ telemetry node; the GUI only
        # parses its 10 Hz snapshot.
        def telemetry(message):
            now = time.monotonic()
            self._last_seen["/dracoviloc/telemetry"] = now
            try:
                self._apply_telemetry(json.loads(message.data), now)
            except (ValueError, KeyError, TypeError) as exc:
                self.log_callback("ros", "warning", f"Bad telemetry snapshot: {exc}")
            if self.telemetry_callback is not None:
                self.telemetry_callback(message.data)
        self._node.create_subscription(String, "/dracoviloc/telemetry", telemetry, 10)

        self._home_client = self._node.create_client(Trigger, "/demo/home")
        self._tracking_client = self._node.create_client(SetBool, "/demo/tracking")
        self._controllers_client = self._node.create_client(ListControllers, "/controller_manager/list_controllers")
        def query_controllers():
            if (not self._controllers_client.service_is_ready() or
                    self._controller_query_pending):
                return
            self._controller_query_pending = True
            future = self._controllers_client.call_async(ListControllers.Request())
            def controllers_complete(done):
                try:
                    response = done.result()
                    self._controllers = {
                        controller.name: controller.state
                        for controller in response.controller
                    }
                except Exception as exc:
                    self.log_callback("ros", "error", f"Controller query failed: {exc}")
                finally:
                    self._controller_query_pending = False
            future.add_done_callback(controllers_complete)

        def publish_status():
            now = time.monotonic()
            freshness = {
                topic: max(0.0, now - stamp) for topic, stamp in self._last_seen.items()
            }
            self.status_callback({
                "freshness": freshness,
                "joint_count": self._joint_count,
                "arms": self._arms,
                "dual_arm_detected": "left" in self._arms and "right" in self._arms,
                "at_home": self._at_home,
                "tracking_event": self._tracking_event,
                "home_service": self._home_client.service_is_ready(),
                "tracking_service": self._tracking_client.service_is_ready(),
                "controllers": self._controllers,
                "directions": self._directions,
            })

        self._node.create_timer(1.0, query_controllers)
        self._node.create_timer(0.5, publish_status)
        try:
            # Spin continuously until stop() shuts the executor down. A
            # spin_once loop handles one message per call and starved the
            # video previews behind 100 Hz joint states.
            if not self._stop.is_set():
                self._executor.spin()
        except Exception as exc:
            # ROS installs process signal handlers; SIGINT/SIGTERM can
            # invalidate the context between the loop check and an API call.
            if not self._stop.is_set() and rclpy.ok():
                self.log_callback("ros", "error", f"ROS monitor stopped: {exc}")
        finally:
            try:
                self._executor.remove_node(self._node)
                self._node.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                # A process signal may already have shut down the default context.
                pass

    HOME = {"joint1": math.pi / 2, "joint2": -math.pi / 2, "joint3": -math.pi / 2,
            "joint4": 0.0, "joint5": math.pi / 2, "joint6": 0.0}

    def _apply_telemetry(self, snapshot: dict, now: float) -> None:
        """Arm and liveness status from a telemetry snapshot."""
        joints = snapshot.get("joints", {})
        observed = {item["name"]: item["position"] for item in joints.get("items", [])}
        if joints.get("rate_hz", 0) > 0 and observed:
            self._last_seen["/joint_states"] = now
            self._joint_count = len(observed)
            single = all(name in observed for name in self.HOME)
            left = all(f"left_{name}" in observed for name in self.HOME)
            right = all(f"right_{name}" in observed for name in self.HOME)

            def matches(prefix: str = "") -> bool:
                return all(abs(math.remainder(observed[prefix + name] - target, 2 * math.pi)) <= 0.035
                           for name, target in self.HOME.items())
            self._at_home = (single and matches()) or (left and right and matches("left_") and matches("right_"))
            if left or right:
                self._arms = {
                    "left": {"joint_count": sum(name.startswith("left_joint") for name in observed),
                             "at_home": left and matches("left_")},
                    "right": {"joint_count": sum(name.startswith("right_joint") for name in observed),
                              "at_home": right and matches("right_")},
                }
            else:
                self._arms = {"primary": {"joint_count": sum(name in self.HOME for name in observed),
                                          "at_home": single and matches()}}
        if snapshot.get("audio", {}).get("rate_hz", 0) > 0:
            self._last_seen["/sst"] = now
        yolo_age = snapshot.get("yolo", {}).get("age_s")
        if yolo_age is not None and yolo_age < 2.0:
            self._last_seen["/detections_output"] = now - yolo_age

    def _service(self, client, request, callback: Callable[[bool, str], None]) -> None:
        if client is None or not client.service_is_ready():
            callback(False, "Service is unavailable")
            return
        future = client.call_async(request)

        def complete(done):
            try:
                response = done.result()
                callback(bool(response.success), str(response.message))
            except Exception as exc:  # ROS futures surface transport failures here.
                callback(False, str(exc))
        future.add_done_callback(complete)

    def set_parameters(self, node: str, values: dict[str, bool | float | int | str],
                       callback: Callable[[bool, str], None]) -> None:
        """Set parameters on a running node (e.g. arrow visibility)."""
        try:
            from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
            from rcl_interfaces.srv import SetParameters
        except ImportError as exc:
            callback(False, str(exc))
            return
        if self._node is None:
            callback(False, "ROS is not running")
            return
        clients = self.__dict__.setdefault("_parameter_clients", {})
        client = clients.get(node) or clients.setdefault(
            node, self._node.create_client(SetParameters, f"{node}/set_parameters"))
        if not client.service_is_ready():
            callback(False, f"{node} is not running")
            return
        request = SetParameters.Request()
        for name, value in values.items():
            parameter = ParameterValue()
            if isinstance(value, bool):
                parameter.type, parameter.bool_value = ParameterType.PARAMETER_BOOL, value
            elif isinstance(value, int):
                parameter.type, parameter.integer_value = ParameterType.PARAMETER_INTEGER, value
            elif isinstance(value, float):
                parameter.type, parameter.double_value = ParameterType.PARAMETER_DOUBLE, value
            else:
                parameter.type, parameter.string_value = ParameterType.PARAMETER_STRING, str(value)
            request.parameters.append(Parameter(name=name, value=parameter))
        future = client.call_async(request)

        def complete(done):
            try:
                results = done.result().results
                failed = [r.reason for r in results if not r.successful]
                callback(not failed, "; ".join(failed) or "ok")
            except Exception as exc:
                callback(False, str(exc))
        future.add_done_callback(complete)

    def request_home(self, callback: Callable[[bool, str], None]) -> None:
        try:
            from std_srvs.srv import Trigger
            self._service(self._home_client, Trigger.Request(), callback)
        except (ImportError, AttributeError) as exc:
            callback(False, str(exc))

    def set_tracking(self, enabled: bool, callback: Callable[[bool, str], None]) -> None:
        try:
            from std_srvs.srv import SetBool
            request = SetBool.Request()
            request.data = enabled
            self._service(self._tracking_client, request, callback)
        except (ImportError, AttributeError) as exc:
            callback(False, str(exc))

    def stop(self) -> None:
        self._stop.set()
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        if self._thread:
            self._thread.join(timeout=2.0)

