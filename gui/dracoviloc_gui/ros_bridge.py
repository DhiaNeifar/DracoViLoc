from __future__ import annotations

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
    ) -> None:
        self.status_callback = status_callback
        self.log_callback = log_callback
        self.frame_callback = frame_callback
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
        self._controller_query_pending = False

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            import rclpy
            from controller_manager_msgs.srv import ListControllers
            from audio_utils_msgs.msg import AudioFrame
            from geometry_msgs.msg import Vector3Stamped
            from odas_ros_msgs.msg import OdasSstArrayStamped
            from sensor_msgs.msg import Image, JointState
            from std_msgs.msg import String
            from std_srvs.srv import SetBool, Trigger
            from vision_msgs.msg import Detection2DArray
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
                if topic == "/joint_states":
                    self._joint_count = len(message.name)
                    home = {
                        "joint1": math.pi / 2, "joint2": -math.pi / 2,
                        "joint3": -math.pi / 2, "joint4": 0.0,
                        "joint5": math.pi / 2, "joint6": 0.0,
                    }
                    observed = dict(zip(message.name, message.position))
                    single = all(name in observed for name in home)
                    left = all(f"left_{name}" in observed for name in home)
                    right = all(f"right_{name}" in observed for name in home)
                    def matches(prefix: str = "") -> bool:
                        return all(abs(math.remainder(observed[prefix + name] - target, 2 * math.pi)) <= 0.035
                                   for name, target in home.items())
                    self._at_home = (single and matches()) or (left and right and matches("left_") and matches("right_"))
                elif topic == "/tracking/events":
                    self._tracking_event = message.data
            return callback

        self._node.create_subscription(JointState, "/joint_states", seen("/joint_states"), 10)
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
        self._node.create_subscription(AudioFrame, "/sss", seen("/sss"), 10)
        self._node.create_subscription(OdasSstArrayStamped, "/sst", seen("/sst"), 10)
        self._node.create_subscription(Detection2DArray, "/detections_output", seen("/detections_output"), 10)
        if self.frame_callback is not None:
            def frame(message):
                self._last_seen["/image"] = time.monotonic()
                if message.encoding in {"rgb8", "bgr8"}:
                    self.frame_callback({
                        "data": bytes(message.data), "width": message.width,
                        "height": message.height, "step": message.step,
                        "encoding": message.encoding,
                    })
            self._node.create_subscription(
                Image, "/image", frame, rclpy.qos.qos_profile_sensor_data)

        self._home_client = self._node.create_client(Trigger, "/demo/home")
        self._tracking_client = self._node.create_client(SetBool, "/demo/tracking")
        self._controllers_client = self._node.create_client(ListControllers, "/controller_manager/list_controllers")
        last_status = 0.0
        last_controller_query = 0.0
        try:
            while rclpy.ok() and not self._stop.is_set():
                self._executor.spin_once(timeout_sec=0.1)
                now = time.monotonic()
                if (now - last_controller_query >= 1.0 and
                        self._controllers_client.service_is_ready() and
                        not self._controller_query_pending):
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
                    last_controller_query = now
                if now - last_status >= 0.5:
                    freshness = {
                        topic: max(0.0, now - stamp) for topic, stamp in self._last_seen.items()
                    }
                    self.status_callback({
                        "freshness": freshness,
                        "joint_count": self._joint_count,
                        "at_home": self._at_home,
                        "tracking_event": self._tracking_event,
                        "home_service": self._home_client.service_is_ready(),
                        "tracking_service": self._tracking_client.service_is_ready(),
                        "controllers": self._controllers,
                        "directions": self._directions,
                    })
                    last_status = now
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
        if self._thread:
            self._thread.join(timeout=2.0)

