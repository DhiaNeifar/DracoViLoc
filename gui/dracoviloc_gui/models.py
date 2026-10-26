from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class AppState(str, Enum):
    IDLE = "Idle"
    PREFLIGHT = "Preflight"
    STARTING_YOLO = "Starting YOLO"
    STARTING_HOST = "Starting Host"
    READY = "Ready"
    HOMING = "Homing"
    HOME_READY = "Home Ready"
    TRACKING = "Tracking"
    STOPPING_MOTION = "Stopping Motion"
    SHUTTING_DOWN = "Shutting Down"
    FAULT = "Fault"


@dataclass(slots=True)
class RunProfile:
    name: str = "Mock Observation"
    execution_target: str = "local"
    ssh_alias: str = "jetson"
    jetson_project_root: str = "~/DracoViLoc"
    jetson_isaac_root: str = "~/workspaces/isaac_ros-dev"
    remote_recordings_root: str = "~/DracoViLoc/runs"
    remote_agent_port: int = 48721
    hardware_mode: str = "mock"
    robot_ip: str = "192.168.58.2"
    dual_arm: bool = False
    left_robot_ip: str = "192.168.58.2"
    right_robot_ip: str = "192.168.58.3"
    second_arm_x: float = -0.75
    second_arm_y: float = 0.0
    second_arm_z: float = 0.0
    dual_tracking_policy: str = "first_arm_lead"
    lead_lock_tolerance_rad: float = 0.10
    audio_enabled: bool = True
    yolo_enabled: bool = False
    ast_enabled: bool = False
    gre_enabled: bool = False
    mobilenetv2_enabled: bool = True
    mobilenetv2_ekf_enabled: bool = True
    fusion_enabled: bool = True
    tracking_mode: str = "off"
    use_rviz: bool = True
    use_moveit: bool = False
    recording_enabled: bool = False
    diagnostics_enabled: bool = True
    first_arrival_mode: str = "off"
    recording_root: str = "~/DracoViLoc/runs"
    # Raw camera image (never the overlay). The JPEG stream arrives complete at
    # full rate over the UDP transport; "/image" is accepted too.
    recording_video_topic: str = "/image/compressed"
    # "auto" lets the launch pick the engine built for this machine's GPU and
    # an interpreter with TensorRT + PyCUDA (see host_platform.py).
    mobilenetv2_engine_path: str = "auto"
    mobilenetv2_venv_python: str = "auto"
    mobilenetv2_threshold: float = 0.75
    mobilenetv2_votes_required: int = 3
    mobilenetv2_vote_window: int = 5
    min_activity: float = 0.10
    ekf_process_noise: float = 0.05
    ekf_measurement_noise: float = 0.02
    ekf_yolo_measurement_noise: float = 0.03
    ekf_mobilenetv2_measurement_noise: float = 0.15
    ekf_innovation_gate: float = 11.34
    ekf_average_window: int = 5
    smoothing_alpha: float = 0.50
    angular_deadband: float = 0.003
    max_velocity: float = 5.0
    max_acceleration: float = 12.0
    max_jerk: float = 80.0
    max_tracking_error: float = 1.57
    command_rate_hz: float = 100.0
    require_home_before_tracking: bool = True
    home_duration_s: float = 12.0
    camera: str = "auto"
    yolo_image: str = "auto"
    # Local runs: "auto" (newest model with a TensorRT plan for this GPU) or a
    # model id from models/yolo (see yolo_models.py). The two paths below are
    # used only by remote (agent) runs.
    yolo_model: str = "auto"
    # Visual detector run in the detector container while yolo_enabled is on:
    # "yolo", "dfine" (D-FINE S) or "dfine_n" (D-FINE N), each with its own
    # model choice ("auto" = newest with a TensorRT plan for this GPU).
    visual_detector: str = "yolo"
    yolo_confidence_threshold: float = 0.25
    dfine_model: str = "auto"
    dfine_n_model: str = "auto"
    yolo_model_path: str = "/workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx"
    yolo_engine_path: str = "/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan"
    use_yolo_viewer: bool = False
    expert_arguments: dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RunProfile":
        allowed = cls.__dataclass_fields__.keys()
        return cls(**{key: value for key, value in data.items() if key in allowed}).apply_rules()

    def visual_model_choice(self) -> str:
        return {"dfine": self.dfine_model, "dfine_n": self.dfine_n_model}.get(
            self.visual_detector, self.yolo_model)

    def apply_rules(self) -> "RunProfile":
        """Keep dependent pipeline options consistent with their sources."""
        if not self.audio_enabled:
            self.mobilenetv2_enabled = False
            self.ast_enabled = False
            self.gre_enabled = False
            self.first_arrival_mode = "off"
            if self.tracking_mode in {
                "direct_mobilenetv2", "direct_ast", "direct_gre", "direct_either"
            }:
                self.tracking_mode = "off"
            if self.tracking_mode == "ekf" and not self.yolo_enabled:
                self.tracking_mode = "off"
        self.fusion_enabled = True
        self.mobilenetv2_ekf_enabled = self.mobilenetv2_enabled
        self.recording_enabled = self.audio_enabled or self.yolo_enabled
        self.diagnostics_enabled = True
        return self


@dataclass(slots=True)
class CheckResult:
    key: str
    label: str
    ok: bool
    detail: str
    blocking: bool = True

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class LogEvent:
    timestamp: str
    subsystem: str
    severity: str
    message: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

