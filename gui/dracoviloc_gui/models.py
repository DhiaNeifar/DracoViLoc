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
    hardware_mode: str = "mock"
    robot_ip: str = "192.168.58.2"
    dual_arm: bool = False
    left_robot_ip: str = "192.168.58.2"
    right_robot_ip: str = "192.168.58.3"
    audio_enabled: bool = False
    yolo_enabled: bool = False
    ast_enabled: bool = False
    gre_enabled: bool = False
    mobilenetv2_enabled: bool = False
    mobilenetv2_ekf_enabled: bool = False
    fusion_enabled: bool = False
    tracking_mode: str = "off"
    use_rviz: bool = True
    use_moveit: bool = False
    recording_enabled: bool = False
    diagnostics_enabled: bool = True
    first_arrival_mode: str = "off"
    recording_root: str = "~/DracoViLoc/runs"
    recording_video_topic: str = "/image"
    mobilenetv2_engine_path: str = "~/DracoViLoc/models/mobilenetv2/drone_fp32_t2000.engine"
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
    max_velocity: float = 2.50
    max_acceleration: float = 12.0
    max_jerk: float = 80.0
    max_tracking_error: float = 0.50
    command_rate_hz: float = 100.0
    require_home_before_tracking: bool = True
    home_duration_s: float = 12.0
    camera: str = "auto"
    yolo_model_path: str = "/workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx"
    yolo_engine_path: str = "/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan"
    use_yolo_viewer: bool = False
    expert_arguments: dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RunProfile":
        allowed = cls.__dataclass_fields__.keys()
        return cls(**{key: value for key, value in data.items() if key in allowed})


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

