from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(slots=True)
class AgentConfig:
    project_root: str = "~/DracoViLoc"
    isaac_root: str = "~/workspaces/isaac_ros-dev"
    recordings_root: str = "~/DracoViLoc/runs"
    listen_host: str = "127.0.0.1"
    listen_port: int = 48721
    yolo_image: str = "isaac_ros_dev-aarch64:latest"
    yolo_entrypoint: str = ""
    camera_topic: str = "/yolov8_processed_image"
    camera_fps: float = 10.0
    camera_jpeg_quality: int = 75

    @classmethod
    def load(cls, path: Path) -> "AgentConfig":
        if not path.exists():
            return cls()
        data = json.loads(path.read_text(encoding="utf-8"))
        allowed = cls.__dataclass_fields__
        return cls(**{key: value for key, value in data.items() if key in allowed})

    def expanded(self) -> "AgentConfig":
        data = asdict(self)
        for key in ("project_root", "isaac_root", "recordings_root"):
            data[key] = str(Path(data[key]).expanduser().resolve())
        return AgentConfig(**data)
