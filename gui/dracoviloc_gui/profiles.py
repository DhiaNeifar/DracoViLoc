from __future__ import annotations

import json
from pathlib import Path

from .models import RunProfile
from .presets import built_in_profiles


SCHEMA_VERSION = 1


class ProfileStore:
    def __init__(self, path: Path | None = None) -> None:
        self.path = path or Path.home() / ".config/dracoviloc-gui/profiles.json"

    def load(self) -> dict[str, RunProfile]:
        profiles = built_in_profiles()
        if not self.path.exists():
            return profiles
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            if payload.get("version") != SCHEMA_VERSION:
                return profiles
            for item in payload.get("profiles", []):
                profile = RunProfile.from_dict(item)
                # Migrate the former shared local/Jetson defaults. Remote runs
                # are overridden by the agent; local runs require the T2000
                # engine and the verified system TensorRT interpreter.
                if profile.execution_target == "local":
                    if item.get("mobilenetv2_engine_path") == (
                            "~/DracoViLoc/models/mobilenetv2/drone_fp32.engine"):
                        profile.mobilenetv2_engine_path = (
                            "~/DracoViLoc/models/mobilenetv2/drone_fp32_t2000.engine")
                    if item.get("mobilenetv2_venv_python") == (
                            "~/DracoViLoc/trt_env/bin/python3"):
                        profile.mobilenetv2_venv_python = "/usr/bin/python3"
                profiles[profile.name] = profile
        except (OSError, ValueError, TypeError):
            return profiles
        return profiles

    def save_custom(self, profiles: list[RunProfile]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps({
            "version": SCHEMA_VERSION,
            "profiles": [profile.to_dict() for profile in profiles],
        }, indent=2), encoding="utf-8")
        temporary.replace(self.path)

