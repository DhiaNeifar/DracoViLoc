from __future__ import annotations

import json
from pathlib import Path

from .host_platform import AUTO
from .models import RunProfile
from .presets import built_in_profiles


SCHEMA_VERSION = 1
LEGACY_ENGINE_DEFAULTS = {
    "~/DracoViLoc/models/mobilenetv2/drone_fp32.engine",
    "~/DracoViLoc/models/mobilenetv2/drone_fp32_t2000.engine",
}
LEGACY_PYTHON_DEFAULTS = {"~/DracoViLoc/trt_env/bin/python3", "/usr/bin/python3"}


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
                # Former per-machine defaults become "auto", so a saved
                # profile works on whichever machine opens it.
                if item.get("mobilenetv2_engine_path") in LEGACY_ENGINE_DEFAULTS:
                    profile.mobilenetv2_engine_path = AUTO
                if item.get("mobilenetv2_venv_python") in LEGACY_PYTHON_DEFAULTS:
                    profile.mobilenetv2_venv_python = AUTO
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

