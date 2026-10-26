from __future__ import annotations

import json
import re
import subprocess
from dataclasses import dataclass, asdict
from pathlib import Path


@dataclass(slots=True)
class Recording:
    timestamp: str
    kind: str
    path: str
    state: str
    size_bytes: int

    def to_dict(self):
        return asdict(self)


RUN_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}")


def _state(directory: Path) -> str:
    try:
        return json.loads((directory / "metadata.json").read_text(encoding="utf-8")).get("state", "unknown")
    except FileNotFoundError:
        return "missing"
    except (OSError, ValueError):
        return "error"


def _size(directory: Path) -> int:
    return sum(item.stat().st_size for item in directory.rglob("*") if item.is_file())


def discover_recordings(root: Path) -> list[Recording]:
    """One row per run folder runs/<YYYY-MM-DD_HH-MM-SS>/ that holds a recording
    (video/ and/or audio/), plus runs in the earlier runs/{video,audio}/<time>/ layout."""
    recordings: list[Recording] = []
    if not root.exists():
        return recordings
    for run in root.iterdir():
        if not (run.is_dir() and RUN_NAME.match(run.name)):
            continue
        kinds = [kind for kind in ("video", "audio") if (run / kind).is_dir()]
        if not kinds:
            continue
        states = {_state(run / kind) for kind in kinds}
        state = states.pop() if len(states) == 1 else ("recording" if "recording" in states else "partial")
        recordings.append(Recording(run.name, " + ".join(kinds), str(run), state, _size(run)))
    for kind in ("video", "audio"):
        directory = root / kind
        if not directory.is_dir():
            continue
        for run in directory.iterdir():
            if run.is_dir():
                recordings.append(Recording(run.name, kind, str(run), _state(run), _size(run)))
    return sorted(recordings, key=lambda item: item.timestamp, reverse=True)


def drive_sync_status() -> tuple[bool, str]:
    result = subprocess.run(
        ["systemctl", "--user", "is-active", "sync-runs-to-drive.timer"],
        capture_output=True, text=True)
    return result.returncode == 0, (result.stdout or result.stderr).strip()


def trigger_drive_sync() -> tuple[bool, str]:
    result = subprocess.run(
        ["systemctl", "--user", "start", "sync-runs-to-drive.service"],
        capture_output=True, text=True)
    return result.returncode == 0, (result.stdout or result.stderr).strip()

