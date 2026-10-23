from __future__ import annotations

import json
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


def discover_recordings(root: Path) -> list[Recording]:
    recordings: list[Recording] = []
    for kind in ("video", "audio"):
        directory = root / kind
        if not directory.exists():
            continue
        for run in directory.iterdir():
            if not run.is_dir():
                continue
            state = "unknown"
            metadata = run / "metadata.json"
            if metadata.exists():
                try:
                    state = json.loads(metadata.read_text(encoding="utf-8")).get("state", "unknown")
                except (OSError, ValueError):
                    state = "error"
            size = sum(item.stat().st_size for item in run.rglob("*") if item.is_file())
            recordings.append(Recording(run.name, kind, str(run), state, size))
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

