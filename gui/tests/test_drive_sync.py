"""scripts/sync_runs_to_drive.sh uploads only complete video/ and audio/ folders."""
import json
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def make(directory, state):
    directory.mkdir(parents=True)
    (directory / "metadata.json").write_text(json.dumps({"state": state}, indent=2).replace(":", ": ", 0))


def test_sync_uploads_recordings_only(tmp_path):
    runs = tmp_path / "runs"
    make(runs / "2026-10-26_10-00-00/video", "complete")
    make(runs / "2026-10-26_10-00-00/audio", "complete")
    (runs / "2026-10-26_10-00-00/telemetry.jsonl").write_text("{}")
    make(runs / "2026-10-26_11-00-00/video", "recording")       # still recording: skipped
    make(runs / "video/2026_10_22_12_00_00", "complete")        # earlier layout
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    calls = tmp_path / "rclone_calls"
    (bin_dir / "rclone").write_text(
        f"#!/bin/bash\nif [ \"$1\" = listremotes ]; then echo gdrive:; exit 0; fi\n"
        f"echo \"$@\" >> {calls}\n")
    (bin_dir / "rclone").chmod(0o755)
    env = dict(os.environ, PATH=f"{bin_dir}:{os.environ['PATH']}",
               DRIVE_SYNC_RUNS_ROOT=str(runs), DRIVE_SYNC_CONFIG=str(tmp_path / "none.env"))
    result = subprocess.run([str(ROOT / "scripts/sync_runs_to_drive.sh")], env=env,
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    uploads = sorted(line.split()[2] for line in calls.read_text().splitlines())
    assert uploads == [
        "gdrive:DracoViLoc/runs/2026-10-26_10-00-00/audio",
        "gdrive:DracoViLoc/runs/2026-10-26_10-00-00/video",
        "gdrive:DracoViLoc/runs/video/2026_10_22_12_00_00",
    ]
