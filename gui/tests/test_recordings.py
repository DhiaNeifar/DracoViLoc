import json

from dracoviloc_gui.recordings import discover_recordings


def make(directory, state):
    directory.mkdir(parents=True)
    (directory / "metadata.json").write_text(json.dumps({"state": state}))


def test_one_row_per_run_folder(tmp_path):
    run = tmp_path / "2026-10-26_10-00-00"
    make(run / "video", "complete")
    make(run / "audio", "complete")
    (run / "video/camera.mp4").write_bytes(b"video")
    (run / "telemetry.jsonl").write_text("{}")
    make(tmp_path / "2026-10-26_11-00-00/video", "recording")
    (tmp_path / "2026-10-26_12-00-00").mkdir()  # no recording: not listed
    (tmp_path / "logs").mkdir()  # not a run
    rows = discover_recordings(tmp_path)
    assert [row.timestamp for row in rows] == ["2026-10-26_11-00-00", "2026-10-26_10-00-00"]
    assert rows[1].kind == "video + audio" and rows[1].state == "complete"
    assert rows[0].state == "recording"


def test_earlier_layout_is_still_listed(tmp_path):
    make(tmp_path / "video/2026_10_22_12_00_00", "complete")
    rows = discover_recordings(tmp_path)
    assert len(rows) == 1 and rows[0].kind == "video"
