import json

from dracoviloc_gui.recordings import discover_recordings


def test_discovers_split_video_and_audio_runs(tmp_path):
    video = tmp_path / "video/2026_10_22_12_00_00"
    audio = tmp_path / "audio/2026_10_22_12_00_00"
    video.mkdir(parents=True)
    audio.mkdir(parents=True)
    (video / "metadata.json").write_text(json.dumps({"state": "complete"}))
    (video / "camera.mp4").write_bytes(b"video")
    (audio / "metadata.json").write_text(json.dumps({"state": "recording"}))
    rows = discover_recordings(tmp_path)
    assert len(rows) == 2
    assert {row.kind for row in rows} == {"video", "audio"}
    assert next(row for row in rows if row.kind == "video").state == "complete"

