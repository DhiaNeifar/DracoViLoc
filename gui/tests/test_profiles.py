import json

from dracoviloc_gui.models import RunProfile
from dracoviloc_gui.profiles import ProfileStore, SCHEMA_VERSION


def test_missing_profile_file_returns_presets(tmp_path):
    profiles = ProfileStore(tmp_path / "profiles.json").load()
    assert "Mock Observation" in profiles
    assert "Physical EKF" in profiles


def test_custom_profile_round_trip(tmp_path):
    store = ProfileStore(tmp_path / "profiles.json")
    store.save_custom([RunProfile(name="Lab", yolo_enabled=True)])
    profiles = store.load()
    assert profiles["Lab"].yolo_enabled is True


def test_unknown_profile_fields_are_ignored(tmp_path):
    path = tmp_path / "profiles.json"
    path.write_text(json.dumps({
        "version": SCHEMA_VERSION,
        "profiles": [{"name": "Future", "unknown": 123}],
    }))
    assert ProfileStore(path).load()["Future"].name == "Future"


def test_wrong_schema_does_not_load_custom_profiles(tmp_path):
    path = tmp_path / "profiles.json"
    path.write_text(json.dumps({"version": 999, "profiles": [{"name": "Bad"}]}))
    assert "Bad" not in ProfileStore(path).load()

