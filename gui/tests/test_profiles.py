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


def test_remote_profile_round_trip(tmp_path):
    store = ProfileStore(tmp_path / "profiles.json")
    store.save_custom([RunProfile(
        name="Jetson", execution_target="remote", ssh_alias="lab-jetson",
        jetson_project_root="/opt/DracoViLoc")])
    loaded = store.load()["Jetson"]
    assert loaded.execution_target == "remote"
    assert loaded.ssh_alias == "lab-jetson"


def test_dual_cobot_profile_round_trip(tmp_path):
    store = ProfileStore(tmp_path / "profiles.json")
    store.save_custom([RunProfile(
        name="Two arms", dual_arm=True, right_robot_ip="10.0.0.3",
        second_arm_x=-0.9, dual_tracking_policy="simultaneous")])
    loaded = store.load()["Two arms"]
    assert loaded.dual_arm is True
    assert loaded.right_robot_ip == "10.0.0.3"
    assert loaded.second_arm_x == -0.9
    assert loaded.dual_tracking_policy == "simultaneous"


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


def test_old_local_mobilenet_defaults_are_migrated(tmp_path):
    path = tmp_path / "profiles.json"
    path.write_text(json.dumps({
        "version": SCHEMA_VERSION,
        "profiles": [{
            "name": "Old local",
            "execution_target": "local",
            "mobilenetv2_engine_path":
                "~/DracoViLoc/models/mobilenetv2/drone_fp32.engine",
            "mobilenetv2_venv_python": "~/DracoViLoc/trt_env/bin/python3",
        }],
    }))
    profile = ProfileStore(path).load()["Old local"]
    assert profile.mobilenetv2_engine_path == "auto"
    assert profile.mobilenetv2_venv_python == "auto"


def test_audio_off_disables_audio_consumers_and_recording():
    profile = RunProfile(
        audio_enabled=False, mobilenetv2_enabled=True, ast_enabled=True,
        gre_enabled=True, first_arrival_mode="enforce",
        tracking_mode="direct_mobilenetv2", recording_enabled=True).apply_rules()
    assert profile.audio_enabled is False
    assert profile.mobilenetv2_enabled is False
    assert profile.mobilenetv2_ekf_enabled is False
    assert profile.ast_enabled is False
    assert profile.gre_enabled is False
    assert profile.first_arrival_mode == "off"
    assert profile.tracking_mode == "off"
    assert profile.recording_enabled is False


def test_audio_off_keeps_camera_pipeline_and_recording():
    profile = RunProfile(
        audio_enabled=False, yolo_enabled=True, mobilenetv2_enabled=True,
        tracking_mode="ekf", recording_enabled=False).apply_rules()
    assert profile.audio_enabled is False
    assert profile.mobilenetv2_enabled is False
    assert profile.tracking_mode == "ekf"
    assert profile.recording_enabled is True

