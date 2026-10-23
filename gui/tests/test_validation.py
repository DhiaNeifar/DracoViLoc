from dracoviloc_gui.models import RunProfile
from dracoviloc_gui.validation import validate_profile


def test_safe_default_is_valid():
    assert validate_profile(RunProfile()) == []


def test_recording_without_source_is_rejected():
    profile = RunProfile(recording_enabled=True)
    assert "Recording requires audio or YOLO." in validate_profile(profile)


def test_direct_yolo_requires_yolo():
    profile = RunProfile(tracking_mode="direct_yolo")
    assert any("direct_yolo" in error for error in validate_profile(profile))


def test_ekf_requires_fusion_and_source():
    errors = validate_profile(RunProfile(tracking_mode="ekf"))
    assert "EKF tracking requires fusion." in errors
    assert "EKF tracking requires audio or YOLO." in errors


def test_mobile_ekf_dependency_chain():
    profile = RunProfile(mobilenetv2_ekf_enabled=True)
    assert "MobileNetV2 EKF input requires MobileNetV2 and fusion." in validate_profile(profile)


def test_physical_multimodal_profile_is_valid():
    profile = RunProfile(
        hardware_mode="real", audio_enabled=True, yolo_enabled=True,
        mobilenetv2_enabled=True, mobilenetv2_ekf_enabled=True,
        fusion_enabled=True, tracking_mode="ekf", recording_enabled=True)
    assert validate_profile(profile) == []

