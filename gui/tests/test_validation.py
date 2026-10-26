from dracoviloc_gui.models import RunProfile
from dracoviloc_gui.validation import validate_profile


def test_safe_default_is_valid():
    assert validate_profile(RunProfile()) == []


def test_remote_requires_ssh_alias():
    errors = validate_profile(RunProfile(execution_target="remote", ssh_alias=""))
    assert "Remote execution requires an SSH host alias." in errors


def test_recording_without_source_is_rejected():
    profile = RunProfile(
        audio_enabled=False, mobilenetv2_enabled=False,
        mobilenetv2_ekf_enabled=False, recording_enabled=True)
    assert "Recording requires audio or YOLO." in validate_profile(profile)


def test_direct_yolo_requires_yolo():
    profile = RunProfile(tracking_mode="direct_yolo")
    assert any("direct_yolo" in error for error in validate_profile(profile))


def test_ekf_requires_fusion_and_source():
    errors = validate_profile(RunProfile(
        audio_enabled=False, mobilenetv2_enabled=False,
        mobilenetv2_ekf_enabled=False, fusion_enabled=False,
        tracking_mode="ekf"))
    assert "EKF tracking requires fusion." in errors
    assert "Following the fused direction needs YOLO or MobileNetV2 turned on." in errors


def test_mobile_ekf_dependency_chain():
    profile = RunProfile(
        mobilenetv2_enabled=False, mobilenetv2_ekf_enabled=True,
        fusion_enabled=False)
    assert "MobileNetV2 EKF input requires MobileNetV2 and fusion." in validate_profile(profile)


def test_physical_multimodal_profile_is_valid():
    profile = RunProfile(
        hardware_mode="real", audio_enabled=True, yolo_enabled=True,
        mobilenetv2_enabled=True, mobilenetv2_ekf_enabled=True,
        fusion_enabled=True, tracking_mode="ekf", recording_enabled=True)
    assert validate_profile(profile) == []


def test_dual_cobot_profile_requires_distinct_valid_addresses():
    profile = RunProfile(dual_arm=True, left_robot_ip="192.168.58.2",
                         right_robot_ip="192.168.58.2")
    assert "The two cobots must use different IP addresses." in validate_profile(profile)
    profile.right_robot_ip = "not-an-ip"
    assert "Right cobot IP address is invalid." in validate_profile(profile)


def test_dual_cobot_coordination_values_are_validated():
    profile = RunProfile(dual_arm=True,
                         lead_lock_tolerance_rad=0,
                         dual_tracking_policy="unknown")
    errors = validate_profile(profile)
    assert "Lead lock tolerance must be positive." in errors
    assert any("Dual tracking policy" in error for error in errors)

