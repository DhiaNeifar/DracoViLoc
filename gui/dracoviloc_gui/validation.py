from __future__ import annotations

import math
import ipaddress

from .models import RunProfile


TRACKING_MODES = {
    "off", "direct_gre", "direct_ast", "direct_either", "direct_yolo",
    "direct_mobilenetv2", "ekf",
}


def validate_profile(profile: RunProfile) -> list[str]:
    errors: list[str] = []
    if profile.execution_target not in {"local", "remote"}:
        errors.append("Execution target must be local or remote.")
    if profile.execution_target == "remote" and not profile.ssh_alias.strip():
        errors.append("Remote execution requires an SSH host alias.")
    if not 1 <= profile.remote_agent_port <= 65535:
        errors.append("Remote agent port must be between 1 and 65535.")
    if profile.hardware_mode not in {"mock", "real"}:
        errors.append("Hardware mode must be mock or real.")
    if profile.dual_tracking_policy not in {"first_arm_lead", "simultaneous"}:
        errors.append("Dual tracking policy must be first_arm_lead or simultaneous.")
    if profile.dual_arm:
        if profile.left_robot_ip == profile.right_robot_ip:
            errors.append("The two cobots must use different IP addresses.")
        for label, address in (("Left cobot", profile.left_robot_ip),
                               ("Right cobot", profile.right_robot_ip)):
            try:
                ipaddress.ip_address(address)
            except ValueError:
                errors.append(f"{label} IP address is invalid.")
        geometry = (profile.second_arm_x, profile.second_arm_y, profile.second_arm_z,
                    profile.lead_lock_tolerance_rad)
        if not all(math.isfinite(value) for value in geometry):
            errors.append("Dual-cobot geometry values must be finite.")
        if profile.lead_lock_tolerance_rad <= 0:
            errors.append("Lead lock tolerance must be positive.")
    if profile.tracking_mode not in TRACKING_MODES:
        errors.append("Unknown tracking mode.")
    if profile.first_arrival_mode not in {"off", "shadow", "enforce"}:
        errors.append("First-arrival mode must be off, shadow, or enforce.")
    if profile.mobilenetv2_enabled and not profile.audio_enabled:
        errors.append("MobileNetV2 requires audio.")
    if profile.ast_enabled and not profile.audio_enabled:
        errors.append("AST requires audio.")
    if profile.gre_enabled and not profile.audio_enabled:
        errors.append("GRE requires audio.")
    if profile.mobilenetv2_ekf_enabled and not (
        profile.mobilenetv2_enabled and profile.fusion_enabled
    ):
        errors.append("MobileNetV2 EKF input requires MobileNetV2 and fusion.")
    if profile.yolo_enabled and not profile.recording_enabled:
        errors.append("Raw camera recording is mandatory whenever YOLO/camera is enabled.")
    if profile.yolo_enabled and profile.recording_video_topic not in {"/image/compressed", "/image"}:
        errors.append("The camera recording must use the raw camera image (/image/compressed or /image).")
    if profile.recording_enabled and not (profile.audio_enabled or profile.yolo_enabled):
        errors.append("Recording requires audio or YOLO.")
    if profile.first_arrival_mode != "off" and not profile.audio_enabled:
        errors.append("First-arrival processing requires audio.")
    if profile.first_arrival_mode == "enforce" and not (
        profile.fusion_enabled and profile.tracking_mode == "ekf"
    ):
        errors.append("First-arrival enforce mode requires EKF tracking and fusion.")
    direct_requirements = {
        "direct_yolo": profile.yolo_enabled,
        "direct_ast": profile.ast_enabled and profile.audio_enabled,
        "direct_gre": profile.gre_enabled and profile.audio_enabled,
        "direct_mobilenetv2": profile.mobilenetv2_enabled and profile.audio_enabled,
        "direct_either": profile.audio_enabled and (profile.ast_enabled or profile.gre_enabled),
    }
    if profile.tracking_mode in direct_requirements and not direct_requirements[profile.tracking_mode]:
        errors.append(f"{profile.tracking_mode} is missing its required detector.")
    if profile.tracking_mode == "ekf":
        if not profile.fusion_enabled:
            errors.append("EKF tracking requires fusion.")
        if not (profile.yolo_enabled or profile.mobilenetv2_ekf_enabled):
            errors.append("Following the fused direction needs YOLO or MobileNetV2 turned on.")
    for name in ("max_velocity", "max_acceleration", "max_jerk", "command_rate_hz"):
        value = getattr(profile, name)
        if not math.isfinite(value) or value <= 0:
            errors.append(f"{name.replace('_', ' ').title()} must be positive.")
    if not 0.0 <= profile.smoothing_alpha <= 1.0:
        errors.append("Smoothing alpha must be between 0 and 1.")
    return errors

