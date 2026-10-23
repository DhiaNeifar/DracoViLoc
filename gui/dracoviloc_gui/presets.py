from __future__ import annotations

from .models import RunProfile


def built_in_profiles() -> dict[str, RunProfile]:
    return {
        "Mock Observation": RunProfile(),
        "Audio Observation": RunProfile(
            name="Audio Observation", audio_enabled=True,
            mobilenetv2_enabled=True, tracking_mode="off", recording_enabled=True),
        "YOLO Observation": RunProfile(
            name="YOLO Observation", yolo_enabled=True,
            tracking_mode="off", recording_enabled=True),
        "Multimodal EKF Mock": RunProfile(
            name="Multimodal EKF Mock", audio_enabled=True, yolo_enabled=True,
            mobilenetv2_enabled=True, mobilenetv2_ekf_enabled=True,
            fusion_enabled=True, tracking_mode="ekf", recording_enabled=True),
        "Physical EKF": RunProfile(
            name="Physical EKF", hardware_mode="real", audio_enabled=True,
            yolo_enabled=True, mobilenetv2_enabled=True,
            mobilenetv2_ekf_enabled=True, fusion_enabled=True,
            tracking_mode="ekf", recording_enabled=True,
            ekf_average_window=10, smoothing_alpha=0.10,
            max_velocity=0.60, max_acceleration=0.80, max_jerk=1.0),
    }

