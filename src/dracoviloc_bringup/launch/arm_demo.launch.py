import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


RAW_CAMERA_TOPICS = ("/image/compressed", "/image")


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("1", "true", "yes", "on")


def _configure_pipeline(context):
    # Each optional stack's share directory is resolved only once its own
    # enabled flag is true, so arm_demo.launch.py can run with any subset of
    # dracoviloc_ast/gre/mobilenetv2/odas/recording/diagnostics installed --
    # e.g. a host that only has YOLO + tracking built doesn't need the audio
    # or recording packages present at all.
    mode = LaunchConfiguration("tracking_mode").perform(context)
    audio_enabled = _enabled(context, "audio_enabled")
    yolo_enabled = _enabled(context, "yolo_enabled")
    fusion_enabled = _enabled(context, "fusion_enabled")
    ast_enabled = _enabled(context, "ast_enabled")
    gre_enabled = _enabled(context, "gre_enabled")
    mobilenetv2_enabled = _enabled(context, "mobilenetv2_enabled")
    mobilenetv2_ekf_enabled = _enabled(context, "mobilenetv2_ekf_enabled")
    recording_enabled = _enabled(context, "recording_enabled")
    sss_channels_recording = _enabled(context, "sss_channels_recording")
    first_arrival_mode = LaunchConfiguration("first_arrival_mode").perform(context)

    if first_arrival_mode not in ("off", "shadow", "enforce"):
        raise RuntimeError("first_arrival_mode must be off, shadow, or enforce")
    if first_arrival_mode != "off" and not audio_enabled:
        raise RuntimeError("first_arrival_mode requires audio_enabled:=true")
    if first_arrival_mode == "enforce" and not (fusion_enabled and mode == "ekf"):
        raise RuntimeError(
            "first_arrival_mode=enforce requires fusion_enabled:=true and tracking_mode:=ekf")

    if mobilenetv2_enabled and not audio_enabled:
        raise RuntimeError("mobilenetv2_enabled requires audio_enabled:=true")
    if mobilenetv2_ekf_enabled and not (mobilenetv2_enabled and fusion_enabled):
        raise RuntimeError(
            "mobilenetv2_ekf_enabled requires mobilenetv2_enabled:=true "
            "and fusion_enabled:=true")

    # The raw camera stream is always recorded (and later synced to Drive)
    # whenever the camera runs; it cannot be switched off.
    forced_recording = yolo_enabled and not recording_enabled
    if yolo_enabled:
        recording_enabled = True
        video_topic = context.launch_configurations.get("recording_video_topic", "/image/compressed")
        if video_topic not in RAW_CAMERA_TOPICS:
            raise RuntimeError(
                "recording_video_topic must be the raw camera image "
                f"({' or '.join(RAW_CAMERA_TOPICS)}, got {video_topic}); "
                "overlay images are never recorded")

    # One folder per run holds everything it produces: video/ and audio/
    # (uploaded to Drive), telemetry.jsonl, bag/, diagnostics/, logs.
    run_dir = context.launch_configurations.get("run_dir", "")
    if not run_dir:
        recording_root = context.launch_configurations.get(
            "recording_root", os.path.expanduser("~/DracoViLoc/runs"))
        run_dir = os.path.join(os.path.expanduser(recording_root),
                               datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    if recording_enabled and not (audio_enabled or yolo_enabled):
        raise RuntimeError(
            "recording_enabled=true requires audio_enabled or yolo_enabled")

    if mode.startswith("direct_"):
        source = mode.removeprefix("direct_")
        if source == "yolo":
            if not yolo_enabled:
                raise RuntimeError("tracking_mode=direct_yolo requires yolo_enabled:=true")
        else:
            if not audio_enabled:
                raise RuntimeError(f"tracking_mode={mode} requires audio_enabled:=true")
            if source == "ast" and not ast_enabled:
                raise RuntimeError(f"tracking_mode={mode} requires ast_enabled:=true")
            if source == "gre" and not gre_enabled:
                raise RuntimeError(f"tracking_mode={mode} requires gre_enabled:=true")
            if source == "mobilenetv2" and not mobilenetv2_enabled:
                raise RuntimeError(
                    "tracking_mode=direct_mobilenetv2 requires mobilenetv2_enabled:=true")
            if source == "either" and not (ast_enabled or gre_enabled):
                raise RuntimeError(f"tracking_mode={mode} requires AST or GRE to be enabled")
    elif mode == "ekf":
        source = "gre"
        if not fusion_enabled:
            raise RuntimeError("tracking_mode=ekf requires fusion_enabled:=true")
        if not (yolo_enabled or mobilenetv2_ekf_enabled):
            raise RuntimeError(
                "tracking_mode=ekf requires yolo_enabled or mobilenetv2_ekf_enabled")
    else:
        source = "gre"

    actions = []
    if forced_recording:
        actions.append(LogInfo(msg=(
            "recording_enabled forced to true: the raw camera stream is always recorded")))
    if audio_enabled:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("dracoviloc_odas"),
                "launch", "audio_bringup.launch.py")),
            launch_arguments={
                "use_sim_time": "false",
                "microphone_frame": "table_mic_link",
                "uma16_card": LaunchConfiguration("uma16_card"),
                "diagnostics_log_path": LaunchConfiguration("odas_diagnostics_log_path"),
                "diagnostics_interval_s": LaunchConfiguration("odas_diagnostics_interval_s"),
                "first_arrival_mode": LaunchConfiguration("first_arrival_mode"),
                "first_arrival_min_confidence": LaunchConfiguration(
                    "first_arrival_min_confidence"),
                "first_arrival_onset_snr_db": LaunchConfiguration(
                    "first_arrival_onset_snr_db"),
                "first_arrival_noise_window_s": LaunchConfiguration(
                    "first_arrival_noise_window_s"),
                "first_arrival_max_span_ms": LaunchConfiguration(
                    "first_arrival_max_span_ms"),
                "first_arrival_residual_us": LaunchConfiguration(
                    "first_arrival_residual_us"),
                "first_arrival_min_inliers": LaunchConfiguration(
                    "first_arrival_min_inliers"),
                "first_arrival_refractory_ms": LaunchConfiguration(
                    "first_arrival_refractory_ms"),
                "first_arrival_diagnostics_path": LaunchConfiguration(
                    "first_arrival_diagnostics_path"),
                "first_arrival_raw_recording_path": LaunchConfiguration(
                    "first_arrival_raw_recording_path"),
            }.items()))
    if recording_enabled:
        actions.append(Node(
            package="dracoviloc_recording",
            executable="multimodal_recorder",
            name="multimodal_recorder",
            output="screen",
            parameters=[{
                "output_directory": run_dir,
                "video_topic": LaunchConfiguration("recording_video_topic"),
                "audio_topic": LaunchConfiguration("recording_audio_topic"),
                "sst_topic": LaunchConfiguration("recording_sst_topic"),
                "require_video": yolo_enabled,
                "require_audio": audio_enabled,
                "require_sst": audio_enabled,
                "video_fps": ParameterValue(
                    LaunchConfiguration("recording_fps"), value_type=float),
                "video_crf": ParameterValue(
                    LaunchConfiguration("recording_crf"), value_type=int),
            }]))
    if sss_channels_recording and audio_enabled:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("dracoviloc_recording"), "launch", "sss_channels.launch.py")),
            launch_arguments={
                "audio_topic": LaunchConfiguration("recording_audio_topic"),
                "output_root": (LaunchConfiguration("sss_channels_root").perform(context)
                                or os.path.join(run_dir, "audio", "channels")),
                "gain_db": LaunchConfiguration("sss_channels_gain_db"),
            }.items()))
    if audio_enabled or yolo_enabled:
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="table_microphone_broadcaster",
            arguments=[
                "--x", LaunchConfiguration("table_mic_x"),
                "--y", LaunchConfiguration("table_mic_y"),
                "--z", LaunchConfiguration("table_mic_z"),
                "--yaw", LaunchConfiguration("table_mic_yaw"),
                "--pitch", LaunchConfiguration("table_mic_pitch"),
                "--roll", LaunchConfiguration("table_mic_roll"),
                "--frame-id", "world",
                "--child-frame-id", "table_mic_link",
            ],
            parameters=[{"use_sim_time": False}]))

    if ast_enabled and audio_enabled:
        actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("dracoviloc_ast"), "launch", "ast.launch.py")),
            launch_arguments={
                "venv_python": PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "DracoViLoc", "trt_env", "bin", "python3"]),
                "model_dir": PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "DracoViLoc", "models", "ast"]),
                "engine_path": PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "DracoViLoc", "models", "ast", "drone_ast.engine"]),
                "threshold": LaunchConfiguration("ast_threshold"),
                "min_activity": LaunchConfiguration("min_activity"),
                "always_classify": LaunchConfiguration("always_classify"),
                "bearing_gate": "false" if mode == "ekf" else
                    LaunchConfiguration("bearing_gate_enabled"),
            }.items())]))
    if gre_enabled and audio_enabled:
        actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("dracoviloc_gre"), "launch", "gre.launch.py")),
            launch_arguments={
                "venv_python": PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "DracoViLoc", "gre_env", "bin", "python3"]),
                "engine_path": PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "DracoViLoc", "models", "gre", "model_logmel.engine"]),
                "meta_path": PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "DracoViLoc", "models", "gre", "model_logmel_meta.json"]),
                "min_activity": LaunchConfiguration("min_activity"),
                "bearing_gate": "false" if mode == "ekf" else
                    LaunchConfiguration("bearing_gate_enabled"),
            }.items())]))
    if mobilenetv2_enabled:
        mobile_arguments = {
            "min_activity": LaunchConfiguration("min_activity"),
            "always_classify": LaunchConfiguration("always_classify"),
            "threshold": LaunchConfiguration("mobilenetv2_threshold"),
            "votes_required": LaunchConfiguration("mobilenetv2_votes_required"),
            "vote_window": LaunchConfiguration("mobilenetv2_vote_window"),
            "bearing_gate": "false" if mode == "ekf" else
                LaunchConfiguration("bearing_gate_enabled"),
        }
        for name in ("engine_path", "venv_python"):
            value = LaunchConfiguration("mobilenetv2_" + name).perform(context)
            if value:
                mobile_arguments[name] = value
        actions.append(GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("dracoviloc_mobilenetv2"), "launch", "mobilenetv2.launch.py")),
            launch_arguments=mobile_arguments.items())]))
    if fusion_enabled:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("dracoviloc_ekf"), "launch", "ekf.launch.py")),
            launch_arguments={"yolo_enabled": "true" if yolo_enabled else "false",
                              "mobilenetv2_ekf_enabled": "true" if mobilenetv2_ekf_enabled else "false",
                              "process_noise": LaunchConfiguration("ekf_process_noise"),
                              "measurement_noise": LaunchConfiguration(
                                  "ekf_measurement_noise"),
                              "yolo_measurement_noise": LaunchConfiguration(
                                  "ekf_yolo_measurement_noise"),
                              "mobilenetv2_measurement_noise": LaunchConfiguration(
                                  "ekf_mobilenetv2_measurement_noise"),
                              "innovation_gate": LaunchConfiguration("ekf_innovation_gate"),
                              "output_average_window": LaunchConfiguration(
                                  "ekf_average_window"),
                              "bearing_gate_enabled": LaunchConfiguration(
                                  "bearing_gate_enabled"),
                              "bearing_gate_base_angle_deg": LaunchConfiguration(
                                  "bearing_gate_base_angle_deg"),
                              "bearing_gate_max_angle_deg": LaunchConfiguration(
                                  "bearing_gate_max_angle_deg"),
                              "bearing_gate_max_rate_deg_s": LaunchConfiguration(
                                  "bearing_gate_max_rate_deg_s"),
                              "bearing_gate_confirmations": LaunchConfiguration(
                                  "bearing_gate_confirmations"),
                              "bearing_gate_confirmation_time_s": LaunchConfiguration(
                                  "bearing_gate_confirmation_time_s"),
                              "bearing_gate_challenger_tolerance_deg": LaunchConfiguration(
                                  "bearing_gate_challenger_tolerance_deg"),
                              "bearing_gate_reacquire_timeout_s": LaunchConfiguration(
                                  "bearing_gate_reacquire_timeout_s"),
                              "first_arrival_mode": LaunchConfiguration(
                                  "first_arrival_mode"),
                              "first_arrival_min_confidence": LaunchConfiguration(
                                  "first_arrival_min_confidence"),
                              "first_arrival_match_angle_deg": LaunchConfiguration(
                                  "first_arrival_match_angle_deg"),
                              "first_arrival_anchor_deviation_deg": LaunchConfiguration(
                                  "first_arrival_anchor_deviation_deg"),
                              "first_arrival_evidence_ttl_s": LaunchConfiguration(
                                  "first_arrival_evidence_ttl_s")}.items()))

    if mobilenetv2_enabled or gre_enabled or fusion_enabled:
        actions.append(Node(
            package="dracoviloc_bringup",
            executable="direction_marker_visualizer",
            parameters=[{
                "mobilenetv2_enabled": mobilenetv2_enabled,
                "gre_enabled": gre_enabled,
                "ekf_enabled": fusion_enabled,
                "first_arrival_enabled":
                    first_arrival_mode != "off",
                "arrow_length": ParameterValue(
                    LaunchConfiguration("direction_marker_length"), value_type=float),
                "marker_timeout": ParameterValue(
                    LaunchConfiguration("direction_marker_timeout"), value_type=float),
            }],
            output="screen"))

    if mode != "off":
        actions.append(Node(
            package="dracoviloc_tracking",
            executable="arm_tracker",
            parameters=[{
                "use_sim_time": False,
                "smoothing_alpha": ParameterValue(
                    LaunchConfiguration("smoothing_alpha"), value_type=float),
                "angular_deadband": ParameterValue(
                    LaunchConfiguration("angular_deadband"), value_type=float),
                "max_velocity": ParameterValue(
                    LaunchConfiguration("max_velocity"), value_type=float),
                "max_acceleration": ParameterValue(
                    LaunchConfiguration("max_acceleration"), value_type=float),
                "max_jerk": ParameterValue(
                    LaunchConfiguration("max_jerk"), value_type=float),
                "max_tracking_error": ParameterValue(
                    LaunchConfiguration("max_tracking_error"), value_type=float),
                "command_rate_hz": ParameterValue(
                    LaunchConfiguration("command_rate_hz"), value_type=float),
                "ekf_direction_log_path": LaunchConfiguration(
                    "ekf_direction_log_path"),
                "yolo_direction_log_path": LaunchConfiguration(
                    "yolo_direction_log_path"),
                "servo_log_path": LaunchConfiguration("servo_log_path"),
                "ekf_enabled": mode == "ekf",
                "direct_classifier_source": source,
                "direct_min_activity": ParameterValue(
                    LaunchConfiguration("min_activity"), value_type=float),
                "require_home_before_tracking": ParameterValue(
                    LaunchConfiguration("require_home_before_tracking"), value_type=bool),
                "home_duration_s": ParameterValue(
                    LaunchConfiguration("home_duration_s"), value_type=float),
                "dual_arm": ParameterValue(
                    LaunchConfiguration("dual_arm"), value_type=bool),
                "dual_tracking_policy": LaunchConfiguration("dual_tracking_policy"),
                "lead_lock_tolerance_rad": ParameterValue(
                    LaunchConfiguration("lead_lock_tolerance_rad"), value_type=float),
            }],
            output="screen"))

    if context.launch_configurations.get("monitor_enabled", "true").lower() == "true":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("dracoviloc_monitor"), "launch", "monitor.launch.py")),
            launch_arguments={"log_dir": run_dir}.items()))
    if _enabled(context, "diagnostics_enabled"):
        diagnostics_dir = os.path.join(run_dir, "diagnostics")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("dracoviloc_diagnostics"), "launch", "diagnostics.launch.py")),
            launch_arguments={
                "ee_frame": LaunchConfiguration("diagnostics_ee_frame"),
                "smoothing_alpha": LaunchConfiguration("diagnostics_smoothing_alpha"),
                "events_log_path": os.path.join(diagnostics_dir, "events.csv"),
                "ee_kinematics_log_path": os.path.join(diagnostics_dir, "ee_kinematics.csv"),
                "live_plot": LaunchConfiguration("diagnostics_live_plot"),
            }.items()))
    return actions


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    hardware_mode = LaunchConfiguration("hardware_mode")
    robot_ip = LaunchConfiguration("robot_ip")
    bringup_share = get_package_share_directory("dracoviloc_bringup")

    arm_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "demo.launch.py")),
        launch_arguments={
            "dual_arm": LaunchConfiguration("dual_arm"),
            "hardware_mode": hardware_mode,
            "robot_ip": robot_ip,
            "left_robot_ip": LaunchConfiguration("left_robot_ip"),
            "right_robot_ip": LaunchConfiguration("right_robot_ip"),
            "second_arm_x": LaunchConfiguration("second_arm_x"),
            "second_arm_y": LaunchConfiguration("second_arm_y"),
            "second_arm_z": LaunchConfiguration("second_arm_z"),
            "use_rviz": use_rviz,
            "use_moveit": LaunchConfiguration("use_moveit"),
        }.items())

    return LaunchDescription([
        DeclareLaunchArgument("dual_arm", default_value="false",
                              description="Configure and launch dual cobot arms."),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.58.2",
                              description="IP address for Arm 1 (left)."),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.58.3",
                              description="IP address for Arm 2 (right)."),
        DeclareLaunchArgument("second_arm_x", default_value="-0.75",
                              description="Arm 2 separation distance along X axis in metres (-0.75 for 75cm to the left of Arm 1)."),
        DeclareLaunchArgument("second_arm_y", default_value="0.0",
                              description="Arm 2 separation distance along Y axis in metres."),
        DeclareLaunchArgument("second_arm_z", default_value="0.0",
                              description="Arm 2 separation distance along Z axis in metres."),
        DeclareLaunchArgument("dual_tracking_policy", default_value="first_arm_lead",
                              choices=["first_arm_lead", "simultaneous"],
                              description="Dual-arm co-tracking policy."),
        DeclareLaunchArgument("lead_lock_tolerance_rad", default_value="0.10",
                              description="Arm 1 alignment tolerance before Arm 2 tracks in first_arm_lead policy."),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "use_moveit", default_value="false",
            description="Launch MoveGroup planning services. Tracking and RViz do not require it."),
        DeclareLaunchArgument("hardware_mode", default_value="mock",
                              choices=["mock", "real"],
                              description="Use mock joints or the physical FAIRINO arm."),
        DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
        DeclareLaunchArgument(
            "audio_enabled", default_value="false",
            description="Launch fixed-table UMA16v2/ODAS localization."),
        DeclareLaunchArgument(
            "odas_diagnostics_log_path", default_value="",
            description="Optional JSONL path for ODAS core, socket, and flow diagnostics."),
        DeclareLaunchArgument(
            "odas_diagnostics_interval_s", default_value="10.0",
            description="Seconds between ODAS health snapshots."),
        DeclareLaunchArgument(
            "tracking_mode", default_value="off",
            choices=["off", "direct_gre", "direct_ast", "direct_either",
                     "direct_yolo", "direct_mobilenetv2", "ekf"],
            description="Arm target source and filtering mode."),
        DeclareLaunchArgument(
            "fusion_enabled", default_value="false",
            description="Launch dracoviloc_ekf using the enabled detector sources."),
        DeclareLaunchArgument(
            "ast_enabled", default_value="false",
            description="Launch the independent AST classifier package."),
        DeclareLaunchArgument(
            "gre_enabled", default_value="false",
            description="Launch the independent GRE classifier package."),
        DeclareLaunchArgument(
            "mobilenetv2_enabled", default_value="false",
            description="Launch the laptop-compatible MobileNetV2 audio classifier."),
        DeclareLaunchArgument(
            "mobilenetv2_ekf_enabled", default_value="false",
            description="Include MobileNetV2 directions in the EKF."),
        DeclareLaunchArgument("mobilenetv2_threshold", default_value="0.75"),
        DeclareLaunchArgument("mobilenetv2_votes_required", default_value="2"),
        DeclareLaunchArgument("mobilenetv2_vote_window", default_value="2"),
        DeclareLaunchArgument("mobilenetv2_engine_path", default_value=""),
        DeclareLaunchArgument(
            "mobilenetv2_venv_python", default_value="",
            description="Python interpreter with TensorRT and PyCUDA installed."),
        DeclareLaunchArgument(
            "yolo_enabled", default_value="false",
            description="Consume externally published YOLO directions in "
                        "the EKF when fusion_enabled is true. Isaac ROS is "
                        "launched separately inside its container."),
        DeclareLaunchArgument(
            "recording_enabled", default_value="true",
            description="Continuously record ROS video and separated audio. "
                        "On by default: every run should leave a reviewable "
                        "recording. Requires audio_enabled or yolo_enabled; "
                        "pass recording_enabled:=false to opt out."),
        DeclareLaunchArgument(
            "recording_root",
            default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "DracoViLoc", "runs"]),
            description="Each run without run_dir is written to <recording_root>/<time>/."),
        DeclareLaunchArgument(
            "run_dir", default_value="",
            description="This run's folder (video/, audio/, telemetry.jsonl, bag/, "
                        "diagnostics/). Empty: <recording_root>/<YYYY-MM-DD_HH-MM-SS>."),
        DeclareLaunchArgument(
            "recording_video_topic", default_value="/image/compressed",
            description="Raw camera feed, never the YOLO-annotated view. "
                        "/image/compressed (the camera's high-quality JPEG stream) "
                        "arrives complete at full rate; /image (uncompressed) may drop "
                        "frames over the UDP transport."),
        DeclareLaunchArgument(
            "recording_audio_topic", default_value="/sss"),
        DeclareLaunchArgument(
            "recording_sst_topic", default_value="/sst"),
        DeclareLaunchArgument("recording_fps", default_value="15.0"),
        DeclareLaunchArgument("recording_crf", default_value="23"),
        DeclareLaunchArgument(
            "sss_channels_recording", default_value="false",
            description="Record /sss into one WAV file per channel under "
                        "sss_channels_root (requires audio_enabled)."),
        DeclareLaunchArgument(
            "sss_channels_root", default_value="",
            description="Empty: <run_dir>/audio/channels (uploaded with the audio)."),
        DeclareLaunchArgument(
            "sss_channels_gain_db", default_value="0.0",
            description="Digital gain in dB for the per-channel WAV recorder; "
                        "e.g. 24.0 makes quiet /sss sources loud (with clipping "
                        "at 0 dBFS if overdone)."),
        DeclareLaunchArgument(
            "ast_threshold", default_value="0.20",
            description="AST drone-probability threshold."),
        DeclareLaunchArgument(
            "min_activity", default_value="0.10",
            description="Shared ODAS activity threshold for classification "
                        "and direct arm tracking."),
        DeclareLaunchArgument(
            "always_classify", default_value="false",
            description="Forwarded to the AST package. true: "
                        "AST classifies continuously regardless of ODAS "
                        "track state, for validating the model independent "
                        "of SSL/SST tuning."),
        DeclareLaunchArgument("smoothing_alpha", default_value="0.50"),
        DeclareLaunchArgument(
            "angular_deadband", default_value="0.003",
            description="Minimum change in the solved aim point, in radians, "
                        "before the commanded target is updated. Ruckig tracks "
                        "continuously, so this only suppresses noise-driven "
                        "micro-retargeting, not a start/stop threshold."),
        DeclareLaunchArgument("max_velocity", default_value="2.50"),
        DeclareLaunchArgument("max_acceleration", default_value="12.0"),
        DeclareLaunchArgument(
            "max_jerk", default_value="80.0",
            description="Maximum tracking jerk in rad/s^3."),
        DeclareLaunchArgument(
            "max_tracking_error", default_value="0.50",
            description="Stop tracking if measured and generated joint positions differ by this many radians."),
        DeclareLaunchArgument(
            "command_rate_hz", default_value="100.0",
            description="Jerk-limited position command rate; match controller_manager update_rate."),
        DeclareLaunchArgument(
            "ekf_direction_log_path", default_value="",
            description="Optional CSV file receiving every raw and filtered EKF direction."),
        DeclareLaunchArgument(
            "yolo_direction_log_path", default_value="",
            description="Optional CSV file receiving every raw YOLO direction."),
        DeclareLaunchArgument(
            "servo_log_path", default_value="",
            description="Optional CSV file receiving every tracker servo tick, "
                        "for diagnosing overshoot/settling behavior."),
        DeclareLaunchArgument(
            "ekf_process_noise", default_value="0.05",
            description="EKF process-noise covariance."),
        DeclareLaunchArgument(
            "ekf_measurement_noise", default_value="0.02",
            description="Fallback EKF direction measurement-noise covariance."),
        DeclareLaunchArgument(
            "ekf_yolo_measurement_noise", default_value="0.03",
            description="YOLO direction measurement-noise covariance."),
        DeclareLaunchArgument(
            "ekf_mobilenetv2_measurement_noise", default_value="0.15",
            description="MobileNetV2/ODAS direction measurement-noise covariance."),
        DeclareLaunchArgument(
            "ekf_innovation_gate", default_value="11.34",
            description="EKF squared Mahalanobis innovation gate."),
        DeclareLaunchArgument(
            "ekf_average_window", default_value="5",
            description="Number of accepted EKF estimates averaged before publication."),
        DeclareLaunchArgument(
            "bearing_gate_enabled", default_value="true",
            description="Enable bearing continuity filtering. In EKF mode the adaptive "
                        "gate runs inside the EKF; direct modes use the classifier gate."),
        DeclareLaunchArgument("bearing_gate_base_angle_deg", default_value="45.0"),
        DeclareLaunchArgument("bearing_gate_max_angle_deg", default_value="70.0"),
        DeclareLaunchArgument("bearing_gate_max_rate_deg_s", default_value="20.0"),
        DeclareLaunchArgument("bearing_gate_confirmations", default_value="5"),
        DeclareLaunchArgument("bearing_gate_confirmation_time_s", default_value="2.0"),
        DeclareLaunchArgument(
            "bearing_gate_challenger_tolerance_deg", default_value="15.0"),
        DeclareLaunchArgument("bearing_gate_reacquire_timeout_s", default_value="10.0"),
        DeclareLaunchArgument(
            "uma16_card", default_value="auto",
            description="ALSA card index of the UMA16v2, or 'auto' to detect it."),
        DeclareLaunchArgument(
            "first_arrival_mode", default_value="off",
            description="First-arrival gating: off, shadow, or enforce."),
        DeclareLaunchArgument("first_arrival_min_confidence", default_value="0.70"),
        DeclareLaunchArgument("first_arrival_match_angle_deg", default_value="20.0"),
        DeclareLaunchArgument(
            "first_arrival_anchor_deviation_deg", default_value="25.0"),
        DeclareLaunchArgument("first_arrival_evidence_ttl_s", default_value="2.0"),
        DeclareLaunchArgument("first_arrival_onset_snr_db", default_value="10.0"),
        DeclareLaunchArgument("first_arrival_noise_window_s", default_value="1.0"),
        DeclareLaunchArgument("first_arrival_max_span_ms", default_value="1.2"),
        DeclareLaunchArgument("first_arrival_residual_us", default_value="150.0"),
        DeclareLaunchArgument("first_arrival_min_inliers", default_value="12"),
        DeclareLaunchArgument("first_arrival_refractory_ms", default_value="50.0"),
        DeclareLaunchArgument("first_arrival_diagnostics_path", default_value=""),
        DeclareLaunchArgument(
            "first_arrival_raw_recording_path", default_value="",
            description="Optional 16-channel signed-32 WAV used to tune first-arrival gating."),
        DeclareLaunchArgument(
            "direction_marker_length", default_value="1.0",
            description="Length in metres of MobileNetV2, GRE, and EKF RViz arrows."),
        DeclareLaunchArgument(
            "direction_marker_timeout", default_value="1.25",
            description="Remove a prediction arrow after this many seconds without an update."),
        DeclareLaunchArgument(
            "require_home_before_tracking", default_value="true",
            description="Require /demo/home to reach the SRDF home pose before tracking can start."),
        DeclareLaunchArgument(
            "home_duration_s", default_value="12.0",
            description="Duration of the explicit SRDF home trajectory in seconds."),
        DeclareLaunchArgument("table_mic_x", default_value="0.0"),
        DeclareLaunchArgument("table_mic_y", default_value="0.0"),
        DeclareLaunchArgument("table_mic_z", default_value="0.75"),
        DeclareLaunchArgument("table_mic_yaw", default_value="3.1415926535897"),
        DeclareLaunchArgument("table_mic_pitch", default_value="0.0"),
        DeclareLaunchArgument("table_mic_roll", default_value="1.57079632679"),
        DeclareLaunchArgument(
            "monitor_enabled", default_value="true",
            description="Video previews and telemetry for the control center (dracoviloc_monitor)."),
        DeclareLaunchArgument(
            "diagnostics_enabled", default_value="true",
            description="Log yolo/mobilenetv2/ekf timing events and "
                        "end-effector position/speed/acceleration/jerk on "
                        "every run, under <run_dir>/diagnostics/."),
        DeclareLaunchArgument(
            "diagnostics_ee_frame", default_value="wrist3_link",
            description="TF frame treated as the end effector "
                        "(wrist3_link, micarray_camera_link, or odas_link)."),
        DeclareLaunchArgument(
            "diagnostics_smoothing_alpha", default_value="0.3",
            description="Exponential smoothing on the end-effector "
                        "velocity/acceleration/jerk derived from TF; "
                        "1.0 disables it."),
        DeclareLaunchArgument(
            "diagnostics_live_plot", default_value="false",
            description="Open the live end-effector speed/acceleration + "
                        "event-timeline plot (requires a display)."),
        arm_demo,
        OpaqueFunction(function=_configure_pipeline),
    ])
