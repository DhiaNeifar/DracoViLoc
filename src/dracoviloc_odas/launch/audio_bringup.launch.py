import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def detect_uma16_card(cards_path="/proc/asound/cards"):
    """Return the ALSA card index of the UMA16v2, or None if it is absent.

    Card indices depend on enumeration order (the UMA16 is card 0 on one
    machine and 1 or 2 on another), so ODAS is pointed at the detected index
    instead of a hardcoded one.
    """
    try:
        with open(cards_path, encoding="utf-8", errors="ignore") as stream:
            text = stream.read()
    except OSError:
        return None
    for match in re.finditer(r"^\s*(\d+)\s+\[([^\]]*)\]:([^\n]*)", text, re.MULTILINE):
        if re.search(r"uma-?16|minidsp", match.group(2) + match.group(3), re.IGNORECASE):
            return int(match.group(1))
    return None


def _resolve_card(requested):
    if requested != "auto":
        if not requested.isdigit():
            raise RuntimeError("uma16_card must be 'auto' or an ALSA card index")
        return int(requested)
    card = detect_uma16_card()
    if card is None:
        raise RuntimeError(
            "UMA16v2 not found in /proc/asound/cards; connect it or pass uma16_card:=<index>")
    return card


def _configure(context):
    mode = LaunchConfiguration("first_arrival_mode").perform(context)
    if mode not in ("off", "shadow", "enforce"):
        raise RuntimeError("first_arrival_mode must be off, shadow, or enforce")
    microphone_frame = LaunchConfiguration("microphone_frame")
    odas_share = get_package_share_directory("odas_ros")
    base_configuration = os.path.join(odas_share, "config", "configuration.cfg")
    with open(base_configuration, encoding="utf-8") as stream:
        configuration = stream.read()
    actions = []
    if mode == "off":
        card = _resolve_card(LaunchConfiguration("uma16_card").perform(context))
        configuration, replacements = re.subn(
            r'(interface\s*:\s*\{\s*type\s*=\s*"soundcard";\s*card\s*=\s*)\d+',
            rf"\g<1>{card}", configuration, count=1)
        if replacements != 1:
            raise RuntimeError("could not set the ALSA card in the ODAS configuration")
        prefix = "dracoviloc_odas_"
        actions.append(LogInfo(msg=f"ODAS capturing from ALSA card {card} (UMA16v2)"))
    else:
        socket_interface = (
            'interface: {\n'
            '       type = "socket"; ip = "127.0.0.1"; port = 9200;\n'
            '    }')
        configuration, replacements = re.subn(
            r'interface\s*:\s*\{\s*type\s*=\s*"soundcard";.*?\}',
            socket_interface, configuration, count=1, flags=re.DOTALL)
        if replacements != 1:
            raise RuntimeError("could not create ODAS raw-socket configuration")
        prefix = "dracoviloc_odas_first_arrival_"
        actions.extend([
            Node(
                package="dracoviloc_odas", executable="uma16_feeder",
                name="uma16_feeder_node",
                arguments=["--bypass", "--frame-id", microphone_frame],
                output="screen"),
            Node(
                package="dracoviloc_odas", executable="first_arrival_node",
                name="first_arrival_node", output="screen",
                parameters=[{
                    "frame_id": microphone_frame,
                    "min_confidence": ParameterValue(
                        LaunchConfiguration("first_arrival_min_confidence"), value_type=float),
                    "onset_snr_db": ParameterValue(
                        LaunchConfiguration("first_arrival_onset_snr_db"), value_type=float),
                    "noise_window_s": ParameterValue(
                        LaunchConfiguration("first_arrival_noise_window_s"), value_type=float),
                    "max_arrival_span_ms": ParameterValue(
                        LaunchConfiguration("first_arrival_max_span_ms"), value_type=float),
                    "residual_us": ParameterValue(
                        LaunchConfiguration("first_arrival_residual_us"), value_type=float),
                    "min_inliers": ParameterValue(
                        LaunchConfiguration("first_arrival_min_inliers"), value_type=int),
                    "refractory_ms": ParameterValue(
                        LaunchConfiguration("first_arrival_refractory_ms"), value_type=float),
                    "diagnostics_path": LaunchConfiguration(
                        "first_arrival_diagnostics_path"),
                    "raw_recording_path": LaunchConfiguration(
                        "first_arrival_raw_recording_path"),
                }]),
        ])
    descriptor, temporary_configuration = tempfile.mkstemp(prefix=prefix, suffix=".cfg")
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        stream.write(configuration)
    odas = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(odas_share, "launch", "odas.launch.xml")),
        launch_arguments={
            "configuration_path": temporary_configuration,
            "frame_id": microphone_frame,
            "audio_queue_size": "8",
            "visualization": "true",
            "rviz": "false",
            "force_publish_tf": "false",
            "diagnostics_log_path": LaunchConfiguration("diagnostics_log_path"),
            "diagnostics_interval_s": LaunchConfiguration("diagnostics_interval_s"),
        }.items(),
    )
    actions.append(odas)

    def cleanup(_context):
        try:
            os.unlink(temporary_configuration)
        except FileNotFoundError:
            pass
        return []
    actions.append(RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulated time instead of the system clock."),
        DeclareLaunchArgument(
            "microphone_frame", default_value="odas_link",
            description="TF frame in which ODAS microphone directions are expressed."),
        DeclareLaunchArgument(
            "diagnostics_log_path", default_value="",
            description="Optional ODAS JSONL diagnostic log path."),
        DeclareLaunchArgument(
            "diagnostics_interval_s", default_value="10.0",
            description="Seconds between ODAS health snapshots."),
        DeclareLaunchArgument(
            "uma16_card", default_value="auto",
            description="ALSA card index of the UMA16v2, or 'auto' to detect it."),
        DeclareLaunchArgument("first_arrival_mode", default_value="off"),
        DeclareLaunchArgument("first_arrival_min_confidence", default_value="0.70"),
        DeclareLaunchArgument("first_arrival_onset_snr_db", default_value="10.0"),
        DeclareLaunchArgument("first_arrival_noise_window_s", default_value="1.0"),
        DeclareLaunchArgument("first_arrival_max_span_ms", default_value="1.2"),
        DeclareLaunchArgument("first_arrival_residual_us", default_value="150.0"),
        DeclareLaunchArgument("first_arrival_min_inliers", default_value="12"),
        DeclareLaunchArgument("first_arrival_refractory_ms", default_value="50.0"),
        DeclareLaunchArgument("first_arrival_diagnostics_path", default_value=""),
        DeclareLaunchArgument("first_arrival_raw_recording_path", default_value=""),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        OpaqueFunction(function=_configure),
    ])
