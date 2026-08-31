import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("1", "true", "yes", "on")


def _configure_pipeline(context, fusion_share):
    mode = LaunchConfiguration("tracking_mode").perform(context)
    audio_enabled = _enabled(context, "audio_enabled")
    visual_enabled = _enabled(context, "visual_enabled")
    fusion_enabled = _enabled(context, "fusion_enabled")
    ast_enabled = _enabled(context, "ast_enabled")
    gre_enabled = _enabled(context, "gre_enabled")

    if mode.startswith("direct_"):
        source = mode.removeprefix("direct_")
        if source != "yolo":
            if not audio_enabled:
                raise RuntimeError(f"tracking_mode={mode} requires audio_enabled:=true")
            if not fusion_enabled:
                raise RuntimeError(f"tracking_mode={mode} requires fusion_enabled:=true")
            if source == "ast" and not ast_enabled:
                raise RuntimeError(f"tracking_mode={mode} requires ast_enabled:=true")
            if source == "gre" and not gre_enabled:
                raise RuntimeError(f"tracking_mode={mode} requires gre_enabled:=true")
            if source == "either" and not (ast_enabled or gre_enabled):
                raise RuntimeError(f"tracking_mode={mode} requires AST or GRE to be enabled")
    elif mode == "ekf":
        source = "gre"
        if not fusion_enabled:
            raise RuntimeError("tracking_mode=ekf requires fusion_enabled:=true")
        if not (audio_enabled or visual_enabled):
            raise RuntimeError("tracking_mode=ekf requires audio_enabled or visual_enabled")
    else:
        source = "gre"

    actions = []
    if audio_enabled or mode == "direct_yolo":
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
            parameters=[{"use_sim_time": True}]))

    if fusion_enabled and (audio_enabled or visual_enabled):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                fusion_share, "launch", "classification_fusion.launch.py")),
            launch_arguments={
                "tracking_frame": "table_mic_link",
                "min_confidence": LaunchConfiguration("min_confidence"),
                "gre_trust": LaunchConfiguration("gre_trust"),
                "threshold": LaunchConfiguration("ast_threshold"),
                "min_activity": LaunchConfiguration("min_activity"),
                "ast_enabled": LaunchConfiguration("ast_enabled"),
                "gre_enabled": LaunchConfiguration("gre_enabled"),
                "always_classify": LaunchConfiguration("always_classify"),
                "audio_enabled": LaunchConfiguration("audio_enabled"),
                "visual_enabled": LaunchConfiguration("visual_enabled"),
                "ekf_enabled": "true" if mode == "ekf" else "false",
            }.items()))

    if mode != "off":
        actions.append(Node(
            package="dracoviloc_tracking",
            executable="arm_audio_tracker",
            parameters=[{
                "use_sim_time": True,
                "smoothing_alpha": ParameterValue(
                    LaunchConfiguration("smoothing_alpha"), value_type=float),
                "max_velocity": ParameterValue(
                    LaunchConfiguration("max_velocity"), value_type=float),
                "max_acceleration": ParameterValue(
                    LaunchConfiguration("max_acceleration"), value_type=float),
                "ekf_enabled": mode == "ekf",
                "direct_classifier_source": source,
                "direct_min_activity": ParameterValue(
                    LaunchConfiguration("min_activity"), value_type=float),
            }],
            output="screen"))
    return actions


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")
    audio_enabled = LaunchConfiguration("audio_enabled")
    bringup_share = get_package_share_directory("dracoviloc_bringup")
    audio_share = get_package_share_directory("dracoviloc_odas")
    fusion_share = get_package_share_directory("dracoviloc_audio_fusion")

    arm_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "demo.launch.py")),
        launch_arguments={
            "sim": "true",
            "use_gui": use_gui,
            "use_rviz": use_rviz,
            "use_moveit": "true",
        }.items())

    audio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(audio_share, "launch", "audio_bringup.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "microphone_frame": "table_mic_link",
        }.items(),
        condition=IfCondition(audio_enabled))

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument(
            "audio_enabled", default_value="false",
            description="Launch fixed-table UMA16v2/ODAS localization."),
        DeclareLaunchArgument(
            "tracking_mode", default_value="off",
            choices=["off", "direct_gre", "direct_ast", "direct_either",
                     "direct_yolo", "ekf"],
            description="Arm target source and filtering mode."),
        DeclareLaunchArgument(
            "fusion_enabled", default_value="true",
            description="Launch audio classification and, when enabled, EKF fusion."),
        DeclareLaunchArgument(
            "ast_enabled", default_value="true",
            description="Forwarded to classification_fusion.launch.py."),
        DeclareLaunchArgument(
            "gre_enabled", default_value="false",
            description="Forwarded to classification_fusion.launch.py. AST "
                        "and GRE are independent - either, both, or neither "
                        "can run."),
        DeclareLaunchArgument(
            "visual_enabled", default_value="false",
            description="Consume externally published YOLO directions in "
                        "the EKF when fusion_enabled is true. Isaac ROS is "
                        "launched separately inside its container."),
        DeclareLaunchArgument(
            "min_confidence", default_value="0.20",
            description="Forwarded to classification_fusion.launch.py."),
        DeclareLaunchArgument(
            "ast_threshold", default_value="0.20",
            description="AST drone-probability threshold."),
        DeclareLaunchArgument(
            "min_activity", default_value="0.10",
            description="Shared ODAS activity threshold for classification "
                        "and direct arm tracking."),
        DeclareLaunchArgument(
            "gre_trust", default_value="0.0",
            description="Forwarded to classification_fusion.launch.py. Only "
                        "matters when both ast_enabled and gre_enabled are "
                        "true; 0.0 = AST decides alone even then."),
        DeclareLaunchArgument(
            "always_classify", default_value="false",
            description="Forwarded to classification_fusion.launch.py. true: "
                        "AST classifies continuously regardless of ODAS "
                        "track state, for validating the model independent "
                        "of SSL/SST tuning."),
        DeclareLaunchArgument("smoothing_alpha", default_value="0.20"),
        DeclareLaunchArgument("max_velocity", default_value="0.60"),
        DeclareLaunchArgument("max_acceleration", default_value="0.80"),
        DeclareLaunchArgument("table_mic_x", default_value="0.0"),
        DeclareLaunchArgument("table_mic_y", default_value="0.0"),
        DeclareLaunchArgument("table_mic_z", default_value="0.75"),
        DeclareLaunchArgument("table_mic_yaw", default_value="3.1415926535897"),
        DeclareLaunchArgument("table_mic_pitch", default_value="0.0"),
        DeclareLaunchArgument("table_mic_roll", default_value="1.57079632679"),
        arm_demo,
        audio,
        OpaqueFunction(
            function=_configure_pipeline,
            kwargs={"fusion_share": fusion_share}),
    ])
