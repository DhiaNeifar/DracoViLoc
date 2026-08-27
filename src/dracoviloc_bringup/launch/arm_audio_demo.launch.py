import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")
    audio_enabled = LaunchConfiguration("audio_enabled")
    audio_tracking_enabled = LaunchConfiguration("audio_tracking_enabled")
    fusion_enabled = LaunchConfiguration("fusion_enabled")
    table_mic_x = LaunchConfiguration("table_mic_x")
    table_mic_y = LaunchConfiguration("table_mic_y")
    table_mic_z = LaunchConfiguration("table_mic_z")
    table_mic_yaw = LaunchConfiguration("table_mic_yaw")
    table_mic_pitch = LaunchConfiguration("table_mic_pitch")
    table_mic_roll = LaunchConfiguration("table_mic_roll")
    min_confidence = LaunchConfiguration("min_confidence")
    gre_trust = LaunchConfiguration("gre_trust")
    ast_enabled = LaunchConfiguration("ast_enabled")
    gre_enabled = LaunchConfiguration("gre_enabled")
    visual_enabled = LaunchConfiguration("visual_enabled")
    always_classify = LaunchConfiguration("always_classify")
    smoothing_alpha = LaunchConfiguration("smoothing_alpha")
    max_velocity = LaunchConfiguration("max_velocity")
    max_acceleration = LaunchConfiguration("max_acceleration")
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
            "use_moveit": "false",
        }.items())

    audio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(audio_share, "launch", "audio_bringup.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "microphone_frame": "table_mic_link",
        }.items(),
        condition=IfCondition(audio_enabled))

    table_microphone_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="table_microphone_broadcaster",
        arguments=[
            "--x", table_mic_x, "--y", table_mic_y, "--z", table_mic_z,
            "--yaw", table_mic_yaw, "--pitch", table_mic_pitch,
            "--roll", table_mic_roll,
            "--frame-id", "world", "--child-frame-id", "table_mic_link",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(audio_enabled))

    audio_tracker = Node(
        package="dracoviloc_tracking",
        executable="arm_audio_tracker",
        parameters=[{
            "use_sim_time": True,
            "smoothing_alpha": ParameterValue(smoothing_alpha, value_type=float),
            "max_velocity": ParameterValue(max_velocity, value_type=float),
            "max_acceleration": ParameterValue(max_acceleration, value_type=float),
        }],
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", audio_enabled, "' == 'true' and '",
            audio_tracking_enabled, "' == 'true'",
        ])))

    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusion_share, "launch", "classification_fusion.launch.py")),
        launch_arguments={
            "tracking_frame": "table_mic_link",
            "min_confidence": min_confidence,
            "gre_trust": gre_trust,
            "ast_enabled": ast_enabled,
            "gre_enabled": gre_enabled,
            "always_classify": always_classify,
            "audio_enabled": audio_enabled,
            "visual_enabled": visual_enabled,
        }.items(),
        # NOT gated on audio_enabled: the EKF and visual (YOLO) fusion must
        # stay independent of the acoustic side - see AUDIO AND VISUAL ARE
        # INDEPENDENT in ekf_fusion_node.py. Launch fusion whenever either
        # modality might feed it; audio_enabled/visual_enabled forwarded
        # above then tell the EKF itself which measurements to actually use.
        condition=IfCondition(PythonExpression([
            "'", fusion_enabled, "' == 'true' and (",
            "'", audio_enabled, "' == 'true' or '",
            visual_enabled, "' == 'true')",
        ])))

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true"),
        DeclareLaunchArgument(
            "audio_enabled", default_value="false",
            description="Launch fixed-table UMA16v2/ODAS localization."),
        DeclareLaunchArgument(
            "audio_tracking_enabled", default_value="true",
            description="Point the simulated wrist along a stable audio direction."),
        DeclareLaunchArgument(
            "fusion_enabled", default_value="true",
            description="Launch classification and EKF fusion "
                        "(dracoviloc_audio_fusion) to produce /fused_target_pose."),
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
            description="Forwarded to classification_fusion.launch.py. Runs "
                        "the YOLO->EKF placeholder bridge (yolo_ekf_adapter) "
                        "and enables the EKF's visual measurement path. "
                        "Independent of audio_enabled - fusion now launches "
                        "whenever either is true, so a UMA-16/ODAS failure "
                        "does not take down visual tracking, and vice versa."),
        DeclareLaunchArgument(
            "min_confidence", default_value="0.20",
            description="Forwarded to classification_fusion.launch.py."),
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
        table_microphone_tf,
        audio,
        fusion,
        audio_tracker,
    ])
