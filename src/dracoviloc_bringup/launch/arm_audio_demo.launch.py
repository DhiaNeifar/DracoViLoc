import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gui = LaunchConfiguration("use_gui")
    audio_enabled = LaunchConfiguration("audio_enabled")
    audio_tracking_enabled = LaunchConfiguration("audio_tracking_enabled")
    table_mic_x = LaunchConfiguration("table_mic_x")
    table_mic_y = LaunchConfiguration("table_mic_y")
    table_mic_z = LaunchConfiguration("table_mic_z")
    table_mic_yaw = LaunchConfiguration("table_mic_yaw")
    table_mic_pitch = LaunchConfiguration("table_mic_pitch")
    table_mic_roll = LaunchConfiguration("table_mic_roll")
    bringup_share = get_package_share_directory("dracoviloc_bringup")
    audio_share = get_package_share_directory("dracoviloc_odas")

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
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", audio_enabled, "' == 'true' and '",
            audio_tracking_enabled, "' == 'true'",
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
        DeclareLaunchArgument("table_mic_x", default_value="0.0"),
        DeclareLaunchArgument("table_mic_y", default_value="0.0"),
        DeclareLaunchArgument("table_mic_z", default_value="0.75"),
        DeclareLaunchArgument("table_mic_yaw", default_value="3.1415926535897"),
        DeclareLaunchArgument("table_mic_pitch", default_value="0.0"),
        DeclareLaunchArgument("table_mic_roll", default_value="1.57079632679"),
        arm_demo,
        table_microphone_tf,
        audio,
        audio_tracker,
    ])
