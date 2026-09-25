from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/sss"),
        DeclareLaunchArgument(
            "gain_db", default_value="0.0",
            description="Digital gain applied to each channel before writing "
                        "(e.g. 24.0 for +24 dB). Int16 clipping above 0 dBFS."),
        DeclareLaunchArgument(
            "output_root",
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("HOME"), "DracoViLoc", "recordings"]),
            description="Directory receiving one timestamped folder per recording."),
        Node(
            package="dracoviloc_recording",
            executable="sss_channel_recorder",
            name="sss_channel_recorder",
            output="screen",
            parameters=[{
                "audio_topic": LaunchConfiguration("audio_topic"),
                "output_root": LaunchConfiguration("output_root"),
                "gain_db": LaunchConfiguration("gain_db"),
            }]),
    ])
