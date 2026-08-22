import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    microphone_frame = LaunchConfiguration("microphone_frame")
    odas_share = get_package_share_directory("odas_ros")
    odas = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(odas_share, "launch", "odas.launch.xml")),
        launch_arguments={
            "configuration_path": os.path.join(
                odas_share, "config", "configuration.cfg"),
            "frame_id": microphone_frame,
            "audio_queue_size": "8",
            "visualization": "true",
            "rviz": "false",
            "force_publish_tf": "false",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use DracoViLoc/Gazebo clock for stamped ODAS outputs."),
        DeclareLaunchArgument(
            "microphone_frame", default_value="odas_link",
            description="TF frame in which ODAS microphone directions are expressed."),
        SetParameter(name="use_sim_time", value=use_sim_time),
        odas,
        Node(
            package="dracoviloc_odas",
            executable="uma16_feeder",
            arguments=["--lo", "3000", "--hi", "9000",
                       "--frame-id", microphone_frame],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen"),
        Node(
            package="dracoviloc_odas",
            executable="audio_target_tracker",
            parameters=[{
                "use_sim_time": use_sim_time,
                "microphone_frame": microphone_frame,
            }],
            output="screen"),
    ])
