import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("dracoviloc_tracking"),
        "config", "tracking.yaml")
    return LaunchDescription([
        Node(
            package="dracoviloc_tracking",
            executable="arm_drone_tracker",
            parameters=[config, {"use_sim_time": False}],
            output="screen")
    ])
