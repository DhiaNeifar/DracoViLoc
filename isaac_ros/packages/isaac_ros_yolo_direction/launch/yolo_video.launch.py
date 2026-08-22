#!/usr/bin/env python3
"""Run the complete YOLO pipeline against the bundled test video."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video = LaunchConfiguration('video')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    publish_rate = LaunchConfiguration('publish_rate')
    horizontal_fov_deg = LaunchConfiguration('horizontal_fov_deg')
    model_path = LaunchConfiguration('model_path')
    engine_path = LaunchConfiguration('engine_path')

    inference = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('isaac_ros_yolo_bringup'),
            'launch', 'yolo_video_inference.launch.py')),
        launch_arguments={
            'model_path': model_path,
            'engine_path': engine_path,
            'num_classes': '1',
            'input_width': width,
            'input_height': height,
            'num_blocks': '8',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'video',
            default_value='/workspaces/isaac_ros-dev/media/drone-video1.mp4'),
        DeclareLaunchArgument('width', default_value='848'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('horizontal_fov_deg', default_value='100.0'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx'),
        DeclareLaunchArgument(
            'engine_path',
            default_value='/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan'),
        inference,
        Node(
            package='yolo_video_publisher',
            executable='video_publisher_node',
            name='video_publisher',
            output='screen',
            parameters=[{
                'video_path': video,
                'publish_rate': publish_rate,
                'horizontal_fov_deg': horizontal_fov_deg,
                'loop': True,
            }],
        ),
        Node(
            package='isaac_ros_yolo_direction',
            executable='direction_publisher',
            name='yolo_direction_publisher',
            output='screen',
        ),
        Node(
            package='isaac_ros_yolo_bringup',
            executable='yolo_visualizer.py',
            name='yolov8_visualizer',
            output='screen',
            parameters=[{'class_names': ['drone']}],
        ),
        Node(
            package='image_tools',
            executable='showimage',
            name='yolo_image_viewer',
            output='screen',
            remappings=[('image', '/yolov8_processed_image')],
        ),
    ])
