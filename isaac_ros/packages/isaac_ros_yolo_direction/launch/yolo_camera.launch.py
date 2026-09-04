#!/usr/bin/env python3
"""Start the USB camera, YOLO TensorRT inference, directions, and display."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera = LaunchConfiguration('camera')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    camera_fps = LaunchConfiguration('camera_fps')
    publish_rate = LaunchConfiguration('publish_rate')
    horizontal_fov_deg = LaunchConfiguration('horizontal_fov_deg')
    model_path = LaunchConfiguration('model_path')
    engine_path = LaunchConfiguration('engine_path')
    direction_frame = LaunchConfiguration('direction_frame')
    use_viewer = LaunchConfiguration('use_viewer')
    record = LaunchConfiguration('record')

    inference = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('isaac_ros_yolo_bringup'),
            'launch',
            'yolo_video_inference.launch.py',
        )),
        launch_arguments={
            'model_path': model_path,
            'engine_path': engine_path,
            'num_classes': '1',
            'input_width': width,
            'input_height': height,
            'num_blocks': '8',
        }.items(),
    )

    configure_camera = ExecuteProcess(
        cmd=[
            'v4l2-ctl', '-d', camera,
            '--set-ctrl=exposure_dynamic_framerate=0',
            ['--set-fmt-video=width=', width, ',height=', height,
             ',pixelformat=MJPG'],
            ['--set-parm=', camera_fps],
        ],
        output='screen',
    )

    camera_publisher = Node(
        package='yolo_video_publisher',
        executable='video_publisher_node',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'video_path': camera,
            'publish_rate': publish_rate,
            'horizontal_fov_deg': horizontal_fov_deg,
            'loop': False,
        }],
    )

    start_camera_after_configuration = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_camera,
            on_exit=[camera_publisher],
        )
    )

    direction = Node(
        package='isaac_ros_yolo_direction',
        executable='direction_publisher',
        name='yolo_direction_publisher',
        output='screen',
        parameters=[{'frame_id': direction_frame}],
    )

    visualizer = Node(
        package='isaac_ros_yolo_bringup',
        executable='yolo_visualizer.py',
        name='yolov8_visualizer',
        output='screen',
        parameters=[{
            'class_names': ['drone'],
        }],
    )

    recorder = Node(
        package='isaac_ros_yolo_bringup',
        executable='multimodal_recorder',
        name='multimodal_recorder',
        output='screen',
        parameters=[{
            'output_root': LaunchConfiguration('recording_root'),
            'audio_device': LaunchConfiguration('recording_audio_device'),
            'video_topic': '/yolov8_processed_image',
            'video_fps': ParameterValue(
                LaunchConfiguration('recording_fps'), value_type=float),
            'video_bitrate': ParameterValue(
                LaunchConfiguration('recording_bitrate'), value_type=int),
        }],
        condition=IfCondition(record),
    )

    viewer = Node(
        package='image_tools',
        executable='showimage',
        name='yolo_image_viewer',
        output='screen',
        remappings=[('image', '/yolov8_processed_image')],
        condition=IfCondition(use_viewer),
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='60'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('horizontal_fov_deg', default_value='100.0'),
        DeclareLaunchArgument('direction_frame', default_value='uma16_camera_direction'),
        DeclareLaunchArgument('use_viewer', default_value='true'),
        DeclareLaunchArgument(
            'record', default_value='false',
            description='Record UMA16 audio and annotated YOLO video.'),
        DeclareLaunchArgument(
            'recording_root', default_value='/home/dhianeifar/DracoViLoc/runs'),
        DeclareLaunchArgument(
            'recording_audio_device', default_value='auto',
            description='ALSA input override; auto discovers the UMA16.'),
        DeclareLaunchArgument('recording_fps', default_value='15.0'),
        DeclareLaunchArgument('recording_bitrate', default_value='4000000'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx',
        ),
        DeclareLaunchArgument(
            'engine_path',
            default_value='/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan',
        ),
        inference,
        direction,
        visualizer,
        recorder,
        viewer,
        start_camera_after_configuration,
        configure_camera,
    ])
