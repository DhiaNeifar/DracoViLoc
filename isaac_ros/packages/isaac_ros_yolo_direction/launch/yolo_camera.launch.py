#!/usr/bin/env python3
"""Start the USB camera, YOLO TensorRT inference, directions, and display."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_microphone_array_camera():
    """Return the primary icSpring video device, independent of video number."""
    candidates = []
    detected = []

    for device_dir in sorted(Path('/sys/class/video4linux').glob('video*')):
        device = Path('/dev') / device_dir.name
        try:
            product = (device_dir / 'name').read_text().strip()
            interface_index = (device_dir / 'index').read_text().strip()
        except OSError:
            continue

        detected.append(f'{device} ({product}, index {interface_index})')
        if 'icspring' in product.lower() and interface_index == '0' and device.exists():
            candidates.append(str(device))

    if candidates:
        return candidates[0]

    devices = ', '.join(detected) if detected else 'none'
    raise RuntimeError(
        'The microphone-array icSpring camera was not found. '
        f'Detected video devices: {devices}. '
        'Connect the camera or explicitly set camera:=/dev/videoX.'
    )


def _launch_camera(context):
    requested_camera = LaunchConfiguration('camera').perform(context)
    camera = (
        _find_microphone_array_camera()
        if requested_camera.strip().lower() == 'auto'
        else requested_camera
    )

    width = LaunchConfiguration('width').perform(context)
    height = LaunchConfiguration('height').perform(context)
    camera_fps = LaunchConfiguration('camera_fps').perform(context)
    publish_rate = LaunchConfiguration('publish_rate').perform(context)
    horizontal_fov_deg = LaunchConfiguration('horizontal_fov_deg').perform(context)

    configure_camera = ExecuteProcess(
        cmd=[
            'v4l2-ctl', '-d', camera,
            '--set-ctrl=exposure_dynamic_framerate=0',
            f'--set-fmt-video=width={width},height={height},pixelformat=MJPG',
            f'--set-parm={camera_fps}',
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
            'publish_rate': float(publish_rate),
            'horizontal_fov_deg': float(horizontal_fov_deg),
            'loop': False,
        }],
    )

    start_camera_after_configuration = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_camera,
            on_exit=[camera_publisher],
        )
    )

    return [
        LogInfo(msg=f'Using camera: {camera}'),
        start_camera_after_configuration,
        configure_camera,
    ]


def generate_launch_description():
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    model_path = LaunchConfiguration('model_path')
    engine_path = LaunchConfiguration('engine_path')
    direction_frame = LaunchConfiguration('direction_frame')
    use_viewer = LaunchConfiguration('use_viewer')

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

    viewer = Node(
        package='image_tools',
        executable='showimage',
        name='yolo_image_viewer',
        output='screen',
        remappings=[('image', '/yolov8_processed_image')],
        condition=IfCondition(use_viewer),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera',
            default_value='auto',
            description='Video device path, or auto to select the icSpring array camera',
        ),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='60'),
        DeclareLaunchArgument('publish_rate', default_value='30.0'),
        DeclareLaunchArgument('horizontal_fov_deg', default_value='100.0'),
        DeclareLaunchArgument('direction_frame', default_value='uma16_camera_direction'),
        DeclareLaunchArgument('use_viewer', default_value='true'),
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
        viewer,
        OpaqueFunction(function=_launch_camera),
    ])
