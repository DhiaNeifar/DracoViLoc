#!/usr/bin/env python3
"""Launch Isaac ROS preprocessing, TensorRT, and YOLO decoding for a video source."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    model_path = LaunchConfiguration('model_path')
    engine_path = LaunchConfiguration('engine_path')
    num_classes = LaunchConfiguration('num_classes')
    input_width = LaunchConfiguration('input_width')
    input_height = LaunchConfiguration('input_height')
    num_blocks = LaunchConfiguration('num_blocks')

    container = ComposableNodeContainer(
        name='yolo_inference_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                name='tensor_rt',
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                parameters=[{
                    'model_file_path': model_path,
                    'engine_file_path': engine_path,
                    'input_tensor_names': ['input_tensor'],
                    'input_binding_names': ['images'],
                    'output_tensor_names': ['output_tensor'],
                    'output_binding_names': ['output0'],
                    'num_blocks': ParameterValue(num_blocks, value_type=int),
                    'force_engine_update': False,
                    'verbose': False,
                }],
            ),
            ComposableNode(
                name='yolov8_decoder_node',
                package='isaac_ros_yolov8',
                plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
                parameters=[{
                    'num_classes': num_classes,
                    'confidence_threshold': 0.25,
                    'nms_threshold': 0.45,
                    'tensor_name': 'output_tensor',
                }],
            ),
        ],
        output='screen',
    )

    encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('isaac_ros_dnn_image_encoder'),
            'launch',
            'dnn_image_encoder.launch.py',
        )),
        launch_arguments={
            'input_image_width': input_width,
            'input_image_height': input_height,
            'network_image_width': '640',
            'network_image_height': '640',
            'image_mean': '[0.0, 0.0, 0.0]',
            'image_stddev': '[1.0, 1.0, 1.0]',
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'yolo_inference_container',
            'dnn_image_encoder_namespace': 'yolov8_encoder',
            'image_input_topic': '/image',
            'camera_info_input_topic': '/camera_info',
            'tensor_output_topic': '/tensor_pub',
            'input_encoding': 'rgb8',
            'num_blocks': num_blocks,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('engine_path'),
        DeclareLaunchArgument('num_classes', default_value='1'),
        DeclareLaunchArgument('input_width', default_value='640'),
        DeclareLaunchArgument('input_height', default_value='360'),
        DeclareLaunchArgument('num_blocks', default_value='8'),
        container,
        encoder_launch,
    ])


if __name__ == '__main__':
    generate_launch_description()
