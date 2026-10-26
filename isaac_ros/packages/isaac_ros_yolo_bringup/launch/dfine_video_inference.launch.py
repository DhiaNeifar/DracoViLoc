"""Launch Isaac ROS preprocessing, TensorRT, and D-FINE decoding for a video source.

D-FINE exports (isaac_ros/tools/export_dfine.py) have the RT-DETR interface,
so isaac_ros_rtdetr's preprocessor and decoder are used:

  /image -> resize 640x640 -> RGB tensor 0..1 -> NCHW -> + orig_target_sizes
         -> TensorRT -> RT-DETR decoder -> /detections_output

D-FINE is trained on images stretched to 640x640 (no letterbox), so the image
is stretched the same way. orig_target_sizes is 640x640, so boxes are in the
resized image, whose intrinsics the resize node publishes on
/yolov8_encoder/resize/camera_info, the same topics as the YOLO pipeline:
the direction publisher, preview overlay and EKF work unchanged.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

NETWORK_SIZE = 640


def generate_launch_description() -> LaunchDescription:
    model_path = LaunchConfiguration('model_path')
    engine_path = LaunchConfiguration('engine_path')
    input_width = LaunchConfiguration('input_width')
    input_height = LaunchConfiguration('input_height')
    num_blocks = LaunchConfiguration('num_blocks')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    blocks = ParameterValue(num_blocks, value_type=int)

    container = ComposableNodeContainer(
        name='dfine_inference_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                name='resize',
                namespace='yolov8_encoder',
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                parameters=[{
                    'input_width': ParameterValue(input_width, value_type=int),
                    'input_height': ParameterValue(input_height, value_type=int),
                    'output_width': NETWORK_SIZE,
                    'output_height': NETWORK_SIZE,
                    'keep_aspect_ratio': False,
                    'encoding_desired': 'rgb8',
                    'num_blocks': blocks,
                }],
                remappings=[('image', '/image'), ('camera_info', '/camera_info')],
            ),
            ComposableNode(
                name='image_to_tensor',
                namespace='dfine',
                package='isaac_ros_tensor_proc',
                plugin='nvidia::isaac_ros::dnn_inference::ImageToTensorNode',
                parameters=[{'scale': True, 'tensor_name': 'image'}],  # 0..255 -> 0..1
                remappings=[('image', '/yolov8_encoder/resize/image'), ('tensor', 'image_tensor')],
            ),
            ComposableNode(
                name='interleaved_to_planar',
                namespace='dfine',
                package='isaac_ros_tensor_proc',
                plugin='nvidia::isaac_ros::dnn_inference::InterleavedToPlanarNode',
                parameters=[{'input_tensor_shape': [NETWORK_SIZE, NETWORK_SIZE, 3]}],
                remappings=[('interleaved_tensor', 'image_tensor')],
            ),
            ComposableNode(
                name='reshape',
                namespace='dfine',
                package='isaac_ros_tensor_proc',
                plugin='nvidia::isaac_ros::dnn_inference::ReshapeNode',
                parameters=[{
                    'output_tensor_name': 'input_tensor',
                    'input_tensor_shape': [3, NETWORK_SIZE, NETWORK_SIZE],
                    'output_tensor_shape': [1, 3, NETWORK_SIZE, NETWORK_SIZE],
                }],
                remappings=[('tensor', 'planar_tensor')],
            ),
            ComposableNode(
                name='rtdetr_preprocessor',
                namespace='dfine',
                package='isaac_ros_rtdetr',
                plugin='nvidia::isaac_ros::rtdetr::RtDetrPreprocessorNode',
                parameters=[{
                    'input_image_tensor_name': 'input_tensor',
                    'image_width': NETWORK_SIZE,
                    'image_height': NETWORK_SIZE,
                }],
                remappings=[('encoded_tensor', 'reshaped_tensor')],
            ),
            ComposableNode(
                name='tensor_rt',
                namespace='dfine',
                package='isaac_ros_tensor_rt',
                plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
                parameters=[{
                    'model_file_path': model_path,
                    'engine_file_path': engine_path,
                    'input_tensor_names': ['images', 'orig_target_sizes'],
                    'input_binding_names': ['images', 'orig_target_sizes'],
                    'output_tensor_names': ['labels', 'boxes', 'scores'],
                    'output_binding_names': ['labels', 'boxes', 'scores'],
                    'num_blocks': blocks,
                    'force_engine_update': False,
                    'verbose': False,
                }],
            ),
            ComposableNode(
                name='rtdetr_decoder',
                namespace='dfine',
                package='isaac_ros_rtdetr',
                plugin='nvidia::isaac_ros::rtdetr::RtDetrDecoderNode',
                parameters=[{
                    'confidence_threshold': ParameterValue(confidence_threshold, value_type=float),
                }],
                remappings=[('detections_output', '/detections_output')],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('engine_path'),
        DeclareLaunchArgument('input_width', default_value='640'),
        DeclareLaunchArgument('input_height', default_value='480'),
        DeclareLaunchArgument('num_blocks', default_value='8'),
        DeclareLaunchArgument(
            'confidence_threshold', default_value='0.5',
            description='D-FINE scores are sigmoid class probabilities'),
        container,
    ])
