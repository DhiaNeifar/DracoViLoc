from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('yolo_enabled', default_value='false'),
        DeclareLaunchArgument('ast_enabled', default_value='false'),
        DeclareLaunchArgument('gre_enabled', default_value='false'),
        DeclareLaunchArgument('tracking_frame', default_value='table_mic_link'),
        Node(package='dracoviloc_ekf', executable='ekf_node', output='screen', parameters=[{
            'yolo_enabled': ParameterValue(LaunchConfiguration('yolo_enabled'), value_type=bool),
            'ast_enabled': ParameterValue(LaunchConfiguration('ast_enabled'), value_type=bool),
            'gre_enabled': ParameterValue(LaunchConfiguration('gre_enabled'), value_type=bool),
            'tracking_frame': LaunchConfiguration('tracking_frame'), 'use_sim_time': False}])])
