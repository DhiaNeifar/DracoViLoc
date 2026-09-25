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
        DeclareLaunchArgument('mobilenetv2_ekf_enabled', default_value='false'),
        DeclareLaunchArgument('tracking_frame', default_value='table_mic_link'),
        DeclareLaunchArgument('process_noise', default_value='0.05'),
        DeclareLaunchArgument('measurement_noise', default_value='0.02'),
        DeclareLaunchArgument('yolo_measurement_noise', default_value='-1.0'),
        DeclareLaunchArgument('mobilenetv2_measurement_noise', default_value='-1.0'),
        DeclareLaunchArgument('innovation_gate', default_value='11.34'),
        DeclareLaunchArgument('output_average_window', default_value='5'),
        Node(package='dracoviloc_ekf', executable='ekf_node', output='screen', parameters=[{
            'yolo_enabled': ParameterValue(LaunchConfiguration('yolo_enabled'), value_type=bool),
            'ast_enabled': ParameterValue(LaunchConfiguration('ast_enabled'), value_type=bool),
            'gre_enabled': ParameterValue(LaunchConfiguration('gre_enabled'), value_type=bool),
            'mobilenetv2_ekf_enabled': ParameterValue(
                LaunchConfiguration('mobilenetv2_ekf_enabled'), value_type=bool),
            'process_noise': ParameterValue(
                LaunchConfiguration('process_noise'), value_type=float),
            'measurement_noise': ParameterValue(
                LaunchConfiguration('measurement_noise'), value_type=float),
            'yolo_measurement_noise': ParameterValue(
                LaunchConfiguration('yolo_measurement_noise'), value_type=float),
            'mobilenetv2_measurement_noise': ParameterValue(
                LaunchConfiguration('mobilenetv2_measurement_noise'), value_type=float),
            'innovation_gate': ParameterValue(
                LaunchConfiguration('innovation_gate'), value_type=float),
            'output_average_window': ParameterValue(
                LaunchConfiguration('output_average_window'), value_type=int),
            'tracking_frame': LaunchConfiguration('tracking_frame'), 'use_sim_time': False}])])
