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
        DeclareLaunchArgument('bearing_gate_enabled', default_value='true'),
        DeclareLaunchArgument('bearing_gate_base_angle_deg', default_value='45.0'),
        DeclareLaunchArgument('bearing_gate_max_angle_deg', default_value='70.0'),
        DeclareLaunchArgument('bearing_gate_max_rate_deg_s', default_value='20.0'),
        DeclareLaunchArgument('bearing_gate_confirmations', default_value='5'),
        DeclareLaunchArgument('bearing_gate_confirmation_time_s', default_value='2.0'),
        DeclareLaunchArgument('bearing_gate_challenger_tolerance_deg', default_value='15.0'),
        DeclareLaunchArgument('bearing_gate_reacquire_timeout_s', default_value='10.0'),
        DeclareLaunchArgument('first_arrival_mode', default_value='off'),
        DeclareLaunchArgument('first_arrival_min_confidence', default_value='0.70'),
        DeclareLaunchArgument('first_arrival_match_angle_deg', default_value='20.0'),
        DeclareLaunchArgument('first_arrival_anchor_deviation_deg', default_value='25.0'),
        DeclareLaunchArgument('first_arrival_evidence_ttl_s', default_value='2.0'),
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
            'bearing_gate_enabled': ParameterValue(
                LaunchConfiguration('bearing_gate_enabled'), value_type=bool),
            'bearing_gate_base_angle_deg': ParameterValue(
                LaunchConfiguration('bearing_gate_base_angle_deg'), value_type=float),
            'bearing_gate_max_angle_deg': ParameterValue(
                LaunchConfiguration('bearing_gate_max_angle_deg'), value_type=float),
            'bearing_gate_max_rate_deg_s': ParameterValue(
                LaunchConfiguration('bearing_gate_max_rate_deg_s'), value_type=float),
            'bearing_gate_confirmations': ParameterValue(
                LaunchConfiguration('bearing_gate_confirmations'), value_type=int),
            'bearing_gate_confirmation_time_s': ParameterValue(
                LaunchConfiguration('bearing_gate_confirmation_time_s'), value_type=float),
            'bearing_gate_challenger_tolerance_deg': ParameterValue(
                LaunchConfiguration('bearing_gate_challenger_tolerance_deg'), value_type=float),
            'bearing_gate_reacquire_timeout_s': ParameterValue(
                LaunchConfiguration('bearing_gate_reacquire_timeout_s'), value_type=float),
            # YAML treats the unquoted words "off" and "on" as booleans.
            # Force a string so the default mode cannot become false and abort
            # the C++ node with an InvalidParameterTypeException.
            'first_arrival_mode': ParameterValue(
                LaunchConfiguration('first_arrival_mode'), value_type=str),
            'first_arrival_min_confidence': ParameterValue(
                LaunchConfiguration('first_arrival_min_confidence'), value_type=float),
            'first_arrival_match_angle_deg': ParameterValue(
                LaunchConfiguration('first_arrival_match_angle_deg'), value_type=float),
            'first_arrival_anchor_deviation_deg': ParameterValue(
                LaunchConfiguration('first_arrival_anchor_deviation_deg'), value_type=float),
            'first_arrival_evidence_ttl_s': ParameterValue(
                LaunchConfiguration('first_arrival_evidence_ttl_s'), value_type=float),
            'tracking_frame': LaunchConfiguration('tracking_frame'), 'use_sim_time': False}])])
