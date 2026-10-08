import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    share = get_package_share_directory('dracoviloc_ast')
    args = ['channels', 'threshold', 'consecutive', 'min_activity', 'always_classify',
            'bearing_gate', 'max_bearing_jump_deg', 'bearing_confirm', 'bearing_timeout']
    node = ExecuteProcess(cmd=[
        LaunchConfiguration('venv_python'), os.path.join(share, 'ast', 'ast_classifier_node.py'),
        '--project-dir', os.path.join(share, 'ast'), '--model-dir', LaunchConfiguration('model_dir'),
        '--engine', LaunchConfiguration('engine_path'), '--channels', LaunchConfiguration('channels'),
        '--threshold', LaunchConfiguration('threshold'), '--consecutive', LaunchConfiguration('consecutive'),
        '--min-activity', LaunchConfiguration('min_activity'), '--always-classify', LaunchConfiguration('always_classify'),
        '--bearing-gate', LaunchConfiguration('bearing_gate'),
        '--max-bearing-jump-deg', LaunchConfiguration('max_bearing_jump_deg'),
        '--bearing-confirm', LaunchConfiguration('bearing_confirm'),
        '--bearing-timeout', LaunchConfiguration('bearing_timeout')],
        output='screen', emulate_tty=True)
    root = [EnvironmentVariable('HOME'), 'DracoViLoc']
    return LaunchDescription([
        DeclareLaunchArgument('venv_python', default_value=PathJoinSubstitution(root + ['trt_env','bin','python3'])),
        DeclareLaunchArgument('model_dir', default_value=PathJoinSubstitution(root + ['models','ast'])),
        DeclareLaunchArgument('engine_path', default_value=PathJoinSubstitution(root + ['models','ast','drone_ast.engine'])),
        DeclareLaunchArgument('channels', default_value='4'), DeclareLaunchArgument('threshold', default_value='0.2'),
        DeclareLaunchArgument('consecutive', default_value='3'), DeclareLaunchArgument('min_activity', default_value='0.1'),
        DeclareLaunchArgument('always_classify', default_value='false'),
        DeclareLaunchArgument('bearing_gate', default_value='true'),
        DeclareLaunchArgument('max_bearing_jump_deg', default_value='60.0'),
        DeclareLaunchArgument('bearing_confirm', default_value='3'),
        DeclareLaunchArgument('bearing_timeout', default_value='2.0'), node])
