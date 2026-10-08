import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    share = get_package_share_directory('dracoviloc_gre')
    root = [EnvironmentVariable('HOME'), 'DracoViLoc']
    node = ExecuteProcess(cmd=[LaunchConfiguration('venv_python'), os.path.join(share,'gre','gre_classifier_node.py'),
        '--repo', os.path.join(share,'gre','repo'), '--engine', LaunchConfiguration('engine_path'),
        '--meta', LaunchConfiguration('meta_path'), '--channels', LaunchConfiguration('channels'),
        '--min-activity', LaunchConfiguration('min_activity'), '--reset-cooldown', LaunchConfiguration('reset_cooldown'),
        '--verbose', LaunchConfiguration('verbose'),
        '--bearing-gate', LaunchConfiguration('bearing_gate'),
        '--max-bearing-jump-deg', LaunchConfiguration('max_bearing_jump_deg'),
        '--bearing-confirm', LaunchConfiguration('bearing_confirm'),
        '--bearing-timeout', LaunchConfiguration('bearing_timeout')], output='screen', emulate_tty=True)
    return LaunchDescription([
        DeclareLaunchArgument('venv_python', default_value=PathJoinSubstitution(root + ['gre_env','bin','python3'])),
        DeclareLaunchArgument('engine_path', default_value=PathJoinSubstitution(root + ['models','gre','model_logmel.engine'])),
        DeclareLaunchArgument('meta_path', default_value=PathJoinSubstitution(root + ['models','gre','model_logmel_meta.json'])),
        DeclareLaunchArgument('channels', default_value='4'), DeclareLaunchArgument('min_activity', default_value='0.1'),
        DeclareLaunchArgument('reset_cooldown', default_value='2.0'), DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('bearing_gate', default_value='true'),
        DeclareLaunchArgument('max_bearing_jump_deg', default_value='60.0'),
        DeclareLaunchArgument('bearing_confirm', default_value='3'),
        DeclareLaunchArgument('bearing_timeout', default_value='2.0'), node])
