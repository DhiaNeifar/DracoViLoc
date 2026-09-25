"""Run MobileNetV2 in a CUDA-capable interpreter, using this checkout's model."""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('dracoviloc_mobilenetv2')
    with open(os.path.join(share, 'launch', 'model_paths.json'), encoding='utf-8') as stream:
        model_dir = json.load(stream)['model_dir']
    engine_path = os.path.join(model_dir, 'drone_fp32.engine')
    defaults = {
        'venv_python': '/usr/bin/python3',
        'engine_path': engine_path,
        'channels': '4', 'threshold': '0.75', 'votes_required': '2',
        'vote_window': '2', 'min_activity': '0.10', 'always_classify': 'false',
        'sst_timeout': '0.25', 'max_audio_age': '0.5',
    }
    cmd = [LaunchConfiguration('venv_python'), '-u',
           os.path.join(share, 'mobilenetv2', 'classifier_node.py')]
    for name in defaults:
        if name != 'venv_python':
            cmd += ['--' + name.replace('_', '-'), LaunchConfiguration(name)]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        ExecuteProcess(cmd=cmd, output='screen', emulate_tty=True),
    ])
