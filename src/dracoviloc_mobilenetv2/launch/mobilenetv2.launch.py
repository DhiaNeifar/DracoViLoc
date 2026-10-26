"""Run MobileNetV2 in a CUDA-capable interpreter, using this checkout's model."""
import json
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

DEFAULTS = {
    'venv_python': 'auto',
    'engine_path': 'auto',
    'channels': '4', 'threshold': '0.75', 'votes_required': '2',
    'vote_window': '2', 'min_activity': '0.10', 'always_classify': 'false',
    'sst_timeout': '0.25', 'max_audio_age': '0.5',
    'bearing_gate': 'true', 'max_bearing_jump_deg': '60.0',
    'bearing_confirm': '3', 'bearing_timeout': '2.0',
}


def _launch_classifier(context):
    share = get_package_share_directory('dracoviloc_mobilenetv2')
    with open(os.path.join(share, 'launch', 'model_paths.json'), encoding='utf-8') as stream:
        model_dir = json.load(stream)['model_dir']
    values = {name: LaunchConfiguration(name).perform(context) for name in DEFAULTS}
    actions = []
    if 'auto' in (values['engine_path'], values['venv_python']):
        # 'auto' picks the engine built for this machine's GPU (Jetson Orin,
        # laptop T2000, ...) and an interpreter that has TensorRT + PyCUDA.
        sys.path.insert(0, os.path.join(share, 'mobilenetv2'))
        from runtime_selection import select_runtime
        selected = select_runtime(model_dir)
        if values['engine_path'] == 'auto':
            values['engine_path'] = selected['engine']
        if values['venv_python'] == 'auto':
            values['venv_python'] = selected['python']
        actions.append(LogInfo(msg=(
            f"MobileNetV2 on {selected['gpu']}: engine={values['engine_path']} "
            f"python={values['venv_python']}")))
    cmd = [values['venv_python'], '-u', os.path.join(share, 'mobilenetv2', 'classifier_node.py')]
    for name, value in values.items():
        if name != 'venv_python':
            cmd += ['--' + name.replace('_', '-'), value]
    actions.append(ExecuteProcess(
        cmd=cmd, output='screen', emulate_tty=True,
        respawn=True, respawn_delay=2.0,
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in DEFAULTS.items()],
        OpaqueFunction(function=_launch_classifier),
    ])
