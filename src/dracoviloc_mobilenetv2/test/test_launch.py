import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.utilities import perform_substitutions

REPO = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    'arm_demo', REPO / 'src/dracoviloc_bringup/launch/arm_demo.launch.py')
bringup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bringup)


def configure(**changes):
    values = dict(tracking_mode='off', audio_enabled='false', ast_enabled='false',
                  gre_enabled='false', yolo_enabled='false', fusion_enabled='false',
                  recording_enabled='false', sss_channels_recording='false',
                  mobilenetv2_enabled='false', mobilenetv2_ekf_enabled='false',
                  mobilenetv2_engine_path='', mobilenetv2_venv_python='',
                  first_arrival_mode='off',
                  diagnostics_enabled='false',
                  odas_diagnostics_log_path='', odas_diagnostics_interval_s='10.0',
                  direction_marker_length='1.0', direction_marker_timeout='1.25')
    values.update(changes)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return bringup._configure_pipeline(context)


def test_disabled_needs_no_runtime():
    assert configure(mobilenetv2_engine_path='/does/not/exist') == []


@pytest.mark.parametrize('settings', [dict(mobilenetv2_enabled='true'),
    dict(mobilenetv2_ekf_enabled='true'),
    dict(audio_enabled='true', mobilenetv2_enabled='true', mobilenetv2_ekf_enabled='true'),
    dict(audio_enabled='true', tracking_mode='direct_mobilenetv2')])
def test_invalid_combinations(settings):
    with pytest.raises(RuntimeError):
        configure(**settings)


@pytest.mark.parametrize('participates', ['false', 'true'])
def test_ekf_choice_is_independent_of_classifier(participates):
    actions = configure(audio_enabled='true', mobilenetv2_enabled='true',
                        fusion_enabled='true', mobilenetv2_ekf_enabled=participates)
    expanded = [child for action in actions
                for child in (action.get_sub_entities() if isinstance(action, GroupAction) else [action])]
    includes = [a for a in expanded if isinstance(a, IncludeLaunchDescription)]
    # Audio bringup, MobileNetV2, and EKF are independent includes.
    assert len(includes) == 3
    ekf_arguments = dict(includes[-1].launch_arguments)
    assert ekf_arguments['mobilenetv2_ekf_enabled'] == participates
    assert ekf_arguments['ast_enabled'] == 'false'


@pytest.mark.parametrize('mode,enabled', [('direct_ast', 'ast_enabled'),
                                        ('direct_gre', 'gre_enabled'),
                                        ('direct_either', 'ast_enabled'),
                                        ('direct_yolo', 'yolo_enabled'),
                                        ('direct_mobilenetv2', 'mobilenetv2_enabled')])
def test_direct_modes_can_still_launch(mode, enabled):
    assert configure(audio_enabled='true', tracking_mode=mode, **{enabled: 'true'})


@pytest.mark.parametrize('override', ['', '/tmp/selected-mobile.engine'])
def test_classifier_launch_defaults_do_not_leak(override):
    context = LaunchContext()
    context.launch_configurations.update(dict(
        tracking_mode='off', audio_enabled='true', ast_enabled='true', gre_enabled='true',
        yolo_enabled='false', fusion_enabled='false', recording_enabled='false',
        sss_channels_recording='false',
        mobilenetv2_enabled='true',
        mobilenetv2_ekf_enabled='false', mobilenetv2_engine_path=override,
        mobilenetv2_venv_python='', ast_threshold='0.20', mobilenetv2_threshold='0.75',
        first_arrival_mode='off',
        diagnostics_enabled='false',
        odas_diagnostics_log_path='', odas_diagnostics_interval_s='10.0',
        first_arrival_min_confidence='0.70', first_arrival_onset_snr_db='10.0',
        first_arrival_noise_window_s='1.0', first_arrival_max_span_ms='1.2',
        first_arrival_residual_us='150.0', first_arrival_min_inliers='12',
        first_arrival_refractory_ms='50.0', first_arrival_diagnostics_path='',
        first_arrival_raw_recording_path='',
        mobilenetv2_votes_required='3', mobilenetv2_vote_window='5',
        always_classify='false', min_activity='0.10', bearing_gate_enabled='true',
        table_mic_x='0.0', table_mic_y='0.0', table_mic_z='0.75',
        table_mic_yaw='3.1415926535897', table_mic_pitch='0.0',
        table_mic_roll='1.57079632679', direction_marker_length='1.0',
        direction_marker_timeout='1.25'))
    context.extend_locals({'ros_specific_arguments': {'name': '', 'ns': ''}})
    actions = bringup._configure_pipeline(context)
    commands = []

    # Execute launch configuration actions, but intercept every process before startup.
    def visit(entity):
        if isinstance(entity, ExecuteProcess):
            commands.append([perform_substitutions(context, part) for part in entity.cmd])
            return
        children = entity.entities if isinstance(entity, LaunchDescription) else entity.execute(context)
        for child in children or []:
            visit(child)

    for action in actions:
        visit(action)
    # The audio include also expands ODAS processes; only the classifier
    # commands below are relevant to this launch-default isolation test.
    assert len(commands) >= 5
    process_commands = [command for command in commands
                        if 'static_transform_publisher' not in command[0]]
    ast = next(command for command in process_commands
               if any(part.endswith('/models/ast/drone_ast.engine') for part in command))
    gre = next(command for command in process_commands
               if any(part.endswith('/models/gre/model_logmel.engine') for part in command))
    mobile = next(command for command in process_commands
                  if '--engine-path' in command)
    assert ast[ast.index('--engine') + 1].endswith('/models/ast/drone_ast.engine')
    assert gre[gre.index('--engine') + 1].endswith('/models/gre/model_logmel.engine')
    assert '/gre_env/' in gre[0]
    assert mobile[mobile.index('--engine-path') + 1] == (
        override or str(REPO / 'models/mobilenetv2/drone_fp32.engine'))
    assert mobile[mobile.index('--threshold') + 1] == '0.75'
    assert 'engine_path' not in context.launch_configurations
