#!/usr/bin/env python3
"""Opt-in GPU/DDS test with reference audio and mock joints on an isolated domain.

Run from a sourced host checkout with python3. Never starts real hardware or ODAS.
Logs go to /tmp; all processes created here are stopped on success or failure.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '83'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import numpy as np
from scipy.signal import resample_poly
import rclpy
from rclpy.qos import qos_profile_sensor_data
from audio_utils_msgs.msg import AudioFrame
from geometry_msgs.msg import Vector3Stamped
from odas_ros_msgs.msg import OdasSst, OdasSstArrayStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

ROOT = Path(__file__).resolve().parents[3]
LOGS = Path('/tmp') / ('mobilenetv2-smoke-' + time.strftime('%Y%m%d-%H%M%S'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine', type=Path,
                        default=ROOT / 'models/mobilenetv2/drone_fp32.engine',
                        help='TensorRT engine compatible with the current GPU')
    args = parser.parse_args()
    if not args.engine.is_file():
        parser.error(f'engine does not exist: {args.engine}')
    LOGS.mkdir()
    processes = []
    result = {}

    def start(name, *command):
        stream = (LOGS / (name + '.log')).open('w')
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((process, stream))
        return process

    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)

    rclpy.init()
    node = rclpy.create_node('mobilenetv2_smoke_observer')
    received = {'mobile': [], 'ekf': [], 'joint': [], 'command': []}
    for key, topic, kind, qos in [
        ('mobile', '/mobilenetv2/direction', Vector3Stamped, 10),
        ('ekf', '/ekf/direction', Vector3Stamped, 10),
        ('joint', '/joint_states', JointState, qos_profile_sensor_data),
        ('command', '/arm_controller/joint_trajectory', JointTrajectory, 10),
    ]:
        node.create_subscription(kind, topic, lambda msg, k=key: received[k].append(msg), qos)
    audio_pub = node.create_publisher(AudioFrame, '/sss', 10)
    sst_pub = node.create_publisher(OdasSstArrayStamped, '/sst', 10)
    with np.load(ROOT / 'models/mobilenetv2/reference_outputs.npz') as data:
        best = int(np.argmax(data['pytorch_probs'][:, 1]))
        waveform = resample_poly(data['waveforms'][best], 441, 160)
    pcm = np.clip(waveform * 32768, -32768, 32767).astype('<i2')
    offset = 0
    direction = [0.6, 0.8, 0.0]

    def run(seconds, publish=True):
        nonlocal offset
        deadline = time.monotonic() + seconds
        next_audio = time.monotonic()
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.001)
            if publish and time.monotonic() >= next_audio:
                stamp = node.get_clock().now().to_msg()
                sst = OdasSstArrayStamped()
                sst.header.stamp = stamp
                sst.header.frame_id = 'table_mic_link'
                sst.sources = [OdasSst(id=1, x=direction[0], y=direction[1], z=direction[2], activity=1.0)]
                sst.sources += [OdasSst() for _ in range(3)]
                sst_pub.publish(sst)
                audio = AudioFrame()
                audio.header = sst.header
                audio.format = 'signed_16'
                audio.sampling_frequency = 44100
                audio.channel_count = 4
                audio.frame_sample_count = 512
                samples = pcm[np.arange(offset, offset + 512) % len(pcm)]
                frames = np.zeros((512, 4), dtype='<i2')
                frames[:, 0] = samples
                audio.data = frames.tobytes()
                audio_pub.publish(audio)
                offset += 512
                next_audio += 512 / 44100

    def positions():
        msg = received['joint'][-1]
        return np.array([msg.position[msg.name.index('joint' + str(i))] for i in range(1, 7)])

    def enable_tracking():
        client = node.create_client(SetBool, '/demo/tracking')
        assert client.wait_for_service(timeout_sec=10), 'tracker service did not become available'
        request = SetBool.Request()
        request.data = True
        future = client.call_async(request)
        deadline = time.monotonic() + 20
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert future.done(), 'tracking service timed out'
        assert future.result().success, f'could not enable tracking: {future.result().message}'
        node.destroy_client(client)

    try:
        robot = start('robot', 'ros2', 'launch', 'dracoviloc_bringup', 'demo.launch.py',
                      'hardware_mode:=mock', 'use_rviz:=false', 'use_moveit:=false')
        start('tf', 'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
              '--frame-id', 'world', '--child-frame-id', 'table_mic_link')
        mobile = start('mobile', 'ros2', 'launch', 'dracoviloc_mobilenetv2',
                       'mobilenetv2.launch.py', f'engine_path:={args.engine}')
        run(12)
        assert robot.poll() is None and mobile.poll() is None, 'startup failed; inspect logs'
        assert received['joint'], 'no mock joint states'
        assert received['mobile'], 'reference audio produced no accepted MobileNetV2 directions'
        output = received['mobile'][-1].vector
        np.testing.assert_allclose([output.x, output.y, output.z], [0.6, 0.8, 0], atol=1e-6)
        result['reference_audio_directions'] = len(received['mobile'])

        # Existing inputs remain subscribed; MobileNetV2 must not affect this EKF.
        ekf = start('ekf_disabled', 'ros2', 'launch', 'dracoviloc_ekf', 'ekf.launch.py',
                    'yolo_enabled:=true', 'ast_enabled:=true', 'gre_enabled:=true',
                    'mobilenetv2_ekf_enabled:=false')
        run(4)
        assert not received['ekf'], 'disabled MobileNetV2 input affected EKF'
        topics = dict(node.get_subscriber_names_and_types_by_node('dracoviloc_ekf', '/'))
        assert '/mobilenetv2/direction' not in topics
        assert all(t in topics for t in ('/ast/direction', '/gre/direction', '/yolo/direction'))
        for topic in ('/ast/direction', '/gre/direction', '/yolo/direction'):
            publisher = node.create_publisher(Vector3Stamped, topic, 10)
            run(0.5)
            previous_count = len(received['ekf'])
            msg = Vector3Stamped()
            msg.header.frame_id = 'table_mic_link'
            msg.vector.x, msg.vector.y, msg.vector.z = direction
            for _ in range(3):
                msg.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(msg)
                run(0.2)
            assert len(received['ekf']) > previous_count, f'{topic} regression'
            node.destroy_publisher(publisher)
        stop(ekf)
        run(1, publish=False)
        received['ekf'].clear()
        result['ekf_opt_out'] = True
        result['existing_ekf_sources'] = True

        # Direct tracker receives only the new direction topic.
        before = positions()
        tracker = start('direct_tracker', 'ros2', 'run', 'dracoviloc_tracking', 'arm_audio_tracker',
                        '--ros-args', '-p', 'ekf_enabled:=false',
                        '-p', 'direct_classifier_source:=mobilenetv2',
                        '-p', 'require_home_before_tracking:=false')
        enable_tracking()
        run(7)
        after = positions()
        np.testing.assert_allclose(after[[1, 2, 4, 5]], before[[1, 2, 4, 5]], atol=1e-6)
        assert np.max(np.abs(after[[0, 3]] - before[[0, 3]])) > 0.02, 'direct tracker did not move mock joints'
        result['direct_joint_change'] = (after - before).tolist()
        run(3, publish=False)
        hold = positions()
        run(1, publish=False)
        np.testing.assert_allclose(positions(), hold, atol=0.01)
        result['stale_target_hold'] = True
        stop(tracker)

        direction[:] = [0.8, -0.6, 0.0]
        before = positions()
        ekf = start('ekf_enabled', 'ros2', 'launch', 'dracoviloc_ekf', 'ekf.launch.py',
                    'mobilenetv2_ekf_enabled:=true')
        tracker = start('ekf_tracker', 'ros2', 'run', 'dracoviloc_tracking', 'arm_audio_tracker',
                        '--ros-args', '-p', 'require_home_before_tracking:=false')
        enable_tracking()
        run(7)
        assert received['ekf'], 'enabled MobileNetV2 input produced no EKF output'
        topics = dict(node.get_subscriber_names_and_types_by_node('dracoviloc_ekf', '/'))
        assert '/mobilenetv2/direction' in topics
        out = received['ekf'][-1].vector
        np.testing.assert_allclose([out.x, out.y, out.z], direction, atol=1e-5)
        after = positions()
        np.testing.assert_allclose(after[[1, 2, 4, 5]], before[[1, 2, 4, 5]], atol=1e-6)
        assert np.max(np.abs(after[[0, 3]] - before[[0, 3]])) > 0.02, 'EKF tracker did not move mock joints'
        result['ekf_joint_change'] = (after - before).tolist()
        assert received['command'], 'no trajectory commands'
        result['ekf_opt_in'] = True
        result['trajectory_commands'] = len(received['command'])
        result['passed'] = True
    finally:
        for process, stream in reversed(processes):
            stop(process)
            stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (LOGS / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        print(f'Logs: {LOGS}', flush=True)


if __name__ == '__main__':
    main()
