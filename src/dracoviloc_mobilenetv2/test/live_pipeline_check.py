#!/usr/bin/env python3
"""Opt-in live camera/UMA16 soak check. Uses mock joints and an isolated ROS domain.

Requires the existing Isaac ROS container. Records counters/logs only, not media.
"""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '84'
os.environ['ROS_LOCALHOST_ONLY'] = '0'
os.environ.setdefault('CUDA_MODULE_LOADING', 'LAZY')

import rclpy
from rclpy.qos import qos_profile_sensor_data
from audio_utils_msgs.msg import AudioFrame
from geometry_msgs.msg import Vector3Stamped
from odas_ros_msgs.msg import OdasSstArrayStamped
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory
from vision_msgs.msg import Detection2DArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=600)
    parser.add_argument('--rviz', choices=['true', 'false'], default='false')
    parser.add_argument('--gre', choices=['true', 'false'], default='false')
    parser.add_argument('--ast', choices=['true', 'false'], default='true')
    parser.add_argument('--always-classify', choices=['true', 'false'], default='true')
    args = parser.parse_args()
    root = Path('/tmp') / ('mobilenetv2-live-' + time.strftime('%Y%m%d-%H%M%S'))
    root.mkdir()
    print(f'Logs: {root}', flush=True)
    processes = []
    counts = Counter()
    errors = []
    memory = []
    summaries = []
    config = {}
    result = {'duration_seconds': args.duration, 'settings': vars(args)}

    def start(name, command):
        stream = (root / (name + '.log')).open('w')
        p = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                             start_new_session=True)
        processes.append((name, p, stream))
        return p

    def stop(p):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=10)

    rclpy.init()
    node = rclpy.create_node('mobilenetv2_live_observer')
    def count(key):
        def callback(msg):
            counts[key] += 1
        return callback

    for key, topic, kind in [('sss', '/sss', AudioFrame), ('sst', '/sst', OdasSstArrayStamped),
        ('mobile', '/mobilenetv2/direction', Vector3Stamped), ('ast', '/ast/direction', Vector3Stamped),
        ('gre', '/gre/direction', Vector3Stamped), ('yolo', '/yolo/direction', Vector3Stamped),
        ('ekf', '/ekf/direction', Vector3Stamped), ('joints', '/joint_states', JointState),
        ('commands', '/arm_controller/joint_trajectory', JointTrajectory),
        ('detections', '/detections_output', Detection2DArray), ('image', '/image', Image)]:
        node.create_subscription(kind, topic, count(key), qos_profile_sensor_data)

    def log(msg):
        if 'MobileNetV2 confidence' in msg.msg:
            counts['mobile_inferences'] += 1
        if 'AST confidence' in msg.msg:
            counts['ast_inferences'] += 1
        if 'GRE confidence' in msg.msg:
            counts['gre_confidence_logs'] += 1
        if 'MobileNetV2 status:' in msg.msg:
            summaries.append(msg.msg)
        if msg.level >= 40:  # rcl_interfaces/Log.ERROR is a byte constant in Humble Python
            errors.append({'node': msg.name, 'message': msg.msg})
    node.create_subscription(Log, '/rosout', log, 100)
    try:
        # timeout provides cleanup even if the docker client is interrupted.
        camera_command = (
            'source /opt/ros/humble/setup.bash; '
            'source /workspaces/isaac_ros-dev/install/setup.bash; '
            'export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=0 CUDA_MODULE_LOADING=LAZY; '
            f'exec timeout --signal=INT --kill-after=20s {int(args.duration + 180)}s '
            'ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py '
            'camera:=/dev/video0 width:=640 height:=480 camera_fps:=30 publish_rate:=15.0 '
            'direction_frame:=table_mic_link use_viewer:=false record:=false '
            'model_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_20260825_best.onnx '
            'engine_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_20260825_best.plan')
        camera = start('camera', ['docker', 'exec', '-u', 'admin',
                                  'isaac_ros_dev-aarch64-container', 'bash', '-c', camera_command])
        # Match the documented two-terminal startup order; avoid simultaneous GPU startup peaks.
        camera_started = time.monotonic()
        while time.monotonic() - camera_started < 60:
            rclpy.spin_once(node, timeout_sec=0.1)
            if counts['image'] >= 15 and time.monotonic() - camera_started >= 10:
                break
            if camera.poll() is not None:
                raise RuntimeError('camera startup failed')
        assert counts['image'], 'camera images are not reaching the host'
        host = start('host', ['ros2', 'launch', 'dracoviloc_bringup', 'arm_audio_demo.launch.py',
            'hardware_mode:=mock', 'audio_enabled:=true', 'ast_enabled:=' + args.ast,
            'gre_enabled:=' + args.gre, 'mobilenetv2_enabled:=true',
            'mobilenetv2_ekf_enabled:=true', 'yolo_enabled:=true', 'fusion_enabled:=true',
            'tracking_mode:=ekf', 'always_classify:=' + args.always_classify, 'use_rviz:=' + args.rviz])
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (counts['sss'] and counts['image'] and counts['joints'] and counts['mobile_inferences']
                    and (args.ast == 'false' or counts['ast_inferences'])):
                break
            if host.poll() is not None or camera.poll() is not None:
                raise RuntimeError('launch exited during startup; inspect logs')
            if 'process has died' in (root / 'host.log').read_text(errors='replace'):
                raise RuntimeError('a host child process failed; inspect logs')
        required = ['sss', 'image', 'joints']
        if args.always_classify == 'true':
            required += ['mobile_inferences']
            if args.ast == 'true':
                required += ['ast_inferences']
        assert all(counts[k] for k in required), f'missing startup streams: {dict(counts)}'
        config['ekf_subscriptions'] = dict(node.get_subscriber_names_and_types_by_node('dracoviloc_ekf', '/'))
        assert '/mobilenetv2/direction' in config['ekf_subscriptions']
        controller_check = subprocess.run(['ros2', 'control', 'list_controllers'],
            capture_output=True, text=True, timeout=20)
        config['controllers'] = controller_check.stdout
        assert controller_check.returncode == 0 and controller_check.stdout.count('active') >= 2
        print('Pipeline ready; starting soak timer.', flush=True)
        start_time = time.monotonic()
        next_report = start_time
        while time.monotonic() - start_time < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if host.poll() is not None or camera.poll() is not None:
                raise RuntimeError('launch exited during soak')
            if time.monotonic() >= next_report:
                snapshot = {'elapsed': round(time.monotonic() - start_time), 'counts': dict(counts)}
                print(json.dumps(snapshot), flush=True)
                # Track process RSS using /proc; no settings or processes are changed.
                rss = {}
                for directory in Path('/proc').glob('[0-9]*'):
                    try:
                        command = (directory / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
                        if any(name in command for name in ('classifier_node.py', 'component_container_mt')):
                            status = (directory / 'status').read_text()
                            rss[directory.name] = next(line for line in status.splitlines() if line.startswith('VmRSS:'))
                    except (OSError, StopIteration):
                        pass
                memory.append({'elapsed': snapshot['elapsed'], 'rss': rss})
                next_report += 30
        # MoveIt reports this on the existing setup, which has no depth/octomap plugin.
        known = {'No 3D sensor plugin(s) defined for octomap updates',
                 'Action server: /recognize_objects not available'}
        result['known_moveit_messages'] = [e for e in errors if e['message'] in known]
        result['passed'] = not [e for e in errors if e['message'] not in known]
        result['errors'] = errors
        result['counts'] = dict(counts)
        result['config'] = config
        result['mobile_status'] = summaries
        result['memory'] = memory
    finally:
        # Signal only the camera launch started on this test's isolated domain.
        # docker exec does not forward client signals to its child, so stop in-container.
        subprocess.run(['docker', 'exec', '-u', 'root', 'isaac_ros_dev-aarch64-container',
            'bash', '-c', 'for p in /proc/[0-9]*; do '
            'if [ -r "$p/environ" ] && tr "\\0" "\\n" < "$p/environ" | '
            'grep -qx "ROS_DOMAIN_ID=84" && tr "\\0" " " < "$p/cmdline" | '
            'grep -q "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch isaac_ros_yolo_direction"; '
            'then kill -INT "${p##*/}"; fi; done'], capture_output=True, timeout=20)
        for name, process, stream in reversed(processes):
            stop(process)
            stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result.setdefault('counts', dict(counts))
        result.setdefault('errors', errors)
        (root / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(f'Result: {root / "result.json"}', flush=True)
    if not result.get('passed'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
