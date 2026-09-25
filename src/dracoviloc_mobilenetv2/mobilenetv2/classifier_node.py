#!/usr/bin/env python3
"""Classify ODAS-separated audio and publish accepted acoustic directions."""
import argparse
import math
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from audio_utils_msgs.msg import AudioFrame
from geometry_msgs.msg import Vector3Stamped
from odas_ros_msgs.msg import OdasSstArrayStamped

from processing import ChannelState, INPUT_RATE, decode_audio


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MobileNetV2Classifier(Node):
    def __init__(self, args, engine=None):
        super().__init__('mobilenetv2_classifier_node')
        self.args = args
        self.channels = [ChannelState(args.threshold, args.votes_required, args.vote_window)
                         for _ in range(args.channels)]
        self.latest_sst = None
        self.last_audio_stamp = None
        self.audio_seen = 0
        self.windows_done = 0
        self.dropped_audio = 0
        self.engine = engine
        try:
            if self.engine is None:
                from trt_engine import TrtEngine
                self.engine = TrtEngine(args.engine_path)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.publisher = self.create_publisher(Vector3Stamped, '/mobilenetv2/direction', qos)
            self.create_subscription(OdasSstArrayStamped, '/sst', self._sst_cb, qos)
            self.create_subscription(AudioFrame, '/sss', self._audio_cb, qos)
            self.create_timer(5.0, self._status)
        except BaseException:
            self.close()
            self.destroy_node()
            raise
        self.get_logger().info(
            f'MobileNetV2 ready: engine={args.engine_path}, {args.channels} channels, '
            f'threshold={args.threshold}, votes={args.votes_required}/{args.vote_window}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _source(self, channel):
        if self.latest_sst is None or channel >= len(self.latest_sst.sources):
            return None
        return self.latest_sst.sources[channel]

    def _active(self, source):
        return (source is not None and source.id > 0 and math.isfinite(source.activity)
                and source.activity >= self.args.min_activity)

    def _identity(self, channel):
        source = self._source(channel)
        return (source.id if source is not None else 0,
                self.latest_sst.header.frame_id if self.latest_sst is not None else '',
                self._active(source))

    def _sst_cb(self, msg):
        self.latest_sst = msg
        for channel, state in enumerate(self.channels):
            state.set_identity(self._identity(channel))

    def _reset_audio(self):
        for state in self.channels:
            state.reset()
        self.last_audio_stamp = None

    def _fresh_source(self, channel, audio_stamp, now):
        source = self._source(channel)
        if not self._active(source) or not self.latest_sst.header.frame_id:
            return False
        stamp = seconds(self.latest_sst.header.stamp)
        return (abs(stamp - audio_stamp) <= self.args.sst_timeout
                and -0.1 <= now - stamp <= self.args.sst_timeout)

    def _audio_cb(self, msg):
        self.audio_seen += 1
        now = self._now()
        stamp = seconds(msg.header.stamp)
        try:
            frames = decode_audio(msg, self.args.channels)
            if not -0.1 <= now - stamp <= self.args.max_audio_age:
                raise ValueError('audio is stale or has a future timestamp')
        except ValueError as exc:
            self.dropped_audio += 1
            self._reset_audio()
            self.get_logger().warning(str(exc), throttle_duration_sec=5.0)
            return

        duration = len(frames) / INPUT_RATE
        if self.last_audio_stamp is not None:
            gap = stamp - self.last_audio_stamp
            if gap < 0 or abs(gap - duration) > self.args.sst_timeout:
                self._reset_audio()
        self.last_audio_stamp = stamp

        for channel, state in enumerate(self.channels):
            state.set_identity(self._identity(channel))
            fresh = self._fresh_source(channel, stamp, now)
            if not fresh and not self.args.always_classify:
                state.reset()
                continue
            identity = state.identity
            for waveform, window_end in state.push(frames[:, channel], stamp - duration):
                # Never drain delayed inference windows into the motion pipeline.
                if not -0.1 <= self._now() - window_end <= self.args.max_audio_age:
                    self.dropped_audio += 1
                    state.reset()
                    break
                probability = float(self.engine.infer(waveform)[1])
                decision = state.vote(probability)
                self.windows_done += 1
                source = self._source(channel)
                self.get_logger().info(
                    f'MobileNetV2 confidence={probability:.3f} '
                    f'decision={"DRONE" if decision else "not-drone"} '
                    f'votes={sum(state.votes)}/{len(state.votes)} '
                    f'track={source.id if source is not None else 0} ch={channel}'
                    f'{" diagnostic-only" if not fresh else ""}')
                if (not decision or identity != self._identity(channel)
                        or not self._fresh_source(channel, stamp, self._now())
                        or self._now() - window_end > self.args.max_audio_age):
                    continue
                direction = np.array([source.x, source.y, source.z], dtype=np.float64)
                norm = np.linalg.norm(direction)
                if not np.isfinite(direction).all() or not math.isfinite(norm) or norm < 1e-6:
                    continue
                out = Vector3Stamped()
                out.header.frame_id = self.latest_sst.header.frame_id
                out.header.stamp = Time(seconds=window_end).to_msg()
                out.vector.x, out.vector.y, out.vector.z = map(float, direction / norm)
                self.publisher.publish(out)

    def _status(self):
        self.get_logger().info(
            f'MobileNetV2 status: audio_frames={self.audio_seen} '
            f'windows={self.windows_done} dropped_audio={self.dropped_audio}')
        if self.audio_seen == 0:
            self.get_logger().warning('No /sss audio: check ODAS and ros2 topic hz /sss')
        elif self.windows_done == 0:
            self.get_logger().warning('Waiting for fresh active /sst tracks and a complete audio window')

    def close(self):
        if self.engine is not None:
            self.engine.close()
            self.engine = None


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine-path', required=True)
    parser.add_argument('--channels', type=int, default=4)
    parser.add_argument('--threshold', type=float, default=0.75)
    parser.add_argument('--votes-required', type=int, default=2)
    parser.add_argument('--vote-window', type=int, default=2)
    parser.add_argument('--min-activity', type=float, default=0.1)
    parser.add_argument('--always-classify', choices=['true', 'false'], default='false')
    parser.add_argument('--sst-timeout', type=float, default=0.25)
    parser.add_argument('--max-audio-age', type=float, default=0.5)
    args = parser.parse_args(argv)
    args.always_classify = args.always_classify == 'true'
    if args.channels <= 0 or not 0 <= args.min_activity <= 1:
        parser.error('channels must be positive and min_activity must be in [0, 1]')
    if not (math.isfinite(args.sst_timeout) and args.sst_timeout > 0
            and math.isfinite(args.max_audio_age) and args.max_audio_age > 0):
        parser.error('sst_timeout and max_audio_age must be finite and positive')
    try:
        ChannelState(args.threshold, args.votes_required, args.vote_window)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def main():
    args = parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    node = None
    rclpy.init()
    try:
        node = MobileNetV2Classifier(args)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A terminal interrupt and launch's forwarded interrupt can arrive together.
        # Finish releasing CUDA before allowing another SIGINT to interrupt cleanup.
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.close()
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_handler)


if __name__ == '__main__':
    main()
