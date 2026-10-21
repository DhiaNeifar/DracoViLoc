#!/usr/bin/env python3
"""Publish direct-path candidates from synchronized UMA-16 raw audio."""

import csv
import math
import os
import sys
import wave

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from audio_utils_msgs.msg import AudioFrame
from odas_ros_msgs.msg import OdasSsl, OdasSslArrayStamped

from first_arrival import FirstArrivalEstimator


class FirstArrivalNode(Node):
    def __init__(self):
        super().__init__("first_arrival_node")
        self.frame_id = self.declare_parameter("frame_id", "table_mic_link").value
        self.min_confidence = self.declare_parameter("min_confidence", 0.70).value
        diagnostics_path = self.declare_parameter("diagnostics_path", "").value
        raw_recording_path = self.declare_parameter("raw_recording_path", "").value
        self.estimator = FirstArrivalEstimator(
            sample_rate=44100,
            sound_speed=self.declare_parameter("sound_speed", 343.0).value,
            low_hz=self.declare_parameter("low_hz", 180.0).value,
            high_hz=self.declare_parameter("high_hz", 3600.0).value,
            noise_window_s=self.declare_parameter("noise_window_s", 1.0).value,
            onset_snr_db=self.declare_parameter("onset_snr_db", 10.0).value,
            max_arrival_span_ms=self.declare_parameter("max_arrival_span_ms", 1.2).value,
            residual_us=self.declare_parameter("residual_us", 150.0).value,
            min_inliers=self.declare_parameter("min_inliers", 12).value,
            min_confidence=self.min_confidence,
            refractory_ms=self.declare_parameter("refractory_ms", 50.0).value,
        )
        qos = QoSProfile(
            depth=8, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(
            OdasSslArrayStamped, "/first_arrival/candidates", 10)
        self.create_subscription(AudioFrame, "/raw", self._audio, qos)
        self.frames = 0
        self.estimates = 0
        self.csv_file = None
        self.csv_writer = None
        self.raw_wave = None
        if diagnostics_path:
            os.makedirs(os.path.dirname(os.path.abspath(diagnostics_path)), exist_ok=True)
            self.csv_file = open(diagnostics_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "time_sec", "x", "y", "z", "confidence", "inliers",
                "residual_us", "onset_snr_db"])
        if raw_recording_path:
            os.makedirs(os.path.dirname(os.path.abspath(raw_recording_path)), exist_ok=True)
            self.raw_wave = wave.open(raw_recording_path, "wb")
            self.raw_wave.setnchannels(16)
            self.raw_wave.setsampwidth(4)
            self.raw_wave.setframerate(44100)
        self.create_timer(5.0, self._status)
        self.get_logger().info(
            f"First-arrival estimator ready: confidence>={self.min_confidence:.2f}, "
            "hold-on-ambiguity input")

    def _audio(self, message):
        self.frames += 1
        if (message.format != "signed_32" or message.channel_count != 16 or
                message.sampling_frequency != 44100 or message.frame_sample_count == 0):
            self.get_logger().warning(
                "Ignoring /raw frame: expected signed_32, 16 channels, 44100 Hz",
                throttle_duration_sec=5.0)
            return
        expected = message.frame_sample_count * message.channel_count * 4
        if len(message.data) != expected:
            self.get_logger().warning(
                "Ignoring malformed /raw payload", throttle_duration_sec=5.0)
            return
        payload = bytes(message.data)
        if self.raw_wave is not None:
            self.raw_wave.writeframesraw(payload)
        samples = np.frombuffer(payload, dtype="<i4").reshape(-1, 16)
        samples = samples.astype(np.float64) / float(2 ** 31 - 1)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        start = stamp - message.frame_sample_count / message.sampling_frequency
        estimate = self.estimator.process(samples, start)
        if estimate is None:
            return
        self.estimates += 1
        output = OdasSslArrayStamped()
        output.header.stamp = Time(seconds=estimate.timestamp).to_msg()
        output.header.frame_id = message.header.frame_id or self.frame_id
        source = OdasSsl()
        source.x, source.y, source.z = map(float, estimate.direction)
        source.e = float(estimate.confidence)
        output.sources.append(source)
        self.publisher.publish(output)
        if self.csv_writer is not None:
            self.csv_writer.writerow([
                estimate.timestamp, *estimate.direction, estimate.confidence,
                estimate.inliers, estimate.residual_us, estimate.onset_snr_db])
            self.csv_file.flush()
        x, y, z = estimate.direction
        self.get_logger().info(
            f"first arrival direction=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"confidence={estimate.confidence:.2f} inliers={estimate.inliers} "
            f"residual={estimate.residual_us:.0f}us "
            f"snr={estimate.onset_snr_db:.1f}dB")

    def _status(self):
        self.get_logger().info(
            f"First-arrival status: raw_frames={self.frames} estimates={self.estimates}")
        if self.frames == 0:
            self.get_logger().warning("No /raw audio; check feeder/socket capture mode")

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        if self.raw_wave is not None:
            self.raw_wave.close()
            self.raw_wave = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = FirstArrivalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
