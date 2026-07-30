#!/usr/bin/env python3
"""Mux keyboard, gamepad, and remote Joy inputs into one /joy command stream."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyMux(Node):
    def __init__(self):
        super().__init__('joy_mux')

        self.declare_parameter('keyboard_topic', '/joy_keyboard')
        self.declare_parameter('gamepad_topic', '/joy_gamepad')
        self.declare_parameter('remote_topic', '/joy_remote')
        self.declare_parameter('output_topic', '/joy')
        self.declare_parameter('threshold', 0.1)
        self.declare_parameter('active_timeout_sec', 0.35)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('min_axes', 8)
        self.declare_parameter('min_buttons', 13)
        self.declare_parameter('command_axes', [0, 1, 4])
        self.declare_parameter('command_buttons', [0, 1, 2, 3, 4, 5, 6, 7])
        self.declare_parameter('remote_mode_buttons', [1, 2, 8])
        self.declare_parameter('calibrate_gamepad', True)
        self.declare_parameter('calibration_samples', 10)
        self.declare_parameter('gamepad_x_axis', 0)
        self.declare_parameter('gamepad_y_axis', 1)
        self.declare_parameter('gamepad_z_axis', 3)
        self.declare_parameter('gamepad_invert_x', False)
        self.declare_parameter('gamepad_invert_y', False)
        self.declare_parameter('gamepad_invert_z', False)

        self.threshold = float(self.get_parameter('threshold').value)
        self.active_timeout_sec = float(self.get_parameter('active_timeout_sec').value)
        self.min_axes = int(self.get_parameter('min_axes').value)
        self.min_buttons = int(self.get_parameter('min_buttons').value)
        self.command_axes = [int(i) for i in self.get_parameter('command_axes').value]
        self.command_buttons = [int(i) for i in self.get_parameter('command_buttons').value]
        self.remote_mode_buttons = [
            int(i) for i in self.get_parameter('remote_mode_buttons').value
        ]
        self.calibrate_gamepad = bool(self.get_parameter('calibrate_gamepad').value)
        self.calibration_samples = int(self.get_parameter('calibration_samples').value)
        self.gamepad_axis_map = {
            0: int(self.get_parameter('gamepad_x_axis').value),
            1: int(self.get_parameter('gamepad_y_axis').value),
            4: int(self.get_parameter('gamepad_z_axis').value),
        }
        self.gamepad_axis_sign = {
            0: -1.0 if bool(self.get_parameter('gamepad_invert_x').value) else 1.0,
            1: -1.0 if bool(self.get_parameter('gamepad_invert_y').value) else 1.0,
            4: -1.0 if bool(self.get_parameter('gamepad_invert_z').value) else 1.0,
        }

        self.active_source = None
        self.last_active_time = 0.0
        self.last_msg = None
        self.neutral_sent = True
        self.axis_offsets = {}
        self.calibration_counts = {}

        output_topic = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(Joy, output_topic, 10)

        self.create_subscription(
            Joy,
            self.get_parameter('keyboard_topic').value,
            lambda msg: self.handle_joy('keyboard', msg),
            10,
        )
        self.create_subscription(
            Joy,
            self.get_parameter('gamepad_topic').value,
            lambda msg: self.handle_joy('gamepad', msg),
            10,
        )
        self.create_subscription(
            Joy,
            self.get_parameter('remote_topic').value,
            lambda msg: self.handle_joy('remote', msg),
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_active)

        self.get_logger().info(
            f"Joy mux publishing {output_topic} from keyboard/gamepad/remote inputs"
        )
        self.get_logger().info(
            "Gamepad axis map to canonical /joy: "
            f"raw[{self.gamepad_axis_map[0]}]->axes[0], "
            f"raw[{self.gamepad_axis_map[1]}]->axes[1], "
            f"raw[{self.gamepad_axis_map[4]}]->axes[4]"
        )

    def handle_joy(self, source, msg):
        msg = self.normalized_msg(msg)
        if source == 'gamepad':
            msg = self.remap_gamepad_axes(msg)
        elif source == 'remote':
            msg = self.remap_remote_buttons(msg)
        msg = self.apply_axis_calibration(source, msg)
        active = self.is_active(msg)
        now = time.monotonic()

        if active:
            if self.active_source != source:
                self.get_logger().info(f"Joy input source: {source}")
            self.active_source = source
            self.last_active_time = now
            self.last_msg = msg
            self.neutral_sent = False
            return

        if self.active_source == source:
            self.last_msg = msg
            self.last_active_time = now
            self.pub.publish(msg)
            self.active_source = None
            self.neutral_sent = True

    def publish_active(self):
        if not self.active_source or self.last_msg is None:
            return

        if time.monotonic() - self.last_active_time > self.active_timeout_sec:
            if not self.neutral_sent:
                self.pub.publish(self.make_neutral_msg())
                self.neutral_sent = True
            self.active_source = None
            self.last_msg = None
            return

        self.pub.publish(self.last_msg)

    def remap_gamepad_axes(self, msg):
        remapped = Joy()
        remapped.header = msg.header
        remapped.axes = [0.0] * max(self.min_axes, len(msg.axes))
        remapped.buttons = list(msg.buttons)

        for canonical_axis, raw_axis in self.gamepad_axis_map.items():
            if raw_axis < len(msg.axes):
                remapped.axes[canonical_axis] = (
                    msg.axes[raw_axis] * self.gamepad_axis_sign[canonical_axis]
                )

        return remapped

    def remap_remote_buttons(self, msg):
        remapped = Joy()
        remapped.header = msg.header
        remapped.axes = list(msg.axes)
        remapped.buttons = list(msg.buttons)

        mode_pressed = any(
            button_index < len(remapped.buttons) and remapped.buttons[button_index] != 0
            for button_index in self.remote_mode_buttons
        )
        if mode_pressed:
            for button_index in self.remote_mode_buttons:
                if button_index < len(remapped.buttons):
                    remapped.buttons[button_index] = 0
            remapped.buttons[1] = 1

        return remapped

    def apply_axis_calibration(self, source, msg):
        if source != 'gamepad' or not self.calibrate_gamepad:
            return msg

        if source not in self.axis_offsets:
            self.axis_offsets[source] = [0.0] * len(msg.axes)
            self.calibration_counts[source] = 0

        count = self.calibration_counts[source]
        if count < self.calibration_samples:
            for axis_index in self.command_axes:
                self.axis_offsets[source][axis_index] += msg.axes[axis_index]
            count += 1
            self.calibration_counts[source] = count
            if count == self.calibration_samples:
                for axis_index in self.command_axes:
                    self.axis_offsets[source][axis_index] /= float(self.calibration_samples)
                self.get_logger().info(
                    f"Gamepad neutral offsets calibrated: {self.axis_offsets[source]}"
                )
            return self.make_neutral_msg()

        calibrated = Joy()
        calibrated.header = msg.header
        calibrated.axes = list(msg.axes)
        calibrated.buttons = list(msg.buttons)
        for axis_index in self.command_axes:
            calibrated.axes[axis_index] -= self.axis_offsets[source][axis_index]
            if abs(calibrated.axes[axis_index]) < self.threshold:
                calibrated.axes[axis_index] = 0.0
            calibrated.axes[axis_index] = max(-1.0, min(1.0, calibrated.axes[axis_index]))
        return calibrated

    def normalized_msg(self, msg):
        normalized = Joy()
        normalized.header = msg.header
        normalized.axes = list(msg.axes)
        normalized.buttons = list(msg.buttons)

        if len(normalized.axes) < self.min_axes:
            normalized.axes.extend([0.0] * (self.min_axes - len(normalized.axes)))
        if len(normalized.buttons) < self.min_buttons:
            normalized.buttons.extend([0] * (self.min_buttons - len(normalized.buttons)))

        return normalized

    def make_neutral_msg(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [0.0] * self.min_axes
        msg.buttons = [0] * self.min_buttons
        return msg

    def is_active(self, msg):
        return (
            any(abs(msg.axes[i]) > self.threshold for i in self.command_axes) or
            any(msg.buttons[i] != 0 for i in self.command_buttons)
        )


def main(args=None):
    rclpy.init(args=args)
    node = JoyMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
