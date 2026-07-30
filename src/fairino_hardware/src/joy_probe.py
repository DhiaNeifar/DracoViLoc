#!/usr/bin/env python3
"""Print active axes/buttons from a Joy topic to identify gamepad mapping."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyProbe(Node):
    def __init__(self):
        super().__init__('joy_probe')
        self.declare_parameter('topic', '/joy_gamepad')
        self.declare_parameter('threshold', 0.1)
        self.declare_parameter('print_all', False)

        self.threshold = float(self.get_parameter('threshold').value)
        self.print_all = bool(self.get_parameter('print_all').value)
        topic = self.get_parameter('topic').value
        self.create_subscription(Joy, topic, self.callback, 10)
        self.get_logger().info(f"Probing {topic}. Move one stick/button at a time.")

    def callback(self, msg):
        active_axes = [
            f"axis[{i}]={value:.3f}"
            for i, value in enumerate(msg.axes)
            if self.print_all or abs(value) > self.threshold
        ]
        active_buttons = [
            f"button[{i}]={value}"
            for i, value in enumerate(msg.buttons)
            if self.print_all or value != 0
        ]

        if active_axes or active_buttons:
            self.get_logger().info(
                " | ".join(active_axes + active_buttons)
            )


def main(args=None):
    rclpy.init(args=args)
    node = JoyProbe()
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
