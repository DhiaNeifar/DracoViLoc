#!/usr/bin/env python3
"""
odas_ekf_adapter.py  -  /sst  ->  /odas/sst

The EKF is written against standard geometry_msgs so it stays portable across
projects. odas_ros publishes a project-specific OdasSstArrayStamped. This node
is the only place that knows about both.

    /sst (OdasSstArrayStamped, N sources per message)
        -> /odas/sst (PointStamped, one message per live source)

The track id travels in header.frame_id as a string, which is how the EKF
recovers it - PointStamped has no integer field, and inventing a custom .msg
for one integer would tie the filter to this project.

Dead slots are dropped here rather than in the EKF: id == 0 means an empty
slot, and N_inactive = 250 keeps expired tracks alive at activity 0 for about
2.9 s. Publishing those would make the filter's gating logic responsible for
an odas_ros implementation detail.

USAGE
-----
    python3 odas_ekf_adapter.py
    python3 odas_ekf_adapter.py --ros-args -p min_activity:=0.5
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped
from odas_ros_msgs.msg import OdasSstArrayStamped


class OdasEkfAdapter(Node):
    def __init__(self):
        super().__init__('odas_ekf_adapter')

        self.min_activity = self.declare_parameter('min_activity', 0.3).value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(PointStamped, '/odas/sst', qos)
        self.create_subscription(
            OdasSstArrayStamped, '/sst', self._cb, qos)

        self.seen = 0
        self.forwarded = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'/sst -> /odas/sst, min_activity={self.min_activity}')

    def _report(self):
        if self.forwarded == 0:
            self.get_logger().warn(
                f'{self.seen} /sst messages, nothing forwarded - '
                f'no track above activity {self.min_activity}')

    def _cb(self, msg: OdasSstArrayStamped):
        self.seen += 1
        for s in msg.sources:
            if s.id == 0 or s.activity < self.min_activity:
                continue
            out = PointStamped()
            out.header.stamp = msg.header.stamp
            # Track id, not a coordinate frame. The EKF parses it back to int.
            out.header.frame_id = str(s.id)
            out.point.x = float(s.x)
            out.point.y = float(s.y)
            out.point.z = float(s.z)
            self.pub.publish(out)
            self.forwarded += 1


def main():
    rclpy.init()
    node = OdasEkfAdapter()
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
