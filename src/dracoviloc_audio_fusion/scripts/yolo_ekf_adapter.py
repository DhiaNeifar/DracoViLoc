#!/usr/bin/env python3
"""
yolo_ekf_adapter.py  -  /yolo/directions  ->  /camera/yolo_detection

PLACEHOLDER - see "WHAT THIS DOES NOT DO YET" below before trusting it on a
multi-target scene.

The EKF is written against a single-target, portable message: one
PointStamped per frame, header.frame_id 'drone' or 'none'. isaac_ros_yolo_direction
publishes something else - a project-specific PoseArray of ALL detected
boxes as camera-frame rays, with no confidence (Detection2DArray carries a
score; PoseArray does not, so it is lost one hop upstream, in
direction_publisher.py). This node is the only place that knows about both,
same division of responsibility as odas_ekf_adapter.py for the acoustic side.

    /yolo/directions (PoseArray, N rays per message, pose.position = (x, y, 1)
                       camera-frame ray, x/y already carrying the direction
                       publisher's sign/axis convention)
        -> /camera/yolo_detection (PointStamped, one message per frame)

ANGLE, NOT SLOPE
----------------
pose.position is an UNNORMALIZED ray with z pinned to 1.0, i.e.
(tan(theta_h), tan(theta_v), 1). The EKF's visual measurement model
(_h_visual in ekf_fusion_node.py) predicts actual angles via
atan2(ux, uz)/atan2(uy, uz), so this node applies the same atan2 rather than
forwarding the raw ray - passing the tangent through unconverted would be a
small-angle approximation that quietly degrades at the edges of the FOV.

WHAT THIS DOES NOT DO YET
--------------------------
With no confidence to rank by, and no established target identity to track
against, "one detection per frame" is resolved by taking the ray closest to
boresight (smallest angular offset). That is a reasonable placeholder for a
single drone approximately centered by a servo loop already converging on
it, and it is WRONG the moment a second object (a bird, a reflection, a
second drone) sits closer to center than the real target. Do not trust this
past a single-target bench test. Real integration needs the association this
node currently skips: gating candidates by proximity to the EKF's current
predicted bearing (available on /fused_target_pose) rather than to the
image center.

FRAME
-----
This node does not know or care what frame_id direction_publisher.py stamped
the input with (default 'uma16_camera_direction') - PointStamped.header.frame_id
is repurposed for the 'drone'/'none' sentinel, exactly as odas_ekf_adapter.py
repurposes it for a track id. The rotation from tracking_frame to the EKF's
camera_frame parameter is looked up via TF inside ekf_fusion_node itself, so
camera_frame there MUST name the same physical frame direction_publisher's
rays are actually expressed in, or the EKF will rotate them wrong.

USAGE
-----
    python3 yolo_ekf_adapter.py
    python3 yolo_ekf_adapter.py --ros-args -p input_topic:=/yolo/directions
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, PoseArray


class YoloEkfAdapter(Node):
    def __init__(self):
        super().__init__('yolo_ekf_adapter')

        self.input_topic = self.declare_parameter(
            'input_topic', '/yolo/directions').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/camera/yolo_detection').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(PointStamped, self.output_topic, qos)
        self.create_subscription(PoseArray, self.input_topic, self._cb, qos)

        self.seen = 0
        self.with_detection = 0
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'PLACEHOLDER: {self.input_topic} -> {self.output_topic}, '
            f'nearest-to-boresight selection, no target association yet')

    def _report(self):
        if self.seen == 0:
            self.get_logger().warn(
                f'no {self.input_topic} - is the YOLO direction publisher '
                f'running in the Isaac ROS container?')
        elif self.with_detection == 0:
            self.get_logger().warn(
                f'{self.seen} frames, no detection yet')

    def _cb(self, msg: PoseArray):
        self.seen += 1
        out = PointStamped()
        out.header.stamp = msg.header.stamp

        if not msg.poses:
            out.header.frame_id = 'none'
            self.pub.publish(out)
            return

        # Placeholder target selection - see WHAT THIS DOES NOT DO YET above.
        best = min(msg.poses,
                  key=lambda pose: pose.position.x ** 2 + pose.position.y ** 2)

        out.header.frame_id = 'drone'
        out.point.x = math.atan2(best.position.x, best.position.z)
        out.point.y = math.atan2(best.position.y, best.position.z)
        self.pub.publish(out)
        self.with_detection += 1


def main():
    rclpy.init()
    node = YoloEkfAdapter()
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
