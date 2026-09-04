#!/usr/bin/env python3
"""Publish the highest-confidence YOLO detection as a camera-frame ray."""

import math

from geometry_msgs.msg import Point, Pose, PoseArray, Vector3Stamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray


class DirectionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('yolo_direction_publisher')

        self._x_sign = self.declare_parameter('x_sign', -1.0).value
        self._y_sign = self.declare_parameter('y_sign', -1.0).value
        self._swap_xy = self.declare_parameter('swap_xy', False).value
        self._frame_id = self.declare_parameter(
            'frame_id', 'uma16_camera_direction').value
        detections_topic = self.declare_parameter(
            'detections_topic', '/detections_output').value
        camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/yolov8_encoder/resize/camera_info').value
        output_topic = self.declare_parameter(
            'output_topic', '/yolo/directions').value
        direction_topic = self.declare_parameter(
            'direction_topic', '/yolo/direction').value
        marker_topic = self.declare_parameter(
            'marker_topic', '/yolo/target_marker').value
        self._arrow_length = self.declare_parameter('arrow_length', 1.0).value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._camera_info = None
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, qos)
        self.create_subscription(
            Detection2DArray, detections_topic, self._detections_callback, qos)
        self._publisher = self.create_publisher(PoseArray, output_topic, qos)
        self._direction_publisher = self.create_publisher(
            Vector3Stamped, direction_topic, qos)
        self._marker_publisher = self.create_publisher(Marker, marker_topic, qos)

        self.get_logger().info(
            f'Publishing Vector3Stamped direction on {direction_topic}, '
            f'legacy PoseArray on {output_topic}, '
            f'and RViz arrow on {marker_topic}; '
            f'x_sign={self._x_sign}, y_sign={self._y_sign}, swap_xy={self._swap_xy}')

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self._camera_info = message

    def _detections_callback(self, message: Detection2DArray) -> None:
        if self._camera_info is None:
            self.get_logger().warning(
                'Waiting for valid resized camera_info', throttle_duration_sec=5.0)
            return

        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]

        output = PoseArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._frame_id

        valid_detections = [d for d in message.detections if d.results]
        if not valid_detections:
            self._publisher.publish(output)
            self._publish_marker(output, None)
            return

        detection = max(
            valid_detections,
            key=lambda d: d.results[0].hypothesis.score)
        u = detection.bbox.center.position.x
        v = detection.bbox.center.position.y
        horizontal = (u - cx) / fx
        vertical = (v - cy) / fy

        if self._swap_xy:
            horizontal, vertical = vertical, horizontal

        pose = Pose()
        pose.position.x = self._x_sign * horizontal
        pose.position.y = self._y_sign * vertical
        pose.position.z = 1.0
        pose.orientation.w = 1.0
        output.poses.append(pose)

        self._publisher.publish(output)
        norm = math.sqrt(
            pose.position.x ** 2 + pose.position.y ** 2 + pose.position.z ** 2)
        direction = Vector3Stamped()
        direction.header = output.header
        direction.vector.x = pose.position.x / norm
        direction.vector.y = pose.position.y / norm
        direction.vector.z = pose.position.z / norm
        self._direction_publisher.publish(direction)
        self._publish_marker(output, pose.position)

    def _publish_marker(self, directions: PoseArray, direction: Point) -> None:
        marker = Marker()
        marker.header = directions.header
        # Isaac ROS camera messages use wall time while the DracoViLoc robot
        # simulation uses /clock. A zero stamp tells RViz to use the latest
        # available world->camera transform instead of comparing those clocks.
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = 'yolo_target'
        marker.id = 0

        if direction is None:
            marker.action = Marker.DELETE
            self._marker_publisher.publish(marker)
            return

        norm = math.sqrt(
            direction.x ** 2 + direction.y ** 2 + direction.z ** 2)
        endpoint = Point(
            x=self._arrow_length * direction.x / norm,
            y=self._arrow_length * direction.y / norm,
            z=self._arrow_length * direction.z / norm)

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [Point(), endpoint]
        marker.scale.x = 0.025
        marker.scale.y = 0.06
        marker.scale.z = 0.09
        marker.color.r = 0.0
        marker.color.g = 0.3
        marker.color.b = 1.0
        marker.color.a = 1.0
        self._marker_publisher.publish(marker)


def main() -> None:
    rclpy.init()
    node = DirectionPublisher()
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
