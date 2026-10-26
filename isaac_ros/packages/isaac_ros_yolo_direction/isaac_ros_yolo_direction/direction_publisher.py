#!/usr/bin/env python3
"""Publish a confirmed detector target as a camera-frame ray."""

import math

from geometry_msgs.msg import Point, Pose, PoseArray, Vector3Stamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray

from .target_selection import DetectionLock


def map_detection_pixel_to_raw(u, v, raw_camera_info, detection_camera_info):
    """Undo detector resize/letterbox using the two pinhole calibrations."""
    raw_k = raw_camera_info.k
    detection_k = detection_camera_info.k
    return (
        (u - detection_k[2]) * raw_k[0] / detection_k[0] + raw_k[2],
        (v - detection_k[5]) * raw_k[4] / detection_k[4] + raw_k[5],
    )


def raw_pixel_to_ray_slopes(u, v, raw_camera_info):
    """Return horizontal/vertical slopes of a raw camera pixel."""
    k = raw_camera_info.k
    return (u - k[2]) / k[0], (v - k[5]) / k[4]


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
        raw_camera_info_topic = self.declare_parameter(
            'raw_camera_info_topic', '/camera_info').value
        output_topic = self.declare_parameter(
            'output_topic', '/yolo/directions').value
        direction_topic = self.declare_parameter(
            'direction_topic', '/yolo/direction').value
        marker_topic = self.declare_parameter(
            'marker_topic', '/yolo/target_marker').value
        self._arrow_length = self.declare_parameter('arrow_length', 1.0).value
        self._selection = DetectionLock(
            max_step_px=self.declare_parameter('max_target_step_px', 120.0).value,
            confirmation_frames=self.declare_parameter(
                'target_confirmation_frames', 3).value,
            lost_timeout_s=self.declare_parameter('target_lost_timeout_s', 0.75).value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._detection_camera_info = None
        self._raw_camera_info = None
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, qos)
        self.create_subscription(
            CameraInfo, raw_camera_info_topic, self._raw_camera_info_callback, qos)
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
            self._detection_camera_info = message

    def _raw_camera_info_callback(self, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self._raw_camera_info = message

    def _detections_callback(self, message: Detection2DArray) -> None:
        if self._detection_camera_info is None or self._raw_camera_info is None:
            self.get_logger().warning(
                'Waiting for valid raw and resized camera_info', throttle_duration_sec=5.0)
            return

        output = PoseArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._frame_id

        stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        detection = self._selection.select(message.detections, stamp_s)
        if detection is None:
            self._publisher.publish(output)
            self._publish_marker(output, None)
            return
        raw_u, raw_v = map_detection_pixel_to_raw(
            detection.bbox.center.position.x,
            detection.bbox.center.position.y,
            self._raw_camera_info,
            self._detection_camera_info)
        horizontal, vertical = raw_pixel_to_ray_slopes(
            raw_u, raw_v, self._raw_camera_info)

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
