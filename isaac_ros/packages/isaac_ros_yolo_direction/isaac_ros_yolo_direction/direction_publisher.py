#!/usr/bin/env python3
"""Publish YOLO bounding-box centers as rays in the camera/UMA-16 frame."""

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray


class DirectionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('yolo_direction_publisher')

        self._x_sign = self.declare_parameter('x_sign', 1.0).value
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

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._camera_info = None
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, qos)
        self.create_subscription(
            Detection2DArray, detections_topic, self._detections_callback, qos)
        self._publisher = self.create_publisher(PoseArray, output_topic, qos)

        self.get_logger().info(
            f'Publishing bounding-box center rays on {output_topic}; '
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

        for detection in message.detections:
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
