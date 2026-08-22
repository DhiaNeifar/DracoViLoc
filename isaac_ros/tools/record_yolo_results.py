#!/usr/bin/env python3
"""Save synchronized YOLO detections as annotated video and CSV."""

from argparse import ArgumentParser
import csv
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class ResultRecorder(Node):
    def __init__(
        self,
        output_video: Path,
        output_csv: Path,
        class_names: list[str],
        fps: float,
    ) -> None:
        super().__init__('yolo_result_recorder')
        self._bridge = CvBridge()
        self._output_video = output_video
        self._class_names = class_names
        self._fps = fps
        self._writer = None
        self._frame_number = 0

        output_video.parent.mkdir(parents=True, exist_ok=True)
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = output_csv.open('w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            'frame', 'stamp_sec', 'stamp_nanosec', 'class_id', 'class_name',
            'confidence', 'center_x', 'center_y', 'width', 'height',
        ])

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        image_sub = message_filters.Subscriber(
            self, Image, '/yolov8_encoder/resize/image', qos_profile=qos)
        detections_sub = message_filters.Subscriber(
            self, Detection2DArray, '/detections_output', qos_profile=qos)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [image_sub, detections_sub], queue_size=20, slop=0.2)
        self._synchronizer.registerCallback(self._callback)
        self.get_logger().info(f'Recording annotated video to {output_video}')
        self.get_logger().info(f'Recording detections to {output_csv}')

    def _callback(self, image_msg: Image, detections_msg: Detection2DArray) -> None:
        frame = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        if self._writer is None:
            self._writer = cv2.VideoWriter(
                str(self._output_video),
                cv2.VideoWriter_fourcc(*'mp4v'),
                self._fps,
                (frame.shape[1], frame.shape[0]),
            )
            if not self._writer.isOpened():
                raise RuntimeError(f'Could not open output video: {self._output_video}')

        for detection in detections_msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            class_id = int(result.hypothesis.class_id)
            class_name = (
                self._class_names[class_id]
                if class_id < len(self._class_names) else str(class_id)
            )
            box = detection.bbox
            x1 = round(box.center.position.x - box.size_x / 2.0)
            y1 = round(box.center.position.y - box.size_y / 2.0)
            x2 = round(box.center.position.x + box.size_x / 2.0)
            y2 = round(box.center.position.y + box.size_y / 2.0)
            label = f'{class_name} {result.hypothesis.score:.2f}'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame, label, (x1, max(20, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2, cv2.LINE_AA)
            self._csv.writerow([
                self._frame_number,
                detections_msg.header.stamp.sec,
                detections_msg.header.stamp.nanosec,
                class_id,
                class_name,
                result.hypothesis.score,
                box.center.position.x,
                box.center.position.y,
                box.size_x,
                box.size_y,
            ])

        self._writer.write(frame)
        self._csv_file.flush()
        self._frame_number += 1

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
        self._csv_file.close()


def main() -> None:
    rclpy.init()
    parser = ArgumentParser()
    parser.add_argument('--output-video', type=Path, required=True)
    parser.add_argument('--output-csv', type=Path, required=True)
    parser.add_argument('--class-names', nargs='+', required=True)
    parser.add_argument('--fps', type=float, default=30.0)
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])

    node = ResultRecorder(
        args.output_video.resolve(),
        args.output_csv.resolve(),
        args.class_names,
        args.fps,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
