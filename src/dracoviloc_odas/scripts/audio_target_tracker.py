#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Vector3Stamped
from odas_ros_msgs.msg import OdasSstArrayStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker


def normalize(vector):
    norm = math.sqrt(sum(component * component for component in vector))
    if norm < 1.0e-9:
        return None
    return tuple(component / norm for component in vector)


def rotate_by_quaternion(vector, quaternion):
    # Efficient q * v * q^-1 for a unit quaternion.
    q = (quaternion.x, quaternion.y, quaternion.z)
    w = quaternion.w
    cross_qv = (
        q[1] * vector[2] - q[2] * vector[1],
        q[2] * vector[0] - q[0] * vector[2],
        q[0] * vector[1] - q[1] * vector[0],
    )
    cross_q_cross_qv = (
        q[1] * cross_qv[2] - q[2] * cross_qv[1],
        q[2] * cross_qv[0] - q[0] * cross_qv[2],
        q[0] * cross_qv[1] - q[1] * cross_qv[0],
    )
    return tuple(
        vector[index] + 2.0 * (w * cross_qv[index] + cross_q_cross_qv[index])
        for index in range(3)
    )


class AudioTargetTracker(Node):
    def __init__(self):
        super().__init__("audio_target_tracker")
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.microphone_frame = self.declare_parameter(
            "microphone_frame", "odas_link").value
        self.target_frame = self.declare_parameter(
            "target_frame", "audio_target").value
        self.target_range = float(self.declare_parameter("target_range", 1.5).value)
        self.minimum_activity = float(self.declare_parameter(
            "minimum_activity", 0.15).value)
        self.switch_hysteresis = float(self.declare_parameter(
            "switch_hysteresis", 0.10).value)
        self.smoothing_alpha = float(self.declare_parameter(
            "smoothing_alpha", 0.20).value)
        self.timeout = float(self.declare_parameter("target_timeout", 0.75).value)

        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if self.target_range <= 0.0:
            raise ValueError("target_range must be positive")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, "/audio/target_pose", 10)
        self.direction_pub = self.create_publisher(
            Vector3Stamped, "/audio/target_direction", 10)
        self.valid_pub = self.create_publisher(Bool, "/audio/target_valid", 10)
        self.marker_pub = self.create_publisher(Marker, "/audio/target_marker", 10)
        self.sst_sub = self.create_subscription(
            OdasSstArrayStamped, "/sst", self.sst_callback, 10)
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)

        self.source_id = None
        self.filtered_direction = None
        self.last_detection_time = None
        self.target_valid = False
        self.get_logger().info(
            f"Audio target: {self.microphone_frame} directions -> "
            f"{self.world_frame} at {self.target_range:.2f} m")

    def select_source(self, sources):
        eligible = [source for source in sources if source.activity >= self.minimum_activity]
        if not eligible:
            return None
        best = max(eligible, key=lambda source: source.activity)
        current = next(
            (source for source in eligible if source.id == self.source_id), None)
        if current is None or best.id == current.id:
            return best
        if best.activity >= current.activity + self.switch_hysteresis:
            return best
        return current

    def sst_callback(self, message):
        source = self.select_source(message.sources)
        if source is None:
            return
        direction_microphone = normalize((source.x, source.y, source.z))
        if direction_microphone is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.microphone_frame, Time())
        except TransformException as exception:
            self.get_logger().warning(
                f"Waiting for {self.world_frame} -> {self.microphone_frame} TF: "
                f"{exception}", throttle_duration_sec=3.0)
            return

        direction_world = normalize(rotate_by_quaternion(
            direction_microphone, transform.transform.rotation))
        if direction_world is None:
            return
        if self.filtered_direction is None:
            self.filtered_direction = direction_world
        else:
            alpha = self.smoothing_alpha
            blended = tuple(
                alpha * direction_world[index]
                + (1.0 - alpha) * self.filtered_direction[index]
                for index in range(3))
            self.filtered_direction = normalize(blended)
            if self.filtered_direction is None:
                return

        self.source_id = source.id
        self.last_detection_time = self.get_clock().now()
        self.target_valid = True
        origin = transform.transform.translation
        target = tuple(
            (origin.x, origin.y, origin.z)[index]
            + self.target_range * self.filtered_direction[index]
            for index in range(3))
        self.publish_target(origin, target, source.activity)

    def publish_target(self, origin, target, activity):
        stamp = self.get_clock().now().to_msg()
        direction = Vector3Stamped()
        direction.header.stamp = stamp
        direction.header.frame_id = self.world_frame
        direction.vector.x, direction.vector.y, direction.vector.z = self.filtered_direction
        self.direction_pub.publish(direction)

        pose = PoseStamped()
        pose.header = direction.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        target_tf = TransformStamped()
        target_tf.header = direction.header
        target_tf.child_frame_id = self.target_frame
        target_tf.transform.translation.x = target[0]
        target_tf.transform.translation.y = target[1]
        target_tf.transform.translation.z = target[2]
        target_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(target_tf)

        marker = Marker()
        marker.header = direction.header
        marker.ns = "audio_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [Point(x=origin.x, y=origin.y, z=origin.z),
                         Point(x=target[0], y=target[1], z=target[2])]
        marker.scale.x = 0.025
        marker.scale.y = 0.06
        marker.scale.z = 0.08
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 1.0
        marker.text = "source={} activity={:.3f}".format(self.source_id, activity)
        self.marker_pub.publish(marker)
        self.valid_pub.publish(Bool(data=True))

    def check_timeout(self):
        if not self.target_valid or self.last_detection_time is None:
            return
        age = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1.0e9
        if age <= self.timeout:
            return
        self.target_valid = False
        self.source_id = None
        self.filtered_direction = None
        self.valid_pub.publish(Bool(data=False))
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = "audio_target"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = AudioTargetTracker()
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
