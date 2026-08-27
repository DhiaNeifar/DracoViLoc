#!/usr/bin/env python3
"""
ast_confidence_monitor.py  -  live drone-classifier confidence readout

Subscribes to /audio_classifier/detection (geometry_msgs/Vector3Stamped:
x = track id, y = is_drone, z = confidence) and prints a single, continuously
refreshed status line.

WHY THIS EXISTS
---------------
`ros2 topic echo /audio_classifier/detection` only prints when AST actually
classifies a window (every ~0.5 s, and only while a track is active). When
nothing is tracked, the topic just goes silent - there is no message that
means "no drone". This node runs its own timer so it keeps printing on a
fixed schedule regardless of whether AST is currently publishing, and reports
NO DRONE once the last message is older than --timeout seconds. A stale
confidence value is never displayed as if it were current.

Works against either classifier: --source ast (default) watches AST's
/audio_classifier/detection, --source gre watches GRE's
/gre_classifier/detection - same Vector3Stamped shape for both, so one
script covers either without relying on `--ros-args -r ...:=...` remapping,
which is easy to mis-paste across a wrapped terminal line.

USAGE
-----
    python3 ast_confidence_monitor.py
    python3 ast_confidence_monitor.py --source gre
    python3 ast_confidence_monitor.py --topic /some/other/detection/topic
    python3 ast_confidence_monitor.py --timeout 1.0 --rate 10
"""

import argparse
import shutil
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Vector3Stamped

SPINNER = '|/-\\'
SOURCE_TOPICS = {
    'ast': '/audio_classifier/detection',
    'gre': '/gre_classifier/detection',
}


class AstConfidenceMonitor(Node):
    def __init__(self, args):
        super().__init__('ast_confidence_monitor')
        self.timeout = args.timeout
        self.topic = args.topic or SOURCE_TOPICS[args.source]
        self.start_time = time.monotonic()
        self.tick = 0

        self.last_msg_time = None
        self.last_track_id = 0
        self.last_is_drone = False
        self.last_confidence = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Vector3Stamped, self.topic, self._cb, qos)
        self.create_timer(1.0 / args.rate, self._render)
        print(f'watching {self.topic}, timeout={self.timeout}s', file=sys.stderr)

    def _cb(self, msg: Vector3Stamped):
        self.last_msg_time = time.monotonic()
        self.last_track_id = int(round(msg.vector.x))
        self.last_is_drone = msg.vector.y >= 0.5
        self.last_confidence = msg.vector.z

    @staticmethod
    def _bar(confidence, width=20):
        filled = int(round(max(0.0, min(1.0, confidence)) * width))
        return '#' * filled + '.' * (width - filled)

    def _render(self):
        self.tick += 1
        spinner = SPINNER[self.tick % len(SPINNER)]
        now = time.monotonic()
        fresh = (self.last_msg_time is not None
                 and (now - self.last_msg_time) <= self.timeout)

        # Publisher count distinguishes "the classifier is running but sees
        # no drone" from "it isn't connected at all" - both would otherwise
        # render as the same NO DRONE line, which is the wrong signal to
        # debug from.
        publishers = self.count_publishers(self.topic)

        uptime = now - self.start_time
        if publishers == 0:
            line = f' {spinner} NO PUBLISHER on {self.topic}  (uptime {uptime:.0f}s)'
        elif fresh and self.last_is_drone:
            line = (f' {spinner} DRONE      [{self._bar(self.last_confidence)}] '
                    f'{self.last_confidence * 100:5.1f}%  track {self.last_track_id}')
        elif fresh:
            line = (f' {spinner} no drone   [{self._bar(self.last_confidence)}] '
                    f'{self.last_confidence * 100:5.1f}%  track {self.last_track_id}')
        elif self.last_msg_time is not None:
            line = f' {spinner} NO DRONE  (last track {now - self.last_msg_time:.1f}s ago)'
        else:
            line = f' {spinner} NO DRONE  (nothing classified yet, uptime {uptime:.0f}s)'

        # \r only rewinds to the start of the terminal's CURRENT row, not the
        # start of a logical line that wrapped onto a second row. A line
        # longer than the terminal width would wrap, leave its first row
        # never cleared, and scroll a new fragment every tick - which is
        # exactly the stacked, repeating output this truncation prevents.
        columns = shutil.get_terminal_size(fallback=(80, 24)).columns
        line = line[:max(columns - 1, 0)]
        sys.stdout.write(f'\r{line}\033[K')
        sys.stdout.flush()


def parse_args(argv):
    p = argparse.ArgumentParser()
    p.add_argument('--source', choices=sorted(SOURCE_TOPICS), default='ast',
                   help='which classifier to watch (ignored if --topic is set)')
    p.add_argument('--topic', type=str, default=None,
                   help='override: watch an arbitrary Vector3Stamped topic '
                        'instead of --source\'s default')
    p.add_argument('--timeout', type=float, default=1.5,
                   help='seconds without a message before reporting NO DRONE '
                        '(AST publishes roughly every 0.5s while a track is '
                        'active, so 1.5s tolerates a couple of missed windows)')
    p.add_argument('--rate', type=float, default=5.0,
                   help='display refresh rate, Hz')
    return p.parse_args(argv)


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    args = parse_args(argv)

    rclpy.init()
    node = AstConfidenceMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
