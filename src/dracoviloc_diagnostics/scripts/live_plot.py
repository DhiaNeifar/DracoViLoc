#!/usr/bin/env python3
"""Live end-effector speed/acceleration plot with an event timeline underneath.

Subscribes to /ee/speed, /ee/acceleration (std_msgs/Float64, published by
event_logger_node) and /diagnostics/events (std_msgs/String, one of
"yolo_in", "mobilenetv2_in", "ekf_out", "target_updated"). Draws a scrolling
window of the last `window_s` seconds; nothing here touches the control loop.
"""

import collections
import threading

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from std_msgs.msg import Float64, String

EVENT_TYPES = ["yolo_in", "mobilenetv2_in", "ekf_out", "target_updated"]
EVENT_COLORS = {
    "yolo_in": "tab:blue",
    "mobilenetv2_in": "tab:green",
    "ekf_out": "tab:orange",
    "target_updated": "tab:red",
}


class LivePlotNode(Node):
    def __init__(self):
        super().__init__("live_plot")
        self.window_s = self.declare_parameter("window_s", 20.0).value
        self.t0 = self.get_clock().now()
        self.lock = threading.Lock()
        self.speed = collections.deque()
        self.accel = collections.deque()
        self.events = collections.deque()

        self.create_subscription(Float64, "/ee/speed", self._on_speed, 20)
        self.create_subscription(Float64, "/ee/acceleration", self._on_accel, 20)
        self.create_subscription(String, "/diagnostics/events", self._on_event, 50)

    def _elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _on_speed(self, msg):
        with self.lock:
            self.speed.append((self._elapsed(), msg.data))

    def _on_accel(self, msg):
        with self.lock:
            self.accel.append((self._elapsed(), msg.data))

    def _on_event(self, msg):
        with self.lock:
            self.events.append((self._elapsed(), msg.data))

    def snapshot(self):
        with self.lock:
            cutoff = self._elapsed() - self.window_s
            while self.speed and self.speed[0][0] < cutoff:
                self.speed.popleft()
            while self.accel and self.accel[0][0] < cutoff:
                self.accel.popleft()
            while self.events and self.events[0][0] < cutoff:
                self.events.popleft()
            return list(self.speed), list(self.accel), list(self.events)


def main():
    rclpy.init()
    node = LivePlotNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, (ax_kin, ax_events) = plt.subplots(
        2, 1, figsize=(11, 6), sharex=True,
        gridspec_kw={"height_ratios": [3, 1]})
    fig.suptitle("End-effector kinematics and tracking events (live)")

    speed_line, = ax_kin.plot([], [], color="tab:blue", label="speed (m/s)")
    ax_kin.set_ylabel("speed (m/s)", color="tab:blue")
    ax_kin.tick_params(axis="y", labelcolor="tab:blue")

    ax_accel = ax_kin.twinx()
    accel_line, = ax_accel.plot([], [], color="tab:orange", label="acceleration (m/s^2)")
    ax_accel.set_ylabel("acceleration (m/s^2)", color="tab:orange")
    ax_accel.tick_params(axis="y", labelcolor="tab:orange")

    ax_kin.legend([speed_line, accel_line], ["speed", "acceleration"], loc="upper left", fontsize=8)

    scatters = {
        e: ax_events.scatter([], [], color=EVENT_COLORS[e], label=e, s=25)
        for e in EVENT_TYPES
    }
    ax_events.set_yticks(range(len(EVENT_TYPES)))
    ax_events.set_yticklabels(EVENT_TYPES)
    ax_events.set_ylim(-0.5, len(EVENT_TYPES) - 0.5)
    ax_events.set_xlabel("time (s)")
    ax_events.set_title("yolo / mobilenetv2 / ekf outputs and arm target updates", fontsize=9)

    def update(_frame):
        speed, accel, events = node.snapshot()
        if speed:
            xs, ys = zip(*speed)
            speed_line.set_data(xs, ys)
        if accel:
            xs, ys = zip(*accel)
            accel_line.set_data(xs, ys)
        for i, etype in enumerate(EVENT_TYPES):
            pts = [(t, i) for t, e in events if e == etype]
            scatters[etype].set_offsets(np.array(pts) if pts else np.empty((0, 2)))

        now = node._elapsed()
        left = max(0.0, now - node.window_s)
        for ax in (ax_kin, ax_accel, ax_events):
            ax.set_xlim(left, max(now, left + 1.0))
        ax_kin.relim()
        ax_kin.autoscale_view(scalex=False, scaley=True)
        ax_accel.relim()
        ax_accel.autoscale_view(scalex=False, scaley=True)
        return speed_line, accel_line

    _anim = FuncAnimation(fig, update, interval=150, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
