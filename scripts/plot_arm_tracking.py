#!/usr/bin/env python3
"""Plot servo/EKF/target logs from one arm_tracking_sim experiment folder.

Usage:
    python3 scripts/plot_arm_tracking.py artifacts/2026_10_12_14_30_05

Reads servo_log.csv (required), and ekf_log.csv / target_log.csv if present,
from the given experiment directory, and writes PNG figures back into it.
Only matplotlib and the standard library are required -- no pandas.
"""

import csv
import math
import os
import sys


def load_csv(path):
    if not os.path.isfile(path):
        return None
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        return None
    columns = {name: [] for name in rows[0]}
    for row in rows:
        for name, value in row.items():
            try:
                columns[name].append(float(value))
            except (TypeError, ValueError):
                columns[name].append(value)
    return columns


def relative_time(seconds):
    t0 = seconds[0]
    return [s - t0 for s in seconds]


def plot_servo_log(servo, out_dir):
    import matplotlib.pyplot as plt

    t = relative_time(servo["time_sec"])

    fig, axes = plt.subplots(4, 1, figsize=(11, 13), sharex=True)

    ax = axes[0]
    ax.plot(t, [math.degrees(v) for v in servo["target_q1"]], label="target q1", lw=1)
    ax.plot(t, [math.degrees(v) for v in servo["actual_q1"]], label="actual q1", lw=1)
    ax.plot(t, [math.degrees(v) for v in servo["target_q4"]], label="target q4", lw=1, ls="--")
    ax.plot(t, [math.degrees(v) for v in servo["actual_q4"]], label="actual q4", lw=1, ls="--")
    ax.set_ylabel("joint angle (deg)")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Joint angle: target vs. actual")

    ax = axes[1]
    ax.plot(t, [math.degrees(v) for v in servo["output_velocity_q1"]], label="velocity q1", lw=1)
    ax.plot(t, [math.degrees(v) for v in servo["output_velocity_q4"]], label="velocity q4", lw=1)
    ax.set_ylabel("velocity (deg/s)")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Commanded joint velocity")

    ax = axes[2]
    if "output_acceleration_q1" in servo:
        ax.plot(t, [math.degrees(v) for v in servo["output_acceleration_q1"]],
                label="accel q1", lw=1)
        ax.plot(t, [math.degrees(v) for v in servo["output_acceleration_q4"]],
                label="accel q4", lw=1)
        ax.legend(loc="upper right", fontsize=8)
    ax.set_ylabel("acceleration (deg/s^2)")
    ax.set_title("Commanded joint acceleration")

    ax = axes[3]
    if "output_jerk_q1" in servo:
        ax.plot(t, [math.degrees(v) for v in servo["output_jerk_q1"]], label="jerk q1", lw=1)
        ax.plot(t, [math.degrees(v) for v in servo["output_jerk_q4"]], label="jerk q4", lw=1)
        ax.legend(loc="upper right", fontsize=8)
    ax.set_ylabel("jerk (deg/s^3)")
    ax.set_xlabel("time (s)")
    ax.set_title("Derived joint jerk")

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "joint_kinematics.png"), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, servo["pointing_error_deg"], label="pointing error (actual vs. desired)", lw=1)
    ax.plot(t, servo["solution_error_deg"], label="solution error (target vs. desired)", lw=1)
    ax.set_ylabel("error (deg)")
    ax.set_xlabel("time (s)")
    ax.set_title("Tracking error")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "tracking_error.png"), dpi=150)
    plt.close(fig)


def plot_target_vs_ekf(target, ekf, out_dir):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(11, 8))

    ax = axes[0]
    if target is not None:
        t = relative_time(target["time_sec"])
        ax.plot(t, target["true_azimuth_deg"], label="true azimuth", lw=1)
        ax.plot(t, target["published_azimuth_deg"], label="published (noisy) azimuth",
                lw=1, ls="--")
    ax.set_ylabel("azimuth (deg, tracking frame)")
    ax.set_xlabel("time (s)")
    ax.set_title("Commanded target path (local tracking frame)")
    ax.legend(loc="upper right", fontsize=8)

    # world_xyz and filtered_xyz are both already in the world frame, so this
    # isolates the EKF's own smoothing lag without a frame mismatch -- unlike
    # comparing against target_log's tracking-frame azimuth directly.
    ax = axes[1]
    if ekf is not None:
        t_ekf = relative_time(ekf["receipt_sec"])
        az_world = [math.degrees(math.atan2(y, x))
                    for x, y in zip(ekf["world_y"], ekf["world_x"])]
        az_filtered = [math.degrees(math.atan2(y, x))
                       for x, y in zip(ekf["filtered_y"], ekf["filtered_x"])]
        ax.plot(t_ekf, az_world, label="unfiltered measurement (world frame)", lw=1)
        ax.plot(t_ekf, az_filtered, label="EKF-filtered output (world frame)", lw=1)
    ax.set_ylabel("azimuth (deg, world frame)")
    ax.set_xlabel("time (s)")
    ax.set_title("EKF smoothing lag")
    ax.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "target_vs_ekf.png"), dpi=150)
    plt.close(fig)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    experiment_dir = sys.argv[1]

    servo = load_csv(os.path.join(experiment_dir, "servo_log.csv"))
    if servo is None:
        print(f"no servo_log.csv found (or it is empty) in {experiment_dir}")
        sys.exit(1)
    plot_servo_log(servo, experiment_dir)

    target = load_csv(os.path.join(experiment_dir, "target_log.csv"))
    ekf = load_csv(os.path.join(experiment_dir, "ekf_log.csv"))
    if target is not None or ekf is not None:
        plot_target_vs_ekf(target, ekf, experiment_dir)

    print(f"Wrote figures to {experiment_dir}")


if __name__ == "__main__":
    main()
