# dracoviloc_tracking

Arm-pointing controllers for DracoViLoc. `arm_audio_tracker` consumes the
EKF direction and publishes a continuous, jerk-limited position stream. Only
joint1 (yaw) and joint4 (elevation) move; joints 2, 3, 5 and 6 are captured and
locked when tracking starts. This joint choice follows the current robot
kinematics: joint6 rotates about the microphone pointing axis and therefore
cannot change its bearing.

For hardware demos, the tracker starts disabled. It will not publish a
tracking trajectory until home has completed and tracking is explicitly armed:

```bash
# Send the SRDF `home` pose. Wait for the node to report "Home reached".
ros2 service call /demo/home std_srvs/srv/Trigger "{}"

# Arm only fresh EKF directions; previous EKF data is discarded.
ros2 service call /demo/tracking std_srvs/srv/SetBool "{data: true}"

# Stop tracking and issue a one-time hold at the current measured position.
ros2 service call /demo/tracking std_srvs/srv/SetBool "{data: false}"
```

The home pose is joint1=+90°, joint2=-90°, joint3=-90°, joint4=0°,
joint5=+90°, joint6=0°. The home service sends this pose through
`arm_controller`; it does not ask MoveIt to plan a collision-checked path.
Validate the route in mock mode and keep the physical arm's working area clear
before using it on hardware.

For the physical arm, tracking switches atomically from MoveIt's
`arm_controller` to `arm_tracking_controller`, a
`forward_command_controller` on the same position interfaces. The tracker
runs online Ruckig at 100 Hz (the same rate as `controller_manager`), carrying
position, velocity, acceleration and jerk from one cycle to the next. It sends
the six joint positions on `/arm_tracking_controller/commands`; no trajectory
is repeatedly preempted and no endpoint is implicitly forced to zero velocity.
When tracking is disabled, Ruckig decelerates to rest and the node switches
back to `arm_controller`, so MoveIt and RViz can be used normally.

`max_velocity`, `max_acceleration` and `max_jerk` are the online servo limits.
`command_rate_hz` defaults to 100 Hz and should match the controller manager
update rate. `command_horizon` remains accepted for compatibility with older
launch commands but is no longer used for tracking. Tracking starts above
`angular_deadband` (0.08 rad by default) and releases below
`angular_deadband_exit` (0.04 rad). If measured hardware position falls more
than `max_tracking_error` (0.35 rad) behind the generated servo state,
tracking stops safely rather than re-anchoring the stream and causing a jump.

To diagnose target jitter, set `ekf_direction_log_path` to a writable CSV
path. In EKF mode, every `/ekf/direction` message is appended with its raw
vector, transformed world vector, filtered vector, angular jump before and
after the TF transform and tracker filtering, and the measured joint1/joint4
positions. `input_jump_deg` measures the EKF message itself;
`world_jump_deg` also includes changes introduced by TF:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  tracking_mode:=ekf fusion_enabled:=true yolo_enabled:=true \
  ekf_direction_log_path:=$HOME/DracoViLoc/runs/ekf_direction.csv
```

An empty path, the default, disables logging. The file is flushed after every
sample so it remains useful if the launch is interrupted.

Set `yolo_direction_log_path` to record every raw `/yolo/direction` sample and
its angular change in a separate CSV. `ekf_average_window` controls the causal
normalized-vector average on the EKF output; its default is 5 and 1 disables
the average.

Recommended launch:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  hardware_mode:=mock audio_enabled:=true ast_enabled:=true \
  fusion_enabled:=true tracking_mode:=ekf use_rviz:=true
```

Inputs are `/ekf/direction` (`Vector3Stamped` from `dracoviloc_ekf`, started
automatically by the bringup when `fusion_enabled:=true`), the legacy
`/audio/target_valid` (an optional external veto), and `/joint_states`.
Home commands are published to `/arm_controller/joint_trajectory`; tracking
commands are published to `/arm_tracking_controller/commands`.
The node applies TF frame rotation, exponential smoothing, deadband hysteresis
and online jerk/acceleration limits. Setting up the upstream classification and
fusion pipeline is documented in
[`docs/AUDIO_FUSION_INTEGRATION.md`](../../docs/AUDIO_FUSION_INTEGRATION.md).
