# dracoviloc_tracking

Arm-pointing controllers for DracoViLoc. `arm_audio_tracker` consumes the
EKF direction and publishes short continuous trajectory commands. Only joints
1 and 4 move; joints 2, 3, 5 and 6 are locked when tracking starts.

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
joint5=+90°, joint6=0°. The current service sends this pose as one timed
controller trajectory; it does not ask MoveIt to plan a collision-checked
path. Validate the route in mock mode and keep the physical arm's working area
clear before using it on hardware.

For the physical arm, tracking computes corrections from measured joint states
and publishes replacement trajectories at 10 Hz with a 0.20 s horizon. The
FAIRINO driver interpolates those commands through `arm_controller` and sends
rate-limited ServoJ samples at the controller's 100 Hz rate.

Recommended launch:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=true
```

Inputs are `/ekf_fused_target_pose` (`Vector3Stamped` direction from
`dracoviloc_ekf`, started automatically by the launch above
via `fusion_enabled:=true`), the legacy `/audio/target_valid` (an optional
external veto), and `/joint_states`; commands are published to
`/arm_controller/joint_trajectory`.
The controller applies TF frame rotation, exponential smoothing and
velocity/acceleration limits without MoveIt planning. Setting up the upstream
classification and fusion pipeline is documented in
[`docs/AUDIO_FUSION_INTEGRATION.md`](../../docs/AUDIO_FUSION_INTEGRATION.md).
