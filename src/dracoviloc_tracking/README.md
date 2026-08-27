# dracoviloc_tracking

Arm-pointing controllers for DracoViLoc. `arm_audio_tracker` consumes the
EKF-fused audio bearing and publishes short continuous trajectory commands.
Only joints 1 and 4 move; joints 2, 3, 5 and 6 are locked when tracking starts.

Recommended launch:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=true
```

Inputs are `/fused_target_pose` (azimuth, elevation, from
`dracoviloc_audio_fusion`'s EKF, started automatically by the launch above
via `fusion_enabled:=true`), the legacy `/audio/target_valid` (an optional
external veto), and `/joint_states`; commands are published to
`/arm_controller/joint_trajectory`.
The controller applies TF frame rotation, exponential smoothing and
velocity/acceleration limits without MoveIt planning. Setting up the upstream
classification and fusion pipeline is documented in
[`docs/AUDIO_FUSION_INTEGRATION.md`](../../docs/AUDIO_FUSION_INTEGRATION.md).
