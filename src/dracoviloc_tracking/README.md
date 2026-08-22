# dracoviloc_tracking

Arm-pointing controllers for DracoViLoc. `arm_audio_tracker` consumes the
filtered audio direction and publishes short continuous trajectory commands.
Only joints 1 and 4 move; joints 2, 3, 5 and 6 are locked when tracking starts.

Recommended launch:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=true
```

Inputs are `/audio/target_direction`, `/audio/target_valid`, and
`/joint_states`; commands are published to `/arm_controller/joint_trajectory`.
The controller applies direction averaging, velocity/acceleration limits and
immediate target updates without MoveIt planning.
