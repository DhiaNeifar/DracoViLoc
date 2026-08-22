# DracoViLoc

ROS 2 Humble workspace for a single FAIRINO arm with a wrist-mounted D435i
camera and a simple kinematic drone in Gazebo Sim.

Installation instructions: [docs/INSTALL_ROS2_GAZEBO.md](docs/INSTALL_ROS2_GAZEBO.md)

## Build and launch

```bash
cd ~/dracoviloc
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch dracoviloc_bringup drone_tracking_demo.launch.py
```

To add wrist-mounted UMA16v2 sound localization to the same demo:

```bash
ros2 launch dracoviloc_bringup drone_tracking_demo.launch.py audio_enabled:=true
```

The repository includes ODAS, its ROS interfaces, and the UMA16v2 feeder. The
audio bringup applies the retained 3000--9000 Hz Butterworth filter. If
PulseAudio has claimed the UMA16v2,
release it before launching:

```bash
pactl suspend-source alsa_input.usb-miniDSP_UMA16v2_00026-00.multichannel-input 1
```

Restore the desktop audio source after stopping the feeder by repeating the
`pactl suspend-source` command with a final value of `0`.

The audio tracker publishes a green fixed-range world-frame ray and target on
`/audio/target_marker`, `/audio/target_pose`, and the `audio_target` TF. The
range is for visualization only; the UMA16v2 measures direction, not distance.

### Measure and select an acoustic band

Record a quiet baseline and then the phone/drone sound from the same position.
Use `--mono-only` because band selection does not require storing all 16
channels:

```bash
ros2 run dracoviloc_odas uma16_feeder --transport none --bypass \
  --record --mono-only --record-seconds 15 --record-dir measurements/quiet
ros2 run dracoviloc_odas uma16_feeder --transport none --bypass \
  --record --mono-only --record-seconds 15 --record-dir measurements/drone
```

Compare the resulting `*_raw_ch1.wav` files:

```bash
ros2 run dracoviloc_odas analyze_audio_band QUIET.wav DRONE.wav
```

Trial the recommended band with `uma16_feeder --lo LOW --hi HIGH`. To test with
a deliberately identifiable broadband beacon, generate a WAV and copy it to
the phone:

```bash
ros2 run dracoviloc_odas generate_beacon_noise beacon.wav --lo LOW --hi HIGH
```

Run drone keyboard control in another terminal:

```bash
cd ~/dracoviloc
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dracoviloc_drone drone_keyboard_teleop
```

Controls: `w/s` forward/back, `a/d` left/right, `r/f` up/down, `q/e` yaw,
Space stops, `x` resets, and Esc exits. The reset pose is `(0, 0, 1.2)`;
the configured bounds are `x [-2, 3]`, `y [-2, 2]`, and `z [0.4, 2.5]`
metres.

## Tracking

The tracker reads the ground-truth `world -> drone_base_link` transform and
points the `+Z` axis of `d435i_link` toward the drone. MoveIt uses planning
group `arm` and controller `arm_controller`. The desired viewing ray is
published on `/tracking/desired_ray`.

Tracking currently uses current-state-seeded IK followed by short OMPL
trajectories; it is responsive but not continuous MoveIt Servo control. Drone
yaw alone does not request a new arm motion.

Gazebo owns the single global `/clock`. Do not run `drone.launch.py` alongside
`drone_tracking_demo.launch.py`, because each standalone launch owns its own
Gazebo instance and clock bridge.
