# DracoViLoc ODAS and UMA16v2

`dracoviloc_odas` connects the bundled UMA16v2 feeder and ODAS localization to
DracoViLoc. ODAS itself, `odas_ros`, its message packages, microphone geometry,
and the retained configuration are all stored in this repository. No external
ODAS workspace is required.

## Build

```bash
cd ~/DracoViLoc
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Recommended arm and audio test

This launches the simulated FAIRINO arm, fixed table microphone, ODAS, RViz and
the continuous joint-1/joint-4 audio servo:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true \
  audio_tracking_enabled:=true
```

The physical microphone remains fixed on the table for this hardware-in-the-loop
test. Adjust its world transform with `table_mic_x`, `table_mic_y`,
`table_mic_z`, `table_mic_roll`, `table_mic_pitch`, and `table_mic_yaw`.
Angles are radians.

To inspect audio localization without moving the simulated arm:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=false
```

## Audio-only launch

```bash
ros2 launch dracoviloc_odas audio_bringup.launch.py \
  use_sim_time:=false microphone_frame:=odas_link
```

The feeder auto-detects the UMA16v2 and applies the 3000--9000 Hz Butterworth
band configured by the launch wrapper. The ODAS configuration used at runtime
is installed from `src/odas_ros/config/configuration.cfg`.

## Device busy

Only one process may capture the ALSA device. Find its owner with:

```bash
fuser -v /dev/snd/pcmC*D0c
```

Stop the old feeder/ODAS launch before starting another. PulseAudio may also
need to release the device:

```bash
pactl suspend-source \
  alsa_input.usb-miniDSP_UMA16v2_00026-00.multichannel-input 1
```

Repeat with a final `0` after testing to restore desktop audio.

## Important topics

- `/ssl` and `/sst`: raw ODAS localization/tracking output.
- `/audio/target_direction`: filtered world-frame direction.
- `/audio/target_valid`: whether a stable source is available.
- `/audio/target_marker`: green RViz direction marker.
- `/arm_controller/joint_trajectory`: joint-1/joint-4 servo commands.

The UMA16v2 estimates direction, not range. Marker length and target distance
are visualization conventions.
