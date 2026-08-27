# DracoViLoc ODAS and UMA16v2

> For the full acoustic pipeline — AST/GRE classification and EKF fusion into
> `/fused_target_pose`, which is what `arm_audio_tracker` actually consumes —
> see [`docs/AUDIO_FUSION_INTEGRATION.md`](../../docs/AUDIO_FUSION_INTEGRATION.md).
> This document covers only ODAS capture and localization on their own.

`dracoviloc_odas` connects ODAS localization to DracoViLoc. ODAS itself,
`odas_ros`, its message packages, microphone geometry, and the retained
configuration are all stored in this repository. No external ODAS workspace
is required. ODAS now opens the UMA-16 directly (`raw.interface = soundcard`
in `configuration.cfg`); the Butterworth-filtered feeder (`uma16_feeder_node.py`)
and `audio_target_tracker.py` remain in `scripts/` but are no longer launched
— see the fusion-integration doc above for why.

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

ODAS opens the UMA16v2 directly (`raw.interface = soundcard`) rather than
through a filtering feeder, so `/sst`/`/ssl`/`/sss` are full-band — required
by the AST classifier in `dracoviloc_audio_fusion`. The ODAS configuration
used at runtime is installed from `src/odas_ros/config/configuration.cfg`.

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

- `/ssl`, `/sst`, `/sss`: raw ODAS localization/tracking/separated-audio output.
- `/audio_classifier/detection`: AST drone verdict per track (`dracoviloc_audio_fusion`).
- `/fused_target_pose`: EKF-fused bearing (`dracoviloc_audio_fusion`).
- `/arm_controller/joint_trajectory`: joint-1/joint-4 servo commands.

The UMA16v2 estimates direction, not range. Marker length and target distance
are visualization conventions.
