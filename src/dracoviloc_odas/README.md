# DracoViLoc ODAS and UMA16v2

> For the full acoustic pipeline — MobileNetV2 classification and EKF fusion
> into `/ekf/direction`, which is what `arm_tracker` consumes in EKF mode —
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
ros2 launch dracoviloc_bringup arm_demo.launch.py \
  audio_enabled:=true \
  audio_tracking_enabled:=true
```

The physical microphone remains fixed on the table for this hardware-in-the-loop
test. Adjust its world transform with `table_mic_x`, `table_mic_y`,
`table_mic_z`, `table_mic_roll`, `table_mic_pitch`, and `table_mic_yaw`.
Angles are radians.

To inspect audio localization without moving the simulated arm:

```bash
ros2 launch dracoviloc_bringup arm_demo.launch.py \
  audio_enabled:=true tracking_mode:=off
```

## Audio-only launch

```bash
ros2 launch dracoviloc_odas audio_bringup.launch.py \
  use_sim_time:=false microphone_frame:=odas_link
```

ODAS opens the UMA16v2 directly (`raw.interface = soundcard`) rather than
through a filtering feeder, so `/sst`/`/ssl`/`/sss` are full-band — required
by the MobileNetV2, AST and GRE classifiers. With `first_arrival_mode` set to
`shadow` or `enforce`, `uma16_feeder` captures instead and forwards raw frames
to ODAS over a socket. The ODAS configuration
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
- `/mobilenetv2/direction`, `/ast/direction`, `/gre/direction`: bearing of an
  ODAS track that the classifier judged to be a drone.
- `/ekf/direction`: fused bearing (`dracoviloc_ekf`).
- `/arm_tracking_controller/commands`: joint-1/joint-4 servo commands.

The UMA16v2 estimates direction, not range. Marker length and target distance
are visualization conventions.

## Passive first-arrival validation

The optional first-arrival path uses the existing UMA-16 feeder to publish the
same full-band 16-channel frames that ODAS consumes. It detects the earliest
coherent onset across the microphones, robustly fits a plane-wave direction,
and publishes confident results on `/first_arrival/candidates`. It does not
replace ODAS and never commands the arm directly.

Start in observation-only mode:

```bash
ros2 launch dracoviloc_bringup arm_demo.launch.py \
  audio_enabled:=true fusion_enabled:=true tracking_mode:=ekf \
  first_arrival_mode:=shadow
```

After checking the green first-arrival marker and diagnostics, use
`first_arrival_mode:=enforce`. In enforce mode, the EKF holds its existing
prediction when a new acoustic bearing lacks direct-path evidence. Set
`first_arrival_raw_recording_path:=/absolute/path/raw_uma16.wav` only for
calibration; 16-channel signed-32 audio uses roughly 170 MB/minute.

This is a passive onset detector, not proof of a direct path. Continuous rotor
noise without a usable onset, a direct path below threshold, or a reflection
inside the array's temporal resolution produces no trusted candidate. The safe
result in enforce mode is to hold rather than follow unvalidated ODAS output.
