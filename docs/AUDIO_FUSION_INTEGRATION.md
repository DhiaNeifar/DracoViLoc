# Audio fusion integration (ODAS + MobileNetV2 + YOLO + EKF)

This pipeline classifies ODAS acoustic tracks with MobileNetV2, fuses the
classified bearings with YOLO camera rays in `dracoviloc_ekf`, and publishes
`/ekf/direction`, which `arm_tracker` (`dracoviloc_tracking`) consumes when
`tracking_mode:=ekf`.

```
UMA-16 ──► ODAS (dracoviloc_odas, full-band, soundcard capture)
             │
             ├─► /ssl  raw potentials, monitoring only
             ├─► /sst  tracked bearings (4 fixed slots)
             └─► /sss  separated audio (4 ch, channel N ↔ /sst slot N)
                    │
                    ▼
        mobilenetv2_classifier ──► /mobilenetv2/direction ─┐
                                                           ▼
Isaac ROS YOLO ──► /yolo/direction ─────────────────► dracoviloc_ekf
                                                           │
                                                           ▼
                                                    /ekf/direction
                                                           │
                                                           ▼
                                        arm_tracker (dracoviloc_tracking)
                                                           │
                                                           ▼
                        /arm_tracking_controller/commands (or dual_*)
```

AST and GRE still exist and publish `/ast/direction` and `/gre/direction`,
but they are not EKF inputs. They only drive the `direct_ast`, `direct_gre`
and `direct_either` tracking modes.

---

## Running

```bash
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash

ros2 launch dracoviloc_bringup arm_demo.launch.py \
  audio_enabled:=true mobilenetv2_enabled:=true mobilenetv2_ekf_enabled:=true \
  yolo_enabled:=true fusion_enabled:=true tracking_mode:=ekf use_rviz:=true
```

YOLO itself runs in the Isaac ROS container; see `instruction.md`.

Verify the chain:

```bash
ros2 topic hz /sss
ros2 topic echo /mobilenetv2/direction --once
ros2 topic echo /yolo/direction --once
ros2 topic echo /ekf/direction
```

---

## Frame requirement

The EKF does not rotate its inputs through TF. It assumes every input is
already expressed in `tracking_frame` (default `table_mic_link`) and stamps
`/ekf/direction` with that frame. `arm_tracker` then looks up
`world -> table_mic_link`, which `arm_demo.launch.py` publishes as a static
transform (`table_mic_*` arguments).

ODAS is launched with `frame_id:=table_mic_link`. YOLO must be launched with
`direction_frame:=table_mic_link`; its own default (`uma16_camera_direction`)
has no TF.

---

## Topic and frame contract

| Topic | Type | Frame | Published by |
|---|---|---|---|
| `/ssl`, `/sst`, `/sss` | ODAS msgs / `AudioFrame` | `table_mic_link` | `dracoviloc_odas` |
| `/mobilenetv2/direction` | `geometry_msgs/Vector3Stamped` (unit vector) | `table_mic_link` | `dracoviloc_mobilenetv2` |
| `/yolo/direction` | `geometry_msgs/Vector3Stamped` (unit vector) | `direction_frame` argument | `isaac_ros_yolo_direction` |
| `/first_arrival/candidates` | `odas_ros_msgs/OdasSslArrayStamped` | `table_mic_link` | `dracoviloc_odas` (first-arrival mode only) |
| `/ekf/direction` | `geometry_msgs/Vector3Stamped` (unit vector) | `tracking_frame` | `dracoviloc_ekf` |

Every direction is bearing-only. The UMA16v2 and the camera estimate
direction, not range, which is enough for slew-to-cue.

---

### Bearing continuity gates

Each audio classifier (`ast_classifier_node`, `gre_classifier_node`,
`mobilenetv2_classifier_node`) gates its published direction stream
(`/ast/direction`, `/gre/direction`, `/mobilenetv2/direction`) for bearing
continuity. ODAS can retarget an SST track to a wall reflection while keeping
the same track id, and reverberant drone audio still classifies as a drone —
without a gate the published bearing snaps to the reflection the moment the
direct path weakens (a far-drone failure mode observed in testing). A
classified bearing more than `max_bearing_jump_deg` (default 60 deg) from the
last published one is held back unless a consistent challenger (same 15 deg
cone) persists for `bearing_confirm` (3) consecutive windows, in which case
the gate retargets; after `bearing_timeout` (2 s) of silence it re-arms.
`bearing_gate:=false` disables it per classifier.

When `tracking_mode:=ekf`, bringup disables those classifier-local gates and
enables the adaptive gate in `dracoviloc_ekf` instead. The EKF gate compares
each acoustic direction with the predicted direction, combines a bounded
elapsed-time motion allowance with the covariance-aware NIS, and defers a
large inconsistent jump. The 70-degree absolute cap is enforced even when
prediction uncertainty makes NIS permissive. Five distinct, spatially
consistent observations spanning at least 2.0 seconds can confirm a real relocation; a gap longer
than 10 seconds expires old challenger evidence but does not automatically
trust the next bearing. A recent visual update stops
audio alone from forcibly resetting the fused state. All thresholds are
exposed as `bearing_gate_*` launch arguments on `arm_demo.launch.py`.

The gate cannot prove that a path is direct. A persistent reflection can be
accepted during recovery, while a genuine abrupt maneuver can be delayed.
Use labeled field recordings to tune the defaults for the installation.

### First-arrival direct-path validation

For stronger reflection resistance, set `first_arrival_mode:=shadow` and then
`enforce` after calibration. Shadow/enforce modes expose the synchronized
UMA-16 `/raw` stream to a passive onset estimator while ODAS consumes those
same frames. Confident earliest-arrival fits appear on
`/first_arrival/candidates` and as a green RViz arrow. They authorize
classified acoustic bearings but never replace ODAS or MobileNetV2.

In enforce mode, acoustic initialization and movement outside the last
validated anchor are rejected without matching direct-path evidence. Lack of
evidence therefore holds the EKF instead of falling back to the strongest
ODAS peak. Use `first_arrival_raw_recording_path` to capture temporary
16-channel calibration data; leave it empty during normal operation.

---

## Calibrating the microphone yaw

`table_mic_yaw` (`arm_demo.launch.py`, currently `π`, chosen for the
mount geometry, not measured against a real bearing) is the single most
likely source of "everything upstream is correct and the arm still points
off by a constant angle." With ODAS running (`audio_enabled:=true`,
`fusion_enabled:=false` is enough — classification isn't needed for this),
put a sustained source on the robot's `+x` and read the median azimuth from
`/ssl`:

```bash
ros2 topic echo /ssl
```

Set `table_mic_yaw := -median` in the launch invocation. **Then verify at a
second bearing 90° away**: the same offset at both bearings means a
mounting rotation (fix via yaw); a different offset at each means a
mic-mapping fault that no yaw value will fix.

A planar array also cannot distinguish `+z` from `-z` elevation — a source
at +40° and its mirror at −40° produce identical inter-mic delays. This does
not affect azimuth (`atan2(y,x)` is invariant under the mirror), which is
why bearing-only slew-to-cue still works for sources behind the array.

---

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `/ekf/direction` never appears | No EKF input is enabled or publishing. Check `/mobilenetv2/direction` (needs a sustained in-band source) and `/yolo/direction` (published only while YOLO detects). |
| `arm_tracker` warns about a missing `table_mic_link` transform | The static transform is published only when `audio_enabled` or `yolo_enabled` is true. |
| Arm points consistently off by a fixed angle | `table_mic_yaw` uncalibrated; see above. |
| `'numpy._DTypeMeta' object is not subscriptable'` | A pip package in a venv pulled in NumPy ≥ 2; downgrade that package, never NumPy. |
| `/ssl` full but `/sst` empty | SSL candidates did not pass the SST active/inactive energy model and the `theta_new`/`theta_prob` gates for `N_prob` frames. `ssl.probMin` trains search-window deltas at startup; it is not the runtime SST acceptance threshold. |

---

## Related documentation

- [`../README.md`](../README.md) — build and top-level launch commands
- [`../src/dracoviloc_ekf/README.md`](../src/dracoviloc_ekf/README.md) — EKF parameters and gates
- [`../src/dracoviloc_mobilenetv2/README.md`](../src/dracoviloc_mobilenetv2/README.md) — MobileNetV2 classifier
- [`../src/dracoviloc_odas/README.md`](../src/dracoviloc_odas/README.md) — ODAS/UMA-16 capture on its own
- [`../src/dracoviloc_tracking/README.md`](../src/dracoviloc_tracking/README.md) — `arm_tracker` servo behavior
