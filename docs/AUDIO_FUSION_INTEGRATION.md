# Audio fusion integration (ODAS + AST + EKF)

This pipeline classifies ODAS acoustic tracks with AST and fuses them with a
bearing-only EKF into `/fused_target_pose`, which `arm_audio_tracker`
(`dracoviloc_tracking`) consumes directly. It replaces the old
`audio_target_tracker.py` path (raw `/sst` peaks, no classification, no
outlier rejection).

The classification and fusion nodes originated in a separate research
repository. They now live inside this repo as
[`src/dracoviloc_audio_fusion`](../src/dracoviloc_audio_fusion/README.md) —
copied in unmodified and wired into DracoViLoc's own launch tree, so the
whole thing runs from a single `ros2 launch` command with no other workspace
to source.

```
UMA-16 ──► ODAS (dracoviloc_odas, full-band, soundcard capture)
             │
             ├─► /ssl  raw potentials, monitoring only
             │
             ├─► /sst  tracked bearings, frame_id = table_mic_link
             │     │
             │     ▼
             │   odas_ekf_adapter ──────► /odas/sst ─────────────┐
             │                                                   │
             └─► /sss  separated audio (4ch) ─► ast_classifier ─►│
                                                                  ▼
        (pending) isaac_ros_yolo_direction ──────────────► ekf_fusion_node
                                                                  │
                                                                  ▼
                                                        /fused_target_pose
                                                                  │
                                                                  ▼
                                          arm_audio_tracker (dracoviloc_tracking)
                                                                  │
                                                                  ▼
                                            /arm_controller/joint_trajectory
```

All three nodes above the tracker (`odas_ekf_adapter`, `ast_classifier_node`,
`ekf_fusion_node`) live in `dracoviloc_audio_fusion` and are started together
by `dracoviloc_bringup/arm_audio_demo.launch.py` when `fusion_enabled:=true`
(the default).

---

## One-time setup

- **`trt_env`** — a Python venv with TensorRT/pycuda/`transformers` bindings,
  built with `--system-site-packages` so it can also see `rclpy`. Lives at
  `~/DracoViLoc/trt_env`, **not tracked in git** (see root `.gitignore`) —
  it's machine-specific and was copied onto this Jetson directly rather than
  rebuilt from scratch. If you ever need to recreate it:

  ```bash
  python3 -m venv --system-site-packages ~/DracoViLoc/trt_env
  source ~/DracoViLoc/trt_env/bin/activate
  pip install "transformers<5" "pycuda<2025"
  export PATH=$PATH:/usr/src/tensorrt/bin
  ```

  Do **not** install `torch` — `ast_patch.py` shims the three torch calls
  the AST feature extractor still makes, and the generic wheel fails on
  Jetson's BLAS with an undefined `sbgemm_` symbol. JetPack fixes NumPy at
  1.26; any pip package that pulls in NumPy ≥ 2 fails with
  `'numpy._DTypeMeta' object is not subscriptable` — downgrade that package,
  never NumPy.

- **`src/dracoviloc_audio_fusion/models/drone_ast.engine`** — the TensorRT
  engine, also machine-specific and not tracked in git. Copy it here once
  per Jetson (same reasoning as `isaac_ros/models/*.plan` elsewhere in this
  repo: engines are tied to the exact GPU/JetPack/TensorRT/CUDA build).

Build normally:

```bash
cd ~/DracoViLoc
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`colcon build` does not touch `trt_env/` or the engine file — they are
plain files on disk, not colcon packages.

---

## Running

```bash
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash

ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=true fusion_enabled:=true \
  table_mic_roll:=0.0 use_rviz:=true \
  smoothing_alpha:=0.60 max_velocity:=1.5 max_acceleration:=2.5 \
  min_confidence:=0.20 gre_trust:=0.0
```

One terminal, one sourced workspace. This starts, in order: the simulated
arm, ODAS (soundcard capture, full-band), the `world -> table_mic_link`
static transform, `odas_ekf_adapter`, `ast_classifier_node` (under
`trt_env`), `ekf_fusion_node`, and `arm_audio_tracker`.

`fusion_enabled:=false` skips all three fusion nodes if you only want raw
ODAS localization running (e.g. for `ssl_top_monitor.py`-style calibration,
below).

Verify the chain:

```bash
ros2 topic hz /sss                        # ODAS separated audio
ros2 topic echo /audio_classifier/detection --once   # AST is classifying
ros2 topic echo /fused_target_pose
ros2 topic echo /arm_controller/joint_trajectory --once
```

`gre_trust:=0.0` means GRE's vote is zeroed even if it were running — this
launch file does not start GRE at all yet (see Known gaps).

---

## Frame requirement: `tracking_frame` must match the physical mic TF

`ekf_fusion_node.py` does **not** rotate the acoustic measurement through
TF — it reads the ODAS unit vector's `(x, y, z)` directly as
azimuth/elevation in whatever frame ODAS physically measured it in, then
stamps `/fused_target_pose` with `header.frame_id = tracking_frame`.
`arm_audio_tracker.cpp` trusts that stamp and looks up `world -> <that
frame>` via TF.

DracoViLoc's ODAS launch is given `frame_id:=table_mic_link`
(`dracoviloc_odas/audio_bringup.launch.py`), and a static transform
`world -> table_mic_link` is published by the `table_microphone_tf` node in
`arm_audio_demo.launch.py`. `arm_audio_demo.launch.py` already forwards
`tracking_frame:=table_mic_link` into `ast_ekf_fusion.launch.py`
automatically — this only matters if you launch
`dracoviloc_audio_fusion/launch/ast_ekf_fusion.launch.py` standalone against
a differently-configured ODAS instance, in which case override it to match.

Leaving it at the EKF's own default (`"odas"`) would make
`arm_audio_tracker.cpp` look up a TF frame that doesn't exist, log a
throttled warning, and never move the arm — with every upstream node
reporting healthy.

---

## Topic and frame contract

| Topic | Type | Frame | Published by |
|---|---|---|---|
| `/ssl`, `/sst`, `/sss` | ODAS msgs | `table_mic_link` | `dracoviloc_odas` |
| `/odas/sst` | `geometry_msgs/PointStamped` | n/a — `frame_id` carries the track id as a string | `odas_ekf_adapter` |
| `/audio_classifier/detection` | `geometry_msgs/Vector3Stamped` (`x`=track id, `y`=is_drone, `z`=confidence) | — | `ast_classifier_node` |
| `/camera/yolo_detection` | `geometry_msgs/PointStamped` (`x`,`y` = angular error, radians; `z`=1.0; `frame_id`=`"drone"`/`"none"`) | camera optical frame, converted before publish | **pending** — see Known gaps |
| `/fused_target_pose` | `geometry_msgs/PointStamped` (`x`=azimuth, `y`=elevation, radians; `z`=1.0, direction only) | `table_mic_link` | `ekf_fusion_node` |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | — | `arm_audio_tracker` |

`/fused_target_pose` is bearing-only. `point.z` is always `1.0`; it marks a
unit direction, never a range. This is sufficient for slew-to-cue — a target
at 5 m and one at 50 m on the same bearing need the same arm pose.

---

## Calibrating the microphone yaw

`table_mic_yaw` (`arm_audio_demo.launch.py`, currently `π`, chosen for the
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

## Known gaps

**`/camera/yolo_detection` is not published by anything in this repo yet.**
`isaac_ros_yolo_direction` publishes a related but incompatible signal
today:

| | EKF expects (`/camera/yolo_detection`) | `isaac_ros_yolo_direction` publishes today (`/yolo/directions`) |
|---|---|---|
| Message type | `PointStamped`, one message per detection cycle | `PoseArray`, one `Pose` per detection, all detections |
| Angle value | `atan2((u-cx), fx)` — a true angle, radians | `(u-cx)/fx` — the tangent, small-angle approximation, no `atan2` |
| No-detection case | published every frame with `frame_id="none"` | nothing distinguishes "no detection" from "node not running"; empty arrays are silent |
| Target selection | implicit — one message is one target | none — all boxes forwarded, no best-target choice |
| TF requirement | live `table_mic_link -> d435i_link` published continuously | not published |

A small adapter node bridging `/yolo/directions` → `/camera/yolo_detection`
is needed before the EKF's visual fusion path does anything. Until then
`ekf_fusion_node.py` runs acoustic-only (its default behavior with no
`/camera/yolo_detection` publisher present) — nothing breaks, the visual
gate is simply never exercised.

**GRE is not started by `ast_ekf_fusion.launch.py`.** The classifier itself
(`gre_classifier_node.py`) and its dependencies were not brought into this
repo in this pass — only AST. `gre_trust:=0.0` reflects that: even if GRE
were added later, its vote starts at zero weight by default here. Adding it
is a symmetric extension of the same launch file (its own venv, `gre_env/`
— already excluded in `.gitignore` for when it's copied in — and an
`ExecuteProcess` entry identical in shape to AST's).

**The EKF has never processed a live measurement in this integration.**
Validate acoustic-only behavior end to end (watch `/audio_classifier/detection`
and `/fused_target_pose` together with a real sound source) before trusting
the arm's motion around people or equipment.

**Classifier thresholds are unmeasured on this pipeline's actual input.**
AST's published accuracy figures were measured on raw microphone audio; fed
ODAS-separated channels (what `ast_classifier_node.py` actually consumes
here), performance is measured lower in the source project. Treat
`min_confidence` and `--threshold`/`--consecutive` as things to re-tune
against this pipeline's real behavior, not the published model numbers.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| `/fused_target_pose` never appears, `ekf_fusion_node` logs "waiting for a classified drone bearing" | No `/sst` track above `min_confidence` — needs sustained, in-band sound and AST actually running |
| `arm_audio_tracker` logs `no table_mic_link -> world transform` (or similar) forever | `tracking_frame` mismatch — see Frame requirement above |
| Arm points consistently off by a fixed angle | `table_mic_yaw` uncalibrated — see Calibrating the microphone yaw |
| AST process exits immediately with an import error | `trt_env` missing or incomplete on this machine, or `ast_venv_python`/`ast_engine_path` launch args point somewhere that doesn't exist — check `ros2 launch ... --show-args` output for the resolved paths |
| `FileNotFoundError` for the engine | `models/drone_ast.engine` was not copied onto this machine — it is intentionally not in git |
| `'numpy._DTypeMeta' object is not subscriptable'` | A pip package inside `trt_env` pulled in NumPy ≥ 2; downgrade that package, never NumPy |
| `/ssl` full but `/sst` empty | No source held a bearing for `N_prob` frames, or `probMin` too high in `configuration.cfg` |

---

## Related documentation

- [`../README.md`](../README.md) — DracoViLoc build and top-level launch commands
- [`../src/dracoviloc_audio_fusion/README.md`](../src/dracoviloc_audio_fusion/README.md) — package internals, why AST runs via `ExecuteProcess`
- [`../src/dracoviloc_odas/README.md`](../src/dracoviloc_odas/README.md) — ODAS/UMA-16 capture on its own
- [`../src/dracoviloc_tracking/README.md`](../src/dracoviloc_tracking/README.md) — `arm_audio_tracker` servo behavior
