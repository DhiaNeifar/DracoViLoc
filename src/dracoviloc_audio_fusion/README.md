# dracoviloc_audio_fusion

AST drone classification and bearing-only EKF fusion of ODAS acoustic tracks,
producing `/fused_target_pose` for `arm_audio_tracker`
(`dracoviloc_tracking`). Originally developed in a separate repository; the
nodes here are unmodified copies, wired into DracoViLoc's own launch tree.

```
/sst ──► odas_ekf_adapter ──► /odas/sst ─────────────────────┐
                                                               │
/sss ──► ast_classifier_node.py ──► /audio_classifier/detection ─┤
                                                               ├─► ekf_fusion_node ─► /fused_target_pose
/yolo/directions ──► yolo_ekf_adapter ──► /camera/yolo_detection ┘
```

Audio (top three arrows) and visual (bottom arrow) are independent inputs to
the same EKF: either can be missing or disabled (`audio_enabled` /
`visual_enabled`) without affecting the other - see AUDIO AND VISUAL ARE
INDEPENDENT in `ekf_fusion_node.py`.

## Contents

- `scripts/odas_ekf_adapter.py` — converts `odas_ros_msgs/OdasSstArrayStamped`
  into portable `PointStamped` on `/odas/sst`. Plain `rclpy`; installed as
  `ros2 run dracoviloc_audio_fusion odas_ekf_adapter`.
- `scripts/ekf_fusion_node.py` — the bearing-only EKF (azimuth, elevation,
  their rates). Gates acoustic updates on classifier confidence, rejects
  outliers via chi-squared, and fuses `/camera/yolo_detection` through an
  independent visual measurement path (`audio_enabled` and `visual_enabled`
  parameters gate each side separately). Installed as `ekf_fusion_node`.
- `scripts/yolo_ekf_adapter.py` — **PLACEHOLDER**, see its module docstring.
  Converts the Isaac ROS container's `/yolo/directions` (`PoseArray`, every
  detected box, no confidence) into the single-target `/camera/yolo_detection`
  the EKF expects, by taking the box nearest to boresight. Does not yet
  associate detections with the EKF's current estimate, so it degrades with
  more than one object in frame. Installed as `yolo_ekf_adapter`.
- `ast/` — `ast_classifier_node.py` plus the `detect_drone_realtime.py`
  module it imports `TRTEngine` from, `ast_patch.py` (a NumPy/torch shim the
  HuggingFace AST feature extractor needs), and `model/` (feature-extractor
  config only — no weights; inference runs from the TensorRT engine).
- `models/drone_ast.engine` — **not tracked in git** (machine-specific
  TensorRT build). Copy it here once per Jetson. See `.gitignore`.
- `launch/ast_ekf_fusion.launch.py` — starts all three nodes.
- `scripts/ast_confidence_monitor.py` — live terminal readout of
  `/audio_classifier/detection`. Prints `NO DRONE` once the last message is
  older than `--timeout` rather than holding a stale confidence value, since
  the topic simply goes silent when nothing is tracked. Installed as
  `ros2 run dracoviloc_audio_fusion ast_confidence_monitor`.

## Why AST doesn't run as a normal ROS node

`ast_classifier_node.py` needs TensorRT, pycuda and `transformers`, which
live only in a Python venv built with `--system-site-packages`
(`trt_env/` at the repo root — not tracked in git either; see the root
`.gitignore`). It cannot be built as an `ament_python` entry point against
the system ROS interpreter. `ast_ekf_fusion.launch.py` instead runs it with
`ExecuteProcess`, invoking the venv's `python3` directly by path. That still
sees `rclpy` and this workspace's message packages, because the venv
inherits `PYTHONPATH` from whatever shell ran `ros2 launch` — no
`source trt_env/bin/activate` step is required at launch time. Activating
the venv is only needed if you want to run `ast_classifier_node.py` by hand
outside of launch (for debugging).

## Running

Normally launched as part of `dracoviloc_bringup arm_audio_demo.launch.py`
(`audio_enabled:=true fusion_enabled:=true`, the defaults). To run this
pipeline on its own against an already-running ODAS instance:

```bash
ros2 launch dracoviloc_audio_fusion ast_ekf_fusion.launch.py \
  tracking_frame:=table_mic_link min_confidence:=0.20 gre_trust:=0.0
```

`tracking_frame` must match the `frame_id` ODAS was started with — see
[`docs/AUDIO_FUSION_INTEGRATION.md`](../../docs/AUDIO_FUSION_INTEGRATION.md)
at the repo root for the full explanation, calibration steps and known gaps.
Pass `visual_enabled:=true` to also run `yolo_ekf_adapter` (PLACEHOLDER,
untested against real detections - see its module docstring) and enable the
EKF's visual path.

## One-time setup this package assumes

- `~/DracoViLoc/trt_env` exists (a `--system-site-packages` venv with the
  pinned `transformers<5`, `pycuda<2025`; no `torch`).
- `models/drone_ast.engine` has been copied onto this machine.

Neither is portable between machines — see the root `.gitignore` and
`docs/AUDIO_FUSION_INTEGRATION.md`.
