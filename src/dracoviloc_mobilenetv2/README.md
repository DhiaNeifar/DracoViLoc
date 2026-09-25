# MobileNetV2 audio classification

Independent host ROS 2 classifier: `/sss` (44.1 kHz, signed 16-bit, four
separated channels) and `/sst` feed `/mobilenetv2/direction`
(`geometry_msgs/Vector3Stamped`). The output is an accepted unit acoustic
direction, not confidence or position. No microphone is opened by this node.

The ONNX graph includes the audio frontend and softmax. The runtime resamples
PCM to 16 kHz and sends one-second raw waveforms with a half-second hop.
Each track has independent audio buffers and requires two consecutive positive
windows at threshold 0.75. A new track needs two completed windows (approximately 1.6 seconds
including resampling lookahead). Track loss, inactivity, and stale audio reset
state. No direction is emitted for an inactive slot, including when
`always_classify:=true` enables diagnostic inference.

## Runtime and models

Default interpreter: `~/DracoViLoc/trt_env/bin/python3`, verified with NumPy,
SciPy, PyCUDA and TensorRT 10.3 after sourcing ROS. This does not require PyTorch,
Transformers, or sounddevice. Override `venv_python` if moving environments.

Models live in the checkout's `models/mobilenetv2`. CMake records the source
model path at build time; override `engine_path` after relocating the checkout,
or rebuild this package. The FP32 engine passed the 64 supplied reference clips.
The FP16 engine tested during integration failed the probability-error tolerance
and is not selected. Engines must be rebuilt after relevant GPU/runtime changes.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dracoviloc_mobilenetv2 mobilenetv2.launch.py
```

ODAS must already be publishing. Standalone arguments are `engine_path`,
`venv_python`, `channels` (4), `threshold` (0.75), `votes_required` (2),
`vote_window` (2), `min_activity` (0.10), `always_classify` (false),
`sst_timeout` (0.25 s), and `max_audio_age` (0.5 s).

## Integrated modes

Use `audio_enabled:=true mobilenetv2_enabled:=true` in the main bringup.
The classifier and its EKF input are both disabled by default.

- Observation: `tracking_mode:=off fusion_enabled:=false mobilenetv2_ekf_enabled:=false`.
- Direct: `tracking_mode:=direct_mobilenetv2 fusion_enabled:=false mobilenetv2_ekf_enabled:=false`.
- EKF: `tracking_mode:=ekf fusion_enabled:=true mobilenetv2_ekf_enabled:=true`.
- Observation alongside an existing EKF: keep `mobilenetv2_ekf_enabled:=false`.

Bringup exposes `mobilenetv2_threshold`, `mobilenetv2_votes_required`,
`mobilenetv2_vote_window`, `mobilenetv2_engine_path`, and `mobilenetv2_venv_python`.
`min_activity` and `always_classify` are shared with the existing launch.
`direct_either` continues to mean AST/GRE only. Direct MobileNetV2 uses the
existing servo limits, timeout, and joints 1/4. See the root `DracoViLoc.md`
for the complete host/container workflow.

On this 8 GB Jetson, concurrent AST + MobileNetV2 + YOLO startup exceeded
available GPU memory during integration. The operating guide selects
MobileNetV2 + YOLO, with AST/GRE available as alternatives. Use
`CUDA_MODULE_LOADING=LAZY` in the host and container terminals and start YOLO
before the host pipeline. This does not guarantee that additional models fit.

## Validation and replacement

From the repository root:

```bash
~/DracoViLoc/trt_env/bin/python3 \
  src/dracoviloc_mobilenetv2/mobilenetv2/validate_engine.py \
  --engine models/mobilenetv2/drone_fp32.engine \
  --reference models/mobilenetv2/reference_outputs.npz
```

Require max drone-probability error below 0.01 and no threshold-decision
mismatches. To rebuild after a model/runtime change, stop GPU-heavy launches,
run `bash scripts/build_mobilenetv2_engine.sh`, then repeat validation.
Never silently fall back to another engine. Update metadata hashes after
replacing artifacts.

The EKF uses the same noise and innovation gate as its other direction sources;
it does not consume classifier confidence or transform incoming frames.
Simultaneous audio classifiers observe correlated ODAS bearings. Reference
agreement does not establish accuracy on ODAS-separated audio or calibrated
camera/microphone alignment.
