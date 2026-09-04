# DracoViLoc

DracoViLoc is a ROS 2 Humble system that combines a FAIRINO arm, MiniDSP
UMA16v2 microphone array, ODAS acoustic localization, AST/GRE audio drone
classification, and Isaac ROS YOLO visual detection on one NVIDIA Jetson Orin
Nano.

The current validated architecture is:

- DracoViLoc, RViz, ODAS, AST, GRE, and physical arm control run on the host.
- Isaac ROS YOLO runs inside its Docker development container.
- The host and container exchange ROS 2 topics through DDS.
- The EKF exists but is deliberately disabled until the individual sensors
  and their frame alignment have been fully validated.

## Current data flow

```text
UMA16v2 -> /sss + /sst -> AST and GRE confidence
                           +-> ODAS points in RViz

USB camera -> Isaac ROS YOLO -> /detections_output
                              -> /yolo/directions
                              -> /yolo/target_marker
                                        +-> RViz and direct_yolo arm servo
```

`direct_yolo` uses the center of the highest-confidence bounding box. It moves
only joints 1 and 4; joints 2, 3, 5, and 6 remain fixed. Joint 4 is constrained
to `[-90 deg, +90 deg]` to avoid folded wrist solutions.

For hardware-in-the-loop simulation, the physical microphone and camera use
the static `table_mic_link`. Moving the simulated arm does not physically move
the sensors, so stamping their measurements as the simulated moving
`odas_link` would create a false feedback loop.

## Repository layout

```text
models/                         Authoritative trained model artifacts
  ast/                          AST weights, configs, and portable ONNX
  gre/                          GRE checkpoint, metadata, ONNX, and engine
  yolo/                         Current and previous YOLO checkpoints
src/                            Host ROS 2 packages
isaac_ros/                      Project-owned Isaac ROS overlay
scripts/export_ast_onnx.py      AST weights-to-ONNX exporter
gre_env/                        Local GRE runtime environment (not committed)
trt_env/                        Local AST runtime environment (not committed)
```

## Model source-of-truth policy

All trained models belong under [`models/`](models/README.md). A file copied
into `src/`, `install/`, an external training repository, or the Isaac
workspace is only a runtime/deployment copy.

When a new model is trained:

1. Copy its portable source artifacts into `models/ast`, `models/gre`, or
   `models/yolo`.
2. Use a filename containing the architecture and preferably a version/date.
3. Export a matching ONNX model where required.
4. Generate a new TensorRT engine on the target Jetson.
5. Copy that engine into the documented runtime location.
6. Update launch paths and documentation if the selected filename changes.
7. Never reuse an engine generated for different weights or ONNX.

TensorRT `.engine` and `.plan` files are machine-specific. Rebuild them after
changing the GPU, JetPack, CUDA, TensorRT, model, input shape, or ONNX graph.

## Host prerequisites and build

```bash
sudo apt update
sudo apt install -y \
  python3.10-venv \
  libportaudio2 \
  portaudio19-dev

cd ~/DracoViLoc
./scripts/build_fairino_sdk.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build in a clean host terminal. Do not source the host copy of the Isaac ROS
workspace before building or running DracoViLoc.

## GRE model

### Artifacts and runtime path

```text
models/gre/model_logmel.pt
models/gre/model_logmel.onnx
models/gre/model_logmel_meta.json
models/gre/model_logmel.engine
```

The ROS launch expects the engine at:

```text
src/dracoviloc_audio_fusion/models/model_logmel.engine
```

### GRE environment

```bash
cd ~/DracoViLoc
python3 -m venv --system-site-packages gre_env
source gre_env/bin/activate

python -m pip install --upgrade pip
python -m pip install pyyaml soundfile sounddevice pandas
```

Confirm the venv can see JetPack and PortAudio:

```bash
grep include-system-site-packages gre_env/pyvenv.cfg
python -c "import pycuda.driver, sounddevice, soundfile, numpy, scipy; print('GRE environment OK')"
```

`include-system-site-packages` must be `true`. Do not install generic CUDA or
PyTorch wheels. They can pull incompatible CUDA 13 packages and fill the
Jetson storage. Prefer the JetPack-provided `pycuda` or the matching Ubuntu
package instead of compiling the newest PyPI version.

### GRE ONNX to TensorRT

```bash
cd ~/DracoViLoc
/usr/src/tensorrt/bin/trtexec \
  --onnx=$PWD/models/gre/model_logmel.onnx \
  --saveEngine=$PWD/models/gre/model_logmel.engine \
  --fp16

cp models/gre/model_logmel.engine \
  src/dracoviloc_audio_fusion/models/model_logmel.engine

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dracoviloc_audio_fusion
```

Current GRE settings are `threshold_on=0.1`, `threshold_off=0.05`, and
`min_presence_s=0.1`. GRE prints confidence approximately once per second:

```text
GRE confidence=0.734 decision=DRONE track=2 ch=0
```

## AST model

### Artifacts and runtime path

```text
models/ast/config.json
models/ast/model.safetensors
models/ast/preprocessor_config.json
models/ast/training_report.json
models/ast/drone_ast.onnx
```

The generated runtime engine belongs at:

```text
src/dracoviloc_audio_fusion/models/drone_ast.engine
```

### AST runtime environment

```bash
cd ~/DracoViLoc
python3 -m venv --system-site-packages trt_env
source trt_env/bin/activate

python -m pip install --upgrade pip
python -m pip install soundfile sounddevice 'transformers==4.46.3'

grep include-system-site-packages trt_env/pyvenv.cfg
python -c "import pycuda.driver, tensorrt, transformers, sounddevice; print('AST runtime OK')"
```

### AST weights to ONNX

Export requires a Jetson-compatible PyTorch environment. The environment used
successfully on this card is:

```text
~/workspaces/isaac_ros-dev/models/yolo/bin/python3
```

Its validated compatibility set is NumPy 1.24.4, Pillow 10.4.0, ONNX 1.16.1,
ml-dtypes 0.3.2, and the existing Jetson-compatible PyTorch build. These avoid
the `numpy.Inf`, `PIL.Image.Resampling`, and `numpy.exceptions` errors caused
by incompatible package combinations.

```bash
cd ~/DracoViLoc
ISAAC_PY=~/workspaces/isaac_ros-dev/models/yolo/bin/python3

$ISAAC_PY scripts/export_ast_onnx.py \
  --model-dir "$PWD/models/ast" \
  --output "$PWD/models/ast/drone_ast.onnx"
```

Expected bindings:

```text
Input:  input_values float32 [1, 128, 128]
Output: logits float32 [1, 2]
```

The exporter intentionally sets `dynamo=False`; its legacy-export deprecation
warning is harmless for this fixed model.

### AST ONNX to TensorRT

Stop YOLO, RViz, and other GPU-heavy processes before building this
large engine:

```bash
cd ~/DracoViLoc
/usr/src/tensorrt/bin/trtexec \
  --onnx=$PWD/models/ast/drone_ast.onnx \
  --saveEngine=$PWD/src/dracoviloc_audio_fusion/models/drone_ast.engine \
  --fp16

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select dracoviloc_audio_fusion
```

TensorRT can remain silent after parsing while profiling tactics. Check
`tegrastats`; changing CPU/GPU activity means it is still working.

AST currently uses a `0.20` confidence threshold and `0.10` ODAS activity
floor. It prints confidence for every classified window:

```text
AST confidence | [CH 0 (ID 2)] 82.4% [DRONE] | ...
```

## YOLO and Isaac ROS

The full workflow is in [`isaac_ros/README.md`](isaac_ros/README.md). The key
rules are:

- Start YOLO inside the Isaac ROS container.
- Start DracoViLoc and RViz on the host.
- Do not source the Isaac install in the host DracoViLoc terminal.
- Do not include the container launch from the host launch file.

The current checkpoint is:

```text
models/yolo/drone_yolo11n_20260825_best.pt
```

Copy it to the Isaac workspace, export a same-version ONNX, and generate a
matching `.plan` inside the container. Runtime inference uses eight NITROS
blocks to reduce memory usage on the 8 GB Jetson.

## Run GRE + AST + YOLO + ODAS + RViz

This is the current end-to-end workflow. EKF is disabled.

### Terminal 1: Isaac ROS container

On the host, enter the persistent container:

```bash
cd ~/workspaces/isaac_ros-dev
./src/isaac_ros_common/scripts/run_dev.sh -b
```

Inside the container:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py \
  camera:=/dev/video0 \
  model_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_20260825_best.onnx \
  engine_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_20260825_best.plan \
  width:=640 \
  height:=480 \
  camera_fps:=30 \
  publish_rate:=15.0 \
  direction_frame:=table_mic_link \
  use_viewer:=true \
  record:=true \
  recording_root:=/home/dhianeifar/DracoViLoc/runs \
  recording_fps:=15.0
```

The YOLO launch starts its C++ recorder when `record:=true`. It records the
annotated image and captures the UMA16 directly through ALSA into one session
directory. Video is hardware-encoded as H.264 in an MP4 container.

The direction publisher uses only the highest-confidence box. It publishes
one `(x,y,1)` direction on `/yolo/directions` and a fixed-length blue marker
on `/yolo/target_marker`. This is a bearing, not a 3-D position; no depth is
estimated. The mounted-camera convention currently inverts image X and Y.

### Terminal 2: DracoViLoc host

```bash
cd ~/DracoViLoc
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true \
  ast_enabled:=true \
  gre_enabled:=true \
  yolo_enabled:=false \
  fusion_enabled:=false \
  tracking_mode:=direct_yolo \
  always_classify:=false \
  use_rviz:=true
```

Here, `fusion_enabled:=false` keeps fusion off. YOLO is already publishing from
the container.

The result includes live ODAS points, AST and GRE confidence, YOLO detections
and marker, RViz, and simulated arm tracking of YOLO. Use
`tracking_mode:=off` for visualization without arm movement.

With `record:=true` in the YOLO launch, the C++ recorder creates:

```text
~/DracoViLoc/runs/DD_MM_YYYY_HH_MM_SS/
├── audio.wav    # original 16-channel UMA16 stream, 44.1 kHz, signed 32-bit PCM
└── video.mp4    # annotated YOLO frames
```

The final timestamp field is seconds. Stop the YOLO launch cleanly with Ctrl+C
so the WAV sizes and video trailer are finalized. Optional YOLO recording
arguments are `recording_root`, `recording_audio_device`, `recording_fps`, and
`recording_bitrate`.
Recording output under `runs/` is ignored by Git.

## Tracking modes

- `off`: visualization only; no arm tracking.
- `direct_gre`: GRE gates the strongest active ODAS direction; no EKF.
- `direct_ast`: AST gates the strongest active ODAS direction; no EKF.
- `direct_either`: AST or GRE gates the strongest ODAS direction; no EKF.
- `direct_yolo`: follow the highest-confidence YOLO box ray; no EKF.
- `ekf`: follow `/ekf_fused_target_pose`.
  validated workflow.

## Useful checks

```bash
ros2 topic hz /sss
ros2 topic hz /sst
ros2 topic hz /detections_output
ros2 topic echo /yolo/directions --once
ros2 topic echo /yolo/target_marker --once
ros2 control list_controllers
```

Expected simulation controllers:

```text
joint_state_broadcaster  ...  active
arm_controller           ...  active
```

The physical-arm bringup activates them through `ros2_control`.

## Main topics

- `/sss`: separated multichannel ODAS audio.
- `/sst`: ODAS tracked acoustic directions.
- `/ssl_pcl2`: ODAS candidate point cloud.
- `/ast/direction`: AST-classified acoustic direction.
- `/gre/direction`: GRE-classified acoustic direction.
- `/detections_output`: YOLO boxes and confidence.
- `/yolo/direction`: highest-confidence YOLO ray.
- `/yolo/target_marker`: RViz YOLO arrow.
- `/ekf_fused_target_pose`: EKF-filtered direction.
- `/joint_states`: FAIRINO joint feedback.

## Troubleshooting

### `PortAudio library not found`

```bash
sudo apt install -y libportaudio2 portaudio19-dev
```

### Isaac packages appear in the host terminal

Open a new terminal and source only:

```bash
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash
```

### `No space left on device`

Do not install generic PyPI `torch`, `torchvision`, or NVIDIA CUDA wheels in
the container. They duplicate JetPack's CUDA stack.

### RViz drops YOLO markers

The physical camera and host ROS nodes use wall time. The marker uses
a zero timestamp so RViz selects the latest transform. Use
`direction_frame:=table_mic_link` for the current hardware-in-the-loop test.

### Arm swings with a stationary physical camera

Do not stamp that measurement as moving `odas_link`. Use `table_mic_link`.
Use `odas_link` only when physical joint feedback describes the same arm that
actually carries and moves the camera.
