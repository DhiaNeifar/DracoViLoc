# DracoViLoc Isaac ROS YOLO overlay

This directory is the source of truth for DracoViLoc's YOLO pipeline. NVIDIA
Isaac ROS itself remains installed in its normal development workspace. The
deployment script copies only these project-owned packages and assets into that
workspace because they must be built and run inside the Isaac ROS container.

## Contents

- `packages/isaac_ros_yolo_bringup`: TensorRT preprocessing, decoding and visualization.
- `packages/isaac_ros_yolo_direction`: converts detection centers into camera-frame rays.
- `packages/yolo_video_publisher`: publishes the bundled MP4 or a V4L2 camera.
- `models/drone_yolo11n_best.pt`: training checkpoint.
- `models/drone_yolo11n_best.onnx`: portable inference model.
- `media/drone-video1.mp4`: reproducible video-inference test.
- `tools`: export, verification, resizing and result-recording utilities.

The TensorRT `.plan` is intentionally not stored. Generate it on the target
Jetson because it is tied to that GPU, JetPack, CUDA and TensorRT version.

## Deploy on the host

The target card must already have Isaac ROS 3.2 at
`$HOME/workspaces/isaac_ros-dev`.

```bash
cd ~/DracoViLoc
./scripts/deploy_isaac_ros.sh ~/workspaces/isaac_ros-dev

cd ~/workspaces/isaac_ros-dev
./src/isaac_ros_common/scripts/run_dev.sh
```

## Build inside the container

```bash
/workspaces/isaac_ros-dev/scripts/build_dracoviloc_yolo.sh
source /workspaces/isaac_ros-dev/install/setup.bash
```

If this is a new/recreated container, install the runtime dependencies first:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-magic-enum \
  ros-humble-foxglove-msgs \
  ros-humble-image-tools
```

## Generate the target-specific engine

```bash
/workspaces/isaac_ros-dev/scripts/generate_tensorrt_engine.sh
```

Once `models/drone_yolo11n_best.plan` exists and the TensorRT node is waiting
for input, stop it with `Ctrl+C`.

## Test with the bundled video

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch isaac_ros_yolo_direction yolo_video.launch.py
```

This launches inference, the native video publisher, direction projection,
visualization and the image viewer. Useful checks are:

```bash
ros2 topic hz /image
ros2 topic hz /detections_output
ros2 topic echo /yolo/directions --once
```

## Run the acoustic camera

The camera is expected to appear as `/dev/video0` by default:

```bash
ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py \
  camera:=/dev/video0 \
  width:=640 height:=480 \
  horizontal_fov_deg:=100.0
```

Override `model_path` and `engine_path` if necessary. The direction output is
`/yolo/directions` in frame `uma16_camera_direction`.

## Export or verify the model

Create the Ultralytics environment outside `src`, then run:

```bash
python3 /workspaces/isaac_ros-dev/scripts/export_yolo.py \
  --model /workspaces/isaac_ros-dev/models/drone_yolo11n_best.pt --imgsz 640

python3 /workspaces/isaac_ros-dev/scripts/verify_yolo_onnx.py \
  --model /workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx \
  --num-classes 1 --imgsz 640
```

The expected output tensor is `[1, 5, 8400]`: four box values plus the single
`drone` class score.
