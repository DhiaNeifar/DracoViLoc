# DracoViLoc

DracoViLoc is a self-contained ROS 2 Humble project for a FAIRINO arm with
UMA16v2/ODAS acoustic localization and Isaac ROS YOLO drone detection. It
supports a simulated arm with a physical table-mounted microphone, physical-arm
operation, prerecorded YOLO regression testing, and live acoustic-camera
inference.

## Repository layout

```text
src/                         Host ROS packages: arm, ODAS and tracking
isaac_ros/packages/          DracoViLoc packages built in the Isaac container
isaac_ros/models/            Portable YOLO checkpoint and ONNX model
isaac_ros/media/             Bundled regression-test video
isaac_ros/tools/             Model and test utilities
scripts/                     Native and Isaac ROS deployment/build helpers
docs/                        Installation documentation
```

The repository includes ODAS and its message packages directly. It does not
depend on a separate `odas_ws`. NVIDIA Isaac ROS is treated as a target
platform: DracoViLoc owns its YOLO overlay, while NVIDIA's packages remain in
the already-installed Isaac ROS workspace.

## Host build

```bash
cd ~/DracoViLoc
./scripts/build_fairino_sdk.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

ROS/Gazebo installation details are in
[`docs/INSTALL_ROS2_GAZEBO.md`](docs/INSTALL_ROS2_GAZEBO.md).

## Arm and ODAS audio pointing

```bash
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash

ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true \
  audio_tracking_enabled:=true
```

The simulation directly and smoothly moves only joints 1 and 4 toward the
filtered audio direction. Joints 2, 3, 5 and 6 remain fixed. Full audio setup,
frame calibration and troubleshooting are documented in
[`src/dracoviloc_odas/README.md`](src/dracoviloc_odas/README.md).

FAIRINO simulation, SDK preparation, physical-arm launch and safety checks are
documented in
[`src/fairino_hardware/README.md`](src/fairino_hardware/README.md).

## Isaac ROS YOLO deployment

On a target card where Isaac ROS 3.2 is already installed:

```bash
git clone <DracoViLoc repository URL> ~/DracoViLoc
cd ~/DracoViLoc
./scripts/deploy_isaac_ros.sh ~/workspaces/isaac_ros-dev

cd ~/workspaces/isaac_ros-dev
./src/isaac_ros_common/scripts/run_dev.sh
```

Inside the Isaac ROS container:

```bash
/workspaces/isaac_ros-dev/scripts/build_dracoviloc_yolo.sh
source /workspaces/isaac_ros-dev/install/setup.bash
/workspaces/isaac_ros-dev/scripts/generate_tensorrt_engine.sh
```

After generating the engine, test the bundled video:

```bash
ros2 launch isaac_ros_yolo_direction yolo_video.launch.py
```

Or use the live acoustic camera:

```bash
ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py camera:=/dev/video0
```

Complete Isaac ROS setup, model export, topics, paths, and troubleshooting are
in [`isaac_ros/README.md`](isaac_ros/README.md).

## Main output topics

- `/audio/target_direction`: ODAS source direction used by the arm servo.
- `/audio/target_valid`: stable-audio-source status.
- `/detections_output`: Isaac ROS YOLO detections.
- `/yolo/directions`: camera-frame rays through YOLO detection centers.
- `/joint_states`: simulated or physical FAIRINO joint feedback.

The audio and visual pipelines remain separate sensors at this stage. Their
direction outputs provide a clean interface for later audio-visual fusion.
