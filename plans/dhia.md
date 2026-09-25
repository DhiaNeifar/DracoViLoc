# DracoViLoc — YOLO + MobileNetV2 run instructions (x86 desktop, Quadro T2000)

Validated end-to-end on 2026-09-18 on this desktop (not the Jetson).
Prerequisites verified: UMA16 mic = ALSA card 2, project camera = `/dev/video4`
(icspring; `/dev/video0` is the laptop webcam), Quadro T2000 driver 595.84,
and system `/usr/bin/python3` has TensorRT 10.3 + PyCUDA + rclpy.

## Build and validate first

Run this after changing any package or pulling new work:

```bash
cd ~/DracoViLoc
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dracoviloc_bringup dracoviloc_mobilenetv2
source install/setup.bash
```

The desktop must use the T2000-specific engine. Validate it before starting
ROS; this command passed on 2026-09-18 with 64 clips and zero threshold
decision mismatches:

```bash
/usr/bin/python3 src/dracoviloc_mobilenetv2/mobilenetv2/validate_engine.py \
  --engine "$PWD/models/mobilenetv2/drone_fp32_t2000.engine" \
  --reference "$PWD/models/mobilenetv2/reference_outputs.npz"
```

For a safe mock-arm/DDS check (no physical arm, camera, or ODAS), run:

```bash
ROS_LOG_DIR=/tmp/dracoviloc-ros-logs \
  /usr/bin/python3 src/dracoviloc_mobilenetv2/test/runtime_smoke.py \
  --engine "$PWD/models/mobilenetv2/drone_fp32_t2000.engine"
```

The T2000 TensorRT engine for MobileNetV2 was built from
`models/mobilenetv2/mobilenetv2_drone.onnx` and validated against
`models/mobilenetv2/reference_outputs.npz` (max prob error 0.00025, 0 decision
mismatches over 64 clips). It is persisted at
`~/DracoViLoc/models/mobilenetv2/drone_fp32_t2000.engine` (machine-specific).
Rebuild it after GPU/TensorRT changes, then rerun the validation command above:

```bash
trtexec --onnx=models/mobilenetv2/mobilenetv2_drone.onnx \
  --saveEngine=models/mobilenetv2/drone_fp32_t2000.engine
```

`drone_fp32.engine` in the same directory is the Orin engine, do NOT use it
here.

## Golden rule: one instance of each

Never run two DracoViLoc bringups or two YOLO pipelines at once. The UMA16
soundcard is exclusive and both bringups spawn a `controller_manager` with
identical names, so the second launch gets: no ODAS (no sound points in RViz),
`spawner_arm_controller: Failed loading controller`, and a continuous
"Aborting, no controller is switched! (STRICT switch)" error loop in BOTH
terminals. Before launching, confirm nothing is still running:

```bash
ps aux | grep -E "odas_core|arm_audio_demo" | grep -v grep   # must print nothing
docker ps                                                    # no isaac containers
```

If YOLO starts but `/detections_output` stays silent (NITROS sometimes logs
`tensor_rt: Could not negotiate` and stalls), just Ctrl+C the YOLO launch and
re-run it — negotiation retries from scratch.

The YOLO plan `~/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan` was
rebuilt for the T2000 on 2026-09-18.

## Terminal 1 — Isaac ROS container (YOLO)

Before opening the container, run this once in a graphical host terminal. The
container's `showimage` process runs as root and needs local X11 permission to
open the camera window:

```bash
DISPLAY=:0 xhost +SI:localuser:root
```

### Quick start: tagged image, no run_dev.sh, no rebuild

The fully fixed image is tagged `isaac_ros_yolo:draco-t2000` (same content as
`isaac_ros_dev-x86_64:latest` with NumPy<2 + image-tools + v4l-utils baked in).
Verified 2026-09-18 — starts YOLO and opens the camera window in one step:

```bash
DISPLAY=:0 xhost +SI:localuser:root
docker run -d --privileged --network host --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/home/admin/.Xauthority:rw \
  -e DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_DOMAIN_ID=42 -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
  -e HOST_USER_UID=$(id -u) -e HOST_USER_GID=$(id -g) \
  -v $HOME/workspaces/isaac_ros-dev:/workspaces/isaac_ros-dev \
  -v /etc/localtime:/etc/localtime:ro \
  --name isaac_yolo --runtime nvidia \
  --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
  --workdir /workspaces/isaac_ros-dev \
  isaac_ros_yolo:draco-t2000 \
  /bin/bash -lc 'export CUDA_MODULE_LOADING=LAZY; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py camera:=/dev/video4 model_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx engine_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan width:=640 height:=480 camera_fps:=30 publish_rate:=15.0 direction_frame:=table_mic_link use_viewer:=true'
```

Stop with `docker rm -f isaac_yolo`. Re-tag after any rebuild:
`docker tag isaac_ros_dev-x86_64:latest isaac_ros_yolo:draco-t2000`.

### Standard flow (run_dev.sh)

> IMPORTANT: `run_dev.sh` does NOT auto-detect `~/workspaces/isaac_ros-dev` on
> this machine. Without an explicit path it falls back to a wrong directory and
> the container won't contain `install/setup.bash` or the YOLO packages
> ("package 'isaac_ros_yolo_direction' not found"). Always pass `-d`:

```bash
cd ~/workspaces/isaac_ros-dev
./src/isaac_ros_common/scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export CUDA_MODULE_LOADING=LAZY
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py \
  camera:=/dev/video4 \
  model_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx \
  engine_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan \
  width:=640 height:=480 camera_fps:=30 publish_rate:=15.0 \
  direction_frame:=table_mic_link \
  use_viewer:=true
```

`use_viewer:=true` opens an annotated camera window on the host display (via the
mounted X11 socket). It now displays live camera frames even when YOLO has not
found a drone yet. To revoke the temporary X11 permission after shutdown, run
`DISPLAY=:0 xhost -SI:localuser:root` on the host.

> Container image fixes are baked into the image build itself (added as a step in
> `~/workspaces/isaac_ros-dev/src/isaac_ros_common/docker/Dockerfile.ros2_humble`,
> before the "Store list of packages (must be last)" step, image rebuilt
> 2026-09-18): NumPy pinned to <2 (the image otherwise ends up with NumPy 2.x
> while its cv2 was built for 1.x, which crashes `yolo_visualizer.py` on
> `import cv2`), plus `ros-humble-image-tools` (the `showimage` viewer used with
> `use_viewer:=true` was missing → launch died with "package 'image_tools' not
> found") and `v4l-utils` (`v4l2-ctl`, used to configure the camera before
> capture). `run_dev.sh` rebuilds this layer automatically from the Dockerfile
> (cached, ~20 s), so the fixes survive image rebuilds. If the workspace is
> re-cloned/redeployed, re-apply that Dockerfile step; to rebuild the image
> manually (e.g. after editing the Dockerfile):

```bash
~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts/build_image_layers.sh \
  --image_key ros2_humble \
  --image_name isaac_ros_dev-x86_64 \
  --base_image x86_64-image:latest
```

If you ever swap YOLO models, regenerate the plan inside the container with:

```bash
~/workspaces/isaac_ros-dev/scripts/generate_tensorrt_engine.sh
```

## Terminal 2 — DracoViLoc host

```bash
cd ~/DracoViLoc
source /opt/ros/humble/setup.bash
source install/setup.bash
export CUDA_MODULE_LOADING=LAZY
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true \
  mobilenetv2_enabled:=true \
  mobilenetv2_engine_path:=$HOME/DracoViLoc/models/mobilenetv2/drone_fp32_t2000.engine \
  yolo_enabled:=true \
  tracking_mode:=direct_mobilenetv2 \
  fusion_enabled:=false \
  use_rviz:=true
```

The integrated launch scopes AST, GRE, and MobileNetV2 independently. This is
important when more than one classifier is enabled: their `engine_path` and
`venv_python` launch arguments no longer leak into one another. On this desktop
keep AST and GRE disabled unless their own engines and virtual environments are
intentionally being used.

`direct_mobilenetv2` creates the local `/demo/home` and `/demo/tracking`
services. Tracking starts disabled, so it is safe to home before explicitly
enabling tracking. `ROS_DOMAIN_ID=42` isolates this desktop workflow from a
Jetson or another ROS graph using the default domain. Use the same domain ID in
every terminal that must communicate.

## Verify (Terminal 2 or any host tab)

```bash
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 topic hz /sss                        # ODAS audio ~87 Hz
ros2 topic hz /sst                        # ODAS tracks ~91 Hz
ros2 topic echo /detections_output --once # YOLO boxes (from container)
ros2 topic echo /yolo/direction --once    # YOLO bearing (needs a visible drone)
ros2 topic echo /mobilenetv2/direction --once  # only after drone sound + 3/5 votes
```

MobileNetV2 publishes only after ODAS locks an active track AND 3-of-5 windows
classify as drone at threshold 0.75 — play a drone-like sound near the mic
array. Classifier status:

```bash
ros2 topic echo /rosout | grep -A1 mobilenetv2
```

## Shutdown

- Ctrl+C in each terminal (clean — avoids the stale SHM lock issue).
- If the container was started detached: `docker rm -f isaac_yolo`
  (or `docker stop isaac_ros_dev-x86_64-container` for the run_dev.sh one).

## Known issues (pre-existing, not fixed)

1. If `src/isaac_ros_common` is ever re-cloned or the workspace redeployed, the
   Dockerfile fixes are lost — re-apply the step described in the Terminal 1
   note (NumPy pin + image-tools + v4l-utils).
2. `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` avoids stale Fast DDS shared-memory lock
   errors such as `Failed init_port ... open_and_lock_file failed`. Export it
   before starting ROS in every terminal. If cleanup is still needed, first
   stop every ROS process and ROS container; only then remove stale locks:

```bash
sudo rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

## Arm homing

Requires `tracking_mode` ≠ `off` (e.g. `direct_yolo` or `direct_mobilenetv2`)
and the arm stack running (`hardware_mode:=mock`, or `hardware_mode:=real
robot_ip:=192.168.58.2` for the physical FAIRINO — make sure the workspace is
clear). Homing takes ~12 s (`home_duration_s`):

```bash
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 service call /demo/home std_srvs/srv/Trigger "{}"
```

Wait for homing to finish, then enable tracking:

```bash
ros2 service call /demo/tracking std_srvs/srv/SetBool "{data: true}"
```
