# DracoViLoc: exact startup instructions

This procedure starts the physical FAIRINO cobot, ODAS audio localization,
MobileNetV2, YOLO with its camera window, EKF fusion, and RViz.

The commands use ROS domain `42` everywhere. Do not omit it: the host and the
Isaac ROS container must use the same domain, and domain 42 prevents commands
from accidentally reaching a Jetson or an older ROS process on domain 0.

Before starting, clear the cobot workspace, release the emergency stop, and
keep the emergency stop within reach. Stop any old DracoViLoc and YOLO launches
with Ctrl+C.

## 1. Terminal 1: start YOLO and the camera window

Run on the host:

```bash
DISPLAY=:0 xhost +SI:localuser:root

cd ~/workspaces/isaac_ros-dev
./src/isaac_ros_common/scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev
```

After the prompt changes to the shell inside the container, run:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export CUDA_MODULE_LOADING=LAZY

ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py \
  camera:=/dev/video4 \
  model_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx \
  engine_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan \
  width:=640 \
  height:=480 \
  camera_fps:=30 \
  publish_rate:=15.0 \
  direction_frame:=table_mic_link \
  use_viewer:=true
```

Leave this terminal running. The annotated camera window should appear even
when no drone is detected.

## 2. Terminal 2: start DracoViLoc, the physical cobot, and RViz

Open a new host terminal and run:

```bash
cd ~/DracoViLoc
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash

export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export CUDA_MODULE_LOADING=LAZY

ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  hardware_mode:=real \
  robot_ip:=192.168.58.2 \
  audio_enabled:=true \
  mobilenetv2_enabled:=true \
  mobilenetv2_ekf_enabled:=true \
  mobilenetv2_engine_path:=$HOME/DracoViLoc/models/mobilenetv2/drone_fp32_t2000.engine \
  yolo_enabled:=true \
  tracking_mode:=ekf \
  fusion_enabled:=true \
  use_rviz:=true
```

Leave this terminal running. `tracking_mode:=ekf` selects the tracking input;
it does not immediately enable cobot tracking.

## 3. Terminal 3: verify the workflow

Open another host terminal and prepare it:

```bash
cd ~/DracoViLoc
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash

export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

Check that YOLO and the cobot services are on this ROS graph:

```bash
ros2 topic info /yolo/direction
ros2 service type /demo/home
ros2 service type /demo/tracking
ros2 control list_controllers -c /controller_manager
```

The expected controller state before homing is:

```text
arm_controller            active
arm_tracking_controller   inactive
```

For `/yolo/direction`, `Publisher count` must be at least `1`. A direction
message is published only while YOLO detects a target:

```bash
ros2 topic echo /yolo/direction --once
```

## 4. Move the cobot home

Make sure the workspace is clear, then run in Terminal 3:

```bash
ros2 service call /demo/home std_srvs/srv/Trigger "{}"
```

The response must contain `success=True`. Wait for the 12-second trajectory:

```bash
sleep 13
ros2 topic echo /joint_states --once
```

The home joint values are:

```text
joint1:  1.5708
joint2: -1.5708
joint3: -1.5708
joint4:  0.0000
joint5:  1.5708
joint6:  0.0000
```

The `/joint_states` message may list the joint names in a different order, so
match each position with its corresponding entry in `name`.

## 5. Enable tracking

After homing has finished, run:

```bash
ros2 service call /demo/tracking std_srvs/srv/SetBool "{data: true}"
```

A successful response says that tracking is armed and waiting for a fresh
direction. The cobot intentionally remains still until EKF receives a current
YOLO or MobileNetV2 target. Check the fused direction with:

```bash
ros2 topic echo /ekf/direction --once
```

If there is no message, place a detectable target in the YOLO camera view or
play a drone-like sound near the microphone array. Also verify:

```bash
ros2 topic info /detections_output
ros2 topic info /yolo/direction
ros2 topic info /mobilenetv2/direction
```

## 6. Stop safely

Disable tracking before shutting down:

```bash
ros2 service call /demo/tracking std_srvs/srv/SetBool "{data: false}"
sleep 3
ros2 control list_controllers -c /controller_manager
```

Wait until `arm_controller` is active and `arm_tracking_controller` is
inactive. Then press Ctrl+C in Terminal 2 and Terminal 1. Finally revoke the
temporary camera-window permission on the host:

```bash
DISPLAY=:0 xhost -SI:localuser:root
```

## Important troubleshooting

- `tracking_mode:=on` is invalid. Use `tracking_mode:=ekf` for this workflow.
- If a service waits forever, confirm that Terminal 1, Terminal 2, and Terminal
  3 all have `ROS_DOMAIN_ID=42`.
- `Failed init_port ... open_and_lock_file failed` is a Fast DDS shared-memory
  lock problem. Keep `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` exported in every
  terminal instead of deleting shared-memory files while ROS is running.
- If `/yolo/direction` shows zero publishers, restart the YOLO launch after
  exporting domain 42 inside the container.
- A successful tracking service response does not guarantee movement. The
  tracker waits for a fresh `/ekf/direction` message and holds position when no
  current target exists.
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  this the command to avoid shaking ....
  
  
  
  
  ros2 launch dracoviloc_bringup arm_audio_demo.launch.py   hardware_mode:=real   robot_ip:=192.168.58.2   audio_enabled:=true   ast_enabled:=false   gre_enabled:=false   mobilenetv2_enabled:=true   mobilenetv2_ekf_enabled:=true   mobilenetv2_threshold:=0.75   mobilenetv2_votes_required:=3   mobilenetv2_vote_window:=5   yolo_enabled:=true   fusion_enabled:=true   tracking_mode:=ekf   ekf_process_noise:=0.1  ekf_measurement_noise:=0.2   ekf_innovation_gate:=5.99   ekf_average_window:=15   command_rate_hz:=100.0   max_velocity:=0.60   max_acceleration:=0.80   max_jerk:=1.0   use_rviz:=true
  
  
  
  
  
    ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
    hardware_mode:=real \
    robot_ip:=192.168.58.2 \
    audio_enabled:=true \
    ast_enabled:=false \
    gre_enabled:=false \
    mobilenetv2_enabled:=true \
    mobilenetv2_ekf_enabled:=true \
    mobilenetv2_engine_path:=$HOME/DracoViLoc/models/mobilenetv2/drone_fp32_t2000.engine \
    mobilenetv2_threshold:=0.75 \
    mobilenetv2_votes_required:=3 \
    mobilenetv2_vote_window:=5 \
    min_activity:=0.10 \
    always_classify:=false \
    yolo_enabled:=true \
    fusion_enabled:=true \
    tracking_mode:=ekf \
    ekf_average_window:=15 \
    command_rate_hz:=100.0 \
    max_velocity:=1 \
    max_acceleration:=1 \
    max_jerk:=1.0 \
    use_rviz:=true


  
  
   cd ~/DracoViLoc
  source /opt/ros/humble/setup.bash
  source ~/DracoViLoc/install/setup.bash

  export ROS_DOMAIN_ID=42
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export CUDA_MODULE_LOADING=LAZY

 ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
    hardware_mode:=real \
    robot_ip:=192.168.58.2 \
    audio_enabled:=true \
    ast_enabled:=false \
    gre_enabled:=false \
    min_activity:=0.03 \
    mobilenetv2_enabled:=true \
    mobilenetv2_ekf_enabled:=true \
    mobilenetv2_engine_path:=$HOME/DracoViLoc/models/mobilenetv2/drone_fp32_t2000.engine \
    mobilenetv2_threshold:=0.75 \
    mobilenetv2_votes_required:=3 \
    mobilenetv2_vote_window:=5 \
    yolo_enabled:=false \
    fusion_enabled:=true \
    tracking_mode:=ekf \
    ekf_process_noise:=0.03 \
    ekf_measurement_noise:=0.02 \
    ekf_yolo_measurement_noise:=0.03 \
    ekf_mobilenetv2_measurement_noise:=0.15 \
    ekf_innovation_gate:=5.99 \
    ekf_average_window:=10 \
    smoothing_alpha:=0.10 \
    angular_deadband:=0.08 \
    angular_deadband_exit:=0.05 \
    target_timeout:=1.0 \
    command_rate_hz:=100.0 \
    max_velocity:=5 \
    max_acceleration:=7 \
    max_jerk:=1 \
    require_home_before_tracking:=true \
    home_duration_s:=12.0 \
    use_rviz:=true
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py     camera:=/dev/video4     model_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_20260917_best.onnx     engine_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_20260917_best.plan     width:=1920     height:=1080     camera_fps:=30     publish_rate:=15.0     direction_frame:=table_mic_link     use_viewer:=true


