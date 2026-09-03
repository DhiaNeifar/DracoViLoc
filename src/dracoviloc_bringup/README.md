# dracoviloc_bringup

Top-level launch and controller configuration for DracoViLoc.

```bash
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash

# Mock FAIRINO with MoveIt and RViz
ros2 launch dracoviloc_bringup demo.launch.py \
  hardware_mode:=mock use_rviz:=true use_moveit:=true

# Hardware-in-the-loop audio pointing
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=true
```

`arm_audio_demo.launch.py` defaults to mock joints. Select the physical arm
with `hardware_mode:=real robot_ip:=192.168.58.2`.

Read `../fairino_hardware/README.md` before commanding hardware.
