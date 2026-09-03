# dracoviloc_bringup

Top-level launch and controller configuration for DracoViLoc.

```bash
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash

# Physical FAIRINO with MoveIt and RViz
ros2 launch dracoviloc_bringup demo.launch.py \
  robot_ip:=192.168.58.2 use_rviz:=true use_moveit:=true

# Hardware-in-the-loop audio pointing
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=true
```

`arm_audio_demo.launch.py` uses a physical table-mounted UMA16v2 and physical
FAIRINO arm. Its microphone transform is controlled by the `table_mic_*`
arguments.

Read `../fairino_hardware/README.md` before commanding hardware.
