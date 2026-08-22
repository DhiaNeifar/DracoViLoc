# dracoviloc_bringup

Top-level launch and controller configuration for DracoViLoc.

```bash
source /opt/ros/humble/setup.bash
source ~/DracoViLoc/install/setup.bash

# FAIRINO simulation with MoveIt and RViz
ros2 launch dracoviloc_bringup demo.launch.py \
  sim:=true use_rviz:=true use_moveit:=true

# Hardware-in-the-loop audio pointing
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=true
```

`arm_audio_demo.launch.py` uses a physical table-mounted UMA16v2 with a
simulated arm. Its microphone transform is controlled by the `table_mic_*`
arguments. Set `audio_tracking_enabled:=false` to inspect ODAS without motion.

For a physical controller, use `demo.launch.py sim:=false
robot_ip:=192.168.58.2`. Read `../fairino_hardware/README.md` before commanding
hardware.
