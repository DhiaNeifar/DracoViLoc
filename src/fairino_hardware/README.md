# FAIRINO arm hardware

The FAIRINO integration provides the `ros2_control` hardware plugin used by the
DracoViLoc robot description.

## Prepare the SDK on a new machine

The SDK shared library is architecture-specific and is intentionally generated
locally. Build and copy the correct native library before the ROS workspace:

```bash
cd ~/DracoViLoc
./scripts/build_fairino_sdk.sh

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For the current audio-pointing demonstration, MoveIt is deliberately disabled
because the controller directly servos only joints 1 and 4:

```bash
ros2 launch dracoviloc_bringup arm_audio_demo.launch.py \
  audio_enabled:=true audio_tracking_enabled:=true
```

## Physical arm

Connect the host to the robot controller network, verify that the controller is
reachable, and pass its IP address:

```bash
ping 192.168.58.2

ros2 launch dracoviloc_bringup demo.launch.py \
  robot_ip:=192.168.58.2 \
  use_rviz:=true use_moveit:=true
```

Start with low speed limits and a clear workspace. Confirm joint-state feedback
before sending motion. Bringup loads
`fairino_hardware/FairinoHardwareInterface`.

## Useful checks

```bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Expected controllers are `joint_state_broadcaster` and `arm_controller`.
Additional joystick notes are in `JOYSTICK_README.md`.
