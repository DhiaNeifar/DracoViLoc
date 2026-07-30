# Fairino Joystick Controller

This node allows you to control the Fairino robot using a Bluetooth joystick/gamepad.

## Features
- **Dual Control Modes**: Switch between Joint space and Cartesian space control
- **Dead-man Switch**: Safety feature requiring button hold to enable movement
- **Configurable Parameters**: Adjust movement speeds and button mappings
- **Real-time Control**: 20Hz update rate for smooth robot motion

## Quick Start

### 1. Make sure your joystick is connected
```bash
ls /dev/input/js*  # Should show js0 or similar
```

### 2. Test joystick input
```bash
ros2 run joy joy_node
ros2 topic echo /joy  # Move joystick to see values
```

### 3. Start the Fairino robot server (if not already running)
```bash
ros2 run fairino_hardware ros2_cmd_server --ros-args --params-file src/frcobot_ros2/fairino_hardware/fairino_remotecmdinterface_para.yaml
```

### 4. Launch the joystick controller
```bash
ros2 launch fairino_hardware joystick_control.launch.py
```

## Default Button/Axis Mapping

### Buttons:
- **Button 0**: Switch between JOINT and CARTESIAN control modes
- **Button 1**: Dead-man switch (HOLD to enable movement)

### Axes (Generic Joystick):
- **Axis 0** (Left stick horizontal): J1 / X-axis
- **Axis 1** (Left stick vertical): J2 / Y-axis
- **Axis 2** (Right stick horizontal): J6 / RZ-rotation
- **Axis 3** (Right stick vertical): J3 / Z-axis
- **Axis 4** (D-pad/Trigger): J4 / RX-rotation
- **Axis 5** (D-pad/Trigger): J5 / RY-rotation

## Control Modes

### JOINT Mode
Controls individual robot joints directly in degrees.
- Each axis controls one joint
- Default step: 2 degrees per update

### CARTESIAN Mode
Controls end-effector position and orientation.
- X, Y, Z: Position in mm
- RX, RY, RZ: Orientation in degrees
- Default position step: 5 mm
- Default rotation step: 2 degrees

## Configuration

### Changing Movement Speeds
```bash
ros2 launch fairino_hardware joystick_control.launch.py \
  joint_step:=5.0 \
  cart_step:=10.0 \
  cart_rot_step:=3.0
```

### Customizing Button/Axis Mappings

Edit the launch file or pass parameters:
```python
parameters=[{
    'button_mode_switch': 2,    # Change mode button
    'button_enable': 3,          # Change dead-man button
    'axis_j1_x': 0,             # Remap axes
    # ... etc
}]
```

### Finding Your Joystick Mappings

1. Run joy node and echo topic:
```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

2. Press buttons and move sticks to see which indices light up
3. Update the launch file parameters accordingly

## Safety Notes

⚠️ **IMPORTANT SAFETY CONSIDERATIONS**:
- Always hold the dead-man switch (Button 1) to enable movement
- Start with small step sizes and increase gradually
- Keep emergency stop accessible
- Test in a safe environment first
- Monitor robot joint limits

## Troubleshooting

### Joystick not detected
```bash
# Check if device exists
ls -l /dev/input/js0

# If using different device
ros2 launch fairino_hardware joystick_control.launch.py joy_dev:=/dev/input/js1
```

### Robot not responding
- Check that Fairino server is running
- Verify service is available: `ros2 service list | grep fairino`
- Check that you're holding the dead-man button
- Look at node output for error messages

### Wrong button/axis mappings
- Use `ros2 topic echo /joy` to identify correct indices
- Update launch file parameters to match your joystick

## Architecture

```
[Joystick] -> [joy_node] -> /joy topic -> [joystick_controller]
                                              |
                                              v
                                    /fairino_remote_command_service
                                              |
                                              v
                                         [Robot Server]
                                              |
                                              v
                                        [Fairino Robot]
```

## Topics

- **Subscribed**: `/joy` (sensor_msgs/Joy)

## Services

- **Used**: `/fairino_remote_command_service` (fairino_msgs/srv/RemoteCmdInterface)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joint_step` | double | 2.0 | Joint movement step (degrees) |
| `cart_step` | double | 5.0 | Cartesian position step (mm) |
| `cart_rot_step` | double | 2.0 | Cartesian rotation step (degrees) |
| `update_rate_hz` | double | 20.0 | Control loop frequency (Hz) |
| `deadzone` | double | 0.1 | Joystick deadzone threshold |
| `button_mode_switch` | int | 0 | Button for mode switching |
| `button_enable` | int | 1 | Dead-man switch button |
| `axis_j1_x` | int | 0 | J1/X axis mapping |
| `axis_j2_y` | int | 1 | J2/Y axis mapping |
| `axis_j3_z` | int | 3 | J3/Z axis mapping |
| `axis_j4_rx` | int | 4 | J4/RX axis mapping |
| `axis_j5_ry` | int | 5 | J5/RY axis mapping |
| `axis_j6_rz` | int | 2 | J6/RZ axis mapping |
