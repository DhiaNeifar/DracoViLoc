# dracoviloc_drone

Legacy simulated-drone model and teleoperation utilities. The current
audio-pointing demonstration intentionally does not spawn a drone; a phone or
other acoustic source supplies the test signal.

When a launch starts the drone node, keyboard control is available with:

```bash
ros2 run dracoviloc_drone drone_keyboard_teleop
```

Controls are `w/s`, `a/d`, `r/f`, `q/e`; Space stops and `x` resets. Do not
start a second Gazebo instance alongside `dracoviloc_bringup`.
