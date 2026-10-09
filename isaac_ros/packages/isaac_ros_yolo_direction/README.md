# isaac_ros_yolo_direction

Runs the complete DracoViLoc YOLO test or camera pipeline and projects each
detection-box center into a normalized camera ray. It publishes a
`geometry_msgs/Vector3Stamped` on `/yolo/direction`.

After deployment, container build and TensorRT engine generation:

```bash
# Bundled test video
ros2 launch isaac_ros_yolo_direction yolo_video.launch.py

# Live acoustic camera
ros2 launch isaac_ros_yolo_direction yolo_camera.launch.py camera:=auto
```

`camera:=auto` is the default. It finds the primary (interface index 0)
icSpring device belonging to the microphone array, so USB enumeration changes
do not silently select the laptop camera. Use `camera:=/dev/videoX` only as an
explicit override.

Direction calibration parameters are `x_sign`, `y_sign`, `swap_xy` and
`frame_id`. The default frame is `uma16_camera_direction`. See
[`../../README.md`](../../README.md) for the full target-card workflow.
