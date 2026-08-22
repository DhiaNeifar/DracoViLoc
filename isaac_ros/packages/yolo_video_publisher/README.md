# yolo_video_publisher

Native C++ ROS 2 publisher used for repeatable Isaac ROS inference tests and
V4L2 camera input. It publishes reliable `sensor_msgs/Image` and
`sensor_msgs/CameraInfo` messages on `/image` and `/camera_info`.

The complete test is normally launched with:

```bash
ros2 launch isaac_ros_yolo_direction yolo_video.launch.py
```

It can also be run directly:

```bash
ros2 run yolo_video_publisher video_publisher_node --ros-args \
  -p video_path:=/workspaces/isaac_ros-dev/media/drone-video1.mp4 \
  -p publish_rate:=30.0 -p loop:=true -p horizontal_fov_deg:=100.0
```

The native publisher avoids Python serialization becoming the bottleneck for
raw RGB frames and uses QoS compatible with the Isaac ROS resize subscriber.
