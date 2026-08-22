# isaac_ros_yolo_bringup

DracoViLoc's Isaac ROS preprocessing, TensorRT, YOLOv8-compatible decoder and
visualization wrapper. The YOLO11 detection export uses the compatible raw
tensor layout `[1, 5, 8400]` for the single `drone` class.

This package is deployed and built through the instructions in
[`../../README.md`](../../README.md). Normally it is included by
`isaac_ros_yolo_direction` rather than launched separately.

Direct invocation:

```bash
ros2 launch isaac_ros_yolo_bringup yolo_video_inference.launch.py \
  model_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_best.onnx \
  engine_path:=/workspaces/isaac_ros-dev/models/drone_yolo11n_best.plan \
  num_classes:=1 input_width:=848 input_height:=480 num_blocks:=8
```

Inputs are `/image` and `/camera_info`. Detections are published on
`/detections_output`; the processed image is `/yolov8_processed_image`.
