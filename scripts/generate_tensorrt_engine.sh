#!/usr/bin/env bash
set -euo pipefail

isaac_ws="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
model="${1:-${isaac_ws}/models/drone_yolo11n_best.onnx}"
engine="${2:-${isaac_ws}/models/drone_yolo11n_best.plan}"

cd "${isaac_ws}"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch isaac_ros_tensor_rt isaac_ros_tensor_rt.launch.py \
  model_file_path:="${model}" \
  engine_file_path:="${engine}" \
  input_tensor_names:='["input_tensor"]' \
  input_binding_names:='["images"]' \
  output_tensor_names:='["output_tensor"]' \
  output_binding_names:='["output0"]' \
  verbose:=True \
  force_engine_update:=True
