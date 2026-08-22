#!/usr/bin/env bash
set -euo pipefail

isaac_ws="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
if [[ ! -d "${isaac_ws}/src/isaac_ros_common" ]]; then
  echo "Run this inside the Isaac ROS container." >&2
  echo "Expected workspace: ${isaac_ws}" >&2
  exit 1
fi

cd "${isaac_ws}"
source /opt/ros/humble/setup.bash

colcon build \
  --base-paths src \
  --packages-up-to \
    isaac_ros_yolo_bringup \
    isaac_ros_yolo_direction \
    yolo_video_publisher \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF

echo "Build complete. Run: source ${isaac_ws}/install/setup.bash"
