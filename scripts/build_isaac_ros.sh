#!/usr/bin/env bash
set -euo pipefail

isaac_ws="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
if [[ ! -d "${isaac_ws}/src/isaac_ros_common" ]]; then
  echo "Run this inside the Isaac ROS container." >&2
  echo "Expected workspace: ${isaac_ws}" >&2
  exit 1
fi

cd "${isaac_ws}"
# The bind-mounted host checkout is owned by the host user, while this build
# runs as the container user. Isaac ROS package introspection invokes Git.
git config --global --add safe.directory "${isaac_ws}"
# ROS Humble's setup script reads optional environment variables that may be unset.
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build \
  --base-paths src \
  --packages-up-to \
    isaac_ros_yolo_bringup \
    isaac_ros_yolo_direction \
    yolo_video_publisher \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF

echo "Build complete. Run: source ${isaac_ws}/install/setup.bash"
