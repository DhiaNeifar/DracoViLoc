#!/usr/bin/env bash
# Build the DracoViLoc ROS 2 workspace (src/) on the host. isaac_ros/ is built
# inside the Isaac container (setup_isaac_ros.sh); yolo-alt/ is not ours.
# Extra arguments go to colcon, e.g. --packages-select dracoviloc_bringup.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS Humble is not installed at /opt/ros/humble." >&2
  exit 1
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
cd "${PROJECT_ROOT}"
exec colcon build --symlink-install --base-paths src \
  --event-handlers console_cohesion+ summary+ "$@"
