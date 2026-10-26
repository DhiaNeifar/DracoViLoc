#!/usr/bin/env bash
# Set up the Isaac ROS workspace and build the DracoViLoc YOLO packages:
#   1. copy isaac_ros/ packages, models and tools into the workspace
#      (deploy_isaac_ros.sh);
#   2. colcon build them inside the project container (build_isaac_ros.sh).
# Workspace: $DRACOVILOC_ISAAC_ROOT, default ~/workspaces/isaac_ros-dev.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLATFORM="${PROJECT_ROOT}/gui/dracoviloc_gui/host_platform.py"
workspace="$(python3 "${PLATFORM}" isaac-root)"
image="$(python3 "${PLATFORM}" yolo-image)"

echo "== 1/2  Sync sources into ${workspace}"
"${PROJECT_ROOT}/scripts/deploy_isaac_ros.sh" "${workspace}"

echo "== 2/2  Build YOLO packages in ${image}"
if [[ -z "${image}" ]] || ! docker image inspect "${image}" >/dev/null 2>&1; then
  echo "No Isaac ROS image found; build the project image first (build_project_image.sh)." >&2
  exit 1
fi
# The Isaac entrypoint runs the command as the host user, so build files stay ours.
exec docker run --rm --network host --runtime nvidia --name dracoviloc_isaac_build \
  -e USERNAME=admin -e "HOST_USER_UID=$(id -u)" -e "HOST_USER_GID=$(id -g)" \
  -v "${workspace}:/workspaces/isaac_ros-dev" --workdir /workspaces/isaac_ros-dev \
  "${image}" /workspaces/isaac_ros-dev/scripts/build_dracoviloc_yolo.sh
