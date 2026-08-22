#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
isaac_ws="${1:-${HOME}/workspaces/isaac_ros-dev}"

if [[ ! -d "${isaac_ws}/src/isaac_ros_common" ]]; then
  echo "Isaac ROS workspace not found at: ${isaac_ws}" >&2
  echo "Pass its host path as the first argument." >&2
  exit 1
fi

mkdir -p "${isaac_ws}/src" "${isaac_ws}/models" "${isaac_ws}/media" "${isaac_ws}/scripts"

for package in isaac_ros_yolo_bringup isaac_ros_yolo_direction yolo_video_publisher; do
  rsync -a --delete \
    "${repo_root}/isaac_ros/packages/${package}/" \
    "${isaac_ws}/src/${package}/"
done

rsync -a "${repo_root}/isaac_ros/tools/" "${isaac_ws}/scripts/"
rsync -a --exclude='*.plan' \
  "${repo_root}/isaac_ros/models/" "${isaac_ws}/models/"
rsync -a "${repo_root}/isaac_ros/media/" "${isaac_ws}/media/"
cp "${repo_root}/scripts/build_isaac_ros.sh" "${isaac_ws}/scripts/build_dracoviloc_yolo.sh"
cp "${repo_root}/scripts/generate_tensorrt_engine.sh" "${isaac_ws}/scripts/generate_tensorrt_engine.sh"
chmod +x "${isaac_ws}/scripts/build_dracoviloc_yolo.sh" \
  "${isaac_ws}/scripts/generate_tensorrt_engine.sh"

echo "Deployed DracoViLoc YOLO files to ${isaac_ws}"
echo "Next: ${isaac_ws}/src/isaac_ros_common/scripts/run_dev.sh"
echo "Inside the container: /workspaces/isaac_ros-dev/scripts/build_dracoviloc_yolo.sh"
