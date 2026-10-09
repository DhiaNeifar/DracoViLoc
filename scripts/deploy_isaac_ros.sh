#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
dry_run=false
if [[ "${1:-}" == "--dry-run" ]]; then
  dry_run=true
  shift
fi
if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [--dry-run] [isaac-workspace]" >&2
  exit 2
fi
isaac_ws="${1:-${HOME}/workspaces/isaac_ros-dev}"

if [[ ! -d "${isaac_ws}/src/isaac_ros_common" ]]; then
  echo "Isaac ROS workspace not found at: ${isaac_ws}" >&2
  echo "Pass its host path as the first argument." >&2
  exit 1
fi

rsync_options=(-a)
if [[ "${dry_run}" == true ]]; then
  rsync_options+=(--dry-run --itemize-changes)
  echo "Previewing deployment from DracoViLoc to: ${isaac_ws}"
else
  mkdir -p "${isaac_ws}/src" "${isaac_ws}/models" "${isaac_ws}/media" "${isaac_ws}/scripts"
fi

for package in isaac_ros_yolo_bringup isaac_ros_yolo_direction yolo_video_publisher; do
  # Container-created Python caches may belong to a remapped UID. Ignore them
  # during host deployment instead of trying to delete them.
  rsync "${rsync_options[@]}" --delete \
    --exclude='__pycache__/' --exclude='*.pyc' --exclude='.pytest_cache/' \
    "${repo_root}/isaac_ros/packages/${package}/" \
    "${isaac_ws}/src/${package}/"
done

rsync "${rsync_options[@]}" "${repo_root}/isaac_ros/tools/" "${isaac_ws}/scripts/"
rsync "${rsync_options[@]}" --exclude='*.plan' \
  "${repo_root}/isaac_ros/models/" "${isaac_ws}/models/"
rsync "${rsync_options[@]}" "${repo_root}/isaac_ros/media/" "${isaac_ws}/media/"
rsync "${rsync_options[@]}" \
  "${repo_root}/scripts/build_isaac_ros.sh" \
  "${isaac_ws}/scripts/build_dracoviloc_yolo.sh"
rsync "${rsync_options[@]}" \
  "${repo_root}/scripts/generate_tensorrt_engine.sh" \
  "${isaac_ws}/scripts/generate_tensorrt_engine.sh"

if [[ "${dry_run}" == true ]]; then
  echo "Dry run complete; no files were changed."
  exit 0
fi

chmod +x "${isaac_ws}/scripts/build_dracoviloc_yolo.sh" \
  "${isaac_ws}/scripts/generate_tensorrt_engine.sh"

echo "Deployed DracoViLoc YOLO files to ${isaac_ws}"
echo "Next: ${isaac_ws}/src/isaac_ros_common/scripts/run_dev.sh"
echo "Inside the container: /workspaces/isaac_ros-dev/scripts/build_dracoviloc_yolo.sh"
