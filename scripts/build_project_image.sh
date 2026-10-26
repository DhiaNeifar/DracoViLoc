#!/usr/bin/env bash
# Build the DracoViLoc project image (isaac_ros/docker) on top of the Isaac ROS
# dev image installed for this CPU. Tags: dracoviloc/isaac_ros:<arch>-<hash of
# the build context> and :<arch>-latest.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLATFORM="${PROJECT_ROOT}/gui/dracoviloc_gui/host_platform.py"
mapfile -t image < <(python3 "${PLATFORM}" project-image "${PROJECT_ROOT}")
versioned="${image[0]}" latest="${image[1]}" version="${image[2]}" base="${image[3]}"
if [[ "${base}" == "-" ]]; then
  echo "No Isaac ROS base image for $(uname -m) is installed on this machine." >&2
  exit 1
fi
echo "Base image: ${base}"
echo "Building:   ${versioned} (+ ${latest})"
exec docker build --progress=plain \
  --build-arg "BASE_IMAGE=${base}" \
  --build-arg "DRACOVILOC_IMAGE_VERSION=${version}" \
  -t "${versioned}" -t "${latest}" \
  "${PROJECT_ROOT}/isaac_ros/docker"
