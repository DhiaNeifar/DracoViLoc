#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GUI_PYTHON="${PROJECT_ROOT}/gui_env/bin/python"

if [[ ! -x "${GUI_PYTHON}" ]]; then
  echo "DracoViLoc GUI is not installed." >&2
  echo "Run: ${PROJECT_ROOT}/scripts/setup_gui.sh" >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS Humble is not installed at /opt/ros/humble." >&2
  exit 1
fi
if [[ ! -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  echo "The DracoViLoc workspace has not been built." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PROJECT_ROOT}/install/setup.bash"
set -u

export DRACOVILOC_ROOT="${PROJECT_ROOT}"
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export CUDA_MODULE_LOADING=LAZY
exec "${GUI_PYTHON}" -m dracoviloc_gui.main "$@"

