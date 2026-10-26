#!/usr/bin/env bash
# Download new YOLO checkpoints from the team's Google Drive folder and export
# them to ONNX. Options: --no-export, --folder <drive folder id>.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GUI_PYTHON="${PROJECT_ROOT}/gui_env/bin/python"

if [[ ! -x "${GUI_PYTHON}" ]]; then
  echo "The GUI environment is not installed. Run: ${PROJECT_ROOT}/scripts/setup_gui.sh" >&2
  exit 1
fi
export DRACOVILOC_ROOT="${PROJECT_ROOT}"
exec "${GUI_PYTHON}" -m dracoviloc_gui.fetch_models "$@"
