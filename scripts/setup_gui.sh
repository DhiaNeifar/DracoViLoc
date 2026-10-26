#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GUI_ENV="${PROJECT_ROOT}/gui_env"

python3 -m venv --system-site-packages "${GUI_ENV}"
"${GUI_ENV}/bin/python" -m pip install --upgrade pip
"${GUI_ENV}/bin/python" -m pip install -e "${PROJECT_ROOT}/gui[test]"
DESKTOP_FILE="${HOME}/.local/share/applications/dracoviloc-control-center.desktop"
mkdir -p "$(dirname "${DESKTOP_FILE}")"
sed "s|@PROJECT_ROOT@|${PROJECT_ROOT}|g" \
  "${PROJECT_ROOT}/gui/dracoviloc-control-center.desktop" > "${DESKTOP_FILE}"
chmod 644 "${DESKTOP_FILE}"

echo "GUI environment installed at ${GUI_ENV}"
echo "Desktop launcher installed for the current user."
echo "Start it with: ${PROJECT_ROOT}/scripts/run_gui.sh"

