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

# Qt's desktop (xcb) plugin needs a few system libraries that are not part
# of the pip wheels; report any that are missing with the command to fix it.
XCB_PLUGIN="$(find "${GUI_ENV}" -name libqxcb.so -path '*platforms*' | head -1)"
if [[ -n "${XCB_PLUGIN}" ]]; then
  MISSING="$(ldd "${XCB_PLUGIN}" | awk '/not found/ {print $1}')"
  if [[ -n "${MISSING}" ]]; then
    PACKAGES="$(for lib in ${MISSING}; do dpkg -S "*/${lib}" 2>/dev/null | cut -d: -f1 | head -1 || true; done | sort -u)"
    [[ -n "${PACKAGES}" ]] || PACKAGES="$(echo "${MISSING}" | sed -E 's/^lib([a-z0-9-]+)\.so\.([0-9]+).*/lib\1\2/' | sort -u)"
    echo "WARNING: the GUI cannot open a window until these libraries are installed: ${MISSING}" >&2
    echo "         sudo apt install -y ${PACKAGES//$'\n'/ }" >&2
  fi
fi

echo "GUI environment installed at ${GUI_ENV}"
echo "Desktop launcher installed for the current user."
echo "Start it with: ${PROJECT_ROOT}/scripts/run_gui.sh"

