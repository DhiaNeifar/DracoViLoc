#!/usr/bin/env bash
# Installs the background Google Drive sync as a user systemd timer. Safe to
# re-run after editing the unit files or the sync script. Does NOT configure
# the rclone remote itself -- that is a one-time interactive step tied to
# your personal Google account; see docs/DRIVE_SYNC.md.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT_DIR="${HOME}/.config/systemd/user"

mkdir -p "${UNIT_DIR}"
cp "${SCRIPT_DIR}/systemd/sync-runs-to-drive.service" "${UNIT_DIR}/"
cp "${SCRIPT_DIR}/systemd/sync-runs-to-drive.timer" "${UNIT_DIR}/"

systemctl --user daemon-reload
systemctl --user enable --now sync-runs-to-drive.timer

if ! loginctl show-user "$(whoami)" -p Linger 2>/dev/null | grep -q "Linger=yes"; then
  echo "Warning: lingering is not enabled for $(whoami)." >&2
  echo "Without it the timer only runs while you are logged in. Enable with:" >&2
  echo "  loginctl enable-linger $(whoami)" >&2
fi

if ! command -v rclone >/dev/null 2>&1; then
  echo "Warning: rclone is not installed yet (sudo apt install rclone)." >&2
  echo "The timer is active but every run will fail until it is." >&2
elif ! rclone listremotes 2>/dev/null | grep -q ":"; then
  echo "Warning: no rclone remote is configured yet." >&2
  echo "See docs/DRIVE_SYNC.md for the one-time Google Drive authorization." >&2
fi

echo "Installed. Check status with: systemctl --user status sync-runs-to-drive.timer"
echo "Check logs with: tail -f ${HOME}/DracoViLoc/runs/.drive_sync.log"
