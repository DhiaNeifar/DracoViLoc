#!/usr/bin/env bash
# Pushes finished recording runs (runs/video, runs/audio) to a personal
# Google Drive via rclone. Safe to run repeatedly and safe to run with no
# network: rclone only transfers files that differ from the remote, so a
# run that failed to upload (e.g. the machine was offline) is simply picked
# up again on the next invocation. Intended to be driven by
# sync-runs-to-drive.timer, not run by hand.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONFIG_FILE="${DRIVE_SYNC_CONFIG:-$HOME/.config/dracoviloc/drive_sync.env}"
if [[ -f "${CONFIG_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
fi

RUNS_ROOT="${DRIVE_SYNC_RUNS_ROOT:-${REPO_ROOT}/runs}"
REMOTE_NAME="${DRIVE_SYNC_REMOTE_NAME:-gdrive}"
REMOTE_PATH="${DRIVE_SYNC_REMOTE_PATH:-DracoViLoc/runs}"
LOG_FILE="${DRIVE_SYNC_LOG_FILE:-${RUNS_ROOT}/.drive_sync.log}"
LOCK_FILE="${DRIVE_SYNC_LOCK_FILE:-${RUNS_ROOT}/.drive_sync.lock}"
RCLONE_FLAGS=(--retries 5 --low-level-retries 10 --checkers 4 --transfers 2
  --contimeout 15s --timeout 60s)

mkdir -p "$(dirname "${LOG_FILE}")"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  echo "$(date -Is) skip: a sync is already running" >> "${LOG_FILE}"
  exit 0
fi

log() { echo "$(date -Is) $*" >> "${LOG_FILE}"; }

if ! command -v rclone >/dev/null 2>&1; then
  log "rclone is not installed; see docs/DRIVE_SYNC.md"
  exit 1
fi

if ! rclone listremotes 2>/dev/null | grep -qx "${REMOTE_NAME}:"; then
  log "remote '${REMOTE_NAME}:' is not configured; see docs/DRIVE_SYNC.md"
  exit 1
fi

is_complete() {
  local metadata="$1/metadata.json"
  [[ -f "${metadata}" ]] && grep -q '"state": "complete"' "${metadata}"
}

synced=0
failed=0
for modality in video audio; do
  modality_root="${RUNS_ROOT}/${modality}"
  [[ -d "${modality_root}" ]] || continue
  for run_dir in "${modality_root}"/*/; do
    [[ -d "${run_dir}" ]] || continue
    run_dir="${run_dir%/}"
    ts="$(basename "${run_dir}")"
    if ! is_complete "${run_dir}"; then
      continue
    fi
    if rclone copy "${run_dir}" "${REMOTE_NAME}:${REMOTE_PATH}/${modality}/${ts}" \
        "${RCLONE_FLAGS[@]}" >> "${LOG_FILE}" 2>&1; then
      synced=$((synced + 1))
    else
      failed=$((failed + 1))
      log "FAILED ${modality}/${ts} (will retry next run)"
    fi
  done
done

log "done: ${synced} run(s) synced, ${failed} failed"
[[ "${failed}" -eq 0 ]]
