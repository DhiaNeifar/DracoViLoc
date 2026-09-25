#!/usr/bin/env bash

set -uo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_CONTAINER="${ISAAC_CONTAINER:-isaac_ros_dev-x86_64-container}"
GRACE_SECONDS="${GRACE_SECONDS:-8}"
DRY_RUN=false

if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=true
elif [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--dry-run]" >&2
  exit 2
fi

# Match only this checkout and the packages/nodes used by its DracoViLoc and
# Isaac ROS YOLO pipelines. Descendants of a matched launch process are also
# included, which catches RViz and controller processes without killing an
# unrelated RViz session.
PROJECT_PATTERN="${PROJECT_ROOT}|/workspaces/isaac_ros-dev|dracoviloc_|arm_audio_demo\.launch\.py|isaac_ros_yolo_direction|yolo_camera\.launch\.py|odas_ros|odas_(core|server|visualization)_node|arm_audio_tracker"
GENERIC_ROS_PATTERN="/opt/ros/.*/(robot_state_publisher|move_group|rviz2|static_transform_publisher|ros2_control_node|component_container[^[:space:]]*)"

declare -A parent_by_pid=()
declare -A command_by_pid=()
declare -A selected=()
declare -A excluded=()

while read -r pid ppid command; do
  [[ -n "${pid:-}" ]] || continue
  parent_by_pid["$pid"]="$ppid"
  command_by_pid["$pid"]="$command"
done < <(ps -eo pid=,ppid=,args=)

# Never select this script or any shell/session manager above it.
ancestor="$$"
while [[ -n "$ancestor" && "$ancestor" != "0" ]]; do
  excluded["$ancestor"]=1
  ancestor="${parent_by_pid[$ancestor]:-0}"
done

for pid in "${!command_by_pid[@]}"; do
  if [[ -z "${excluded[$pid]:-}" && "${command_by_pid[$pid]}" =~ $PROJECT_PATTERN ]]; then
    selected["$pid"]=1
  elif [[ -z "${excluded[$pid]:-}" && "${command_by_pid[$pid]}" =~ $GENERIC_ROS_PATTERN && -r "/proc/$pid/environ" ]]; then
    # A failed launch can exit before its children. Those children are then
    # adopted by systemd and lose the launch-parent relationship. Recognize
    # them by the DracoViLoc workspace inherited in their environment.
    process_environment="$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null || true)"
    if [[ "$process_environment" == *"COLCON_PREFIX_PATH=${PROJECT_ROOT}/install"* ]]; then
      selected["$pid"]=1
    fi
  fi
done

# Include children of selected launch processes, even when their executable
# name is generic (for example rviz2 or a component container).
changed=true
while $changed; do
  changed=false
  for pid in "${!parent_by_pid[@]}"; do
    ppid="${parent_by_pid[$pid]}"
    if [[ -n "${selected[$ppid]:-}" && -z "${selected[$pid]:-}" && -z "${excluded[$pid]:-}" ]]; then
      selected["$pid"]=1
      changed=true
    fi
  done
done

mapfile -t project_pids < <(printf '%s\n' "${!selected[@]}" | sed '/^$/d' | sort -n)

echo "DracoViLoc/Isaac ROS process scan:"
if [[ ${#project_pids[@]} -eq 0 ]]; then
  echo "  No matching host processes found."
else
  for pid in "${project_pids[@]}"; do
    printf '  PID %-7s %s\n' "$pid" "${command_by_pid[$pid]}"
  done
fi

docker_available=false
docker_unverified=false
docker_prefix=()
if command -v docker >/dev/null 2>&1; then
  if docker info >/dev/null 2>&1; then
    docker_available=true
    docker_prefix=(docker)
  elif command -v sudo >/dev/null 2>&1 && sudo -n docker info >/dev/null 2>&1; then
    docker_available=true
    docker_prefix=(sudo -n docker)
  else
    docker_unverified=true
  fi
fi

container_running=false
if $docker_available; then
  if "${docker_prefix[@]}" ps --format '{{.Names}}' | grep -Fxq "$ISAAC_CONTAINER"; then
    container_running=true
    echo "  Container: $ISAAC_CONTAINER (running)"
  else
    echo "  Container: $ISAAC_CONTAINER (not running)"
  fi
else
  echo "  Docker could not be inspected with the current permissions."
fi

if $DRY_RUN; then
  echo "Dry run: nothing was stopped."
  exit 0
fi

if [[ ${#project_pids[@]} -gt 0 ]]; then
  # Ask the tracker to release the arm before stopping its controller. Failure
  # is harmless when ROS is unavailable or the service is not running.
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
  # ROS environment hooks commonly inspect variables that are intentionally
  # unset. Temporarily disable nounset while sourcing them.
  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$PROJECT_ROOT/install/setup.bash"
  fi
  set -u
  if command -v ros2 >/dev/null 2>&1; then
    timeout 4 ros2 service call /demo/tracking std_srvs/srv/SetBool \
      '{data: false}' >/dev/null 2>&1 || true
  fi

  echo "Sending SIGINT to ${#project_pids[@]} host process(es)..."
  kill -INT "${project_pids[@]}" 2>/dev/null || true

  deadline=$((SECONDS + GRACE_SECONDS))
  while (( SECONDS < deadline )); do
    remaining=()
    for pid in "${project_pids[@]}"; do
      kill -0 "$pid" 2>/dev/null && remaining+=("$pid")
    done
    [[ ${#remaining[@]} -eq 0 ]] && break
    sleep 1
  done

  if [[ ${#remaining[@]} -gt 0 ]]; then
    echo "Sending SIGTERM to remaining process(es): ${remaining[*]}"
    kill -TERM "${remaining[@]}" 2>/dev/null || true
    sleep 2
  fi

  still_running=()
  for pid in "${remaining[@]:-}"; do
    [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && still_running+=("$pid")
  done
  if [[ ${#still_running[@]} -gt 0 ]]; then
    echo "Sending SIGKILL to unresponsive process(es): ${still_running[*]}"
    kill -KILL "${still_running[@]}" 2>/dev/null || true
  fi
fi

if $container_running; then
  echo "Stopping container $ISAAC_CONTAINER..."
  "${docker_prefix[@]}" stop --time 10 "$ISAAC_CONTAINER" >/dev/null
fi

failed=false
if $docker_unverified; then
  echo "  Docker remains unverified; run this script with sudo." >&2
  failed=true
fi
for pid in "${project_pids[@]}"; do
  if kill -0 "$pid" 2>/dev/null; then
    echo "  Still running: PID $pid (${command_by_pid[$pid]})" >&2
    failed=true
  fi
done
if $docker_available && "${docker_prefix[@]}" ps --format '{{.Names}}' | grep -Fxq "$ISAAC_CONTAINER"; then
  echo "  Still running: container $ISAAC_CONTAINER" >&2
  failed=true
fi

if $failed; then
  echo "Cleanup incomplete. Re-run with sufficient permissions." >&2
  exit 1
fi

echo "Cleanup complete: no identified DracoViLoc/Isaac ROS targets remain."
