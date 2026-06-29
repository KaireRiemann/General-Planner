#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Build and smoke-test the ROS1 general_planner package.

Default usage from the host:
  sh_files/test_general_planner_ros1.sh

The script defaults to running inside the running ros1_noetic Docker container.
To run directly in the current shell:
  GP_USE_DOCKER=0 sh_files/test_general_planner_ros1.sh

Environment overrides:
  GP_DOCKER_CONTAINER   Docker container name. Default: ros1_noetic
  GP_CONTAINER_REPO     Repo path inside Docker.
                        Default: /root/ws/real_planner/src/General-Planner
  GP_SKIP_BUILD=1       Skip catkin_make and only run smoke tests.
  GP_CATKIN_ARGS        Extra args appended to catkin_make.
  GP_SMOKE_TIMEOUT      Per-node timeout in seconds. Default: 8
  GP_SMOKE_BASE_PORT    First ROS master port. Default: 18520
  GP_SMOKE_CONFIGS      Space-separated config list. Default covers
                        state2state corridor/esdf/plain, tracking, and
                        tracking-perching.
EOF
}

for arg in "$@"; do
  case "${arg}" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[test_general_planner] Unknown argument: ${arg}" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DOCKER_CONTAINER="${GP_DOCKER_CONTAINER:-ros1_noetic}"
CONTAINER_REPO="${GP_CONTAINER_REPO:-/root/ws/real_planner/src/General-Planner}"

inside_docker=0
if [[ -f /.dockerenv || -n "${GP_INSIDE_DOCKER:-}" ]]; then
  inside_docker=1
fi

if [[ "${GP_USE_DOCKER:-1}" != "0" && "${inside_docker}" == "0" ]]; then
  if ! command -v docker >/dev/null 2>&1; then
    echo "[test_general_planner] docker not found. Set GP_USE_DOCKER=0 to test locally." >&2
    exit 1
  fi
  if ! docker inspect -f '{{.State.Running}}' "${DOCKER_CONTAINER}" 2>/dev/null | grep -q true; then
    echo "[test_general_planner] Docker container '${DOCKER_CONTAINER}' is not running." >&2
    exit 1
  fi
  echo "[test_general_planner] Enter Docker container: ${DOCKER_CONTAINER}"
  docker exec \
    -e GP_USE_DOCKER=0 \
    -e GP_INSIDE_DOCKER=1 \
    -e GP_SKIP_BUILD="${GP_SKIP_BUILD:-0}" \
    -e GP_CATKIN_ARGS="${GP_CATKIN_ARGS:-}" \
    -e GP_SMOKE_TIMEOUT="${GP_SMOKE_TIMEOUT:-8}" \
    -e GP_SMOKE_BASE_PORT="${GP_SMOKE_BASE_PORT:-18520}" \
    -e GP_SMOKE_CONFIGS="${GP_SMOKE_CONFIGS:-}" \
    "${DOCKER_CONTAINER}" \
    bash -lc "cd $(printf '%q' "${CONTAINER_REPO}") && bash sh_files/test_general_planner_ros1.sh"
  exit $?
fi

WORKSPACE_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"
SMOKE_TIMEOUT="${GP_SMOKE_TIMEOUT:-8}"
BASE_PORT="${GP_SMOKE_BASE_PORT:-18520}"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[test_general_planner] /opt/ros/noetic/setup.bash not found. ROS Noetic is required." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
set -u

cd "${WORKSPACE_ROOT}"
if [[ "${GP_SKIP_BUILD:-0}" != "1" ]]; then
  catkin_cmd=(catkin_make --pkg general_planner)
  if [[ -n "${GP_CATKIN_ARGS:-}" ]]; then
    # shellcheck disable=SC2206
    extra_args=(${GP_CATKIN_ARGS})
    catkin_cmd+=("${extra_args[@]}")
  fi
  echo "[test_general_planner] Build command: ${catkin_cmd[*]}"
  "${catkin_cmd[@]}"
fi

set +u
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -u

if [[ -n "${GP_SMOKE_CONFIGS:-}" ]]; then
  # shellcheck disable=SC2206
  configs=(${GP_SMOKE_CONFIGS})
else
  configs=(
    click_smooth_ros1.yaml
    click_esdf_ros1.yaml
    click_plain_ros1.yaml
    tracking_tracker_drone1_ros1.yaml
    tracking_perching_chain_ros1.yaml
  )
fi

run_smoke() {
  local config="$1"
  local port="$2"
  local log="/tmp/general_planner_smoke_${config}.log"
  local roscore_log="/tmp/general_planner_smoke_roscore_${port}.log"

  export ROS_MASTER_URI="http://127.0.0.1:${port}"
  roscore -p "${port}" >"${roscore_log}" 2>&1 &
  local roscore_pid=$!
  sleep 2

  set +e
  timeout "${SMOKE_TIMEOUT}s" rosrun general_planner fsm_node _config_name:="${config}" >"${log}" 2>&1
  local status=$?
  kill "${roscore_pid}" >/dev/null 2>&1 || true
  wait "${roscore_pid}" >/dev/null 2>&1 || true
  set -e

  echo "[test_general_planner] ${config} status=${status}"
  grep -E "Active task executor|CLICKGOAL|TRACKING TASK|No odom|config|ERROR|FATAL" "${log}" | tail -10 || true
  if [[ "${status}" -eq 124 ]]; then
    return 0
  fi

  echo "[test_general_planner] Smoke failed for ${config}. Log tail:" >&2
  tail -60 "${log}" >&2 || true
  return "${status}"
}

index=0
for config in "${configs[@]}"; do
  run_smoke "${config}" "$((BASE_PORT + index))"
  index=$((index + 1))
done

echo "[test_general_planner] All smoke tests passed."
