#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Run the final exploration/NHBP closure validation.

Default usage from the host:
  sh_files/test_exploration_final_closure.sh

Useful environment overrides:
  GP_DOCKER_CONTAINER              Docker container name. Default: ros1_noetic
  GP_CONTAINER_REPO                Repo path inside Docker.
  GP_FINAL_TRAP_LOG                Optional historical trap log to check.
  GP_FINAL_SKIP_BUILD=1            Skip catkin_make --pkg general_planner.
  GP_FINAL_SKIP_FULL_BUILD=1       Skip full workspace catkin_make.
  GP_FINAL_SKIP_EXPLORATION=1      Skip big-field exploration guard.
  GP_FINAL_RUN_GENERAL_SMOKE=1     Also run legacy general_planner ROS1 smoke tests.
  GP_EXPLORATION_GUARD_DURATION    Big-field guard duration. Default: 90.
EOF
}

for arg in "$@"; do
    case "${arg}" in
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[exploration_final] Unknown argument: ${arg}" >&2
            usage >&2
            exit 2
            ;;
    esac
done

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "${ROOT_DIR}/../.." && pwd)"
DOCKER_CONTAINER="${GP_DOCKER_CONTAINER:-ros1_noetic}"
CONTAINER_REPO="${GP_CONTAINER_REPO:-/root/ws/real_planner/src/General-Planner}"
FINAL_TRAP_LOG="${GP_FINAL_TRAP_LOG:-}"

inside_docker=0
if [[ -f /.dockerenv || -n "${GP_INSIDE_DOCKER:-}" ]]; then
    inside_docker=1
fi

run_static_checks() {
    cd "${ROOT_DIR}"
    git diff --check
    bash -n sh_files/test_exploration_big_field_guard.sh
    bash -n sh_files/analyze_exploration_trap_log.sh
    bash -n sh_files/test_exploration_trap_log_guard.sh
}

run_optional_historical_log_guard() {
    if [[ -z "${FINAL_TRAP_LOG}" ]]; then
        echo "[exploration_final] historical_trap_log=skipped"
        return 0
    fi
    GP_TRAP_LOG_EXPECT_SIGNATURE="${GP_TRAP_LOG_EXPECT_SIGNATURE:-1}" \
        sh_files/test_exploration_trap_log_guard.sh "${FINAL_TRAP_LOG}"
}

if [[ "${GP_USE_DOCKER:-1}" != "0" && "${inside_docker}" == "0" ]]; then
    run_static_checks
    run_optional_historical_log_guard
    if ! command -v docker >/dev/null 2>&1; then
        echo "[exploration_final] FAIL: docker not found. Set GP_USE_DOCKER=0 to run locally." >&2
        exit 1
    fi
    if ! docker inspect -f '{{.State.Running}}' "${DOCKER_CONTAINER}" 2>/dev/null | grep -q true; then
        echo "[exploration_final] FAIL: Docker container '${DOCKER_CONTAINER}' is not running." >&2
        exit 1
    fi
    echo "[exploration_final] Enter Docker container: ${DOCKER_CONTAINER}"
    docker exec \
        -e GP_USE_DOCKER=0 \
        -e GP_INSIDE_DOCKER=1 \
        -e GP_FINAL_SKIP_BUILD="${GP_FINAL_SKIP_BUILD:-0}" \
        -e GP_FINAL_SKIP_FULL_BUILD="${GP_FINAL_SKIP_FULL_BUILD:-0}" \
        -e GP_FINAL_SKIP_EXPLORATION="${GP_FINAL_SKIP_EXPLORATION:-0}" \
        -e GP_FINAL_RUN_GENERAL_SMOKE="${GP_FINAL_RUN_GENERAL_SMOKE:-0}" \
        -e GP_EXPLORATION_GUARD_DURATION="${GP_EXPLORATION_GUARD_DURATION:-90}" \
        "${DOCKER_CONTAINER}" \
        bash -lc "cd $(printf '%q' "${CONTAINER_REPO}") && bash sh_files/test_exploration_final_closure.sh"
    exit $?
fi

run_static_checks

NEEDS_ROS=0
if [[ "${GP_FINAL_SKIP_BUILD:-0}" != "1" ||
      "${GP_FINAL_SKIP_FULL_BUILD:-0}" != "1" ||
      "${GP_FINAL_SKIP_EXPLORATION:-0}" != "1" ||
      "${GP_FINAL_RUN_GENERAL_SMOKE:-0}" == "1" ]]; then
    NEEDS_ROS=1
fi

if [[ "${NEEDS_ROS}" == "1" ]]; then
    if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
        echo "[exploration_final] FAIL: /opt/ros/noetic/setup.bash not found." >&2
        exit 1
    fi

    set +u
    source /opt/ros/noetic/setup.bash
    set -u
fi

if [[ "${GP_FINAL_SKIP_BUILD:-0}" != "1" ]]; then
    echo "[exploration_final] build=catkin_make --pkg general_planner"
    cd "${WORKSPACE_DIR}"
    catkin_make --pkg general_planner
else
    echo "[exploration_final] build=skipped"
fi

if [[ "${GP_FINAL_SKIP_FULL_BUILD:-0}" != "1" ]]; then
    echo "[exploration_final] full_build=catkin_make"
    cd "${WORKSPACE_DIR}"
    catkin_make
else
    echo "[exploration_final] full_build=skipped"
fi

if [[ "${NEEDS_ROS}" == "1" ]]; then
    set +u
    source "${WORKSPACE_DIR}/devel/setup.bash"
    set -u
fi

if [[ "${GP_FINAL_SKIP_EXPLORATION:-0}" != "1" ]]; then
    cd "${ROOT_DIR}"
    bash sh_files/test_exploration_big_field_guard.sh
    GP_TRAP_LOG_EXPECT_SIGNATURE=any GP_TRAP_LOG_EXPECT_RESPONSE=1 \
        bash sh_files/test_exploration_trap_log_guard.sh \
        "${GP_EXPLORATION_GUARD_LOG:-/tmp/general_planner_exploration_big_field_guard.log}"
else
    echo "[exploration_final] exploration_guard=skipped"
fi

if [[ "${GP_FINAL_RUN_GENERAL_SMOKE:-0}" == "1" ]]; then
    cd "${ROOT_DIR}"
    GP_USE_DOCKER=0 GP_SKIP_BUILD=1 bash sh_files/test_general_planner_ros1.sh
else
    echo "[exploration_final] general_smoke=skipped"
fi

echo "[exploration_final] PASS"
