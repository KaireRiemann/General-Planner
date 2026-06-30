#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "${ROOT_DIR}/../.." && pwd)"
cd "${ROOT_DIR}"

DURATION="${GP_EXPLORATION_GUARD_DURATION:-90}"
LOG_FILE="${GP_EXPLORATION_GUARD_LOG:-/tmp/general_planner_exploration_big_field_guard.log}"
TRAP_ANALYZER="${ROOT_DIR}/sh_files/analyze_exploration_trap_log.sh"

MAX_ODOM_STALE="${GP_EXPLORATION_MAX_ODOM_STALE:-0}"
MIN_GOAL_SELECTED="${GP_EXPLORATION_MIN_GOAL_SELECTED:-5}"
MIN_PLAN_SUCCESS="${GP_EXPLORATION_MIN_PLAN_SUCCESS:-3}"
MAX_PLAN_FAILED="${GP_EXPLORATION_MAX_PLAN_FAILED:-20}"
MAX_NHBP_REJECT="${GP_EXPLORATION_MAX_NHBP_REJECT:-8}"
MAX_FRONTIERS="${GP_EXPLORATION_MAX_FRONTIERS:-1600}"
MAX_ASTAR_TIMEOUT="${GP_EXPLORATION_MAX_ASTAR_TIMEOUT:-300}"

set +u
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_DIR}/devel/setup.bash"
set -u

rosnode kill /fsm_node /perfect_drone >/dev/null 2>&1 || true
sleep 1

rm -f "${LOG_FILE}"
set +e
timeout "${DURATION}s" roslaunch task_planner exploration.launch \
    rviz:=false sim_output:=screen planner_output:=screen \
    > "${LOG_FILE}" 2>&1
ROSLAUNCH_STATUS=$?
set -e

count_regex() {
    local pattern="$1"
    grep -E -c "${pattern}" "${LOG_FILE}" 2>/dev/null || true
}

max_metric() {
    local key="$1"
    local value
    value="$(grep -oE "(^|[^A-Za-z_])${key}=[0-9]+" "${LOG_FILE}" 2>/dev/null | \
        grep -oE "${key}=[0-9]+" | \
        awk -F= 'BEGIN { max_value = 0 } { if ($2 > max_value) max_value = $2 } END { print max_value + 0 }' || true)"
    if [[ -z "${value}" ]]; then
        echo 0
    else
        echo "${value}"
    fi
}

ODOM_STALE_COUNT="$(count_regex "ODOM_STALE")"
ASTAR_TIMEOUT_COUNT="$(count_regex "time limit exceeded")"
NHBP_REJECT_COUNT="$(count_regex "NHBP rejected")"
LOCAL_TRAP_COUNT="$(count_regex "local_trap_escape_requested")"
MEMORY_RECOVERY_COUNT="$(count_regex "Use memory recovery goal|Delay finish and use memory recovery goal")"
PLAN_SUCCESS_COUNT="$(count_regex "ReplanOnce succeed|PlanFromRest succeed|GenerateExpTrajectory SUCCESS")"
PLAN_FAILED_COUNT="$(count_regex "GenerateExpTrajectory failed|Replan failed:|PlanFromRest failed: (no odom|map is not ready|frontend|runtime manager)")"
GOAL_SELECTED_COUNT="$(count_regex "Goal selected")"
FATAL_COUNT="$(count_regex "FATAL|Segmentation fault|core dumped")"
MAX_FRONTIERS_SEEN="$(max_metric "frontiers")"
MAX_RAW_FRONTIERS_SEEN="$(max_metric "raw_frontiers")"
MAX_ASTAR_CHECKS_SEEN="$(max_metric "astar_checks")"

cat <<SUMMARY
[exploration_guard] roslaunch_status=${ROSLAUNCH_STATUS}
[exploration_guard] log=${LOG_FILE}
[exploration_guard] goal_selected=${GOAL_SELECTED_COUNT}
[exploration_guard] plan_success=${PLAN_SUCCESS_COUNT}
[exploration_guard] plan_failed=${PLAN_FAILED_COUNT}
[exploration_guard] odom_stale=${ODOM_STALE_COUNT}
[exploration_guard] nhbp_reject=${NHBP_REJECT_COUNT}
[exploration_guard] local_trap=${LOCAL_TRAP_COUNT}
[exploration_guard] memory_recovery=${MEMORY_RECOVERY_COUNT}
[exploration_guard] astar_timeout=${ASTAR_TIMEOUT_COUNT}
[exploration_guard] max_astar_checks=${MAX_ASTAR_CHECKS_SEEN}
[exploration_guard] max_frontiers=${MAX_FRONTIERS_SEEN}
[exploration_guard] max_raw_frontiers=${MAX_RAW_FRONTIERS_SEEN}
[exploration_guard] fatal=${FATAL_COUNT}
SUMMARY

TRAP_ANALYZER_STATUS=0
if [[ -x "${TRAP_ANALYZER}" ]]; then
    set +e
    GP_TRAP_LOG_REQUIRE_RESPONSE=1 "${TRAP_ANALYZER}" "${LOG_FILE}"
    TRAP_ANALYZER_STATUS=$?
    set -e
fi

FAILED=0
if [[ "${ROSLAUNCH_STATUS}" -ne 0 && "${ROSLAUNCH_STATUS}" -ne 124 ]]; then
    echo "[exploration_guard] FAIL: roslaunch exited with unexpected status ${ROSLAUNCH_STATUS}."
    FAILED=1
fi
if [[ "${FATAL_COUNT}" -gt 0 ]]; then
    echo "[exploration_guard] FAIL: fatal runtime error found in log."
    FAILED=1
fi
if [[ "${ODOM_STALE_COUNT}" -gt "${MAX_ODOM_STALE}" ]]; then
    echo "[exploration_guard] FAIL: ODOM_STALE ${ODOM_STALE_COUNT} > ${MAX_ODOM_STALE}."
    FAILED=1
fi
if [[ "${GOAL_SELECTED_COUNT}" -lt "${MIN_GOAL_SELECTED}" ]]; then
    echo "[exploration_guard] FAIL: Goal selected ${GOAL_SELECTED_COUNT} < ${MIN_GOAL_SELECTED}."
    FAILED=1
fi
if [[ "${PLAN_SUCCESS_COUNT}" -lt "${MIN_PLAN_SUCCESS}" ]]; then
    echo "[exploration_guard] FAIL: Plan success ${PLAN_SUCCESS_COUNT} < ${MIN_PLAN_SUCCESS}."
    FAILED=1
fi
if [[ "${PLAN_FAILED_COUNT}" -gt "${MAX_PLAN_FAILED}" ]]; then
    echo "[exploration_guard] FAIL: Plan failed ${PLAN_FAILED_COUNT} > ${MAX_PLAN_FAILED}."
    FAILED=1
fi
if [[ "${NHBP_REJECT_COUNT}" -gt "${MAX_NHBP_REJECT}" ]]; then
    echo "[exploration_guard] FAIL: NHBP rejected ${NHBP_REJECT_COUNT} > ${MAX_NHBP_REJECT}."
    FAILED=1
fi
if [[ "${MAX_FRONTIERS_SEEN}" -gt "${MAX_FRONTIERS}" ]]; then
    echo "[exploration_guard] FAIL: frontiers cap ${MAX_FRONTIERS_SEEN} > ${MAX_FRONTIERS}."
    FAILED=1
fi
if [[ "${ASTAR_TIMEOUT_COUNT}" -gt "${MAX_ASTAR_TIMEOUT}" ]]; then
    echo "[exploration_guard] FAIL: A* timeout ${ASTAR_TIMEOUT_COUNT} > ${MAX_ASTAR_TIMEOUT}."
    FAILED=1
fi
if [[ "${TRAP_ANALYZER_STATUS}" -ne 0 ]]; then
    echo "[exploration_guard] FAIL: exploration trap analyzer failed."
    FAILED=1
fi

if [[ "${FAILED}" -ne 0 ]]; then
    echo "[exploration_guard] --- log tail ---"
    tail -120 "${LOG_FILE}"
    exit 1
fi

echo "[exploration_guard] PASS"
