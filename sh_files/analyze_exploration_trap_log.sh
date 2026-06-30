#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 <exploration_log_file>"
    exit 2
fi

LOG_FILE="$1"
if [[ ! -f "${LOG_FILE}" ]]; then
    echo "[exploration_trap_log] FAIL: log file not found: ${LOG_FILE}"
    exit 2
fi

REPEAT_THRESHOLD="${GP_TRAP_LOG_REPEAT_THRESHOLD:-4}"
ASTAR_THRESHOLD="${GP_TRAP_LOG_ASTAR_THRESHOLD:-12}"
CLUSTER_THRESHOLD="${GP_TRAP_LOG_CLUSTER_THRESHOLD:-2}"
INFO_THRESHOLD="${GP_TRAP_LOG_INFO_THRESHOLD:-800}"
RAW_FRONTIER_THRESHOLD="${GP_TRAP_LOG_RAW_FRONTIER_THRESHOLD:-100000}"
REQUIRE_RESPONSE="${GP_TRAP_LOG_REQUIRE_RESPONSE:-1}"

count_regex() {
    local pattern="$1"
    grep -E -c "${pattern}" "${LOG_FILE}" 2>/dev/null || true
}

metric_values() {
    local key="$1"
    grep -oE "(^|[^A-Za-z_])${key}=[0-9.]+" "${LOG_FILE}" 2>/dev/null | \
        grep -oE "${key}=[0-9.]+" | cut -d= -f2 || true
}

max_metric() {
    local key="$1"
    metric_values "${key}" | \
        awk 'BEGIN { found = 0; max_value = 0 } { value = $1 + 0; if (!found || value > max_value) { max_value = value; found = 1 } } END { print found ? max_value : 0 }'
}

low_cluster_count() {
    metric_values "clusters" | \
        awk -v threshold="${CLUSTER_THRESHOLD}" '{ if (($1 + 0) <= threshold) count++ } END { print count + 0 }'
}

max_frontier_repeat() {
    grep -oE "frontier_id=[0-9-]+" "${LOG_FILE}" 2>/dev/null | \
        sort | uniq -c | \
        awk 'BEGIN { max_count = 0; max_key = "none" } { if ($1 > max_count) { max_count = $1; max_key = $2 } } END { print max_count ":" max_key }'
}

float_ge() {
    awk -v lhs="$1" -v rhs="$2" 'BEGIN { exit !(lhs + 0 >= rhs + 0) }'
}

GOAL_SELECTED_COUNT="$(count_regex "ExplorationFrontend] Goal selected")"
LOCAL_TRAP_COUNT="$(count_regex "local_trap_escape_requested")"
MEMORY_RECOVERY_COUNT="$(count_regex "Use memory recovery goal|Delay finish and use memory recovery goal")"
NHBP_REJECT_COUNT="$(count_regex "NHBP rejected")"
MAX_ASTAR_CHECKS="$(max_metric "astar_checks")"
MAX_INFO="$(max_metric "info")"
MAX_RAW_FRONTIERS="$(max_metric "raw_frontiers")"
LOW_CLUSTER_LINES="$(low_cluster_count)"
FRONTIER_REPEAT="$(max_frontier_repeat)"
MAX_FRONTIER_REPEAT="${FRONTIER_REPEAT%%:*}"
MAX_REPEAT_FRONTIER="${FRONTIER_REPEAT#*:}"

TRAP_SIGNATURE=0
if [[ "${GOAL_SELECTED_COUNT}" -gt 0 &&
      "${MAX_FRONTIER_REPEAT}" -ge "${REPEAT_THRESHOLD}" &&
      "${MAX_ASTAR_CHECKS}" -ge "${ASTAR_THRESHOLD}" &&
      "${LOW_CLUSTER_LINES}" -ge "${REPEAT_THRESHOLD}" ]] &&
   float_ge "${MAX_INFO}" "${INFO_THRESHOLD}" &&
   float_ge "${MAX_RAW_FRONTIERS}" "${RAW_FRONTIER_THRESHOLD}"; then
    TRAP_SIGNATURE=1
fi

cat <<SUMMARY
[exploration_trap_log] log=${LOG_FILE}
[exploration_trap_log] goal_selected=${GOAL_SELECTED_COUNT}
[exploration_trap_log] trap_signature=${TRAP_SIGNATURE}
[exploration_trap_log] max_frontier_repeat=${MAX_FRONTIER_REPEAT}
[exploration_trap_log] max_repeat_frontier=${MAX_REPEAT_FRONTIER}
[exploration_trap_log] max_astar_checks=${MAX_ASTAR_CHECKS}
[exploration_trap_log] low_cluster_lines=${LOW_CLUSTER_LINES}
[exploration_trap_log] max_info=${MAX_INFO}
[exploration_trap_log] max_raw_frontiers=${MAX_RAW_FRONTIERS}
[exploration_trap_log] local_trap=${LOCAL_TRAP_COUNT}
[exploration_trap_log] memory_recovery=${MEMORY_RECOVERY_COUNT}
[exploration_trap_log] nhbp_reject=${NHBP_REJECT_COUNT}
SUMMARY

if [[ "${REQUIRE_RESPONSE}" -ne 0 &&
      "${TRAP_SIGNATURE}" -ne 0 &&
      "${LOCAL_TRAP_COUNT}" -eq 0 &&
      "${MEMORY_RECOVERY_COUNT}" -eq 0 ]]; then
    echo "[exploration_trap_log] FAIL: trap signature found without local-trap or memory-recovery response."
    exit 1
fi

echo "[exploration_trap_log] PASS"
