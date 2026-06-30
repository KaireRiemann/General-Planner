#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ANALYZER="${ROOT_DIR}/sh_files/analyze_exploration_trap_log.sh"
LOG_FILE="${1:-${GP_EXPLORATION_TRAP_LOG:-}}"
EXPECT_SIGNATURE="${GP_TRAP_LOG_EXPECT_SIGNATURE:-1}"
EXPECT_RESPONSE="${GP_TRAP_LOG_EXPECT_RESPONSE:-0}"

if [[ -z "${LOG_FILE}" ]]; then
    echo "usage: $0 <exploration_log_file>"
    echo "[exploration_trap_log_guard] FAIL: no log file provided."
    exit 2
fi
if [[ ! -x "${ANALYZER}" ]]; then
    echo "[exploration_trap_log_guard] FAIL: analyzer not executable: ${ANALYZER}"
    exit 2
fi

TMP_OUTPUT="$(mktemp)"
cleanup() {
    rm -f "${TMP_OUTPUT}"
}
trap cleanup EXIT

GP_TRAP_LOG_REQUIRE_RESPONSE=0 "${ANALYZER}" "${LOG_FILE}" | tee "${TMP_OUTPUT}"

metric_value() {
    local key="$1"
    grep -E "^\[exploration_trap_log\] ${key}=" "${TMP_OUTPUT}" | \
        tail -1 | sed -E "s/^.*${key}=//"
}

TRAP_SIGNATURE="$(metric_value "trap_signature")"
LOCAL_TRAP="$(metric_value "local_trap")"
MEMORY_RECOVERY="$(metric_value "memory_recovery")"
LOCAL_TRAP="${LOCAL_TRAP:-0}"
MEMORY_RECOVERY="${MEMORY_RECOVERY:-0}"

FAILED=0
if [[ "${EXPECT_SIGNATURE}" != "any" &&
      "${TRAP_SIGNATURE}" != "${EXPECT_SIGNATURE}" ]]; then
    echo "[exploration_trap_log_guard] FAIL: trap_signature=${TRAP_SIGNATURE}, expected=${EXPECT_SIGNATURE}."
    FAILED=1
fi
if [[ "${EXPECT_RESPONSE}" -ne 0 &&
      "${LOCAL_TRAP}" -eq 0 &&
      "${MEMORY_RECOVERY}" -eq 0 ]]; then
    echo "[exploration_trap_log_guard] FAIL: expected local-trap or memory-recovery response."
    FAILED=1
fi

if [[ "${FAILED}" -ne 0 ]]; then
    exit 1
fi

echo "[exploration_trap_log_guard] PASS"
