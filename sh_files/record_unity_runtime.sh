#!/usr/bin/env bash
# Record a Unity UAV-Diff + planner_runtime test using the standard runtime
# recorder, plus the Unity ROS-TCP boundary on both sides of the bridge.
#
# The recorder deliberately captures both the original Unity messages and the
# bridge-normalised planner inputs.  This makes timestamp, frame, conversion,
# and command-to-pose failures diagnosable from one bag.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUNTIME_RECORDER="${SCRIPT_DIR}/record_runtime.sh"

if [[ ! -x "${RUNTIME_RECORDER}" ]]; then
  echo "[record_unity_runtime] missing executable runtime recorder: ${RUNTIME_RECORDER}" >&2
  exit 1
fi

# These are the concrete topic contracts in UAV-Diff/MainScene and
# unity_planner_bridge.  Each remains overrideable for an alternate Unity
# scene without modifying this task.
UNITY_RAW_ODOM_TOPIC="${UNITY_RAW_ODOM_TOPIC:-/unity_odom}"
UNITY_COMMAND_ODOM_TOPIC="${UNITY_COMMAND_ODOM_TOPIC:-/drone_0_visual_slam/odom}"
UNITY_LEGACY_COMMAND_ODOM_TOPIC="${UNITY_LEGACY_COMMAND_ODOM_TOPIC:-/odom}"
UNITY_RAW_CLOUD_TOPIC="${UNITY_RAW_CLOUD_TOPIC:-/drone_0_pcl_render_node/cloud}"
UNITY_LEGACY_RAW_CLOUD_TOPIC="${UNITY_LEGACY_RAW_CLOUD_TOPIC:-/mid360/points}"

# planner_runtime must consume bridge-normalised data, not raw Unity odometry:
# /unity_odom uses Unity-relative time and does not provide twist.
export ODOM_TOPIC="${ODOM_TOPIC:-/lidar_slam/odom}"
export CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_registered}"
export RUNTIME_NS="${RUNTIME_NS:-/planner_runtime_node}"
# The Unity stack is always the composed runtime; do not emit legacy-serial
# namespace warnings unless a caller explicitly requests those namespaces.
export EXPLORATION_NS="${EXPLORATION_NS:-${RUNTIME_NS}}"
export NAVIGATION_NS="${NAVIGATION_NS:-${RUNTIME_NS}}"
export BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags/unity_runtime}"
export BAG_PREFIX="${BAG_PREFIX:-unity_runtime_$(date +%Y%m%d_%H%M%S)}"

# Keep any caller-supplied diagnostic topics and append the complete Unity
# boundary. record_runtime.sh deduplicates this list before calling rosbag.
export EXTRA_TOPICS="${EXTRA_TOPICS:-} ${UNITY_RAW_ODOM_TOPIC} ${UNITY_COMMAND_ODOM_TOPIC} ${UNITY_LEGACY_COMMAND_ODOM_TOPIC} ${UNITY_RAW_CLOUD_TOPIC} ${UNITY_LEGACY_RAW_CLOUD_TOPIC}"

UNITY_INTERFACE_FILE="${BAG_DIR}/${BAG_PREFIX}_unity_interface.txt"
if [[ "${DRY_RUN:-0}" != "1" ]]; then
  mkdir -p "${BAG_DIR}"
  {
    printf 'recorded_at_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'record_profile=unity_runtime\n'
    printf 'runtime_namespace=%s\n' "${RUNTIME_NS}"
    printf 'unity_raw_odom_topic=%s\n' "${UNITY_RAW_ODOM_TOPIC}"
    printf 'unity_command_odom_topic=%s\n' "${UNITY_COMMAND_ODOM_TOPIC}"
    printf 'unity_legacy_command_odom_topic=%s\n' "${UNITY_LEGACY_COMMAND_ODOM_TOPIC}"
    printf 'unity_raw_cloud_topic=%s\n' "${UNITY_RAW_CLOUD_TOPIC}"
    printf 'unity_legacy_raw_cloud_topic=%s\n' "${UNITY_LEGACY_RAW_CLOUD_TOPIC}"
    printf 'planner_odom_topic=%s\n' "${ODOM_TOPIC}"
    printf 'planner_cloud_topic=%s\n' "${CLOUD_TOPIC}"
    printf 'planner_command_topic=/planning/pos_cmd\n'
  } >"${UNITY_INTERFACE_FILE}"
  echo "[record_unity_runtime] Unity interface: ${UNITY_INTERFACE_FILE}"
else
  echo "[record_unity_runtime] dry run: Unity interface sidecar would be ${UNITY_INTERFACE_FILE}"
fi

echo "[record_unity_runtime] raw Unity cloud: ${UNITY_RAW_CLOUD_TOPIC}"
echo "[record_unity_runtime] bridge inputs: ${ODOM_TOPIC}, ${CLOUD_TOPIC}"
echo "[record_unity_runtime] Unity pose command: ${UNITY_COMMAND_ODOM_TOPIC}"

# A recorder may be started before Play is pressed, so absence is a warning by
# default. REQUIRE_UNITY=1 turns this into a preflight gate for a test that is
# expected to have already connected through ROS-TCP.
if [[ "${CHECK_UNITY:-1}" == "1" ]] && command -v rostopic >/dev/null 2>&1; then
  missing_topics=()
  for topic in "${UNITY_RAW_ODOM_TOPIC}" "${UNITY_RAW_CLOUD_TOPIC}" \
               "${UNITY_COMMAND_ODOM_TOPIC}" "${ODOM_TOPIC}" "${CLOUD_TOPIC}"; do
    if ! rostopic list 2>/dev/null | grep -Fxq "${topic}"; then
      missing_topics+=("${topic}")
    fi
  done
  if (( ${#missing_topics[@]} == 0 )); then
    echo "[record_unity_runtime] Unity ROS-TCP boundary detected"
  else
    echo "[record_unity_runtime] warning: missing Unity/runtime topics: ${missing_topics[*]}" >&2
    if [[ "${REQUIRE_UNITY:-0}" == "1" ]]; then
      exit 4
    fi
  fi
fi

exec "${RUNTIME_RECORDER}" "$@"
