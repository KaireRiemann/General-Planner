#!/usr/bin/env bash
# Record the unified planner_runtime decision / handover / execution chain:
# supervisor status, mode requests, exploration + navigation side commands,
# gateway output, and the same exploration diagnostics used by
# record_exploration.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags/runtime}"
BAG_PREFIX="${BAG_PREFIX:-runtime_$(date +%Y%m%d_%H%M%S)}"
EXPLORATION_NS="${EXPLORATION_NS:-/exploration_node}"
NAVIGATION_NS="${NAVIGATION_NS:-/navigation_fsm_node}"
RUNTIME_NS="${RUNTIME_NS:-/planner_runtime_node}"
SPLIT_DURATION="${SPLIT_DURATION:-10m}"
COMPRESSION="${COMPRESSION:-lz4}"

normalize_ns() {
  local ns="$1"
  if [[ -z "${ns}" || "${ns}" == "/" ]]; then
    echo ""
    return
  fi
  ns="/${ns#/}"
  echo "${ns%/}"
}

EXPLORATION_NS="$(normalize_ns "${EXPLORATION_NS}")"
NAVIGATION_NS="$(normalize_ns "${NAVIGATION_NS}")"
RUNTIME_NS="$(normalize_ns "${RUNTIME_NS}")"

if ! command -v rosbag >/dev/null 2>&1; then
  echo "[record_runtime] rosbag not found. Source your ROS environment first, for example:" >&2
  echo "  source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

mkdir -p "${BAG_DIR}"

# Prefer topics configured on the running exploration node when available.
ODOM_TOPIC="${ODOM_TOPIC:-}"
CLOUD_TOPIC="${CLOUD_TOPIC:-}"
if command -v rosparam >/dev/null 2>&1 && [[ -n "${EXPLORATION_NS}" ]]; then
  if [[ -z "${ODOM_TOPIC}" ]]; then
    ODOM_TOPIC="$(rosparam get "${EXPLORATION_NS}/odometry_topic" 2>/dev/null || true)"
  fi
  if [[ -z "${CLOUD_TOPIC}" ]]; then
    CLOUD_TOPIC="$(rosparam get "${EXPLORATION_NS}/cloud_topic" 2>/dev/null || true)"
  fi
fi
if command -v rosparam >/dev/null 2>&1 && [[ -z "${ODOM_TOPIC}" && -n "${RUNTIME_NS}" ]]; then
  ODOM_TOPIC="$(rosparam get "${RUNTIME_NS}/odometry_topic" 2>/dev/null || true)"
fi
ODOM_TOPIC="${ODOM_TOPIC:-/lidar_slam/odom}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/cloud_registered}"

declare -A seen_topics=()
topics=()

add_topic() {
  local topic="$1"
  if [[ -z "${topic}" ]]; then
    return
  fi
  topic="/${topic#/}"
  if [[ -n "${seen_topics[${topic}]:-}" ]]; then
    return
  fi
  seen_topics["${topic}"]=1
  topics+=("${topic}")
}

# Time, transforms, logs, and structured planner diagnostics.
add_topic /clock
add_topic /tf
add_topic /tf_static
add_topic /rosout
add_topic /rosout_agg
add_topic /diagnostics
add_topic /diagnostics_agg
add_topic /planning/diagnostics/events

# Unified runtime control plane (mode switch, hover handoff, click routing).
add_topic /planner/status
add_topic /planner/mode_request
add_topic /planner/mode_request_text
add_topic /planner/click_goal
add_topic /planner/navigation/goal
add_topic /planner/exploration/trigger
add_topic /goal
add_topic /move_base_simple/goal

# Perception / state inputs.
add_topic "${ODOM_TOPIC}"
add_topic "${CLOUD_TOPIC}"
add_topic /lidar_slam/odom
add_topic /Odometry
add_topic /ekf/ekf_odom
add_topic /ekf/ekf_odom_lidar
add_topic /mavros/local_position/odom
add_topic /cloud_registered
add_topic /cloud_registered_body
add_topic /quad_0/lidar_slam/odom
add_topic /quad0_pcl_render_node/cloud
add_topic /drone_0_unity_odom
add_topic /drone_0_pcl_render_node/cloud

# Exploration adapter command / status / traj_server chain.
add_topic /planning/exploration/command
add_topic /planning/exploration/status
add_topic /planning/exploration/command_enabled
add_topic /planning/exploration/pos_cmd
add_topic /planning/replan
add_topic /planning/heartbeat
add_topic /planning/trajectory
add_topic /planning/yaw_trajectory
add_topic /planning/position_cmd_vis
add_topic /planning/travel_traj
add_topic /planning/static
add_topic /planning/state
add_topic /planning/speed
add_topic /time_cost

# Navigation adapter command / status / goal handoff.
add_topic /planning/navigation/command
add_topic /planning/navigation/status
add_topic /planning/navigation/pos_cmd
add_topic /planning/navigation_task_mode
add_topic /planning/click_goal
add_topic /planning/task_mode

# Gateway final flight authority (controller input).
add_topic /planning/pos_cmd
add_topic /planning_cmd/poly_traj
add_topic /debugPx4ctrl
add_topic /px4ctrl/takeoff_land
add_topic /mavros/state
add_topic /mavros/imu/data

# Exploration decision diagnostics (frontier / bubble / coverage).
add_topic /global_tour
add_topic /viz_graph_topic
add_topic /bubble_visualizer/sphere
add_topic /bubble_visualizer/sphere_debug
add_topic /bubble_visualizer/frontend_traj
add_topic "${EXPLORATION_NS}/frt"
add_topic "${EXPLORATION_NS}/occ"
add_topic "${EXPLORATION_NS}/pocc"
add_topic "${EXPLORATION_NS}/sf_cluster_marker"
add_topic "${EXPLORATION_NS}/norm_directions"
add_topic "${EXPLORATION_NS}/viewpoint_centers"
add_topic "${EXPLORATION_NS}/bad_obs"
add_topic "${EXPLORATION_NS}/good_obs"
add_topic "${EXPLORATION_NS}/coverage_guidance/route"
add_topic /mem_cost
add_topic /mem_cost_2
add_topic /mem_cost_3

# Navigation FSM visualization and topology.
add_topic "${NAVIGATION_NS}/fsm/path"
add_topic "${NAVIGATION_NS}/visualization/committed_traj"
add_topic "${NAVIGATION_NS}/visualization/exp_traj"
add_topic "${NAVIGATION_NS}/visualization/backup_traj"
add_topic "${NAVIGATION_NS}/visualization/yaw_traj"
add_topic "${NAVIGATION_NS}/visualization/goal"
add_topic "${NAVIGATION_NS}/visualization/frontend_path"
add_topic "${NAVIGATION_NS}/visualization/exp_sfc"
add_topic "${NAVIGATION_NS}/visualization/backup_sfc"
add_topic "${NAVIGATION_NS}/visualization/astar_debug"
add_topic "${NAVIGATION_NS}/topology/markers"
add_topic "${NAVIGATION_NS}/topology/graph"

if [[ "${RECORD_MAP:-1}" == "1" ]]; then
  add_topic "${EXPLORATION_NS}/rog_map/occ"
  add_topic "${EXPLORATION_NS}/rog_map/inf_occ"
  add_topic "${EXPLORATION_NS}/rog_map/unk"
  add_topic "${EXPLORATION_NS}/rog_map/inf_unk"
  add_topic "${EXPLORATION_NS}/rog_map/frontier"
  add_topic "${EXPLORATION_NS}/rog_map/esdf"
  add_topic "${EXPLORATION_NS}/rog_map/esdf/neg"
  add_topic "${EXPLORATION_NS}/rog_map/esdf/occ"
  add_topic "${EXPLORATION_NS}/rog_map/map_bound"

  add_topic "${NAVIGATION_NS}/rog_map/occ"
  add_topic "${NAVIGATION_NS}/rog_map/inf_occ"
  add_topic "${NAVIGATION_NS}/rog_map/unk"
  add_topic "${NAVIGATION_NS}/rog_map/inf_unk"
  add_topic "${NAVIGATION_NS}/rog_map/frontier"
  add_topic "${NAVIGATION_NS}/rog_map/esdf"
  add_topic "${NAVIGATION_NS}/rog_map/esdf/neg"
  add_topic "${NAVIGATION_NS}/rog_map/esdf/occ"
  add_topic "${NAVIGATION_NS}/rog_map/map_bound"
fi

if [[ "${RECORD_RAW_LIDAR:-0}" == "1" ]]; then
  add_topic /livox/lidar
  add_topic /livox/imu
  add_topic /velodyne_points
  add_topic /os_cloud_node/points
  add_topic /os_cloud_node/imu
fi

if [[ -n "${EXTRA_TOPICS:-}" ]]; then
  # shellcheck disable=SC2206
  extra_topics=(${EXTRA_TOPICS})
  for topic in "${extra_topics[@]}"; do
    add_topic "${topic}"
  done
fi

# Sidecar parameter dump: needed to reconstruct mode gates, remaps, and map
# ceilings when comparing exploration.launch vs planner_runtime.launch runs.
if [[ "${RECORD_PARAMS:-1}" == "1" ]] && command -v rosparam >/dev/null 2>&1; then
  PARAM_FILE="${BAG_DIR}/${BAG_PREFIX}_params.yaml"
  if rosparam dump "${PARAM_FILE}" "${PARAM_NAMESPACE:-/}" 2>/dev/null; then
    echo "[record_runtime] parameters: ${PARAM_FILE}"
  else
    echo "[record_runtime] warning: failed to snapshot ROS parameters" >&2
    rm -f "${PARAM_FILE}"
  fi
fi

if command -v rosparam >/dev/null 2>&1 && [[ -n "${EXPLORATION_NS}" ]]; then
  for debug_param in FrontierManager/view_frt FrontierManager/view_cluster; do
    debug_value="$(rosparam get "${EXPLORATION_NS}/${debug_param}" 2>/dev/null || true)"
    if [[ "${debug_value}" == "false" || "${debug_value}" == "0" ]]; then
      echo "[record_runtime] warning: ${EXPLORATION_NS}/${debug_param}=${debug_value}" >&2
      echo "  Enable it in the scene YAML before starting exploration to record frontier decisions." >&2
    fi
  done
fi

record_args=(--tcpnodelay -O "${BAG_DIR}/${BAG_PREFIX}")
if [[ "${COMPRESSION}" == "lz4" ]]; then
  record_args+=(--lz4)
elif [[ "${COMPRESSION}" == "bz2" ]]; then
  record_args+=(--bz2)
fi

if [[ "${NO_SPLIT:-0}" != "1" ]]; then
  record_args+=(--split --duration="${SPLIT_DURATION}")
fi

echo "[record_runtime] output: ${BAG_DIR}/${BAG_PREFIX}*.bag"
echo "[record_runtime] runtime namespace: ${RUNTIME_NS:-/}"
echo "[record_runtime] exploration namespace: ${EXPLORATION_NS:-/}"
echo "[record_runtime] navigation namespace: ${NAVIGATION_NS:-/}"
echo "[record_runtime] odometry topic: ${ODOM_TOPIC}"
echo "[record_runtime] cloud topic: ${CLOUD_TOPIC}"
echo "[record_runtime] topics: ${#topics[@]}"
printf '  %s\n' "${topics[@]}"

exec rosbag record "${record_args[@]}" "${topics[@]}"
