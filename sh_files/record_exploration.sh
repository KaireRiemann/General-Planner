#!/usr/bin/env bash
# Record the complete high-speed exploration decision and execution chain.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags/exploration}"
BAG_PREFIX="${BAG_PREFIX:-exploration_$(date +%Y%m%d_%H%M%S)}"
EXPLORATION_NS="${EXPLORATION_NS:-/exploration_node}"
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

if ! command -v rosbag >/dev/null 2>&1; then
  echo "[record_exploration] rosbag not found. Source your ROS environment first, for example:" >&2
  echo "  source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

mkdir -p "${BAG_DIR}"

# Prefer the topics actually configured on the running exploration node. They
# can still be overridden explicitly for recording before the node is started.
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

# Exploration trigger and the exact perception/state inputs in use.
add_topic /move_base_simple/goal
add_topic /goal
add_topic "${ODOM_TOPIC}"
add_topic "${CLOUD_TOPIC}"
add_topic /lidar_slam/odom
add_topic /Odometry
add_topic /ekf/ekf_odom
add_topic /ekf/ekf_odom_lidar
add_topic /mavros/local_position/odom
add_topic /cloud_registered
add_topic /cloud_registered_body

# Common simulation topic variants used by the exploration launch files.
add_topic /quad_0/lidar_slam/odom
add_topic /quad0_pcl_render_node/cloud
add_topic /drone_0_unity_odom
add_topic /drone_0_pcl_render_node/cloud

# High-speed exploration FSM, generated trajectories, execution, and timing.
add_topic /planning/replan
add_topic /planning/heartbeat
add_topic /planning/trajectory
add_topic /planning/yaw_trajectory
add_topic /planning/pos_cmd
add_topic /planning/position_cmd_vis
add_topic /planning/travel_traj
add_topic /planning/static
add_topic /planning/state
add_topic /planning/speed
add_topic /time_cost
add_topic /debugPx4ctrl
add_topic /px4ctrl/takeoff_land
add_topic /mavros/state
add_topic /mavros/imu/data

# Global tour, Bubble-A*, raw frontier cells, clusters/viewpoints, and coverage
# guidance. These are needed to distinguish raw-frontier orphaning from a true
# completed exploration.
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

if [[ "${RECORD_MAP:-1}" == "1" ]]; then
  # ROG-Map occupancy/frontier products and map boundary under the private node
  # namespace used by high-speed exploration.
  add_topic "${EXPLORATION_NS}/rog_map/occ"
  add_topic "${EXPLORATION_NS}/rog_map/inf_occ"
  add_topic "${EXPLORATION_NS}/rog_map/unk"
  add_topic "${EXPLORATION_NS}/rog_map/inf_unk"
  add_topic "${EXPLORATION_NS}/rog_map/frontier"
  add_topic "${EXPLORATION_NS}/rog_map/esdf"
  add_topic "${EXPLORATION_NS}/rog_map/esdf/neg"
  add_topic "${EXPLORATION_NS}/rog_map/esdf/occ"
  add_topic "${EXPLORATION_NS}/rog_map/map_bound"
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

# A bag does not contain the ROS parameter server. Keep a sidecar snapshot so
# frontier thresholds, task bounds, finish guards, and topic remaps can be
# reconstructed when diagnosing the run.
if [[ "${RECORD_PARAMS:-1}" == "1" ]] && command -v rosparam >/dev/null 2>&1; then
  PARAM_FILE="${BAG_DIR}/${BAG_PREFIX}_params.yaml"
  if rosparam dump "${PARAM_FILE}" "${PARAM_NAMESPACE:-/}" 2>/dev/null; then
    echo "[record_exploration] parameters: ${PARAM_FILE}"
  else
    echo "[record_exploration] warning: failed to snapshot ROS parameters" >&2
    rm -f "${PARAM_FILE}"
  fi
fi

# Most high-speed scene configs disable these publishers by default. Merely
# subscribing in rosbag cannot recover frontier/cluster data that the node never
# publishes; tell the operator before the diagnostic run becomes unusable.
if command -v rosparam >/dev/null 2>&1 && [[ -n "${EXPLORATION_NS}" ]]; then
  for debug_param in FrontierManager/view_frt FrontierManager/view_cluster; do
    debug_value="$(rosparam get "${EXPLORATION_NS}/${debug_param}" 2>/dev/null || true)"
    if [[ "${debug_value}" == "false" || "${debug_value}" == "0" ]]; then
      echo "[record_exploration] warning: ${EXPLORATION_NS}/${debug_param}=${debug_value}" >&2
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

echo "[record_exploration] output: ${BAG_DIR}/${BAG_PREFIX}*.bag"
echo "[record_exploration] exploration namespace: ${EXPLORATION_NS:-/}"
echo "[record_exploration] odometry topic: ${ODOM_TOPIC}"
echo "[record_exploration] cloud topic: ${CLOUD_TOPIC}"
echo "[record_exploration] topics: ${#topics[@]}"
printf '  %s\n' "${topics[@]}"

exec rosbag record "${record_args[@]}" "${topics[@]}"
