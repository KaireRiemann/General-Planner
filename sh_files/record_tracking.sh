#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags/tracking}"
BAG_PREFIX="${BAG_PREFIX:-tracking_$(date +%Y%m%d_%H%M%S)}"
PLANNER_NS="${PLANNER_NS:-/fsm_node}"
DRONE_NS="${DRONE_NS:-/drone_1}"
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

PLANNER_NS="$(normalize_ns "${PLANNER_NS}")"
DRONE_NS="$(normalize_ns "${DRONE_NS}")"

if ! command -v rosbag >/dev/null 2>&1; then
  echo "[record_tracking] rosbag not found. Source your ROS environment first, for example:" >&2
  echo "  source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

mkdir -p "${BAG_DIR}"

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

# Time, transforms, and logs.
add_topic /clock
add_topic /tf
add_topic /tf_static
add_topic /rosout
add_topic /rosout_agg
add_topic /planning/diagnostics/events

# Tracking target inputs. These are the most important topics for replaying and
# diagnosing target prediction problems.
add_topic /tracking/target_odom
add_topic /tracking/target_path
add_topic /tracking/target_prediction

# Perching/transition inputs are useful when the same tracking stack is used in
# tracking-perching modes.
add_topic /planning/task_mode
add_topic /perching/surface_odom
add_topic /perching/surface_markers

# Robot state and planner outputs, with both namespaced and non-namespaced forms.
add_topic /lidar_slam/odom
add_topic /Odometry
add_topic /ekf/ekf_odom
add_topic /ekf/ekf_odom_lidar
add_topic /mavros/local_position/odom
add_topic /planning/pos_cmd
add_topic /planning_cmd/poly_traj
add_topic "${DRONE_NS}/lidar_slam/odom"
add_topic "${DRONE_NS}/cloud_registered"
add_topic "${DRONE_NS}/planning/pos_cmd"
add_topic "${DRONE_NS}/planning_cmd/poly_traj"

# Controller and execution diagnostics.
add_topic /debugPx4ctrl
add_topic /mavros/state
add_topic /mavros/imu/data
add_topic /px4ctrl/takeoff_land

# Planner visual/debug outputs.
add_topic "${PLANNER_NS}/fsm/path"
add_topic "${PLANNER_NS}/visualization/goal"
add_topic "${PLANNER_NS}/visualization/frontend_path"
add_topic "${PLANNER_NS}/visualization/committed_traj"
add_topic "${PLANNER_NS}/visualization/exp_traj"
add_topic "${PLANNER_NS}/visualization/backup_traj"
add_topic "${PLANNER_NS}/visualization/yaw_traj"
add_topic "${PLANNER_NS}/visualization/tracking_fov"
add_topic "${PLANNER_NS}/visualization/exp_sfc"
add_topic "${PLANNER_NS}/visualization/backup_sfc"
add_topic "${PLANNER_NS}/visualization/astar_debug"
add_topic "${PLANNER_NS}/visualization/ciri_debug_mkr"
add_topic "${PLANNER_NS}/visualization/ciri_debug_pc"
add_topic "${PLANNER_NS}/visualization/replan_log_mkr"
add_topic "${PLANNER_NS}/visualization/replan_log_pc"

if [[ "${RECORD_MAP:-1}" == "1" ]]; then
  add_topic "${PLANNER_NS}/rog_map/occ"
  add_topic "${PLANNER_NS}/rog_map/inf_occ"
  add_topic "${PLANNER_NS}/rog_map/unk"
  add_topic "${PLANNER_NS}/rog_map/inf_unk"
  add_topic "${PLANNER_NS}/rog_map/frontier"
  add_topic "${PLANNER_NS}/rog_map/esdf"
  add_topic "${PLANNER_NS}/rog_map/esdf/neg"
  add_topic "${PLANNER_NS}/rog_map/esdf/occ"
  add_topic "${PLANNER_NS}/rog_map/map_bound"
fi

if [[ "${RECORD_RAW_LIDAR:-0}" == "1" ]]; then
  add_topic /cloud_registered
  add_topic /cloud_registered_body
  add_topic /livox/lidar
  add_topic /livox/imu
  add_topic /velodyne_points
  add_topic /os_cloud_node/points
  add_topic /os_cloud_node/imu
fi

if [[ "${RECORD_SWARM:-0}" == "1" ]]; then
  add_topic /broadcast_traj_from_planner
  add_topic /broadcast_traj_to_planner
  add_topic /swarm/trajectory
  add_topic /swarm/state
  add_topic /drone_0/lidar_slam/odom
  add_topic /drone_0/cloud_registered
  add_topic /drone_0/planning/pos_cmd
  add_topic /drone_0/planning_cmd/poly_traj
  add_topic /drone_1/lidar_slam/odom
  add_topic /drone_1/cloud_registered
  add_topic /drone_1/planning/pos_cmd
  add_topic /drone_1/planning_cmd/poly_traj
fi

if [[ -n "${EXTRA_TOPICS:-}" ]]; then
  # shellcheck disable=SC2206
  extra_topics=(${EXTRA_TOPICS})
  for topic in "${extra_topics[@]}"; do
    add_topic "${topic}"
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

echo "[record_tracking] output: ${BAG_DIR}/${BAG_PREFIX}*.bag"
echo "[record_tracking] planner namespace: ${PLANNER_NS:-/}"
echo "[record_tracking] drone namespace: ${DRONE_NS:-/}"
echo "[record_tracking] topics: ${#topics[@]}"
printf '  %s\n' "${topics[@]}"

exec rosbag record "${record_args[@]}" "${topics[@]}"
