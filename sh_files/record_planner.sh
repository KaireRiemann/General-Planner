#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags/planner}"
BAG_PREFIX="${BAG_PREFIX:-planner_$(date +%Y%m%d_%H%M%S)}"
PLANNER_NS="${PLANNER_NS:-/fsm_node}"
PLANNER_NS="/${PLANNER_NS#/}"
PLANNER_NS="${PLANNER_NS%/}"
SPLIT_DURATION="${SPLIT_DURATION:-10m}"
COMPRESSION="${COMPRESSION:-lz4}"

if ! command -v rosbag >/dev/null 2>&1; then
  echo "[record_planner] rosbag not found. Source your ROS environment first, for example:" >&2
  echo "  source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

mkdir -p "${BAG_DIR}"

topics=(
  # Time, transforms, and ROS logs.
  /tf
  /tf_static
  /rosout
  /rosout_agg

  # Planner perception and state inputs.
  /cloud_registered
  /cloud_registered_body
  /Odometry
  /path
  /ekf/ekf_odom
  /ekf/ekf_odom_lidar
  /lidar_slam/odom

  # Goal, task, tracking, and perching inputs.
  /goal
  /planning/click_goal
  /move_base_simple/goal
  /planning/task_mode
  /tracking/target_odom
  /tracking/target_path
  /tracking/target_prediction
  /perching/surface_odom
  /perching/surface_markers

  # Executed and committed trajectory outputs.
  /planning/pos_cmd
  /planning_cmd/poly_traj
  "${PLANNER_NS}/fsm/path"
  "${PLANNER_NS}/visualization/committed_traj"
  "${PLANNER_NS}/visualization/exp_traj"
  "${PLANNER_NS}/visualization/backup_traj"
  "${PLANNER_NS}/visualization/yaw_traj"
  "${PLANNER_NS}/visualization/tracking_fov"

  # Safe corridor, frontend path, and planner debug visualization.
  "${PLANNER_NS}/visualization/goal"
  "${PLANNER_NS}/visualization/frontend_path"
  "${PLANNER_NS}/visualization/exp_sfc"
  "${PLANNER_NS}/visualization/backup_sfc"
  "${PLANNER_NS}/visualization/astar_debug"
  "${PLANNER_NS}/visualization/ciri_debug_mkr"
  "${PLANNER_NS}/visualization/ciri_debug_pc"
  "${PLANNER_NS}/visualization/replan_log_mkr"
  "${PLANNER_NS}/visualization/replan_log_pc"

  # ROG-Map occupancy, inflated occupancy, ESDF, and map bounds.
  "${PLANNER_NS}/rog_map/occ"
  "${PLANNER_NS}/rog_map/inf_occ"
  "${PLANNER_NS}/rog_map/unk"
  "${PLANNER_NS}/rog_map/inf_unk"
  "${PLANNER_NS}/rog_map/frontier"
  "${PLANNER_NS}/rog_map/esdf"
  "${PLANNER_NS}/rog_map/esdf/neg"
  "${PLANNER_NS}/rog_map/esdf/occ"
  "${PLANNER_NS}/rog_map/map_bound"

  # Minimal controller and MAVROS state for checking whether trajectory execution followed planning.
  /debugPx4ctrl
  /mavros/state
  /mavros/local_position/odom
  /mavros/imu/data
  /px4ctrl/takeoff_land
)

if [[ "${RECORD_RAW_LIDAR:-0}" == "1" ]]; then
  topics+=(
    /livox/lidar
    /livox/imu
    /velodyne_points
    /os_cloud_node/points
    /os_cloud_node/imu
  )
fi

if [[ "${RECORD_SWARM:-0}" == "1" ]]; then
  topics+=(
    /broadcast_traj_from_planner
    /broadcast_traj_to_planner
    /drone_0/planning_cmd/poly_traj
    /drone_0/planning/pos_cmd
    /drone_0/lidar_slam/odom
    /drone_0/cloud_registered
    /drone_1/planning_cmd/poly_traj
    /drone_1/planning/pos_cmd
    /drone_1/lidar_slam/odom
    /drone_1/cloud_registered
  )
fi

if [[ -n "${EXTRA_TOPICS:-}" ]]; then
  # shellcheck disable=SC2206
  extra_topics=(${EXTRA_TOPICS})
  topics+=("${extra_topics[@]}")
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

echo "[record_planner] output: ${BAG_DIR}/${BAG_PREFIX}*.bag"
echo "[record_planner] planner namespace: ${PLANNER_NS}"
echo "[record_planner] topics: ${#topics[@]}"
printf '  %s\n' "${topics[@]}"

exec rosbag record "${record_args[@]}" "${topics[@]}"
