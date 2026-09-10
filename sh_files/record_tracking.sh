#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG_DIR="${BAG_DIR:-${REPO_ROOT}/bags/tracking}"
BAG_PREFIX="${BAG_PREFIX:-tracking_$(date +%Y%m%d_%H%M%S)}"
PLANNER_NS="${PLANNER_NS:-/planner_runtime_node}"
DRONE_NS="${DRONE_NS:-/drone_1}"
SPLIT_DURATION="${SPLIT_DURATION:-10m}"
COMPRESSION="${COMPRESSION:-lz4}"
BUFFER_MB="${BUFFER_MB:-512}"

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

case "${COMPRESSION}" in
  lz4|bz2|none) ;;
  *) echo "COMPRESSION must be lz4, bz2 or none" >&2; exit 1 ;;
esac
if [[ ! "${BUFFER_MB}" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUFFER_MB must be a positive integer" >&2
  exit 1
fi

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
add_topic "${TARGET_ODOM_TOPIC:-/target_ekf_node/target_odom}"
add_topic /target_ekf_node/yolo_odom
add_topic "${BBOX_TOPIC:-/tracking/bboxes}"
add_topic /tracking/status
add_topic /tracking/target_valid
add_topic /tracking/observation_age

# Keep explicit subscriptions even if detectors have not started yet.
# CameraInfo and original sensor-clock odometry are needed for ranging replay.
add_topic "${RGB_INFO_TOPIC:-/camera0/color/info}"
add_topic "${DEPTH_INFO_TOPIC:-/camera0/depth/info}"
if [[ "${RECORD_IMAGES:-1}" == "1" ]]; then
  add_topic "${RGB_TOPIC:-/camera0/color/image/compressed}"
  add_topic "${DEPTH_TOPIC:-/camera0/depth/image/compressed}"
  add_topic /tracking/detections/image
fi
if [[ "${RECORD_DEPTH_POINTS:-0}" == "1" ]]; then
  add_topic /camera0/depth/points
fi

# Supervisor lifecycle and both sides of the command gateway.
for topic in /planner/status /planner/mode_request /planner/mode_request_text \
             /planner/target_exploration/status /planning/navigation_task_mode \
             /planning/navigation/status /planning/navigation/command \
             /planning/navigation/pos_cmd /planning/exploration/status \
             /planning/exploration/command /planning/exploration/command_enabled \
             /planning/exploration/pos_cmd /planning/heartbeat /planning/static \
             /planning/state /planning/speed /planning/click_goal /planning/click_goal_3d \
             /goal /goal_3d; do
  add_topic "${topic}"
done

# Perching/transition inputs are useful when the same tracking stack is used in
# tracking-perching modes.
add_topic /planning/task_mode
add_topic /perching/surface_odom
add_topic /perching/surface_markers

# Robot state and planner outputs, with both namespaced and non-namespaced forms.
add_topic /unity_odom
add_topic /odom
add_topic /drone_0_visual_slam/odom
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

# The cloud consumed by the planner is on by default, independently of raw LiDAR.
if [[ "${RECORD_CLOUD:-1}" == "1" ]]; then
  add_topic /cloud_registered
fi

# Controller and execution diagnostics.
add_topic /debugPx4ctrl
add_topic /mavros/state
add_topic /mavros/imu/data
add_topic /px4ctrl/takeoff_land

# Planner visual/debug outputs.
add_topic "${PLANNER_NS}/fsm/path"
add_topic "${PLANNER_NS}/visualization/goal"
add_topic "${PLANNER_NS}/visualization/receding_traj"
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
  add_topic /planner/global_topology
  add_topic /planner/world/topology_expansion_points
  add_topic "${PLANNER_NS}/good_obs"
  add_topic "${PLANNER_NS}/bad_obs"
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
  read -r -a extra_topics <<< "${EXTRA_TOPICS}"
  for topic in "${extra_topics[@]}"; do
    add_topic "${topic}"
  done
fi

record_args=(--tcpnodelay --buffsize="${BUFFER_MB}" -O "${BAG_DIR}/${BAG_PREFIX}")
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

# DRY_RUN prints the subscription list without starting ROS recording or writing files.
if [[ "${DRY_RUN:-0}" == "1" ]]; then
  exit 0
fi

mkdir -p "${BAG_DIR}"
METADATA_DIR="${BAG_DIR}/${BAG_PREFIX}_metadata"
# Refuse to overwrite a previous recording or its provenance.
if [[ -e "${METADATA_DIR}" ]] || compgen -G "${BAG_DIR}/${BAG_PREFIX}*.bag*" >/dev/null; then
  echo "[record_tracking] output prefix already exists; choose another BAG_PREFIX" >&2
  exit 1
fi
mkdir "${METADATA_DIR}"
printf '%s\n' "${topics[@]}" > "${METADATA_DIR}/requested_topics.txt"
{
  date -u '+start_utc=%Y-%m-%dT%H:%M:%SZ'
  printf 'ROS_MASTER_URI=%s\nROS_DISTRO=%s\n' "${ROS_MASTER_URI:-}" "${ROS_DISTRO:-}"
  printf 'planner_ns=%s\ncompression=%s\nbuffer_mb=%s\n' "${PLANNER_NS}" "${COMPRESSION}" "${BUFFER_MB}"
  printf 'command='
  printf '%q ' rosbag record "${record_args[@]}" "${topics[@]}"
  printf '\n'
} > "${METADATA_DIR}/recording.txt"

snapshot() {
  local output="$1"
  shift
  if ! timeout 10 "$@" > "${METADATA_DIR}/${output}" 2> "${METADATA_DIR}/${output}.stderr"; then
    echo "[record_tracking] snapshot incomplete: ${output}; see metadata stderr" >&2
  fi
}
snapshot git_head.txt git -C "${REPO_ROOT}" rev-parse HEAD
snapshot git_status.txt git -C "${REPO_ROOT}" status --short --branch
# Include local source changes, but not large generated runtime logs or release.
snapshot source_changes.patch git -C "${REPO_ROOT}" diff HEAD -- sh_files src/Planner src/Perceptor \
  ':!src/Planner/general_planner/log'
snapshot parameters_at_start.yaml rosparam dump /dev/stdout
snapshot topics_at_start.txt rostopic list -v
snapshot nodes_at_start.txt rosnode list
cp "${BASH_SOURCE[0]}" "${METADATA_DIR}/record_tracking.sh"
# C++ planner configs may be loaded directly from YAML rather than the parameter server.
for relative in src/Planner/general_planner/config src/Planner/task_planner/launch \
                src/Perceptor/tracking_detector/config src/Perceptor/tracking_detector/launch; do
  if [[ -d "${REPO_ROOT}/${relative}" ]]; then
    mkdir -p "${METADATA_DIR}/source/${relative}"
    cp -a "${REPO_ROOT}/${relative}/." "${METADATA_DIR}/source/${relative}/"
  fi
done
BRIDGE_ROOT="${REPO_ROOT}/../unity_planner_bridge"
if [[ -d "${BRIDGE_ROOT}" ]]; then
  mkdir -p "${METADATA_DIR}/unity_planner_bridge"
  for relative in launch scripts/unity_cmd_odom_bridge.py; do
    if [[ -e "${BRIDGE_ROOT}/${relative}" ]]; then
      cp -a "${BRIDGE_ROOT}/${relative}" "${METADATA_DIR}/unity_planner_bridge/"
    fi
  done
fi
echo "[record_tracking] metadata: ${METADATA_DIR}"
echo "[record_tracking] stop with Ctrl+C; keep all split bags and the metadata directory."
exec rosbag record "${record_args[@]}" "${topics[@]}"
