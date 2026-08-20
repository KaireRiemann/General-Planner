#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Build and refresh the binary-only general_planner_release package.

Default usage from the host:
  sh_files/build_general_planner_release.sh

The script defaults to running the build inside the running ros1_noetic Docker
container. To run directly in the current shell instead:
  GP_USE_DOCKER=0 sh_files/build_general_planner_release.sh

Environment overrides:
  GP_DOCKER_CONTAINER   Docker container name. Default: ros1_noetic
  GP_CONTAINER_REPO     Repo path inside Docker.
                        Default: /root/ws/real_planner/src/General-Planner
  GP_BUILD_TYPE         CMake build type. Default: Release
  GP_RELEASE_ARCHIVE    Output archive path.
                        Default: <repo>/general_planner_release.tar.gz
  GP_SKIP_TESTS=1       Skip release smoke tests.
  GP_SKIP_ARCHIVE=1     Skip tar.gz archive generation.
  GP_NO_STRIP=1         Do not strip the release binary.
  GP_CATKIN_ARGS        Extra args appended to catkin_make.

What is synced:
  - devel/lib/general_planner/general_planner_runtime_node
  - devel/lib/general_planner/planner_runtime_node
  - devel/lib/general_planner/exploration_node
  - devel/lib/general_planner/highspeed_traj_server
  - M2 planner_runtime launch + mode RViz configs
  - M2 state2state and global-topology configuration
  - general_planner PlannerStatus/PlannerModeRequest/TopologyExpansionPoint msgs
  - devel/lib/liblkh_tsp_solver.so
  - General Planner 3D Nav Goal RViz plugin
  - General Planner garage exploration YAML/RViz configs
  - src/Utils/quadrotor_msgs/msg
  - traj_utils message interfaces used by exploration
  - devel/include/quadrotor_msgs
  - devel/lib/python3/dist-packages/quadrotor_msgs
  - general_planner_release.tar.gz unless GP_SKIP_ARCHIVE=1
EOF
}

for arg in "$@"; do
  case "${arg}" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[build_release] Unknown argument: ${arg}" >&2
      usage >&2
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DOCKER_CONTAINER="${GP_DOCKER_CONTAINER:-ros1_noetic}"
CONTAINER_REPO="${GP_CONTAINER_REPO:-/root/ws/real_planner/src/General-Planner}"

inside_docker=0
if [[ -f /.dockerenv || -n "${GP_INSIDE_DOCKER:-}" ]]; then
  inside_docker=1
fi

if [[ "${GP_USE_DOCKER:-1}" != "0" && "${inside_docker}" == "0" ]]; then
  if ! command -v docker >/dev/null 2>&1; then
    echo "[build_release] docker not found. Set GP_USE_DOCKER=0 to build locally." >&2
    exit 1
  fi
  if ! docker inspect -f '{{.State.Running}}' "${DOCKER_CONTAINER}" 2>/dev/null | grep -q true; then
    echo "[build_release] Docker container '${DOCKER_CONTAINER}' is not running." >&2
    exit 1
  fi
  echo "[build_release] Enter Docker container: ${DOCKER_CONTAINER}"
  docker exec \
    -e GP_USE_DOCKER=0 \
    -e GP_INSIDE_DOCKER=1 \
    -e GP_BUILD_TYPE="${GP_BUILD_TYPE:-Release}" \
    -e GP_SKIP_TESTS="${GP_SKIP_TESTS:-0}" \
    -e GP_SKIP_ARCHIVE="${GP_SKIP_ARCHIVE:-0}" \
    -e GP_NO_STRIP="${GP_NO_STRIP:-0}" \
    -e GP_CATKIN_ARGS="${GP_CATKIN_ARGS:-}" \
    -e GP_RELEASE_ARCHIVE="${GP_RELEASE_ARCHIVE:-}" \
    "${DOCKER_CONTAINER}" \
    bash -lc "cd $(printf '%q' "${CONTAINER_REPO}") && bash sh_files/build_general_planner_release.sh"
  exit $?
fi

WORKSPACE_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"
RELEASE_ROOT="${REPO_ROOT}/general_planner_release"
BUILD_TYPE="${GP_BUILD_TYPE:-Release}"
ARCHIVE_PATH="${GP_RELEASE_ARCHIVE:-${REPO_ROOT}/general_planner_release.tar.gz}"

if [[ ! -d "${RELEASE_ROOT}" ]]; then
  echo "[build_release] Release directory not found: ${RELEASE_ROOT}" >&2
  exit 1
fi

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "[build_release] /opt/ros/noetic/setup.bash not found. ROS Noetic is required." >&2
  exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/noetic/setup.bash
set -u

echo "[build_release] Workspace: ${WORKSPACE_ROOT}"
echo "[build_release] Repo: ${REPO_ROOT}"
echo "[build_release] Release: ${RELEASE_ROOT}"
echo "[build_release] Build type: ${BUILD_TYPE}"

cd "${WORKSPACE_ROOT}"
catkin_cmd=(
  catkin_make
  --force-cmake
  "-DCATKIN_WHITELIST_PACKAGES=general_planner;general_planner_rviz_plugins"
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
)
if [[ -n "${GP_CATKIN_ARGS:-}" ]]; then
  # shellcheck disable=SC2206
  extra_args=(${GP_CATKIN_ARGS})
  catkin_cmd+=("${extra_args[@]}")
fi

echo "[build_release] Build command: ${catkin_cmd[*]}"
"${catkin_cmd[@]}"

RUNTIME_BINARY_SRC="${WORKSPACE_ROOT}/devel/lib/general_planner/general_planner_runtime_node"
RUNTIME_BINARY_DST="${RELEASE_ROOT}/src/general_planner_release/bin/general_planner_runtime_node.bin"
PLANNER_RUNTIME_BINARY_SRC="${WORKSPACE_ROOT}/devel/lib/general_planner/planner_runtime_node"
PLANNER_RUNTIME_BINARY_DST="${RELEASE_ROOT}/src/general_planner_release/bin/planner_runtime_node.bin"
EXPLORATION_BINARY_SRC="${WORKSPACE_ROOT}/devel/lib/general_planner/exploration_node"
EXPLORATION_BINARY_DST="${RELEASE_ROOT}/src/general_planner_release/bin/exploration_node.bin"
TRAJ_SERVER_BINARY_SRC="${WORKSPACE_ROOT}/devel/lib/general_planner/highspeed_traj_server"
TRAJ_SERVER_BINARY_DST="${RELEASE_ROOT}/src/general_planner_release/bin/highspeed_traj_server.bin"
LKH_LIBRARY_SRC="${WORKSPACE_ROOT}/devel/lib/liblkh_tsp_solver.so"
LKH_LIBRARY_DST="${RELEASE_ROOT}/lib/liblkh_tsp_solver.so"
RVIZ_PLUGIN_LIBRARY_SRC="${WORKSPACE_ROOT}/devel/lib/libgeneral_planner_rviz_plugins.so"
RVIZ_PLUGIN_SRC="${REPO_ROOT}/src/Utils/general_planner_rviz_plugins"
RVIZ_PLUGIN_DST="${RELEASE_ROOT}/src/general_planner_rviz_plugins"
RVIZ_PLUGIN_LIBRARY_DST="${RVIZ_PLUGIN_DST}/lib/libgeneral_planner_rviz_plugins.so"
LEGACY_RVIZ_PLUGIN_LIBRARY_DST="${RELEASE_ROOT}/lib/libgeneral_planner_rviz_plugins.so"

PLANNER_CONFIG_DIR="${REPO_ROOT}/src/Planner/general_planner/config"
PLANNER_RVIZ_DIR="${REPO_ROOT}/src/Planner/general_planner/rviz"
PLANNER_SCRIPTS_DIR="${REPO_ROOT}/src/Planner/general_planner/scripts"
PLANNER_MSG_DIR="${REPO_ROOT}/src/Planner/general_planner/msg"
RELEASE_PKG_DIR="${RELEASE_ROOT}/src/general_planner_release"
RELEASE_CONFIG_DIR="${RELEASE_PKG_DIR}/config"
EXPLORATION_CONFIG_SRC="${PLANNER_CONFIG_DIR}/exploration.yaml"
EXPLORATION_HOUSE_CONFIG_SRC="${PLANNER_CONFIG_DIR}/exploration_house.yaml"
EXPLORATION_SIM_CONFIG_SRC="${PLANNER_CONFIG_DIR}/exploration_sim.yaml"
EXPLORATION_ROG_MAP_CONFIG_SRC="${PLANNER_CONFIG_DIR}/exploration_rog_map.yaml"
M2_STATE2STATE_CONFIG_SRC="${PLANNER_CONFIG_DIR}/task_planner_runtime_state2state.yaml"
GLOBAL_TOPOLOGY_CONFIG_SRC="${PLANNER_CONFIG_DIR}/global_topology.yaml"
M2_CONVEX_HULL_CONFIG_SRC="${PLANNER_CONFIG_DIR}/traj_opt/convex_hull/click_real_highspeed.yaml"
EXPLORATION_RVIZ_CONFIG_SRC="${PLANNER_CONFIG_DIR}/exploration/highspeed/traj.rviz"
RUNTIME_EXPLORATION_RVIZ_SRC="${PLANNER_RVIZ_DIR}/planner_runtime_exploration.rviz"
RUNTIME_STATE2STATE_RVIZ_SRC="${PLANNER_RVIZ_DIR}/planner_runtime_state2state.rviz"
SERIAL_HANDOVER_SCRIPT_SRC="${PLANNER_SCRIPTS_DIR}/planner_serial_handover.py"
RVIZ_SWITCHER_SCRIPT_SRC="${PLANNER_SCRIPTS_DIR}/planner_rviz_switcher.py"
GP_MSG_PKG_DST="${RELEASE_ROOT}/src/general_planner"
GP_CPP_MSG_SRC="${WORKSPACE_ROOT}/devel/include/general_planner"
GP_CPP_MSG_DST="${RELEASE_ROOT}/include/general_planner"
GP_PY_MSG_SRC="${WORKSPACE_ROOT}/devel/lib/python3/dist-packages/general_planner"
GP_PY_MSG_DST="${RELEASE_ROOT}/lib/python3/dist-packages/general_planner"

MSG_SRC="${REPO_ROOT}/src/Utils/quadrotor_msgs/msg"
MSG_DST="${RELEASE_ROOT}/src/quadrotor_msgs/msg"
CPP_MSG_SRC="${WORKSPACE_ROOT}/devel/include/quadrotor_msgs"
CPP_MSG_DST="${RELEASE_ROOT}/include/quadrotor_msgs"
PY_MSG_SRC="${WORKSPACE_ROOT}/devel/lib/python3/dist-packages/quadrotor_msgs"
PY_MSG_DST="${RELEASE_ROOT}/lib/python3/dist-packages/quadrotor_msgs"

TRAJ_MSG_SRC="${REPO_ROOT}/src/Controller/mpc/ommpc_controller/traj_utils/msg"
TRAJ_MSG_DST="${RELEASE_ROOT}/src/traj_utils/msg"
TRAJ_CPP_MSG_SRC="${WORKSPACE_ROOT}/devel/include/traj_utils"
TRAJ_CPP_MSG_DST="${RELEASE_ROOT}/include/traj_utils"
TRAJ_PY_MSG_SRC="${WORKSPACE_ROOT}/devel/lib/python3/dist-packages/traj_utils"
TRAJ_PY_MSG_DST="${RELEASE_ROOT}/lib/python3/dist-packages/traj_utils"

for required in \
  "${RUNTIME_BINARY_SRC}" \
  "${PLANNER_RUNTIME_BINARY_SRC}" \
  "${EXPLORATION_BINARY_SRC}" \
  "${TRAJ_SERVER_BINARY_SRC}" \
  "${LKH_LIBRARY_SRC}" \
  "${RVIZ_PLUGIN_LIBRARY_SRC}" \
  "${RVIZ_PLUGIN_SRC}/package.xml" \
  "${RVIZ_PLUGIN_SRC}/plugin_description.xml" \
  "${RVIZ_PLUGIN_SRC}/LICENSE" \
  "${EXPLORATION_CONFIG_SRC}" \
  "${EXPLORATION_HOUSE_CONFIG_SRC}" \
  "${EXPLORATION_SIM_CONFIG_SRC}" \
  "${EXPLORATION_ROG_MAP_CONFIG_SRC}" \
  "${M2_STATE2STATE_CONFIG_SRC}" \
  "${GLOBAL_TOPOLOGY_CONFIG_SRC}" \
  "${M2_CONVEX_HULL_CONFIG_SRC}" \
  "${EXPLORATION_RVIZ_CONFIG_SRC}" \
  "${RUNTIME_EXPLORATION_RVIZ_SRC}" \
  "${RUNTIME_STATE2STATE_RVIZ_SRC}" \
  "${SERIAL_HANDOVER_SCRIPT_SRC}" \
  "${RVIZ_SWITCHER_SCRIPT_SRC}" \
  "${PLANNER_MSG_DIR}/PlannerStatus.msg" \
  "${PLANNER_MSG_DIR}/PlannerModeRequest.msg" \
  "${PLANNER_MSG_DIR}/TopologyExpansionPoint.msg" \
  "${PLANNER_MSG_DIR}/TopologyExpansionPointArray.msg" \
  "${GP_CPP_MSG_SRC}" \
  "${GP_PY_MSG_SRC}" \
  "${RELEASE_PKG_DIR}/launch/planner_runtime.launch" \
  "${RELEASE_PKG_DIR}/planner_runtime_node" \
  "${MSG_SRC}" \
  "${CPP_MSG_SRC}" \
  "${PY_MSG_SRC}" \
  "${TRAJ_MSG_SRC}" \
  "${TRAJ_CPP_MSG_SRC}" \
  "${TRAJ_PY_MSG_SRC}"; do
  if [[ ! -e "${required}" ]]; then
    echo "[build_release] Required build artifact missing: ${required}" >&2
    exit 1
  fi
done

sync_binary() {
  local src="$1"
  local dst="$2"
  mkdir -p "$(dirname "${dst}")"
  cp "${src}" "${dst}"
  chmod 755 "${dst}"
  if [[ "${GP_NO_STRIP:-0}" != "1" ]] && command -v strip >/dev/null 2>&1; then
    strip "${dst}"
  fi
}

echo "[build_release] Sync planner runtime binaries"
sync_binary "${RUNTIME_BINARY_SRC}" "${RUNTIME_BINARY_DST}"
sync_binary "${PLANNER_RUNTIME_BINARY_SRC}" "${PLANNER_RUNTIME_BINARY_DST}"
sync_binary "${EXPLORATION_BINARY_SRC}" "${EXPLORATION_BINARY_DST}"
sync_binary "${TRAJ_SERVER_BINARY_SRC}" "${TRAJ_SERVER_BINARY_DST}"

echo "[build_release] Sync planner_runtime helper scripts"
cp "${SERIAL_HANDOVER_SCRIPT_SRC}" "${RELEASE_PKG_DIR}/planner_serial_handover.py"
cp "${RVIZ_SWITCHER_SCRIPT_SRC}" "${RELEASE_PKG_DIR}/planner_rviz_switcher.py"
chmod +x \
  "${RELEASE_PKG_DIR}/planner_runtime_node" \
  "${RELEASE_PKG_DIR}/planner_serial_handover.py" \
  "${RELEASE_PKG_DIR}/planner_rviz_switcher.py"

echo "[build_release] Sync exploration runtime library"
mkdir -p "$(dirname "${LKH_LIBRARY_DST}")"
cp "${LKH_LIBRARY_SRC}" "${LKH_LIBRARY_DST}"
chmod 755 "${LKH_LIBRARY_DST}"
if [[ "${GP_NO_STRIP:-0}" != "1" ]] && command -v strip >/dev/null 2>&1; then
  strip "${LKH_LIBRARY_DST}"
fi

echo "[build_release] Sync 3D Nav Goal RViz plugin"
rm -f "${LEGACY_RVIZ_PLUGIN_LIBRARY_DST}"
mkdir -p "$(dirname "${RVIZ_PLUGIN_LIBRARY_DST}")"
cp "${RVIZ_PLUGIN_LIBRARY_SRC}" "${RVIZ_PLUGIN_LIBRARY_DST}"
chmod 755 "${RVIZ_PLUGIN_LIBRARY_DST}"
if [[ "${GP_NO_STRIP:-0}" != "1" ]] && command -v strip >/dev/null 2>&1; then
  strip "${RVIZ_PLUGIN_LIBRARY_DST}"
fi
mkdir -p "${RVIZ_PLUGIN_DST}"
cp "${RVIZ_PLUGIN_SRC}/package.xml" "${RVIZ_PLUGIN_DST}/package.xml"
cp "${RVIZ_PLUGIN_SRC}/plugin_description.xml" \
  "${RVIZ_PLUGIN_DST}/plugin_description.xml"
cp "${RVIZ_PLUGIN_SRC}/LICENSE" "${RVIZ_PLUGIN_DST}/LICENSE"

echo "[build_release] Sync garage exploration configs"
mkdir -p "${RELEASE_CONFIG_DIR}"
cp "${EXPLORATION_CONFIG_SRC}" "${RELEASE_CONFIG_DIR}/exploration.yaml"
cp "${EXPLORATION_HOUSE_CONFIG_SRC}" "${RELEASE_CONFIG_DIR}/exploration_house.yaml"
cp "${EXPLORATION_SIM_CONFIG_SRC}" "${RELEASE_CONFIG_DIR}/exploration_sim.yaml"
cp "${EXPLORATION_ROG_MAP_CONFIG_SRC}" "${RELEASE_CONFIG_DIR}/exploration_rog_map.yaml"
cp "${M2_STATE2STATE_CONFIG_SRC}" \
  "${RELEASE_CONFIG_DIR}/task_planner_runtime_state2state.yaml"
cp "${GLOBAL_TOPOLOGY_CONFIG_SRC}" \
  "${RELEASE_CONFIG_DIR}/global_topology.yaml"
mkdir -p "${RELEASE_CONFIG_DIR}/traj_opt/convex_hull"
cp "${M2_CONVEX_HULL_CONFIG_SRC}" \
  "${RELEASE_CONFIG_DIR}/traj_opt/convex_hull/click_real_highspeed.yaml"
cp "${EXPLORATION_RVIZ_CONFIG_SRC}" "${RELEASE_CONFIG_DIR}/exploration.rviz"
cp "${RUNTIME_EXPLORATION_RVIZ_SRC}" \
  "${RELEASE_CONFIG_DIR}/planner_runtime_exploration.rviz"
cp "${RUNTIME_STATE2STATE_RVIZ_SRC}" \
  "${RELEASE_CONFIG_DIR}/planner_runtime_state2state.rviz"

echo "[build_release] Sync general_planner runtime messages"
rm -rf "${GP_MSG_PKG_DST}/msg"
mkdir -p "${GP_MSG_PKG_DST}/msg"
cp "${PLANNER_MSG_DIR}/PlannerStatus.msg" "${GP_MSG_PKG_DST}/msg/PlannerStatus.msg"
cp "${PLANNER_MSG_DIR}/PlannerModeRequest.msg" "${GP_MSG_PKG_DST}/msg/PlannerModeRequest.msg"
cp "${PLANNER_MSG_DIR}/TopologyExpansionPoint.msg" \
  "${GP_MSG_PKG_DST}/msg/TopologyExpansionPoint.msg"
cp "${PLANNER_MSG_DIR}/TopologyExpansionPointArray.msg" \
  "${GP_MSG_PKG_DST}/msg/TopologyExpansionPointArray.msg"
cat >"${GP_MSG_PKG_DST}/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>general_planner</name>
  <version>0.1.0</version>
  <description>Message interfaces required by planner_runtime in the binary release.</description>
  <maintainer email="release@example.com">general_planner_release</maintainer>
  <license>Interface definitions only.</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
</package>
EOF
rm -rf "${GP_CPP_MSG_DST}"
mkdir -p "$(dirname "${GP_CPP_MSG_DST}")"
cp -a "${GP_CPP_MSG_SRC}" "${GP_CPP_MSG_DST}"
rm -rf "${GP_PY_MSG_DST}"
mkdir -p "$(dirname "${GP_PY_MSG_DST}")"
cp -a "${GP_PY_MSG_SRC}" "${GP_PY_MSG_DST}"

echo "[build_release] Sync quadrotor_msgs source definitions"
rm -rf "${MSG_DST}"
mkdir -p "${MSG_DST}"
cp -a "${MSG_SRC}/." "${MSG_DST}/"

echo "[build_release] Sync generated C++ message headers"
rm -rf "${CPP_MSG_DST}"
mkdir -p "$(dirname "${CPP_MSG_DST}")"
cp -a "${CPP_MSG_SRC}" "${CPP_MSG_DST}"

echo "[build_release] Sync generated Python message package"
rm -rf "${PY_MSG_DST}"
mkdir -p "$(dirname "${PY_MSG_DST}")"
cp -a "${PY_MSG_SRC}" "${PY_MSG_DST}"

echo "[build_release] Sync traj_utils message interfaces"
rm -rf "${TRAJ_MSG_DST}"
mkdir -p "${TRAJ_MSG_DST}"
cp -a "${TRAJ_MSG_SRC}/." "${TRAJ_MSG_DST}/"

rm -rf "${TRAJ_CPP_MSG_DST}"
mkdir -p "$(dirname "${TRAJ_CPP_MSG_DST}")"
cp -a "${TRAJ_CPP_MSG_SRC}" "${TRAJ_CPP_MSG_DST}"

rm -rf "${TRAJ_PY_MSG_DST}"
mkdir -p "$(dirname "${TRAJ_PY_MSG_DST}")"
cp -a "${TRAJ_PY_MSG_SRC}" "${TRAJ_PY_MSG_DST}"

chmod +x \
  "${RELEASE_ROOT}/src/general_planner_release/general_planner_runtime_node" \
  "${RELEASE_ROOT}/src/general_planner_release/planner_runtime_node" \
  "${RELEASE_ROOT}/src/general_planner_release/exploration_node" \
  "${RELEASE_ROOT}/src/general_planner_release/highspeed_traj_server" \
  "${RELEASE_ROOT}/src/general_planner_release/planner_serial_handover.py" \
  "${RELEASE_ROOT}/src/general_planner_release/planner_rviz_switcher.py"

run_smoke_tests() {
  echo "[build_release] Run release smoke tests"

  # shellcheck disable=SC1091
  set +u
  source "${WORKSPACE_ROOT}/devel/setup.bash"
  # shellcheck disable=SC1091
  source "${RELEASE_ROOT}/setup.bash"
  set -u

  rospack find general_planner_release >/dev/null
  rospack find general_planner_rviz_plugins >/dev/null
  rospack find quadrotor_msgs >/dev/null
  rospack find traj_utils >/dev/null
  rospack plugins --attrib=plugin rviz |
    grep -q '^general_planner_rviz_plugins '

  python3 - <<'PY'
import quadrotor_msgs.msg
import traj_utils.msg
print("quadrotor_msgs python import ok")
print("traj_utils python import ok")
PY

  roslaunch --files general_planner_release tracking.launch >/dev/null
  for backend in corridor esdf plain; do
    roslaunch --files general_planner_release state2state.launch planner_backend:="${backend}" >/dev/null
  done
  roslaunch --files general_planner_release exploration.launch >/dev/null
  roslaunch --files general_planner_release exploration_sim.launch rviz:=false >/dev/null
  roslaunch --files general_planner_release planner_runtime.launch \
    marsim:=false rviz:=false >/dev/null
  roslaunch --files general_planner_release planner_runtime_sim.launch \
    rviz:=false >/dev/null
  rospack find general_planner >/dev/null
  python3 - <<'PY'
from general_planner.msg import (
    PlannerStatus,
    PlannerModeRequest,
    TopologyExpansionPoint,
    TopologyExpansionPointArray,
)
print("general_planner runtime msgs import ok")
PY

  for binary in \
    "${RUNTIME_BINARY_DST}" \
    "${PLANNER_RUNTIME_BINARY_DST}" \
    "${EXPLORATION_BINARY_DST}" \
    "${TRAJ_SERVER_BINARY_DST}" \
    "${RVIZ_PLUGIN_LIBRARY_DST}"; do
    if ldd "${binary}" | grep -q 'not found'; then
      echo "[build_release] Unresolved runtime dependency in ${binary}:" >&2
      ldd "${binary}" | grep 'not found' >&2 || true
      exit 1
    fi
  done

  local roscore_log="/tmp/general_planner_release_build_roscore.log"
  roscore >"${roscore_log}" 2>&1 &
  local roscore_pid=$!
  trap 'kill ${roscore_pid} >/dev/null 2>&1 || true' RETURN
  sleep 2

  local cfg_dir="${RELEASE_ROOT}/src/general_planner_release/config"
  local cases=(
    "tracking:${cfg_dir}/interface.yaml"
    "corridor:${cfg_dir}/interface_state2state_corridor.yaml"
    "esdf:${cfg_dir}/interface_state2state_esdf.yaml"
    "plain:${cfg_dir}/interface_state2state_plain.yaml"
  )

  for entry in "${cases[@]}"; do
    local name="${entry%%:*}"
    local cfg="${entry#*:}"
    local out="/tmp/general_planner_release_${name}.yaml"
    local log="/tmp/general_planner_release_${name}.log"
    rm -f "${out}" "${log}"
    timeout 6s rosrun general_planner_release general_planner_runtime_node \
      _interface_config_path:="${cfg}" \
      _generated_config_path:="${out}" >"${log}" 2>&1 || true
    if [[ ! -s "${out}" ]]; then
      echo "[build_release] Failed to generate runtime YAML for ${name}" >&2
      cat "${log}" >&2 || true
      exit 1
    fi
  done

  local exploration_log="/tmp/general_planner_release_exploration.log"
  rm -f "${exploration_log}"
  timeout --signal=INT --kill-after=2s 6s \
    roslaunch general_planner_release exploration.launch \
      auto_start:=false >"${exploration_log}" 2>&1 || true
  if grep -Eqi 'process has died|error while loading shared libraries|No such file|cannot open shared object' \
      "${exploration_log}"; then
    echo "[build_release] Exploration release launch failed" >&2
    cat "${exploration_log}" >&2
    exit 1
  fi

  python3 - <<'PY'
import pathlib
import yaml

checks = {
    "tracking": ("tracking", None, 15.0, "/drone_1/planning/pos_cmd", "/drone_1/lidar_slam/odom"),
    "corridor": ("state2state", (True, False, False), 5.0, "/planning/pos_cmd", "/lidar_slam/odom"),
    "esdf": ("state2state", (False, True, False), 10.0, "/planning/pos_cmd", "/lidar_slam/odom"),
    "plain": ("state2state", (False, False, True), 5.0, "/planning/pos_cmd", "/lidar_slam/odom"),
}

for name, (mode, switches, replan_rate, cmd_topic, odom_topic) in checks.items():
    path = pathlib.Path(f"/tmp/general_planner_release_{name}.yaml")
    data = yaml.safe_load(path.read_text())
    for section in ("fsm", "general_planner", "traj_opt", "rog_map"):
        assert section in data and isinstance(data[section], dict), (name, section)
    assert data["fsm"]["task_mode"] == mode, (name, data["fsm"]["task_mode"], mode)
    assert data["fsm"]["replan_rate"] == replan_rate, (name, data["fsm"]["replan_rate"], replan_rate)
    assert data["fsm"]["cmd_topic"] == cmd_topic, (name, data["fsm"]["cmd_topic"], cmd_topic)
    assert data["general_planner"]["yaw_dot_max"] == 3.0, (name, data["general_planner"]["yaw_dot_max"])
    assert data["rog_map"]["ros_callback"]["odom_topic"] == odom_topic, (
        name, data["rog_map"]["ros_callback"]["odom_topic"], odom_topic)
    if switches is not None:
        gp = data["general_planner"]
        actual = (
            bool(gp.get("backup_traj_en", False)),
            bool(gp.get("esdf_traj_en", False)),
            bool(gp.get("plain_traj_en", False)),
        )
        assert actual == switches, (name, actual, switches)
    if name == "tracking":
        assert data["general_planner"]["tracking"]["fov_check_strict"] is False
        assert data["general_planner"]["tracking"]["use_snap"] is False
        assert data["rog_map"]["load_pcd_en"] is False

print("runtime config generation ok")
PY

  # Launch the composed M2 release for real.  `roslaunch --files` above only
  # validates XML; this catches a missing M2 runtime parameter before an
  # archive is produced.  Start in state2state so /goal must be owned by the
  # persistent planner_runtime_node (never by serial handover).
  local m2_runtime_log="/tmp/general_planner_release_m2_runtime.log"
  local m2_status="/tmp/general_planner_release_m2_status.yaml"
  local m2_runtime_pid=0
  rm -f "${m2_runtime_log}" "${m2_status}"
  roslaunch general_planner_release planner_runtime_sim.launch \
    initial_mode:=state2state rviz:=false >"${m2_runtime_log}" 2>&1 &
  m2_runtime_pid=$!
  trap 'kill ${m2_runtime_pid} >/dev/null 2>&1 || true; kill ${roscore_pid} >/dev/null 2>&1 || true' RETURN

  local m2_ready=0
  for _ in $(seq 1 24); do
    if timeout 2s rostopic echo -n 1 /planner/status >"${m2_status}" 2>/dev/null; then
      m2_ready=1
      break
    fi
    if ! kill -0 "${m2_runtime_pid}" >/dev/null 2>&1; then
      break
    fi
    sleep 0.5
  done
  if [[ "${m2_ready}" != "1" ]]; then
    echo "[build_release] M2 planner_runtime did not publish /planner/status" >&2
    cat "${m2_runtime_log}" >&2 || true
    exit 1
  fi
  grep -q 'active_mode_str: "state2state"' "${m2_status}"
  grep -q 'mode_state_str: "s2s_wait_goal"' "${m2_status}"
  if ! rostopic info /goal | grep -q '/planner_runtime_node'; then
    echo "[build_release] M2 runtime does not subscribe to /goal" >&2
    rostopic info /goal >&2 || true
    cat "${m2_runtime_log}" >&2 || true
    exit 1
  fi
  rostopic pub -1 /goal geometry_msgs/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: -12.57, y: -13.32, z: 1.5}, orientation: {w: 1.0}}}" \
    >/dev/null
  local goal_accepted=0
  for _ in $(seq 1 20); do
    if grep -q '\[planner_supervisor\] navigation goal accepted' "${m2_runtime_log}"; then
      goal_accepted=1
      break
    fi
    sleep 0.25
  done
  if [[ "${goal_accepted}" != "1" ]]; then
    echo "[build_release] M2 runtime did not accept the /goal test target" >&2
    cat "${m2_runtime_log}" >&2 || true
    exit 1
  fi

  kill "${m2_runtime_pid}" >/dev/null 2>&1 || true
  wait "${m2_runtime_pid}" 2>/dev/null || true
  trap 'kill ${roscore_pid} >/dev/null 2>&1 || true' RETURN

  kill "${roscore_pid}" >/dev/null 2>&1 || true
  trap - RETURN
}

if [[ "${GP_SKIP_TESTS:-0}" != "1" ]]; then
  run_smoke_tests
else
  echo "[build_release] Skip smoke tests"
fi

if [[ "${GP_SKIP_ARCHIVE:-0}" != "1" ]]; then
  echo "[build_release] Create archive: ${ARCHIVE_PATH}"
  mkdir -p "$(dirname "${ARCHIVE_PATH}")"
  tar -C "${REPO_ROOT}" -czf "${ARCHIVE_PATH}" general_planner_release
  ls -lh "${ARCHIVE_PATH}"
else
  echo "[build_release] Skip archive generation"
fi

echo "[build_release] Done"
