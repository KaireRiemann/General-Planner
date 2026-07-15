# general_planner_release

Binary-only ROS1 Noetic deployment package for General Planner tracking,
state2state and HighSpeedExp-based exploration.

## Contents

- `src/general_planner_release/general_planner_runtime_node`: ROS wrapper entry.
- `src/general_planner_release/bin/general_planner_runtime_node.bin`: planner runtime binary.
- `src/general_planner_release/exploration_node` and
  `src/general_planner_release/highspeed_traj_server`: exploration wrapper entries.
- `src/general_planner_release/bin/exploration_node.bin` and
  `src/general_planner_release/bin/highspeed_traj_server.bin`: exploration binaries.
- `lib/liblkh_tsp_solver.so`: bundled global-tour solver used by exploration.
- `src/general_planner_release/config/interface.yaml`: full tracking runtime config.
- `src/general_planner_release/config/interface_state2state_*.yaml`: full state2state runtime configs.
- `src/general_planner_release/config/exploration.yaml`: delivered garage exploration profile.
- `src/general_planner_release/config/exploration_sim.yaml`: garage simulator and LiDAR profile.
- `src/general_planner_release/config/exploration_rog_map.yaml`: exploration map-backend profile.
- `src/general_planner_release/config/exploration.rviz`: exploration visualization profile.
- `src/general_planner_release/launch/tracking.launch`: tracking launch entry.
- `src/general_planner_release/launch/state2state.launch`: state2state launch entry.
- `src/general_planner_release/launch/state2state_sim.launch`: optional development launch that also starts `perfect_drone_sim`.
- `src/general_planner_release/launch/exploration.launch`: planner-only exploration entry for real sensors, rosbag or an external simulator.
- `src/general_planner_release/launch/exploration_sim.launch`: garage simulator plus exploration entry.
- `src/quadrotor_msgs/msg`: public message interface definitions.
- `src/traj_utils/msg`: trajectory messages exchanged by the exploration nodes.
- `lib/python3/dist-packages/quadrotor_msgs`: generated Python message package for tools such as `rostopic echo`.

This package does not include planner source code. Runtime parameters are exposed through the full YAML files under `config/`.

## Target Machine Requirements

- Ubuntu 20.04 x86_64
- ROS Noetic
- Runtime libraries used by the binary: `ros-noetic-roscpp`, `ros-noetic-tf2-ros`, `ros-noetic-pcl-ros`, `libyaml-cpp0.6`, `libdw1`, Boost 1.71, PCL 1.10

Install the common runtime dependencies with:

```bash
sudo apt update
sudo apt install ros-noetic-desktop ros-noetic-pcl-ros libyaml-cpp0.6 libdw1
```

## Run

```bash
source /path/to/general_planner_release/setup.bash
roslaunch general_planner_release tracking.launch
```

To use another interface file:

```bash
roslaunch general_planner_release tracking.launch \
  interface_config:=/path/to/interface.yaml
```

Run state2state:

```bash
roslaunch general_planner_release state2state.launch planner_backend:=corridor
roslaunch general_planner_release state2state.launch planner_backend:=esdf
roslaunch general_planner_release state2state.launch planner_backend:=plain
```

For a custom state2state interface file:

```bash
roslaunch general_planner_release state2state.launch \
  interface_config:=/path/to/interface_state2state.yaml
```

In this development workspace, the planner can be tested with the bundled simulator:

```bash
roslaunch general_planner_release state2state_sim.launch planner_backend:=corridor rviz:=false
rostopic pub -1 /goal geometry_msgs/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

Run exploration with real sensors, rosbag or an externally started simulator:

```bash
roslaunch general_planner_release exploration.launch
```

Exploration starts in `WAIT_TRIGGER`. After odometry and point clouds are
available, click RViz `2D Nav Goal` once. The click publishes a
`geometry_msgs/PoseStamped` on `/move_base_simple/goal`; its position is used
only as a start trigger, not as an exploration destination. To use another
topic or restore automatic startup:

```bash
roslaunch general_planner_release exploration.launch \
  trigger_topic:=/your/start_trigger

roslaunch general_planner_release exploration.launch auto_start:=true
```

The delivered garage profile enables persistent long-horizon coverage
guidance. It only orders existing, executable HighSpeedExp frontiers; unknown
coverage anchors are never sent to the trajectory planner. To compare or roll
back behavior without changing the YAML, select one of the four launch modes:

```bash
roslaunch general_planner_release exploration_sim.launch \
  coverage_guidance_mode:=shadow rviz:=true

roslaunch general_planner_release exploration_sim.launch \
  coverage_guidance_mode:=off rviz:=true
```

`shadow` computes and displays the route without changing frontier selection;
`soft` adds a bounded cost bias; `full` (default) uses the coverage order and
guards `FINISH` while reachable unknown coverage remains. A stale or invalid
coverage result automatically falls back to the original frontier objective.
The visualization topic is
`/exploration_node/coverage_guidance/route`.

The default input topics are `/lidar_slam/odom` and `/cloud_registered`.
Override them directly when the real robot uses different names:

```bash
roslaunch general_planner_release exploration.launch \
  odom_topic:=/your/odom \
  cloud_topic:=/your/world_frame_scan
```

The default `cloud_odom_mode:=approximate_sync` pairs messages by
`header.stamp`. For an external simulator with a different timestamp
convention, use the latest locally received odometry instead:

```bash
roslaunch general_planner_release exploration.launch \
  cloud_odom_mode:=latest_odom \
  latest_odom_timeout:=0.5
```

This changes only the internal pairing policy; topic names remain unchanged.
Keep `/use_sim_time` false unless the simulator continuously publishes
`/clock`. The point cloud must still be a current world-frame scan.

Run the garage simulation in the development workspace:

```bash
roslaunch general_planner_release exploration_sim.launch rviz:=true
```

Wait for the garage point cloud to appear, select RViz `2D Nav Goal`, and
click anywhere once to start exploration.

Like `state2state_sim.launch`, `exploration_sim.launch` expects the
`perfect_drone_sim` package and its garage map assets to be installed or
available in the sourced workspace. The simulator itself is not copied into
this binary-only planner archive.

## Runtime Config

The launch files pass a complete runtime YAML into `general_planner_runtime_node`. Edit these files directly to tune planner behavior, map behavior, FSM behavior, trajectory optimization, and topics:

- `config/interface.yaml`: tracking.
- `config/interface_state2state_corridor.yaml`: state2state corridor/backup optimizer.
- `config/interface_state2state_esdf.yaml`: state2state ESDF optimizer.
- `config/interface_state2state_plain.yaml`: state2state plain optimizer.
- `config/exploration.yaml`: exploration frontend, General Planner corridor and trajectory tuning.
- `config/exploration_sim.yaml`: garage map, initial pose and simulated LiDAR contract.
- `config/exploration_rog_map.yaml`: ROG-Map settings used by the exploration frontend.

Tracking topics are configured in the full YAML:

```yaml
fsm:
  task_mode: "tracking"
  tracking_target_odom_topic: "/tracking/target_odom"
  tracking_target_prediction_topic: "/tracking/target_prediction"
  tracking_use_target_prediction_path: true
  cmd_topic: "/drone_1/planning/pos_cmd"
  mpc_cmd_topic: "/drone_1/planning_cmd/poly_traj"

rog_map:
  ros_callback:
    odom_topic: "/drone_1/lidar_slam/odom"
    cloud_topic: "/drone_1/cloud_registered"
```

State2state topics are configured the same way:

```yaml
fsm:
  task_mode: "state2state"
  click_goal_topic: "/goal"
  cmd_topic: "/planning/pos_cmd"
  mpc_cmd_topic: "/planning_cmd/poly_traj"

rog_map:
  ros_callback:
    odom_topic: "/lidar_slam/odom"
    cloud_topic: "/cloud_registered"
```

`state2state.launch planner_backend:=corridor|esdf|plain` selects one of the three complete YAML files above. For a custom file, pass `interface_config:=/path/to/full_runtime.yaml`.

## Refresh This Release

After changing planner C++ code, embedded preset YAML, or `quadrotor_msgs`, rebuild and refresh the release from the source repo:

```bash
cd /home/diffbot/ros1_ws/real_planner/src/General-Planner
sh_files/build_general_planner_release.sh
```

The script runs in the `ros1_noetic` Docker container by default, forces CMake
reconfiguration so embedded presets are updated, copies all three planner
binaries, the LKH runtime library, garage exploration configs and message
interfaces into this release folder, runs smoke tests, and regenerates
`general_planner_release.tar.gz`.

Useful overrides:

```bash
GP_DOCKER_CONTAINER=ros1_noetic sh_files/build_general_planner_release.sh
GP_SKIP_TESTS=1 sh_files/build_general_planner_release.sh
GP_USE_DOCKER=0 sh_files/build_general_planner_release.sh
```

## Required Inputs

```bash
rostopic hz /drone_1/lidar_slam/odom
rostopic hz /drone_1/cloud_registered
rostopic hz /tracking/target_odom
```

If `use_target_prediction_path: true`:

```bash
rostopic hz /tracking/target_prediction
```

`/tracking/target_prediction` must be `nav_msgs/Path`. Samples are interpreted with the tracking prediction settings in `fsm`.

## Outputs

```bash
rostopic echo /drone_1/planning/pos_cmd
rostopic echo /drone_1/planning_cmd/poly_traj
```

For state2state with the default interface files, the required inputs are `/lidar_slam/odom`, `/cloud_registered`, and `/goal`; outputs are `/planning/pos_cmd` and `/planning_cmd/poly_traj`.

For exploration, the required inputs are `/lidar_slam/odom` and a current
world-frame scan on `/cloud_registered`. The exploration nodes exchange
`traj_utils/PolyTraj` on `/planning/trajectory` and
`/planning/yaw_trajectory`; `highspeed_traj_server` publishes the controller
command on `/planning/pos_cmd`.

The controller should consume `quadrotor_msgs/PositionCommand` and, if needed, `quadrotor_msgs/PolynomialTrajectory`.

## Notes

- Do not edit generated runtime YAML under `~/.ros/general_planner_runtime`; edit the full YAML under `config/` or pass your own `interface_config`.
- If target hardware is not Ubuntu 20.04 x86_64 / ROS Noetic, rebuild the binary on the target ABI.
- The bundled `quadrotor_msgs` are public interface definitions, not planner implementation source.
