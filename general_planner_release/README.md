# general_planner_release

Binary-only ROS1 Noetic deployment package for General Planner tracking and state2state.

## Contents

- `src/general_planner_release/general_planner_runtime_node`: ROS wrapper entry.
- `src/general_planner_release/bin/general_planner_runtime_node.bin`: planner runtime binary.
- `src/general_planner_release/config/interface.yaml`: full tracking runtime config.
- `src/general_planner_release/config/interface_state2state_*.yaml`: full state2state runtime configs.
- `src/general_planner_release/launch/tracking.launch`: tracking launch entry.
- `src/general_planner_release/launch/state2state.launch`: state2state launch entry.
- `src/general_planner_release/launch/state2state_sim.launch`: optional development launch that also starts `perfect_drone_sim`.
- `src/quadrotor_msgs/msg`: public message interface definitions.
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

## Runtime Config

The launch files pass a complete runtime YAML into `general_planner_runtime_node`. Edit these files directly to tune planner behavior, map behavior, FSM behavior, trajectory optimization, and topics:

- `config/interface.yaml`: tracking.
- `config/interface_state2state_corridor.yaml`: state2state corridor/backup optimizer.
- `config/interface_state2state_esdf.yaml`: state2state ESDF optimizer.
- `config/interface_state2state_plain.yaml`: state2state plain optimizer.

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

The script runs in the `ros1_noetic` Docker container by default, forces CMake reconfiguration so embedded presets are updated, copies the rebuilt runtime binary and message interfaces into this release folder, runs smoke tests, and regenerates `general_planner_release.tar.gz`.

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

The controller should consume `quadrotor_msgs/PositionCommand` and, if needed, `quadrotor_msgs/PolynomialTrajectory`.

## Notes

- Do not edit generated runtime YAML under `~/.ros/general_planner_runtime`; edit the full YAML under `config/` or pass your own `interface_config`.
- If target hardware is not Ubuntu 20.04 x86_64 / ROS Noetic, rebuild the binary on the target ABI.
- The bundled `quadrotor_msgs` are public interface definitions, not planner implementation source.
