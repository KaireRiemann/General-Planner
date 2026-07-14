# HighSpeedExp exploration frontend for General Planner

This package keeps the effective HighSpeedExp exploration frontend and uses
General Planner only for corridor generation, trajectory optimization and
trajectory commitment.

## Source layout

- `include/general_core/exploration/highspeed` and
  `src/general_core/exploration/highspeed` own the frontend, exploration FSM,
  General Planner adapter and trajectory server.
- `config/exploration.yaml`, `config/exploration_sim.yaml` and
  `config/exploration_rog_map.yaml` are the delivered garage planner,
  simulator and map-backend profiles. Other scene profiles remain in
  `config/exploration/highspeed`.
- `task_planner/launch/exploration.launch` directly owns the delivered runtime
  composition: simulator, exploration node, trajectory server and RViz.
  `general_planner/launch/highspeed_exploration.launch` remains a compatible
  scene-oriented entry point.
- `include/general_core/exploration/exploration_utils` and
  `src/general_core/exploration/exploration_utils` contain lidar map,
  pointcloud topology, path searching and frontier management. They build as
  the internal `general_planner_exploration_utils` library rather than as
  separate catkin packages.

The former `GeneralPlanner::PlanExplorationFromRest` pipeline and its
frontier/runtime/ATSP implementation have been removed. Exploration now has
one implementation and one runtime entry point: `general_planner`'s
`exploration_node`.

## Default launch

```bash
source /root/ws/real_planner/devel/setup.bash
roslaunch task_planner exploration.launch rviz:=true
```

The default compatibility settings are:

- `auto_start:=false`: after odometry arrives the FSM remains in
  `WAIT_TRIGGER`. Click RViz `2D Nav Goal` once to publish a
  `geometry_msgs/PoseStamped` on `/move_base_simple/goal` and start
  exploration. The clicked position is only a trigger and is not used as the
  exploration target. Use `trigger_topic:=...` to select another PoseStamped
  topic, or `auto_start:=true` to restore automatic startup.
- `original_frontend_compatibility:=true`: instantaneous high-speed gating and
  original frontier/LKH semantics. Controlled reorientation is deliberately
  kept enabled: a target behind a moving vehicle is held while the vehicle
  brakes, then planned from rest instead of clearing the goal and selecting a
  different target on every retry.
- `global_planning/epic_simple_cost: false` and
  `global_planning/composite_candidate_cost_enable: true`: choose the next
  frontier with `travel + turn/brake + future return - information gain - wait
  age - pass debt`. Rewards are used only for the next-goal decision (putting a
  fixed reward on every node of an all-node TSP would be a constant). The
  directed TSP then orders the remaining clusters with the selected cluster
  forced first.
- Global routing contains one representative viewpoint per frontier cluster.
  Alternate viewpoints remain attached to the cluster for local use; they are
  no longer inserted as separate mandatory TSP visits.
- `frontier_pass_*` parameters implement an event-based side-room debt. A
  cluster earns debt only after the vehicle enters its pass radius and later
  leaves it without selecting that cluster, preventing callback frequency from
  inflating the reward.
- `use_lkh:=true`: LKH solves the directed global tour. A deterministic exact
  or directed-insertion fallback is used if LKH cannot run.
- `cloud_subscriber_queue:=1`, `odom_subscriber_queue:=50`, `sync_queue:=20`:
  old point clouds are not allowed to build a long callback backlog.
- `cloud_odom_mode:=approximate_sync` preserves timestamp-based pairing.
  `latest_odom` instead processes each cloud with the most recently received
  odometry and is intended for external simulators whose header timestamps are
  not comparable. `latest_odom_timeout` uses local wall-clock receive time.
- `max_cloud_age:=0.5`: synchronized clouds older than 0.5 seconds are dropped.
  Set it to `0` to disable age filtering. The header-age gate is not applied in
  `latest_odom` mode.

Each process writes LKH files to a private directory below `tsp_dir` (default
`/tmp`), so simultaneous robots do not overwrite each other's tours.

## First-delivery trajectory safety behaviour

- The FSM no longer prepends a future switch point to an A* path rooted at
  current odometry. The General adapter is the only owner of the future replan
  head, preventing a synthetic `[future, current, goal]` reverse segment.
- Path gates evaluate the same `max_traj_len` horizon that MINCO will commit.
  Whole-route accumulated turn and remote viewpoint yaw no longer block the
  current 26 m planning window.
- `TurnVelocityEnable` is independent of `DynamicVelocityEnable`. Each
  simplified General corridor receives its own velocity bound; straight
  corridors retain 8--10 m/s while local bends use radius/lateral-acceleration
  and angle caps. A forward/backward acceleration envelope starts braking
  before the bend.
- The default garage profile uses a zero-velocity far endpoint. Rolling replans
  normally replace it long before it is reached, while a failed backup
  optimizer can safely fall back to the verified known-free primary instead of
  deadlocking in `PLAN_TRAJ`.
- Backup generation tries multiple start/seed pairs in a tight General line
  corridor. A minimum-snap analytic brake is available if backup MINCO is
  unusable. Every accepted backup must remain known-free, progress forward,
  avoid stop-then-reaccelerate behaviour, and pass position/yaw magnitude
  checks.
- Failed plans use a bounded retry delay and keep any still-safe committed
  trajectory. Goal refresh is allowed only after the vehicle has stopped and
  no safe command remains.
- A candidate that passes MINCO but exceeds the exact committed speed or
  acceleration limit is rejected and retried with progressively lower guide
  speeds down to `TrajectoryRetryMinVel`. `MinSegmentVel` remains the normal
  cruise floor, so safety retries no longer get accidentally clamped back to
  the same speed on every attempt.
- Live FSM odometry is subscribed independently from cloud/odometry pairing.
  A delayed or dropped synchronized cloud can no longer freeze the speed used
  by controlled reorientation. Stop requests are retried at a bounded interval
  until fresh odometry confirms that the vehicle is slow.
- The trajectory server samples the exact mathematical endpoint and enters an
  explicit zero-derivative HOLD; it no longer holds the previous 100 Hz sample
  and hard-zeros from the wrong position.
- Reorientation and ordinary stop requests commit a known-free seventh-order
  braking polynomial with zero terminal velocity, acceleration and jerk. The
  old `/planning/replan` duration truncation is retained only as an emergency
  fallback when no valid braking segment can be constructed.

The main garage tuning parameters are `candidate_*`, `frontier_pass_*`,
`TurnLateralAcceleration`,
`TurnSoftAngle`, `TurnHardAngle`, `TurnSoftVelocity`, `TurnHardVelocity`,
`TrajectoryRetryMinVel`, `ReorientationHeadingAngle`, `ReplanCommitDelay`,
`BackupMinStartTime`, and `fsm/replan_time_before_traj_end`.

## Simulator input contract

The bundled `perfect_drone_*.yaml` presets reproduce the original MARSIM
Mid360 contract: 10 Hz, 0.2 degree angular resolution, 90 degree vertical FOV,
40 degree lidar pitch and the original sensing ranges/initial poses. The cloud
is in `world`, and its timestamp is the timestamp of the exact body-pose
snapshot used by the renderer.

The bundled Mid360 renderer uses double-precision mission time and an
integer-indexed sample loop. This is important for long runs: a float loop
incremented by 5 microseconds stops advancing at about 128 seconds and would
otherwise hang the simulated cloud producer while odometry remains alive.

Expected garage values are approximately:

- `/cloud_registered`: 9.9--10.0 Hz;
- 7k--9k points per frame;
- cloud age below 0.1 s;
- total cloud/map/frontier callback comfortably below the 100 ms input period.

Garage, cave, big-field and spiral use maps identical to the available
HighSpeedExp assets. The original tree references a missing `factory.pcd`, so
the factory preset intentionally retains `random_map_24_6635.pcd`; factory is
therefore not a strict map-level A/B baseline.

## Real robot

Disable the simulator and select the real world-frame registered scan and
odometry topics:

```bash
roslaunch task_planner exploration.launch \
  marsim:=false \
  odom_topic:=/lidar_slam/odom \
  cloud_topic:=/cloud_registered
```

If an external simulator cannot publish cloud and odometry with comparable
header timestamps, select receive-order pairing explicitly:

```bash
roslaunch task_planner exploration.launch \
  marsim:=false \
  cloud_odom_mode:=latest_odom \
  latest_odom_timeout:=0.5
```

This mode changes only internal pairing; input and output topic names do not
change. Point clouds must still be current world-frame scans. Also keep
`/use_sim_time` false unless a continuously advancing `/clock` is published.

Before flight, verify:

```bash
rostopic hz /cloud_registered
rostopic bw /cloud_registered
rostopic delay /cloud_registered
```

The frontend expects a current single scan, not a locally accumulated map. If
the localization system must publish a much denser cloud, provide a separate
world-frame planner topic near the original Mid360 point count instead of
feeding the accumulated cloud into the five-frame frontier projection.

For rosbag A/B tests, replay with `/clock` and enable simulated time so
`max_cloud_age` compares timestamps in the same time domain:

```bash
rosparam set use_sim_time true
rosbag play --clock input.bag
```

Useful runtime diagnostics are emitted as `[cloud input]`,
`cloud odom callback cost`, `[global update]`, `[frontend compatibility]` and
`tour solver cost`.
