# EPICON exploration frontend for General Planner

The default coverage mode uses the vendored EPICON point-cloud frontend under
`third_party/epicon`, with General Planner trajectory optimization and commitment.
The default `exploration.yaml` matches EPICON garage frontier, viewpoint and
topology parameters; `exploration_sim.yaml` starts garage at `(5, 0, 2)`.

## Source layout

- `include/general_core/exploration/highspeed` and
  `src/general_core/exploration/highspeed` own the frontend, exploration FSM,
  General Planner adapter and trajectory server.
- `config/exploration.yaml`, `config/exploration_sim.yaml` and
  `config/map.yaml` are the unified planner,
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

The default coverage settings are:

- Exploration uses the garage box `[-10, -6, -0.2]` to `[182, 150.2, 3.8]`.
  Dynamic box selection is disabled in this simulation profile; set the bounds
  for the actual site before deployment.
- After odometry arrives the FSM waits for `/move_base_simple/goal`. The clicked
  position starts exploration; the frontend selects the exploration goals.
- Each cluster has 72 initial viewpoint samples: 3 radii, 8 angles and 3 heights.
  Frontier cells are 0.4 m and connection A* uses 0.2 m resolution.
- Frontier clustering updates the observed region incrementally; viewpoint
  generation considers the nearest `local_tsp_size` eligible clusters.
  Completion confirmation does not force global reclustering or
  full-map viewpoint resampling. `epicon/execution/audit_period` is the minimum
  interval between finish-confirmation observations, not a periodic global rebuild.
- Candidate filtering and tour ordering follow native EPICON, including its
  existing goal-continuation cost. The measured odometry root is reconnected
  before historical edges are updated, preserving the native update order.
  Odometry connects to nearby topology nodes
  in parallel without an extra three-edge cap. There is no separate wrapper
  goal hold, viewpoint retry cache, backtracking goal selector or cached local
  route override.
- Task reset and goal-cooldown expiry reactivate non-dormant clusters for the
  next local candidate check, so temporary exclusions cannot persist indefinitely.
- General Planner still owns the future execution head, trajectory stitching,
  collision validation and command server. These are distinct from the native
  EPICON trajectory backend and MARSIM dynamics simulator.
- `FINISH` requires repeated fresh no-actionable-goal observations while the
  vehicle is settled. Residual clusters do not independently block completion.
- The default `cloud_odom_mode` is `latest_odom`. Input queue and freshness
  settings remain in the unified YAML.

The embedded LKH solver uses an instance-specific temporary directory.
The `candidate_*` and `frontier_pass_*` composite-score settings belong to the
alternative HighSpeedExp manager; the default EPICON coverage frontend does not
read them. The following legacy feature notes describe that alternative path,
not additional layers enabled in the default garage run.

## Long-horizon coverage guidance

The alternative HighSpeedExp path supports `coverage_guidance/mode: full`.
The default EPICON garage profile sets this mode to `off`. This layer keeps a
persistent coarse map from the raw ROG state (it intentionally does not use the
LIO known-free safety fallback), splits free and unknown voxels into connected
zones inside fine cells, and builds a free/unknown adjacency graph. Active-free
zones are anchored by current HighSpeedExp frontiers; reachable unknown zones
are aggregated at a coarser scale when they are far away. A deterministic open
coverage route then supplies a priority for each existing frontier.

Coverage guidance never creates a flight goal in unknown space. It only adds a
bounded set of frontiers from the current and next coverage zones to the old
nearest-cluster shortlist. The existing viewpoint visibility, topology search,
safety gates, General corridor, MINCO, backup and trajectory server remain the
owners of execution. Map sampling is rate-limited and route construction runs
on a latest-wins worker thread. Missing, invalid or stale coverage results
therefore fall back to the existing frontier objective without blocking the
sensor callback.

The launch argument `coverage_guidance_mode:=off|shadow|soft|full` controls the
integration:

- `off`: legacy frontend behavior and no coverage worker;
- `shadow`: persistent map/route, logging and visualization only;
- `soft`: add a bounded coverage priority to the composite candidate cost;
- `full`: use the stronger route rank. A persistent unknown macro zone can
  delay `FINISH` only while the frontend still has a reachable executable
  viewpoint; coverage guidance does not synthesize goals itself, so an
  unknown-only map target cannot deadlock the FSM.

The persistent resolution and fine/macro scales are configured under
`coverage_guidance/*` in `config/exploration.yaml`. The route, active-free
anchors and reachable-unknown anchors are published on
`/exploration_node/coverage_guidance/route`. For a new scene, set `box_num`,
`box_i/down`, `box_i/up` and dead areas first; these bounds define both frontier
processing and the persistent coverage domain.

`full_rank_weight` converts the normalized active-frontier visit order into a
candidate cost. The normalization is over active frontier order only; unknown
macro nodes in the coverage route must not inflate this value. The optional
`rank_penalty_cap` bounds the per-candidate coverage term (`0` disables the
cap). On the garage full-mission test, `cap=8` improved mid-run coverage AUC but
increased total finish time and path length, so the delivered finish-time
profile leaves it disabled.

The final same-binary garage A/B run used eight consecutive `Finished.` reports
as the terminal condition. `shadow` finished in 977.9 s at 60.00% observed
coverage over 2390.8 m. The delivered `full` profile finished in 791.8 s at
60.34% over 2057.6 m (19.0% less time and 13.9% less path). The stronger global
order was faster to complete but had lower mean time-integrated coverage in
this single run; use `shadow`/`full` repeated trials when tuning for a new map.

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
- Failed goals enter a bounded cooldown set so several bad candidates can be
  bypassed instead of repeatedly selecting only the most recent one.
- A short topology spur that returns to its incoming branch is collapsed
  before MINCO. A genuine non-returning hairpin is still truncated to a safe
  rolling horizon.
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
- `FINISH` requires repeated executable-frontier-empty observations, elapsed
  debounce time, a slow vehicle and an ended committed trajectory. Raw
  clusters that repeatedly produce no reachable viewpoint no longer cause an
  infinite retry, while a newly executable frontier still resumes planning.

For the alternative HighSpeedExp frontend, tuning parameters include `candidate_*`, `frontier_pass_*`,
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
`cloud odom callback cost`, `[global update]`, `[coverage guidance]`,
`[candidate cost]`, `[frontend compatibility]` and `tour solver cost`.

## Unified configuration

`roslaunch task_planner exploration.launch` loads algorithm settings from
`exploration.yaml` and mapping settings from `map.yaml`. Both paths can be selected
with `exploration_config:=...` and `map_config:=...`. The trajectory server reads
`replan_time` and `heartbeat_timeout` from the same exploration profile.

Algorithm arguments (replanning intervals, sensor queues, topology resolution and
visualization flags) have been removed from the single-UAV launch chain; edit the
YAML instead. `sim_config` still selects the simulator PCD and initial pose.
