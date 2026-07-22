# General-Planner decentralized swarm exploration

## Compatibility contract

`swarm_exploration/enabled` defaults to `false`.  In that mode the coordinator
creates no ROS publishers, subscribers or timers and contributes no candidate
cost.  Frontier extraction, coverage guidance, topology, global routing,
MINCO/GCOPTER, backup trajectories and the existing finish gate use the same
single-UAV code path.

When enabled, every robot still runs a complete single-UAV explorer.  The
coordination plane only adds:

1. stable spatial task identities and evidence replication;
2. lease-based task exclusion;
3. an incrementally replicated history graph;
4. graph-Voronoi ownership with workload and assist terms;
5. commit-time inter-UAV trajectory checking;
6. a team finish barrier above the local finish gate.

## Runtime data flow

```text
local ROG map
  -> FrontierManager + CoverageGuidance
  -> executable TopoNode candidates
  -> stable TaskKey / graph-Voronoi and lease cost
  -> existing composite candidate cost and goal lock
  -> existing Bubble A* / corridor / MINCO-GCopter
  -> swarm trajectory commit gate
  -> existing trajectory server
```

The coordinator never sends a low-level motion command and never replaces a
local trajectory optimizer.

## Stable tasks

A task is keyed by `(floor, ix, iy, iz)` using the common mission origin and
task size.  Local frontier cluster IDs are intentionally excluded because they
split, merge and differ between robots.  Task states are `AVAILABLE`,
`CLAIMED`, `EXPLORING`, `COMPLETED`, `EXHAUSTED` and `UNREACHABLE`.

An executing robot renews its lease at half the configured lease duration.
Conflicting claims compare estimated travel cost and then robot ID.  A peer's
live lease is a hard-sized candidate penalty.  Graph-Voronoi ownership is a
soft ranking term so a robot can still assist when it has no locally owned
task; this prevents temporary graph disagreement from losing exploration
work.

## Sparse MR-DTG subset

Executed odometry positions create stable history anchors at a bounded spatial
interval.  Consecutive local anchors are connected by the already executed
safe motion distance.  Anchors belonging to different robots form handshake
edges when they are spatially close.  Both new nodes and periodic full-node
refreshes are published, so a late subscriber can reconstruct the graph.

For each candidate, task ownership uses shortest-path distance from every live
robot anchor to the graph node nearest the task.  Euclidean distance is only a
fallback while graph components have not yet met.  Active claim count adds a
small workload term analogous to RACER's balancing objective without running
LKH/ACVRP on every interaction.

The graph is bounded by anchor spacing and `max_graph_nodes`.  Task evidence is
expired, terminal records have a longer retention interval, and trajectory
records are removed after validity plus the peer timeout.

## Trajectory safety

The position polynomial is broadcast on
`/swarm/exploration/trajectory`. Before the normal trajectory publication, the
coordinator samples overlapping polynomial time ranges and evaluates an
ellipsoidal horizontal/vertical clearance.  A deterministic priority rejects
the lower-priority commit.  If a higher-priority remote commit arrives after a
local commit, the existing controlled-stop and replanning path handles the
yield.

This gate complements rather than replaces local map collision checking,
backup generation or runtime trajectory safety.

## Finish protocol

The existing local condition remains:

```text
stable no executable frontier
+ full frontier reachability audit
+ coverage plateau
+ all observation targets exhausted
+ trajectory ended / vehicle slow
```

It becomes `LOCAL_CONVERGED`.  `TEAM_FINISH` additionally requires all fixed
team members to be alive and locally converged, no live task/evidence or lease,
and no registry change throughout the quiet and hold periods.  A missing peer
therefore causes local idle, never a false global finish.  New local reachable
frontiers immediately leave the converged state.

## Wire protocol

The workspace's `package.xml` is system-owned, so the implementation avoids a
new generated-message dependency.  Versioned records are encoded only inside
`SwarmExplorationCoordinator` using `std_msgs/Float64MultiArray`; callers never
index the arrays directly.  Separate topics carry robot state, task records,
graph deltas, finish votes and polynomial trajectories.  Field zero is the
wire version and field one is the mission epoch in every record.

This transport can later be replaced with generated messages without changing
the allocator or planner interfaces.

## Launch

Two-UAV house simulation:

```bash
roslaunch general_planner swarm_house_2uav.launch rviz:=true
```

The swarm RViz profile overlays both local ROG maps (robot 0 in cyan and robot
1 in orange), robot paths, planned trajectories, coverage routes and frontier
clusters.  Click the standard **2D Nav Goal** tool once; both explorers
subscribe to `/move_base_simple/goal` and use the click only as a synchronized
start trigger.  Each robot continues to choose its own exploration goal.  To
start without a click, use:

```bash
roslaunch general_planner swarm_house_2uav.launch auto_start:=true
```

The convenience entry point is:

```bash
roslaunch task_planner swarm_exploration.launch
```

Use a new `mission_epoch` for every fresh mission so delayed packets from a
previous run are ignored.

## Initial limitations

The first implementation shares planning-critical task and topology data, not
raw voxel maps.  A remote task is used to influence ownership only after it is
locally observable as an executable frontier/coverage candidate.  This is the
safe behavior for the current local-map corridor backend.  A future optional
on-demand corridor-map exchange may expose a remote-only task, but it must not
be allowed to bypass local known-free and commit checks.
