# Map Manager

This ROS package owns both occupancy representations used by General Planner:

- `ROGMap`: fixed-size, robot-centric dense map for collision checking, ESDF,
  inflation, and local trajectory generation.
- `BoundaryMap`: persistent global sparse map. It stores only BDM boundary
  voxels in `(x,y)` hash columns sorted along `z`; known-free volume is
  recovered by directional boundary queries and unknown volume is implicit.
- `MapManager`: the only facade exposed to planning code. It forwards strict
  local safety queries to ROGMap and exposes explicit global occupancy/frontier
  queries backed by BoundaryMap.
- `IncrementalTopologyGraph`: persistent bubble-based free-space graph with
  stable node IDs, stored edge costs, bounded dirty-region rebuilding,
  weighted shortest-path queries, and immutable snapshots.

ROGMap reports only sensor-driven discrete occupancy transitions. MapManager
re-evaluates the changed voxel and its six neighbors and incrementally updates
their boundary status. Ring-buffer resets caused by sliding are deliberately
not reported as observations, so leaving the local window cannot erase global
map evidence.

Exploration policy is intentionally outside this package. It may consume
`getGlobalFrontiers()` and `getGlobalGridType()`, but frontier scoring, route
commitment, failure cooldown, and behavior state machines belong to planner
modules.

## Incremental topology

`MapManager` feeds the topology from the same sensor-driven voxel transitions
used by `BoundaryMap`. The mapping callback only marks padded spatial regions
dirty. Call `updateTopology()` from a low-rate planning or visualization timer;
each call rebuilds a configured number of regions, preserves matching node IDs,
removes invalid incident edges, and validates new edges against the current map.
Within each dirty region it adaptively subdivides space, creates clearance
bubbles, unions overlapping bubbles, and keeps one representative per connected
component. `updateTopologyAround()` prioritizes work close to a planning query.
Successful state2state guide paths call `observePlannedTopologyPath()`. This
seeds previously unseen route regions for later budgeted expansion; it does not
trust a planned polyline as a permanent edge without map validation.

The graph core depends only on `TopologyMapView` (`isTraversable()` and
`getClearance()`), not on ROGMap, LIO or any planner state machine. MapManager's
adapter uses local inflated ROG occupancy plus persistent BoundaryMap evidence.
If `unknown_as_free` is selected it applies only inside the current local map;
unknown global volume is never stored as historical free space.

The module is disabled by default. It can be enabled directly:

```cpp
general_planner::IncrementalTopologyGraph::Config topology;
topology.enabled = true;
map_manager->configureTopology(topology);
map_manager->updateTopology();
auto snapshot = map_manager->topologySnapshot();
```

For ROS1, constructing `TopologyGraphROS1(nh, map_manager)` loads the following
private parameters, performs budgeted updates on a timer, and publishes
`visualization_msgs/MarkerArray` on `map_manager/topology` by default:

```yaml
topology:
  enabled: true
  frame_id: world
  topic: map_manager/topology
  update_period: 0.2
  region_size: 4.0
  sample_spacing: 1.0
  min_clearance: 0.45
  max_clearance: 2.5
  candidate_separation: 1.5
  stable_match_distance: 1.0
  connection_radius: 6.0
  edge_sample_spacing: 0.2
  dirty_padding: 2.5
  bubble_overlap_margin: 0.1
  unknown_as_free: false
  max_nodes_per_region: 4
  max_bubbles_per_region: 256
  max_neighbors: 8
  max_regions_per_update: 4
  node_scale: 0.22
  edge_scale: 0.06
```

Planning uses `findTopologyPath(start, goal, path)`. Endpoint attachment and all
stored edges must pass current visibility checks; a disconnected graph returns
`false` with an empty path rather than inventing an unvalidated straight line.
