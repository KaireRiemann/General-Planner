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

ROGMap reports only sensor-driven discrete occupancy transitions. MapManager
re-evaluates the changed voxel and its six neighbors and incrementally updates
their boundary status. Ring-buffer resets caused by sliding are deliberately
not reported as observations, so leaving the local window cannot erase global
map evidence.

Exploration policy is intentionally outside this package. It may consume
`getGlobalFrontiers()` and `getGlobalGridType()`, but frontier scoring, route
commitment, failure cooldown, and behavior state machines belong to planner
modules.
