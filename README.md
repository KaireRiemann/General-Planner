# General-Planner

## EPIC exploration migration note

The EPIC exploration frontend is wired through `GeneralPlanner::PlanExplorationOnce`.
When `general_planner/exploration/enable` and
`general_planner/exploration/use_epic_frontend` are both true, startup now checks:

- a `PointCloudMap` backend is attached to `MapManager`;
- `general_planner/exploration/tsp_dir` exists or can be created and written;
- exploration observation-map bounds are valid.

For legacy goal-based or non-exploration runs, keep exploration disabled or set
`general_planner/exploration/use_epic_frontend: false`. The original
`PlanFromRest` and `ReplanOnce` APIs remain unchanged.
