# General-Planner RViz 3D Goal Tool

The tool is derived from the interaction used by EGO-Planner V2, but publishes
only `geometry_msgs/PoseStamped` and has no swarm-message dependency.

- Select **3D Nav Goal**.
- Hold the left mouse button and drag to choose XY and yaw.
- While still holding left, also hold right and move vertically to explicitly
  choose z.
- Release left. If height mode was entered, the tool publishes on `/goal_3d`;
  otherwise it publishes on `/goal`, so General-Planner applies `click_height`.

The regular RViz **2D Nav Goal** remains on `/goal`; General-Planner applies
`fsm/click_height` only to that input.

The RViz Fixed Frame must match the planner/map frame (`world` in the supplied
click-demo configurations).
