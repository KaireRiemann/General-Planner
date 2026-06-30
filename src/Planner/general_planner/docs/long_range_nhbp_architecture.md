# Long-Range NHBP and Sparse Geometry Architecture

This document fixes the four-stage target for long-range, cross-floor navigation
under local perception. The goal is not to copy BDM directly. BDM-like geometry
memory and NHBP solve different problems:

- Sparse geometry memory answers whether the robot still remembers large-scale
  occupancy or boundary information.
- NHBP answers why the robot came here, what failed before, whether the current
  intent should be preserved, and how to escape local decision traps.

## Final Runtime Shape

```text
ROG local map
  -> local safety query and trajectory optimization

Optional SparseGlobalMap
  -> sparse historical geometry / boundary / frontier query

NHBP
  -> navigation memory
  -> trajectory branch memory
  -> topological memory
  -> frontier / coverage / failure memory
  -> hysteresis and commitment
  -> recovery escape

FarGoalReasoner
  -> direct known goal
  -> frontier toward far goal
  -> topology recovery / backtrack node

General Planner backend
  -> state2state / exploration local trajectory
  -> safety monitor / backup
  -> commit / log / memory update
```

## Stage 1: General State2State NHBP

Goal: use NHBP in ordinary click/state2state planning, not only exploration.

Implemented components:

- `State2StateNHBPAdapter`
- trajectory branch signature: straight / left / right relative to long-range
  goal direction
- branch-level decision memory through `NavigationMemory`
- conservative commit gate before state2state replan commit
- current trajectory reuse only when the committed trajectory remains safe,
  has enough remaining time, and the candidate does not improve over the switch
  margin
- success and failure feedback into NHBP memory

This stage targets trajectory-level NDO:

```text
same goal:
  left branch -> right branch -> left branch -> right branch
```

## Stage 2: Topological Memory Upgrade

Goal: represent reusable navigation structure without a dense global grid.

Implemented base:

- `TopoNodeType`: pose, branch, frontier, failure, vertical connector
- `floor_id` on topology nodes
- explicit `observeVerticalConnector()`
- existing edge blacklist and recovery selection remain compatible
- ordinary state2state updates pose, branch/frontier subgoal, transition, and
  failure-near records when the module is enabled

Remaining integration:

- promote exploration route branch points into topology nodes
- detect and label vertical connectors from repeated altitude transitions
- add route-level backtracking to prior branch nodes when NDO is deadlocked

## Stage 3: Far-Goal Reasoner

Goal: do not let a far goal outside the local map degenerate into greedy local
replanning.

Implemented base:

- `FarGoalReasoner`
- direct-goal mode when the goal is close enough
- sparse frontier selection toward a far goal
- topology recovery fallback
- optional state2state integration through a temporary local subgoal; the
  requested long-range goal remains unchanged in the task/FSM state

Remaining integration:

- extend the same subgoal mechanism to exploration routes and future multi-floor
  missions
- write route and subgoal decisions into richer NHBP diagnostics

## Stage 4: Optional BDM-Like SparseGlobalMap

Goal: keep large-scale geometry memory without building a dense global
occupancy grid.

Implemented base:

- `SparseGlobalMap`
- sparse boundary records: free boundary, occupied boundary, frontier boundary
- quantized 3D hash query
- frontier retrieval around a center with stale-record pruning
- optional state2state runtime update from sampled local-map boundary cells
- explicit config block for sample rate, sample resolution, memory size, and
  stale-record pruning

Important boundary:

- `SparseGlobalMap` does not decide when to switch goals.
- It only provides historical geometry and frontier candidates.
- NHBP remains responsible for commitment, failure memory, branch memory, and
  recovery decisions.

Remaining integration:

- replace boundary sampling with exact slide-out ROG boundary/frontier events
  when ROG exposes that event stream
- persist / reload sparse memory across long missions if required by deployment
- keep all local safety checks on the current ROG local map and dynamic layer

## Safety Rule

NHBP may keep or reject candidate decisions only when the currently committed
trajectory is still safe under runtime checks. If current trajectory safety is
unknown, unsafe, on backup, or nearly finished, the candidate replan is allowed.

This rule preserves safety while suppressing local-optimal oscillation.
