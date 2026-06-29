# Exploration and NHBP System Architecture

This document defines the target implementation plan for the complete
exploration task and the Navigation-memory and Hysteresis Belief Planner
(NHBP). It is a task-level architecture document that extends
`system_architecture.md` without changing the top-level ownership model.

## Purpose

The exploration task must become a complete mission plugin instead of a thin
frontier-to-goal wrapper. The target system must:

- use ROG-Map frontier information when available;
- keep stable exploration memory across replans;
- solve a frontier/viewpoint ordering problem with ATSP semantics;
- prevent Navigation Deadlock Oscillation (NDO) caused by local greedy
  replanning;
- route all trajectory generation through existing General Planner backends;
- keep ROS adapters out of exploration policy.

NHBP is the long-term anti-NDO layer:

```text
NHBP =
NavigationMemory
+ FrontierMemory
+ CoverageGrid
+ TopologicalMemory
+ Hysteresis Commit
+ Recovery Escape
+ Optional learned value prior
```

The optional learned prior is only a soft scoring term. It must never replace
safety checks, reachability checks, or deterministic recovery logic.

## Current Baseline

The current implementation already has:

- `ExplorationMissionAdapter`, exposed as the exploration task adapter;
- `ExplorationTaskExecutor`, integrated with the existing FSM task executor
  boundary;
- `ExplorationFrontend`, which scans the local map, clusters frontier-like
  free cells, samples viewpoints, scores candidates, and estimates A* travel
  cost;
- `ExplorationRuntimeManager`, which keeps or reuses the latest exploration
  goal with a simple score margin;
- `rog_map` support for frontier cells through `isFrontier()` and
  `boxSearch(..., FRONTIER, ...)`;
- common `CommitGovernor`, `SafetyMonitor`, trajectory execution, diagnostics,
  and ROS adapters.

Exploration code lives under the dedicated module paths:

```text
include/general_core/exploration/
src/general_core/exploration/
```

The current implementation is not yet a complete exploration system because it
does not have persistent frontier memory, coverage memory, topological memory,
ATSP ordering, explicit NDO detection, or recovery escape behavior.

## Ownership Rules

The complete exploration implementation must follow these ownership rules:

- The top-level `Fsm` remains the only execution FSM for trajectory lifecycle,
  safety, publication, and timer-driven replanning.
- Exploration may have an internal mission controller state machine, but it
  must live below `ExplorationMissionAdapter`.
- Do not add another ROS-level exploration FSM.
- Do not add new public `GeneralPlanner::PlanXXX` entry points unless they are
  compatibility wrappers required by the existing task plugin contract.
- Core exploration logic must not publish ROS messages directly.
- Core exploration logic must not depend on ROS time, ROS topics, or RViz
  markers directly; diagnostics must cross through planner diagnostics
  services.
- Backends generate trajectories. Exploration chooses candidates, tours,
  recovery goals, and task state.
- Safety checks remain hard constraints. NHBP can bias or reject decisions, but
  it cannot override `SafetyMonitor`.

## Target Data Flow

```text
Map / Perception / ROG-Map
  -> BeliefStateUpdate
  -> NavigationMemory
       SpatialMemory
       FrontierMemory
       CoverageMemory
       TopologicalMemory
       DecisionMemory
       FailureBlacklistMemory
  -> ExplorationCandidateGenerator
       frontier viewpoints
       current tour continuation
       topology escape node
       frontier-to-goal intermediate
       safe recovery node
  -> ATSPTourPlanner
  -> NdoDetector
  -> DecisionStabilizer
  -> ExplorationMissionController
  -> State2StatePlanner / existing backend
  -> CommitGovernor
  -> SafetyMonitor / Backup trajectory
  -> Execute / Commit / Log
```

## Exploration Mission Controller

`ExplorationMissionController` is the target replacement for the current thin
`ExplorationRuntimeManager`. It owns task-internal state, not ROS execution.

Required states:

```text
IDLE
WAIT_MAP
UPDATE_BELIEF
SELECT_TOUR
SELECT_LOCAL_GOAL
PLAN_LOCAL_TRAJECTORY
EXECUTE_COMMITTED
RECOVERY_ESCAPE
FINISHED
FAILED
```

State responsibilities:

- `WAIT_MAP`: wait for odom and a usable map window.
- `UPDATE_BELIEF`: update frontier memory, coverage memory, topology memory,
  and failure memory.
- `SELECT_TOUR`: run ATSP over active candidate viewpoints.
- `SELECT_LOCAL_GOAL`: choose the next receding local goal from the tour or a
  recovery candidate.
- `PLAN_LOCAL_TRAJECTORY`: call existing state2state planning through the task
  adapter boundary.
- `EXECUTE_COMMITTED`: keep or switch the active goal using hysteresis and
  progress checks.
- `RECOVERY_ESCAPE`: generate escape candidates when NDO, repeated failure, or
  no-progress conditions are detected.
- `FINISHED`: return exploration finish only when no active, dormant, or
  recoverable frontier remains and coverage criteria pass.
- `FAILED`: temporary failure state; top-level FSM decides retry/hold behavior
  through existing commit governance.

The controller must expose a small API:

```text
reset()
updateBelief(robot_state, current_yaw)
selectGoal(new_task, committed_remaining) -> ExplorationDecision
onPlanResult(decision, ret_code)
snapshotDiagnostics()
```

`ExplorationDecision` must include the selected goal, yaw, candidate id,
frontier id, tour id, reason, score breakdown, guide path, NDO state, and
recovery flag.

## Frontier Provider

The frontier provider must support two sources:

- `rog_map_frontier`: call `MapManager::boxSearch(..., FRONTIER, points)`;
- `fallback_scan`: current local-map scan for known-free cells adjacent to
  unknown cells.

The default source should be `auto`:

```text
auto:
  if rog_map/frontier_extraction_en is enabled and frontier points are present:
      use rog_map_frontier
  else:
      use fallback_scan
```

Important semantic difference:

- ROG-Map frontier cells are unknown-side frontier cells.
- The current frontend frontier cells are free-side boundary cells.

Therefore ROG-Map frontier cells must not be used directly as motion goals.
They must be clustered first, then converted into safe known-free viewpoints.

## Frontier Memory

`FrontierMemory` owns persistent frontier identity across replans.

Each frontier record must contain:

```text
frontier_id
centroid
bbox_min / bbox_max
normal_or_unknown_direction
cell_count
source
first_seen_time
last_seen_time
last_selected_time
last_covered_time
viewpoints
best_viewpoint_id
status
attempt_count
failure_count
blacklist_until
topology_anchor_id
coverage_gain_history
```

Required statuses:

```text
ACTIVE
DORMANT
COVERED
BLACKLISTED
UNREACHABLE
STALE
```

Rules:

- A frontier must not disappear immediately because it is half inside the local
  map or temporarily not well observed.
- A frontier becomes covered only after the coverage grid and observation
  checks agree.
- Failed frontiers enter blacklist memory with reason and TTL.
- A blacklisted frontier can be revalidated after TTL or when map evidence
  changes significantly.

## Coverage Grid

`CoverageGrid` tracks exploration completeness and prevents repeated local
looping. It must support:

- visited free-space cells;
- observed unknown-to-known transitions;
- stale local-map regions;
- coverage timestamps;
- local progress around the active goal;
- global coverage finish criteria.

Finish is valid only when:

- no active reachable frontier remains;
- no dormant frontier can be revalidated;
- no recovery frontier-to-goal candidate exists;
- coverage growth remains below threshold for a configured hold window.

## Topological Memory

`TopologicalMemory` is the generalized form of fy_node posegraph plus topo
blacklist. It must be task-generic and usable by exploration and future
state2state recovery.

Required records:

```text
TopoNode:
  node_id
  position
  created_time
  last_seen_time
  coverage_state
  failure_count
  blacklist_until

TopoEdge:
  edge_id
  from_node
  to_node
  length
  status
  last_validated_time
  failure_count
  blacklist_until
```

Edge statuses:

```text
UNKNOWN
VALID
SUSPECT
BLOCKED
STALE
```

Rules:

- Topological memory stores traversed free-space structure.
- Direct reachability must use line-free checks plus A* length sanity, not only
  Euclidean distance.
- Blacklist should prefer edge/corridor records over node-only records when the
  failure evidence is edge-specific.
- Blacklist records must have TTL, reason, and revalidation rules.
- Start-node selection for topo paths must filter blacklisted/suspect nodes and
  edges.

## ATSP Tour Planner

Exploration must include an ATSP layer because viewpoint-to-viewpoint cost is
asymmetric:

- yaw transition is directional;
- dynamic start state makes `cost(i, j)` differ from `cost(j, i)`;
- topology memory can penalize one direction more than the reverse;
- failure memory and recovery policy can add directed edge penalties;
- candidate information gain is node-specific and must be folded into the
  directed tour objective.

The ATSP interface must be independent from any concrete solver:

```text
ATSPProblem:
  depot
  candidates
  directed_cost_matrix
  node_reward
  required_nodes
  forbidden_edges
  time_budget_ms

ATSPSolution:
  ordered_candidate_ids
  total_cost
  solver_status
  solve_time_ms
  fallback_used
```

Cost terms:

```text
cost(i, j) =
  travel_cost(i, j)
+ yaw_transition_cost(i, j)
+ curvature_cost(i, j)
+ unknown_risk_cost(i, j)
+ topology_penalty(i, j)
+ revisit_penalty(j)
+ failure_penalty(j)
+ switch_penalty(j)
- information_gain_reward(j)
- committed_goal_bonus(j)
- topology_diversity_bonus(j)
```

Safety is not a cost term. Unsafe nodes or edges must be excluded.

Solver tiers:

- `small_exact`: exact dynamic-programming ATSP for small candidate sets.
- `heuristic`: deterministic nearest/cheapest insertion plus directed local
  search for normal online replanning.
- `external_lkh`: optional file-based adapter for LKH-style solvers when
  configured and available.

Required behavior:

- Solver must respect a strict time budget.
- Solver output must be deterministic for the same input.
- If the solver times out, it must return the best partial valid tour or fall
  back to greedy.
- Receding exploration should commit only the first local goal or a short tour
  prefix, not the full tour as an irreversible command.
- Tour recomputation should be suppressed by hysteresis unless frontier memory
  changes materially.

## Navigation Memory

`NavigationMemory` owns the NHBP belief state and should be a shared service
inside planner context once stable.

It contains:

```text
SpatialMemory:
  visited cells
  observed cells
  stale cells
  local no-progress regions

FrontierMemory:
  persistent frontier records
  frontier attempts/failures
  dormant/covered/blacklisted status

CoverageMemory:
  coverage grid
  coverage growth history
  finish criteria

TopologicalMemory:
  posegraph nodes
  directed edges
  topo path validation
  topo blacklist and revalidation

DecisionMemory:
  recent candidate ids
  recent committed goals
  selected tour ids
  switch count
  progress toward active goal

FailureBlacklistMemory:
  failed frontier ids
  failed viewpoint ids
  failed topo nodes/edges
  failed corridors
  failure reason
  count
  confidence
  TTL
```

Failure reasons:

```text
ASTAR_FAIL
ASTAR_TOO_LONG
OPTIMIZATION_FAIL
SAFETY_COLLISION
NO_PROGRESS
NDO_OSCILLATION
BACKUP_TRIGGERED
VIEWPOINT_UNSAFE
FRONTIER_COVERED
MAP_STALE
```

## NDO Detector

`NdoDetector` must detect behavioral oscillation from history, not only path
search failure.

Inputs:

- recent robot positions and velocities;
- recent committed candidate ids;
- recent frontier ids and tour ids;
- guide path start direction;
- progress along active goal;
- replan result history;
- backup/safety events;
- blacklisted node/edge/corridor history.

Detection rules:

- Candidate cycle: repeated `A -> B -> A -> B` or equivalent candidate set
  oscillation in a short window.
- Direction cycle: local guide path first segment alternates left/right while
  global progress remains low.
- No progress: displacement or projected progress below threshold despite
  repeated replans.
- Failure loop: same frontier/viewpoint/topo edge fails more than threshold.
- Safety loop: repeated backup or keep-old decisions around the same region.
- Score thrashing: candidate score differences remain within switch margin but
  selected goal changes repeatedly.

Outputs:

```text
NdoState:
  STABLE
  SUSPECT
  DEADLOCKED

NdoDiagnosis:
  state
  reason
  implicated_candidate_ids
  implicated_frontier_ids
  implicated_topo_edges
  suggested_action
```

## Decision Stabilizer

`DecisionStabilizer` sits before `CommitGovernor` and after candidate/tour
selection.

It must implement:

- hysteresis: keep current goal unless the new goal is better by an adaptive
  margin;
- commitment: minimum hold time or minimum progress before switching;
- switch margin: larger margin when NDO is suspected;
- revisit penalty: decaying penalty for recently attempted regions;
- recovery trigger: force a recovery candidate when NDO is deadlocked;
- safety override: unsafe current trajectory or emergency state bypasses
  hysteresis.

The stabilizer returns:

```text
KEEP_CURRENT
SWITCH_TO_CANDIDATE
RUN_RECOVERY
FINISH_EXPLORATION
TEMPORARY_FAILURE
```

## Recovery Escape

Recovery candidates are generated only when normal exploration cannot make
progress or NDO is detected.

Required recovery candidate types:

- continue current corridor if still safe;
- choose the next ATSP candidate that is topologically different;
- move to a nearby valid topology escape node;
- select a frontier-to-goal intermediate candidate;
- retreat to a recently visited safe coverage cell;
- hold/brake if no safe recovery candidate exists.

Recovery must write failure evidence into `FailureBlacklistMemory`, but the
record must be reversible through TTL or revalidation.

## Configuration Additions

Target config namespace:

```yaml
general_planner:
  exploration_frontier_source: auto
  exploration_use_frontier_memory: true
  exploration_frontier_memory_max_records: 256
  exploration_frontier_memory_ttl: 45.0
  exploration_frontier_memory_failure_ttl: 12.0
  exploration_frontier_memory_covered_radius: 0.8
  exploration_frontier_memory_recovery_min_distance: 0.8
  exploration_use_coverage_grid: true
  exploration_coverage_grid_resolution: 1.0
  exploration_coverage_revisit_radius: 0.8
  exploration_coverage_revisit_time_window: 5.0
  exploration_coverage_revisit_penalty_weight: 0.5
  exploration_coverage_grid_max_cells: 4096
  exploration_use_topological_memory: true
  exploration_use_atsp: true

  exploration_atsp_solver: fy_node_lkh
  exploration_atsp_work_dir: /tmp/general_planner_atsp
  exploration_atsp_external_command: ""
  exploration_atsp_time_budget_ms: 30
  exploration_atsp_cost_scale: 100
  exploration_atsp_max_candidate_num: 96

  exploration_nhbp_enable: true
  exploration_nhbp_decision_history: 16
  exploration_nhbp_blacklist_ttl: 12.0
  exploration_nhbp_min_commit_time: 1.0
  exploration_nhbp_min_progress_distance: 0.3
  exploration_nhbp_switch_margin: 0.25
  exploration_nhbp_max_switches: 4
  exploration_nhbp_no_progress_time: 3.0
  exploration_nhbp_recovery_enable: true
```

Defaults must preserve current behavior as much as possible when NHBP is off.

## Implementation Phases

### Phase 0: Instrumentation

- Add candidate/tour/decision trace structures.
- Log selected frontier id, viewpoint id, tour id, score breakdown, guide path
  length, switch reason, and NDO diagnosis.
- Add offline metrics for switch rate, progress rate, coverage growth, repeated
  failure count, and ATSP solve time.

### Phase 1: Frontier Provider

- Add a `FrontierProvider` abstraction.
- Use ROG-Map `FRONTIER` box search when enabled.
- Keep current scan as fallback.
- Normalize both sources into one frontier-cluster format.

Current status: partially complete. `ExplorationFrontend` now supports
`exploration_frontier_source` with `auto`, `rog_map_frontier`, and
`fallback_scan` behavior. In `auto` mode it first converts ROG-Map unknown-side
frontier cells into free-side frontier cells for the existing clustering and
viewpoint pipeline, then falls back to the previous local scan when ROG-Map
frontiers are disabled or unusable.

### Phase 2: Memory Core

- Add `FrontierMemory`, `CoverageGrid`, and `FailureBlacklistMemory`.
- Preserve frontiers across local map sliding.
- Add TTL-based blacklist and revalidation.
- Keep behavior-compatible mode with memory disabled.

Current status: partially complete. `general_core/nhbp` now exists as the
long-term home for NHBP state, while `general_core/exploration` owns
exploration-specific `FrontierMemory` and `CoverageGrid`. `NavigationMemory`
records decisions, failure TTL blacklist entries, and basic NDO diagnoses for
repeated switching and no-progress windows. `FrontierMemory` persists observed,
committed, failed, and covered frontiers with TTL-based recovery filtering.
`CoverageGrid` records recently visited cells and contributes a revisit penalty
to runtime goal stabilization. Successful decisions, failed goals, and coverage
observations are now persisted across replans through `ExplorationRuntimeManager`.

### Phase 3: ATSP Planner

- Add `ATSPTourPlanner` interface.
- Implement exact small solver and deterministic online heuristic.
- Add optional external solver adapter behind config.
- Use receding prefix execution.

Current status: partially complete. `general_core/exploration/atsp` now owns an
`ATSPTourPlanner` interface. The file format and LKH parameter/tour parsing
follow the `fy_node` `single.tsp` / `single.par` / `single.txt` ATSP workflow.
The `fy_node` `lkh_tsp_solver` package has been migrated into this workspace,
so `solver=fy_node_lkh` directly calls `solveTSPLKH()`. The external command
adapter is still available for alternate LKH binaries, and a deterministic
greedy directed-tour solver remains as fallback. `ExplorationFrontend` now uses
the ATSP module to rank reachable frontier viewpoints and executes the first
receding-horizon candidate.

### Phase 4: Exploration Mission Controller

- Replace the thin runtime manager with `ExplorationMissionController`.
- Move exploration-specific state transitions out of `GeneralPlanner`.
- Keep top-level FSM trajectory execution unchanged.

Current status: partially complete. `ExplorationRuntimeManager` is still a thin
mission-state holder, not the full target `ExplorationMissionController`.
However, it now owns committed-goal state separately from transient selected
goals, exposes NHBP-stabilized candidate selection, delays premature finish when
recoverable frontier memory exists, and provides memory recovery goals when the
frontend temporarily fails or NHBP rejects the current candidate. The remaining
controller work is to move the full `WAIT_MAP -> UPDATE_BELIEF -> SELECT_TOUR ->
SELECT_LOCAL_GOAL -> PLAN_LOCAL_TRAJECTORY -> EXECUTE_COMMITTED ->
RECOVERY_ESCAPE` loop out of `GeneralPlanner::PlanExplorationFromRest` /
`ReplanExplorationOnce`.

### Phase 5: Topological Memory

- Add posegraph/topology memory.
- Validate directed edges with line-free and A* sanity checks.
- Add topo blacklist with TTL and reason.
- Use topology escape candidates in recovery.

Current status: first compatibility implementation is in place.
`general_core/nhbp/TopologicalMemory` records visited topology nodes,
directed transitions, failure TTL blacklist evidence, and recovery positions
from previously visited safe nodes. `ExplorationRuntimeManager` now writes
successful exploration transitions into topology memory, marks failed goal
regions as temporarily blocked, and falls back to topology recovery when
frontier-memory recovery has no valid candidate. Edge validation is still
lightweight in this step; the next refinement is to validate topology edges
with explicit line-free plus A* sanity checks before using them as multi-hop
escape paths.

### Phase 6: NHBP Anti-NDO

- Add `NavigationMemory` aggregate.
- Add `NdoDetector`.
- Add `DecisionStabilizer`.
- Add recovery escape policy.
- Integrate decision output before `CommitGovernor`.

Current status: first closed loop is implemented. The current implementation
provides `NavigationMemory`, `DecisionStabilizer`, a diagnosis result type,
decision history, TTL failure blacklist, switch-margin hysteresis, minimum
commit-time hysteresis, repeated-switch/no-progress NDO detection, frontier
failure memory, coverage revisit penalty, finish-delay checks, and memory-based
recovery goals. The stabilizer now runs before local trajectory optimization in
the exploration pipeline. The recovery chain now tries frontier memory first
and topology memory second. The remaining work is richer multi-hop topology
escape, learned/value-prior hooks, and scenario-level NDO regression tests.

### Phase 7: Cleanup and Extraction

- Move residual exploration policy out of `GeneralPlanner`.
- Keep `ExplorationMissionAdapter` as the task plugin boundary.
- Reduce duplicated frontier logic once ROG-Map frontier source is stable.

Current status: in progress. Exploration headers/sources are now physically
under `general_core/exploration`, ATSP is isolated in
`general_core/exploration/atsp`, shared string normalization moved into
`general_core/utils`, and NHBP-specific state lives under `general_core/nhbp`.
ROG frontier input is now bounded before clustering by raw frontier sampling,
free-side frontier-cell sampling, cluster caps, A* check caps, and reachable
candidate caps. A dedicated guard script,
`sh_files/test_exploration_big_field_guard.sh`, records the big-field
exploration smoke-test metrics and fails on odom-stale regressions, planning
deadlock symptoms, frontier cap regressions, or excessive candidate-search
timeouts.

## Progress Log

### 2026-06-29

Latest exploration/NHBP push:

- Added configurable ROG frontier sampling and clustering caps for big-field
  exploration.
- Added separate `exploration_max_astar_checks` and
  `exploration_max_reachable_candidate_num` limits so candidate scoring cannot
  spend unbounded time on unreachable frontier viewpoints.
- Added optional topological memory with node/edge records, TTL blacklist
  evidence, and topology recovery positions.
- Integrated topology memory into exploration success/failure recording and
  the recovery goal chain.
- Added `diagnosticSummary()` for exploration memory/NDO state snapshots.
- Added `sh_files/test_exploration_big_field_guard.sh` as a Docker-friendly
  regression guard for the `big_field.pcd` perfect-drone exploration scenario.

## Testing Requirements

Unit tests:

- ROG-Map frontier normalization.
- fallback frontier scan parity.
- frontier clustering and stable id assignment.
- viewpoint generation and safety filtering.
- memory TTL and blacklist revalidation.
- ATSP exact solver on small directed matrices.
- ATSP heuristic determinism and timeout fallback.
- NDO cycle/no-progress/failure-loop detection.
- decision stabilizer hysteresis behavior.

Integration tests in Docker `ros1_noetic`:

- `catkin_make --pkg general_planner`;
- full workspace `catkin_make`;
- existing `GP_SKIP_BUILD=1 sh_files/test_general_planner_ros1.sh`;
- exploration launch smoke test with ROG-Map frontier disabled;
- exploration launch smoke test with ROG-Map frontier enabled.

Scenario tests:

- empty map / no obstacle: no early left-right oscillation;
- blocked frontier: frontier enters blacklist and planner switches;
- local map sliding: frontier is not deleted prematurely;
- narrow corridor: topo memory avoids repeated wrong entrance;
- loop corridor: NDO detector triggers recovery;
- no frontier left: finish only after coverage criteria pass;
- ATSP stress: many frontier clusters under time budget.

Required metrics:

```text
coverage_growth_rate
goal_switch_rate
same_frontier_retry_count
same_topo_edge_failure_count
NDO_suspect_count
NDO_deadlocked_count
recovery_success_count
ATSP_solve_time_ms
candidate_count
tour_cost
progress_to_active_goal
```

## Definition of Done

Exploration is considered complete when:

- it can run as a task plugin without a separate top-level FSM;
- ROG-Map frontier source and fallback scan both work;
- frontier memory, coverage memory, topology memory, decision memory, and
  failure blacklist are active and logged;
- ATSP produces deterministic receding tours within the configured time budget;
- NHBP detects and suppresses NDO in repeated local-optimum scenarios;
- recovery escape can leave a deadlocked region or safely hold;
- existing state2state/tracking/perching tests remain behavior-compatible;
- exploration smoke tests pass in Docker `ros1_noetic`;
- logs are sufficient to explain every goal switch, blacklist, recovery, and
  finish decision.

## Progress Log

### 2026-06-29

Completed in this exploration/NHBP step:

- Created the dedicated `general_core/exploration` module for exploration
  headers and sources.
- Moved `ExplorationFrontend`, `ExplorationRuntimeManager`, and
  `ExplorationMissionAdapter` into the exploration module.
- Added `general_planner/exploration_frontier_source` config with
  behavior-compatible default `fallback_scan`.
- Enabled `exploration.yaml` to use `exploration_frontier_source: auto` and
  ROG-Map `frontier_extraction_en: true`.
- Added ROG-Map frontier ingestion in the exploration frontend. ROG-Map
  unknown-side frontier cells are converted to adjacent safe free-side frontier
  cells before clustering and viewpoint sampling.
- Added frontier-source diagnostics to exploration logs:
  `source`, `raw_frontiers`, and `fallback`.
- Added `general_core/exploration/atsp` and integrated ATSP ordering into
  reachable exploration viewpoint selection.
- Migrated the `fy_node` `lkh_tsp_solver` package into the current workspace.
- Mirrored the `fy_node` ATSP file workflow for linked/external LKH integration:
  `single.tsp`, `single.par`, and `single.txt`.
- Added deterministic greedy directed-tour fallback when linked/external LKH is
  unavailable or fails.
- Added `general_core/nhbp` with `NavigationMemory`, TTL failure blacklist
  records, and initial repeated-switch / no-progress NDO diagnosis.
- Added stable exploration goal IDs and memory keys for frontier/viewpoint
  decisions.
- Added `general_core/nhbp/DecisionStabilizer` and wired it into exploration
  replan selection before trajectory optimization.
- Split transient selected goals from committed exploration goals, so a failed
  candidate no longer becomes the reusable active goal.
- Added NHBP config under `exploration_nhbp_*`, including decision history,
  blacklist TTL, minimum commit time, switch margin, no-progress window, and
  recovery-enable flag.
- Added `general_core/utils` for reusable framework helpers, starting with
  normalized token parsing used by frontier-source and solver dispatch.
- Added `general_core/exploration/FrontierMemory` for observed/committed/failed/
  covered frontier records with TTL, failure blocking, and recoverable-goal
  lookup.
- Added `general_core/exploration/CoverageGrid` for visited-cell memory and
  revisit penalty.
- Integrated frontier memory and coverage memory into
  `ExplorationRuntimeManager` so successful plans, failed plans, and visited
  robot poses are recorded even when the NHBP decision layer is disabled.
- Added recovery hooks in `PlanExplorationFromRest` and
  `ReplanExplorationOnce`: frontend failure, premature finish, and NHBP reject
  can now fall back to a filtered memory recovery goal.
- Added coverage revisit penalty to NHBP stabilization so recently visited
  candidate regions have a higher switch threshold.
- Added blacklist filtering to memory recovery so navigation-memory failures do
  not get bypassed by frontier-memory recovery.
- Passed Docker validation for `catkin_make --pkg general_planner`, full
  workspace `catkin_make`, existing ROS1 smoke configs, and
  `exploration.yaml`.

Progress estimate after this step:

```text
Exploration task completeness: 72% - 78%
NHBP / NDO suppression completeness: 58% - 63%
ATSP exploration ordering completeness: 55% - 65%
```

The next 10% should extract a real `ExplorationMissionController`, add topology
escape memory, expose memory/recovery metrics in logs, and add scenario tests
for repeated A-B-A-B goal switching and no-progress local-optimum traps.

Additional completed work in the next framework push:

- Added `general_core/nhbp/TopologicalMemory` with merged topology nodes,
  directed transition edges, node/edge failure TTL blacklists, and recovery-node
  lookup.
- Integrated topology memory into `ExplorationRuntimeManager`: successful
  exploration commits observe robot-to-goal transitions, failed goals blacklist
  nearby topology nodes/edges, and recovery now tries frontier memory first and
  topology memory second.
- Improved topology recovery selection so the planner first prefers recovery
  nodes connected to the current topology node by usable non-blacklisted edges,
  then falls back to any active historical node in the configured recovery
  radius.
- Added explicit exploration runtime phase diagnostics:
  `SELECT_LOCAL_GOAL`, `PLAN_LOCAL_TRAJECTORY`, `RECOVERY_ESCAPE`,
  `EXECUTE_COMMITTED`, `FINISHED`, and `FAILED` are now included in
  `diagnosticSummary()`.
- Added backend-entry validation for memory recovery goals. Recovery goals are
  now checked for finite position, local-map containment, known-free target
  when required, inflated-map safety, optional ESDF clearance, direct-line
  reachability, and bounded A* reachability before they can be committed to the
  trajectory backend.
- Failed recovery candidates are written back to NHBP/frontier/topology memory
  before retrying, preventing unsafe or unreachable memory-recovery points from
  being selected repeatedly.
- Reworked exploration reachability evaluation into a two-stage filter:
  direct-line reachable candidates are collected first, and A* is only used as a
  capped fallback for candidates that fail direct-line reachability.
- Tuned `exploration.yaml` for the big-field perfect-drone exploration test:
  `exploration_max_candidate_num=80`, `exploration_max_astar_checks=16`,
  `exploration_max_reachable_candidate_num=16`, and
  `exploration_atsp_max_candidate_num=24`.
- Extended exploration logs with `astar_checks=` so candidate sampling pressure
  and expensive A* fallback pressure can be diagnosed separately.
- Fixed the big-field guard script to source the workspace-root `devel`
  environment instead of the stale `General-Planner/devel` environment.

Docker validation after this push:

```text
git diff --check: pass
bash -n sh_files/test_exploration_big_field_guard.sh: pass
catkin_make --pkg general_planner: pass
full workspace catkin_make: pass
90s big_field perfect_drone exploration guard: pass
  roslaunch_status=124
  goal_selected=201
  plan_success=201
  plan_failed=1
  odom_stale=0
  nhbp_reject=0
  astar_timeout=145
  max_frontiers=1536
  max_raw_frontiers=306001
  fatal=0
  astar_checks_count=201
  astar_checks_max=16
  astar_checks_avg=2.31
```

Progress estimate after this push:

```text
Exploration task completeness: 82% - 86%
NHBP / NDO suppression completeness: 72% - 78%
ATSP exploration ordering completeness: 68% - 73%
```

Remaining high-value work:

- Extract a dedicated `ExplorationMissionController` if exploration needs
  mission-level pause/resume/finish policies beyond the current task planner
  calls.
- Add scenario-level NDO tests for repeated A-B-A-B switching, blocked topology
  entrance, loop corridor, and no-progress local optimum.
- Persist or visualize NHBP memories for offline bag/log diagnosis.
- Add recovery success/failure counters to ROS diagnostics and guard scripts.
