# General Planner System Architecture

This package is moving from a single local planner implementation toward a
general planning runtime. The migration must keep existing ROS1 behavior
compatible while separating semantic layers that were previously mixed in
`TaskMode`, `Fsm`, `FsmRos1`, and `GeneralPlanner`.

## Stable Layers

The target ownership is:

- Mission orchestration owns mission lifecycle and task switching.
- Task plugins own task-specific readiness plus a uniform `plan` / `replan`
  contract.
- Planning backends own concrete optimization/search implementations.
- Safety monitor owns safety result vocabulary and action selection.
- Commit governor owns candidate-vs-current trajectory commit actions.
- Trajectory manager owns committed trajectory execution and sampling.
- ROS adapters only translate messages, timers, and topics.

The current code still uses the legacy planner entry points, but the new
semantic types in `general_core/planning_semantics.hpp` are the compatibility
boundary for future extraction.

## Semantic Split

Do not add new features by extending `TaskMode` unless the value is truly a
legacy compatibility mode. New code should use:

- `MissionMode`: `idle`, `single_task`, `tracking_mission`,
  `exploration_mission`, `perching_mission`.
- `TaskType`: `state2state`, `tracking`, `perching`, `exploration`, `takeoff`.
- `BackendType`: `corridor`, `esdf`, `plain`, `jerk_tracking`,
  `snap_tracking`, `se3`.
- `ExecutionPhase`: `waiting_input`, `planning`, `executing`, `holding`,
  `recovering`, `emergency`.
- `SafetyAction`: common safety decision such as replan, hold, emergency stop.
- `CommitAction`: concrete execution action such as `commit_candidate`,
  `keep_old_trajectory`, `hold`, `brake`, `emergency_stop`,
  `request_new_input`, `retry_planning`, or `finish_mission`.

Legacy yaml remains supported. For example, `fsm/task_mode: corridor` is still
accepted as state2state, but the backend is now represented separately as
`BackendType::CORRIDOR`.

`se3`, `se3_aggressive`, `aggressive`, and `racing` are legacy input strings.
They are parsed as `TaskType::STATE_TO_STATE` with `BackendType::SE3`; SE3 is
not a separate task mode in the semantic model.

## Migration Rules

- Keep planner algorithms behind existing `GeneralPlanner` methods until each
  task backend has a dedicated class, but do not add new public `PlanXXX`
  methods. Add a task plugin or backend adapter instead.
- Move policy decisions out of ROS adapters before changing backend behavior.
- Runtime safety decisions should return a `SafetyAction`, not a task-specific
  boolean.
- Plan and replan calls should return `PlanResult`; FSM code should translate
  that through `CommitGovernor` before changing state or publishing commands.
- Tracking/perching/exploration should be task plugins. Combined behaviors such
  as tracking-perching should be mission orchestration, not a backend.
- Swarm and formation data should first live in the world model. Soft penalties
  are backend costs, not complete multi-agent coordination.

## Current Compatibility Step

This revision adds:

- explicit task, mission, backend, phase, and safety semantic types;
- `PlannerContext` as the narrow service boundary exposed to task plugins;
- formal module contracts:
  `TaskPlugin`, `PlanningBackend`, `SafetyMonitor`, `MissionOrchestrator`, and
  `RosAdapterContract`;
- `StateToStatePlanner` as the first compatibility task adapter. It owns
  state2state goal projection and delegates concrete backend calls through
  `StateToStateBackendRouter`;
- compatibility task adapters for `TrackingPlanner`, `PerchingPlanner`,
  `TakeoffPlanner`, and `ExplorationMissionAdapter`;
- task executor implementations are physically split from `fsm.cpp` into
  `src/general_core/fsm_task_executors.cpp`;
- uniform `PlanRequest`, `PlanResult`, `CommitDecision`, and `CommitGovernor`;
- backward-compatible parsing from existing yaml and task-mode strings;
- task executor identity logging with mission/task/backend fields;
- FSM-owned `MissionOrchestrator` snapshots for active task and phase;
- FSM-owned `SafetyMonitor` for runtime trajectory collision decisions;
- FSM-owned `CommitGovernor` for plan/replan result action selection;
- ROS1 adapter contract snapshots for input/output topic ownership;
- centralized handling for executed trajectory completion in the FSM;
- a runtime safety policy wrapper for committed trajectory collision decisions.
- a compatibility `tracking_perching_mission` adapter that exposes mission
  nodes: `track_target`, `handover_to_perching`, `perching`, and
  `contact_reached`.

The optimization path, frontend path search, corridor generation, ESDF/plain
selection, ROS topics, and existing planner public APIs are intentionally
kept compatible in this step.

## Current Module Mapping

- `TaskPlugin`: implemented by the existing FSM task executors through the
  `TaskPlugin<Fsm>` interface. They now expose uniform `plan` and `replan`
  calls returning `PlanResult`. The executors now delegate concrete planning
  calls to task adapters instead of calling legacy planner APIs directly. The
  implementations live in `fsm_task_executors.cpp`, leaving `fsm.cpp` focused
  on state transitions, timer callbacks, diagnostics, safety checks, and commit
  decisions.
- `PlannerContext`: carries planner services, map access, and diagnostics into
  task adapters without requiring adapters to know the full FSM. `Fsm` owns a
  single `makePlannerContext()` helper so task adapters receive the same
  diagnostic bridge.
- `StateToStatePlanner`: first extracted task adapter. The FSM state2state
  executor now constructs a `StateToStateRequest`. The adapter owns occupied
  goal projection and delegates backend calls to `StateToStateBackendRouter`.
- `StateToStateBackendRouter`: compatibility backend boundary for
  `corridor`, `esdf`, `plain`, and `se3`. Corridor/ESDF/plain now run through
  `state2state_task` operations. SE3 also runs through a state2state task
  operation and is no longer a separate task mode. SE3 optimization now lives
  behind `StateToStateSE3BackendServices`; only head-state sampling and commit
  still cross a state2state runtime adapter. Exp and backup generation now have
  dedicated `StateToStateExpBackendServices` and
  `StateToStateBackupBackendServices`. Exp and backup generation are both
  implemented in state2state backend helpers. State2state path search and ESDF
  guide endpoint preparation now use `StateToStateFrontendServices` instead of
  `GeneralPlanner` private methods. Current-trajectory no-need safety now uses
  `RuntimeTrajectorySafetyServices`, so exp generation no longer needs a
  dedicated state2state exp runtime adapter. Z diagnostics now live in the
  state2state backend domain.
  `StateToStateBackendContext` is now reduced to state2state goal mutation.
- `TrackingPlanner`: compatibility task adapter for tracking and
  tracking-perching handover calls. It now routes tracking optimization through
  `TrackingBackendServices`, with `jerk_tracking` and `snap_tracking`
  resolved from config before entering the legacy optimizer runtime.
- `PerchingPlanner` and `TakeoffPlanner`: compatibility task adapters for
  surface-based contact and dynamic takeoff tasks.
- `ExplorationMissionAdapter`: compatibility mission adapter for exploration
  plan/replan entry points.
- `PlanningBackend`: currently a registry of backend descriptors plus the
  state2state backend router. State2state and tracking now have explicit
  backend service boundaries; state2state frontend services are explicit.
  State2state no-need safety is behind runtime trajectory safety services.
  Tracking has concrete jerk/snap backend dispatch classes; the underlying
  legacy optimizer body is still kept behind a runtime adapter for behavior
  compatibility.
- `SafetyMonitor`: wraps runtime committed-trajectory collision decisions and
  returns `SafetyAction`.
- `CommitGovernor`: maps task `PlanResult` values to explicit commit actions.
  It replaces scattered top-level `RET_CODE` handling in the FSM replan and
  plan-from-rest paths.
- `MissionOrchestrator`: owns the active mission/task/backend/phase snapshot.
  The legacy FSM still drives transitions, but every transition updates this
  orchestrator.
- `RosAdapterContract`: records ROS1 topic ownership. The ROS1 adapter still
  publishes commands, but task policy is being moved out of the adapter.

## Task Plugin Direction

The current task plugin boundary is:

- `StateToStatePlanner` plus `StateToStateBackendRouter`: state2state goal
  handling and backend dispatch. SE3, exp, and backup now have explicit backend
  services instead of sharing one generic compatibility context. Exp and backup
  generation have moved into backend helpers, and path search/ESDF guide
  preparation now run through `StateToStateFrontendServices`. The next backend
  step is turning those services into a concrete frontend class and moving
  commit helpers into backend-owned classes.
- `TrackingPlanner`: tracking compatibility adapter. Tracking now has a
  dedicated backend service boundary and resolves `jerk_tracking` versus
  `snap_tracking` before optimizer execution. `JerkTrackingBackend` and
  `SnapTrackingBackend` are concrete backend dispatch classes; the legacy
  optimizer implementation is still called through the compatibility runtime.
- `PerchingPlanner`: contact trajectory compatibility adapter.
- `TakeoffPlanner`: dynamic takeoff/unperching compatibility adapter.
- `ExplorationMissionAdapter`: exploration mission compatibility adapter.
  Exploration-specific headers and sources now live under
  `general_core/exploration`. That work should extend the exploration task
  plugin without adding a second top-level ROS/FSM.

Combined missions compose these plugins. `TrackingPerching` should remain a
mission behavior:

```text
TrackTarget
  -> readiness(surface, distance, relative_speed, prediction_horizon)
  -> HandoverToPerching
  -> Perching
  -> ContactReached
```

## Six-Step Migration Status

The migration can be tracked as six major steps:

1. Semantic split: mostly complete. `MissionMode`, `TaskType`, `BackendType`,
   `ExecutionPhase`, `SafetyAction`, and `CommitAction` exist, and legacy task
   strings are parsed into the new model. Remaining work is removing places
   where legacy `TaskMode` still drives behavior directly.
2. Task plugin boundary: mostly complete as a compatibility layer.
   `StateToStatePlanner`, `TrackingPlanner`, `PerchingPlanner`,
   `TakeoffPlanner`, and `ExplorationMissionAdapter` exist and expose uniform
   plan/replan style calls. Remaining work is turning the compatibility
   adapters into fully independent task plugins with fewer `GeneralPlanner`
   service callbacks.
3. Backend extraction: complete for the compatibility architecture.
   State2state has a backend router,
   corridor/ESDF/plain outer plan/replan logic is in `state2state_task`, and
   SE3 is treated as a state2state backend. State2state backend dependencies
   now cross through dedicated services rather than `GeneralPlanner`
   inheritance. SE3 has its own backend helper, backup generation is now a
   state2state backend helper, and exp generation now lives in a state2state
   backend helper. State2state path search and ESDF guide endpoint preparation
   are explicit frontend services, and current-trajectory no-need safety uses
   shared runtime trajectory safety services. Tracking has an explicit backend
   service boundary and concrete `JerkTrackingBackend` / `SnapTrackingBackend`
   dispatch classes.
4. Mission orchestration: complete as a compatibility layer.
   `MissionOrchestrator` owns mission/task/backend/phase snapshots, and
   tracking-perching is represented as mission nodes. Full replacement of all
   legacy FSM branches is a post-architecture cleanup, not a blocker for the
   system boundary.
5. Safety and commit governance: complete as a compatibility layer.
   `SafetyMonitor`, `RuntimeSafetyPolicy`, runtime trajectory safety services,
   and `CommitGovernor` exist and are used by key FSM/state2state paths.
   Residual candidate-specific checks remain close to their task domains where
   they depend on task-local diagnostics and fallback behavior.
6. ROS adapter and deployment boundary: complete as a compatibility layer.
   ROS1 topic ownership has `RosAdapterContract`, release packaging/config work
   exists, and ROS adapters continue to preserve existing topic behavior.
   Further cleanup should reduce residual policy in `FsmRos1` without changing
   runtime behavior.

## Current Progress Snapshot

As of this step, the main state2state backend path is close to the target
modular boundary: corridor/ESDF/plain dispatch, SE3 generation, exp generation,
backup generation, state2state frontend path search, ESDF guide endpoint
preparation, and tracking backend service routing are outside direct
`GeneralPlanner` method bodies. Current-trajectory no-need safety also uses a
shared runtime trajectory safety service. The practical progress estimate is
about 95% for the system architecture scaffolding and compatibility runtime.
The remaining work is cleanup, not missing architecture: moving more legacy
tracking optimizer internals out of `GeneralPlanner`, reducing residual
task-specific commit checks, and replacing more FSM branches with pure mission
behavior objects.

## Refactor Progress Log

### 2026-06-29

Completed in the current migration step:

- Tracking task operations no longer require `friend` access to
  `GeneralPlanner`. `TrackingTaskServices` is now the explicit dependency
  boundary for tracking plan/replan and tracking-to-perching commit calls.
- State2state backend dispatch no longer calls
  `GeneralPlanner::PlanFromRest` / `ReplanOnce` directly. It routes through
  `state2state_task` operations.
- Corridor/ESDF/plain state2state `planFromRest` and `replanOnce` logic has
  been moved into `state2state_task` operations. `GeneralPlanner` keeps only
  compatibility wrappers plus `makeStateToStateTaskServices()`.
- SE3 plan/replan outer logic has also been moved into `state2state_task`
  operations. `GeneralPlanner::PlanSE3AggressiveFromRest` and
  `ReplanSE3AggressiveOnce` now only build services and forward the call.
- `state2state_plan_operations.hpp` now uses forward declarations instead of
  including the legacy trajectory/log headers directly.
- State2state, tracking, takeoff, and perching headers/sources now live under
  their domain folders in `general_core/{state2state,tracking,takeoff,perching}`.
  The old root-level task wrapper headers were removed. SE3 aggressive
  frontend/manager files were moved under `general_core/state2state` because
  SE3 is now a state2state backend.
- `StateToStateBackendContext` is now reduced to state2state goal mutation.
  Exp, backup, and SE3 no longer share the generic backend context.
- `GeneralPlanner` no longer inherits from `StateToStateBackendContext`. It
  owns a private state2state backend context adapter, so the compatibility
  boundary stays in the state2state domain while preserving existing runtime
  behavior.
- SE3 state2state optimization no longer sits on the generic
  `StateToStateBackendContext`. It now runs through
  `StateToStateSE3BackendServices` and `state2state_se3_backend.cpp`; the
  remaining legacy crossings are explicit head-state sampling and SE3
  trajectory commit through a state2state runtime adapter.
- State2state exp and backup generation no longer enter through the generic
  backend context. Exp generation uses `StateToStateExpBackendServices` and now
  lives in `state2state_exp_generation.cpp` as a backend helper. Backup
  generation lives in `state2state_backup_generation.cpp` as a backend helper.
  Both helpers use explicit map, corridor, FOV, optimizer, log, command, and
  timing services instead of broad `GeneralPlanner` access.
- `State2StateZDebug` moved from private `GeneralPlanner` helper state into
  the state2state backend domain. Exp generation now records path, optimized
  trajectory, and backup Z summaries through that explicit service field.
- The exp runtime adapter has been removed. `generateExpTraj` lives in the
  state2state backend helper, frontend path search and ESDF guide preparation
  use `StateToStateFrontendServices`, and current-trajectory no-need safety
  uses `RuntimeTrajectorySafetyServices`.
- State2state frontend path search and ESDF guide endpoint preparation moved
  from `GeneralPlanner` private methods into `StateToStateFrontendServices`.
  Exp generation now calls those frontend helpers directly through explicit
  services.
- Tracking plan/replan no longer stores the tracking optimizer as an ad-hoc
  `TrackingTaskServices` callback. It now resolves `jerk_tracking` or
  `snap_tracking` into `TrackingBackendServices` and runs through
  concrete `JerkTrackingBackend` / `SnapTrackingBackend` dispatch classes in
  `tracking_backend.cpp`.
- Utility naming has moved from `super_utils` to canonical `general_utils`.
  The public `map_manager/include/general_utils` headers define the canonical
  namespace, while `map_manager/include/super_utils` remains a compatibility
  include path for older downstream code.
- Exploration headers and sources now live under `general_core/exploration`.
  The exploration frontend has a configurable frontier source and can ingest
  ROG-Map frontier cells in `auto` mode while preserving the previous fallback
  local scan behavior.
- Exploration ATSP ordering now has its own
  `general_core/exploration/atsp` module. The implementation mirrors the
  `fy_node` ATSP file workflow (`single.tsp`, `single.par`, `single.txt`).
  The `fy_node` `lkh_tsp_solver` package has been migrated into this workspace,
  so `solver=fy_node_lkh` directly calls `solveTSPLKH()`, with external-command
  and deterministic greedy directed-tour fallbacks.
- Exploration anti-trap handling now uses frontier-selection diagnostics
  exported by `ExplorationFrontend`, saturated information gain for both normal
  scoring and ATSP entry costs, and a local-trap detector in
  `ExplorationRuntimeManager`. Repeated low-diversity, high-A* local attractors
  are written back to frontier failure memory before the pipeline asks for a
  validated recovery goal.
- Frontier failure memory now blocks a spatial region around failed candidates,
  not only the exact memory key. Recovery selection also filters the latest
  local-trap region, and the exploration guard runs an offline trap-signature
  analyzer so house-like frontier absorption can be detected from logs.
- Exploration runtime diagnostics now include recovery-chain counters, and
  `sh_files/test_exploration_trap_log_guard.sh` provides a direct offline
  regression check for historical trap logs or fresh trap-free runs.
- Shared generic helpers now have a `general_core/utils` home. The first helper
  is normalized string-token handling used by frontier-source and solver-mode
  dispatch.

Validation for this step:

- `git diff --check`
- Docker `ros1_noetic`: `catkin_make --pkg general_planner`
- Docker `ros1_noetic`: full workspace `catkin_make`
- Docker `ros1_noetic`: `GP_SKIP_BUILD=1 sh_files/test_general_planner_ros1.sh`
  covering `click_smooth_ros1.yaml`, `click_esdf_ros1.yaml`,
  `click_plain_ros1.yaml`, `tracking_tracker_drone1_ros1.yaml`, and
  `tracking_perching_chain_ros1.yaml`.
- Docker `ros1_noetic`: exploration smoke with `exploration.yaml`.
- Docker `ros1_noetic`: small ATSP file smoke through migrated
  `lkh_tsp_solver`, confirming a generated `single.txt` tour.
- Docker `ros1_noetic`: 90s `big_field` perfect-drone exploration guard after
  the diagnostic closure update: 218 selected goals, 218 planning successes, 0
  counted planning failures, 0 odom-stale events, 0 local-trap triggers in the
  normal open-field case, 3 validated memory recoveries, `trap_signature=0`,
  and PASS.
- Docker `ros1_noetic`: offline trap-log guard on the fresh big-field log with
  expected `trap_signature=0`: PASS. Offline trap-log guard on the provided
  pasted problem log with expected `trap_signature=1`: PASS.
- `sh_files/test_exploration_final_closure.sh` now provides the final closure
  entry point. Latest run: historical trap-log guard PASS, package build PASS,
  full workspace build PASS, 90s big-field exploration guard PASS with 218
  selected goals and 218 planning successes, fresh-log guard PASS, and optional
  ROS1 smoke tests PASS for corridor/esdf/plain state2state, tracking, and
  tracking-perching.

Post-architecture cleanup target:

- Reduce `GeneralPlanner` further by moving tracking optimizer internals and
  residual task-specific commit checks into concrete backend-owned classes.
- Replace more legacy FSM branches with mission behavior objects once runtime
  behavior snapshots are stable.
- Add live house-structure replay or a deterministic map-slice scenario test for
  the NDO trap case; the offline log guard is in place, but live closed-loop
  replay is still the stronger validation.
- Build a fy_node-style global exploration tour only if the task needs global
  optimality beyond the current memory-backed local/topology recovery behavior.
- Keep behavior-compatible ROS1 smoke tests after each step.
