# Exploration direct-launch A/B record — 2026-07-19

## Scope and test entry

Both completed runs used the same house profile and direct launch entry (no bag replay):

```bash
roslaunch task_planner exploration.launch \
  rviz:=false auto_start:=true view_frontier:=false view_cluster:=false
```

The processes ran in Docker container `ros1_noetic`, with the workspace built at
`/root/ws/real_planner`.

Baseline artifacts:

- `baseline.log`
- `baseline_exploration_house.yaml`
- `baseline_worktree.patch`

Final artifacts:

- `optimized_final.log`

Intermediate, rejected test artifacts:

- `optimized_aggressive.log`: allowed coverage candidates while ordinary
  frontiers remained. It finished, but stole work from the stable frontier
  frontend and was rejected.
- `optimized_raw_gate_failed.log`: promotion depended on raw active-cluster
  count. One cached but unreachable cluster (`active=1, reachable=0`) blocked
  coverage execution and FINISH, so the gate was corrected to use executable
  reachability.
- `optimized_safety_gate_failed.log`: exposed a CAUTION threshold dead band at
  clearance `0.608294 m`; the FSM required `0.62 m`, while the recovery backend
  returned no recovery above `0.60 m`. Both sides now use `0.62 m`.
- `optimized.log`: first unified-candidate run. It finished in 439.241 s, but
  allowed coverage/frontier interleaving and was rejected.
- `optimized_strict.log`: strict post-cooldown empty-set gate. It finished in
  372.056 s and removed mixed pools, but transient empty cycles still triggered
  seven audits.
- `optimized_strict_final.log`: stopped diagnostic run; a generated frontier
  viewpoint had no topology path, exposing that executability must be checked
  after topology reachability.
- `optimized_strict_topology.log`: stopped diagnostic run; it exposed a
  stationary high-clearance local minimum with repeated distant-goal MINCO
  failures. The final version adds bounded known-free spatial relocation.

## Comparable completed-run results

Mission duration is measured from the first global planning timestamp to the
FINISH convergence timestamp.

| Metric | Baseline | Final |
|---|---:|---:|
| Entered FINISH | yes | yes |
| Mission duration | 326.381 s | 352.003 s |
| FINISH coverage | 0.8753 | 0.8774 |
| FINISH plateau | 20.7 s | 20.0 s |
| Exhausted actionable observations | 24/24 | 28/28 |
| Forced frontier audits | 146 | 2 |
| Final frontier semantic revision | 1857 | 458 |
| Final frontier-storage estimate | 158.188 KiB | 158.285 KiB |
| Successful trajectory publications | 242 | 226 |
| Coverage trajectory failures | 26 | 14 |
| Occluded observation outcomes | 44 | 25 |
| Known-free spatial relocations | 0 | 2 |
| Crash / abnormal node death | 0 | 0 |

Interpretation:

- The final run completed the stable FINISH protocol: no executable frontend
  frontier, 20.0 seconds without meaningful coverage growth, and 28/28
  actionable observations exhausted.
- Semantic revisioning removes callback-driven audit churn: forced audits fell
  from 146 to 2 (98.6%). The final semantic revision fell from 1857 to 458
  (75.3%); completed optimized runs vary from 394 to 458 because they discover
  different real cluster transitions, while the callback-driven increment
  mechanism remains removed.
- Storage stayed bounded and effectively unchanged at mission end; the change is
  about lifecycle/cache bounds and revision semantics, not hiding data by
  deleting the persistent label map.
- The final run is 25.622 seconds (7.85%) slower than the baseline while ending
  0.21 percentage points higher in coverage. It recovered 87.14 seconds from
  the rejected 439.241-second implementation and stays inside the 10% direct-run
  efficiency acceptance band.

## Implemented behavior

1. Coverage groups expose a deterministic component ID and at most four
   deduplicated known-free approach candidates.
2. Coverage observations enter a strict coverage-only topology pool only after
   the post-cooldown executable frontier set remains empty for four cycles and
   1.5 seconds. A second reachability check handles generated viewpoints with no
   topology path. At most eight observations are evaluated per cycle.
3. Every promoted observation uses the same clearance, ray visibility,
   topology reachability, edge cost, goal lock, global route, corridor, MINCO,
   backup and commit-safety pipeline as an ordinary frontier.
4. Deferred/reached/timeout/unsafe/occluded/disconnected observations are
   tracked by deterministic ID with a spatial fallback and a hard 256-entry
   deferred-goal bound.
5. Frontier revision changes only for a quantized semantic cluster change.
   Per-cycle derived viewpoint caches are released, while persistent label and
   candidate state remains available.
6. Invalid frontier normals and transient label inconsistencies no longer call
   `exit(1)`; the affected cell is safely discarded or reconsidered.
7. CAUTION entry, recovery candidate acceptance and CAUTION exit use the same
   restored-clearance threshold.
8. Twelve consecutive stationary local-plan failures trigger a bounded
   approximately 1.2 m relocation inside known-free space. This escapes a
   high-clearance spatial local minimum without weakening collision clearance.

## Validation

Final Docker build completed for:

- `exploration_node`
- `highspeed_traj_server`

Final self-tests (9/9 passed):

- `boundary_map_self_test`
- `nhbp_route_intent_self_test`
- `route_intent_selector_self_test`
- `state2state_altitude_corridor_self_test`
- `state2state_route_goal_generator_self_test`
- `coverage_guidance_self_test`
- `yaw_candidate_selector_self_test`
- `goal_height_policy_self_test`
- `inf_map_bounds_self_test`

`git diff --check` passed, and the final direct-launch log contains no
segmentation fault, abort, core dump, or ROS process-death record.
