#pragma once

namespace fast_planner {
namespace coverage_candidates {

// Candidate admission is independent of trajectory feasibility. In joint
// exploration, frontier and observed-free unknown approaches share one tour.
// Legacy fallback may delay NEW actions, but must never revoke an active one.
struct Admission {
  bool retain_active{false};
  bool admit_new{false};
  bool waiting{false};
};

inline Admission admission(bool enabled, bool joint, bool frontier_empty,
                           bool empty_stable, bool structural, bool active,
                           bool terminal_audit, bool moving_handoff,
                           double speed, double entry_speed) {
  Admission result;
  if (!enabled) return result;
  result.retain_active = active;
  result.admit_new = !terminal_audit &&
      (joint || (frontier_empty && (structural || empty_stable) &&
                 (moving_handoff || speed <= entry_speed)));
  result.waiting = frontier_empty && !terminal_audit && !result.admit_new;
  return result;
}

}  // namespace coverage_candidates
}  // namespace fast_planner
