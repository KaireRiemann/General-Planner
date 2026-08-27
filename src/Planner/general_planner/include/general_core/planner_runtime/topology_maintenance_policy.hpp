#pragma once

#include <general_core/planner_runtime/planner_status.hpp>

namespace general_planner::planner_runtime {

/**
 * Topology maintenance mutates the persistent navigation graph; map fusion
 * does not. Freeze the former during every handover and terminal hold while
 * continuing to ingest odometry/cloud data into the world map.
 */
constexpr bool shouldMaintainTopology(
    const PlannerMode active_mode, const PlannerPhase phase,
    const bool transition_active, const bool maintain_during_stable_hold) {
  if (transition_active || phase == PlannerPhase::BOOT ||
      phase == PlannerPhase::BRAKING || phase == PlannerPhase::HOLD_VERIFY ||
      phase == PlannerPhase::FAILED || phase == PlannerPhase::EMERGENCY) {
    return false;
  }
  if (phase == PlannerPhase::STABLE_HOLD) {
    return maintain_during_stable_hold;
  }
  return active_mode != PlannerMode::HOLD &&
         active_mode != PlannerMode::EMERGENCY_STOP;
}

}  // namespace general_planner::planner_runtime
