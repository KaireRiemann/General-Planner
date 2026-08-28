#pragma once

#include <general_core/planner_runtime/planner_status.hpp>

namespace general_planner::planner_runtime {

/**
 * The global topology belongs to the world model, not to an individual
 * planner task.  HOLD, handovers and emergency command ownership changes may
 * stop trajectories, but must never stop topology maintenance.  Locality is
 * enforced by the odometry-centred dirty-region window in MapManager.
 */
constexpr bool shouldMaintainTopology() {
  return true;
}

}  // namespace general_planner::planner_runtime
