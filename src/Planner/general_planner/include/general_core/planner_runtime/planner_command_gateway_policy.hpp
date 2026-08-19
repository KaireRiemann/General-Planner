#pragma once

#include <general_core/planner_runtime/planner_status.hpp>

namespace general_planner::planner_runtime {

// Keep source-selection policy independent from ROS plumbing so the safety
// behavior can be regression-tested without a running ROS master.
enum class GatewayOutputMode {
  NAVIGATION,
  EXPLORATION,
  EXPLICIT_HOLD,
  SOURCE_TIMEOUT_HOLD,
};

constexpr GatewayOutputMode selectGatewayOutputMode(
    const CommandOwner authorized_owner, const bool navigation_fresh,
    const bool exploration_fresh) {
  if (authorized_owner == CommandOwner::STATE2STATE) {
    return navigation_fresh ? GatewayOutputMode::NAVIGATION
                            : GatewayOutputMode::SOURCE_TIMEOUT_HOLD;
  }
  if (authorized_owner == CommandOwner::EXPLORATION) {
    return exploration_fresh ? GatewayOutputMode::EXPLORATION
                             : GatewayOutputMode::SOURCE_TIMEOUT_HOLD;
  }
  return GatewayOutputMode::EXPLICIT_HOLD;
}

} // namespace general_planner::planner_runtime
