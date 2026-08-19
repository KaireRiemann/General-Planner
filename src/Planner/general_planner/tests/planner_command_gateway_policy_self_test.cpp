#include <general_core/planner_runtime/planner_command_gateway_policy.hpp>

#include <iostream>
#include <stdexcept>

using general_planner::planner_runtime::CommandOwner;
using general_planner::planner_runtime::GatewayOutputMode;
using general_planner::planner_runtime::selectGatewayOutputMode;

namespace {
void expect(const bool ok, const char *message) {
  if (!ok) {
    throw std::runtime_error(message);
  }
}
} // namespace

int main() {
  expect(selectGatewayOutputMode(CommandOwner::STATE2STATE, true, false) ==
             GatewayOutputMode::NAVIGATION,
         "fresh navigation must pass through");
  expect(selectGatewayOutputMode(CommandOwner::STATE2STATE, false, false) ==
             GatewayOutputMode::SOURCE_TIMEOUT_HOLD,
         "stale navigation must use source-timeout hold, not explicit hold");
  expect(selectGatewayOutputMode(CommandOwner::EXPLORATION, false, true) ==
             GatewayOutputMode::EXPLORATION,
         "fresh exploration must pass through");
  expect(selectGatewayOutputMode(CommandOwner::EXPLORATION, false, false) ==
             GatewayOutputMode::SOURCE_TIMEOUT_HOLD,
         "stale exploration must use source-timeout hold");
  expect(selectGatewayOutputMode(CommandOwner::HOLD, true, true) ==
             GatewayOutputMode::EXPLICIT_HOLD,
         "explicit hold must remain independent of source freshness");

  std::cout << "planner_command_gateway_policy_self_test passed\n";
  return 0;
}
