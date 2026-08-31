#include <general_core/planner_runtime/planner_command_gateway_policy.hpp>
#include <general_core/planner_runtime/topology_maintenance_policy.hpp>

#include <iostream>
#include <limits>
#include <stdexcept>

using general_planner::planner_runtime::CommandOwner;
using general_planner::planner_runtime::GatewayOutputMode;
using general_planner::planner_runtime::PlannerMode;
using general_planner::planner_runtime::PlannerPhase;
using general_planner::planner_runtime::selectGatewayOutputMode;
using general_planner::planner_runtime::shouldAbortCommandSource;
using general_planner::planner_runtime::shouldMaintainTopology;
using general_planner::planner_runtime::shouldMonitorCommandSource;

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
  expect(selectGatewayOutputMode(CommandOwner::TRACKING, true, false) ==
             GatewayOutputMode::NAVIGATION,
         "fresh tracking must pass through navigation adapter");
  expect(selectGatewayOutputMode(CommandOwner::TRACKING, false, false) ==
             GatewayOutputMode::SOURCE_TIMEOUT_HOLD,
         "stale tracking must use source-timeout hold");
  expect(selectGatewayOutputMode(CommandOwner::EXPLORATION, false, true) ==
             GatewayOutputMode::EXPLORATION,
         "fresh exploration must pass through");
  expect(selectGatewayOutputMode(CommandOwner::EXPLORATION, false, false) ==
             GatewayOutputMode::SOURCE_TIMEOUT_HOLD,
         "stale exploration must use source-timeout hold");
  expect(selectGatewayOutputMode(CommandOwner::HOLD, true, true) ==
             GatewayOutputMode::EXPLICIT_HOLD,
         "explicit hold must remain independent of source freshness");
  expect(selectGatewayOutputMode(CommandOwner::GATE, true, true) ==
             GatewayOutputMode::EXTERNAL_GATE_SUPPRESSED,
         "gate must suppress every runtime position-command source");

  // The authorization-to-first-command interval must not be interpreted as
  // an infinite-age stale source. The gateway still publishes a safe fallback
  // during this interval; only the supervisor task retirement is delayed.
  expect(!shouldAbortCommandSource(
             true, false, false, std::numeric_limits<double>::infinity(),
             0.01, 2.0, 1.0),
         "first-command grace must prevent immediate task abort");
  expect(shouldAbortCommandSource(
             true, false, false, std::numeric_limits<double>::infinity(),
             2.01, 2.0, 1.0),
         "missing first command must abort after its bounded grace");
  expect(!shouldAbortCommandSource(true, true, true, 0.01, 3.0, 2.0, 1.0),
         "fresh command must not abort");
  expect(shouldAbortCommandSource(true, true, false, 1.01, 3.0, 2.0, 1.0),
         "received command source must abort after stale timeout");

  // The bag regression: a goal was accepted and command ownership transferred
  // to state2state, but its initial plan never produced a first command. This
  // is PLANNING, not EXECUTING, and must still be bounded by the startup grace.
  expect(shouldMonitorCommandSource(PlannerPhase::PLANNING),
         "planning must monitor the first planner command");
  expect(shouldMonitorCommandSource(PlannerPhase::EXECUTING),
         "execution must monitor source freshness");
  expect(!shouldMonitorCommandSource(PlannerPhase::WAITING_INPUT),
         "idle input wait must not be treated as a source failure");

  expect(shouldMaintainTopology(),
         "global topology must remain active through hold and task handover");

  std::cout << "planner_command_gateway_policy_self_test passed\n";
  return 0;
}
