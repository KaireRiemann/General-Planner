#include <general_core/planner_runtime/planner_status.hpp>

#include <cstring>
#include <iostream>
#include <stdexcept>

using general_planner::planner_runtime::ModeState;
using general_planner::planner_runtime::NavigationAdapterStatus;
using general_planner::planner_runtime::PlannerMode;
using general_planner::planner_runtime::modeStateFromExplorationString;
using general_planner::planner_runtime::modeStateFromNavigationString;
using general_planner::planner_runtime::parseNavigationAdapterStatus;
using general_planner::planner_runtime::parsePlannerMode;
using general_planner::planner_runtime::toString;

namespace {
void expect(const bool ok, const char *msg) {
  if (!ok) {
    throw std::runtime_error(msg);
  }
}
} // namespace

int main() {
  PlannerMode mode = PlannerMode::HOLD;
  expect(parsePlannerMode("exploration", mode), "parse exploration");
  expect(mode == PlannerMode::EXPLORATION, "mode exploration");
  expect(parsePlannerMode("state2state", mode), "parse state2state");
  expect(mode == PlannerMode::STATE2STATE, "mode state2state");
  expect(parsePlannerMode("hold", mode), "parse hold");
  expect(mode == PlannerMode::HOLD, "mode hold");
  expect(!parsePlannerMode("not_a_mode", mode), "reject unknown");

  expect(std::strcmp(toString(PlannerMode::EXPLORATION), "exploration") == 0,
         "toString exploration");
  expect(modeStateFromNavigationString("WAIT_GOAL") == ModeState::S2S_WAIT_GOAL,
         "nav wait goal");
  expect(modeStateFromNavigationString("FOLLOW_TRAJ") ==
             ModeState::S2S_FOLLOW_TRAJ,
         "nav follow");
  expect(modeStateFromExplorationString("SUCCEEDED") == ModeState::EXP_PAUSED,
         "exp succeeded");
  expect(modeStateFromExplorationString("EXEC_TRAJ") == ModeState::EXP_EXEC_TRAJ,
         "exp exec");

  NavigationAdapterStatus navigation_status;
  expect(parseNavigationAdapterStatus("WAIT_GOAL 7 3 IDLE", navigation_status),
         "parse lifecycle status");
  expect(navigation_status.state == "WAIT_GOAL", "lifecycle state");
  expect(navigation_status.task_epoch == 7, "lifecycle epoch");
  expect(navigation_status.goal_sequence == 3, "lifecycle goal sequence");
  expect(navigation_status.has_lifecycle && !navigation_status.goal_active,
         "lifecycle idle");
  expect(parseNavigationAdapterStatus("FOLLOW_TRAJ 7 3 ACTIVE", navigation_status),
         "parse active lifecycle status");
  expect(navigation_status.has_lifecycle && navigation_status.goal_active,
         "lifecycle active");
  expect(parseNavigationAdapterStatus("WAIT_GOAL 7", navigation_status),
         "parse legacy status");
  expect(!navigation_status.has_lifecycle, "legacy status remains nonterminal");

  std::cout << "planner_status_self_test passed\n";
  return 0;
}
