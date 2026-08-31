#include <general_core/planner_runtime/planner_status.hpp>

#include <cstring>
#include <iostream>
#include <stdexcept>

using general_planner::planner_runtime::ModeState;
using general_planner::planner_runtime::NavigationAdapterStatus;
using general_planner::planner_runtime::PlannerMode;
using general_planner::planner_runtime::PlannerTaskResult;
using general_planner::planner_runtime::isExplorationMode;
using general_planner::planner_runtime::isTargetExplorationMode;
using general_planner::planner_runtime::isNavigationAdapterMode;
using general_planner::planner_runtime::ownerForMode;
using general_planner::planner_runtime::modeStateFromExplorationString;
using general_planner::planner_runtime::modeStateFromNavigationString;
using general_planner::planner_runtime::modeStateFromTrackingString;
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
  expect(parsePlannerMode("target_exploration", mode),
         "parse target exploration");
  expect(mode == PlannerMode::TARGET_EXPLORATION,
         "mode target exploration");
  expect(isExplorationMode(mode), "target exploration uses exploration adapter");
  expect(isTargetExplorationMode(mode), "target exploration identity");
  expect(std::strcmp(toString(PlannerTaskResult::BLOCKED), "blocked") == 0,
         "toString blocked result");
  expect(parsePlannerMode("state2state", mode), "parse state2state");
  expect(mode == PlannerMode::STATE2STATE, "mode state2state");
  expect(parsePlannerMode("gate", mode), "parse gate");
  expect(mode == PlannerMode::GATE, "mode gate");
  expect(std::strcmp(toString(PlannerMode::GATE), "gate") == 0,
         "toString gate mode");
  expect(std::strcmp(toString(ownerForMode(PlannerMode::GATE)), "gate") == 0,
         "gate has external command owner");
  expect(parsePlannerMode("tracking", mode), "parse tracking");
  expect(mode == PlannerMode::TRACKING, "mode tracking");
  expect(isNavigationAdapterMode(mode), "tracking uses navigation adapter");
  expect(std::strcmp(toString(ownerForMode(mode)), "tracking") == 0,
         "tracking has isolated command owner");
  expect(parsePlannerMode("hold", mode), "parse hold");
  expect(mode == PlannerMode::HOLD, "mode hold");
  expect(!parsePlannerMode("not_a_mode", mode), "reject unknown");

  expect(std::strcmp(toString(PlannerMode::EXPLORATION), "exploration") == 0,
         "toString exploration");
  expect(std::strcmp(toString(PlannerMode::TARGET_EXPLORATION),
                     "target_exploration") == 0,
         "toString target exploration");
  expect(modeStateFromNavigationString("WAIT_GOAL") == ModeState::S2S_WAIT_GOAL,
         "nav wait goal");
  expect(modeStateFromExplorationString("WAITING_TARGET") ==
             ModeState::EXP_WAIT_TARGET,
         "target exploration wait target");
  expect(modeStateFromExplorationString("BLOCKED") == ModeState::EXP_PAUSED,
         "target exploration blocked");
  expect(modeStateFromNavigationString("FOLLOW_TRAJ") ==
             ModeState::S2S_FOLLOW_TRAJ,
         "nav follow");
  expect(modeStateFromTrackingString("STATIC_TRACKING") ==
             ModeState::TRACK_STATIC,
         "tracking static");
  expect(modeStateFromExplorationString("SUCCEEDED") == ModeState::EXP_PAUSED,
         "exp succeeded");
  expect(modeStateFromExplorationString("EXEC_TRAJ") == ModeState::EXP_EXEC_TRAJ,
         "exp exec");

  NavigationAdapterStatus navigation_status;
  expect(parseNavigationAdapterStatus("WAIT_GOAL 7 3 IDLE READY stage=idle", navigation_status),
         "parse lifecycle status");
  expect(navigation_status.state == "WAIT_GOAL", "lifecycle state");
  expect(navigation_status.task_epoch == 7, "lifecycle epoch");
  expect(navigation_status.goal_sequence == 3, "lifecycle goal sequence");
  expect(navigation_status.has_lifecycle && !navigation_status.goal_active,
         "lifecycle idle");
  expect(navigation_status.has_worker_readiness &&
             navigation_status.planning_worker_ready,
         "worker ready");
  expect(navigation_status.has_planning_stage &&
             navigation_status.planning_stage == "idle",
         "worker ready stage");
  expect(parseNavigationAdapterStatus("FOLLOW_TRAJ 7 3 ACTIVE BUSY stage=topology_query", navigation_status),
         "parse active lifecycle status");
  expect(navigation_status.has_lifecycle && navigation_status.goal_active,
         "lifecycle active");
  expect(navigation_status.has_worker_readiness &&
             !navigation_status.planning_worker_ready,
         "worker busy");
  expect(navigation_status.has_planning_stage &&
             navigation_status.planning_stage == "topology_query",
         "worker busy stage");
  expect(parseNavigationAdapterStatus("WAIT_GOAL 7", navigation_status),
         "parse legacy status");
  expect(!navigation_status.has_lifecycle, "legacy status remains nonterminal");

  std::cout << "planner_status_self_test passed\n";
  return 0;
}
