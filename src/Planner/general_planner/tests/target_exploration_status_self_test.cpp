#include <general_core/planner_runtime/target_exploration_status.hpp>
#include <iostream>
#include <stdexcept>
using namespace general_planner::planner_runtime;

int main() {
  TargetExplorationStatusProjector p;
  PlannerStatusData s;
  auto check = [&](TargetExplorationResult expected, bool busy = false,
                   bool hover = true) {
    if (p.update(s, busy, hover).result != expected)
      throw std::runtime_error("unexpected target status");
  };
  using R = TargetExplorationResult;
  check(R::FAILED); // Inactive mode never grants dispatch.
  s.active_mode = PlannerMode::TARGET_EXPLORATION;
  s.phase = PlannerPhase::WAITING_INPUT;
  s.ready_for_new_task = s.stable_hover = s.odom_valid = s.map_ready = true;
  check(R::READY);
  check(R::RUNNING, true); // Replacement/start acknowledgement pending.
  check(R::RUNNING, false, false); // Cached stable_hover is not enough.
  s.phase = PlannerPhase::PLANNING;
  check(R::RUNNING);
  s.task_result = PlannerTaskResult::SUCCEEDED;
  s.phase = PlannerPhase::HOLD_VERIFY;
  check(R::RUNNING, true);
  s.phase = PlannerPhase::STABLE_HOLD;
  check(R::SUCCEEDED);
  s.odom_valid = false;
  check(R::FAILED); // Success must not grant dispatch with stale odometry.
  s.odom_valid = true;
  check(R::SUCCEEDED);
  s.task_result = PlannerTaskResult::BLOCKED;
  check(R::FAILED);
  check(R::READY);
  ++s.task_epoch;
  check(R::FAILED); // Each new blocked task gets its own failure report.
  check(R::READY);
  s.task_result = PlannerTaskResult::FAILED;
  check(R::FAILED);
  check(R::FAILED); // No automatic hard-failure recovery.
  s.task_result = PlannerTaskResult::NONE;
  s.phase = PlannerPhase::WAITING_INPUT;
  check(R::READY);
  s.command_owner = CommandOwner::EXPLORATION;
  check(R::RUNNING);
  std::cout << "target_exploration_status_self_test passed\n";
}
