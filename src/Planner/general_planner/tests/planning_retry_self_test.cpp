#include <general_core/planning_retry_policy.hpp>
#include <general_core/planner_runtime/planner_status.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>

void expect(bool ok, const char *why) {
  if (!ok) { std::cerr << why << '\n'; std::exit(1); }
}
int main() {
  using namespace general_planner;
  expect(planningFailureLimit(0) == 5, "legacy zero cannot disable retry limit");
  expect(planningFailureLimit(-1) == 5, "negative retry limit bounded");
  expect(planningFailureLimit(3) == 3, "explicit limit preserved");
  expect(planningFailureLimit(10000) == 100, "excessive limit bounded");
  expect(planningRetryBackoff(0) >= .05, "zero backoff cannot busy loop");
  expect(planningRetryBackoff(std::numeric_limits<double>::quiet_NaN()) == .25,
         "invalid backoff has safe default");
  // Simulate deterministic fast failures on a 100 Hz FSM.
  double next = 0; int attempts = 0; bool failed = false;
  for (int tick = 0; tick < 1000; ++tick) {
    double now = tick * .01;
    if (failed || now < next) continue;
    ++attempts; next = now + planningRetryBackoff(.25);
    failed = attempts >= planningFailureLimit(0);
  }
  expect(attempts == 5 && failed, "repeated failures must terminate, not wrap counter");
  PlanningDeadline deadline;
  expect(!deadline.expired(true, 1, 1, 10, 5), "first attempt starts deadline");
  expect(!deadline.expired(true, 1, 1, 14.9, 5), "status updates cannot restart deadline");
  expect(deadline.expired(true, 1, 1, 15, 5), "blocked optimizer deadline");
  expect(!deadline.expired(true, 2, 1, 16, 5), "new epoch resets deadline");
  expect(!deadline.expired(true, 2, 2, 20, 5), "new accepted goal resets deadline");
  expect(!deadline.expired(false, 2, 2, 21, 5), "execution clears planning deadline");
  expect(!deadline.expired(true, 2, 2, 100, 5), "later planning starts a new episode");
  planner_runtime::NavigationAdapterStatus status;
  expect(planner_runtime::parseNavigationAdapterStatus("FAILED 42 3 IDLE", status),
         "failure protocol parses");
  expect(status.has_lifecycle && status.task_epoch == 42 &&
         status.goal_sequence == 3 && !status.goal_active && status.state == "FAILED",
         "failure retains task identity and cannot be success");
  expect(!status.quiescent, "legacy status cannot acknowledge worker exit");
  planner_runtime::parseNavigationAdapterStatus("FAILED 43 3 IDLE BUSY", status);
  expect(!status.quiescent, "busy optimizer cannot acknowledge exit");
  planner_runtime::parseNavigationAdapterStatus("FAILED 43 3 IDLE QUIESCENT", status);
  expect(status.quiescent, "stopped worker acknowledgement");
  planner_runtime::parseNavigationAdapterStatus("WAIT_GOAL 43 3 ACTIVE QUIESCENT", status);
  expect(!status.quiescent, "active goal cannot acknowledge stop");
  std::cout << "planning_retry_self_test passed\n";
}
