#include <general_core/exploration/highspeed/target_directed_exploration.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace {

bool require(const bool condition, const char *message) {
  if (!condition) {
    std::cerr << "target_directed_exploration_self_test: " << message
              << std::endl;
  }
  return condition;
}

}  // namespace

int main() {
  using fast_planner::TargetDirectedExplorationConfig;
  using fast_planner::hasSufficientTargetProgress;
  using fast_planner::scoreTargetDirectedViewpoint;
  using fast_planner::targetArrivalSettled;

  TargetDirectedExplorationConfig config;
  config.heuristic_weight = 3.0;
  config.lateral_weight = 0.5;
  config.nominal_speed = 2.0;

  const Eigen::Vector3d start(0.0, 0.0, 1.5);
  const Eigen::Vector3d goal(100.0, 0.0, 1.5);
  const auto forward = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(18.0, 0.0, 1.5), config);
  const auto side_room = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(2.0, 18.0, 1.5), config);
  const auto regression = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(-8.0, 0.0, 1.5), config);

  if (!require(forward.valid && side_room.valid && regression.valid,
               "finite viewpoints were rejected") ||
      !require(forward.progress_distance > 0.0,
               "forward viewpoint has no progress") ||
      !require(regression.progress_distance < 0.0,
               "regression viewpoint has no negative progress") ||
      !require(forward.cost < side_room.cost,
               "side-room viewpoint outranked forward progress") ||
      !require(forward.cost < regression.cost,
               "backtracking viewpoint outranked forward progress") ||
      !require(hasSufficientTargetProgress(forward, 0.5),
               "forward bridge did not satisfy minimum progress") ||
      !require(!hasSufficientTargetProgress(regression, 0.5),
               "regressing bridge satisfied minimum progress")) {
    return 1;
  }

  // Bridge progress is evaluated from the current vehicle pose.  Once the
  // vehicle has reached x=60, a candidate at x=40 is still ahead of the
  // original mission start but is a regression for the next planning step.
  const Eigen::Vector3d current_pose(60.0, 0.0, 1.5);
  const auto stale_start_progress = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(40.0, 0.0, 1.5), config);
  const auto current_regression = scoreTargetDirectedViewpoint(
      current_pose, goal, Eigen::Vector3d(40.0, 0.0, 1.5), config);
  if (!require(hasSufficientTargetProgress(stale_start_progress, 0.5),
               "start-relative bridge setup is invalid") ||
      !require(!hasSufficientTargetProgress(current_regression, 0.5),
               "current-relative bridge filter accepted a regression")) {
    return 1;
  }

  TargetDirectedExplorationConfig flat_config = config;
  flat_config.vertical_weight = 0.0;
  const auto flat = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(18.0, 0.0, 10.0), flat_config);
  if (!require(std::fabs(flat.remaining_distance - 82.0) < 1.0e-6,
               "vertical weighting did not support 2D goal mode")) {
    return 1;
  }

  // Regression from runtime_20260817_100022: the vehicle first entered the
  // 0.50 m radius, then a handover brake settled 0.767 m from the target. A
  // task must remain active for a terminal correction rather than publishing
  // SUCCEEDED from that intermediate crossing.
  if (!require(targetArrivalSettled(0.164, 0.02, true, 0.50, 0.10),
               "settled in-tolerance target was rejected") ||
      !require(!targetArrivalSettled(0.767, 0.02, true, 0.50, 0.10),
               "post-brake target drift was accepted as success") ||
      !require(!targetArrivalSettled(0.164, 0.30, true, 0.50, 0.10),
               "moving target crossing was accepted as success") ||
      !require(!targetArrivalSettled(0.164, 0.02, false, 0.50, 0.10),
               "unfinished braking trajectory was accepted as success")) {
    return 1;
  }

  using fast_planner::TargetClusterCandidate;
  using fast_planner::selectPreferredTargetClusterIds;
  using fast_planner::shouldRetainTargetGoalLock;
  using fast_planner::targetDirectedSelectionCost;

  const std::vector<TargetClusterCandidate> clusters = {
      {1, Eigen::Vector3d(2.0, 18.0, 1.5)},
      {2, Eigen::Vector3d(40.0, 0.0, 1.5)},
      {3, Eigen::Vector3d(-8.0, 0.0, 1.5)},
      {4, Eigen::Vector3d(80.0, 1.0, 1.5)},
  };
  const auto preferred = selectPreferredTargetClusterIds(
      start, goal, clusters, config, 0.5, 2);
  if (!require(preferred.size() == 2,
               "destination shortlist did not keep two progressing clusters") ||
      !require(preferred[0] == 4,
               "cluster closest to the goal was not ranked first") ||
      !require(preferred[1] == 2,
               "next forward cluster was not retained")) {
    return 1;
  }

  const auto only_detours = selectPreferredTargetClusterIds(
      Eigen::Vector3d(90.0, 0.0, 1.5), goal,
      std::vector<TargetClusterCandidate>{
          {7, Eigen::Vector3d(70.0, 0.0, 1.5)},
          {8, Eigen::Vector3d(60.0, 12.0, 1.5)},
      },
      config, 0.5, 2);
  if (!require(only_detours.size() == 2,
               "detour shortlist was empty when no progressing cluster exists") ||
      !require(only_detours[0] == 7,
               "cheapest detour was not selected as the escape hatch")) {
    return 1;
  }

  const auto locked = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(18.0, 0.0, 1.5), config);
  const auto slightly_better = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(18.4, 0.0, 1.5), config);
  const auto much_better = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(40.0, 0.0, 1.5), config);
  const auto side = scoreTargetDirectedViewpoint(
      start, goal, Eigen::Vector3d(2.0, 18.0, 1.5), config);
  if (!require(shouldRetainTargetGoalLock(locked, slightly_better, 0.5, 1.0),
               "goal lock released a still-useful nearby bridge") ||
      !require(!shouldRetainTargetGoalLock(locked, much_better, 0.5, 1.0),
               "goal lock blocked a materially closer destination bridge") ||
      !require(!shouldRetainTargetGoalLock(side, locked, 0.5, 1.0),
               "goal lock kept a non-progressing side room over a forward bridge")) {
    return 1;
  }

  const double forward_cost = targetDirectedSelectionCost(forward, 4.0);
  const double side_cost = targetDirectedSelectionCost(side_room, 1.0);
  if (!require(forward_cost < side_cost,
               "cheap side-room travel outranked destination remaining cost")) {
    return 1;
  }

  std::cout << "target_directed_exploration_self_test: PASS" << std::endl;
  return 0;
}
