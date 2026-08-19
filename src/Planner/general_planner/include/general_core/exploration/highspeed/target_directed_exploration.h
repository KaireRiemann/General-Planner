#pragma once

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace fast_planner {

// A target-directed exploration task never commands an unobserved mission
// point.  This helper only scores already executable frontier viewpoints: the
// normal frontier/topology/MINCO pipeline remains the safety authority.
struct TargetDirectedExplorationConfig {
  double heuristic_weight{3.0};
  double lateral_weight{0.15};
  double vertical_weight{1.0};
  double nominal_speed{2.0};
};

struct TargetDirectedExplorationScore {
  bool valid{false};
  double remaining_distance{0.0};
  double progress_distance{0.0};
  double lateral_distance{0.0};
  double cost{0.0};
};

// A frontier bridge is useful to a destination task only when it reduces the
// weighted start-to-goal distance by a meaningful amount.  The caller may
// still keep a non-progressing bridge as an escape hatch when *no* progressing
// candidate is currently reachable (for example, a U-shaped obstacle).
inline bool hasSufficientTargetProgress(
    const TargetDirectedExplorationScore &score,
    const double minimum_progress) {
  return score.valid && std::isfinite(minimum_progress) &&
         score.progress_distance >= std::max(0.0, minimum_progress);
}

// Entering the target radius while still moving is only an arrival candidate.
// A handover brake can carry the vehicle beyond the target, so a task may be
// reported as complete only after the braking trajectory has ended and the
// final stationary odometry remains inside the same weighted radius.
inline bool targetArrivalSettled(const double weighted_target_error,
                                 const double speed_mps,
                                 const bool trajectory_ended,
                                 const double target_reached_radius,
                                 const double max_settle_speed) {
  return std::isfinite(weighted_target_error) && std::isfinite(speed_mps) &&
         std::isfinite(target_reached_radius) &&
         std::isfinite(max_settle_speed) && trajectory_ended &&
         weighted_target_error <= std::max(0.0, target_reached_radius) &&
         std::abs(speed_mps) <= std::max(0.0, max_settle_speed);
}

inline TargetDirectedExplorationScore scoreTargetDirectedViewpoint(
    const Eigen::Vector3d &mission_start,
    const Eigen::Vector3d &mission_goal,
    const Eigen::Vector3d &viewpoint,
    TargetDirectedExplorationConfig config = {}) {
  TargetDirectedExplorationScore result;
  if (!mission_start.allFinite() || !mission_goal.allFinite() ||
      !viewpoint.allFinite()) {
    return result;
  }

  config.heuristic_weight = std::max(0.0, config.heuristic_weight);
  config.lateral_weight = std::max(0.0, config.lateral_weight);
  config.vertical_weight = std::max(0.0, config.vertical_weight);
  config.nominal_speed = std::max(0.1, config.nominal_speed);

  const Eigen::Vector3d scale(1.0, 1.0, config.vertical_weight);
  const Eigen::Vector3d start = mission_start.cwiseProduct(scale);
  const Eigen::Vector3d goal = mission_goal.cwiseProduct(scale);
  const Eigen::Vector3d candidate = viewpoint.cwiseProduct(scale);
  const Eigen::Vector3d goal_delta = goal - start;
  const double start_distance = goal_delta.norm();
  result.remaining_distance = (goal - candidate).norm();
  result.progress_distance = start_distance - result.remaining_distance;

  if (start_distance > 1.0e-6) {
    const Eigen::Vector3d direction = goal_delta / start_distance;
    const Eigen::Vector3d relative = candidate - start;
    result.lateral_distance =
        (relative - direction * relative.dot(direction)).norm();
  }
  result.cost =
      (config.heuristic_weight * result.remaining_distance +
       config.lateral_weight * result.lateral_distance) /
      config.nominal_speed;
  result.valid = std::isfinite(result.cost);
  return result;
}

inline double targetDirectedSelectionCost(
    const TargetDirectedExplorationScore &score, const double travel_time) {
  if (!score.valid || !std::isfinite(travel_time)) {
    return std::numeric_limits<double>::infinity();
  }
  return travel_time + score.cost;
}

// Keep a locked frontier bridge only when it is still a destination-useful
// option.  A newly discovered progressing bridge with a materially smaller
// remaining distance must be allowed to replace it.
inline bool shouldRetainTargetGoalLock(
    const TargetDirectedExplorationScore &locked,
    const TargetDirectedExplorationScore &candidate,
    const double minimum_progress, const double remaining_margin) {
  if (!locked.valid) {
    return false;
  }
  if (!candidate.valid) {
    return true;
  }
  const bool locked_progress =
      hasSufficientTargetProgress(locked, minimum_progress);
  const bool candidate_progress =
      hasSufficientTargetProgress(candidate, minimum_progress);
  if (candidate_progress && !locked_progress) {
    return false;
  }
  return locked.remaining_distance <=
         candidate.remaining_distance + std::max(0.0, remaining_margin);
}

struct TargetClusterCandidate {
  int id{-1};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

// Inject up to max_count geometrically useful clusters into the local
// viewpoint shortlist.  Progressing clusters are preferred; if none exist,
// the cheapest detours are returned so a U-shaped obstacle still has an
// escape hatch.
inline std::vector<int> selectPreferredTargetClusterIds(
    const Eigen::Vector3d &current_pose, const Eigen::Vector3d &mission_goal,
    const std::vector<TargetClusterCandidate> &clusters,
    TargetDirectedExplorationConfig config, const double minimum_progress,
    int max_count) {
  max_count = std::max(0, max_count);
  struct RankedCluster {
    int id{-1};
    TargetDirectedExplorationScore score;
  };
  std::vector<RankedCluster> progressing;
  std::vector<RankedCluster> detours;
  progressing.reserve(clusters.size());
  detours.reserve(clusters.size());
  for (const TargetClusterCandidate &cluster : clusters) {
    if (cluster.id < 0 || !cluster.position.allFinite()) {
      continue;
    }
    const TargetDirectedExplorationScore score = scoreTargetDirectedViewpoint(
        current_pose, mission_goal, cluster.position, config);
    if (!score.valid) {
      continue;
    }
    RankedCluster ranked{cluster.id, score};
    if (hasSufficientTargetProgress(score, minimum_progress)) {
      progressing.emplace_back(ranked);
    } else {
      detours.emplace_back(ranked);
    }
  }
  const auto by_destination_cost = [](const RankedCluster &first,
                                      const RankedCluster &second) {
    if (first.score.cost != second.score.cost) {
      return first.score.cost < second.score.cost;
    }
    if (first.score.remaining_distance != second.score.remaining_distance) {
      return first.score.remaining_distance < second.score.remaining_distance;
    }
    return first.id < second.id;
  };
  std::stable_sort(progressing.begin(), progressing.end(), by_destination_cost);
  std::stable_sort(detours.begin(), detours.end(), by_destination_cost);

  std::vector<int> ids;
  ids.reserve(static_cast<std::size_t>(max_count));
  const std::vector<RankedCluster> &primary =
      !progressing.empty() ? progressing : detours;
  for (const RankedCluster &ranked : primary) {
    if (static_cast<int>(ids.size()) >= max_count) {
      break;
    }
    ids.emplace_back(ranked.id);
  }
  return ids;
}

}  // namespace fast_planner
