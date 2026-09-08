#pragma once

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace general_planner {
class MapManager;
}

namespace fast_planner {

// This is deliberately a guide, never a trajectory authority.  The global
// MapManager topology is persistent and can see beyond HighSpeedExp's rolling
// Bubble-Topo map, but every selected point still goes through the local
// Bubble-A*, known-free and MINCO checks before it can be executed.
struct TargetTopologyGuidanceConfig {
  bool enabled{true};
  double query_interval{1.0};
  double local_prefix_length{12.0};
  double goal_change_tolerance{0.5};
  double minimum_progress{0.5};
  double progress_weight{1.0};
  double lateral_penalty{0.15};
  double radial_penalty{0.10};
  double remaining_penalty{0.05};
  double expansion_bonus{2.0};
  int anchor_candidate_count{8};
  int anchor_query_attempts{4};
  double minimum_anchor_route_length{1.0};
  double vertical_weight{1.0};
};

enum class TargetTopologyGuideStatus {
  DISABLED,
  NO_MAP_MANAGER,
  NO_TOPOLOGY,
  NO_SNAPSHOT,
  ROUTE_TO_GOAL,
  PROGRESS_ANCHOR,
  NO_ROUTE,
  QUERY_RATE_LIMIT,
  RESET
};

inline const char *targetTopologyGuideStatusName(
    const TargetTopologyGuideStatus status) {
  switch (status) {
    case TargetTopologyGuideStatus::DISABLED:
      return "DISABLED";
    case TargetTopologyGuideStatus::NO_MAP_MANAGER:
      return "NO_MAP_MANAGER";
    case TargetTopologyGuideStatus::NO_TOPOLOGY:
      return "NO_TOPOLOGY";
    case TargetTopologyGuideStatus::NO_SNAPSHOT:
      return "NO_SNAPSHOT";
    case TargetTopologyGuideStatus::ROUTE_TO_GOAL:
      return "ROUTE_TO_GOAL";
    case TargetTopologyGuideStatus::PROGRESS_ANCHOR:
      return "PROGRESS_ANCHOR";
    case TargetTopologyGuideStatus::NO_ROUTE:
      return "NO_ROUTE";
    case TargetTopologyGuideStatus::QUERY_RATE_LIMIT:
      return "QUERY_RATE_LIMIT";
    case TargetTopologyGuideStatus::RESET:
      return "RESET";
  }
  return "UNKNOWN";
}

struct TargetTopologyAnchorCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};
  std::uint8_t expansion_mask{0};
};

struct TargetTopologyAnchorScore {
  bool valid{false};
  double progress{0.0};
  double lateral_distance{0.0};
  double radial_distance{0.0};
  double remaining_distance{0.0};
  double score{-std::numeric_limits<double>::infinity()};
};

inline TargetTopologyAnchorScore scoreTargetTopologyAnchor(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const TargetTopologyAnchorCandidate &candidate,
    TargetTopologyGuidanceConfig config = {}) {
  TargetTopologyAnchorScore result;
  if (!start.allFinite() || !goal.allFinite() ||
      !candidate.position.allFinite()) {
    return result;
  }
  config.vertical_weight = std::max(0.0, config.vertical_weight);
  const Eigen::Vector3d scale(1.0, 1.0, config.vertical_weight);
  const Eigen::Vector3d scaled_start = start.cwiseProduct(scale);
  const Eigen::Vector3d scaled_goal = goal.cwiseProduct(scale);
  const Eigen::Vector3d scaled_candidate =
      candidate.position.cast<double>().cwiseProduct(scale);
  const Eigen::Vector3d goal_delta = scaled_goal - scaled_start;
  const double start_to_goal = goal_delta.norm();
  if (start_to_goal <= 1.0e-6) {
    return result;
  }
  const Eigen::Vector3d candidate_delta = scaled_candidate - scaled_start;
  result.radial_distance = candidate_delta.norm();
  result.remaining_distance = (scaled_goal - scaled_candidate).norm();
  result.progress = start_to_goal - result.remaining_distance;
  const Eigen::Vector3d direction = goal_delta / start_to_goal;
  result.lateral_distance =
      (candidate_delta - direction * candidate_delta.dot(direction)).norm();
  result.score = std::max(0.0, config.progress_weight) * result.progress -
                 std::max(0.0, config.lateral_penalty) *
                     result.lateral_distance -
                 std::max(0.0, config.radial_penalty) *
                     result.radial_distance -
                 std::max(0.0, config.remaining_penalty) *
                     result.remaining_distance +
                 (candidate.expansion_mask != 0
                      ? std::max(0.0, config.expansion_bonus)
                      : 0.0);
  result.valid = std::isfinite(result.score);
  return result;
}

// Rank persistent-topology anchors before their attachment is checked.  A
// forward expanding portal is preferred, while detours are retained only when
// no forward anchor exists so a U-shaped obstacle does not deadlock the task.
inline std::vector<TargetTopologyAnchorCandidate>
rankTargetTopologyAnchors(
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const std::vector<TargetTopologyAnchorCandidate> &candidates,
    TargetTopologyGuidanceConfig config = {}) {
  struct Ranked {
    TargetTopologyAnchorCandidate candidate;
    TargetTopologyAnchorScore score;
  };
  std::vector<Ranked> progressing;
  std::vector<Ranked> detours;
  progressing.reserve(candidates.size());
  detours.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    const auto score = scoreTargetTopologyAnchor(start, goal, candidate, config);
    if (!score.valid) {
      continue;
    }
    if (score.progress >= std::max(0.0, config.minimum_progress)) {
      progressing.push_back({candidate, score});
    } else {
      detours.push_back({candidate, score});
    }
  }
  const auto compare = [](const Ranked &lhs, const Ranked &rhs) {
    if (lhs.score.score != rhs.score.score) {
      return lhs.score.score > rhs.score.score;
    }
    if (lhs.score.remaining_distance != rhs.score.remaining_distance) {
      return lhs.score.remaining_distance < rhs.score.remaining_distance;
    }
    return lhs.score.radial_distance < rhs.score.radial_distance;
  };
  std::stable_sort(progressing.begin(), progressing.end(), compare);
  std::stable_sort(detours.begin(), detours.end(), compare);
  const auto &primary = progressing.empty() ? detours : progressing;
  const int max_count = std::max(0, config.anchor_candidate_count);
  std::vector<TargetTopologyAnchorCandidate> result;
  result.reserve(std::min<int>(max_count, static_cast<int>(primary.size())));
  for (const auto &ranked : primary) {
    if (static_cast<int>(result.size()) >= max_count) {
      break;
    }
    result.emplace_back(ranked.candidate);
  }
  return result;
}

inline double topologyRouteLength(const std::vector<Eigen::Vector3f> &route) {
  double length = 0.0;
  for (std::size_t i = 1; i < route.size(); ++i) {
    length += (route[i] - route[i - 1]).norm();
  }
  return length;
}

inline bool topologyRoutePointAtDistance(const std::vector<Eigen::Vector3f> &route,
                                         const double distance,
                                         Eigen::Vector3f &point) {
  if (route.empty() || !std::isfinite(distance)) {
    return false;
  }
  if (route.size() == 1 || distance <= 0.0) {
    point = route.front();
    return point.allFinite();
  }
  double remaining = distance;
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector3f delta = route[i] - route[i - 1];
    const double segment = delta.norm();
    if (segment <= 1.0e-6) {
      continue;
    }
    if (remaining <= segment) {
      point = route[i - 1] + delta * static_cast<float>(remaining / segment);
      return point.allFinite();
    }
    remaining -= segment;
  }
  point = route.back();
  return point.allFinite();
}

struct TargetTopologyGuide {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid{false};
  bool reaches_goal{false};
  std::uint64_t route_id{0};
  std::uint64_t map_revision{0};
  std::uint64_t topology_revision{0};
  double last_query_time{-std::numeric_limits<double>::infinity()};
  Eigen::Vector3f goal{Eigen::Vector3f::Zero()};
  Eigen::Vector3f anchor{Eigen::Vector3f::Zero()};
  // A bounded point along the global route.  It is only a preferred local
  // direction; the candidate is injected only after local known-free checks.
  Eigen::Vector3f local_prefix_point{Eigen::Vector3f::Zero()};
  std::vector<Eigen::Vector3f> route;
  TargetTopologyGuideStatus status{TargetTopologyGuideStatus::RESET};
};

class TargetTopologyGuidance {
 public:
  void configure(const TargetTopologyGuidanceConfig &config) {
    config_ = config;
    config_.query_interval = std::clamp(config_.query_interval, 0.0, 30.0);
    config_.local_prefix_length =
        std::clamp(config_.local_prefix_length, 1.0, 50.0);
    config_.goal_change_tolerance =
        std::clamp(config_.goal_change_tolerance, 0.05, 10.0);
    config_.minimum_progress =
        std::clamp(config_.minimum_progress, 0.0, 20.0);
    config_.anchor_candidate_count =
        std::clamp(config_.anchor_candidate_count, 1, 32);
    config_.anchor_query_attempts =
        std::clamp(config_.anchor_query_attempts, 1, 16);
    config_.minimum_anchor_route_length =
        std::clamp(config_.minimum_anchor_route_length, 0.0, 20.0);
    config_.vertical_weight = std::clamp(config_.vertical_weight, 0.0, 10.0);
  }

  void reset() {
    guide_ = TargetTopologyGuide{};
    guide_.status = TargetTopologyGuideStatus::RESET;
  }

  const TargetTopologyGuide &guide() const { return guide_; }

  const TargetTopologyGuide &update(
      const std::shared_ptr<general_planner::MapManager> &map_manager,
      const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
      const double now, bool force = false);

 private:
  void invalidate(const TargetTopologyGuideStatus status,
                  const Eigen::Vector3d &goal, double now);

  TargetTopologyGuidanceConfig config_;
  TargetTopologyGuide guide_;
  std::uint64_t next_route_id_{0};
};

}  // namespace fast_planner
