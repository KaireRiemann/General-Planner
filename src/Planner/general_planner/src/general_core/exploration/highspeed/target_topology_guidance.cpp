#include <general_core/exploration/highspeed/target_topology_guidance.h>

#include <map_manager/map_manager.hpp>

namespace fast_planner {

const TargetTopologyGuide &TargetTopologyGuidance::update(
    const std::shared_ptr<general_planner::MapManager> &map_manager,
    const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
    const double now, const bool force) {
  if (!config_.enabled) {
    invalidate(TargetTopologyGuideStatus::DISABLED, goal, now);
    return guide_;
  }
  if (!map_manager) {
    invalidate(TargetTopologyGuideStatus::NO_MAP_MANAGER, goal, now);
    return guide_;
  }
  if (!start.allFinite() || !goal.allFinite()) {
    invalidate(TargetTopologyGuideStatus::NO_ROUTE, goal, now);
    return guide_;
  }

  const bool same_goal =
      guide_.goal.allFinite() &&
      (guide_.goal.cast<double>() - goal).norm() <=
          config_.goal_change_tolerance;
  if (!force && same_goal && std::isfinite(guide_.last_query_time) &&
      now - guide_.last_query_time < config_.query_interval) {
    return guide_;
  }
  const double query_time = std::isfinite(now) ? now : 0.0;
  if (!map_manager->topologyReady()) {
    invalidate(TargetTopologyGuideStatus::NO_TOPOLOGY, goal, query_time);
    return guide_;
  }
  const rog_map::Vec3f start_f = start;
  map_manager->requestTopologyUpdateAround(start_f);
  const auto snapshot = map_manager->topologySearchSnapshot();
  if (!snapshot || snapshot->graph.empty()) {
    invalidate(TargetTopologyGuideStatus::NO_SNAPSHOT, goal, query_time);
    return guide_;
  }

  rog_map::vec_Vec3f raw_route;
  const rog_map::Vec3f goal_f = goal;
  bool reaches_goal =
      map_manager->findTopologyPath(snapshot, start_f, goal_f, raw_route);
  if (!reaches_goal) {
    std::vector<TargetTopologyAnchorCandidate> candidates;
    candidates.reserve(snapshot->graph.size());
    for (const auto &entry : snapshot->graph) {
      const auto &node = entry.second.node;
      if (!node.position.allFinite()) {
        continue;
      }
      candidates.push_back({node.position.cast<float>(), node.expansion_mask});
    }
    const auto ranked =
        rankTargetTopologyAnchors(start, goal, candidates, config_);
    raw_route.clear();
    const int attempts = std::min<int>(
        config_.anchor_query_attempts, static_cast<int>(ranked.size()));
    for (int i = 0; i < attempts; ++i) {
      rog_map::vec_Vec3f anchor_route;
      if (!map_manager->findTopologyPath(
              snapshot, start_f, ranked[i].position.cast<double>(),
              anchor_route) ||
          anchor_route.size() < 2) {
        continue;
      }
      std::vector<Eigen::Vector3f> route;
      route.reserve(anchor_route.size());
      for (const auto &point : anchor_route) {
        route.emplace_back(point.cast<float>());
      }
      if (topologyRouteLength(route) + 1.0e-6 <
          config_.minimum_anchor_route_length) {
        continue;
      }
      raw_route = std::move(anchor_route);
      break;
    }
  }
  if (raw_route.size() < 2) {
    invalidate(TargetTopologyGuideStatus::NO_ROUTE, goal, query_time);
    return guide_;
  }

  guide_.valid = true;
  guide_.reaches_goal = reaches_goal;
  ++next_route_id_;
  guide_.route_id = next_route_id_;
  guide_.map_revision = map_manager->mapRevision();
  guide_.topology_revision = snapshot->revision;
  guide_.last_query_time = query_time;
  guide_.goal = goal.cast<float>();
  guide_.route.clear();
  guide_.route.reserve(raw_route.size());
  for (const auto &point : raw_route) {
    guide_.route.emplace_back(point.cast<float>());
  }
  guide_.anchor = guide_.route.back();
  if (!topologyRoutePointAtDistance(guide_.route,
                                    config_.local_prefix_length,
                                    guide_.local_prefix_point)) {
    invalidate(TargetTopologyGuideStatus::NO_ROUTE, goal, query_time);
    return guide_;
  }
  guide_.status = reaches_goal ? TargetTopologyGuideStatus::ROUTE_TO_GOAL
                               : TargetTopologyGuideStatus::PROGRESS_ANCHOR;
  return guide_;
}

void TargetTopologyGuidance::invalidate(const TargetTopologyGuideStatus status,
                                        const Eigen::Vector3d &goal,
                                        const double now) {
  const std::uint64_t route_id = guide_.route_id;
  guide_ = TargetTopologyGuide{};
  guide_.route_id = route_id;
  guide_.goal = goal.cast<float>();
  guide_.last_query_time = now;
  guide_.status = status;
}

}  // namespace fast_planner
