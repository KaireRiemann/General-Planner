#include <general_core/exploration_viewpoint_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace general_planner {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kInfCost = 1.0e9;

double pathLength(const super_utils::vec_E<super_utils::Vec3f> &path) {
    if (path.size() < 2) {
        return 0.0;
    }
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}

bool unknownLike(const rog_map::GridType type) {
    return type == rog_map::GridType::UNKNOWN ||
           type == rog_map::GridType::UNDEFINED ||
           type == rog_map::GridType::FRONTIER;
}
}  // namespace

ExplorationViewpointPlanner::ExplorationViewpointPlanner(const Config &cfg,
                                                         const MapManager::Ptr &map_manager,
                                                         const path_search::Astar::Ptr &astar)
        : cfg_(cfg),
          map_manager_(map_manager),
          astar_(astar) {
    cfg_.min_distance = std::max(0.1, cfg_.min_distance);
    cfg_.max_distance = std::max(cfg_.min_distance, cfg_.max_distance);
    cfg_.min_robot_distance = std::max(0.0, cfg_.min_robot_distance);
    cfg_.yaw_sample_num = std::max(4, cfg_.yaw_sample_num);
    cfg_.radius_sample_num = std::max(1, cfg_.radius_sample_num);
    cfg_.max_viewpoints_per_frontier = std::max(1, cfg_.max_viewpoints_per_frontier);
    cfg_.max_total_candidates = std::max(1, cfg_.max_total_candidates);
}

double ExplorationViewpointPlanner::wrapAngleDiff(const double lhs, const double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

bool ExplorationViewpointPlanner::viewpointSafe(const super_utils::Vec3f &pos) const {
    if (map_manager_ == nullptr || !map_manager_->ready() ||
        !pos.allFinite() || !map_manager_->insideLocalMap(pos)) {
        return false;
    }
    const rog_map::GridType grid_type = map_manager_->getGridType(pos);
    const rog_map::GridType inf_type = map_manager_->getInfGridType(pos);
    if (grid_type == rog_map::GridType::OCCUPIED ||
        grid_type == rog_map::GridType::OUT_OF_MAP ||
        inf_type == rog_map::GridType::OCCUPIED ||
        inf_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }
    if (cfg_.unknown_as_occupied_for_motion &&
        grid_type != rog_map::GridType::KNOWN_FREE) {
        return false;
    }
    if (map_manager_->hasESDF() && cfg_.safe_distance > 0.0) {
        double dist = 0.0;
        super_utils::Vec3f grad = super_utils::Vec3f::Zero();
        if (!map_manager_->evaluateESDF(pos, dist, grad) ||
            !std::isfinite(dist) ||
            dist < cfg_.safe_distance) {
            return false;
        }
    }
    return true;
}

bool ExplorationViewpointPlanner::insideYawFov(const super_utils::Vec3f &viewpoint,
                                               const double yaw,
                                               const super_utils::Vec3f &target) const {
    const super_utils::Vec3f diff = target - viewpoint;
    if (diff.head<2>().norm() < 1.0e-3) {
        return true;
    }
    const double target_yaw = std::atan2(diff.y(), diff.x());
    return std::abs(wrapAngleDiff(target_yaw, yaw)) <= 0.5 * kPi;
}

int ExplorationViewpointPlanner::countVisibleCells(const super_utils::Vec3f &viewpoint,
                                                   const double yaw,
                                                   const CompleteFrontierCluster &frontier) const {
    if (map_manager_ == nullptr) {
        return 0;
    }
    int visible = 0;
    const double max_range = std::max(6.0, cfg_.max_distance * 2.5);
    const double max_range_sq = max_range * max_range;
    for (const auto &cell : frontier.cells) {
        if ((cell.position - viewpoint).squaredNorm() > max_range_sq ||
            !insideYawFov(viewpoint, yaw, cell.position)) {
            continue;
        }
        if (map_manager_->insideLocalMap(cell.position) &&
            map_manager_->isLineFree(viewpoint, cell.position, true, false)) {
            ++visible;
        }
    }
    return visible;
}

double ExplorationViewpointPlanner::estimateTravelCost(const super_utils::Vec3f &robot_pos,
                                                       const super_utils::Vec3f &viewpoint,
                                                       super_utils::vec_E<super_utils::Vec3f> &guide_path) const {
    guide_path.clear();
    if (map_manager_ == nullptr || !robot_pos.allFinite() || !viewpoint.allFinite()) {
        return kInfCost;
    }
    const bool unknown_as_occ = cfg_.unknown_as_occupied_for_motion;
    if (map_manager_->insideLocalMap(robot_pos) &&
        map_manager_->insideLocalMap(viewpoint) &&
        map_manager_->isLineFree(robot_pos, viewpoint, true, unknown_as_occ)) {
        guide_path = {robot_pos, viewpoint};
        return (viewpoint - robot_pos).norm();
    }
    if (!cfg_.use_astar_cost || astar_ == nullptr ||
        !map_manager_->insideLocalMap(robot_pos) ||
        !map_manager_->insideLocalMap(viewpoint)) {
        return kInfCost;
    }
    const int flag = unknown_as_occ
                             ? (path_search::ON_PROB_MAP |
                                path_search::UNKNOWN_AS_OCCUPIED |
                                path_search::DONT_USE_INF_NEIGHBOR)
                             : (path_search::ON_INF_MAP |
                                path_search::UNKNOWN_AS_FREE);
    const double horizon = std::max(6.0, (viewpoint - robot_pos).norm() * 2.0 + 2.0);
    super_utils::vec_E<super_utils::Vec3f> path;
    const super_utils::RET_CODE ret =
            astar_->pointToPointPathSearch(robot_pos, viewpoint, flag, horizon, path, 0.02);
    if (ret != super_utils::REACH_GOAL || path.empty()) {
        return kInfCost;
    }
    guide_path = path;
    return pathLength(path);
}

double ExplorationViewpointPlanner::estimateYawCost(const double current_yaw,
                                                    const double candidate_yaw) const {
    return std::abs(wrapAngleDiff(candidate_yaw, current_yaw));
}

double ExplorationViewpointPlanner::estimateCurvatureCost(const super_utils::StatePVAJ &robot_state,
                                                          const super_utils::Vec3f &viewpoint) const {
    const super_utils::Vec3f robot_pos = robot_state.col(0);
    const super_utils::Vec3f robot_vel = robot_state.col(1);
    const super_utils::Vec3f to_goal = viewpoint - robot_pos;
    if (robot_vel.norm() < 0.1 || to_goal.norm() < 1.0e-3) {
        return 0.0;
    }
    const double align = std::max(-1.0, std::min(1.0,
            robot_vel.normalized().dot(to_goal.normalized())));
    return std::acos(align);
}

double ExplorationViewpointPlanner::estimateUnknownRisk(const super_utils::Vec3f &viewpoint) const {
    if (map_manager_ == nullptr) {
        return 1.0;
    }
    const double res = std::max(0.1, map_manager_->getResolution());
    int unknown_count = 0;
    int total = 0;
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const super_utils::Vec3f query =
                        viewpoint + res * super_utils::Vec3f(dx, dy, dz);
                if (!map_manager_->insideLocalMap(query)) {
                    continue;
                }
                ++total;
                if (unknownLike(map_manager_->getGridType(query))) {
                    ++unknown_count;
                }
            }
        }
    }
    return total == 0 ? 1.0 : static_cast<double>(unknown_count) / static_cast<double>(total);
}

bool ExplorationViewpointPlanner::sampleAndScore(const CompleteFrontierCluster &frontier,
                                                 const ExplorationRegionGraph &region_graph,
                                                 const CoveragePathPlanner &coverage_planner,
                                                 const super_utils::StatePVAJ &robot_state,
                                                 const double current_yaw,
                                                 std::vector<CompleteExplorationViewpoint> &out) const {
    if (frontier.status != FrontierStatus::ACTIVE ||
        frontier.cells.empty() ||
        map_manager_ == nullptr ||
        !map_manager_->ready()) {
        return false;
    }
    const std::size_t initial_size = out.size();
    const super_utils::Vec3f robot_pos = robot_state.col(0);
    std::vector<CompleteExplorationViewpoint> local_candidates;
    local_candidates.reserve(static_cast<std::size_t>(cfg_.yaw_sample_num * cfg_.radius_sample_num));

    super_utils::Vec3f preferred_dir = -frontier.unknown_direction;
    preferred_dir.z() = 0.0;
    if (preferred_dir.head<2>().norm() < 1.0e-3) {
        preferred_dir = robot_pos - frontier.center;
        preferred_dir.z() = 0.0;
    }
    const double base_angle = preferred_dir.head<2>().norm() > 1.0e-3
                                      ? std::atan2(preferred_dir.y(), preferred_dir.x())
                                      : 0.0;
    const int region_id = region_graph.regionOfFrontier(frontier.id);
    ExplorationRegion region;
    const bool has_region = region_graph.getRegion(region_id, region);
    const double revisit_penalty = has_region && region.visited ? 1.0 : 0.0;

    for (int r = 0; r < cfg_.radius_sample_num; ++r) {
        const double alpha = cfg_.radius_sample_num == 1
                                     ? 0.0
                                     : static_cast<double>(r) /
                                       static_cast<double>(cfg_.radius_sample_num - 1);
        const double radius = cfg_.min_distance +
                              alpha * (cfg_.max_distance - cfg_.min_distance);
        for (int i = 0; i < cfg_.yaw_sample_num; ++i) {
            const double angle = base_angle +
                                 2.0 * kPi * static_cast<double>(i) /
                                 static_cast<double>(cfg_.yaw_sample_num);
            super_utils::Vec3f viewpoint =
                    frontier.center + radius * super_utils::Vec3f(std::cos(angle), std::sin(angle), 0.0);
            viewpoint.z() += cfg_.height_offset;
            const double robot_distance = (viewpoint - robot_pos).norm();
            if (robot_distance < cfg_.min_robot_distance) {
                continue;
            }
            if (!viewpointSafe(viewpoint)) {
                continue;
            }
            if (cfg_.require_line_free_to_frontier &&
                !map_manager_->isLineFree(viewpoint, frontier.center, true, false)) {
                continue;
            }

            CompleteExplorationViewpoint candidate;
            candidate.frontier_id = frontier.id;
            candidate.region_id = region_id;
            candidate.position = viewpoint;
            candidate.yaw = std::atan2((frontier.center - viewpoint).y(),
                                       (frontier.center - viewpoint).x());
            const int visible = countVisibleCells(viewpoint, candidate.yaw, frontier);
            candidate.information_gain = static_cast<double>(visible);
            if (candidate.information_gain < cfg_.min_information_gain) {
                continue;
            }
            candidate.travel_cost = estimateTravelCost(robot_pos, viewpoint, candidate.guide_path);
            if (!std::isfinite(candidate.travel_cost) ||
                candidate.travel_cost >= kInfCost ||
                candidate.travel_cost < cfg_.min_robot_distance) {
                continue;
            }
            candidate.reachable = true;
            candidate.yaw_cost = estimateYawCost(current_yaw, candidate.yaw);
            candidate.curvature_cost = estimateCurvatureCost(robot_state, viewpoint);
            candidate.unknown_risk = estimateUnknownRisk(viewpoint);
            candidate.coverage_cost = region_id > 0 ? coverage_planner.orderCost(region_id) : 0.0;
            candidate.revisit_penalty = revisit_penalty;
            candidate.fail_penalty = static_cast<double>(frontier.fail_count);
            candidate.score =
                    cfg_.weight_travel * candidate.travel_cost +
                    cfg_.weight_yaw * candidate.yaw_cost +
                    cfg_.weight_curvature * candidate.curvature_cost +
                    cfg_.weight_unknown_risk * candidate.unknown_risk +
                    cfg_.weight_coverage_order * candidate.coverage_cost +
                    cfg_.weight_revisit * candidate.revisit_penalty +
                    cfg_.weight_fail * candidate.fail_penalty +
                    cfg_.weight_info_gain * candidate.information_gain;
            local_candidates.push_back(candidate);
        }
    }

    std::sort(local_candidates.begin(), local_candidates.end(),
              [](const CompleteExplorationViewpoint &lhs,
                 const CompleteExplorationViewpoint &rhs) {
                  if (std::abs(lhs.score - rhs.score) > 1.0e-9) return lhs.score < rhs.score;
                  if (lhs.frontier_id != rhs.frontier_id) return lhs.frontier_id < rhs.frontier_id;
                  if (lhs.position.x() != rhs.position.x()) return lhs.position.x() < rhs.position.x();
                  if (lhs.position.y() != rhs.position.y()) return lhs.position.y() < rhs.position.y();
                  return lhs.position.z() < rhs.position.z();
              });
    const std::size_t add_num = std::min<std::size_t>(
            local_candidates.size(),
            static_cast<std::size_t>(cfg_.max_viewpoints_per_frontier));
    for (std::size_t i = 0; i < add_num && out.size() < static_cast<std::size_t>(cfg_.max_total_candidates); ++i) {
        out.push_back(local_candidates[i]);
    }
    return out.size() > initial_size;
}

}  // namespace general_planner
