#include <general_core/exploration_frontend.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using namespace general_utils;

namespace general_planner {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kInfCost = 1.0e9;

struct GridKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const GridKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHasher {
    std::size_t operator()(const GridKey &key) const {
        std::size_t seed = 0;
        const auto mix = [&seed](const int value) {
            const std::size_t h = std::hash<int>{}(value);
            seed ^= h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
        };
        mix(key.x);
        mix(key.y);
        mix(key.z);
        return seed;
    }
};

GridKey makeKey(const Vec3i &id) {
    return GridKey{id.x(), id.y(), id.z()};
}

GridKey makeBucketKey(const Vec3f &pos, const double bucket_size) {
    return GridKey{
            static_cast<int>(std::floor(pos.x() / bucket_size)),
            static_cast<int>(std::floor(pos.y() / bucket_size)),
            static_cast<int>(std::floor(pos.z() / bucket_size))};
}

double angleBetween(Vec3f lhs, Vec3f rhs) {
    lhs.z() = 0.0;
    rhs.z() = 0.0;
    const double lhs_norm = lhs.norm();
    const double rhs_norm = rhs.norm();
    if (lhs_norm < 1.0e-6 || rhs_norm < 1.0e-6) {
        return 0.0;
    }
    const double c = std::clamp(lhs.dot(rhs) / (lhs_norm * rhs_norm), -1.0, 1.0);
    return std::acos(c);
}

double pathLength(const vec_E<Vec3f> &path) {
    if (path.size() < 2) {
        return 0.0;
    }
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}
}  // namespace

ExplorationFrontend::ExplorationFrontend(const Config &cfg,
                                         const MapManager::Ptr &map_manager,
                                         const path_search::Astar::Ptr &astar)
        : cfg_(cfg),
          map_manager_(map_manager),
          astar_(astar) {
}

bool ExplorationFrontend::planNextGoal(const StatePVAJ &robot_state,
                                       const double current_yaw,
                                       ExplorationGoal &goal) {
    goal = ExplorationGoal{};
    exploration_finished_ = false;

    if (!cfg_.enable) {
        goal.reason = "exploration disabled";
        return false;
    }
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        goal.reason = "map manager not ready";
        return false;
    }

    const Vec3f robot_pos = robot_state.col(0);
    if (!robot_pos.allFinite()) {
        goal.reason = "robot state is not finite";
        return false;
    }
    if (!map_manager_->insideLocalMap(robot_pos)) {
        goal.reason = "robot is outside local map";
        return false;
    }

    vec_E<FrontierCell> frontier_cells;
    FrontierSearchStats search_stats;
    if (!collectFrontierCells(robot_pos, frontier_cells, search_stats)) {
        goal.reason = "frontier search failed";
        return false;
    }
    if (frontier_cells.empty()) {
        if (!mapObservationReady(search_stats)) {
            exploration_finished_ = false;
            std::ostringstream oss;
            oss << "map observation not ready"
                << " searched=" << search_stats.searched_cells
                << " free=" << search_stats.known_free_cells
                << " unknown=" << search_stats.unknown_cells
                << " occupied=" << search_stats.occupied_cells;
            goal.reason = oss.str();
            if (cfg_.print_log) {
                std::cout << " -- [ExplorationFrontend] Waiting for usable map: "
                          << goal.reason << "." << std::endl;
            }
            return false;
        }
        exploration_finished_ = true;
        goal.reason = "no frontier";
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] Exploration finished: no frontier. "
                      << "searched=" << search_stats.searched_cells
                      << ", free=" << search_stats.known_free_cells
                      << ", unknown=" << search_stats.unknown_cells
                      << ", occupied=" << search_stats.occupied_cells
                      << "." << std::endl;
        }
        return false;
    }

    vec_E<FrontierCluster> clusters;
    clusterFrontiers(frontier_cells, clusters);
    if (clusters.empty()) {
        exploration_finished_ = false;
        goal.reason = "no frontier cluster";
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No frontier cluster yet. "
                      << "frontiers=" << frontier_cells.size()
                      << ", min_cluster_size=" << cfg_.min_frontier_cluster_size
                      << "." << std::endl;
        }
        return false;
    }

    std::sort(clusters.begin(), clusters.end(),
              [&robot_pos](const FrontierCluster &lhs, const FrontierCluster &rhs) {
                  if (lhs.size != rhs.size) {
                      return lhs.size > rhs.size;
                  }
                  return (lhs.center - robot_pos).squaredNorm() <
                         (rhs.center - robot_pos).squaredNorm();
              });

    vec_E<ExplorationGoal> candidates;
    for (const auto &cluster : clusters) {
        sampleViewpointsForCluster(cluster, robot_state, current_yaw, candidates);
        if (static_cast<int>(candidates.size()) >= std::max(1, cfg_.max_candidate_num)) {
            break;
        }
    }

    if (candidates.empty()) {
        exploration_finished_ = false;
        goal.reason = "no valid viewpoint";
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No valid viewpoint from "
                      << clusters.size() << " clusters." << std::endl;
        }
        return false;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                  return lhs.score < rhs.score;
              });

    double best_score = std::numeric_limits<double>::infinity();
    ExplorationGoal best_goal;
    const int max_checks = std::max(1, cfg_.max_candidate_num);
    int checked = 0;
    for (auto candidate : candidates) {
        if (checked >= max_checks) {
            break;
        }
        ++checked;
        if (candidate.information_gain < cfg_.min_information_gain) {
            continue;
        }

        vec_E<Vec3f> guide_path;
        const double travel_cost = estimateTravelCost(robot_pos, candidate.position, guide_path);
        if (!std::isfinite(travel_cost) || travel_cost >= kInfCost) {
            continue;
        }
        candidate.travel_cost = travel_cost;
        candidate.distance_to_robot = (candidate.position - robot_pos).norm();
        candidate.guide_path = guide_path;
        const double unknown_risk = estimateUnknownRisk(candidate.position);
        candidate.score = scoreCandidate(candidate, unknown_risk);
        candidate.reason = "selected reachable frontier viewpoint";

        if (candidate.score < best_score) {
            best_score = candidate.score;
            best_goal = candidate;
        }
    }

    if (!std::isfinite(best_score)) {
        exploration_finished_ = false;
        goal.reason = "no reachable frontier";
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No reachable frontier yet. "
                      << "frontiers=" << frontier_cells.size()
                      << ", clusters=" << clusters.size()
                      << ", candidates=" << candidates.size() << std::endl;
        }
        return false;
    }

    best_goal.valid = true;
    goal = best_goal;
    exploration_finished_ = false;

    if (cfg_.print_log) {
        std::cout << " -- [ExplorationFrontend] Goal selected: p=["
                  << goal.position.transpose() << "], yaw=" << goal.yaw
                  << ", score=" << goal.score
                  << ", info=" << goal.information_gain
                  << ", travel=" << goal.travel_cost
                  << ", yaw_cost=" << goal.yaw_cost
                  << ", curvature=" << goal.curvature_cost
                  << ", checked=" << checked
                  << ", clusters=" << clusters.size()
                  << ", frontiers=" << frontier_cells.size() << std::endl;
    }
    return true;
}

bool ExplorationFrontend::isExplorationFinished() const {
    return exploration_finished_;
}

void ExplorationFrontend::reset() {
    exploration_finished_ = false;
}

bool ExplorationFrontend::collectFrontierCells(const Vec3f &robot_pos,
                                               vec_E<FrontierCell> &frontier_cells,
                                               FrontierSearchStats &stats) const {
    frontier_cells.clear();
    stats = FrontierSearchStats{};
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }

    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    const double sample_res = std::max(map_res, cfg_.map_resolution);
    const int index_step = std::max(1, static_cast<int>(std::round(sample_res / map_res)));

    Vec3f box_min = robot_pos - Vec3f::Constant(cfg_.frontier_search_radius);
    Vec3f box_max = robot_pos + Vec3f::Constant(cfg_.frontier_search_radius);
    map_manager_->boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0.0) {
        return false;
    }

    Vec3i min_id;
    Vec3i max_id;
    map_manager_->probMapPosToGlobalIndex(box_min, min_id);
    map_manager_->probMapPosToGlobalIndex(box_max, max_id);
    for (int axis = 0; axis < 3; ++axis) {
        if (min_id(axis) > max_id(axis)) {
            std::swap(min_id(axis), max_id(axis));
        }
    }

    const double radius_sq = cfg_.frontier_search_radius * cfg_.frontier_search_radius;
    std::unordered_set<GridKey, GridKeyHasher> inserted;
    for (int ix = min_id.x(); ix <= max_id.x(); ix += index_step) {
        for (int iy = min_id.y(); iy <= max_id.y(); iy += index_step) {
            for (int iz = min_id.z(); iz <= max_id.z(); iz += index_step) {
                FrontierCell cell;
                cell.index = Vec3i(ix, iy, iz);
                map_manager_->probMapGlobalIndexToPos(cell.index, cell.position);
                if ((cell.position - robot_pos).squaredNorm() > radius_sq ||
                    !map_manager_->insideLocalMap(cell.position)) {
                    continue;
                }
                ++stats.searched_cells;
                if (inserted.find(makeKey(cell.index)) != inserted.end()) {
                    continue;
                }
                const rog_map::GridType grid_type = map_manager_->getGridType(cell.position);
                if (isFreeLike(grid_type)) {
                    ++stats.known_free_cells;
                } else if (isUnknownLike(grid_type)) {
                    ++stats.unknown_cells;
                } else if (grid_type == rog_map::GridType::OCCUPIED) {
                    ++stats.occupied_cells;
                }

                if (isFrontierCell(cell, grid_type)) {
                    inserted.insert(makeKey(cell.index));
                    ++stats.frontier_cells;
                    frontier_cells.push_back(cell);
                }
            }
        }
    }
    return true;
}

bool ExplorationFrontend::isFrontierCell(const FrontierCell &cell,
                                         const rog_map::GridType grid_type) const {
    if (!isFreeLike(grid_type)) {
        return false;
    }
    const rog_map::GridType inf_type = map_manager_->getInfGridType(cell.position);
    if (inf_type == rog_map::GridType::OCCUPIED ||
        inf_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }

    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const Vec3f neighbor_pos =
                        cell.position + map_res * Vec3f(dx, dy, dz);
                if (!map_manager_->insideLocalMap(neighbor_pos)) {
                    continue;
                }
                if (isUnknownLike(map_manager_->getGridType(neighbor_pos))) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ExplorationFrontend::mapObservationReady(const FrontierSearchStats &stats) const {
    const int min_known_free_cells = std::max(10, cfg_.min_frontier_cluster_size);
    return stats.known_free_cells >= min_known_free_cells;
}

void ExplorationFrontend::clusterFrontiers(const vec_E<FrontierCell> &frontier_cells,
                                           vec_E<FrontierCluster> &clusters) const {
    clusters.clear();
    if (frontier_cells.empty()) {
        return;
    }

    const double radius = std::max(map_manager_->getResolution(), cfg_.frontier_cluster_radius);
    const double radius_sq = radius * radius;
    std::unordered_map<GridKey, std::vector<int>, GridKeyHasher> buckets;
    buckets.reserve(frontier_cells.size());
    for (int i = 0; i < static_cast<int>(frontier_cells.size()); ++i) {
        buckets[makeBucketKey(frontier_cells[static_cast<size_t>(i)].position, radius)].push_back(i);
    }

    std::vector<char> visited(frontier_cells.size(), 0);
    std::queue<int> q;
    for (int seed = 0; seed < static_cast<int>(frontier_cells.size()); ++seed) {
        if (visited[static_cast<size_t>(seed)] != 0) {
            continue;
        }

        FrontierCluster cluster;
        visited[static_cast<size_t>(seed)] = 1;
        q.push(seed);
        while (!q.empty()) {
            const int current = q.front();
            q.pop();
            const FrontierCell &current_cell = frontier_cells[static_cast<size_t>(current)];
            cluster.cells.push_back(current_cell);

            const GridKey bucket = makeBucketKey(current_cell.position, radius);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const GridKey neighbor_bucket{bucket.x + dx, bucket.y + dy, bucket.z + dz};
                        const auto it = buckets.find(neighbor_bucket);
                        if (it == buckets.end()) {
                            continue;
                        }
                        for (const int neighbor_id : it->second) {
                            if (visited[static_cast<size_t>(neighbor_id)] != 0) {
                                continue;
                            }
                            const FrontierCell &neighbor =
                                    frontier_cells[static_cast<size_t>(neighbor_id)];
                            if ((neighbor.position - current_cell.position).squaredNorm() > radius_sq) {
                                continue;
                            }
                            visited[static_cast<size_t>(neighbor_id)] = 1;
                            q.push(neighbor_id);
                        }
                    }
                }
            }
        }

        if (static_cast<int>(cluster.cells.size()) < cfg_.min_frontier_cluster_size) {
            continue;
        }

        cluster.size = static_cast<int>(cluster.cells.size());
        cluster.center.setZero();
        cluster.bbox_min = cluster.cells.front().position;
        cluster.bbox_max = cluster.cells.front().position;
        Vec3f unknown_dir = Vec3f::Zero();
        const double map_res = std::max(1.0e-3, map_manager_->getResolution());
        for (const FrontierCell &cell : cluster.cells) {
            cluster.center += cell.position;
            cluster.bbox_min = cluster.bbox_min.cwiseMin(cell.position);
            cluster.bbox_max = cluster.bbox_max.cwiseMax(cell.position);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const Vec3f neighbor_pos = cell.position + map_res * Vec3f(dx, dy, dz);
                        if (map_manager_->insideLocalMap(neighbor_pos) &&
                            isUnknownLike(map_manager_->getGridType(neighbor_pos))) {
                            unknown_dir += neighbor_pos - cell.position;
                        }
                    }
                }
            }
        }
        cluster.center /= static_cast<double>(cluster.size);
        if (unknown_dir.norm() > 1.0e-6) {
            cluster.unknown_direction = unknown_dir.normalized();
        }
        clusters.push_back(cluster);
    }
}

void ExplorationFrontend::sampleViewpointsForCluster(const FrontierCluster &cluster,
                                                     const StatePVAJ &robot_state,
                                                     const double current_yaw,
                                                     vec_E<ExplorationGoal> &candidates) const {
    const int max_candidate_num = std::max(1, cfg_.max_candidate_num);
    const int yaw_num = std::max(1, cfg_.viewpoint_yaw_sample_num);
    const int radius_num = std::max(1, cfg_.viewpoint_radius_sample_num);
    const double min_radius = std::max(0.0, cfg_.viewpoint_min_distance);
    const double max_radius = std::max(min_radius, cfg_.viewpoint_max_distance);
    const Vec3f robot_pos = robot_state.col(0);

    for (int ri = 0; ri < radius_num; ++ri) {
        const double alpha = radius_num == 1
                                     ? 0.0
                                     : static_cast<double>(ri) / static_cast<double>(radius_num - 1);
        const double radius = min_radius + alpha * (max_radius - min_radius);
        for (int yi = 0; yi < yaw_num; ++yi) {
            if (static_cast<int>(candidates.size()) >= max_candidate_num) {
                return;
            }
            const double angle = 2.0 * kPi * static_cast<double>(yi) / static_cast<double>(yaw_num);
            Vec3f viewpoint = cluster.center + radius * Vec3f(std::cos(angle), std::sin(angle), 0.0);
            viewpoint.z() = cluster.center.z() + cfg_.viewpoint_height_offset;

            const double distance_to_robot = (viewpoint - robot_pos).norm();
            if (distance_to_robot <= std::max(0.2, cfg_.goal_reached_distance)) {
                continue;
            }
            if (!viewpointSafe(viewpoint) || !viewpointVisible(viewpoint, cluster)) {
                continue;
            }

            const double information_gain = estimateInformationGain(viewpoint, cluster);
            if (information_gain < cfg_.min_information_gain) {
                continue;
            }

            ExplorationGoal candidate;
            candidate.valid = true;
            candidate.position = viewpoint;
            candidate.yaw = std::atan2(cluster.center.y() - viewpoint.y(),
                                       cluster.center.x() - viewpoint.x());
            candidate.information_gain = information_gain;
            candidate.distance_to_robot = distance_to_robot;
            candidate.travel_cost = distance_to_robot;
            candidate.yaw_cost = estimateYawCost(current_yaw, candidate.yaw);
            candidate.curvature_cost = estimateCurvatureCost(robot_state, viewpoint, cluster);
            const double unknown_risk = estimateUnknownRisk(viewpoint);
            candidate.score = scoreCandidate(candidate, unknown_risk);
            candidate.reason = "sampled frontier viewpoint";
            candidates.push_back(candidate);
        }
    }
}

bool ExplorationFrontend::viewpointSafe(const Vec3f &viewpoint) const {
    if (!viewpoint.allFinite() ||
        map_manager_ == nullptr ||
        !map_manager_->ready() ||
        !map_manager_->insideLocalMap(viewpoint)) {
        return false;
    }

    const rog_map::GridType grid_type = map_manager_->getGridType(viewpoint);
    if (grid_type == rog_map::GridType::OCCUPIED ||
        grid_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }
    if (cfg_.unknown_as_occupied_for_motion && !isFreeLike(grid_type)) {
        return false;
    }

    const rog_map::GridType inf_type = map_manager_->getInfGridType(viewpoint);
    if (inf_type == rog_map::GridType::OCCUPIED ||
        inf_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }

    if (map_manager_->hasESDF() && cfg_.viewpoint_safe_distance > 0.0) {
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        if (!map_manager_->evaluateESDF(viewpoint, dist, grad) ||
            !std::isfinite(dist) ||
            dist < cfg_.viewpoint_safe_distance) {
            return false;
        }
    }
    return true;
}

bool ExplorationFrontend::viewpointVisible(const Vec3f &viewpoint,
                                           const FrontierCluster &cluster) const {
    if (!cfg_.require_line_free_to_frontier) {
        return true;
    }
    return map_manager_->isLineFree(viewpoint, cluster.center, true, false);
}

double ExplorationFrontend::estimateInformationGain(const Vec3f &viewpoint,
                                                    const FrontierCluster &cluster) const {
    if (cluster.cells.empty()) {
        return 0.0;
    }

    const int max_rays = 32;
    const int stride = std::max(1, static_cast<int>(cluster.cells.size()) / max_rays);
    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    int unknown_count = 0;
    int visible_count = 0;

    for (size_t i = 0; i < cluster.cells.size(); i += static_cast<size_t>(stride)) {
        const Vec3f &frontier_pos = cluster.cells[i].position;
        if (!map_manager_->isLineFree(viewpoint, frontier_pos, true, false)) {
            continue;
        }
        ++visible_count;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const Vec3f neighbor_pos = frontier_pos + map_res * Vec3f(dx, dy, dz);
                    if (map_manager_->insideLocalMap(neighbor_pos) &&
                        isUnknownLike(map_manager_->getGridType(neighbor_pos))) {
                        ++unknown_count;
                    }
                }
            }
        }
    }

    const double fallback_gain = static_cast<double>(cluster.size);
    const double raw_gain = unknown_count > 0
                                    ? static_cast<double>(unknown_count)
                                    : (visible_count > 0 ? static_cast<double>(visible_count) : fallback_gain);
    const double distance = std::max(0.0, (viewpoint - cluster.center).norm());
    return raw_gain / (1.0 + 0.1 * distance);
}

double ExplorationFrontend::estimateTravelCost(const Vec3f &robot_pos,
                                               const Vec3f &viewpoint,
                                               vec_E<Vec3f> &guide_path) const {
    guide_path.clear();
    if (!robot_pos.allFinite() || !viewpoint.allFinite()) {
        return kInfCost;
    }

    const bool inflated_line_free = map_manager_->isLineFree(robot_pos, viewpoint, true, false);
    const bool known_line_free =
            !cfg_.unknown_as_occupied_for_motion ||
            map_manager_->isLineFree(robot_pos, viewpoint, false, true);
    if (inflated_line_free && known_line_free) {
        guide_path.push_back(robot_pos);
        guide_path.push_back(viewpoint);
        return (viewpoint - robot_pos).norm();
    }

    if (!cfg_.use_astar_cost || astar_ == nullptr) {
        return kInfCost;
    }

    vec_E<Vec3f> path;
    const int astar_flag =
            cfg_.unknown_as_occupied_for_motion
                    ? (path_search::ON_PROB_MAP |
                       path_search::UNKNOWN_AS_OCCUPIED |
                       path_search::DONT_USE_INF_NEIGHBOR)
                    : (path_search::ON_INF_MAP |
                       path_search::UNKNOWN_AS_FREE);
    const double distance = (viewpoint - robot_pos).norm();
    const double search_horizon = std::max(cfg_.frontier_search_radius * 1.5,
                                           distance * 1.8 + 2.0);
    const RET_CODE ret = astar_->pointToPointPathSearch(robot_pos,
                                                        viewpoint,
                                                        astar_flag,
                                                        search_horizon,
                                                        path,
                                                        0.02);
    if (ret != REACH_GOAL || path.empty()) {
        return kInfCost;
    }

    if ((path.front() - robot_pos).norm() > map_manager_->getResolution()) {
        path.insert(path.begin(), robot_pos);
    }
    if ((path.back() - viewpoint).norm() > map_manager_->getResolution()) {
        path.push_back(viewpoint);
    }

    for (size_t i = 1; i < path.size(); ++i) {
        const bool segment_inflated_free = map_manager_->isLineFree(path[i - 1], path[i], true, false);
        const bool segment_known_free =
                !cfg_.unknown_as_occupied_for_motion ||
                map_manager_->isLineFree(path[i - 1], path[i], false, true);
        if (!segment_inflated_free || !segment_known_free) {
            return kInfCost;
        }
    }

    guide_path = path;
    return pathLength(guide_path);
}

double ExplorationFrontend::estimateYawCost(const double current_yaw,
                                            const double candidate_yaw) const {
    return std::abs(wrapAngleDiff(candidate_yaw, current_yaw));
}

double ExplorationFrontend::estimateCurvatureCost(const StatePVAJ &robot_state,
                                                  const Vec3f &viewpoint,
                                                  const FrontierCluster &cluster) const {
    const Vec3f robot_pos = robot_state.col(0);
    const Vec3f robot_vel = robot_state.col(1);
    const Vec3f to_goal = viewpoint - robot_pos;
    const Vec3f observe_dir = cluster.center - viewpoint;

    double cost = angleBetween(to_goal, observe_dir);
    Vec3f vel_dir = robot_vel;
    vel_dir.z() = 0.0;
    if (vel_dir.norm() > 0.25) {
        cost += angleBetween(vel_dir, to_goal);
    }
    return cost;
}

double ExplorationFrontend::estimateUnknownRisk(const Vec3f &viewpoint) const {
    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    int unknown_count = 0;
    int checked_count = 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const Vec3f neighbor_pos = viewpoint + map_res * Vec3f(dx, dy, dz);
                if (!map_manager_->insideLocalMap(neighbor_pos)) {
                    continue;
                }
                ++checked_count;
                if (isUnknownLike(map_manager_->getGridType(neighbor_pos))) {
                    ++unknown_count;
                }
            }
        }
    }
    if (checked_count == 0) {
        return 1.0;
    }
    return static_cast<double>(unknown_count) / static_cast<double>(checked_count);
}

double ExplorationFrontend::scoreCandidate(const ExplorationGoal &candidate,
                                           const double unknown_risk) const {
    return cfg_.weight_travel * candidate.travel_cost +
           cfg_.weight_yaw * candidate.yaw_cost +
           cfg_.weight_curvature * candidate.curvature_cost +
           cfg_.weight_info_gain * candidate.information_gain +
           cfg_.weight_unknown_risk * unknown_risk;
}

bool ExplorationFrontend::isUnknownLike(const rog_map::GridType type) const {
    return type == rog_map::GridType::UNKNOWN ||
           type == rog_map::GridType::UNDEFINED ||
           type == rog_map::GridType::FRONTIER;
}

bool ExplorationFrontend::isFreeLike(const rog_map::GridType type) const {
    return type == rog_map::GridType::KNOWN_FREE;
}

double ExplorationFrontend::wrapAngleDiff(const double lhs, const double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

}  // namespace general_planner
