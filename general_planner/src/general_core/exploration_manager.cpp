#include <general_core/exploration_manager.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using namespace super_utils;

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

struct GridKeyHash {
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

GridKey makeBucketKey(const Vec3f &p, const double bucket_size) {
    return GridKey{static_cast<int>(std::floor(p.x() / bucket_size)),
                   static_cast<int>(std::floor(p.y() / bucket_size)),
                   static_cast<int>(std::floor(p.z() / bucket_size))};
}

double wrapAngle(const double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
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
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        len += (path[i] - path[i - 1]).norm();
    }
    return len;
}

void fillClusterGeometry(FrontierCluster &cluster) {
    cluster.raw_size = static_cast<int>(cluster.cells.size());
    if (cluster.cells.empty()) {
        cluster.valid = false;
        return;
    }
    cluster.center.setZero();
    cluster.bbox_min = cluster.cells.front();
    cluster.bbox_max = cluster.cells.front();
    for (const auto &p : cluster.cells) {
        cluster.center += p;
        cluster.bbox_min = cluster.bbox_min.cwiseMin(p);
        cluster.bbox_max = cluster.bbox_max.cwiseMax(p);
    }
    cluster.center /= static_cast<double>(cluster.cells.size());
    cluster.valid = true;
}
}  // namespace

ExplorationManager::ExplorationManager(const ExplorationConfig &cfg,
                                       const MapManager::Ptr &map_manager,
                                       const path_search::Astar::Ptr &astar,
                                       const ros_interface::RosInterface::Ptr &ros_ptr)
        : cfg_(cfg),
          map_manager_(map_manager),
          astar_(astar),
          ros_ptr_(ros_ptr) {
    CoverageGuideConfig guide_cfg;
    guide_cfg.enable = cfg_.coverage_guide_enable;
    guide_cfg.max_active_regions = cfg_.coverage_max_active_regions;
    guide_cfg.weight_distance = cfg_.coverage_weight_distance;
    guide_cfg.weight_height = cfg_.coverage_weight_height;
    guide_cfg.weight_gain = cfg_.coverage_weight_gain;
    guide_cfg.weight_revisit = cfg_.coverage_weight_revisit;
    guide_cfg.weight_switch_region = cfg_.coverage_weight_switch_region;
    guide_cfg.use_astar_cost = cfg_.coverage_use_astar_cost;
    guide_cfg.max_route_length = cfg_.coverage_max_route_length;
    coverage_guide_planner_ = std::make_unique<CoverageGuidePlanner>(guide_cfg);
}

ExplorationFrontend::ExplorationFrontend(const Config &cfg,
                                         const MapManager::Ptr &map_manager,
                                         const path_search::Astar::Ptr &astar)
        : ExplorationManager(cfg, map_manager, astar, nullptr) {
}

bool ExplorationManager::planNextGoal(const StatePVAJ &robot_state,
                                      const double current_yaw,
                                      const double committed_traj_remaining,
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
    if (!robot_pos.allFinite() || !map_manager_->insideLocalMap(robot_pos)) {
        goal.reason = "robot state is outside local map";
        return false;
    }

    purgeBlacklist();

    std::vector<FrontierCluster> clusters;
    const bool global_ready = map_manager_->globalExplorationMapReady();
    bool using_global_frontiers = false;
    if (cfg_.use_global_frontiers && global_ready) {
        using_global_frontiers = updateFrontierClusters(clusters);
    }
    if (!using_global_frontiers) {
        if (cfg_.print_log && cfg_.use_global_frontiers && !global_ready) {
            const std::string msg =
                    "[Exploration] Global exploration map is not ready, fallback to local ROGMap frontier.";
            if (ros_ptr_) {
                ros_ptr_->warn(msg);
            } else {
                std::cout << msg << std::endl;
            }
        }
        if (!updateLocalFrontierClusters(robot_pos, clusters)) {
            goal.reason = "frontier search failed";
            return false;
        }
    }

    splitLargeClusters(clusters);
    filterClusters(clusters);

    int frontier_count = 0;
    for (const auto &cluster : clusters) {
        frontier_count += cluster.raw_size;
    }

    if (clusters.empty()) {
        goal.reason = "no frontier";
        if (global_ready) {
            exploration_finished_ = true;
            if (cfg_.print_log) {
                const std::string msg =
                        ros_interface::RosInterface::format(
                                "[Exploration] Exploration finished:\n"
                                "    explored_volume={:.3f},\n"
                                "    frontier_count={}",
                                map_manager_->globalExploredVolume(),
                                map_manager_->globalFrontierCount());
                if (ros_ptr_) {
                    ros_ptr_->info(msg);
                } else {
                    std::cout << msg << std::endl;
                }
            }
        }
        return false;
    }

    map_manager_->updateGlobalRegions(clusters);

    std::vector<ExplorationRegion> active_regions;
    map_manager_->getActiveGlobalRegions(active_regions);

    std::vector<int> ordered_region_ids;
    std::vector<int> selected_region_ids;
    if (coverage_guide_planner_ != nullptr &&
        coverage_guide_planner_->buildGuidePath(robot_pos, active_regions, ordered_region_ids)) {
        coverage_guide_planner_->selectPriorityRegions(ordered_region_ids,
                                                       std::max(1, std::min(5, cfg_.coverage_max_route_length)),
                                                       selected_region_ids);
    }

    std::unordered_set<int> selected_region_set(selected_region_ids.begin(), selected_region_ids.end());
    std::unordered_map<int, int> region_order;
    for (int i = 0; i < static_cast<int>(ordered_region_ids.size()); ++i) {
        region_order[ordered_region_ids[static_cast<size_t>(i)]] = i;
    }

    std::vector<FrontierCluster> local_clusters;
    const double local_region_radius =
            cfg_.frontier_search_radius + cfg_.viewpoint_max_distance + cfg_.sensor_range;
    for (const auto &cluster : clusters) {
        if ((cluster.center - robot_pos).norm() > local_region_radius) {
            continue;
        }
        if (!selected_region_set.empty() &&
            selected_region_set.find(cluster.region_id) == selected_region_set.end()) {
            continue;
        }
        local_clusters.push_back(cluster);
    }
    if (local_clusters.empty()) {
        for (const auto &cluster : clusters) {
            if ((cluster.center - robot_pos).norm() <= local_region_radius) {
                local_clusters.push_back(cluster);
            }
        }
    }
    if (local_clusters.empty()) {
        local_clusters = clusters;
    }

    std::sort(local_clusters.begin(), local_clusters.end(),
              [&robot_pos, &region_order](const FrontierCluster &lhs, const FrontierCluster &rhs) {
                  const int lhs_order = region_order.count(lhs.region_id) ? region_order[lhs.region_id] : 100000;
                  const int rhs_order = region_order.count(rhs.region_id) ? region_order[rhs.region_id] : 100000;
                  if (lhs_order != rhs_order) {
                      return lhs_order < rhs_order;
                  }
                  return (lhs.center - robot_pos).squaredNorm() <
                         (rhs.center - robot_pos).squaredNorm();
              });

    std::vector<ViewpointCandidate> candidates;
    sampleViewpointsForClusters(local_clusters, robot_state, current_yaw, candidates);
    if (candidates.empty()) {
        goal.reason = "no valid viewpoint";
        logNoGoal(goal.reason, 0, frontier_count, static_cast<int>(clusters.size()));
        return false;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const ViewpointCandidate &lhs, const ViewpointCandidate &rhs) {
                  return lhs.cheap_score < rhs.cheap_score;
              });

    const int max_checks = cfg_.use_astar_cost
                                   ? std::min(static_cast<int>(candidates.size()),
                                              std::max(1, cfg_.max_astar_candidate_num))
                                   : static_cast<int>(candidates.size());
    int astar_checked = 0;
    ExplorationGoal best_goal;
    double best_score = std::numeric_limits<double>::infinity();
    for (int i = 0; i < max_checks; ++i) {
        ViewpointCandidate &candidate = candidates[static_cast<size_t>(i)];
        if (cfg_.use_astar_cost) {
            ++astar_checked;
            if (!runAstarForCandidate(robot_pos, candidate)) {
                addFailedCandidateToBlacklist(candidate);
                continue;
            }
        } else {
            candidate.reachable = true;
            candidate.astar_checked = false;
        }
        candidate.final_score = computeCheapScore(candidate);
        ExplorationGoal candidate_goal = toGoal(candidate);
        if (candidate_goal.score < best_score) {
            best_score = candidate_goal.score;
            best_goal = candidate_goal;
        }
    }

    if (!best_goal.valid) {
        goal.reason = "no reachable frontier";
        logNoGoal(goal.reason, static_cast<int>(candidates.size()), frontier_count, static_cast<int>(clusters.size()));
        return false;
    }

    if (shouldKeepCurrentGoal(current_goal_, best_goal, committed_traj_remaining)) {
        if (cfg_.print_log) {
            const std::string msg =
                    ros_interface::RosInterface::format(
                            "[Exploration] Keep current goal:\n"
                            "    remaining={:.3f},\n"
                            "    current_score={:.3f},\n"
                            "    candidate_score={:.3f},\n"
                            "    current_gain={:.3f},\n"
                            "    candidate_gain={:.3f}",
                            committed_traj_remaining,
                            current_goal_.score,
                            best_goal.score,
                            current_goal_.information_gain_norm,
                            best_goal.information_gain_norm);
            if (ros_ptr_) {
                ros_ptr_->info(msg);
            } else {
                std::cout << msg << std::endl;
            }
        }
        goal = current_goal_;
        return true;
    }

    for (const int region_id : ordered_region_ids) {
        if (map_manager_->positionToGlobalRegionId(best_goal.position) == region_id) {
            best_goal.coverage_node_id = region_id;
            break;
        }
    }
    current_goal_ = best_goal;
    goal = best_goal;
    logGoalSelected(goal,
                    static_cast<int>(candidates.size()),
                    astar_checked,
                    frontier_count,
                    static_cast<int>(clusters.size()));
    return true;
}

bool ExplorationManager::isExplorationFinished() const {
    return exploration_finished_;
}

void ExplorationManager::reset() {
    current_goal_ = ExplorationGoal{};
    exploration_finished_ = false;
    failed_candidates_.clear();
    if (coverage_guide_planner_) {
        coverage_guide_planner_->reset();
    }
}

bool ExplorationManager::getCurrentGoal(ExplorationGoal &goal) const {
    goal = current_goal_;
    return current_goal_.valid;
}

bool ExplorationManager::updateFrontierClusters(std::vector<FrontierCluster> &clusters) {
    clusters.clear();
    if (map_manager_ == nullptr || !map_manager_->globalExplorationMapReady()) {
        return false;
    }
    map_manager_->getGlobalFrontierClusters(cfg_.frontier_cluster_radius,
                                            cfg_.min_frontier_cluster_size,
                                            clusters);
    return true;
}

bool ExplorationManager::updateLocalFrontierClusters(const Vec3f &robot_pos,
                                                     std::vector<FrontierCluster> &clusters) const {
    clusters.clear();
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }

    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    const int index_step = std::max(1, cfg_.frontier_downsample_step);
    Vec3f box_min = robot_pos - Vec3f::Constant(cfg_.frontier_search_radius);
    Vec3f box_max = robot_pos + Vec3f::Constant(cfg_.frontier_search_radius);
    map_manager_->boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0.0) {
        return true;
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

    vec_E<Vec3f> frontier_points;
    const double radius_sq = cfg_.frontier_search_radius * cfg_.frontier_search_radius;
    for (int ix = min_id.x(); ix <= max_id.x(); ix += index_step) {
        for (int iy = min_id.y(); iy <= max_id.y(); iy += index_step) {
            for (int iz = min_id.z(); iz <= max_id.z(); iz += index_step) {
                Vec3i id(ix, iy, iz);
                Vec3f p;
                map_manager_->probMapGlobalIndexToPos(id, p);
                if ((p - robot_pos).squaredNorm() > radius_sq ||
                    !map_manager_->insideLocalMap(p) ||
                    !isFreeLike(map_manager_->getGridType(p)) ||
                    !localCellHasUnknownNeighbor(p)) {
                    continue;
                }
                const auto inf_type = map_manager_->getInfGridType(p);
                if (inf_type == rog_map::GridType::OCCUPIED ||
                    inf_type == rog_map::GridType::OUT_OF_MAP) {
                    continue;
                }
                frontier_points.push_back(p);
            }
        }
    }
    if (frontier_points.empty()) {
        return true;
    }

    const double radius = std::max(map_res, cfg_.frontier_cluster_radius);
    const double radius_sq_cluster = radius * radius;
    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> buckets;
    buckets.reserve(frontier_points.size());
    for (int i = 0; i < static_cast<int>(frontier_points.size()); ++i) {
        buckets[makeBucketKey(frontier_points[static_cast<size_t>(i)], radius)].push_back(i);
    }

    std::vector<char> visited(frontier_points.size(), 0);
    std::queue<int> q;
    int next_id = 0;
    for (int seed = 0; seed < static_cast<int>(frontier_points.size()); ++seed) {
        if (visited[static_cast<size_t>(seed)] != 0) {
            continue;
        }
        FrontierCluster cluster;
        cluster.id = next_id++;
        visited[static_cast<size_t>(seed)] = 1;
        q.push(seed);
        while (!q.empty()) {
            const int current = q.front();
            q.pop();
            const Vec3f p = frontier_points[static_cast<size_t>(current)];
            cluster.cells.push_back(p);
            const GridKey bucket = makeBucketKey(p, radius);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const GridKey nb{bucket.x + dx, bucket.y + dy, bucket.z + dz};
                        const auto it = buckets.find(nb);
                        if (it == buckets.end()) {
                            continue;
                        }
                        for (const int neighbor : it->second) {
                            if (visited[static_cast<size_t>(neighbor)] != 0) {
                                continue;
                            }
                            if ((frontier_points[static_cast<size_t>(neighbor)] - p).squaredNorm() >
                                radius_sq_cluster) {
                                continue;
                            }
                            visited[static_cast<size_t>(neighbor)] = 1;
                            q.push(neighbor);
                        }
                    }
                }
            }
        }
        fillClusterGeometry(cluster);
        cluster.region_id = map_manager_->positionToGlobalRegionId(cluster.center);
        cluster.filtered_cells = cluster.cells;
        clusters.push_back(cluster);
    }
    return true;
}

void ExplorationManager::splitLargeClusters(std::vector<FrontierCluster> &clusters) const {
    std::vector<FrontierCluster> split_clusters;
    split_clusters.reserve(clusters.size());
    const double bin_size = std::max(cfg_.frontier_cluster_radius, 0.5 * cfg_.max_cluster_extent);
    for (const auto &cluster : clusters) {
        const double diagonal = (cluster.bbox_max - cluster.bbox_min).norm();
        if (cluster.raw_size <= cfg_.max_frontiers_per_cluster &&
            diagonal <= cfg_.max_cluster_extent) {
            split_clusters.push_back(cluster);
            continue;
        }

        std::unordered_map<GridKey, vec_E<Vec3f>, GridKeyHash> bins;
        bins.reserve(cluster.cells.size());
        for (const auto &p : cluster.cells) {
            bins[makeBucketKey(p, bin_size)].push_back(p);
        }
        for (const auto &kv : bins) {
            FrontierCluster sub;
            sub.region_id = cluster.region_id;
            sub.cells = kv.second;
            fillClusterGeometry(sub);
            sub.filtered_cells = sub.cells;
            split_clusters.push_back(sub);
        }
    }

    int next_id = 0;
    for (auto &cluster : split_clusters) {
        cluster.id = next_id++;
        if (map_manager_ != nullptr) {
            cluster.region_id = map_manager_->positionToGlobalRegionId(cluster.center);
        }
    }
    clusters = std::move(split_clusters);
}

void ExplorationManager::filterClusters(std::vector<FrontierCluster> &clusters) const {
    std::vector<FrontierCluster> filtered;
    filtered.reserve(clusters.size());
    const int downsample_step = std::max(1, cfg_.frontier_downsample_step);
    for (auto cluster : clusters) {
        fillClusterGeometry(cluster);
        if (cluster.raw_size < cfg_.min_frontier_cluster_size) {
            continue;
        }
        cluster.filtered_cells.clear();
        for (int i = 0; i < static_cast<int>(cluster.cells.size()); i += downsample_step) {
            cluster.filtered_cells.push_back(cluster.cells[static_cast<size_t>(i)]);
        }
        if (cluster.filtered_cells.empty()) {
            cluster.filtered_cells = cluster.cells;
        }
        cluster.valid = true;
        filtered.push_back(cluster);
    }
    int next_id = 0;
    for (auto &cluster : filtered) {
        cluster.id = next_id++;
    }
    clusters = std::move(filtered);
}

void ExplorationManager::sampleViewpointsForClusters(const std::vector<FrontierCluster> &clusters,
                                                     const StatePVAJ &robot_state,
                                                     const double current_yaw,
                                                     std::vector<ViewpointCandidate> &candidates) {
    candidates.clear();
    const int max_candidate_num = std::max(1, cfg_.max_candidate_num);
    const int radius_num = std::max(1, cfg_.viewpoint_radius_num);
    const int yaw_num = std::max(1, cfg_.viewpoint_yaw_num);
    const double min_radius = std::max(0.1, cfg_.viewpoint_min_distance);
    const double max_radius = std::max(min_radius, cfg_.viewpoint_max_distance);
    const Vec3f robot_pos = robot_state.col(0);

    for (const auto &cluster : clusters) {
        if (!cluster.valid) {
            continue;
        }
        for (int ri = 0; ri < radius_num; ++ri) {
            const double alpha = radius_num == 1
                                         ? 0.0
                                         : static_cast<double>(ri) / static_cast<double>(radius_num - 1);
            const double radius = min_radius + alpha * (max_radius - min_radius);
            for (int ai = 0; ai < yaw_num; ++ai) {
                if (static_cast<int>(candidates.size()) >= max_candidate_num) {
                    return;
                }
                const double angle = 2.0 * kPi * static_cast<double>(ai) / static_cast<double>(yaw_num);
                ViewpointCandidate candidate;
                candidate.cluster_id = cluster.id;
                candidate.region_id = cluster.region_id;
                candidate.position = cluster.center + radius * Vec3f(std::cos(angle), std::sin(angle), 0.0);
                candidate.position.z() = cluster.center.z() + cfg_.viewpoint_height_offset;
                const double candidate_robot_distance = (candidate.position - robot_pos).norm();
                const double min_goal_distance =
                        std::max(cfg_.goal_reached_distance,
                                 std::max(0.0, cfg_.min_goal_distance));
                if (candidate_robot_distance > cfg_.frontier_search_radius ||
                    candidate_robot_distance < min_goal_distance ||
                    !viewpointSafe(candidate.position) ||
                    candidateInBlacklist(candidate.position)) {
                    continue;
                }

                const double yaw_to_cluster =
                        std::atan2(cluster.center.y() - candidate.position.y(),
                                   cluster.center.x() - candidate.position.x());
                double best_yaw = yaw_to_cluster;
                double best_gain = -1.0;
                if (cfg_.use_fov_gain && cfg_.sensor_horizontal_fov_deg < 350.0) {
                    for (int yi = 0; yi < yaw_num; ++yi) {
                        const double yaw = wrapAngle(yaw_to_cluster +
                                                     2.0 * kPi *
                                                     (static_cast<double>(yi) / static_cast<double>(yaw_num) - 0.5));
                        candidate.yaw = yaw;
                        const double gain = computeVisibilityGain(candidate, cluster);
                        if (gain > best_gain) {
                            best_gain = gain;
                            best_yaw = yaw;
                        }
                    }
                } else {
                    const Vec3f vel = robot_state.col(1);
                    best_yaw = vel.head<2>().norm() > 0.25 ? std::atan2(vel.y(), vel.x()) : yaw_to_cluster;
                    candidate.yaw = best_yaw;
                    best_gain = computeVisibilityGain(candidate, cluster);
                }

                candidate.yaw = best_yaw;
                candidate.gain_raw = std::max(0.0, best_gain);
                if (candidate.gain_raw < cfg_.min_information_gain) {
                    continue;
                }
                candidate.gain_norm =
                        std::clamp(std::log1p(std::min(candidate.gain_raw, cfg_.info_gain_cap)) /
                                   std::log1p(std::max(1.0, cfg_.info_gain_cap)),
                                   0.0,
                                   1.0);
                candidate.travel_cost = computeTravelCheapCost(robot_pos, candidate.position);
                candidate.yaw_cost = computeYawCost(current_yaw, candidate.yaw);
                candidate.curvature_cost =
                        computeCurvatureCost(robot_state, candidate.position, cluster.center);
                candidate.switching_cost = computeSwitchingCost(candidate);
                candidate.valid = true;
                candidate.cheap_score = computeCheapScore(candidate);
                candidates.push_back(candidate);
            }
        }
    }
}

bool ExplorationManager::viewpointSafe(const Vec3f &p) const {
    if (!p.allFinite() || map_manager_ == nullptr || !map_manager_->ready() ||
        !map_manager_->insideLocalMap(p)) {
        return false;
    }
    const auto grid_type = map_manager_->getGridType(p);
    const auto inf_type = map_manager_->getInfGridType(p);
    if (!isFreeLike(grid_type) ||
        inf_type == rog_map::GridType::OCCUPIED ||
        inf_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }
    if (map_manager_->hasESDF() && cfg_.viewpoint_safe_distance > 0.0) {
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        if (!map_manager_->evaluateESDF(p, dist, grad) ||
            !std::isfinite(dist) ||
            dist < cfg_.viewpoint_safe_distance) {
            return false;
        }
    }
    return true;
}

bool ExplorationManager::insideSensorFov(const Vec3f &viewpoint,
                                         const double yaw,
                                         const Vec3f &target) const {
    const Vec3f delta = target - viewpoint;
    const double range = delta.norm();
    if (range > cfg_.sensor_range || range < 1.0e-6) {
        return false;
    }
    if (!cfg_.use_fov_gain || cfg_.sensor_horizontal_fov_deg >= 359.0) {
        return true;
    }
    const double horizontal_angle = wrapAngle(std::atan2(delta.y(), delta.x()) - yaw);
    const double horizontal_half = 0.5 * cfg_.sensor_horizontal_fov_deg * kPi / 180.0;
    if (std::abs(horizontal_angle) > horizontal_half) {
        return false;
    }
    if (cfg_.sensor_vertical_fov_deg >= 179.0) {
        return true;
    }
    const double vertical_half = 0.5 * cfg_.sensor_vertical_fov_deg * kPi / 180.0;
    const double vertical_angle =
            std::atan2(delta.z(), std::max(1.0e-6, std::hypot(delta.x(), delta.y())));
    return std::abs(vertical_angle) <= vertical_half;
}

bool ExplorationManager::lineOfSightFree(const Vec3f &viewpoint,
                                         const Vec3f &target) const {
    if (!cfg_.require_line_free_to_frontier) {
        return true;
    }
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }
    return map_manager_->isLineFree(viewpoint, target, true, false);
}

double ExplorationManager::computeVisibilityGain(const ViewpointCandidate &candidate,
                                                 const FrontierCluster &cluster) const {
    const auto &cells = cluster.filtered_cells.empty() ? cluster.cells : cluster.filtered_cells;
    if (cells.empty()) {
        return 0.0;
    }
    const int max_samples = std::max(1, cfg_.visibility_sample_max_points);
    const int stride = std::max(1, static_cast<int>(cells.size()) / max_samples);
    int gain = 0;
    for (int i = 0; i < static_cast<int>(cells.size()); i += stride) {
        const Vec3f &target = cells[static_cast<size_t>(i)];
        if (!insideSensorFov(candidate.position, candidate.yaw, target) ||
            !lineOfSightFree(candidate.position, target)) {
            continue;
        }
        const bool informative =
                cfg_.use_global_frontiers && map_manager_->globalExplorationMapReady()
                        ? (map_manager_->isGlobalFrontier(target) || globalCellHasUnknownNeighbor(target))
                        : localCellHasUnknownNeighbor(target);
        if (informative) {
            ++gain;
        }
    }
    return static_cast<double>(gain);
}

double ExplorationManager::computeTravelCheapCost(const Vec3f &robot_pos,
                                                  const Vec3f &candidate_pos) const {
    return (candidate_pos - robot_pos).norm();
}

double ExplorationManager::computeYawCost(const double current_yaw,
                                          const double candidate_yaw) const {
    return std::abs(wrapAngle(candidate_yaw - current_yaw));
}

double ExplorationManager::computeCurvatureCost(const StatePVAJ &robot_state,
                                                const Vec3f &candidate_pos,
                                                const Vec3f &cluster_center) const {
    const Vec3f robot_pos = robot_state.col(0);
    const Vec3f robot_vel = robot_state.col(1);
    const Vec3f to_goal = candidate_pos - robot_pos;
    const Vec3f observe_dir = cluster_center - candidate_pos;
    double cost = angleBetween(to_goal, observe_dir);
    if (robot_vel.head<2>().norm() > 0.25) {
        cost += angleBetween(robot_vel, to_goal);
    }
    return cost;
}

double ExplorationManager::computeSwitchingCost(const ViewpointCandidate &candidate) const {
    if (!current_goal_.valid) {
        return 0.0;
    }
    if ((candidate.position - current_goal_.position).norm() < cfg_.min_switch_distance) {
        return 0.0;
    }
    if (candidate.cluster_id == current_goal_.cluster_id ||
        candidate.region_id == current_goal_.region_id) {
        return 0.35;
    }
    return 1.0;
}

double ExplorationManager::computeCheapScore(ViewpointCandidate &candidate) const {
    const double travel_norm =
            candidate.travel_cost / std::max(1.0e-3, cfg_.travel_cost_norm);
    const double yaw_norm = std::clamp(candidate.yaw_cost / kPi, 0.0, 1.0);
    const double curvature_norm = std::clamp(candidate.curvature_cost / kPi, 0.0, 2.0);
    const double reachability_cost =
            candidate.astar_checked && !candidate.reachable ? 1.0 : 0.0;
    return cfg_.weight_travel * travel_norm +
           cfg_.weight_yaw * yaw_norm +
           cfg_.weight_curvature * curvature_norm +
           cfg_.weight_switch * candidate.switching_cost +
           cfg_.weight_reachability * reachability_cost -
           cfg_.weight_gain * candidate.gain_norm;
}

bool ExplorationManager::runAstarForCandidate(const Vec3f &start,
                                              ViewpointCandidate &candidate) {
    candidate.astar_checked = true;
    candidate.reachable = false;
    candidate.astar_path.clear();
    if (map_manager_ == nullptr || astar_ == nullptr || !viewpointSafe(candidate.position)) {
        return false;
    }
    const bool inflated_line_free = map_manager_->isLineFree(start, candidate.position, true, false);
    const bool known_line_free = map_manager_->isLineFree(start, candidate.position, false, true);
    if (inflated_line_free && known_line_free) {
        candidate.astar_path.push_back(start);
        candidate.astar_path.push_back(candidate.position);
        candidate.travel_cost = (candidate.position - start).norm();
        candidate.reachable = true;
        return true;
    }
    if (!cfg_.use_astar_cost) {
        return false;
    }
    const int astar_flag = path_search::ON_PROB_MAP |
                           path_search::UNKNOWN_AS_OCCUPIED |
                           path_search::DONT_USE_INF_NEIGHBOR;
    const double distance = (candidate.position - start).norm();
    const double horizon = std::max(cfg_.astar_search_horizon, distance * 1.5 + 2.0);
    vec_E<Vec3f> path;
    const RET_CODE ret = astar_->pointToPointPathSearch(start,
                                                        candidate.position,
                                                        astar_flag,
                                                        horizon,
                                                        path,
                                                        cfg_.astar_timeout);
    if (ret != REACH_GOAL || path.empty()) {
        return false;
    }
    if ((path.front() - start).norm() > map_manager_->getResolution()) {
        path.insert(path.begin(), start);
    }
    if ((path.back() - candidate.position).norm() > map_manager_->getResolution()) {
        path.push_back(candidate.position);
    }
    candidate.astar_path = path;
    candidate.travel_cost = pathLength(path);
    candidate.reachable = true;
    return true;
}

bool ExplorationManager::shouldKeepCurrentGoal(const ExplorationGoal &current,
                                               const ExplorationGoal &candidate,
                                               const double committed_traj_remaining) const {
    if (!current.valid ||
        !candidate.valid ||
        committed_traj_remaining <= cfg_.keep_goal_min_remaining_time ||
        !currentGoalStillSafe(current) ||
        !currentGoalStillInformative(current)) {
        return false;
    }
    const bool candidate_clearly_better =
            candidate.astar_reachable &&
            candidate.information_gain_norm > current.information_gain_norm + cfg_.switch_gain_margin &&
            candidate.score < current.score - cfg_.switch_score_margin &&
            (candidate.position - current.position).norm() > cfg_.min_switch_distance;
    return !candidate_clearly_better;
}

bool ExplorationManager::currentGoalStillInformative(const ExplorationGoal &goal) const {
    if (!goal.valid || map_manager_ == nullptr) {
        return false;
    }
    int visible = 0;
    const int need = std::max(1, static_cast<int>(std::ceil(cfg_.min_information_gain)));

    if (cfg_.use_global_frontiers && map_manager_->globalExplorationMapReady()) {
        rog_map::vec_E<rog_map::Vec3f> frontiers;
        map_manager_->getGlobalFrontierPoints(frontiers);
        for (const auto &p : frontiers) {
            if ((p - goal.position).norm() > cfg_.sensor_range ||
                !insideSensorFov(goal.position, goal.yaw, p) ||
                !lineOfSightFree(goal.position, p)) {
                continue;
            }
            if (++visible >= need) {
                return true;
            }
        }
        return false;
    }

    if (!map_manager_->ready()) {
        return false;
    }

    const double radius = std::max(map_manager_->getResolution(), cfg_.sensor_range);
    const double radius_sq = radius * radius;
    Vec3f box_min = goal.position - Vec3f::Constant(radius);
    Vec3f box_max = goal.position + Vec3f::Constant(radius);
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

    const int index_step = std::max(1, cfg_.frontier_downsample_step);
    for (int ix = min_id.x(); ix <= max_id.x(); ix += index_step) {
        for (int iy = min_id.y(); iy <= max_id.y(); iy += index_step) {
            for (int iz = min_id.z(); iz <= max_id.z(); iz += index_step) {
                Vec3i id(ix, iy, iz);
                Vec3f p;
                map_manager_->probMapGlobalIndexToPos(id, p);
                if ((p - goal.position).squaredNorm() > radius_sq ||
                    !insideSensorFov(goal.position, goal.yaw, p) ||
                    !isFreeLike(map_manager_->getGridType(p)) ||
                    !localCellHasUnknownNeighbor(p) ||
                    !lineOfSightFree(goal.position, p)) {
                    continue;
                }
                const auto inf_type = map_manager_->getInfGridType(p);
                if (inf_type == rog_map::GridType::OCCUPIED ||
                    inf_type == rog_map::GridType::OUT_OF_MAP) {
                    continue;
                }
                if (++visible >= need) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ExplorationManager::currentGoalStillSafe(const ExplorationGoal &goal) const {
    return goal.valid && viewpointSafe(goal.position);
}

void ExplorationManager::addFailedCandidateToBlacklist(const ViewpointCandidate &candidate) {
    FailedCandidate failed;
    failed.position = candidate.position;
    failed.stamp = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
    failed_candidates_.push_back(failed);
}

bool ExplorationManager::candidateInBlacklist(const Vec3f &p) const {
    const double now = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
    const double radius = std::max(0.0, cfg_.failed_candidate_blacklist_radius);
    for (const auto &failed : failed_candidates_) {
        if (now - failed.stamp > cfg_.failed_candidate_blacklist_time) {
            continue;
        }
        if ((p - failed.position).norm() <= radius) {
            return true;
        }
    }
    return false;
}

ExplorationGoal ExplorationManager::toGoal(const ViewpointCandidate &candidate) const {
    ExplorationGoal goal;
    goal.valid = candidate.valid && candidate.reachable;
    goal.position = candidate.position;
    goal.yaw = candidate.yaw;
    goal.type = ExplorationGoalType::COVERAGE_REGION_VIEWPOINT;
    goal.cluster_id = candidate.cluster_id;
    goal.region_id = candidate.region_id;
    goal.score = candidate.final_score;
    goal.information_gain = candidate.gain_raw;
    goal.information_gain_norm = candidate.gain_norm;
    goal.travel_cost = candidate.travel_cost;
    goal.yaw_cost = candidate.yaw_cost;
    goal.curvature_cost = candidate.curvature_cost;
    goal.switching_cost = candidate.switching_cost;
    goal.reachability_cost = candidate.reachable ? 0.0 : 1.0;
    goal.astar_checked = candidate.astar_checked;
    goal.astar_reachable = candidate.reachable;
    goal.guide_path = candidate.astar_path;
    goal.reason = "selected reachable frontier viewpoint";
    return goal;
}

bool ExplorationManager::isUnknownLike(const rog_map::GridType type) const {
    return type == rog_map::GridType::UNKNOWN ||
           type == rog_map::GridType::UNDEFINED ||
           type == rog_map::GridType::FRONTIER;
}

bool ExplorationManager::isFreeLike(const rog_map::GridType type) const {
    return type == rog_map::GridType::KNOWN_FREE;
}

bool ExplorationManager::localCellHasUnknownNeighbor(const Vec3f &p) const {
    const double res = std::max(1.0e-3, map_manager_->getResolution());
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const Vec3f n = p + res * Vec3f(dx, dy, dz);
                if (map_manager_->insideLocalMap(n) &&
                    isUnknownLike(map_manager_->getGridType(n))) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ExplorationManager::globalCellHasUnknownNeighbor(const Vec3f &p) const {
    const double res = std::max(0.2, map_manager_ ? map_manager_->getResolution() : 0.2);
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const Vec3f n = p + res * Vec3f(dx, dy, dz);
                if (map_manager_->isGloballyUnknown(n)) {
                    return true;
                }
            }
        }
    }
    return false;
}

void ExplorationManager::purgeBlacklist() {
    const double now = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
    failed_candidates_.erase(
            std::remove_if(failed_candidates_.begin(),
                           failed_candidates_.end(),
                           [&](const FailedCandidate &failed) {
                               return now - failed.stamp > cfg_.failed_candidate_blacklist_time;
                           }),
            failed_candidates_.end());
}

void ExplorationManager::logGoalSelected(const ExplorationGoal &goal,
                                         const int candidate_count,
                                         const int astar_checked,
                                         const int frontier_count,
                                         const int cluster_count) const {
    if (!cfg_.print_log) {
        return;
    }
    const std::string msg =
            ros_interface::RosInterface::format(
                    "[Exploration] Goal selected:\n"
                    "    p=[{:.3f} {:.3f} {:.3f}],\n"
                    "    yaw={:.3f},\n"
                    "    score={:.3f},\n"
                    "    gain_raw={:.3f},\n"
                    "    gain_norm={:.3f},\n"
                    "    travel={:.3f},\n"
                    "    yaw_cost={:.3f},\n"
                    "    curvature={:.3f},\n"
                    "    region_id={},\n"
                    "    cluster_id={},\n"
                    "    candidates={},\n"
                    "    astar_checked={},\n"
                    "    frontiers={},\n"
                    "    clusters={}",
                    goal.position.x(),
                    goal.position.y(),
                    goal.position.z(),
                    goal.yaw,
                    goal.score,
                    goal.information_gain,
                    goal.information_gain_norm,
                    goal.travel_cost,
                    goal.yaw_cost,
                    goal.curvature_cost,
                    goal.region_id,
                    goal.cluster_id,
                    candidate_count,
                    astar_checked,
                    frontier_count,
                    cluster_count);
    if (ros_ptr_) {
        ros_ptr_->info(msg);
    } else {
        std::cout << msg << std::endl;
    }
}

void ExplorationManager::logNoGoal(const std::string &reason,
                                   const int candidate_count,
                                   const int frontier_count,
                                   const int cluster_count) const {
    if (!cfg_.print_log) {
        return;
    }
    const std::string msg =
            ros_interface::RosInterface::format(
                    "[Exploration] No reachable frontier:\n"
                    "    frontiers={},\n"
                    "    clusters={},\n"
                    "    candidates={},\n"
                    "    reason={}",
                    frontier_count,
                    cluster_count,
                    candidate_count,
                    reason);
    if (ros_ptr_) {
        ros_ptr_->warn(msg);
    } else {
        std::cout << msg << std::endl;
    }
}

}  // namespace general_planner
