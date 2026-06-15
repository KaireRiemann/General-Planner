#include <general_core/exploration_frontend.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <unordered_map>

using namespace super_utils;

namespace general_planner {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kInfCost = 1.0e9;

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

FrontierClusterManager::Config makeFrontierClusterManagerConfig(
        const ExplorationFrontend::Config &cfg) {
    FrontierClusterManager::Config cluster_cfg;
    cluster_cfg.cluster_radius = cfg.frontier_cluster_radius;
    cluster_cfg.min_cluster_size = cfg.min_frontier_cluster_size;
    cluster_cfg.lifecycle_match_distance = cfg.frontier_lifecycle_match_distance;
    cluster_cfg.lifecycle_min_observations = cfg.frontier_lifecycle_min_observations;
    cluster_cfg.lifecycle_max_missing_frames = cfg.frontier_lifecycle_max_missing_frames;
    return cluster_cfg;
}

ViewpointSelector::Config makeViewpointSelectorConfig(const ExplorationFrontend::Config &cfg) {
    ViewpointSelector::Config selector_cfg;
    selector_cfg.min_radius = cfg.viewpoint_min_distance;
    selector_cfg.max_radius = cfg.viewpoint_max_distance;
    selector_cfg.radius_sample_num = cfg.viewpoint_radius_sample_num;
    selector_cfg.yaw_sample_num = cfg.viewpoint_yaw_sample_num;
    selector_cfg.height_offset = cfg.viewpoint_height_offset;
    selector_cfg.safe_distance = cfg.viewpoint_safe_distance;
    selector_cfg.unknown_clearance = 0.0;
    selector_cfg.occupied_clearance = cfg.viewpoint_safe_distance;
    selector_cfg.min_visible_cells = cfg.viewpoint_min_visible_cells;
    selector_cfg.top_view_num = cfg.viewpoint_top_view_num;
    selector_cfg.max_decay = cfg.viewpoint_max_decay;
    selector_cfg.sensor_range = cfg.frontier_search_radius;
    selector_cfg.map_resolution = cfg.map_resolution;
    return selector_cfg;
}

GlobalCoveragePlanner::Config makeGlobalCoveragePlannerConfig(
        const ExplorationFrontend::Config &cfg) {
    GlobalCoveragePlanner::Config planner_cfg;
    planner_cfg.grid.cell_size = cfg.global_grid_cell_size;
    planner_cfg.grid.max_active_nodes = cfg.global_max_nodes;
    planner_cfg.cost.max_vel = 3.0;
    planner_cfg.cost.max_acc = 3.0;
    planner_cfg.cost.max_yaw_rate = 2.0;
    planner_cfg.cost.hybrid_search_radius = cfg.global_hybrid_search_radius;
    planner_cfg.cost.unknown_penalty_factor = cfg.global_unknown_penalty_factor;
    planner_cfg.cost.use_astar = cfg.use_astar_cost;
    planner_cfg.cost.unknown_as_occupied_for_motion = cfg.unknown_as_occupied_for_motion;
    planner_cfg.lkh.binary_path = cfg.lkh_binary;
    planner_cfg.lkh.work_dir = cfg.lkh_work_dir;
    planner_cfg.refined_num = cfg.global_refined_num;
    planner_cfg.refined_radius = cfg.global_refined_radius;
    planner_cfg.max_tour_nodes = cfg.global_max_nodes;
    return planner_cfg;
}
}  // namespace

ExplorationFrontend::ExplorationFrontend(const Config &cfg,
                                         const MapManager::Ptr &map_manager,
                                         const path_search::Astar::Ptr &astar)
        : cfg_(cfg),
          map_manager_(map_manager),
          astar_(astar),
          frontier_cluster_manager_(makeFrontierClusterManagerConfig(cfg)),
          viewpoint_selector_(makeViewpointSelectorConfig(cfg), map_manager),
          global_coverage_planner_(makeGlobalCoveragePlannerConfig(cfg), map_manager, astar) {
}

bool ExplorationFrontend::planNextGoal(const StatePVAJ &robot_state,
                                       const double current_yaw,
                                       ExplorationGoal &goal) {
    goal = ExplorationGoal{};
    exploration_finished_ = false;
    latest_debug_info_ = ExplorationDebugInfo{};
    latest_debug_info_.sequence = ++debug_sequence_;
    latest_debug_info_.robot_yaw = current_yaw;
    latest_debug_info_.has_robot_state = robot_state.col(0).allFinite();
    if (latest_debug_info_.has_robot_state) {
        latest_debug_info_.robot_position = robot_state.col(0);
    }

    auto failWithReason = [&](const std::string &reason, const bool finished = false) {
        exploration_finished_ = finished;
        goal.reason = reason;
        setDebugReason(reason, false, finished);
        return false;
    };

    if (!cfg_.enable) {
        return failWithReason("exploration disabled");
    }
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return failWithReason("map manager not ready");
    }

    const Vec3f robot_pos = robot_state.col(0);
    if (!robot_pos.allFinite()) {
        return failWithReason("robot state is not finite");
    }
    if (!map_manager_->insideLocalMap(robot_pos)) {
        return failWithReason("robot is outside local map");
    }

    rog_map::vec_E<FrontierVoxel> frontier_voxels;
    FrontierSearchStats search_stats;
    if (!map_manager_->extractFrontierVoxels(robot_pos,
                                             cfg_.frontier_search_radius,
                                             cfg_.map_resolution,
                                             frontier_voxels,
                                             &search_stats)) {
        latest_debug_info_.search_stats = search_stats;
        return failWithReason("frontier search failed");
    }
    latest_debug_info_.search_stats = search_stats;
    latest_debug_info_.frontier_voxels = frontier_voxels;
    if (frontier_voxels.empty()) {
        if (!mapObservationReady(search_stats)) {
            std::ostringstream oss;
            oss << "map observation not ready"
                << " searched=" << search_stats.searched_cells
                << " free=" << search_stats.known_free_cells
                << " unknown=" << search_stats.unknown_cells
                << " occupied=" << search_stats.occupied_cells;
            if (cfg_.print_log) {
                std::cout << " -- [ExplorationFrontend] Waiting for usable map: "
                          << oss.str() << "." << std::endl;
            }
            return failWithReason(oss.str());
        }
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] Exploration finished: no frontier. "
                      << "searched=" << search_stats.searched_cells
                      << ", free=" << search_stats.known_free_cells
                      << ", unknown=" << search_stats.unknown_cells
                      << ", occupied=" << search_stats.occupied_cells
                      << "." << std::endl;
        }
        return failWithReason("no frontier", true);
    }

    vec_E<FrontierCluster> clusters;
    frontier_cluster_manager_.update(frontier_voxels, *map_manager_, clusters);
    latest_debug_info_.active_clusters = clusters;
    latest_debug_info_.tracked_clusters = frontier_cluster_manager_.trackedClusters();
    if (clusters.empty()) {
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No stable frontier cluster yet. "
                      << "frontiers=" << frontier_voxels.size()
                      << ", min_cluster_size=" << cfg_.min_frontier_cluster_size
                      << ", min_observations=" << cfg_.frontier_lifecycle_min_observations
                      << "." << std::endl;
        }
        return failWithReason("no stable frontier cluster");
    }

    std::sort(clusters.begin(), clusters.end(),
              [&robot_pos](const FrontierCluster &lhs, const FrontierCluster &rhs) {
                  if (lhs.size != rhs.size) {
                      return lhs.size > rhs.size;
                  }
                  return (lhs.center - robot_pos).squaredNorm() <
                         (rhs.center - robot_pos).squaredNorm();
              });

    std::string failure_reason;
    if (!planWithGlobalCoverage(robot_state,
                                current_yaw,
                                clusters,
                                goal,
                                failure_reason)) {
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] Global exploration planning failed: "
                      << failure_reason
                      << ". frontiers=" << frontier_voxels.size()
                      << ", clusters=" << clusters.size()
                      << ", rog_frontier=" << search_stats.used_rog_frontier_extractor
                      << std::endl;
        }
        return failWithReason(failure_reason.empty()
                                      ? std::string("global exploration planning failed")
                                      : failure_reason);
    }

    exploration_finished_ = false;
    markViewpointSelected(goal);
    setDebugReason("selected global coverage frontier viewpoint", true, false);

    if (cfg_.print_log) {
        std::cout << " -- [ExplorationFrontend] Global goal selected: p=["
                  << goal.position.transpose() << "], yaw=" << goal.yaw
                  << ", score=" << goal.score
                  << ", info=" << goal.information_gain
                  << ", travel=" << goal.travel_cost
                  << ", yaw_cost=" << goal.yaw_cost
                  << ", curvature=" << goal.curvature_cost
                  << ", open=" << goal.open_space_score
                  << ", high_speed=" << goal.high_speed_score
                  << ", cluster=" << goal.frontier_cluster_id
                  << ", case=" << goal.viewpoint_case
                  << ", clusters=" << clusters.size()
                  << ", frontiers=" << frontier_voxels.size()
                  << ", rog_frontier=" << search_stats.used_rog_frontier_extractor
                  << std::endl;
    }
    return true;
}

bool ExplorationFrontend::isExplorationFinished() const {
    return exploration_finished_;
}

const ExplorationDebugInfo &ExplorationFrontend::latestDebugInfo() const {
    return latest_debug_info_;
}

void ExplorationFrontend::reset() {
    exploration_finished_ = false;
    frontier_cluster_manager_.reset();
    global_coverage_planner_.reset();
    latest_debug_info_ = ExplorationDebugInfo{};
}

bool ExplorationFrontend::mapObservationReady(const FrontierSearchStats &stats) const {
    const int min_known_free_cells = std::max(10, cfg_.min_frontier_cluster_size);
    return stats.known_free_cells >= min_known_free_cells;
}

bool ExplorationFrontend::planWithGlobalCoverage(const StatePVAJ &robot_state,
                                                 const double current_yaw,
                                                 const rog_map::vec_E<FrontierCluster> &clusters,
                                                 ExplorationGoal &goal,
                                                 std::string &failure_reason) {
    goal = ExplorationGoal{};
    failure_reason.clear();
    const Vec3f robot_pos = robot_state.col(0);

    std::unordered_map<int, vec_E<ExplorationViewpoint>> viewpoints_by_cluster;
    viewpoints_by_cluster.reserve(clusters.size());
    std::unordered_map<int, const FrontierCluster *> cluster_by_id;
    cluster_by_id.reserve(clusters.size());

    int sampled_viewpoint_count = 0;
    for (const auto &cluster : clusters) {
        cluster_by_id.emplace(cluster.id, &cluster);

        vec_E<ExplorationViewpoint> viewpoints;
        viewpoint_selector_.selectViewpoints(cluster, robot_pos, viewpoints);
        if (viewpoints.empty()) {
            continue;
        }

        for (auto &viewpoint : viewpoints) {
            const double open_space = estimateOpenSpaceScore(viewpoint.position);
            const double high_speed = estimateHighSpeedScore(robot_state, viewpoint.position);
            viewpoint.score += 2.0 * high_speed + 0.5 * open_space;
            if (high_speed > 0.65 && open_space > 0.65) {
                viewpoint.viewpoint_case = "high_speed_global";
            }
        }
        std::sort(viewpoints.begin(),
                  viewpoints.end(),
                  [](const ExplorationViewpoint &lhs, const ExplorationViewpoint &rhs) {
                      if (std::abs(lhs.score - rhs.score) > 1.0e-6) {
                          return lhs.score > rhs.score;
                      }
                      return lhs.visible_frontier_cells > rhs.visible_frontier_cells;
                  });

        for (const auto &viewpoint : viewpoints) {
            ExplorationGoal debug_candidate;
            debug_candidate.valid = true;
            debug_candidate.position = viewpoint.position;
            debug_candidate.yaw = viewpoint.yaw;
            debug_candidate.information_gain =
                    static_cast<double>(viewpoint.visible_frontier_cells) +
                    0.1 * viewpoint.unknown_gain;
            debug_candidate.travel_cost = (viewpoint.position - robot_pos).norm();
            debug_candidate.distance_to_robot = debug_candidate.travel_cost;
            debug_candidate.yaw_cost = estimateYawCost(current_yaw, viewpoint.yaw);
            debug_candidate.curvature_cost =
                    estimateCurvatureCost(robot_state, viewpoint.position, cluster);
            debug_candidate.unknown_risk = estimateUnknownRisk(viewpoint.position);
            debug_candidate.open_space_score = estimateOpenSpaceScore(viewpoint.position);
            debug_candidate.velocity_alignment_score =
                    estimateVelocityAlignmentScore(robot_state, viewpoint.position);
            debug_candidate.high_speed_score =
                    estimateHighSpeedScore(robot_state, viewpoint.position);
            debug_candidate.lifecycle_score = estimateLifecycleScore(cluster);
            debug_candidate.frontier_cluster_id = cluster.id;
            debug_candidate.viewpoint_case = viewpoint.viewpoint_case;
            debug_candidate.score = -viewpoint.score;

            appendViewpointDebug(cluster,
                                 viewpoint.position,
                                 viewpoint.yaw,
                                 viewpoint.viewpoint_case,
                                 "accepted_global_candidate",
                                 true,
                                 &debug_candidate);
            ++sampled_viewpoint_count;
        }

        viewpoints_by_cluster.emplace(cluster.id, std::move(viewpoints));
    }

    if (viewpoints_by_cluster.empty()) {
        failure_reason = "no valid known-free viewpoint";
        return false;
    }

    ExplorationCoveragePlan coverage_plan;
    if (!global_coverage_planner_.plan(robot_state,
                                       current_yaw,
                                       clusters,
                                       viewpoints_by_cluster,
                                       coverage_plan)) {
        failure_reason = coverage_plan.reason.empty()
                                 ? std::string("global coverage planner failed")
                                 : coverage_plan.reason;
        return false;
    }
    if (coverage_plan.local_viewpoint_sequence.empty()) {
        failure_reason = "global coverage planner produced no viewpoint";
        return false;
    }

    auto findDebugId = [&](const int cluster_id, const Vec3f &position) {
        int best_id = -1;
        double best_dist = std::numeric_limits<double>::infinity();
        for (const auto &debug : latest_debug_info_.viewpoints) {
            if (debug.frontier_cluster_id != cluster_id) {
                continue;
            }
            const double dist = (debug.position - position).norm();
            if (dist < best_dist) {
                best_dist = dist;
                best_id = debug.debug_id;
            }
        }
        return best_dist <= std::max(0.2, 2.0 * map_manager_->getResolution())
                       ? best_id
                       : -1;
    };

    for (const auto &viewpoint : coverage_plan.local_viewpoint_sequence) {
        const int debug_id = findDebugId(viewpoint.frontier_cluster_id, viewpoint.position);
        if (debug_id >= 0) {
            markViewpointReachable(debug_id,
                                   (viewpoint.position - robot_pos).norm(),
                                   -viewpoint.score);
        }
    }

    const auto &selected_viewpoint = coverage_plan.local_viewpoint_sequence.front();
    const auto cluster_it = cluster_by_id.find(selected_viewpoint.frontier_cluster_id);
    const FrontierCluster *selected_cluster =
            cluster_it == cluster_by_id.end() ? nullptr : cluster_it->second;

    goal.valid = true;
    goal.position = selected_viewpoint.position;
    goal.yaw = selected_viewpoint.yaw;
    goal.information_gain =
            static_cast<double>(selected_viewpoint.visible_frontier_cells) +
            0.1 * selected_viewpoint.unknown_gain;
    goal.guide_path = coverage_plan.guide_path;
    if (goal.guide_path.empty()) {
        goal.guide_path.push_back(robot_pos);
        goal.guide_path.push_back(goal.position);
    }
    goal.travel_cost = pathLength(goal.guide_path);
    goal.distance_to_robot = (goal.position - robot_pos).norm();
    goal.yaw_cost = estimateYawCost(current_yaw, goal.yaw);
    goal.unknown_risk = estimateUnknownRisk(goal.position);
    goal.open_space_score = estimateOpenSpaceScore(goal.position);
    goal.velocity_alignment_score = estimateVelocityAlignmentScore(robot_state, goal.position);
    goal.high_speed_score = estimateHighSpeedScore(robot_state, goal.position);
    goal.frontier_cluster_id = selected_viewpoint.frontier_cluster_id;
    goal.viewpoint_debug_id = findDebugId(goal.frontier_cluster_id, goal.position);
    goal.viewpoint_case = selected_viewpoint.viewpoint_case.empty()
                                  ? std::string("global_coverage")
                                  : selected_viewpoint.viewpoint_case;
    if (selected_cluster != nullptr) {
        goal.curvature_cost = estimateCurvatureCost(robot_state, goal.position, *selected_cluster);
        goal.lifecycle_score = estimateLifecycleScore(*selected_cluster);
    }
    goal.score = scoreCandidate(goal, goal.unknown_risk) + 0.1 * coverage_plan.tour_cost;
    goal.reason = coverage_plan.used_lkh
                          ? "selected by LKH ATSP global coverage"
                          : "selected by fallback ATSP global coverage";

    if (goal.viewpoint_debug_id >= 0) {
        markViewpointReachable(goal.viewpoint_debug_id, goal.travel_cost, goal.score);
    }

    if (cfg_.print_log) {
        std::cout << " -- [ExplorationFrontend] Global coverage plan: "
                  << "coverage_nodes=" << global_coverage_planner_.activeCoverageNodes().size()
                  << ", cluster_tour=" << coverage_plan.cluster_tour.size()
                  << ", refined_views=" << coverage_plan.local_viewpoint_sequence.size()
                  << ", valid_view_clusters=" << viewpoints_by_cluster.size()
                  << ", sampled_viewpoints=" << sampled_viewpoint_count
                  << ", used_lkh=" << coverage_plan.used_lkh
                  << ", tour_cost=" << coverage_plan.tour_cost
                  << ", reason=" << coverage_plan.reason
                  << std::endl;
    }

    return true;
}

void ExplorationFrontend::sampleViewpointsForCluster(const FrontierCluster &cluster,
                                                     const StatePVAJ &robot_state,
                                                     const double current_yaw,
                                                     vec_E<ExplorationGoal> &candidates) {
    const int max_candidate_num = std::max(1, cfg_.max_candidate_num);
    const int yaw_num = std::max(1, cfg_.viewpoint_yaw_sample_num);
    const int radius_num = std::max(1, cfg_.viewpoint_radius_sample_num);
    const double min_radius = std::max(0.0, cfg_.viewpoint_min_distance);
    const double max_radius = std::max(min_radius, cfg_.viewpoint_max_distance);
    const Vec3f robot_pos = robot_state.col(0);
    const std::size_t initial_candidate_count = candidates.size();

    Vec3f unknown_dir = cluster.unknown_direction;
    unknown_dir.z() = 0.0;
    if (unknown_dir.norm() < 1.0e-6) {
        unknown_dir = cluster.center - robot_pos;
        unknown_dir.z() = 0.0;
    }
    if (unknown_dir.norm() < 1.0e-6) {
        unknown_dir = Vec3f::UnitX();
    }
    unknown_dir.normalize();
    const Vec3f known_free_dir = -unknown_dir;
    const Vec3f tangent(-unknown_dir.y(), unknown_dir.x(), 0.0);

    auto tryAddCandidate = [&](Vec3f viewpoint, const std::string &viewpoint_case) {
        if (static_cast<int>(candidates.size()) >= max_candidate_num) {
            return false;
        }
        viewpoint.z() = cluster.center.z() + cfg_.viewpoint_height_offset;
        const double distance_to_robot = (viewpoint - robot_pos).norm();
        const double yaw = std::atan2(cluster.center.y() - viewpoint.y(),
                                      cluster.center.x() - viewpoint.x());
        if (distance_to_robot <= std::max(0.2, cfg_.goal_reached_distance)) {
            appendViewpointDebug(cluster,
                                 viewpoint,
                                 yaw,
                                 viewpoint_case,
                                 "too_close_to_robot",
                                 false);
            return true;
        }
        if (!viewpointSafe(viewpoint)) {
            appendViewpointDebug(cluster,
                                 viewpoint,
                                 yaw,
                                 viewpoint_case,
                                 "not_known_free_or_not_safe",
                                 false);
            return true;
        }
        if (!viewpointVisible(viewpoint, cluster)) {
            appendViewpointDebug(cluster,
                                 viewpoint,
                                 yaw,
                                 viewpoint_case,
                                 "frontier_not_visible",
                                 false);
            return true;
        }

        const double information_gain = estimateInformationGain(viewpoint, cluster);
        if (information_gain < cfg_.min_information_gain) {
            ExplorationGoal rejected;
            rejected.information_gain = information_gain;
            appendViewpointDebug(cluster,
                                 viewpoint,
                                 yaw,
                                 viewpoint_case,
                                 "low_information_gain",
                                 false,
                                 &rejected);
            return true;
        }

        ExplorationGoal candidate;
        candidate.valid = true;
        candidate.position = viewpoint;
        candidate.yaw = yaw;
        candidate.information_gain = information_gain;
        candidate.distance_to_robot = distance_to_robot;
        candidate.travel_cost = distance_to_robot;
        candidate.yaw_cost = estimateYawCost(current_yaw, candidate.yaw);
        candidate.curvature_cost = estimateCurvatureCost(robot_state, viewpoint, cluster);
        candidate.unknown_risk = estimateUnknownRisk(viewpoint);
        candidate.open_space_score = estimateOpenSpaceScore(viewpoint);
        candidate.velocity_alignment_score = estimateVelocityAlignmentScore(robot_state, viewpoint);
        candidate.high_speed_score = estimateHighSpeedScore(robot_state, viewpoint);
        candidate.lifecycle_score = estimateLifecycleScore(cluster);
        candidate.frontier_cluster_id = cluster.id;
        candidate.viewpoint_case = viewpoint_case;
        candidate.score = scoreCandidate(candidate, candidate.unknown_risk);
        candidate.reason = "sampled frontier viewpoint";
        candidate.viewpoint_debug_id = appendViewpointDebug(cluster,
                                                            viewpoint,
                                                            candidate.yaw,
                                                            viewpoint_case,
                                                            "accepted_pending_reachability",
                                                            true,
                                                            &candidate);
        candidates.push_back(candidate);
        return true;
    };

    for (int ri = 0; ri < radius_num; ++ri) {
        const double alpha = radius_num == 1
                                     ? 0.0
                                     : static_cast<double>(ri) / static_cast<double>(radius_num - 1);
        const double radius = min_radius + alpha * (max_radius - min_radius);
        if (!tryAddCandidate(cluster.center + known_free_dir * radius, "normal")) {
            return;
        }

        const double side_offset =
                std::max(map_manager_->getResolution(), cfg_.viewpoint_side_step) *
                static_cast<double>(ri + 1);
        if (!tryAddCandidate(cluster.center + known_free_dir * radius + tangent * side_offset,
                             "side_left")) {
            return;
        }
        if (!tryAddCandidate(cluster.center + known_free_dir * radius - tangent * side_offset,
                             "side_right")) {
            return;
        }
    }

    Vec3f vel_dir = robot_state.col(1);
    vel_dir.z() = 0.0;
    if (vel_dir.norm() > std::max(0.05, cfg_.high_speed_min_speed)) {
        vel_dir.normalize();
        const double side_sign = vel_dir.dot(tangent) >= 0.0 ? 1.0 : -1.0;
        const Vec3f high_speed_viewpoint =
                cluster.center +
                known_free_dir * max_radius +
                tangent * side_sign * std::max(cfg_.viewpoint_side_step, map_manager_->getResolution());
        if (!tryAddCandidate(high_speed_viewpoint, "high_speed")) {
            return;
        }
    }

    if (candidates.size() > initial_candidate_count) {
        return;
    }

    for (int ri = 0; ri < radius_num; ++ri) {
        const double alpha = radius_num == 1
                                     ? 0.0
                                     : static_cast<double>(ri) / static_cast<double>(radius_num - 1);
        const double radius = min_radius + alpha * (max_radius - min_radius);
        for (int yi = 0; yi < yaw_num; ++yi) {
            const double angle = 2.0 * kPi * static_cast<double>(yi) / static_cast<double>(yaw_num);
            if (!tryAddCandidate(cluster.center +
                                         radius * Vec3f(std::cos(angle), std::sin(angle), 0.0),
                                 "orbit_fallback")) {
                return;
            }
        }
    }
}

int ExplorationFrontend::appendViewpointDebug(const FrontierCluster &cluster,
                                              const Vec3f &viewpoint,
                                              const double yaw,
                                              const std::string &viewpoint_case,
                                              const std::string &status,
                                              const bool accepted,
                                              const ExplorationGoal *candidate) {
    ExplorationViewpointDebug debug;
    debug.debug_id = static_cast<int>(latest_debug_info_.viewpoints.size());
    debug.frontier_cluster_id = cluster.id;
    debug.position = viewpoint;
    debug.yaw = yaw;
    debug.accepted = accepted;
    debug.viewpoint_case = viewpoint_case;
    debug.status = status;

    if (candidate != nullptr) {
        debug.score = candidate->score;
        debug.information_gain = candidate->information_gain;
        debug.travel_cost = candidate->travel_cost;
        debug.yaw_cost = candidate->yaw_cost;
        debug.curvature_cost = candidate->curvature_cost;
        debug.unknown_risk = candidate->unknown_risk;
        debug.open_space_score = candidate->open_space_score;
        debug.high_speed_score = candidate->high_speed_score;
        debug.lifecycle_score = candidate->lifecycle_score;
    }

    latest_debug_info_.viewpoints.push_back(debug);
    return debug.debug_id;
}

void ExplorationFrontend::markViewpointReachable(const int debug_id,
                                                 const double travel_cost,
                                                 const double score) {
    if (debug_id < 0 ||
        debug_id >= static_cast<int>(latest_debug_info_.viewpoints.size())) {
        return;
    }

    auto &debug = latest_debug_info_.viewpoints[debug_id];
    debug.travel_cost = travel_cost;
    debug.score = score;
    if (!std::isfinite(travel_cost) || travel_cost >= kInfCost) {
        debug.reachable = false;
        debug.status = "unreachable";
        return;
    }

    debug.reachable = true;
    debug.status = "reachable";
}

void ExplorationFrontend::markViewpointSelected(const ExplorationGoal &goal) {
    latest_debug_info_.selected_goal.valid = goal.valid;
    latest_debug_info_.selected_goal.position = goal.position;
    latest_debug_info_.selected_goal.yaw = goal.yaw;
    latest_debug_info_.selected_goal.frontier_cluster_id = goal.frontier_cluster_id;
    latest_debug_info_.selected_goal.viewpoint_debug_id = goal.viewpoint_debug_id;
    latest_debug_info_.selected_goal.viewpoint_case = goal.viewpoint_case;
    latest_debug_info_.selected_goal.score = goal.score;
    latest_debug_info_.selected_goal.information_gain = goal.information_gain;
    latest_debug_info_.selected_goal.travel_cost = goal.travel_cost;
    latest_debug_info_.selected_goal.guide_path = goal.guide_path;

    const int debug_id = goal.viewpoint_debug_id;
    if (debug_id >= 0 &&
        debug_id < static_cast<int>(latest_debug_info_.viewpoints.size())) {
        auto &debug = latest_debug_info_.viewpoints[debug_id];
        debug.selected = true;
        debug.reachable = true;
        debug.status = "selected";
        debug.score = goal.score;
        debug.travel_cost = goal.travel_cost;
    }
}

void ExplorationFrontend::setDebugReason(const std::string &reason,
                                         const bool planning_success,
                                         const bool exploration_finished) {
    latest_debug_info_.reason = reason;
    latest_debug_info_.planning_success = planning_success;
    latest_debug_info_.exploration_finished = exploration_finished;
}

bool ExplorationFrontend::viewpointSafe(const Vec3f &viewpoint) const {
    if (!viewpoint.allFinite() ||
        map_manager_ == nullptr ||
        !map_manager_->ready()) {
        return false;
    }

    return map_manager_->isKnownFreeForViewpoint(viewpoint, cfg_.viewpoint_safe_distance);
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

double ExplorationFrontend::estimateOpenSpaceScore(const Vec3f &viewpoint) const {
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return 0.0;
    }

    const double radius = std::max(map_manager_->getResolution(), cfg_.high_speed_open_space_radius);
    const double step = std::max(2.0 * map_manager_->getResolution(), 0.35);
    const int max_step = std::max(1, static_cast<int>(std::ceil(radius / step)));
    int checked_count = 0;
    int free_count = 0;

    for (int ix = -max_step; ix <= max_step; ++ix) {
        for (int iy = -max_step; iy <= max_step; ++iy) {
            const Vec3f offset(step * static_cast<double>(ix),
                               step * static_cast<double>(iy),
                               0.0);
            if (offset.squaredNorm() > radius * radius) {
                continue;
            }
            const Vec3f query = viewpoint + offset;
            if (!map_manager_->insideLocalMap(query)) {
                continue;
            }
            ++checked_count;
            if (map_manager_->isKnownFreeForViewpoint(query, 0.0)) {
                ++free_count;
            }
        }
    }

    if (checked_count == 0) {
        return 0.0;
    }
    return static_cast<double>(free_count) / static_cast<double>(checked_count);
}

double ExplorationFrontend::estimateVelocityAlignmentScore(const StatePVAJ &robot_state,
                                                           const Vec3f &viewpoint) const {
    Vec3f vel = robot_state.col(1);
    Vec3f to_goal = viewpoint - robot_state.col(0);
    vel.z() = 0.0;
    to_goal.z() = 0.0;
    const double vel_norm = vel.norm();
    const double goal_norm = to_goal.norm();
    if (goal_norm < 1.0e-6) {
        return 0.0;
    }
    if (vel_norm < std::max(0.05, cfg_.high_speed_min_speed)) {
        return 0.5;
    }
    return std::clamp(vel.dot(to_goal) / (vel_norm * goal_norm), 0.0, 1.0);
}

double ExplorationFrontend::estimateHighSpeedScore(const StatePVAJ &robot_state,
                                                   const Vec3f &viewpoint) const {
    const double open_space_score = estimateOpenSpaceScore(viewpoint);
    const double alignment_score = estimateVelocityAlignmentScore(robot_state, viewpoint);
    const double distance = (viewpoint - robot_state.col(0)).norm();
    const double distance_score =
            std::clamp(distance / std::max(1.0, cfg_.viewpoint_max_distance), 0.0, 1.0);
    return open_space_score * (0.5 + 0.5 * alignment_score) * (0.5 + 0.5 * distance_score);
}

double ExplorationFrontend::estimateLifecycleScore(const FrontierCluster &cluster) const {
    const double age_score =
            std::clamp(static_cast<double>(cluster.age) /
                       static_cast<double>(std::max(1, cfg_.frontier_lifecycle_min_observations + 3)),
                       0.0,
                       1.0);
    const double size_score =
            std::clamp(std::log1p(static_cast<double>(std::max(0, cluster.size))) / 5.0,
                       0.0,
                       1.0);
    return 0.5 * age_score + 0.5 * size_score;
}

double ExplorationFrontend::scoreCandidate(const ExplorationGoal &candidate,
                                           const double unknown_risk) const {
    return cfg_.weight_travel * candidate.travel_cost +
           cfg_.weight_yaw * candidate.yaw_cost +
           cfg_.weight_curvature * candidate.curvature_cost +
           cfg_.weight_info_gain * candidate.information_gain +
           cfg_.weight_unknown_risk * unknown_risk +
           cfg_.weight_high_speed * candidate.high_speed_score +
           cfg_.weight_lifecycle * candidate.lifecycle_score;
}

bool ExplorationFrontend::isUnknownLike(const rog_map::GridType type) const {
    return type == rog_map::GridType::UNKNOWN ||
           type == rog_map::GridType::UNDEFINED ||
           type == rog_map::GridType::FRONTIER;
}

double ExplorationFrontend::wrapAngleDiff(const double lhs, const double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

}  // namespace general_planner
