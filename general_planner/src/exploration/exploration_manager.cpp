#include "exploration/exploration_manager.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <boost/filesystem.hpp>

namespace general_planner {
namespace exploration {

namespace {
double headingFromPath(const super_utils::vec_E<super_utils::Vec3f> &path,
                       const std::size_t index,
                       const double fallback_yaw) {
    if (path.size() < 2U) {
        return fallback_yaw;
    }
    const std::size_t next = std::min(index + 1U, path.size() - 1U);
    const super_utils::Vec3f delta = path[next] - path[index];
    if (delta.head<2>().norm() < 1.0e-6) {
        return fallback_yaw;
    }
    return std::atan2(delta.y(), delta.x());
}
}  // namespace

ExplorationManager::ExplorationManager(Config cfg,
                                       MapManager::Ptr map_manager,
                                       path_search::Astar::Ptr astar,
                                       ros_interface::RosInterface::Ptr ros_ptr)
        : cfg_(std::move(cfg)),
          map_manager_(std::move(map_manager)),
          astar_(std::move(astar)),
          ros_ptr_(std::move(ros_ptr)) {
    validateRuntimeConfig();
    observation_map_ = std::make_shared<ObservationMap>(cfg_.observation_map);
    map_adapter_ = std::make_shared<EpicMapAdapter>(map_manager_);
    frontier_db_ = std::make_unique<FrontierDatabase>(cfg_.frontier_database);
    viewpoint_manager_ = std::make_unique<ViewpointManager>(cfg_.viewpoint_manager);
    topo_graph_ = std::make_unique<TopoGraph>(cfg_.topo_graph, map_manager_, astar_, observation_map_);
    global_guidance_planner_ = std::make_unique<GlobalGuidancePlanner>(cfg_.global_guidance);
}

void ExplorationManager::updateObservation(const rog_map::PointCloud &cloud,
                                           const super_utils::Pose &pose,
                                           const CloudFrame frame,
                                           const super_utils::Vec3f &sensor_position,
                                           const double stamp) {
    if (!cfg_.enable || observation_map_ == nullptr) {
        return;
    }
    if (has_last_observation_position_) {
        travel_distance_ += (sensor_position - last_observation_position_).norm();
    }
    last_observation_position_ = sensor_position;
    has_last_observation_position_ = true;
    if (map_adapter_ != nullptr) {
        map_adapter_->updateCloudOdom(cloud, pose, frame, sensor_position);
    }
    observation_map_->update(cloud, pose, frame, sensor_position, travel_distance_, stamp);
}

int ExplorationManager::updateFrontiers(const double stamp) {
    if (!cfg_.enable || observation_map_ == nullptr || frontier_db_ == nullptr) {
        return 0;
    }
    std::vector<SurfaceFrontierCluster> clusters;
    observation_map_->getFrontierClusters(clusters);
    frontier_db_->update(clusters, stamp);
    return frontier_db_->activeCount();
}

void ExplorationManager::updateTopoGraph(const rog_map::RobotState &robot,
                                         const double current_yaw,
                                         const double stamp_in) {
    if (!cfg_.enable || !robot.rcv || frontier_db_ == nullptr ||
        viewpoint_manager_ == nullptr || topo_graph_ == nullptr) {
        return;
    }
    const double stamp = stamp_in >= 0.0 ? stamp_in : (ros_ptr_ ? ros_ptr_->getSimTime() : 0.0);
    topo_graph_->updateOdomNode(robot.p, current_yaw);
    topo_graph_->updateHistoryOdomNodes(robot.p, current_yaw);

    std::vector<FrontierRecord> active_frontiers = frontier_db_->getActiveFrontiers();
    for (const auto &frontier : active_frontiers) {
        std::vector<ExplorationViewpoint> viewpoints;
        ExplorationViewpoint best;
        viewpoint_manager_->generateBestViewpoints(frontier,
                                                   *observation_map_,
                                                   map_manager_,
                                                   map_adapter_,
                                                   topo_graph_.get(),
                                                   robot.p,
                                                   current_yaw,
                                                   stamp,
                                                   viewpoints,
                                                   best);
        frontier_db_->setViewpoints(frontier.stable_id, viewpoints, best, stamp);
    }

    active_frontiers = frontier_db_->getActiveFrontiers();
    topo_graph_->insertOrUpdateFrontierNodes(active_frontiers);
}

bool ExplorationManager::planGlobalTour(const rog_map::RobotState &robot,
                                        const double current_yaw,
                                        GlobalGuidanceResult &result) {
    result = GlobalGuidanceResult{};
    if (!cfg_.enable || !cfg_.use_epic_frontend || !robot.rcv ||
        frontier_db_ == nullptr || topo_graph_ == nullptr ||
        global_guidance_planner_ == nullptr) {
        result.reason = !cfg_.enable ? "exploration disabled" : "exploration modules are not ready";
        return false;
    }
    (void)current_yaw;
    const std::vector<FrontierRecord> active_frontiers = frontier_db_->getActiveFrontiers();
    if (!global_guidance_planner_->buildGuidance(robot.p,
                                                 travel_distance_,
                                                 active_frontiers,
                                                 *topo_graph_,
                                                 current_target_frontier_id_,
                                                 current_route_,
                                                 result)) {
        return false;
    }
    current_route_ = result.route;
    current_target_frontier_id_ = current_route_.target_frontier_id;
    return true;
}

bool ExplorationManager::planNextGoal(const rog_map::RobotState &robot,
                                      const double current_yaw,
                                      ExplorationGoal &goal) {
    ExplorationPlan plan;
    const bool ok = planOnce(robot, current_yaw, plan);
    goal = plan.goal;
    return ok;
}

bool ExplorationManager::planOnce(const rog_map::RobotState &robot,
                                  const double current_yaw,
                                  ExplorationPlan &plan) {
    plan = ExplorationPlan{};
    ExplorationGoal &goal = plan.goal;
    goal = ExplorationGoal{};
    const double stamp = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
    if (!cfg_.enable) {
        goal.reason = "exploration disabled";
        plan.reason = goal.reason;
        logPlanSummary(goal, goal.reason, false);
        return false;
    }
    if (!cfg_.use_epic_frontend) {
        goal.reason = "epic exploration frontend disabled";
        plan.reason = goal.reason;
        logPlanSummary(goal, goal.reason, false);
        return false;
    }
    if (observation_map_ == nullptr || frontier_db_ == nullptr ||
        viewpoint_manager_ == nullptr || topo_graph_ == nullptr ||
        global_guidance_planner_ == nullptr) {
        goal.reason = "exploration modules are not ready";
        plan.reason = goal.reason;
        logPlanSummary(goal, goal.reason, false);
        return false;
    }
    if (!robot.rcv) {
        goal.reason = "no odom";
        plan.reason = goal.reason;
        logPlanSummary(goal, goal.reason, false);
        return false;
    }

    updateFrontiers(stamp);
    updateTopoGraph(robot, current_yaw, stamp);
    std::vector<FrontierRecord> active_frontiers = frontier_db_->getActiveFrontiers();

    if (active_frontiers.empty()) {
        exploration_finished_ = observation_map_->observedVoxelCount() > 0;
        goal.reason = exploration_finished_ ? "no active frontier" : "observation map empty";
        plan.no_frontier = true;
        plan.reason = goal.reason;
        logPlanSummary(goal, goal.reason, false);
        visualizePlan(plan, active_frontiers);
        return false;
    }

    GlobalGuidanceResult guidance;
    if (!planGlobalTour(robot, current_yaw, guidance)) {
        current_target_frontier_id_ = -1;
        current_route_ = GlobalRoute{};
        goal.reason = guidance.reason;
        plan.reason = goal.reason;
        logPlanSummary(goal, goal.reason, false);
        visualizePlan(plan, active_frontiers);
        return false;
    }

    current_route_ = guidance.route;
    current_target_frontier_id_ = current_route_.target_frontier_id;
    frontier_db_->markSelected(current_target_frontier_id_, stamp);

    if (!selectNextLocalSubgoal(current_route_, robot, goal)) {
        frontier_db_->onFailed(current_target_frontier_id_, stamp);
        current_target_frontier_id_ = -1;
        current_route_ = GlobalRoute{};
        goal.reason = "global route has no local reachable subgoal";
        plan.reason = goal.reason;
        logPlanSummary(goal, goal.reason, false);
        visualizePlan(plan, active_frontiers);
        return false;
    }

    if (detectStuck(goal, robot.p, stamp)) {
        frontier_db_->markBlacklisted(current_target_frontier_id_, stamp);
        current_target_frontier_id_ = -1;
        current_route_ = GlobalRoute{};
        goal.valid = false;
        goal.reason = "stuck detected, blacklist current frontier";
        plan.reason = goal.reason;
        logPlanSummary(goal, goal.reason, false);
        visualizePlan(plan, active_frontiers);
        return false;
    }

    exploration_finished_ = false;
    plan.valid = true;
    plan.no_frontier = false;
    plan.next_goal = goal.position;
    plan.next_yaw = goal.yaw;
    plan.guide_path = buildGuidePathToGoal(goal.route, robot.p, goal.position);
    plan.reason = "goal selected";
    logPlanSummary(goal, "goal selected", false);
    visualizePlan(plan, active_frontiers);
    return true;
}

void ExplorationManager::onGoalReached(const ExplorationGoal &goal, const double stamp) {
    if (!frontier_db_ || !goal.valid) {
        return;
    }
    if (goal.type == ExplorationGoalType::FRONTIER_VIEWPOINT) {
        frontier_db_->markCovered(goal.frontier_id, stamp);
        if (ros_ptr_) {
            ros_ptr_->info(" -- [Exploration] Frontier covered id={}.", goal.frontier_id);
        }
    }
}

void ExplorationManager::onGoalFailed(const ExplorationGoal &goal,
                                      const std::string &reason,
                                      const double stamp) {
    if (!frontier_db_ || goal.frontier_id < 0) {
        return;
    }
    frontier_db_->onFailed(goal.frontier_id, stamp);
    if (ros_ptr_) {
        ros_ptr_->warn(" -- [Exploration] Goal failed frontier_id={}, reason={}.",
                       goal.frontier_id, reason);
    }
}

void ExplorationManager::onLowGain(const ExplorationGoal &goal,
                                   const double actual_gain,
                                   const double stamp) {
    if (!frontier_db_ || goal.frontier_id < 0) {
        return;
    }
    frontier_db_->onLowGain(goal.frontier_id, actual_gain, stamp);
}

void ExplorationManager::reset() {
    if (observation_map_) {
        observation_map_->reset();
    }
    if (frontier_db_) {
        frontier_db_->reset();
    }
    if (topo_graph_) {
        topo_graph_->reset();
    }
    if (global_guidance_planner_) {
        global_guidance_planner_->reset();
    }
    current_target_frontier_id_ = -1;
    current_route_ = GlobalRoute{};
    repeated_goal_count_ = 0;
    last_selected_goal_ = ExplorationGoal{};
    last_significant_gain_time_ = -1.0;
    last_explored_volume_ = 0.0;
    travel_distance_ = 0.0;
    has_last_observation_position_ = false;
    exploration_finished_ = false;
}

void ExplorationManager::validateRuntimeConfig() const {
    if (!cfg_.enable || !cfg_.use_epic_frontend) {
        return;
    }

    if (map_manager_ == nullptr || !map_manager_->hasPointCloudMap()) {
        throw std::runtime_error(
                "EPIC exploration frontend requires PointCloudMap when exploration/use_epic_frontend=true");
    }

    const bool bbox_valid =
            cfg_.observation_map.bbox_min_x < cfg_.observation_map.bbox_max_x &&
            cfg_.observation_map.bbox_min_y < cfg_.observation_map.bbox_max_y &&
            cfg_.observation_map.bbox_min_z < cfg_.observation_map.bbox_max_z;
    if (!bbox_valid) {
        throw std::invalid_argument("EPIC exploration observation map boundaries are invalid");
    }

    if (cfg_.global_guidance.tsp_dir.empty()) {
        throw std::invalid_argument("EPIC exploration tsp_dir is empty");
    }

    boost::system::error_code ec;
    const boost::filesystem::path tsp_dir(cfg_.global_guidance.tsp_dir);
    boost::filesystem::create_directories(tsp_dir, ec);
    if (ec || !boost::filesystem::is_directory(tsp_dir)) {
        throw std::runtime_error("EPIC exploration tsp_dir does not exist or cannot be created: " +
                                 cfg_.global_guidance.tsp_dir);
    }

    const boost::filesystem::path probe_path =
            tsp_dir / ".general_planner_epic_write_probe";
    {
        std::ofstream probe(probe_path.string(), std::ios::out | std::ios::trunc);
        if (!probe.good()) {
            throw std::runtime_error("EPIC exploration tsp_dir is not writable: " +
                                     cfg_.global_guidance.tsp_dir);
        }
    }
    boost::filesystem::remove(probe_path, ec);
}

super_utils::vec_E<super_utils::Vec3f> ExplorationManager::buildGuidePathToGoal(
        const GlobalRoute &route,
        const super_utils::Vec3f &robot_pos,
        const super_utils::Vec3f &goal_pos) {
    super_utils::vec_E<super_utils::Vec3f> guide;
    auto append_unique = [&guide](const super_utils::Vec3f &point) {
        if (!point.allFinite()) {
            return;
        }
        if (guide.empty() || (guide.back() - point).norm() > 1.0e-4) {
            guide.emplace_back(point);
        }
    };

    append_unique(robot_pos);
    if (route.path.empty()) {
        append_unique(goal_pos);
        return guide;
    }

    std::size_t goal_index = 0U;
    double best_dist = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < route.path.size(); ++i) {
        const double dist = (route.path[i] - goal_pos).squaredNorm();
        if (dist < best_dist) {
            best_dist = dist;
            goal_index = i;
        }
    }

    for (std::size_t i = 0; i <= goal_index && i < route.path.size(); ++i) {
        append_unique(route.path[i]);
    }
    append_unique(goal_pos);
    return guide;
}

bool ExplorationManager::selectNextLocalSubgoal(const GlobalRoute &route,
                                                const rog_map::RobotState &robot,
                                                ExplorationGoal &goal) const {
    goal = ExplorationGoal{};
    if (!route.valid || route.target_frontier_id < 0) {
        return false;
    }
    FrontierRecord target;
    if (!frontier_db_->getFrontier(route.target_frontier_id, target)) {
        return false;
    }

    if (target.best_viewpoint.global_safe &&
        routeWaypointSafe(target.best_viewpoint.position)) {
        goal.valid = true;
        goal.type = ExplorationGoalType::FRONTIER_VIEWPOINT;
        goal.position = target.best_viewpoint.position;
        goal.yaw = target.best_viewpoint.yaw;
        goal.frontier_id = target.stable_id;
        goal.gain = target.best_viewpoint.gain_raw;
        goal.travel_cost = route.cost;
        goal.route = route;
        return true;
    }

    for (std::size_t i = 0; i < route.path.size(); ++i) {
        const super_utils::Vec3f &candidate = route.path[i];
        const double dist = (candidate - robot.p).norm();
        if (dist < cfg_.route_waypoint_min_distance) {
            continue;
        }
        if (dist > std::max(cfg_.route_waypoint_max_distance,
                            cfg_.route_waypoint_lookahead)) {
            continue;
        }
        if (!routeWaypointSafe(candidate)) {
            continue;
        }
        goal.valid = true;
        goal.type = ExplorationGoalType::GLOBAL_ROUTE_WAYPOINT;
        goal.position = candidate;
        goal.yaw = headingFromPath(route.path, i, target.best_viewpoint.yaw);
        goal.frontier_id = target.stable_id;
        goal.travel_cost = route.cost;
        goal.route = route;
        return true;
    }

    if (!route.path.empty()) {
        const auto &last = route.path.back();
        if (routeWaypointSafe(last)) {
            goal.valid = true;
            goal.type = ExplorationGoalType::GLOBAL_ROUTE_WAYPOINT;
            goal.position = last;
            goal.yaw = target.best_viewpoint.yaw;
            goal.frontier_id = target.stable_id;
            goal.travel_cost = route.cost;
            goal.route = route;
            return true;
        }
    }
    return false;
}

bool ExplorationManager::routeWaypointSafe(const super_utils::Vec3f &position) const {
    if (observation_map_ != nullptr) {
        const double surface_distance = observation_map_->nearestSurfaceDistance(
                position, std::max(1.0, cfg_.local_goal_safe_distance * 2.0));
        if (surface_distance < cfg_.local_goal_safe_distance) {
            return false;
        }
    }
    if (!cfg_.use_local_map_goal_safety) {
        return true;
    }
    if (map_manager_ == nullptr || !map_manager_->insideLocalMap(position)) {
        return true;
    }
    const auto grid = map_manager_->getGridType(position);
    const auto inf = map_manager_->getInfGridType(position);
    if (grid == rog_map::GridType::OCCUPIED ||
        grid == rog_map::GridType::OUT_OF_MAP ||
        inf == rog_map::GridType::OCCUPIED ||
        inf == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }
    if (map_manager_->hasESDF()) {
        return map_manager_->getESDFDistance(position) >= cfg_.local_goal_safe_distance;
    }
    return true;
}

bool ExplorationManager::detectStuck(const ExplorationGoal &goal,
                                     const super_utils::Vec3f &robot_pos,
                                     const double stamp) {
    if (!goal.valid) {
        return false;
    }
    if (last_selected_goal_.valid &&
        (goal.position - last_selected_goal_.position).norm() < cfg_.repeated_goal_distance) {
        repeated_goal_count_++;
    } else {
        repeated_goal_count_ = 0;
    }
    last_selected_goal_ = goal;
    if (repeated_goal_count_ >= cfg_.repeated_goal_threshold) {
        if (ros_ptr_) {
            ros_ptr_->warn(" -- [Exploration] STUCK_DETECTED repeated_goal_count={}, frontier_id={}, action=blacklist_and_switch.",
                           repeated_goal_count_, goal.frontier_id);
        }
        (void)robot_pos;
        (void)stamp;
        return true;
    }
    return false;
}

void ExplorationManager::logPlanSummary(const ExplorationGoal &goal,
                                        const std::string &reason,
                                        const bool fallback) const {
    if (!cfg_.print_log || !ros_ptr_) {
        return;
    }
    ros_ptr_->info(" -- [Exploration] Global frontiers observation_frontier_count={}, observation_voxels={}.",
                   observation_map_ ? observation_map_->frontierVoxelCount() : 0,
                   observation_map_ ? observation_map_->observedVoxelCount() : 0);
    ros_ptr_->info(" -- [Exploration] FrontierDatabase active={}.",
                   frontier_db_ ? frontier_db_->activeCount() : 0);
    ros_ptr_->info(" -- [Exploration] TopoGraph nodes={}, edges={}.",
                   topo_graph_ ? topo_graph_->nodeCount() : 0,
                   topo_graph_ ? topo_graph_->edgeCount() : 0);
    ros_ptr_->info(" -- [Exploration] GlobalTour target frontier={}, route_valid={}, route_cost={:.3f}.",
                   current_target_frontier_id_,
                   current_route_.valid,
                   current_route_.cost);
    ros_ptr_->info(" -- [Exploration] Goal selected type={}, frontier_id={}, p=[{:.3f} {:.3f} {:.3f}], fallback_to_local={}, reason={}.",
                   goalTypeName(goal.type),
                   goal.frontier_id,
                   goal.position.x(),
                   goal.position.y(),
                   goal.position.z(),
                   fallback,
                   reason);
}

void ExplorationManager::visualizePlan(const ExplorationPlan &plan,
                                       const std::vector<FrontierRecord> &active_frontiers) const {
    if (!ros_ptr_) {
        return;
    }
    std::vector<ExplorationViewpoint> viewpoints;
    for (const auto &frontier : active_frontiers) {
        viewpoints.insert(viewpoints.end(), frontier.viewpoints.begin(), frontier.viewpoints.end());
    }

    std::vector<ExplorationTopoNode> topo_nodes;
    std::vector<ExplorationTopoEdge> topo_edges;
    if (topo_graph_) {
        topo_graph_->getGraph(topo_nodes, topo_edges);
    }

    ros_ptr_->vizExplorationFrontierClusters(active_frontiers);
    ros_ptr_->vizExplorationTopoGraph(topo_nodes, topo_edges);
    ros_ptr_->vizExplorationViewpoints(viewpoints);
    if (!plan.guide_path.empty()) {
        ros_ptr_->vizExplorationGlobalTour(plan.guide_path);
    } else if (current_route_.valid && !current_route_.path.empty()) {
        ros_ptr_->vizExplorationGlobalTour(current_route_.path);
    }
}

}  // namespace exploration
}  // namespace general_planner
