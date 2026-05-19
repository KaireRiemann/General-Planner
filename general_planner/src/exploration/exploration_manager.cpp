#include "exploration/exploration_manager.hpp"

#include <algorithm>
#include <cmath>

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
    observation_map_ = std::make_shared<ObservationMap>(cfg_.observation_map);
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
    observation_map_->update(cloud, pose, frame, sensor_position, travel_distance_, stamp);
}

bool ExplorationManager::planNextGoal(const rog_map::RobotState &robot,
                                      const double current_yaw,
                                      ExplorationGoal &goal) {
    goal = ExplorationGoal{};
    const double stamp = ros_ptr_ ? ros_ptr_->getSimTime() : 0.0;
    if (!cfg_.enable) {
        goal.reason = "exploration disabled";
        logPlanSummary(goal, goal.reason, false);
        return false;
    }
    if (observation_map_ == nullptr || frontier_db_ == nullptr ||
        viewpoint_manager_ == nullptr || topo_graph_ == nullptr ||
        global_guidance_planner_ == nullptr) {
        goal.reason = "exploration modules are not ready";
        logPlanSummary(goal, goal.reason, false);
        return false;
    }
    if (!robot.rcv) {
        goal.reason = "no odom";
        logPlanSummary(goal, goal.reason, false);
        return false;
    }

    std::vector<SurfaceFrontierCluster> clusters;
    observation_map_->getFrontierClusters(clusters);
    frontier_db_->update(clusters, stamp);

    std::vector<FrontierRecord> active_frontiers = frontier_db_->getActiveFrontiers();
    for (const auto &frontier : active_frontiers) {
        std::vector<ExplorationViewpoint> viewpoints;
        ExplorationViewpoint best;
        viewpoint_manager_->generateBestViewpoints(frontier,
                                                   *observation_map_,
                                                   map_manager_,
                                                   robot.p,
                                                   current_yaw,
                                                   stamp,
                                                   viewpoints,
                                                   best);
        frontier_db_->setViewpoints(frontier.stable_id, viewpoints, best, stamp);
    }

    active_frontiers = frontier_db_->getActiveFrontiers();
    topo_graph_->updateOdomNode(robot.p, current_yaw);
    topo_graph_->updateHistoryOdomNodes(robot.p, current_yaw);
    topo_graph_->insertOrUpdateFrontierNodes(active_frontiers);

    if (active_frontiers.empty()) {
        exploration_finished_ = observation_map_->observedVoxelCount() > 0;
        goal.reason = exploration_finished_ ? "no active frontier" : "observation map empty";
        logPlanSummary(goal, goal.reason, false);
        return false;
    }

    GlobalGuidanceResult guidance;
    if (!global_guidance_planner_->buildGuidance(robot.p,
                                                 travel_distance_,
                                                 active_frontiers,
                                                 *topo_graph_,
                                                 current_target_frontier_id_,
                                                 current_route_,
                                                 guidance)) {
        current_target_frontier_id_ = -1;
        current_route_ = GlobalRoute{};
        goal.reason = guidance.reason;
        logPlanSummary(goal, goal.reason, false);
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
        logPlanSummary(goal, goal.reason, false);
        return false;
    }

    if (detectStuck(goal, robot.p, stamp)) {
        frontier_db_->markBlacklisted(current_target_frontier_id_, stamp);
        current_target_frontier_id_ = -1;
        current_route_ = GlobalRoute{};
        goal.valid = false;
        goal.reason = "stuck detected, blacklist current frontier";
        logPlanSummary(goal, goal.reason, false);
        return false;
    }

    exploration_finished_ = false;
    logPlanSummary(goal, "goal selected", false);
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

}  // namespace exploration
}  // namespace general_planner
