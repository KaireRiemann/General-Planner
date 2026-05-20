#pragma once

#include <memory>

#include <ros_interface/ros_interface.hpp>

#include "exploration/global_guidance_planner.hpp"

namespace general_planner {
namespace exploration {

class ExplorationManager {
public:
    using Ptr = std::shared_ptr<ExplorationManager>;

    struct Config {
        bool enable{false};
        bool use_epic_frontend{true};
        bool print_log{true};

        ObservationMap::Config observation_map;
        FrontierDatabase::Config frontier_database;
        ViewpointManager::Config viewpoint_manager;
        TopoGraph::Config topo_graph;
        GlobalGuidancePlanner::Config global_guidance;

        double route_waypoint_lookahead{3.0};
        double route_waypoint_min_distance{1.0};
        double route_waypoint_max_distance{5.0};
        double local_goal_safe_distance{0.35};
        double route_replan_distance_threshold{1.0};
        bool use_local_map_goal_safety{false};

        int repeated_goal_threshold{3};
        double repeated_goal_distance{0.5};
        double min_robot_displacement{0.5};
        double min_explored_volume_gain{1.0};
    };

    ExplorationManager(Config cfg,
                       MapManager::Ptr map_manager,
                       path_search::Astar::Ptr astar,
                       ros_interface::RosInterface::Ptr ros_ptr);

    void updateObservation(const rog_map::PointCloud &cloud,
                           const super_utils::Pose &pose,
                           CloudFrame frame,
                           const super_utils::Vec3f &sensor_position,
                           double stamp);

    int updateFrontiers(double stamp);
    void updateTopoGraph(const rog_map::RobotState &robot,
                         double current_yaw,
                         double stamp = -1.0);
    bool planGlobalTour(const rog_map::RobotState &robot,
                        double current_yaw,
                        GlobalGuidanceResult &result);

    bool planNextGoal(const rog_map::RobotState &robot,
                      double current_yaw,
                      ExplorationGoal &goal);

    bool planOnce(const rog_map::RobotState &robot,
                  double current_yaw,
                  ExplorationPlan &plan);

    void onGoalReached(const ExplorationGoal &goal, double stamp);
    void onGoalFailed(const ExplorationGoal &goal, const std::string &reason, double stamp);
    void onLowGain(const ExplorationGoal &goal, double actual_gain, double stamp);

    bool isExplorationFinished() const { return exploration_finished_; }
    void reset();

    int activeFrontierCount() const { return frontier_db_ ? frontier_db_->activeCount() : 0; }
    int topoNodeCount() const { return topo_graph_ ? topo_graph_->nodeCount() : 0; }
    int topoEdgeCount() const { return topo_graph_ ? topo_graph_->edgeCount() : 0; }
    int observationFrontierCount() const { return observation_map_ ? observation_map_->frontierVoxelCount() : 0; }

private:
    void validateRuntimeConfig() const;

    static super_utils::vec_E<super_utils::Vec3f> buildGuidePathToGoal(
            const GlobalRoute &route,
            const super_utils::Vec3f &robot_pos,
            const super_utils::Vec3f &goal_pos);

    bool selectNextLocalSubgoal(const GlobalRoute &route,
                                const rog_map::RobotState &robot,
                                ExplorationGoal &goal) const;

    bool routeWaypointSafe(const super_utils::Vec3f &position) const;

    bool detectStuck(const ExplorationGoal &goal,
                     const super_utils::Vec3f &robot_pos,
                     double stamp);

    void logPlanSummary(const ExplorationGoal &goal,
                        const std::string &reason,
                        bool fallback) const;

    void visualizePlan(const ExplorationPlan &plan,
                       const std::vector<FrontierRecord> &active_frontiers) const;

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
    ros_interface::RosInterface::Ptr ros_ptr_;

    std::shared_ptr<ObservationMap> observation_map_;
    EpicMapAdapter::Ptr map_adapter_;
    std::unique_ptr<FrontierDatabase> frontier_db_;
    std::unique_ptr<ViewpointManager> viewpoint_manager_;
    std::unique_ptr<TopoGraph> topo_graph_;
    std::unique_ptr<GlobalGuidancePlanner> global_guidance_planner_;

    int current_target_frontier_id_{-1};
    GlobalRoute current_route_;
    int repeated_goal_count_{0};
    ExplorationGoal last_selected_goal_;
    double last_significant_gain_time_{-1.0};
    double last_explored_volume_{0.0};
    double travel_distance_{0.0};
    super_utils::Vec3f last_observation_position_{super_utils::Vec3f::Zero()};
    bool has_last_observation_position_{false};
    bool exploration_finished_{false};
};

using EpicExplorationFrontend = ExplorationManager;

}  // namespace exploration
}  // namespace general_planner
