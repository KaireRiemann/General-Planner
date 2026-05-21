#pragma once

#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "exploration/exploration_types.hpp"
#include "exploration/local_guide_builder.hpp"

class FrontierManager;
class GraphVisualizer;
class ParallelBubbleAstar;
class TopoNode;
class TopoGraph;

namespace fast_planner {
class LIOInterface;
}

namespace general_planner {
namespace exploration {

class EpicExplorationManager {
public:
    using Ptr = std::shared_ptr<EpicExplorationManager>;

    struct Config {
        struct ObservationConfig {
            bool enable{false};
            double resolution{0.25};
            double min_observation_distance{0.2};
            double well_observed_distance{4.0};
            double max_observation_distance{12.0};
            double good_observation_force_trust_length{1.5};
            double good_observation_trust_length{4.0};
            double good_observation_direction_score{0.5};
            int cloud_downsample_step{1};
            int max_points_per_update{12000};
            double frontier_cluster_radius{0.65};
            double frontier_normal_similarity{0.35};
            int min_frontier_cluster_size{8};
            double bbox_min_x{-50.0};
            double bbox_min_y{-50.0};
            double bbox_min_z{-2.0};
            double bbox_max_x{50.0};
            double bbox_max_y{50.0};
            double bbox_max_z{10.0};
        };

        struct FrontierDatabaseConfig {
            double association_distance{1.2};
            double bbox_overlap_min_ratio{0.08};
            int max_failed_count{3};
            int max_selected_count_without_gain{4};
            double blacklist_time{8.0};
            double covered_gain_threshold{4.0};
            double missing_frontier_timeout{2.0};
            double dormant_time{4.0};
        };

        struct ViewpointConfig {
            double min_distance{1.4};
            double max_distance{4.0};
            int radius_samples{3};
            int yaw_samples{16};
            int height_samples{3};
            double height_step{0.6};
            double safe_distance{0.45};
            double sensor_range{7.0};
            double horizontal_fov_deg{90.0};
            double vertical_fov_deg{60.0};
            double normal_dot_min{0.25};
            int max_cells_per_gain_eval{260};
            double line_of_sight_step{0.20};
            double min_gain{3.0};
            bool use_local_map_safety{false};
            bool cluster_by_visibility_sphere{true};
            bool use_topo_reachability_filter{true};
            int max_viewpoint_clusters{8};
            double viewpoint_cluster_connectivity_scale{1.0};
            double topo_reachability_timeout{0.03};
            int epic_yaw_bins{8};
        };

        struct TopoConfig {
            double history_node_min_distance{1.0};
            double connect_radius{8.0};
            double local_edge_astar_timeout{0.05};
            double global_edge_max_length{14.0};
            int max_history_nodes{500};
            bool use_local_astar_for_edges{true};
            bool use_global_line_free_for_edges{true};
            double global_line_safe_distance{0.45};
            double global_line_step{0.25};
            bool use_parallel_bubble_astar_for_edges{false};
            double bubble_astar_resolution{0.5};
            double bubble_astar_safe_distance{0.45};
            int bubble_astar_max_nodes{8000};
            bool use_epic_region_bubble_graph{true};
            double region_size_xy{4.0};
            double region_size_z{2.0};
            double min_subregion_size_xy{0.5};
            double min_subregion_size_z{0.5};
            double bubble_min_radius{0.5};
            double frontier_bubble_min_radius{0.5};
            double cube_discrete_size{0.3};
            int max_update_region_num{20};
            int neighbor_mode{26};
            double edge_search_padding_scale{1.0};
            double edge_safe_distance{0.45};
            int max_edges_per_node{10};
            int max_region_edges_per_node{8};
            int max_frontier_edges_per_node{4};
            int max_history_edges_per_node{3};
            int max_candidate_neighbors{24};
            super_utils::Vec3f bbox_min{super_utils::Vec3f(-50.0, -50.0, -2.0)};
            super_utils::Vec3f bbox_max{super_utils::Vec3f(50.0, 50.0, 10.0)};
        };

        struct GlobalGuidanceConfig {
            bool enable{true};
            int max_frontiers_in_tour{16};
            double weight_path_cost{1.0};
            double weight_gain{2.0};
            double weight_revisit{0.5};
            bool use_two_opt{true};
            bool keep_current_target{true};
            bool use_lkh{true};
            std::string tsp_dir{"/tmp/general_planner_tsp"};
            std::string tsp_problem_name{"general_planner_global"};
            std::string lkh_executable;
            int lkh_cost_scale{100};
        };

        bool enable{false};
        bool use_epic_frontend{true};
        bool print_log{true};
        ObservationConfig observation_map;
        FrontierDatabaseConfig frontier_database;
        ViewpointConfig viewpoint_manager;
        TopoConfig topo_graph;
        GlobalGuidanceConfig global_guidance;
        double route_waypoint_lookahead{3.0};
        double route_waypoint_min_distance{1.0};
        double route_waypoint_max_distance{5.0};
        double local_goal_safe_distance{0.35};
        double route_replan_distance_threshold{1.0};
        bool use_local_map_goal_safety{false};
        double final_goal_radius{0.7};
        double route_progress_min{0.5};
        int max_local_segment_fail_count{3};
        bool keep_active_target{true};
        double switch_score_ratio{1.25};
        LocalGuideBuilder::Config local_guide;
        int repeated_goal_threshold{3};
        double repeated_goal_distance{0.5};
        double min_robot_displacement{0.5};
        double min_explored_volume_gain{1.0};
    };

    EpicExplorationManager(Config cfg,
                           MapManager::Ptr map_manager,
                           path_search::Astar::Ptr astar,
                           ros_interface::RosInterface::Ptr ros_ptr);

    void updateObservation(const rog_map::PointCloud &cloud,
                           const super_utils::Pose &pose,
                           CloudFrame frame,
                           const super_utils::Vec3f &sensor_position,
                           double stamp);
    void onCloudOdom(const rog_map::PointCloud &cloud,
                     const super_utils::Pose &pose,
                     CloudFrame frame,
                     const rog_map::RobotState &robot,
                     double stamp);

    int updateFrontiers(double stamp);
    void updateTopoGraph(const rog_map::RobotState &robot,
                         double current_yaw,
                         double stamp = -1.0);
    bool planNextGoal(const rog_map::RobotState &robot,
                      double current_yaw,
                      ExplorationGoal &goal);
    bool planOnce(const rog_map::RobotState &robot,
                  double current_yaw,
                  ExplorationPlan &plan);

    void onGoalReached(const ExplorationGoal &goal, double stamp);
    void onGoalFailed(const ExplorationGoal &goal,
                      const std::string &reason,
                      double stamp);
    void onLocalSegmentCommitted(const ExplorationPlan &plan, double stamp);
    void onLocalSegmentFailed(const ExplorationPlan &plan,
                              const std::string &reason,
                              double stamp);
    void onLowGain(const ExplorationGoal &goal, double actual_gain, double stamp);

    bool isExplorationFinished() const;
    bool hasObservation() const;
    double lastObservationStamp() const;
    void reset();

private:
    void seedEpicParams();
    void initializeNativeModules();

    bool updateNativeGlobalPlan(const rog_map::RobotState &robot,
                                double current_yaw,
                                ExplorationPlan &plan);
    bool buildNativeGuide(const rog_map::RobotState &robot,
                          double current_yaw,
                          ExplorationPlan &plan);
    bool routeBetweenNativeNodes(const std::shared_ptr<TopoNode> &start,
                                 const std::shared_ptr<TopoNode> &goal,
                                 double timeout,
                                 super_utils::vec_E<super_utils::Vec3f> &path,
                                 double &cost) const;
    bool solveNativeAtspTour(const Eigen::MatrixXd &cost_matrix,
                             std::vector<int> &order) const;
    bool solveNativeAtspTourLkh(const Eigen::MatrixXd &cost_matrix,
                                std::vector<int> &order) const;
    static void normalizeTourOrder(std::vector<int> &order, int dimension);
    int frontierIdForViewpoint(const super_utils::Vec3f &viewpoint) const;
    void setNativeFrontierDormant(int frontier_id, bool unreachable);
    void updateNativeFrontiersFromLatestCloud(double stamp);
    void visualizeNativeState() const;
    void resetRuntimeState();

    static super_utils::Vec3f toVec3d(const Eigen::Vector3f &p);
    static double pathLength(const super_utils::vec_E<super_utils::Vec3f> &path);
    static void appendUnique(super_utils::vec_E<super_utils::Vec3f> &path,
                             const super_utils::Vec3f &point);

private:
    Config cfg_;
    MapManager::Ptr map_manager_;
    path_search::Astar::Ptr astar_;
    ros_interface::RosInterface::Ptr ros_ptr_;
    ros::NodeHandle nh_;

    std::shared_ptr<fast_planner::LIOInterface> lio_interface_;
    std::shared_ptr<ParallelBubbleAstar> parallel_path_finder_;
    std::shared_ptr<FrontierManager> frontier_manager_;
    std::shared_ptr<TopoGraph> graph_;
    std::shared_ptr<GraphVisualizer> graph_visualizer_;
    std::unique_ptr<LocalGuideBuilder> local_guide_builder_;

    mutable std::mutex mutex_;
    bool initialized_{false};
    bool has_observation_{false};
    bool exploration_finished_{false};
    int last_native_result_{-1};
    int active_frontier_id_{-1};
    int active_viewpoint_id_{-1};
    int plan_seq_{0};
    std::unordered_map<int, int> local_fail_count_by_frontier_;
    super_utils::Vec3f last_sensor_position_{super_utils::Vec3f::Zero()};
    double last_observation_stamp_{-1.0};
    bool has_last_sensor_position_{false};
    int observation_update_count_{0};
    std::size_t last_observation_cloud_size_{0};
    double last_empty_cloud_log_stamp_{-1.0};
    double travel_distance_{0.0};
    ExplorationPlan last_plan_;
};

}  // namespace exploration
}  // namespace general_planner
