#pragma once

#include <iostream>
#include <string>

#include <super_utils/type_utils.hpp>

namespace general_planner {

enum class ExplorationGoalType {
    LOCAL_FRONTIER_VIEWPOINT,
    GLOBAL_ROUTE_WAYPOINT,
    COVERAGE_REGION_VIEWPOINT,
    RETURN_TO_UNFINISHED_FRONTIER,
    UNKNOWN
};

struct ExplorationGoal {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};

    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};

    ExplorationGoalType type{ExplorationGoalType::UNKNOWN};

    int frontier_id{-1};
    int cluster_id{-1};
    int region_id{-1};
    int coverage_node_id{-1};

    double score{0.0};
    double information_gain{0.0};
    double information_gain_norm{0.0};
    double travel_cost{0.0};
    double yaw_cost{0.0};
    double curvature_cost{0.0};
    double switching_cost{0.0};
    double reachability_cost{0.0};
    double distance_to_robot{0.0};

    bool astar_checked{false};
    bool astar_reachable{false};

    std::string reason;

    super_utils::vec_E<super_utils::Vec3f> guide_path;
};

struct FrontierCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id{-1};
    int region_id{-1};

    super_utils::vec_E<super_utils::Vec3f> cells;
    super_utils::vec_E<super_utils::Vec3f> filtered_cells;

    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};

    int raw_size{0};
    int visible_candidate_count{0};

    bool valid{false};
};

struct ViewpointCandidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int cluster_id{-1};
    int region_id{-1};

    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};

    double gain_raw{0.0};
    double gain_norm{0.0};

    double travel_cost{0.0};
    double yaw_cost{0.0};
    double curvature_cost{0.0};
    double switching_cost{0.0};
    double cheap_score{0.0};
    double final_score{0.0};

    bool valid{false};
    bool reachable{false};
    bool astar_checked{false};

    super_utils::vec_E<super_utils::Vec3f> astar_path;
};

struct ExplorationConfig {
    bool enable{false};
    bool print_log{true};

    double frontier_search_radius{12.0};
    bool use_global_frontiers{false};
    double frontier_cluster_radius{0.5};
    int min_frontier_cluster_size{5};
    int max_frontiers_per_cluster{500};
    double max_cluster_extent{4.0};
    int frontier_downsample_step{2};

    double viewpoint_min_distance{1.5};
    double viewpoint_max_distance{3.5};
    int viewpoint_radius_num{3};
    int viewpoint_yaw_num{16};
    double viewpoint_height_offset{0.0};
    double viewpoint_safe_distance{0.45};
    int max_candidate_num{128};

    bool use_fov_gain{true};
    double sensor_range{6.0};
    double sensor_horizontal_fov_deg{90.0};
    double sensor_vertical_fov_deg{60.0};
    int visibility_sample_max_points{300};
    double line_of_sight_sample_step{0.15};
    bool require_line_free_to_frontier{true};

    double info_gain_cap{300.0};
    double min_information_gain{20.0};
    double travel_cost_norm{10.0};

    double weight_gain{2.0};
    double weight_travel{1.0};
    double weight_yaw{0.3};
    double weight_curvature{0.8};
    double weight_switch{0.5};
    double weight_reachability{1.0};

    bool use_astar_cost{true};
    int max_astar_candidate_num{8};
    double astar_timeout{0.08};
    double astar_search_horizon{20.0};

    double min_switch_distance{1.0};
    double switch_score_margin{0.5};
    double switch_gain_margin{0.15};
    double keep_goal_min_remaining_time{1.0};
    double goal_reached_distance{0.5};
    double min_goal_distance{1.2};

    double failed_candidate_blacklist_time{3.0};
    double failed_candidate_blacklist_radius{0.8};

    bool coverage_guide_enable{true};
    int coverage_max_active_regions{30};
    double coverage_weight_distance{1.0};
    double coverage_weight_height{1.5};
    double coverage_weight_gain{2.0};
    double coverage_weight_revisit{0.5};
    double coverage_weight_switch_region{0.5};
    bool coverage_use_astar_cost{false};
    int coverage_max_route_length{10};
};

}  // namespace general_planner
