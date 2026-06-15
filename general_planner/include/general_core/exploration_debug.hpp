#pragma once

#include <cstdint>
#include <string>

#include <map_manager/frontier_cluster_manager.hpp>
#include <super_utils/type_utils.hpp>

namespace general_planner {

struct ExplorationViewpointDebug {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int debug_id{-1};
    int frontier_cluster_id{-1};
    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};

    bool accepted{false};
    bool reachable{false};
    bool selected{false};

    std::string viewpoint_case;
    std::string status;

    double score{0.0};
    double information_gain{0.0};
    double travel_cost{0.0};
    double yaw_cost{0.0};
    double curvature_cost{0.0};
    double unknown_risk{0.0};
    double open_space_score{0.0};
    double high_speed_score{0.0};
    double lifecycle_score{0.0};
};

struct ExplorationSelectedGoalDebug {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};
    int frontier_cluster_id{-1};
    int viewpoint_debug_id{-1};
    std::string viewpoint_case;
    double score{0.0};
    double information_gain{0.0};
    double travel_cost{0.0};
    super_utils::vec_E<super_utils::Vec3f> guide_path;
};

struct ExplorationDebugInfo {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t sequence{0};
    bool has_robot_state{false};
    bool planning_success{false};
    bool exploration_finished{false};
    super_utils::Vec3f robot_position{super_utils::Vec3f::Zero()};
    double robot_yaw{0.0};

    std::string reason;
    FrontierSearchStats search_stats;
    rog_map::vec_E<FrontierVoxel> frontier_voxels;
    rog_map::vec_E<FrontierCluster> active_clusters;
    rog_map::vec_E<FrontierCluster> tracked_clusters;
    super_utils::vec_E<ExplorationViewpointDebug> viewpoints;
    ExplorationSelectedGoalDebug selected_goal;
};

}  // namespace general_planner
