#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <super_utils/type_utils.hpp>

namespace general_planner {
namespace exploration {

enum class ObservationCellState : uint8_t {
    DENSE = 0,
    SPARSE = 1,
    UNKNOWN = 2,
    FRONTIER_DIS = 3,
    FRONTIER_DIR = 4
};

using SurfaceVoxelState = ObservationCellState;

enum class FrontierState {
    ACTIVE,
    SELECTED,
    COVERED,
    DORMANT,
    UNREACHABLE,
    BLACKLISTED
};

enum class ExplorationGoalType {
    FRONTIER_VIEWPOINT,
    GLOBAL_ROUTE_WAYPOINT,
    UNKNOWN
};

enum class TopoNodeType {
    ODOM,
    HISTORY_ODOM,
    REGION,
    FRONTIER_VIEWPOINT,
    ROUTE_WAYPOINT
};

enum class GuidanceEdgeSource {
    UNKNOWN,
    LOCAL_ASTAR,
    BUBBLE_ASTAR,
    OBSERVED_LINE
};

struct SurfaceFrontierCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int transient_id{-1};
    super_utils::vec_E<super_utils::Vec3f> cells;
    super_utils::vec_E<super_utils::Vec3f> normals;
    std::vector<ObservationCellState> cell_states;

    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f normal{super_utils::Vec3f::UnitX()};
    super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};
    ObservationCellState dominant_state{ObservationCellState::FRONTIER_DIS};

    int raw_size{0};
    double stamp{0.0};
    super_utils::Vec3f generated_position{super_utils::Vec3f::Zero()};
    double generated_travel_distance{0.0};
};

struct ExplorationViewpoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int frontier_id{-1};
    int viewpoint_id{-1};

    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    super_utils::Vec3f frontier_center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f frontier_normal{super_utils::Vec3f::UnitX()};
    double yaw{0.0};

    double gain_raw{0.0};
    double gain_norm{0.0};
    double distance_to_surface{std::numeric_limits<double>::infinity()};
    double topo_cost{std::numeric_limits<double>::infinity()};
    int covered_frontier_count{0};

    bool reachable{false};
    bool visited{false};
    bool local_safe{false};
    bool global_safe{false};

    double last_checked_time{-1.0};
};

struct FrontierRecord {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int stable_id{-1};
    int region_id{-1};

    FrontierState state{FrontierState::ACTIVE};

    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f normal{super_utils::Vec3f::UnitX()};
    super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};

    super_utils::vec_E<super_utils::Vec3f> cells;
    super_utils::vec_E<super_utils::Vec3f> normals;
    std::vector<ObservationCellState> cell_states;

    int cell_count{0};
    ObservationCellState dominant_state{ObservationCellState::FRONTIER_DIS};
    double last_gain{0.0};
    double best_gain{0.0};

    std::vector<ExplorationViewpoint> viewpoints;
    ExplorationViewpoint best_viewpoint;

    int selected_count{0};
    int failed_count{0};
    double last_selected_time{-1.0};
    double last_observed_time{-1.0};
    double first_observed_time{-1.0};

    super_utils::Vec3f generated_position{super_utils::Vec3f::Zero()};
    double generated_travel_distance{0.0};

    bool has_reachable_viewpoint{false};
};

struct ExplorationTopoNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id{-1};
    TopoNodeType type{TopoNodeType::REGION};

    int frontier_id{-1};
    int region_id{-1};

    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};

    bool active{true};
};

struct ExplorationTopoEdge {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int from{-1};
    int to{-1};
    double cost{0.0};
    bool reachable{true};
    GuidanceEdgeSource source{GuidanceEdgeSource::UNKNOWN};
    super_utils::vec_E<super_utils::Vec3f> path;
};

struct GlobalRoute {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    int target_frontier_id{-1};
    std::vector<int> node_ids;
    super_utils::vec_E<super_utils::Vec3f> path;
    double cost{0.0};

    int local_astar_edge_count{0};
    int bubble_astar_edge_count{0};
    int observed_line_edge_count{0};
    int unknown_edge_count{0};
};

struct ExplorationGoal {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    ExplorationGoalType type{ExplorationGoalType::UNKNOWN};

    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};

    int frontier_id{-1};
    int route_node_id{-1};

    double gain{0.0};
    double travel_cost{0.0};
    std::string reason;
    GlobalRoute route;
};

struct ExplorationPlan {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid{false};
    bool no_frontier{false};

    super_utils::vec_E<super_utils::Vec3f> guide_path;
    super_utils::Vec3f next_goal{super_utils::Vec3f::Zero()};
    double next_yaw{0.0};

    super_utils::Vec3f final_goal{super_utils::Vec3f::Zero()};
    double final_yaw{0.0};
    bool local_goal_is_final{false};
    bool goal_switched{false};
    int target_frontier_id{-1};
    int target_viewpoint_id{-1};
    double route_progress_length{0.0};
    int local_fail_count{0};
    std::size_t raw_route_path_size{0U};
    std::size_t refined_guide_path_size{0U};

    ExplorationGoal goal;
    std::string reason;
};

inline const char *goalTypeName(const ExplorationGoalType type) {
    switch (type) {
        case ExplorationGoalType::FRONTIER_VIEWPOINT:
            return "FRONTIER_VIEWPOINT";
        case ExplorationGoalType::GLOBAL_ROUTE_WAYPOINT:
            return "GLOBAL_ROUTE_WAYPOINT";
        case ExplorationGoalType::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

inline const char *guidanceEdgeSourceName(const GuidanceEdgeSource source) {
    switch (source) {
        case GuidanceEdgeSource::LOCAL_ASTAR:
            return "local_astar";
        case GuidanceEdgeSource::BUBBLE_ASTAR:
            return "bubble_astar";
        case GuidanceEdgeSource::OBSERVED_LINE:
            return "observed_line";
        case GuidanceEdgeSource::UNKNOWN:
        default:
            return "unknown";
    }
}

inline std::string guidanceRouteSourceSummary(const GlobalRoute &route) {
    return std::string("local_astar=") + std::to_string(route.local_astar_edge_count) +
           ", bubble_astar=" + std::to_string(route.bubble_astar_edge_count) +
           ", observed_line=" + std::to_string(route.observed_line_edge_count) +
           ", unknown=" + std::to_string(route.unknown_edge_count);
}

inline bool frontierStateSelectable(const FrontierState state) {
    return state == FrontierState::ACTIVE || state == FrontierState::SELECTED;
}

inline bool isFrontierCellState(const ObservationCellState state) {
    return state == ObservationCellState::FRONTIER_DIS ||
           state == ObservationCellState::FRONTIER_DIR;
}

inline bool isObservedCellState(const ObservationCellState state) {
    return state == ObservationCellState::DENSE ||
           state == ObservationCellState::SPARSE ||
           isFrontierCellState(state);
}

inline const char *observationCellStateName(const ObservationCellState state) {
    switch (state) {
        case ObservationCellState::DENSE:
            return "DENSE";
        case ObservationCellState::SPARSE:
            return "SPARSE";
        case ObservationCellState::UNKNOWN:
            return "UNKNOWN";
        case ObservationCellState::FRONTIER_DIS:
            return "FRONTIER_DIS";
        case ObservationCellState::FRONTIER_DIR:
            return "FRONTIER_DIR";
        default:
            return "UNKNOWN";
    }
}

}  // namespace exploration
}  // namespace general_planner
