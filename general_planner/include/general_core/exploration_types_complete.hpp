#pragma once

#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <super_utils/type_utils.hpp>

namespace general_planner {

enum class ExplorationVoxelState {
    UNKNOWN,
    KNOWN_FREE,
    OCCUPIED,
    OUT_OF_BOUNDS
};

enum class FrontierStatus {
    ACTIVE,
    COVERED,
    UNREACHABLE,
    DORMANT,
    BLACKLISTED
};

struct CompleteFrontierCell {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    super_utils::Vec3i index{super_utils::Vec3i::Zero()};
    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
};

struct CompleteFrontierCluster {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id{-1};
    FrontierStatus status{FrontierStatus::ACTIVE};

    super_utils::vec_E<CompleteFrontierCell> cells;
    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};
    super_utils::Vec3f unknown_direction{super_utils::Vec3f::UnitX()};

    int size{0};
    double first_seen_time{0.0};
    double last_seen_time{0.0};
    double last_selected_time{-1.0};

    int fail_count{0};
    double estimated_gain{0.0};
    double best_travel_cost{std::numeric_limits<double>::infinity()};
    double coverage_priority{0.0};
};

struct CompleteExplorationViewpoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int frontier_id{-1};
    int region_id{-1};

    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    double yaw{0.0};

    double information_gain{0.0};
    double travel_cost{0.0};
    double yaw_cost{0.0};
    double curvature_cost{0.0};
    double coverage_cost{0.0};
    double revisit_penalty{0.0};
    double unknown_risk{0.0};
    double fail_penalty{0.0};
    double score{std::numeric_limits<double>::infinity()};

    bool reachable{false};
    super_utils::vec_E<super_utils::Vec3f> guide_path;
};

inline std::string frontierStatusName(const FrontierStatus status) {
    switch (status) {
        case FrontierStatus::ACTIVE:
            return "active";
        case FrontierStatus::COVERED:
            return "covered";
        case FrontierStatus::UNREACHABLE:
            return "unreachable";
        case FrontierStatus::DORMANT:
            return "dormant";
        case FrontierStatus::BLACKLISTED:
            return "blacklisted";
    }
    return "unknown";
}

inline std::string voxelStateName(const ExplorationVoxelState state) {
    switch (state) {
        case ExplorationVoxelState::UNKNOWN:
            return "unknown";
        case ExplorationVoxelState::KNOWN_FREE:
            return "known_free";
        case ExplorationVoxelState::OCCUPIED:
            return "occupied";
        case ExplorationVoxelState::OUT_OF_BOUNDS:
            return "out_of_bounds";
    }
    return "unknown";
}

}  // namespace general_planner
