#pragma once

#include <limits>
#include <memory>
#include <string>

#include <data_structure/base/trajectory.h>
#include <general_core/dynamic_obstacle_layer.hpp>
#include <map_manager/map_manager.hpp>
#include <rog_map/rog_map.h>

namespace general_planner {

struct CommittedTrajectorySafetyReport {
    bool valid{false};
    bool safe{true};
    double check_start_t{0.0};
    double check_horizon{0.0};
    double collision_t{0.0};
    double time_to_collision{std::numeric_limits<double>::infinity()};
    general_utils::Vec3f collision_pos{general_utils::Vec3f::Zero()};
    int grid_type{static_cast<int>(rog_map::GridType::KNOWN_FREE)};
    int hit_count{0};
    std::string reason;
};

struct RuntimeTrajectorySafetyServices {
    std::shared_ptr<MapManager> map_manager;
    const DynamicObstacleLayer *dynamic_obstacle_layer{nullptr};
};

bool checkPositionTrajectorySafety(const RuntimeTrajectorySafetyServices &services,
                                   const geometry_utils::Trajectory &traj,
                                   double now_wt,
                                   double horizon,
                                   double dt,
                                   int consecutive_hits,
                                   bool unknown_as_occupied,
                                   CommittedTrajectorySafetyReport *report = nullptr);

} // namespace general_planner
