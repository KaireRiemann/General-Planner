#pragma once

#include <optional>

#include "data_structure/cmd_traj.h"
#include "general_core/map_manager.hpp"
#include "rog_map/rog_map.h"
#include "ros_interface/ros_interface.hpp"
#include "traj_opt/perching_surface_state.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace general_planner {

struct TaskContext {
    double now{0.0};

    rog_map::RobotState robot;
    MapManager::Ptr map_manager;
    ros_interface::RosInterface::Ptr ros_ptr;

    CmdTraj *committed_traj{nullptr};

    bool new_task{false};
    bool emergency{false};

    std::optional<super_utils::Vec3f> state_goal_p;
    std::optional<double> state_goal_yaw;
    bool state_goal_new{false};

    std::optional<traj_opt::DynamicTargetStates> target_prediction;
    std::optional<traj_opt::PerchingSurfaceState> surface;
};

} // namespace general_planner
