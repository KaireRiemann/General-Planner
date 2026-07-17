#include <general_core/state2state/state2state_se3_backend.hpp>

#include <algorithm>

#include <general_core/config.hpp>
#include <map_manager/map_manager.hpp>
#include <general_core/state2state/se3_aggressive_manager.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>

using namespace general_utils;

namespace general_planner::state2state_task {

RET_CODE optimizeSE3AggressiveTask(StateToStateSE3BackendServices &services,
                                   const Vec3f &goal_p,
                                   const double goal_yaw,
                                   const bool from_rest) {
    if (!services.manager) {
        services.ros_ptr->warn(" -- [SE3Aggressive] Manager is not initialized.");
        return FAILED;
    }

    StatePVAJ head_state = services.runtime.makeTaskHeadState(from_rest);
    if (from_rest && services.map_manager) {
        Vec3f shifted_start = head_state.col(0);
        if (services.map_manager->getNearestCellNot(GridType::OCCUPIED,
                                                    head_state.col(0),
                                                    shifted_start,
                                                    3.0)) {
            head_state.col(0) = shifted_start;
        }
    }

    StatePVAJ tail_state = StatePVAJ::Zero();
    tail_state.col(0) = goal_p;
    if (services.cfg.goal_vel_en && !from_rest) {
        tail_state.col(1).setZero();
    }

    const double finish_thresh = std::max(0.08, 2.0 * services.cfg.resolution);
    if ((goal_p - services.robot_state.p).norm() < finish_thresh) {
        return FINISH;
    }

    services.goal_context.setGoalInfo(goal_p, goal_yaw, false, true);

    geometry_utils::Trajectory pos_traj;
    std::string reason;
    if (!services.manager->plan(head_state, tail_state, pos_traj, &reason, goal_yaw, 0.0)) {
        services.ros_ptr->warn(" -- [SE3Aggressive] Plan failed: {}.", reason);
        return FAILED;
    }

    if (!services.runtime.commitSE3AggressiveTrajectory(pos_traj, "se3_aggressive")) {
        services.ros_ptr->warn(" -- [SE3Aggressive] Commit failed.");
        return FAILED;
    }
    return SUCCESS;
}

} // namespace general_planner::state2state_task
