/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

#include <algorithm>

using namespace general_utils;

namespace general_planner {

    bool GeneralPlanner::trackingTrajectorySafeForHorizon(const Trajectory &traj,
                                                          const double start_t,
                                                          const double horizon,
                                                          const double dt) const {
        return trackingTrajectorySafeForHorizonDetailed(traj, start_t, horizon, dt);
    }

    GeneralPlanner::TrackingTrajectoryActivity
    GeneralPlanner::evaluateTrackingTrajectoryActivity(
            const Trajectory &traj,
            const double local_start_t,
            const traj_opt::DynamicTargetStates &target_prediction,
            const double horizon,
            const double dt) const {
        auto activity = tracking_runtime_manager_->evaluateActivity(
                traj, local_start_t, target_prediction, horizon, dt);
        return activity;
    }

    bool GeneralPlanner::currentTrackingTrajectorySafeAndActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            TrackingTrajectoryActivity *activity) const {
        if (cmd_traj_info_.empty()) {
            if (activity) {
                activity->reason = "empty committed trajectory";
            }
            return false;
        }

        Trajectory old_pos_traj;
        double old_start_wt = 0.0;
        double total_dur = 0.0;
        auto &mutable_cmd_traj = const_cast<CmdTraj &>(cmd_traj_info_);
        mutable_cmd_traj.lock();
        old_pos_traj = cmd_traj_info_.posTraj();
        old_start_wt = cmd_traj_info_.getStartWallTime();
        total_dur = cmd_traj_info_.getTotalDuration();
        mutable_cmd_traj.unlock();

        const double now = ros_ptr_->getSimTime();
        const double cur_t = std::clamp(now - old_start_wt, 0.0, total_dur);
        TrackingTrajectoryActivity local_activity =
                evaluateTrackingTrajectoryActivity(old_pos_traj,
                                                   cur_t,
                                                   target_prediction,
                                                   cfg_.tracking_keep_old_horizon,
                                                   cfg_.tracking_keep_old_safety_dt);
        if (activity) {
            *activity = local_activity;
        }

        return local_activity.valid &&
               local_activity.safe &&
               local_activity.active;
    }

} // namespace general_planner
