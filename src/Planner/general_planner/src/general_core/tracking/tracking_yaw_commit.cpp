/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_internal_utils.hpp>

#include <algorithm>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {

    bool GeneralPlanner::buildTrackingTargetYawTrajectory(
            const Trajectory &pos_traj,
            const traj_opt::DynamicTargetStates &target_prediction,
            Trajectory &yaw_traj) {
        if (pos_traj.empty() || target_prediction.empty()) {
            return false;
        }

        Vec4f init_yaw{robot_state_.yaw, 0.0, 0.0, 0.0};
        if (!cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory committed_yaw_traj = cmd_traj_info_.yawTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            const double eval_t = ros_ptr_->getSimTime() - start_wt;
            StatePVAJ yaw_state;
            if (!committed_yaw_traj.empty() && eval_t >= 0.0 && eval_t <= total_dur &&
                committed_yaw_traj.getState(eval_t, yaw_state)) {
                init_yaw = yaw_state.row(0);
            }
        }

        VecDf times;
        traj_manager_->yaw()->getYawTimeAllocation(pos_traj.getTotalDuration(), times);
        if (times.size() == 0 || !times.allFinite()) {
            return false;
        }

        VecDf way_pts;
        way_pts.resize(std::max<Eigen::Index>(0, times.size() - 1));
        double eval_t = 0.0;
        double last_yaw = init_yaw[0];
        for (Eigen::Index i = 0; i < way_pts.size(); ++i) {
            eval_t += times(i);
            const double yaw = yawFacingTarget(pos_traj, target_prediction, eval_t, last_yaw);
            way_pts(i) = yaw;
            last_yaw = yaw;
        }

        Vec4f goal_yaw{0.0, 0.0, 0.0, 0.0};
        goal_yaw[0] = yawFacingTarget(pos_traj, target_prediction, pos_traj.getTotalDuration(), last_yaw);
        if (way_pts.size() == 0) {
            geometry_utils::normalizeNextYaw(init_yaw[0], goal_yaw[0]);
        } else {
            geometry_utils::normalizeNextYaw(way_pts(way_pts.size() - 1), goal_yaw[0]);
        }

        const Vec2f init_state = init_yaw.head(2);
        const Vec2f goal_state = goal_yaw.head(2);
        yaw_traj = poly_interpo::minimumAccInterpolation<1>(init_state,
                                                            goal_state,
                                                            way_pts,
                                                            times);
        yaw_traj.start_WT = pos_traj.start_WT;
        return !yaw_traj.empty();
    }

    bool GeneralPlanner::commitTrackingToPerchingTrajectory(
            const Trajectory &tracking_pos,
            const Trajectory &tracking_yaw,
            const double current_tracking_local_t,
            const double handover_delay,
            const Trajectory &perching_pos,
            const Trajectory &perching_yaw,
            const std::string &traj_ns) {
        if (perching_pos.empty() || perching_yaw.empty()) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason=empty_perching_suffix");
            return false;
        }

        const double commit_wt = ros_ptr_->getSimTime();
        Trajectory committed_pos = perching_pos;
        Trajectory committed_yaw = perching_yaw;
        bool stitched = false;
        const bool use_prefix =
                cfg_.tracking_to_perching_stitch_prefix &&
                handover_delay > 1.0e-4 &&
                !tracking_pos.empty() &&
                !tracking_yaw.empty();
        if (use_prefix) {
            const double prefix_start = current_tracking_local_t;
            const double prefix_end =
                    std::min(current_tracking_local_t + handover_delay,
                             tracking_pos.getTotalDuration());
            const double prefix_duration = prefix_end - prefix_start;
            Trajectory prefix_pos;
            Trajectory prefix_yaw;
            bool used_sampled_yaw_prefix = false;
            const bool prefix_pos_ok =
                    prefix_end > prefix_start + 1.0e-4 &&
                    tracking_pos.getPartialTrajectoryByTime(prefix_start, prefix_end, prefix_pos);
            const bool prefix_yaw_ok =
                    prefix_pos_ok &&
                    extractYawPrefixForStitching(tracking_yaw,
                                                 prefix_start,
                                                 prefix_duration,
                                                 prefix_yaw,
                                                 used_sampled_yaw_prefix);
            if (prefix_pos_ok && prefix_yaw_ok) {
                committed_pos = prefix_pos + perching_pos;
                committed_yaw = prefix_yaw + perching_yaw;
                stitched = true;

                const StatePVAJ prefix_tail = prefix_pos.getState(prefix_pos.getTotalDuration());
                const StatePVAJ suffix_head = perching_pos.getState(0.0);
                ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_STITCHED_COMMIT prefix_dt={:.3f}, suffix_dt={:.3f}, pos_jump={:.4f}, vel_jump={:.4f}, sampled_yaw_prefix={}",
                               prefix_pos.getTotalDuration(),
                               perching_pos.getTotalDuration(),
                               (prefix_tail.col(0) - suffix_head.col(0)).norm(),
                               (prefix_tail.col(1) - suffix_head.col(1)).norm(),
                               used_sampled_yaw_prefix);
            } else {
                ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason=prefix_extract_failed, pos_ok={}, yaw_ok={}, prefix_start={:.3f}, prefix_end={:.3f}, tracking_pos_dur={:.3f}, tracking_yaw_dur={:.3f}",
                               prefix_pos_ok,
                               prefix_yaw_ok,
                               prefix_start,
                               prefix_end,
                               tracking_pos.getTotalDuration(),
                               tracking_yaw.getTotalDuration());
                return false;
            }
        }

        committed_pos.start_WT = commit_wt;
        committed_yaw.start_WT = commit_wt;

        ExpTraj task_exp_traj;
        task_exp_traj.setGoalConnectedFlag(true);
        task_exp_traj.setWholeTrajKnownFreeFlag(true);
        task_exp_traj.setTrajectory(commit_wt, committed_pos, committed_yaw);

        cmd_traj_info_.setTrajectory(task_exp_traj);
        last_exp_traj_info_ = task_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        {
            TimeConsuming t_viz("tracking_perching_task_viz", false);
            ros_ptr_->vizExpTraj(committed_pos, traj_ns);
            ros_ptr_->vizYawTraj(committed_pos, committed_yaw);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        latest_replan.setExpTraj(committed_pos);
        latest_replan.setExpYawTraj(committed_yaw);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        if (perching_runtime_manager_) {
            perching_runtime_manager_->updateStatusAfterCommit();
        }
        ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_COMMIT_SUCCESS stitched={}, total_duration={:.3f}, handover_delay={:.3f}",
                       stitched,
                       committed_pos.getTotalDuration(),
                       handover_delay);
        return true;
    }

}
