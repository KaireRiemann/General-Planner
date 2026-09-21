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
        robot_on_backup_traj_.store(false);
        gi_.new_goal = false;
        // Perching handover flies the exact perching yaw polynomial.
        setTrackingYawServo(false);

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
