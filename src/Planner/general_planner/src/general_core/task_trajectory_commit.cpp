/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <cmath>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {

    StatePVAJ GeneralPlanner::makeTaskHeadState(const bool &from_rest,
                                                const double eval_wall_time) {
        StatePVAJ head = StatePVAJ::Zero();
        head.col(0) = robot_state_.p;
        if (!from_rest) {
            head.col(1) = robot_state_.v;
            head.col(2) = robot_state_.a;
            head.col(3) = robot_state_.j;
        }

        if (!from_rest && !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory pos_traj = cmd_traj_info_.posTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            if (!pos_traj.empty()) {
                const double eval_t = (std::isfinite(eval_wall_time)
                                           ? eval_wall_time
                                           : ros_ptr_->getSimTime() + cfg_.replan_forward_dt) -
                                      start_wt;
                if (eval_t >= 0.0 && eval_t <= total_dur) {
                    return pos_traj.getState(eval_t);
                }
            }
        }
        return head;
    }

    bool GeneralPlanner::commitTaskTrajectory(const Trajectory &pos_traj,
                                              const double &terminal_yaw,
                                              const bool &fix_terminal_yaw,
                                              const std::string &traj_ns) {
        if (pos_traj.empty()) {
            ros_ptr_->warn(" -- [GeneralPlanner] Task trajectory is empty, cannot commit.");
            return false;
        }

        Vec4f init_yaw{robot_state_.yaw, 0.0, 0.0, 0.0};
        if (!cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory yaw_traj = cmd_traj_info_.yawTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            const double eval_t = ros_ptr_->getSimTime() - start_wt;
            StatePVAJ yaw_state;
            if (!yaw_traj.empty() && eval_t >= 0.0 && eval_t <= total_dur &&
                yaw_traj.getState(eval_t, yaw_state)) {
                init_yaw = yaw_state.row(0);
            }
        }

        Vec4f fina_yaw{0.0, 0.0, 0.0, 0.0};
        bool free_end = true;
        if (fix_terminal_yaw && std::isfinite(terminal_yaw)) {
            fina_yaw[0] = terminal_yaw;
            free_end = false;
        }

        Trajectory yaw_traj;
        if (!traj_manager_->yaw()->optimize(init_yaw, fina_yaw, pos_traj, yaw_traj, 3, false, free_end)) {
            ros_ptr_->warn(" -- [GeneralPlanner] Task yaw optimization failed.");
            return false;
        }

        ExpTraj task_exp_traj;
        task_exp_traj.setGoalConnectedFlag(true);
        task_exp_traj.setWholeTrajKnownFreeFlag(true);
        task_exp_traj.setTrajectory(ros_ptr_->getSimTime(), pos_traj, yaw_traj);

        cmd_traj_info_.setTrajectory(task_exp_traj);
        last_exp_traj_info_ = task_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        {
            TimeConsuming t_viz("task_viz", false);
            ros_ptr_->vizExpTraj(pos_traj, traj_ns);
            ros_ptr_->vizYawTraj(pos_traj, yaw_traj);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        latest_replan.setExpTraj(pos_traj);
        latest_replan.setExpYawTraj(yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        return true;
    }

    bool GeneralPlanner::commitTakeoffTrajectory(const Trajectory &pos_traj,
                                                 const std::string &traj_ns) {
        const bool committed = commitTaskTrajectory(pos_traj, NAN, false, traj_ns);
        if (committed && takeoff_runtime_manager_) {
            takeoff_runtime_manager_->updateStatusAfterCommit();
        }
        return committed;
    }

    bool GeneralPlanner::commitSE3AggressiveTrajectory(const Trajectory &pos_traj,
                                                       const std::string &traj_ns) {
        static bool warning_printed = false;
        if (!warning_printed) {
            ros_ptr_->warn(" -- [SE3Aggressive] SE3 trajectory requires a flatness-aware controller for reliable execution.");
            warning_printed = true;
        }
        const bool fix_terminal_yaw = cfg_.se3_use_yaw && std::isfinite(gi_.goal_yaw);
        return commitTaskTrajectory(pos_traj, gi_.goal_yaw, fix_terminal_yaw, traj_ns);
    }

}
