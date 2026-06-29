/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <checker/common_checker.hpp>
#include <algorithm>
#include <cmath>

namespace general_planner {
    namespace {
        void makeHoldCommandFromRobotState(const rog_map::RobotState &robot_state,
                                           StatePVAJ &pvaj,
                                           double &yaw,
                                           double &yaw_dot,
                                           bool &on_backup_traj,
                                           bool &traj_finish) {
            pvaj.setZero();
            if (robot_state.rcv && robot_state.p.allFinite()) {
                pvaj.col(0) = robot_state.p;
            }
            yaw = std::isfinite(robot_state.yaw) ? robot_state.yaw : 0.0;
            yaw_dot = 0.0;
            on_backup_traj = false;
            traj_finish = true;
        }
    }

    void GeneralPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                               double &yaw,
                                               double &yaw_dot,
                                               bool &on_backup_traj,
                                               bool &traj_finish) {
        cmd_traj_info_.lock();
        if (cmd_traj_info_.empty()) {
            cmd_traj_info_.unlock();
            ros_ptr_->warn(" -- [Checker] getOneCommandFromTraj called with empty committed trajectory.");
            makeHoldCommandFromRobotState(robot_state_, pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            return;
        }
        const double cur_t = ros_ptr_->getSimTime();
        const double cmd_start_WT = cmd_traj_info_.getStartWallTime();
//        const bool &backup_avilibale = cmd_traj_info_.backupTrajAvilibale();
//        const double &backup_start_TT = cmd_traj_info_.getBackupTrajStartTT();
        const double total_dur = cmd_traj_info_.getTotalDuration();
        if (!std::isfinite(cur_t) || !std::isfinite(cmd_start_WT) ||
            !std::isfinite(total_dur) || total_dur <= 1.0e-6) {
            cmd_traj_info_.unlock();
            ros_ptr_->warn(" -- [Checker] getOneCommandFromTraj has invalid timing: cur_t={}, start_WT={}, duration={}.",
                           cur_t, cmd_start_WT, total_dur);
            makeHoldCommandFromRobotState(robot_state_, pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            return;
        }

        traj_finish = (cur_t - cmd_start_WT) > total_dur;
        const double eval_t = traj_finish ? total_dur : std::clamp(cur_t - cmd_start_WT, 0.0, total_dur);

//        bool last_round_robot_on_backup_traj = robot_on_backup_traj_;
        robot_on_backup_traj_ = cmd_traj_info_.isTTOnBackupTraj(eval_t);
        on_backup_traj = robot_on_backup_traj_;

        pvaj = cmd_traj_info_.posTraj().getState(eval_t);

        /// Get Yaw planning
        static double last_yaw = robot_state_.yaw;

        yaw = cmd_traj_info_.getYaw((eval_t))[0];
        yaw_dot = cmd_traj_info_.getYawRate((eval_t))[0];

        if (isnan(yaw)) {
            yaw = last_yaw;
            yaw_dot = 0;
        } else {
            last_yaw = yaw;
        }
        if (isnan(yaw_dot)) {
            yaw_dot = 0;
        }
        if (checker::checkStateFinite(pvaj, "cmd_pvaj").rejected() ||
            !std::isfinite(yaw) || !std::isfinite(yaw_dot)) {
            cmd_traj_info_.unlock();
            ros_ptr_->warn(" -- [Checker] getOneCommandFromTraj sampled invalid command at eval_t={}.", eval_t);
            makeHoldCommandFromRobotState(robot_state_, pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            return;
        }
        if (takeoff_runtime_manager_ && active_takeoff_problem_valid_) {
            takeoff_runtime_manager_->updateStatusByPosition(pvaj.col(0),
                                                             active_takeoff_problem_);
        }

//        if (last_round_robot_on_backup_traj != robot_on_backup_traj_) {
//            if (last_round_robot_on_backup_traj) {
//                ros_ptr_->info(" -- [CMD] Emergency Stop End ========================");
//            } else {
//                ros_ptr_->info(" -- [CMD] Emergency Stop Start ========================");
//            }
//        }

//        double cur_yaw = geometry_utils::get_yaw_from_quaternion(robot_state_.q);
        cmd_traj_info_.unlock();
    }

    void GeneralPlanner::getModuleTimeConsuming(vector<double> &time) {
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }

}
