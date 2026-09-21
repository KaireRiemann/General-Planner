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

    void GeneralPlanner::setTrackingYawServo(const bool active) {
        const bool was = tracking_yaw_servo_active_.exchange(active);
        if (active && !was) {
            std::lock_guard<std::mutex> lock(tracking_yaw_servo_mutex_);
            // Enabling must never jump: start from the measured heading and
            // let the slew walk toward the committed setpoint.
            tracking_yaw_servo_last_ = std::isfinite(robot_state_.yaw) ? robot_state_.yaw : 0.0;
            tracking_yaw_servo_last_wt_ = -1.0;
        }
    }

    double GeneralPlanner::trackingYawServoYaw() const {
        std::lock_guard<std::mutex> lock(tracking_yaw_servo_mutex_);
        if (tracking_yaw_servo_last_wt_ < 0.0 || !std::isfinite(tracking_yaw_servo_last_)) {
            return std::isfinite(robot_state_.yaw) ? robot_state_.yaw : 0.0;
        }
        return tracking_yaw_servo_last_;
    }

    void GeneralPlanner::applyTrackingYawServo(double &yaw, double &yaw_dot) {
        if (!std::isfinite(yaw)) {
            yaw = trackingYawServoYaw();
            yaw_dot = 0.0;
            return;
        }
        const double now = ros_ptr_->getSimTime();
        std::lock_guard<std::mutex> lock(tracking_yaw_servo_mutex_);
        if (tracking_yaw_servo_last_wt_ < 0.0 || !std::isfinite(tracking_yaw_servo_last_) ||
            !std::isfinite(now)) {
            tracking_yaw_servo_last_ = std::isfinite(robot_state_.yaw) ? robot_state_.yaw : yaw;
            tracking_yaw_servo_last_wt_ = now;
            yaw = tracking_yaw_servo_last_;
            yaw_dot = 0.0;
            return;
        }
        // Elastic traj_server: at most 0.02 rad per 10 ms control tick toward
        // the scalar setpoint, wrap-aware, with yaw_dot reporting the applied
        // rate instead of a polynomial derivative.
        const double dt = std::clamp(now - tracking_yaw_servo_last_wt_, 1.0e-3, 0.05);
        const double max_step = std::max(0.1, cfg_.tracking_yaw_rate_limit) * dt;
        const double delta = std::remainder(yaw - tracking_yaw_servo_last_, 2.0 * M_PI);
        const double step = std::clamp(delta, -max_step, max_step);
        tracking_yaw_servo_last_ += step;
        tracking_yaw_servo_last_wt_ = now;
        yaw = tracking_yaw_servo_last_;
        yaw_dot = step / dt;
    }

    bool GeneralPlanner::sampleCommittedCommand(StatePVAJ &pvaj, double &yaw,
                                                 double &yaw_dot, bool &on_backup,
                                                 double &start_wt) {
        cmd_traj_info_.lock();
        if (cmd_traj_info_.empty()) {
            cmd_traj_info_.unlock();
            return false;
        }
        start_wt = cmd_traj_info_.getStartWallTime();
        const double duration = cmd_traj_info_.getTotalDuration();
        const double now = ros_ptr_->getSimTime();
        if (!std::isfinite(start_wt) || !std::isfinite(duration) ||
            duration <= 0.0 || !std::isfinite(now)) {
            cmd_traj_info_.unlock();
            return false;
        }
        const double t = std::clamp(now-start_wt, 0.0, duration);
        pvaj = cmd_traj_info_.posTraj().getState(t);
        yaw = cmd_traj_info_.getYaw(t)[0];
        yaw_dot = cmd_traj_info_.getYawRate(t)[0];
        on_backup = cmd_traj_info_.isTTOnBackupTraj(t);
        cmd_traj_info_.unlock();
        if (tracking_yaw_servo_active_.load()) applyTrackingYawServo(yaw, yaw_dot);
        // An exhausted moving endpoint is not a fresh executable command.
        // Leave it to the gateway watchdog instead of advertising it forever.
        if (now-start_wt > duration &&
            (pvaj.col(1).norm()>1.0e-2 || pvaj.col(2).norm()>1.0e-1 ||
             pvaj.col(3).norm()>1.0e-1 || std::abs(yaw_dot)>1.0e-2)) return false;
        return pvaj.allFinite() && std::isfinite(yaw) && std::isfinite(yaw_dot);
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
        robot_on_backup_traj_.store(cmd_traj_info_.isTTOnBackupTraj(eval_t));
        on_backup_traj = robot_on_backup_traj_.load();

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
        if (tracking_yaw_servo_active_.load()) applyTrackingYawServo(yaw, yaw_dot);
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
