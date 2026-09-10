/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_brake.hpp>

#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {
    bool GeneralPlanner::commitTrackingHoldTrajectory(const std::string &reason,
                                                      const double duration,
                                                      const bool require_safe) {
        if (!robot_state_.rcv || !robot_state_.p.allFinite()) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMIT_FAILED reason={}, robot_state_valid=0",
                           reason);
            return false;
        }

        const double commit_wt = ros_ptr_->getSimTime();
        double hold_duration =
                std::max(0.2,
                         std::isfinite(duration) && duration > 1.0e-5
                             ? duration
                             : std::max(0.8, cfg_.tracking_min_commit_duration));
        Trajectory hold_pos_traj;
        Trajectory hold_yaw_traj;
        StatePVAJ start = StatePVAJ::Zero();
        start.col(0) = robot_state_.p;
        start.col(1) = robot_state_.v;
        start.col(2) = robot_state_.a;
        StatePVAJ yaw_start = StatePVAJ::Zero();
        yaw_start(0,0) = std::isfinite(robot_state_.yaw) ? robot_state_.yaw : 0.0;
        // Start at the currently issued command, preserving p/v/a/j and yaw rate.
        cmd_traj_info_.lock();
        if (!cmd_traj_info_.empty()) {
            const double t = std::clamp(commit_wt-cmd_traj_info_.getStartWallTime(),
                                      0.0,cmd_traj_info_.getTotalDuration());
            cmd_traj_info_.posTraj().getState(t,start);
            if (!cmd_traj_info_.yawTraj().empty()) cmd_traj_info_.yawTraj().getState(t,yaw_start);
        }
        cmd_traj_info_.unlock();
        if (!start.allFinite() || !yaw_start.allFinite()) return false;
        const double acc_limit = std::max(0.5, cfg_.tracking_traj_cfg.max_acc);
        const double jerk_limit = std::max(1.0, cfg_.tracking_traj_cfg.max_jerk);
        hold_duration = std::max(hold_duration, 2.0*start.col(1).norm()/acc_limit);
        bool feasible = false;
        for (int attempt=0; attempt<8; ++attempt) {
            hold_pos_traj.clear();
            hold_yaw_traj.clear();
            hold_pos_traj.emplace_back(hold_duration, trackingBrakeCoefficients(start,hold_duration));
            hold_yaw_traj.emplace_back(hold_duration, trackingBrakeCoefficients(yaw_start,hold_duration));
            hold_pos_traj.start_WT = hold_yaw_traj.start_WT = commit_wt;
            feasible = true;
            for (int k=0;k<=100;++k) {
                const auto state = hold_pos_traj.getState(hold_duration*k/100.0);
                const auto yaw = hold_yaw_traj.getState(hold_duration*k/100.0);
                if (state.col(2).norm() > std::max(acc_limit,start.col(2).norm())*1.02 ||
                    state.col(3).norm() > std::max(jerk_limit,start.col(3).norm())*1.02 ||
                    std::abs(yaw(0,1)) > std::max(cfg_.tracking_yaw_rate_limit,std::abs(yaw_start(0,1)))*1.02 ||
                    std::abs(yaw(0,2)) > std::max(cfg_.tracking_yaw_acceleration_limit,std::abs(yaw_start(0,2)))*1.02)
                    feasible = false;
            }
            if (feasible) break;
            hold_duration *= 1.3;
        }
        if (!feasible) {
            ros_ptr_->warn(" -- [Tracking] braking boundary infeasible");
            return false;
        }

        std::string safety_reason;
        std::string safety_detail;
        const double safety_horizon =
                hold_pos_traj.getTotalDuration();
        const bool hold_safe =
                trackingTrajectorySafeForHorizonDetailed(hold_pos_traj,
                                                         0.0,
                                                         safety_horizon,
                                                         cfg_.tracking_keep_old_safety_dt,
                                                         &safety_reason,
                                                         &safety_detail);
        if (!hold_safe) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMIT_FAILED reason={}, require_safe={}, safety_reason={}, safety_detail={}",
                           reason, require_safe,
                           safety_reason.empty() ? "none" : safety_reason,
                           safety_detail.empty() ? "none" : safety_detail);
            return false;
        }

        ExpTraj hold_exp_traj;
        hold_exp_traj.setGoalConnectedFlag(false);
        hold_exp_traj.setWholeTrajKnownFreeFlag(hold_safe);
        hold_exp_traj.setTrajectory(commit_wt, hold_pos_traj, hold_yaw_traj);

        cmd_traj_info_.setTrajectory(hold_exp_traj);
        last_exp_traj_info_ = hold_exp_traj;
        robot_on_backup_traj_.store(false);
        gi_.new_goal = false;

        latest_replan.setExpTraj(hold_pos_traj);
        latest_replan.setExpYawTraj(hold_yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        setTrackingDiagnostic("recovery_hold",
                              fmt::format("reason={};hold_safe={};safety_reason={};safety_detail={}",
                                          reason,
                                          static_cast<int>(hold_safe),
                                          safety_reason.empty() ? "none" : safety_reason,
                                          safety_detail.empty() ? "none" : safety_detail),
                              0,
                              0,
                              0,
                              hold_duration);

        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            tracking_runtime_manager_->onCommitted();
        }
        resetTrackingCommitCounters();
        clearTrackingCommitRejectInfo();

        {
            TimeConsuming t_viz("tracking_hold_viz", false);
            ros_ptr_->vizExpTraj(hold_pos_traj, "tracking_hold");
            ros_ptr_->vizYawTraj(hold_pos_traj, hold_yaw_traj);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMITTED reason={}, duration={:.3f}, hold_safe={}, safety_reason={}, pos={}",
                       reason,
                       hold_duration,
                       hold_safe,
                       safety_reason.empty() ? "none" : safety_reason,
                       robot_state_.p);
        return true;
    }

}
