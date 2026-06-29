/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {
    namespace {
        bool buildConstantPositionTrajectory(const Vec3f &position,
                                             const double duration,
                                             const double start_wt,
                                             Trajectory &traj) {
            if (!position.allFinite() ||
                !std::isfinite(duration) ||
                duration <= 1.0e-5 ||
                !std::isfinite(start_wt)) {
                return false;
            }

            Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 8);
            coeff.col(7) = position;
            traj.clear();
            traj.emplace_back(duration, coeff);
            traj.start_WT = start_wt;
            return !traj.empty();
        }

        bool buildConstantYawTrajectory(const double yaw,
                                        const double duration,
                                        const double start_wt,
                                        Trajectory &traj) {
            if (!std::isfinite(yaw) ||
                !std::isfinite(duration) ||
                duration <= 1.0e-5 ||
                !std::isfinite(start_wt)) {
                return false;
            }

            Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 8);
            coeff(0, 7) = yaw;
            traj.clear();
            traj.emplace_back(duration, coeff);
            traj.start_WT = start_wt;
            return !traj.empty();
        }
    }

    bool GeneralPlanner::commitTrackingHoldTrajectory(const std::string &reason,
                                                      const double duration,
                                                      const bool require_safe) {
        if (!robot_state_.rcv || !robot_state_.p.allFinite()) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMIT_FAILED reason={}, robot_state_valid=0",
                           reason);
            return false;
        }

        const double commit_wt = ros_ptr_->getSimTime();
        const double hold_duration =
                std::max(0.2,
                         std::isfinite(duration) && duration > 1.0e-5
                             ? duration
                             : std::max(0.8, cfg_.tracking_min_commit_duration));
        Trajectory hold_pos_traj;
        Trajectory hold_yaw_traj;
        const double hold_yaw =
                std::isfinite(robot_state_.yaw) ? robot_state_.yaw : 0.0;
        if (!buildConstantPositionTrajectory(robot_state_.p,
                                             hold_duration,
                                             commit_wt,
                                             hold_pos_traj) ||
            !buildConstantYawTrajectory(hold_yaw,
                                        hold_duration,
                                        commit_wt,
                                        hold_yaw_traj)) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMIT_FAILED reason={}, build_constant_traj=0",
                           reason);
            return false;
        }

        std::string safety_reason;
        std::string safety_detail;
        const double safety_horizon =
                std::min(std::max(0.0, cfg_.tracking_keep_old_horizon),
                         hold_pos_traj.getTotalDuration());
        const bool hold_safe =
                trackingTrajectorySafeForHorizonDetailed(hold_pos_traj,
                                                         0.0,
                                                         safety_horizon,
                                                         cfg_.tracking_keep_old_safety_dt,
                                                         &safety_reason,
                                                         &safety_detail);
        if (require_safe && !hold_safe) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMIT_FAILED reason={}, require_safe=1, safety_reason={}, safety_detail={}",
                           reason,
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
        robot_on_backup_traj_ = false;
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
