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
#include <cmath>
#include <fmt/format.h>

namespace general_planner {
    void GeneralPlanner::invalidateTrackingCommand(const std::string &reason) {
        cmd_traj_info_.lock();
        cmd_traj_info_.setEmpty();
        cmd_traj_info_.unlock();
        last_exp_traj_info_.setEmpty();
        robot_on_backup_traj_.store(false);
        if (tracking_runtime_manager_) tracking_runtime_manager_->reset();
        resetTrackingRuntimeDecision(reason);
        last_tracking_runtime_reset_ = true;
    }

    void GeneralPlanner::resetTrackingRuntimeDecision(const std::string &reason) {
        last_tracking_runtime_reset_ = false;
        last_tracking_runtime_preserved_ = false;
        last_tracking_runtime_reason_ = reason;
    }

    void GeneralPlanner::maybeResetTrackingRuntimeForReplan(
            const bool new_task,
            const std::string &context) {
        resetTrackingRuntimeDecision(new_task ? "new_task_runtime_check" : "not_new_task");
        if (!new_task) {
            last_tracking_runtime_reason_ = context + ":not_new_task";
            return;
        }

        const double committed_remaining = getCommittedTrajectoryRemainingDuration();
        const bool has_committed_tracking =
                tracking_runtime_manager_->hasCommittedTracking();
        if (tracking_runtime_manager_->hasExecutionHistory() ||
            (committed_remaining > 1.0e-3 &&
             (has_committed_tracking || tracking_runtime_manager_->consecutiveReject() > 0))) {
            last_tracking_runtime_preserved_ = true;
            last_tracking_runtime_reason_ =
                    fmt::format("{}:prediction_update_preserve_command,remaining={:.3f},has_committed_tracking={}",
                                context,
                                committed_remaining,
                                static_cast<int>(has_committed_tracking));
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [Tracking] TRACKING_RUNTIME_PRESERVED reason={}",
                               last_tracking_runtime_reason_);
            }
            return;
        }

        tracking_runtime_manager_->reset();
        last_tracking_runtime_reset_ = true;
        last_tracking_runtime_reason_ =
                fmt::format("{}:hard_reset_no_committed_tracking,remaining={:.3f},has_committed_tracking={}",
                            context,
                            committed_remaining,
                            static_cast<int>(has_committed_tracking));
        if (cfg_.print_log) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_RUNTIME_RESET reason={}",
                           last_tracking_runtime_reason_);
        }
    }

    void GeneralPlanner::clearTrackingCommitRejectInfo() {
        last_tracking_commit_reject_reason_.clear();
        last_tracking_commit_reject_detail_.clear();
    }

    void GeneralPlanner::setTrackingCommitRejectInfo(const std::string &reason,
                                                     const std::string &detail) {
        last_tracking_commit_reject_reason_ = reason;
        last_tracking_commit_reject_detail_ = detail;
    }

    void GeneralPlanner::setTrackingDiagnostic(const std::string &phase,
                                               const std::string &reason,
                                               const std::size_t guide_path_size,
                                               const std::size_t sfc_size,
                                               const std::size_t target_prediction_size,
                                               const double out_traj_duration) {
        last_tracking_diag_phase_ = phase;
        last_tracking_diag_reason_ = reason;
        last_tracking_diag_guide_path_size_ = guide_path_size;
        last_tracking_diag_sfc_size_ = sfc_size;
        last_tracking_diag_target_prediction_size_ = target_prediction_size;
        last_tracking_diag_out_traj_duration_ = out_traj_duration;
    }

    void GeneralPlanner::setTrackingDiagnostic(const std::string &phase,
                                               const std::string &reason,
                                               const traj_opt::TrackingProblem &problem,
                                               const double out_traj_duration) {
        setTrackingDiagnostic(phase,
                              reason,
                              problem.guide_path.size(),
                              problem.sfcs.size(),
                              problem.target_prediction.size(),
                              out_traj_duration);
    }

    GeneralPlanner::TrackingDiagnosticSnapshot
    GeneralPlanner::getLatestTrackingDiagnosticSnapshot() {
        TrackingDiagnosticSnapshot snapshot;
        snapshot.phase = last_tracking_diag_phase_;
        snapshot.reason = last_tracking_diag_reason_;
        snapshot.guide_path_size = last_tracking_diag_guide_path_size_;
        snapshot.sfc_size = last_tracking_diag_sfc_size_;
        snapshot.target_prediction_size = last_tracking_diag_target_prediction_size_;
        snapshot.out_traj_duration = last_tracking_diag_out_traj_duration_;
        snapshot.consecutive_keep_old =
                tracking_runtime_manager_->consecutiveKeepOld();
        snapshot.consecutive_reject =
                tracking_runtime_manager_->consecutiveReject();
        snapshot.last_commit_wt = last_tracking_commit_wt_;
        snapshot.last_commit_reject_reason = last_tracking_commit_reject_reason_;
        snapshot.last_commit_reject_detail = last_tracking_commit_reject_detail_;
        snapshot.runtime_manager_enabled = static_cast<bool>(tracking_runtime_manager_);
        snapshot.has_committed_tracking =
                tracking_runtime_manager_->hasCommittedTracking();
        snapshot.committed_remaining = getCommittedTrajectoryRemainingDuration();
        snapshot.runtime_reset = last_tracking_runtime_reset_;
        snapshot.runtime_preserved = last_tracking_runtime_preserved_;
        snapshot.runtime_reason = last_tracking_runtime_reason_;
        return snapshot;
    }

    std::string GeneralPlanner::getTrackingConfigSummary() const {
        return fmt::format("backend=elastic_tracker;corridor=CIRI;distance={:.3f};tolerance={:.3f};"
            "height_offset={:.3f};horizon={:.3f};sample_dt={:.3f};rho_tracking={:.1f};rho_visibility={:.1f};"
            "max_vel={:.3f};max_acc={:.3f}",cfg_.tracking_distance,cfg_.tracking_distance_tolerance,
            cfg_.tracking_height_offset,cfg_.tracking_nominal_horizon,cfg_.tracking_sample_dt,
            cfg_.tracking_weight_tracking,cfg_.tracking_weight_visible_region,
            cfg_.tracking_traj_cfg.max_vel,cfg_.tracking_traj_cfg.max_acc);
    }
} // namespace general_planner
