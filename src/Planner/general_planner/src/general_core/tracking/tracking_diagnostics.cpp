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

namespace general_planner {
    namespace {
        double trackingAdaptiveFovRange(const double configured_range,
                                        const double tracking_distance,
                                        const double distance_upper_tolerance,
                                        const double distance_tolerance,
                                        const double height_offset,
                                        const double height_tolerance,
                                        const double horizontal_fov_deg,
                                        const double vertical_fov_deg) {
            constexpr double kPi = 3.14159265358979323846;
            constexpr double kDegToRad = kPi / 180.0;
            const double horizontal_upper =
                    std::max(0.05,
                             tracking_distance +
                                     std::max({0.0,
                                               distance_upper_tolerance,
                                               distance_tolerance}));
            const double vertical_upper =
                    std::max(0.0, std::abs(height_offset) + std::max(0.0, height_tolerance));
            const double base_range = configured_range > 0.0 ? configured_range : horizontal_upper;
            const double half_h =
                    std::clamp(0.5 * std::max(1.0, horizontal_fov_deg) * kDegToRad,
                               kPi / 180.0,
                               0.5 * kPi - 1.0e-3);
            const double half_v =
                    std::clamp(0.5 * std::max(1.0, vertical_fov_deg) * kDegToRad,
                               kPi / 180.0,
                               0.5 * kPi - 1.0e-3);
            const double footprint_scale = std::hypot(std::tan(half_h), std::tan(half_v));
            const double geometry_range = std::hypot(horizontal_upper, vertical_upper);
            const double footprint_range = horizontal_upper + vertical_upper * footprint_scale;
            return std::max({0.05, base_range, geometry_range, footprint_range});
        }

        double trackingAdaptiveFovRange(const Config &cfg) {
            return trackingAdaptiveFovRange(cfg.tracking_fov_range,
                                            cfg.tracking_distance,
                                            cfg.tracking_distance_upper_tolerance,
                                            cfg.tracking_distance_tolerance,
                                            cfg.tracking_height_offset,
                                            cfg.tracking_height_tolerance,
                                            cfg.tracking_fov_horizontal_deg,
                                            cfg.tracking_fov_vertical_deg);
        }

        double trackingUpperBand(const Config &cfg) {
            return std::max(0.05,
                            cfg.tracking_distance +
                            std::max({0.0,
                                      cfg.tracking_distance_upper_tolerance,
                                      cfg.tracking_distance_tolerance}));
        }

        double trackingSoftRecoveryEntryDistance(const Config &cfg) {
            const double upper_band = trackingUpperBand(cfg);
            if (!cfg.tracking_soft_recovery_enable) {
                return std::max(upper_band, cfg.tracking_reacquire_distance);
            }
            return std::max(upper_band,
                            upper_band +
                            std::max(0.0, cfg.tracking_soft_recovery_margin));
        }
    }

    void GeneralPlanner::resetTrackingCommitCounters() {
        tracking_consecutive_keep_old_ = 0;
        tracking_consecutive_reject_ = 0;
        last_tracking_commit_wt_ = ros_ptr_ ? ros_ptr_->getSimTime() : -1.0;
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
        if (!cfg_.tracking_runtime_manager_enable || !tracking_runtime_manager_) {
            last_tracking_runtime_reason_ = context + ":runtime_manager_disabled";
            return;
        }
        if (!new_task) {
            last_tracking_runtime_reason_ = context + ":not_new_task";
            return;
        }

        const double committed_remaining = getCommittedTrajectoryRemainingDuration();
        const bool has_committed_tracking =
                tracking_runtime_manager_->hasCommittedTracking();
        if (has_committed_tracking && committed_remaining > 1.0e-3) {
            last_tracking_runtime_preserved_ = true;
            last_tracking_runtime_reason_ =
                    fmt::format("{}:prediction_update_preserve_committed,remaining={:.3f},has_committed_tracking=1",
                                context,
                                committed_remaining);
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
                tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld()
                                          : tracking_consecutive_keep_old_;
        snapshot.consecutive_reject =
                tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject()
                                          : tracking_consecutive_reject_;
        snapshot.last_commit_wt = last_tracking_commit_wt_;
        snapshot.last_commit_reject_reason = last_tracking_commit_reject_reason_;
        snapshot.last_commit_reject_detail = last_tracking_commit_reject_detail_;
        snapshot.runtime_manager_enabled = cfg_.tracking_runtime_manager_enable &&
                                           static_cast<bool>(tracking_runtime_manager_);
        snapshot.has_committed_tracking =
                tracking_runtime_manager_ ? tracking_runtime_manager_->hasCommittedTracking()
                                          : !cmd_traj_info_.empty();
        snapshot.committed_remaining = getCommittedTrajectoryRemainingDuration();
        snapshot.runtime_reset = last_tracking_runtime_reset_;
        snapshot.runtime_preserved = last_tracking_runtime_preserved_;
        snapshot.runtime_reason = last_tracking_runtime_reason_;
        return snapshot;
    }

    std::string GeneralPlanner::getTrackingConfigSummary() const {
        return fmt::format("tracking_distance={:.3f};distance_tolerance={:.3f};"
                           "distance_lower_tolerance={:.3f};distance_upper_tolerance={:.3f};"
                           "height_offset={:.3f};height_tolerance={:.3f};safe_distance={:.3f};"
                           "hard_safe_distance={:.3f};fov_h_deg={:.3f};fov_v_deg={:.3f};"
                           "fov_range={:.3f};fov_range_effective={:.3f};"
                           "fov_check_strict={};fov_commit_check_enable={};"
                           "frontend_fov_feasibility_enable={};runtime_manager_enable={};"
                           "keep_old_horizon={:.3f};keep_old_safety_dt={:.3f};"
                           "keep_old_requires_fov={};short_safety_grace_enable={};"
                           "anti_rollback_enable={};reacquire_distance={:.3f};"
                           "soft_recovery_enable={};soft_recovery_entry={:.3f};"
                           "reacquire_min_progress_distance={:.3f};reacquire_min_progress_ratio={:.3f};"
                           "reacquire_fov_relax_enable={};reacquire_fov_deferred_strict_enable={};"
                           "optimizer_commit_safety_precheck_enable={};"
                           "detour_grace_enable={};detour_grace_horizon={:.3f};"
                           "frontend_astar={};use_visible_region={};max_vel={:.3f};max_acc={:.3f}",
                           cfg_.tracking_distance,
                           cfg_.tracking_distance_tolerance,
                           cfg_.tracking_distance_lower_tolerance,
                           cfg_.tracking_distance_upper_tolerance,
                           cfg_.tracking_height_offset,
                           cfg_.tracking_height_tolerance,
                           cfg_.tracking_safe_distance,
                           cfg_.tracking_hard_safe_distance,
                           cfg_.tracking_fov_horizontal_deg,
                           cfg_.tracking_fov_vertical_deg,
                           cfg_.tracking_fov_range,
                           trackingAdaptiveFovRange(cfg_),
                           static_cast<int>(cfg_.tracking_fov_check_strict),
                           static_cast<int>(cfg_.tracking_fov_commit_check_enable),
                           static_cast<int>(cfg_.tracking_frontend_fov_feasibility_enable &&
                                            cfg_.tracking_fov_check_strict),
                           static_cast<int>(cfg_.tracking_runtime_manager_enable),
                           cfg_.tracking_keep_old_horizon,
                           cfg_.tracking_keep_old_safety_dt,
                           static_cast<int>(cfg_.tracking_keep_old_requires_fov &&
                                            cfg_.tracking_fov_check_strict),
                           static_cast<int>(cfg_.tracking_keep_old_short_safety_grace_enable),
                           static_cast<int>(cfg_.tracking_anti_rollback_enable),
                           cfg_.tracking_reacquire_distance,
                           static_cast<int>(cfg_.tracking_soft_recovery_enable),
                           trackingSoftRecoveryEntryDistance(cfg_),
                           cfg_.tracking_reacquire_min_progress_distance,
                           cfg_.tracking_reacquire_min_progress_ratio,
                           static_cast<int>(cfg_.tracking_reacquire_fov_relax_enable),
                           static_cast<int>(cfg_.tracking_reacquire_fov_deferred_strict_enable),
                           static_cast<int>(cfg_.tracking_optimizer_commit_safety_precheck_enable),
                           static_cast<int>(cfg_.tracking_detour_grace_enable),
                           cfg_.tracking_detour_grace_horizon,
                           static_cast<int>(cfg_.tracking_frontend_astar),
                           static_cast<int>(cfg_.tracking_use_visible_region),
                           cfg_.exp_traj_cfg.max_vel,
                           cfg_.exp_traj_cfg.max_acc);
    }

}
