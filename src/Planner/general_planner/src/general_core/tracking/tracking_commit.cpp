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
#include <limits>
#include <fmt/format.h>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {

    bool GeneralPlanner::commitTrackingTrajectory(const Trajectory &pos_traj,
                                                const Trajectory &optimized_yaw_traj,
                                                const traj_opt::DynamicTargetStates &target_prediction,
                                                const std::string &traj_ns,
                                                const double candidate_head_wt,
                                                const bool allow_reacquire_fov_relax,
                                                const bool allow_old_prefix) {
        if (trackingPerchingPerchingActive()) {
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            setTrackingCommitRejectInfo("perching owns committed trajectory",
                                        "failure=perching_owns_committed_trajectory");
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_COMMIT_BLOCKED_PERCHING_ACTIVE reason=perching_owns_committed_trajectory");
            return false;
        }
        if (pos_traj.empty()) {
            setTrackingCommitRejectInfo("empty tracking trajectory",
                                        "failure=empty_tracking_trajectory");
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking trajectory is empty, cannot commit.");
            return false;
        }
        clearTrackingCommitRejectInfo();

        const double commit_wt = ros_ptr_->getSimTime();
        bool has_old_cmd = false;
        Trajectory old_pos_traj;
        Trajectory old_yaw_traj;
        double old_start_wt = 0.0;
        double old_total_dur = 0.0;
        if (!cmd_traj_info_.empty()) {
            has_old_cmd = true;
            cmd_traj_info_.lock();
            old_pos_traj = cmd_traj_info_.posTraj();
            old_yaw_traj = cmd_traj_info_.yawTraj();
            old_start_wt = cmd_traj_info_.getStartWallTime();
            old_total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();
        }

        const bool runtime_has_committed_tracking =
                cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_
                    ? tracking_runtime_manager_->hasCommittedTracking()
                    : has_old_cmd;
        const double old_local_t_raw = has_old_cmd
                                           ? commit_wt - old_start_wt
                                           : std::numeric_limits<double>::quiet_NaN();
        const bool old_time_valid =
                has_old_cmd &&
                std::isfinite(old_start_wt) &&
                std::isfinite(old_total_dur) &&
                old_total_dur > 1.0e-6;
        const bool old_currently_active =
                old_time_valid &&
                old_local_t_raw >= -std::max(0.0, cfg_.tracking_commit_start_time_tolerance) &&
                old_local_t_raw < old_total_dur - 1.0e-3;
        const bool old_tracking_active_for_prefix =
                has_old_cmd &&
                runtime_has_committed_tracking &&
                old_currently_active &&
                !old_pos_traj.empty() &&
                !old_yaw_traj.empty();
        const bool should_stitch_old_prefix =
                allow_old_prefix && old_tracking_active_for_prefix;

        if (has_old_cmd && !should_stitch_old_prefix && cfg_.print_log) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking old prefix disabled: allow_old_prefix={}, runtime_has_committed_tracking={}, old_currently_active={}, old_local_t={:.3f}, old_total_dur={:.3f}, candidate_head_wt={:.3f}, commit_wt={:.3f}.",
                           allow_old_prefix,
                           runtime_has_committed_tracking,
                           old_currently_active,
                           std::isfinite(old_local_t_raw) ? old_local_t_raw : 0.0,
                           old_total_dur,
                           candidate_head_wt,
                           commit_wt);
        }

        auto keepOldFromSnapshot = [&](const std::string &reason) -> bool {
            if (!has_old_cmd ||
                !runtime_has_committed_tracking ||
                !old_currently_active ||
                old_pos_traj.empty() ||
                old_yaw_traj.empty()) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_INACTIVE reason={}, activity_reason=no active committed tracking snapshot, runtime_has_committed_tracking={}, old_currently_active={}, old_local_t={:.3f}, old_total_dur={:.3f}",
                                   reason,
                                   runtime_has_committed_tracking,
                                   old_currently_active,
                                   std::isfinite(old_local_t_raw) ? old_local_t_raw : 0.0,
                                   old_total_dur);
                }
                return false;
            }

            const double old_local_t =
                    std::clamp(commit_wt - old_start_wt, 0.0, old_total_dur);
            const auto activity =
                    evaluateTrackingTrajectoryActivity(old_pos_traj,
                                                       old_local_t,
                                                       target_prediction,
                                                       cfg_.tracking_keep_old_horizon,
                                                       cfg_.tracking_keep_old_safety_dt);
            if (!activity.valid || !activity.safe || !activity.active) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_INACTIVE reason={}, activity_reason={}, old_local_t={:.3f}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                                   reason,
                                   activity.reason,
                                   old_local_t,
                                   activity.remaining,
                                   activity.speed0,
                                   activity.displacement,
                                   activity.progress,
                                   activity.expected_progress,
                                   activity.avg_tracking_error);
                }
                return false;
            }

            std::string old_fov_reason;
            if (!trackingSnapshotSatisfiesFovForKeepOld(old_pos_traj,
                                                        old_yaw_traj,
                                                        old_local_t,
                                                        target_prediction,
                                                        &old_fov_reason)) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_FOV reason={}, fov_reason={}, old_local_t={:.3f}, old_remaining={:.3f}",
                                   reason,
                                   old_fov_reason,
                                   old_local_t,
                                   activity.remaining);
                }
                return false;
            }

            latest_replan.setExpTraj(old_pos_traj);
            latest_replan.setExpYawTraj(old_yaw_traj);
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [Tracking] TRACKING_KEEP_OLD_ACTIVE reason={}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}, keep_old_count={}, reject_count={}",
                               reason,
                               activity.remaining,
                               activity.speed0,
                               activity.displacement,
                               activity.progress,
                               activity.expected_progress,
                               activity.avg_tracking_error,
                               tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld() : tracking_consecutive_keep_old_,
                               tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject() : tracking_consecutive_reject_);
            }
            return true;
        };

        auto decisionTypeName = [](const TrackingRuntimeManager::DecisionType type) -> const char * {
            switch (type) {
                case TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE:
                    return "COMMIT_CANDIDATE";
                case TrackingRuntimeManager::DecisionType::KEEP_OLD:
                    return "KEEP_OLD";
                case TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE:
                    return "FORCE_COMMIT_CANDIDATE";
                case TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL:
                    return "REJECT_AND_FAIL";
            }
            return "UNKNOWN";
        };

        auto logRuntimeDecision =
                [&](const TrackingRuntimeManager::Decision &decision,
                    const Trajectory &candidate,
                    const bool anti_rollback_pass,
                    const bool candidate_fov_ok,
                    const double prefix_duration,
                    const double runtime_eval_start,
                    const int anti_rollback_worse_count,
                    const double anti_rollback_max_regression,
                    const std::string &candidate_safe_reason,
                    const std::string &tag) {
            if (!cfg_.print_log) {
                return;
            }
            const double guard_h =
                    std::min(cfg_.tracking_no_motion_check_horizon,
                             std::max(0.0, candidate.getTotalDuration() -
                                            std::clamp(runtime_eval_start,
                                                       0.0,
                                                       candidate.getTotalDuration())));
            const TrackingMotionMetrics candidate_metrics =
                    computeTrackingMotionMetrics(candidate,
                                                 target_prediction,
                                                 cfg_,
                                                 runtime_eval_start,
                                                 runtime_eval_start,
                                                 guard_h);
            const char *log_name = "TRACKING_MANAGER_DECISION_REJECT";
            if (decision.type == TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE) {
                log_name = "TRACKING_MANAGER_DECISION_COMMIT";
            } else if (decision.type ==
                       TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE) {
                log_name = "TRACKING_MANAGER_DECISION_FORCE_COMMIT";
            } else if (decision.type == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
                log_name = "TRACKING_MANAGER_DECISION_KEEP_OLD";
            }
            ros_ptr_->info(" -- [Tracking] {} tag={}, decision={}, reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, candidate_safe={}, candidate_safe_reason={}, candidate_commandable={}, candidate_fov_ok={}, anti_rollback_pass={}, anti_rollback_worse_count={}, anti_rollback_max_regression={:.3f}, bypass_anti_rollback={}, candidate_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, target_speed_xy={:.3f}, target_speed_z={:.3f}, target_speed_3d={:.3f}, old_remaining={:.3f}, old_activity_reason={}, old_speed_xy={:.3f}, old_speed_z={:.3f}, old_speed_3d={:.3f}, old_disp_xy={:.3f}, old_disp_z={:.3f}, old_disp_3d={:.3f}, old_progress_xy={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}, keep_old_count={}, reject_count={}",
                           log_name,
                           tag,
                           decisionTypeName(decision.type),
                           decision.reason,
                           last_tracking_diag_guide_path_size_,
                           last_tracking_diag_sfc_size_,
                           last_tracking_diag_target_prediction_size_,
                           last_tracking_diag_out_traj_duration_,
                           decision.candidate_safe,
                           candidate_safe_reason,
                           decision.candidate_commandable,
                           candidate_fov_ok,
                           anti_rollback_pass,
                           anti_rollback_worse_count,
                           anti_rollback_max_regression,
                           decision.bypass_anti_rollback,
                           candidate.getTotalDuration(),
                           prefix_duration,
                           runtime_eval_start,
                           candidate_metrics.displacement_xy,
                           candidate_metrics.displacement_z,
                           candidate_metrics.displacement_3d,
                           candidate_metrics.speed_xy,
                           candidate_metrics.speed_z,
                           candidate_metrics.speed_3d,
                           candidate_metrics.target_speed_xy,
                           candidate_metrics.target_speed_z,
                           candidate_metrics.target_speed_3d,
                           decision.old_activity.remaining,
                           decision.old_activity.reason,
                           decision.old_activity.speed_xy,
                           decision.old_activity.speed_z,
                           decision.old_activity.speed_3d,
                           decision.old_activity.displacement_xy,
                           decision.old_activity.displacement_z,
                           decision.old_activity.displacement_3d,
                           decision.old_activity.progress_xy,
                           decision.old_activity.progress_3d,
                           decision.old_activity.expected_progress,
                           decision.old_activity.avg_tracking_error,
                           tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld() : tracking_consecutive_keep_old_,
                           tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject() : tracking_consecutive_reject_);
            if (decision.candidate_safe && !decision.candidate_commandable) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_NO_MOTION reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, candidate_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, target_speed_z={:.3f}, old_remaining={:.3f}, old_activity_reason={}, old_speed_3d={:.3f}, old_disp_3d={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               decision.reason,
                               last_tracking_diag_guide_path_size_,
                               last_tracking_diag_sfc_size_,
                               last_tracking_diag_target_prediction_size_,
                               last_tracking_diag_out_traj_duration_,
                               candidate.getTotalDuration(),
                               prefix_duration,
                               runtime_eval_start,
                               candidate_metrics.displacement_xy,
                               candidate_metrics.displacement_z,
                               candidate_metrics.displacement_3d,
                               candidate_metrics.speed_xy,
                               candidate_metrics.speed_z,
                               candidate_metrics.speed_3d,
                               candidate_metrics.target_speed_z,
                               decision.old_activity.remaining,
                               decision.old_activity.reason,
                               decision.old_activity.speed_3d,
                               decision.old_activity.displacement_3d,
                               decision.old_activity.progress_3d,
                               decision.old_activity.expected_progress,
                               decision.old_activity.avg_tracking_error);
            }
            if (!decision.candidate_safe) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_UNSAFE reason={}, candidate_safe_reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, candidate_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, old_remaining={:.3f}, old_activity_reason={}",
                               decision.reason,
                               candidate_safe_reason,
                               last_tracking_diag_guide_path_size_,
                               last_tracking_diag_sfc_size_,
                               last_tracking_diag_target_prediction_size_,
                               last_tracking_diag_out_traj_duration_,
                               candidate.getTotalDuration(),
                               prefix_duration,
                               runtime_eval_start,
                               candidate_metrics.displacement_xy,
                               candidate_metrics.displacement_z,
                               candidate_metrics.displacement_3d,
                               decision.old_activity.remaining,
                               decision.old_activity.reason);
            }
            if (!anti_rollback_pass &&
                decision.type == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_ANTI_ROLLBACK reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_fov_ok={}, worse_count={}, max_regression={:.3f}, candidate_disp_3d={:.3f}, candidate_progress_3d={:.3f}, old_remaining={:.3f}, old_activity_reason={}, old_speed_3d={:.3f}, old_disp_3d={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               decision.reason,
                               last_tracking_diag_guide_path_size_,
                               last_tracking_diag_sfc_size_,
                               last_tracking_diag_target_prediction_size_,
                               last_tracking_diag_out_traj_duration_,
                               prefix_duration,
                               runtime_eval_start,
                               candidate_fov_ok,
                               anti_rollback_worse_count,
                               anti_rollback_max_regression,
                               candidate_metrics.displacement_3d,
                               candidate_metrics.progress_3d,
                               decision.old_activity.remaining,
                               decision.old_activity.reason,
                               decision.old_activity.speed_3d,
                               decision.old_activity.displacement_3d,
                               decision.old_activity.progress_3d,
                               decision.old_activity.expected_progress,
                               decision.old_activity.avg_tracking_error);
            }
        };

        auto applyRuntimeDecision =
                [&](const Trajectory &candidate,
                    const std::string &tag,
                    const double candidate_eval_start_t,
                    const double target_eval_start_t,
                    const bool candidate_fov_ok) -> TrackingRuntimeManager::DecisionType {
            if (!cfg_.tracking_runtime_manager_enable || !tracking_runtime_manager_) {
                int worse_count = 0;
                double max_regression = 0.0;
                std::string anti_reason;
                const bool pass =
                        trackingCommitPassesAntiRollback(candidate,
                                                         target_prediction,
                                                         commit_wt,
                                                         candidate_eval_start_t,
                                                         target_eval_start_t,
                                                         true,
                                                         candidate_fov_ok,
                                                         &worse_count,
                                                         &max_regression,
                                                         &anti_reason);
                if (!pass) {
                    setTrackingCommitRejectInfo(
                            anti_reason.empty() ? "anti-rollback rejected candidate" : anti_reason,
                            fmt::format(
                                    "decision=REJECT_AND_FAIL|failure=anti_rollback|anti_rollback_pass=0|anti_rollback_reason={}|worse_count={}|max_regression={:.3f}|candidate_eval_start_t={:.3f}|target_eval_start_t={:.3f}|candidate_duration={:.3f}",
                                    anti_reason.empty() ? "none" : anti_reason,
                                    worse_count,
                                    max_regression,
                                    candidate_eval_start_t,
                                    target_eval_start_t,
                                    candidate.getTotalDuration()));
                }
                return pass ? TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE
                            : TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
            }

            const bool has_old_tracking =
                    allow_old_prefix &&
                    old_tracking_active_for_prefix;
            const double old_local_t =
                    has_old_tracking
                        ? std::clamp(commit_wt - old_start_wt, 0.0, old_total_dur)
                        : 0.0;
            std::string candidate_safe_reason;
            std::string candidate_safe_detail;
            const double candidate_total = candidate.getTotalDuration();
            const double candidate_safety_start =
                    std::clamp(candidate_eval_start_t, 0.0, candidate_total);
            const double candidate_safety_horizon =
                    std::min(cfg_.tracking_keep_old_horizon,
                             std::max(0.0, candidate_total - candidate_safety_start));
            const bool candidate_safe =
                    trackingTrajectorySafeForHorizonDetailed(
                            candidate,
                            candidate_safety_start,
                            candidate_safety_horizon,
                            cfg_.tracking_keep_old_safety_dt,
                            &candidate_safe_reason,
                            &candidate_safe_detail);
            int anti_rollback_worse_count = 0;
            double anti_rollback_max_regression = 0.0;
            std::string anti_rollback_reason;
            const bool anti_rollback_pass =
                    has_old_tracking
                        ? trackingCommitPassesAntiRollback(candidate,
                                                           target_prediction,
                                                           commit_wt,
                                                           candidate_eval_start_t,
                                                           target_eval_start_t,
                                                           candidate_safe,
                                                           candidate_fov_ok,
                                                           &anti_rollback_worse_count,
                                                           &anti_rollback_max_regression,
                                                           &anti_rollback_reason)
                        : true;
            auto decision =
                    tracking_runtime_manager_->decide(has_old_tracking ? &old_pos_traj : nullptr,
                                                      old_local_t,
                                                      candidate,
                                                      target_prediction,
                                                      candidate_safe,
                                                      anti_rollback_pass,
                                                      candidate_eval_start_t,
                                                      target_eval_start_t);
            if (!candidate_safe && !candidate_safe_reason.empty()) {
                decision.reason = decision.reason.empty()
                                      ? candidate_safe_reason
                                      : decision.reason + ": " + candidate_safe_reason;
            }
            if (!anti_rollback_pass && !anti_rollback_reason.empty()) {
                decision.reason = decision.reason.empty()
                                      ? anti_rollback_reason
                                      : decision.reason + ": " + anti_rollback_reason;
            }
            const std::string runtime_decision_detail = fmt::format(
                    "tag={}|decision={}|has_old_tracking={}|candidate_safe={}|candidate_commandable={}|candidate_fov_ok={}|bypass_anti_rollback={}|candidate_safe_reason={}|candidate_safe_detail={}|anti_rollback_pass={}|anti_rollback_reason={}|anti_rollback_worse_count={}|anti_rollback_max_regression={:.3f}|candidate_eval_start_t={:.3f}|target_eval_start_t={:.3f}|candidate_duration={:.3f}|candidate_safety_start={:.3f}|candidate_safety_horizon={:.3f}|old_local_t={:.3f}",
                    tag,
                    decisionTypeName(decision.type),
                    static_cast<int>(has_old_tracking),
                    static_cast<int>(candidate_safe),
                    static_cast<int>(decision.candidate_commandable),
                    static_cast<int>(candidate_fov_ok),
                    static_cast<int>(decision.bypass_anti_rollback),
                    candidate_safe_reason.empty() ? "none" : candidate_safe_reason,
                    candidate_safe_detail.empty() ? "none" : candidate_safe_detail,
                    static_cast<int>(anti_rollback_pass),
                    anti_rollback_reason.empty() ? "none" : anti_rollback_reason,
                    anti_rollback_worse_count,
                    anti_rollback_max_regression,
                    candidate_eval_start_t,
                    target_eval_start_t,
                    candidate.getTotalDuration(),
                    candidate_safety_start,
                    candidate_safety_horizon,
                    old_local_t);
            logRuntimeDecision(decision,
                               candidate,
                               anti_rollback_pass,
                               candidate_fov_ok,
                               candidate_eval_start_t,
                               candidate_eval_start_t,
                               anti_rollback_worse_count,
                               anti_rollback_max_regression,
                               candidate_safe_reason,
                               tag);

            switch (decision.type) {
                case TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE:
                case TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE:
                    return decision.type;
                case TrackingRuntimeManager::DecisionType::KEEP_OLD:
                    if (keepOldFromSnapshot(decision.reason)) {
                        tracking_runtime_manager_->onKeepOld();
                        return TrackingRuntimeManager::DecisionType::KEEP_OLD;
                    }
                    if (decision.candidate_safe && decision.candidate_commandable) {
                        if (cfg_.print_log) {
                            ros_ptr_->warn(" -- [Tracking] TRACKING_FORCE_COMMIT_SAFE_RECOVERY reason=old_tracking_not_legally_keepable, original_decision={}, original_reason={}, prefix_duration={:.3f}, runtime_eval_start={:.3f}",
                                           decisionTypeName(decision.type),
                                           decision.reason,
                                           candidate_eval_start_t,
                                           candidate_eval_start_t);
                        }
                        return TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE;
                    }
                    setTrackingCommitRejectInfo(decision.reason, runtime_decision_detail);
                    tracking_runtime_manager_->onRejected();
                    return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
                case TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL:
                    setTrackingCommitRejectInfo(decision.reason, runtime_decision_detail);
                    tracking_runtime_manager_->onRejected();
                    return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
            }
            setTrackingCommitRejectInfo("runtime decision fell through", runtime_decision_detail);
            tracking_runtime_manager_->onRejected();
            return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
        };

        Trajectory yaw_traj = optimized_yaw_traj;
        if (yaw_traj.empty() && !buildTrackingTargetYawTrajectory(pos_traj, target_prediction, yaw_traj)) {
            setTrackingCommitRejectInfo(
                    "yaw generation failed",
                    fmt::format("failure=yaw_generation_failed|candidate_duration={:.3f}|target_prediction_size={}",
                                pos_traj.getTotalDuration(),
                                target_prediction.size()));
            ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_FOV reason=yaw_generation_failed");
            if (keepOldFromSnapshot("tracking yaw generation failed")) {
                if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                    tracking_runtime_manager_->onKeepOld();
                }
                return true;
            }
            if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                tracking_runtime_manager_->onRejected();
            }
            return false;
        }

        Trajectory committed_pos_traj = pos_traj;
        Trajectory committed_yaw_traj = yaw_traj;
        committed_pos_traj.start_WT = commit_wt;
        committed_yaw_traj.start_WT = commit_wt;
        double stitched_prefix_duration = 0.0;
        if (should_stitch_old_prefix) {
            const bool fixed_head_time_valid =
                    std::isfinite(candidate_head_wt) &&
                    candidate_head_wt > commit_wt + 1.0e-4;
            const double prefix_end_wt =
                    fixed_head_time_valid
                        ? candidate_head_wt
                        : commit_wt + std::max(0.0, cfg_.replan_forward_dt);
            const double prefix_start_t = commit_wt - old_start_wt;
            const double prefix_end_t = prefix_end_wt - old_start_wt;
            const bool prefix_window_valid =
                    !old_pos_traj.empty() &&
                    !old_yaw_traj.empty() &&
                    fixed_head_time_valid &&
                    prefix_start_t >= 0.0 &&
                    prefix_end_t > prefix_start_t + 1.0e-4 &&
                    prefix_end_t <= old_total_dur + 1.0e-6;

            if (prefix_window_valid) {
                Trajectory prefix_pos_traj;
                Trajectory prefix_yaw_traj;
                const double clipped_prefix_end_t = std::min(prefix_end_t, old_total_dur);
                const double prefix_duration = clipped_prefix_end_t - prefix_start_t;
                bool used_sampled_yaw_prefix = false;
                const bool prefix_pos_ok =
                        old_pos_traj.getPartialTrajectoryByTime(prefix_start_t,
                                                                clipped_prefix_end_t,
                                                                prefix_pos_traj);
                const bool prefix_yaw_ok =
                        prefix_pos_ok &&
                        extractYawPrefixForStitching(old_yaw_traj,
                                                     prefix_start_t,
                                                     prefix_duration,
                                                     prefix_yaw_traj,
                                                     used_sampled_yaw_prefix);
                if (prefix_pos_ok && prefix_yaw_ok) {
                    committed_pos_traj = prefix_pos_traj + pos_traj;
                    committed_yaw_traj = prefix_yaw_traj + yaw_traj;
                    stitched_prefix_duration = prefix_pos_traj.getTotalDuration();
                    committed_pos_traj.start_WT = commit_wt;
                    committed_yaw_traj.start_WT = commit_wt;

                    if (cfg_.print_log) {
                        const StatePVAJ old_tail = prefix_pos_traj.getState(prefix_pos_traj.getTotalDuration());
                        const StatePVAJ new_head = pos_traj.getState(0.0);
                        const double pos_jump = (old_tail.col(0) - new_head.col(0)).norm();
                        const double vel_jump = (old_tail.col(1) - new_head.col(1)).norm();
                        const double acc_jump = (old_tail.col(2) - new_head.col(2)).norm();
                        const double jerk_jump = (old_tail.col(3) - new_head.col(3)).norm();
                        ros_ptr_->info(" -- [GeneralPlanner] Tracking replan stitched old prefix: dt={:.3f}s, start_t={:.3f}, end_t={:.3f}, candidate_head_wt={:.3f}, commit_wt={:.3f}, pos_jump={:.4f}, vel_jump={:.4f}, acc_jump={:.4f}, jerk_jump={:.4f}, sampled_yaw_prefix={}.",
                                       prefix_pos_traj.getTotalDuration(),
                                       prefix_start_t,
                                       clipped_prefix_end_t,
                                       candidate_head_wt,
                                       commit_wt,
                                       pos_jump,
                                       vel_jump,
                                       acc_jump,
                                       jerk_jump,
                                       used_sampled_yaw_prefix);
                    }
                } else {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [GeneralPlanner] Tracking replan prefix extraction failed: pos_ok={}, yaw_ok={}, start_t={:.3f}, end_t={:.3f}, candidate_head_wt={:.3f}, commit_wt={:.3f}.",
                                       prefix_pos_ok,
                                       prefix_yaw_ok,
                                       prefix_start_t,
                                       clipped_prefix_end_t,
                                       candidate_head_wt,
                                       commit_wt);
                    }
                    if (keepOldFromSnapshot("tracking replan prefix extraction failed")) {
                        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                            tracking_runtime_manager_->onKeepOld();
                        }
                        return true;
                    }
                    setTrackingCommitRejectInfo(
                            "tracking replan prefix extraction failed",
                            fmt::format("failure=prefix_extract_failed|pos_ok={}|yaw_ok={}|prefix_start_t={:.3f}|prefix_end_t={:.3f}|candidate_head_wt={:.3f}|commit_wt={:.3f}",
                                        static_cast<int>(prefix_pos_ok),
                                        static_cast<int>(prefix_yaw_ok),
                                        prefix_start_t,
                                        clipped_prefix_end_t,
                                        candidate_head_wt,
                                        commit_wt));
                    return false;
                }
            } else {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [GeneralPlanner] Tracking replan prefix unavailable: start_t={:.3f}, end_t={:.3f}, total={:.3f}, fixed_head_time_valid={}, candidate_head_wt={:.3f}, commit_wt={:.3f}.",
                                   prefix_start_t,
                                   prefix_end_t,
                                   old_total_dur,
                                   fixed_head_time_valid,
                                   candidate_head_wt,
                                   commit_wt);
                }
                if (keepOldFromSnapshot(fixed_head_time_valid
                                            ? "tracking replan prefix unavailable"
                                            : "tracking replan head time stale")) {
                    if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                        tracking_runtime_manager_->onKeepOld();
                    }
                    return true;
                }
                setTrackingCommitRejectInfo(
                        fixed_head_time_valid
                            ? "tracking replan prefix unavailable"
                            : "tracking replan head time stale",
                        fmt::format("failure=prefix_unavailable|prefix_start_t={:.3f}|prefix_end_t={:.3f}|old_total_dur={:.3f}|fixed_head_time_valid={}|candidate_head_wt={:.3f}|commit_wt={:.3f}",
                                    prefix_start_t,
                                    prefix_end_t,
                                    old_total_dur,
                                    static_cast<int>(fixed_head_time_valid),
                                    candidate_head_wt,
                                    commit_wt));
                return false;
            }
        }
        committed_pos_traj.start_WT = commit_wt;
        committed_yaw_traj.start_WT = commit_wt;

        const bool should_check_fov =
                cfg_.tracking_fov_commit_check_enable &&
                cfg_.tracking_fov_check_strict &&
                (cfg_.tracking_fov_check_first_commit || runtime_has_committed_tracking);
        bool candidate_fov_ok_for_commit = true;
        if (should_check_fov) {
            std::string fov_reject_reason;
            const double fov_start_t = 0.0;
            const double fov_horizon =
                    std::min({std::max(0.0, cfg_.tracking_keep_old_horizon),
                              committed_pos_traj.getTotalDuration(),
                              target_prediction.empty() ? 0.0
                                                        : std::max(0.0, target_prediction.back().t)});
            if (!trackingTrajectorySatisfiesFov(committed_pos_traj,
                                                committed_yaw_traj,
                                                target_prediction,
                                                fov_start_t,
                                                fov_horizon,
                                                cfg_.tracking_fov_check_dt,
                                                0.0,
                                                &fov_reject_reason,
                                                false,
                                                allow_reacquire_fov_relax)) {
                Trajectory target_yaw_traj;
                std::string rebuilt_yaw_fov_reason;
                const bool rebuilt_yaw_generated =
                        buildTrackingTargetYawTrajectory(committed_pos_traj,
                                                         target_prediction,
                                                         target_yaw_traj);
                const bool rebuilt_yaw_ok =
                        rebuilt_yaw_generated &&
                        trackingTrajectorySatisfiesFov(committed_pos_traj,
                                                       target_yaw_traj,
                                                       target_prediction,
                                                       fov_start_t,
                                                       fov_horizon,
                                                       cfg_.tracking_fov_check_dt,
                                                       0.0,
                                                       &rebuilt_yaw_fov_reason,
                                                       false,
                                                       allow_reacquire_fov_relax);
                if (rebuilt_yaw_ok) {
                    committed_yaw_traj = target_yaw_traj;
                    committed_yaw_traj.start_WT = commit_wt;
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_YAW_REBUILT_FOR_FOV reason={}, horizon={:.3f}, reacquire_fov_relax={}",
                                       fov_reject_reason,
                                       fov_horizon,
                                       allow_reacquire_fov_relax);
                    }
                } else if (allow_reacquire_fov_relax) {
                    candidate_fov_ok_for_commit = false;
                    if (rebuilt_yaw_generated && !target_yaw_traj.empty()) {
                        committed_yaw_traj = target_yaw_traj;
                        committed_yaw_traj.start_WT = commit_wt;
                    }
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_FOV_DEGRADED_ACCEPT reason={}, target_yaw_fallback={}, start_t={:.3f}, horizon={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}",
                                       fov_reject_reason,
                                       rebuilt_yaw_generated
                                           ? (rebuilt_yaw_fov_reason.empty()
                                                  ? "failed_without_reason"
                                                  : rebuilt_yaw_fov_reason)
                                           : "generation_failed",
                                       fov_start_t,
                                       fov_horizon,
                                       stitched_prefix_duration,
                                       stitched_prefix_duration,
                                       last_tracking_diag_guide_path_size_,
                                       last_tracking_diag_sfc_size_,
                                       last_tracking_diag_target_prediction_size_,
                                       last_tracking_diag_out_traj_duration_);
                    }
                } else {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_FOV reason={}, start_t={:.3f}, horizon={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, reacquire_fov_relax={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}",
                                       fov_reject_reason,
                                       fov_start_t,
                                       fov_horizon,
                                       stitched_prefix_duration,
                                       stitched_prefix_duration,
                                       allow_reacquire_fov_relax,
                                       last_tracking_diag_guide_path_size_,
                                       last_tracking_diag_sfc_size_,
                                       last_tracking_diag_target_prediction_size_,
                                       last_tracking_diag_out_traj_duration_);
                    }
                    setTrackingCommitRejectInfo(
                            "candidate FOV rejected: " + fov_reject_reason +
                            "; target_yaw_fallback=" +
                            (rebuilt_yaw_fov_reason.empty() ? "failed" : rebuilt_yaw_fov_reason),
                            fmt::format(
                                    "failure=fov_rejected|fov_start_t={:.3f}|fov_horizon={:.3f}|prefix_duration={:.3f}|reacquire_fov_relax={}|candidate_duration={:.3f}|target_prediction_size={}|target_yaw_fallback_reason={}",
                                    fov_start_t,
                                    fov_horizon,
                                    stitched_prefix_duration,
                                    static_cast<int>(allow_reacquire_fov_relax),
                                    committed_pos_traj.getTotalDuration(),
                                    target_prediction.size(),
                                    rebuilt_yaw_fov_reason.empty() ? "failed" : rebuilt_yaw_fov_reason));
                    if (keepOldFromSnapshot("tracking candidate rejected by FOV: " + fov_reject_reason)) {
                        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                            tracking_runtime_manager_->onKeepOld();
                        }
                        return true;
                    }
                    if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                        tracking_runtime_manager_->onRejected();
                    }
                    return false;
                }
            }
        }

        const auto runtime_decision =
                applyRuntimeDecision(committed_pos_traj,
                                     "final_commit",
                                     cfg_.tracking_anti_rollback_eval_after_prefix
                                         ? stitched_prefix_duration
                                         : 0.0,
                                     cfg_.tracking_anti_rollback_eval_after_prefix
                                         ? stitched_prefix_duration
                                         : 0.0,
                                     candidate_fov_ok_for_commit);
        if (runtime_decision == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
            return true;
        }
        if (runtime_decision == TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL) {
            return false;
        }

        ExpTraj task_exp_traj;
        task_exp_traj.setGoalConnectedFlag(true);
        task_exp_traj.setWholeTrajKnownFreeFlag(true);
        task_exp_traj.setTrajectory(commit_wt, committed_pos_traj, committed_yaw_traj);

        cmd_traj_info_.setTrajectory(task_exp_traj);
        last_exp_traj_info_ = task_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        {
            TimeConsuming t_viz("tracking_task_viz", false);
            ros_ptr_->vizExpTraj(committed_pos_traj, traj_ns);
            ros_ptr_->vizYawTraj(committed_pos_traj, committed_yaw_traj);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        latest_replan.setExpTraj(committed_pos_traj);
        latest_replan.setExpYawTraj(committed_yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            tracking_runtime_manager_->onCommitted();
        }
        resetTrackingCommitCounters();

        if (cfg_.print_log) {
            const double guard_h =
                    std::min(cfg_.tracking_no_motion_check_horizon,
                             std::max(0.0, committed_pos_traj.getTotalDuration() -
                                            stitched_prefix_duration));
            const TrackingMotionMetrics committed_metrics =
                    computeTrackingMotionMetrics(committed_pos_traj,
                                                 target_prediction,
                                                 cfg_,
                                                 stitched_prefix_duration,
                                                 stitched_prefix_duration,
                                                 guard_h);
            const double now_minus_start_wt = ros_ptr_->getSimTime() - committed_pos_traj.start_WT;
            ros_ptr_->info(" -- [Tracking] TRACKING_CANDIDATE_COMMITTED candidate_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_head_wt={:.3f}, commit_wt={:.3f}, head_lag={:.3f}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, target_speed_z={:.3f}, now_minus_start_WT={:.3f}, committed_total_duration={:.3f}",
                           committed_pos_traj.getTotalDuration(),
                           stitched_prefix_duration,
                           stitched_prefix_duration,
                           candidate_head_wt,
                           commit_wt,
                           candidate_head_wt - commit_wt,
                           committed_metrics.displacement_xy,
                           committed_metrics.displacement_z,
                           committed_metrics.displacement_3d,
                           committed_metrics.speed_xy,
                           committed_metrics.speed_z,
                           committed_metrics.speed_3d,
                           committed_metrics.target_speed_z,
                           now_minus_start_wt,
                           committed_pos_traj.getTotalDuration());

            const double short_h = std::min(0.15, committed_pos_traj.getTotalDuration());
            const TrackingMotionMetrics short_metrics =
                    computeTrackingMotionMetrics(committed_pos_traj,
                                                 target_prediction,
                                                 cfg_,
                                                 stitched_prefix_duration,
                                                 stitched_prefix_duration,
                                                 short_h);
            if (short_metrics.target_moving &&
                short_metrics.displacement_3d < cfg_.tracking_no_motion_min_displacement &&
                short_metrics.displacement_z < cfg_.tracking_no_motion_min_displacement_z &&
                short_metrics.speed_3d < cfg_.tracking_keep_old_min_speed) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_COMMITTED_BUT_NO_MOTION start_WT={:.3f}, now={:.3f}, duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, now_minus_start_WT={:.3f}, speed_3d={:.3f}, displacement_3d={:.3f}, displacement_z={:.3f}",
                               committed_pos_traj.start_WT,
                               ros_ptr_->getSimTime(),
                               committed_pos_traj.getTotalDuration(),
                               stitched_prefix_duration,
                               stitched_prefix_duration,
                               now_minus_start_wt,
                               short_metrics.speed_3d,
                               short_metrics.displacement_3d,
                               short_metrics.displacement_z);
            }
            if (std::abs(now_minus_start_wt) > cfg_.tracking_commit_start_time_tolerance) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_COMMIT_START_TIME_OFFSET now_minus_start_WT={:.3f}, tolerance={:.3f}, start_WT={:.3f}, now={:.3f}",
                               now_minus_start_wt,
                               cfg_.tracking_commit_start_time_tolerance,
                               committed_pos_traj.start_WT,
                               ros_ptr_->getSimTime());
            }
        }
        return true;
    }

}
