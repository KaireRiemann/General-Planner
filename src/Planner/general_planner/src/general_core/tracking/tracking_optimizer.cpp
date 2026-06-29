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
    namespace {
        void setFailureReason(std::string *out, const std::string &reason) {
            if (out != nullptr) {
                *out = reason;
            }
        }

        void appendGuideTimedUnique(const Vec3f &point,
                                    const double stamp,
                                    vec_Vec3f &path,
                                    std::vector<double> &path_t) {
            if (!point.allFinite() || !std::isfinite(stamp)) {
                return;
            }
            if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
                path.emplace_back(point);
                path_t.emplace_back(stamp);
            } else if (!path_t.empty()) {
                path_t.back() = stamp;
            }
        }

        int validVisibleRegionCount(const traj_opt::TrackingProblem &problem) {
            return static_cast<int>(std::count_if(problem.visible_regions.begin(),
                                                  problem.visible_regions.end(),
                                                  [](const traj_opt::TrackingVisibleRegion &region) {
                                                      return region.valid;
                                                  }));
        }

        bool staticTargetPrediction(const traj_opt::DynamicTargetStates &prediction,
                                    const double position_epsilon,
                                    const double velocity_epsilon) {
            if (prediction.empty()) {
                return false;
            }
            const Vec3f ref = prediction.front().position;
            double max_span = 0.0;
            double max_vel = 0.0;
            for (const auto &state : prediction) {
                max_span = std::max(max_span, (state.position - ref).norm());
                max_vel = std::max(max_vel, state.velocity.norm());
            }
            return max_span <= std::max(0.0, position_epsilon) &&
                   max_vel <= std::max(0.0, velocity_epsilon);
        }

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

        bool trackingSoftRecoveryRequired(const Config &cfg,
                                          const Vec3f &tracker,
                                          const Vec3f &target) {
            if (!tracker.allFinite() || !target.allFinite()) {
                return false;
            }
            return (tracker - target).head<2>().norm() >
                   trackingSoftRecoveryEntryDistance(cfg);
        }

    }

    bool GeneralPlanner::optimizeTrackingProblemWithRetries(
            const traj_opt::TrackingProblem &normal_problem,
            const traj_opt::DynamicTargetStates &active_target_prediction,
            Trajectory &out_traj,
            Trajectory &out_yaw_traj,
            traj_opt::DynamicTargetStates *accepted_target_prediction,
            bool *accepted_reacquire_fov_relax,
            std::string *failure_reason) {
        out_traj = Trajectory();
        out_yaw_traj = Trajectory();
        if (accepted_target_prediction != nullptr) {
            *accepted_target_prediction = active_target_prediction;
        }
        if (accepted_reacquire_fov_relax != nullptr) {
            *accepted_reacquire_fov_relax = false;
        }
        std::vector<std::string> attempt_failures;

        auto shouldFovPostcheckOptimization = [&]() -> bool {
            if (!cfg_.tracking_adaptive_occlusion_postcheck_enable ||
                !cfg_.tracking_fov_commit_check_enable ||
                !cfg_.tracking_fov_check_strict) {
                return false;
            }
            const bool has_committed_tracking =
                    cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_
                        ? tracking_runtime_manager_->hasCommittedTracking()
                        : !cmd_traj_info_.empty();
            return cfg_.tracking_fov_check_first_commit || has_committed_tracking;
        };

        auto candidatePassesOptimizationFovPostcheck =
                [&](const Trajectory &candidate_pos,
                    Trajectory &candidate_yaw,
                    const traj_opt::DynamicTargetStates &check_target_prediction,
                    const bool allow_reacquire_fov_relax,
                    std::string *reject_reason) -> bool {
            if (!shouldFovPostcheckOptimization()) {
                return true;
            }
            if (candidate_pos.empty() || check_target_prediction.empty()) {
                setFailureReason(reject_reason, "empty candidate or target prediction");
                return false;
            }
            if (candidate_yaw.empty() &&
                !buildTrackingTargetYawTrajectory(candidate_pos,
                                                  check_target_prediction,
                                                  candidate_yaw)) {
                setFailureReason(reject_reason, "yaw generation failed for FOV postcheck");
                return false;
            }
            const double fov_horizon =
                    std::min({std::max(0.0, cfg_.tracking_keep_old_horizon),
                              candidate_pos.getTotalDuration(),
                              check_target_prediction.empty()
                                  ? 0.0
                                  : std::max(0.0, check_target_prediction.back().t)});
            std::string optimized_yaw_reason;
            if (trackingTrajectorySatisfiesFov(candidate_pos,
                                               candidate_yaw,
                                               check_target_prediction,
                                               0.0,
                                               fov_horizon,
                                               cfg_.tracking_fov_check_dt,
                                               0.0,
                                               &optimized_yaw_reason,
                                               false,
                                               allow_reacquire_fov_relax)) {
                return true;
            }

            Trajectory target_yaw_traj;
            std::string target_yaw_reason;
            const bool target_yaw_generated =
                    buildTrackingTargetYawTrajectory(candidate_pos,
                                                     check_target_prediction,
                                                     target_yaw_traj);
            if (target_yaw_generated &&
                trackingTrajectorySatisfiesFov(candidate_pos,
                                               target_yaw_traj,
                                               check_target_prediction,
                                               0.0,
                                               fov_horizon,
                                               cfg_.tracking_fov_check_dt,
                                               0.0,
                                               &target_yaw_reason,
                                               false,
                                               allow_reacquire_fov_relax)) {
                candidate_yaw = target_yaw_traj;
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_YAW_REBUILT_FOR_FOV reason={}, horizon={:.3f}, reacquire_fov_relax={}",
                                   optimized_yaw_reason,
                                   fov_horizon,
                                   allow_reacquire_fov_relax);
                }
                return true;
            }

            if (allow_reacquire_fov_relax) {
                if (target_yaw_generated && !target_yaw_traj.empty()) {
                    candidate_yaw = target_yaw_traj;
                }
                const std::string fallback_reason =
                        target_yaw_generated
                            ? (target_yaw_reason.empty()
                                   ? "failed_without_reason"
                                   : target_yaw_reason)
                            : "generation_failed";
                setFailureReason(reject_reason,
                                 optimized_yaw_reason +
                                 "; target_yaw_fallback=" +
                                 fallback_reason +
                                 "; degraded_fov_accepted=1");
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_FOV_DEGRADED_ACCEPT reason={}, target_yaw_fallback={}, horizon={:.3f}, reacquire_fov_relax={}",
                                   optimized_yaw_reason,
                                   fallback_reason,
                                   fov_horizon,
                                   allow_reacquire_fov_relax);
                }
                return true;
            }

            setFailureReason(reject_reason,
                             optimized_yaw_reason +
                             "; target_yaw_fallback=" +
                             (target_yaw_generated
                                  ? (target_yaw_reason.empty()
                                         ? "failed_without_reason"
                                         : target_yaw_reason)
                                  : "generation_failed"));
            return false;
        };

        auto hasCommittedTrackingSnapshot = [&]() -> bool {
            if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                return tracking_runtime_manager_->hasCommittedTracking() && !cmd_traj_info_.empty();
            }
            return !cmd_traj_info_.empty();
        };

        auto candidatePassesOptimizationCommitSafety =
                [&](const Trajectory &candidate_pos,
                    std::string *reject_reason) -> bool {
            if (!cfg_.tracking_optimizer_commit_safety_precheck_enable) {
                return true;
            }
            if (candidate_pos.empty()) {
                setFailureReason(reject_reason, "empty candidate trajectory");
                return false;
            }

            Trajectory committed_candidate = candidate_pos;
            const double commit_wt = ros_ptr_->getSimTime();
            double stitched_prefix_duration = 0.0;
            if (!cmd_traj_info_.empty()) {
                cmd_traj_info_.lock();
                const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
                const double old_start_wt = cmd_traj_info_.getStartWallTime();
                const double old_total_dur = cmd_traj_info_.getTotalDuration();
                cmd_traj_info_.unlock();

                const double prefix_start_t = commit_wt - old_start_wt;
                const double prefix_end_t =
                        prefix_start_t + std::max(0.0, cfg_.replan_forward_dt);
                const bool prefix_window_valid =
                        !old_pos_traj.empty() &&
                        prefix_start_t >= 0.0 &&
                        prefix_end_t > prefix_start_t + 1.0e-4 &&
                        prefix_end_t <= old_total_dur + 1.0e-6;
                if (prefix_window_valid) {
                    Trajectory prefix_pos_traj;
                    if (old_pos_traj.getPartialTrajectoryByTime(prefix_start_t,
                                                                std::min(prefix_end_t, old_total_dur),
                                                                prefix_pos_traj)) {
                        committed_candidate = prefix_pos_traj + candidate_pos;
                        stitched_prefix_duration = prefix_pos_traj.getTotalDuration();
                        committed_candidate.start_WT = commit_wt;
                    }
                }
            }

            const double safety_start =
                    cfg_.tracking_anti_rollback_eval_after_prefix
                        ? stitched_prefix_duration
                        : 0.0;
            const double safety_horizon =
                    std::min(std::max(0.0, cfg_.tracking_keep_old_horizon),
                             std::max(0.0,
                                      committed_candidate.getTotalDuration() -
                                      std::clamp(safety_start,
                                                 0.0,
                                                 committed_candidate.getTotalDuration())));
            std::string safety_reason;
            std::string safety_detail;
            if (!trackingTrajectorySafeForHorizonDetailed(committed_candidate,
                                                          safety_start,
                                                          safety_horizon,
                                                          cfg_.tracking_keep_old_safety_dt,
                                                          &safety_reason,
                                                          &safety_detail)) {
                setFailureReason(reject_reason,
                                 fmt::format(
                                         "{}; commit_candidate_duration={:.3f}; stitched_prefix_duration={:.3f}; safety_start={:.3f}; safety_horizon={:.3f}; {}",
                                         safety_reason.empty() ? "commit safety precheck failed"
                                                               : safety_reason,
                                         committed_candidate.getTotalDuration(),
                                         stitched_prefix_duration,
                                         safety_start,
                                         safety_horizon,
                                         safety_detail.empty() ? "detail=none"
                                                               : safety_detail));
                return false;
            }
            return true;
        };

        auto estimateReacquireTransitHorizon =
                [&](const traj_opt::TrackingProblem &problem) -> double {
            const double base_h =
                    std::max(0.3, cfg_.tracking_reacquire_recovery_horizon);
            if (problem.target_prediction.empty()) {
                return base_h;
            }
            const double initial_horizontal_dist =
                    (problem.head_pvaj.col(0) - problem.target_prediction.front().position)
                            .head<2>()
                            .norm();
            const double reacquire_band =
                    cfg_.tracking_soft_recovery_enable
                        ? trackingSoftRecoveryEntryDistance(cfg_)
                        : std::max(problem.tracking_distance + problem.distance_tolerance,
                                   cfg_.tracking_reacquire_distance);
            const double gap = std::max(0.0, initial_horizontal_dist - reacquire_band);
            if (gap <= 1.0e-6) {
                return base_h;
            }
            const double speed_cap =
                    std::max(0.8, cfg_.esdf_traj_cfg.max_vel);
            const double scaled_h =
                    gap * std::max(0.1, cfg_.tracking_reacquire_transit_horizon_scale) / speed_cap;
            return std::clamp(std::max(base_h, scaled_h),
                              base_h,
                              std::max(base_h, cfg_.tracking_reacquire_transit_max_horizon));
        };

        auto runOptimization = [&](const std::string &attempt_name,
                                   traj_opt::TrackingProblem problem,
                                   const bool recovery_attempt) -> bool {
            std::string narrow_reason;
            const bool narrow_adjusted = applyTrackingNarrowPassageSoftDistance(problem, &narrow_reason);
            if (cfg_.print_log && narrow_adjusted) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_NARROW_PASSAGE_SOFT_CLEARANCE attempt={}, reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}",
                               attempt_name,
                               narrow_reason,
                               problem.guide_path.size(),
                               problem.sfcs.size(),
                               problem.target_prediction.size());
            }

            Trajectory candidate_pos;
            Trajectory candidate_yaw;
            const bool ok = cfg_.tracking_use_snap
                                ? traj_manager_->trackingSnap()->optimize(problem, candidate_pos, &candidate_yaw)
                                : traj_manager_->trackingJerk()->optimize(problem, candidate_pos, &candidate_yaw);
            if (ok && !candidate_pos.empty()) {
                const traj_opt::DynamicTargetStates &check_target_prediction =
                        problem.target_prediction.empty()
                            ? active_target_prediction
                            : problem.target_prediction;
                const bool recovery_like_reacquire =
                        recovery_attempt &&
                        ((!check_target_prediction.empty() &&
                          trackingSoftRecoveryRequired(cfg_,
                                                       problem.head_pvaj.col(0),
                                                       check_target_prediction.front().position)) ||
                         !hasCommittedTrackingSnapshot());
                const bool allow_reacquire_fov_relax =
                        (problem.reacquire_mode || recovery_like_reacquire) &&
                        cfg_.tracking_reacquire_fov_relax_enable;
                std::string fov_postcheck_reason;
                if (!candidatePassesOptimizationFovPostcheck(candidate_pos,
                                                             candidate_yaw,
                                                             check_target_prediction,
                                                             allow_reacquire_fov_relax,
                                                             &fov_postcheck_reason)) {
                    attempt_failures.emplace_back(
                            fmt::format("{} rejected_by_fov_postcheck(reason={}, guide={}, sfc={}, target={}, duration={:.3f}, safe_distance={:.3f}, use_corridor={})",
                                        attempt_name,
                                        fov_postcheck_reason,
                                        problem.guide_path.size(),
                                        problem.sfcs.size(),
                                        problem.target_prediction.size(),
                                        candidate_pos.getTotalDuration(),
                                        problem.safe_distance,
                                        problem.use_corridor));
                    ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_RECOVERY_FAILED attempt={}, reason=fov_postcheck_failed:{}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, safe_distance={:.3f}, use_corridor={}, narrow_adjusted={}",
                                   attempt_name,
                                   fov_postcheck_reason,
                                   problem.guide_path.size(),
                                   problem.sfcs.size(),
                                   problem.target_prediction.size(),
                                   candidate_pos.getTotalDuration(),
                                   problem.safe_distance,
                                   problem.use_corridor,
                                   narrow_adjusted);
                    return false;
                }
                std::string commit_safety_precheck_reason;
                if (!candidatePassesOptimizationCommitSafety(candidate_pos,
                                                             &commit_safety_precheck_reason)) {
                    attempt_failures.emplace_back(
                            fmt::format("{} rejected_by_commit_safety_precheck(reason={}, guide={}, sfc={}, target={}, duration={:.3f}, safe_distance={:.3f}, use_corridor={})",
                                        attempt_name,
                                        commit_safety_precheck_reason,
                                        problem.guide_path.size(),
                                        problem.sfcs.size(),
                                        problem.target_prediction.size(),
                                        candidate_pos.getTotalDuration(),
                                        problem.safe_distance,
                                        problem.use_corridor));
                    if (recovery_attempt) {
                        ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_RECOVERY_FAILED attempt={}, reason=commit_safety_precheck_failed:{}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, safe_distance={:.3f}, use_corridor={}, narrow_adjusted={}",
                                       attempt_name,
                                       commit_safety_precheck_reason,
                                       problem.guide_path.size(),
                                       problem.sfcs.size(),
                                       problem.target_prediction.size(),
                                       candidate_pos.getTotalDuration(),
                                       problem.safe_distance,
                                       problem.use_corridor,
                                       narrow_adjusted);
                    }
                    return false;
                }
                out_traj = candidate_pos;
                out_yaw_traj = candidate_yaw;
                if (accepted_target_prediction != nullptr) {
                    *accepted_target_prediction = check_target_prediction;
                }
                if (accepted_reacquire_fov_relax != nullptr) {
                    *accepted_reacquire_fov_relax = allow_reacquire_fov_relax;
                }
                if (recovery_attempt) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_RECOVERY_SUCCESS attempt={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, safe_distance={:.3f}, use_corridor={}, narrow_adjusted={}",
                                   attempt_name,
                                   problem.guide_path.size(),
                                   problem.sfcs.size(),
                                   problem.target_prediction.size(),
                                   out_traj.getTotalDuration(),
                                   problem.safe_distance,
                                   problem.use_corridor,
                                   narrow_adjusted);
                }
                return true;
            }

            attempt_failures.emplace_back(
                    fmt::format("{} failed(ok={}, empty={}, guide={}, sfc={}, target={}, safe_distance={:.3f}, use_corridor={})",
                                attempt_name,
                                ok,
                                candidate_pos.empty(),
                                problem.guide_path.size(),
                                problem.sfcs.size(),
                                problem.target_prediction.size(),
                                problem.safe_distance,
                                problem.use_corridor));
            if (recovery_attempt) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_RECOVERY_FAILED attempt={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, safe_distance={:.3f}, use_corridor={}, narrow_adjusted={}",
                               attempt_name,
                               problem.guide_path.size(),
                               problem.sfcs.size(),
                               problem.target_prediction.size(),
                               candidate_pos.empty() ? 0.0 : candidate_pos.getTotalDuration(),
                               problem.safe_distance,
                               problem.use_corridor,
                               narrow_adjusted);
            }
            return false;
        };

        traj_opt::TrackingProblem normal = normal_problem;
        normal.safe_distance = std::max(0.0, normal.safe_distance);
        if (runOptimization("normal", normal, false)) {
            return true;
        }

        if (!cfg_.tracking_recovery_enable) {
            setFailureReason(failure_reason,
                             attempt_failures.empty() ? "normal optimization failed"
                                                      : attempt_failures.back());
            return false;
        }

        auto truncateRecoveryProblem = [&](traj_opt::TrackingProblem &problem,
                                           const double horizon) {
            const double h = std::max(0.3, horizon);
            if (!problem.target_prediction.empty() &&
                problem.target_prediction.back().t > h) {
                traj_opt::DynamicTargetStates truncated;
                for (const auto &state : problem.target_prediction) {
                    if (state.t <= h + 1.0e-6) {
                        truncated.emplace_back(state);
                    }
                }
                const auto horizon_state = interpolateTargetPrediction(problem.target_prediction, h);
                if (truncated.empty() || std::abs(truncated.back().t - h) > 1.0e-4) {
                    truncated.emplace_back(horizon_state);
                }
                problem.target_prediction = std::move(truncated);
            }

            if (problem.guide_path.size() >= 2 &&
                problem.guide_t.size() == problem.guide_path.size() &&
                problem.guide_t.back() > h) {
                vec_Vec3f guide;
                std::vector<double> guide_t;
                guide.reserve(problem.guide_path.size());
                guide_t.reserve(problem.guide_t.size());
                appendGuideTimedUnique(problem.guide_path.front(), problem.guide_t.front(), guide, guide_t);
                for (std::size_t i = 1; i < problem.guide_path.size(); ++i) {
                    if (problem.guide_t[i] <= h + 1.0e-6) {
                        appendGuideTimedUnique(problem.guide_path[i], problem.guide_t[i], guide, guide_t);
                        continue;
                    }
                    const double t0 = problem.guide_t[i - 1];
                    const double t1 = problem.guide_t[i];
                    if (h > t0 + 1.0e-6 && t1 > t0 + 1.0e-6) {
                        const double alpha = std::clamp((h - t0) / (t1 - t0), 0.0, 1.0);
                        const Vec3f p = problem.guide_path[i - 1] +
                                        alpha * (problem.guide_path[i] - problem.guide_path[i - 1]);
                        appendGuideTimedUnique(p, h, guide, guide_t);
                    }
                    break;
                }
                if (guide.size() >= 2 && guide.size() == guide_t.size()) {
                    problem.guide_path = std::move(guide);
                    problem.guide_t = std::move(guide_t);
                    refreshTrackingGuideEndpoint(problem);
                }
            }

            if (!problem.target_prediction.empty()) {
                problem.min_total_duration =
                        std::max(0.6,
                                 std::min(problem.min_total_duration,
                                          problem.target_prediction.back().t));
            }
        };

        traj_opt::TrackingProblem recovery = normal_problem;
        truncateRecoveryProblem(recovery, cfg_.tracking_recovery_horizon);
        recovery.distance_tolerance *= std::max(1.0, cfg_.tracking_recovery_distance_tolerance_scale);
        recovery.height_tolerance *= std::max(1.0, cfg_.tracking_recovery_height_tolerance_scale);
        recovery.od_h_lower =
                std::max(0.05,
                         recovery.tracking_distance - recovery.distance_tolerance);
        recovery.od_h_upper =
                std::max(recovery.od_h_lower + 0.05,
                         recovery.tracking_distance + recovery.distance_tolerance);
        recovery.od_v_lower = recovery.height_offset - recovery.height_tolerance;
        recovery.od_v_upper = recovery.height_offset + recovery.height_tolerance;
        recovery.min_total_duration =
                std::max(0.6,
                         recovery.min_total_duration *
                         std::max(1.0, cfg_.tracking_recovery_time_scale));
        if (!recovery.target_prediction.empty()) {
            recovery.min_total_duration =
                    std::max(recovery.min_total_duration,
                             recovery.target_prediction.back().t);
        }
        recovery.weight_visible_region *=
                std::clamp(cfg_.tracking_recovery_reduce_visible_region_weight, 0.0, 1.0);
        recovery.weight_target_forward *=
                std::clamp(cfg_.tracking_recovery_reduce_target_forward_weight, 0.0, 1.0);

        if (runOptimization("recovery_relaxed", recovery, true)) {
            return true;
        }

        auto applyAdaptiveOcclusionRecovery = [&](traj_opt::TrackingProblem &problem) {
            problem.adaptive_occlusion_enable = true;
            problem.weight_oe *=
                    std::max(1.0, cfg_.tracking_adaptive_occlusion_recovery_oe_scale);
            problem.weight_visibility = problem.weight_oe;
            problem.weight_od_far *=
                    std::max(1.0, cfg_.tracking_adaptive_occlusion_od_far_weight_scale);
            problem.visibility_samples =
                    std::max(problem.visibility_samples,
                             std::max(7, 2 * cfg_.tracking_visibility_samples - 1));
            if (problem.joint_sample_dt > 0.0) {
                problem.joint_sample_dt =
                        std::min(problem.joint_sample_dt,
                                 std::max(0.02, cfg_.tracking_fov_check_dt));
            }
            const double adaptive_upper =
                    std::max(problem.od_h_lower + 0.05,
                             std::max(problem.adaptive_occlusion_min_horizontal_upper,
                                      problem.tracking_distance *
                                      std::clamp(problem.adaptive_occlusion_distance_upper_scale,
                                                 0.1,
                                                 1.0)));
            problem.od_h_upper = std::min(problem.od_h_upper, adaptive_upper);
            problem.weight_viewpoint_attractor *= 0.65;
            problem.weight_visible_region *= 0.8;
        };

        traj_opt::TrackingProblem adaptive_recovery = recovery;
        if (cfg_.tracking_adaptive_occlusion_enable) {
            applyAdaptiveOcclusionRecovery(adaptive_recovery);
            if (runOptimization("recovery_adaptive_occlusion", adaptive_recovery, true)) {
                return true;
            }
        }

        traj_opt::TrackingProblem obstacle_recovery =
                cfg_.tracking_adaptive_occlusion_enable ? adaptive_recovery : recovery;
        obstacle_recovery.safe_distance =
                std::max(trackingHardSafeDistance(cfg_),
                         cfg_.tracking_safe_distance *
                         std::clamp(cfg_.tracking_narrow_passage_soft_safe_distance_scale, 0.1, 1.0));
        if (runOptimization("recovery_narrow_corridor", obstacle_recovery, true)) {
            return true;
        }

        if (cfg_.tracking_retry_without_corridor_enable && normal_problem.use_corridor) {
            traj_opt::TrackingProblem no_corridor = obstacle_recovery;
            no_corridor.use_corridor = false;
            no_corridor.sfcs.clear();
            if (runOptimization("recovery_without_corridor", no_corridor, true)) {
                return true;
            }
        }

        if (cfg_.tracking_reacquire_recovery_enable) {
            traj_opt::TrackingProblem reacquire_recovery = obstacle_recovery;
            reacquire_recovery.reacquire_mode = true;
            const double dynamic_reacquire_horizon =
                    estimateReacquireTransitHorizon(reacquire_recovery);
            truncateRecoveryProblem(reacquire_recovery, dynamic_reacquire_horizon);
            const double visible_scale =
                    std::clamp(cfg_.tracking_reacquire_visible_region_weight_scale, 0.0, 1.0);
            reacquire_recovery.weight_visible_region *= visible_scale;
            if (visible_scale <= 1.0e-6) {
                reacquire_recovery.use_visible_region = false;
            }
            if (runOptimization("recovery_reacquire_short", reacquire_recovery, true)) {
                return true;
            }

            if (cfg_.tracking_reacquire_transit_enable) {
                traj_opt::TrackingProblem reacquire_transit = reacquire_recovery;
                reacquire_transit.use_corridor = false;
                reacquire_transit.sfcs.clear();
                reacquire_transit.use_visible_region = false;
                reacquire_transit.weight_visible_region = 0.0;
                reacquire_transit.weight_target_forward *= 0.35;
                reacquire_transit.weight_viewpoint_attractor *= 1.35;
                reacquire_transit.distance_tolerance *= 1.35;
                reacquire_transit.height_tolerance *= 1.35;
                reacquire_transit.od_h_lower =
                        std::max(0.05,
                                 reacquire_transit.tracking_distance -
                                         reacquire_transit.distance_tolerance);
                reacquire_transit.od_h_upper =
                        std::max(reacquire_transit.od_h_lower + 0.05,
                                 reacquire_transit.tracking_distance +
                                         reacquire_transit.distance_tolerance);
                reacquire_transit.od_v_lower =
                        reacquire_transit.height_offset -
                        reacquire_transit.height_tolerance;
                reacquire_transit.od_v_upper =
                        reacquire_transit.height_offset +
                        reacquire_transit.height_tolerance;
                reacquire_transit.min_total_duration =
                        std::max(reacquire_transit.min_total_duration,
                                 dynamic_reacquire_horizon);
                if (runOptimization("recovery_reacquire_transit",
                                    reacquire_transit,
                                    true)) {
                    return true;
                }
            }
        }

        std::string joined;
        for (const std::string &failure : attempt_failures) {
            if (!joined.empty()) {
                joined += "; ";
            }
            joined += failure;
        }
        setFailureReason(failure_reason,
                         joined.empty() ? "all tracking optimization attempts failed" : joined);
        ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_FAILED reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, prefix_duration=0.000, runtime_eval_start=0.000",
                       failure_reason != nullptr ? *failure_reason : joined,
                       normal_problem.guide_path.size(),
                       normal_problem.sfcs.size(),
                       normal_problem.target_prediction.size(),
                       out_traj.empty() ? 0.0 : out_traj.getTotalDuration());
        return false;
    }

    RET_CODE GeneralPlanner::optimizeTrackingTask(const traj_opt::DynamicTargetStates &target_prediction,
                                                const bool &from_rest) {
        if (target_prediction.empty()) {
            setTrackingDiagnostic("input",
                                  "empty_target_prediction",
                                  0,
                                  0,
                                  0,
                                  0.0);
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking task has no target prediction.");
            return FAILED;
        }

        auto failOrKeepOld = [this, &target_prediction, &from_rest](const std::string &reason) -> RET_CODE {
            if (keepOldTrackingTrajectoryIfActive(target_prediction, reason)) {
                return NO_NEED;
            }
            const double committed_remaining = getCommittedTrajectoryRemainingDuration();
            const bool soft_recovery_needed =
                    cfg_.tracking_soft_recovery_enable &&
                    robot_state_.rcv &&
                    robot_state_.p.allFinite() &&
                    !target_prediction.empty() &&
                    trackingSoftRecoveryRequired(cfg_,
                                                 robot_state_.p,
                                                 target_prediction.front().position);
            const bool hold_recovery_allowed =
                    from_rest ||
                    committed_remaining <= 1.0e-3 ||
                    soft_recovery_needed;
            const double hold_duration =
                    soft_recovery_needed
                        ? std::max(0.2, cfg_.tracking_soft_recovery_hold_duration)
                        : 0.0;
            if (hold_recovery_allowed &&
                commitTrackingHoldTrajectory("tracking recovery hold after failure: " + reason,
                                             hold_duration,
                                             !from_rest)) {
                return NO_NEED;
            }
            if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                tracking_runtime_manager_->onRejected();
            }
            ros_ptr_->warn(" -- [GeneralPlanner] {}", reason);
            return FAILED;
        };

        const bool static_tracking =
                staticTargetPrediction(target_prediction,
                                       0.05,
                                       cfg_.tracking_static_tail_speed_epsilon);
        const double static_distance_tolerance =
                std::max(0.05,
                         cfg_.tracking_distance_tolerance *
                         std::clamp(cfg_.tracking_static_distance_tolerance_scale, 0.05, 1.0));
        const double static_height_tolerance =
                std::max(0.05,
                         cfg_.tracking_height_tolerance *
                         std::clamp(cfg_.tracking_static_height_tolerance_scale, 0.05, 1.0));

        TrackingFrontend::Config frontend_cfg;
        frontend_cfg.tracking_distance = cfg_.tracking_distance;
        frontend_cfg.distance_tolerance = static_tracking ? static_distance_tolerance
                                                          : cfg_.tracking_distance_tolerance;
        frontend_cfg.distance_lower_tolerance =
                static_tracking ? static_distance_tolerance
                                : cfg_.tracking_distance_lower_tolerance;
        frontend_cfg.distance_upper_tolerance =
                static_tracking ? static_distance_tolerance
                                : cfg_.tracking_distance_upper_tolerance;
        frontend_cfg.height_offset = cfg_.tracking_height_offset;
        frontend_cfg.height_tolerance = static_tracking ? static_height_tolerance
                                                        : cfg_.tracking_height_tolerance;
        frontend_cfg.safe_distance = cfg_.tracking_safe_distance;
        frontend_cfg.visibility_safe_distance = cfg_.tracking_visibility_safe_distance;
        frontend_cfg.visibility_cone_ratio = cfg_.tracking_visibility_cone_ratio;
        frontend_cfg.visibility_angle_clearance = cfg_.tracking_visibility_angle_clearance;
        frontend_cfg.reacquire_distance = cfg_.tracking_reacquire_distance;
        frontend_cfg.soft_recovery_enable = cfg_.tracking_soft_recovery_enable;
        frontend_cfg.soft_recovery_margin = cfg_.tracking_soft_recovery_margin;
        frontend_cfg.searching_horizon = cfg_.planning_horizon;
        frontend_cfg.low_speed_velocity_threshold = cfg_.tracking_low_speed_velocity_threshold;
        frontend_cfg.angular_hysteresis = cfg_.tracking_angular_hysteresis;
        frontend_cfg.candidate_angle_step = cfg_.tracking_candidate_angle_step;
        frontend_cfg.candidate_radius_num = cfg_.tracking_candidate_radius_num;
        frontend_cfg.visibility_samples = cfg_.tracking_visibility_samples;
        frontend_cfg.fallback_relax_enable = cfg_.tracking_fallback_relax_enable;
        frontend_cfg.fallback_distance_tolerance_scale = cfg_.tracking_fallback_distance_tolerance_scale;
        frontend_cfg.fallback_height_tolerance_scale = cfg_.tracking_fallback_height_tolerance_scale;
        frontend_cfg.fallback_candidate_radius_extra = cfg_.tracking_fallback_candidate_radius_extra;
        frontend_cfg.fallback_candidate_angle_step_scale = cfg_.tracking_fallback_candidate_angle_step_scale;
        frontend_cfg.fallback_search_horizon_scale = cfg_.tracking_fallback_search_horizon_scale;
        frontend_cfg.elastic_guide_enable = cfg_.tracking_frontend_elastic_enable;
        frontend_cfg.elastic_distance_tolerance_scale = cfg_.tracking_frontend_elastic_distance_tolerance_scale;
        frontend_cfg.elastic_height_tolerance_scale = cfg_.tracking_frontend_elastic_height_tolerance_scale;
        frontend_cfg.partial_guide_enable = cfg_.tracking_frontend_partial_guide_enable;
        frontend_cfg.partial_guide_min_duration = cfg_.tracking_frontend_partial_min_duration;
        frontend_cfg.partial_guide_min_samples = cfg_.tracking_frontend_partial_min_samples;
        frontend_cfg.fov_feasibility_enable =
                cfg_.tracking_frontend_fov_feasibility_enable &&
                cfg_.tracking_fov_check_strict;
        frontend_cfg.yaw_rate_feasibility_enable = cfg_.tracking_frontend_yaw_rate_feasibility_enable;
        const double effective_tracking_fov_range = trackingAdaptiveFovRange(cfg_);
        frontend_cfg.fov_horizontal_deg = cfg_.tracking_fov_horizontal_deg;
        frontend_cfg.fov_vertical_deg = cfg_.tracking_fov_vertical_deg;
        frontend_cfg.fov_range = effective_tracking_fov_range;
        frontend_cfg.fov_range_margin = cfg_.tracking_frontend_fov_range_margin;
        frontend_cfg.fov_front_margin = cfg_.tracking_fov_front_margin;
        frontend_cfg.max_yaw_rate = cfg_.yaw_dot_max;
        frontend_cfg.yaw_rate_margin = cfg_.tracking_frontend_yaw_rate_margin;
        frontend_cfg.obstacle_recovery_enable = cfg_.tracking_frontend_obstacle_recovery_enable;
        frontend_cfg.grid_neighbor_mode = cfg_.tracking_frontend_grid_neighbor_mode;
        frontend_cfg.over_wall_enable = cfg_.tracking_frontend_over_wall_enable;
        frontend_cfg.over_wall_max_climb = cfg_.tracking_frontend_over_wall_max_climb;
        frontend_cfg.side_pass_enable = cfg_.tracking_frontend_side_pass_enable;
        frontend_cfg.side_pass_width = cfg_.tracking_frontend_side_pass_width;
        frontend_cfg.reacquire_relax_yaw_rate = cfg_.tracking_frontend_reacquire_relax_yaw_rate;
        frontend_cfg.unknown_as_occupied = cfg_.tracking_unknown_as_occupied;
        frontend_cfg.use_astar = cfg_.tracking_frontend_astar;
        frontend_cfg.use_visible_region = cfg_.tracking_use_visible_region;
        frontend_cfg.print_log = cfg_.print_log;

        traj_opt::TrackingProblem problem;
        TimeConsuming t_frontend("tracking_frontend", false);
        const double tracking_plan_start_wt = ros_ptr_->getSimTime();
        const double tracking_candidate_head_wt =
                tracking_plan_start_wt + std::max(0.0, cfg_.replan_forward_dt);
        const StatePVAJ head_state = makeTaskHeadState(from_rest, tracking_candidate_head_wt);
        const bool has_committed_tracking_for_frontend =
                cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_
                    ? tracking_runtime_manager_->hasCommittedTracking()
                    : !cmd_traj_info_.empty();
        const double initial_target_distance =
                (head_state.col(0) - target_prediction.front().position).head<2>().norm();
        const double frontend_fov_range = effective_tracking_fov_range;
        const double frontend_reacquire_entry_distance =
                cfg_.tracking_soft_recovery_enable
                    ? trackingSoftRecoveryEntryDistance(cfg_)
                    : std::max({frontend_fov_range,
                                std::max(0.0, cfg_.tracking_reacquire_distance),
                                std::max(0.0, cfg_.tracking_reacquire_fov_entry_distance)});
        const bool frontend_reacquire_mode =
                cfg_.tracking_reacquire_fov_relax_enable &&
                (from_rest ||
                 !has_committed_tracking_for_frontend ||
                 initial_target_distance > frontend_reacquire_entry_distance);
        if (frontend_reacquire_mode && frontend_cfg.fov_feasibility_enable) {
            frontend_cfg.fov_feasibility_enable = false;
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_FRONTEND_FOV_FEASIBILITY_DISABLED reason=reacquire_or_uncommitted, from_rest={}, has_committed_tracking={}, initial_dist={:.3f}, entry_dist={:.3f}",
                               from_rest,
                               has_committed_tracking_for_frontend,
                               initial_target_distance,
                               frontend_reacquire_entry_distance);
            }
        }
        TrackingFrontend frontend(frontend_cfg, map_manager_, astar_ptr_);
        Vec3f reference_viewpoint = Vec3f::Zero();
        traj_opt::DynamicTargetState reference_target;
        const bool has_viewpoint_reference =
                findTrackingViewpointReference(target_prediction,
                                               reference_viewpoint,
                                               reference_target);
        if (!frontend.buildProblem(head_state,
                                   target_prediction,
                                   problem,
                                   has_viewpoint_reference ? &reference_viewpoint : nullptr,
                                   has_viewpoint_reference ? &reference_target : nullptr)) {
            time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
            setTrackingDiagnostic("frontend",
                                  "frontend_failed",
                                  problem,
                                  0.0);
            ros_ptr_->warn(" -- [Tracking] TRACKING_FRONTEND_FAILED guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration=0.000, prefix_duration=0.000, runtime_eval_start=0.000, old_remaining=0.000, old_activity.reason=frontend_failed",
                           problem.guide_path.size(),
                           problem.sfcs.size(),
                           problem.target_prediction.size());
            return failOrKeepOld("Tracking frontend failed.");
        }
        problem.static_tracking_mode = static_tracking;
        const double tracking_frontend_t = t_frontend.stop();
        time_consuming_[EPX_TRAJ_FRONTEND] = tracking_frontend_t;

        TimeConsuming t_tracking_sfc("tracking_sfc", false);
        std::string tracking_sfc_failure_reason;
        if (!buildTrackingGuideCorridor(problem, &tracking_sfc_failure_reason)) {
            time_consuming_[EPX_TRAJ_FRONTEND] += t_tracking_sfc.stop();
            setTrackingDiagnostic("sfc",
                                  tracking_sfc_failure_reason,
                                  problem,
                                  0.0);
            ros_ptr_->warn(" -- [Tracking] TRACKING_SFC_FAILED reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration=0.000, prefix_duration=0.000, runtime_eval_start=0.000",
                           tracking_sfc_failure_reason,
                           problem.guide_path.size(),
                           problem.sfcs.size(),
                           problem.target_prediction.size());
            return failOrKeepOld("Tracking SFC generation failed: " + tracking_sfc_failure_reason);
        }
        time_consuming_[EPX_TRAJ_FRONTEND] += t_tracking_sfc.stop();
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj, problem.tail_pvaj, problem.sfcs);
        const traj_opt::DynamicTargetStates &active_target_prediction =
                problem.target_prediction.empty() ? target_prediction : problem.target_prediction;

        problem.head_yaw << robot_state_.yaw, 0.0;
        if (!from_rest && !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory yaw_traj = cmd_traj_info_.yawTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            const double eval_t = tracking_candidate_head_wt - start_wt;
            StatePVAJ yaw_state;
            if (!yaw_traj.empty() && eval_t >= 0.0 && eval_t <= total_dur &&
                yaw_traj.getState(eval_t, yaw_state)) {
                problem.head_yaw = yaw_state.row(0).head<2>();
            }
        }
        double terminal_yaw = active_target_prediction.back().yaw;
        const Vec3f face_dir = active_target_prediction.back().position - problem.tail_pvaj.col(0);
        if (face_dir.head<2>().norm() > 1.0e-3) {
            terminal_yaw = std::atan2(face_dir.y(), face_dir.x());
            geometry_utils::normalizeNextYaw(problem.head_yaw(0, 0), terminal_yaw);
        }
        problem.tail_yaw << terminal_yaw, static_tracking ? 0.0 : active_target_prediction.back().yaw_rate;
        problem.weight_od_near = cfg_.tracking_weight_od_near;
        problem.weight_od_far = cfg_.tracking_weight_od_far;
        problem.weight_od_vertical = cfg_.tracking_weight_od_vertical;
        problem.weight_oa = cfg_.tracking_weight_oa;
        problem.weight_oe = cfg_.tracking_weight_oe;
        problem.weight_visibility = cfg_.tracking_weight_oe;
        problem.weight_relative_velocity = cfg_.tracking_weight_relative_velocity;
        problem.weight_tangent_velocity = cfg_.tracking_weight_tangent_velocity;
        problem.weight_viewpoint_attractor = cfg_.tracking_weight_viewpoint_attractor;
        problem.weight_visible_region = cfg_.tracking_weight_visible_region;
        problem.weight_fov = cfg_.tracking_weight_fov;
        problem.weight_target_forward = cfg_.tracking_weight_target_forward;
        problem.adaptive_occlusion_enable = cfg_.tracking_adaptive_occlusion_enable;
        problem.adaptive_occlusion_activation_distance = cfg_.tracking_adaptive_occlusion_activation_distance;
        problem.adaptive_occlusion_max_weight_scale = cfg_.tracking_adaptive_occlusion_max_weight_scale;
        problem.adaptive_occlusion_od_far_weight_scale = cfg_.tracking_adaptive_occlusion_od_far_weight_scale;
        problem.adaptive_occlusion_distance_upper_scale = cfg_.tracking_adaptive_occlusion_distance_upper_scale;
        problem.adaptive_occlusion_min_horizontal_upper = cfg_.tracking_adaptive_occlusion_min_horizontal_upper;
        constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
        problem.fov_horizontal = std::max(1.0, cfg_.tracking_fov_horizontal_deg) * kDegToRad;
        problem.fov_vertical = std::max(1.0, cfg_.tracking_fov_vertical_deg) * kDegToRad;
        problem.fov_range = effective_tracking_fov_range;
        problem.fov_range_margin = cfg_.tracking_fov_range_margin;
        problem.fov_front_margin = cfg_.tracking_fov_front_margin;
        problem.target_front_margin = cfg_.tracking_target_front_margin;
        problem.target_motion_speed_threshold =
                std::max(0.0, cfg_.tracking_no_motion_target_speed_threshold);
        problem.joint_sample_dt = cfg_.tracking_joint_sample_dt;
        problem.dense_joint_sample_enable = cfg_.tracking_dense_joint_sample_enable;
        if (problem.reacquire_mode) {
            problem.weight_visible_region *= 0.25;
        }
        if (static_tracking) {
            problem.distance_tolerance = static_distance_tolerance;
            problem.height_tolerance = static_height_tolerance;
            problem.od_h_lower = std::max(0.05, cfg_.tracking_distance - static_distance_tolerance);
            problem.od_h_upper = std::max(problem.od_h_lower + 0.05,
                                          cfg_.tracking_distance + static_distance_tolerance);
            problem.od_v_lower = cfg_.tracking_height_offset - static_height_tolerance;
            problem.od_v_upper = cfg_.tracking_height_offset + static_height_tolerance;
            problem.tail_pvaj.col(1).setZero();
            problem.tail_pvaj.col(2).setZero();
            problem.tail_pvaj.col(3).setZero();
            for (auto &state : problem.target_prediction) {
                state.velocity.setZero();
                state.acceleration.setZero();
                state.yaw_rate = 0.0;
            }
            problem.weight_tangent_velocity *=
                    std::max(1.0, cfg_.tracking_static_tangent_weight_scale);
        }
        const int valid_visible_regions = validVisibleRegionCount(problem);
        problem.use_visible_region = cfg_.tracking_use_visible_region && valid_visible_regions > 0;
        problem.visibility_angle_clearance = cfg_.tracking_visibility_angle_clearance;
        if (cfg_.print_log && cfg_.tracking_use_visible_region) {
            ros_ptr_->info(" -- [GeneralPlanner] Tracking visible-region soft prior: valid={}/{}, enabled={}, reacquire={}.",
                           valid_visible_regions,
                           problem.visible_regions.size(),
                           problem.use_visible_region,
                           problem.reacquire_mode);
        }
        if (static_tracking && cfg_.print_log) {
            ros_ptr_->info(" -- [GeneralPlanner] Static tracking mode: OD_h=[{:.2f},{:.2f}], OD_v=[{:.2f},{:.2f}], tangent_weight={:.2f}.",
                           problem.od_h_lower,
                           problem.od_h_upper,
                           problem.od_v_lower,
                           problem.od_v_upper,
                           problem.weight_tangent_velocity);
        }

        {
            TimeConsuming t_viz("tracking_frontend_viz", false);
            ros_ptr_->vizFrontendPath(problem.guide_path);
            ros_ptr_->vizExpSfc(problem.sfcs);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        Trajectory out_traj;
        Trajectory out_yaw_traj;
        traj_opt::DynamicTargetStates accepted_target_prediction = active_target_prediction;
        bool accepted_reacquire_fov_relax = false;
        TimeConsuming t_opt("tracking_opt", false);
        std::string tracking_opt_failure_reason;
        const bool ok = optimizeTrackingProblemWithRetries(problem,
                                                           active_target_prediction,
                                                           out_traj,
                                                           out_yaw_traj,
                                                           &accepted_target_prediction,
                                                           &accepted_reacquire_fov_relax,
                                                           &tracking_opt_failure_reason);
        time_consuming_[EXP_TRAJ_OPT] = t_opt.stop();
        if (!ok || out_traj.empty()) {
            setTrackingDiagnostic("optimizer",
                                  tracking_opt_failure_reason,
                                  problem,
                                  out_traj.empty() ? 0.0 : out_traj.getTotalDuration());
            ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_FAILED reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, prefix_duration=0.000, runtime_eval_start=0.000",
                           tracking_opt_failure_reason,
                           problem.guide_path.size(),
                           problem.sfcs.size(),
                           problem.target_prediction.size(),
                           out_traj.empty() ? 0.0 : out_traj.getTotalDuration());
            const std::string failure_message =
                    "Tracking optimization failed: " + tracking_opt_failure_reason;
            return failOrKeepOld(failure_message);
        }
        setTrackingDiagnostic("candidate",
                              "optimizer_success",
                              problem,
                              out_traj.getTotalDuration());
        if (out_traj.getTotalDuration() < cfg_.tracking_min_commit_duration) {
            setTrackingDiagnostic("commit_guard",
                                  fmt::format("trajectory_too_short:{:.3f}<{}",
                                              out_traj.getTotalDuration(),
                                              cfg_.tracking_min_commit_duration),
                                  problem,
                                  out_traj.getTotalDuration());
            return failOrKeepOld(fmt::format("Tracking trajectory too short ({:.3f}s < {:.3f}s).",
                                             out_traj.getTotalDuration(),
                                             cfg_.tracking_min_commit_duration));
        }

        const double candidate_guard_h =
                std::min(cfg_.tracking_no_motion_check_horizon,
                         out_traj.getTotalDuration());
        const TrackingMotionMetrics candidate_metrics =
                computeTrackingMotionMetrics(out_traj,
                                             accepted_target_prediction,
                                             cfg_,
                                             0.0,
                                             0.0,
                                             candidate_guard_h);
        double old_remaining = 0.0;
        double old_speed0 = 0.0;
        double old_speed_z = 0.0;
        double old_speed_3d = 0.0;
        double old_displacement = 0.0;
        double old_displacement_z = 0.0;
        double old_displacement_3d = 0.0;
        double old_progress = 0.0;
        double old_progress_3d = 0.0;
        double old_expected_progress = 0.0;
        double old_avg_tracking_error = 0.0;
        std::string old_activity_reason = "none";
        if (cfg_.tracking_runtime_manager_enable &&
            tracking_runtime_manager_ &&
            tracking_runtime_manager_->hasCommittedTracking() &&
            !cmd_traj_info_.empty()) {
            Trajectory old_pos_traj;
            double old_start_wt = 0.0;
            double old_total_dur = 0.0;
            cmd_traj_info_.lock();
            old_pos_traj = cmd_traj_info_.posTraj();
            old_start_wt = cmd_traj_info_.getStartWallTime();
            old_total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();
            const auto old_activity =
                    tracking_runtime_manager_->evaluateActivity(
                            old_pos_traj,
                            std::clamp(ros_ptr_->getSimTime() - old_start_wt,
                                       0.0,
                                       old_total_dur),
                            accepted_target_prediction,
                            cfg_.tracking_keep_old_horizon,
                            cfg_.tracking_keep_old_safety_dt);
            old_remaining = old_activity.remaining;
            old_speed0 = old_activity.speed0;
            old_speed_z = old_activity.speed_z;
            old_speed_3d = old_activity.speed_3d;
            old_displacement = old_activity.displacement;
            old_displacement_z = old_activity.displacement_z;
            old_displacement_3d = old_activity.displacement_3d;
            old_progress = old_activity.progress;
            old_progress_3d = old_activity.progress_3d;
            old_expected_progress = old_activity.expected_progress;
            old_avg_tracking_error = old_activity.avg_tracking_error;
            old_activity_reason = old_activity.reason;
        } else if (!cfg_.tracking_runtime_manager_enable) {
            TrackingTrajectoryActivity old_activity;
            currentTrackingTrajectorySafeAndActive(accepted_target_prediction, &old_activity);
            old_remaining = old_activity.remaining;
            old_speed0 = old_activity.speed0;
            old_speed_z = old_activity.speed_z;
            old_speed_3d = old_activity.speed_3d;
            old_displacement = old_activity.displacement;
            old_displacement_z = old_activity.displacement_z;
            old_displacement_3d = old_activity.displacement_3d;
            old_progress = old_activity.progress;
            old_progress_3d = old_activity.progress_3d;
            old_expected_progress = old_activity.expected_progress;
            old_avg_tracking_error = old_activity.avg_tracking_error;
            old_activity_reason = old_activity.reason;
        }
        if (cfg_.print_log) {
            ros_ptr_->info(" -- [Tracking] TRACKING_CANDIDATE_OPT_SUCCESS guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, candidate_duration={:.3f}, prefix_duration=0.000, runtime_eval_start=0.000, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, target_speed_z={:.3f}, old_remaining={:.3f}, old_activity.reason={}, old_speed_xy={:.3f}, old_speed_z={:.3f}, old_speed_3d={:.3f}, old_displacement_xy={:.3f}, old_displacement_z={:.3f}, old_displacement_3d={:.3f}, old_progress_xy={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                           problem.guide_path.size(),
                           problem.sfcs.size(),
                           problem.target_prediction.size(),
                           out_traj.getTotalDuration(),
                           candidate_metrics.displacement_xy,
                           candidate_metrics.displacement_z,
                           candidate_metrics.displacement_3d,
                           candidate_metrics.speed_xy,
                           candidate_metrics.speed_z,
                           candidate_metrics.speed_3d,
                           candidate_metrics.target_speed_z,
                           old_remaining,
                           old_activity_reason,
                           old_speed0,
                           old_speed_z,
                           old_speed_3d,
                           old_displacement,
                           old_displacement_z,
                           old_displacement_3d,
                           old_progress,
                           old_progress_3d,
                           old_expected_progress,
                           old_avg_tracking_error);
        }

        std::string commandable_reject_reason;
        if (!cfg_.tracking_runtime_manager_enable &&
            !candidateTrackingTrajectoryCommandable(out_traj,
                                                    accepted_target_prediction,
                                                    0.0,
                                                    0.0,
                                                    &commandable_reject_reason)) {
            TrackingTrajectoryActivity old_activity;
            currentTrackingTrajectorySafeAndActive(accepted_target_prediction, &old_activity);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_NO_MOTION reject_reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, candidate_duration={:.3f}, prefix_duration=0.000, runtime_eval_start=0.000, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, old_remaining={:.3f}, old_activity.reason={}, old_speed_3d={:.3f}, old_displacement_3d={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               commandable_reject_reason,
                               problem.guide_path.size(),
                               problem.sfcs.size(),
                               problem.target_prediction.size(),
                               out_traj.getTotalDuration(),
                               candidate_metrics.displacement_xy,
                               candidate_metrics.displacement_z,
                               candidate_metrics.displacement_3d,
                               candidate_metrics.speed_xy,
                               candidate_metrics.speed_z,
                               candidate_metrics.speed_3d,
                               old_activity.remaining,
                               old_activity.reason,
                               old_activity.speed_3d,
                               old_activity.displacement_3d,
                               old_activity.progress_3d,
                               old_activity.expected_progress,
                               old_activity.avg_tracking_error);
            }

            if (old_activity.active &&
                keepOldTrackingTrajectoryIfActive(active_target_prediction,
                                                  "tracking candidate rejected by no-motion guard")) {
                return NO_NEED;
            }

            std::string candidate_safe_reason;
            std::string candidate_safe_detail;
            const bool candidate_safe_for_commit =
                    trackingCandidateSafeForCommit(out_traj,
                                                  &candidate_safe_reason,
                                                  &candidate_safe_detail);
            if (old_activity.valid &&
                !old_activity.safe &&
                candidate_safe_for_commit) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] no-motion guard allows safe candidate because old trajectory is unsafe. old_reason={}",
                                   old_activity.reason);
                }
            } else {
                setTrackingCommitRejectInfo(
                        "candidate rejected by no-motion guard: " + commandable_reject_reason,
                        fmt::format(
                                "failure=no_motion_guard|candidate_safe_for_commit={}|candidate_safe_reason={}|candidate_safe_detail={}|old_activity_valid={}|old_activity_safe={}|old_activity_active={}|old_activity_reason={}|candidate_duration={:.3f}",
                                static_cast<int>(candidate_safe_for_commit),
                                candidate_safe_reason.empty() ? "none" : candidate_safe_reason,
                                candidate_safe_detail.empty() ? "none" : candidate_safe_detail,
                                static_cast<int>(old_activity.valid),
                                static_cast<int>(old_activity.safe),
                                static_cast<int>(old_activity.active),
                                old_activity.reason.empty() ? "none" : old_activity.reason,
                                out_traj.getTotalDuration()));
                setTrackingDiagnostic("commit_guard",
                                      "candidate_rejected_no_motion:" + commandable_reject_reason,
                                      problem,
                                      out_traj.getTotalDuration());
                return failOrKeepOld("Tracking candidate rejected by no-motion guard: " +
                                     commandable_reject_reason);
            }
        }

        {
            TimeConsuming t_viz("tracking_fov_viz", false);
            ros_ptr_->vizTrackingFov(out_traj,
                                     out_yaw_traj,
                                     cfg_.tracking_fov_horizontal_deg,
                                     cfg_.tracking_fov_vertical_deg,
                                     effective_tracking_fov_range);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        if (!commitTrackingTrajectory(out_traj,
                                      out_yaw_traj,
                                      accepted_target_prediction,
                                      cfg_.tracking_use_snap ? "tracking_snap" : "tracking_jerk",
                                      tracking_candidate_head_wt,
                                      accepted_reacquire_fov_relax,
                                      !from_rest)) {
            const std::string commit_reason =
                    last_tracking_commit_reject_reason_.empty()
                        ? "unknown"
                        : last_tracking_commit_reject_reason_;
            setTrackingDiagnostic("commit",
                                  commit_reason,
                                  problem,
                                  out_traj.getTotalDuration());
            if (keepOldTrackingTrajectoryIfActive(active_target_prediction,
                                                  "tracking trajectory commit rejected: " + commit_reason)) {
                return NO_NEED;
            }
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking trajectory commit rejected: reason={}.", commit_reason);
            return failOrKeepOld("Tracking trajectory commit rejected: " + commit_reason);
        }
        rememberTrackingViewpointReference(problem);
        setTrackingDiagnostic("success",
                              "candidate_committed",
                              problem,
                              out_traj.getTotalDuration());
        ros_ptr_->info(" -- [GeneralPlanner] Tracking task success: pieces={}, duration={}.",
                       out_traj.getPieceNum(), out_traj.getTotalDuration());
        return SUCCESS;
    }

} // namespace general_planner
