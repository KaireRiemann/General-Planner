/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <utils/geometry/tracking_attitude.hpp>
#include <general_core/tracking/tracking_internal_utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <fmt/format.h>

using namespace general_utils;

namespace general_planner {
    namespace {
        void setFailureReason(std::string *out, const std::string &reason) {
            if (out != nullptr) {
                *out = reason;
            }
        }

        const char *gridTypeName(const rog_map::GridType type) {
            switch (type) {
                case rog_map::GridType::UNKNOWN:
                    return "UNKNOWN";
                case rog_map::GridType::OCCUPIED:
                    return "OCCUPIED";
                case rog_map::GridType::KNOWN_FREE:
                    return "KNOWN_FREE";
                case rog_map::GridType::OUT_OF_MAP:
                    return "OUT_OF_MAP";
                case rog_map::GridType::UNDEFINED:
                    return "UNDEFINED";
                case rog_map::GridType::FRONTIER:
                    return "FRONTIER";
            }
            return "UNKNOWN_GRID_TYPE";
        }

        std::string formatVec3Compact(const Vec3f &v) {
            if (!v.allFinite()) {
                return "[nan,nan,nan]";
            }
            return fmt::format("[{:.3f},{:.3f},{:.3f}]", v.x(), v.y(), v.z());
        }

        struct TrackingFovSampleStatus {
            bool inside{false};
            double h_violation{0.0};
            double v_violation{0.0};
            double range_violation{0.0};
            double front_violation{0.0};
            double distance{0.0};
            double qx{0.0};
        };

        TrackingFovSampleStatus evaluateTrackingCameraFov(
                const Vec3f &optical,
                const double horizontal_fov_deg,
                const double vertical_fov_deg,
                const double range,
                const double range_margin,
                const double front_margin) {
            TrackingFovSampleStatus out;
            if (!optical.allFinite()) {
                out.h_violation = std::numeric_limits<double>::infinity();
                out.v_violation = std::numeric_limits<double>::infinity();
                out.range_violation = std::numeric_limits<double>::infinity();
                out.front_violation = std::numeric_limits<double>::infinity();
                return out;
            }

            constexpr double kPi = 3.14159265358979323846;
            constexpr double kDegToRad = kPi / 180.0;
            const double half_h =
                    std::clamp(0.5 * std::max(1.0, horizontal_fov_deg) * kDegToRad,
                               kPi / 180.0,
                               0.5 * kPi - 1.0e-3);
            const double half_v =
                    std::clamp(0.5 * std::max(1.0, vertical_fov_deg) * kDegToRad,
                               kPi / 180.0,
                               0.5 * kPi - 1.0e-3);
            const double effective_range =
                    range > 0.0 ? std::max(0.05, range - std::max(0.0, range_margin)) : -1.0;
            const double min_forward = std::max(1.0e-3, front_margin);

            const Vec3f q(optical.z(), optical.x(), optical.y());
            out.qx = q.x();
            out.distance = q.norm();

            const double h_angle = q.x() > 1.0e-6
                                       ? std::atan2(std::abs(q.y()), q.x())
                                       : kPi;
            const double v_angle = q.x() > 1.0e-6
                                       ? std::atan2(std::abs(q.z()), q.x())
                                       : kPi;
            out.h_violation = h_angle - half_h;
            out.v_violation = v_angle - half_v;
            out.range_violation =
                    effective_range > 0.0 ? out.distance - effective_range : 0.0;
            out.front_violation = min_forward - q.x();
            out.inside =
                    out.h_violation <= 1.0e-6 &&
                    out.v_violation <= 1.0e-6 &&
                    out.range_violation <= 1.0e-6 &&
                    out.front_violation <= 1.0e-6;
            return out;
        }
    }

    bool GeneralPlanner::keepOldTrackingTrajectoryIfActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            const std::string &reason) {
        auto keepOldReason = [this](const std::string &status) {
            return fmt::format("{};trigger_phase={};trigger_reason={}",
                               status,
                               last_tracking_diag_phase_,
                               last_tracking_diag_reason_);
        };
        auto tryShortSafetyGrace =
                [&](const Trajectory &old_pos_traj,
                    const Trajectory &old_yaw_traj,
                    const double old_local_t,
                    const auto &activity) -> bool {
            if (!cfg_.tracking_keep_old_short_safety_grace_enable ||
                old_pos_traj.empty() ||
                target_prediction.empty()) {
                return false;
            }
            if (activity.remaining < cfg_.tracking_keep_old_min_remaining) {
                return false;
            }

            const double grace_horizon =
                    std::min({std::max(0.0, cfg_.tracking_keep_old_short_safety_grace_horizon),
                              std::max(0.0, activity.remaining),
                              std::max(0.0, old_pos_traj.getTotalDuration() - old_local_t),
                              std::max(0.0, target_prediction.back().t)});
            if (grace_horizon < 1.0e-3) {
                return false;
            }
            if (!trackingTrajectorySafeForHorizon(old_pos_traj,
                                                  old_local_t,
                                                  grace_horizon,
                                                  cfg_.tracking_keep_old_safety_dt)) {
                return false;
            }

            if (!trackingCandidateHasMotion(old_pos_traj, target_prediction, cfg_, old_local_t, 0.0))
                return false;

            std::string old_fov_reason;
            const bool old_fov_ok =
                    !cfg_.tracking_keep_old_requires_fov ||
                    (!old_yaw_traj.empty() &&
                     trackingTrajectorySatisfiesFov(old_pos_traj,
                                                    old_yaw_traj,
                                                    target_prediction,
                                                    old_local_t,
                                                    grace_horizon,
                                                    cfg_.tracking_fov_check_dt,
                                                    0.0,
                                                    &old_fov_reason,
                                                    true,
                                                    true));
            if (!old_fov_ok) {
                setTrackingCommitRejectInfo("keep-old FOV rejected", old_fov_reason);
                ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_FOV reason={}, detail={}",
                               reason, old_fov_reason);
                return false;
            }

            tracking_runtime_manager_->onKeepOld();
            setTrackingDiagnostic("keep_old",
                                  keepOldReason(fmt::format("{}:{};grace_horizon={:.3f};old_fov_ok={};old_fov_reason={}",
                                                            "short_safety_grace",
                                                            activity.reason,
                                                            grace_horizon,
                                                            static_cast<int>(old_fov_ok),
                                                            old_fov_reason.empty() ? "none" : old_fov_reason)),
                                  last_tracking_diag_guide_path_size_,
                                  last_tracking_diag_sfc_size_,
                                  target_prediction.size(),
                                  old_pos_traj.getTotalDuration());
            latest_replan.setExpTraj(old_pos_traj);
            latest_replan.setExpYawTraj(old_yaw_traj);
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] {} reason={}, activity_reason={}, grace_horizon={:.3f}, old_remaining={:.3f}, old_fov_ok={}, old_fov_reason={}, keep_old_count={}, reject_count={}",
                               "TRACKING_KEEP_OLD_SHORT_SAFETY_GRACE",
                               reason,
                               activity.reason,
                               grace_horizon,
                               activity.remaining,
                               old_fov_ok,
                               old_fov_reason.empty() ? "none" : old_fov_reason,
                               tracking_runtime_manager_->consecutiveKeepOld(),
                               tracking_runtime_manager_->consecutiveReject());
            }
            return true;
        };

        if (cmd_traj_info_.empty() ||
            !tracking_runtime_manager_->hasCommittedTracking()) {
            setTrackingDiagnostic("keep_old",
                                  keepOldReason("inactive:no_committed_tracking_trajectory"),
                                  last_tracking_diag_guide_path_size_,
                                  last_tracking_diag_sfc_size_,
                                  target_prediction.size(),
                                  last_tracking_diag_out_traj_duration_);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_INACTIVE reason={}, activity_reason=no committed tracking trajectory, keep_old_count={}, reject_count={}",
                               reason,
                               tracking_runtime_manager_->consecutiveKeepOld(),
                               tracking_runtime_manager_->consecutiveReject());
            }
            return false;
        }

        Trajectory old_pos_traj;
        Trajectory old_yaw_traj;
        double old_start_wt = 0.0;
        double old_total_dur = 0.0;
        cmd_traj_info_.lock();
        old_pos_traj = cmd_traj_info_.posTraj();
        old_yaw_traj = cmd_traj_info_.yawTraj();
        old_start_wt = cmd_traj_info_.getStartWallTime();
        old_total_dur = cmd_traj_info_.getTotalDuration();
        cmd_traj_info_.unlock();

        const double now = ros_ptr_->getSimTime();
        const double old_local_t =
                std::clamp(now - old_start_wt, 0.0, old_total_dur);
        const auto activity =
                tracking_runtime_manager_->evaluateActivity(old_pos_traj,
                                                             old_local_t,
                                                             target_prediction,
                                                             cfg_.tracking_keep_old_horizon,
                                                             cfg_.tracking_keep_old_safety_dt);
        if (!activity.active) {
            if (tryShortSafetyGrace(old_pos_traj,
                                    old_yaw_traj,
                                    old_local_t,
                                    activity)) {
                return true;
            }
            setTrackingDiagnostic("keep_old",
                                  keepOldReason("inactive:" + activity.reason),
                                  last_tracking_diag_guide_path_size_,
                                  last_tracking_diag_sfc_size_,
                                  target_prediction.size(),
                                  last_tracking_diag_out_traj_duration_);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_INACTIVE reason={}, activity_reason={}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}, keep_old_count={}, reject_count={}",
                               reason,
                               activity.reason,
                               activity.remaining,
                               activity.speed0,
                               activity.displacement,
                               activity.progress,
                               activity.expected_progress,
                               activity.avg_tracking_error,
                               tracking_runtime_manager_->consecutiveKeepOld(),
                               tracking_runtime_manager_->consecutiveReject());
            }
            return false;
        }

        std::string old_fov_reason;
        if (!trackingSnapshotSatisfiesFovForKeepOld(old_pos_traj,
                                                    old_yaw_traj,
                                                    old_local_t,
                                                    target_prediction,
                                                    &old_fov_reason)) {
            if (tryShortSafetyGrace(old_pos_traj,
                                    old_yaw_traj,
                                    old_local_t,
                                    activity)) {
                return true;
            }
            setTrackingDiagnostic("keep_old",
                                  keepOldReason("fov_rejected:" + old_fov_reason),
                                  last_tracking_diag_guide_path_size_,
                                  last_tracking_diag_sfc_size_,
                                  target_prediction.size(),
                                  last_tracking_diag_out_traj_duration_);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_FOV reason={}, fov_reason={}, old_local_t={:.3f}, old_remaining={:.3f}, keep_old_count={}, reject_count={}",
                               reason,
                               old_fov_reason,
                               old_local_t,
                               activity.remaining,
                               tracking_runtime_manager_->consecutiveKeepOld(),
                               tracking_runtime_manager_->consecutiveReject());
            }
            return false;
        }

        tracking_runtime_manager_->onKeepOld();
        setTrackingDiagnostic("keep_old",
                              keepOldReason("active:" + reason),
                              last_tracking_diag_guide_path_size_,
                              last_tracking_diag_sfc_size_,
                              target_prediction.size(),
                              old_pos_traj.getTotalDuration());
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
                           tracking_runtime_manager_->consecutiveKeepOld(),
                           tracking_runtime_manager_->consecutiveReject());
        }
        return true;
    }

    bool GeneralPlanner::trackingTrajectorySafeForHorizonDetailed(
            const Trajectory &traj,
            const double start_t,
            const double horizon,
            const double dt,
            std::string *reason,
            std::string *detail) const {
        setFailureReason(reason, "");
        setFailureReason(detail, "");

        const auto set_reject =
                [&](const std::string &reject_reason,
                    const double sample_dt,
                    const double eval_t,
                    const Vec3f &pos,
                    const Vec3f &prev,
                    const rog_map::GridType grid_type,
                    const bool inside_local_map,
                    const bool line_free,
                    const double esdf_dist) -> bool {
            setFailureReason(reason, reject_reason);
            if (detail != nullptr) {
                *detail = fmt::format(
                        "check_start_t={:.3f}|check_horizon={:.3f}|sample_dt={:.3f}|fail_offset={:.3f}|fail_t={:.3f}|pos={}|prev={}|segment_len={:.3f}|inside_local_map={}|grid={}|unknown_as_occupied={}|esdf_dist={:.3f}|line_free={}",
                        start_t,
                        horizon,
                        sample_dt,
                        eval_t - start_t,
                        eval_t,
                        formatVec3Compact(pos),
                        formatVec3Compact(prev),
                        (pos - prev).norm(),
                        static_cast<int>(inside_local_map),
                        gridTypeName(grid_type),
                        static_cast<int>(cfg_.tracking_unknown_as_occupied),
                        esdf_dist,
                        static_cast<int>(line_free));
            }
            return false;
        };

        if (traj.empty()) {
            setFailureReason(reason, "empty trajectory");
            if (detail != nullptr) {
                *detail = fmt::format("check_start_t={:.3f}|check_horizon={:.3f}|failure=empty_trajectory",
                                      start_t,
                                      horizon);
            }
            return false;
        }
        if (horizon <= 1.0e-6) {
            return true;
        }
        const double total_dur = traj.getTotalDuration();
        if (!std::isfinite(start_t) ||
            start_t < -1.0e-6 ||
            start_t + horizon > total_dur + 1.0e-6) {
            setFailureReason(reason, "invalid safety horizon");
            if (detail != nullptr) {
                *detail = fmt::format(
                        "check_start_t={:.3f}|check_horizon={:.3f}|total_duration={:.3f}|failure=invalid_safety_horizon",
                        start_t,
                        horizon,
                        total_dur);
            }
            return false;
        }
        if (map_manager_ == nullptr || !map_manager_->ready()) {
            return true;
        }

        const double safe_dt = std::max(0.05, dt);
        Vec3f last = traj.getPos(std::clamp(start_t, 0.0, total_dur));
        if (!last.allFinite()) {
            setFailureReason(reason, "non-finite initial point");
            if (detail != nullptr) {
                *detail = fmt::format(
                        "check_start_t={:.3f}|check_horizon={:.3f}|sample_dt={:.3f}|failure=non_finite_initial_point|pos={}",
                        start_t,
                        horizon,
                        safe_dt,
                        formatVec3Compact(last));
            }
            return false;
        }

        for (double offset = 0.0; offset <= horizon + 1.0e-6; offset += safe_dt) {
            const double t = std::clamp(start_t + offset, 0.0, total_dur);
            const Vec3f pos = traj.getPos(t);
            if (!pos.allFinite()) {
                return set_reject("non-finite sample",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  rog_map::GridType::UNDEFINED,
                                  true,
                                  true,
                                  std::numeric_limits<double>::quiet_NaN());
            }
            if (!map_manager_->insideLocalMap(pos)) {
                return set_reject("outside local map",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  rog_map::GridType::OUT_OF_MAP,
                                  false,
                                  true,
                                  std::numeric_limits<double>::quiet_NaN());
            }

            const auto grid_type = map_manager_->getInfGridType(pos);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP) {
                return set_reject("occupied or out-of-map",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  grid_type,
                                  true,
                                  true,
                                  std::numeric_limits<double>::quiet_NaN());
            }
            if (cfg_.tracking_unknown_as_occupied &&
                (grid_type == rog_map::GridType::UNKNOWN ||
                 grid_type == rog_map::GridType::UNDEFINED ||
                 grid_type == rog_map::GridType::FRONTIER)) {
                return set_reject("unknown treated as occupied",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  grid_type,
                                  true,
                                  true,
                                  std::numeric_limits<double>::quiet_NaN());
            }

            double esdf_dist = std::numeric_limits<double>::quiet_NaN();
            if (map_manager_->hasESDF()) {
                Vec3f grad = Vec3f::Zero();
                if (map_manager_->evaluateESDF(pos, esdf_dist, grad) &&
                    esdf_dist < trackingHardSafeDistance(cfg_)) {
                    return set_reject("inside tracking hard safe distance",
                                      safe_dt,
                                      t,
                                      pos,
                                      last,
                                      grid_type,
                                      true,
                                      true,
                                      esdf_dist);
                }
            }

            if ((pos - last).norm() > 1.0e-4 &&
                !map_manager_->isLineFree(last, pos, true, cfg_.tracking_unknown_as_occupied)) {
                return set_reject("segment not line-free",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  grid_type,
                                  true,
                                  false,
                                  esdf_dist);
            }
            last = pos;
        }
        return true;
    }

    bool GeneralPlanner::trackingSnapshotSatisfiesFovForKeepOld(
            const Trajectory &pos_traj,
            const Trajectory &yaw_traj,
            const double local_start_t,
            const traj_opt::DynamicTargetStates &input_prediction,
            std::string *reason) const {
        if (!cfg_.tracking_keep_old_requires_fov) {
            return true;
        }
        if (pos_traj.empty() || yaw_traj.empty()) {
            setFailureReason(reason, "old trajectory has empty position or yaw trajectory");
            return false;
        }

        const auto target_prediction = trackingPredictionAtTime(input_prediction, pos_traj.start_WT + local_start_t);
        const double total = std::min(pos_traj.getTotalDuration(), yaw_traj.getTotalDuration());
        const double begin = std::clamp(local_start_t, 0.0, total);
        const double horizon =
                std::min({std::max(0.0, cfg_.tracking_keep_old_horizon),
                          std::max(0.0, total - begin),
                          target_prediction.empty() ? 0.0
                                                    : std::max(0.0, target_prediction.back().t)});
        return trackingTrajectorySatisfiesFov(pos_traj,
                                              yaw_traj,
                                              target_prediction,
                                              begin,
                                              horizon,
                                              cfg_.tracking_fov_check_dt,
                                              0.0,
                                              reason,
                                              true,
                                              true);
    }

    bool GeneralPlanner::trackingTrajectorySatisfiesFov(
            const Trajectory &pos_traj,
            const Trajectory &yaw_traj,
            const traj_opt::DynamicTargetStates &target_prediction,
            const double start_t,
            const double horizon,
            const double dt,
            const double target_start_t,
            std::string *reason,
            const bool allow_keep_old_grace,
            const bool allow_reacquire_range_grace) const {
        if (!cfg_.tracking_fov_commit_check_enable) {
            return true;
        }
        if (!cfg_.tracking_fov_check_strict) {
            return true;
        }
        if (pos_traj.empty() || yaw_traj.empty() || target_prediction.empty()) {
            setFailureReason(reason, "empty trajectory, yaw trajectory, or target prediction");
            return false;
        }

        const double pos_total = pos_traj.getTotalDuration();
        const double yaw_total = yaw_traj.getTotalDuration();
        const double total = std::min(pos_total, yaw_total);
        if (total <= 1.0e-6) {
            setFailureReason(reason, "trajectory duration too short");
            return false;
        }

        constexpr double kPi = 3.14159265358979323846;
        constexpr double kDegToRad = kPi / 180.0;
        const double range = trackingAdaptiveFovRange(cfg_);
        const double begin = std::clamp(start_t, 0.0, total);
        const double target_begin = std::max(0.0, target_start_t);
        const double eval_horizon =
                std::min({std::max(0.0, horizon),
                          std::max(0.0, total - begin),
                          std::max(0.0, target_prediction.back().t - target_begin)});
        if (eval_horizon < 1.e-3) {
            setFailureReason(reason, "NO_PREDICTION: no trusted prediction horizon for FOV check");
            return false;
        }
        const double safe_dt = std::max(0.01, dt);
        const double reacquire_angular_grace_rad =
                std::max(0.0, cfg_.tracking_reacquire_fov_angular_grace_deg) * kDegToRad;

        int total_sample_count = 0;
        double total_max_h_violation = 0.0;
        double total_max_v_violation = 0.0;
        double total_max_range_violation = 0.0;
        double total_max_front_violation = 0.0;
        double initial_distance = std::numeric_limits<double>::infinity();
        double final_distance = std::numeric_limits<double>::infinity();

        const int fov_samples = std::max(1, static_cast<int>(std::ceil(eval_horizon / safe_dt)));
        for (int sample_id = 0; sample_id <= fov_samples; ++sample_id) {
            const double s = eval_horizon * sample_id / fov_samples;
            const double t = std::min(total, begin + s);
            const Vec3f p = pos_traj.getPos(t);
            StatePVAJ yaw_state;
            if (!p.allFinite() ||
                !yaw_traj.getState(std::min(t, yaw_total), yaw_state)) {
                setFailureReason(reason, "non-finite FOV sample");
                return false;
            }

            const double yaw = yaw_state(0, 0);
            const Vec3f target =
                    interpolateTargetPrediction(target_prediction, target_begin + s).position;
            // Match the flatness attitude actually sent to the vehicle.
            const geometry_utils::TrackingAttitude frame(pos_traj.getAcc(t), yaw);
            if (!frame.valid) {
                setFailureReason(reason, "invalid FOV thrust/heading frame");
                return false;
            }
            const Eigen::Matrix3d &body_R = frame.rotation;
            Eigen::Matrix3d camera_R;
            for (int row=0; row<3; ++row)
                for (int col=0; col<3; ++col)
                    camera_R(row,col)=cfg_.tracking_camera_R[row*3+col];
            if (!camera_R.allFinite() ||
                !(camera_R.transpose()*camera_R).isApprox(Eigen::Matrix3d::Identity(),1.e-5) ||
                std::abs(camera_R.determinant()-1.0)>1.e-5) {
                setFailureReason(reason, "invalid tracking camera rotation");
                return false;
            }
            const Vec3f camera_p = p + body_R * Vec3f(cfg_.tracking_camera_p[0],
                                                       cfg_.tracking_camera_p[1],
                                                       cfg_.tracking_camera_p[2]);
            const Eigen::Matrix3d world_to_camera = (body_R*camera_R).transpose();
            auto sample = [&](const Vec3f &point) {
                const Vec3f optical = world_to_camera * (point-camera_p);
                return evaluateTrackingCameraFov(optical,
                    cfg_.tracking_fov_horizontal_deg,cfg_.tracking_fov_vertical_deg,
                    range,cfg_.tracking_fov_range_margin,cfg_.tracking_fov_front_margin);
            };
            auto fov = sample(target);
            // Include the ground contact and a conservative target width,
            // instead of allowing a visible center with a clipped bbox bottom.
            for (const Vec3f &offset : std::vector<Vec3f>{
                    Vec3f(0,0,-cfg_.tracking_target_half_height),
                    Vec3f(0,0,cfg_.tracking_target_half_height),
                    Vec3f(cfg_.tracking_target_half_width,0,0),
                    Vec3f(-cfg_.tracking_target_half_width,0,0),
                    Vec3f(0,cfg_.tracking_target_half_width,0),
                    Vec3f(0,-cfg_.tracking_target_half_width,0)}) {
                const auto edge = sample(target+offset);
                fov.inside = fov.inside && edge.inside;
                fov.h_violation = std::max(fov.h_violation,edge.h_violation);
                fov.v_violation = std::max(fov.v_violation,edge.v_violation);
                fov.range_violation = std::max(fov.range_violation,edge.range_violation);
                fov.front_violation = std::max(fov.front_violation,edge.front_violation);
            }
            const bool violated = !fov.inside;
            if (total_sample_count == 0) {
                initial_distance = fov.distance;
            }
            final_distance = fov.distance;
            if (violated) {
                total_max_h_violation = std::max(total_max_h_violation, fov.h_violation);
                total_max_v_violation = std::max(total_max_v_violation, fov.v_violation);
                total_max_range_violation = std::max(total_max_range_violation, fov.range_violation);
                total_max_front_violation = std::max(total_max_front_violation, fov.front_violation);
            }
            ++total_sample_count;

        }

        // Angular visibility and observation distance have distinct semantics.
        // Approach progress is checked separately on the NEW executable suffix;
        // the already committed braking prefix must still remain in view.
        const double angular_grace = allow_keep_old_grace
            ? std::max(0.0, cfg_.tracking_fov_keep_old_angular_grace_deg) * kDegToRad
            : (allow_reacquire_range_grace ? reacquire_angular_grace_rad : 0.0);
        if (total_max_h_violation > angular_grace + 1.e-6 ||
            total_max_v_violation > angular_grace + 1.e-6 ||
            total_max_front_violation > 1.e-6) {
            setFailureReason(reason, fmt::format(
                "ANGULAR_FOV: max_h={:.2f}deg, max_v={:.2f}deg, max_front={:.3f}, horizon={:.3f}",
                total_max_h_violation / kDegToRad, total_max_v_violation / kDegToRad,
                total_max_front_violation, eval_horizon));
            return false;
        }
        const bool approach_range = allow_reacquire_range_grace &&
            cfg_.tracking_reacquire_fov_relax_enable;
        const double range_grace = cfg_.tracking_fov_range_grace_enable
            ? std::max(0.0, cfg_.tracking_fov_range_grace) : 0.0;
        if (!approach_range && total_max_range_violation > range_grace + 1.e-6) {
            setFailureReason(reason, fmt::format(
                "RANGE: max_violation={:.3f}, grace={:.3f}, initial={:.3f}, final={:.3f}",
                total_max_range_violation, range_grace, initial_distance, final_distance));
            return false;
        }

        return true;
    }

    double GeneralPlanner::trackingViewpointErrorScore(const Vec3f &tracker,
                                                       const Vec3f &target) const {
        return trackingDistanceError(tracker,
                                     target,
                                     cfg_.tracking_distance,
                                     cfg_.tracking_height_offset);
    }

    bool GeneralPlanner::trackingCommitPassesAntiRollback(
            const Trajectory &candidate_pos_traj,
            const traj_opt::DynamicTargetStates &target_prediction,
            const double commit_wt,
            const double candidate_eval_start_t,
            const double target_eval_start_t,
            const bool candidate_safe,
            const bool candidate_fov_ok,
            int *worse_count_out,
            double *max_regression_out,
            std::string *reason) {
        if (worse_count_out != nullptr) {
            *worse_count_out = 0;
        }
        if (max_regression_out != nullptr) {
            *max_regression_out = 0.0;
        }
        if (!cfg_.tracking_anti_rollback_enable) {
            return true;
        }
        if (candidate_pos_traj.empty() || target_prediction.empty()) {
            setFailureReason(reason, "empty candidate or target prediction");
            return false;
        }
        if (cmd_traj_info_.empty()) {
            return true;
        }

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const Trajectory old_yaw_traj = cmd_traj_info_.yawTraj();
        const double old_start_wt = cmd_traj_info_.getStartWallTime();
        const double old_total_dur = cmd_traj_info_.getTotalDuration();
        cmd_traj_info_.unlock();

        // Compare both commands at the same absolute time after the frozen prefix.
        const double old_eval_t = commit_wt - old_start_wt + std::max(0.0, candidate_eval_start_t);
        if (old_pos_traj.empty() || old_eval_t < -1.0e-6) {
            return true;
        }

        const double dt = std::max(0.05, cfg_.tracking_anti_rollback_dt);
        const double candidate_start =
                std::clamp(candidate_eval_start_t, 0.0, candidate_pos_traj.getTotalDuration());
        const double target_start = std::max(0.0, target_eval_start_t);
        const double horizon =
                std::min({std::max(0.0, cfg_.tracking_anti_rollback_horizon),
                          std::max(0.0, old_total_dur - old_eval_t),
                          std::max(0.0, candidate_pos_traj.getTotalDuration() - candidate_start),
                          std::max(0.0, target_prediction.back().t - target_start)});
        if (horizon < dt + 1.0e-6) {
            return true;
        }
        if (!trackingTrajectorySafeForHorizon(old_pos_traj, old_eval_t, horizon, dt)) {
            return true;
        }

        const double margin = std::max(0.0, cfg_.tracking_anti_rollback_margin);
        int worse_count = 0;
        double max_regression = 0.0;
        double max_new_score = 0.0;
        for (double offset = dt; offset <= horizon + 1.0e-6; offset += dt) {
            const Vec3f old_pos =
                    old_pos_traj.getPos(std::clamp(old_eval_t + offset,
                                                   0.0,
                                                   old_pos_traj.getTotalDuration()));
            const Vec3f new_pos =
                    candidate_pos_traj.getPos(std::clamp(candidate_start + offset,
                                                         0.0,
                                                         candidate_pos_traj.getTotalDuration()));
            const Vec3f target_pos =
                    interpolateTargetPrediction(target_prediction, target_start + offset).position;
            const double old_score = trackingViewpointErrorScore(old_pos, target_pos);
            const double new_score = trackingViewpointErrorScore(new_pos, target_pos);
            max_new_score = std::max(max_new_score, new_score);
            const double regression = new_score - old_score;
            if (regression > margin) {
                ++worse_count;
                max_regression = std::max(max_regression, regression);
            }
        }
        if (worse_count_out != nullptr) {
            *worse_count_out = worse_count;
        }
        if (max_regression_out != nullptr) {
            *max_regression_out = max_regression;
        }

        if (worse_count >= 2 || max_regression > 2.0 * margin) {
            TrackingTrajectoryActivity old_activity =
                    evaluateTrackingTrajectoryActivity(old_pos_traj,
                                                       std::clamp(old_eval_t, 0.0, old_total_dur),
                                                       target_prediction,
                                                       cfg_.tracking_keep_old_horizon,
                                                       cfg_.tracking_keep_old_safety_dt);
            std::string old_fov_reason;
            const bool old_fov_ok =
                    old_yaw_traj.empty()
                        ? false
                        : trackingSnapshotSatisfiesFovForKeepOld(old_pos_traj,
                                                                 old_yaw_traj,
                                                                 std::clamp(old_eval_t, 0.0, old_total_dur),
                                                                 target_prediction,
                                                                 &old_fov_reason);
            const double grace_horizon =
                    std::min({std::max(0.0, cfg_.tracking_detour_grace_horizon),
                              std::max(0.0, candidate_pos_traj.getTotalDuration() - candidate_start),
                              std::max(0.0, target_prediction.back().t - target_start)});
            const TrackingMotionMetrics candidate_metrics =
                    computeTrackingMotionMetrics(candidate_pos_traj,
                                                 target_prediction,
                                                 cfg_,
                                                 candidate_start,
                                                 target_start,
                                                 grace_horizon);
            const double min_positive_progress =
                    std::max(0.02,
                             candidate_metrics.target_speed_3d *
                             grace_horizon *
                             std::max(0.0, cfg_.tracking_keep_old_min_progress_3d_ratio));
            const bool positive_3d_progress =
                    candidate_metrics.progress_3d >= min_positive_progress ||
                    candidate_metrics.displacement_3d >= cfg_.tracking_no_motion_min_displacement ||
                    candidate_metrics.displacement_z >= cfg_.tracking_no_motion_min_displacement_z;
            const bool old_unusable =
                    !old_activity.active ||
                    old_activity.remaining < cfg_.tracking_keep_old_min_remaining ||
                    !old_fov_ok;
            const double detour_error_limit =
                    cfg_.tracking_detour_max_tracking_error_scale *
                    std::max(0.1, cfg_.tracking_distance_tolerance);
            if (cfg_.tracking_detour_grace_enable &&
                candidate_safe &&
                candidate_fov_ok &&
                positive_3d_progress &&
                old_unusable &&
                max_new_score <= detour_error_limit) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_FORCE_COMMIT_SAFE_RECOVERY reason=detour_grace, worse_count={}, max_regression={:.3f}, max_new_error={:.3f}, error_limit={:.3f}, candidate_safe={}, candidate_fov_ok={}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_progress_3d={:.3f}, old_active={}, old_remaining={:.3f}, old_activity_reason={}, old_fov_ok={}, old_fov_reason={}, prefix_duration={:.3f}, target_eval_start={:.3f}",
                                   worse_count,
                                   max_regression,
                                   max_new_score,
                                   detour_error_limit,
                                   candidate_safe,
                                   candidate_fov_ok,
                                   candidate_metrics.displacement_xy,
                                   candidate_metrics.displacement_z,
                                   candidate_metrics.displacement_3d,
                                   candidate_metrics.progress_3d,
                                   old_activity.active,
                                   old_activity.remaining,
                                   old_activity.reason,
                                   old_fov_ok,
                                   old_fov_reason,
                                   candidate_start,
                                   target_start);
                }
                return true;
            }
            setFailureReason(reason,
                             fmt::format("anti-rollback regression: worse_count={}, max_regression={:.3f}, max_new_error={:.3f}, old_activity={}, old_fov_ok={}",
                                         worse_count,
                                         max_regression,
                                         max_new_score,
                                         old_activity.reason,
                                         old_fov_ok));
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_ANTI_ROLLBACK reject_count={}, keep_old_count={}, worse_count={}, max_regression={:.3f}, max_new_error={:.3f}, horizon={:.2f}, prefix_duration={:.3f}, target_eval_start={:.3f}, candidate_safe={}, candidate_fov_ok={}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_progress_3d={:.3f}, old_remaining={:.3f}, old_activity_reason={}, old_fov_ok={}, old_fov_reason={}, old_speed_xy={:.3f}, old_speed_z={:.3f}, old_speed_3d={:.3f}, old_disp_xy={:.3f}, old_disp_z={:.3f}, old_disp_3d={:.3f}, old_progress_xy={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               tracking_runtime_manager_->consecutiveReject(),
                               tracking_runtime_manager_->consecutiveKeepOld(),
                               worse_count,
                               max_regression,
                               max_new_score,
                               horizon,
                               candidate_start,
                               target_start,
                               candidate_safe,
                               candidate_fov_ok,
                               candidate_metrics.displacement_xy,
                               candidate_metrics.displacement_z,
                               candidate_metrics.displacement_3d,
                               candidate_metrics.progress_3d,
                               old_activity.remaining,
                               old_activity.reason,
                               old_fov_ok,
                               old_fov_reason,
                               old_activity.speed_xy,
                               old_activity.speed_z,
                               old_activity.speed_3d,
                               old_activity.displacement_xy,
                               old_activity.displacement_z,
                               old_activity.displacement_3d,
                               old_activity.progress_xy,
                               old_activity.progress_3d,
                               old_activity.expected_progress,
                               old_activity.avg_tracking_error);
            } else {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking commit rejected by anti-rollback gate: worse_count={}, max_regression={:.3f}, horizon={:.2f}s.",
                               worse_count,
                               max_regression,
                               horizon);
            }
            return false;
        }
        return true;
    }

} // namespace general_planner
