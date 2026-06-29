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

using namespace general_utils;

namespace general_planner {

    bool GeneralPlanner::trackingTrajectorySafeForHorizon(const Trajectory &traj,
                                                          const double start_t,
                                                          const double horizon,
                                                          const double dt) const {
        if (traj.empty()) {
            return false;
        }
        if (horizon <= 1.0e-6) {
            return true;
        }
        const double total_dur = traj.getTotalDuration();
        if (!std::isfinite(start_t) ||
            start_t < -1.0e-6 ||
            start_t + horizon > total_dur + 1.0e-6) {
            return false;
        }
        if (map_manager_ == nullptr || !map_manager_->ready()) {
            return true;
        }

        const double safe_dt = std::max(0.05, dt);
        Vec3f last = traj.getPos(std::clamp(start_t, 0.0, total_dur));
        if (!last.allFinite()) {
            return false;
        }

        for (double offset = 0.0; offset <= horizon + 1.0e-6; offset += safe_dt) {
            const double t = std::clamp(start_t + offset, 0.0, total_dur);
            const Vec3f pos = traj.getPos(t);
            if (!pos.allFinite() || !map_manager_->insideLocalMap(pos)) {
                return false;
            }
            const auto grid_type = map_manager_->getInfGridType(pos);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP) {
                return false;
            }
            if (cfg_.tracking_unknown_as_occupied &&
                (grid_type == rog_map::GridType::UNKNOWN ||
                 grid_type == rog_map::GridType::UNDEFINED ||
                 grid_type == rog_map::GridType::FRONTIER)) {
                return false;
            }
            if (map_manager_->hasESDF()) {
                double dist = 0.0;
                Vec3f grad = Vec3f::Zero();
                if (map_manager_->evaluateESDF(pos, dist, grad) &&
                    dist < trackingHardSafeDistance(cfg_)) {
                    return false;
                }
            }
            if ((pos - last).norm() > 1.0e-4 &&
                !map_manager_->isLineFree(last, pos, true, cfg_.tracking_unknown_as_occupied)) {
                return false;
            }
            last = pos;
        }
        return true;
    }

    bool GeneralPlanner::currentTrackingTrajectorySafeForHorizon(const double horizon) {
        if (cmd_traj_info_.empty()) {
            return false;
        }

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const double old_start_wt = cmd_traj_info_.getStartWallTime();
        cmd_traj_info_.unlock();

        const double start_t = ros_ptr_->getSimTime() - old_start_wt;
        return trackingTrajectorySafeForHorizon(old_pos_traj,
                                                start_t,
                                                std::max(0.0, horizon),
                                                cfg_.tracking_keep_old_safety_dt);
    }

    bool GeneralPlanner::keepOldTrackingTrajectory(const std::string &reason) {
        const double keep_horizon = std::max(0.0, cfg_.tracking_keep_old_horizon);
        if (!currentTrackingTrajectorySafeForHorizon(keep_horizon)) {
            return false;
        }

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const Trajectory old_yaw_traj = cmd_traj_info_.yawTraj();
        cmd_traj_info_.unlock();

        latest_replan.setExpTraj(old_pos_traj);
        latest_replan.setExpYawTraj(old_yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        ros_ptr_->info(" -- [GeneralPlanner] {}; keep old tracking trajectory for {:.2f}s.",
                       reason,
                       keep_horizon);
        return true;
    }

    GeneralPlanner::TrackingTrajectoryActivity
    GeneralPlanner::evaluateTrackingTrajectoryActivity(
            const Trajectory &traj,
            const double local_start_t,
            const traj_opt::DynamicTargetStates &target_prediction,
            const double horizon,
            const double dt) const {
        TrackingTrajectoryActivity out;

        if (traj.empty() || target_prediction.empty()) {
            out.reason = "empty trajectory or target prediction";
            return out;
        }

        const double total_dur = traj.getTotalDuration();
        if (!std::isfinite(local_start_t) ||
            local_start_t < -1.0e-6 ||
            local_start_t > total_dur + 1.0e-6) {
            out.reason = "local_start_t outside duration";
            return out;
        }

        out.valid = true;
        out.remaining = std::max(0.0, total_dur - local_start_t);
        if (out.remaining < cfg_.tracking_keep_old_min_remaining) {
            out.reason = "remaining time too short";
            return out;
        }

        const double eval_horizon = std::min(std::max(0.0, horizon), out.remaining);
        const double safe_dt = std::max(0.03, dt);
        const auto target0 = interpolateTargetPrediction(target_prediction, 0.0);
        const Vec3f target_dir =
                trackingTargetDirection(target_prediction,
                                        cfg_.tracking_no_motion_target_speed_threshold,
                                        cfg_.tracking_vertical_motion_threshold,
                                        cfg_.tracking_motion_3d_enable);

        Vec3f last_p = traj.getPos(std::clamp(local_start_t, 0.0, total_dur));
        if (!last_p.allFinite()) {
            out.reason = "non-finite initial point";
            return out;
        }

        const TrackingMotionMetrics initial_metrics =
                computeTrackingMotionMetrics(traj,
                                             target_prediction,
                                             cfg_,
                                             local_start_t,
                                             0.0,
                                             eval_horizon);
        out.target_moving = initial_metrics.target_moving;
        out.target_vertical_moving = initial_metrics.target_vertical_moving;
        out.speed_xy = initial_metrics.speed_xy;
        out.speed_z = initial_metrics.speed_z;
        out.speed_3d = initial_metrics.speed_3d;
        out.speed0 = out.speed_xy;
        out.target_speed_xy = initial_metrics.target_speed_xy;
        out.target_speed_z = initial_metrics.target_speed_z;
        out.target_speed_3d = initial_metrics.target_speed_3d;
        out.safe = true;
        double total_error = 0.0;
        int sample_count = 0;

        for (double s = 0.0; s <= eval_horizon + 1.0e-6; s += safe_dt) {
            const double traj_t = std::min(total_dur, local_start_t + s);
            const Vec3f p = traj.getPos(traj_t);
            if (!p.allFinite()) {
                out.safe = false;
                out.reason = "non-finite sample";
                return out;
            }

            if (map_manager_ != nullptr && map_manager_->ready()) {
                if (!map_manager_->insideLocalMap(p)) {
                    out.safe = false;
                    out.reason = "outside local map";
                    return out;
                }

                const auto grid_type = map_manager_->getInfGridType(p);
                if (grid_type == rog_map::GridType::OCCUPIED ||
                    grid_type == rog_map::GridType::OUT_OF_MAP) {
                    out.safe = false;
                    out.reason = "occupied or out-of-map";
                    return out;
                }

                if (cfg_.tracking_unknown_as_occupied &&
                    (grid_type == rog_map::GridType::UNKNOWN ||
                     grid_type == rog_map::GridType::UNDEFINED ||
                     grid_type == rog_map::GridType::FRONTIER)) {
                    out.safe = false;
                    out.reason = "unknown treated as occupied";
                    return out;
                }

                if ((p - last_p).norm() > 1.0e-4 &&
                    !map_manager_->isLineFree(last_p, p, true, cfg_.tracking_unknown_as_occupied)) {
                    out.safe = false;
                    out.reason = "segment not line-free";
                    return out;
                }
            }

            const auto target = interpolateTargetPrediction(target_prediction, s);
            total_error += trackingDistanceError(p,
                                                 target.position,
                                                 cfg_.tracking_distance,
                                                 cfg_.tracking_height_offset);
            ++sample_count;

            if (s > 1.0e-6) {
                const Vec3f dp = p - last_p;
                out.displacement_xy += dp.head<2>().norm();
                out.displacement_z += std::abs(dp.z());
                out.displacement_3d += dp.norm();
                out.progress_xy += dp.head<2>().dot(target_dir.head<2>());
                out.progress_3d += dp.dot(target_dir);
            }
            last_p = p;
        }
        out.displacement = out.displacement_xy;
        out.progress = out.progress_xy;

        out.avg_tracking_error =
                sample_count > 0 ? total_error / static_cast<double>(sample_count) : 0.0;
        out.tracking_error =
                trackingDistanceError(traj.getPos(std::clamp(local_start_t, 0.0, total_dur)),
                                      target0.position,
                                      cfg_.tracking_distance,
                                      cfg_.tracking_height_offset);

        if (out.target_moving) {
            const bool use_3d_motion =
                    cfg_.tracking_motion_3d_enable || out.target_vertical_moving;
            const double active_speed =
                    use_3d_motion ? out.speed_3d : out.speed_xy;
            const double active_displacement =
                    use_3d_motion ? out.displacement_3d : out.displacement_xy;
            const double active_progress =
                    use_3d_motion ? out.progress_3d : out.progress_xy;
            const double target_speed =
                    use_3d_motion ? out.target_speed_3d : out.target_speed_xy;
            const double progress_ratio =
                    use_3d_motion ? cfg_.tracking_keep_old_min_progress_3d_ratio
                                  : cfg_.tracking_keep_old_min_progress_ratio;

            if (active_speed < cfg_.tracking_keep_old_min_speed) {
                out.reason = "speed too small";
                return out;
            }

            if (active_displacement < cfg_.tracking_keep_old_min_displacement &&
                (!out.target_vertical_moving ||
                 out.displacement_z < cfg_.tracking_no_motion_min_displacement_z)) {
                out.reason = "displacement too small";
                return out;
            }

            out.expected_progress =
                    target_speed *
                    eval_horizon *
                    progress_ratio;
            if (active_progress < out.expected_progress) {
                out.reason = "insufficient target-direction progress";
                return out;
            }

            const double max_err =
                    cfg_.tracking_keep_old_max_tracking_error_scale *
                    std::max(0.1, cfg_.tracking_distance_tolerance);
            if (out.avg_tracking_error > max_err) {
                out.reason = "tracking error too large";
                return out;
            }
        }

        out.active = true;
        out.reason = "safe and active";
        return out;
    }

    bool GeneralPlanner::currentTrackingTrajectorySafeAndActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            TrackingTrajectoryActivity *activity) const {
        if (cmd_traj_info_.empty()) {
            if (activity) {
                activity->reason = "empty committed trajectory";
            }
            return false;
        }

        Trajectory old_pos_traj;
        double old_start_wt = 0.0;
        double total_dur = 0.0;
        auto &mutable_cmd_traj = const_cast<CmdTraj &>(cmd_traj_info_);
        mutable_cmd_traj.lock();
        old_pos_traj = cmd_traj_info_.posTraj();
        old_start_wt = cmd_traj_info_.getStartWallTime();
        total_dur = cmd_traj_info_.getTotalDuration();
        mutable_cmd_traj.unlock();

        const double now = ros_ptr_->getSimTime();
        const double cur_t = std::clamp(now - old_start_wt, 0.0, total_dur);
        TrackingTrajectoryActivity local_activity =
                evaluateTrackingTrajectoryActivity(old_pos_traj,
                                                   cur_t,
                                                   target_prediction,
                                                   cfg_.tracking_keep_old_horizon,
                                                   cfg_.tracking_keep_old_safety_dt);
        if (activity) {
            *activity = local_activity;
        }

        return local_activity.valid &&
               local_activity.safe &&
               local_activity.active;
    }

    bool GeneralPlanner::candidateTrackingTrajectoryCommandable(
            const Trajectory &candidate_pos_traj,
            const traj_opt::DynamicTargetStates &target_prediction,
            const double candidate_eval_start_t,
            const double target_eval_start_t,
            std::string *reason) const {
        if (!cfg_.tracking_no_motion_guard_enable) {
            return true;
        }
        if (candidate_pos_traj.empty() || target_prediction.empty()) {
            if (reason) {
                *reason = "empty candidate or target prediction";
            }
            return false;
        }

        const double h =
                std::min(cfg_.tracking_no_motion_check_horizon,
                         std::max(0.0, candidate_pos_traj.getTotalDuration() -
                                        std::clamp(candidate_eval_start_t,
                                                   0.0,
                                                   candidate_pos_traj.getTotalDuration())));
        if (h < 1.0e-3) {
            if (reason) {
                *reason = "candidate duration too short";
            }
            return false;
        }

        const double start_t =
                std::clamp(candidate_eval_start_t, 0.0, candidate_pos_traj.getTotalDuration());
        const Vec3f p0 = candidate_pos_traj.getPos(start_t);
        const Vec3f p1 = candidate_pos_traj.getPos(start_t + h);
        const Vec3f v0 = candidate_pos_traj.getVel(start_t);
        if (!p0.allFinite() || !p1.allFinite() || !v0.allFinite()) {
            if (reason) {
                *reason = "candidate contains non-finite state";
            }
            return false;
        }

        const TrackingMotionMetrics metrics =
                computeTrackingMotionMetrics(candidate_pos_traj,
                                             target_prediction,
                                             cfg_,
                                             start_t,
                                             target_eval_start_t,
                                             h);
        if (!metrics.target_moving) {
            return true;
        }

        const bool use_3d_motion =
                cfg_.tracking_motion_3d_enable || metrics.target_vertical_moving;
        const bool commandable =
                use_3d_motion
                    ? (metrics.displacement_3d >= cfg_.tracking_no_motion_min_displacement ||
                       metrics.displacement_z >= cfg_.tracking_no_motion_min_displacement_z ||
                       metrics.speed_3d >= cfg_.tracking_keep_old_min_speed)
                    : (metrics.displacement_xy >= cfg_.tracking_no_motion_min_displacement ||
                       metrics.speed_xy >= cfg_.tracking_keep_old_min_speed);
        if (!commandable) {
            if (reason) {
                *reason = fmt::format("candidate no-motion: disp_xy={:.3f}, disp_z={:.3f}, disp_3d={:.3f}, speed_xy={:.3f}, speed_z={:.3f}, speed_3d={:.3f}, target_speed_xy={:.3f}, target_speed_z={:.3f}, target_speed_3d={:.3f}",
                                      metrics.displacement_xy,
                                      metrics.displacement_z,
                                      metrics.displacement_3d,
                                      metrics.speed_xy,
                                      metrics.speed_z,
                                      metrics.speed_3d,
                                      metrics.target_speed_xy,
                                      metrics.target_speed_z,
                                      metrics.target_speed_3d);
            }
            return false;
        }

        return true;
    }

} // namespace general_planner
