#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

#include <data_structure/base/trajectory.h>
#include <general_core/config.hpp>
#include <traj_opt/tracking_perching_traj_opt.hpp>
#include <utils/geometry/geometry_utils.h>
#include <utils/header/eigen_alias.hpp>
#include <utils/optimization/polynomial_interpolation.h>

namespace general_planner {

inline traj_opt::DynamicTargetState interpolateTargetPrediction(
        const traj_opt::DynamicTargetStates &prediction,
        const double &t) {
    if (prediction.empty()) {
        return {};
    }
    if (prediction.size() == 1 || t <= prediction.front().t) {
        return prediction.front();
    }
    if (t >= prediction.back().t) {
        return prediction.back();
    }

    const auto it = std::lower_bound(prediction.begin(),
                                     prediction.end(),
                                     t,
                                     [](const traj_opt::DynamicTargetState &state, double query_t) {
                                         return state.t < query_t;
                                     });
    const int idx = static_cast<int>(std::distance(prediction.begin(), it));
    const auto &left = prediction[static_cast<std::size_t>(idx - 1)];
    const auto &right = prediction[static_cast<std::size_t>(idx)];
    const double alpha = (t - left.t) / std::max(1.0e-9, right.t - left.t);

    traj_opt::DynamicTargetState out;
    out.t = t;
    out.position = left.position + alpha * (right.position - left.position);
    out.velocity = left.velocity + alpha * (right.velocity - left.velocity);
    out.acceleration = left.acceleration + alpha * (right.acceleration - left.acceleration);
    out.yaw = left.yaw + alpha * (right.yaw - left.yaw);
    out.yaw_rate = left.yaw_rate + alpha * (right.yaw_rate - left.yaw_rate);
    return out;
}

inline double trackingHardSafeDistance(const Config &cfg) {
    return std::max(cfg.tracking_hard_safe_distance, cfg.robot_r + 0.02);
}

inline double trackingDistanceError(const general_utils::Vec3f &tracker,
                                    const general_utils::Vec3f &target,
                                    const double desired_distance,
                                    const double desired_height) {
    if (!tracker.allFinite() || !target.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    const general_utils::Vec3f rel = tracker - target;
    const double h_err = std::abs(rel.head<2>().norm() - desired_distance);
    const double z_err = std::abs(rel.z() - desired_height);
    return h_err + 0.5 * z_err;
}

inline general_utils::Vec3f trackingTargetDirection(
        const traj_opt::DynamicTargetStates &prediction,
        const double speed_threshold,
        const double vertical_threshold = 0.12,
        const bool motion_3d_enable = false) {
    if (prediction.empty()) {
        return general_utils::Vec3f::UnitX();
    }

    general_utils::Vec3f dir = prediction.front().velocity;
    if (!motion_3d_enable) {
        dir.z() = 0.0;
    }
    const double threshold =
            motion_3d_enable ? std::min(speed_threshold, vertical_threshold)
                             : speed_threshold;
    if (dir.norm() > threshold) {
        return dir.normalized();
    }

    if (prediction.size() >= 2) {
        dir = prediction.back().position - prediction.front().position;
        if (!motion_3d_enable) {
            dir.z() = 0.0;
        }
        if (dir.norm() > 1.0e-4) {
            return dir.normalized();
        }
    }

    return general_utils::Vec3f::UnitX();
}

struct TrackingMotionMetrics {
    double speed_xy{0.0};
    double speed_z{0.0};
    double speed_3d{0.0};
    double displacement_xy{0.0};
    double displacement_z{0.0};
    double displacement_3d{0.0};
    double progress_xy{0.0};
    double progress_3d{0.0};
    double target_speed_xy{0.0};
    double target_speed_z{0.0};
    double target_speed_3d{0.0};
    bool target_vertical_moving{false};
    bool target_moving{false};
};

inline TrackingMotionMetrics computeTrackingMotionMetrics(
        const geometry_utils::Trajectory &traj,
        const traj_opt::DynamicTargetStates &target_prediction,
        const Config &cfg,
        const double candidate_eval_start_t,
        const double target_eval_start_t,
        const double horizon) {
    TrackingMotionMetrics metrics;
    if (traj.empty() || target_prediction.empty()) {
        return metrics;
    }
    const double total = traj.getTotalDuration();
    const double start_t = std::clamp(candidate_eval_start_t, 0.0, total);
    const double target_start = std::max(0.0, target_eval_start_t);
    const double eval_horizon =
            std::min({std::max(0.0, horizon),
                      std::max(0.0, total - start_t),
                      std::max(0.0, target_prediction.back().t - target_start)});
    const double end_t = std::clamp(start_t + eval_horizon, 0.0, total);
    const general_utils::Vec3f p0 = traj.getPos(start_t);
    const general_utils::Vec3f p1 = traj.getPos(end_t);
    const general_utils::Vec3f v0 = traj.getVel(start_t);
    if (p0.allFinite() && p1.allFinite()) {
        const general_utils::Vec3f dp = p1 - p0;
        metrics.displacement_xy = dp.head<2>().norm();
        metrics.displacement_z = std::abs(dp.z());
        metrics.displacement_3d = dp.norm();
        const general_utils::Vec3f target_dir =
                trackingTargetDirection(target_prediction,
                                        cfg.tracking_no_motion_target_speed_threshold,
                                        cfg.tracking_vertical_motion_threshold,
                                        cfg.tracking_motion_3d_enable);
        metrics.progress_xy = dp.head<2>().dot(target_dir.head<2>());
        metrics.progress_3d = dp.dot(target_dir);
    }
    if (v0.allFinite()) {
        metrics.speed_xy = v0.head<2>().norm();
        metrics.speed_z = std::abs(v0.z());
        metrics.speed_3d = v0.norm();
    }
    const auto target0 = interpolateTargetPrediction(target_prediction, target_start);
    metrics.target_speed_xy = target0.velocity.head<2>().norm();
    metrics.target_speed_z = std::abs(target0.velocity.z());
    metrics.target_speed_3d = target0.velocity.norm();
    double target_span_3d = 0.0;
    double target_span_z = 0.0;
    if (target_prediction.size() >= 2) {
        const auto target1 =
                interpolateTargetPrediction(target_prediction,
                                            std::min(target_prediction.back().t,
                                                     target_start + eval_horizon));
        const general_utils::Vec3f target_dp = target1.position - target0.position;
        target_span_3d = target_dp.norm();
        target_span_z = std::abs(target_dp.z());
    }
    metrics.target_vertical_moving =
            metrics.target_speed_z > cfg.tracking_vertical_motion_threshold ||
            target_span_z > cfg.tracking_no_motion_min_displacement_z;
    metrics.target_moving =
            metrics.target_speed_xy > cfg.tracking_no_motion_target_speed_threshold ||
            metrics.target_vertical_moving ||
            target_span_3d > std::max(cfg.tracking_no_motion_min_displacement,
                                      cfg.tracking_no_motion_min_displacement_z);
    return metrics;
}

inline bool buildYawPrefixFromSamples(const geometry_utils::Trajectory &yaw_traj,
                                      const double sample_start_t,
                                      const double sample_end_t,
                                      const double prefix_duration,
                                      geometry_utils::Trajectory &prefix_yaw) {
    if (yaw_traj.empty() ||
        prefix_duration <= 1.0e-5 ||
        sample_start_t < -1.0e-6 ||
        sample_end_t < sample_start_t - 1.0e-6) {
        return false;
    }

    const double total_duration = yaw_traj.getTotalDuration();
    if (sample_start_t > total_duration + 1.0e-6) {
        return false;
    }

    const double clamped_start = std::clamp(sample_start_t, 0.0, total_duration);
    const double clamped_end = std::clamp(sample_end_t, clamped_start, total_duration);
    general_utils::StatePVAJ start_state;
    general_utils::StatePVAJ end_state;
    if (!yaw_traj.getState(clamped_start, start_state) ||
        !yaw_traj.getState(clamped_end, end_state)) {
        return false;
    }

    Eigen::Matrix<double, 1, 2> init_state;
    Eigen::Matrix<double, 1, 2> goal_state;
    init_state << start_state(0, 0), start_state(0, 1);
    goal_state << end_state(0, 0), end_state(0, 1);
    geometry_utils::normalizeNextYaw(init_state(0, 0), goal_state(0, 0));

    Eigen::Matrix<double, 1, -1> waypoints(1, 0);
    general_utils::VecDf times(1);
    times(0) = prefix_duration;
    prefix_yaw = poly_interpo::minimumAccInterpolation<1>(init_state,
                                                          goal_state,
                                                          waypoints,
                                                          times);
    prefix_yaw.start_WT = yaw_traj.start_WT + clamped_start;
    return !prefix_yaw.empty();
}

inline bool extractYawPrefixForStitching(const geometry_utils::Trajectory &tracking_yaw,
                                         const double prefix_start,
                                         const double prefix_duration,
                                         geometry_utils::Trajectory &prefix_yaw,
                                         bool &used_sampled_fallback) {
    used_sampled_fallback = false;
    if (tracking_yaw.empty() || prefix_duration <= 1.0e-5) {
        return false;
    }

    const double yaw_total = tracking_yaw.getTotalDuration();
    if (prefix_start < -1.0e-6 || prefix_start > yaw_total + 1.0e-6) {
        return false;
    }

    const double prefix_end = prefix_start + prefix_duration;
    const double sample_end = std::min(prefix_end, yaw_total);
    const double query_t = std::clamp(prefix_start, 0.0, std::max(0.0, yaw_total - 1.0e-7));
    double local_query_t = query_t;
    const int piece_idx = tracking_yaw.locatePieceIdx(local_query_t);
    const int degree = tracking_yaw[piece_idx].getDegree();
    if ((degree == 3 || degree == 5 || degree == 7) &&
        sample_end > prefix_start + 1.0e-5 &&
        tracking_yaw.getPartialTrajectoryByTime(prefix_start, sample_end, prefix_yaw)) {
        if (std::abs(prefix_yaw.getTotalDuration() - prefix_duration) > 1.0e-4) {
            return buildYawPrefixFromSamples(tracking_yaw,
                                             prefix_start,
                                             sample_end,
                                             prefix_duration,
                                             prefix_yaw);
        }
        return true;
    }

    used_sampled_fallback = true;
    return buildYawPrefixFromSamples(tracking_yaw,
                                     prefix_start,
                                     sample_end,
                                     prefix_duration,
                                     prefix_yaw);
}

inline double yawFacingTarget(const geometry_utils::Trajectory &pos_traj,
                              const traj_opt::DynamicTargetStates &target_prediction,
                              const double &t,
                              const double &last_yaw) {
    const double eval_t = std::clamp(t, 0.0, pos_traj.getTotalDuration());
    const general_utils::Vec3f tracker_p = pos_traj.getPos(eval_t);
    const general_utils::Vec3f target_p =
            interpolateTargetPrediction(target_prediction, eval_t).position;
    const general_utils::Vec3f face_dir = target_p - tracker_p;
    double yaw = last_yaw;
    if (face_dir.head<2>().norm() > 1.0e-4) {
        yaw = std::atan2(face_dir.y(), face_dir.x());
        geometry_utils::normalizeNextYaw(last_yaw, yaw);
    }
    return yaw;
}

} // namespace general_planner
