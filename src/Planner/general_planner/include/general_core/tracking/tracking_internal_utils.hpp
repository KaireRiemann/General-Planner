#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

#include <data_structure/base/trajectory.h>
#include <general_core/config.hpp>
#include <traj_opt/tracking_problem.hpp>
#include <utils/geometry/geometry_utils.h>
#include <utils/geometry/tracking_attitude.hpp>
#include <fmt/format.h>
#include <utils/header/eigen_alias.hpp>
#include <utils/optimization/polynomial_interpolation.h>

namespace general_planner {

inline double trackingAdaptiveFovRange(const double configured_range,
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

inline double trackingAdaptiveFovRange(const Config &cfg) {
    return trackingAdaptiveFovRange(cfg.tracking_fov_range,
                                    cfg.tracking_distance,
                                    cfg.tracking_distance_upper_tolerance,
                                    cfg.tracking_distance_tolerance,
                                    cfg.tracking_height_offset,
                                    cfg.tracking_height_tolerance,
                                    cfg.tracking_fov_horizontal_deg,
                                    cfg.tracking_fov_vertical_deg);
}

// Rebase only the observed interval. Never turn extrapolation into a trusted
// prediction or restart its lifetime on each planner call.
inline traj_opt::DynamicTargetStates trackingPredictionAtTime(
        const traj_opt::DynamicTargetStates &prediction, double reference_time) {
    traj_opt::DynamicTargetStates out;
    if (prediction.empty() || !std::isfinite(reference_time)) return out;
    for (std::size_t i = 0; i < prediction.size(); ++i) {
        if (!std::isfinite(prediction[i].t) || !prediction[i].position.allFinite() ||
            !prediction[i].velocity.allFinite() ||
            (i > 0 && prediction[i].t <= prediction[i-1].t)) return out;
    }
    const double epoch = std::isfinite(prediction.front().reference_time)
        ? prediction.front().reference_time : reference_time;
    const double offset = reference_time - epoch;
    if (offset < prediction.front().t - 1.e-6 ||
        offset >= prediction.back().t - 1.e-3) return out;
    out.emplace_back(traj_opt::sampleTrackingTarget(prediction, offset));
    out.back().t = 0.0;
    out.back().reference_time = reference_time;
    for (const auto &sample : prediction) {
        if (sample.t <= offset + 1.e-6) continue;
        out.emplace_back(sample);
        out.back().t -= offset;
        out.back().reference_time = reference_time;
    }
    return out;
}

struct TrackingDynamicsCheck {
    bool valid{false};
    std::string reason;
};

// Shared by candidate and final stitched-command validation.
inline TrackingDynamicsCheck checkTrackingDynamics(
        const geometry_utils::Trajectory &pos,
        const geometry_utils::Trajectory &yaw, const Config &cfg) {
    TrackingDynamicsCheck out;
    if (pos.empty() || yaw.empty() ||
        std::abs(pos.getTotalDuration() - yaw.getTotalDuration()) > 1.e-3) {
        out.reason = "DYNAMIC_LIMIT: empty or mismatched position/yaw duration";
        return out;
    }
    const auto head = pos.getState(0.0);
    const auto head_yaw = yaw.getState(0.0);
    const geometry_utils::TrackingAttitude head_frame(head.col(2), head_yaw(0,0));
    const double initial_rate = std::sqrt(head_frame.bodyRateSquared(head.col(3), head_yaw(0,1)));
    const double vcap = std::max(cfg.tracking_traj_cfg.max_vel, head.col(1).norm()) * 1.05;
    const double acap = std::max(cfg.tracking_traj_cfg.max_acc, head.col(2).norm()) * 1.05;
    const double jcap = std::max(cfg.tracking_traj_cfg.max_jerk, head.col(3).norm()) * 1.05;
    const double tiltcap = std::max(cfg.tracking_traj_cfg.max_tilt,
        std::atan2(head.col(2).head<2>().norm(), 9.81 + head(2,2))) + 0.01;
    const double ratecap = std::max(cfg.tracking_traj_cfg.max_omg, initial_rate) + 0.02;
    const double yvcap = std::max(cfg.tracking_yaw_rate_limit, std::abs(head_yaw(0,1))) + 0.02;
    const double yacap = std::max(cfg.tracking_yaw_acceleration_limit, std::abs(head_yaw(0,2))) + 0.02;
    const double v = pos.getMaxVelRate(), a = pos.getMaxAccRate();
    const double yv = yaw.getMaxVelRate(), ya = yaw.getMaxAccRate();
    double j = 0.0, tilt = 0.0, rate = 0.0;
    double min_thrust = std::numeric_limits<double>::infinity(), max_thrust = 0.0;
    bool finite = head.allFinite() && head_yaw.allFinite() && head_frame.valid &&
        std::isfinite(initial_rate) && std::isfinite(v) && std::isfinite(a) &&
        std::isfinite(yv) && std::isfinite(ya);
    // Include every piece boundary as well as regular interior samples.
    double begin = 0.0;
    for (const auto &piece : pos) {
        const double duration = piece.getDuration();
        if (!std::isfinite(duration) || duration <= 0.0) { finite = false; break; }
        const int count = std::max(1, static_cast<int>(std::ceil(duration / 0.02)));
        for (int i = 0; i <= count; ++i) {
            const double t = begin + duration * i / count;
            const auto state = pos.getState(t);
            const auto ys = yaw.getState(t);
            const geometry_utils::TrackingAttitude frame(state.col(2), ys(0,0));
            const double omega = std::sqrt(frame.bodyRateSquared(state.col(3), ys(0,1)));
            finite = finite && state.allFinite() && ys.allFinite() && frame.valid && std::isfinite(omega);
            const double thrust = (state.col(2) + Eigen::Vector3d(0,0,9.81)).norm();
            min_thrust = std::min(min_thrust, thrust);
            max_thrust = std::max(max_thrust, thrust);
            j = std::max(j, state.col(3).norm());
            tilt = std::max(tilt, std::atan2(state.col(2).head<2>().norm(), 9.81 + state(2,2)));
            rate = std::max(rate, omega);
        }
        begin += duration;
    }
    const bool thrust_valid = cfg.tracking_traj_cfg.max_acc_thr <= 0.0 ||
        (min_thrust >= 0.95 * cfg.tracking_traj_cfg.min_acc_thr &&
         max_thrust <= 1.05 * cfg.tracking_traj_cfg.max_acc_thr);
    out.valid = finite && thrust_valid && v <= vcap && a <= acap && j <= jcap &&
        tilt <= tiltcap && rate <= ratecap && yv <= yvcap && ya <= yacap;
    out.reason = fmt::format("DYNAMIC_LIMIT: finite={}|velocity={:.4f}/{:.4f}|acceleration={:.4f}/{:.4f}|jerk={:.4f}/{:.4f}|tilt={:.4f}/{:.4f}|body_rate={:.4f}/{:.4f}|yaw_rate={:.4f}/{:.4f}|yaw_acc={:.4f}/{:.4f}",
        finite, v, vcap, a, acap, j, jcap, tilt, tiltcap, rate, ratecap, yv, yvcap, ya, yacap);
    if (!thrust_valid) out.reason += fmt::format("|thrust=[{:.3f},{:.3f}]", min_thrust, max_thrust);
    return out;
}

inline traj_opt::DynamicTargetState interpolateTargetPrediction(
        const traj_opt::DynamicTargetStates &prediction,
        const double &t) {
    return traj_opt::sampleTrackingTarget(prediction, t);
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

// Positive progress over a finite, trusted prediction window. Range may grow
// while a vehicle accelerates toward a faster receding target; angular FOV,
// collision and dynamic feasibility remain independent mandatory checks.
struct TrackingApproachMotion {
    bool valid{false};
    double progress{0.0};
    double required_progress{0.0};
    double end_speed{0.0};
};

inline TrackingApproachMotion trackingApproachMotion(
        const geometry_utils::Trajectory &traj,
        const traj_opt::DynamicTargetStates &prediction,
        const Config &cfg, double start_t, double target_start_t, double horizon,
        const general_utils::vec_Vec3f *guide = nullptr) {
    TrackingApproachMotion out;
    if (traj.empty() || prediction.empty()) return out;
    start_t = std::clamp(start_t, 0.0, traj.getTotalDuration());
    const double h = std::min({horizon, traj.getTotalDuration() - start_t,
                              prediction.back().t - target_start_t});
    if (!std::isfinite(h) || h < 0.15) return out;
    const auto target = interpolateTargetPrediction(prediction, target_start_t);
    auto dir = (target.position - traj.getPos(start_t)).eval();
    // Use the beginning of the verified local route, including a side-pass.
    // A distant terminal chord can point opposite to the required first turn.
    if (guide && guide->size() >= 2) {
        std::size_t segment_id = 1;
        general_utils::Vec3f route_start = guide->front();
        double nearest = std::numeric_limits<double>::infinity();
        const auto current = traj.getPos(start_t);
        for (std::size_t i = 1; i < guide->size(); ++i) {
            const auto delta = ((*guide)[i] - (*guide)[i-1]).eval();
            if (delta.squaredNorm() < 1.e-10) continue;
            const double alpha = std::clamp((current-(*guide)[i-1]).dot(delta)/delta.squaredNorm(), 0.0, 1.0);
            const auto point = ((*guide)[i-1] + alpha*delta).eval();
            const double distance = (point-current).squaredNorm();
            if (distance < nearest) { nearest = distance; route_start = point; segment_id = i; }
        }
        double arc = 0.0;
        general_utils::Vec3f previous = route_start;
        for (std::size_t i = segment_id; i < guide->size(); ++i) {
            const auto segment = ((*guide)[i] - previous).eval();
            const double length = segment.norm();
            if (length < 1.e-5) continue;
            const double step = std::min(length, 0.5 - arc);
            dir = previous + segment * (step / length) - route_start;
            arc += step;
            previous = (*guide)[i];
            if (arc >= 0.5 - 1.e-5) break;
        }
    }
    if (!cfg.tracking_motion_3d_enable) dir.z() = 0.0;
    if (!dir.allFinite() || dir.norm() < 1.e-4) return out;
    dir.normalize();
    const auto dp = (traj.getPos(start_t + h) - traj.getPos(start_t)).eval();
    const auto v0 = traj.getVel(start_t);
    const auto v1 = traj.getVel(start_t + h);
    if (!dp.allFinite() || !v0.allFinite() || !v1.allFinite()) return out;
    out.progress = dp.dot(dir);
    out.end_speed = v1.dot(dir);
    const double speed0 = std::max(0.0, v0.dot(dir));
    const double a = std::max(0.1, cfg.tracking_traj_cfg.max_acc);
    const double j = std::max(0.1, cfg.tracking_traj_cfg.max_jerk);
    const double ramp = std::min(h, a / j);
    const double cruise = h - ramp;
    const double reachable = speed0 * h + j * ramp * ramp * ramp / 6.0 +
        0.5 * j * ramp * ramp * cruise + 0.5 * a * cruise * cruise;
    const double requested = std::max({0.15, cfg.tracking_no_motion_min_displacement,
        std::max(0.0, cfg.tracking_keep_old_min_progress_3d_ratio) * target.velocity.norm() * h});
    const double reachable_speed = speed0 + 0.5 * j * ramp * ramp + a * cruise;
    out.required_progress = std::max(0.001, std::min(requested, 0.1 * reachable));
    out.valid = out.progress >= out.required_progress &&
        out.end_speed >= std::min(std::max(0.05, cfg.tracking_keep_old_min_speed),
                                  0.1 * reachable_speed) &&
        out.progress / h >= std::min(0.02, 0.1 * reachable / h);
    return out;
}

inline bool trackingCandidateHasMotion(
        const geometry_utils::Trajectory &traj,
        const traj_opt::DynamicTargetStates &prediction,
        const Config &cfg, double start_t, double target_start_t) {
    if (traj.empty() || prediction.empty()) return false;
    start_t = std::clamp(start_t, 0.0, traj.getTotalDuration());
    const double h = std::min({std::max(0.0, cfg.tracking_no_motion_check_horizon),
                              traj.getTotalDuration() - start_t,
                              prediction.back().t - target_start_t});
    if (!std::isfinite(h) || h < 1.e-3 ||
        !traj.getPos(start_t).allFinite() || !traj.getPos(start_t + h).allFinite() ||
        !traj.getVel(start_t).allFinite()) return false;
    const auto metrics = computeTrackingMotionMetrics(traj, prediction, cfg, start_t, target_start_t, h);
    if (!metrics.target_moving) return true;
    const bool three_d = cfg.tracking_motion_3d_enable || metrics.target_vertical_moving;
    const double displacement = three_d ? metrics.displacement_3d : metrics.displacement_xy;
    const double speed = three_d ? metrics.speed_3d : metrics.speed_xy;
    const auto end_v = traj.getVel(start_t + h);
    if (!end_v.allFinite()) return false;
    const double end_speed = three_d ? end_v.norm() : end_v.head<2>().norm();
    // The same reachable-progress rule is used before and after stitching.
    general_utils::vec_Vec3f motion_guide{traj.getPos(start_t), traj.getPos(start_t + h)};
    return trackingApproachMotion(traj, prediction, cfg, start_t, target_start_t, h, &motion_guide).valid ||
        displacement >= std::max(0.01, cfg.tracking_no_motion_min_displacement) ||
        (displacement >= 0.01 && speed >= cfg.tracking_keep_old_min_speed) ||
        (displacement >= std::max(0.01, 0.5 * cfg.tracking_no_motion_min_displacement) &&
         end_speed >= cfg.tracking_keep_old_min_speed);
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
    prefix_yaw = geometry_utils::poly_interpo::minimumAccInterpolation<1>(init_state,
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

} // namespace general_planner
