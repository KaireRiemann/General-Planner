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
        offset >= prediction.back().t) return out;
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
    const double vcap = std::max(cfg.tracking_traj_cfg.max_vel,head.col(1).norm())*1.05;
    const double acap = std::max(cfg.tracking_traj_cfg.max_acc,head.col(2).norm())*1.05;
    const double yvcap = std::max(cfg.tracking_yaw_rate_limit,std::abs(head_yaw(0,1)))+0.02;
    const double yacap = std::max(cfg.tracking_yaw_acceleration_limit,std::abs(head_yaw(0,2)))+0.02;
    const double v=pos.getMaxVelRate(),a=pos.getMaxAccRate();
    const double yv=yaw.getMaxVelRate(),ya=yaw.getMaxAccRate();
    out.valid=head.allFinite() && head_yaw.allFinite() && std::isfinite(v) && std::isfinite(a) &&
        std::isfinite(yv) && std::isfinite(ya) && v<=vcap && a<=acap && yv<=yvcap && ya<=yacap;
    out.reason=out.valid ? "ok" : fmt::format(
        "DYNAMIC_LIMIT: velocity={:.3f}/{:.3f};acceleration={:.3f}/{:.3f};yaw_rate={:.3f}/{:.3f};yaw_acc={:.3f}/{:.3f}",
        v,vcap,a,acap,yv,yvcap,ya,yacap);
    return out;
}

inline traj_opt::DynamicTargetState interpolateTargetPrediction(
        const traj_opt::DynamicTargetStates &prediction,
        const double &t) {
    return traj_opt::sampleTrackingTarget(prediction, t);
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
