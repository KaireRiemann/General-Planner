#include "general_core/tracking/tracking_perching_transition_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace general_planner {
namespace {

bool finiteSurface(const traj_opt::PerchingSurfaceState &surface)
{
    return surface.position.allFinite() &&
           surface.velocity.allFinite() &&
           surface.acceleration.allFinite() &&
           surface.surface_x.allFinite() &&
           surface.surface_y.allFinite() &&
           surface.surface_z.allFinite() &&
           std::isfinite(surface.yaw) &&
           std::isfinite(surface.yaw_rate) &&
           surface.surface_z.norm() > 1.0e-6;
}

const char *statusName(const TrackingPerchingTransitionManager::Status status)
{
    switch (status) {
    case TrackingPerchingTransitionManager::Status::TRACKING_ONLY:
        return "TRACKING_ONLY";
    case TrackingPerchingTransitionManager::Status::PERCHING_REQUESTED:
        return "PERCHING_REQUESTED";
    case TrackingPerchingTransitionManager::Status::PERCHING_READY:
        return "PERCHING_READY";
    case TrackingPerchingTransitionManager::Status::PERCHING_CANDIDATE_TESTING:
        return "PERCHING_CANDIDATE_TESTING";
    case TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED:
        return "PERCHING_COMMITTED";
    case TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING:
        return "PERCHING_EXECUTING";
    case TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT:
        return "CONTACT_IMMINENT";
    case TrackingPerchingTransitionManager::Status::CONTACT:
        return "CONTACT";
    case TrackingPerchingTransitionManager::Status::ABORT_TO_TRACKING:
        return "ABORT_TO_TRACKING";
    }
    return "UNKNOWN";
}

void logReadiness(const TrackingPerchingTransitionManager::Readiness &r,
                  const TrackingPerchingTransitionManager::Status status)
{
    std::cout << " -- [TrackingPerching] TRACKING_PERCHING_READINESS"
              << " status=" << statusName(status)
              << ", request=" << r.request
              << ", tracking_active=" << r.tracking_active
              << ", surface_valid=" << r.surface_valid
              << ", relative_state_good=" << r.relative_state_good
              << ", duration_good=" << r.duration_good
              << ", prediction_good=" << r.prediction_good
              << ", distance=" << r.distance
              << ", relative_speed=" << r.relative_speed
              << ", lateral_speed=" << r.lateral_speed
              << ", estimated_duration=" << r.estimated_duration
              << ", ready_count=" << r.ready_count
              << ", reason=" << r.reason << std::endl;
}

} // namespace

void TrackingPerchingTransitionManager::reset()
{
    status_ = Status::TRACKING_ONLY;
    perching_request_ = false;
    ready_count_ = 0;
}

void TrackingPerchingTransitionManager::setPerchingRequest(const bool request)
{
    if (request && !perching_request_) {
        std::cout << " -- [TrackingPerching] TRACKING_PERCHING_REQUESTED" << std::endl;
    }
    perching_request_ = request;
    if (!request) {
        ready_count_ = 0;
        if (status_ != Status::PERCHING_COMMITTED &&
            status_ != Status::PERCHING_EXECUTING &&
            status_ != Status::CONTACT_IMMINENT &&
            status_ != Status::CONTACT) {
            status_ = Status::TRACKING_ONLY;
        }
    } else if (status_ == Status::TRACKING_ONLY) {
        status_ = Status::PERCHING_REQUESTED;
    }
}

bool TrackingPerchingTransitionManager::perchingRequested() const
{
    return perching_request_;
}

TrackingPerchingTransitionManager::Readiness
TrackingPerchingTransitionManager::evaluateReadiness(
        const bool tracking_active,
        const geometry_utils::Trajectory &tracking_pos,
        const geometry_utils::Trajectory &tracking_yaw,
        const double tracking_local_t,
        const traj_opt::PerchingSurfaceState &surface,
        const Config &cfg)
{
    Readiness out;
    out.request = perching_request_ ||
                  cfg.tracking_perching_auto_trigger_enable ||
                  !cfg.tracking_perching_require_external_request;
    out.tracking_active = tracking_active;

    auto reject = [&](const std::string &reason) {
        ready_count_ = 0;
        out.ready_count = ready_count_;
        out.reason = reason;
        if (status_ != Status::PERCHING_COMMITTED &&
            status_ != Status::PERCHING_EXECUTING &&
            status_ != Status::CONTACT_IMMINENT &&
            status_ != Status::CONTACT) {
            status_ = out.request ? Status::PERCHING_REQUESTED : Status::TRACKING_ONLY;
        }
        logReadiness(out, status_);
        return out;
    };

    if (!cfg.tracking_perching_enable) {
        return reject("tracking_perching_disabled");
    }
    if (cfg.tracking_perching_require_external_request && !perching_request_) {
        return reject("waiting_external_request");
    }
    if (!out.request) {
        return reject("no_perching_request");
    }
    if (!tracking_active) {
        return reject("tracking_not_active");
    }
    if (tracking_pos.empty()) {
        return reject("empty_tracking_trajectory");
    }
    const double total_t = tracking_pos.getTotalDuration();
    if (!std::isfinite(tracking_local_t) ||
        tracking_local_t < 0.0 ||
        tracking_local_t > total_t) {
        return reject("invalid_tracking_local_time");
    }
    (void)tracking_yaw;

    out.surface_valid = finiteSurface(surface);
    if (!out.surface_valid) {
        return reject("invalid_surface_state");
    }

    const Eigen::Vector3d surface_z = surface.surface_z.normalized();
    const Eigen::Vector3d head_p = tracking_pos.getPos(tracking_local_t);
    const Eigen::Vector3d head_v = tracking_pos.getVel(tracking_local_t);
    if (!head_p.allFinite() || !head_v.allFinite()) {
        return reject("nonfinite_tracking_head");
    }

    const Eigen::Vector3d contact_now =
            surface.position + std::max(0.0, cfg.perching_robot_l) * surface_z;
    const Eigen::Vector3d rel_p = head_p - contact_now;
    const Eigen::Vector3d rel_v = head_v - surface.velocity;
    out.distance = rel_p.norm();
    out.relative_speed = rel_v.norm();
    const Eigen::Vector3d lateral_v = rel_v - rel_v.dot(surface_z) * surface_z;
    out.lateral_speed = lateral_v.norm();
    out.estimated_duration =
            out.distance / std::max(0.5, cfg.tracking_to_perching_reference_speed);

    const bool distance_good =
            out.distance >= cfg.tracking_perching_readiness_min_distance &&
            out.distance <= cfg.tracking_perching_readiness_max_distance;
    const bool rel_speed_good =
            out.relative_speed <= cfg.tracking_perching_readiness_max_relative_speed;
    const bool lateral_speed_good =
            out.lateral_speed <= cfg.tracking_perching_readiness_max_lateral_speed;
    out.relative_state_good = distance_good && rel_speed_good && lateral_speed_good;
    out.duration_good =
            out.estimated_duration <= cfg.tracking_perching_readiness_max_required_duration;
    const double remaining_tracking = std::max(0.0, total_t - tracking_local_t);
    out.prediction_good =
            out.estimated_duration <= cfg.tracking_to_perching_max_seed_duration &&
            remaining_tracking >= cfg.tracking_perching_readiness_min_prediction_time;

    if (!out.relative_state_good) {
        if (!distance_good) {
            return reject("distance_out_of_range");
        }
        if (!rel_speed_good) {
            return reject("relative_speed_too_large");
        }
        return reject("lateral_speed_too_large");
    }
    if (!out.duration_good) {
        return reject("required_duration_too_large");
    }
    if (!out.prediction_good) {
        return reject("prediction_or_seed_duration_not_good");
    }

    ++ready_count_;
    out.ready_count = ready_count_;
    const int required_count =
            std::max(1, cfg.tracking_perching_readiness_hold_cycles);
    out.ready = ready_count_ >= required_count;
    out.reason = out.ready ? "ready" : "hold_readiness";
    status_ = out.ready ? Status::PERCHING_READY : Status::PERCHING_REQUESTED;
    logReadiness(out, status_);
    return out;
}

void TrackingPerchingTransitionManager::onCandidateTesting()
{
    status_ = Status::PERCHING_CANDIDATE_TESTING;
}

void TrackingPerchingTransitionManager::onPerchingCommitted()
{
    status_ = Status::PERCHING_EXECUTING;
    perching_request_ = false;
    ready_count_ = 0;
}

void TrackingPerchingTransitionManager::onContact()
{
    status_ = Status::CONTACT;
    perching_request_ = false;
    ready_count_ = 0;
}

void TrackingPerchingTransitionManager::onPerchingRejected()
{
    ready_count_ = 0;
    status_ = perching_request_ ? Status::PERCHING_REQUESTED : Status::TRACKING_ONLY;
}

void TrackingPerchingTransitionManager::onAbortToTracking()
{
    status_ = Status::ABORT_TO_TRACKING;
    perching_request_ = false;
    ready_count_ = 0;
}

TrackingPerchingTransitionManager::Status
TrackingPerchingTransitionManager::status() const
{
    return status_;
}

} // namespace general_planner
