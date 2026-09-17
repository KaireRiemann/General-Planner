#include "general_core/tracking/tracking_runtime_manager.hpp"
#include "general_core/tracking/tracking_internal_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace general_planner {
namespace {

double effectiveTrackingHardSafeDistance(const Config &cfg)
{
    return std::max(cfg.tracking_hard_safe_distance, cfg.robot_r + 0.02);
}

} // namespace

TrackingRuntimeManager::TrackingRuntimeManager(const Config &cfg,
                                               const MapManager::Ptr &map_manager)
        : cfg_(cfg),
          map_manager_(map_manager)
{
}

void TrackingRuntimeManager::reset()
{
    consecutive_keep_old_ = 0;
    consecutive_reject_ = 0;
    status_ = Status::IDLE;
    has_committed_tracking_ = false;
    execution_start_ = observation_time_ = -1.0;
    execution_progress_observed_ = false;
    committed_guide_.clear();
}

general_utils::Vec3f TrackingRuntimeManager::targetDirection(
        const traj_opt::DynamicTargetStates &prediction) const
{
    if (prediction.empty()) {
        return general_utils::Vec3f::UnitX();
    }

    general_utils::Vec3f dir = prediction.front().velocity;
    if (!cfg_.tracking_motion_3d_enable) {
        dir.z() = 0.0;
    }
    const double speed_threshold =
            cfg_.tracking_motion_3d_enable
                ? std::max(0.0, cfg_.tracking_vertical_motion_threshold)
                : std::max(0.0, cfg_.tracking_no_motion_target_speed_threshold);
    if (dir.norm() > speed_threshold) {
        return dir.normalized();
    }

    if (prediction.size() >= 2) {
        dir = prediction.back().position - prediction.front().position;
        if (!cfg_.tracking_motion_3d_enable) {
            dir.z() = 0.0;
        }
        if (dir.norm() > 1.0e-4) {
            return dir.normalized();
        }
    }

    return general_utils::Vec3f::UnitX();
}

double TrackingRuntimeManager::trackingDistanceError(
        const general_utils::Vec3f &tracker,
        const general_utils::Vec3f &target) const
{
    if (!tracker.allFinite() || !target.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    const general_utils::Vec3f rel = tracker - target;
    const double horizontal_err =
            std::abs(rel.head<2>().norm() - cfg_.tracking_distance);
    const double height_err =
            std::abs(rel.z() - cfg_.tracking_height_offset);
    return horizontal_err + 0.5 * height_err;
}

bool TrackingRuntimeManager::trajectorySafe(const geometry_utils::Trajectory &traj,
                                            const double start_t,
                                            const double horizon,
                                            const double dt,
                                            std::string *reason) const
{
    if (traj.empty()) {
        if (reason) {
            *reason = "empty trajectory";
        }
        return false;
    }
    if (!std::isfinite(start_t) || !std::isfinite(horizon)) {
        if (reason) {
            *reason = "non-finite time";
        }
        return false;
    }

    const double total_dur = traj.getTotalDuration();
    if (start_t < -1.0e-6 || start_t > total_dur + 1.0e-6) {
        if (reason) {
            *reason = "start_t outside duration";
        }
        return false;
    }
    if (horizon <= 1.0e-6) {
        return true;
    }

    const double eval_start = std::clamp(start_t, 0.0, total_dur);
    const double eval_end = std::min(total_dur, eval_start + std::max(0.0, horizon));
    const double sample_dt = std::max(0.03, dt);

    general_utils::Vec3f last = traj.getPos(eval_start);
    if (!last.allFinite()) {
        if (reason) {
            *reason = "non-finite initial point";
        }
        return false;
    }

    const int samples = std::max(1, static_cast<int>(std::ceil((eval_end - eval_start) / sample_dt)));
    for (int i = 0; i <= samples; ++i) {
        const double eval_t = eval_start + (eval_end - eval_start) * i / samples;
        const general_utils::Vec3f pos = traj.getPos(eval_t);
        if (!pos.allFinite()) {
            if (reason) {
                *reason = "non-finite sample";
            }
            return false;
        }

        if (map_manager_ != nullptr && map_manager_->ready()) {
            if (!map_manager_->insideLocalMap(pos)) {
                if (reason) {
                    *reason = "outside local map";
                }
                return false;
            }

            const auto grid_type = map_manager_->getInfGridType(pos);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP) {
                if (reason) {
                    *reason = "occupied or out-of-map";
                }
                return false;
            }

            if (cfg_.tracking_unknown_as_occupied &&
                (grid_type == rog_map::GridType::UNKNOWN ||
                 grid_type == rog_map::GridType::UNDEFINED ||
                 grid_type == rog_map::GridType::FRONTIER)) {
                if (reason) {
                    *reason = "unknown treated as occupied";
                }
                return false;
            }

            if (map_manager_->hasESDF()) {
                double dist = 0.0;
                general_utils::Vec3f grad = general_utils::Vec3f::Zero();
                if (map_manager_->evaluateESDF(pos, dist, grad) &&
                    dist < effectiveTrackingHardSafeDistance(cfg_)) {
                    if (reason) {
                        *reason = "inside tracking hard safe distance";
                    }
                    return false;
                }
            }

            if ((pos - last).norm() > 1.0e-4 &&
                !map_manager_->isLineFree(last, pos, true, cfg_.tracking_unknown_as_occupied)) {
                if (reason) {
                    *reason = "segment not line-free";
                }
                return false;
            }
        }
        last = pos;
    }

    return true;
}

TrackingRuntimeManager::MotionMetrics TrackingRuntimeManager::computeMotionMetrics(
        const geometry_utils::Trajectory &candidate,
        const traj_opt::DynamicTargetStates &target_prediction,
        const double candidate_eval_start_t,
        const double target_eval_start_t,
        const double horizon) const
{
    MotionMetrics metrics;
    if (candidate.empty() || target_prediction.empty()) {
        return metrics;
    }

    const double total_dur = candidate.getTotalDuration();
    const double start_t = std::clamp(candidate_eval_start_t, 0.0, total_dur);
    const double eval_horizon =
            std::min({std::max(0.0, horizon),
                      std::max(0.0, total_dur - start_t),
                      std::max(0.0, target_prediction.back().t - std::max(0.0, target_eval_start_t))});
    const double end_t = std::clamp(start_t + eval_horizon, 0.0, total_dur);
    const general_utils::Vec3f p0 = candidate.getPos(start_t);
    const general_utils::Vec3f p1 = candidate.getPos(end_t);
    const general_utils::Vec3f v0 = candidate.getVel(start_t);
    if (p0.allFinite() && p1.allFinite()) {
        const general_utils::Vec3f dp = p1 - p0;
        metrics.displacement_xy = dp.head<2>().norm();
        metrics.displacement_z = std::abs(dp.z());
        metrics.displacement_3d = dp.norm();
    }
    if (v0.allFinite()) {
        metrics.speed_xy = v0.head<2>().norm();
        metrics.speed_z = std::abs(v0.z());
        metrics.speed_3d = v0.norm();
    }

    const auto target0 = interpolateTargetPrediction(target_prediction,
                                                     std::max(0.0, target_eval_start_t));
    metrics.target_speed_xy = target0.velocity.head<2>().norm();
    metrics.target_speed_z = std::abs(target0.velocity.z());
    metrics.target_speed_3d = target0.velocity.norm();

    double target_span_3d = 0.0;
    double target_span_z = 0.0;
    if (target_prediction.size() >= 2) {
        const double target_end_t =
                std::min(target_prediction.back().t,
                         std::max(0.0, target_eval_start_t) + eval_horizon);
        const auto target1 = interpolateTargetPrediction(target_prediction, target_end_t);
        const general_utils::Vec3f target_dp = target1.position - target0.position;
        target_span_3d = target_dp.norm();
        target_span_z = std::abs(target_dp.z());
    }
    metrics.target_vertical_moving =
            metrics.target_speed_z > cfg_.tracking_vertical_motion_threshold ||
            target_span_z > cfg_.tracking_no_motion_min_displacement_z;
    metrics.target_moving =
            metrics.target_speed_xy > cfg_.tracking_no_motion_target_speed_threshold ||
            metrics.target_vertical_moving ||
            target_span_3d > std::max(cfg_.tracking_no_motion_min_displacement,
                                      cfg_.tracking_no_motion_min_displacement_z);

    const general_utils::Vec3f target_dir = targetDirection(target_prediction);
    general_utils::Vec3f dp = general_utils::Vec3f::Zero();
    if (p1.allFinite() && p0.allFinite()) {
        dp = p1 - p0;
    }
    metrics.progress_xy = dp.head<2>().dot(target_dir.head<2>());
    metrics.progress_3d = dp.dot(target_dir);
    return metrics;
}

TrackingRuntimeManager::Activity TrackingRuntimeManager::evaluateActivity(
        const geometry_utils::Trajectory &traj,
        const double local_start_t,
        const traj_opt::DynamicTargetStates &input_prediction,
        const double horizon,
        const double dt) const
{
    Activity out;
    const auto target_prediction = trackingPredictionAtTime(input_prediction,
        traj.start_WT + local_start_t);
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
    const double start_t = std::clamp(local_start_t, 0.0, total_dur);
    out.remaining = std::max(0.0, total_dur - start_t);
    const double eval_horizon = std::min({std::max(0.0, horizon), out.remaining,
                                          target_prediction.back().t});
    const double sample_dt = std::max(0.03, dt);

    const auto target0 = interpolateTargetPrediction(target_prediction, 0.0);
    const general_utils::Vec3f target_dir = targetDirection(target_prediction);

    general_utils::Vec3f last_p = traj.getPos(start_t);
    if (!last_p.allFinite()) {
        out.reason = "non-finite initial point";
        return out;
    }
    const MotionMetrics initial_metrics =
            computeMotionMetrics(traj, target_prediction, start_t, 0.0, eval_horizon);
    out.target_moving = initial_metrics.target_moving;
    out.target_vertical_moving = initial_metrics.target_vertical_moving;
    out.speed_xy = initial_metrics.speed_xy;
    out.speed_z = initial_metrics.speed_z;
    out.speed_3d = initial_metrics.speed_3d;
    out.speed0 = out.speed_xy;
    out.target_speed_xy = initial_metrics.target_speed_xy;
    out.target_speed_z = initial_metrics.target_speed_z;
    out.target_speed_3d = initial_metrics.target_speed_3d;

    out.safe = trajectorySafe(traj, start_t, eval_horizon, dt, &out.reason);
    if (!out.safe) return out;
    double total_error = 0.0;
    int sample_count = 0;

    const int samples = std::max(1, static_cast<int>(std::ceil(eval_horizon / sample_dt)));
    for (int i = 0; i <= samples; ++i) {
        const double s = eval_horizon * i / samples;
        const double traj_t = std::min(total_dur, start_t + s);
        const general_utils::Vec3f p = traj.getPos(traj_t);
        if (!p.allFinite()) {
            out.safe = false;
            out.reason = "non-finite sample";
            return out;
        }

        const auto target = interpolateTargetPrediction(target_prediction, s);
        total_error += trackingDistanceError(p, target.position);
        ++sample_count;

        if (s > 1.0e-6) {
            const general_utils::Vec3f dp = p - last_p;
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
    out.tracking_error = trackingDistanceError(traj.getPos(start_t), target0.position);

    if (out.remaining < cfg_.tracking_keep_old_min_remaining) {
        out.reason = "remaining time too short";
        return out;
    }

    if (out.target_moving && execution_start_ >= 0.0 &&
        observation_time_ > execution_start_ + cfg_.tracking_keep_old_startup_grace &&
        !execution_progress_observed_) {
        out.reason = "EXECUTION_STALLED: no measured route progress after startup grace";
        return out;
    }
    const auto approach = trackingApproachMotion(traj, target_prediction, cfg_, start_t,
        0.0, eval_horizon, committed_guide_.empty() ? nullptr : &committed_guide_);
    if (out.target_moving && approach.valid) {
        out.active = out.safe;
        out.reason = "safe route progress in trusted prediction window";
        return out;
    }
    const bool startup_grace = execution_start_ >= 0.0 &&
        observation_time_ <= execution_start_ + cfg_.tracking_keep_old_startup_grace;
    if (out.target_moving && startup_grace &&
        trackingCandidateHasMotion(traj, target_prediction, cfg_, start_t, 0.0)) {
        out.active = out.safe;
        out.reason = "bounded startup grace with predicted motion";
        return out;
    }

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

        if (active_speed < cfg_.tracking_keep_old_min_speed &&
            !trackingCandidateHasMotion(traj, target_prediction, cfg_, start_t, 0.0)) {
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

    out.active = out.safe;
    out.reason = out.active ? "safe and active" : "unsafe trajectory";
    return out;
}

bool TrackingRuntimeManager::candidateCommandable(
        const geometry_utils::Trajectory &candidate,
        const traj_opt::DynamicTargetStates &target_prediction,
        const double candidate_eval_start_t,
        const double target_eval_start_t,
        std::string *reason) const
{
    if (!cfg_.tracking_no_motion_guard_enable) {
        return true;
    }
    if (candidate.empty() || target_prediction.empty()) {
        if (reason) {
            *reason = "empty candidate or target prediction";
        }
        return false;
    }

    const bool commandable = trackingCandidateHasMotion(
        candidate, target_prediction, cfg_, candidate_eval_start_t, target_eval_start_t);
    if (!commandable && reason) *reason = "candidate no-motion in trusted prediction window";
    return commandable;
}

TrackingRuntimeManager::Decision TrackingRuntimeManager::decide(
        const geometry_utils::Trajectory *old_committed,
        const double old_local_t,
        const geometry_utils::Trajectory &candidate,
        const traj_opt::DynamicTargetStates &target_prediction,
        const bool candidate_safe,
        const bool anti_rollback_pass,
        const double candidate_eval_start_t,
        const double target_eval_start_t)
{
    Decision decision;
    decision.candidate_safe = candidate_safe;
    decision.candidate_commandable =
            candidateCommandable(candidate,
                                 target_prediction,
                                 candidate_eval_start_t,
                                 target_eval_start_t,
                                 &decision.reason);

    if (old_committed == nullptr) {
        if (candidate_safe && decision.candidate_commandable) {
            decision.type = DecisionType::COMMIT_CANDIDATE;
            decision.status = Status::CANDIDATE_ACCEPTED;
            decision.reason = "commandable candidate without old tracking trajectory";
            return decision;
        }
        decision.type = DecisionType::REJECT_AND_FAIL;
        decision.status = Status::LOST;
        if (decision.reason.empty()) {
            decision.reason = "candidate unsafe and no old tracking trajectory";
        }
        return decision;
    }

    decision.old_activity =
            evaluateActivity(*old_committed,
                             old_local_t,
                             target_prediction,
                             cfg_.tracking_keep_old_horizon,
                             cfg_.tracking_keep_old_safety_dt);

    if (!candidate_safe) {
        if (decision.old_activity.active) {
            decision.type = DecisionType::KEEP_OLD;
            decision.status = Status::KEEP_OLD_ACTIVE;
            decision.reason = "candidate unsafe";
            return decision;
        }
        decision.type = DecisionType::REJECT_AND_FAIL;
        decision.status = Status::LOST;
        decision.reason = "candidate unsafe and old inactive";
        return decision;
    }

    if (!decision.candidate_commandable) {
        if (decision.old_activity.active) {
            decision.type = DecisionType::KEEP_OLD;
            decision.status = Status::CANDIDATE_REJECTED_NO_MOTION;
            if (decision.reason.empty()) {
                decision.reason = "candidate no-motion";
            }
            return decision;
        }
        decision.type = DecisionType::REJECT_AND_FAIL;
        decision.status = Status::CANDIDATE_REJECTED_NO_MOTION;
        if (decision.reason.empty()) {
            decision.reason = "candidate no-motion and old inactive";
        }
        return decision;
    }

    if (!decision.old_activity.active) {
        decision.type = DecisionType::FORCE_COMMIT_CANDIDATE;
        decision.status = Status::FORCE_COMMIT_SAFE_CANDIDATE;
        decision.bypass_anti_rollback = true;
        decision.reason = "old tracking trajectory inactive";
        return decision;
    }

    if (consecutive_keep_old_ >= cfg_.tracking_max_consecutive_keep_old) {
        decision.type = DecisionType::FORCE_COMMIT_CANDIDATE;
        decision.status = Status::FORCE_COMMIT_SAFE_CANDIDATE;
        decision.bypass_anti_rollback = true;
        decision.reason = "consecutive keep-old limit reached";
        return decision;
    }

    if (!anti_rollback_pass) {
        decision.type = DecisionType::KEEP_OLD;
        decision.status = Status::CANDIDATE_REJECTED_ANTI_ROLLBACK;
        decision.reason = "candidate rejected by anti-rollback";
        return decision;
    }

    decision.type = DecisionType::COMMIT_CANDIDATE;
    decision.status = Status::CANDIDATE_ACCEPTED;
    decision.reason = "candidate accepted";
    return decision;
}

void TrackingRuntimeManager::onHold()
{
    has_committed_tracking_ = false;
    consecutive_keep_old_ = 0;
    ++consecutive_reject_;
    status_ = Status::ACQUIRE;
}

void TrackingRuntimeManager::observeExecution(double now, const general_utils::Vec3f &position)
{
    // A gap between tracking sessions can start a new acquisition; normal
    // prediction updates and recovery HOLDs continue to observe every cycle.
    if (observation_time_ >= 0.0 && (now < observation_time_ ||
        now - observation_time_ > std::max(2.0, 4.0 * cfg_.tracking_keep_old_startup_grace))) reset();
    observation_time_ = now;
    if (execution_start_ < 0.0 || now < execution_start_ || !position.allFinite()) return;
    const auto displacement = (position - execution_origin_).eval();
    const double progress = execution_direction_.squaredNorm() > 0.5
        ? displacement.dot(execution_direction_) : displacement.norm();
    execution_progress_observed_ = execution_progress_observed_ ||
        progress >= std::max(0.01, 0.5 * cfg_.tracking_no_motion_min_displacement);
}

void TrackingRuntimeManager::onCommitted(double execution_start,
                                         const general_utils::Vec3f &start_position,
                                         const general_utils::vec_Vec3f &guide)
{
    committed_guide_ = guide;
    // Neither another commit nor HOLD restarts this grace period.
    if (execution_start_ < 0.0 && guide.size() >= 2 &&
        std::isfinite(execution_start) && execution_start >= 0.0) {
        execution_start_ = execution_start;
        execution_origin_ = start_position;
        execution_direction_.setZero();
        for (const auto &point : guide) {
            const auto direction = (point - start_position).eval();
            if (direction.norm() > 0.1) {
                execution_direction_ = direction.normalized();
                break;
            }
        }
    }
    consecutive_keep_old_ = 0;
    consecutive_reject_ = 0;
    status_ = Status::ACTIVE_COMMITTED;
    has_committed_tracking_ = true;
}

void TrackingRuntimeManager::onKeepOld()
{
    ++consecutive_keep_old_;
    status_ = Status::KEEP_OLD_ACTIVE;
}

void TrackingRuntimeManager::onRejected()
{
    ++consecutive_reject_;
    status_ = Status::LOST;
}

int TrackingRuntimeManager::consecutiveKeepOld() const
{
    return consecutive_keep_old_;
}

int TrackingRuntimeManager::consecutiveReject() const
{
    return consecutive_reject_;
}

TrackingRuntimeManager::Status TrackingRuntimeManager::status() const
{
    return status_;
}

bool TrackingRuntimeManager::hasCommittedTracking() const
{
    return has_committed_tracking_;
}

} // namespace general_planner
