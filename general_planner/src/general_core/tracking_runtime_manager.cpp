#include "general_core/tracking_runtime_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace general_planner {
namespace {

traj_opt::DynamicTargetState interpolateTargetPrediction(
        const traj_opt::DynamicTargetStates &prediction,
        const double t)
{
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
                                     [](const traj_opt::DynamicTargetState &state,
                                        const double query_t) {
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
}

super_utils::Vec3f TrackingRuntimeManager::targetDirection(
        const traj_opt::DynamicTargetStates &prediction) const
{
    if (prediction.empty()) {
        return super_utils::Vec3f::UnitX();
    }

    super_utils::Vec3f dir = prediction.front().velocity;
    dir.z() = 0.0;
    if (dir.norm() > cfg_.tracking_no_motion_target_speed_threshold) {
        return dir.normalized();
    }

    if (prediction.size() >= 2) {
        dir = prediction.back().position - prediction.front().position;
        dir.z() = 0.0;
        if (dir.norm() > 1.0e-4) {
            return dir.normalized();
        }
    }

    return super_utils::Vec3f::UnitX();
}

double TrackingRuntimeManager::trackingDistanceError(
        const super_utils::Vec3f &tracker,
        const super_utils::Vec3f &target) const
{
    if (!tracker.allFinite() || !target.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    const super_utils::Vec3f rel = tracker - target;
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

    super_utils::Vec3f last = traj.getPos(eval_start);
    if (!last.allFinite()) {
        if (reason) {
            *reason = "non-finite initial point";
        }
        return false;
    }

    for (double t = eval_start; t <= eval_end + 1.0e-6; t += sample_dt) {
        const double eval_t = std::min(t, eval_end);
        const super_utils::Vec3f pos = traj.getPos(eval_t);
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
                super_utils::Vec3f grad = super_utils::Vec3f::Zero();
                if (map_manager_->evaluateESDF(pos, dist, grad) &&
                    dist < cfg_.tracking_safe_distance) {
                    if (reason) {
                        *reason = "inside tracking safe distance";
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

TrackingRuntimeManager::Activity TrackingRuntimeManager::evaluateActivity(
        const geometry_utils::Trajectory &traj,
        const double local_start_t,
        const traj_opt::DynamicTargetStates &target_prediction,
        const double horizon,
        const double dt) const
{
    Activity out;
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
    const double eval_horizon = std::min(std::max(0.0, horizon), out.remaining);
    const double sample_dt = std::max(0.03, dt);

    const auto target0 = interpolateTargetPrediction(target_prediction, 0.0);
    out.target_moving =
            target0.velocity.head<2>().norm() >
            cfg_.tracking_no_motion_target_speed_threshold;
    const super_utils::Vec3f target_dir = targetDirection(target_prediction);

    super_utils::Vec3f last_p = traj.getPos(start_t);
    if (!last_p.allFinite()) {
        out.reason = "non-finite initial point";
        return out;
    }
    out.speed0 = traj.getVel(start_t).head<2>().norm();

    out.safe = true;
    double total_error = 0.0;
    int sample_count = 0;

    for (double s = 0.0; s <= eval_horizon + 1.0e-6; s += sample_dt) {
        const double traj_t = std::min(total_dur, start_t + s);
        const super_utils::Vec3f p = traj.getPos(traj_t);
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
        total_error += trackingDistanceError(p, target.position);
        ++sample_count;

        if (s > 1.0e-6) {
            const super_utils::Vec3f dp = p - last_p;
            out.displacement += dp.head<2>().norm();
            out.progress += dp.head<2>().dot(target_dir.head<2>());
        }
        last_p = p;
    }

    out.avg_tracking_error =
            sample_count > 0 ? total_error / static_cast<double>(sample_count) : 0.0;
    out.tracking_error = trackingDistanceError(traj.getPos(start_t), target0.position);

    if (out.remaining < cfg_.tracking_keep_old_min_remaining) {
        out.reason = "remaining time too short";
        return out;
    }

    if (out.target_moving) {
        if (out.speed0 < cfg_.tracking_keep_old_min_speed) {
            out.reason = "speed too small";
            return out;
        }

        if (out.displacement < cfg_.tracking_keep_old_min_displacement) {
            out.reason = "displacement too small";
            return out;
        }

        out.expected_progress =
                target0.velocity.head<2>().norm() *
                eval_horizon *
                cfg_.tracking_keep_old_min_progress_ratio;
        if (out.progress < out.expected_progress) {
            out.reason = "insufficient target-direction progress";
            return out;
        }

        const double max_err =
                cfg_.tracking_keep_old_max_tracking_error_scale *
                std::max({0.1,
                          cfg_.tracking_distance_tolerance,
                          cfg_.tracking_distance_upper_tolerance});
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

    const auto target0 = target_prediction.front();
    const bool target_moving =
            target0.velocity.head<2>().norm() >
            cfg_.tracking_no_motion_target_speed_threshold;
    if (!target_moving) {
        return true;
    }

    const double h =
            std::min(cfg_.tracking_no_motion_check_horizon,
                     candidate.getTotalDuration());
    if (h < 1.0e-3) {
        if (reason) {
            *reason = "candidate duration too short";
        }
        return false;
    }

    const super_utils::Vec3f p0 = candidate.getPos(0.0);
    const super_utils::Vec3f p1 = candidate.getPos(h);
    const super_utils::Vec3f v0 = candidate.getVel(0.0);
    if (!p0.allFinite() || !p1.allFinite() || !v0.allFinite()) {
        if (reason) {
            *reason = "candidate contains non-finite state";
        }
        return false;
    }

    const double disp = (p1 - p0).head<2>().norm();
    const double speed = v0.head<2>().norm();
    if (disp < cfg_.tracking_no_motion_min_displacement &&
        speed < cfg_.tracking_keep_old_min_speed) {
        if (reason) {
            *reason = "candidate no-motion";
        }
        return false;
    }

    return true;
}

TrackingRuntimeManager::Decision TrackingRuntimeManager::decide(
        const geometry_utils::Trajectory *old_committed,
        const double old_local_t,
        const geometry_utils::Trajectory &candidate,
        const traj_opt::DynamicTargetStates &target_prediction,
        const bool candidate_safe,
        const bool anti_rollback_pass)
{
    Decision decision;
    decision.candidate_safe = candidate_safe;
    decision.candidate_commandable =
            candidateCommandable(candidate, target_prediction, &decision.reason);

    if (old_committed == nullptr) {
        if (candidate_safe && decision.candidate_commandable) {
            decision.type = DecisionType::COMMIT_CANDIDATE;
            decision.status = Status::CANDIDATE_ACCEPTED;
            decision.reason = "no old tracking trajectory";
            return decision;
        }
        decision.type = DecisionType::REJECT_AND_FAIL;
        decision.status =
                candidate_safe ? Status::CANDIDATE_REJECTED_NO_MOTION : Status::LOST;
        if (decision.reason.empty()) {
            decision.reason = candidate_safe ? "candidate not commandable"
                                             : "candidate unsafe and no old tracking trajectory";
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
        decision.type = DecisionType::FORCE_COMMIT_CANDIDATE;
        decision.status = Status::FORCE_COMMIT_SAFE_CANDIDATE;
        decision.bypass_anti_rollback = true;
        if (decision.reason.empty()) {
            decision.reason = "candidate no-motion but old inactive";
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

void TrackingRuntimeManager::onCommitted()
{
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
