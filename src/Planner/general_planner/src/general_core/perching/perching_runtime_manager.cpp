#include "general_core/perching/perching_runtime_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

namespace general_planner {
namespace {

Eigen::Vector3d normalizedOr(const Eigen::Vector3d &v,
                             const Eigen::Vector3d &fallback)
{
    if (!v.allFinite() || v.norm() < 1.0e-6) {
        return fallback;
    }
    return v.normalized();
}

Eigen::Vector3d rotateYaw(const Eigen::Vector3d &v, const double yaw)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Eigen::Vector3d(c * v.x() - s * v.y(),
                           s * v.x() + c * v.y(),
                           v.z());
}

bool finiteState(const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 const Eigen::Vector3d &j)
{
    return p.allFinite() && v.allFinite() && a.allFinite() && j.allFinite();
}

double normalizedThrust(const Eigen::Vector3d &acc,
                        const double gravity)
{
    return (acc + Eigen::Vector3d(0.0, 0.0, std::abs(gravity))).norm();
}

double bodyRate12(const Eigen::Vector3d &acc,
                  const Eigen::Vector3d &jerk,
                  const double gravity)
{
    const Eigen::Vector3d thrust = acc + Eigen::Vector3d(0.0, 0.0, std::abs(gravity));
    const double thrust_norm = thrust.norm();
    if (thrust_norm < 1.0e-6) {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector3d zb = thrust / thrust_norm;
    const Eigen::Matrix3d d_norm =
            (Eigen::Matrix3d::Identity() - zb * zb.transpose()) / thrust_norm;
    return (d_norm * jerk).norm();
}

} // namespace

PerchingRuntimeManager::PerchingRuntimeManager(const Config &cfg,
                                               const MapManager::Ptr &map_manager)
        : cfg_(cfg),
          map_manager_(map_manager)
{
}

void PerchingRuntimeManager::reset()
{
    status_ = Status::IDLE;
    has_committed_perching_ = false;
    consecutive_reject_ = 0;
    last_rejected_candidate_ = RejectedCandidateSignature{};
}

PerchingRuntimeManager::SurfaceFrame
PerchingRuntimeManager::frameAt(const traj_opt::PerchingSurfaceState &surface,
                                const traj_opt::PerchingProblem &problem,
                                const double t) const
{
    SurfaceFrame frame;
    frame.origin = surface.position + surface.velocity * t + 0.5 * surface.acceleration * t * t;
    frame.x = surface.surface_x;
    frame.y = surface.surface_y;
    frame.z = surface.surface_z;
    const bool rotate = problem.terminal.rotate_surface_with_yaw_rate ||
                        cfg_.perching_rotate_surface_with_yaw_rate;
    if (rotate && std::abs(surface.yaw_rate) > 1.0e-9) {
        const double yaw_dt = surface.yaw_rate * t;
        frame.x = rotateYaw(frame.x, yaw_dt);
        frame.y = rotateYaw(frame.y, yaw_dt);
        frame.z = rotateYaw(frame.z, yaw_dt);
    }
    frame.z = normalizedOr(frame.z, Eigen::Vector3d::UnitZ());
    frame.x = normalizedOr(frame.x, Eigen::Vector3d::UnitX());
    frame.y = normalizedOr(frame.z.cross(frame.x), Eigen::Vector3d::UnitY());
    frame.x = normalizedOr(frame.y.cross(frame.z), Eigen::Vector3d::UnitX());
    return frame;
}

bool PerchingRuntimeManager::isFinalContactWindow(const double eval_t,
                                                  const double duration) const
{
    return duration - eval_t <= std::max(0.0, cfg_.perching_contact_time_margin);
}

bool PerchingRuntimeManager::isExpectedContactSample(const Eigen::Vector3d &p,
                                                     const SurfaceFrame &frame,
                                                     const traj_opt::PerchingProblem &problem,
                                                     const double eval_t,
                                                     const double duration,
                                                     double *normal_dist,
                                                     double *tangent_dist) const
{
    const Eigen::Vector3d rel = p - frame.origin;
    const double normal = rel.dot(frame.z);
    const double tangent = std::hypot(rel.dot(frame.x), rel.dot(frame.y));
    if (normal_dist != nullptr) {
        *normal_dist = normal;
    }
    if (tangent_dist != nullptr) {
        *tangent_dist = tangent;
    }
    if (!isFinalContactWindow(eval_t, duration)) {
        return false;
    }

    const double expected_normal = std::max(0.0, problem.robot_l);
    const bool normal_ok =
            std::abs(normal - expected_normal) <=
            std::max(0.0, cfg_.perching_contact_occupancy_normal_tolerance) +
            std::max(0.0, problem.platform_clearance);
    const bool tangent_ok =
            tangent <= std::max(0.0, problem.platform_radius) +
                       std::max(0.0, problem.robot_radius) +
                       std::max(0.0, cfg_.perching_contact_occupancy_tangent_margin);
    return normal_ok && tangent_ok;
}

std::string PerchingRuntimeManager::gridTypeName(const rog_map::GridType type) const
{
    switch (type) {
    case rog_map::GridType::OCCUPIED:
        return "OCCUPIED";
    case rog_map::GridType::OUT_OF_MAP:
        return "OUT_OF_MAP";
    case rog_map::GridType::UNKNOWN:
        return "UNKNOWN";
    case rog_map::GridType::FRONTIER:
        return "FRONTIER";
    case rog_map::GridType::KNOWN_FREE:
        return "KNOWN_FREE";
    case rog_map::GridType::UNDEFINED:
        return "UNDEFINED";
    default:
        return "UNKNOWN_GRID_TYPE";
    }
}

double PerchingRuntimeManager::platformMargin(const Eigen::Vector3d &position,
                                              const Eigen::Vector3d &acceleration,
                                              const SurfaceFrame &frame,
                                              const traj_opt::PerchingProblem &problem) const
{
    const Eigen::Vector3d rel = position - frame.origin;
    const Eigen::Vector3d thrust =
            acceleration + Eigen::Vector3d(0.0, 0.0, std::abs(cfg_.esdf_traj_cfg.grav));
    const double thrust_norm = thrust.norm();
    if (thrust_norm < 1.0e-6) {
        return rel.dot(frame.z) -
               std::max(0.0, problem.robot_radius + problem.platform_clearance);
    }

    const Eigen::Vector3d body_z = thrust / thrust_norm;
    const double normal_align = std::clamp(frame.z.dot(body_z), -1.0, 1.0);
    const double tangent_component =
            std::sqrt(std::max(0.0, 1.0 - normal_align * normal_align));
    const double min_disk_clearance =
            rel.dot(frame.z) -
            std::max(0.0, problem.robot_l) * normal_align -
            std::max(0.0, problem.robot_radius) * tangent_component;
    return min_disk_clearance - std::max(0.0, problem.platform_clearance);
}

Eigen::Vector3d PerchingRuntimeManager::expectedTerminalPosition(
        const traj_opt::PerchingProblem &problem,
        const traj_opt::PerchingSurfaceState &surface,
        const double T) const
{
    const auto frame = frameAt(surface, problem, T);
    return surface.position + surface.velocity * T + 0.5 * surface.acceleration * T * T +
           std::max(0.0, problem.robot_l) * frame.z;
}

Eigen::Vector3d PerchingRuntimeManager::expectedTerminalVelocityBase(
        const traj_opt::PerchingProblem &problem,
        const traj_opt::PerchingSurfaceState &surface,
        const double T) const
{
    const auto frame = frameAt(surface, problem, T);
    return surface.velocity + surface.acceleration * T -
           std::max(0.0, problem.terminal.v_plus) * frame.z;
}

PerchingRuntimeManager::CheckResult
PerchingRuntimeManager::checkCandidate(const geometry_utils::Trajectory &pos_traj,
                                       const geometry_utils::Trajectory *yaw_traj,
                                       const traj_opt::PerchingProblem &problem,
                                       const traj_opt::PerchingSurfaceState &surface) const
{
    CheckResult out;
    if (pos_traj.empty()) {
        out.reason = "empty trajectory";
        return out;
    }

    const double duration = pos_traj.getTotalDuration();
    const double duration_tol = std::max(0.0, cfg_.perching_duration_tolerance);
    auto rejectDuration = [&](const std::string &reason) {
        std::cout << " -- [PerchingRuntime] PERCHING_REJECT_DURATION duration="
                  << duration << ", min=" << cfg_.perching_min_duration
                  << ", max=" << cfg_.perching_max_duration
                  << ", tol=" << duration_tol
                  << ", reason=" << reason << std::endl;
        out.reason = reason;
    };
    if (!std::isfinite(duration)) {
        rejectDuration("duration not finite");
        return out;
    }
    if (duration < cfg_.perching_min_duration - duration_tol) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3)
           << "duration below perching bound: duration=" << duration
           << ", min=" << cfg_.perching_min_duration
           << ", tol=" << duration_tol;
        rejectDuration(ss.str());
        return out;
    }
    if (duration > cfg_.perching_max_duration + duration_tol) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3)
           << "duration above perching bound: duration=" << duration
           << ", max=" << cfg_.perching_max_duration
           << ", tol=" << duration_tol;
        rejectDuration(ss.str());
        return out;
    }

    out.valid = true;
    out.safe = true;
    out.dynamics_feasible = true;

    const double sample_dt = std::clamp(duration / 80.0, 0.03, 0.08);
    const double gravity = std::abs(cfg_.esdf_traj_cfg.grav);
    const double max_vel = cfg_.esdf_traj_cfg.max_vel > 0.0
                               ? cfg_.esdf_traj_cfg.max_vel
                               : std::numeric_limits<double>::infinity();
    const double min_thrust = cfg_.esdf_traj_cfg.min_acc_thr > 0.0
                                  ? cfg_.esdf_traj_cfg.min_acc_thr
                                  : 0.0;
    const double max_thrust = cfg_.esdf_traj_cfg.max_acc_thr > 0.0
                                  ? cfg_.esdf_traj_cfg.max_acc_thr
                                  : std::numeric_limits<double>::infinity();
    const double max_omega = cfg_.esdf_traj_cfg.max_omg > 0.0
                                 ? cfg_.esdf_traj_cfg.max_omg
                                 : std::numeric_limits<double>::infinity();

    Eigen::Vector3d last_p = pos_traj.getPos(0.0);
    const auto last_frame0 = frameAt(surface, problem, 0.0);
    bool last_expected_contact_sample =
            isExpectedContactSample(last_p, last_frame0, problem, 0.0, duration);
    auto logMapReject = [&](const double eval_t,
                            const Eigen::Vector3d &p,
                            const rog_map::GridType grid_type,
                            const bool final_contact_window,
                            const bool expected_contact_sample,
                            const double normal_dist,
                            const double tangent_dist,
                            const bool inside_local_map,
                            const std::string &reason) {
        std::cout << " -- [PerchingRuntime] PERCHING_REJECT_MAP_SAMPLE t="
                  << eval_t << ", T=" << duration
                  << ", p=[" << p.x() << ", " << p.y() << ", " << p.z() << "]"
                  << ", grid=" << gridTypeName(grid_type)
                  << ", final_contact_window=" << final_contact_window
                  << ", expected_contact_sample=" << expected_contact_sample
                  << ", normal_dist=" << normal_dist
                  << ", tangent_dist=" << tangent_dist
                  << ", inside_local_map=" << inside_local_map
                  << ", surface_z=[" << frameAt(surface, problem, eval_t).z.x()
                  << ", " << frameAt(surface, problem, eval_t).z.y()
                  << ", " << frameAt(surface, problem, eval_t).z.z() << "]"
                  << ", reason=" << reason << std::endl;
    };
    for (double t = 0.0; t <= duration + 1.0e-6; t += sample_dt) {
        const double eval_t = std::min(t, duration);
        const Eigen::Vector3d p = pos_traj.getPos(eval_t);
        const Eigen::Vector3d v = pos_traj.getVel(eval_t);
        const Eigen::Vector3d a = pos_traj.getAcc(eval_t);
        const Eigen::Vector3d j = pos_traj.getJer(eval_t);
        if (!finiteState(p, v, a, j)) {
            out.safe = false;
            out.dynamics_feasible = false;
            out.reason = "non-finite trajectory state";
            return out;
        }

        const double thrust = normalizedThrust(a, gravity);
        const double omega = bodyRate12(a, j, gravity);
        out.max_thrust = std::max(out.max_thrust, thrust);
        out.max_omega = std::max(out.max_omega, omega);
        if (v.norm() > max_vel + 1.0e-3 ||
            thrust < min_thrust - 1.0e-3 ||
            thrust > max_thrust + 1.0e-3 ||
            omega > max_omega + 1.0e-3) {
            out.dynamics_feasible = false;
            out.reason = "dynamics bound violation";
        }

        const auto frame = frameAt(surface, problem, eval_t);
        const double margin = platformMargin(p, a, frame, problem);
        out.min_platform_margin = std::min(out.min_platform_margin, margin);
        const bool final_contact_window = isFinalContactWindow(eval_t, duration);
        double normal_dist = 0.0;
        double tangent_dist = 0.0;
        const bool expected_contact_sample =
                isExpectedContactSample(p,
                                        frame,
                                        problem,
                                        eval_t,
                                        duration,
                                        &normal_dist,
                                        &tangent_dist);
        if (!final_contact_window &&
            tangent_dist <= std::max(0.1, problem.platform_collision_activation_distance) &&
            margin < -1.0e-3) {
            out.safe = false;
            out.reason = "platform collision before contact window";
            return out;
        }

        if (map_manager_ != nullptr && map_manager_->ready()) {
            if (!map_manager_->insideLocalMap(p)) {
                out.safe = false;
                out.reason = "outside local map";
                logMapReject(eval_t,
                             p,
                             rog_map::GridType::OUT_OF_MAP,
                             final_contact_window,
                             expected_contact_sample,
                             normal_dist,
                             tangent_dist,
                             false,
                             out.reason);
                return out;
            }
            const auto grid_type = map_manager_->getInfGridType(p);
            if (grid_type == rog_map::GridType::OUT_OF_MAP) {
                out.safe = false;
                out.reason = "out-of-map grid";
                logMapReject(eval_t,
                             p,
                             grid_type,
                             final_contact_window,
                             expected_contact_sample,
                             normal_dist,
                             tangent_dist,
                             true,
                             out.reason);
                return out;
            }
            if (grid_type == rog_map::GridType::OCCUPIED) {
                const bool allow_contact_occupancy =
                        cfg_.perching_contact_occupancy_allowance_enable &&
                        expected_contact_sample;
                if (!allow_contact_occupancy) {
                    out.safe = false;
                    out.reason = "occupied grid before contact";
                    logMapReject(eval_t,
                                 p,
                                 grid_type,
                                 final_contact_window,
                                 expected_contact_sample,
                                 normal_dist,
                                 tangent_dist,
                                 true,
                                 out.reason);
                    return out;
                }
            }
            if (map_manager_->hasESDF()) {
                double dist = 0.0;
                Eigen::Vector3d grad = Eigen::Vector3d::Zero();
                if (map_manager_->evaluateESDF(p, dist, grad)) {
                    out.min_esdf_clearance = std::min(out.min_esdf_clearance, dist);
                    if (!final_contact_window &&
                        problem.safe_distance > 0.0 &&
                        dist < problem.safe_distance) {
                        out.safe = false;
                        out.reason = "ESDF clearance violation";
                        return out;
                    }
                }
            }
            if ((p - last_p).norm() > 1.0e-4 &&
                !map_manager_->isLineFree(last_p, p, true, false)) {
                const bool allow_contact_line =
                        cfg_.perching_contact_linefree_allowance_enable &&
                        (expected_contact_sample || last_expected_contact_sample);
                if (!allow_contact_line) {
                    out.safe = false;
                    out.reason = "sample segment not line-free";
                    logMapReject(eval_t,
                                 p,
                                 grid_type,
                                 final_contact_window,
                                 expected_contact_sample,
                                 normal_dist,
                                 tangent_dist,
                                 true,
                                 out.reason);
                    return out;
                }
            }
        }
        last_p = p;
        last_expected_contact_sample = expected_contact_sample;
    }

    const Eigen::Vector3d terminal_p = pos_traj.getPos(duration);
    const Eigen::Vector3d terminal_v = pos_traj.getVel(duration);
    const Eigen::Vector3d expected_p = expectedTerminalPosition(problem, surface, duration);
    const Eigen::Vector3d expected_v = expectedTerminalVelocityBase(problem, surface, duration);
    const auto terminal_frame = frameAt(surface, problem, duration);
    out.terminal_position_error = (terminal_p - expected_p).norm();
    out.terminal_velocity_error = std::abs((terminal_v - expected_v).dot(terminal_frame.z));
    out.terminal_sync =
            out.terminal_position_error <= cfg_.perching_terminal_pos_tolerance &&
            out.terminal_velocity_error <= cfg_.perching_terminal_vel_tolerance;
    const Eigen::Vector3d start_p = pos_traj.getPos(0.0);
    const Eigen::Vector3d current_contact_p = expectedTerminalPosition(problem, surface, 0.0);
    out.contact_imminent =
            duration <= cfg_.perching_contact_time_margin ||
            (start_p - current_contact_p).norm() <= cfg_.perching_contact_distance_tolerance;

    if (yaw_traj != nullptr && !yaw_traj->empty()) {
        const double yaw_duration = yaw_traj->getTotalDuration();
        if (std::abs(yaw_duration - duration) > 0.15) {
            out.safe = false;
            out.reason = "yaw trajectory duration mismatch";
            return out;
        }
        const double terminal_yaw = yaw_traj->getPos(std::min(yaw_duration, duration)).x();
        const double expected_yaw = surface.yaw + surface.yaw_rate * duration;
        const double yaw_error = std::abs(std::atan2(std::sin(terminal_yaw - expected_yaw),
                                                     std::cos(terminal_yaw - expected_yaw)));
        if (yaw_error > 0.35) {
            out.safe = false;
            out.reason = "terminal yaw not aligned";
            return out;
        }
    }

    if (!out.terminal_sync && out.reason.empty()) {
        out.reason = "terminal sync violation";
    }
    if (!out.dynamics_feasible && out.reason.empty()) {
        out.reason = "dynamics infeasible";
    }
    if (out.safe && out.terminal_sync && out.dynamics_feasible) {
        out.reason = "candidate accepted";
    }
    return out;
}

void PerchingRuntimeManager::rememberRejectedCandidate(
        const traj_opt::PerchingProblem &problem,
        const std::string &reason,
        const double stamp)
{
    last_rejected_candidate_.valid = true;
    last_rejected_candidate_.head_position = problem.head_pvaj.col(0);
    last_rejected_candidate_.head_velocity = problem.head_pvaj.col(1);
    last_rejected_candidate_.surface_position = problem.surface.position;
    last_rejected_candidate_.surface_velocity = problem.surface.velocity;
    last_rejected_candidate_.surface_normal = normalizedOr(problem.surface.surface_z,
                                                           Eigen::Vector3d::UnitZ());
    last_rejected_candidate_.duration_seed = problem.initial_guess.total_time;
    last_rejected_candidate_.piece_num = problem.piece_num;
    last_rejected_candidate_.stamp = stamp;
    last_rejected_candidate_.reason = reason;
}

bool PerchingRuntimeManager::shouldSkipRejectedCandidate(
        const traj_opt::PerchingProblem &problem,
        const double stamp,
        std::string *reason) const
{
    if (!last_rejected_candidate_.valid) {
        return false;
    }

    const double age = stamp - last_rejected_candidate_.stamp;
    if (!std::isfinite(age) || age < -1.0e-3 || age > 0.5) {
        return false;
    }

    const Eigen::Vector3d surface_normal =
            normalizedOr(problem.surface.surface_z, Eigen::Vector3d::UnitZ());
    const bool same =
            problem.piece_num == last_rejected_candidate_.piece_num &&
            std::abs(problem.initial_guess.total_time -
                     last_rejected_candidate_.duration_seed) <= 0.08 &&
            (problem.head_pvaj.col(0) -
             last_rejected_candidate_.head_position).norm() <= 0.08 &&
            (problem.head_pvaj.col(1) -
             last_rejected_candidate_.head_velocity).norm() <= 0.20 &&
            (problem.surface.position -
             last_rejected_candidate_.surface_position).norm() <= 0.25 &&
            (problem.surface.velocity -
             last_rejected_candidate_.surface_velocity).norm() <= 0.10 &&
            surface_normal.dot(last_rejected_candidate_.surface_normal) >=
                    std::cos(0.08);
    if (!same) {
        return false;
    }

    if (reason != nullptr) {
        *reason = last_rejected_candidate_.reason;
    }
    return true;
}

PerchingRuntimeManager::DecisionType
PerchingRuntimeManager::decideCommit(const CheckResult &candidate_check,
                                     const CheckResult *current_perching_check)
{
    if (candidate_check.valid &&
        candidate_check.safe &&
        candidate_check.terminal_sync &&
        candidate_check.dynamics_feasible) {
        status_ = candidate_check.contact_imminent ? Status::CONTACT_IMMINENT
                                                   : Status::CANDIDATE_OPTIMIZED;
        consecutive_reject_ = 0;
        return DecisionType::COMMIT_CANDIDATE;
    }

    ++consecutive_reject_;
    if (current_perching_check != nullptr &&
        current_perching_check->valid &&
        current_perching_check->safe &&
        current_perching_check->dynamics_feasible &&
        current_perching_check->contact_imminent &&
        consecutive_reject_ <= std::max(0, cfg_.perching_max_replan_fail_keep)) {
        status_ = Status::CONTACT_IMMINENT;
        return DecisionType::KEEP_CURRENT_PERCHING;
    }

    status_ = consecutive_reject_ > std::max(0, cfg_.perching_max_replan_fail_keep)
                  ? Status::ABORT
                  : Status::LOST;
    return status_ == Status::ABORT ? DecisionType::ABORT : DecisionType::REJECT;
}

bool PerchingRuntimeManager::candidateAccepted(const CheckResult &candidate_check) const
{
    return candidate_check.valid &&
           candidate_check.safe &&
           candidate_check.terminal_sync &&
           candidate_check.dynamics_feasible;
}

void PerchingRuntimeManager::updateStatusAfterCommit()
{
    has_committed_perching_ = true;
    consecutive_reject_ = 0;
    last_rejected_candidate_ = RejectedCandidateSignature{};
    if (status_ != Status::CONTACT_IMMINENT) {
        status_ = Status::EXECUTING;
    }
}

void PerchingRuntimeManager::updateStatusAfterContact()
{
    status_ = Status::CONTACT;
}

PerchingRuntimeManager::Status PerchingRuntimeManager::status() const
{
    return status_;
}

bool PerchingRuntimeManager::hasCommittedPerching() const
{
    return has_committed_perching_;
}

int PerchingRuntimeManager::consecutiveReject() const
{
    return consecutive_reject_;
}

} // namespace general_planner
