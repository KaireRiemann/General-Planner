#include "general_core/takeoff/takeoff_runtime_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

namespace general_planner
{
namespace
{

Eigen::Vector3d normalizedOr(const Eigen::Vector3d &v,
                             const Eigen::Vector3d &fallback)
{
    if (!v.allFinite() || v.norm() < 1.0e-6)
    {
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
    const Eigen::Vector3d thrust =
        acc + Eigen::Vector3d(0.0, 0.0, std::abs(gravity));
    const double thrust_norm = thrust.norm();
    if (thrust_norm < 1.0e-6)
    {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector3d zb = thrust / thrust_norm;
    const Eigen::Matrix3d d_norm =
        (Eigen::Matrix3d::Identity() - zb * zb.transpose()) / thrust_norm;
    return (d_norm * jerk).norm();
}

} // namespace

TakeoffRuntimeManager::TakeoffRuntimeManager(const Config &cfg,
                                             MapManager::Ptr map_manager)
    : cfg_(cfg),
      map_manager_(std::move(map_manager))
{
}

void TakeoffRuntimeManager::reset()
{
    status_ = Status::IDLE;
    has_committed_takeoff_ = false;
}

TakeoffRuntimeManager::SurfaceFrame
TakeoffRuntimeManager::frameAt(const traj_opt::DynamicTakeoffProblem &problem,
                               const double t) const
{
    SurfaceFrame frame;
    const double dt = t - problem.surface.t;
    frame.origin = problem.surface.position +
                   problem.surface.velocity * dt +
                   0.5 * problem.surface.acceleration * dt * dt;
    frame.x = problem.surface.surface_x;
    frame.y = problem.surface.surface_y;
    frame.z = problem.surface.surface_z;
    if (problem.boundary.rotate_surface_with_yaw_rate &&
        std::abs(problem.surface.yaw_rate) > 1.0e-9)
    {
        const double yaw_dt = problem.surface.yaw_rate * dt;
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

std::string TakeoffRuntimeManager::gridTypeName(const rog_map::GridType type) const
{
    switch (type)
    {
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

double TakeoffRuntimeManager::platformMargin(
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &acceleration,
    const SurfaceFrame &frame,
    const traj_opt::DynamicTakeoffProblem &problem) const
{
    const Eigen::Vector3d rel = position - frame.origin;
    const Eigen::Vector3d thrust =
        acceleration + Eigen::Vector3d(0.0, 0.0, std::abs(cfg_.esdf_traj_cfg.grav));
    const double thrust_norm = thrust.norm();
    if (thrust_norm < 1.0e-6)
    {
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

TakeoffRuntimeManager::CheckResult
TakeoffRuntimeManager::checkCandidate(
    const geometry_utils::Trajectory &pos_traj,
    const traj_opt::DynamicTakeoffProblem &problem) const
{
    CheckResult out;
    if (pos_traj.empty())
    {
        out.reason = "empty trajectory";
        return out;
    }

    const double duration = pos_traj.getTotalDuration();
    if (!std::isfinite(duration))
    {
        out.reason = "duration not finite";
        return out;
    }
    if (duration < problem.min_duration - 1.0e-6)
    {
        out.reason = "duration below takeoff bound";
        return out;
    }
    if (duration > problem.max_duration + 1.0e-6)
    {
        out.reason = "duration above takeoff bound";
        return out;
    }

    out.valid = true;
    out.safe = true;
    out.dynamics_feasible = true;
    out.platform_clear_after_release = true;

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

    Eigen::Vector3d last_after_release_p = Eigen::Vector3d::Zero();
    bool have_last_after_release = false;

    auto logRejectSample = [&](const double eval_t,
                               const Eigen::Vector3d &p,
                               const std::string &reason,
                               const double platform_margin,
                               const double esdf,
                               const rog_map::GridType grid_type = rog_map::GridType::UNDEFINED) {
        std::cout << " -- [TakeoffRuntime] TAKEOFF_REJECT_SAMPLE t="
                  << eval_t
                  << ", p=[" << p.x() << ", " << p.y() << ", " << p.z() << "]"
                  << ", reason=" << reason
                  << ", platform_margin=" << platform_margin
                  << ", esdf=" << esdf
                  << ", grid=" << gridTypeName(grid_type) << std::endl;
    };

    for (double t = 0.0; t <= duration + 1.0e-6; t += sample_dt)
    {
        const double eval_t = std::min(t, duration);
        const Eigen::Vector3d p = pos_traj.getPos(eval_t);
        const Eigen::Vector3d v = pos_traj.getVel(eval_t);
        const Eigen::Vector3d a = pos_traj.getAcc(eval_t);
        const Eigen::Vector3d j = pos_traj.getJer(eval_t);
        if (!finiteState(p, v, a, j))
        {
            out.safe = false;
            out.dynamics_feasible = false;
            out.reason = "non-finite trajectory state";
            logRejectSample(eval_t, p, out.reason, 0.0, 0.0);
            return out;
        }

        const double thrust = normalizedThrust(a, gravity);
        const double omega = bodyRate12(a, j, gravity);
        out.max_thrust = std::max(out.max_thrust, thrust);
        out.max_omega = std::max(out.max_omega, omega);
        if (v.norm() > max_vel + 1.0e-3 ||
            thrust < min_thrust - 1.0e-3 ||
            thrust > max_thrust + 1.0e-3 ||
            omega > max_omega + 1.0e-3)
        {
            out.dynamics_feasible = false;
            if (out.reason.empty())
            {
                out.reason = "dynamics bound violation";
            }
        }

        if (eval_t <= problem.release_contact_time + 1.0e-9)
        {
            continue;
        }

        const auto frame = frameAt(problem, eval_t);
        const double margin = platformMargin(p, a, frame, problem);
        out.min_platform_margin_after_release =
            std::min(out.min_platform_margin_after_release, margin);
        if (margin < problem.platform_clearance_after_release)
        {
            out.safe = false;
            out.platform_clear_after_release = false;
            out.reason = "platform clearance violation after release";
            logRejectSample(eval_t,
                            p,
                            out.reason,
                            margin,
                            out.min_esdf_clearance);
            return out;
        }

        if (map_manager_ != nullptr && map_manager_->ready())
        {
            if (!map_manager_->insideLocalMap(p))
            {
                out.safe = false;
                out.reason = "outside local map after release";
                logRejectSample(eval_t,
                                p,
                                out.reason,
                                margin,
                                out.min_esdf_clearance,
                                rog_map::GridType::OUT_OF_MAP);
                return out;
            }
            const auto grid_type = map_manager_->getInfGridType(p);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP)
            {
                out.safe = false;
                out.reason = "occupied/out-of-map grid after release";
                logRejectSample(eval_t,
                                p,
                                out.reason,
                                margin,
                                out.min_esdf_clearance,
                                grid_type);
                return out;
            }
            if (map_manager_->hasESDF())
            {
                double dist = 0.0;
                Eigen::Vector3d grad = Eigen::Vector3d::Zero();
                if (map_manager_->evaluateESDF(p, dist, grad))
                {
                    out.min_esdf_clearance = std::min(out.min_esdf_clearance, dist);
                    if (problem.safe_distance > 0.0 && dist < problem.safe_distance)
                    {
                        out.safe = false;
                        out.reason = "ESDF clearance violation after release";
                        logRejectSample(eval_t, p, out.reason, margin, dist, grid_type);
                        return out;
                    }
                }
            }
            if (have_last_after_release &&
                (p - last_after_release_p).norm() > 1.0e-4 &&
                !map_manager_->isLineFree(last_after_release_p, p, true, false))
            {
                out.safe = false;
                out.reason = "sample segment not line-free after release";
                logRejectSample(eval_t,
                                p,
                                out.reason,
                                margin,
                                out.min_esdf_clearance,
                                grid_type);
                return out;
            }
        }
        last_after_release_p = p;
        have_last_after_release = true;
    }

    const Eigen::Vector3d terminal_p = pos_traj.getPos(duration);
    const double terminal_error = (terminal_p - problem.tail_pvaj.col(0)).norm();
    out.terminal_escape_valid =
        terminal_error <= std::max(0.20, 0.5 * std::max(0.0, problem.safe_distance));
    if (!out.terminal_escape_valid && out.reason.empty())
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3)
           << "terminal escape error=" << terminal_error;
        out.reason = ss.str();
    }
    if (!out.dynamics_feasible && out.reason.empty())
    {
        out.reason = "dynamics infeasible";
    }
    if (out.valid &&
        out.safe &&
        out.dynamics_feasible &&
        out.platform_clear_after_release &&
        out.terminal_escape_valid)
    {
        out.reason = "candidate accepted";
    }
    return out;
}

bool TakeoffRuntimeManager::decideCommit(const CheckResult &candidate_check)
{
    if (candidate_check.valid &&
        candidate_check.safe &&
        candidate_check.dynamics_feasible &&
        candidate_check.platform_clear_after_release &&
        candidate_check.terminal_escape_valid)
    {
        status_ = Status::PREPARE;
        return true;
    }
    status_ = Status::ABORT;
    return false;
}

void TakeoffRuntimeManager::updateStatusAfterCommit()
{
    has_committed_takeoff_ = true;
    status_ = Status::EXECUTING;
}

void TakeoffRuntimeManager::updateStatusByPosition(
    const Eigen::Vector3d &position,
    const traj_opt::DynamicTakeoffProblem &problem)
{
    if (!has_committed_takeoff_ || status_ != Status::EXECUTING)
    {
        return;
    }
    const auto frame = frameAt(problem, 0.0);
    const double normal =
        (position - frame.origin).dot(frame.z) - std::max(0.0, problem.robot_l);
    if (normal >= std::max(0.0, problem.platform_clearance_after_release))
    {
        status_ = Status::PLATFORM_CLEAR;
    }
}

TakeoffRuntimeManager::Status TakeoffRuntimeManager::status() const
{
    return status_;
}

bool TakeoffRuntimeManager::hasCommittedTakeoff() const
{
    return has_committed_takeoff_;
}

} // namespace general_planner
