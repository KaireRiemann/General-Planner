#include "general_core/tracking/tracking_to_perching_initializer.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace general_planner {
namespace {

using general_utils::StatePVAJ;
using general_utils::Vec3f;
using general_utils::vec_E;

Vec3f normalizedOr(const Vec3f &v, const Vec3f &fallback)
{
    if (!v.allFinite() || v.norm() < 1.0e-6) {
        return fallback;
    }
    return v.normalized();
}

Vec3f rotateYaw(const Vec3f &v, const double yaw)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Vec3f(c * v.x() - s * v.y(),
                 s * v.x() + c * v.y(),
                 v.z());
}

void appendTimedUnique(const Vec3f &p,
                       const double t,
                       vec_E<Vec3f> &path,
                       std::vector<double> &path_t)
{
    if (!p.allFinite() || !std::isfinite(t)) {
        return;
    }
    const double safe_t = path_t.empty() ? std::max(0.0, t)
                                         : std::max(t, path_t.back() + 1.0e-4);
    if (path.empty() || (path.back() - p).norm() > 1.0e-4) {
        path.emplace_back(p);
        path_t.emplace_back(safe_t);
    } else if (!path_t.empty()) {
        path_t.back() = safe_t;
    }
}

void surfaceFrameAt(const traj_opt::PerchingSurfaceState &surface,
                    const bool rotate_with_yaw_rate,
                    const double t,
                    Vec3f &x_s,
                    Vec3f &y_s,
                    Vec3f &z_s)
{
    z_s = normalizedOr(surface.surface_z, Vec3f::UnitZ());
    x_s = normalizedOr(surface.surface_x, Vec3f::UnitX());
    y_s = normalizedOr(z_s.cross(x_s), Vec3f::UnitY());
    x_s = normalizedOr(y_s.cross(z_s), Vec3f::UnitX());
    if (rotate_with_yaw_rate && std::abs(surface.yaw_rate) > 1.0e-9) {
        const double yaw_dt = surface.yaw_rate * t;
        x_s = rotateYaw(x_s, yaw_dt);
        y_s = rotateYaw(y_s, yaw_dt);
        z_s = rotateYaw(z_s, yaw_dt);
        z_s = normalizedOr(z_s, Vec3f::UnitZ());
        x_s = normalizedOr(x_s, Vec3f::UnitX());
        y_s = normalizedOr(z_s.cross(x_s), Vec3f::UnitY());
        x_s = normalizedOr(y_s.cross(z_s), Vec3f::UnitX());
    }
}

traj_opt::PerchingSurfaceState rebaseSurface(
        const traj_opt::PerchingSurfaceState &surface,
        const double s,
        const Config &cfg)
{
    traj_opt::PerchingSurfaceState rebased = surface;
    rebased.position = surface.position + surface.velocity * s +
                       0.5 * surface.acceleration * s * s;
    rebased.velocity = surface.velocity + surface.acceleration * s;
    rebased.acceleration = surface.acceleration;
    if (cfg.perching_rotate_surface_with_yaw_rate &&
        std::abs(surface.yaw_rate) > 1.0e-9) {
        const double yaw_dt = surface.yaw_rate * s;
        rebased.surface_x = rotateYaw(surface.surface_x, yaw_dt);
        rebased.surface_y = rotateYaw(surface.surface_y, yaw_dt);
        rebased.surface_z = rotateYaw(surface.surface_z, yaw_dt);
        rebased.yaw = surface.yaw + yaw_dt;
    }
    rebased.surface_z = normalizedOr(rebased.surface_z, Vec3f::UnitZ());
    rebased.surface_x = normalizedOr(rebased.surface_x, Vec3f::UnitX());
    rebased.surface_y = normalizedOr(rebased.surface_z.cross(rebased.surface_x), Vec3f::UnitY());
    rebased.surface_x = normalizedOr(rebased.surface_y.cross(rebased.surface_z), Vec3f::UnitX());
    rebased.yaw_rate = surface.yaw_rate;
    rebased.t = 0.0;
    return rebased;
}

bool localTimeInside(const geometry_utils::Trajectory &traj, const double t)
{
    return !traj.empty() &&
           std::isfinite(t) &&
           t >= -1.0e-6 &&
           t <= traj.getTotalDuration() + 1.0e-6;
}

} // namespace

bool TrackingToPerchingInitializer::build(
        const geometry_utils::Trajectory &tracking_pos,
        const geometry_utils::Trajectory &tracking_yaw,
        const double current_tracking_local_t,
        const traj_opt::PerchingSurfaceState &surface,
        const Config &cfg,
        TrackingToPerchingInitialGuess &guess) const
{
    guess = TrackingToPerchingInitialGuess{};
    if (tracking_pos.empty()) {
        std::cout << " -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_FAILED reason=empty_tracking_pos"
                  << std::endl;
        return false;
    }

    double handover_delay = cfg.tracking_to_perching_handover_delay;
    if (handover_delay <= 0.0) {
        handover_delay = std::max(0.0, cfg.replan_forward_dt);
    }
    const double t_h = current_tracking_local_t + handover_delay;
    if (!localTimeInside(tracking_pos, t_h)) {
        std::cout << " -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_FAILED reason=handover_outside_tracking"
                  << ", current_t=" << current_tracking_local_t
                  << ", handover_delay=" << handover_delay
                  << ", duration=" << tracking_pos.getTotalDuration() << std::endl;
        return false;
    }

    guess.handover_delay = handover_delay;
    guess.head_pvaj = tracking_pos.getState(std::clamp(t_h, 0.0, tracking_pos.getTotalDuration()));
    if (!guess.head_pvaj.allFinite()) {
        std::cout << " -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_FAILED reason=nonfinite_head_state"
                  << std::endl;
        return false;
    }

    if (localTimeInside(tracking_yaw, t_h)) {
        const double yaw_t = std::clamp(t_h, 0.0, tracking_yaw.getTotalDuration());
        guess.head_yaw(0, 0) = tracking_yaw.getPos(yaw_t).x();
        guess.head_yaw(0, 1) = tracking_yaw.getVel(yaw_t).x();
    } else {
        guess.head_yaw.setZero();
    }

    guess.rebased_surface = rebaseSurface(surface, handover_delay, cfg);

    const Vec3f z0 = normalizedOr(guess.rebased_surface.surface_z, Vec3f::UnitZ());
    const Vec3f contact0 =
            guess.rebased_surface.position + std::max(0.0, cfg.perching_robot_l) * z0;
    const double raw_T0 =
            (guess.head_pvaj.col(0) - contact0).norm() /
            std::max(0.5, cfg.tracking_to_perching_reference_speed);
    const double T0 =
            std::clamp(raw_T0,
                       std::max(0.05, cfg.perching_min_duration),
                       std::max(cfg.perching_min_duration,
                                cfg.tracking_to_perching_max_seed_duration));
    guess.total_time = T0;

    appendTimedUnique(guess.head_pvaj.col(0), 0.0, guess.guide_path, guess.guide_t);
    if (cfg.tracking_to_perching_use_tracking_suffix) {
        const double prefix_T =
                std::clamp(cfg.tracking_to_perching_prefix_ratio, 0.0, 1.0) * T0;
        const double max_suffix_t =
                std::max(0.0, tracking_pos.getTotalDuration() - t_h);
        const double sample_T = std::min(prefix_T, max_suffix_t);
        const int sample_num = sample_T > 1.0e-3 ? 3 : 0;
        for (int i = 1; i <= sample_num; ++i) {
            const double s = sample_T * static_cast<double>(i) /
                             static_cast<double>(sample_num + 1);
            appendTimedUnique(tracking_pos.getPos(t_h + s),
                              s,
                              guess.guide_path,
                              guess.guide_t);
        }
    }

    Vec3f x_T;
    Vec3f y_T;
    Vec3f z_T;
    surfaceFrameAt(guess.rebased_surface,
                   cfg.perching_rotate_surface_with_yaw_rate,
                   T0,
                   x_T,
                   y_T,
                   z_T);
    const Vec3f surface_p_T =
            guess.rebased_surface.position +
            guess.rebased_surface.velocity * T0 +
            0.5 * guess.rebased_surface.acceleration * T0 * T0;
    const Vec3f contact_T =
            surface_p_T + std::max(0.0, cfg.perching_robot_l) * z_T;
    const Vec3f pre_T =
            contact_T + std::max(0.0, cfg.perching_pre_contact_distance) * z_T;
    appendTimedUnique(pre_T, 0.85 * T0, guess.guide_path, guess.guide_t);
    appendTimedUnique(contact_T, T0, guess.guide_path, guess.guide_t);

    const double track_tail_t =
            std::min(t_h + T0, tracking_pos.getTotalDuration());
    const Vec3f v_track_T = tracking_pos.getVel(track_tail_t);
    const Vec3f a_track_T = tracking_pos.getAcc(track_tail_t);
    const Vec3f surface_v_T = guess.rebased_surface.velocity +
                              guess.rebased_surface.acceleration * T0;
    const Vec3f rel_v = v_track_T - surface_v_T +
                        std::max(0.0, cfg.perching_v_plus) * z_T;
    guess.nu_seed.x() = std::clamp(rel_v.dot(x_T), -2.0, 2.0);
    guess.nu_seed.y() = std::clamp(rel_v.dot(y_T), -2.0, 2.0);

    const Vec3f gravity_vector(0.0, 0.0, -std::abs(cfg.esdf_traj_cfg.grav));
    const double tau0 = (a_track_T - gravity_vector).dot(z_T);
    const double tau_m = cfg.perching_thrust_nominal;
    const double tau_r = std::max(1.0e-6, cfg.perching_thrust_range);
    const double ratio = std::clamp((tau0 - tau_m) / tau_r, -0.95, 0.95);
    guess.tau_f_seed = std::clamp(std::asin(ratio), -1.3, 1.3);

    guess.valid = guess.guide_path.size() >= 2 &&
                  guess.guide_path.size() == guess.guide_t.size();
    if (!guess.valid) {
        std::cout << " -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_FAILED reason=invalid_guide"
                  << std::endl;
        return false;
    }

    std::cout << " -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_SUCCESS"
              << " handover_delay=" << guess.handover_delay
              << ", T0=" << guess.total_time
              << ", guide_size=" << guess.guide_path.size()
              << ", nu_seed=[" << guess.nu_seed.x() << ", " << guess.nu_seed.y() << "]"
              << ", tau_f_seed=" << guess.tau_f_seed
              << ", head_p=[" << guess.head_pvaj.col(0).x() << ", "
              << guess.head_pvaj.col(0).y() << ", "
              << guess.head_pvaj.col(0).z() << "]"
              << ", p_pre=[" << pre_T.x() << ", " << pre_T.y() << ", " << pre_T.z() << "]"
              << ", p_contact=[" << contact_T.x() << ", " << contact_T.y() << ", "
              << contact_T.z() << "]" << std::endl;
    return true;
}

} // namespace general_planner
