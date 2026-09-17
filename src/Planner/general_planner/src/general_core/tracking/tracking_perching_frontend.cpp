#include "general_core/tracking/tracking_perching_frontend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>

namespace general_planner
{
namespace
{

using general_utils::StatePVAJ;
using general_utils::Vec3f;
using general_utils::vec_E;

Vec3f normalizedOr(const Vec3f &v, const Vec3f &fallback)
{
    if (!v.allFinite() || v.norm() < 1.0e-6)
    {
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

double estimateMovingTargetInterceptTime(const Vec3f &head_p,
                                         const Vec3f &head_v,
                                         const Vec3f &target_p,
                                         const Vec3f &target_v,
                                         const double travel_speed,
                                         const double min_t,
                                         const double max_t)
{
    const double lower = std::max(0.05, min_t);
    const double upper = std::max(lower, max_t);
    const double speed = std::max(0.5, travel_speed);
    const Vec3f rel_p = target_p - head_p;
    const Vec3f rel_v = target_v - head_v;
    const double a = rel_v.squaredNorm() - speed * speed;
    const double b = 2.0 * rel_p.dot(rel_v);
    const double c = rel_p.squaredNorm();

    double best_t = std::numeric_limits<double>::infinity();
    auto considerRoot = [&](const double t) {
        if (std::isfinite(t) && t > 1.0e-6 && t < best_t)
        {
            best_t = t;
        }
    };

    if (std::abs(a) < 1.0e-9)
    {
        if (std::abs(b) > 1.0e-9)
        {
            considerRoot(-c / b);
        }
    }
    else
    {
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant >= 0.0)
        {
            const double sqrt_disc = std::sqrt(discriminant);
            considerRoot((-b - sqrt_disc) / (2.0 * a));
            considerRoot((-b + sqrt_disc) / (2.0 * a));
        }
    }

    if (!std::isfinite(best_t))
    {
        if (rel_p.dot(rel_v) > 0.0 && rel_v.norm() >= speed)
        {
            return upper;
        }
        return std::clamp(rel_p.norm() / speed, lower, upper);
    }
    return std::clamp(best_t, lower, upper);
}

double estimateTrapezoidalDuration(const double length,
                                   double start_speed,
                                   double end_speed,
                                   double max_vel,
                                   double max_acc)
{
    if (!std::isfinite(length) || length < 1.0e-6)
    {
        return 0.0;
    }

    max_vel = std::max(0.5, max_vel);
    max_acc = std::max(0.5, max_acc);
    start_speed = std::clamp(start_speed, 0.0, max_vel);
    end_speed = std::clamp(end_speed, 0.0, max_vel);

    const double acc_len =
        std::max(0.0, (max_vel * max_vel - start_speed * start_speed) /
                          (2.0 * max_acc));
    const double dec_len =
        std::max(0.0, (max_vel * max_vel - end_speed * end_speed) /
                          (2.0 * max_acc));
    if (length > acc_len + dec_len)
    {
        return (max_vel - start_speed) / max_acc +
               (max_vel - end_speed) / max_acc +
               (length - acc_len - dec_len) / max_vel;
    }

    const double peak_sq =
        std::max(0.0,
                 0.5 * (start_speed * start_speed + end_speed * end_speed) +
                     max_acc * length);
    const double peak = std::sqrt(peak_sq);
    return std::max(0.0, (peak - start_speed) / max_acc) +
           std::max(0.0, (peak - end_speed) / max_acc);
}

double vectorAngle(const Vec3f &lhs, const Vec3f &rhs)
{
    if (!lhs.allFinite() || !rhs.allFinite() ||
        lhs.norm() < 1.0e-6 || rhs.norm() < 1.0e-6)
    {
        return 0.0;
    }
    const double c = std::clamp(lhs.normalized().dot(rhs.normalized()),
                                -1.0,
                                1.0);
    return std::acos(c);
}

double estimatePerchingDynamicDuration(
    const StatePVAJ &head_pvaj,
    const Vec3f &pre_contact,
    const Vec3f &contact,
    const Vec3f &tail_velocity,
    const Vec3f &tail_acceleration,
    const PerchingFrontend::Config &cfg)
{
    const Vec3f head_p = head_pvaj.col(0);
    const Vec3f head_v = head_pvaj.col(1);
    const Vec3f head_a = head_pvaj.col(2);
    const double path_length =
        (pre_contact - head_p).norm() + (contact - pre_contact).norm();
    const double translational =
        estimateTrapezoidalDuration(path_length,
                                    head_v.norm(),
                                    tail_velocity.norm(),
                                    cfg.max_speed,
                                    cfg.max_acc);

    const Vec3f gravity_up(0.0, 0.0, std::abs(cfg.gravity));
    const Vec3f head_thrust = head_a + gravity_up;
    const Vec3f tail_thrust = tail_acceleration + gravity_up;
    const double attitude =
        cfg.max_omega > 0.0
            ? 1.15 * vectorAngle(head_thrust, tail_thrust) /
                  std::max(0.5, cfg.max_omega)
            : 0.0;
    const double jerk =
        cfg.max_jerk > 0.0
            ? 1.10 * (tail_acceleration - head_a).norm() /
                  std::max(1.0, cfg.max_jerk)
            : 0.0;

    return std::max({std::max(0.05, cfg.min_duration),
                     translational,
                     attitude,
                     jerk});
}


void appendUnique(const Vec3f &p, vec_E<Vec3f> &path)
{
    if (path.empty() || (path.back() - p).norm() > 1.0e-4)
    {
        path.emplace_back(p);
    }
}

void appendTimedUnique(const Vec3f &p,
                       const double t,
                       vec_E<Vec3f> &path,
                       std::vector<double> &path_t)
{
    if (!p.allFinite() || !std::isfinite(t))
    {
        return;
    }
    if (path.empty() || (path.back() - p).norm() > 1.0e-4)
    {
        path.emplace_back(p);
        path_t.emplace_back(t);
    }
    else if (!path_t.empty())
    {
        path_t.back() = t;
    }
}

void appendTimedPath(const vec_E<Vec3f> &segment_path,
                     const double start_t,
                     const double end_t,
                     vec_E<Vec3f> &path,
                     std::vector<double> &path_t)
{
    if (segment_path.empty())
    {
        return;
    }

    std::vector<double> accum(segment_path.size(), 0.0);
    for (int i = 1; i < static_cast<int>(segment_path.size()); ++i)
    {
        accum[static_cast<std::size_t>(i)] =
            accum[static_cast<std::size_t>(i - 1)] +
            (segment_path[static_cast<std::size_t>(i)] -
             segment_path[static_cast<std::size_t>(i - 1)])
                .norm();
    }

    const double total_len = accum.back();
    const double safe_end_t = std::max(end_t, start_t);
    for (int i = 0; i < static_cast<int>(segment_path.size()); ++i)
    {
        const double ratio = total_len > 1.0e-6
                                 ? accum[static_cast<std::size_t>(i)] / total_len
                                 : 1.0;
        const double stamp = start_t + ratio * (safe_end_t - start_t);
        appendTimedUnique(segment_path[static_cast<std::size_t>(i)], stamp, path, path_t);
    }
}

std::string pointToString(const Vec3f &p)
{
    return "[" + std::to_string(p.x()) + ", " +
           std::to_string(p.y()) + ", " +
           std::to_string(p.z()) + "]";
}

} // namespace

PerchingFrontend::PerchingFrontend(const Config &cfg,
                                   const MapManager::Ptr &map_manager,
                                   const path_search::Astar::Ptr &astar)
    : cfg_(cfg),
      map_manager_(map_manager),
      astar_(astar)
{
}

bool PerchingFrontend::appendPathSegment(const Vec3f &start,
                                         const Vec3f &goal,
                                         vec_E<Vec3f> &path) const
{
    if (map_manager_ == nullptr || !map_manager_->ready() ||
        map_manager_->isLineFree(start, goal, true, false))
    {
        appendUnique(goal, path);
        return true;
    }

    if (!cfg_.use_astar || astar_ == nullptr)
    {
        return false;
    }

    auto searchSegment = [&](const int flag, vec_E<Vec3f> &astar_path) {
        astar_path.clear();
        const auto ret =
            astar_->pointToPointPathSearch(start, goal, flag, cfg_.searching_horizon, astar_path, 0.08);
        return (ret == general_utils::SUCCESS || ret == general_utils::REACH_GOAL) && !astar_path.empty();
    };

    vec_E<Vec3f> astar_path;
    const int prob_flag = path_search::ON_PROB_MAP |
                          path_search::UNKNOWN_AS_FREE |
                          path_search::DONT_USE_INF_NEIGHBOR;
    const int inf_flag = path_search::ON_INF_MAP |
                         path_search::UNKNOWN_AS_FREE |
                         path_search::USE_INF_NEIGHBOR;
    if (!searchSegment(prob_flag, astar_path) && !searchSegment(inf_flag, astar_path))
    {
        return false;
    }
    for (const auto &p : astar_path)
    {
        appendUnique(p, path);
    }
    return true;
}

bool PerchingFrontend::buildProblem(const StatePVAJ &head_pvaj,
                                    const traj_opt::PerchingSurfaceState &surface,
                                    traj_opt::PerchingProblem &problem) const
{
    const bool use_tracking_warm_start =
        problem.use_tracking_warm_start &&
        problem.init_total_time > 0.0 &&
        problem.warm_start_guide_path.size() >= 2;
    const double warm_start_total_time = problem.init_total_time;
    const Eigen::Vector2d warm_start_nu = problem.init_nu;
    const double warm_start_tau_f = problem.init_tau_f;
    const auto warm_start_guide_path = problem.warm_start_guide_path;
    const auto warm_start_guide_t = problem.warm_start_guide_t;
    const auto warm_start_head_yaw = problem.warm_start_head_yaw;

    Vec3f z_s = normalizedOr(surface.surface_z, Vec3f::UnitZ());
    Vec3f x_s = normalizedOr(surface.surface_x, Vec3f::UnitX());
    Vec3f y_s = normalizedOr(z_s.cross(x_s), Vec3f::UnitY());
    x_s = normalizedOr(y_s.cross(z_s), Vec3f::UnitX());
    auto surfaceFrameAt = [&](const double t, Vec3f &x_out, Vec3f &y_out, Vec3f &z_out) {
        x_out = x_s;
        y_out = y_s;
        z_out = z_s;
        if (cfg_.rotate_surface_with_yaw_rate && std::abs(surface.yaw_rate) > 1.0e-9)
        {
            const double yaw_dt = surface.yaw_rate * t;
            x_out = rotateYaw(x_out, yaw_dt);
            y_out = rotateYaw(y_out, yaw_dt);
            z_out = rotateYaw(z_out, yaw_dt);
            z_out = normalizedOr(z_out, z_s);
            x_out = normalizedOr(x_out, x_s);
            y_out = normalizedOr(z_out.cross(x_out), y_s);
            x_out = normalizedOr(y_out.cross(z_out), x_s);
        }
    };

    const Vec3f contact_seed = surface.position + cfg_.robot_l * z_s;
    const double surface_speed = surface.velocity.norm();
    const double max_speed = std::max(0.5, cfg_.max_speed);
    const Vec3f gravity_vector(0.0, 0.0, -std::abs(cfg_.gravity));
    const double v_ref =
        std::clamp(std::max({std::max(0.5, cfg_.reference_speed),
                             surface_speed + std::max(0.0, cfg_.v_plus) + 0.5,
                             0.75 * max_speed}),
                   0.5,
                   std::max(0.5, 0.95 * max_speed));
    const double max_duration = std::max(cfg_.min_duration, cfg_.max_duration);
    const double intercept_T =
        estimateMovingTargetInterceptTime(head_pvaj.col(0),
                                          head_pvaj.col(1),
                                          contact_seed,
                                          surface.velocity,
                                          v_ref,
                                          std::max(0.05, cfg_.min_duration),
                                          max_duration);
    const double seed_T =
        use_tracking_warm_start ? warm_start_total_time : intercept_T;
    double T0 =
        std::clamp(seed_T,
                   std::max(0.05, cfg_.min_duration),
                   max_duration);
    double dynamic_T = T0;
    for (int iter = 0; iter < 3; ++iter)
    {
        Vec3f x_iter = x_s;
        Vec3f y_iter = y_s;
        Vec3f z_iter = z_s;
        surfaceFrameAt(T0, x_iter, y_iter, z_iter);
        (void)x_iter;
        (void)y_iter;
        const Vec3f surface_p_iter =
            surface.position + surface.velocity * T0 +
            0.5 * surface.acceleration * T0 * T0;
        const Vec3f surface_v_iter = surface.velocity + surface.acceleration * T0;
        const Vec3f contact_iter = surface_p_iter + cfg_.robot_l * z_iter;
        const Vec3f pre_contact_iter =
            contact_iter + cfg_.pre_contact_distance * z_iter;
        const Vec3f tail_v_iter = surface_v_iter - cfg_.v_plus * z_iter;
        const Vec3f tail_a_iter =
            cfg_.thrust_nominal * z_iter + gravity_vector;
        dynamic_T =
            estimatePerchingDynamicDuration(head_pvaj,
                                            pre_contact_iter,
                                            contact_iter,
                                            tail_v_iter,
                                            tail_a_iter,
                                            cfg_);
        const double required_T = std::max(seed_T, dynamic_T);
        if (std::abs(required_T - T0) < 0.03)
        {
            T0 = required_T;
            break;
        }
        T0 = std::clamp(required_T,
                        std::max(0.05, cfg_.min_duration),
                        max_duration);
    }

    if (!cfg_.allow_long_standalone &&
        T0 > max_duration - std::max(0.0, cfg_.duration_margin))
    {
        std::cout << " -- [PerchingFrontend] PERCHING_NOT_READY reason=required_duration_exceeds_bound T0="
                  << T0 << ", intercept_T=" << intercept_T
                  << ", dynamic_T=" << dynamic_T
                  << ", max=" << max_duration << std::endl;
        return false;
    }
    T0 = std::clamp(T0, std::max(0.05, cfg_.min_duration), max_duration);

    Vec3f x_s_T = x_s;
    Vec3f y_s_T = y_s;
    Vec3f z_s_T = z_s;
    surfaceFrameAt(T0, x_s_T, y_s_T, z_s_T);    

    const Vec3f surface_p_T =
        surface.position + surface.velocity * T0 + 0.5 * surface.acceleration * T0 * T0;
    const Vec3f surface_v_T = surface.velocity + surface.acceleration * T0;
    const double surface_yaw_T = surface.yaw + surface.yaw_rate * T0;
    (void)surface_yaw_T;

    const Vec3f contact = surface_p_T + cfg_.robot_l * z_s_T;
    const Vec3f pre_contact = contact + cfg_.pre_contact_distance * z_s_T;

    problem = traj_opt::PerchingProblem{};
    problem.head_pvaj = head_pvaj;
    problem.surface = surface;
    if (cfg_.reset_surface_time)
    {
        problem.surface.t = 0.0;
    }
    problem.surface.surface_x = x_s;
    problem.surface.surface_y = y_s;
    problem.surface.surface_z = z_s;
    problem.safe_distance = cfg_.safe_distance;
    problem.robot_l = cfg_.robot_l;
    problem.platform_radius = cfg_.platform_radius;
    problem.robot_radius = cfg_.robot_radius;
    problem.platform_clearance = cfg_.platform_clearance;
    problem.relative_z_min = cfg_.relative_z_min;
    problem.relative_z_max = cfg_.relative_z_max;
    problem.weight_relative_height = cfg_.weight_relative_height;
    problem.visual_min_distance = cfg_.visual_min_distance;
    problem.visual_activation_distance = cfg_.visual_activation_distance;
    problem.visual_fx = cfg_.visual_fx;
    problem.visual_fy = cfg_.visual_fy;
    const double max_total_duration_cfg =
        cfg_.max_total_duration > 0.0 ? cfg_.max_total_duration : max_duration;
    const double guide_length_seed =
        (pre_contact - head_pvaj.col(0)).norm() +
        (contact - pre_contact).norm();
    const int time_piece_num =
        static_cast<int>(std::ceil(T0 / std::max(0.2, cfg_.max_piece_duration)));
    const int distance_piece_num =
        static_cast<int>(std::ceil(guide_length_seed /
                                   std::max(0.6, 0.45 * max_speed)));
    const int max_piece_by_duration =
        max_total_duration_cfg > 0.0 && cfg_.min_piece_duration > 0.0
            ? std::max(1,
                       static_cast<int>(std::floor(max_total_duration_cfg /
                                                   cfg_.min_piece_duration)))
            : std::max(1, cfg_.max_piece_num);
    const int piece_upper =
        std::max(std::max(1, cfg_.min_piece_num),
                 std::min(std::max(1, cfg_.max_piece_num),
                          max_piece_by_duration));
    const int adaptive_piece_num =
        std::clamp(std::max(time_piece_num, distance_piece_num),
                   std::max(1, cfg_.min_piece_num),
                   piece_upper);
    problem.piece_num =
        std::min(piece_upper, std::max({cfg_.piece_num, 2, adaptive_piece_num}));
    problem.min_piece_duration = cfg_.min_piece_duration;
    problem.max_total_duration = max_total_duration_cfg;
    problem.min_total_duration = std::max(cfg_.min_total_duration, cfg_.min_duration);
    problem.time_lower_bound_weight = cfg_.time_lower_bound_weight;
    problem.time_upper_bound_weight = cfg_.time_upper_bound_weight;
    problem.duration_seed = T0;
    problem.duration_seed_weight = cfg_.duration_seed_weight;

    problem.nominal_tail_pvaj.setZero();
    problem.nominal_tail_pvaj.col(0) = contact;
    problem.nominal_tail_pvaj.col(1) = surface_v_T - cfg_.v_plus * z_s_T;
    problem.nominal_tail_pvaj.col(2) = cfg_.thrust_nominal * z_s_T + gravity_vector;

    problem.terminal.plate_position = surface.position;
    problem.terminal.plate_velocity = surface.velocity;
    problem.terminal.plate_acceleration = surface.acceleration;
    problem.terminal.reference_time = cfg_.reset_surface_time ? 0.0 : surface.t;
    problem.terminal.surface_x = x_s;
    problem.terminal.surface_y = y_s;
    problem.terminal.surface_z = z_s;
    problem.terminal.yaw = surface.yaw;
    problem.terminal.yaw_rate = surface.yaw_rate;
    problem.terminal.rotate_surface_with_yaw_rate = cfg_.rotate_surface_with_yaw_rate;
    problem.terminal.gravity = std::abs(cfg_.gravity);
    problem.terminal.terminal_time_seed = T0;
    problem.terminal.robot_l = cfg_.robot_l;
    problem.terminal.v_plus = cfg_.v_plus;
    problem.terminal.thrust_nominal = cfg_.thrust_nominal;
    problem.terminal.thrust_range = cfg_.thrust_range;
    problem.terminal.use_dynamics_terminal_accel = cfg_.use_dynamics_terminal_accel;
    problem.terminal.pre_contact_distance = cfg_.pre_contact_distance;
    problem.terminal.terminal_relax_time = cfg_.terminal_relax_time;
    problem.terminal.weight_nu = cfg_.weight_nu;
    problem.terminal.weight_tau_f = cfg_.weight_tau_f;
    problem.use_terminal_config = true;
    problem.use_tracking_warm_start = use_tracking_warm_start;
    problem.init_total_time = use_tracking_warm_start ? T0 : 0.0;
    problem.init_nu = use_tracking_warm_start ? warm_start_nu : Eigen::Vector2d::Zero();
    problem.init_tau_f = use_tracking_warm_start ? warm_start_tau_f : 0.0;
    problem.warm_start_head_yaw = warm_start_head_yaw;

    double tau0 = cfg_.thrust_nominal;
    if (head_pvaj.col(2).allFinite())
    {
        tau0 = (head_pvaj.col(2) - gravity_vector).dot(z_s_T);
    }
    const double thrust_range = std::max(1.0e-6, cfg_.thrust_range);
    const double tau_min = cfg_.thrust_nominal - thrust_range + 1.0e-4;
    const double tau_max = cfg_.thrust_nominal + thrust_range - 1.0e-4;
    tau0 = std::clamp(tau0, tau_min, tau_max);
    const double tau_f_seed = std::asin(
        std::clamp((tau0 - cfg_.thrust_nominal) / thrust_range, -1.0, 1.0));
    problem.terminal.nu_seed = Eigen::Vector2d::Zero();
    problem.terminal.tau_f_seed =
        std::clamp(tau_f_seed,
                   -std::max(0.0, cfg_.tau_f_seed_limit),
                   std::max(0.0, cfg_.tau_f_seed_limit));;
    if (use_tracking_warm_start) {
        problem.terminal.nu_seed = warm_start_nu;
        problem.terminal.tau_f_seed =
            std::clamp(warm_start_tau_f,
                       -std::max(0.0, cfg_.tau_f_seed_limit),
                       std::max(0.0, cfg_.tau_f_seed_limit));
    }

    problem.guide_path.clear();
    problem.guide_t.clear();
    if (use_tracking_warm_start) {
        problem.guide_path = warm_start_guide_path;
        problem.guide_t = warm_start_guide_t;
        if (problem.guide_t.size() != problem.guide_path.size()) {
            problem.guide_t.clear();
            problem.guide_t.reserve(problem.guide_path.size());
            for (int i = 0; i < static_cast<int>(problem.guide_path.size()); ++i) {
                problem.guide_t.emplace_back(T0 * static_cast<double>(i) /
                                             static_cast<double>(std::max(1, static_cast<int>(problem.guide_path.size()) - 1)));
            }
        }
        problem.guide_t.front() = 0.0;
        problem.guide_t.back() = T0;
    } else {
        vec_E<Vec3f> guide_nodes;
        std::vector<double> guide_node_t;
        appendTimedUnique(head_pvaj.col(0), 0.0, guide_nodes, guide_node_t);
        if (cfg_.multi_point_guide_enable && T0 > 2.0)
        {
           const int sample_num = std::max(2, cfg_.moving_guide_sample_num);
            for (int k = 1; k < sample_num; ++k)
            {
                const double tk = T0 * static_cast<double>(k) / static_cast<double>(sample_num);
                Vec3f x_k, y_k, z_k;
                surfaceFrameAt(tk, x_k, y_k, z_k);
                (void)x_k;
                (void)y_k;
                const Vec3f surface_p_k =
                    surface.position + surface.velocity * tk + 0.5 * surface.acceleration * tk * tk;
                const Vec3f pre_k =
                    surface_p_k + cfg_.robot_l * z_k + cfg_.pre_contact_distance * z_k;
                appendTimedUnique(pre_k, tk, guide_nodes, guide_node_t);
            }
            appendTimedUnique(contact, T0, guide_nodes, guide_node_t);
        }
        else
        {
            appendTimedUnique(pre_contact, 0.7 * T0, guide_nodes, guide_node_t);
            appendTimedUnique(contact, T0, guide_nodes, guide_node_t);
        }

        for (int i = 0; i + 1 < static_cast<int>(guide_nodes.size()); ++i)
        {
            vec_E<Vec3f> segment;
            appendUnique(guide_nodes[static_cast<std::size_t>(i)], segment);
            if (!appendPathSegment(guide_nodes[static_cast<std::size_t>(i)],
                                   guide_nodes[static_cast<std::size_t>(i + 1)],
                                   segment))
            {
                std::cout << " -- [PerchingFrontend] PERCHING_GUIDE_SEGMENT_BLOCKED from="
                          << pointToString(guide_nodes[static_cast<std::size_t>(i)])
                          << ", to=" << pointToString(guide_nodes[static_cast<std::size_t>(i + 1)])
                          << std::endl;
                return false;
            }
            appendTimedPath(segment,
                            guide_node_t[static_cast<std::size_t>(i)],
                            guide_node_t[static_cast<std::size_t>(i + 1)],
                            problem.guide_path,
                            problem.guide_t);
        }
    }

    problem.use_initial_guess = true;
    problem.initial_guess.valid = true;
    problem.initial_guess.total_time = T0;
    problem.initial_guess.guide_path = problem.guide_path;
    problem.initial_guess.guide_t = problem.guide_t;
    problem.initial_guess.nu = problem.terminal.nu_seed;
    problem.initial_guess.tau_f = problem.terminal.tau_f_seed;

    std::cout << " -- [PerchingFrontend] PERCHING_BUILD_PROBLEM_SUCCESS T0="
              << T0 << ", intercept_T=" << intercept_T
              << ", dynamic_T=" << dynamic_T
              << ", piece_num=" << problem.piece_num
              << ", guide_size=" << problem.guide_path.size()
              << ", tracking_warm_start=" << use_tracking_warm_start
              << ", nu_seed=[" << problem.initial_guess.nu.x()
              << ", " << problem.initial_guess.nu.y() << "]"
              << ", tau_f_seed=" << problem.terminal.tau_f_seed
              << ", max_total_duration=" << problem.max_total_duration << std::endl;
    return problem.guide_path.size() >= 2;
}

} // namespace general_planner
