#include "general_core/takeoff/takeoff_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

namespace general_planner
{
namespace
{

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

void appendUnique(const Vec3f &p, vec_E<Vec3f> &path)
{
    if (!p.allFinite())
    {
        return;
    }
    if (path.empty() || (path.back() - p).norm() > 1.0e-4)
    {
        path.emplace_back(p);
    }
}

} // namespace

TakeoffFrontend::TakeoffFrontend(Config cfg,
                                 MapManager::Ptr map_manager,
                                 path_search::Astar::Ptr astar)
    : cfg_(std::move(cfg)),
      map_manager_(std::move(map_manager)),
      astar_(std::move(astar))
{
}

bool TakeoffFrontend::pointSafe(const Vec3f &p) const
{
    if (!p.allFinite())
    {
        return false;
    }
    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        return true;
    }
    if (!map_manager_->insideLocalMap(p))
    {
        return false;
    }
    const auto grid_type = map_manager_->getInfGridType(p);
    if (grid_type == rog_map::GridType::OCCUPIED ||
        grid_type == rog_map::GridType::OUT_OF_MAP)
    {
        return false;
    }
    if (map_manager_->hasESDF() && cfg_.safe_distance > 0.0)
    {
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        if (map_manager_->evaluateESDF(p, dist, grad) &&
            std::isfinite(dist) &&
            dist < cfg_.safe_distance)
        {
            return false;
        }
    }
    return true;
}

bool TakeoffFrontend::appendSafeEscapeSegment(const Vec3f &start,
                                              const Vec3f &goal,
                                              vec_E<Vec3f> &path) const
{
    if (map_manager_ == nullptr || !map_manager_->ready() ||
        map_manager_->isLineFree(start, goal, true, false))
    {
        appendUnique(goal, path);
        return true;
    }

    if (!cfg_.frontend_astar || astar_ == nullptr)
    {
        return false;
    }

    vec_E<Vec3f> astar_path;
    const int prob_flag = path_search::ON_PROB_MAP |
                          path_search::UNKNOWN_AS_FREE |
                          path_search::DONT_USE_INF_NEIGHBOR;
    const int inf_flag = path_search::ON_INF_MAP |
                         path_search::UNKNOWN_AS_FREE |
                         path_search::USE_INF_NEIGHBOR;
    const double horizon = std::max(3.0, (goal - start).norm() + 2.0);
    auto searchSegment = [&](const int flag) {
        astar_path.clear();
        const auto ret =
            astar_->pointToPointPathSearch(start, goal, flag, horizon, astar_path, 0.08);
        return (ret == general_utils::SUCCESS || ret == general_utils::REACH_GOAL) &&
               !astar_path.empty();
    };
    if (!searchSegment(prob_flag) && !searchSegment(inf_flag))
    {
        return false;
    }

    for (const auto &p : astar_path)
    {
        appendUnique(p, path);
    }
    appendUnique(goal, path);
    return true;
}

bool TakeoffFrontend::buildProblem(const traj_opt::PerchingSurfaceState &surface,
                                   traj_opt::DynamicTakeoffProblem &problem) const
{
    traj_opt::PerchingSurfaceState normalized_surface = surface;
    Vec3f z_s = normalizedOr(surface.surface_z, Vec3f::UnitZ());
    Vec3f x_s = normalizedOr(surface.surface_x, Vec3f::UnitX());
    Vec3f y_s = normalizedOr(z_s.cross(x_s), Vec3f::UnitY());
    x_s = normalizedOr(y_s.cross(z_s), Vec3f::UnitX());
    normalized_surface.surface_x = x_s;
    normalized_surface.surface_y = y_s;
    normalized_surface.surface_z = z_s;
    normalized_surface.t = 0.0;

    const Vec3f p_contact = normalized_surface.position + cfg_.robot_l * z_s;
    const double release_distance = std::max(0.15, 0.5 * std::max(0.0, cfg_.robot_l));
    const Vec3f p_release = p_contact + release_distance * z_s;

    Vec3f p_escape = Vec3f::Zero();
    bool escape_ok = false;
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const double height = cfg_.escape_height + 0.35 * static_cast<double>(attempt);
        p_escape = p_contact +
                   std::max(0.0, cfg_.escape_distance) * z_s +
                   height * Vec3f::UnitZ();
        if (pointSafe(p_escape))
        {
            escape_ok = true;
            break;
        }
    }

    if (!pointSafe(p_release))
    {
        std::cout << " -- [TakeoffFrontend] TAKEOFF_BUILD_PROBLEM_FAILED reason=release_not_safe"
                  << ", p_release=[" << p_release.transpose() << "]" << std::endl;
        return false;
    }
    if (!escape_ok)
    {
        std::cout << " -- [TakeoffFrontend] TAKEOFF_BUILD_PROBLEM_FAILED reason=escape_not_safe"
                  << ", p_escape=[" << p_escape.transpose() << "]" << std::endl;
        return false;
    }

    vec_E<Vec3f> guide_path;
    appendUnique(p_contact, guide_path);
    appendUnique(p_release, guide_path);
    if (!appendSafeEscapeSegment(p_release, p_escape, guide_path))
    {
        std::cout << " -- [TakeoffFrontend] TAKEOFF_BUILD_PROBLEM_FAILED reason=escape_segment_blocked"
                  << ", p_release=[" << p_release.transpose() << "]"
                  << ", p_escape=[" << p_escape.transpose() << "]" << std::endl;
        return false;
    }
    if ((guide_path.back() - p_escape).norm() > 1.0e-4)
    {
        appendUnique(p_escape, guide_path);
    }

    std::vector<double> guide_t(guide_path.size(), 0.0);
    const double ref_speed = std::max(0.1, cfg_.reference_speed);
    for (int i = 1; i < static_cast<int>(guide_path.size()); ++i)
    {
        guide_t[static_cast<std::size_t>(i)] =
            guide_t[static_cast<std::size_t>(i - 1)] +
            std::max(0.05,
                     (guide_path[static_cast<std::size_t>(i)] -
                      guide_path[static_cast<std::size_t>(i - 1)])
                         .norm() /
                         ref_speed);
    }
    const double raw_total_t = guide_t.empty() ? 0.0 : guide_t.back();
    const double total_t =
        std::clamp(std::max(raw_total_t, cfg_.min_duration),
                   std::max(0.05, cfg_.min_duration),
                   std::max(cfg_.min_duration, cfg_.max_duration));
    if (raw_total_t > 1.0e-6)
    {
        const double scale = total_t / raw_total_t;
        for (double &t : guide_t)
        {
            t *= scale;
        }
    }
    else if (!guide_t.empty())
    {
        guide_t.back() = total_t;
    }

    problem = traj_opt::DynamicTakeoffProblem{};
    problem.nominal_head_pvaj.setZero();
    problem.nominal_head_pvaj.col(0) = p_contact;
    problem.nominal_head_pvaj.col(1) = normalized_surface.velocity;
    problem.tail_pvaj.setZero();
    problem.tail_pvaj.col(0) = p_escape;
    problem.surface = normalized_surface;
    problem.use_head_mapping = true;
    problem.guide_path = guide_path;
    problem.guide_t = guide_t;
    problem.release_contact_time = std::max(0.0, cfg_.release_contact_time);
    problem.platform_clearance_after_release =
        std::max(0.0, cfg_.platform_clearance_after_release);
    problem.escape_distance = cfg_.escape_distance;
    problem.escape_height = cfg_.escape_height;
    problem.safe_distance = cfg_.safe_distance;
    problem.platform_radius = cfg_.platform_radius;
    problem.robot_radius = cfg_.robot_radius;
    problem.robot_l = cfg_.robot_l;
    problem.platform_clearance = cfg_.platform_clearance;
    problem.piece_num = std::max(1, cfg_.piece_num);
    problem.min_duration = std::max(0.05, cfg_.min_duration);
    problem.max_duration = std::max(problem.min_duration, cfg_.max_duration);
    problem.reference_speed = cfg_.reference_speed;

    problem.boundary.surface = normalized_surface;
    problem.boundary.robot_l = cfg_.robot_l;
    problem.boundary.thrust_nominal = cfg_.thrust_nominal;
    problem.boundary.thrust_range = cfg_.thrust_range;
    problem.boundary.gravity = cfg_.gravity;
    problem.boundary.use_tangent_release_velocity = cfg_.use_tangent_release_velocity;
    problem.boundary.weight_eta = cfg_.weight_eta;
    problem.boundary.weight_tau_f = cfg_.weight_tau_f;
    problem.boundary.rotate_surface_with_yaw_rate = cfg_.rotate_surface_with_yaw_rate;

    std::cout << " -- [TakeoffFrontend] TAKEOFF_BUILD_PROBLEM_SUCCESS T0="
              << total_t << ", guide_size=" << problem.guide_path.size()
              << ", p_contact=[" << p_contact.transpose() << "]"
              << ", p_release=[" << p_release.transpose() << "]"
              << ", p_escape=[" << p_escape.transpose() << "]" << std::endl;
    return problem.guide_path.size() >= 2;
}

} // namespace general_planner
