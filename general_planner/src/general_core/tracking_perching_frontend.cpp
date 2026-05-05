#include "general_core/tracking_perching_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace general_planner
{
namespace
{

using super_utils::StatePVAJ;
using super_utils::Vec3f;
using super_utils::vec_E;

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
    if (path.empty() || (path.back() - p).norm() > 1.0e-4)
    {
        path.emplace_back(p);
    }
}

} // namespace

TrackingFrontend::TrackingFrontend(const Config &cfg,
                                   const MapManager::Ptr &map_manager,
                                   const path_search::Astar::Ptr &astar)
    : cfg_(cfg),
      map_manager_(map_manager),
      astar_(astar)
{
}

bool TrackingFrontend::isViewpointSafe(const Vec3f &viewpoint) const
{
    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        return true;
    }
    if (!map_manager_->insideLocalMap(viewpoint))
    {
        return false;
    }
    if (map_manager_->getInfGridType(viewpoint) == super_utils::GridType::OCCUPIED)
    {
        return false;
    }
    if (map_manager_->hasESDF())
    {
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        if (map_manager_->evaluateESDF(viewpoint, dist, grad) && dist < cfg_.safe_distance)
        {
            return false;
        }
    }
    return true;
}

bool TrackingFrontend::isViewpointVisible(const Vec3f &viewpoint,
                                          const Vec3f &target) const
{
    if (!isViewpointSafe(viewpoint))
    {
        return false;
    }
    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        return true;
    }

    const bool line_free =
        map_manager_->isLineFree(viewpoint,
                                 target,
                                 true,
                                 cfg_.unknown_as_occupied);
    if (!line_free)
    {
        return false;
    }

    if (map_manager_->hasESDF())
    {
        const int sample_num = std::max(1, cfg_.visibility_samples);
        const double view_dist = (target - viewpoint).norm();
        for (int i = 1; i <= sample_num; ++i)
        {
            const double alpha = static_cast<double>(i) / static_cast<double>(sample_num + 1);
            const Vec3f p = viewpoint + alpha * (target - viewpoint);
            double dist = 0.0;
            Vec3f grad = Vec3f::Zero();
            const double required_clearance =
                cfg_.visibility_safe_distance +
                std::max(0.0, cfg_.visibility_cone_ratio) * alpha * view_dist;
            if (map_manager_->evaluateESDF(p, dist, grad) && dist < required_clearance)
            {
                return false;
            }
        }
    }
    return true;
}

Vec3f TrackingFrontend::chooseVisibleViewpoint(const Vec3f &seed,
                                               const traj_opt::DynamicTargetState &target) const
{
    Vec3f base_dir = target.velocity.head<3>();
    base_dir.z() = 0.0;
    if (base_dir.norm() < 1.0e-4)
    {
        base_dir = seed - target.position;
        base_dir.z() = 0.0;
    }
    base_dir = normalizedOr(base_dir, Vec3f::UnitX());

    Vec3f best = target.position - cfg_.tracking_distance * base_dir;
    best.z() = target.position.z() + cfg_.height_offset;
    double best_score = std::numeric_limits<double>::infinity();

    const int angle_count =
        std::max(8, static_cast<int>(std::ceil(2.0 * M_PI / std::max(0.05, cfg_.candidate_angle_step))));
    const int radius_count = std::max(1, cfg_.candidate_radius_num);
    for (int r = 0; r < radius_count; ++r)
    {
        const double radius_offset =
            (static_cast<double>(r) - 0.5 * static_cast<double>(radius_count - 1)) *
            std::max(0.2, cfg_.distance_tolerance);
        const double radius = std::max(0.3, cfg_.tracking_distance + radius_offset);
        for (int i = 0; i < angle_count; ++i)
        {
            const double yaw = static_cast<double>(i) * 2.0 * M_PI / static_cast<double>(angle_count);
            Vec3f dir(std::cos(yaw), std::sin(yaw), 0.0);
            Vec3f candidate = target.position - radius * dir;
            candidate.z() = target.position.z() + cfg_.height_offset;
            if (!isViewpointVisible(candidate, target.position))
            {
                continue;
            }
            const double score = (candidate - seed).squaredNorm() +
                                 0.2 * std::abs(radius - cfg_.tracking_distance);
            if (score < best_score)
            {
                best_score = score;
                best = candidate;
            }
        }
    }

    if (!std::isfinite(best_score) && map_manager_ != nullptr && map_manager_->hasESDF())
    {
        Vec3f nearest = best;
        if (map_manager_->findNearestESDFSafe(best,
                                              cfg_.safe_distance,
                                              nearest,
                                              std::max(1.0, cfg_.distance_tolerance + cfg_.safe_distance)))
        {
            best = nearest;
        }
    }
    return best;
}

bool TrackingFrontend::appendPathSegment(const Vec3f &start,
                                         const Vec3f &goal,
                                         vec_E<Vec3f> &path) const
{
    if (map_manager_ == nullptr || !map_manager_->ready() ||
        map_manager_->isLineFree(start, goal, true, cfg_.unknown_as_occupied))
    {
        appendUnique(goal, path);
        return true;
    }

    if (!cfg_.use_astar || astar_ == nullptr)
    {
        appendUnique(goal, path);
        return true;
    }

    auto searchSegment = [&](const int flag, vec_E<Vec3f> &astar_path) {
        astar_path.clear();
        const auto ret =
            astar_->pointToPointPathSearch(start, goal, flag, cfg_.searching_horizon, astar_path, 0.08);
        return (ret == super_utils::SUCCESS || ret == super_utils::REACH_GOAL) && !astar_path.empty();
    };

    vec_E<Vec3f> astar_path;
    bool found = false;
    if (!cfg_.unknown_as_occupied)
    {
        const int prob_flag = path_search::ON_PROB_MAP |
                              path_search::UNKNOWN_AS_FREE |
                              path_search::DONT_USE_INF_NEIGHBOR;
        found = searchSegment(prob_flag, astar_path);
    }
    if (!found)
    {
        const int inf_flag = path_search::ON_INF_MAP |
                             (cfg_.unknown_as_occupied ? path_search::UNKNOWN_AS_OCCUPIED
                                                       : path_search::UNKNOWN_AS_FREE) |
                             path_search::USE_INF_NEIGHBOR;
        found = searchSegment(inf_flag, astar_path);
    }
    if (!found)
    {
        appendUnique(goal, path);
        return false;
    }
    for (const auto &p : astar_path)
    {
        appendUnique(p, path);
    }
    return true;
}

bool TrackingFrontend::buildProblem(const StatePVAJ &head_pvaj,
                                    const traj_opt::DynamicTargetStates &target_prediction,
                                    traj_opt::TrackingProblem &problem) const
{
    if (target_prediction.empty())
    {
        return false;
    }

    problem = traj_opt::TrackingProblem{};
    problem.head_pvaj = head_pvaj;
    problem.target_prediction = target_prediction;
    problem.safe_distance = cfg_.safe_distance;
    problem.tracking_distance = cfg_.tracking_distance;
    problem.distance_tolerance = cfg_.distance_tolerance;
    problem.height_offset = cfg_.height_offset;
    problem.height_tolerance = cfg_.height_tolerance;
    problem.visibility_safe_distance = cfg_.visibility_safe_distance;
    problem.visibility_cone_ratio = cfg_.visibility_cone_ratio;
    problem.visibility_samples = cfg_.visibility_samples;

    vec_E<Vec3f> guide;
    appendUnique(head_pvaj.col(0), guide);

    Vec3f seed = head_pvaj.col(0);
    for (const auto &target : target_prediction)
    {
        const Vec3f viewpoint = chooseVisibleViewpoint(seed, target);
        appendPathSegment(seed, viewpoint, guide);
        seed = viewpoint;
    }

    problem.guide_path = guide;
    problem.guide_t.clear();
    problem.guide_t.reserve(guide.size());
    double stamp = 0.0;
    problem.guide_t.emplace_back(stamp);
    for (int i = 1; i < static_cast<int>(guide.size()); ++i)
    {
        stamp += std::max(0.1, (guide[i] - guide[i - 1]).norm() / 2.0);
        problem.guide_t.emplace_back(stamp);
    }

    problem.tail_pvaj.setZero();
    problem.tail_pvaj.col(0) = guide.back();
    if (!target_prediction.empty())
    {
        const Vec3f rel = problem.tail_pvaj.col(0) - target_prediction.back().position;
        Vec3f tangent(-rel.y(), rel.x(), 0.0);
        problem.tail_pvaj.col(1) = 0.3 * normalizedOr(tangent, Vec3f::Zero());
    }
    problem.min_total_duration = std::max(0.0, target_prediction.back().t);
    return problem.guide_path.size() >= 2;
}

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
        appendUnique(goal, path);
        return true;
    }

    auto searchSegment = [&](const int flag, vec_E<Vec3f> &astar_path) {
        astar_path.clear();
        const auto ret =
            astar_->pointToPointPathSearch(start, goal, flag, cfg_.searching_horizon, astar_path, 0.08);
        return (ret == super_utils::SUCCESS || ret == super_utils::REACH_GOAL) && !astar_path.empty();
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
        appendUnique(goal, path);
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
    Vec3f z_s = normalizedOr(surface.surface_z, Vec3f::UnitZ());
    Vec3f x_s = normalizedOr(surface.surface_x, Vec3f::UnitX());
    Vec3f y_s = normalizedOr(z_s.cross(x_s), Vec3f::UnitY());
    x_s = normalizedOr(y_s.cross(z_s), Vec3f::UnitX());

    const Vec3f contact = surface.position + cfg_.robot_l * z_s;
    const Vec3f pre_contact = surface.position + (cfg_.robot_l + cfg_.pre_contact_distance) * z_s;

    problem = traj_opt::PerchingProblem{};
    problem.head_pvaj = head_pvaj;
    problem.surface = surface;
    problem.surface.surface_x = x_s;
    problem.surface.surface_y = y_s;
    problem.surface.surface_z = z_s;
    problem.safe_distance = cfg_.safe_distance;
    problem.robot_l = cfg_.robot_l;
    problem.platform_radius = cfg_.platform_radius;
    problem.robot_radius = cfg_.robot_radius;
    problem.platform_clearance = cfg_.platform_clearance;

    problem.nominal_tail_pvaj.setZero();
    problem.nominal_tail_pvaj.col(0) = contact;
    problem.nominal_tail_pvaj.col(1) = surface.velocity - cfg_.v_plus * z_s;

    problem.terminal.plate_position = surface.position;
    problem.terminal.plate_velocity = surface.velocity;
    problem.terminal.plate_acceleration = surface.acceleration;
    problem.terminal.reference_time = surface.t;
    problem.terminal.surface_x = x_s;
    problem.terminal.surface_y = y_s;
    problem.terminal.surface_z = z_s;
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

    appendUnique(head_pvaj.col(0), problem.guide_path);
    appendPathSegment(head_pvaj.col(0), pre_contact, problem.guide_path);
    appendPathSegment(pre_contact, contact, problem.guide_path);

    problem.guide_t.clear();
    problem.guide_t.reserve(problem.guide_path.size());
    double stamp = 0.0;
    problem.guide_t.emplace_back(stamp);
    for (int i = 1; i < static_cast<int>(problem.guide_path.size()); ++i)
    {
        stamp += std::max(0.1, (problem.guide_path[i] - problem.guide_path[i - 1]).norm() / 1.5);
        problem.guide_t.emplace_back(stamp);
    }
    return problem.guide_path.size() >= 2;
}

} // namespace general_planner
