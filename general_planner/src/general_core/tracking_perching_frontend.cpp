#include "general_core/tracking_perching_frontend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
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

void logTrackingFrontendFailure(const bool print_log, const std::string &message)
{
    if (!print_log)
    {
        return;
    }
    static int log_count = 0;
    if (log_count++ < 20)
    {
        std::cout << " -- [TrackingFrontend] " << message << std::endl;
    }
}

void logTrackingFrontendDebug(const bool print_log, const std::string &message)
{
    if (!print_log)
    {
        return;
    }
    static int log_count = 0;
    if (log_count++ < 120)
    {
        std::cout << " -- [TrackingFrontend] " << message << std::endl;
    }
}

void logUnsafeViewpointRejection(const bool print_log, const std::string &message)
{
    if (!print_log)
    {
        return;
    }
    static int log_count = 0;
    if (log_count++ < 60)
    {
        std::cout << " -- [TrackingFrontend] " << message << std::endl;
    }
}

std::string pointToString(const Vec3f &p)
{
    return "[" + std::to_string(p.x()) + ", " +
           std::to_string(p.y()) + ", " +
           std::to_string(p.z()) + "]";
}

std::string trackingFrontendLogPath()
{
    return std::string(ROOT_DIR) + "log/tracking_frontend_latest.csv";
}

struct TrackingFrontendLogQuality
{
    bool tracking_success{false};
    std::string tracking_state{"lost"};
    bool in_od_band{false};
    bool in_height_band{false};
    bool visible{false};
    double visibility_margin{0.0};
};

void writeTrackingFrontendLogHeader(std::ofstream &log)
{
    log << "sample,t,stage,success,frontend_success,tracking_state,"
           "target_x,target_y,target_z,seed_x,seed_y,seed_z,"
           "viewpoint_x,viewpoint_y,viewpoint_z,horizontal_dist,vertical_offset,"
           "in_od_band,in_height_band,visible,visibility_margin,"
           "score,od_cost,oe_margin,path_cost,preferred_cost,source_bias,path_points,message\n";
}

void writeTrackingFrontendLogRow(std::ofstream &log,
                                 const std::size_t sample,
                                 const double t,
                                 const std::string &stage,
                                 const bool frontend_success,
                                 const TrackingFrontendLogQuality &quality,
                                 const Vec3f &target,
                                 const Vec3f &seed,
                                 const Vec3f &viewpoint,
                                 const double score,
                                 const double od_cost,
                                 const double oe_margin,
                                 const double path_cost,
                                 const double preferred_cost,
                                 const double source_bias,
                                 const std::size_t path_points,
                                 const std::string &message)
{
    if (!log.is_open() || !log.good())
    {
        return;
    }
    const Vec3f rel = viewpoint - target;
    const double horizontal = rel.head<2>().norm();
    const double vertical = rel.z();
    log << sample << ','
        << std::fixed << std::setprecision(6) << t << ','
        << stage << ','
        << (quality.tracking_success ? 1 : 0) << ','
        << (frontend_success ? 1 : 0) << ','
        << quality.tracking_state << ','
        << target.x() << ',' << target.y() << ',' << target.z() << ','
        << seed.x() << ',' << seed.y() << ',' << seed.z() << ','
        << viewpoint.x() << ',' << viewpoint.y() << ',' << viewpoint.z() << ','
        << horizontal << ',' << vertical << ','
        << (quality.in_od_band ? 1 : 0) << ','
        << (quality.in_height_band ? 1 : 0) << ','
        << (quality.visible ? 1 : 0) << ','
        << quality.visibility_margin << ',';
    log << score << ','
        << od_cost << ','
        << oe_margin << ','
        << path_cost << ','
        << preferred_cost << ','
        << source_bias << ',';
    log << path_points << ',' << message << '\n';
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

Vec3f TrackingFrontend::preferredViewpoint(const Vec3f &seed,
                                           const traj_opt::DynamicTargetState &target) const
{
    Vec3f preferred_rel_dir = target.velocity;
    preferred_rel_dir.z() = 0.0;
    if (preferred_rel_dir.norm() > 1.0e-4)
    {
        preferred_rel_dir = -preferred_rel_dir.normalized();
    }
    else
    {
        preferred_rel_dir = seed - target.position;
        preferred_rel_dir.z() = 0.0;
        preferred_rel_dir = normalizedOr(preferred_rel_dir, Vec3f::UnitX());
    }

    Vec3f preferred = target.position + cfg_.tracking_distance * preferred_rel_dir;
    preferred.z() = target.position.z() + cfg_.height_offset;
    return preferred;
}

double TrackingFrontend::estimateVisibilityMargin(const Vec3f &viewpoint,
                                                  const Vec3f &target) const
{
    if (!viewpoint.allFinite() || !target.allFinite())
    {
        return -std::max(0.0, cfg_.visibility_safe_distance);
    }
    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        return cfg_.visibility_safe_distance;
    }
    if (!isViewpointSafe(viewpoint) ||
        !map_manager_->isLineFree(viewpoint, target, false, cfg_.unknown_as_occupied))
    {
        return -std::max(0.0, cfg_.visibility_safe_distance);
    }
    if (!map_manager_->hasESDF())
    {
        return cfg_.visibility_safe_distance;
    }

    const Vec3f rel = target - viewpoint;
    const double view_dist = rel.norm();
    if (view_dist < 1.0e-6)
    {
        return -std::max(0.0, cfg_.visibility_safe_distance);
    }

    double min_margin = std::numeric_limits<double>::infinity();
    const int sample_num = std::max(1, cfg_.visibility_samples);
    const double cone_ratio = std::max(0.0, cfg_.visibility_cone_ratio);
    for (int k = 1; k <= sample_num; ++k)
    {
        const double alpha = static_cast<double>(k) / static_cast<double>(sample_num + 1);
        const Vec3f center = viewpoint + alpha * rel;
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        if (!map_manager_->evaluateESDF(center, dist, grad))
        {
            continue;
        }
        const double radius = std::max(0.0, cfg_.visibility_safe_distance) +
                              cone_ratio * alpha * view_dist;
        min_margin = std::min(min_margin, dist - radius);
    }

    if (!std::isfinite(min_margin))
    {
        return cfg_.visibility_safe_distance;
    }
    return min_margin;
}

bool TrackingFrontend::scoreViewpointCandidate(const Vec3f &candidate,
                                               const Vec3f &seed,
                                               const Vec3f &preferred,
                                               const traj_opt::DynamicTargetState &target,
                                               const double source_bias,
                                               ViewpointCandidate &scored) const
{
    if (!isViewpointSafe(candidate))
    {
        return false;
    }

    const Vec3f rel = candidate - target.position;
    const double horizontal = rel.head<2>().norm();
    const double min_h = std::max(0.3, cfg_.tracking_distance - std::max(0.0, cfg_.distance_tolerance));
    const double max_h = std::max(min_h + 0.05,
                                  cfg_.tracking_distance + std::max(0.0, cfg_.distance_tolerance));
    const double vertical = rel.z();
    const double vertical_error =
        std::max(0.0, std::abs(vertical - cfg_.height_offset) - std::max(0.0, cfg_.height_tolerance));
    const double near_error = std::max(0.0, min_h - horizontal);
    const double far_error = std::max(0.0, horizontal - max_h);
    const double desired_error = std::abs(horizontal - cfg_.tracking_distance);
    const double od_cost = 4.0 * near_error * near_error +
                           far_error * far_error +
                           vertical_error * vertical_error +
                           0.05 * desired_error * desired_error;

    const double oe_margin = estimateVisibilityMargin(candidate, target.position);
    const double oe_violation = std::max(0.0, -oe_margin);
    const double path_cost = (candidate - seed).norm();
    const double preferred_cost = (candidate - preferred).norm();
    const double clearance_reward =
        std::clamp(oe_margin, 0.0, std::max(0.3, cfg_.tracking_distance));

    scored.point = candidate;
    scored.od_cost = od_cost;
    scored.oe_margin = oe_margin;
    scored.path_cost = path_cost;
    scored.preferred_cost = preferred_cost;
    scored.source_bias = source_bias;
    scored.score =
        std::max(0.0, cfg_.score_path_weight) * path_cost +
        std::max(0.0, cfg_.score_preferred_weight) * preferred_cost +
        std::max(0.0, cfg_.score_od_weight) * od_cost +
        std::max(0.0, cfg_.score_oe_weight) * oe_violation * oe_violation +
        std::max(0.0, cfg_.score_source_bias_weight) * source_bias -
        std::max(0.0, cfg_.score_clearance_reward) * clearance_reward;
    return std::isfinite(scored.score);
}

bool TrackingFrontend::isViewpointSafe(const Vec3f &viewpoint) const
{
    if (!viewpoint.allFinite())
    {
        logUnsafeViewpointRejection(cfg_.print_log, "Unsafe viewpoint rejected: non-finite point.");
        return false;
    }
    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        return true;
    }
    if (!map_manager_->insideLocalMap(viewpoint))
    {
        logUnsafeViewpointRejection(cfg_.print_log,
                                    "Unsafe viewpoint rejected: outside local map, p=" + pointToString(viewpoint));
        return false;
    }
    const auto inf_grid_type = map_manager_->getInfGridType(viewpoint);
    if (inf_grid_type == super_utils::GridType::OCCUPIED ||
        inf_grid_type == super_utils::GridType::OUT_OF_MAP)
    {
        logUnsafeViewpointRejection(cfg_.print_log,
                                    "Unsafe viewpoint rejected: occupied/out-of-map, p=" + pointToString(viewpoint));
        return false;
    }
    if (cfg_.unknown_as_occupied &&
        (inf_grid_type == super_utils::GridType::UNKNOWN ||
         inf_grid_type == super_utils::GridType::UNDEFINED ||
         inf_grid_type == super_utils::GridType::FRONTIER))
    {
        logUnsafeViewpointRejection(cfg_.print_log,
                                    "Unsafe viewpoint rejected: unknown treated as occupied, p=" + pointToString(viewpoint));
        return false;
    }
    if (map_manager_->hasESDF())
    {
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        if (map_manager_->evaluateESDF(viewpoint, dist, grad) && dist < cfg_.safe_distance)
        {
            logUnsafeViewpointRejection(cfg_.print_log,
                                        "Unsafe viewpoint rejected: ESDF clearance " +
                                        std::to_string(dist) + " < " +
                                        std::to_string(cfg_.safe_distance) +
                                        ", p=" + pointToString(viewpoint));
            return false;
        }
    }
    return true;
}

bool TrackingFrontend::isGuideStartUsable(const Vec3f &point) const
{
    if (!point.allFinite())
    {
        return false;
    }
    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        return true;
    }
    if (!map_manager_->insideLocalMap(point))
    {
        logUnsafeViewpointRejection(cfg_.print_log,
                                    "Guide start rejected: outside local map, p=" + pointToString(point));
        return false;
    }
    const auto inf_grid_type = map_manager_->getInfGridType(point);
    if (inf_grid_type == super_utils::GridType::OCCUPIED ||
        inf_grid_type == super_utils::GridType::OUT_OF_MAP)
    {
        logUnsafeViewpointRejection(cfg_.print_log,
                                    "Guide start rejected: occupied/out-of-map, p=" + pointToString(point));
        return false;
    }
    if (cfg_.unknown_as_occupied &&
        (inf_grid_type == super_utils::GridType::UNKNOWN ||
         inf_grid_type == super_utils::GridType::UNDEFINED ||
         inf_grid_type == super_utils::GridType::FRONTIER))
    {
        logUnsafeViewpointRejection(cfg_.print_log,
                                    "Guide start rejected: unknown treated as occupied, p=" + pointToString(point));
        return false;
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
                                 false,
                                 cfg_.unknown_as_occupied);
    if (!line_free)
    {
        return false;
    }

    return true;
}

bool TrackingFrontend::repairViewpointEndpoint(const Vec3f &raw_viewpoint,
                                               const Vec3f &target,
                                               Vec3f &repaired_viewpoint) const
{
    if (!raw_viewpoint.allFinite())
    {
        return false;
    }

    (void)target;
    if (isViewpointSafe(raw_viewpoint))
    {
        repaired_viewpoint = raw_viewpoint;
        return true;
    }

    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        return false;
    }

    const double res = std::max(0.05, map_manager_->getResolution());
    const double repair_radius = std::max({1.0,
                                           2.0 * res,
                                           cfg_.safe_distance + cfg_.visibility_safe_distance + cfg_.distance_tolerance});

    Vec3f nearest = raw_viewpoint;
    if (map_manager_->hasESDF() &&
        map_manager_->findNearestESDFSafe(raw_viewpoint,
                                          cfg_.safe_distance,
                                          nearest,
                                          repair_radius) &&
        isViewpointSafe(nearest))
    {
        repaired_viewpoint = nearest;
        return true;
    }

    nearest = raw_viewpoint;
    if (map_manager_->getNearestInfCellNot(super_utils::GridType::OCCUPIED,
                                           raw_viewpoint,
                                           nearest,
                                           repair_radius) &&
        isViewpointSafe(nearest))
    {
        repaired_viewpoint = nearest;
        return true;
    }

    return false;
}

bool TrackingFrontend::collectVisibleViewpointCandidates(
    const Vec3f &seed,
    const traj_opt::DynamicTargetState &target,
    std::vector<ViewpointCandidate> &candidates) const
{
    candidates.clear();
    const Vec3f preferred = preferredViewpoint(seed, target);

    auto tryCandidate = [&](const Vec3f &raw_candidate, const double bias) {
        Vec3f candidate = raw_candidate;
        if (!repairViewpointEndpoint(raw_candidate, target.position, candidate))
        {
            return;
        }

        ViewpointCandidate scored;
        if (scoreViewpointCandidate(candidate, seed, preferred, target, bias, scored))
        {
            candidates.emplace_back(scored);
        }
    };

    tryCandidate(preferred, 0.0);
    tryCandidate(seed, 1.0);

    const int angle_count =
        std::max(8, static_cast<int>(std::ceil(2.0 * M_PI / std::max(0.05, cfg_.candidate_angle_step))));
    Vec3f preferred_rel_dir = preferred - target.position;
    preferred_rel_dir.z() = 0.0;
    preferred_rel_dir = normalizedOr(preferred_rel_dir, Vec3f::UnitX());
    const double base_yaw = std::atan2(preferred_rel_dir.y(), preferred_rel_dir.x());
    const std::array<double, 7> z_offsets{
        0.0,
        -0.5 * std::max(0.0, cfg_.height_tolerance),
        0.5 * std::max(0.0, cfg_.height_tolerance),
        -std::max(0.0, cfg_.height_tolerance),
        std::max(0.0, cfg_.height_tolerance),
        -1.5 * std::max(0.0, cfg_.height_tolerance),
        1.5 * std::max(0.0, cfg_.height_tolerance)};
    const double min_radius =
        std::max(0.3, cfg_.tracking_distance - std::max(0.0, cfg_.distance_tolerance));
    const double max_radius =
        std::max(min_radius,
                 cfg_.tracking_distance + std::max(2.0, 3.0 * std::max(0.0, cfg_.distance_tolerance)));
    const int radius_count = std::max(5, 2 * std::max(1, cfg_.candidate_radius_num) + 3);
    for (int r = 0; r < radius_count; ++r)
    {
        const double ratio = radius_count == 1 ? 0.0 : static_cast<double>(r) / static_cast<double>(radius_count - 1);
        const double radius = min_radius + ratio * (max_radius - min_radius);
        for (int i = 0; i < angle_count; ++i)
        {
            const int side = (i % 2 == 0) ? 1 : -1;
            const int step = (i + 1) / 2;
            const double yaw = base_yaw + static_cast<double>(side * step) *
                                             2.0 * M_PI / static_cast<double>(angle_count);
            Vec3f dir(std::cos(yaw), std::sin(yaw), 0.0);
            for (const double z_offset : z_offsets)
            {
                Vec3f candidate = target.position + radius * dir;
                candidate.z() = target.position.z() + cfg_.height_offset + z_offset;
                tryCandidate(candidate,
                             0.05 * std::abs(radius - cfg_.tracking_distance) +
                                 0.05 * std::abs(z_offset));
            }
        }
    }

    std::sort(candidates.begin(),
              candidates.end(),
              [](const ViewpointCandidate &lhs, const ViewpointCandidate &rhs) {
                  return lhs.score < rhs.score;
              });
    return !candidates.empty();
}

bool TrackingFrontend::chooseVisibleViewpoint(const Vec3f &seed,
                                              const traj_opt::DynamicTargetState &target,
                                              Vec3f &viewpoint) const
{
    std::vector<ViewpointCandidate> candidates;
    if (!collectVisibleViewpointCandidates(seed, target, candidates))
    {
        return false;
    }
    viewpoint = candidates.front().point;
    return true;
}

bool TrackingFrontend::chooseConnectedVisibleViewpoint(
    const Vec3f &seed,
    const traj_opt::DynamicTargetState &target,
    vec_E<Vec3f> &path,
    Vec3f &viewpoint) const
{
    std::vector<ViewpointCandidate> candidates;
    if (!collectVisibleViewpointCandidates(seed, target, candidates))
    {
        return false;
    }

    const std::size_t max_trials = std::min<std::size_t>(candidates.size(), 64);
    for (std::size_t i = 0; i < max_trials; ++i)
    {
        vec_E<Vec3f> trial_path = path;
        if (appendPathSegment(seed, candidates[i].point, trial_path, false))
        {
            path = std::move(trial_path);
            viewpoint = candidates[i].point;
            return true;
        }
    }
    return false;
}

bool TrackingFrontend::computeVisibleRegion(const traj_opt::DynamicTargetState &target,
                                            const Vec3f &seed,
                                            traj_opt::TrackingVisibleRegion &region) const
{
    if (!target.position.allFinite() || !seed.allFinite())
    {
        return false;
    }

    Vec3f seed_dir = seed - target.position;
    seed_dir.z() = 0.0;
    if (seed_dir.norm() < 1.0e-4)
    {
        return false;
    }

    const double desired_dist = std::max(0.3, cfg_.tracking_distance);
    const double res = map_manager_ != nullptr && map_manager_->ready()
                           ? std::max(0.05, map_manager_->getInfResolution())
                           : 0.1;
    const double d_theta = std::clamp(res / desired_dist / 2.0,
                                      0.01,
                                      std::max(0.01, cfg_.candidate_angle_step));
    const double theta0 = std::atan2(seed_dir.y(), seed_dir.x());
    const double desired_z = target.position.z() + cfg_.height_offset;
    const double tol = std::max(0.0, cfg_.distance_tolerance);
    const double min_radius = std::max(0.3, desired_dist - tol);
    const double max_radius = std::max(min_radius, desired_dist + tol);
    const double seed_radius = std::clamp(seed_dir.norm(), min_radius, max_radius);

    auto pointAtYaw = [&](const double yaw, const double radius) {
        Vec3f p = target.position;
        p.x() += radius * std::cos(yaw);
        p.y() += radius * std::sin(yaw);
        p.z() = desired_z;
        return p;
    };

    auto visiblePointAtYaw = [&](const double yaw, Vec3f &visible_point) {
        const std::array<double, 5> radii{
            desired_dist,
            seed_radius,
            0.5 * (min_radius + max_radius),
            min_radius,
            max_radius};
        for (const double radius : radii)
        {
            const Vec3f p = pointAtYaw(yaw, radius);
            if (isViewpointVisible(p, target.position))
            {
                visible_point = p;
                return true;
            }
        }
        return false;
    };

    Vec3f center_visible_point = Vec3f::Zero();
    if (!visiblePointAtYaw(theta0, center_visible_point))
    {
        logTrackingFrontendDebug(cfg_.print_log,
                                 "Visible region rejected: seed ray is not visible, target=" +
                                     pointToString(target.position) + ", seed=" + pointToString(seed));
        return false;
    }

    double theta_left = theta0;
    for (double yaw = theta0 - d_theta; yaw > theta0 - M_PI; yaw -= d_theta)
    {
        Vec3f p = Vec3f::Zero();
        if (!visiblePointAtYaw(yaw, p))
        {
            break;
        }
        theta_left = yaw;
    }

    double theta_right = theta0;
    for (double yaw = theta0 + d_theta; yaw < theta0 + M_PI; yaw += d_theta)
    {
        Vec3f p = Vec3f::Zero();
        if (!visiblePointAtYaw(yaw, p))
        {
            break;
        }
        theta_right = yaw;
    }

    const double half_angle = 0.5 * std::max(0.0, theta_right - theta_left);
    if (half_angle < 0.5 * d_theta)
    {
        logTrackingFrontendDebug(cfg_.print_log,
                                 "Visible region rejected: angular fan too narrow at target=" +
                                     pointToString(target.position));
        return false;
    }

    const double theta_mid = 0.5 * (theta_left + theta_right);
    Vec3f visible_mid = center_visible_point;
    visiblePointAtYaw(theta_mid, visible_mid);
    region.t = target.t;
    region.target_position = target.position;
    region.visible_point = visible_mid;
    region.theta = half_angle;
    region.confidence = std::clamp(half_angle / std::max(d_theta, cfg_.candidate_angle_step), 0.0, 1.0);
    region.valid = true;

    logTrackingFrontendDebug(cfg_.print_log,
                             "Visible region success: target=" + pointToString(target.position) +
                                 ", visible_point=" + pointToString(region.visible_point) +
                                 ", theta=" + std::to_string(region.theta));
    return true;
}

bool TrackingFrontend::findOcclusionAwareSeed(const Vec3f &last_viewpoint,
                                              const Vec3f &last_target,
                                              const Vec3f &target,
                                              Vec3f &seed) const
{
    auto visibleFrom = [&](const Vec3f &candidate) {
        if (map_manager_ != nullptr && map_manager_->ready() &&
            !map_manager_->isLineFree(candidate, target, false, cfg_.unknown_as_occupied))
        {
            return false;
        }
        return true;
    };

    const double res = map_manager_ != nullptr ? std::max(0.05, map_manager_->getResolution()) : 0.15;
    const Vec3f ray = last_target - last_viewpoint;
    const double ray_len = ray.norm();
    if (ray_len > 1.0e-4)
    {
        const int sample_num = std::max(1, static_cast<int>(std::ceil(ray_len / res)));
        for (int i = 0; i <= sample_num; ++i)
        {
            const double alpha = static_cast<double>(i) / static_cast<double>(sample_num);
            const Vec3f candidate = last_viewpoint + alpha * ray;
            if (visibleFrom(candidate))
            {
                seed = candidate;
                return true;
            }
        }
    }

    const Vec3f target_seg = target - last_target;
    const double target_seg_len = target_seg.norm();
    if (target_seg_len > 1.0e-4)
    {
        const int sample_num = std::max(1, static_cast<int>(std::ceil(target_seg_len / res)));
        for (int i = 0; i <= sample_num; ++i)
        {
            const double alpha = static_cast<double>(i) / static_cast<double>(sample_num);
            const Vec3f candidate = last_target + alpha * target_seg;
            if (visibleFrom(candidate))
            {
                seed = candidate;
                return true;
            }
        }
    }

    return false;
}

bool TrackingFrontend::extendToTrackingViewpoint(const Vec3f &seed,
                                                 const Vec3f &target,
                                                 const Vec3f &fallback,
                                                 Vec3f &viewpoint) const
{
    Vec3f dir = seed - target;
    dir.z() = 0.0;
    if (dir.norm() < 1.0e-4)
    {
        dir = fallback - target;
        dir.z() = 0.0;
    }
    dir = normalizedOr(dir, Vec3f::UnitX());

    const double desired = std::max(0.3, cfg_.tracking_distance);
    const double min_radius = std::max(0.3, desired - std::max(0.0, cfg_.distance_tolerance));
    const double max_radius = std::max(min_radius + 0.05,
                                       desired + std::max(0.0, cfg_.distance_tolerance));
    const double seed_radius = std::max(0.3, (seed - target).head<2>().norm());
    const double res = map_manager_ != nullptr && map_manager_->ready()
                           ? std::max(0.05, map_manager_->getInfResolution())
                           : 0.15;

    bool found = false;
    ViewpointCandidate best;
    traj_opt::DynamicTargetState target_state;
    target_state.position = target;
    target_state.velocity.setZero();
    const Vec3f preferred = target + desired * dir + Vec3f(0.0, 0.0, cfg_.height_offset);

    std::vector<double> radii;
    auto appendRadius = [&](const double r) {
        const double clamped = std::clamp(r, 0.3, max_radius);
        if (radii.empty() || std::abs(radii.back() - clamped) > 0.5 * res)
        {
            radii.emplace_back(clamped);
        }
    };

    const double start_radius = std::clamp(seed_radius, 0.3, max_radius);
    const double elastic_goal_radius = std::clamp(desired, 0.3, max_radius);
    const int primary_num =
        std::max(1, static_cast<int>(std::ceil(std::abs(elastic_goal_radius - start_radius) / res)));
    for (int i = 0; i <= primary_num; ++i)
    {
        const double alpha = static_cast<double>(i) / static_cast<double>(primary_num);
        appendRadius(start_radius + alpha * (elastic_goal_radius - start_radius));
    }
    if (max_radius > elastic_goal_radius + 0.5 * res)
    {
        const int extra_num =
            std::max(1, static_cast<int>(std::ceil((max_radius - elastic_goal_radius) / res)));
        for (int i = 1; i <= extra_num; ++i)
        {
            const double alpha = static_cast<double>(i) / static_cast<double>(extra_num);
            appendRadius(elastic_goal_radius + alpha * (max_radius - elastic_goal_radius));
        }
    }

    bool has_last_extension_point = false;
    Vec3f last_extension_point = Vec3f::Zero();
    for (const double radius : radii)
    {
        Vec3f candidate = target + radius * dir;
        candidate.z() = target.z() + cfg_.height_offset;
        if (has_last_extension_point &&
            map_manager_ != nullptr &&
            map_manager_->ready() &&
            !map_manager_->isLineFree(last_extension_point, candidate, true, cfg_.unknown_as_occupied))
        {
            break;
        }

        ViewpointCandidate scored;
        const double source_bias = 0.05 * std::abs(radius - std::clamp(seed_radius, min_radius, max_radius));
        if (!scoreViewpointCandidate(candidate, fallback, preferred, target_state, source_bias, scored))
        {
            if (found)
            {
                break;
            }
            continue;
        }
        has_last_extension_point = true;
        last_extension_point = candidate;
        if (!found || scored.score < best.score)
        {
            best = scored;
            found = true;
        }
    }

    if (!found)
    {
        return false;
    }
    viewpoint = best.point;
    return true;
}

bool TrackingFrontend::searchVisibleViewpointOnGrid(
    const Vec3f &start,
    const traj_opt::DynamicTargetState &target,
    Vec3f &viewpoint,
    vec_E<Vec3f> &path_to_viewpoint) const
{
    path_to_viewpoint.clear();
    if (!start.allFinite() || !target.position.allFinite())
    {
        return false;
    }

    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        if (!chooseVisibleViewpoint(start, target, viewpoint))
        {
            return false;
        }
        appendUnique(start, path_to_viewpoint);
        appendUnique(viewpoint, path_to_viewpoint);
        return true;
    }

    if (!isGuideStartUsable(start))
    {
        logTrackingFrontendFailure(cfg_.print_log,
                                   "Visible grid search start is not usable: " + pointToString(start));
        return false;
    }

    double res = map_manager_->getInfResolution();
    if (!std::isfinite(res) || res <= 1.0e-3)
    {
        res = map_manager_->getResolution();
    }
    res = std::max(0.05, res);

    const double tol = std::max(0.0, cfg_.distance_tolerance);
    const double min_h = std::max(0.3, cfg_.tracking_distance - tol);
    const double max_h = std::max(min_h + res, cfg_.tracking_distance + tol);
    const double desired_z = target.position.z() + cfg_.height_offset;
    const double z_tol = std::max(0.5 * res, std::max(0.0, cfg_.height_tolerance));
    const double start_target_dist = (start - target.position).norm();
    const bool reacquire_mode =
        start_target_dist >
        std::max(cfg_.reacquire_distance, cfg_.tracking_distance + cfg_.distance_tolerance);

    Vec3f preferred = preferredViewpoint(start, target);
    if (reacquire_mode)
    {
        Vec3f approach_dir = target.position - start;
        approach_dir.z() = 0.0;
        approach_dir = normalizedOr(approach_dir, Vec3f::UnitX());
        const double approach_step =
            std::min(std::max(0.8, 0.8 * cfg_.searching_horizon),
                     std::max(0.8, start_target_dist - cfg_.reacquire_distance));
        preferred = start + approach_step * approach_dir;
        preferred.z() = desired_z;
    }

    Vec3f box_min(std::min(start.x(), target.position.x() - max_h) - res,
                  std::min(start.y(), target.position.y() - max_h) - res,
                  std::min(start.z(), desired_z - z_tol) - res);
    Vec3f box_max(std::max(start.x(), target.position.x() + max_h) + res,
                  std::max(start.y(), target.position.y() + max_h) + res,
                  std::max(start.z(), desired_z + z_tol) + res);
    map_manager_->boundBoxByLocalMap(box_min, box_max);

    rog_map::Vec3i box_min_id, box_max_id;
    map_manager_->infMapPosToGlobalIndex(box_min, box_min_id);
    map_manager_->infMapPosToGlobalIndex(box_max, box_max_id);
    for (int dim = 0; dim < 3; ++dim)
    {
        if (box_min_id(dim) > box_max_id(dim))
        {
            std::swap(box_min_id(dim), box_max_id(dim));
        }
    }

    auto keyOf = [](const rog_map::Vec3i &id) {
        return std::to_string(id.x()) + "," +
               std::to_string(id.y()) + "," +
               std::to_string(id.z());
    };

    auto insideSearchBox = [&](const rog_map::Vec3i &id) {
        return id.x() >= box_min_id.x() && id.x() <= box_max_id.x() &&
               id.y() >= box_min_id.y() && id.y() <= box_max_id.y() &&
               id.z() >= box_min_id.z() && id.z() <= box_max_id.z();
    };

    auto isVisibleTrackingCandidate = [&](const Vec3f &candidate) {
        if (reacquire_mode)
        {
            const double candidate_target_dist = (candidate - target.position).norm();
            const double min_progress =
                std::min(1.0, std::max(2.0 * res, 0.05 * start_target_dist));
            return isViewpointSafe(candidate) &&
                   (start_target_dist - candidate_target_dist >= min_progress ||
                    candidate_target_dist <= cfg_.reacquire_distance) &&
                   std::abs(candidate.z() - desired_z) <= std::max(1.5, 2.0 * z_tol);
        }

        const Vec3f rel = candidate - target.position;
        const double h = rel.head<2>().norm();
        return h >= min_h &&
               h <= max_h &&
               std::abs(candidate.z() - desired_z) <= z_tol &&
               isViewpointSafe(candidate);
    };

    auto scoreVisibleTrackingCandidate = [&](const Vec3f &candidate, ViewpointCandidate &scored) {
        if (!isVisibleTrackingCandidate(candidate))
        {
            return false;
        }
        if (reacquire_mode)
        {
            const double oe_margin = estimateVisibilityMargin(candidate, target.position);
            const double oe_violation = std::max(0.0, -oe_margin);
            const double path_cost = (candidate - start).norm();
            const double preferred_cost = (candidate - preferred).norm();
            const double target_dist = (candidate - target.position).norm();
            const double clearance_reward =
                std::clamp(oe_margin, 0.0, std::max(0.3, cfg_.tracking_distance));
            scored.point = candidate;
            scored.od_cost = target_dist;
            scored.oe_margin = oe_margin;
            scored.path_cost = path_cost;
            scored.preferred_cost = preferred_cost;
            scored.source_bias = 0.0;
            scored.score =
                std::max(0.0, cfg_.score_path_weight) * path_cost +
                std::max(0.0, cfg_.score_preferred_weight) * preferred_cost +
                0.25 * target_dist +
                std::max(0.0, cfg_.score_oe_weight) * oe_violation * oe_violation -
                std::max(0.0, cfg_.score_clearance_reward) * clearance_reward;
            return std::isfinite(scored.score);
        }
        return scoreViewpointCandidate(candidate, start, preferred, target, 0.0, scored);
    };

    struct QueueNode
    {
        rog_map::Vec3i id{rog_map::Vec3i::Zero()};
        double g{0.0};
        double score{0.0};
    };

    struct QueueNodeCompare
    {
        bool operator()(const QueueNode &lhs, const QueueNode &rhs) const
        {
            return lhs.score > rhs.score;
        }
    };

    struct SearchRecord
    {
        rog_map::Vec3i parent{rog_map::Vec3i::Zero()};
        Vec3f position{Vec3f::Zero()};
        double g{0.0};
        bool has_parent{false};
        bool closed{false};
    };

    auto heuristic = [&](const Vec3f &p) {
        const Vec3f rel = p - target.position;
        const double h = rel.head<2>().norm();
        const double h_err = std::max(0.0, std::max(min_h - h, h - max_h));
        const double z_err = std::max(0.0, std::abs(p.z() - desired_z) - z_tol);
        return 0.35 * (p - preferred).norm() + 2.0 * h_err + 2.0 * z_err;
    };

    rog_map::Vec3i start_id;
    map_manager_->infMapPosToGlobalIndex(start, start_id);
    if (!insideSearchBox(start_id) || !map_manager_->insideLocalMap(start_id))
    {
        logTrackingFrontendDebug(cfg_.print_log,
                                 "Visible grid search start is outside the bounded search box: start=" +
                                     pointToString(start) + ", target=" +
                                     pointToString(target.position) + ", box_min=" +
                                     pointToString(box_min) + ", box_max=" +
                                     pointToString(box_max));
        return false;
    }

    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
    std::unordered_map<std::string, SearchRecord> records;
    const std::string start_key = keyOf(start_id);
    records[start_key] = SearchRecord{rog_map::Vec3i::Zero(), start, 0.0, false, false};
    open_set.push(QueueNode{start_id, 0.0, heuristic(start)});

    const std::array<rog_map::Vec3i, 6> neighbors{
        rog_map::Vec3i(1, 0, 0),
        rog_map::Vec3i(-1, 0, 0),
        rog_map::Vec3i(0, 1, 0),
        rog_map::Vec3i(0, -1, 0),
        rog_map::Vec3i(0, 0, 1),
        rog_map::Vec3i(0, 0, -1)};
    const double horizon = std::max({cfg_.searching_horizon,
                                     (start - target.position).norm() + max_h,
                                     2.0 * max_h});
    logTrackingFrontendDebug(cfg_.print_log,
                             "Visible grid search setup: start=" + pointToString(start) +
                                 ", target=" + pointToString(target.position) +
                                 ", start_target_dist=" + std::to_string(start_target_dist) +
                                 ", local_horizon=" + std::to_string(horizon) +
                                 ", tracking_band=[" + std::to_string(min_h) + ", " +
                                 std::to_string(max_h) + "], desired_z=" +
                                 std::to_string(desired_z));
    const int max_expand = 25000;
    int expanded = 0;

    auto reconstructPath = [&](const rog_map::Vec3i &goal_id) {
        std::vector<Vec3f> reversed_path;
        rog_map::Vec3i cur_id = goal_id;
        while (true)
        {
            const auto cur_it = records.find(keyOf(cur_id));
            if (cur_it == records.end())
            {
                break;
            }
            reversed_path.push_back(cur_it->second.position);
            if (!cur_it->second.has_parent)
            {
                break;
            }
            cur_id = cur_it->second.parent;
        }
        path_to_viewpoint.clear();
        for (auto it = reversed_path.rbegin(); it != reversed_path.rend(); ++it)
        {
            appendUnique(*it, path_to_viewpoint);
        }
    };

    while (!open_set.empty() && expanded < max_expand)
    {
        const QueueNode node = open_set.top();
        open_set.pop();
        const std::string node_key = keyOf(node.id);
        auto rec_it = records.find(node_key);
        if (rec_it == records.end() || rec_it->second.closed)
        {
            continue;
        }
        SearchRecord &record = rec_it->second;
        record.closed = true;
        ++expanded;

        ViewpointCandidate scored;
        if (scoreVisibleTrackingCandidate(record.position, scored))
        {
            viewpoint = record.position;
            reconstructPath(node.id);
            logTrackingFrontendDebug(cfg_.print_log,
                                     "Visible grid search success: viewpoint=" +
                                     pointToString(viewpoint) + ", expanded=" +
                                     std::to_string(expanded) + ", score=" +
                                     std::to_string(scored.score) + ", oe_margin=" +
                                     std::to_string(scored.oe_margin) + ", path_points=" +
                                     std::to_string(path_to_viewpoint.size()));
            return true;
        }

        for (const auto &delta : neighbors)
        {
            const rog_map::Vec3i next_id = node.id + delta;
            if (!insideSearchBox(next_id) || !map_manager_->insideLocalMap(next_id))
            {
                continue;
            }

            Vec3f next_pos = Vec3f::Zero();
            map_manager_->infMapGlobalIndexToPos(next_id, next_pos);
            if (!isViewpointSafe(next_pos) ||
                !map_manager_->isLineFree(record.position, next_pos, true, cfg_.unknown_as_occupied))
            {
                continue;
            }

            const double step = (next_pos - record.position).norm();
            const double next_g = record.g + step;
            if (next_g > horizon)
            {
                continue;
            }

            const std::string next_key = keyOf(next_id);
            auto next_it = records.find(next_key);
            if (next_it != records.end() && next_g >= next_it->second.g)
            {
                continue;
            }

            records[next_key] = SearchRecord{node.id, next_pos, next_g, true, false};
            open_set.push(QueueNode{next_id, next_g, next_g + heuristic(next_pos)});
        }
    }

    logTrackingFrontendDebug(cfg_.print_log,
                             "Visible grid search failed: expanded=" +
                             std::to_string(expanded) + ", open_remaining=" +
                             std::to_string(open_set.size()));
    return false;
}

bool TrackingFrontend::chooseApproachViewpoint(const Vec3f &start,
                                               const traj_opt::DynamicTargetState &target,
                                               Vec3f &viewpoint,
                                               vec_E<Vec3f> &path_to_viewpoint) const
{
    path_to_viewpoint.clear();
    if (!start.allFinite() || !target.position.allFinite())
    {
        return false;
    }

    const double start_target_dist = (start - target.position).norm();
    if (start_target_dist <= std::max(cfg_.reacquire_distance,
                                      cfg_.tracking_distance + cfg_.distance_tolerance))
    {
        return false;
    }

    Vec3f dir = target.position - start;
    dir.z() = 0.0;
    dir = normalizedOr(dir, Vec3f::UnitX());
    const double desired_z = target.position.z() + cfg_.height_offset;
    const double max_step = std::min(std::max(0.8, 0.8 * cfg_.searching_horizon),
                                     std::max(0.8, start_target_dist - cfg_.reacquire_distance));

    const std::array<double, 6> step_scales{1.0, 0.75, 0.5, 0.35, 0.2, 0.1};
    const std::array<double, 5> z_offsets{
        0.0,
        -0.5 * std::max(0.0, cfg_.height_tolerance),
        0.5 * std::max(0.0, cfg_.height_tolerance),
        -std::max(0.0, cfg_.height_tolerance),
        std::max(0.0, cfg_.height_tolerance)};

    for (const double step_scale : step_scales)
    {
        for (const double z_offset : z_offsets)
        {
            Vec3f raw = start + std::max(0.5, max_step * step_scale) * dir;
            raw.z() = desired_z + z_offset;
            Vec3f candidate = raw;
            if (!repairViewpointEndpoint(raw, target.position, candidate))
            {
                continue;
            }
            if ((candidate - target.position).norm() >= start_target_dist)
            {
                continue;
            }
            vec_E<Vec3f> trial_path;
            if (appendPathSegment(start, candidate, trial_path, true))
            {
                viewpoint = candidate;
                path_to_viewpoint = std::move(trial_path);
                logTrackingFrontendDebug(cfg_.print_log,
                                         "Approach viewpoint success: start=" +
                                             pointToString(start) + ", target=" +
                                             pointToString(target.position) + ", viewpoint=" +
                                             pointToString(viewpoint) + ", dist_progress=" +
                                             std::to_string(start_target_dist -
                                                            (viewpoint - target.position).norm()));
                return true;
            }
        }
    }

    logTrackingFrontendDebug(cfg_.print_log,
                             "Approach viewpoint failed: start=" + pointToString(start) +
                                 ", target=" + pointToString(target.position) +
                                 ", start_target_dist=" + std::to_string(start_target_dist));
    return false;
}

bool TrackingFrontend::choosePropagatedViewpoint(const Vec3f &last_viewpoint,
                                                 const traj_opt::DynamicTargetState &last_target,
                                                 const traj_opt::DynamicTargetState &target,
                                                 Vec3f &viewpoint,
                                                 vec_E<Vec3f> &path_to_viewpoint) const
{
    path_to_viewpoint.clear();
    Vec3f seed = Vec3f::Zero();
    if (findOcclusionAwareSeed(last_viewpoint, last_target.position, target.position, seed))
    {
        if (extendToTrackingViewpoint(seed, target.position, last_viewpoint, viewpoint))
        {
            if (appendPathSegment(last_viewpoint, viewpoint, path_to_viewpoint, true))
            {
                return true;
            }
            logTrackingFrontendDebug(cfg_.print_log,
                                     "Propagated viewpoint found but cannot connect: last_viewpoint=" +
                                         pointToString(last_viewpoint) + ", viewpoint=" +
                                         pointToString(viewpoint) + ", target=" +
                                         pointToString(target.position));
        }
        else
        {
            logTrackingFrontendDebug(cfg_.print_log,
                                     "Propagated viewpoint extension failed: seed=" +
                                         pointToString(seed) + ", target=" +
                                         pointToString(target.position));
        }
    }
    else
    {
        logTrackingFrontendDebug(cfg_.print_log,
                                 "Occlusion-aware seed search failed: last_viewpoint=" +
                                     pointToString(last_viewpoint) + ", last_target=" +
                                     pointToString(last_target.position) + ", target=" +
                                     pointToString(target.position));
    }

    if (searchVisibleViewpointOnGrid(last_viewpoint, target, viewpoint, path_to_viewpoint))
    {
        return true;
    }
    logTrackingFrontendDebug(cfg_.print_log,
                             "Visible grid fallback failed: start=" + pointToString(last_viewpoint) +
                                 ", target=" + pointToString(target.position) +
                                 ", start_target_dist=" +
                                 std::to_string((last_viewpoint - target.position).norm()));

    if (chooseApproachViewpoint(last_viewpoint, target, viewpoint, path_to_viewpoint))
    {
        return true;
    }

    if (!chooseVisibleViewpoint(last_viewpoint, target, viewpoint))
    {
        logTrackingFrontendDebug(cfg_.print_log,
                                 "Ring candidate fallback failed: start=" + pointToString(last_viewpoint) +
                                     ", target=" + pointToString(target.position));
        return false;
    }
    path_to_viewpoint.clear();
    const bool connected = appendPathSegment(last_viewpoint, viewpoint, path_to_viewpoint, true);
    if (!connected)
    {
        logTrackingFrontendDebug(cfg_.print_log,
                                 "Ring candidate fallback found viewpoint but cannot connect: start=" +
                                     pointToString(last_viewpoint) + ", viewpoint=" +
                                     pointToString(viewpoint) + ", target=" +
                                     pointToString(target.position));
    }
    return connected;
}

bool TrackingFrontend::appendPathSegment(const Vec3f &start,
                                         const Vec3f &goal,
                                         vec_E<Vec3f> &path,
                                         bool verbose) const
{
    if (!start.allFinite() || !goal.allFinite())
    {
        return false;
    }

    if (map_manager_ == nullptr || !map_manager_->ready())
    {
        return appendLineSegmentSamples(start, goal, path);
    }

    if (!isGuideStartUsable(start))
    {
        if (verbose)
        {
            logTrackingFrontendFailure(cfg_.print_log,
                                       "A* segment start is not usable: " + pointToString(start));
        }
        return false;
    }
    if (!isViewpointSafe(goal))
    {
        if (verbose)
        {
            logTrackingFrontendFailure(cfg_.print_log,
                                       "A* segment goal is not safe: " + pointToString(goal));
        }
        return false;
    }

    const Vec3f search_start = start;
    const double segment_dist = (search_start - goal).norm();

    if (map_manager_->isLineFree(search_start, goal, true, cfg_.unknown_as_occupied))
    {
        if (verbose)
        {
            logTrackingFrontendDebug(cfg_.print_log,
                                     "Line-free success for tracking guide segment: start=" +
                                     pointToString(search_start) + ", goal=" + pointToString(goal));
        }
        return appendLineSegmentSamples(search_start, goal, path);
    }

    if (!cfg_.use_astar || astar_ == nullptr)
    {
        if (verbose)
        {
            logTrackingFrontendDebug(cfg_.print_log,
                                     "A* failure: segment blocked and tracking frontend A* is disabled.");
        }
        return false;
    }

    auto searchSegment = [&](const int flag, vec_E<Vec3f> &astar_path) {
        astar_path.clear();
        const auto ret =
            astar_->pointToPointPathSearch(search_start,
                                           goal,
                                           flag,
                                           std::max(cfg_.searching_horizon,
                                                    segment_dist + 2.0 * cfg_.tracking_distance),
                                           astar_path,
                                           0.08);
        return (ret == super_utils::SUCCESS || ret == super_utils::REACH_GOAL) && !astar_path.empty();
    };

    vec_E<Vec3f> astar_path;
    const int inf_flag = path_search::ON_INF_MAP |
                         (cfg_.unknown_as_occupied ? path_search::UNKNOWN_AS_OCCUPIED
                                                   : path_search::UNKNOWN_AS_FREE) |
                         path_search::USE_INF_NEIGHBOR;
    const bool found = searchSegment(inf_flag, astar_path);
    if (!found)
    {
        if (verbose)
        {
            logTrackingFrontendDebug(cfg_.print_log,
                                     "A* failure: inflated-map search cannot connect tracking guide segment, start=" +
                                         pointToString(search_start) + ", goal=" + pointToString(goal) +
                                         ", segment_dist=" + std::to_string(segment_dist) +
                                         ", search_horizon=" +
                                         std::to_string(std::max(cfg_.searching_horizon,
                                                                 segment_dist + 2.0 * cfg_.tracking_distance)));
        }
        return false;
    }

    for (const auto &p : astar_path)
    {
        if (!isViewpointSafe(p))
        {
            if (verbose)
            {
                logTrackingFrontendDebug(cfg_.print_log,
                                         "A* failure: returned unsafe tracking guide point, p=" + pointToString(p));
            }
            return false;
        }
    }

    Vec3f last = search_start;
    for (const auto &p : astar_path)
    {
        if (!map_manager_->isLineFree(last, p, true, cfg_.unknown_as_occupied))
        {
            if (verbose)
            {
                logTrackingFrontendDebug(cfg_.print_log,
                                         "A* failure: returned guide has blocked sub-segment, start=" +
                                         pointToString(last) + ", goal=" + pointToString(p));
            }
            return false;
        }
        last = p;
    }
    if (!map_manager_->isLineFree(last, goal, true, cfg_.unknown_as_occupied))
    {
        if (verbose)
        {
            logTrackingFrontendDebug(cfg_.print_log,
                                     "A* failure: returned guide has blocked final sub-segment, start=" +
                                     pointToString(last) + ", goal=" + pointToString(goal));
        }
        return false;
    }

    if (verbose)
    {
        logTrackingFrontendDebug(cfg_.print_log,
                                 "A* success for tracking guide segment: start=" +
                                 pointToString(search_start) + ", goal=" +
                                 pointToString(goal) + ", astar_points=" +
                                 std::to_string(astar_path.size()));
    }
    appendUnique(search_start, path);
    Vec3f last_appended = search_start;
    for (const auto &p : astar_path)
    {
        if (!appendLineSegmentSamples(last_appended, p, path))
        {
            return false;
        }
        last_appended = p;
    }
    return appendLineSegmentSamples(last_appended, goal, path);
}

bool TrackingFrontend::appendLineSegmentSamples(const Vec3f &start,
                                                const Vec3f &goal,
                                                vec_E<Vec3f> &path) const
{
    if (!start.allFinite() || !goal.allFinite())
    {
        return false;
    }

    double max_step = 0.25;
    if (map_manager_ != nullptr && map_manager_->ready())
    {
        max_step = std::max(0.05, 1.5 * map_manager_->getInfResolution());
    }
    const double len = (goal - start).norm();
    if (len < 1.0e-4)
    {
        appendUnique(goal, path);
        return true;
    }
    const int segment_num = std::max(1, static_cast<int>(std::ceil(len / max_step)));

    appendUnique(start, path);
    Vec3f last = start;
    for (int i = 1; i <= segment_num; ++i)
    {
        const double alpha = static_cast<double>(i) / static_cast<double>(segment_num);
        Vec3f point = start + alpha * (goal - start);
        if (i == segment_num)
        {
            point = goal;
        }
        if (map_manager_ != nullptr && map_manager_->ready())
        {
            if (!isViewpointSafe(point) ||
                !map_manager_->isLineFree(last, point, true, cfg_.unknown_as_occupied))
            {
                return false;
            }
        }
        appendUnique(point, path);
        last = point;
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
    problem.safe_distance = cfg_.safe_distance;
    problem.tracking_distance = cfg_.tracking_distance;
    problem.distance_tolerance = cfg_.distance_tolerance;
    problem.height_offset = cfg_.height_offset;
    problem.height_tolerance = cfg_.height_tolerance;
    problem.visibility_safe_distance = cfg_.visibility_safe_distance;
    problem.visibility_cone_ratio = cfg_.visibility_cone_ratio;
    problem.visibility_angle_clearance = cfg_.visibility_angle_clearance;
    problem.visibility_samples = cfg_.visibility_samples;
    problem.use_visible_region = cfg_.use_visible_region;
    const double initial_horizontal_dist =
        (head_pvaj.col(0) - target_prediction.front().position).head<2>().norm();
    problem.reacquire_mode =
        initial_horizontal_dist >
        std::max(cfg_.tracking_distance + cfg_.distance_tolerance,
                 cfg_.reacquire_distance);
    problem.od_h_lower = std::max(0.05, cfg_.tracking_distance - cfg_.distance_tolerance);
    problem.od_h_upper = std::max(problem.od_h_lower + 0.05, cfg_.tracking_distance + cfg_.distance_tolerance);
    problem.od_v_lower = cfg_.height_offset - cfg_.height_tolerance;
    problem.od_v_upper = cfg_.height_offset + cfg_.height_tolerance;

    logTrackingFrontendDebug(cfg_.print_log,
                             "BuildProblem start: head=" + pointToString(head_pvaj.col(0)) +
                                 ", first_target=" + pointToString(target_prediction.front().position) +
                                 ", first_target_dist=" +
                                 std::to_string((head_pvaj.col(0) - target_prediction.front().position).norm()) +
                                 ", samples=" + std::to_string(target_prediction.size()) +
                                 ", search_horizon=" + std::to_string(cfg_.searching_horizon) +
                                 ", tracking_band=[" + std::to_string(problem.od_h_lower) +
                                 ", " + std::to_string(problem.od_h_upper) +
                                 "], use_visible_region=" +
                                 (cfg_.use_visible_region ? "true" : "false") +
                                 ", reacquire_mode=" +
                                 (problem.reacquire_mode ? "true" : "false"));

    std::ofstream frontend_log;
    if (cfg_.save_log)
    {
        frontend_log.open(trackingFrontendLogPath(), std::ios::out | std::ios::trunc);
        if (frontend_log.good())
        {
            writeTrackingFrontendLogHeader(frontend_log);
        }
    }

    auto makeLogQuality = [&](const Vec3f &viewpoint,
                              const traj_opt::DynamicTargetState &target) {
        TrackingFrontendLogQuality quality;
        const Vec3f rel = viewpoint - target.position;
        const double horizontal = rel.head<2>().norm();
        const double vertical = rel.z();
        const double min_h = std::max(0.05, cfg_.tracking_distance - std::max(0.0, cfg_.distance_tolerance));
        const double max_h = std::max(min_h + 0.05, cfg_.tracking_distance + std::max(0.0, cfg_.distance_tolerance));
        const double min_z = cfg_.height_offset - std::max(0.0, cfg_.height_tolerance);
        const double max_z = cfg_.height_offset + std::max(0.0, cfg_.height_tolerance);
        quality.in_od_band = horizontal >= min_h && horizontal <= max_h;
        quality.in_height_band = vertical >= min_z && vertical <= max_z;
        quality.visible = isViewpointVisible(viewpoint, target.position);
        quality.visibility_margin = estimateVisibilityMargin(viewpoint, target.position);
        quality.tracking_success = quality.in_od_band &&
                                   quality.in_height_band &&
                                   quality.visible &&
                                   quality.visibility_margin >= -1.0e-3;
        if (quality.tracking_success)
        {
            quality.tracking_state = "tracking";
        }
        else if (horizontal > max_h)
        {
            quality.tracking_state = "acquiring";
        }
        else
        {
            quality.tracking_state = "lost";
        }
        return quality;
    };

    vec_E<Vec3f> guide;
    std::vector<double> guide_t;
    appendTimedUnique(head_pvaj.col(0), target_prediction.front().t, guide, guide_t);
    problem.viewpoints.clear();
    problem.viewpoints.reserve(target_prediction.size());
    problem.visible_regions.clear();
    problem.visible_regions.reserve(target_prediction.size());
    problem.target_sample_times.clear();
    problem.target_sample_times.reserve(target_prediction.size());
    traj_opt::DynamicTargetStates used_prediction;
    used_prediction.reserve(target_prediction.size());

    auto finalizeProblem = [&]() -> bool {
        if (used_prediction.empty() || guide.empty() || guide.size() != guide_t.size())
        {
            logTrackingFrontendFailure(cfg_.print_log,
                                       "No usable tracking guide can be finalized.");
            return false;
        }

        problem.guide_path = guide;
        problem.guide_t = guide_t;
        problem.tail_pvaj.setZero();
        problem.tail_pvaj.col(0) = guide.back();
        problem.target_prediction = used_prediction;
        Vec3f tail_vel = used_prediction.back().velocity;
        if (!tail_vel.allFinite() || tail_vel.norm() < 0.05)
        {
            tail_vel.setZero();
        }
        problem.tail_pvaj.col(1) = tail_vel;
        problem.min_total_duration = std::max(0.6, used_prediction.back().t);
        int tracking_ready_count = 0;
        int visible_count = 0;
        for (std::size_t k = 0; k < problem.viewpoints.size() && k < used_prediction.size(); ++k)
        {
            const auto quality = makeLogQuality(problem.viewpoints[k], used_prediction[k]);
            if (quality.tracking_success)
            {
                ++tracking_ready_count;
            }
            if (quality.visible)
            {
                ++visible_count;
            }
        }
        const double denom = std::max(1.0, static_cast<double>(std::min(problem.viewpoints.size(),
                                                                         used_prediction.size())));
        const auto terminal_quality = makeLogQuality(problem.viewpoints.back(), used_prediction.back());
        problem.frontend_initial_horizontal_distance =
            (problem.viewpoints.front() - used_prediction.front().position).head<2>().norm();
        problem.frontend_terminal_horizontal_distance =
            (problem.viewpoints.back() - used_prediction.back().position).head<2>().norm();
        problem.frontend_terminal_vertical_offset =
            (problem.viewpoints.back() - used_prediction.back().position).z();
        problem.frontend_tracking_ready = terminal_quality.tracking_success;
        problem.frontend_acquiring = !terminal_quality.tracking_success &&
                                     problem.frontend_terminal_horizontal_distance <
                                         problem.frontend_initial_horizontal_distance;
        problem.frontend_in_band_ratio = static_cast<double>(tracking_ready_count) / denom;
        problem.frontend_visible_ratio = static_cast<double>(visible_count) / denom;
        return true;
    };

    auto canUsePartialGuide = [&]() -> bool {
        return used_prediction.size() >= 2 && !guide.empty() && guide.size() == guide_t.size();
    };

    Vec3f seed = head_pvaj.col(0);
    problem.viewpoints.emplace_back(seed);
    problem.target_sample_times.emplace_back(target_prediction.front().t);
    used_prediction.emplace_back(target_prediction.front());
    writeTrackingFrontendLogRow(frontend_log,
                                0,
                                target_prediction.front().t,
                                "head",
                                true,
                                makeLogQuality(seed, target_prediction.front()),
                                target_prediction.front().position,
                                seed,
                                seed,
                                0.0,
                                0.0,
                                estimateVisibilityMargin(seed, target_prediction.front().position),
                                0.0,
                                0.0,
                                0.0,
                                1,
                                "initial_state");
    if (cfg_.use_visible_region)
    {
        traj_opt::TrackingVisibleRegion region;
        region.t = target_prediction.front().t;
        region.target_position = target_prediction.front().position;
        region.visible_point = seed;
        region.theta = 0.0;
        region.confidence = 0.0;
        region.valid = false;
        problem.visible_regions.emplace_back(region);
    }

    auto appendEmergencyPartialGuide = [&](const traj_opt::DynamicTargetState &target,
                                           const std::string &reason) -> bool {
        Vec3f emergency = preferredViewpoint(seed, target);
        bool found = repairViewpointEndpoint(emergency, target.position, emergency);
        Vec3f fallback_dir = head_pvaj.col(1);
        fallback_dir.z() = 0.0;
        if (fallback_dir.norm() < 1.0e-3)
        {
            fallback_dir = seed - target.position;
            fallback_dir.z() = 0.0;
        }
        fallback_dir = normalizedOr(fallback_dir, Vec3f::UnitX());

        if (!found || (emergency - seed).norm() < 0.05)
        {
            const std::array<double, 8> yaw_offsets{
                0.0, M_PI_2, -M_PI_2, M_PI, M_PI / 4.0, -M_PI / 4.0, 3.0 * M_PI / 4.0, -3.0 * M_PI / 4.0};
            const std::array<double, 4> step_sizes{0.25, 0.4, 0.6, 0.8};
            const double base_yaw = std::atan2(fallback_dir.y(), fallback_dir.x());
            found = false;
            for (const double step : step_sizes)
            {
                for (const double yaw_offset : yaw_offsets)
                {
                    Vec3f dir(std::cos(base_yaw + yaw_offset),
                              std::sin(base_yaw + yaw_offset),
                              0.0);
                    Vec3f candidate = seed + step * dir;
                    candidate.z() = seed.z();
                    if (isViewpointSafe(candidate))
                    {
                        emergency = candidate;
                        found = true;
                        break;
                    }
                }
                if (found)
                {
                    break;
                }
            }
        }

        if (!found || (emergency - seed).norm() < 1.0e-4)
        {
            return false;
        }

        appendTimedUnique(emergency, target.t, guide, guide_t);
        problem.viewpoints.emplace_back(emergency);
        problem.target_sample_times.emplace_back(target.t);
        used_prediction.emplace_back(target);
        writeTrackingFrontendLogRow(frontend_log,
                                    problem.viewpoints.size() - 1,
                                    target.t,
                                    "emergency_partial",
                                    true,
                                    makeLogQuality(emergency, target),
                                    target.position,
                                    seed,
                                    emergency,
                                    0.0,
                                    0.0,
                                    estimateVisibilityMargin(emergency, target.position),
                                    0.0,
                                    0.0,
                                    0.0,
                                    1,
                                    reason);
        if (cfg_.use_visible_region)
        {
            traj_opt::TrackingVisibleRegion region;
            region.t = target.t;
            region.target_position = target.position;
            region.visible_point = emergency;
            region.theta = 0.0;
            region.confidence = 0.0;
            region.valid = false;
            problem.visible_regions.emplace_back(region);
        }
        seed = emergency;
        return true;
    };

    for (std::size_t i = 1; i < target_prediction.size(); ++i)
    {
        const auto &last_target = used_prediction.back();
        const auto &target = target_prediction[i];
        logTrackingFrontendDebug(cfg_.print_log,
                                 "BuildProblem sample " + std::to_string(i) +
                                     ": seed=" + pointToString(seed) +
                                     ", target=" + pointToString(target.position) +
                                     ", seed_target_dist=" +
                                     std::to_string((seed - target.position).norm()) +
                                     ", t=" + std::to_string(target.t));
        Vec3f viewpoint = Vec3f::Zero();
        vec_E<Vec3f> path_to_viewpoint;
        if (!choosePropagatedViewpoint(seed, last_target, target, viewpoint, path_to_viewpoint))
        {
            writeTrackingFrontendLogRow(frontend_log,
                                        i,
                                        target.t,
                                        "choose_viewpoint",
                                        false,
                                        makeLogQuality(seed, target),
                                        target.position,
                                        seed,
                                        seed,
                                        0.0,
                                        0.0,
                                        estimateVisibilityMargin(seed, target.position),
                                        0.0,
                                        0.0,
                                        0.0,
                                        0,
                                        "no_safe_visible_viewpoint");
            logTrackingFrontendFailure(cfg_.print_log,
                                       "No safe visible tracking viewpoint found at target sample " +
                                       std::to_string(i) + ", t=" + std::to_string(target.t) + ".");
            if (canUsePartialGuide())
            {
                logTrackingFrontendDebug(cfg_.print_log,
                                         "Truncate tracking frontend at last valid sample instead of failing.");
                return finalizeProblem();
            }
            if (appendEmergencyPartialGuide(target, "emergency_after_no_safe_visible_viewpoint"))
            {
                return finalizeProblem();
            }
            return false;
        }
        if (path_to_viewpoint.empty())
        {
            writeTrackingFrontendLogRow(frontend_log,
                                        i,
                                        target.t,
                                        "connect_viewpoint",
                                        false,
                                        makeLogQuality(viewpoint, target),
                                        target.position,
                                        seed,
                                        viewpoint,
                                        0.0,
                                        0.0,
                                        estimateVisibilityMargin(viewpoint, target.position),
                                        0.0,
                                        0.0,
                                        0.0,
                                        0,
                                        "empty_path_to_viewpoint");
            logTrackingFrontendFailure(cfg_.print_log,
                                       "Failed to connect required tracking viewpoint at target sample " +
                                       std::to_string(i) + ", t=" + std::to_string(target.t) +
                                       ", seed=" + pointToString(seed) +
                                       ", viewpoint=" + pointToString(viewpoint) + ".");
            if (canUsePartialGuide())
            {
                logTrackingFrontendDebug(cfg_.print_log,
                                         "Truncate tracking frontend at last connected sample instead of failing.");
                return finalizeProblem();
            }
            if (appendEmergencyPartialGuide(target, "emergency_after_empty_path_to_viewpoint"))
            {
                return finalizeProblem();
            }
            return false;
        }
        appendTimedPath(path_to_viewpoint,
                        guide_t.empty() ? target_prediction.front().t : guide_t.back(),
                        target.t,
                        guide,
                        guide_t);
        ViewpointCandidate metrics;
        const Vec3f preferred = preferredViewpoint(seed, target);
        const bool scored = scoreViewpointCandidate(viewpoint,
                                                    seed,
                                                    preferred,
                                                    target,
                                                    0.0,
                                                    metrics);
        writeTrackingFrontendLogRow(frontend_log,
                                    i,
                                    target.t,
                                    "viewpoint",
                                    true,
                                    makeLogQuality(viewpoint, target),
                                    target.position,
                                    seed,
                                    viewpoint,
                                    scored ? metrics.score : 0.0,
                                    scored ? metrics.od_cost : 0.0,
                                    scored ? metrics.oe_margin
                                           : estimateVisibilityMargin(viewpoint, target.position),
                                    scored ? metrics.path_cost : 0.0,
                                    scored ? metrics.preferred_cost : 0.0,
                                    scored ? metrics.source_bias : 0.0,
                                    path_to_viewpoint.size(),
                                    "ok");
        if (cfg_.use_visible_region)
        {
            traj_opt::TrackingVisibleRegion region;
            if (!computeVisibleRegion(target, viewpoint, region))
            {
                region.t = target.t;
                region.target_position = target.position;
                region.visible_point = viewpoint;
                region.theta = 0.0;
                region.confidence = 0.0;
                region.valid = false;
                logTrackingFrontendDebug(cfg_.print_log,
                                         "Visible region unavailable at target sample " +
                                             std::to_string(i) + ", t=" + std::to_string(target.t) +
                                             "; skip fan soft prior and keep OD/OA/OE/FoV costs.");
            }
            problem.visible_regions.emplace_back(region);
        }
        seed = viewpoint;
        problem.viewpoints.emplace_back(viewpoint);
        problem.target_sample_times.emplace_back(target.t);
        used_prediction.emplace_back(target);
    }

    return finalizeProblem();
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
