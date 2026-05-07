#include "general_core/tracking_perching_frontend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
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

    if (isViewpointVisible(raw_viewpoint, target))
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
        isViewpointVisible(nearest, target))
    {
        repaired_viewpoint = nearest;
        return true;
    }

    nearest = raw_viewpoint;
    if (map_manager_->getNearestInfCellNot(super_utils::GridType::OCCUPIED,
                                           raw_viewpoint,
                                           nearest,
                                           repair_radius) &&
        isViewpointVisible(nearest, target))
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

    auto tryCandidate = [&](const Vec3f &raw_candidate, const double bias) {
        Vec3f candidate = raw_candidate;
        if (!repairViewpointEndpoint(raw_candidate, target.position, candidate))
        {
            return;
        }

        const double horizontal_error =
            (candidate - target.position).head<2>().norm() - cfg_.tracking_distance;
        const double vertical_error =
            (candidate.z() - target.position.z()) - cfg_.height_offset;
        const double score =
            (candidate - seed).squaredNorm() +
            0.35 * (candidate - preferred).squaredNorm() +
            2.0 * horizontal_error * horizontal_error +
            2.0 * vertical_error * vertical_error +
            bias;
        candidates.push_back(ViewpointCandidate{candidate, score});
    };

    tryCandidate(preferred, 0.0);
    tryCandidate(seed, 1.0);

    const int angle_count =
        std::max(8, static_cast<int>(std::ceil(2.0 * M_PI / std::max(0.05, cfg_.candidate_angle_step))));
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

Vec3f TrackingFrontend::extendToTrackingDistance(const Vec3f &seed,
                                                 const Vec3f &target,
                                                 const Vec3f &fallback) const
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
    Vec3f candidate = target + desired * dir;
    candidate.z() = target.z() + cfg_.height_offset;
    return candidate;
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

    if (!isViewpointSafe(start))
    {
        logTrackingFrontendFailure(cfg_.print_log,
                                   "Visible grid search start is not safe: " + pointToString(start));
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

    Vec3f preferred_rel_dir = target.velocity;
    preferred_rel_dir.z() = 0.0;
    if (preferred_rel_dir.norm() > 1.0e-4)
    {
        preferred_rel_dir = -preferred_rel_dir.normalized();
    }
    else
    {
        preferred_rel_dir = start - target.position;
        preferred_rel_dir.z() = 0.0;
        preferred_rel_dir = normalizedOr(preferred_rel_dir, Vec3f::UnitX());
    }
    Vec3f preferred = target.position + cfg_.tracking_distance * preferred_rel_dir;
    preferred.z() = desired_z;

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
        const Vec3f rel = candidate - target.position;
        const double h = rel.head<2>().norm();
        return h >= min_h &&
               h <= max_h &&
               std::abs(candidate.z() - desired_z) <= z_tol &&
               isViewpointVisible(candidate, target.position);
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
                                 "Visible grid search start is outside the bounded search box.");
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

        if (isVisibleTrackingCandidate(record.position))
        {
            viewpoint = record.position;
            reconstructPath(node.id);
            logTrackingFrontendDebug(cfg_.print_log,
                                     "Visible grid search success: viewpoint=" +
                                     pointToString(viewpoint) + ", expanded=" +
                                     std::to_string(expanded) + ", path_points=" +
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
        const Vec3f candidate = extendToTrackingDistance(seed, target.position, last_viewpoint);
        if (repairViewpointEndpoint(candidate, target.position, viewpoint))
        {
            if (appendPathSegment(last_viewpoint, viewpoint, path_to_viewpoint, true))
            {
                return true;
            }
        }
    }

    if (searchVisibleViewpointOnGrid(last_viewpoint, target, viewpoint, path_to_viewpoint))
    {
        return true;
    }

    if (!chooseVisibleViewpoint(last_viewpoint, target, viewpoint))
    {
        return false;
    }
    path_to_viewpoint.clear();
    return appendPathSegment(last_viewpoint, viewpoint, path_to_viewpoint, true);
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
        appendUnique(goal, path);
        return true;
    }

    if (!isViewpointSafe(start))
    {
        if (verbose)
        {
            logTrackingFrontendFailure(cfg_.print_log,
                                       "A* segment start is not safe: " + pointToString(start));
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

    if (map_manager_->isLineFree(search_start, goal, true, cfg_.unknown_as_occupied))
    {
        if (verbose)
        {
            logTrackingFrontendDebug(cfg_.print_log,
                                     "Line-free success for tracking guide segment: start=" +
                                     pointToString(search_start) + ", goal=" + pointToString(goal));
        }
        appendUnique(search_start, path);
        appendUnique(goal, path);
        return true;
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
            astar_->pointToPointPathSearch(search_start, goal, flag, cfg_.searching_horizon, astar_path, 0.08);
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
                                     pointToString(search_start) + ", goal=" + pointToString(goal));
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
    for (const auto &p : astar_path)
    {
        appendUnique(p, path);
    }
    appendUnique(goal, path);
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
    problem.visibility_samples = cfg_.visibility_samples;
    problem.od_h_lower = std::max(0.05, cfg_.tracking_distance - cfg_.distance_tolerance);
    problem.od_h_upper = std::max(problem.od_h_lower + 0.05, cfg_.tracking_distance + cfg_.distance_tolerance);
    problem.od_v_lower = cfg_.height_offset - cfg_.height_tolerance;
    problem.od_v_upper = cfg_.height_offset + cfg_.height_tolerance;

    vec_E<Vec3f> guide;
    appendUnique(head_pvaj.col(0), guide);
    problem.viewpoints.clear();
    problem.viewpoints.reserve(target_prediction.size());
    problem.target_sample_times.clear();
    problem.target_sample_times.reserve(target_prediction.size());
    traj_opt::DynamicTargetStates used_prediction;
    used_prediction.reserve(target_prediction.size());

    Vec3f seed = head_pvaj.col(0);
    for (std::size_t i = 0; i < target_prediction.size(); ++i)
    {
        const auto &last_target = used_prediction.empty() ? target_prediction.front() : used_prediction.back();
        const auto &target = target_prediction[i];
        Vec3f viewpoint = Vec3f::Zero();
        vec_E<Vec3f> path_to_viewpoint;
        if (!choosePropagatedViewpoint(seed, last_target, target, viewpoint, path_to_viewpoint))
        {
            logTrackingFrontendFailure(cfg_.print_log,
                                       "No safe visible tracking viewpoint found at target sample " +
                                       std::to_string(i) + ", t=" + std::to_string(target.t) + ".");
            return false;
        }
        if (path_to_viewpoint.empty())
        {
            logTrackingFrontendFailure(cfg_.print_log,
                                       "Failed to connect required tracking viewpoint at target sample " +
                                       std::to_string(i) + ", t=" + std::to_string(target.t) +
                                       ", seed=" + pointToString(seed) +
                                       ", viewpoint=" + pointToString(viewpoint) + ".");
            return false;
        }
        for (const auto &p : path_to_viewpoint)
        {
            appendUnique(p, guide);
        }
        seed = viewpoint;
        problem.viewpoints.emplace_back(viewpoint);
        problem.target_sample_times.emplace_back(target.t);
        used_prediction.emplace_back(target);
    }

    if (used_prediction.empty())
    {
        logTrackingFrontendFailure(cfg_.print_log,
                                   "No safe connected visible tracking viewpoint found.");
        return false;
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
    problem.target_prediction = used_prediction;
    if (!used_prediction.empty())
    {
        Vec3f tail_vel = used_prediction.back().velocity;
        if (!tail_vel.allFinite() || tail_vel.norm() < 0.05)
        {
            tail_vel.setZero();
        }
        problem.tail_pvaj.col(1) = tail_vel;
    }
    problem.min_total_duration = std::max(0.6, used_prediction.back().t);
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
