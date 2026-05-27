#include "exploration/local_guide_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace general_planner {
namespace exploration {

namespace {

ExplorationFailureType localGuideFailureFromReason(const std::string &reason) {
    if (reason.find("start is unsafe") != std::string::npos) {
        return ExplorationFailureType::LOCAL_START_UNSAFE;
    }
    if (reason.find("repair failed") != std::string::npos) {
        return ExplorationFailureType::LOCAL_REPAIR_FAILED;
    }
    if (reason.find("unsafe") != std::string::npos) {
        return ExplorationFailureType::LOCAL_SEGMENT_UNSAFE;
    }
    return ExplorationFailureType::LOCAL_SEGMENT_UNSAFE;
}

}  // namespace

LocalGuideBuilder::LocalGuideBuilder(Config cfg,
                                     MapManager::Ptr map_manager,
                                     path_search::Astar::Ptr astar)
        : cfg_(std::move(cfg)),
          map_manager_(std::move(map_manager)),
          astar_(std::move(astar)) {
    cfg_.local_goal_lookahead = std::max(0.5, cfg_.local_goal_lookahead);
    cfg_.local_goal_min_distance = std::max(0.0, cfg_.local_goal_min_distance);
    cfg_.final_goal_radius = std::max(0.1, cfg_.final_goal_radius);
    cfg_.planning_horizon = std::max(cfg_.local_goal_lookahead, cfg_.planning_horizon);
    cfg_.max_segment_length = std::max(0.2, cfg_.max_segment_length);
    cfg_.safe_distance = std::max(0.0, cfg_.safe_distance);
    cfg_.start_safe_distance = std::min(cfg_.safe_distance,
                                        std::max(0.0, cfg_.start_safe_distance));
    cfg_.line_step = std::max(0.05, cfg_.line_step);
}

bool LocalGuideBuilder::build(const Request &request, Result &result) const {
    result = Result{};
    if (!request.robot_pos.allFinite() || !request.final_goal.allFinite()) {
        result.failure_type = ExplorationFailureType::LOCAL_SEGMENT_UNSAFE;
        result.reason = "invalid guide endpoints";
        return false;
    }

    const auto &raw = request.route.path;
    super_utils::vec_E<super_utils::Vec3f> source;
    double progress_to_nearest = 0.0;

    if (raw.empty()) {
        source.push_back(request.robot_pos);
        source.push_back(request.final_goal);
    } else {
        struct RouteProjection {
            bool valid{false};
            std::size_t segment_id{0U};
            double segment_t{0.0};
            double distance_sq{std::numeric_limits<double>::infinity()};
            double progress{0.0};
            super_utils::Vec3f point{super_utils::Vec3f::Zero()};
        };

        RouteProjection best;
        double accumulated = 0.0;
        for (std::size_t i = 0; i + 1U < raw.size(); ++i) {
            const auto &a = raw[i];
            const auto &b = raw[i + 1U];
            const super_utils::Vec3f ab = b - a;
            const double len_sq = ab.squaredNorm();
            const double len = std::sqrt(len_sq);
            double t = 0.0;
            super_utils::Vec3f projected = a;
            if (len_sq > 1.0e-8) {
                t = std::clamp((request.robot_pos - a).dot(ab) / len_sq, 0.0, 1.0);
                projected = a + t * ab;
            }
            const double distance_sq = (projected - request.robot_pos).squaredNorm();
            if (!best.valid || distance_sq < best.distance_sq) {
                best.valid = true;
                best.segment_id = i;
                best.segment_t = t;
                best.distance_sq = distance_sq;
                best.progress = accumulated + t * len;
                best.point = projected;
            }
            accumulated += len;
        }
        if (!best.valid) {
            for (std::size_t i = 0; i < raw.size(); ++i) {
                const double distance_sq = (raw[i] - request.robot_pos).squaredNorm();
                if (!best.valid || distance_sq < best.distance_sq) {
                    best.valid = true;
                    best.segment_id = i;
                    best.segment_t = 0.0;
                    best.distance_sq = distance_sq;
                    best.progress = 0.0;
                    best.point = raw[i];
                }
            }
        }
        progress_to_nearest = best.valid ? best.progress : 0.0;

        source.push_back(request.robot_pos);
        if (best.valid && (best.point - source.back()).norm() > 1.0e-4) {
            source.push_back(best.point);
        }
        const std::size_t next_id = best.valid
                                    ? std::min(best.segment_id + 1U, raw.size())
                                    : 0U;
        for (std::size_t i = next_id; i < raw.size(); ++i) {
            if ((raw[i] - source.back()).norm() > 1.0e-4) {
                source.push_back(raw[i]);
            }
        }
    }
    if ((source.back() - request.final_goal).norm() > cfg_.final_goal_radius) {
        source.push_back(request.final_goal);
    }

    super_utils::vec_E<super_utils::Vec3f> prefix;
    prefix.push_back(request.robot_pos);
    double length = 0.0;
    super_utils::Vec3f last = request.robot_pos;
    bool reached_final = (request.robot_pos - request.final_goal).norm() <= cfg_.final_goal_radius;
    for (std::size_t i = 0U; i < source.size(); ++i) {
        const super_utils::Vec3f point = source[i];
        const double step = (point - last).norm();
        if (step < 1.0e-4) {
            continue;
        }
        if (length + step > cfg_.local_goal_lookahead &&
            length >= cfg_.local_goal_min_distance) {
            const double remain = std::max(0.0, cfg_.local_goal_lookahead - length);
            if (remain > 0.2) {
                prefix.push_back(last + (point - last) / step * remain);
            }
            break;
        }
        prefix.push_back(point);
        length += step;
        last = point;
        if ((point - request.final_goal).norm() <= cfg_.final_goal_radius) {
            reached_final = true;
            break;
        }
        if (length >= cfg_.planning_horizon) {
            break;
        }
    }
    if (!reached_final &&
        (prefix.back() - request.final_goal).norm() <= std::max(cfg_.final_goal_radius,
                                                                 cfg_.local_goal_min_distance)) {
        prefix.push_back(request.final_goal);
        reached_final = true;
    }

    if (prefix.size() < 2U) {
        prefix.push_back(request.final_goal);
        reached_final = true;
    }

    super_utils::vec_E<super_utils::Vec3f> refined =
            cfg_.shortcut_enable ? shortcutPath(prefix) : prefix;
    super_utils::vec_E<super_utils::Vec3f> repaired;
    repaired.reserve(refined.size());
    repaired.push_back(refined.front());
    bool trimmed_before_final = false;
    std::string trim_reason;
    for (std::size_t i = 0; i + 1U < refined.size(); ++i) {
        const auto &a = repaired.back();
        const auto &b = refined[i + 1U];
        if (!appendSafeOrRepairedSegment(a, b, repaired, trimmed_before_final, trim_reason)) {
            result.reason = trim_reason.empty() ? "local guide unsafe and repair failed"
                                                : trim_reason;
            result.failure_type = localGuideFailureFromReason(result.reason);
            return false;
        }
        if (trimmed_before_final) {
            reached_final = false;
            break;
        }
    }
    densifyPath(repaired);
    if (repaired.size() < 2U) {
        result.failure_type = ExplorationFailureType::LOCAL_SEGMENT_UNSAFE;
        result.reason = "refined guide too short";
        return false;
    }

    result.valid = true;
    result.guide_path = std::move(repaired);
    result.next_goal = result.guide_path.back();
    result.local_goal_is_final =
            !trimmed_before_final &&
            (reached_final || (result.next_goal - request.final_goal).norm() <= cfg_.final_goal_radius);
    result.next_yaw = result.local_goal_is_final
                      ? request.final_yaw
                      : headingFromPath(result.guide_path, request.current_yaw);
    result.route_progress_length = progress_to_nearest;
    result.reason = trimmed_before_final ? trim_reason : "local guide refined";
    return true;
}

bool LocalGuideBuilder::segmentSafe(const super_utils::Vec3f &a,
                                    const super_utils::Vec3f &b) const {
    return segmentSafe(a, b, cfg_.safe_distance);
}

bool LocalGuideBuilder::segmentSafe(const super_utils::Vec3f &a,
                                    const super_utils::Vec3f &b,
                                    const double safe_distance) const {
    return map_manager_ == nullptr ||
           map_manager_->isSegmentSafe(a, b, safe_distance, cfg_.backend, cfg_.line_step);
}

bool LocalGuideBuilder::repairSegment(const super_utils::Vec3f &a,
                                      const super_utils::Vec3f &b,
                                      super_utils::vec_E<super_utils::Vec3f> &path) const {
    path.clear();
    if (!cfg_.astar_repair_enable || astar_ == nullptr || map_manager_ == nullptr ||
        !map_manager_->insideLocalMap(a) || !map_manager_->insideLocalMap(b)) {
        return false;
    }
    const int flag = path_search::ON_INF_MAP |
                     (cfg_.unknown_as_occupied ? path_search::UNKNOWN_AS_OCCUPIED : 0);
    const auto ret = astar_->pointToPointPathSearch(a,
                                                    b,
                                                    flag,
                                                    std::max(cfg_.planning_horizon,
                                                             (b - a).norm() + 1.0),
                                                    path,
                                                    0.05);
    return ret == super_utils::SUCCESS && path.size() >= 2U;
}

bool LocalGuideBuilder::appendSafeOrRepairedSegment(
        const super_utils::Vec3f &a,
        const super_utils::Vec3f &b,
        super_utils::vec_E<super_utils::Vec3f> &path,
        bool &trimmed,
        std::string &reason) const {
    trimmed = false;
    if (segmentSafe(a, b)) {
        if ((b - path.back()).norm() > 1.0e-4) {
            path.push_back(b);
        }
        return true;
    }

    const bool first_segment =
            path.size() <= 1U && !path.empty() && (a - path.front()).norm() < 1.0e-4;
    const bool relaxed_start =
            first_segment && !stateSafe(a) && stateSafe(a, cfg_.start_safe_distance);
    if (relaxed_start && stateSafe(b) && segmentSafe(a, b, cfg_.start_safe_distance)) {
        if ((b - path.back()).norm() > 1.0e-4) {
            path.push_back(b);
        }
        return true;
    }

    super_utils::vec_E<super_utils::Vec3f> repair;
    if (repairSegment(a, b, repair) && repair.size() >= 2U) {
        for (std::size_t i = 1; i < repair.size(); ++i) {
            if ((repair[i] - path.back()).norm() > 1.0e-4) {
                path.push_back(repair[i]);
            }
        }
        return true;
    }

    return appendSafePrefixOfSegment(a, b, path, trimmed, reason);
}

bool LocalGuideBuilder::appendSafePrefixOfSegment(
        const super_utils::Vec3f &a,
        const super_utils::Vec3f &b,
        super_utils::vec_E<super_utils::Vec3f> &path,
        bool &trimmed,
        std::string &reason) const {
    trimmed = false;
    const super_utils::Vec3f delta = b - a;
    const double length = delta.norm();
    if (length < 1.0e-4) {
        reason = "local guide unsafe zero-length segment";
        return false;
    }
    const bool first_segment =
            path.size() <= 1U && !path.empty() && (a - path.front()).norm() < 1.0e-4;
    const bool start_full_safe = stateSafe(a);
    const bool relaxed_start =
            first_segment && !start_full_safe && stateSafe(a, cfg_.start_safe_distance);
    if (!start_full_safe && !relaxed_start) {
        reason = "local guide start is unsafe";
        return false;
    }

    const int samples = std::max(2, static_cast<int>(
            std::ceil(length / std::max(0.05, cfg_.line_step))));
    super_utils::Vec3f last_safe = a;
    bool waiting_for_full_clearance = relaxed_start;
    for (int i = 1; i <= samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples);
        const super_utils::Vec3f p = a + ratio * delta;
        if (waiting_for_full_clearance) {
            if (!stateSafe(p, cfg_.start_safe_distance) ||
                !segmentSafe(last_safe, p, cfg_.start_safe_distance)) {
                break;
            }
            last_safe = p;
            if (stateSafe(p)) {
                waiting_for_full_clearance = false;
            }
            continue;
        }
        if (!stateSafe(p) || !segmentSafe(last_safe, p)) {
            break;
        }
        last_safe = p;
    }

    const double usable = (last_safe - a).norm();
    const double total = pathLength(path) + usable;
    const double min_usable =
            first_segment ? std::max(0.15, cfg_.max_segment_length * 0.15)
                          : std::max(0.2, cfg_.max_segment_length * 0.25);
    const double min_total =
            first_segment ? min_usable : cfg_.local_goal_min_distance;
    if (usable < min_usable || total < min_total) {
        reason = "local guide unsafe and repair failed";
        return false;
    }
    if ((last_safe - path.back()).norm() > 1.0e-4) {
        path.push_back(last_safe);
    }
    trimmed = true;
    reason = "local guide trimmed before unsafe segment";
    return true;
}

bool LocalGuideBuilder::stateSafe(const super_utils::Vec3f &p) const {
    return stateSafe(p, cfg_.safe_distance);
}

bool LocalGuideBuilder::stateSafe(const super_utils::Vec3f &p,
                                  const double safe_distance) const {
    return map_manager_ == nullptr || map_manager_->isStateSafe(p, safe_distance, cfg_.backend);
}

super_utils::vec_E<super_utils::Vec3f> LocalGuideBuilder::shortcutPath(
        const super_utils::vec_E<super_utils::Vec3f> &path) const {
    if (path.size() <= 2U) {
        return path;
    }
    super_utils::vec_E<super_utils::Vec3f> out;
    std::size_t anchor = 0U;
    out.push_back(path.front());
    while (anchor + 1U < path.size()) {
        std::size_t best = anchor + 1U;
        for (std::size_t candidate = path.size() - 1U; candidate > anchor + 1U; --candidate) {
            if (segmentSafe(path[anchor], path[candidate])) {
                best = candidate;
                break;
            }
        }
        out.push_back(path[best]);
        anchor = best;
    }
    return out;
}

void LocalGuideBuilder::densifyPath(super_utils::vec_E<super_utils::Vec3f> &path) const {
    if (path.size() < 2U) {
        return;
    }
    super_utils::vec_E<super_utils::Vec3f> dense;
    dense.push_back(path.front());
    for (std::size_t i = 1; i < path.size(); ++i) {
        const super_utils::Vec3f start = dense.back();
        const super_utils::Vec3f end = path[i];
        const double length = (end - start).norm();
        const int pieces = std::max(1, static_cast<int>(std::ceil(length / cfg_.max_segment_length)));
        for (int j = 1; j <= pieces; ++j) {
            const double ratio = static_cast<double>(j) / static_cast<double>(pieces);
            dense.push_back(start + ratio * (end - start));
        }
    }
    path.swap(dense);
}

double LocalGuideBuilder::pathLength(const super_utils::vec_E<super_utils::Vec3f> &path) {
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}

double LocalGuideBuilder::headingFromPath(const super_utils::vec_E<super_utils::Vec3f> &path,
                                          const double fallback_yaw) {
    if (path.size() < 2U) {
        return fallback_yaw;
    }
    const super_utils::Vec3f delta = path.back() - path[path.size() - 2U];
    if (delta.head<2>().norm() < 1.0e-6) {
        return fallback_yaw;
    }
    return std::atan2(delta.y(), delta.x());
}

}  // namespace exploration
}  // namespace general_planner
