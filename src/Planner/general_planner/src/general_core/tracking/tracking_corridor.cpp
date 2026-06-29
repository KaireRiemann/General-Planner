/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_internal_utils.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <fmt/format.h>

using namespace general_utils;

namespace general_planner {
    namespace {
        void setFailureReason(std::string *out, const std::string &reason) {
            if (out != nullptr) {
                *out = reason;
            }
        }

        void appendGuideUnique(const Vec3f &point, vec_Vec3f &path) {
            if (!point.allFinite()) {
                return;
            }
            if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
                path.emplace_back(point);
            }
        }

        void appendGuideTimedUnique(const Vec3f &point,
                                    const double stamp,
                                    vec_Vec3f &path,
                                    std::vector<double> &path_t) {
            if (!point.allFinite() || !std::isfinite(stamp)) {
                return;
            }
            if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
                path.emplace_back(point);
                path_t.emplace_back(stamp);
            } else if (!path_t.empty()) {
                path_t.back() = stamp;
            }
        }

        Vec3f interpolatePointOnTimedGuide(const vec_Vec3f &path,
                                           const std::vector<double> &path_t,
                                           const double query_t) {
            if (path.empty()) {
                return Vec3f::Zero();
            }
            if (path_t.size() != path.size()) {
                return path.back();
            }
            if (path.size() == 1 || query_t <= path_t.front()) {
                return path.front();
            }
            if (query_t >= path_t.back()) {
                return path.back();
            }
            const auto it = std::lower_bound(path_t.begin(), path_t.end(), query_t);
            const std::size_t right = static_cast<std::size_t>(std::distance(path_t.begin(), it));
            if (right == 0 || right >= path.size()) {
                return path.back();
            }
            const std::size_t left = right - 1;
            const double dt = std::max(1.0e-9, path_t[right] - path_t[left]);
            const double alpha = std::clamp((query_t - path_t[left]) / dt, 0.0, 1.0);
            return path[left] + alpha * (path[right] - path[left]);
        }

        double interpolateSegmentStamp(const std::vector<double> &times,
                                       const int left_id,
                                       const double alpha,
                                       const double fallback_start_t,
                                       const double fallback_end_t) {
            if (times.size() > static_cast<std::size_t>(left_id + 1) &&
                std::isfinite(times[static_cast<std::size_t>(left_id)]) &&
                std::isfinite(times[static_cast<std::size_t>(left_id + 1)])) {
                const double left_t = times[static_cast<std::size_t>(left_id)];
                const double right_t = std::max(left_t, times[static_cast<std::size_t>(left_id + 1)]);
                return left_t + alpha * (right_t - left_t);
            }
            return fallback_start_t + alpha * std::max(0.0, fallback_end_t - fallback_start_t);
        }

    }

    bool GeneralPlanner::trackingGuidePointSafe(const Vec3f &point) const {
        if (!point.allFinite()) {
            return false;
        }
        if (map_manager_ == nullptr || !map_manager_->ready()) {
            return true;
        }
        if (!map_manager_->insideLocalMap(point)) {
            return false;
        }
        const auto grid_type = map_manager_->getInfGridType(point);
        if (grid_type == rog_map::GridType::OCCUPIED ||
            grid_type == rog_map::GridType::OUT_OF_MAP) {
            return false;
        }
        if (cfg_.tracking_unknown_as_occupied &&
            (grid_type == rog_map::GridType::UNKNOWN ||
             grid_type == rog_map::GridType::UNDEFINED ||
             grid_type == rog_map::GridType::FRONTIER)) {
            return false;
        }
        return true;
    }

    bool GeneralPlanner::densifyTrackingGuideForCorridor(const vec_Vec3f &guide_path,
                                                         const std::vector<double> &guide_t,
                                                         vec_Vec3f &dense_path,
                                                         std::vector<double> &dense_t) const {
        dense_path.clear();
        dense_t.clear();
        if (guide_path.size() < 2) {
            return false;
        }

        double max_step = 0.8 * cfg_.corridor_line_max_length;
        if (!std::isfinite(max_step) || max_step <= 1.0e-3) {
            max_step = map_manager_ != nullptr ? 4.0 * std::max(0.05, map_manager_->getResolution()) : 0.5;
        }
        max_step = std::clamp(max_step, 0.2, std::max(0.2, cfg_.corridor_line_max_length));

        if (!trackingGuidePointSafe(guide_path.front())) {
            return false;
        }
        const bool has_valid_times = guide_t.size() == guide_path.size();
        double fallback_stamp = has_valid_times && std::isfinite(guide_t.front()) ? guide_t.front() : 0.0;
        appendGuideTimedUnique(guide_path.front(), fallback_stamp, dense_path, dense_t);

        for (int i = 1; i < static_cast<int>(guide_path.size()); ++i) {
            const Vec3f start = dense_path.back();
            const Vec3f goal = guide_path[static_cast<std::size_t>(i)];
            if (!trackingGuidePointSafe(goal)) {
                dense_path.clear();
                dense_t.clear();
                return false;
            }

            const double segment_len = (goal - start).norm();
            if (!std::isfinite(segment_len)) {
                dense_path.clear();
                dense_t.clear();
                return false;
            }
            const int segment_num = std::max(1, static_cast<int>(std::ceil(segment_len / max_step)));
            Vec3f last = start;
            const double fallback_start_t = fallback_stamp;
            fallback_stamp += std::max(0.05, segment_len / 2.0);
            for (int seg = 1; seg <= segment_num; ++seg) {
                const double alpha = static_cast<double>(seg) / static_cast<double>(segment_num);
                Vec3f point = start + alpha * (goal - start);
                if (seg == segment_num) {
                    point = goal;
                }
                if (!trackingGuidePointSafe(point)) {
                    dense_path.clear();
                    dense_t.clear();
                    return false;
                }
                if (map_manager_ != nullptr && map_manager_->ready() &&
                    !map_manager_->isLineFree(last, point, true, cfg_.tracking_unknown_as_occupied)) {
                    dense_path.clear();
                    dense_t.clear();
                    return false;
                }
                const double stamp = interpolateSegmentStamp(guide_t,
                                                             i - 1,
                                                             alpha,
                                                             fallback_start_t,
                                                             fallback_stamp);
                appendGuideTimedUnique(point, stamp, dense_path, dense_t);
                last = point;
            }
        }

        return dense_path.size() >= 2 && dense_path.size() == dense_t.size();
    }

    void GeneralPlanner::refreshTrackingGuideTiming(traj_opt::TrackingProblem &problem) const {
        problem.guide_t.clear();
        problem.guide_t.reserve(problem.guide_path.size());
        double stamp = 0.0;
        problem.guide_t.emplace_back(stamp);
        for (int i = 1; i < static_cast<int>(problem.guide_path.size()); ++i) {
            stamp += std::max(0.1, (problem.guide_path[i] - problem.guide_path[i - 1]).norm() / 2.0);
            problem.guide_t.emplace_back(stamp);
        }

        problem.tail_pvaj.col(0) = problem.guide_path.back();
        if (!problem.target_prediction.empty()) {
            Vec3f tail_vel = problem.target_prediction.back().velocity;
            if (problem.static_tracking_mode ||
                !tail_vel.allFinite() ||
                tail_vel.norm() < cfg_.tracking_static_tail_speed_epsilon) {
                tail_vel.setZero();
            }
            problem.tail_pvaj.col(1) = tail_vel;
            problem.min_total_duration = std::max(0.6, problem.target_prediction.back().t);
        } else {
            problem.tail_pvaj.col(1).setZero();
            problem.min_total_duration = std::max(0.6, problem.guide_t.back());
        }
    }

    void GeneralPlanner::refreshTrackingGuideEndpoint(traj_opt::TrackingProblem &problem) const {
        if (problem.guide_path.empty()) {
            return;
        }
        problem.tail_pvaj.col(0) = problem.guide_path.back();
        if (!problem.target_prediction.empty()) {
            Vec3f tail_vel = problem.target_prediction.back().velocity;
            if (problem.static_tracking_mode ||
                !tail_vel.allFinite() ||
                tail_vel.norm() < cfg_.tracking_static_tail_speed_epsilon) {
                tail_vel.setZero();
            }
            problem.tail_pvaj.col(1) = tail_vel;
            problem.min_total_duration = std::max(0.6, problem.target_prediction.back().t);
        } else if (!problem.guide_t.empty()) {
            problem.tail_pvaj.col(1).setZero();
            problem.min_total_duration = std::max(0.6, problem.guide_t.back());
        }
    }

    bool GeneralPlanner::findTrackingViewpointReference(
            const traj_opt::DynamicTargetStates &target_prediction,
            Vec3f &reference_viewpoint,
            traj_opt::DynamicTargetState &reference_target) const {
        if (target_prediction.empty() ||
            last_tracking_frontend_prediction_.empty() ||
            last_tracking_frontend_viewpoints_.empty() ||
            last_tracking_frontend_prediction_.size() != last_tracking_frontend_viewpoints_.size()) {
            return false;
        }

        const Vec3f &target0 = target_prediction.front().position;
        double best_score = std::numeric_limits<double>::infinity();
        std::size_t best_idx = 0;
        for (std::size_t i = 0; i < last_tracking_frontend_prediction_.size(); ++i) {
            const auto &old_target = last_tracking_frontend_prediction_[i];
            const auto &old_viewpoint = last_tracking_frontend_viewpoints_[i];
            if (!old_target.position.allFinite() || !old_viewpoint.allFinite()) {
                continue;
            }
            const double score = (old_target.position - target0).norm();
            if (score < best_score) {
                best_score = score;
                best_idx = i;
            }
        }

        const double max_match_dist =
                std::max({1.0,
                          cfg_.tracking_distance,
                          cfg_.tracking_distance + cfg_.tracking_distance_upper_tolerance});
        if (!std::isfinite(best_score) || best_score > max_match_dist) {
            return false;
        }

        reference_viewpoint = last_tracking_frontend_viewpoints_[best_idx];
        reference_target = last_tracking_frontend_prediction_[best_idx];
        return trackingGuidePointSafe(reference_viewpoint);
    }

    void GeneralPlanner::rememberTrackingViewpointReference(
            const traj_opt::TrackingProblem &problem) {
        if (problem.target_prediction.empty() ||
            problem.viewpoints.empty() ||
            problem.target_prediction.size() != problem.viewpoints.size()) {
            last_tracking_frontend_prediction_.clear();
            last_tracking_frontend_viewpoints_.clear();
            return;
        }
        last_tracking_frontend_prediction_ = problem.target_prediction;
        last_tracking_frontend_viewpoints_ = problem.viewpoints;
    }

    bool GeneralPlanner::tryGenerateTrackingCorridor(const vec_Vec3f &guide_path,
                                                     PolytopeVec &sfcs,
                                                     std::string *failure_reason) {
        sfcs.clear();
        if (cg_ptr_ == nullptr) {
            setFailureReason(failure_reason, "corridor_generator_null");
            return false;
        }
        if (guide_path.size() < 2) {
            setFailureReason(failure_reason,
                             fmt::format("guide_path_too_short(size={})", guide_path.size()));
            return false;
        }
        for (std::size_t i = 0; i < guide_path.size(); ++i) {
            const auto &point = guide_path[i];
            if (!trackingGuidePointSafe(point)) {
                setFailureReason(failure_reason,
                                 fmt::format("unsafe_guide_point(index={}, p=[{:.3f},{:.3f},{:.3f}])",
                                             i, point.x(), point.y(), point.z()));
                return false;
            }
        }

        Vec3f shifted_start_pt = Vec3f(9999, 9999, 9999);
        bool ok = false;
        try {
            ok = cg_ptr_->SearchPolytopeOnPath(guide_path, sfcs, shifted_start_pt, false);
        } catch (const std::exception &e) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking SFC generation threw exception: {}", e.what());
            setFailureReason(failure_reason, fmt::format("SearchPolytopeOnPath_exception({})", e.what()));
            sfcs.clear();
            return false;
        }
        if (!ok) {
            setFailureReason(failure_reason,
                             fmt::format("SearchPolytopeOnPath_returned_false(guide_size={})",
                                         guide_path.size()));
            sfcs.clear();
            return false;
        }
        if (sfcs.empty()) {
            setFailureReason(failure_reason, "SearchPolytopeOnPath_returned_empty_sfc");
            sfcs.clear();
            return false;
        }

        for (std::size_t i = 0; i < sfcs.size(); ++i) {
            const auto &poly = sfcs[i];
            const auto planes = poly.GetPlanes();
            if (planes.rows() == 0 || !std::isfinite(planes.sum())) {
                setFailureReason(failure_reason,
                                 fmt::format("invalid_sfc_poly(index={}, rows={}, finite={})",
                                             i, planes.rows(), std::isfinite(planes.sum())));
                sfcs.clear();
                return false;
            }
        }

        for (std::size_t i = 0; i < guide_path.size(); ++i) {
            const auto &point = guide_path[i];
            bool covered = false;
            for (const auto &poly: sfcs) {
                if (poly.PointIsInside(point, 0.05)) {
                    covered = true;
                    break;
                }
            }
            if (!covered) {
                setFailureReason(failure_reason,
                                 fmt::format("guide_point_not_covered_by_sfc(index={}, p=[{:.3f},{:.3f},{:.3f}], sfc_count={})",
                                             i, point.x(), point.y(), point.z(), sfcs.size()));
                sfcs.clear();
                return false;
            }
        }
        return true;
    }

    bool GeneralPlanner::repairTrackingGuideWithAstar(const vec_Vec3f &guide_path,
                                                      const std::vector<double> &guide_t,
                                                      vec_Vec3f &repaired_path,
                                                      std::vector<double> &repaired_t,
                                                      std::string *failure_reason) {
        repaired_path.clear();
        repaired_t.clear();
        if (guide_path.size() < 2) {
            setFailureReason(failure_reason,
                             fmt::format("astar_repair_guide_too_short(size={})", guide_path.size()));
            return false;
        }
        if (astar_ptr_ == nullptr) {
            setFailureReason(failure_reason, "astar_repair_astar_null");
            return false;
        }
        if (!trackingGuidePointSafe(guide_path.front())) {
            const auto &p = guide_path.front();
            setFailureReason(failure_reason,
                             fmt::format("astar_repair_start_unsafe(p=[{:.3f},{:.3f},{:.3f}])",
                                         p.x(), p.y(), p.z()));
            return false;
        }

        const bool has_valid_times = guide_t.size() == guide_path.size();
        double fallback_stamp = has_valid_times && std::isfinite(guide_t.front()) ? guide_t.front() : 0.0;
        appendGuideTimedUnique(guide_path.front(), fallback_stamp, repaired_path, repaired_t);
        const int astar_flag = path_search::ON_INF_MAP |
                               (cfg_.tracking_unknown_as_occupied
                                    ? path_search::UNKNOWN_AS_OCCUPIED
                                    : path_search::UNKNOWN_AS_FREE) |
                               path_search::USE_INF_NEIGHBOR;

        bool full_repair = true;
        for (int i = 1; i < static_cast<int>(guide_path.size()); ++i) {
            const Vec3f start = repaired_path.back();
            const Vec3f goal = guide_path[i];
            if (!trackingGuidePointSafe(goal)) {
                setFailureReason(failure_reason,
                                 fmt::format("astar_repair_goal_unsafe(segment={}, goal=[{:.3f},{:.3f},{:.3f}])",
                                             i, goal.x(), goal.y(), goal.z()));
                full_repair = false;
                break;
            }

            if (map_manager_ == nullptr || !map_manager_->ready() ||
                map_manager_->isLineFree(start, goal, true, cfg_.tracking_unknown_as_occupied)) {
                const double stamp = has_valid_times ? guide_t[static_cast<std::size_t>(i)]
                                                     : fallback_stamp + std::max(0.05, (goal - start).norm() / 2.0);
                appendGuideTimedUnique(goal, stamp, repaired_path, repaired_t);
                fallback_stamp = stamp;
                continue;
            }

            vec_Vec3f astar_path;
            const RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(start,
                                                                         goal,
                                                                         astar_flag,
                                                                         cfg_.planning_horizon,
                                                                         astar_path,
                                                                         0.08);
            if ((ret_code != SUCCESS && ret_code != REACH_GOAL) || astar_path.empty()) {
                setFailureReason(failure_reason,
                                 fmt::format("astar_repair_search_failed(segment={}, ret={}, path_size={}, start=[{:.3f},{:.3f},{:.3f}], goal=[{:.3f},{:.3f},{:.3f}])",
                                             i, ret_code, astar_path.size(),
                                             start.x(), start.y(), start.z(),
                                             goal.x(), goal.y(), goal.z()));
                full_repair = false;
                break;
            }

            Vec3f last = start;
            for (std::size_t path_id = 0; path_id < astar_path.size(); ++path_id) {
                const auto &point = astar_path[path_id];
                if (!trackingGuidePointSafe(point)) {
                    setFailureReason(failure_reason,
                                     fmt::format("astar_repair_path_point_unsafe(segment={}, path_index={}, p=[{:.3f},{:.3f},{:.3f}])",
                                                 i, path_id, point.x(), point.y(), point.z()));
                    full_repair = false;
                    break;
                }
                if (map_manager_ != nullptr && map_manager_->ready() &&
                    !map_manager_->isLineFree(last, point, true, cfg_.tracking_unknown_as_occupied)) {
                    setFailureReason(failure_reason,
                                     fmt::format("astar_repair_path_segment_blocked(segment={}, path_index={}, from=[{:.3f},{:.3f},{:.3f}], to=[{:.3f},{:.3f},{:.3f}])",
                                                 i, path_id,
                                                 last.x(), last.y(), last.z(),
                                                 point.x(), point.y(), point.z()));
                    full_repair = false;
                    break;
                }
                last = point;
            }
            if (!full_repair) {
                break;
            }
            if (map_manager_ != nullptr && map_manager_->ready() &&
                !map_manager_->isLineFree(repaired_path.back(), goal, true, cfg_.tracking_unknown_as_occupied)) {
                setFailureReason(failure_reason,
                                 fmt::format("astar_repair_final_segment_blocked(segment={}, from=[{:.3f},{:.3f},{:.3f}], goal=[{:.3f},{:.3f},{:.3f}])",
                                             i,
                                             repaired_path.back().x(), repaired_path.back().y(), repaired_path.back().z(),
                                             goal.x(), goal.y(), goal.z()));
                full_repair = false;
                break;
            }

            vec_Vec3f segment_path;
            appendGuideUnique(start, segment_path);
            for (const auto &point: astar_path) {
                appendGuideUnique(point, segment_path);
            }
            appendGuideUnique(goal, segment_path);

            std::vector<double> segment_accum(segment_path.size(), 0.0);
            for (int sid = 1; sid < static_cast<int>(segment_path.size()); ++sid) {
                segment_accum[static_cast<std::size_t>(sid)] =
                        segment_accum[static_cast<std::size_t>(sid - 1)] +
                        (segment_path[static_cast<std::size_t>(sid)] -
                         segment_path[static_cast<std::size_t>(sid - 1)]).norm();
            }
            const double start_t = repaired_t.empty() ? fallback_stamp : repaired_t.back();
            const double goal_t = has_valid_times
                                      ? std::max(start_t, guide_t[static_cast<std::size_t>(i)])
                                      : start_t + std::max(0.05, segment_accum.back() / 2.0);
            for (int sid = 1; sid < static_cast<int>(segment_path.size()); ++sid) {
                const double ratio = segment_accum.back() > 1.0e-6
                                         ? segment_accum[static_cast<std::size_t>(sid)] / segment_accum.back()
                                         : 1.0;
                appendGuideTimedUnique(segment_path[static_cast<std::size_t>(sid)],
                                       start_t + ratio * (goal_t - start_t),
                                       repaired_path,
                                       repaired_t);
            }
            fallback_stamp = goal_t;
        }

        if (!full_repair) {
            if (failure_reason != nullptr && failure_reason->empty()) {
                *failure_reason = "astar_repair_failed";
            }
            return false;
        }
        if (repaired_path.size() < 2 || repaired_path.size() != repaired_t.size()) {
            setFailureReason(failure_reason,
                             fmt::format("astar_repair_invalid_output(path_size={}, time_size={})",
                                         repaired_path.size(), repaired_t.size()));
            return false;
        }
        return true;
    }

    bool GeneralPlanner::truncateTrackingProblemForCorridor(traj_opt::TrackingProblem &problem,
                                                            const vec_Vec3f &candidate_guide,
                                                            const std::vector<double> &candidate_guide_t,
                                                            PolytopeVec &sfcs,
                                                            std::string *failure_reason) {
        if (candidate_guide.size() < 2) {
            setFailureReason(failure_reason,
                             fmt::format("truncate_candidate_too_short(size={})", candidate_guide.size()));
            return false;
        }
        if (candidate_guide.size() != candidate_guide_t.size()) {
            setFailureReason(failure_reason,
                             fmt::format("truncate_candidate_time_size_mismatch(path_size={}, time_size={})",
                                         candidate_guide.size(), candidate_guide_t.size()));
            return false;
        }
        if (problem.viewpoints.empty()) {
            setFailureReason(failure_reason, "truncate_no_viewpoints");
            return false;
        }

        auto applyViewpointTruncation =
                [&](const vec_Vec3f &prefix,
                    const std::vector<double> &prefix_t,
                    const std::size_t keep_count) {
                    problem.guide_path = prefix;
                    problem.guide_t = prefix_t;
                    problem.viewpoints.resize(keep_count);
                    problem.target_sample_times.resize(keep_count);
                    problem.target_prediction.resize(keep_count);
                    if (problem.visible_regions.size() > keep_count) {
                        problem.visible_regions.resize(keep_count);
                    }
                    refreshTrackingGuideEndpoint(problem);
                };

        auto applyTimeAlignedTruncation =
                [&](const vec_Vec3f &prefix,
                    const std::vector<double> &prefix_t) -> bool {
                    if (prefix.size() < 2 ||
                        prefix_t.size() != prefix.size() ||
                        problem.target_prediction.empty()) {
                        return false;
                    }
                    const double end_t = prefix_t.back();
                    if (!std::isfinite(end_t)) {
                        return false;
                    }

                    traj_opt::DynamicTargetStates truncated_target_prediction;
                    std::vector<double> truncated_target_sample_times;
                    vec_Vec3f truncated_viewpoints;
                    general_utils::vec_E<traj_opt::TrackingVisibleRegion> truncated_visible_regions;

                    auto appendTimedTrackingSample =
                            [&](const double sample_t,
                                const traj_opt::DynamicTargetState &state,
                                const traj_opt::TrackingVisibleRegion *source_region) {
                                if (!std::isfinite(sample_t)) {
                                    return;
                                }
                                const Vec3f viewpoint =
                                        interpolatePointOnTimedGuide(prefix, prefix_t, sample_t);
                                truncated_target_prediction.emplace_back(state);
                                truncated_target_prediction.back().t = sample_t;
                                truncated_target_sample_times.emplace_back(sample_t);
                                truncated_viewpoints.emplace_back(viewpoint);
                                if (!problem.visible_regions.empty()) {
                                    traj_opt::TrackingVisibleRegion region;
                                    if (source_region != nullptr) {
                                        region = *source_region;
                                    } else {
                                        region.valid = false;
                                        region.confidence = 0.0;
                                        region.theta = 0.0;
                                    }
                                    region.t = sample_t;
                                    region.target_position = state.position;
                                    region.visible_point = viewpoint;
                                    truncated_visible_regions.emplace_back(region);
                                }
                            };

                    const bool has_explicit_sample_times =
                            problem.target_sample_times.size() == problem.target_prediction.size();
                    for (std::size_t i = 0; i < problem.target_prediction.size(); ++i) {
                        const double sample_t =
                                has_explicit_sample_times
                                    ? problem.target_sample_times[i]
                                    : problem.target_prediction[i].t;
                        if (sample_t <= end_t + 1.0e-6) {
                            const traj_opt::TrackingVisibleRegion *source_region =
                                    i < problem.visible_regions.size()
                                        ? &problem.visible_regions[i]
                                        : nullptr;
                            appendTimedTrackingSample(sample_t,
                                                      problem.target_prediction[i],
                                                      source_region);
                        }
                    }

                    if (truncated_target_prediction.empty()) {
                        const double first_t =
                                has_explicit_sample_times
                                    ? problem.target_sample_times.front()
                                    : problem.target_prediction.front().t;
                        appendTimedTrackingSample(std::min(first_t, end_t),
                                                  problem.target_prediction.front(),
                                                  problem.visible_regions.empty()
                                                      ? nullptr
                                                      : &problem.visible_regions.front());
                    }

                    if (truncated_target_prediction.empty() ||
                        truncated_viewpoints.size() != truncated_target_prediction.size() ||
                        truncated_target_sample_times.size() != truncated_target_prediction.size()) {
                        return false;
                    }

                    if (end_t > truncated_target_sample_times.back() + 1.0e-4) {
                        const auto horizon_state =
                                interpolateTargetPrediction(problem.target_prediction, end_t);
                        appendTimedTrackingSample(end_t, horizon_state, nullptr);
                    }

                    if (truncated_target_prediction.empty() ||
                        truncated_viewpoints.size() != truncated_target_prediction.size() ||
                        truncated_target_sample_times.size() != truncated_target_prediction.size()) {
                        return false;
                    }

                    problem.guide_path = prefix;
                    problem.guide_t = prefix_t;
                    problem.target_prediction = std::move(truncated_target_prediction);
                    problem.target_sample_times = std::move(truncated_target_sample_times);
                    problem.viewpoints = std::move(truncated_viewpoints);
                    if (!problem.visible_regions.empty()) {
                        problem.visible_regions = std::move(truncated_visible_regions);
                    } else {
                        problem.visible_regions.clear();
                    }
                    refreshTrackingGuideEndpoint(problem);
                    return true;
                };

        const double match_tol = std::max({0.05,
                                           1.5 * std::max(1.0e-3, cfg_.resolution),
                                           0.25 * std::max(0.2, cfg_.corridor_line_max_length)});
        std::string last_prefix_reason;
        for (int view_id = static_cast<int>(problem.viewpoints.size()) - 1; view_id >= 0; --view_id) {
            const Vec3f &viewpoint = problem.viewpoints[static_cast<std::size_t>(view_id)];
            int end_id = -1;
            for (int guide_id = static_cast<int>(candidate_guide.size()) - 1; guide_id >= 1; --guide_id) {
                if ((candidate_guide[static_cast<std::size_t>(guide_id)] - viewpoint).norm() <= match_tol) {
                    end_id = guide_id;
                    break;
                }
            }
            if (end_id < 1) {
                last_prefix_reason = fmt::format("viewpoint_not_found_in_candidate(view_id={}, viewpoint=[{:.3f},{:.3f},{:.3f}])",
                                                 view_id, viewpoint.x(), viewpoint.y(), viewpoint.z());
                continue;
            }
            if (!trackingGuidePointSafe(candidate_guide[static_cast<std::size_t>(end_id)])) {
                const auto &p = candidate_guide[static_cast<std::size_t>(end_id)];
                last_prefix_reason = fmt::format("truncate_endpoint_unsafe(view_id={}, end_id={}, p=[{:.3f},{:.3f},{:.3f}])",
                                                 view_id, end_id, p.x(), p.y(), p.z());
                continue;
            }

            vec_Vec3f prefix(candidate_guide.begin(), candidate_guide.begin() + end_id + 1);
            std::vector<double> prefix_t(candidate_guide_t.begin(), candidate_guide_t.begin() + end_id + 1);
            std::string prefix_reason;
            if (!tryGenerateTrackingCorridor(prefix, sfcs, &prefix_reason)) {
                last_prefix_reason = fmt::format("truncate_prefix_sfc_failed(view_id={}, end_id={}, reason={})",
                                                 view_id, end_id, prefix_reason);
                continue;
            }

            const std::size_t keep_count = static_cast<std::size_t>(view_id + 1);
            applyViewpointTruncation(prefix, prefix_t, keep_count);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking guide truncated to safe local SFC endpoint, kept {} target samples.",
                               keep_count);
            }
            return true;
        }

        std::string last_time_prefix_reason;
        for (int end_id = static_cast<int>(candidate_guide.size()) - 1; end_id >= 1; --end_id) {
            vec_Vec3f prefix(candidate_guide.begin(), candidate_guide.begin() + end_id + 1);
            std::vector<double> prefix_t(candidate_guide_t.begin(), candidate_guide_t.begin() + end_id + 1);
            std::string prefix_reason;
            if (!tryGenerateTrackingCorridor(prefix, sfcs, &prefix_reason)) {
                last_time_prefix_reason =
                        fmt::format("time_prefix_sfc_failed(end_id={}, reason={})",
                                    end_id,
                                    prefix_reason);
                continue;
            }
            if (!applyTimeAlignedTruncation(prefix, prefix_t)) {
                last_time_prefix_reason =
                        fmt::format("time_prefix_apply_failed(end_id={}, end_t={:.3f})",
                                    end_id,
                                    prefix_t.empty() ? 0.0 : prefix_t.back());
                continue;
            }
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking guide truncated by timed prefix fallback, kept {} target samples.",
                               problem.target_prediction.size());
            }
            return true;
        }

        if (!last_prefix_reason.empty() && !last_time_prefix_reason.empty()) {
            setFailureReason(failure_reason,
                             last_prefix_reason + "; " + last_time_prefix_reason);
        } else {
            setFailureReason(failure_reason,
                             !last_prefix_reason.empty()
                                 ? last_prefix_reason
                                 : (!last_time_prefix_reason.empty()
                                        ? last_time_prefix_reason
                                        : "truncate_no_safe_prefix_found"));
        }
        return false;
    }

    bool GeneralPlanner::buildTrackingGuideCorridor(traj_opt::TrackingProblem &problem,
                                                    std::string *failure_reason) {
        problem.sfcs.clear();
        problem.use_corridor = false;

        if (cg_ptr_ != nullptr && !problem.guide_path.empty()) {
            double guide_length = 0.0;
            for (int i = 1; i < static_cast<int>(problem.guide_path.size()); ++i) {
                guide_length += (problem.guide_path[static_cast<std::size_t>(i)] -
                                 problem.guide_path[static_cast<std::size_t>(i - 1)]).norm();
            }
            if (guide_length < 1.0e-4) {
                const Vec3f hover_point = problem.guide_path.front();
                if (!trackingGuidePointSafe(hover_point)) {
                    setFailureReason(failure_reason,
                                     fmt::format("hover_guide_point_unsafe(p=[{:.3f},{:.3f},{:.3f}])",
                                                 hover_point.x(), hover_point.y(), hover_point.z()));
                    return false;
                }
                Polytope hover_sfc;
                if (!cg_ptr_->GeneratePolytopeFromPoint(hover_point, hover_sfc)) {
                    setFailureReason(failure_reason,
                                     fmt::format("hover_GeneratePolytopeFromPoint_failed(p=[{:.3f},{:.3f},{:.3f}])",
                                                 hover_point.x(), hover_point.y(), hover_point.z()));
                    return false;
                }
                problem.guide_path.clear();
                problem.guide_path.emplace_back(hover_point);
                if (problem.guide_t.empty()) {
                    const double hover_t = !problem.target_prediction.empty()
                                               ? problem.target_prediction.back().t
                                               : std::max(0.6, problem.min_total_duration);
                    problem.guide_t.emplace_back(std::max(0.6, hover_t));
                } else {
                    problem.guide_t.resize(1);
                    problem.guide_t.front() = std::max(0.6, problem.guide_t.front());
                }
                problem.sfcs.emplace_back(hover_sfc);
                problem.use_corridor = true;
                refreshTrackingGuideEndpoint(problem);
                return true;
            }
        }

        PolytopeVec sfcs;
        vec_Vec3f dense_guide;
        std::vector<double> dense_guide_t;
        std::string dense_reason;
        const bool dense_ok =
                densifyTrackingGuideForCorridor(problem.guide_path, problem.guide_t, dense_guide, dense_guide_t);
        if (!dense_ok) {
            dense_reason = fmt::format("densify_original_guide_failed(guide_size={}, guide_t_size={})",
                                       problem.guide_path.size(), problem.guide_t.size());
        } else if (!tryGenerateTrackingCorridor(dense_guide, sfcs, &dense_reason)) {
            dense_reason = "original_dense_sfc_failed:" + dense_reason;
        } else {
            problem.guide_path = std::move(dense_guide);
            problem.guide_t = std::move(dense_guide_t);
            refreshTrackingGuideEndpoint(problem);
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            return true;
        }

        std::string truncate_original_reason;
        if (!problem.guide_path.empty() &&
            truncateTrackingProblemForCorridor(problem,
                                               problem.guide_path,
                                               problem.guide_t,
                                               sfcs,
                                               &truncate_original_reason)) {
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking SFC fallback accepted truncated original guide prefix.");
            }
            return true;
        }

        vec_Vec3f astar_repaired;
        std::vector<double> astar_repaired_t;
        std::string astar_reason;
        const bool full_astar_repair =
                repairTrackingGuideWithAstar(problem.guide_path,
                                             problem.guide_t,
                                             astar_repaired,
                                             astar_repaired_t,
                                             &astar_reason);
        vec_Vec3f dense_astar_repaired;
        std::vector<double> dense_astar_repaired_t;
        std::string dense_astar_reason;
        std::string truncate_repaired_reason;
        if (full_astar_repair) {
            if (truncateTrackingProblemForCorridor(problem,
                                                  astar_repaired,
                                                  astar_repaired_t,
                                                  sfcs,
                                                  &truncate_repaired_reason)) {
                problem.sfcs = std::move(sfcs);
                problem.use_corridor = true;
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [GeneralPlanner] Tracking SFC fallback accepted truncated A* repaired guide prefix.");
                }
                return true;
            }
            const bool dense_astar_ok =
                    densifyTrackingGuideForCorridor(astar_repaired,
                                                    astar_repaired_t,
                                                    dense_astar_repaired,
                                                    dense_astar_repaired_t);
            if (!dense_astar_ok) {
                dense_astar_reason = fmt::format("densify_astar_repair_failed(path_size={}, time_size={})",
                                                 astar_repaired.size(), astar_repaired_t.size());
            } else if (!tryGenerateTrackingCorridor(dense_astar_repaired, sfcs, &dense_astar_reason)) {
                dense_astar_reason = "astar_dense_sfc_failed:" + dense_astar_reason;
            } else {
            problem.guide_path = std::move(dense_astar_repaired);
            problem.guide_t = std::move(dense_astar_repaired_t);
            refreshTrackingGuideEndpoint(problem);
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking SFC built after A* guide repair.");
            }
            return true;
            }
        }

        std::string truncate_astar_reason;
        if (!dense_astar_repaired.empty() &&
            truncateTrackingProblemForCorridor(problem,
                                               dense_astar_repaired,
                                               dense_astar_repaired_t,
                                               sfcs,
                                               &truncate_astar_reason)) {
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            return true;
        }
        std::string truncate_dense_reason;
        if (!dense_guide.empty() &&
            truncateTrackingProblemForCorridor(problem,
                                               dense_guide,
                                               dense_guide_t,
                                               sfcs,
                                               &truncate_dense_reason)) {
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            return true;
        }

        problem.sfcs.clear();
        problem.use_corridor = false;
        setFailureReason(failure_reason,
                         fmt::format("guide_size={}, target_samples={}, dense={}, truncate_original={}, astar={}, truncate_repaired={}, dense_astar={}, truncate_astar={}, truncate_dense={}",
                                     problem.guide_path.size(),
                                     problem.target_prediction.size(),
                                     dense_reason.empty() ? "ok-but-unused" : dense_reason,
                                     truncate_original_reason.empty() ? "not_attempted_or_failed" : truncate_original_reason,
                                     full_astar_repair ? "ok" : astar_reason,
                                     truncate_repaired_reason.empty() ? "not_attempted_or_failed" : truncate_repaired_reason,
                                     dense_astar_reason.empty() ? "not_attempted_or_ok" : dense_astar_reason,
                                     truncate_astar_reason.empty() ? "not_attempted" : truncate_astar_reason,
                                     truncate_dense_reason.empty() ? "not_attempted" : truncate_dense_reason));
        return false;
    }

    bool GeneralPlanner::applyTrackingNarrowPassageSoftDistance(
            traj_opt::TrackingProblem &problem,
            std::string *reason) const {
        if (!cfg_.tracking_narrow_passage_enable ||
            problem.guide_path.empty() ||
            map_manager_ == nullptr ||
            !map_manager_->ready() ||
            !map_manager_->hasESDF()) {
            return false;
        }

        double min_clearance = std::numeric_limits<double>::infinity();
        for (const Vec3f &p : problem.guide_path) {
            double dist = 0.0;
            Vec3f grad = Vec3f::Zero();
            if (p.allFinite() && map_manager_->evaluateESDF(p, dist, grad)) {
                min_clearance = std::min(min_clearance, dist);
            }
        }
        if (!std::isfinite(min_clearance) ||
            min_clearance >= cfg_.tracking_narrow_passage_clearance_threshold) {
            return false;
        }

        const double old_safe_distance = problem.safe_distance;
        const double hard_distance = trackingHardSafeDistance(cfg_);
        const double scaled_soft =
                cfg_.tracking_safe_distance *
                std::clamp(cfg_.tracking_narrow_passage_soft_safe_distance_scale, 0.1, 1.0);
        problem.safe_distance = std::max(hard_distance, scaled_soft);
        setFailureReason(reason,
                         fmt::format("narrow_passage min_clearance={:.3f}, optimizer_safe_distance {:.3f}->{:.3f}, hard_safe_distance={:.3f}",
                                     min_clearance,
                                     old_safe_distance,
                                     problem.safe_distance,
                                     hard_distance));
        return true;
    }

} // namespace general_planner
