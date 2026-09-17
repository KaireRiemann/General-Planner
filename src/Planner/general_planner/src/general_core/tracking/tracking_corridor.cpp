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

        double max_step = 0.8 * cfg_.tracking_corridor_line_max_length;
        if (!std::isfinite(max_step) || max_step <= 1.0e-3) {
            max_step = map_manager_ != nullptr ? 4.0 * std::max(0.05, map_manager_->getResolution()) : 0.5;
        }
        max_step = std::clamp(max_step, 0.2, std::max(0.2, cfg_.tracking_corridor_line_max_length));

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
        last_tracking_frontend_prediction_.clear();
        last_tracking_frontend_viewpoints_.clear();
        if (problem.target_prediction.empty() || problem.viewpoints.empty() ||
            problem.target_sample_times.size() != problem.viewpoints.size()) return;
        for (const double t : problem.target_sample_times)
            last_tracking_frontend_prediction_.emplace_back(
                interpolateTargetPrediction(problem.target_prediction, t));
        last_tracking_frontend_viewpoints_ = problem.viewpoints;
    }

    bool GeneralPlanner::tryGenerateTrackingCorridor(const vec_Vec3f &guide_path,
                                                     PolytopeVec &sfcs,
                                                     std::string *failure_reason) {
        sfcs.clear();
        if (tracking_cg_ptr_ == nullptr) {
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
            ok = tracking_cg_ptr_->SearchPolytopeOnPath(guide_path, sfcs, shifted_start_pt, false);
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

    bool GeneralPlanner::buildTrackingGuideCorridor(traj_opt::TrackingProblem &problem,
                                                    std::string *reason) {
        problem.sfcs.clear(); problem.use_corridor = false;
        if (!tracking_cg_ptr_ || problem.guide_path.size() < 2) {
            setFailureReason(reason, "missing corridor generator or guide"); return false;
        }
        double length=0.0;
        for (std::size_t i=1;i<problem.guide_path.size();++i)
            length+=(problem.guide_path[i]-problem.guide_path[i-1]).norm();
        if (length<1.e-4) {
            Polytope poly;
            if (!trackingGuidePointSafe(problem.head_pvaj.col(0)) ||
                !tracking_cg_ptr_->GeneratePolytopeFromPoint(problem.head_pvaj.col(0),poly)) {
                setFailureReason(reason,"hover corridor unavailable"); return false;
            }
            problem.sfcs.push_back(poly);
        } else {
            vec_Vec3f dense; std::vector<double> times;
            if (!densifyTrackingGuideForCorridor(problem.guide_path,problem.guide_t,dense,times)) {
                setFailureReason(reason,"guide contains an unsafe segment"); return false;
            }
            if (!tryGenerateTrackingCorridor(dense,problem.sfcs,reason)) return false;
            problem.guide_path=std::move(dense); problem.guide_t=std::move(times);
        }
        // Corridor construction may propose a shifted seed. Never shift the
        // execution boundary or rewrite the target/guide time axes to accept it.
        if (!problem.sfcs.front().PointIsInside(problem.head_pvaj.col(0),1.e-3) ||
            !problem.sfcs.back().PointIsInside(problem.tail_pvaj.col(0),1.e-3)) {
            setFailureReason(reason,"corridor does not cover execution boundaries"); return false;
        }
        for (std::size_t i=1;i<problem.sfcs.size();++i) {
            const auto a=problem.sfcs[i-1].GetPlanes(), b=problem.sfcs[i].GetPlanes();
            Eigen::Matrix<double,Eigen::Dynamic,4> overlap(a.rows()+b.rows(),4);
            overlap << a,b;
            Vec3f point;
            const double depth=geometry_utils::findInteriorDist(overlap,point);
            if (!std::isfinite(depth) || depth<=1.e-4) {
                setFailureReason(reason,"disconnected corridor sequence"); return false;
            }
        }
        problem.use_corridor=true;
        return true;
    }

} // namespace general_planner
