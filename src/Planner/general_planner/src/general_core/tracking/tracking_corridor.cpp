/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_internal_utils.hpp>
#include <general_core/tracking/tracking_map_query.hpp>

#include <algorithm>
#include <cmath>
#include <fmt/format.h>

using namespace general_utils;

namespace general_planner {
    namespace {
        void setFailureReason(std::string *out, const std::string &reason) {
            if (out != nullptr) {
                *out = reason;
            }
        }
    }

    bool GeneralPlanner::trackingGuidePointSafe(const Vec3f &point) const {
        if (!point.allFinite()) {
            return false;
        }
        if (tracking_map_manager_ == nullptr || !tracking_map_manager_->ready()) {
            return true;
        }
        return !trackingInflatedOccupied(tracking_map_manager_, point, cfg_.tracking_unknown_as_occupied);
    }

    bool GeneralPlanner::generateElasticTrackingSFC(const vec_Vec3f &path,
                                                    PolytopeVec &sfcs,
                                                    std::string *failure_reason) {
        sfcs.clear();
        if (tracking_cg_ptr_ == nullptr) {
            setFailureReason(failure_reason, "corridor_generator_null");
            return false;
        }
        if (path.size() < 2) {
            setFailureReason(failure_reason,
                             fmt::format("guide_path_too_short(size={})", path.size()));
            return false;
        }
        const double bbox = std::max(0.5, cfg_.tracking_corridor_bound_dis);
        const int path_len = static_cast<int>(path.size());
        int idx = 0;
        while (idx < path_len - 1) {
            int next = idx;
            while (next + 1 < path_len &&
                   trackingSeedLineFree(tracking_map_manager_, path[idx], path[next + 1],
                                        cfg_.tracking_unknown_as_occupied, bbox)) {
                ++next;
            }
            if (next == idx) {
                Polytope poly;
                if (!tracking_cg_ptr_->GeneratePolytopeFromPoint(path[idx], poly)) {
                    setFailureReason(failure_reason,
                                     fmt::format("GeneratePolytopeFromPoint_failed(index={})", idx));
                    sfcs.clear();
                    return false;
                }
                sfcs.push_back(poly);
                if (idx + 1 >= path_len) {
                    break;
                }
                ++idx;
                continue;
            }
            Line seed{path[idx], path[next]};
            Polytope poly;
            if (!tracking_cg_ptr_->GeneratePolytopeFromLine(seed, poly)) {
                setFailureReason(failure_reason,
                                 fmt::format("GeneratePolytopeFromLine_failed(index={}, next={})",
                                             idx, next));
                sfcs.clear();
                return false;
            }
            sfcs.push_back(poly);
            idx = next;
            while (idx + 1 < path_len && poly.PointIsInside(path[idx + 1], 0.05)) {
                ++idx;
            }
        }
        if (sfcs.empty()) {
            setFailureReason(failure_reason, "elastic_sfc_empty");
            return false;
        }
        for (std::size_t i = 1; i < sfcs.size(); ++i) {
            const auto a = sfcs[i - 1].GetPlanes(), b = sfcs[i].GetPlanes();
            Eigen::Matrix<double, Eigen::Dynamic, 4> overlap(a.rows() + b.rows(), 4);
            overlap << a, b;
            Vec3f interior;
            const double depth = geometry_utils::findInteriorDist(overlap, interior);
            if (!std::isfinite(depth) || depth <= 1.0e-4) {
                Polytope fill;
                const Vec3f seed = path[std::min(path.size() - 1, i)];
                if (!tracking_cg_ptr_->GeneratePolytopeFromPoint(seed, fill) ||
                    !sfcs[i - 1].HaveOverlapWith(fill, 1.0e-4) ||
                    !fill.HaveOverlapWith(sfcs[i], 1.0e-4)) {
                    setFailureReason(failure_reason, "disconnected elastic corridor");
                    sfcs.clear();
                    return false;
                }
                sfcs.insert(sfcs.begin() + static_cast<std::ptrdiff_t>(i), fill);
                ++i;
            }
        }
        return true;
    }

    bool GeneralPlanner::buildTrackingGuideCorridor(traj_opt::TrackingProblem &problem,
                                                    std::string *reason) {
        problem.sfcs.clear();
        problem.use_corridor = false;
        if (!tracking_cg_ptr_ || problem.guide_path.size() < 2) {
            setFailureReason(reason, "missing corridor generator or guide");
            return false;
        }
        double length = 0.0;
        for (std::size_t i = 1; i < problem.guide_path.size(); ++i) {
            length += (problem.guide_path[i] - problem.guide_path[i - 1]).norm();
        }
        if (length < 1.e-4) {
            Polytope poly;
            if (!trackingGuidePointSafe(problem.head_pvaj.col(0)) ||
                !tracking_cg_ptr_->GeneratePolytopeFromPoint(problem.head_pvaj.col(0), poly)) {
                setFailureReason(reason, "hover corridor unavailable");
                return false;
            }
            problem.sfcs.push_back(poly);
        } else if (!generateElasticTrackingSFC(problem.guide_path, problem.sfcs, reason)) {
            return false;
        }
        const double margin = 0.05;
        if (!problem.sfcs.front().PointIsInside(problem.head_pvaj.col(0), margin)) {
            Polytope head;
            if (!tracking_cg_ptr_->GeneratePolytopeFromPoint(problem.head_pvaj.col(0), head) ||
                !head.HaveOverlapWith(problem.sfcs.front(), 1.e-4)) {
                setFailureReason(reason, "corridor does not cover execution head");
                return false;
            }
            problem.sfcs.insert(problem.sfcs.begin(), head);
        }
        if (!problem.sfcs.back().PointIsInside(problem.tail_pvaj.col(0), margin)) {
            Polytope tail;
            if (!tracking_cg_ptr_->GeneratePolytopeFromPoint(problem.tail_pvaj.col(0), tail) ||
                !problem.sfcs.back().HaveOverlapWith(tail, 1.e-4)) {
                setFailureReason(reason, "corridor does not cover execution tail");
                return false;
            }
            problem.sfcs.push_back(tail);
        }
        problem.use_corridor = true;
        return true;
    }

} // namespace general_planner

