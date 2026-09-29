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
#include <general_core/tracking/tracking_corridor_builder.hpp>

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
        double floor, ceiling;
        tracking_map_manager_->getInflatedVirtualHeightBounds(floor, ceiling);
        return tracking_map_manager_->insideLocalMap(point) &&
               point.z() >= floor && point.z() <= ceiling &&
               !trackingInflatedOccupied(tracking_map_manager_, point, cfg_.tracking_unknown_as_occupied);
    }

    bool GeneralPlanner::generateElasticTrackingSFC(const vec_Vec3f &path,
                                                    PolytopeVec &sfcs,
                                                    std::string *failure_reason) {
        if (tracking_cg_ptr_ == nullptr) {
            setFailureReason(failure_reason, "corridor_generator_null");
            return false;
        }
        TrackingCorridorConfig config;
        config.max_seed_length = std::min(std::max(0.5, cfg_.tracking_corridor_bound_dis),
                                         cfg_.tracking_corridor_line_max_length);
        config.min_seed_length = tracking_map_manager_
            ? 0.5 * tracking_map_manager_->getInfResolution() : 0.075;
        config.budget_seconds = cfg_.tracking_frontend_budget;
        TrackingCorridorOps ops;
        ops.point_safe = [&](const Vec3f &p) { return trackingGuidePointSafe(p); };
        ops.line_free = [&](const Vec3f &a, const Vec3f &b) {
            return trackingSeedLineFree(tracking_map_manager_, a, b, cfg_.tracking_unknown_as_occupied);
        };
        ops.point_poly = [&](const Vec3f &p, Polytope &poly) {
            return tracking_cg_ptr_->GeneratePolytopeFromPoint(p, poly);
        };
        ops.line_poly = [&](const Vec3f &a, const Vec3f &b, Polytope &poly) {
            Line line{a, b};
            return tracking_cg_ptr_->GeneratePolytopeFromLine(line, poly);
        };
        TrackingCorridorReport report;
        const bool ok = buildTrackingCorridor(path, config, ops, sfcs, report);
        if (!ok) {
            setFailureReason(failure_reason, report.reason + trackingCorridorGeometry(report, sfcs));
            if (cfg_.visualization_en) {
                ros_ptr_->vizFrontendPath(report.path.empty() ? path : report.path);
                ros_ptr_->vizExpSfc(sfcs);
                if (!report.candidate.empty()) ros_ptr_->vizCiriPolytope(report.candidate, "tracking_failed_seed");
                if (!report.bridge.empty()) ros_ptr_->vizCiriPolytope(report.bridge, "tracking_failed_bridge");
            }
        }
        return ok;
    }

    bool GeneralPlanner::buildTrackingGuideCorridor(traj_opt::TrackingProblem &problem,
                                                    std::string *reason) {
        problem.sfcs.clear();
        problem.use_corridor = false;
        if (!tracking_cg_ptr_ || problem.guide_path.size() < 2) {
            setFailureReason(reason, "missing corridor generator or guide");
            return false;
        }
        if (!problem.head_pvaj.allFinite() || !problem.tail_pvaj.allFinite() ||
            (problem.guide_path.front() - problem.head_pvaj.col(0)).norm() > 1.e-6 ||
            (problem.guide_path.back() - problem.tail_pvaj.col(0)).norm() > 1.e-6) {
            setFailureReason(reason, "tracking guide does not match execution endpoints");
            return false;
        }
        if (!generateElasticTrackingSFC(problem.guide_path, problem.sfcs, reason)) {
            return false;
        }
        // PointIsInside's positive margin permits a point outside a plane;
        // it is NOT an interior-clearance requirement. Keep only roundoff.
        if (!problem.sfcs.front().PointIsInside(problem.head_pvaj.col(0), 1.e-6) ||
            !problem.sfcs.back().PointIsInside(problem.tail_pvaj.col(0), 1.e-6)) {
            setFailureReason(reason, "tracking corridor endpoint verification failed");
            return false;
        }
        problem.use_corridor = true;
        return true;
    }

} // namespace general_planner
