#include <general_core/tracking/tracking_corridor_builder.hpp>
#include <utils/geometry/geometry_utils.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace general_planner {
namespace {
using general_utils::Vec3f;
using geometry_utils::Polytope;
constexpr double containment_epsilon = 1.e-6;
constexpr double overlap_epsilon = 1.e-4;

bool contains(const Polytope &poly, const Vec3f &point) {
    const auto planes = poly.GetPlanes();
    return !poly.empty() && planes.rows() >= 4 && planes.allFinite() &&
           poly.PointIsInside(point, containment_epsilon);
}

double overlapDepth(const Polytope &a, const Polytope &b) {
    const auto ap = a.GetPlanes(), bp = b.GetPlanes();
    if (a.empty() || b.empty() || !ap.allFinite() || !bp.allFinite()) return -1.0;
    Eigen::Matrix<double, Eigen::Dynamic, 4> planes(ap.rows() + bp.rows(), 4);
    planes << ap, bp;
    Vec3f interior;
    const double depth = geometry_utils::findInteriorDist(planes, interior);
    return std::isfinite(depth) ? depth : -1.0;
}
} // namespace

bool buildTrackingCorridor(const general_utils::vec_Vec3f &path,
                           const TrackingCorridorConfig &config,
                           const TrackingCorridorOps &ops,
                           geometry_utils::PolytopeVec &sfcs,
                           TrackingCorridorReport &report) {
    sfcs.clear();
    report = {};
    const auto fail = [&](const std::string &reason) {
        report.reason = reason;
        return false;
    };
    if (path.size() < 2 || !ops.point_safe || !ops.line_free ||
        !ops.point_poly || !ops.line_poly || !std::isfinite(config.max_seed_length) ||
        config.max_seed_length <= 0.0 || !std::isfinite(config.min_seed_length) ||
        config.min_seed_length <= 0.0 || !std::isfinite(config.budget_seconds) ||
        config.budget_seconds <= 0.0 || config.max_points < 2 || config.max_attempts == 0)
        return fail("invalid tracking corridor input");
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(config.budget_seconds));
    const auto exhausted = [&]() {
        return report.attempts >= config.max_attempts ||
               std::chrono::steady_clock::now() >= deadline;
    };
    // Sparse ring guides commonly contain a single catch-up edge > 2 m.
    // Subdivide it instead of advancing past it without a covering corridor.
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!path[i].allFinite() || !ops.point_safe(path[i]))
            return fail("tracking guide point outside corridor domain or occupied");
        if (i == 0) {
            report.path.push_back(path[i]);
            continue;
        }
        if (!ops.line_free(path[i - 1], path[i]))
            return fail("tracking guide contains a blocked edge");
        const double length = (path[i] - path[i - 1]).norm();
        if (length < 1.e-8) continue;
        const double parts_d = std::ceil(length / config.max_seed_length);
        if (!std::isfinite(parts_d) || parts_d > config.max_points ||
            report.path.size() + static_cast<std::size_t>(parts_d) > config.max_points)
            return fail("tracking corridor point limit");
        const std::size_t parts = std::max<std::size_t>(1, static_cast<std::size_t>(parts_d));
        for (std::size_t k = 1; k <= parts; ++k) {
            const Vec3f point = path[i - 1] + (path[i] - path[i - 1]) *
                (static_cast<double>(k) / parts);
            if (!ops.point_safe(point)) return fail("tracking subdivided seed occupied");
            report.path.push_back(point);
        }
        if (exhausted()) return fail("tracking corridor budget exhausted");
    }
    if (report.path.size() == 1) {
        ++report.attempts;
        Polytope poly;
        if (!ops.point_poly(report.path.front(), poly) || !contains(poly, report.path.front()))
            return fail("hover corridor unavailable or does not contain seed");
        sfcs.push_back(poly);
        report.spans.push_back({0, 0, false});
        return true;
    }
    std::size_t begin = 0;
    while (begin + 1 < report.path.size()) {
        std::size_t end = begin + 1;
        while (end + 1 < report.path.size() &&
               (report.path[end + 1] - report.path[begin]).norm() <= config.max_seed_length &&
               ops.line_free(report.path[begin], report.path[end + 1])) ++end;
        while (true) {
            report.failed_begin = begin;
            report.failed_end = end;
            if (exhausted()) return fail("tracking corridor budget exhausted");
            ++report.attempts;
            Polytope poly, fill;
            bool accepted = ops.line_poly(report.path[begin], report.path[end], poly);
            for (std::size_t k = begin; accepted && k <= end; ++k)
                accepted = contains(poly, report.path[k]);
            report.candidate = poly;
            bool need_bridge = false;
            if (accepted && !sfcs.empty()) {
                report.overlap_depth = overlapDepth(sfcs.back(), poly);
                if (report.overlap_depth <= overlap_epsilon) {
                    // begin is the real shared path vertex, regardless of how
                    // many path points previous polytopes have covered.
                    if (exhausted()) return fail("tracking corridor budget exhausted");
                    ++report.attempts;
                    need_bridge = ops.point_poly(report.path[begin], fill) &&
                                  contains(fill, report.path[begin]);
                    report.bridge = fill;
                    accepted = need_bridge &&
                               overlapDepth(sfcs.back(), fill) > overlap_epsilon &&
                               overlapDepth(fill, poly) > overlap_epsilon;
                }
            }
            if (accepted) {
                if (need_bridge) {
                    sfcs.push_back(fill);
                    report.spans.push_back({begin, begin, true});
                }
                sfcs.push_back(poly);
                report.spans.push_back({begin, end, false});
                begin = end;
                break;
            }
            // A shortcut can exclude an intermediate bend or fail to overlap.
            // Retry a shorter seed, always retaining the actual join vertex.
            if (end > begin + 1) {
                end = begin + std::max<std::size_t>(1, (end - begin) / 2);
                continue;
            }
            const double length = (report.path[end] - report.path[begin]).norm();
            if (length <= config.min_seed_length || report.path.size() >= config.max_points)
                return fail("tracking_corridor_infeasible: uncovered seed or disconnected corridor");
            const Vec3f middle = 0.5 * (report.path[begin] + report.path[end]);
            if (!ops.point_safe(middle)) return fail("tracking midpoint occupied");
            report.path.insert(report.path.begin() + static_cast<std::ptrdiff_t>(end), middle);
        }
    }
    // No skipped edges and no permissive 5 cm plane residual: every seed,
    // including the execution tail, has been checked inside its polytope.
    return !sfcs.empty();
}

std::string trackingCorridorGeometry(const TrackingCorridorReport &report,
                                     const geometry_utils::PolytopeVec &sfcs) {
    std::ostringstream out;
    out << std::setprecision(9) << ";seed_begin=" << report.failed_begin
        << ";seed_end=" << report.failed_end << ";ciri_attempts=" << report.attempts
        << ";overlap_depth=" << report.overlap_depth << ";corridor_path=[";
    for (const auto &point : report.path) out << '(' << point.transpose() << ')';
    out << "];corridor_spans=[";
    for (const auto &span : report.spans)
        out << '(' << span.begin << ',' << span.end << ',' << span.bridge << ')';
    const auto planes = [&](const Polytope &poly) {
        out << '[';
        if (!poly.empty()) {
            const auto p = poly.GetPlanes();
            for (int i = 0; i < p.rows(); ++i) out << '(' << p.row(i) << ')';
        }
        out << ']';
    };
    out << "];corridor_planes=[";
    for (const auto &poly : sfcs) planes(poly);
    out << "];candidate_planes="; planes(report.candidate);
    out << ";bridge_planes="; planes(report.bridge);
    return out.str();
}
} // namespace general_planner
