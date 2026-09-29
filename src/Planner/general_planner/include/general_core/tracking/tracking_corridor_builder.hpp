#pragma once

#include <data_structure/base/polytope.h>
#include <functional>
#include <string>
#include <vector>

namespace general_planner {

// Tracking-only path assembly. The callbacks use the existing CIRI backend;
// other tasks continue to use CorridorGenerator::SearchPolytopeOnPath.
struct TrackingCorridorOps {
    std::function<bool(const general_utils::Vec3f &)> point_safe;
    std::function<bool(const general_utils::Vec3f &, const general_utils::Vec3f &)> line_free;
    std::function<bool(const general_utils::Vec3f &, geometry_utils::Polytope &)> point_poly;
    std::function<bool(const general_utils::Vec3f &, const general_utils::Vec3f &,
                       geometry_utils::Polytope &)> line_poly;
};

struct TrackingCorridorConfig {
    double max_seed_length{2.0};
    double min_seed_length{0.075};
    double budget_seconds{0.2};
    std::size_t max_points{2048};
    std::size_t max_attempts{512};
};

struct TrackingCorridorSpan {
    std::size_t begin{0};
    std::size_t end{0};
    bool bridge{false};
};

struct TrackingCorridorReport {
    general_utils::vec_Vec3f path;
    std::vector<TrackingCorridorSpan> spans;
    std::string reason;
    std::size_t failed_begin{0};
    std::size_t failed_end{0};
    std::size_t attempts{0};
    double overlap_depth{0.0};
    geometry_utils::Polytope candidate;
    geometry_utils::Polytope bridge;
};

bool buildTrackingCorridor(const general_utils::vec_Vec3f &path,
                           const TrackingCorridorConfig &config,
                           const TrackingCorridorOps &ops,
                           geometry_utils::PolytopeVec &sfcs,
                           TrackingCorridorReport &report);

// Contains the actual path indices and polytope planes at failure, suitable for
// a diagnostics event. It does not write a file or mutate a map.
std::string trackingCorridorGeometry(const TrackingCorridorReport &report,
                                     const geometry_utils::PolytopeVec &sfcs);
} // namespace general_planner
