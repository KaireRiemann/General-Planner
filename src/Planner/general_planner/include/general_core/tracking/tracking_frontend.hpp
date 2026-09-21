#pragma once
#include <chrono>
#include <memory>
#include <map_manager/map_manager.hpp>
#include <traj_opt/tracking_problem.hpp>

namespace general_planner {
// Elastic-Tracker sequential visible-ring search on inflated occupancy
// (unknown-as-free). Corridor generation still uses GP CIRI, but the seed
// path is Elastic's sparse way_pts / pts2path reconstruction.
class TrackingFrontend {
public:
    using Ptr = std::shared_ptr<TrackingFrontend>;
    struct Config {
        double nominal_horizon{3.0};
        double sample_dt{0.2};
        double search_budget_seconds{0.08};
        double tracking_distance{3.0};
        double distance_tolerance{0.3};
        double height_offset{1.0};
        double height_tolerance{0.3};
        double visibility_angle_clearance{0.4};
        double max_speed{5.0};
        bool unknown_as_occupied{false};
        bool use_visible_region{true};
    };
    TrackingFrontend(const Config &cfg, const MapManager::Ptr &map_manager);
    bool buildProblem(const general_utils::StatePVAJ &head,
                      const traj_opt::DynamicTargetStates &prediction,
                      traj_opt::TrackingProblem &problem) const;
private:
    using Vec = general_utils::Vec3f;
    using Path = general_utils::vec_Vec3f;
    using Deadline = std::chrono::steady_clock::time_point;
    bool safe(const Vec &point) const;
    bool lineFree(const Vec &a, const Vec &b) const;
    bool visible(const Vec &point, const Vec &center) const;
    bool search(const Vec &start, const Vec &goal, bool ring,
                Path &path, const Deadline &deadline) const;
    bool visibleRegion(const Vec &center, Vec &seed,
                       traj_opt::TrackingVisibleRegion &region,
                       const Deadline &deadline) const;
    void pts2path(const Path &way_pts, Path &path, const Deadline &deadline) const;
    Config cfg_;
    MapManager::Ptr map_manager_;
};
} // namespace general_planner
