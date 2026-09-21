#pragma once
#include <algorithm>
#include <cmath>
#include <mutex>
#include <map_manager/map_manager.hpp>

namespace general_planner {
// Query-time mask used only by tracking occupancy queries. The shared ROG is
// unchanged, so exploration / state2state / gate keep the original map.
class TrackingOccupancyExclusion {
public:
    void setCylinder(const general_utils::Vec3f &center, double xy_radius, double z_radius) {
        std::lock_guard<std::mutex> lock(mutex_);
        valid_ = center.allFinite() && std::isfinite(xy_radius) && xy_radius > 0.0 &&
                 std::isfinite(z_radius) && z_radius > 0.0;
        center_ = center;
        xy_radius_sq_ = xy_radius * xy_radius;
        z_radius_ = z_radius;
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        valid_ = false;
    }
    bool contains(const general_utils::Vec3f &p) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_ || !p.allFinite()) return false;
        const general_utils::Vec3f d = p - center_;
        return d.head<2>().squaredNorm() <= xy_radius_sq_ && std::abs(d.z()) <= z_radius_;
    }
    bool valid() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return valid_;
    }
private:
    mutable std::mutex mutex_;
    bool valid_{false};
    general_utils::Vec3f center_{general_utils::Vec3f::Zero()};
    double xy_radius_sq_{0.0};
    double z_radius_{0.0};
};

inline TrackingOccupancyExclusion &trackingOccupancyExclusion() {
    static TrackingOccupancyExclusion mask;
    return mask;
}

inline void setTrackingOccupancyExclusion(const general_utils::Vec3f &center,
                                          double xy_radius, double z_radius) {
    if (center.allFinite() && std::isfinite(xy_radius) && xy_radius > 0.0 &&
        std::isfinite(z_radius) && z_radius > 0.0) {
        trackingOccupancyExclusion().setCylinder(center, xy_radius, z_radius);
        return;
    }
    trackingOccupancyExclusion().clear();
}

// Elastic OccGridMap::isOccupied: in-map inflated occupied only. Unknown and
// out-of-map cells are free unless tracking explicitly treats unknown as occupied.
inline bool trackingInflatedOccupied(const MapManager::Ptr &map,
                                     const general_utils::Vec3f &p,
                                     bool unknown_as_occupied = false) {
    if (!p.allFinite()) return true;
    if (trackingOccupancyExclusion().contains(p)) return false;
    if (!map || !map->ready()) return false;
    if (!map->insideLocalMap(p)) return unknown_as_occupied;
    if (map->getInfGridType(p) == rog_map::OCCUPIED) return true;
    if (unknown_as_occupied && map->getGridType(p) != rog_map::KNOWN_FREE) return true;
    return false;
}

inline void trackingOccupiedVoxels(const MapManager::Ptr &map,
                                  const general_utils::Vec3f &lower,
                                  const general_utils::Vec3f &upper,
                                  general_utils::vec_Vec3f &occupied) {
    map->boxSearchInflate(lower,upper,rog_map::OCCUPIED,occupied);
    double floor, ceiling;
    map->getInflatedVirtualHeightBounds(floor,ceiling);
    occupied.erase(std::remove_if(occupied.begin(),occupied.end(),
        [&](const auto &p) {
            return p.z() < floor || p.z() > ceiling || trackingOccupancyExclusion().contains(p);
        }),occupied.end());
}

// Elastic env::checkRayValid: sample the inflated occupancy grid. No extra
// circumscribed-sphere inflation on top of ROG's inflation_step.
inline bool trackingSeedLineFree(const MapManager::Ptr &map,
                                 const general_utils::Vec3f &a,
                                 const general_utils::Vec3f &b,
                                 bool unknown_as_occupied = false,
                                 double max_dist = 0.0) {
    if (!a.allFinite() || !b.allFinite()) return false;
    const general_utils::Vec3f delta = b - a;
    const double length = delta.norm();
    if (max_dist > 0.0 && length > max_dist) return false;
    if (!map || !map->ready()) return true;
    const double step = std::max(0.05, 0.5 * map->getInfResolution());
    const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int i = 0; i <= samples; ++i) {
        const general_utils::Vec3f p = a + delta * (static_cast<double>(i) / samples);
        if (trackingInflatedOccupied(map, p, unknown_as_occupied)) return false;
    }
    return true;
}

inline bool trackingVisibilityRayFree(const MapManager::Ptr &map,
                                      const general_utils::Vec3f &a,
                                      const general_utils::Vec3f &b,
                                      bool unknown_as_occupied = false) {
    return trackingSeedLineFree(map, a, b, unknown_as_occupied);
}
} // namespace general_planner
