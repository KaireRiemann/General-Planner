#include <map_manager/global_region_grid.hpp>

#include <algorithm>
#include <cmath>

namespace general_planner {

GlobalRegionGrid::GlobalRegionGrid(const GlobalRegionGridConfig &cfg)
        : cfg_(cfg) {
    cfg_.region_size_xy = std::max(0.5, cfg_.region_size_xy);
    cfg_.region_size_z = std::max(0.5, cfg_.region_size_z);
    cfg_.min_frontier_count = std::max(1, cfg_.min_frontier_count);
    cfg_.explored_ratio_threshold = std::clamp(cfg_.explored_ratio_threshold, 0.0, 1.0);
}

void GlobalRegionGrid::updateFromGlobalMap(const GlobalExplorationMap &global_map,
                                           const std::vector<FrontierCluster> &clusters) {
    if (!cfg_.enable) {
        return;
    }

    std::vector<Eigen::Vector3d> free_points;
    std::vector<Eigen::Vector3d> occupied_points;
    global_map.getObservedFreePoints(free_points);
    global_map.getOccupiedPoints(occupied_points);

    std::unordered_map<int, ExplorationRegion> old_regions;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        old_regions = regions_;
    }

    std::unordered_map<int, ExplorationRegion> next_regions;
    next_regions.reserve(free_points.size() / 8 + occupied_points.size() / 8 + clusters.size() + 8);

    const auto touchRegion = [&](const super_utils::Vec3f &p) -> ExplorationRegion & {
        const int id = positionToRegionId(p);
        ExplorationRegion &region = next_regions[id];
        if (region.id < 0) {
            const auto old_it = old_regions.find(id);
            if (old_it != old_regions.end()) {
                region.last_visit_time = old_it->second.last_visit_time;
            }
            region.id = id;
            initializeRegionGeometry(p, region);
        }
        return region;
    };

    for (const auto &p : free_points) {
        ExplorationRegion &region = touchRegion(p);
        ++region.observed_free_count;
    }
    for (const auto &p : occupied_points) {
        ExplorationRegion &region = touchRegion(p);
        ++region.occupied_count;
    }
    for (const auto &cluster : clusters) {
        if (!cluster.valid || cluster.raw_size <= 0) {
            continue;
        }
        ExplorationRegion &region = touchRegion(cluster.center);
        region.frontier_count += cluster.raw_size;
        region.frontier_cluster_ids.push_back(cluster.id);
    }

    const double region_volume = cfg_.region_size_xy * cfg_.region_size_xy * cfg_.region_size_z;
    const double voxel_volume = std::max(1.0e-6,
                                         global_map.resolution() *
                                         global_map.resolution() *
                                         global_map.resolution());
    const int estimated_voxels = std::max(1, static_cast<int>(std::round(region_volume / voxel_volume)));

    for (auto &kv : next_regions) {
        ExplorationRegion &region = kv.second;
        const int observed = region.observed_free_count + region.occupied_count;
        region.unknown_count_estimate = std::max(0, estimated_voxels - observed);
        const double observed_ratio =
                static_cast<double>(observed) / static_cast<double>(estimated_voxels);
        region.information_gain_estimate = static_cast<double>(region.frontier_count);
        region.active = region.frontier_count >= cfg_.min_frontier_count;
        region.explored = region.frontier_count < cfg_.min_frontier_count ||
                          observed_ratio >= cfg_.explored_ratio_threshold;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    regions_ = std::move(next_regions);
}

void GlobalRegionGrid::getActiveRegions(std::vector<ExplorationRegion> &regions) const {
    regions.clear();
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &kv : regions_) {
        if (kv.second.active && !kv.second.explored) {
            regions.push_back(kv.second);
        }
    }
    std::sort(regions.begin(), regions.end(),
              [](const ExplorationRegion &lhs, const ExplorationRegion &rhs) {
                  if (lhs.frontier_count != rhs.frontier_count) {
                      return lhs.frontier_count > rhs.frontier_count;
                  }
                  return lhs.id < rhs.id;
              });
}

bool GlobalRegionGrid::getRegion(const int id, ExplorationRegion &region) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = regions_.find(id);
    if (it == regions_.end()) {
        return false;
    }
    region = it->second;
    return true;
}

int GlobalRegionGrid::positionToRegionId(const super_utils::Vec3f &p) const {
    const int ix = static_cast<int>(std::floor(p.x() / cfg_.region_size_xy));
    const int iy = static_cast<int>(std::floor(p.y() / cfg_.region_size_xy));
    const int iz = static_cast<int>(std::floor(p.z() / cfg_.region_size_z));
    const int hash = ix * 73856093 ^ iy * 19349663 ^ iz * 83492791;
    return hash & 0x7fffffff;
}

void GlobalRegionGrid::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    regions_.clear();
}

void GlobalRegionGrid::initializeRegionGeometry(const super_utils::Vec3f &p,
                                                ExplorationRegion &region) const {
    const int ix = static_cast<int>(std::floor(p.x() / cfg_.region_size_xy));
    const int iy = static_cast<int>(std::floor(p.y() / cfg_.region_size_xy));
    const int iz = static_cast<int>(std::floor(p.z() / cfg_.region_size_z));
    region.bbox_min = super_utils::Vec3f(static_cast<double>(ix) * cfg_.region_size_xy,
                                         static_cast<double>(iy) * cfg_.region_size_xy,
                                         static_cast<double>(iz) * cfg_.region_size_z);
    region.bbox_max = region.bbox_min +
                      super_utils::Vec3f(cfg_.region_size_xy, cfg_.region_size_xy, cfg_.region_size_z);
    region.center = 0.5 * (region.bbox_min + region.bbox_max);
}

}  // namespace general_planner
