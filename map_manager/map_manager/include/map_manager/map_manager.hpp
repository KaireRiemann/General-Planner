#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <map_manager/global_exploration_map.hpp>
#include <map_manager/global_pointcloud_map.hpp>
#include <map_manager/global_region_grid.hpp>
#include <rog_map/rog_map.h>
#include <rog_map_ros/rog_map_ros1.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <super_utils/type_utils.hpp>

namespace general_planner
{
class MapManager
{
public:
    using Ptr = std::shared_ptr<MapManager>;

    MapManager() = default;

    explicit MapManager(const rog_map::ROGMapROS::Ptr &map)
        : map_(map)
    {
    }

    void setMap(const rog_map::ROGMapROS::Ptr &map)
    {
        map_ = map;
    }

    bool ready() const
    {
        return map_ != nullptr;
    }

    const rog_map::ROGMap *rawMap() const
    {
        return map_.get();
    }

    rog_map::ROGMapROS::Ptr rawRosMap() const
    {
        return map_;
    }

    rog_map::Config getMapConfig() const
    {
        return map_->getMapConfig();
    }

    void updateMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) const
    {
        map_->updateMap(cloud, pose);
    }

    void enableGlobalExplorationMap(const GlobalExplorationMapConfig &cfg)
    {
        if (cfg.enable) {
            global_exploration_map_ = std::make_unique<GlobalExplorationMap>(cfg);
            global_update_min_interval_ = std::max(0.0, cfg.update_min_interval);
            global_update_cloud_downsample_step_ = std::max(1, cfg.cloud_downsample_step);
            global_update_max_points_per_update_ = std::max(0, cfg.max_points_per_update);
        } else {
            global_exploration_map_.reset();
            global_update_min_interval_ = 0.0;
            global_update_cloud_downsample_step_ = 1;
            global_update_max_points_per_update_ = 0;
        }
    }

    void enableGlobalPointCloudMap(const GlobalPointCloudMapConfig &cfg)
    {
        if (cfg.enable) {
            global_pointcloud_map_ = std::make_unique<GlobalPointCloudMap>(cfg);
        } else {
            global_pointcloud_map_.reset();
        }
    }

    void enableGlobalRegionGrid(const GlobalRegionGridConfig &cfg)
    {
        if (cfg.enable) {
            global_region_grid_ = std::make_unique<GlobalRegionGrid>(cfg);
        } else {
            global_region_grid_.reset();
        }
    }

    void updateMapWithGlobal(const rog_map::PointCloud &cloud,
                             const super_utils::Pose &pose,
                             const CloudFrame frame,
                             const double stamp)
    {
        if (map_ != nullptr) {
            map_->updateMap(cloud, pose);
        }
        updateGlobalMapsOnly(cloud, pose, frame, stamp);
    }

    void updateGlobalMapsOnly(const rog_map::PointCloud &cloud,
                              const super_utils::Pose &pose,
                              const CloudFrame frame,
                              const double stamp)
    {
        if (global_exploration_map_ == nullptr && global_pointcloud_map_ == nullptr) {
            return;
        }
        if (cloud.empty()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(global_update_mutex_);
            if (global_update_min_interval_ > 0.0 &&
                stamp > 0.0 &&
                last_global_update_stamp_ > 0.0 &&
                stamp - last_global_update_stamp_ < global_update_min_interval_) {
                return;
            }
            if (stamp > 0.0) {
                last_global_update_stamp_ = stamp;
            }
        }

        const rog_map::PointCloud *update_cloud = &cloud;
        rog_map::PointCloud filtered_cloud;
        int step = global_update_cloud_downsample_step_;
        if (global_update_max_points_per_update_ > 0 &&
            static_cast<int>(cloud.size()) > global_update_max_points_per_update_) {
            step = std::max(step,
                            static_cast<int>(std::ceil(
                                    static_cast<double>(cloud.size()) /
                                    static_cast<double>(global_update_max_points_per_update_))));
        }
        if (step > 1) {
            filtered_cloud.points.reserve((cloud.size() + static_cast<std::size_t>(step) - 1U) /
                                          static_cast<std::size_t>(step));
            for (std::size_t i = 0; i < cloud.size(); i += static_cast<std::size_t>(step)) {
                filtered_cloud.points.emplace_back(cloud.points[i]);
            }
            filtered_cloud.width = static_cast<uint32_t>(filtered_cloud.points.size());
            filtered_cloud.height = 1;
            filtered_cloud.is_dense = cloud.is_dense;
            update_cloud = &filtered_cloud;
        }

        if (global_exploration_map_ != nullptr) {
            global_exploration_map_->updateObservation(*update_cloud, pose, frame, stamp);
        }
        if (global_pointcloud_map_ != nullptr) {
            global_pointcloud_map_->insertCloud(*update_cloud, pose, frame);
        }
    }

    bool globalExplorationMapReady() const
    {
        return global_exploration_map_ != nullptr &&
               global_exploration_map_->observedVoxelCount() > 0;
    }

    GlobalVoxelState getGlobalVoxelState(const rog_map::Vec3f &p) const
    {
        if (global_exploration_map_ == nullptr) {
            return GlobalVoxelState::UNKNOWN;
        }
        return global_exploration_map_->getVoxelState(p);
    }

    bool isGloballyObserved(const rog_map::Vec3f &p) const
    {
        return global_exploration_map_ != nullptr &&
               global_exploration_map_->isObserved(p);
    }

    bool isGloballyUnknown(const rog_map::Vec3f &p) const
    {
        return global_exploration_map_ == nullptr ||
               global_exploration_map_->isUnknown(p);
    }

    bool isGlobalFrontier(const rog_map::Vec3f &p) const
    {
        return global_exploration_map_ != nullptr &&
               global_exploration_map_->isFrontier(p);
    }

    void getGlobalFrontierPoints(rog_map::vec_E<rog_map::Vec3f> &points) const
    {
        points.clear();
        if (global_exploration_map_ == nullptr) {
            return;
        }
        std::vector<Eigen::Vector3d> tmp;
        global_exploration_map_->getFrontierPoints(tmp);
        points.reserve(tmp.size());
        for (const auto &p : tmp) {
            points.emplace_back(p.x(), p.y(), p.z());
        }
    }

    void getGlobalFrontierClusters(const double cluster_radius,
                                   const int min_cluster_size,
                                   std::vector<FrontierCluster> &clusters) const
    {
        clusters.clear();
        if (global_exploration_map_ == nullptr) {
            return;
        }
        global_exploration_map_->getFrontierClusters(cluster_radius, min_cluster_size, clusters);
        if (global_region_grid_ != nullptr) {
            for (auto &cluster : clusters) {
                cluster.region_id = global_region_grid_->positionToRegionId(cluster.center);
            }
        }
    }

    void updateGlobalRegions(const std::vector<FrontierCluster> &clusters)
    {
        if (global_region_grid_ != nullptr && global_exploration_map_ != nullptr) {
            global_region_grid_->updateFromGlobalMap(*global_exploration_map_, clusters);
        }
    }

    void getActiveGlobalRegions(std::vector<ExplorationRegion> &regions) const
    {
        regions.clear();
        if (global_region_grid_ != nullptr) {
            global_region_grid_->getActiveRegions(regions);
        }
    }

    int positionToGlobalRegionId(const rog_map::Vec3f &p) const
    {
        return global_region_grid_ != nullptr ? global_region_grid_->positionToRegionId(p) : -1;
    }

    bool saveGlobalPointCloudPCD(const std::string &path) const
    {
        return global_pointcloud_map_ != nullptr &&
               global_pointcloud_map_->savePCD(path);
    }

    int globalPointCloudSize() const
    {
        return global_pointcloud_map_ != nullptr ? global_pointcloud_map_->pointCount() : 0;
    }

    double globalExploredVolume() const
    {
        return global_exploration_map_ != nullptr ? global_exploration_map_->exploredVolume() : 0.0;
    }

    int globalFrontierCount() const
    {
        return global_exploration_map_ != nullptr ? global_exploration_map_->frontierCount() : 0;
    }

    rog_map::RobotState getRobotState() const
    {
        return map_->getRobotState();
    }

    double getResolution() const
    {
        return map_->getResolution();
    }

    double getInfResolution() const
    {
        return map_->getInfResolution();
    }

    bool insideLocalMap(const rog_map::Vec3f &pos) const
    {
        return map_->insideLocalMap(pos);
    }

    bool insideLocalMap(const rog_map::Vec3i &id_g) const
    {
        return map_->insideLocalMap(id_g);
    }

    rog_map::GridType getGridType(const rog_map::Vec3f &pos) const
    {
        return map_->getGridType(pos);
    }

    rog_map::GridType getInfGridType(const rog_map::Vec3f &pos) const
    {
        return map_->getInfGridType(pos);
    }

    bool isOccupiedInflate(const rog_map::Vec3f &pos) const
    {
        return map_->isOccupiedInflate(pos);
    }

    bool isLineFree(const rog_map::Vec3f &start_pt,
                    const rog_map::Vec3f &end_pt,
                    const double &max_dis,
                    const rog_map::vec_E<rog_map::Vec3i> &neighbor_list) const
    {
        return map_->isLineFree(start_pt, end_pt, max_dis, neighbor_list);
    }

    bool isLineFree(const rog_map::Vec3f &start_pt,
                    const rog_map::Vec3f &end_pt,
                    const bool &use_inf_map,
                    const bool &use_unk_as_occ) const
    {
        return map_->isLineFree(start_pt, end_pt, use_inf_map, use_unk_as_occ);
    }

    bool getNearestCellNot(const rog_map::GridType &target_type,
                           const rog_map::Vec3f &start_pos,
                           rog_map::Vec3f &nearest_pt,
                           const double &max_dis) const
    {
        return map_->getNearestCellNot(target_type, start_pos, nearest_pt, max_dis);
    }

    bool getNearestInfCellNot(const rog_map::GridType &target_type,
                              const rog_map::Vec3f &start_pos,
                              rog_map::Vec3f &nearest_pt,
                              const double &max_dis) const
    {
        return map_->getNearestInfCellNot(target_type, start_pos, nearest_pt, max_dis);
    }

    void probMapPosToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const
    {
        map_->probMapPosToGlobalIndex(pos, id_g);
    }

    void probMapGlobalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const
    {
        map_->probMapGlobalIndexToPos(id_g, pos);
    }

    void infMapPosToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const
    {
        map_->infMapPosToGlobalIndex(pos, id_g);
    }

    void infMapGlobalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const
    {
        map_->infMapGlobalIndexToPos(id_g, pos);
    }

    void boundBoxByLocalMap(rog_map::Vec3f &box_min, rog_map::Vec3f &box_max) const
    {
        map_->boundBoxByLocalMap(box_min, box_max);
    }

    void boxSearch(const rog_map::Vec3f &box_min,
                   const rog_map::Vec3f &box_max,
                   const rog_map::GridType &gt,
                   rog_map::vec_E<rog_map::Vec3f> &out_points) const
    {
        map_->boxSearch(box_min, box_max, gt, out_points);
    }

    void boxSearchInflate(const rog_map::Vec3f &box_min,
                          const rog_map::Vec3f &box_max,
                          const rog_map::GridType &gt,
                          rog_map::vec_E<rog_map::Vec3f> &out_points) const
    {
        map_->boxSearchInflate(box_min, box_max, gt, out_points);
    }

    bool hasESDF() const
    {
        return map_ != nullptr && map_->hasESDF();
    }

    bool evaluateESDF(const rog_map::Vec3f &pos,
                      double &dist,
                      rog_map::Vec3f &grad) const
    {
        if (map_ == nullptr) {
            dist = 0.0;
            grad.setZero();
            return false;
        }
        return map_->evaluateESDF(pos, dist, grad);
    }

    double getESDFDistance(const rog_map::Vec3f &pos) const
    {
        return map_ == nullptr ? 0.0 : map_->getESDFDistance(pos);
    }

    bool findNearestESDFSafe(const rog_map::Vec3f &start_pos,
                             const double min_distance,
                             rog_map::Vec3f &nearest_pt,
                             const double max_dis) const
    {
        if (map_ == nullptr || !hasESDF() || min_distance <= 0.0 || max_dis < 0.0) {
            return false;
        }

        auto isSafe = [&](const rog_map::Vec3f &pos, double *dist_out = nullptr) {
            if (!insideLocalMap(pos)) {
                return false;
            }
            const auto grid_type = getGridType(pos);
            const auto inf_grid_type = getInfGridType(pos);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP ||
                inf_grid_type == rog_map::GridType::OCCUPIED ||
                inf_grid_type == rog_map::GridType::OUT_OF_MAP) {
                return false;
            }

            double dist = 0.0;
            rog_map::Vec3f grad = rog_map::Vec3f::Zero();
            if (!evaluateESDF(pos, dist, grad)) {
                return false;
            }
            if (dist_out != nullptr) {
                *dist_out = dist;
            }
            return std::isfinite(dist) && dist >= min_distance;
        };

        if (isSafe(start_pos)) {
            nearest_pt = start_pos;
            return true;
        }

        const double res = std::max(0.05, getResolution());
        const int max_step = static_cast<int>(std::ceil(max_dis / res));
        const int max_z_step = std::max(1, static_cast<int>(std::ceil(0.6 / res)));
        double best_sq = std::numeric_limits<double>::infinity();
        bool found = false;

        for (int r = 1; r <= max_step; ++r) {
            const int z_bound = std::min(r, max_z_step);
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    for (int dz = -z_bound; dz <= z_bound; ++dz) {
                        if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
                            continue;
                        }
                        const rog_map::Vec3f candidate = start_pos + res * rog_map::Vec3f(dx, dy, dz);
                        const double sq = (candidate - start_pos).squaredNorm();
                        if (sq > max_dis * max_dis || sq >= best_sq) {
                            continue;
                        }
                        if (isSafe(candidate)) {
                            nearest_pt = candidate;
                            best_sq = sq;
                            found = true;
                        }
                    }
                }
            }
            if (found) {
                return true;
            }
        }
        return false;
    }

private:
    rog_map::ROGMapROS::Ptr map_;
    std::unique_ptr<GlobalExplorationMap> global_exploration_map_;
    std::unique_ptr<GlobalPointCloudMap> global_pointcloud_map_;
    std::unique_ptr<GlobalRegionGrid> global_region_grid_;
    std::mutex global_update_mutex_;
    double last_global_update_stamp_{-1.0};
    double global_update_min_interval_{0.0};
    int global_update_cloud_downsample_step_{1};
    int global_update_max_points_per_update_{0};
};
} // namespace general_planner
