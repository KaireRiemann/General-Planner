#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <map_manager/global_exploration_map.hpp>
#include <map_manager/global_pointcloud_map.hpp>
#include <map_manager/global_region_grid.hpp>
#include <map_manager/map_backend.hpp>
#include <map_manager/pointcloud_map.hpp>
#include <rog_map/rog_map.h>
#include <rog_map_ros/rog_map_ros1.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <super_utils/type_utils.hpp>

namespace fast_planner
{
class LIOInterface;
}

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

    void setPointCloudMap(const PointCloudMap::Ptr &map)
    {
        pointcloud_map_ = map;
    }

    bool ready() const
    {
        return hasBackend(MapBackend::ROG);
    }

    bool ready(const MapBackend backend) const
    {
        return hasBackend(backend);
    }

    bool hasBackend(const MapBackend backend) const
    {
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                return hasPointCloudBackend();
            case MapBackend::EPIC_LIO:
                return hasEpicLioMap();
            case MapBackend::LOCAL_EDT:
                return hasESDF();
            case MapBackend::HYBRID:
                return map_ != nullptr;
            case MapBackend::ROG:
            default:
                return map_ != nullptr;
        }
    }

    const rog_map::ROGMap *rawMap() const
    {
        return map_.get();
    }

    rog_map::ROGMapROS::Ptr rawRosMap() const
    {
        return map_;
    }

    PointCloudMap::Ptr rawPointCloudMap() const
    {
        return pointcloud_map_;
    }

    bool hasPointCloudMap() const
    {
        return pointcloud_map_ != nullptr;
    }

    bool hasPointCloudBackend() const
    {
        return pointcloud_map_ != nullptr;
    }

    void setEpicLioMap(const std::shared_ptr<fast_planner::LIOInterface> &lio);
    bool hasEpicLioMap() const;
    std::shared_ptr<fast_planner::LIOInterface> epicLio() const;
    void initEpicLioMap(ros::NodeHandle &nh);
    void updateEpicLioMap(const rog_map::PointCloud &cloud,
                          const super_utils::Pose &pose,
                          CloudFrame frame,
                          const rog_map::RobotState &robot);
    void publishEpicLioMap(const ros::Time &stamp = ros::Time::now()) const;
    double getEpicDisToOcc(const Eigen::Vector3f &pt) const;
    void epicKNN(const PointCloudMap::PointType &pt,
                 int k,
                 PointCloudMap::PointVector &pts,
                 std::vector<float> &sqr_distances) const;
    void epicKNN(const Eigen::Vector3f &pt,
                 int k,
                 PointCloudMap::PointVector &pts,
                 std::vector<float> &sqr_distances) const;
    void epicBoxSearch(const Eigen::Vector3f &box_min,
                       const Eigen::Vector3f &box_max,
                       PointCloudMap::PointVector &pts) const;
    bool epicIsInBox(const Eigen::Vector3f &pt) const;
    bool epicIsInMap(const Eigen::Vector3f &pt) const;

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
            const bool use_global_pointcloud_backend =
                    pointcloud_map_ == nullptr ||
                    pointcloud_map_.get() == global_pointcloud_map_.get();
            global_pointcloud_map_ = std::make_shared<GlobalPointCloudMap>(cfg);
            if (use_global_pointcloud_backend) {
                pointcloud_map_ = global_pointcloud_map_;
            }
        } else {
            if (pointcloud_map_.get() == global_pointcloud_map_.get()) {
                pointcloud_map_.reset();
            }
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
                             const double stamp,
                             const bool update_rog_map = true)
    {
        if (update_rog_map && map_ != nullptr) {
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

    bool isStateValid(const rog_map::Vec3f &pos,
                      const MapBackend backend = MapBackend::ROG) const
    {
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                return isPointCloudStateValid(pos);
            case MapBackend::EPIC_LIO:
                return isEpicLioStateValid(pos);
            case MapBackend::LOCAL_EDT:
                return hasESDF() && getESDFDistance(pos) > 0.0;
            case MapBackend::HYBRID:
                if (!isRogStateValid(pos)) {
                    return false;
                }
                return !hasPointCloudBackend() || isPointCloudStateValid(pos);
            case MapBackend::ROG:
            default:
                return isRogStateValid(pos);
        }
    }

    bool isStateValid(const rog_map::Vec3f &pos,
                      const MapBackend backend,
                      const bool use_inf_map,
                      const bool unknown_as_occupied) const
    {
        const auto state = getPolicyGridType(pos, backend, use_inf_map);
        if (state == rog_map::GridType::OCCUPIED ||
            state == rog_map::GridType::OUT_OF_MAP) {
            return false;
        }
        return !(unknown_as_occupied && state == rog_map::GridType::UNKNOWN);
    }

    bool isLineFree(const rog_map::Vec3f &start_pt,
                    const rog_map::Vec3f &end_pt,
                    const MapBackend backend) const
    {
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                return isPointCloudLineFree(start_pt, end_pt);
            case MapBackend::EPIC_LIO:
                return isEpicLioLineFree(start_pt, end_pt);
            case MapBackend::LOCAL_EDT:
                return isSegmentSafe(start_pt, end_pt, 0.0, MapBackend::LOCAL_EDT, getResolution());
            case MapBackend::HYBRID:
                if (map_ == nullptr || !map_->isLineFree(start_pt, end_pt, true, false)) {
                    return false;
                }
                return !hasPointCloudBackend() || isPointCloudLineFree(start_pt, end_pt);
            case MapBackend::ROG:
            default:
                return map_ != nullptr && map_->isLineFree(start_pt, end_pt, true, false);
        }
    }

    bool isLineFree(const rog_map::Vec3f &start_pt,
                    const rog_map::Vec3f &end_pt,
                    const MapBackend backend,
                    const double &max_dis,
                    const rog_map::vec_E<rog_map::Vec3i> &neighbor_list) const
    {
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                return isPointCloudLineFree(start_pt, end_pt, max_dis);
            case MapBackend::EPIC_LIO:
                return isEpicLioLineFree(start_pt, end_pt, max_dis);
            case MapBackend::LOCAL_EDT:
                return isSegmentSafe(start_pt, end_pt, max_dis, MapBackend::LOCAL_EDT, getResolution());
            case MapBackend::HYBRID:
                if (map_ == nullptr || !map_->isLineFree(start_pt, end_pt, max_dis, neighbor_list)) {
                    return false;
                }
                return !hasPointCloudBackend() || isPointCloudLineFree(start_pt, end_pt, max_dis);
            case MapBackend::ROG:
            default:
                return map_ != nullptr && map_->isLineFree(start_pt, end_pt, max_dis, neighbor_list);
        }
    }

    bool isLineFree(const rog_map::Vec3f &start_pt,
                    const rog_map::Vec3f &end_pt,
                    const MapBackend backend,
                    const bool &use_inf_map,
                    const bool &use_unk_as_occ) const
    {
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                return isPointCloudLineFree(start_pt, end_pt);
            case MapBackend::EPIC_LIO:
                return isEpicLioLineFree(start_pt, end_pt);
            case MapBackend::LOCAL_EDT:
                return isSegmentSafe(start_pt, end_pt, 0.0, MapBackend::LOCAL_EDT, getResolution());
            case MapBackend::HYBRID:
                if (map_ == nullptr ||
                    !map_->isLineFree(start_pt, end_pt, use_inf_map, use_unk_as_occ)) {
                    return false;
                }
                return !hasPointCloudBackend() || isPointCloudLineFree(start_pt, end_pt);
            case MapBackend::ROG:
            default:
                return map_ != nullptr && map_->isLineFree(start_pt, end_pt, use_inf_map, use_unk_as_occ);
        }
    }

    void getObstaclePointsInBox(rog_map::Vec3f &box_min,
                                rog_map::Vec3f &box_max,
                                const MapBackend backend,
                                rog_map::vec_E<rog_map::Vec3f> &out_points) const
    {
        out_points.clear();
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                getPointCloudObstaclePointsInBox(box_min, box_max, out_points);
                return;
            case MapBackend::EPIC_LIO:
                getEpicLioObstaclePointsInBox(box_min, box_max, out_points);
                return;
            case MapBackend::LOCAL_EDT:
                if (map_ != nullptr) {
                    map_->boundBoxByLocalMap(box_min, box_max);
                    map_->boxSearch(box_min, box_max, rog_map::GridType::OCCUPIED, out_points);
                }
                return;
            case MapBackend::HYBRID: {
                if (map_ != nullptr) {
                    map_->boundBoxByLocalMap(box_min, box_max);
                    map_->boxSearch(box_min, box_max, rog_map::GridType::OCCUPIED, out_points);
                }
                if (hasPointCloudBackend()) {
                    rog_map::vec_E<rog_map::Vec3f> pc_points;
                    getPointCloudObstaclePointsInBox(box_min, box_max, pc_points);
                    out_points.insert(out_points.end(), pc_points.begin(), pc_points.end());
                }
                return;
            }
            case MapBackend::ROG:
            default:
                if (map_ != nullptr) {
                    map_->boundBoxByLocalMap(box_min, box_max);
                    map_->boxSearch(box_min, box_max, rog_map::GridType::OCCUPIED, out_points);
                }
                return;
        }
    }

    rog_map::GridType getPolicyGridType(const rog_map::Vec3f &pos,
                                        const MapBackend backend,
                                        const bool use_inf_map) const
    {
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                return getPointCloudGridType(pos);
            case MapBackend::EPIC_LIO:
                return getEpicLioGridType(pos);
            case MapBackend::LOCAL_EDT:
                if (!hasESDF()) {
                    return rog_map::GridType::UNKNOWN;
                }
                return getESDFDistance(pos) > 0.0 ? rog_map::GridType::KNOWN_FREE
                                                  : rog_map::GridType::OCCUPIED;
            case MapBackend::HYBRID: {
                const auto rog_state = getRogGridType(pos, use_inf_map);
                if (rog_state == rog_map::GridType::OCCUPIED ||
                    rog_state == rog_map::GridType::OUT_OF_MAP) {
                    return rog_state;
                }
                if (!hasPointCloudBackend()) {
                    return rog_state;
                }
                const auto pc_state = getPointCloudGridType(pos);
                if (pc_state == rog_map::GridType::OCCUPIED ||
                    pc_state == rog_map::GridType::OUT_OF_MAP) {
                    return pc_state;
                }
                return rog_state;
            }
            case MapBackend::ROG:
            default:
                return getRogGridType(pos, use_inf_map);
        }
    }

    bool findNearestStateValid(const rog_map::Vec3f &start_pos,
                               const MapBackend backend,
                               rog_map::Vec3f &nearest_pt,
                               const double &max_dis,
                               const bool use_inf_map) const
    {
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                return findNearestPointCloudStateValid(start_pos, nearest_pt, max_dis);
            case MapBackend::EPIC_LIO:
                return findNearestEpicLioStateValid(start_pos, nearest_pt, max_dis);
            case MapBackend::LOCAL_EDT:
                return findNearestRogStateValid(start_pos, nearest_pt, max_dis, use_inf_map);
            case MapBackend::HYBRID:
                if (!hasPointCloudBackend()) {
                    return findNearestRogStateValid(start_pos, nearest_pt, max_dis, use_inf_map);
                }
                return findNearestSampledStateValid(start_pos, backend, nearest_pt, max_dis, use_inf_map);
            case MapBackend::ROG:
            default:
                return findNearestRogStateValid(start_pos, nearest_pt, max_dis, use_inf_map);
        }
    }

    bool isInMap(const rog_map::Vec3f &pos,
                 const MapBackend backend) const
    {
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                if (pointcloud_map_ != nullptr) {
                    return pos.allFinite() && pointcloud_map_->isInMap(pos.cast<float>());
                }
                return false;
            case MapBackend::EPIC_LIO:
                return pos.allFinite() && hasEpicLioMap() && epicIsInMap(pos.cast<float>());
            case MapBackend::LOCAL_EDT:
                return map_ != nullptr && map_->insideLocalMap(pos);
            case MapBackend::HYBRID:
                if (map_ == nullptr || !map_->insideLocalMap(pos)) {
                    return false;
                }
                if (hasPointCloudBackend()) {
                    return isInMap(pos.cast<float>());
                }
                return true;
            case MapBackend::ROG:
            default:
                return map_ != nullptr && map_->insideLocalMap(pos);
        }
    }

    double getDisToOcc(const Eigen::Vector3f &pt) const
    {
        if (pointcloud_map_ != nullptr) {
            return pointcloud_map_->getDisToOcc(pt);
        }

        if (hasEpicLioMap()) {
            return getEpicDisToOcc(pt);
        }

        if (map_ == nullptr || !pt.allFinite()) {
            return 0.0;
        }

        const rog_map::Vec3f pos = pt.cast<double>();
        double dist = 0.0;
        rog_map::Vec3f grad = rog_map::Vec3f::Zero();
        if (evaluateESDF(pos, dist, grad)) {
            return dist;
        }

        PointCloudMap::PointVector nearest;
        std::vector<float> sqr_distances;
        KNN(pt, 1, nearest, sqr_distances);
        if (!sqr_distances.empty()) {
            return std::sqrt(static_cast<double>(sqr_distances.front()));
        }

        // Match EPIC's empty-neighbor placeholder scale until a real point-cloud backend is wired.
        return 10.0;
    }

    double getClearance(const rog_map::Vec3f &pos,
                        const MapBackend backend = MapBackend::HYBRID) const
    {
        if (!pos.allFinite()) {
            return 0.0;
        }
        switch (backend) {
            case MapBackend::POINT_CLOUD:
                return pointcloud_map_ != nullptr
                       ? pointcloud_map_->getDisToOcc(pos.cast<float>())
                       : 0.0;
            case MapBackend::EPIC_LIO:
                return hasEpicLioMap() ? getEpicDisToOcc(pos.cast<float>()) : 0.0;
            case MapBackend::LOCAL_EDT:
                return hasESDF() ? getESDFDistance(pos) : 0.0;
            case MapBackend::HYBRID: {
                double clearance = std::numeric_limits<double>::infinity();
                if (hasPointCloudBackend()) {
                    clearance = std::min(clearance, getDisToOcc(pos.cast<float>()));
                }
                if (hasESDF()) {
                    clearance = std::min(clearance, getESDFDistance(pos));
                }
                if (!std::isfinite(clearance)) {
                    clearance = getDisToOcc(pos.cast<float>());
                }
                return clearance;
            }
            case MapBackend::ROG:
            default:
                if (hasESDF()) {
                    return getESDFDistance(pos);
                }
                return isRogStateValid(pos) ? std::numeric_limits<double>::infinity() : 0.0;
        }
    }

    bool isStateSafe(const rog_map::Vec3f &pos,
                     const double safe_distance,
                     const MapBackend backend = MapBackend::HYBRID) const
    {
        if (!pos.allFinite()) {
            return false;
        }
        if (!isInMap(pos, backend)) {
            return false;
        }
        if (!isStateValid(pos, backend, true, false)) {
            return false;
        }
        const double clearance = getClearance(pos, backend);
        return !std::isfinite(clearance) || clearance >= safe_distance;
    }

    bool isSegmentSafe(const rog_map::Vec3f &start_pt,
                       const rog_map::Vec3f &end_pt,
                       const double safe_distance,
                       const MapBackend backend = MapBackend::HYBRID,
                       const double step = 0.20) const
    {
        if (!start_pt.allFinite() || !end_pt.allFinite()) {
            return false;
        }
        const double length = (end_pt - start_pt).norm();
        const int samples = std::max(1, static_cast<int>(
                std::ceil(length / std::max(1.0e-3, step))));
        for (int i = 0; i <= samples; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(samples);
            const rog_map::Vec3f p = start_pt + ratio * (end_pt - start_pt);
            if (!isStateSafe(p, safe_distance, backend)) {
                return false;
            }
        }
        return true;
    }

    void KNN(const PointCloudMap::PointType &pt,
             const int k,
             PointCloudMap::PointVector &pts,
             std::vector<float> &sqr_distances) const
    {
        if (pointcloud_map_ != nullptr) {
            pointcloud_map_->KNN(pt, k, pts, sqr_distances);
            return;
        }

        if (hasEpicLioMap()) {
            epicKNN(pt, k, pts, sqr_distances);
            return;
        }

        pts.clear();
        sqr_distances.clear();
        if (map_ == nullptr || k <= 0) {
            return;
        }

        const Eigen::Vector3f query(pt.x, pt.y, pt.z);
        if (!query.allFinite()) {
            return;
        }

        rog_map::Vec3f box_min = query.cast<double>() -
                                 rog_map::Vec3f::Constant(10.0);
        rog_map::Vec3f box_max = query.cast<double>() +
                                 rog_map::Vec3f::Constant(10.0);
        boundBoxByLocalMap(box_min, box_max);

        rog_map::vec_E<rog_map::Vec3f> occupied_points;
        boxSearch(box_min, box_max, rog_map::GridType::OCCUPIED, occupied_points);

        std::vector<std::pair<float, PointCloudMap::PointType>> candidates;
        candidates.reserve(occupied_points.size());
        for (const auto &p : occupied_points) {
            const Eigen::Vector3f pf = p.cast<float>();
            const float sqr_dist = (pf - query).squaredNorm();
            PointCloudMap::PointType out;
            out.x = pf.x();
            out.y = pf.y();
            out.z = pf.z();
            candidates.emplace_back(sqr_dist, out);
        }

        const auto keep = std::min<std::size_t>(static_cast<std::size_t>(k), candidates.size());
        std::partial_sort(candidates.begin(),
                          candidates.begin() + static_cast<std::ptrdiff_t>(keep),
                          candidates.end(),
                          [](const auto &lhs, const auto &rhs) {
                              return lhs.first < rhs.first;
                          });

        pts.reserve(keep);
        sqr_distances.reserve(keep);
        for (std::size_t i = 0; i < keep; ++i) {
            sqr_distances.emplace_back(candidates[i].first);
            pts.emplace_back(candidates[i].second);
        }
    }

    void KNN(const Eigen::Vector3f &pt,
             const int k,
             PointCloudMap::PointVector &pts,
             std::vector<float> &sqr_distances) const
    {
        PointCloudMap::PointType query;
        query.x = pt.x();
        query.y = pt.y();
        query.z = pt.z();
        KNN(query, k, pts, sqr_distances);
    }

    void boxSearchPointCloud(const Eigen::Vector3f &box_min,
                             const Eigen::Vector3f &box_max,
                             PointCloudMap::PointVector &pts) const
    {
        if (pointcloud_map_ != nullptr) {
            pointcloud_map_->boxSearchPointCloud(box_min, box_max, pts);
            return;
        }

        if (hasEpicLioMap()) {
            epicBoxSearch(box_min, box_max, pts);
            return;
        }

        pts.clear();
        if (map_ == nullptr || !box_min.allFinite() || !box_max.allFinite()) {
            return;
        }

        rog_map::Vec3f min_d = box_min.cast<double>();
        rog_map::Vec3f max_d = box_max.cast<double>();
        boundBoxByLocalMap(min_d, max_d);

        rog_map::vec_E<rog_map::Vec3f> occupied_points;
        boxSearch(min_d, max_d, rog_map::GridType::OCCUPIED, occupied_points);
        pts.reserve(occupied_points.size());
        for (const auto &p : occupied_points) {
            const Eigen::Vector3f pf = p.cast<float>();
            PointCloudMap::PointType out;
            out.x = pf.x();
            out.y = pf.y();
            out.z = pf.z();
            pts.emplace_back(out);
        }
    }

    bool isInBox(const Eigen::Vector3f &pt) const
    {
        if (pointcloud_map_ != nullptr) {
            return pointcloud_map_->isInBox(pt);
        }
        if (hasEpicLioMap()) {
            return epicIsInBox(pt);
        }
        // TODO(pointcloud-map): ROGMap has no exploration-box/dead-area policy yet.
        const rog_map::Vec3f pos = pt.cast<double>();
        return map_ != nullptr && pt.allFinite() && map_->insideLocalMap(pos);
    }

    bool isInMap(const Eigen::Vector3f &pt) const
    {
        if (pointcloud_map_ != nullptr) {
            return pointcloud_map_->isInMap(pt);
        }
        if (hasEpicLioMap()) {
            return epicIsInMap(pt);
        }
        const rog_map::Vec3f pos = pt.cast<double>();
        return map_ != nullptr && pt.allFinite() && map_->insideLocalMap(pos);
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
    rog_map::GridType getRogGridType(const rog_map::Vec3f &pos,
                                     const bool use_inf_map) const
    {
        if (map_ == nullptr) {
            return rog_map::GridType::OUT_OF_MAP;
        }
        return use_inf_map ? map_->getInfGridType(pos) : map_->getGridType(pos);
    }

    bool isRogStateValid(const rog_map::Vec3f &pos) const
    {
        return map_ != nullptr && !map_->isOccupiedInflate(pos);
    }

    rog_map::GridType getPointCloudGridType(const rog_map::Vec3f &pos) const
    {
        if (pointcloud_map_ == nullptr || !pos.allFinite()) {
            return rog_map::GridType::OUT_OF_MAP;
        }
        const Eigen::Vector3f pos_f = pos.cast<float>();
        if (!pointcloud_map_->isInMap(pos_f) || !pointcloud_map_->isInBox(pos_f)) {
            return rog_map::GridType::OUT_OF_MAP;
        }
        const double dist = pointcloud_map_->getDisToOcc(pos_f);
        if (!std::isfinite(dist) || dist <= 1.0e-3) {
            return rog_map::GridType::OCCUPIED;
        }
        return rog_map::GridType::KNOWN_FREE;
    }

    rog_map::GridType getEpicLioGridType(const rog_map::Vec3f &pos) const
    {
        if (!hasEpicLioMap() || !pos.allFinite()) {
            return rog_map::GridType::OUT_OF_MAP;
        }
        const Eigen::Vector3f pos_f = pos.cast<float>();
        if (!epicIsInMap(pos_f) || !epicIsInBox(pos_f)) {
            return rog_map::GridType::OUT_OF_MAP;
        }
        const double dist = getEpicDisToOcc(pos_f);
        if (!std::isfinite(dist) || dist <= 1.0e-3) {
            return rog_map::GridType::OCCUPIED;
        }
        return rog_map::GridType::KNOWN_FREE;
    }

    bool isPointCloudStateValid(const rog_map::Vec3f &pos) const
    {
        return getPointCloudGridType(pos) == rog_map::GridType::KNOWN_FREE;
    }

    bool isEpicLioStateValid(const rog_map::Vec3f &pos) const
    {
        return getEpicLioGridType(pos) == rog_map::GridType::KNOWN_FREE;
    }

    bool isPointCloudLineFree(const rog_map::Vec3f &start_pt,
                              const rog_map::Vec3f &end_pt,
                              const double max_dis = -1.0) const
    {
        if (pointcloud_map_ == nullptr ||
            !start_pt.allFinite() ||
            !end_pt.allFinite()) {
            return false;
        }
        const rog_map::Vec3f delta = end_pt - start_pt;
        const double length = delta.norm();
        if (max_dis > 0.0 && length > max_dis) {
            return false;
        }
        const double step = map_ != nullptr ? std::max(0.05, map_->getResolution()) : 0.10;
        const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
        for (int i = 0; i <= samples; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(samples);
            if (!isPointCloudStateValid(start_pt + ratio * delta)) {
                return false;
            }
        }
        return true;
    }

    bool isEpicLioLineFree(const rog_map::Vec3f &start_pt,
                           const rog_map::Vec3f &end_pt,
                           const double max_dis = -1.0) const
    {
        if (!hasEpicLioMap() ||
            !start_pt.allFinite() ||
            !end_pt.allFinite()) {
            return false;
        }
        const rog_map::Vec3f delta = end_pt - start_pt;
        const double length = delta.norm();
        if (max_dis > 0.0 && length > max_dis) {
            return false;
        }
        const double step = map_ != nullptr ? std::max(0.05, map_->getResolution()) : 0.10;
        const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
        for (int i = 0; i <= samples; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(samples);
            if (!isEpicLioStateValid(start_pt + ratio * delta)) {
                return false;
            }
        }
        return true;
    }

    void getPointCloudObstaclePointsInBox(const rog_map::Vec3f &box_min,
                                          const rog_map::Vec3f &box_max,
                                          rog_map::vec_E<rog_map::Vec3f> &out_points) const
    {
        out_points.clear();
        if (pointcloud_map_ == nullptr ||
            !box_min.allFinite() ||
            !box_max.allFinite()) {
            return;
        }
        PointCloudMap::PointVector pc_points;
        boxSearchPointCloud(box_min.cast<float>(), box_max.cast<float>(), pc_points);
        out_points.reserve(pc_points.size());
        for (const auto &p : pc_points) {
            out_points.emplace_back(p.x, p.y, p.z);
        }
    }

    void getEpicLioObstaclePointsInBox(const rog_map::Vec3f &box_min,
                                       const rog_map::Vec3f &box_max,
                                       rog_map::vec_E<rog_map::Vec3f> &out_points) const
    {
        out_points.clear();
        if (!hasEpicLioMap() ||
            !box_min.allFinite() ||
            !box_max.allFinite()) {
            return;
        }
        PointCloudMap::PointVector lio_points;
        epicBoxSearch(box_min.cast<float>(), box_max.cast<float>(), lio_points);
        out_points.reserve(lio_points.size());
        for (const auto &p : lio_points) {
            out_points.emplace_back(p.x, p.y, p.z);
        }
    }

    bool findNearestRogStateValid(const rog_map::Vec3f &start_pos,
                                  rog_map::Vec3f &nearest_pt,
                                  const double &max_dis,
                                  const bool use_inf_map) const
    {
        if (map_ == nullptr) {
            return false;
        }
        if (use_inf_map) {
            return map_->getNearestInfCellNot(rog_map::GridType::OCCUPIED,
                                              start_pos,
                                              nearest_pt,
                                              max_dis);
        }
        return map_->getNearestCellNot(rog_map::GridType::OCCUPIED,
                                       start_pos,
                                       nearest_pt,
                                       max_dis);
    }

    bool findNearestPointCloudStateValid(const rog_map::Vec3f &start_pos,
                                         rog_map::Vec3f &nearest_pt,
                                         const double &max_dis) const
    {
        return findNearestSampledStateValid(start_pos,
                                            MapBackend::POINT_CLOUD,
                                            nearest_pt,
                                            max_dis,
                                            true);
    }

    bool findNearestEpicLioStateValid(const rog_map::Vec3f &start_pos,
                                      rog_map::Vec3f &nearest_pt,
                                      const double &max_dis) const
    {
        return findNearestSampledStateValid(start_pos,
                                            MapBackend::EPIC_LIO,
                                            nearest_pt,
                                            max_dis,
                                            true);
    }

    bool findNearestSampledStateValid(const rog_map::Vec3f &start_pos,
                                      const MapBackend backend,
                                      rog_map::Vec3f &nearest_pt,
                                      const double &max_dis,
                                      const bool use_inf_map) const
    {
        if (!start_pos.allFinite() || max_dis < 0.0) {
            return false;
        }
        if (isStateValid(start_pos, backend, use_inf_map, false)) {
            nearest_pt = start_pos;
            return true;
        }

        const double step = map_ != nullptr ? std::max(0.05, map_->getResolution()) : 0.10;
        const int max_step = static_cast<int>(std::ceil(max_dis / step));
        double best_sq = std::numeric_limits<double>::infinity();
        bool found = false;
        for (int r = 1; r <= max_step; ++r) {
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    for (int dz = -r; dz <= r; ++dz) {
                        if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
                            continue;
                        }
                        const rog_map::Vec3f candidate =
                                start_pos + step * rog_map::Vec3f(dx, dy, dz);
                        const double sq = (candidate - start_pos).squaredNorm();
                        if (sq > max_dis * max_dis || sq >= best_sq) {
                            continue;
                        }
                        if (isStateValid(candidate, backend, use_inf_map, false)) {
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

    rog_map::ROGMapROS::Ptr map_;
    PointCloudMap::Ptr pointcloud_map_;
    std::shared_ptr<fast_planner::LIOInterface> epic_lio_;
    mutable std::mutex epic_lio_mutex_;
    mutable ros::Publisher epic_lio_map_pub_;
    mutable ros::Publisher epic_lio_legacy_map_pub_;
    bool epic_lio_publish_map_{true};
    double epic_lio_publish_map_period_{0.5};
    double epic_lio_self_filter_radius_{0.0};
    mutable ros::Time last_epic_lio_map_publish_stamp_;
    std::unique_ptr<GlobalExplorationMap> global_exploration_map_;
    GlobalPointCloudMap::Ptr global_pointcloud_map_;
    std::unique_ptr<GlobalRegionGrid> global_region_grid_;
    std::mutex global_update_mutex_;
    double last_global_update_stamp_{-1.0};
    double global_update_min_interval_{0.0};
    int global_update_cloud_downsample_step_{1};
    int global_update_max_points_per_update_{0};
};
} // namespace general_planner
