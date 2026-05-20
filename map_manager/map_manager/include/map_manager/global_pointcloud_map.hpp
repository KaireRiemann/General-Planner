#pragma once

#include <mutex>
#include <string>
#include <unordered_set>

#include <Eigen/Eigen>
#include <map_manager/global_exploration_map.hpp>
#include <map_manager/pointcloud_map.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rog_map/rog_map.h>
#include <super_utils/type_utils.hpp>

namespace general_planner {

struct GlobalPointCloudMapConfig {
    bool enable{true};
    double voxel_size{0.10};
    std::string frame_id{"world"};
    std::string save_path{"/tmp/explored_global_map.pcd"};

    bool crop_enable{false};
    Eigen::Vector3d crop_min{-50.0, -50.0, -2.0};
    Eigen::Vector3d crop_max{50.0, 50.0, 10.0};
};

class GlobalPointCloudMap : public PointCloudMap {
public:
    using Ptr = std::shared_ptr<GlobalPointCloudMap>;

    explicit GlobalPointCloudMap(const GlobalPointCloudMapConfig &cfg);

    void insertCloud(const rog_map::PointCloud &cloud,
                     const super_utils::Pose &pose,
                     CloudFrame frame);

    bool getCloud(rog_map::PointCloud &out) const;

    bool savePCD(const std::string &path) const;

    int pointCount() const;

    void reset();

    double getDisToOcc(const Eigen::Vector3f &pt) const override;
    void KNN(const PointType &pt,
             int k,
             PointVector &pts,
             std::vector<float> &sqr_distances) const override;
    void boxSearchPointCloud(const Eigen::Vector3f &box_min,
                             const Eigen::Vector3f &box_max,
                             PointVector &pts) const override;
    bool isInBox(const Eigen::Vector3f &pt) const override;
    bool isInMap(const Eigen::Vector3f &pt) const override;

private:
    Eigen::Vector3i posToKey(const Eigen::Vector3d &p) const;
    Eigen::Vector3d transformPointToWorld(const rog_map::PclPoint &p,
                                          const super_utils::Pose &pose,
                                          CloudFrame frame) const;
    bool insideCrop(const Eigen::Vector3d &p) const;

private:
    GlobalPointCloudMapConfig cfg_;
    mutable std::mutex mutex_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    std::unordered_set<Eigen::Vector3i, VoxelKeyHash, VoxelKeyEqual> inserted_keys_;
};

}  // namespace general_planner
