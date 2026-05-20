#pragma once

#include <mutex>

#include <map_manager/map_manager.hpp>

namespace general_planner {
namespace exploration {

class EpicMapAdapter {
public:
    using Ptr = std::shared_ptr<EpicMapAdapter>;
    using PointType = PointCloudMap::PointType;
    using PointVector = PointCloudMap::PointVector;
    using PointCloud = pcl::PointCloud<PointType>;

    explicit EpicMapAdapter(MapManager::Ptr map_manager);

    void updateCloudOdom(const rog_map::PointCloud &cloud,
                         const super_utils::Pose &pose,
                         CloudFrame frame,
                         const super_utils::Vec3f &sensor_position,
                         const super_utils::Vec3f &velocity = super_utils::Vec3f::Zero());

    const PointCloud &lidarCloud() const;
    Eigen::Vector3f lidarPose() const;
    Eigen::Quaternionf lidarQ() const;
    Eigen::Vector3f lidarVel() const;

    double getDisToOcc(const Eigen::Vector3f &pt) const;
    double getDisToOcc(const super_utils::Vec3f &pt) const;
    double getDisToOcc(const PointType &pt) const;

    void KNN(const PointType &pt,
             int k,
             PointVector &pts,
             std::vector<float> &sqr_distances) const;
    void KNN(const Eigen::Vector3f &pt,
             int k,
             PointVector &pts,
             std::vector<float> &sqr_distances) const;

    void boxSearch(const Eigen::Vector3f &box_min,
                   const Eigen::Vector3f &box_max,
                   PointVector &pts) const;

    bool isInBox(const Eigen::Vector3f &pt) const;
    bool isInBox(const PointType &pt) const;
    bool isInMap(const Eigen::Vector3f &pt) const;
    bool isInMap(const PointType &pt) const;

private:
    static Eigen::Vector3f transformPointToWorld(const rog_map::PclPoint &point,
                                                 const super_utils::Pose &pose,
                                                 CloudFrame frame);

private:
    MapManager::Ptr map_manager_;
    mutable std::mutex mutex_;
    PointCloud lidar_cloud_;
    Eigen::Vector3f lidar_pose_{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf lidar_q_{Eigen::Quaternionf::Identity()};
    Eigen::Vector3f lidar_vel_{Eigen::Vector3f::Zero()};
};

}  // namespace exploration
}  // namespace general_planner
