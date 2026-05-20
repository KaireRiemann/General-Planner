#include "exploration/epic_map_adapter.hpp"

#include <algorithm>

namespace general_planner {
namespace exploration {

EpicMapAdapter::EpicMapAdapter(MapManager::Ptr map_manager)
        : map_manager_(std::move(map_manager)) {}

void EpicMapAdapter::updateCloudOdom(const rog_map::PointCloud &cloud,
                                     const super_utils::Pose &pose,
                                     const CloudFrame frame,
                                     const super_utils::Vec3f &sensor_position,
                                     const super_utils::Vec3f &velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    lidar_cloud_.clear();
    lidar_cloud_.points.reserve(cloud.points.size());
    for (const auto &point : cloud.points) {
        const Eigen::Vector3f world = transformPointToWorld(point, pose, frame);
        if (!world.allFinite()) {
            continue;
        }
        PointType out;
        out.x = world.x();
        out.y = world.y();
        out.z = world.z();
        lidar_cloud_.points.emplace_back(out);
    }
    lidar_cloud_.width = static_cast<uint32_t>(lidar_cloud_.points.size());
    lidar_cloud_.height = 1;
    lidar_cloud_.is_dense = cloud.is_dense;
    lidar_pose_ = sensor_position.cast<float>();
    lidar_q_ = pose.second.cast<float>();
    lidar_vel_ = velocity.cast<float>();
}

const EpicMapAdapter::PointCloud &EpicMapAdapter::lidarCloud() const {
    return lidar_cloud_;
}

Eigen::Vector3f EpicMapAdapter::lidarPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lidar_pose_;
}

Eigen::Quaternionf EpicMapAdapter::lidarQ() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lidar_q_;
}

Eigen::Vector3f EpicMapAdapter::lidarVel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lidar_vel_;
}

double EpicMapAdapter::getDisToOcc(const Eigen::Vector3f &pt) const {
    if (map_manager_ == nullptr) {
        return 0.0;
    }
    return map_manager_->getDisToOcc(pt);
}

double EpicMapAdapter::getDisToOcc(const super_utils::Vec3f &pt) const {
    const Eigen::Vector3f pt_f = pt.cast<float>();
    return getDisToOcc(pt_f);
}

double EpicMapAdapter::getDisToOcc(const PointType &pt) const {
    return getDisToOcc(Eigen::Vector3f(pt.x, pt.y, pt.z));
}

void EpicMapAdapter::KNN(const PointType &pt,
                         const int k,
                         PointVector &pts,
                         std::vector<float> &sqr_distances) const {
    pts.clear();
    sqr_distances.clear();
    if (map_manager_ == nullptr) {
        return;
    }
    map_manager_->KNN(pt, k, pts, sqr_distances);
}

void EpicMapAdapter::KNN(const Eigen::Vector3f &pt,
                         const int k,
                         PointVector &pts,
                         std::vector<float> &sqr_distances) const {
    pts.clear();
    sqr_distances.clear();
    if (map_manager_ == nullptr) {
        return;
    }
    map_manager_->KNN(pt, k, pts, sqr_distances);
}

void EpicMapAdapter::boxSearch(const Eigen::Vector3f &box_min,
                               const Eigen::Vector3f &box_max,
                               PointVector &pts) const {
    pts.clear();
    if (map_manager_ == nullptr) {
        return;
    }
    map_manager_->boxSearchPointCloud(box_min, box_max, pts);
}

bool EpicMapAdapter::isInBox(const Eigen::Vector3f &pt) const {
    return map_manager_ != nullptr && map_manager_->isInBox(pt);
}

bool EpicMapAdapter::isInBox(const PointType &pt) const {
    return isInBox(Eigen::Vector3f(pt.x, pt.y, pt.z));
}

bool EpicMapAdapter::isInMap(const Eigen::Vector3f &pt) const {
    return map_manager_ != nullptr && map_manager_->isInMap(pt);
}

bool EpicMapAdapter::isInMap(const PointType &pt) const {
    return isInMap(Eigen::Vector3f(pt.x, pt.y, pt.z));
}

Eigen::Vector3f EpicMapAdapter::transformPointToWorld(const rog_map::PclPoint &point,
                                                      const super_utils::Pose &pose,
                                                      const CloudFrame frame) {
    const super_utils::Vec3f local(point.x, point.y, point.z);
    if (frame == CloudFrame::WORLD) {
        return local.cast<float>();
    }
    return (pose.first + pose.second * local).cast<float>();
}

}  // namespace exploration
}  // namespace general_planner
