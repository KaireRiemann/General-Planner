#include <map_manager/global_pointcloud_map.hpp>

#include <algorithm>
#include <cmath>

#include <pcl/io/pcd_io.h>

namespace general_planner {

GlobalPointCloudMap::GlobalPointCloudMap(const GlobalPointCloudMapConfig &cfg)
        : cfg_(cfg),
          cloud_(new pcl::PointCloud<pcl::PointXYZI>) {
    cfg_.voxel_size = std::max(1.0e-3, cfg_.voxel_size);
}

void GlobalPointCloudMap::insertCloud(const rog_map::PointCloud &cloud,
                                      const super_utils::Pose &pose,
                                      const CloudFrame frame) {
    if (!cfg_.enable || cloud.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &p : cloud) {
        const Eigen::Vector3d world = transformPointToWorld(p, pose, frame);
        if (!world.allFinite() || !insideCrop(world)) {
            continue;
        }
        const Eigen::Vector3i key = posToKey(world);
        if (inserted_keys_.find(key) != inserted_keys_.end()) {
            continue;
        }
        inserted_keys_.insert(key);
        pcl::PointXYZI out;
        out.x = static_cast<float>(world.x());
        out.y = static_cast<float>(world.y());
        out.z = static_cast<float>(world.z());
        out.intensity = p.intensity;
        cloud_->push_back(out);
    }
}

bool GlobalPointCloudMap::getCloud(rog_map::PointCloud &out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    out.clear();
    if (!cloud_ || cloud_->empty()) {
        return false;
    }
    out.reserve(cloud_->size());
    for (const auto &p : cloud_->points) {
        rog_map::PclPoint q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = p.intensity;
        out.push_back(q);
    }
    return true;
}

bool GlobalPointCloudMap::savePCD(const std::string &path) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cloud_ || cloud_->empty()) {
        return false;
    }
    const std::string out_path = path.empty() ? cfg_.save_path : path;
    return pcl::io::savePCDFileBinary(out_path, *cloud_) == 0;
}

int GlobalPointCloudMap::pointCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cloud_ ? static_cast<int>(cloud_->size()) : 0;
}

void GlobalPointCloudMap::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_->clear();
    inserted_keys_.clear();
}

Eigen::Vector3i GlobalPointCloudMap::posToKey(const Eigen::Vector3d &p) const {
    return Eigen::Vector3i(static_cast<int>(std::floor(p.x() / cfg_.voxel_size)),
                           static_cast<int>(std::floor(p.y() / cfg_.voxel_size)),
                           static_cast<int>(std::floor(p.z() / cfg_.voxel_size)));
}

Eigen::Vector3d GlobalPointCloudMap::transformPointToWorld(const rog_map::PclPoint &p,
                                                           const super_utils::Pose &pose,
                                                           const CloudFrame frame) const {
    const Eigen::Vector3d local(p.x, p.y, p.z);
    if (frame == CloudFrame::WORLD) {
        return local;
    }
    return pose.first + pose.second * local;
}

bool GlobalPointCloudMap::insideCrop(const Eigen::Vector3d &p) const {
    if (!cfg_.crop_enable) {
        return true;
    }
    return (p - cfg_.crop_min).minCoeff() >= -1.0e-9 &&
           (cfg_.crop_max - p).minCoeff() >= -1.0e-9;
}

}  // namespace general_planner
