#include <map_manager/map_manager.hpp>

#include <lidar_map/lidar_map.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace general_planner
{

void MapManager::setEpicLioMap(const std::shared_ptr<fast_planner::LIOInterface> &lio)
{
    std::lock_guard<std::mutex> lock(epic_lio_mutex_);
    epic_lio_ = lio;
}

bool MapManager::hasEpicLioMap() const
{
    std::lock_guard<std::mutex> lock(epic_lio_mutex_);
    return epic_lio_ != nullptr && epic_lio_->ld_ != nullptr;
}

std::shared_ptr<fast_planner::LIOInterface> MapManager::epicLio() const
{
    std::lock_guard<std::mutex> lock(epic_lio_mutex_);
    return epic_lio_;
}

void MapManager::initEpicLioMap(ros::NodeHandle &nh)
{
    std::lock_guard<std::mutex> lock(epic_lio_mutex_);
    if (epic_lio_ == nullptr) {
        epic_lio_ = std::make_shared<fast_planner::LIOInterface>();
        epic_lio_->init(nh);
    }
}

void MapManager::updateEpicLioMap(const rog_map::PointCloud &cloud,
                                  const super_utils::Pose &pose,
                                  const CloudFrame frame,
                                  const rog_map::RobotState &robot)
{
    std::shared_ptr<fast_planner::LIOInterface> lio;
    {
        std::lock_guard<std::mutex> lock(epic_lio_mutex_);
        lio = epic_lio_;
    }
    if (lio == nullptr || cloud.empty()) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> world_cloud;
    world_cloud.points.reserve(cloud.points.size());
    for (const auto &point : cloud.points) {
        const rog_map::Vec3f raw(point.x, point.y, point.z);
        const rog_map::Vec3f world =
                frame == CloudFrame::WORLD ? raw : pose.first + pose.second * raw;
        if (!world.allFinite()) {
            continue;
        }
        world_cloud.points.emplace_back(static_cast<float>(world.x()),
                                        static_cast<float>(world.y()),
                                        static_cast<float>(world.z()));
    }
    world_cloud.width = static_cast<uint32_t>(world_cloud.points.size());
    world_cloud.height = 1;
    world_cloud.is_dense = cloud.is_dense;
    if (world_cloud.empty()) {
        return;
    }

    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(world_cloud, *cloud_msg);
    cloud_msg->header.frame_id = "world";
    cloud_msg->header.stamp = robot.rcv && robot.rcv_time > 0.0
                              ? ros::Time(robot.rcv_time)
                              : ros::Time::now();

    nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
    odom->header = cloud_msg->header;
    const rog_map::Vec3f odom_p = robot.rcv ? robot.p : pose.first;
    const rog_map::Vec3f odom_v = robot.rcv ? robot.v : rog_map::Vec3f::Zero();
    const Eigen::Quaterniond odom_q = robot.rcv ? robot.q.normalized()
                                                : pose.second.normalized();
    odom->pose.pose.position.x = odom_p.x();
    odom->pose.pose.position.y = odom_p.y();
    odom->pose.pose.position.z = odom_p.z();
    odom->pose.pose.orientation.w = odom_q.w();
    odom->pose.pose.orientation.x = odom_q.x();
    odom->pose.pose.orientation.y = odom_q.y();
    odom->pose.pose.orientation.z = odom_q.z();
    odom->twist.twist.linear.x = odom_v.x();
    odom->twist.twist.linear.y = odom_v.y();
    odom->twist.twist.linear.z = odom_v.z();

    lio->updateCloudMapOdometry(cloud_msg, odom);
}

double MapManager::getEpicDisToOcc(const Eigen::Vector3f &pt) const
{
    std::shared_ptr<fast_planner::LIOInterface> lio;
    {
        std::lock_guard<std::mutex> lock(epic_lio_mutex_);
        lio = epic_lio_;
    }
    return lio != nullptr ? lio->getDisToOcc(pt) : 10.0;
}

void MapManager::epicKNN(const PointCloudMap::PointType &pt,
                         const int k,
                         PointCloudMap::PointVector &pts,
                         std::vector<float> &sqr_distances) const
{
    pts.clear();
    sqr_distances.clear();
    std::shared_ptr<fast_planner::LIOInterface> lio;
    {
        std::lock_guard<std::mutex> lock(epic_lio_mutex_);
        lio = epic_lio_;
    }
    if (lio == nullptr || k <= 0) {
        return;
    }
    PointVector native_pts;
    lio->KNN(pt, k, native_pts, sqr_distances);
    pts.reserve(native_pts.size());
    for (const auto &p : native_pts) {
        pts.emplace_back(p);
    }
}

void MapManager::epicKNN(const Eigen::Vector3f &pt,
                         const int k,
                         PointCloudMap::PointVector &pts,
                         std::vector<float> &sqr_distances) const
{
    PointCloudMap::PointType query;
    query.x = pt.x();
    query.y = pt.y();
    query.z = pt.z();
    epicKNN(query, k, pts, sqr_distances);
}

void MapManager::epicBoxSearch(const Eigen::Vector3f &box_min,
                               const Eigen::Vector3f &box_max,
                               PointCloudMap::PointVector &pts) const
{
    pts.clear();
    std::shared_ptr<fast_planner::LIOInterface> lio;
    {
        std::lock_guard<std::mutex> lock(epic_lio_mutex_);
        lio = epic_lio_;
    }
    if (lio == nullptr) {
        return;
    }
    PointVector native_pts;
    lio->boxSearch(box_min, box_max, native_pts);
    pts.reserve(native_pts.size());
    for (const auto &p : native_pts) {
        pts.emplace_back(p);
    }
}

bool MapManager::epicIsInBox(const Eigen::Vector3f &pt) const
{
    std::shared_ptr<fast_planner::LIOInterface> lio;
    {
        std::lock_guard<std::mutex> lock(epic_lio_mutex_);
        lio = epic_lio_;
    }
    return lio != nullptr && pt.allFinite() && lio->IsInBox(pt);
}

bool MapManager::epicIsInMap(const Eigen::Vector3f &pt) const
{
    std::shared_ptr<fast_planner::LIOInterface> lio;
    {
        std::lock_guard<std::mutex> lock(epic_lio_mutex_);
        lio = epic_lio_;
    }
    return lio != nullptr && pt.allFinite() && lio->IsInMap(pt);
}

} // namespace general_planner
