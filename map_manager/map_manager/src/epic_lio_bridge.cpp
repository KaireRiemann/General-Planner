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
    nh.param("epic_lio/publish_map", epic_lio_publish_map_, true);
    nh.param("epic_lio/publish_map_period", epic_lio_publish_map_period_, 0.5);
    nh.param("epic_lio/self_filter_radius", epic_lio_self_filter_radius_, 0.0);
    epic_lio_publish_map_period_ = std::max(0.05, epic_lio_publish_map_period_);
    epic_lio_self_filter_radius_ = std::max(0.0, epic_lio_self_filter_radius_);
    if (epic_lio_publish_map_ && !epic_lio_map_pub_) {
        epic_lio_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("epic_lio/cloud", 1);
        ros::NodeHandle root_nh;
        epic_lio_legacy_map_pub_ =
                root_nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
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

    const rog_map::Vec3f odom_p = robot.rcv ? robot.p : pose.first;
    const rog_map::Vec3f odom_v = robot.rcv ? robot.v : rog_map::Vec3f::Zero();
    const Eigen::Quaterniond odom_q = robot.rcv ? robot.q.normalized()
                                                : pose.second.normalized();
    const double self_filter_radius_sq =
            epic_lio_self_filter_radius_ * epic_lio_self_filter_radius_;

    pcl::PointCloud<pcl::PointXYZ> world_cloud;
    world_cloud.points.reserve(cloud.points.size());
    for (const auto &point : cloud.points) {
        const rog_map::Vec3f raw(point.x, point.y, point.z);
        const rog_map::Vec3f world =
                frame == CloudFrame::WORLD ? raw : pose.first + pose.second * raw;
        if (!world.allFinite()) {
            continue;
        }
        if (self_filter_radius_sq > 0.0 &&
            (world - odom_p).squaredNorm() < self_filter_radius_sq) {
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
    publishEpicLioMap(cloud_msg->header.stamp);
}

void MapManager::publishEpicLioMap(const ros::Time &stamp) const
{
    std::shared_ptr<fast_planner::LIOInterface> lio;
    ros::Publisher map_pub;
    ros::Publisher legacy_map_pub;
    {
        std::lock_guard<std::mutex> lock(epic_lio_mutex_);
        if (!epic_lio_publish_map_) {
            return;
        }
        const ros::Time now = ros::Time::now();
        if (!last_epic_lio_map_publish_stamp_.isZero() &&
            (now - last_epic_lio_map_publish_stamp_).toSec() <
                    epic_lio_publish_map_period_) {
            return;
        }
        last_epic_lio_map_publish_stamp_ = now;
        lio = epic_lio_;
        map_pub = epic_lio_map_pub_;
        legacy_map_pub = epic_lio_legacy_map_pub_;
    }

    if (lio == nullptr || lio->lp_ == nullptr) {
        return;
    }
    if ((map_pub.getNumSubscribers() <= 0) &&
        (legacy_map_pub.getNumSubscribers() <= 0)) {
        return;
    }

    PointVector native_pts;
    lio->boxSearch(lio->lp_->global_map_min_boundary_,
                   lio->lp_->global_map_max_boundary_,
                   native_pts);
    if (native_pts.empty()) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    map_cloud.points = native_pts;
    map_cloud.width = static_cast<uint32_t>(map_cloud.points.size());
    map_cloud.height = 1;
    map_cloud.is_dense = true;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(map_cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    if (map_pub.getNumSubscribers() > 0) {
        map_pub.publish(msg);
    }
    if (legacy_map_pub.getNumSubscribers() > 0) {
        legacy_map_pub.publish(msg);
    }
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
