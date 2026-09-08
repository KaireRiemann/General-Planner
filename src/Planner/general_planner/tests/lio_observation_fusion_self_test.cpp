#include <general_core/exploration/exploration_utils/lidar_map/lidar_map.h>
#include <iostream>
#include <stdexcept>

sensor_msgs::PointCloud2Ptr cloud(float x, bool empty = false) {
    pcl::PointCloud<pcl::PointXYZ> points;
    if (!empty) points.push_back(pcl::PointXYZ(x, 0, 1.5));
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(points, *msg);
    return msg;
}
void expect(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
int main() {
    ros::Time::init();
    // IKD embeds a large rebuild queue; production also owns it on the heap.
    auto storage = std::make_shared<fast_planner::LIOInterface>();
    auto &map = *storage;
    map.lp_.reset(new fast_planner::LIOInterfaceParam{});
    map.ld_.reset(new fast_planner::LIOInterfaceData{});
    map.ld_->first_map_flag_ = true;
    nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
    odom->pose.pose.orientation.w = 1;
    auto near = cloud(1), far = cloud(3), empty = cloud(0, true);
    const Eigen::Vector3d p(1, 0, 1.5), low(-5, -5, -5), high(5, 5, 5);
    map.updateCloudMapOdometry(near, odom, empty);
    expect(map.getDisToOcc(p) < 0.01, "unconfirmed current obstacle missing from collision query");
    PointVector points;
    map.boxSearch(Eigen::Vector3f(0.9, -0.1, 1.4), Eigen::Vector3f(1.1, 0.1, 1.6), points);
    expect(!points.empty(), "unconfirmed current obstacle missing from box query");
    map.updateCloudMapOdometry(far, odom, empty);
    expect(map.getDisToOcc(p) > 1.5, "previous raw frame left a permanent ghost");
    map.updateCloudMapOdometry(near, odom, near);
    map.updateCloudMapOdometry(far, odom, empty, [](const Eigen::Vector3d &) { return false; }, low, high);
    expect(map.getDisToOcc(p) < 0.1, "unobserved/occluded static geometry must be retained");
    map.updateCloudMapOdometry(far, odom, empty, [](const Eigen::Vector3d &q) { return q.x() < 2; }, low, high);
    expect(map.getDisToOcc(p) > 1.5, "explicit free observation must remove a persistent ghost");
    std::cout << "lio_observation_fusion_self_test: PASS\n";
}
