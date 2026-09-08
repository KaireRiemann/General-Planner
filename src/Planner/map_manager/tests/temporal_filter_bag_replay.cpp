#include <map_manager/temporal_static_filter.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <chrono>

class ReplayMap : public rog_map::ProbMap {
public:
    explicit ReplayMap(const std::string &config) {
        cfg_ = rog_map::Config(config);
        initProbMap();
    }
};

// Read-only regression: no ROS master, publishers, or writes to the source bag.
int main(int argc, char **argv) {
    if (argc < 2 || argc > 4) return 2;
    const int stride = argc == 4 ? std::max(1, std::stoi(argv[3])) : 1;
    std::unique_ptr<ReplayMap> map;
    if (argc >= 3) map.reset(new ReplayMap(argv[2]));
    general_planner::TemporalStaticFilter filter;
    general_planner::TemporalStaticFilter::Config cfg;
    cfg.enabled = true; cfg.voxel_size = 0.20; cfg.min_observations = 10;
    cfg.min_observation_span = 0.90; cfg.min_observer_baseline = 0.40;
    cfg.max_observation_gap = 0.45;
    filter.configure(cfg); // Even a legacy motion-gated profile must bootstrap.
    rosbag::Bag bag(argv[1], rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{"/cloud_registered", "/lidar_slam/odom"}));
    rog_map::Vec3f position = rog_map::Vec3f::Zero(), first = position;
    bool have_odom = false;
    std::size_t scans = 0, nonempty = 0, promoted = 0, received = 0;
    double first_cloud = -1, first_promotion = -1, max_motion = 0;
    double fusion_seconds = 0;
    for (const auto &entry : view) {
        if (auto odom = entry.instantiate<nav_msgs::Odometry>()) {
            position = rog_map::Vec3f(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
            if (!have_odom) first = position;
            have_odom = true; max_motion = std::max(max_motion, (position - first).norm());
        } else if (auto msg = entry.instantiate<sensor_msgs::PointCloud2>()) {
            if (!have_odom) continue;
            if (received++ % stride) continue;
            pcl::PointCloud<pcl::PointXYZ> xyz;
            pcl::fromROSMsg(*msg, xyz);
            rog_map::PointCloud points;
            for (const auto &p : xyz) {
                rog_map::PclPoint q; q.x=p.x; q.y=p.y; q.z=p.z; q.intensity=0;
                points.push_back(q);
            }
            const double stamp = msg->header.stamp.toSec();
            if (first_cloud < 0) first_cloud = stamp;
            auto output = filter.filter(points, stamp, &position);
            ++scans; promoted += output.size();
            // Exercise the entire moving recording, not merely bootstrap.
            // No ROS callbacks or actuator output are used.
            if (map) {
                rog_map::Pose pose{position, Eigen::Quaterniond::Identity()};
                const auto start = std::chrono::steady_clock::now();
                map->updateProbMap(points, pose);
                fusion_seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            }
            if (!output.empty()) { ++nonempty; if (first_promotion < 0) first_promotion = stamp; }
        }
    }
    std::cout << "scans=" << scans << " nonempty_static_scans=" << nonempty
              << " promoted_points=" << promoted << " max_motion=" << max_motion
              << " first_promotion_seconds=" << (first_promotion < 0 ? -1 : first_promotion-first_cloud) << std::endl;
    bool fusion_ok = true;
    if (map) {
        // Stay above the virtual floor: count sensor-built geometry, not
        // configured ground/ceiling obstacles.
        const rog_map::Vec3f extent(5, 5, 0.75);
        std::size_t free_count = 0, occupied_count = 0;
        const double resolution = map->getResolution();
        for (double x = position.x()-extent.x(); x < position.x()+extent.x(); x += resolution)
            for (double y = position.y()-extent.y(); y < position.y()+extent.y(); y += resolution)
                for (double z = position.z()-extent.z(); z < position.z()+extent.z(); z += resolution) {
                    const rog_map::Vec3f p(x, y, z);
                    free_count += map->isKnownFree(p);
                    occupied_count += map->isOccupied(p);
                }
        std::cout << "final_local_free_voxels=" << free_count
                  << " final_local_occupied_voxels=" << occupied_count
                  << " fusion_seconds=" << fusion_seconds << std::endl;
        fusion_ok = free_count > 0 && occupied_count > 0;
    }
    // Filter output is diagnostic only: even zero confirmed points must not
    // prevent the actual local map from building at reduced input rates.
    return map ? (fusion_ok ? 0 : 1) : (nonempty > 0 ? 0 : 1);
}
