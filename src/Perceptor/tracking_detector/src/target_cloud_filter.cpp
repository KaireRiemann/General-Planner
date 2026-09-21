#include <tracking_detector/target_dynamic_cloud.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <stdexcept>
#include <unordered_set>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// Frontend publishes dynamic_points for the selected target only, using the
// LiDAR scan stamp. Mapping waits for that stamp, then drops matching voxels.
class TargetCloudFilter {
public:
    TargetCloudFilter() : nh_("~") {
        nh_.param("frontend_voxel_resolution", resolution_, 0.1);
        nh_.param("cluster_wait_seconds", wait_seconds_, 0.2);
        nh_.param("cluster_hold_seconds", hold_seconds_, 0.5);
        if (!(resolution_ > 0.0) || !(wait_seconds_ > 0.0) || !(hold_seconds_ > 0.0)) {
            ROS_FATAL("Invalid target cluster filter parameters");
            ros::shutdown();
            return;
        }
        // Zip waits on exact scan stamps. Bound the queue by wait time at 10 Hz
        // so a slow first cluster cannot force unfiltered pass-through.
        max_pending_ = static_cast<std::size_t>(
            std::max(4, static_cast<int>(std::ceil(wait_seconds_ / 0.08)) + 1));
        cloud_sub_ = nh_.subscribe("cloud_in", 2, &TargetCloudFilter::onCloud, this);
        target_sub_ = nh_.subscribe("target_cluster", 2, &TargetCloudFilter::onTargetCluster, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_out", 2);
        timer_ = nh_.createTimer(ros::Duration(0.02), &TargetCloudFilter::drain, this);
    }

private:
    struct Pending {
        sensor_msgs::PointCloud2ConstPtr cloud;
        ros::WallTime received;
    };

    void onCloud(const sensor_msgs::PointCloud2ConstPtr &cloud) {
        if (!cloud) {
            return;
        }
        if (cloud->header.stamp.isZero()) {
            cloud_pub_.publish(cloud);
            return;
        }
        pending_.push_back({cloud, ros::WallTime::now()});
        drain(ros::TimerEvent());
    }

    void onTargetCluster(const sensor_msgs::PointCloud2ConstPtr &cluster) {
        if (!cluster || cluster->header.stamp.isZero()) {
            return;
        }
        clusters_[cluster->header.stamp.toNSec()] = cluster;
        last_cluster_wall_ = ros::WallTime::now();
        have_cluster_ = true;
        while (clusters_.size() > 8) {
            clusters_.erase(clusters_.begin());
        }
        drain(ros::TimerEvent());
    }

    bool expectingCluster(const ros::WallTime &now) const {
        return have_cluster_ && (now - last_cluster_wall_).toSec() <= hold_seconds_;
    }

    void drain(const ros::TimerEvent &) {
        const ros::WallTime now = ros::WallTime::now();
        const bool expect = expectingCluster(now);
        while (!pending_.empty()) {
            const auto &front = pending_.front();
            const auto matched = clusters_.find(front.cloud->header.stamp.toNSec());
            const bool timed_out = (now - front.received).toSec() >= wait_seconds_;
            if (matched == clusters_.end() && expect && !timed_out &&
                pending_.size() < max_pending_) {
                break;
            }
            if (matched == clusters_.end()) {
                if (expect) {
                    ROS_WARN_THROTTLE(2.0,
                        "Target cluster missed scan stamp; forwarding unfiltered cloud");
                }
                cloud_pub_.publish(front.cloud);
            } else {
                publishFiltered(*front.cloud, *matched->second);
                clusters_.erase(matched);
            }
            pending_.pop_front();
        }
        if (!pending_.empty()) {
            const uint64_t oldest = pending_.front().cloud->header.stamp.toNSec();
            while (!clusters_.empty() && clusters_.begin()->first < oldest) {
                clusters_.erase(clusters_.begin());
            }
        }
    }

    void publishFiltered(const sensor_msgs::PointCloud2 &cloud,
                         const sensor_msgs::PointCloud2 &cluster) {
        if (cluster.width * cluster.height == 0 || cloud.point_step == 0 ||
            cloud.row_step != cloud.width * cloud.point_step ||
            cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height ||
            (!cloud.header.frame_id.empty() && !cluster.header.frame_id.empty() &&
             cloud.header.frame_id != cluster.header.frame_id)) {
            cloud_pub_.publish(cloud);
            return;
        }
        try {
            std::unordered_set<tracking_detector::Voxel, tracking_detector::VoxelHash> target_voxels;
            target_voxels.reserve(static_cast<std::size_t>(cluster.width) * cluster.height);
            sensor_msgs::PointCloud2ConstIterator<float> cx(cluster, "x"), cy(cluster, "y"),
                cz(cluster, "z");
            for (; cx != cx.end(); ++cx, ++cy, ++cz) {
                if (std::isfinite(*cx) && std::isfinite(*cy) && std::isfinite(*cz)) {
                    target_voxels.insert(
                        tracking_detector::makeVoxel(*cx, *cy, *cz, resolution_));
                }
            }
            if (target_voxels.empty()) {
                cloud_pub_.publish(cloud);
                return;
            }
            sensor_msgs::PointCloud2 filtered = cloud;
            filtered.height = 1;
            filtered.width = 0;
            filtered.data.clear();
            filtered.data.reserve(cloud.data.size());
            sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
            std::size_t i = 0;
            for (; x != x.end(); ++x, ++y, ++z, ++i) {
                const bool is_target = std::isfinite(*x) && std::isfinite(*y) &&
                    std::isfinite(*z) &&
                    target_voxels.count(tracking_detector::makeVoxel(*x, *y, *z, resolution_));
                if (is_target) {
                    continue;
                }
                const std::size_t offset = i * cloud.point_step;
                filtered.data.insert(filtered.data.end(), cloud.data.begin() + offset,
                                     cloud.data.begin() + offset + cloud.point_step);
                ++filtered.width;
            }
            filtered.row_step = filtered.width * filtered.point_step;
            cloud_pub_.publish(filtered);
            ROS_INFO_THROTTLE(5.0, "Removed %u points from selected target cluster",
                              cloud.width * cloud.height - filtered.width);
        } catch (const std::runtime_error &e) {
            ROS_ERROR_THROTTLE(5.0, "Invalid target cluster cloud: %s", e.what());
            cloud_pub_.publish(cloud);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_, target_sub_;
    ros::Publisher cloud_pub_;
    ros::Timer timer_;
    std::deque<Pending> pending_;
    std::map<uint64_t, sensor_msgs::PointCloud2ConstPtr> clusters_;
    ros::WallTime last_cluster_wall_;
    double resolution_{0.1};
    double wait_seconds_{0.2};
    double hold_seconds_{0.5};
    std::size_t max_pending_{4};
    bool have_cluster_{false};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_cloud_filter");
    TargetCloudFilter filter;
    ros::spin();
    return 0;
}
