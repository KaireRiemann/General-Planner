#include <tracking_detector/target_dynamic_cloud.hpp>
#include <tracking_detector/BoundingBox.h>
#include <tracking_detector/BoundingBoxes.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>

namespace {

Eigen::Matrix3d rotationFromQuat(double x, double y, double z, double w) {
    Eigen::Quaterniond q(w, x, y, z);
    if (q.norm() < 1e-9) {
        return Eigen::Matrix3d::Identity();
    }
    q.normalize();
    return q.toRotationMatrix();
}

bool interpolatePose(const std::deque<nav_msgs::Odometry> &samples, const ros::Time &stamp,
                     double tolerance, Eigen::Vector3d *p, Eigen::Matrix3d *R) {
    if (!p || !R || samples.empty() || stamp.isZero()) {
        return false;
    }
    const double t = stamp.toSec();
    std::size_t i = 0;
    while (i < samples.size() && samples[i].header.stamp.toSec() < t) {
        ++i;
    }
    auto accept = [&](const nav_msgs::Odometry &s) {
        if (std::abs(s.header.stamp.toSec() - t) > tolerance) {
            return false;
        }
        const auto &pos = s.pose.pose.position;
        const auto &q = s.pose.pose.orientation;
        *p = Eigen::Vector3d(pos.x, pos.y, pos.z);
        *R = rotationFromQuat(q.x, q.y, q.z, q.w);
        return p->allFinite();
    };
    if (i == 0) {
        return accept(samples.front());
    }
    if (i >= samples.size()) {
        return accept(samples.back());
    }
    const nav_msgs::Odometry &a = samples[i - 1];
    const nav_msgs::Odometry &b = samples[i];
    const double ta = a.header.stamp.toSec();
    const double tb = b.header.stamp.toSec();
    if (tb - ta > 2.0 * tolerance || tb <= ta) {
        return accept(std::abs(t - ta) <= std::abs(t - tb) ? a : b);
    }
    const double u = (t - ta) / (tb - ta);
    const auto &pa = a.pose.pose.position;
    const auto &pb = b.pose.pose.position;
    *p = (1.0 - u) * Eigen::Vector3d(pa.x, pa.y, pa.z) + u * Eigen::Vector3d(pb.x, pb.y, pb.z);
    Eigen::Quaterniond qa(a.pose.pose.orientation.w, a.pose.pose.orientation.x,
                          a.pose.pose.orientation.y, a.pose.pose.orientation.z);
    Eigen::Quaterniond qb(b.pose.pose.orientation.w, b.pose.pose.orientation.x,
                          b.pose.pose.orientation.y, b.pose.pose.orientation.z);
    if (qa.norm() < 1e-9 || qb.norm() < 1e-9) {
        return false;
    }
    qa.normalize();
    qb.normalize();
    if (qa.dot(qb) < 0.0) {
        qb.coeffs() *= -1.0;
    }
    *R = qa.slerp(u, qb).toRotationMatrix();
    return p->allFinite();
}

std::vector<Eigen::Vector3d> cloudXyz(const sensor_msgs::PointCloud2 &cloud) {
    std::vector<Eigen::Vector3d> points;
    if (cloud.point_step == 0) {
        return points;
    }
    try {
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        points.reserve(static_cast<std::size_t>(cloud.width) * cloud.height);
        for (; x != x.end(); ++x, ++y, ++z) {
            points.emplace_back(*x, *y, *z);
        }
    } catch (const std::exception &) {
        points.clear();
    }
    return points;
}

sensor_msgs::PointCloud2 makeXyzCloud(const std_msgs::Header &header,
                                      const std::vector<Eigen::Vector3d> &points,
                                      const std::vector<int> &indices) {
    sensor_msgs::PointCloud2 msg;
    msg.header = header;
    msg.height = 1;
    msg.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(indices.size());
    sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    std::size_t count = 0;
    for (int index : indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= points.size()) {
            continue;
        }
        const Eigen::Vector3d &p = points[static_cast<std::size_t>(index)];
        if (!p.allFinite()) {
            continue;
        }
        *x = static_cast<float>(p.x());
        *y = static_cast<float>(p.y());
        *z = static_cast<float>(p.z());
        ++x;
        ++y;
        ++z;
        ++count;
    }
    modifier.resize(count);
    msg.width = static_cast<uint32_t>(count);
    msg.row_step = msg.width * msg.point_step;
    return msg;
}

}  // namespace

class TargetLidarCluster {
public:
    TargetLidarCluster() : nh_("~") {
        nh_.param("target_label", target_label_, target_label_);
        nh_.param("bbox_sync_tolerance", bbox_sync_tolerance_, bbox_sync_tolerance_);
        nh_.param("odom_tolerance", odom_tolerance_, odom_tolerance_);
        nh_.param("association_gate", association_gate_, association_gate_);
        nh_.param("bbox_width", params_.width, params_.width);
        nh_.param("bbox_height", params_.height, params_.height);
        nh_.param("box_inset", params_.box_inset, params_.box_inset);
        nh_.param("min_depth", params_.min_depth, params_.min_depth);
        nh_.param("max_depth", params_.max_depth, params_.max_depth);
        nh_.param("cluster_tolerance", params_.cluster_tolerance, params_.cluster_tolerance);
        nh_.param("min_points", params_.min_points, params_.min_points);
        nh_.param("min_extent", params_.min_extent, params_.min_extent);
        nh_.param("max_extent", params_.max_extent, params_.max_extent);
        nh_.param("min_box_coverage", params_.min_box_coverage, params_.min_box_coverage);
        double ground_z = 0.0;
        double min_height = 0.05;
        double max_height = 3.5;
        nh_.param("ground_z", ground_z, ground_z);
        nh_.param("min_height_above_ground", min_height, min_height);
        nh_.param("max_height_above_ground", max_height, max_height);
        params_.min_world_z = ground_z + min_height;
        params_.max_world_z = ground_z + max_height;

        std::vector<double> R, p;
        nh_.param("cam2body_R", R, std::vector<double>{0, 0, 1, -1, 0, 0, 0, -1, 0});
        nh_.param("cam2body_p", p, std::vector<double>{0, 0, 0.1});
        if (R.size() != 9 || p.size() != 3) {
            ROS_FATAL("cam2body_R/p must have 9 and 3 elements");
            ros::shutdown();
            return;
        }
        cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R.data());
        cam2body_p_ = Eigen::Vector3d(p[0], p[1], p[2]);
        nh_.param("cam_fx", params_.fx, params_.fx);
        nh_.param("cam_fy", params_.fy, params_.fy);
        nh_.param("cam_cx", params_.cx, params_.cx);
        nh_.param("cam_cy", params_.cy, params_.cy);

        // Pose must share the LiDAR /cloud_registered clock (ROS time via
        // /lidar_slam/odom). YOLO boxes keep the Unity camera clock; associate
        // them by arrival wall-time, not by stamp equality.
        cloud_sub_ = nh_.subscribe("cloud_in", 2, &TargetLidarCluster::onCloud, this);
        bbox_sub_ = nh_.subscribe("yolo", 8, &TargetLidarCluster::onBbox, this);
        odom_sub_ = nh_.subscribe("odom", 50, &TargetLidarCluster::onOdom, this);
        camera_info_sub_ = nh_.subscribe("camera_info", 1, &TargetLidarCluster::onCameraInfo, this);
        target_odom_sub_ = nh_.subscribe("target_odom", 20, &TargetLidarCluster::onTargetOdom, this);
        cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_out", 2);
    }

private:
    struct BboxSample {
        tracking_detector::BoundingBoxes msg;
        ros::WallTime received;
    };

    void onCameraInfo(const sensor_msgs::CameraInfoConstPtr &msg) {
        if (!msg || msg->width < 2 || msg->height < 2 || msg->K[0] <= 0.0 || msg->K[4] <= 0.0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const double sx = static_cast<double>(params_.width) / static_cast<double>(msg->width);
        const double sy = static_cast<double>(params_.height) / static_cast<double>(msg->height);
        params_.fx = msg->K[0] * sx;
        params_.fy = msg->K[4] * sy;
        params_.cx = msg->K[2] * sx;
        params_.cy = msg->K[5] * sy;
    }

    void onOdom(const nav_msgs::OdometryConstPtr &msg) {
        if (!msg || msg->header.stamp.isZero()) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!odom_.empty() && msg->header.stamp < odom_.back().header.stamp - ros::Duration(0.5)) {
            odom_.clear();
        }
        if (!odom_.empty() && msg->header.stamp <= odom_.back().header.stamp) {
            return;
        }
        odom_.push_back(*msg);
        while (odom_.size() > 600) {
            odom_.pop_front();
        }
    }

    void onTargetOdom(const nav_msgs::OdometryConstPtr &msg) {
        if (!msg) {
            return;
        }
        const auto &p = msg->pose.pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        target_p_ = Eigen::Vector3d(p.x, p.y, p.z);
        target_received_ = ros::WallTime::now();
        have_target_ = true;
    }

    void onBbox(const tracking_detector::BoundingBoxesConstPtr &msg) {
        if (!msg) {
            return;
        }
        const ros::Time stamp = msg->header.stamp.isZero() ? msg->image_header.stamp : msg->header.stamp;
        BboxSample sample;
        sample.msg = *msg;
        if (!stamp.isZero()) {
            sample.msg.header.stamp = stamp;
        }
        sample.received = ros::WallTime::now();
        std::lock_guard<std::mutex> lock(mutex_);
        bboxes_.push_back(std::move(sample));
        while (bboxes_.size() > 12) {
            bboxes_.pop_front();
        }
    }

    // Prefer stamp match when clocks agree; otherwise take the freshest box by
    // wall arrival (Unity camera clock vs ROS LiDAR clock).
    const tracking_detector::BoundingBoxes *nearestBbox(const ros::Time &cloud_stamp,
                                                       const ros::WallTime &cloud_received) const {
        const tracking_detector::BoundingBoxes *best = nullptr;
        double best_score = bbox_sync_tolerance_;
        for (const auto &sample : bboxes_) {
            const double wall_age = std::abs((sample.received - cloud_received).toSec());
            double score = wall_age;
            if (!cloud_stamp.isZero() && !sample.msg.header.stamp.isZero()) {
                const double stamp_age = std::abs((sample.msg.header.stamp - cloud_stamp).toSec());
                // Same-clock association when stamps are within a few seconds;
                // otherwise Unity↔ROS offset would dominate and always miss.
                if (stamp_age <= 2.0) {
                    score = stamp_age;
                }
            }
            if (score <= best_score) {
                best_score = score;
                best = &sample.msg;
            }
        }
        return best;
    }

    void onCloud(const sensor_msgs::PointCloud2ConstPtr &cloud) {
        if (!cloud) {
            return;
        }
        const ros::WallTime cloud_received = ros::WallTime::now();
        tracking_detector::BoundingBoxes bbox_msg;
        tracking_detector::ProjectionParameters params;
        Eigen::Vector3d body_p = Eigen::Vector3d::Zero();
        Eigen::Matrix3d body_R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d target_p = Eigen::Vector3d::Zero();
        bool have_pose = false;
        bool have_target = false;
        bool have_bbox = false;
        std::size_t odom_buf = 0;
        std::size_t bbox_buf = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            params = params_;
            odom_buf = odom_.size();
            bbox_buf = bboxes_.size();
            have_pose = interpolatePose(odom_, cloud->header.stamp, odom_tolerance_, &body_p, &body_R);
            if (!have_pose && !odom_.empty()) {
                const auto &latest = odom_.back();
                if (std::abs((latest.header.stamp - cloud->header.stamp).toSec()) <= odom_tolerance_ ||
                    (ros::Time::now() - latest.header.stamp).toSec() <= odom_tolerance_) {
                    body_p = Eigen::Vector3d(latest.pose.pose.position.x, latest.pose.pose.position.y,
                                             latest.pose.pose.position.z);
                    body_R = rotationFromQuat(latest.pose.pose.orientation.x,
                                              latest.pose.pose.orientation.y,
                                              latest.pose.pose.orientation.z,
                                              latest.pose.pose.orientation.w);
                    have_pose = body_p.allFinite();
                }
            }
            if (have_target_ && (ros::WallTime::now() - target_received_).toSec() <= 0.5) {
                target_p = target_p_;
                have_target = true;
            }
            if (const tracking_detector::BoundingBoxes *bbox =
                    nearestBbox(cloud->header.stamp, cloud_received)) {
                bbox_msg = *bbox;
                have_bbox = true;
            }
        }

        const std::vector<Eigen::Vector3d> points = cloudXyz(*cloud);
        std::vector<int> indices;
        if (have_pose && have_bbox && !points.empty()) {
            const Eigen::Isometry3d world_from_camera =
                tracking_detector::worldFromCamera(body_p, body_R, cam2body_p_, cam2body_R_);
            std::vector<const tracking_detector::BoundingBox *> boxes;
            for (const auto &box : bbox_msg.bounding_boxes) {
                if (box.Class == target_label_ && box.xmax > box.xmin && box.ymax > box.ymin) {
                    boxes.push_back(&box);
                }
            }
            std::sort(boxes.begin(), boxes.end(),
                      [](const tracking_detector::BoundingBox *a,
                         const tracking_detector::BoundingBox *b) {
                          return a->probability > b->probability;
                      });
            tracking_detector::ProjectedSeed best;
            tracking_detector::ProjectedSeed visual;
            double best_assoc = std::numeric_limits<double>::infinity();
            for (const tracking_detector::BoundingBox *box : boxes) {
                tracking_detector::ImageBox image_box{static_cast<double>(box->xmin),
                                                      static_cast<double>(box->ymin),
                                                      static_cast<double>(box->xmax),
                                                      static_cast<double>(box->ymax)};
                const tracking_detector::ProjectedSeed seed =
                    tracking_detector::selectProjectedSeed(points, image_box, world_from_camera, params);
                if (!seed.valid) {
                    continue;
                }
                if (!visual.valid) {
                    visual = seed;
                }
                if (!have_target) {
                    best = seed;
                    break;
                }
                const double assoc = (seed.centroid - target_p).norm();
                if (assoc > association_gate_) {
                    continue;
                }
                if (!best.valid || assoc < best_assoc) {
                    best = seed;
                    best_assoc = assoc;
                }
            }
            if (!best.valid) {
                best = visual;
            }
            if (best.valid) {
                indices = std::move(best.indices);
            }
        } else if (!have_pose || !have_bbox) {
            ROS_WARN_THROTTLE(5.0,
                              "LiDAR cluster skip: have_pose=%d have_bbox=%d odom_buf=%zu "
                              "bbox_buf=%zu (use /lidar_slam/odom for pose; Unity boxes by wall time)",
                              static_cast<int>(have_pose), static_cast<int>(have_bbox), odom_buf,
                              bbox_buf);
        }

        cluster_pub_.publish(makeXyzCloud(cloud->header, points, indices));
        ROS_INFO_THROTTLE(5.0, "Selected %zu LiDAR points for the YOLO target cluster",
                          indices.size());
    }

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_, bbox_sub_, odom_sub_, camera_info_sub_, target_odom_sub_;
    ros::Publisher cluster_pub_;
    std::mutex mutex_;
    std::deque<nav_msgs::Odometry> odom_;
    std::deque<BboxSample> bboxes_;
    tracking_detector::ProjectionParameters params_;
    Eigen::Matrix3d cam2body_R_{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d cam2body_p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_p_{Eigen::Vector3d::Zero()};
    ros::WallTime target_received_;
    std::string target_label_{"car"};
    double bbox_sync_tolerance_{0.25};
    double odom_tolerance_{0.15};
    double association_gate_{4.0};
    bool have_target_{false};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_lidar_cluster");
    TargetLidarCluster node;
    ros::spin();
    return 0;
}
