/*
If you need to replace with other point cloud data structures, please
re-implement the following interfaces:

1. getDisToOcc: Returns the distance from the specified point to the nearest
obstacle in the map.
2. KNN: Nearest neighbor search. boxSearch: Region search.
3. updateCloudMapOdometry: Point cloud map update. 
4. LIOInterfaceData::Ptr ld: Current frame world coordinate point clouds and
lidar-odometry.

If you need to integrate EPIC with Lidar SLAM algorithm and shares
memory, thread mutual exclusion should be noted.
*/
#include "visualization_msgs/Marker.h"
#include <general_core/exploration/exploration_utils/lidar_map/lidar_map.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
namespace fast_planner {

double LIOInterface::getDisToOcc(const PointType &pt) {
  PointVector nes_pts;
  vector<float> diss;
  KNN(pt, 1, nes_pts, diss);
  if (nes_pts.size() == 0)
    return 10.0;
  else
    return sqrt(diss[0]);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3d &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
double LIOInterface::getDisToOcc(const Eigen::Vector3f &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return getDisToOcc(p);
}
void LIOInterface::KNN(const PointType &pt, int k, PointVector &pts,
                       vector<float> &dis) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (k <= 0) { pts.clear(); dis.clear(); return; }
  ikd_Tree_map.Nearest_Search(pt, k, pts, dis, 10.0);
  // Immediate collision evidence is replaced each scan, never accumulated
  // into the persistent tree. It includes unconfirmed/moving obstacles.
  if (k > 0 && current_scan_ && !current_scan_->empty()) {
    std::vector<int> indices;
    std::vector<float> distances;
    current_scan_tree_.nearestKSearch(pt, k, indices, distances);
    for (std::size_t i = 0; i < indices.size(); ++i) {
      if (distances[i] > 100.0f) continue;  // Same 10 m query radius as IKD.
      const auto &candidate = current_scan_->points[indices[i]];
      const bool duplicate = std::any_of(pts.begin(), pts.end(), [&](const PointType &p) {
        return (p.getVector3fMap() - candidate.getVector3fMap()).squaredNorm() < 1.0e-10f;
      });
      if (duplicate) continue;
      pts.push_back(candidate);
      dis.push_back(distances[i]);
    }
    for (std::size_t i = 0; i < dis.size(); ++i)
      for (std::size_t j = i + 1; j < dis.size(); ++j)
        if (dis[j] < dis[i]) { std::swap(dis[i], dis[j]); std::swap(pts[i], pts[j]); }
    if (pts.size() > static_cast<std::size_t>(k)) { pts.resize(k); dis.resize(k); }
  }
}
void LIOInterface::boxSearch(const Eigen::Vector3f &min_bd,
                             const Eigen::Vector3f &max_bd, PointVector &pts) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  BoxPointType boxpoint;
  for (int i = 0; i < 3; i++) {
    boxpoint.vertex_min[i] = min_bd(i);
    boxpoint.vertex_max[i] = max_bd(i);
  }
  ikd_Tree_map.Box_Search(boxpoint, pts);
  if (current_scan_ && !current_scan_->empty()) {
    const Eigen::Vector3f center = 0.5f * (min_bd + max_bd);
    PointType query(center.x(), center.y(), center.z());
    std::vector<int> indices;
    std::vector<float> distances;
    current_scan_tree_.radiusSearch(query, (max_bd - min_bd).norm() * 0.5f,
                                   indices, distances);
    for (int index : indices) {
      const auto &p = current_scan_->points[index];
      if (p.x >= min_bd.x() && p.x <= max_bd.x() &&
          p.y >= min_bd.y() && p.y <= max_bd.y() &&
          p.z >= min_bd.z() && p.z <= max_bd.z()) pts.push_back(p);
    }
  }
}
void LIOInterface::updateCloudMapOdometry(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    const nav_msgs::Odometry::ConstPtr &odom_,
    const sensor_msgs::PointCloud2ConstPtr &confirmed,
    const std::function<bool(const Eigen::Vector3d &)> &observed_free,
    const Eigen::Vector3d &changed_min, const Eigen::Vector3d &changed_max) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  ld_->map_update = true;
  static Eigen::Vector3f last_lidar_pose(0, 0, 0);
  Eigen::Vector3f lidar_pos_(odom_->pose.pose.position.x,
                             odom_->pose.pose.position.y,
                             odom_->pose.pose.position.z);
  Eigen::Vector3f lidar_vel_(odom_->twist.twist.linear.x,
                             odom_->twist.twist.linear.y,
                             odom_->twist.twist.linear.z);
  last_lidar_pose = lidar_pos_;
  ld_->lidar_pose_ = lidar_pos_;
  ld_->lidar_vel_ = lidar_vel_;
  //处理雷达姿态
  Eigen::AngleAxisf y_axis_angle(M_PI / 180.0 * lp_->lidar_pitch_,
                                 Eigen::Vector3f::UnitY());
  Eigen::Quaternionf q_y(y_axis_angle);
  ld_->lidar_q_ = Eigen::Quaternionf(odom_->pose.pose.orientation.w,
                                     odom_->pose.pose.orientation.x,
                                     odom_->pose.pose.orientation.y,
                                     odom_->pose.pose.orientation.z) *
                  q_y;
  
  //0.1体素降采样
  pcl::fromROSMsg(*msg, ld_->lidar_cloud_);
  auto &points = ld_->lidar_cloud_.points;
  points.erase(std::remove_if(points.begin(), points.end(), [](const PointType &p) {
    return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
  }), points.end());
  ld_->lidar_cloud_.width = points.size();
  ld_->lidar_cloud_.height = 1;
  current_scan_.reset(new pcl::PointCloud<PointType>(ld_->lidar_cloud_));
  if (!current_scan_->empty()) current_scan_tree_.setInputCloud(current_scan_);
  // Delete only geometry contradicted by measured free-space evidence.
  // Absence from a scan (occlusion/outside FOV) is not a deletion signal.
  if (observed_free && !ld_->first_map_flag_) {
    BoxPointType box;
    for (int i = 0; i < 3; ++i) {
      box.vertex_min[i] = changed_min[i]; box.vertex_max[i] = changed_max[i];
    }
    PointVector candidates, removed;
    ikd_Tree_map.Box_Search(box, candidates);
    for (const auto &p : candidates)
      if (observed_free(Eigen::Vector3d(p.x, p.y, p.z))) removed.push_back(p);
    if (!removed.empty()) ikd_Tree_map.Delete_Points(removed);
  }
  ros::Time start = ros::Time::now();
  // PointVector pcl_map = points.points;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(0.1, 0.1, 0.1);
  pcl::PointCloud<PointType>::Ptr persistent(new pcl::PointCloud<PointType>);
  if (confirmed) pcl::fromROSMsg(*confirmed, *persistent);
  else *persistent = ld_->lidar_cloud_;
  vg.setInputCloud(persistent);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  vg.filter(*filtered_points);
  PointVector pcl_map = filtered_points->points;

  if (pcl_map.empty())
    return;

  if (ld_->first_map_flag_) {
    // this->ikd_Tree_map(0.3,0.6,0.2);
    this->ikd_Tree_map.set_downsample_param(0.1);
    this->ikd_Tree_map.Build(pcl_map);
    ld_->first_map_flag_ = false;
  } else {
    this->ikd_Tree_map.Add_Points(pcl_map, true);
  }
  ros::Time ikd_update_end_stamp = ros::Time::now();
}

} // namespace fast_planner
