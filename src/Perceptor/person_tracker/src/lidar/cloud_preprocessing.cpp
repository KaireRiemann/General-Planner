#include "core/mapping_manager.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common/tracking_geometry.hpp"

namespace person_tracker
{

using detail::finitePoint;

PointCloudPtr MappingRos::preprocessCloud(const sensor_msgs::PointCloud2 & msg) const
{
  PointCloudPtr raw_cloud(new PointCloud);
  pcl::fromROSMsg(msg, *raw_cloud);

  PointCloudPtr local_cloud(new PointCloud);
  local_cloud->reserve(raw_cloud->size());
  const double minimum_range_squared = params_.min_range * params_.min_range;
  const double maximum_range_squared = params_.max_range * params_.max_range;

  for (const auto & input_point : raw_cloud->points) {
    if (!finitePoint(input_point)) {
      continue;
    }

    Eigen::Vector3d point(input_point.x, input_point.y, input_point.z);
    if (!params_.cloud_is_registered) {
      point = odom_orientation_ * point + odom_position_;
    }
    const Eigen::Vector3d offset = point - odom_position_;
    const double range_squared = offset.squaredNorm();
    if (
      std::abs(offset.x()) >= params_.local_update_range.x() ||
      std::abs(offset.y()) >= params_.local_update_range.y() ||
      std::abs(offset.z()) >= params_.local_update_range.z() ||
      range_squared < minimum_range_squared || range_squared > maximum_range_squared)
    {
      continue;
    }

    PointType output_point;
    output_point.x = static_cast<float>(point.x());
    output_point.y = static_cast<float>(point.y());
    output_point.z = static_cast<float>(point.z());
    local_cloud->push_back(output_point);
  }

  local_cloud->width = local_cloud->size();
  local_cloud->height = 1;
  local_cloud->is_dense = true;
  return removeGroundPlane(voxelizeCloud(local_cloud));
}

PointCloudPtr MappingRos::voxelizeCloud(const PointCloudPtr & cloud) const
{
  // Quantize in the fixed world grid before any expensive processing. This is
  // equivalent to the 0.1 m FAPP VoxelGrid, but stable surfaces get exactly the
  // same voxel centre across scans, which makes point-wise background distance
  // much less sensitive to FAST-LIO sampling changes.
  PointCloudPtr voxelized(new PointCloud);
  std::unordered_set<VoxelIndex, VoxelIndexHash> occupied_indices;
  occupied_indices.reserve(cloud->size());
  for (const auto & point : cloud->points) {
    occupied_indices.insert(positionToVoxel(Eigen::Vector3d(point.x, point.y, point.z)));
  }
  voxelized->reserve(occupied_indices.size());
  for (const auto & index : occupied_indices) {
    const Eigen::Vector3d center = voxelToPosition(index);
    voxelized->push_back(PointType(
      static_cast<float>(center.x()), static_cast<float>(center.y()),
      static_cast<float>(center.z())));
  }
  voxelized->width = voxelized->size();
  voxelized->height = 1;
  voxelized->is_dense = true;
  return voxelized;
}

PointCloudPtr MappingRos::removeGroundPlane(const PointCloudPtr & cloud) const
{
  if (!params_.remove_ground || static_cast<int>(cloud->size()) < params_.ground_min_inliers) {
    return cloud;
  }

  pcl::SACSegmentation<PointType> segmentation;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());
  constexpr double pi = 3.14159265358979323846;
  segmentation.setEpsAngle(params_.ground_eps_angle_deg * pi / 180.0);
  segmentation.setDistanceThreshold(params_.ground_distance_threshold);
  segmentation.setMaxIterations(100);
  segmentation.setInputCloud(cloud);
  segmentation.segment(*inliers, *coefficients);

  if (static_cast<int>(inliers->indices.size()) < params_.ground_min_inliers) {
    return cloud;
  }

  PointCloudPtr without_ground(new PointCloud);
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*without_ground);
  return without_ground;
}

}  // namespace person_tracker
