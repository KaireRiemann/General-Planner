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

namespace person_tracker
{

std::size_t VoxelIndexHash::operator()(const VoxelIndex & index) const
{
  const std::size_t hx = std::hash<int>{}(index.x);
  const std::size_t hy = std::hash<int>{}(index.y);
  const std::size_t hz = std::hash<int>{}(index.z);
  return hx ^ (hy + 0x9e3779b9U + (hx << 6U) + (hx >> 2U)) ^
         (hz + 0x9e3779b9U + (hy << 6U) + (hy >> 2U));
}

VoxelIndex MappingRos::positionToVoxel(const Eigen::Vector3d & position) const
{
  const double inverse_resolution = 1.0 / params_.map_resolution;
  return VoxelIndex{
    static_cast<int>(std::floor(position.x() * inverse_resolution)),
    static_cast<int>(std::floor(position.y() * inverse_resolution)),
    static_cast<int>(std::floor(position.z() * inverse_resolution))};
}

Eigen::Vector3d MappingRos::voxelToPosition(const VoxelIndex & index) const
{
  return params_.map_resolution *
         (Eigen::Vector3d(index.x, index.y, index.z) + Eigen::Vector3d::Constant(0.5));
}

bool MappingRos::voxelInLocalMap(const VoxelIndex & index) const
{
  const Eigen::Vector3d offset = voxelToPosition(index) - odom_position_;
  return std::abs(offset.x()) <= params_.local_update_range.x() &&
         std::abs(offset.y()) <= params_.local_update_range.y() &&
         std::abs(offset.z()) <= params_.local_update_range.z();
}

void MappingRos::updateVoxelMap(const PointCloudPtr & cloud)
{
  // Current-cloud mode treats the live observation layer separately
  // from the persistent map: points not present in the new scan disappear from
  // the observation layer immediately. Tracking never consumes the optional
  // fading visualization map below.
  ++cloud_frame_index_;
  if (params_.map_current_frame_only) {
    // For real-time tracking tests, a fading log-odds layer looks like sensor
    // latency even though detection already uses only the newest scan. Publish
    // a true current-frame voxel map by default, matching the rebuilt
    // point-cloud observation/inflation layer.
    occupancy_map_.clear();
  } else {
    for (auto iterator = occupancy_map_.begin(); iterator != occupancy_map_.end();) {
      if (!voxelInLocalMap(iterator->first)) {
        iterator = occupancy_map_.erase(iterator);
      } else {
        if (iterator->second.last_hit_frame != cloud_frame_index_) {
          iterator->second.log_odds = std::max(
            min_log_odds_, iterator->second.log_odds + miss_log_odds_);
        }
        if (
          iterator->second.log_odds <= min_log_odds_ + 1e-6 &&
          cloud_frame_index_ - iterator->second.last_hit_frame >
          static_cast<std::uint64_t>(params_.recent_voxel_frames))
        {
          iterator = occupancy_map_.erase(iterator);
          continue;
        }
        ++iterator;
      }
    }
  }

  for (const auto & point : cloud->points) {
    const VoxelIndex endpoint_index = positionToVoxel(
      Eigen::Vector3d(point.x, point.y, point.z));
    if (!voxelInLocalMap(endpoint_index)) {
      continue;
    }
    auto & cell = occupancy_map_[endpoint_index];
    cell.log_odds = max_log_odds_;
    cell.last_hit_frame = cloud_frame_index_;
  }
}

PointCloudPtr MappingRos::extractOccupiedCloud() const
{
  PointCloudPtr occupied(new PointCloud);
  occupied->reserve(occupancy_map_.size());
  for (const auto & entry : occupancy_map_) {
    if (entry.second.log_odds <= occupied_log_odds_) {
      continue;
    }
    const Eigen::Vector3d center = voxelToPosition(entry.first);
    occupied->push_back(PointType(
      static_cast<float>(center.x()), static_cast<float>(center.y()),
      static_cast<float>(center.z())));
  }
  occupied->width = occupied->size();
  occupied->height = 1;
  occupied->is_dense = true;
  return occupied;
}

PointCloudPtr MappingRos::buildInflatedCloud(const PointCloudPtr & occupied) const
{
  PointCloudPtr inflated(new PointCloud);
  if (occupied->empty()) {
    inflated->width = 0;
    inflated->height = 1;
    inflated->is_dense = true;
    return inflated;
  }

  const int xy_steps = static_cast<int>(std::ceil(params_.map_inflation_xy / params_.map_resolution));
  const int z_up_steps = static_cast<int>(
    std::ceil(params_.map_inflation_z_up / params_.map_resolution));
  const int z_down_steps = static_cast<int>(
    std::ceil(params_.map_inflation_z_down / params_.map_resolution));
  std::unordered_set<VoxelIndex, VoxelIndexHash> inflated_indices;
  inflated_indices.reserve(occupied->size() * 8U);

  for (const auto & point : occupied->points) {
    const VoxelIndex base = positionToVoxel(Eigen::Vector3d(point.x, point.y, point.z));
    for (int dx = -xy_steps; dx <= xy_steps; ++dx) {
      for (int dy = -xy_steps; dy <= xy_steps; ++dy) {
        const double horizontal_distance =
          params_.map_resolution * std::sqrt(static_cast<double>(dx * dx + dy * dy));
        if (horizontal_distance > params_.map_inflation_xy + 1e-9) {
          continue;
        }
        for (int dz = -z_down_steps; dz <= z_up_steps; ++dz) {
          const VoxelIndex index{base.x + dx, base.y + dy, base.z + dz};
          if (voxelInLocalMap(index)) {
            inflated_indices.insert(index);
          }
        }
      }
    }
  }

  inflated->reserve(inflated_indices.size());
  for (const auto & index : inflated_indices) {
    const Eigen::Vector3d center = voxelToPosition(index);
    inflated->push_back(PointType(
      static_cast<float>(center.x()), static_cast<float>(center.y()),
      static_cast<float>(center.z())));
  }
  inflated->width = inflated->size();
  inflated->height = 1;
  inflated->is_dense = true;
  return inflated;
}

}  // namespace person_tracker
