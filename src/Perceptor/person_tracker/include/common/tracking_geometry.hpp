#pragma once

#include "core/mapping_manager.h"
#include <array>

// Shared geometry helpers used only by the tracker implementation.
namespace person_tracker::detail
{
bool finitePoint(const PointType & point);
double logit(double probability);
std::array<std::uint8_t, 3> clusterColor(std::size_t index);
Eigen::Vector3d boundedHorizontalMeasurement(
  const Eigen::Vector3d & current, const Eigen::Vector3d & measurement,
  double maximum_correction);
Eigen::Vector3d orientationInvariantClusterSize(
  const PointCloudPtr & cloud, const std::vector<int> & indices, double voxel_size);
}  // namespace person_tracker::detail
