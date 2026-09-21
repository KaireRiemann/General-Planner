#include "common/tracking_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Eigenvalues>

namespace person_tracker::detail
{
bool finitePoint(const PointType & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double logit(double probability)
{
  return std::log(probability / (1.0 - probability));
}

std::array<std::uint8_t, 3> clusterColor(std::size_t index)
{
  static constexpr std::array<std::array<std::uint8_t, 3>, 12> colors{{
    {{230, 25, 75}}, {{60, 180, 75}}, {{255, 225, 25}}, {{0, 130, 200}},
    {{245, 130, 48}}, {{145, 30, 180}}, {{70, 240, 240}}, {{240, 50, 230}},
    {{210, 245, 60}}, {{250, 190, 212}}, {{0, 128, 128}}, {{220, 190, 255}}
  }};
  return colors[index % colors.size()];
}

Eigen::Vector3d boundedHorizontalMeasurement(
  const Eigen::Vector3d & current, const Eigen::Vector3d & measurement,
  double maximum_correction)
{
  Eigen::Vector3d bounded = measurement;
  const Eigen::Vector2d offset = measurement.head<2>() - current.head<2>();
  const double distance = offset.norm();
  if (distance > maximum_correction && distance > 1e-6) {
    bounded.head<2>() = current.head<2>() + maximum_correction * offset / distance;
  }
  return bounded;
}

// A world-axis AABB changes when the same object yaws.  Measure the two
// horizontal dimensions in the cluster's PCA frame and sort them as
// [short-side, long-side].  This is an orientation-independent size signature;
// slow bounded adaptation in TargetEkf handles partial views and occlusion.
Eigen::Vector3d orientationInvariantClusterSize(
  const PointCloudPtr & cloud, const std::vector<int> & indices, double voxel_size)
{
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  double minimum_z = std::numeric_limits<double>::max();
  double maximum_z = std::numeric_limits<double>::lowest();
  std::size_t count = 0U;
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
      continue;
    }
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    if (!finitePoint(point)) {
      continue;
    }
    mean += Eigen::Vector2d(point.x, point.y);
    minimum_z = std::min(minimum_z, static_cast<double>(point.z));
    maximum_z = std::max(maximum_z, static_cast<double>(point.z));
    ++count;
  }
  if (count == 0U) {
    return Eigen::Vector3d::Constant(voxel_size);
  }
  mean /= static_cast<double>(count);

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
      continue;
    }
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    if (!finitePoint(point)) {
      continue;
    }
    const Eigen::Vector2d offset = Eigen::Vector2d(point.x, point.y) - mean;
    covariance.noalias() += offset * offset.transpose();
  }
  covariance /= static_cast<double>(count);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  Eigen::Matrix2d axes = Eigen::Matrix2d::Identity();
  if (solver.info() == Eigen::Success) {
    axes = solver.eigenvectors();
  }

  Eigen::Vector2d minimum_xy = Eigen::Vector2d::Constant(
    std::numeric_limits<double>::max());
  Eigen::Vector2d maximum_xy = Eigen::Vector2d::Constant(
    std::numeric_limits<double>::lowest());
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
      continue;
    }
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    if (!finitePoint(point)) {
      continue;
    }
    const Eigen::Vector2d projected =
      axes.transpose() * (Eigen::Vector2d(point.x, point.y) - mean);
    minimum_xy = minimum_xy.cwiseMin(projected);
    maximum_xy = maximum_xy.cwiseMax(projected);
  }
  Eigen::Vector2d horizontal_size = maximum_xy - minimum_xy +
    Eigen::Vector2d::Constant(voxel_size);
  if (horizontal_size.x() > horizontal_size.y()) {
    std::swap(horizontal_size.x(), horizontal_size.y());
  }
  return Eigen::Vector3d(
    horizontal_size.x(), horizontal_size.y(), maximum_z - minimum_z + voxel_size);
}

}  // namespace person_tracker::detail
