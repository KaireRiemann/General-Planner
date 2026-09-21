#include "lidar/ground_height_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>
#include <pcl/search/kdtree.h>

namespace person_tracker
{

namespace
{

bool finitePoint(const pcl::PointXYZ & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double gaussianWeight(double value, double sigma)
{
  const double normalized = value / std::max(sigma, 1e-6);
  return std::exp(-0.5 * normalized * normalized);
}

}  // namespace

GroundHeightEstimator::GroundHeightEstimator(const GroundHeightParameters & parameters)
{
  setParameters(parameters);
}

void GroundHeightEstimator::setParameters(const GroundHeightParameters & parameters)
{
  parameters_ = parameters;
  parameters_.person_center_height = std::max(0.05, parameters_.person_center_height);
  parameters_.search_radius = std::max(0.10, parameters_.search_radius);
  parameters_.normal_radius = std::max(0.05, parameters_.normal_radius);
  parameters_.vertical_search_range = std::max(0.10, parameters_.vertical_search_range);
  parameters_.radial_sigma = std::max(0.05, parameters_.radial_sigma);
  parameters_.height_sigma = std::max(0.05, parameters_.height_sigma);
  parameters_.minimum_normal_z = std::clamp(parameters_.minimum_normal_z, 0.0, 1.0);
  parameters_.maximum_curvature = std::clamp(parameters_.maximum_curvature, 0.0, 1.0);
  parameters_.minimum_neighbors = std::max(3, parameters_.minimum_neighbors);
  parameters_.minimum_support_points = std::max(3, parameters_.minimum_support_points);
}

GroundHeightEstimate GroundHeightEstimator::estimate(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const Eigen::Vector2d & target_xy,
  double predicted_center_z) const
{
  GroundHeightEstimate result;
  if (
    !parameters_.enabled || !cloud || cloud->empty() || !target_xy.allFinite() ||
    !std::isfinite(predicted_center_z))
  {
    return result;
  }

  pcl::search::KdTree<pcl::PointXYZ> search;
  search.setInputCloud(cloud);

  const double expected_foot_z = predicted_center_z - parameters_.person_center_height;
  const double search_radius_squared = parameters_.search_radius * parameters_.search_radius;
  double total_weight = 0.0;
  double weighted_height = 0.0;
  Eigen::Vector3d weighted_normal = Eigen::Vector3d::Zero();
  double weighted_normal_quality = 0.0;

  std::vector<int> neighbors;
  std::vector<float> squared_distances;
  neighbors.reserve(32);
  squared_distances.reserve(32);

  for (std::size_t index = 0; index < cloud->size(); ++index) {
    const auto & point = cloud->points[index];
    if (!finitePoint(point)) {
      continue;
    }

    const Eigen::Vector2d horizontal_offset(
      static_cast<double>(point.x) - target_xy.x(),
      static_cast<double>(point.y) - target_xy.y());
    const double horizontal_distance_squared = horizontal_offset.squaredNorm();
    const double height_offset = static_cast<double>(point.z) - expected_foot_z;
    if (
      horizontal_distance_squared > search_radius_squared ||
      std::abs(height_offset) > parameters_.vertical_search_range)
    {
      continue;
    }

    neighbors.clear();
    squared_distances.clear();
    if (
      search.radiusSearch(
        static_cast<int>(index), parameters_.normal_radius,
        neighbors, squared_distances) < parameters_.minimum_neighbors)
    {
      continue;
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    int finite_neighbors = 0;
    for (const int neighbor_index : neighbors) {
      const auto & neighbor = cloud->points[static_cast<std::size_t>(neighbor_index)];
      if (!finitePoint(neighbor)) {
        continue;
      }
      mean += Eigen::Vector3d(neighbor.x, neighbor.y, neighbor.z);
      ++finite_neighbors;
    }
    if (finite_neighbors < parameters_.minimum_neighbors) {
      continue;
    }
    mean /= static_cast<double>(finite_neighbors);

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const int neighbor_index : neighbors) {
      const auto & neighbor = cloud->points[static_cast<std::size_t>(neighbor_index)];
      if (!finitePoint(neighbor)) {
        continue;
      }
      const Eigen::Vector3d offset =
        Eigen::Vector3d(neighbor.x, neighbor.y, neighbor.z) - mean;
      covariance.noalias() += offset * offset.transpose();
    }
    covariance /= static_cast<double>(finite_neighbors);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      continue;
    }
    const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    const double eigenvalue_sum = eigenvalues.sum();
    if (eigenvalue_sum <= 1e-10) {
      continue;
    }
    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    if (normal.z() < 0.0) {
      normal = -normal;
    }
    const double curvature = eigenvalues.x() / eigenvalue_sum;
    if (
      normal.z() < parameters_.minimum_normal_z ||
      curvature > parameters_.maximum_curvature)
    {
      continue;
    }

    const double radial_weight = gaussianWeight(
      std::sqrt(horizontal_distance_squared), parameters_.radial_sigma);
    const double height_weight = gaussianWeight(height_offset, parameters_.height_sigma);
    const double normal_weight = std::pow(normal.z(), 4.0);
    const double planarity_weight = std::clamp(
      1.0 - curvature / std::max(parameters_.maximum_curvature, 1e-6), 0.0, 1.0);
    const double weight = radial_weight * height_weight * normal_weight * planarity_weight;
    if (weight <= 1e-6) {
      continue;
    }

    result.support_indices.push_back(static_cast<int>(index));
    total_weight += weight;
    weighted_height += weight * static_cast<double>(point.z);
    weighted_normal += weight * normal;
    weighted_normal_quality += weight * normal.z();
  }

  if (
    static_cast<int>(result.support_indices.size()) < parameters_.minimum_support_points ||
    total_weight <= 1e-6)
  {
    result.support_indices.clear();
    return result;
  }

  result.height = weighted_height / total_weight;
  if (weighted_normal.norm() > 1e-6) {
    result.normal = weighted_normal.normalized();
  }
  const double count_confidence = std::clamp(
    static_cast<double>(result.support_indices.size()) /
    static_cast<double>(2 * parameters_.minimum_support_points), 0.0, 1.0);
  const double orientation_confidence = std::clamp(
    weighted_normal_quality / total_weight, 0.0, 1.0);
  result.confidence = count_confidence * orientation_confidence;
  result.valid = std::isfinite(result.height) && result.normal.allFinite();
  return result;
}

}  // namespace person_tracker

