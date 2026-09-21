#pragma once

#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace person_tracker
{

struct GroundHeightParameters
{
  bool enabled{true};
  double person_center_height{0.85};
  double search_radius{0.80};
  double normal_radius{0.22};
  double vertical_search_range{0.55};
  double radial_sigma{0.40};
  double height_sigma{0.30};
  double minimum_normal_z{0.75};
  double maximum_curvature{0.18};
  int minimum_neighbors{6};
  int minimum_support_points{8};
};

struct GroundHeightEstimate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid{false};
  double height{0.0};
  double confidence{0.0};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  std::vector<int> support_indices;
};

// Estimates the local walkable-surface height below a tracked person.  Each
// point receives a local PCA normal.  Up-facing points near the expected foot
// height contribute continuously, so the same calculation works on floors,
// ramps and stair treads without a terrain-state classifier.
class GroundHeightEstimator
{
public:
  explicit GroundHeightEstimator(
    const GroundHeightParameters & parameters = GroundHeightParameters{});

  void setParameters(const GroundHeightParameters & parameters);
  const GroundHeightParameters & parameters() const {return parameters_;}

  GroundHeightEstimate estimate(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
    const Eigen::Vector2d & target_xy,
    double predicted_center_z) const;

private:
  GroundHeightParameters parameters_;
};

}  // namespace person_tracker

