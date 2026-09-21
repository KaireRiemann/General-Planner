#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <optional>
#include <vector>

namespace person_tracker
{
struct ProjectionParameters
{
  double fx{346.74048}, fy{349.12708}, cx{322.83899}, cy{234.54992};
  int width{640}, height{480};
  double box_inset{0.05};
  double min_depth{0.3}, max_depth{30.0};
  double cluster_tolerance{0.3};
  int min_points{5};
  double min_extent{0.10}, max_extent{4.0};
  double min_box_coverage{0.02};
};

struct ImageBox
{
  double xmin, ymin, xmax, ymax;
};

struct ProjectedSeed
{
  // Indices always reference the input world cloud, never a cropped copy.
  std::vector<int> indices;
  double depth{0.0};
};

// Input is one registered scan. world_from_camera maps optical coordinates
// (right, down, forward) to the same world frame as the cloud.
std::optional<ProjectedSeed> selectProjectedSeed(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const ImageBox & box, const Eigen::Isometry3d & world_from_camera,
  const ProjectionParameters & parameters);
}  // namespace person_tracker
