#include "yolo/yolo_projection.hpp"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace person_tracker
{
std::optional<ProjectedSeed> selectProjectedSeed(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const ImageBox & box, const Eigen::Isometry3d & world_from_camera,
  const ProjectionParameters & p)
{
  if (!cloud || cloud->empty() || !world_from_camera.matrix().allFinite() ||
      !std::isfinite(p.fx) || !std::isfinite(p.fy) || p.fx <= 0 || p.fy <= 0 ||
      !std::isfinite(p.cx) || !std::isfinite(p.cy) || p.width <= 0 || p.height <= 0 ||
      !std::isfinite(box.xmin) || !std::isfinite(box.xmax) ||
      !std::isfinite(box.ymin) || !std::isfinite(box.ymax) ||
      box.xmax <= box.xmin || box.ymax <= box.ymin || p.min_points < 1 ||
      !(p.cluster_tolerance > 0) || !(p.box_inset >= 0 && p.box_inset < 0.5)) {
    return std::nullopt;
  }
  const double dx = (box.xmax - box.xmin) * p.box_inset;
  const double dy = (box.ymax - box.ymin) * p.box_inset;
  const double xmin = std::max(0.0, box.xmin + dx);
  const double xmax = std::min(double(p.width - 1), box.xmax - dx);
  const double ymin = std::max(0.0, box.ymin + dy);
  const double ymax = std::min(double(p.height - 1), box.ymax - dy);
  if (xmin >= xmax || ymin >= ymax) return std::nullopt;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> original_indices;
  std::vector<Eigen::Vector3d> projections;
  const Eigen::Isometry3d camera_from_world = world_from_camera.inverse();
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto & point = cloud->points[i];
    const Eigen::Vector3d camera_point = camera_from_world *
      Eigen::Vector3d(point.x, point.y, point.z);
    if (!camera_point.allFinite() || camera_point.z() < p.min_depth ||
        camera_point.z() > p.max_depth) continue;
    const double u = p.fx * camera_point.x() / camera_point.z() + p.cx;
    const double v = p.fy * camera_point.y() / camera_point.z() + p.cy;
    if (u < xmin || u > xmax || v < ymin || v > ymax) continue;
    cropped->push_back(point);
    original_indices.push_back(static_cast<int>(i));
    projections.emplace_back(u, v, camera_point.z());
  }
  if (cropped->size() < static_cast<std::size_t>(p.min_points)) return std::nullopt;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cropped);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
  extraction.setClusterTolerance(p.cluster_tolerance);
  extraction.setMinClusterSize(p.min_points);
  extraction.setMaxClusterSize(static_cast<int>(cropped->size()));
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cropped);
  std::vector<pcl::PointIndices> clusters;
  extraction.extract(clusters);

  std::optional<ProjectedSeed> best;
  double best_score = std::numeric_limits<double>::infinity();
  for (const auto & cluster : clusters) {
    Eigen::Vector3d low = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d high = -low;
    Eigen::Vector2d uv_low = low.head<2>(), uv_high = high.head<2>();
    std::vector<double> depths;
    for (int index : cluster.indices) {
      const auto & point = cropped->points[index];
      const Eigen::Vector3d position(point.x, point.y, point.z);
      low = low.cwiseMin(position);
      high = high.cwiseMax(position);
      uv_low = uv_low.cwiseMin(projections[index].head<2>());
      uv_high = uv_high.cwiseMax(projections[index].head<2>());
      depths.push_back(projections[index].z());
    }
    const double extent = (high - low).maxCoeff();
    const double coverage = (uv_high - uv_low).prod() / ((xmax - xmin) * (ymax - ymin));
    if (extent < p.min_extent || extent > p.max_extent || coverage < p.min_box_coverage) continue;
    const Eigen::Vector2d centre = 0.5 * (uv_low + uv_high);
    const double centre_offset = std::hypot(
      (centre.x() - 0.5 * (xmin + xmax)) / (xmax - xmin),
      (centre.y() - 0.5 * (ymin + ymax)) / (ymax - ymin));
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    const double depth = depths[depths.size() / 2];
    // Choose a supported foreground depth component. Never average all points
    // in the 2D rectangle, which commonly includes a larger wall behind it.
    const double score = depth + 0.5 * centre_offset;
    if (score >= best_score) continue;
    best_score = score;
    best = ProjectedSeed{};
    best->depth = depth;
    for (int index : cluster.indices) best->indices.push_back(original_indices[index]);
  }
  return best;
}
}  // namespace person_tracker
