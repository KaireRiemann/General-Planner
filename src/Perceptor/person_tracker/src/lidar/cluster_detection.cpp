#include "lidar/body_center.hpp"
#include "core/mapping_manager.h"

#include "tracking/crossing_logic.hpp"

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

#include "common/tracking_geometry.hpp"

namespace person_tracker
{

using detail::orientationInvariantClusterSize;

std::vector<pcl::PointIndices> MappingRos::clusterCloud(const PointCloudPtr & cloud)
{
  std::vector<pcl::PointIndices> clusters;
  if (static_cast<int>(cloud->size()) < params_.min_cluster_size) {
    return clusters;
  }

  PointCloudPtr clustering_cloud = cloud;
  if (params_.cluster_in_xy) {
    clustering_cloud.reset(new PointCloud(*cloud));
    for (auto & point : clustering_cloud->points) {
      point.z = 0.0F;
    }
  }

  dbscan_.setCorePointMinPts(params_.dbscan_core_points);
  dbscan_.setClusterTolerance(params_.cluster_tolerance);
  dbscan_.setMinClusterSize(params_.min_cluster_size);
  dbscan_.setMaxClusterSize(params_.max_cluster_size);
  dbscan_.setInputCloud(clustering_cloud);
  dbscan_.setSearchMethod();
  dbscan_.extractNano(clusters);
  return clusters;
}

std::vector<pcl::PointIndices> MappingRos::refineInteractionClusters(
  const PointCloudPtr & cloud,
  const std::vector<pcl::PointIndices> & coarse_clusters) const
{
  if (
    !params_.interaction_refinement_enabled || trackers_.empty() ||
    auxiliary_tracks_.empty())
  {
    return coarse_clusters;
  }

  std::vector<const AuxiliaryTrack *> reliable_auxiliaries;
  reliable_auxiliaries.reserve(auxiliary_tracks_.size());
  for (const auto & auxiliary : auxiliary_tracks_) {
    if (
      auxiliaryTrackReliable(auxiliary) &&
      (auxiliary.filter->associationPosition().head<2>() -
      trackers_.front()->associationPosition().head<2>()).norm() <=
      params_.crossing_interaction_distance)
    {
      reliable_auxiliaries.push_back(&auxiliary);
    }
  }
  if (reliable_auxiliaries.empty()) {
    return coarse_clusters;
  }

  std::vector<pcl::PointIndices> refined_clusters;
  refined_clusters.reserve(coarse_clusters.size() + 4U);
  for (const auto & coarse : coarse_clusters) {
    Detection coarse_detection;
    if (!buildDetectionFromIndices(cloud, coarse.indices, coarse_detection)) {
      refined_clusters.push_back(coarse);
      continue;
    }
    const double coarse_extent = std::max(
      coarse_detection.size.x(), coarse_detection.size.y());
    if (
      coarse_extent < params_.crossing_merged_min_extent_xy ||
      !detectionSupportsTrack(coarse_detection, cloud, *trackers_.front()))
    {
      refined_clusters.push_back(coarse);
      continue;
    }

    const AuxiliaryTrack * interacting_auxiliary = nullptr;
    for (const auto * auxiliary : reliable_auxiliaries) {
      if (detectionSupportsTrack(coarse_detection, cloud, *auxiliary->filter)) {
        interacting_auxiliary = auxiliary;
        break;
      }
    }
    if (interacting_auxiliary == nullptr) {
      refined_clusters.push_back(coarse);
      continue;
    }

    PointCloudPtr local_cloud(new PointCloud);
    std::vector<int> local_to_global;
    local_cloud->reserve(coarse.indices.size());
    local_to_global.reserve(coarse.indices.size());
    for (const int global_index : coarse.indices) {
      if (global_index < 0 || static_cast<std::size_t>(global_index) >= cloud->size()) {
        continue;
      }
      PointType point = cloud->points[static_cast<std::size_t>(global_index)];
      if (params_.cluster_in_xy) {
        point.z = 0.0F;
      }
      local_cloud->push_back(point);
      local_to_global.push_back(global_index);
    }
    if (
      static_cast<int>(local_cloud->size()) <
      2 * params_.interaction_refinement_min_cluster_size)
    {
      refined_clusters.push_back(coarse);
      continue;
    }

    DBSCANKdtreeCluster<PointType> fine_dbscan;
    fine_dbscan.setCorePointMinPts(params_.interaction_refinement_core_points);
    fine_dbscan.setClusterTolerance(params_.interaction_refinement_tolerance);
    fine_dbscan.setMinClusterSize(params_.interaction_refinement_min_cluster_size);
    fine_dbscan.setMaxClusterSize(params_.max_cluster_size);
    fine_dbscan.setInputCloud(local_cloud);
    fine_dbscan.setSearchMethod();
    std::vector<pcl::PointIndices> local_clusters;
    fine_dbscan.extractNano(local_clusters);
    if (local_clusters.size() < 2U) {
      refined_clusters.push_back(coarse);
      continue;
    }

    std::vector<pcl::PointIndices> global_children;
    global_children.reserve(local_clusters.size());
    std::size_t covered_points = 0U;
    for (const auto & local_cluster : local_clusters) {
      pcl::PointIndices child;
      child.indices.reserve(local_cluster.indices.size());
      for (const int local_index : local_cluster.indices) {
        if (
          local_index >= 0 &&
          static_cast<std::size_t>(local_index) < local_to_global.size())
        {
          child.indices.push_back(local_to_global[static_cast<std::size_t>(local_index)]);
        }
      }
      if (
        static_cast<int>(child.indices.size()) >=
        params_.interaction_refinement_min_cluster_size)
      {
        covered_points += child.indices.size();
        global_children.push_back(std::move(child));
      }
    }
    const double coverage = static_cast<double>(covered_points) /
      std::max<std::size_t>(1U, coarse.indices.size());
    if (
      global_children.size() < 2U ||
      coverage < params_.interaction_refinement_min_coverage)
    {
      refined_clusters.push_back(coarse);
      continue;
    }

    bool found_distinct_people = false;
    for (std::size_t i = 0; i < global_children.size() && !found_distinct_people; ++i) {
      Detection target_child;
      if (
        !buildDetectionFromIndices(cloud, global_children[i].indices, target_child) ||
        !detectionSupportsTrack(target_child, cloud, *trackers_.front()))
      {
        continue;
      }
      for (std::size_t j = 0; j < global_children.size(); ++j) {
        if (i == j) {
          continue;
        }
        Detection auxiliary_child;
        if (
          !buildDetectionFromIndices(cloud, global_children[j].indices, auxiliary_child) ||
          !detectionSupportsTrack(
            auxiliary_child, cloud, *interacting_auxiliary->filter))
        {
          continue;
        }
        if (
          (target_child.position.head<2>() - auxiliary_child.position.head<2>()).norm() >=
          params_.crossing_split_min_separation)
        {
          found_distinct_people = true;
          break;
        }
      }
    }

    if (found_distinct_people) {
      refined_clusters.insert(
        refined_clusters.end(), global_children.begin(), global_children.end());
    } else {
      refined_clusters.push_back(coarse);
    }
  }
  std::sort(
    refined_clusters.begin(), refined_clusters.end(),
    [](const pcl::PointIndices & left, const pcl::PointIndices & right) {
      return left.indices.size() > right.indices.size();
    });
  return refined_clusters;
}

bool MappingRos::looksLikePerson(const Eigen::Vector3d & size) const
{
  if (!params_.human_filter_enabled) {
    return true;
  }
  const double horizontal_extent = std::max(size.x(), size.y());
  return size.z() >= params_.human_min_height &&
         size.z() <= params_.human_max_height + params_.human_measurement_tolerance &&
         horizontal_extent >= params_.human_min_width &&
         horizontal_extent <= params_.human_max_width + params_.human_measurement_tolerance;
}

bool MappingRos::compatibleTargetSize(
  const Eigen::Vector3d & measured, const Eigen::Vector3d & reference) const
{
  // Use only the long horizontal extent and height. The short side is often
  // incomplete in one Mid360 scan, while these two dimensions still reject
  // small moving fragments and oversized mixed clusters without assuming that
  // the clicked target is a person.
  const double reference_width = std::max(
    params_.map_resolution, std::max(reference.x(), reference.y()));
  const double measured_width = std::max(
    params_.map_resolution, std::max(measured.x(), measured.y()));
  const double reference_height = std::max(params_.map_resolution, reference.z());
  const double measured_height = std::max(params_.map_resolution, measured.z());
  const double width_ratio = measured_width / reference_width;
  const double height_ratio = measured_height / reference_height;
  return width_ratio >= params_.selection_size_compatibility_min_ratio &&
         width_ratio <= params_.selection_size_compatibility_max_ratio &&
         height_ratio >= params_.selection_size_compatibility_min_ratio &&
         height_ratio <= params_.selection_size_compatibility_max_ratio;
}

bool MappingRos::isNearMovingTrack(const Eigen::Vector3d & position) const
{
  return std::any_of(
    trackers_.begin(), trackers_.end(),
    [&](const std::shared_ptr<TargetEkf> & tracker) {
      return tracker->hits() >= params_.min_confirmed_hits &&
             tracker->velocity().norm() >= params_.track_keep_min_speed &&
             (tracker->associationPosition() - position).norm() <= params_.track_keep_distance;
    });
}

std::vector<Detection> MappingRos::buildDetections(
  const PointCloudPtr & cloud, const std::vector<pcl::PointIndices> & clusters,
  DetectionStats * stats, bool apply_detection_filters) const
{
  std::vector<Detection> detections;
  detections.reserve(clusters.size());

  for (const auto & cluster_indices : clusters) {
    if (cluster_indices.indices.empty()) {
      continue;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    double centroid_weight = 0.0;
    std::vector<double> distances;
    distances.reserve(cluster_indices.indices.size());

    for (const int index : cluster_indices.indices) {
      const auto & point = cloud->points[static_cast<std::size_t>(index)];
      const Eigen::Vector3d position(point.x, point.y, point.z);
      const double point_weight = targetPointWeight(point);
      centroid += point_weight * position;
      centroid_weight += point_weight;

      if (background_tree_->Root_Node != nullptr) {
        PointVector nearest_points;
        std::vector<float> squared_distances;
        background_tree_->Nearest_Search(point, 1, nearest_points, squared_distances);
        if (!nearest_points.empty() && !squared_distances.empty()) {
          distances.push_back(std::sqrt(std::max(0.0F, squared_distances.front())));
        }
      }
    }

    centroid /= std::max(centroid_weight, 1e-6);
    const Eigen::Vector3d size = orientationInvariantClusterSize(
      cloud, cluster_indices.indices, params_.map_resolution);
    if (apply_detection_filters && !looksLikePerson(size)) {
      if (stats != nullptr) {
        ++stats->shape_rejected;
      }
      continue;
    }

    double mean_distance = 0.0;
    double variance = 0.0;
    if (!distances.empty()) {
      mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0) /
        static_cast<double>(distances.size());
      for (const double distance : distances) {
        const double residual = distance - mean_distance;
        variance += residual * residual;
      }
      variance /= static_cast<double>(distances.size()) *
        std::max(mean_distance * mean_distance, 1e-6);
    }

    const bool dynamic = !distances.empty() &&
      mean_distance > params_.dynamic_distance_threshold &&
      mean_distance < params_.dynamic_max_distance &&
      variance < params_.dynamic_variance_threshold;
    if (
      apply_detection_filters && params_.dynamic_only && !dynamic &&
      !isNearMovingTrack(centroid))
    {
      if (stats != nullptr) {
        ++stats->dynamic_rejected;
      }
      continue;
    }

    Detection detection;
    detection.position = centroid;
    detection.size = size;
    detection.indices = cluster_indices;
    detection.mean_background_distance = mean_distance;
    detection.normalized_distance_variance = variance;
    detection.dynamic = dynamic;
    estimateBodyCenter(cloud, detection);
    detections.push_back(std::move(detection));
  }
  return detections;
}

bool MappingRos::buildRoiDetection(
  const PointCloudPtr & cloud, const Eigen::Vector3d & center,
  double radius_xy, double half_height, Detection & detection) const
{
  detection = Detection{};
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double centroid_weight = 0.0;
  const double radius_squared = radius_xy * radius_xy;

  for (std::size_t index = 0; index < cloud->size(); ++index) {
    const auto & point = cloud->points[index];
    const Eigen::Vector3d position(point.x, point.y, point.z);
    const Eigen::Vector3d offset = position - center;
    if (
      offset.x() * offset.x() + offset.y() * offset.y() > radius_squared ||
      std::abs(offset.z()) > half_height)
    {
      continue;
    }
    detection.indices.indices.push_back(static_cast<int>(index));
    const double point_weight = targetPointWeight(point);
    centroid += point_weight * position;
    centroid_weight += point_weight;
  }

  if (static_cast<int>(detection.indices.indices.size()) < params_.selection_roi_min_points) {
    return false;
  }
  centroid /= std::max(centroid_weight, 1e-6);
  detection.position = centroid;
  detection.size = orientationInvariantClusterSize(
    cloud, detection.indices.indices, params_.map_resolution);
  estimateBodyCenter(cloud, detection);
  return true;
}

bool MappingRos::buildDetectionFromIndices(
  const PointCloudPtr & cloud, const std::vector<int> & indices,
  Detection & detection) const
{
  detection = Detection{};
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double centroid_weight = 0.0;
  for (const int index : indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
      continue;
    }
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    const Eigen::Vector3d position(point.x, point.y, point.z);
    const double weight = targetPointWeight(point);
    centroid += weight * position;
    centroid_weight += weight;
    detection.indices.indices.push_back(index);
  }
  if (static_cast<int>(detection.indices.indices.size()) < params_.crossing_seed_min_points) {
    return false;
  }
  detection.position = centroid / std::max(centroid_weight, 1e-6);
  detection.size = orientationInvariantClusterSize(
    cloud, detection.indices.indices, params_.map_resolution);
  estimateBodyCenter(cloud, detection);
  return true;
}

void MappingRos::estimateBodyCenter(const PointCloudPtr & cloud, Detection & detection) const
{
  if (!params_.body_center_enabled || !odom_position_.allFinite()) return;
  std::vector<Eigen::Vector3d> points;
  points.reserve(detection.indices.indices.size());
  for (int index : detection.indices.indices) {
    if (index<0 || static_cast<std::size_t>(index)>=cloud->size()) continue;
    const auto & p=cloud->points[index]; points.emplace_back(p.x,p.y,p.z);
  }
  const auto free_fit=fitBodyCenter(points,detection.position.head<2>(),odom_position_.head<2>());
  if (free_fit.valid) detection.body_fit_radius=free_fit.radius;
  const bool remembered = !trackers_.empty() && trackers_.front()->id()==body_radius_target_id_ &&
    body_radius_>0 && last_cloud_stamp_sec_>=body_radius_stamp_ && last_cloud_stamp_sec_-body_radius_stamp_<=4.0 &&
    (detection.position.head<2>()-trackers_.front()->associationPosition().head<2>()).norm()<1.2;
  auto fit=free_fit;
  if (remembered) {
    const auto constrained=fitBodyCenterWithRadius(points,detection.position.head<2>(),odom_position_.head<2>(),body_radius_);
    if (constrained.valid) fit=constrained;
  }
  Eigen::Vector2d ray=detection.position.head<2>()-odom_position_.head<2>();
  if(ray.norm()<0.5)return;
  ray.normalize();const Eigen::Vector2d tangent(-ray.y(),ray.x());
  Eigen::Vector2d correction=Eigen::Vector2d::Zero();
  if(fit.valid){
    const Eigen::Vector2d delta=fit.center-detection.position.head<2>();
    correction=Eigen::Vector2d(delta.dot(ray),delta.dot(tangent));
  }
  if(remembered && body_center_offset_.x()>0.0){
    // Treat the shape-derived offset as a slowly changing observation bias.
    // Missing arc support does not redefine an already tracked body's origin.
    if(fit.valid){
      const double dt=std::clamp(last_cloud_stamp_sec_-last_target_measurement_stamp_sec_,0.01,0.2);
      Eigen::Vector2d change=(1.0-std::exp(-dt/0.6))*(correction-body_center_offset_);
      const double max_change=0.1*dt;
      if(change.norm()>max_change)change*=max_change/change.norm();
      correction=body_center_offset_+change;
    }else correction=body_center_offset_;
  }else if(!fit.valid)return;
  detection.body_center_offset=correction;
  detection.position.head<2>()+=correction.x()*ray+correction.y()*tangent;
}

void MappingRos::rememberBodyRadius(const Detection & detection, int target_id, double stamp_sec)
{
  if (!params_.body_center_enabled) return;
  if(target_id!=body_radius_target_id_ || stamp_sec-body_radius_stamp_>4.0){
    body_radius_target_id_=target_id;body_radius_=0.0;body_center_offset_.setZero();
  }
  if(detection.body_center_offset.x()>0.0)body_center_offset_=detection.body_center_offset;
  if(detection.body_fit_radius<=0.0)return;
  if(body_radius_<=0.0)body_radius_=detection.body_fit_radius;
  else if(std::abs(detection.body_fit_radius-body_radius_)<=0.08)
    body_radius_+=std::clamp(0.1*(detection.body_fit_radius-body_radius_),-0.005,0.005);
  else return;
  body_radius_stamp_=stamp_sec;
}

}  // namespace person_tracker
