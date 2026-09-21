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

namespace person_tracker
{

void MappingRos::updateObservedSignature(
  Eigen::Vector3d & size_signature, double & point_signature,
  const Detection & detection) const
{
  if (size_signature.minCoeff() <= 0.0) {
    size_signature = detection.size;
  } else {
    size_signature = 0.80 * size_signature + 0.20 * detection.size;
  }
  const double points = static_cast<double>(detection.indices.indices.size());
  point_signature = point_signature <= 0.5 ? points : 0.80 * point_signature + 0.20 * points;
}

SensorObservation MappingRos::sensorObservation(const Eigen::Vector3d & center) const
{
  SensorObservation observation;
  if (!has_odom_) {
    return observation;
  }
  const Eigen::Vector3d sensor_relative =
    odom_orientation_.conjugate() * (center - odom_position_);
  observation.range = sensor_relative.norm();
  observation.bearing = std::atan2(sensor_relative.y(), sensor_relative.x());
  observation.valid = std::isfinite(observation.range) &&
    std::isfinite(observation.bearing);
  return observation;
}

bool MappingRos::supportMemoryReady(const TrackSupportMemory & memory) const
{
  return static_cast<int>(memory.templates.size()) >= params_.merge_v1_min_templates;
}

double MappingRos::identityAssociationCost(
  const TargetEkf & tracker, const Detection & detection,
  const Eigen::Vector3d & signature_size, double signature_points,
  const PointCloudPtr & cloud, const TrackSupportMemory & memory) const
{
  double cost = crossingAssociationCost(
    tracker, detection, signature_size, signature_points);
  if (
    !params_.merge_v1_enabled || cloud == nullptr ||
    !supportMemoryReady(memory))
  {
    return cost;
  }
  const double support = merge_v1_matcher_.clusterSupportScore(
    *cloud, detection.indices.indices, detection.position, memory,
    sensorObservation(detection.position));
  cost += params_.merge_v1_split_identity_support_weight * (1.0 - support);
  return cost;
}

void MappingRos::setSupportMemoriesFrozen(bool frozen)
{
  target_support_memory_.frozen = frozen;
  for (auto & auxiliary : auxiliary_tracks_) {
    auxiliary.support_memory.frozen = frozen;
  }
}

void MappingRos::updateCleanSupportMemories(
  const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
  double stamp_sec)
{
  if (!params_.merge_v1_enabled || !manual_target_selected_ || trackers_.empty()) {
    return;
  }
  const bool crossing_ambiguous =
    crossing_occlusion_active_ || crossing_pending_merge_frames_ > 0 ||
    !identity_hypotheses_.empty();
  if (crossing_ambiguous) {
    setSupportMemoriesFrozen(true);
    return;
  }
  if (support_memory_resume_countdown_ > 0) {
    --support_memory_resume_countdown_;
    setSupportMemoriesFrozen(true);
    return;
  }
  setSupportMemoriesFrozen(false);

  const auto valid_template = [&](
      const Detection & detection, const Eigen::Vector3d & expected_size,
      const TrackSupportMemory & memory) {
      if (
        static_cast<int>(detection.indices.indices.size()) <
        params_.merge_v1_min_template_points ||
        stamp_sec - memory.last_update_stamp < params_.merge_v1_min_template_interval)
      {
        return false;
      }
      const double expected_width = std::max(
        params_.map_resolution, std::max(expected_size.x(), expected_size.y()));
      const double measured_width = std::max(
        params_.map_resolution, std::max(detection.size.x(), detection.size.y()));
      const double expected_height = std::max(params_.map_resolution, expected_size.z());
      const double measured_height = std::max(params_.map_resolution, detection.size.z());
      const double size_ratio = std::max({
        expected_width / measured_width, measured_width / expected_width,
        expected_height / measured_height, measured_height / expected_height
      });
      return size_ratio <= params_.merge_v1_max_template_size_ratio;
    };

  int target_candidate = -1;
  double target_cost = params_.selection_association_max_distance;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto & candidate = candidates[index];
    if (!valid_template(candidate, target_observed_size_, target_support_memory_)) {
      continue;
    }
    bool shared_with_auxiliary = false;
    for (const auto & auxiliary : auxiliary_tracks_) {
      if (
        auxiliaryTrackReliable(auxiliary) &&
        detectionSupportsTrack(candidate, cloud, *auxiliary.filter))
      {
        shared_with_auxiliary = true;
        break;
      }
    }
    if (shared_with_auxiliary) {
      continue;
    }
    const double cost = trackers_.front()->associationDistance(candidate.position);
    if (cost < target_cost) {
      target_cost = cost;
      target_candidate = static_cast<int>(index);
    }
  }
  if (target_candidate >= 0) {
    const auto & candidate = candidates[static_cast<std::size_t>(target_candidate)];
    const double point_quality = std::clamp(
      static_cast<double>(candidate.indices.indices.size()) /
      std::max(1.0, target_observed_point_count_), 0.0, 1.0);
    const double motion_quality = std::exp(
      -0.5 * target_cost * target_cost /
      std::max(0.01, params_.selection_association_max_distance *
      params_.selection_association_max_distance));
    const double quality = 0.55 * motion_quality + 0.45 * point_quality;
    if (quality >= params_.merge_v1_min_template_quality) {
      merge_v1_matcher_.appendTemplate(
        target_support_memory_,
        merge_v1_matcher_.makeTemplate(
          *cloud, candidate.indices.indices, candidate.position, stamp_sec,
          sensorObservation(candidate.position), quality),
        static_cast<std::size_t>(params_.merge_v1_max_templates));
    }
  }

  for (auto & auxiliary : auxiliary_tracks_) {
    if (
      !auxiliaryTrackReliable(auxiliary) || auxiliary.last_candidate_index < 0 ||
      std::abs(auxiliary.last_candidate_stamp - stamp_sec) > 1e-4 ||
      static_cast<std::size_t>(auxiliary.last_candidate_index) >= candidates.size() ||
      auxiliary.last_candidate_index == target_candidate)
    {
      continue;
    }
    const auto & candidate = candidates[static_cast<std::size_t>(auxiliary.last_candidate_index)];
    if (
      !valid_template(candidate, auxiliary.observed_size, auxiliary.support_memory) ||
      detectionSupportsTrack(candidate, cloud, *trackers_.front()))
    {
      continue;
    }
    const double cost = auxiliary.filter->associationDistance(candidate.position);
    const double point_quality = std::clamp(
      static_cast<double>(candidate.indices.indices.size()) /
      std::max(1.0, auxiliary.observed_point_count), 0.0, 1.0);
    const double motion_quality = std::exp(
      -0.5 * cost * cost /
      std::max(0.01, params_.crossing_shadow_association_distance *
      params_.crossing_shadow_association_distance));
    const double quality = 0.55 * motion_quality + 0.45 * point_quality;
    if (quality < params_.merge_v1_min_template_quality) {
      continue;
    }
    merge_v1_matcher_.appendTemplate(
      auxiliary.support_memory,
      merge_v1_matcher_.makeTemplate(
        *cloud, candidate.indices.indices, candidate.position, stamp_sec,
        sensorObservation(candidate.position), quality),
      static_cast<std::size_t>(params_.merge_v1_max_templates));
  }
}

void MappingRos::resetMergeDebug()
{
  merge_debug_active_ = false;
  merge_debug_target_support_.clear();
  merge_debug_distractor_support_.clear();
  merge_debug_merged_.clear();
  merge_debug_target_assigned_.clear();
  merge_debug_distractor_assigned_.clear();
  merge_debug_unknown_.clear();
}

void MappingRos::fillMergeDebug(
  const Detection & merged, const PointCloudPtr & cloud,
  const MergeAssignment & assignment, const TargetEkf & target,
  const TargetEkf & distractor, const TrackSupportMemory & target_memory,
  const TrackSupportMemory & distractor_memory)
{
  if (!params_.merge_v1_debug_enabled || cloud == nullptr) {
    return;
  }
  merge_debug_active_ = true;
  merge_debug_target_prediction_ = target.associationPosition();
  merge_debug_distractor_prediction_ = distractor.associationPosition();
  merge_debug_target_visibility_ = assignment.target_evaluation.visibility;
  merge_debug_distractor_visibility_ = assignment.distractor_evaluation.visibility;
  merge_debug_target_support_ = merge_v1_matcher_.predictedSupportCloud(
    target_memory, target.associationPosition(), sensorObservation(target.associationPosition()));
  merge_debug_distractor_support_ = merge_v1_matcher_.predictedSupportCloud(
    distractor_memory, distractor.associationPosition(), sensorObservation(distractor.associationPosition()));

  const auto copy_indices = [&](const std::vector<int> & indices, PointCloud & output) {
      output.clear();
      output.reserve(indices.size());
      for (const int index : indices) {
        if (index >= 0 && static_cast<std::size_t>(index) < cloud->size()) {
          output.push_back(cloud->points[static_cast<std::size_t>(index)]);
        }
      }
      output.width = output.size();
      output.height = 1;
      output.is_dense = true;
    };
  copy_indices(merged.indices.indices, merge_debug_merged_);
  copy_indices(assignment.target_indices, merge_debug_target_assigned_);
  copy_indices(assignment.distractor_indices, merge_debug_distractor_assigned_);
  copy_indices(assignment.unknown_indices, merge_debug_unknown_);
}

}  // namespace person_tracker
