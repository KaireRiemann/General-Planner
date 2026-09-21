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

double MappingRos::seedDistanceSquared(
  const PointType & point, const TargetEkf & tracker) const
{
  double best = std::numeric_limits<double>::infinity();
  // Ordinary support checks use only the short stop seed. The longer history
  // seed is deliberately limited to splitMergedDetection(): using it here can
  // make an auxiliary track claim the target's new post-crossing cluster.
  for (const auto & seed : tracker.motionSeeds()) {
    const double dx = static_cast<double>(point.x) - seed.x();
    const double dy = static_cast<double>(point.y) - seed.y();
    best = std::min(best, dx * dx + dy * dy);
  }
  return best;
}

bool MappingRos::detectionSupportsTrack(
  const Detection & detection, const PointCloudPtr & cloud,
  const TargetEkf & tracker) const
{
  const double radius_squared =
    params_.crossing_seed_support_radius * params_.crossing_seed_support_radius;
  int support = 0;
  for (const int index : detection.indices.indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
      continue;
    }
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    if (
      seedDistanceSquared(point, tracker) <= radius_squared &&
      std::abs(static_cast<double>(point.z) - tracker.associationPosition().z()) <=
      params_.selection_roi_half_height)
    {
      ++support;
      if (support >= params_.crossing_seed_min_points) {
        return true;
      }
    }
  }
  return false;
}

bool MappingRos::splitMergedDetection(
  const Detection & merged, const PointCloudPtr & cloud,
  const TargetEkf & target, const TargetEkf & distractor,
  Detection & target_part, Detection & distractor_part) const
{
  std::vector<int> target_indices;
  std::vector<int> distractor_indices;
  target_indices.reserve(merged.indices.indices.size());
  distractor_indices.reserve(merged.indices.indices.size());
  const auto target_seeds = target.motionSeeds(params_.crossing_recovery_gate);
  const auto distractor_seeds = distractor.motionSeeds(params_.crossing_recovery_gate);

  const auto assign_points = [&](bool reject_ambiguous) {
      target_indices.clear();
      distractor_indices.clear();
      for (const int index : merged.indices.indices) {
        if (index < 0 || static_cast<std::size_t>(index) >= cloud->size()) {
          continue;
        }
        const auto & point = cloud->points[static_cast<std::size_t>(index)];
        const Eigen::Vector3d position(point.x, point.y, point.z);
        const int assignment = crossing_logic::assignSeedFamily(
          position, target_seeds, distractor_seeds,
          reject_ambiguous ? params_.crossing_seed_assignment_margin : 0.0);
        if (assignment < 0) {
          continue;
        }
        if (assignment == 0) {
          target_indices.push_back(index);
        } else {
          distractor_indices.push_back(index);
        }
      }
    };

  assign_points(true);
  if (
    static_cast<int>(target_indices.size()) < params_.crossing_seed_min_points ||
    static_cast<int>(distractor_indices.size()) < params_.crossing_seed_min_points)
  {
    // Sparse Mid360 returns sometimes leave no points after removing the
    // bisector band. Fall back to a strict nearest-seed Voronoi partition.
    assign_points(false);
  }
  return buildDetectionFromIndices(cloud, target_indices, target_part) &&
         buildDetectionFromIndices(cloud, distractor_indices, distractor_part);
}

MergeAssignment MappingRos::splitMergedDetectionV1(
  const Detection & merged, const PointCloudPtr & cloud,
  const TargetEkf & target, const TargetEkf & distractor,
  const TrackSupportMemory & target_memory,
  const TrackSupportMemory & distractor_memory) const
{
  return merge_v1_matcher_.assignMergedPoints(
    *cloud, merged.indices.indices, target.associationPosition(), distractor.associationPosition(),
    target_memory, distractor_memory,
    sensorObservation(target.associationPosition()), sensorObservation(distractor.associationPosition()));
}

double MappingRos::crossingAssociationCost(
  const TargetEkf & tracker, const Detection & detection,
  const Eigen::Vector3d & signature_size, double signature_points) const
{
  double cost = tracker.associationDistance(detection.position);
  cost += params_.crossing_position_continuity_weight *
    (detection.position.head<2>() - tracker.associationPosition().head<2>()).norm();
  if (signature_size.minCoeff() > 0.0) {
    const double expected_width = std::max(signature_size.x(), signature_size.y());
    const double measured_width = std::max(detection.size.x(), detection.size.y());
    const double size_error =
      std::abs(measured_width - expected_width) +
      0.5 * std::abs(detection.size.z() - signature_size.z());
    cost += params_.crossing_signature_size_weight * size_error;
  }
  if (signature_points > 0.5 && !detection.indices.indices.empty()) {
    const double point_ratio =
      static_cast<double>(detection.indices.indices.size()) / signature_points;
    cost += params_.crossing_signature_points_weight * std::abs(std::log(
      std::clamp(point_ratio, 0.1, 10.0)));
  }
  return cost;
}

bool MappingRos::auxiliaryTrackReliable(const AuxiliaryTrack & auxiliary) const
{
  return auxiliary.filter != nullptr &&
         auxiliary.filter->hits() >= params_.crossing_shadow_min_confirmed_hits &&
         auxiliary.filter->missed() <= params_.crossing_shadow_max_missed_frames &&
         auxiliary.filter->velocity().head<2>().norm() >= params_.crossing_shadow_min_speed;
}

int MappingRos::closestAuxiliaryTrack() const
{
  if (trackers_.empty()) {
    return -1;
  }
  int best_index = -1;
  double best_distance = params_.crossing_interaction_distance;
  for (std::size_t i = 0; i < auxiliary_tracks_.size(); ++i) {
    if (!auxiliaryTrackReliable(auxiliary_tracks_[i])) {
      continue;
    }
    const double distance =
      (auxiliary_tracks_[i].filter->associationPosition().head<2>() -
      trackers_.front()->associationPosition().head<2>()).norm();
    if (distance <= best_distance) {
      best_distance = distance;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}

int MappingRos::findMergedCandidate(
  const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
  int auxiliary_index) const
{
  if (
    trackers_.empty() || auxiliary_index < 0 ||
    static_cast<std::size_t>(auxiliary_index) >= auxiliary_tracks_.size())
  {
    return -1;
  }
  const auto & target = *trackers_.front();
  const auto & distractor = *auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)].filter;
  const double track_separation =
    (target.associationPosition().head<2>() - distractor.associationPosition().head<2>()).norm();
  if (
    track_separation > params_.crossing_interaction_distance ||
    ((!params_.merge_v1_enabled || !crossing_occlusion_active_) &&
    track_separation < params_.crossing_min_track_separation))
  {
    return -1;
  }

  int best_index = -1;
  std::size_t best_points = 0U;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates[i];
    const double extent = std::max(candidate.size.x(), candidate.size.y());
    if (params_.merge_v1_enabled && crossing_occlusion_active_) {
      if (
        static_cast<int>(candidate.indices.indices.size()) >=
        params_.merge_v1_matcher.partial_min_points &&
        (detectionSupportsTrack(candidate, cloud, target) ||
        detectionSupportsTrack(candidate, cloud, distractor)) &&
        candidate.indices.indices.size() > best_points)
      {
        best_index = static_cast<int>(i);
        best_points = candidate.indices.indices.size();
      }
      continue;
    }
    if (
      static_cast<int>(candidate.indices.indices.size()) <
      2 * params_.crossing_seed_min_points ||
      extent < params_.crossing_merged_min_extent_xy)
    {
      continue;
    }
    Detection target_part;
    Detection distractor_part;
    if (!splitMergedDetection(
        candidate, cloud, target, distractor, target_part, distractor_part))
    {
      continue;
    }
    const double split_separation =
      (target_part.position.head<2>() - distractor_part.position.head<2>()).norm();
    const double smaller_fraction = static_cast<double>(std::min(
      target_part.indices.indices.size(), distractor_part.indices.indices.size())) /
      std::max<std::size_t>(1U, candidate.indices.indices.size());
    if (
      split_separation >= params_.crossing_min_track_separation &&
      smaller_fraction >= 0.12 && candidate.indices.indices.size() > best_points)
    {
      best_index = static_cast<int>(i);
      best_points = candidate.indices.indices.size();
    }
  }
  return best_index;
}

bool MappingRos::findRecoveryPair(
  const std::vector<Detection> & candidates, const TargetEkf & target,
  const TargetEkf & distractor, int & first, int & second) const
{
  first = -1;
  second = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (
      !compatibleTargetSize(candidates[i].size, target.size()) &&
      !compatibleTargetSize(candidates[i].size, distractor.size()))
    {
      continue;
    }
    for (std::size_t j = i + 1U; j < candidates.size(); ++j) {
      if (
        !compatibleTargetSize(candidates[j].size, target.size()) &&
        !compatibleTargetSize(candidates[j].size, distractor.size()))
      {
        continue;
      }
      if (
        (candidates[i].position.head<2>() - candidates[j].position.head<2>()).norm() <
        params_.crossing_split_min_separation)
      {
        continue;
      }
      const double target_i = target.associationDistance(candidates[i].position);
      const double target_j = target.associationDistance(candidates[j].position);
      const double distractor_i = distractor.associationDistance(candidates[i].position);
      const double distractor_j = distractor.associationDistance(candidates[j].position);
      const double assignment_cost = std::min(
        target_i + distractor_j, target_j + distractor_i);
      if (
        std::min(target_i, target_j) > params_.crossing_recovery_gate ||
        std::min(distractor_i, distractor_j) > params_.crossing_recovery_gate ||
        assignment_cost >= best_cost)
      {
        continue;
      }
      first = static_cast<int>(i);
      second = static_cast<int>(j);
      best_cost = assignment_cost;
    }
  }
  return first >= 0 && second >= 0;
}

}  // namespace person_tracker
