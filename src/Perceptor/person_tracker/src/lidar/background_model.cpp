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

PointCloudPtr MappingRos::extractForegroundCloud(const PointCloudPtr & cloud) const
{
  PointCloudPtr foreground(new PointCloud);
  foreground->reserve(cloud->size());

  if (background_tree_->Root_Node == nullptr) {
    *foreground = *cloud;
    return foreground;
  }

  const double foreground_distance =
    params_.manual_selection_enabled && manual_target_selected_ ?
    params_.selection_tracking_background_distance : params_.dynamic_distance_threshold;
  for (const auto & point : cloud->points) {
    PointVector nearest_points;
    std::vector<float> squared_distances;
    background_tree_->Nearest_Search(point, 1, nearest_points, squared_distances);
    if (
      nearest_points.empty() || squared_distances.empty() ||
      std::sqrt(std::max(0.0F, squared_distances.front())) >
      foreground_distance)
    {
      foreground->push_back(point);
    }
  }

  foreground->width = foreground->size();
  foreground->height = 1;
  foreground->is_dense = true;
  return foreground;
}

double MappingRos::targetPointWeight(const PointType & point) const
{
  if (
    !params_.manual_selection_enabled || !manual_target_selected_ ||
    background_tree_->Root_Node == nullptr)
  {
    return 1.0;
  }
  PointVector nearest_points;
  std::vector<float> squared_distances;
  background_tree_->Nearest_Search(point, 1, nearest_points, squared_distances);
  if (squared_distances.empty()) {
    return 1.0;
  }
  const double distance = std::sqrt(std::max(0.0F, squared_distances.front()));
  const double low = params_.dynamic_distance_threshold;
  const double full = std::max(
    low + params_.map_resolution,
    0.5 * std::min(
      params_.selection_fixed_body_size.x(), params_.selection_fixed_body_size.y()));
  const double normalized = std::clamp((distance - low) / (full - low), 0.0, 1.0);
  // Never delete a point here. A wall-adjacent human return remains usable,
  // while a point only slightly beyond the FAPP threshold contributes much
  // less to the measured centre than a body point farther from the surface.
  return 0.15 + 0.85 * normalized;
}

double MappingRos::backgroundClearance(const Eigen::Vector3d & position) const
{
  if (background_tree_->Root_Node == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  PointType query;
  query.x = static_cast<float>(position.x());
  query.y = static_cast<float>(position.y());
  query.z = static_cast<float>(position.z());
  PointVector nearest_points;
  std::vector<float> squared_distances;
  background_tree_->Nearest_Search(query, 1, nearest_points, squared_distances);
  return squared_distances.empty() ? std::numeric_limits<double>::infinity() :
         std::sqrt(std::max(0.0F, squared_distances.front()));
}

bool MappingRos::insideTargetProtection(const Eigen::Vector3d & position) const
{
  if (!params_.manual_selection_enabled || !manual_target_selected_ || trackers_.empty()) {
    return false;
  }
  const auto inside_filter = [&](const TargetEkf & tracker) {
      const Eigen::Vector3d offset = position - tracker.associationPosition();
      const double radius_xy = std::max(
        params_.selection_roi_radius_xy,
        0.5 * std::max(tracker.size().x(), tracker.size().y()) + 0.20);
      const double half_height = std::max(
        params_.selection_roi_half_height, 0.5 * tracker.size().z() + 0.30);
      return offset.x() * offset.x() + offset.y() * offset.y() <= radius_xy * radius_xy &&
             std::abs(offset.z()) <= half_height;
    };
  if (inside_filter(*trackers_.front())) {
    return true;
  }
  for (const auto & auxiliary : auxiliary_tracks_) {
    if (inside_filter(*auxiliary.filter)) {
      return true;
    }
  }
  for (const auto & hypothesis : identity_hypotheses_) {
    if (
      inside_filter(*hypothesis.target_filter) ||
      inside_filter(*hypothesis.distractor_filter))
    {
      return true;
    }
  }
  return false;
}

void MappingRos::removeTargetFromTransientBackground()
{
  if (!manual_target_selected_ || trackers_.empty() || background_tree_->Root_Node == nullptr) {
    return;
  }

  // The first background scan is marked persistent.  Later background points
  // that have already stayed fixed for long enough are also treated as static
  // structure.  Only young points in the selected body gate are removed.  A
  // moving person can therefore be recovered after it was briefly absorbed by
  // the sliding background, without erasing a table or wall touching the body.
  std::unordered_map<VoxelIndex, PointType, VoxelIndexHash> unique_points_to_delete;
  for (auto & chunk : background_chunks_) {
    PointVector retained;
    retained.reserve(chunk.size());
    for (const auto & point : chunk) {
      const VoxelIndex index = positionToVoxel(Eigen::Vector3d(point.x, point.y, point.z));
      const auto first_seen = background_first_seen_.find(index);
      const bool stable_static =
        first_seen != background_first_seen_.end() &&
        cloud_frame_index_ >= first_seen->second &&
        cloud_frame_index_ - first_seen->second >=
        static_cast<std::uint64_t>(params_.selection_background_static_min_age_frames);
      if (
        persistent_background_indices_.find(index) == persistent_background_indices_.end() &&
        !stable_static &&
        insideTargetProtection(Eigen::Vector3d(point.x, point.y, point.z)))
      {
        unique_points_to_delete[index] = point;
      } else {
        retained.push_back(point);
      }
    }
    chunk = std::move(retained);
  }
  if (!unique_points_to_delete.empty()) {
    PointVector points_to_delete;
    points_to_delete.reserve(unique_points_to_delete.size());
    for (const auto & entry : unique_points_to_delete) {
      points_to_delete.push_back(entry.second);
      background_last_seen_.erase(entry.first);
      background_first_seen_.erase(entry.first);
    }
    deleteBackgroundVoxels(points_to_delete);
  }

  for (auto iterator = background_candidates_.begin(); iterator != background_candidates_.end();) {
    const auto & point = iterator->second.point;
    if (insideTargetProtection(Eigen::Vector3d(point.x, point.y, point.z))) {
      iterator = background_candidates_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

void MappingRos::updateBackground(const PointCloudPtr & cloud)
{
  if (cloud->empty()) {
    return;
  }

  // FAPP-style sliding background: current static samples are inserted into an
  // ikd-tree and remain for N frames. Unlike the previous implementation,
  // every repeated observation refreshes its expiry. Otherwise all non-base
  // walls vanished from the tree together every 25 frames and were cyclically
  // misclassified as dynamic. The selected target region is never refreshed.
  PointVector chunk;
  chunk.reserve(cloud->size());
  PointVector points_to_add;
  points_to_add.reserve(cloud->size());
  std::unordered_set<VoxelIndex, VoxelIndexHash> chunk_indices;
  chunk_indices.reserve(cloud->size());
  bool built_initial_background = false;
  if (background_tree_->Root_Node == nullptr) {
    for (const auto & point : cloud->points) {
      if (!insideTargetProtection(Eigen::Vector3d(point.x, point.y, point.z))) {
        chunk.push_back(point);
        persistent_background_indices_.insert(
          positionToVoxel(Eigen::Vector3d(point.x, point.y, point.z)));
      }
    }
    if (!chunk.empty()) {
      background_tree_->Build(chunk);
      built_initial_background = true;
    }
  } else {
    const double new_point_distance = 0.5 * params_.map_resolution;
    const double new_point_distance_squared = new_point_distance * new_point_distance;
    for (const auto & point : cloud->points) {
      const Eigen::Vector3d position(point.x, point.y, point.z);
      const VoxelIndex index = positionToVoxel(position);
      PointVector nearest_points;
      std::vector<float> squared_distances;
      background_tree_->Nearest_Search(point, 1, nearest_points, squared_distances);
      if (
        !nearest_points.empty() && !squared_distances.empty() &&
        squared_distances.front() <= new_point_distance_squared)
      {
        const auto & background_point = nearest_points.front();
        const VoxelIndex background_index = positionToVoxel(
          Eigen::Vector3d(background_point.x, background_point.y, background_point.z));
        if (
          persistent_background_indices_.find(background_index) ==
          persistent_background_indices_.end())
        {
          background_last_seen_[background_index] = cloud_frame_index_;
          if (chunk_indices.insert(background_index).second) {
            chunk.push_back(background_point);
          }
        }
        background_candidates_.erase(index);
        continue;
      }

      // Existing background is refreshed even inside the target gate.  This
      // is the key contact-case rule: an already known wall/table stays
      // background, while unmatched target returns are protected from being
      // learned and remain foreground when the person stops moving.
      if (insideTargetProtection(position)) {
        background_candidates_.erase(index);
        continue;
      }

      auto & candidate = background_candidates_[index];
      if (candidate.last_seen_frame + 1U == cloud_frame_index_) {
        ++candidate.consecutive_hits;
      } else {
        candidate.consecutive_hits = 1;
      }
      candidate.point = point;
      candidate.last_seen_frame = cloud_frame_index_;
      if (candidate.consecutive_hits >= params_.background_confirm_frames) {
        background_last_seen_[index] = cloud_frame_index_;
        background_first_seen_.try_emplace(index, cloud_frame_index_);
        if (chunk_indices.insert(index).second) {
          chunk.push_back(point);
          points_to_add.push_back(point);
        }
      }
    }

    for (auto iterator = background_candidates_.begin(); iterator != background_candidates_.end();) {
      if (
        iterator->second.last_seen_frame != cloud_frame_index_ ||
        iterator->second.consecutive_hits >= params_.background_confirm_frames)
      {
        iterator = background_candidates_.erase(iterator);
      } else {
        ++iterator;
      }
    }
    if (!points_to_add.empty()) {
      background_tree_->Add_Points(points_to_add, false);
    }
  }
  // The first scan remains the persistent base. Later chunks contain both new
  // points and refreshed sightings, allowing true time-based sliding expiry.
  background_chunks_.push_back(built_initial_background ? PointVector{} : chunk);

  while (static_cast<int>(background_chunks_.size()) > params_.background_history_frames) {
    PointVector expired = std::move(background_chunks_.front());
    background_chunks_.pop_front();
    if (!expired.empty()) {
      PointVector points_to_delete;
      points_to_delete.reserve(expired.size());
      const std::uint64_t history = static_cast<std::uint64_t>(params_.background_history_frames);
      for (const auto & point : expired) {
        const VoxelIndex index = positionToVoxel(Eigen::Vector3d(point.x, point.y, point.z));
        if (persistent_background_indices_.find(index) != persistent_background_indices_.end()) {
          continue;
        }
        const auto last_seen = background_last_seen_.find(index);
        if (
          last_seen != background_last_seen_.end() &&
          cloud_frame_index_ - last_seen->second >= history)
        {
          points_to_delete.push_back(point);
          background_last_seen_.erase(last_seen);
          background_first_seen_.erase(index);
        }
      }
      if (!points_to_delete.empty()) {
        deleteBackgroundVoxels(points_to_delete);
      }
    }
  }
}

}  // namespace person_tracker
