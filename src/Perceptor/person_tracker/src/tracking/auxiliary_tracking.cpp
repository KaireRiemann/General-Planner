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

void MappingRos::updateAuxiliaryTracks(
  const std::vector<Detection> & candidates,
  const std::unordered_set<std::size_t> & excluded_candidates, double stamp_sec)
{
  if (!params_.crossing_enabled || !manual_target_selected_ || trackers_.empty()) {
    auxiliary_tracks_.clear();
    return;
  }

  std::vector<bool> candidate_used(candidates.size(), false);
  for (const std::size_t index : excluded_candidates) {
    if (index < candidate_used.size()) {
      candidate_used[index] = true;
    }
  }

  for (auto & auxiliary : auxiliary_tracks_) {
    int best_index = -1;
    double best_cost = params_.crossing_shadow_association_distance;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      if (
        candidate_used[i] ||
        !compatibleTargetSize(candidates[i].size, auxiliary.observed_size))
      {
        continue;
      }
      const double cost = crossingAssociationCost(
        *auxiliary.filter, candidates[i], auxiliary.observed_size,
        auxiliary.observed_point_count);
      const double target_cost = crossingAssociationCost(
        *trackers_.front(), candidates[i], target_observed_size_,
        target_observed_point_count_);
      if (target_cost + 0.08 < cost) {
        continue;
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_index = static_cast<int>(i);
      }
    }
    if (best_index < 0) {
      auxiliary.filter->markMissed();
      auxiliary.last_candidate_index = -1;
      auxiliary.last_candidate_stamp = stamp_sec;
      continue;
    }
    const auto & detection = candidates[static_cast<std::size_t>(best_index)];
    auxiliary.filter->updateWithMeasurement(
      detection.position, detection.size, stamp_sec,
      params_.selection_measurement_velocity_blend,
      params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
      params_.selection_max_speed, params_.acceleration_smoothing,
      params_.turn_rate_smoothing, params_.maximum_acceleration,
      params_.maximum_turn_rate);
    updateObservedSignature(
      auxiliary.observed_size, auxiliary.observed_point_count, detection);
    auxiliary.last_candidate_index = best_index;
    auxiliary.last_candidate_stamp = stamp_sec;
    candidate_used[static_cast<std::size_t>(best_index)] = true;
  }

  auxiliary_tracks_.erase(
    std::remove_if(
      auxiliary_tracks_.begin(), auxiliary_tracks_.end(),
      [&](const AuxiliaryTrack & track) {
        return track.filter->missed() > params_.crossing_shadow_max_missed_frames;
      }),
    auxiliary_tracks_.end());

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (
      candidate_used[i] ||
      !compatibleTargetSize(candidates[i].size, target_observed_size_) ||
      (params_.crossing_shadow_dynamic_only && !candidates[i].dynamic) ||
      static_cast<int>(auxiliary_tracks_.size()) >= params_.crossing_max_shadow_tracks)
    {
      continue;
    }
    const double target_distance =
      (candidates[i].position.head<2>() - trackers_.front()->associationPosition().head<2>()).norm();
    if (
      target_distance < params_.selection_roi_radius_xy ||
      target_distance > params_.crossing_shadow_spawn_radius ||
      trackers_.front()->associationDistance(candidates[i].position) <
      params_.selection_association_max_distance)
    {
      continue;
    }
    bool duplicate = false;
    for (const auto & auxiliary : auxiliary_tracks_) {
      if (
        (candidates[i].position.head<2>() - auxiliary.filter->associationPosition().head<2>()).norm() <
        0.5 * params_.crossing_shadow_association_distance)
      {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      continue;
    }
    AuxiliaryTrack track;
    track.filter = std::make_shared<TargetEkf>(
      next_track_id_++, candidates[i].position, candidates[i].size, stamp_sec,
      params_.acceleration_noise, params_.measurement_noise,
      params_.selection_adaptive_size_enabled, params_.selection_size_smoothing,
      params_.selection_size_max_relative_step);
    track.observed_size = candidates[i].size;
    track.observed_point_count = static_cast<double>(candidates[i].indices.indices.size());
    track.last_candidate_index = static_cast<int>(i);
    track.last_candidate_stamp = stamp_sec;
    auxiliary_tracks_.push_back(std::move(track));
  }
}

}  // namespace person_tracker
