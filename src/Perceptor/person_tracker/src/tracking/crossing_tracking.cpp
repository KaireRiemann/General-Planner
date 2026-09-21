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

using detail::boundedHorizontalMeasurement;

bool MappingRos::handleCrossingState(
  const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
  double stamp_sec, std::vector<Detection> & selected_detection)
{
  if (!params_.crossing_enabled || trackers_.empty()) {
    return false;
  }
  if (!identity_hypotheses_.empty()) {
    crossing_pending_merge_frames_ = 0;
    crossing_pending_auxiliary_index_ = -1;
    setSupportMemoriesFrozen(true);
    advanceIdentityHypotheses(candidates, cloud, stamp_sec, selected_detection);
    return true;
  }

  int auxiliary_index = crossing_occlusion_active_ ?
    crossing_auxiliary_index_ : closestAuxiliaryTrack();
  if (
    auxiliary_index < 0 ||
    static_cast<std::size_t>(auxiliary_index) >= auxiliary_tracks_.size())
  {
    crossing_occlusion_active_ = false;
    crossing_occlusion_frames_ = 0;
    crossing_auxiliary_index_ = -1;
    crossing_pending_merge_frames_ = 0;
    crossing_pending_auxiliary_index_ = -1;
    support_memory_resume_countdown_ = params_.merge_v1_resume_delay_frames;
    return false;
  }

  const bool use_v1 = params_.merge_v1_enabled &&
    supportMemoryReady(target_support_memory_) &&
    supportMemoryReady(auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)].support_memory);

  // Once a V1 merge is active, prefer the existing two-cluster recovery state
  // machine before treating a single remaining cluster as a partial observation.
  if (use_v1 && crossing_occlusion_active_) {
    int first = -1;
    int second = -1;
    if (findRecoveryPair(
        candidates, *trackers_.front(),
        *auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)].filter,
        first, second))
    {
      startIdentityHypotheses(
        candidates, cloud, first, second, auxiliary_index, stamp_sec);
      if (!identity_hypotheses_.empty()) {
        const auto best = std::min_element(
          identity_hypotheses_.begin(), identity_hypotheses_.end(),
          [](const IdentityHypothesis & left, const IdentityHypothesis & right) {
            return left.cumulative_score < right.cumulative_score;
          });
        selected_detection.push_back(best->last_target_detection);
        ROS_INFO( "Merged people separated; retaining %zu V1 identity hypotheses",
          identity_hypotheses_.size());
      }
      return true;
    }
  }

  const int merged_index = findMergedCandidate(candidates, cloud, auxiliary_index);
  if (merged_index >= 0) {
    if (!crossing_occlusion_active_) {
      if (crossing_pending_auxiliary_index_ == auxiliary_index) {
        ++crossing_pending_merge_frames_;
      } else {
        crossing_pending_auxiliary_index_ = auxiliary_index;
        crossing_pending_merge_frames_ = 1;
      }
      if (crossing_pending_merge_frames_ < params_.crossing_merge_confirm_frames) {
        return false;
      }
      ROS_WARN(
        "Target and a confirmed moving auxiliary stayed in one DBSCAN cluster for %d frames; "
        "using %s point partitions (templates A=%zu B=%zu)",
        crossing_pending_merge_frames_, use_v1 ? "persistent-support V1" : "prediction-seeded v0",
        target_support_memory_.templates.size(),
        auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)].support_memory.templates.size());
    }
    crossing_occlusion_active_ = true;
    crossing_auxiliary_index_ = auxiliary_index;
    crossing_pending_merge_frames_ = 0;
    crossing_pending_auxiliary_index_ = -1;
    setSupportMemoriesFrozen(true);
    ++crossing_occlusion_frames_;

    auto & target = *trackers_.front();
    auto & auxiliary = auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)];
    auto & distractor = *auxiliary.filter;
    if (use_v1) {
      const auto & merged = candidates[static_cast<std::size_t>(merged_index)];
      const Eigen::Vector3d target_prediction = target.associationPosition();
      const Eigen::Vector3d distractor_prediction = distractor.associationPosition();
      const MergeAssignment assignment = splitMergedDetectionV1(
        merged, cloud, target, distractor,
        target_support_memory_, auxiliary.support_memory);
      fillMergeDebug(
        merged, cloud, assignment, target, distractor,
        target_support_memory_, auxiliary.support_memory);

      Detection target_part;
      Detection distractor_part;
      const bool target_part_valid = buildDetectionFromIndices(
        cloud, assignment.target_indices, target_part);
      const bool distractor_part_valid = buildDetectionFromIndices(
        cloud, assignment.distractor_indices, distractor_part);
      const bool target_visible = target_part_valid &&
        assignment.target_evaluation.visibility == TrackVisibility::VISIBLE;
      const bool distractor_visible = distractor_part_valid &&
        assignment.distractor_evaluation.visibility == TrackVisibility::VISIBLE;

      if (target_visible && params_.crossing_split_update_enabled) {
        const double correction =
          (target_part.position.head<2>() - target.associationPosition().head<2>()).norm();
        if (correction <= params_.crossing_recovery_gate) {
          Eigen::Vector3d measurement = boundedHorizontalMeasurement(
            target.associationPosition(), target_part.position,
            params_.crossing_split_update_max_correction);
          const Eigen::Vector3d target_size = params_.selection_fixed_body_size_enabled ?
            params_.selection_fixed_body_size : target_part.size;
          if (params_.selection_fixed_body_size_enabled) {
            measurement.z() = target.associationPosition().z();
            target_part.size = target_size;
          }
          target.updateWithMeasurement(
            measurement, target_size, stamp_sec,
            params_.crossing_split_velocity_blend,
            params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
            params_.selection_max_speed, params_.acceleration_smoothing,
            params_.turn_rate_smoothing, params_.maximum_acceleration,
            params_.maximum_turn_rate, false, false);
          last_target_measurement_position_ = measurement;
          last_target_measurement_stamp_sec_ = stamp_sec;
          has_last_target_measurement_ = true;
        }
      }

      if (distractor_visible && params_.crossing_split_update_enabled) {
        const double correction =
          (distractor_part.position.head<2>() - distractor.associationPosition().head<2>()).norm();
        if (correction <= params_.crossing_recovery_gate) {
          const Eigen::Vector3d measurement = boundedHorizontalMeasurement(
            distractor.associationPosition(), distractor_part.position,
            params_.crossing_split_update_max_correction);
          distractor.updateWithMeasurement(
            measurement, distractor_part.size, stamp_sec,
            params_.crossing_split_velocity_blend,
            params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
            params_.selection_max_speed, params_.acceleration_smoothing,
            params_.turn_rate_smoothing, params_.maximum_acceleration,
            params_.maximum_turn_rate, false, false);
        }
      }

      if (
        target_part_valid &&
        assignment.target_evaluation.visibility != TrackVisibility::OCCLUDED)
      {
        selected_detection.push_back(std::move(target_part));
      }
      ROS_INFO_THROTTLE(1.0,
        "[MERGE-V1] A pred=[%.2f %.2f %.2f] templates=%d assigned=%d "
        "mean=%.2f high=%d state=%s; B pred=[%.2f %.2f %.2f] templates=%d "
        "assigned=%d mean=%.2f high=%d state=%s; unknown=%zu",
        target_prediction.x(), target_prediction.y(), target_prediction.z(),
        assignment.target_selected_templates,
        assignment.target_evaluation.assigned_count,
        assignment.target_evaluation.mean_support,
        assignment.target_evaluation.high_confidence_count,
        visibilityName(assignment.target_evaluation.visibility),
        distractor_prediction.x(), distractor_prediction.y(), distractor_prediction.z(),
        assignment.distractor_selected_templates,
        assignment.distractor_evaluation.assigned_count,
        assignment.distractor_evaluation.mean_support,
        assignment.distractor_evaluation.high_confidence_count,
        visibilityName(assignment.distractor_evaluation.visibility),
        assignment.unknown_indices.size());
      return true;
    }

    Detection target_part;
    Detection distractor_part;
    if (splitMergedDetection(
        candidates[static_cast<std::size_t>(merged_index)], cloud,
        *trackers_.front(),
        *auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)].filter,
        target_part, distractor_part))
    {
      const double target_correction =
        (target_part.position.head<2>() - target.associationPosition().head<2>()).norm();
      const double distractor_correction =
        (distractor_part.position.head<2>() - distractor.associationPosition().head<2>()).norm();
      if (
        params_.crossing_split_update_enabled &&
        target_correction <= params_.crossing_recovery_gate &&
        distractor_correction <= params_.crossing_recovery_gate)
      {
        // A genuine U-turn can put a seed-owned partition 0.4--0.8 m behind
        // the CV prediction. Rejecting it outright made the prediction keep
        // walking away. Accept the partition inside the identity continuity
        // gate, but move each private filter by only the configured per-scan
        // correction so neither person can teleport to the other.
        Eigen::Vector3d target_measurement = target_part.position;
        if (
          target_correction > params_.crossing_split_update_max_correction &&
          target_correction > 1e-6)
        {
          target_measurement.head<2>() = target.associationPosition().head<2>() +
            params_.crossing_split_update_max_correction *
            (target_part.position.head<2>() - target.associationPosition().head<2>()) /
            target_correction;
        }
        Eigen::Vector3d distractor_measurement = distractor_part.position;
        if (
          distractor_correction > params_.crossing_split_update_max_correction &&
          distractor_correction > 1e-6)
        {
          distractor_measurement.head<2>() = distractor.associationPosition().head<2>() +
            params_.crossing_split_update_max_correction *
            (distractor_part.position.head<2>() - distractor.associationPosition().head<2>()) /
            distractor_correction;
        }
        const Eigen::Vector3d target_size = params_.selection_fixed_body_size_enabled ?
          params_.selection_fixed_body_size : target_part.size;
        target_measurement.z() = params_.selection_fixed_body_size_enabled ?
          target.associationPosition().z() : target_measurement.z();
        target_part.size = target_size;
        target.updateWithMeasurement(
          target_measurement, target_size, stamp_sec,
          params_.crossing_split_velocity_blend,
          params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
          params_.selection_max_speed, params_.acceleration_smoothing,
          params_.turn_rate_smoothing, params_.maximum_acceleration,
          params_.maximum_turn_rate, false, false);
        distractor.updateWithMeasurement(
          distractor_measurement, distractor_part.size, stamp_sec,
          params_.crossing_split_velocity_blend,
          params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
          params_.selection_max_speed, params_.acceleration_smoothing,
          params_.turn_rate_smoothing, params_.maximum_acceleration,
          params_.maximum_turn_rate, false, false);
        last_target_measurement_position_ = target_measurement;
        last_target_measurement_stamp_sec_ = stamp_sec;
        has_last_target_measurement_ = true;
      }
      // Only the target's seed partition is observable outside this node. The
      // coarse two-person centroid is never fed to either identity.
      selected_detection.push_back(std::move(target_part));
    }
    return true;
  }

  if (!crossing_occlusion_active_) {
    crossing_pending_merge_frames_ = 0;
    crossing_pending_auxiliary_index_ = -1;
    return false;
  }

  int first = -1;
  int second = -1;
  if (findRecoveryPair(
      candidates, *trackers_.front(),
      *auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)].filter,
      first, second))
  {
    startIdentityHypotheses(
      candidates, cloud, first, second, auxiliary_index, stamp_sec);
    if (!identity_hypotheses_.empty()) {
      const auto best = std::min_element(
        identity_hypotheses_.begin(), identity_hypotheses_.end(),
        [](const IdentityHypothesis & left, const IdentityHypothesis & right) {
          return left.cumulative_score < right.cumulative_score;
        });
      selected_detection.push_back(best->last_target_detection);
      ROS_INFO( "Merged people separated; retaining %zu identity hypotheses",
        identity_hypotheses_.size());
    }
    return true;
  }

  ++crossing_occlusion_frames_;
  if (crossing_occlusion_frames_ <= params_.crossing_occlusion_max_frames) {
    // No trusted observation: keep the predicted motion unchanged.
    return true;
  }
  ROS_WARN( "Crossing occlusion exceeded %d frames; returning to gated reacquisition",
    params_.crossing_occlusion_max_frames);
  crossing_occlusion_active_ = false;
  crossing_occlusion_frames_ = 0;
  crossing_auxiliary_index_ = -1;
  crossing_pending_merge_frames_ = 0;
  crossing_pending_auxiliary_index_ = -1;
  support_memory_resume_countdown_ = params_.merge_v1_resume_delay_frames;
  return false;
}

void MappingRos::clearCrossingState()
{
  auxiliary_tracks_.clear();
  identity_hypotheses_.clear();
  target_support_memory_ = TrackSupportMemory{};
  support_memory_resume_countdown_ = 0;
  resetMergeDebug();
  target_observed_size_.setZero();
  target_observed_point_count_ = 0.0;
  crossing_occlusion_active_ = false;
  crossing_occlusion_frames_ = 0;
  crossing_auxiliary_index_ = -1;
  crossing_pending_merge_frames_ = 0;
  crossing_pending_auxiliary_index_ = -1;
  pending_reacquisition_position_.setZero();
  pending_reacquisition_stamp_sec_ = -1.0;
  pending_reacquisition_frames_ = 0;
}

}  // namespace person_tracker
