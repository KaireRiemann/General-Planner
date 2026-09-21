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

void MappingRos::startIdentityHypotheses(
  const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
  int first, int second,
  int auxiliary_index, double stamp_sec)
{
  identity_hypotheses_.clear();
  if (
    trackers_.empty() || first < 0 || second < 0 ||
    static_cast<std::size_t>(first) >= candidates.size() ||
    static_cast<std::size_t>(second) >= candidates.size() || auxiliary_index < 0 ||
    static_cast<std::size_t>(auxiliary_index) >= auxiliary_tracks_.size())
  {
    return;
  }
  const auto & auxiliary = auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)];
  const std::array<std::pair<int, int>, 2> assignments{{
    {first, second}, {second, first}
  }};
  for (const auto & assignment : assignments) {
    IdentityHypothesis hypothesis;
    hypothesis.target_filter = std::make_shared<TargetEkf>(*trackers_.front());
    hypothesis.distractor_filter = std::make_shared<TargetEkf>(*auxiliary.filter);
    const auto & target_detection = candidates[static_cast<std::size_t>(assignment.first)];
    const auto & distractor_detection = candidates[static_cast<std::size_t>(assignment.second)];
    hypothesis.cumulative_score = identityAssociationCost(
      *hypothesis.target_filter, target_detection, target_observed_size_,
      target_observed_point_count_, cloud, target_support_memory_) + identityAssociationCost(
      *hypothesis.distractor_filter, distractor_detection, auxiliary.observed_size,
      auxiliary.observed_point_count, cloud, auxiliary.support_memory);
    const Eigen::Vector3d target_size = params_.selection_fixed_body_size_enabled ?
      params_.selection_fixed_body_size : target_detection.size;
    const Eigen::Vector3d target_measurement = boundedHorizontalMeasurement(
      hypothesis.target_filter->associationPosition(), target_detection.position,
      params_.crossing_split_update_max_correction);
    const Eigen::Vector3d distractor_measurement = boundedHorizontalMeasurement(
      hypothesis.distractor_filter->associationPosition(), distractor_detection.position,
      params_.crossing_split_update_max_correction);
    hypothesis.target_filter->updateWithMeasurement(
      target_measurement, target_size, stamp_sec,
      params_.crossing_split_velocity_blend,
      params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
      params_.selection_max_speed, params_.acceleration_smoothing,
      params_.turn_rate_smoothing, params_.maximum_acceleration,
      params_.maximum_turn_rate, false, false);
    hypothesis.distractor_filter->updateWithMeasurement(
      distractor_measurement, distractor_detection.size, stamp_sec,
      params_.crossing_split_velocity_blend,
      params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
      params_.selection_max_speed, params_.acceleration_smoothing,
      params_.turn_rate_smoothing, params_.maximum_acceleration,
      params_.maximum_turn_rate, false, false);
    hypothesis.frames = 1;
    hypothesis.last_target_candidate = assignment.first;
    hypothesis.last_distractor_candidate = assignment.second;
    hypothesis.last_target_detection = target_detection;
    identity_hypotheses_.push_back(std::move(hypothesis));
  }
}

void MappingRos::advanceIdentityHypotheses(
  const std::vector<Detection> & candidates, const PointCloudPtr & cloud,
  double stamp_sec,
  std::vector<Detection> & selected_detection)
{
  if (identity_hypotheses_.empty()) {
    return;
  }
  const int auxiliary_index = crossing_auxiliary_index_;
  for (auto & hypothesis : identity_hypotheses_) {
    int first = -1;
    int second = -1;
    if (!findRecoveryPair(
        candidates, *hypothesis.target_filter, *hypothesis.distractor_filter,
        first, second))
    {
      hypothesis.cumulative_score += params_.crossing_recovery_gate;
      ++hypothesis.frames;
      continue;
    }

    const auto & auxiliary = auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)];
    const double direct_cost = identityAssociationCost(
      *hypothesis.target_filter, candidates[static_cast<std::size_t>(first)],
      target_observed_size_, target_observed_point_count_, cloud,
      target_support_memory_) + identityAssociationCost(
      *hypothesis.distractor_filter, candidates[static_cast<std::size_t>(second)],
      auxiliary.observed_size, auxiliary.observed_point_count, cloud,
      auxiliary.support_memory);
    const double swapped_cost = identityAssociationCost(
      *hypothesis.target_filter, candidates[static_cast<std::size_t>(second)],
      target_observed_size_, target_observed_point_count_, cloud,
      target_support_memory_) + identityAssociationCost(
      *hypothesis.distractor_filter, candidates[static_cast<std::size_t>(first)],
      auxiliary.observed_size, auxiliary.observed_point_count, cloud,
      auxiliary.support_memory);
    const int target_index = direct_cost <= swapped_cost ? first : second;
    const int distractor_index = direct_cost <= swapped_cost ? second : first;
    const auto & target_detection = candidates[static_cast<std::size_t>(target_index)];
    const auto & distractor_detection = candidates[static_cast<std::size_t>(distractor_index)];
    hypothesis.cumulative_score += std::min(direct_cost, swapped_cost);
    const Eigen::Vector3d target_size = params_.selection_fixed_body_size_enabled ?
      params_.selection_fixed_body_size : target_detection.size;
    const Eigen::Vector3d target_measurement = boundedHorizontalMeasurement(
      hypothesis.target_filter->associationPosition(), target_detection.position,
      params_.crossing_split_update_max_correction);
    const Eigen::Vector3d distractor_measurement = boundedHorizontalMeasurement(
      hypothesis.distractor_filter->associationPosition(), distractor_detection.position,
      params_.crossing_split_update_max_correction);
    hypothesis.target_filter->updateWithMeasurement(
      target_measurement, target_size, stamp_sec,
      params_.crossing_split_velocity_blend,
      params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
      params_.selection_max_speed, params_.acceleration_smoothing,
      params_.turn_rate_smoothing, params_.maximum_acceleration,
      params_.maximum_turn_rate, false, false);
    hypothesis.distractor_filter->updateWithMeasurement(
      distractor_measurement, distractor_detection.size, stamp_sec,
      params_.crossing_split_velocity_blend,
      params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
      params_.selection_max_speed, params_.acceleration_smoothing,
      params_.turn_rate_smoothing, params_.maximum_acceleration,
      params_.maximum_turn_rate, false, false);
    hypothesis.last_target_candidate = target_index;
    hypothesis.last_distractor_candidate = distractor_index;
    hypothesis.last_target_detection = target_detection;
    ++hypothesis.frames;
  }

  const auto better = [](const IdentityHypothesis & left, const IdentityHypothesis & right) {
      return left.cumulative_score / std::max(1, left.frames) <
             right.cumulative_score / std::max(1, right.frames);
    };
  const auto best_iterator = std::min_element(
    identity_hypotheses_.begin(), identity_hypotheses_.end(), better);
  if (best_iterator == identity_hypotheses_.end()) {
    return;
  }
  if (best_iterator->last_target_candidate >= 0) {
    selected_detection.push_back(best_iterator->last_target_detection);
  }

  double second_score = std::numeric_limits<double>::infinity();
  for (auto iterator = identity_hypotheses_.begin(); iterator != identity_hypotheses_.end(); ++iterator) {
    if (iterator == best_iterator) {
      continue;
    }
    second_score = std::min(
      second_score, iterator->cumulative_score / std::max(1, iterator->frames));
  }
  const double best_score =
    best_iterator->cumulative_score / std::max(1, best_iterator->frames);
  const double score_margin = second_score - best_score;
  if (
    best_iterator->frames >= params_.crossing_hypothesis_max_frames &&
    std::isfinite(second_score) &&
    score_margin < params_.crossing_hypothesis_timeout_min_margin)
  {
    ROS_WARN(
      "Discarded ambiguous crossing hypotheses at timeout "
      "(best=%.3f second=%.3f margin=%.3f)",
      best_score, second_score, score_margin);
    selected_detection.clear();
    identity_hypotheses_.clear();
    crossing_occlusion_active_ = false;
    crossing_occlusion_frames_ = 0;
    crossing_auxiliary_index_ = -1;
    crossing_pending_merge_frames_ = 0;
    crossing_pending_auxiliary_index_ = -1;
    support_memory_resume_countdown_ = params_.merge_v1_resume_delay_frames;
    return;
  }
  if (!crossing_logic::shouldResolveIdentity(
      best_score, second_score, best_iterator->frames,
      params_.crossing_hypothesis_confirm_frames,
      params_.crossing_hypothesis_max_frames,
      params_.crossing_hypothesis_score_margin))
  {
    return;
  }

  const double resolved_jump =
    (best_iterator->target_filter->associationPosition().head<2>() -
    trackers_.front()->associationPosition().head<2>()).norm();
  if (resolved_jump > params_.crossing_recovery_gate) {
    ROS_WARN(
      "Rejected crossing identity commit: %.2f m jump exceeds %.2f m continuity gate",
      resolved_jump, params_.crossing_recovery_gate);
    // Reject the identity branch without inventing a target deceleration.
    identity_hypotheses_.clear();
    crossing_occlusion_active_ = false;
    crossing_occlusion_frames_ = 0;
    crossing_auxiliary_index_ = -1;
    crossing_pending_merge_frames_ = 0;
    crossing_pending_auxiliary_index_ = -1;
    support_memory_resume_countdown_ = params_.merge_v1_resume_delay_frames;
    return;
  }

  TargetEkf resolved_target = *best_iterator->target_filter;
  const TargetEkf resolved_distractor = *best_iterator->distractor_filter;
  const Detection resolved_detection = best_iterator->last_target_detection;
  if (
    resolved_jump > params_.selection_reacquisition_max_position_correction &&
    resolved_jump > 1e-6)
  {
    const Eigen::Vector2d old_position = trackers_.front()->associationPosition().head<2>();
    const Eigen::Vector2d direction =
      resolved_target.associationPosition().head<2>() - old_position;
    Eigen::Vector3d bounded_position = resolved_target.associationPosition();
    bounded_position.head<2>() = old_position +
      params_.selection_reacquisition_max_position_correction * direction / resolved_jump;
    resolved_target.snapPosition(bounded_position);
    ROS_INFO(
      "Bounded crossing identity correction from %.2f m to %.2f m",
      resolved_jump, params_.selection_reacquisition_max_position_correction);
  }
  trackers_.front() = std::make_shared<TargetEkf>(resolved_target);
  if (
    auxiliary_index >= 0 &&
    static_cast<std::size_t>(auxiliary_index) < auxiliary_tracks_.size())
  {
    auxiliary_tracks_[static_cast<std::size_t>(auxiliary_index)].filter =
      std::make_shared<TargetEkf>(resolved_distractor);
  }
  updateObservedSignature(
    target_observed_size_, target_observed_point_count_, resolved_detection);
  last_target_measurement_position_ = trackers_.front()->lastMeasurementPosition();
  last_target_measurement_stamp_sec_ = trackers_.front()->lastMeasurementStamp();
  has_last_target_measurement_ = true;
  identity_hypotheses_.clear();
  crossing_occlusion_active_ = false;
  crossing_occlusion_frames_ = 0;
  crossing_auxiliary_index_ = -1;
  crossing_pending_merge_frames_ = 0;
  crossing_pending_auxiliary_index_ = -1;
  support_memory_resume_countdown_ = params_.merge_v1_resume_delay_frames;
  ROS_INFO(
    "Crossing identity resolved after multi-frame hypotheses (best=%.3f second=%.3f)",
    best_score, second_score);
}

}  // namespace person_tracker
