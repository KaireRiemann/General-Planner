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

void MappingRos::predictTrackers(double stamp_sec)
{
  for (auto & tracker : trackers_) {
    tracker->predict(stamp_sec);
  }
  for (auto & auxiliary : auxiliary_tracks_) {
    auxiliary.filter->predict(stamp_sec);
  }
  for (auto & hypothesis : identity_hypotheses_) {
    hypothesis.target_filter->predict(stamp_sec);
    hypothesis.distractor_filter->predict(stamp_sec);
  }
}

void MappingRos::updateTrackers(std::vector<Detection> & detections, double stamp_sec)
{
  std::vector<bool> matched_tracks(trackers_.size(), false);
  std::vector<bool> matched_detections(detections.size(), false);

  if (!detections.empty() && !trackers_.empty()) {
    std::vector<std::vector<double>> cost_matrix(
      detections.size(), std::vector<double>(trackers_.size(), 0.0));
    for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
      for (std::size_t tracker_index = 0; tracker_index < trackers_.size(); ++tracker_index) {
        cost_matrix[detection_index][tracker_index] =
          (detections[detection_index].position - trackers_[tracker_index]->associationPosition()).norm();
      }
    }

    HungarianAlgorithm solver;
    std::vector<int> assignments;
    solver.Solve(cost_matrix, assignments);
    for (std::size_t detection_index = 0; detection_index < assignments.size(); ++detection_index) {
      const int tracker_index = assignments[detection_index];
      if (
        tracker_index < 0 || static_cast<std::size_t>(tracker_index) >= trackers_.size() ||
        cost_matrix[detection_index][static_cast<std::size_t>(tracker_index)] >
        params_.association_max_distance)
      {
        continue;
      }
      trackers_[static_cast<std::size_t>(tracker_index)]->updateWithMeasurement(
        detections[detection_index].position, detections[detection_index].size, stamp_sec,
        params_.selection_measurement_velocity_blend,
        params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
        params_.selection_max_speed, params_.acceleration_smoothing,
        params_.turn_rate_smoothing, params_.maximum_acceleration,
        params_.maximum_turn_rate);
      matched_tracks[static_cast<std::size_t>(tracker_index)] = true;
      matched_detections[detection_index] = true;
    }
  }

  for (std::size_t tracker_index = 0; tracker_index < trackers_.size(); ++tracker_index) {
    if (!matched_tracks[tracker_index]) {
      trackers_[tracker_index]->markMissed();
    }
  }

  for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
    if (!matched_detections[detection_index]) {
      trackers_.push_back(std::make_shared<TargetEkf>(
        next_track_id_++, detections[detection_index].position, detections[detection_index].size,
        stamp_sec, params_.acceleration_noise, params_.measurement_noise,
        params_.selection_adaptive_size_enabled, params_.selection_size_smoothing,
        params_.selection_size_max_relative_step));
    }
  }

  trackers_.erase(
    std::remove_if(
      trackers_.begin(), trackers_.end(),
      [&](const std::shared_ptr<TargetEkf> & tracker) {
        return tracker->missed() > params_.max_missed_frames;
      }),
    trackers_.end());
}

}  // namespace person_tracker
