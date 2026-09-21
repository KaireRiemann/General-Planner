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

std::vector<Detection> MappingRos::updateManualTarget(
  const std::vector<Detection> & candidates, const PointCloudPtr & candidate_cloud,
  const PointCloudPtr & current_cloud, double stamp_sec)
{
  std::vector<Detection> selected_detection;

  if (pending_target_click_) {
    double best_distance = std::numeric_limits<double>::infinity();
    const Detection * best_candidate = nullptr;
    int best_candidate_index = -1;
    Detection roi_candidate;
    for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
      const auto & candidate = candidates[candidate_index];
      for (const int point_index : candidate.indices.indices) {
        const auto & point = candidate_cloud->points[static_cast<std::size_t>(point_index)];
        const Eigen::Vector3d offset =
          Eigen::Vector3d(point.x, point.y, point.z) - clicked_position_;
        const double distance = pending_click_xy_only_ ? offset.head<2>().norm() : offset.norm();
        if (distance < best_distance) {
          best_distance = distance;
          best_candidate = &candidate;
          best_candidate_index = static_cast<int>(candidate_index);
        }
      }
    }

    if (
      (best_candidate == nullptr || best_distance > params_.selection_max_click_distance) &&
      params_.selection_roi_fallback_enabled &&
      buildRoiDetection(
        current_cloud, clicked_position_, params_.selection_roi_radius_xy,
        pending_click_xy_only_ ? params_.local_update_range.z() :
        params_.selection_roi_half_height, roi_candidate))
    {
      roi_candidate.indices_from_current_cloud = true;
      best_candidate = &roi_candidate;
      best_candidate_index = -1;
      best_distance = 0.0;
      ROS_INFO( "No complete DBSCAN cluster at click; initialized target from %zu local "
        "occupied voxels",
        roi_candidate.indices.indices.size());
    }

    if (best_candidate == nullptr || best_distance > params_.selection_max_click_distance) {
      ROS_WARN_THROTTLE(2.0,
        "No cluster within %.2f m of click (nearest=%.2f m). Click a colored point in "
        "Recent XY clusters",
        params_.selection_max_click_distance, best_distance);
      return selected_detection;
    }

    Detection selected_candidate = *best_candidate;
    ros::Time measurement_stamp;
    measurement_stamp.fromSec(stamp_sec);
    anchorTargetHeight(
      selected_candidate.position, current_cloud, measurement_stamp,
      selected_candidate.position.z(), true);
    const Eigen::Vector3d tracking_size = params_.selection_fixed_body_size_enabled ?
      params_.selection_fixed_body_size : selected_candidate.size;
    trackers_.clear();
    trackers_.push_back(std::make_shared<TargetEkf>(
      next_track_id_++, selected_candidate.position, tracking_size, stamp_sec,
      params_.acceleration_noise, params_.measurement_noise,
      !params_.selection_fixed_body_size_enabled && params_.selection_adaptive_size_enabled,
      params_.selection_size_smoothing, params_.selection_size_max_relative_step));
    rememberBodyRadius(selected_candidate, trackers_.front()->id(), stamp_sec);
    pending_target_click_ = false;
    pending_click_xy_only_ = false;
    manual_target_selected_ = true;
    fusion_state_machine_.lockTarget();
    fusion_reason_ = "target selected from lidar cluster";
    last_target_measurement_position_ = selected_candidate.position;
    has_last_target_measurement_ = true;
    last_target_measurement_stamp_sec_ = stamp_sec;
    target_observed_size_ = selected_candidate.size;
    target_observed_point_count_ =
      static_cast<double>(best_candidate->indices.indices.size());
    std::unordered_set<std::size_t> excluded_candidates;
    if (best_candidate_index >= 0) {
      excluded_candidates.insert(static_cast<std::size_t>(best_candidate_index));
    }
    updateAuxiliaryTracks(candidates, excluded_candidates, stamp_sec);
    removeTargetFromTransientBackground();
    selected_detection.push_back(selected_candidate);
    ROS_INFO( "Selected target ID %d: click distance=%.2f m, points=%zu, "
      "center=[%.2f, %.2f, %.2f], size=[%.2f, %.2f, %.2f]",
      trackers_.front()->id(), best_distance, best_candidate->indices.indices.size(),
      selected_candidate.position.x(), selected_candidate.position.y(),
      selected_candidate.position.z(),
      tracking_size.x(), tracking_size.y(), tracking_size.z());
    return selected_detection;
  }

  if (!manual_target_selected_ || trackers_.empty()) {
    return selected_detection;
  }

  const auto & tracker = trackers_.front();
  const double semantic_age = yolo_semantic_stamp_.isZero() ?
    std::numeric_limits<double>::infinity() : stamp_sec - yolo_semantic_stamp_.toSec();
  if (yolo_semantic_observation_valid_ &&
      semantic_age > yolo_continuous_validation_max_age_) {
    ros::Time semantic_stamp;
    semantic_stamp.fromSec(stamp_sec);
    observeFusion(SemanticEvidence::UNKNOWN, semantic_stamp,
      "YOLO semantic observation timed out; lidar-only fallback");
  }
  const bool semantic_guard_active = !lidar_evidence_policy_ && yolo_enabled_ && fusion_enabled_ &&
    fusion_state_machine_.state() == FusionState::SUSPECT &&
    fusion_state_machine_.reselectionReady() &&
    yolo_semantic_observation_valid_ && semantic_age >= -yolo_cloud_slop_ &&
    semantic_age <= yolo_continuous_validation_max_age_;
  Eigen::Vector3d semantic_reference = yolo_semantic_position_;
  if (semantic_guard_active && params_.selection_fixed_body_size_enabled) {
    semantic_reference.z() = tracker->associationPosition().z();
  }
  const auto semantic_distance = [&](const Detection & candidate) {
      return (candidate.position.head<2>() - semantic_reference.head<2>()).norm();
    };
  const auto semantic_supports = [&](const Detection & candidate) {
      if (!semantic_guard_active) return true;
      const bool vertical_support = params_.selection_fixed_body_size_enabled ||
        std::abs(candidate.position.z() - semantic_reference.z()) <=
        yolo_reacquisition_max_z_difference_;
      return semantic_distance(candidate) <= yolo_continuous_validation_distance_ &&
        vertical_support;
    };

  const auto visual_bonus = [&](const Detection & candidate) {
    if (!lidar_evidence_policy_ || !fusion_enabled_ || !evidence_boxes_) return 0.0;
    const auto & boxes = *evidence_boxes_;
    const ros::Time image_stamp = boxes.image_header.stamp.isZero() ?
      boxes.header.stamp : boxes.image_header.stamp;
    const double age = stamp_sec - image_stamp.toSec();
    if (image_stamp.isZero() || age < -yolo_cloud_slop_ ||
        age > yolo_continuous_validation_max_age_) return 0.0;
    Eigen::Vector3d position = candidate.position;
    position.z() = tracker->associationPosition().z();
    const auto projection = cameraVisibility(position, image_stamp);
    if (!projection.projection_valid || !projection.in_fov) return 0.0;
    int matches = 0;
    for (const auto & box : boxes.bounding_boxes) {
      if (box.Class == target_label_ && std::isfinite(box.probability) &&
          box.probability >= yolo_min_probability_ &&
          projection.u >= box.xmin && projection.u <= box.xmax &&
          projection.v >= box.ymin && projection.v <= box.ymax) ++matches;
    }
    // Overlapping boxes are ambiguous. This bounded preference cannot bypass
    // motion, foreground, size, competing-track or recovery-confirmation gates.
    return matches == 1 ? 0.08 : 0.0;
  };

  std::vector<Detection> crossing_candidates;
  if (semantic_guard_active) {
    crossing_candidates.reserve(candidates.size());
    std::copy_if(candidates.begin(), candidates.end(),
      std::back_inserter(crossing_candidates), semantic_supports);
  }
  const auto & crossing_input = semantic_guard_active ? crossing_candidates : candidates;
  if (handleCrossingState(crossing_input, candidate_cloud, stamp_sec, selected_detection)) {
    if (!selected_detection.empty()) {
      cancelYoloReacquisition();
      if (semantic_guard_active) {
        body_radius_=0.0;body_radius_target_id_=-1;body_center_offset_.setZero();
      tracker->resetFromMeasurement(
          semantic_reference, tracker->size(), stamp_sec,
          !params_.selection_fixed_body_size_enabled);
        fusion_state_machine_.acceptReselection();
        fusion_reason_ = "confirmed semantic lidar cluster reset EKF during crossing";
      }
      ros::Time measurement_stamp;
      measurement_stamp.fromSec(stamp_sec);
      Eigen::Vector3d grounded_position = tracker->position();
      anchorTargetHeight(
        grounded_position, current_cloud, measurement_stamp,
        tracker->associationPosition().z(), false);
      tracker->snapPosition(grounded_position);
      last_target_measurement_position_.z() = grounded_position.z();
    }
    pending_reacquisition_stamp_sec_ = -1.0;
    pending_reacquisition_frames_ = 0;
    return selected_detection;
  }

  const Detection * best_candidate = nullptr;
  int best_candidate_index = -1;
  Detection roi_candidate;
  Detection reacquisition_candidate;
  Detection semantic_candidate;
  bool reacquired = false;
  bool best_used_stop_hypothesis = false;
  double best_cost = std::numeric_limits<double>::infinity();
  double best_position_distance = std::numeric_limits<double>::infinity();
  const int interacting_auxiliary_index = closestAuxiliaryTrack();
  const TargetEkf * interacting_auxiliary =
    interacting_auxiliary_index >= 0 &&
    static_cast<std::size_t>(interacting_auxiliary_index) < auxiliary_tracks_.size() ?
    auxiliary_tracks_[static_cast<std::size_t>(interacting_auxiliary_index)].filter.get() :
    nullptr;

  // SUSPECT is the only state in which the camera may nominate a component.
  // This component was already formed from real lidar points projected into a
  // timestamp-matched YOLO box. Keeping it as a fallback is important when
  // background subtraction fails to expose the person to the FAPP clusters.
  if (semantic_guard_active) {
    semantic_candidate.position = semantic_reference;
    semantic_candidate.size = params_.selection_fixed_body_size_enabled ?
      params_.selection_fixed_body_size : yolo_semantic_size_;
    semantic_candidate.dynamic = true;
    semantic_candidate.indices_from_current_cloud = true;
    best_candidate = &semantic_candidate;
    best_position_distance =
      (semantic_reference.head<2>() - tracker->associationPosition().head<2>()).norm();
    best_cost = 0.0;
    reacquired = true;
  }

  struct MotionScore
  {
    double cost{std::numeric_limits<double>::infinity()};
    double predicted_distance{std::numeric_limits<double>::infinity()};
    double gate_distance{std::numeric_limits<double>::infinity()};
    bool used_stop_hypothesis{false};
  };

  // Score continuity against the last accepted observation, with no motion extrapolation.
  const auto score_motion = [&](const Eigen::Vector3d & position) {
      MotionScore score;
      score.predicted_distance =
        (position.head<2>() - tracker->associationPosition().head<2>()).norm();
      score.cost = tracker->associationDistance(position);
      score.gate_distance = score.predicted_distance;
      return score;
    };

  // First follow the actual foreground points inside a target-sized gate
  // centred on the last accepted observation. The configured values are minimums;
  // larger non-human targets expand the gate from their learned dimensions.
  // This is deliberately evaluated before
  // global clusters: when the target has no return, a different cluster 0.5 m
  // away must not steal the ID.
  const double adaptive_roi_radius_xy = std::max(
    params_.selection_roi_radius_xy,
    0.5 * std::max(tracker->size().x(), tracker->size().y()) + params_.map_resolution);
  const double adaptive_roi_half_height = std::max(
    params_.selection_roi_half_height,
    0.5 * tracker->size().z() + params_.map_resolution);
  if (
    params_.selection_roi_fallback_enabled &&
    buildRoiDetection(
      candidate_cloud, tracker->associationPosition(),
      adaptive_roi_radius_xy, adaptive_roi_half_height,
      roi_candidate))
  {
    const bool roi_contains_both_people =
      interacting_auxiliary != nullptr &&
      detectionSupportsTrack(roi_candidate, candidate_cloud, *tracker) &&
      detectionSupportsTrack(roi_candidate, candidate_cloud, *interacting_auxiliary);
    const Eigen::Vector3d offset = roi_candidate.position - tracker->associationPosition();
    // Mid360 often observes only a person's torso or legs in one scan, so the
    // measured vertical centroid can jump even though XY is correct.  With a
    // known body height, association is intentionally horizontal and Z is
    // retained from the track below.
    const double roi_distance = params_.selection_fixed_body_size_enabled ?
      offset.head<2>().norm() :
      std::sqrt(
      offset.x() * offset.x() + offset.y() * offset.y() + 0.25 * offset.z() * offset.z());
    if (
      roi_distance <= params_.selection_association_max_distance &&
      !roi_contains_both_people && semantic_supports(roi_candidate))
    {
      if (params_.selection_fixed_body_size_enabled) {
        // Sparse vertical returns make the measured Z centre jump. Keep the
        // selected person's fixed body centre/size and update horizontal motion.
        roi_candidate.position.z() = tracker->associationPosition().z();
        roi_candidate.size = params_.selection_fixed_body_size;
      }
      const MotionScore motion_score = score_motion(roi_candidate.position);
      best_candidate = &roi_candidate;
      best_position_distance = roi_distance;
      best_used_stop_hypothesis = motion_score.used_stop_hypothesis;
      const double roi_clearance = backgroundClearance(roi_candidate.position);
      best_cost = motion_score.cost + params_.selection_reacquisition_clearance_weight *
        std::max(
        0.0, params_.selection_reacquisition_preferred_center_clearance - roi_clearance);
      ROS_DEBUG( "Target ROI fallback used with %zu voxels",
        roi_candidate.indices.indices.size());
    }
  }

  if (best_candidate == nullptr && !params_.selection_prediction_gate_only) {
    for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
      const auto & candidate = candidates[candidate_index];
      if (!semantic_supports(candidate)) continue;
      const Eigen::Vector3d offset = candidate.position - tracker->associationPosition();
      const double position_distance = std::sqrt(
        offset.x() * offset.x() + offset.y() * offset.y() + 0.25 * offset.z() * offset.z());
      best_position_distance = std::min(best_position_distance, position_distance);
      const double size_change = (candidate.size - tracker->size()).cwiseAbs().maxCoeff();
      if (
        position_distance > params_.selection_association_max_distance ||
        size_change > params_.selection_max_size_change)
      {
        continue;
      }
      const double cost = position_distance + 0.10 * size_change - visual_bonus(candidate) +
        (semantic_guard_active ? semantic_distance(candidate) : 0.0);
      if (cost < best_cost) {
        best_cost = cost;
        best_candidate = &candidate;
        best_candidate_index = static_cast<int>(candidate_index);
      }
    }
  }

  // The precision ROI intentionally stays small so nearby clutter cannot drag
  // the target.  Fast motion can nevertheless move a person outside it in one
  // 8-10 Hz lidar frame.  Reacquire only from complete FAPP foreground DBSCAN
  // clusters: static background points are not eligible, tiny speckle is
  // rejected, and an oversized wall/table-like cluster is rejected.
  if (params_.selection_reacquisition_enabled) {
    const Eigen::Vector3d expected_size = params_.selection_fixed_body_size_enabled ?
      params_.selection_fixed_body_size : tracker->size();
    const double maximum_x = expected_size.x() + params_.selection_max_size_change;
    const double maximum_y = expected_size.y() + params_.selection_max_size_change;
    const double maximum_z = expected_size.z() + params_.selection_max_size_change;

    for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
      const auto & candidate = candidates[candidate_index];
      if (
        static_cast<int>(candidate.indices.indices.size()) < params_.selection_roi_min_points ||
        !compatibleTargetSize(candidate.size, expected_size) ||
        !semantic_supports(candidate))
      {
        continue;
      }
      if (
        interacting_auxiliary != nullptr &&
        detectionSupportsTrack(candidate, candidate_cloud, *tracker) &&
        detectionSupportsTrack(candidate, candidate_cloud, *interacting_auxiliary))
      {
        // This is still a coarse two-person cluster. Wait for the crossing
        // state machine or use a refined child instead of its mixed centroid.
        continue;
      }
      if (interacting_auxiliary != nullptr) {
        const bool target_support =
          detectionSupportsTrack(candidate, candidate_cloud, *tracker);
        const bool auxiliary_support =
          detectionSupportsTrack(candidate, candidate_cloud, *interacting_auxiliary);
        const double target_motion_cost = tracker->associationDistance(candidate.position);
        const double auxiliary_motion_cost =
          interacting_auxiliary->associationDistance(candidate.position);
        if (
          auxiliary_support &&
          (!target_support || auxiliary_motion_cost + 0.08 < target_motion_cost))
        {
          // The clicked target may be temporarily invisible while the other
          // person remains a clean cluster. That cluster is already owned by
          // a private auxiliary track; ordinary reacquisition must not steal
          // it after two frames. Identity exchange is allowed only through
          // the crossing hypotheses above.
          continue;
        }
      }
      const Eigen::Vector3d offset = candidate.position - tracker->associationPosition();
      const MotionScore motion_score = score_motion(candidate.position);
      const double horizontal_distance = motion_score.predicted_distance;
      best_position_distance = std::min(best_position_distance, horizontal_distance);
      const double center_clearance = backgroundClearance(candidate.position);
      if (
        motion_score.gate_distance > params_.selection_reacquisition_max_distance ||
        std::abs(offset.z()) > params_.selection_reacquisition_max_z_difference ||
        candidate.size.x() > maximum_x || candidate.size.y() > maximum_y ||
        candidate.size.z() > maximum_z)
      {
        continue;
      }

      const double vertical_penalty = 0.10 * std::abs(offset.z());
      const double clearance_penalty = params_.selection_reacquisition_clearance_weight *
        std::max(
        0.0, params_.selection_reacquisition_preferred_center_clearance - center_clearance);
      const double cost =
        motion_score.cost + vertical_penalty + clearance_penalty - visual_bonus(candidate) +
        (semantic_guard_active ? semantic_distance(candidate) : 0.0);
      if (cost < best_cost) {
        best_cost = cost;
        reacquisition_candidate = candidate;
        best_candidate = &reacquisition_candidate;
        best_candidate_index = static_cast<int>(candidate_index);
        best_position_distance = horizontal_distance;
        reacquired = true;
        best_used_stop_hypothesis = motion_score.used_stop_hypothesis;
      }
    }

    if (reacquired && params_.selection_fixed_body_size_enabled) {
      reacquisition_candidate.position.z() = tracker->associationPosition().z();
      reacquisition_candidate.size = params_.selection_fixed_body_size;
    }
  }

  // Only after actual lidar misses may vision nominate a search region.
  // Rebuild from CURRENT FAPP foreground; old image-cloud centroids are not
  // measurements, and confirmation/correction limits below still apply.
  if (lidar_evidence_policy_ && best_candidate == nullptr && tracker->missed() >= 3 &&
      yolo_semantic_observation_valid_ && semantic_age >= -yolo_cloud_slop_ &&
      semantic_age <= yolo_continuous_validation_max_age_ &&
      buildRoiDetection(candidate_cloud, yolo_semantic_position_,
        adaptive_roi_radius_xy, adaptive_roi_half_height, semantic_candidate)) {
    const double distance = (semantic_candidate.position.head<2>() - tracker->associationPosition().head<2>()).norm();
    const double since_last = std::max(0.0, stamp_sec - last_target_measurement_stamp_sec_);
    const bool reachable = has_last_target_measurement_ &&
      (semantic_candidate.position.head<2>() - last_target_measurement_position_.head<2>()).norm() <=
        params_.selection_max_speed * since_last + params_.selection_reacquisition_max_position_correction;
    const bool occupied = interacting_auxiliary &&
      detectionSupportsTrack(semantic_candidate, candidate_cloud, *interacting_auxiliary);
    if (reachable && !occupied && distance <= params_.selection_reacquisition_max_distance &&
        compatibleTargetSize(semantic_candidate.size, tracker->size()) &&
        backgroundClearance(semantic_candidate.position) >= params_.map_resolution) {
      best_candidate = &semantic_candidate;
      best_position_distance = distance;
      best_cost = score_motion(semantic_candidate.position).cost;
      reacquired = true;
    }
  }

  if (best_candidate != nullptr) {
    const bool fusion_reselection = semantic_guard_active;
    const Eigen::Vector2d correction_offset =
      best_candidate->position.head<2>() - tracker->associationPosition().head<2>();
    const double correction_distance = correction_offset.norm();
    const bool far_reacquisition =
      reacquired && !fusion_reselection &&
      correction_distance > params_.selection_reacquisition_immediate_distance;
    if (far_reacquisition) {
      const bool recent_pending = pending_reacquisition_stamp_sec_ >= 0.0 &&
        stamp_sec - pending_reacquisition_stamp_sec_ <=
        2.5 * params_.selection_velocity_max_dt;
      const bool consistent_pending = recent_pending &&
        (best_candidate->position.head<2>() -
        pending_reacquisition_position_.head<2>()).norm() <=
        params_.selection_reacquisition_confirmation_radius;
      pending_reacquisition_frames_ = consistent_pending ?
        pending_reacquisition_frames_ + 1 : 1;
      pending_reacquisition_position_ = best_candidate->position;
      pending_reacquisition_stamp_sec_ = stamp_sec;
      if (
        pending_reacquisition_frames_ <
        params_.selection_reacquisition_confirm_frames)
      {
        std::unordered_set<std::size_t> excluded_candidates;
        if (best_candidate_index >= 0) {
          excluded_candidates.insert(static_cast<std::size_t>(best_candidate_index));
        }
        updateAuxiliaryTracks(candidates, excluded_candidates, stamp_sec);
        tracker->markMissed();
        ROS_WARN_THROTTLE(1.0,
          "Far target candidate at %.2f m is waiting for confirmation (%d/%d)",
          correction_distance, pending_reacquisition_frames_,
          params_.selection_reacquisition_confirm_frames);
        return selected_detection;
      }
    } else {
      pending_reacquisition_stamp_sec_ = -1.0;
      pending_reacquisition_frames_ = 0;
    }

    Eigen::Vector3d measurement_position = best_candidate->position;
    if (fusion_reselection) {
      // The semantic position is itself the centroid of a lidar component
      // inside the matched image box. It is therefore a real 3-D measurement,
      // not a monocular depth estimate.
      measurement_position = semantic_reference;
    }
    ros::Time measurement_stamp;
    measurement_stamp.fromSec(stamp_sec);
    anchorTargetHeight(
      measurement_position, current_cloud, measurement_stamp,
      tracker->associationPosition().z(), false);
    // Both a far-cluster reacquisition and an abrupt-stop hypothesis can be
    // far from the current EKF state after several prediction-only frames.
    // Confirming the physical candidate must not create a one-frame teleport
    // in the published target, so converge over several scans instead.
    const Eigen::Vector2d measurement_correction_offset =
      measurement_position.head<2>() - tracker->associationPosition().head<2>();
    const double measurement_correction_distance = measurement_correction_offset.norm();
    if (
      !fusion_reselection &&
      (far_reacquisition || best_used_stop_hypothesis) &&
      measurement_correction_distance > params_.selection_reacquisition_max_position_correction)
    {
      measurement_position.head<2>() = tracker->associationPosition().head<2>() +
        params_.selection_reacquisition_max_position_correction *
        measurement_correction_offset / measurement_correction_distance;
    }
    const double speed_before_update = tracker->velocity().head<2>().norm();
    const double last_measurement_distance = has_last_target_measurement_ ?
      (measurement_position.head<2>() -
      last_target_measurement_position_.head<2>()).norm() :
      std::numeric_limits<double>::infinity();
    // A bounded far-reacquisition displacement is not a physical one-scan
    // human velocity. Give it only the conservative recovery weight or the
    // CV/CA/CT predictor will learn a short high-speed spike and overshoot.
    const bool correction_is_not_velocity = far_reacquisition;
    const double velocity_blend = best_used_stop_hypothesis ? 0.0 :
      (correction_is_not_velocity ? params_.selection_reacquisition_velocity_blend :
      params_.selection_measurement_velocity_blend);
    const Eigen::Vector3d debug_velocity_before = tracker->velocity();
    TargetEkf::MeasurementUpdate measurement_update;
    if (fusion_reselection) {
      body_radius_=0.0;body_radius_target_id_=-1;body_center_offset_.setZero();
      tracker->resetFromMeasurement(
        measurement_position, best_candidate->size, stamp_sec,
        !params_.selection_fixed_body_size_enabled);
      fusion_state_machine_.acceptReselection();
      fusion_reason_ = "confirmed new lidar cluster; EKF reset with target ID preserved";
      ROS_WARN("Fusion relock accepted for target %d at [%.2f, %.2f, %.2f]",
        tracker->id(), measurement_position.x(), measurement_position.y(),
        measurement_position.z());
    } else {
      measurement_update = tracker->updateWithMeasurement(
        measurement_position, best_candidate->size, stamp_sec,
        velocity_blend,
        params_.selection_velocity_min_displacement, params_.selection_velocity_max_dt,
        params_.selection_max_speed, params_.acceleration_smoothing,
        params_.turn_rate_smoothing, params_.maximum_acceleration,
        params_.maximum_turn_rate, true,
        !far_reacquisition && !best_used_stop_hypothesis);
    }
    const Eigen::Vector3d debug_velocity_updated = tracker->velocity();
    if (!best_used_stop_hypothesis && measurement_update.velocity_valid) {
      if (measurement_update.measured_speed > 2.0) {
        ROS_INFO_THROTTLE(1.0,
          "High-speed measurement: displacement %.2f m / %.3f s = %.2f m/s, "
          "track velocity %.2f m/s",
          measurement_update.displacement, measurement_update.dt,
          measurement_update.measured_speed,
          tracker->velocity().head<2>().norm());
      }
    }
    if (reacquired || best_used_stop_hypothesis) {
      tracker->snapPosition(measurement_position);
    }
    if (best_used_stop_hypothesis) {
      tracker->dampVelocity(params_.selection_stop_velocity_damping);
      ROS_INFO_THROTTLE(1.0,
        "Abrupt-stop hypothesis selected: speed %.2f -> %.2f m/s, "
        "measurement stayed near previous position (%.2f m)",
        speed_before_update, tracker->velocity().head<2>().norm(), last_measurement_distance);
    }
    if (reacquired && !best_used_stop_hypothesis) {
      ROS_INFO_THROTTLE(1.0,
        "Fast-motion target reacquired at %.2f m (score %.2f, points=%zu)",
        best_position_distance, best_cost, best_candidate->indices.indices.size());
    }
    if (observation_debug_pub_.getNumSubscribers()>0) {
      ObservationDebug debug;
      debug.header.stamp.fromSec(stamp_sec);debug.header.frame_id=params_.world_frame;
      debug.target_id=tracker->id();
      debug.accepted_position.x=measurement_position.x();
      debug.accepted_position.y=measurement_position.y();
      debug.accepted_position.z=measurement_position.z();
      auto assign=[](geometry_msgs::Vector3 & output,const Eigen::Vector3d & v) {
        output.x=v.x();output.y=v.y();output.z=v.z();
      };
      assign(debug.velocity_before_update,debug_velocity_before);
      assign(debug.velocity_after_update,debug_velocity_updated);
      assign(debug.velocity_after_outer_rules,tracker->velocity());
      debug.stop_hypothesis=best_used_stop_hypothesis;debug.far_reacquisition=far_reacquisition;
      debug.position_snapped=reacquired || best_used_stop_hypothesis;debug.semantic_reset=fusion_reselection;
      observation_debug_pub_.publish(debug);
    }
    if (fusion_reselection) { body_radius_ = 0.0; body_radius_target_id_ = -1; }
    else rememberBodyRadius(*best_candidate, tracker->id(), stamp_sec);
    last_target_measurement_position_ = measurement_position;
    has_last_target_measurement_ = true;
    last_target_measurement_stamp_sec_ = stamp_sec;
    std::unordered_set<std::size_t> excluded_candidates;
    if (best_candidate_index >= 0) {
      excluded_candidates.insert(static_cast<std::size_t>(best_candidate_index));
      updateObservedSignature(
        target_observed_size_, target_observed_point_count_,
        candidates[static_cast<std::size_t>(best_candidate_index)]);
    } else {
      double nearest = params_.selection_association_max_distance;
      int nearest_index = -1;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        const double distance =
          (candidates[i].position.head<2>() - best_candidate->position.head<2>()).norm();
        if (distance < nearest) {
          nearest = distance;
          nearest_index = static_cast<int>(i);
        }
      }
      if (nearest_index >= 0) {
        excluded_candidates.insert(static_cast<std::size_t>(nearest_index));
      }
    }
    updateAuxiliaryTracks(candidates, excluded_candidates, stamp_sec);
    removeTargetFromTransientBackground();
    cancelYoloReacquisition();
    selected_detection.push_back(*best_candidate);
    return selected_detection;
  }

  if (
    pending_reacquisition_stamp_sec_ >= 0.0 &&
    stamp_sec - pending_reacquisition_stamp_sec_ >
    2.5 * params_.selection_velocity_max_dt)
  {
    pending_reacquisition_stamp_sec_ = -1.0;
    pending_reacquisition_frames_ = 0;
  }
  updateAuxiliaryTracks(candidates, {}, stamp_sec);
  tracker->markMissed();
  if (tracker->missed() > params_.selection_max_missed_frames) {
    armYoloReacquisition("lidar miss limit reached");
    ROS_WARN( "Selected target ID %d lost after %d frames; %s",
      tracker->id(), tracker->missed(),
      yolo_reacquisition_active_ ? "waiting for YOLO+cloud reacquisition" :
      "click the target again");
    trackers_.clear();
    fusion_state_machine_.loseTarget();
    fusion_reason_ = "lidar miss limit reached; bounded YOLO recovery armed";
    clearCrossingState();
    manual_target_selected_ = false;
    has_last_target_measurement_ = false;
    last_target_measurement_stamp_sec_ = -1.0;
  } else {
    ROS_WARN_THROTTLE(2.0,
      "Selected target temporarily unmatched (%d/%d frames, nearest accepted distance=%.2f m)",
      tracker->missed(), params_.selection_max_missed_frames, best_position_distance);
  }
  return selected_detection;
}

}  // namespace person_tracker
