#include "core/mapping_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace person_tracker
{

Eigen::Vector3d MappingRos::predictedTargetAt(
  const TargetEkf & tracker, const ros::Time & stamp) const
{
  (void)stamp;
  // Keep target geometry at the accepted observation; camera pose is still timestamped.
  return tracker.associationPosition();
}

MappingRos::CameraVisibility MappingRos::cameraVisibility(
  const Eigen::Vector3d & world_position, const ros::Time & stamp) const
{
  CameraVisibility result;
  if (!world_position.allFinite() || stamp.isZero()) return result;
  const auto & camera_history = camera_odom_topic_ == params_.odom_topic ?
    odom_history_ : camera_odom_history_;
  Eigen::Isometry3d world_from_body;
  if (!poseAt(camera_history, stamp, world_from_body)) return result;

  const Eigen::Vector3d camera_point =
    (world_from_body * body_from_camera_).inverse() * world_position;
  if (!camera_point.allFinite() || camera_point.z() <= projection_.min_depth ||
    camera_point.z() >= projection_.max_depth)
  {
    result.projection_valid = true;
    result.depth = camera_point.z();
    return result;
  }
  result.projection_valid = true;
  result.depth = camera_point.z();
  result.u = projection_.fx * camera_point.x() / camera_point.z() + projection_.cx;
  result.v = projection_.fy * camera_point.y() / camera_point.z() + projection_.cy;
  const double maximum_margin = 0.5 * std::max(
    0.0, static_cast<double>(std::min(projection_.width, projection_.height)) - 1.0);
  const double margin = std::clamp(fusion_camera_margin_pixels_, 0.0, maximum_margin);
  result.in_fov = result.u >= margin &&
    result.u < static_cast<double>(projection_.width) - margin &&
    result.v >= margin && result.v < static_cast<double>(projection_.height) - margin;
  return result;
}

void MappingRos::observeFusion(
  SemanticEvidence evidence, const ros::Time & stamp, const std::string & reason,
  const std::optional<Eigen::Vector2d> & candidate, double semantic_distance)
{
  if (!fusion_enabled_) return;
  const FusionState old_state = fusion_state_machine_.state();
  if (lidar_evidence_policy_) {
    // Track life is governed by lidar support. Visual conflicts are diagnostics.
    if (!trackers_.empty()) fusion_state_machine_.lockTarget();
    fusion_state_machine_.recordEvidence(evidence);
    fusion_candidate_eligible_ = false;
  } else {
    fusion_state_machine_.observe(evidence, candidate);
  }
  fusion_evidence_stamp_ = stamp;
  fusion_semantic_distance_ = semantic_distance;
  fusion_reason_ = reason;
  if (evidence == SemanticEvidence::UNKNOWN ||
    evidence == SemanticEvidence::OUT_OF_VIEW)
  {
    // Missing or geometrically irrelevant camera evidence has no right to
    // keep an old semantic gate alive.
    yolo_semantic_observation_valid_ = false;
    yolo_semantic_stamp_ = ros::Time();
  }
  if (old_state != fusion_state_machine_.state()) {
    ROS_WARN("Fusion state %s -> %s: %s",
      fusionStateName(old_state), fusionStateName(fusion_state_machine_.state()),
      reason.c_str());
  }
}

void MappingRos::expireFusionEvidence(const ros::Time & stamp)
{
  if (!fusion_enabled_ || fusion_evidence_stamp_.isZero()) return;
  const double age = (stamp - fusion_evidence_stamp_).toSec();
  if (age <= yolo_continuous_validation_max_age_) return;
  if (fusion_state_machine_.evidence() == SemanticEvidence::UNKNOWN) return;
  fusion_yolo_target_present_ = false;
  fusion_yolo_component_valid_ = false;
  fusion_candidate_eligible_ = false;
  clearYoloDebugClouds(stamp, true);
  observeFusion(SemanticEvidence::UNKNOWN, stamp,
    "YOLO evidence timed out; lidar-only fallback");
}

void MappingRos::publishFusionStatus(const ros::Time & stamp)
{
  FusionStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = params_.world_frame;
  status.fusion_state = static_cast<std::uint8_t>(fusion_state_machine_.state());
  status.fusion_state_text = fusionStateName(fusion_state_machine_.state());
  status.semantic_evidence =
    static_cast<std::uint8_t>(fusion_state_machine_.evidence());
  status.semantic_evidence_text =
    semanticEvidenceName(fusion_state_machine_.evidence());
  status.camera_projection_valid = fusion_camera_projection_valid_;
  status.predicted_in_fov = fusion_predicted_in_fov_;
  status.yolo_target_present = fusion_yolo_target_present_;
  status.yolo_lidar_component_valid = fusion_yolo_component_valid_;
  status.automatic_reselection_eligible = fusion_candidate_eligible_;
  status.reselection_ready = fusion_state_machine_.reselectionReady();
  status.conflict_count =
    static_cast<std::uint32_t>(fusion_state_machine_.conflictCount());
  status.confirmation_count =
    static_cast<std::uint32_t>(fusion_state_machine_.confirmationCount());
  status.semantic_distance = fusion_semantic_distance_;
  status.semantic_age = fusion_evidence_stamp_.isZero() ? -1.0 :
    std::max(0.0, (stamp - fusion_evidence_stamp_).toSec());
  status.target_id = trackers_.empty() ? last_target_id_ : trackers_.front()->id();
  status.reason = fusion_reason_;
  fusion_status_pub_.publish(status);
}

void MappingRos::clearYoloDebugClouds(
  const ros::Time & stamp, bool clear_candidate)
{
  std_msgs::Header header;
  header.stamp = stamp;
  header.frame_id = params_.world_frame;
  const PointCloud empty;
  if (clear_candidate) publishPointCloud(empty, header, yolo_candidate_pub_);
  publishPointCloud(empty, header, yolo_seed_pub_);
}

}  // namespace person_tracker
