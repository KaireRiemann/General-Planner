#include "core/mapping_manager.h"
#include <algorithm>
#include <cmath>

namespace person_tracker
{
bool MappingRos::resetTarget(std_srvs::Trigger::Request &, std_srvs::Trigger::Response & response)
{
  trackers_.clear();
  clearCrossingState();
  resetGroundHeight();
  pending_target_click_ = false;
  manual_target_selected_ = false;
  has_last_target_measurement_ = false;
  last_target_measurement_stamp_sec_ = -1.0;
  yolo_armed_ = true;
  yolo_reacquisition_active_ = false;
  yolo_reacquisition_reference_valid_ = false;
  yolo_reacquisition_target_id_ = -1;
  yolo_reacquisition_size_.setZero();
  yolo_semantic_observation_valid_ = false;
  yolo_semantic_stamp_ = ros::Time();
  target_ever_selected_ = false;
  yolo_confirmation_hits_ = 0;
  yolo_processed_stamp_ = ros::Time();
  yolo_candidate_stamp_ = ros::Time();
  fusion_state_machine_.reset();
  fusion_evidence_stamp_ = ros::Time();
  fusion_camera_projection_valid_ = false;
  fusion_predicted_in_fov_ = false;
  fusion_yolo_target_present_ = false;
  fusion_yolo_component_valid_ = false;
  fusion_candidate_eligible_ = false;
  fusion_semantic_distance_ = -1.0;
  fusion_reason_ = "explicit reset; waiting for target";
  pending_boxes_.reset();
  scan_history_.clear();
  output_last_observation_stamp_ = ros::Time();
  last_target_id_ = -1;
  response.success = true;
  response.message = yolo_enabled_ ? "Waiting for a new YOLO-confirmed point cluster" : "Waiting for target selection";
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = params_.world_frame;
  publishTracks(header);
  publishMarkers(header);
  publishPointCloud(PointCloud{}, header, dynamic_cloud_pub_);
  publishPointCloud(PointCloud{}, header, yolo_seed_pub_);
  publishPointCloud(PointCloud{}, header, yolo_candidate_pub_);
  publishPointCloud(PointCloud{}, header, ground_support_pub_);
  publishGroundEstimate(GroundHeightEstimate{}, 0.0, PointCloudPtr{}, header.stamp);
  return true;
}

void MappingRos::expireTarget(const ros::Time & stamp)
{
  if (trackers_.empty() || !has_last_target_measurement_) return;
  if (stamp.toSec() - last_target_measurement_stamp_sec_ <= identity_timeout_) return;
  armYoloReacquisition("observation timeout");
  fusion_state_machine_.loseTarget();
  fusion_reason_ = "identity timeout; bounded YOLO+lidar recovery only";
  ROS_WARN("Target %d lost: no accepted lidar observation for %.2f s. %s",
    trackers_.front()->id(), stamp.toSec() - last_target_measurement_stamp_sec_,
    yolo_reacquisition_active_ ? "Waiting for YOLO+cloud reacquisition." :
    "Reset target or select in RViz to start a new identity.");
  last_target_id_ = trackers_.front()->id();
  output_last_observation_stamp_.fromSec(last_target_measurement_stamp_sec_);
  trackers_.clear();
  clearCrossingState();
  resetGroundHeight();
  manual_target_selected_ = false;
  has_last_target_measurement_ = false;
  last_target_measurement_stamp_sec_ = -1.0;
}

void MappingRos::publishTargetStatus(const ros::Time & stamp)
{
  TargetStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = params_.world_frame;
  status.target_id = last_target_id_;
  status.label = target_label_;
  status.last_observation_stamp = output_last_observation_stamp_;
  status.observation_age = output_last_observation_stamp_.isZero() ? -1.0 :
    std::max(0.0, (stamp - output_last_observation_stamp_).toSec());
  status.state = target_ever_selected_ ? TargetStatus::LOST : TargetStatus::WAITING;
  if (!trackers_.empty()) {
    const double scan_age = stamp.toSec() - last_cloud_stamp_sec_;
    const bool fresh_scan = last_cloud_stamp_sec_ >= 0 && scan_age >= -0.001 && scan_age <= max_cloud_age_;
    const bool observed_in_scan = has_last_target_measurement_ &&
      std::abs(last_target_measurement_stamp_sec_ - last_cloud_stamp_sec_) < 1e-5;
    status.state = fresh_scan && observed_in_scan ? TargetStatus::TRACKING : TargetStatus::PREDICTING;
    status.odom_publishable = fresh_scan && status.observation_age >= 0 &&
      status.observation_age <= max_prediction_publish_age_;
  }
  std_msgs::Bool validity;
  validity.data = status.odom_publishable;
  target_valid_pub_.publish(validity);
  std_msgs::Float64 observation_age;
  observation_age.data = status.observation_age >= 0.0 ? status.observation_age :
    std::numeric_limits<double>::infinity();
  observation_age_pub_.publish(observation_age);
  // Clear latched paths on reset, loss and stale input, including watchdog ticks.
  if (!status.odom_publishable) publishTargetPath(status.header, false);
  PredictionStatus prediction;
  prediction.header = status.header;
  prediction.target_id = status.target_id;
  prediction.backend = TargetEkf::defaultImmEnabled() ? "imm" :
    (TargetEkf::defaultMmkNetConfig().network ? "mmknet" : "analytic");
  if (!TargetEkf::defaultUpstreamMode().empty()) prediction.backend=TargetEkf::defaultUpstreamMode();
  prediction.reason = "waiting for target";
  if (!trackers_.empty()) {
    const auto & tracker = trackers_.front();
    prediction.network_ready = tracker->mmknetReady();
    prediction.feature_frames = tracker->mmknetSamples();
    prediction.turn_rate = tracker->turnRate();
    for (int i=0;i<3;++i) prediction.model_probabilities[i]=tracker->modelProbabilities()[i];
    if (tracker->usesUpstream())
      for(int i=0;i<4;++i) prediction.upstream_model_probabilities[i]=tracker->upstreamProbabilities()[i];
    prediction.reason = tracker->usesUpstream() ? "IMM-MOT position-only adapter" : tracker->usesImm() ? "IMM-CV/CA/CTRV" : tracker->usesMmkNet() ?
      (tracker->mmknetReady() ? "learned XY turn rate" :
      "CV fallback: warming up, observation gap, or bounded correction") : "analytic CV/CA/CT";
  }
  prediction_status_pub_.publish(prediction);
  target_status_pub_.publish(status);
  publishFusionStatus(stamp);
}

void MappingRos::publishTargetState(const std_msgs::Header & header)
{
  if (!trackers_.empty()) {
    target_ever_selected_ = true;
    last_target_id_ = trackers_.front()->id();
    if (has_last_target_measurement_) output_last_observation_stamp_.fromSec(last_target_measurement_stamp_sec_);
    const double age = has_last_target_measurement_ ?
      header.stamp.toSec() - last_target_measurement_stamp_sec_ : identity_timeout_;
    if (age >= -1e-5 && age <= max_prediction_publish_age_) {
      const auto & tracker = trackers_.front();
      nav_msgs::Odometry message;
      message.header = header;  // State time; true observation time lives in TargetStatus.
      // EKF velocity is in world coordinates, so twist's frame is also world.
      message.child_frame_id = params_.world_frame;
      const auto position = tracker->position(), velocity = tracker->velocity();
      message.pose.pose.position.x = position.x();
      message.pose.pose.position.y = position.y();
      message.pose.pose.position.z = position.z();
      message.pose.pose.orientation.w = 1.0;
      message.twist.twist.linear.x = velocity.x();
      message.twist.twist.linear.y = velocity.y();
      message.twist.twist.linear.z = velocity.z();
      const auto & covariance = tracker->covariance();
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          message.pose.covariance[row * 6 + column] = covariance(row, column);
          message.twist.covariance[row * 6 + column] = covariance(row + 3, column + 3);
        }
        message.pose.covariance[(row + 3) * 6 + row + 3] = 1e6;
        message.twist.covariance[(row + 3) * 6 + row + 3] = 1e6;
      }
      target_odom_pub_.publish(message);
      publishTargetPath(header, true);
    }
  }
  publishTargetStatus(header.stamp);
}

void MappingRos::targetWatchdog(const ros::TimerEvent &)
{
  const ros::Time stamp = ros::Time::now();
  const bool had_target = !trackers_.empty();
  expireTarget(stamp);
  if (had_target && trackers_.empty()) {
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = params_.world_frame;
    publishTracks(header);
    publishMarkers(header);
    publishPointCloud(PointCloud{}, header, dynamic_cloud_pub_);
  }
  // Never restamp/re-publish an old pose when the cloud stream stops.
  publishTargetStatus(stamp);
}
}  // namespace person_tracker
