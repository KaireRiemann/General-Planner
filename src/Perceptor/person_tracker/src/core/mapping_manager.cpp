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

#include <visualization_msgs/Marker.h>

#include "common/tracking_geometry.hpp"

namespace person_tracker
{

using detail::logit;

MappingRos::MappingRos()
: nh_("~"),
  background_tree_(std::make_shared<KD_TREE<PointType>>(0.3, 0.6, 0.08))
{
  declareAndLoadParameters();
  loadYoloParameters();
  validateParameters();
  configurePredictionModel();
  path_horizon_ = readParameter<double>("prediction/path_horizon", 1.0);
  path_step_ = readParameter<double>("prediction/path_step", 0.25);
  if (!std::isfinite(path_horizon_) || !std::isfinite(path_step_) ||
      path_step_ <= 0.0 || path_horizon_ < path_step_ || path_horizon_ / path_step_ > 1000.0) {
    throw std::runtime_error("Invalid prediction path horizon/step");
  }
  ground_height_estimator_.setParameters(params_.ground_height);
  merge_v1_matcher_.setParameters(params_.merge_v1_matcher);

  hit_log_odds_ = logit(params_.map_hit_probability);
  miss_log_odds_ = logit(params_.map_miss_probability);
  min_log_odds_ = logit(params_.map_min_probability);
  max_log_odds_ = logit(params_.map_max_probability);
  occupied_log_odds_ = logit(params_.map_occupied_probability);
  unknown_log_odds_ = min_log_odds_ - 0.01;
  background_tree_ = std::make_shared<KD_TREE<PointType>>(
    0.3, 0.6, params_.map_resolution);

  // Point clouds are live observations. A depth-one queue prevents RViz from
  // replaying stale dense frames when rendering briefly falls behind.
  filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
  voxel_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 1);
  inflated_voxel_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 1);
  detection_voxel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detection_voxels", 1);
  foreground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("foreground_points", 1);
  dynamic_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dynamic_points", 1);
  cluster_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1);
  merge_target_support_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merge_v1/support_target", 1);
  merge_distractor_support_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merge_v1/support_auxiliary", 1);
  merge_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merge_v1/merged_points", 1);
  merge_target_assigned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merge_v1/assigned_target", 1);
  merge_distractor_assigned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merge_v1/assigned_auxiliary", 1);
  merge_unknown_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merge_v1/unknown_points", 1);
  tracks_pub_ = nh_.advertise<TrackedObjectArray>("tracks", 1);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1);
  ground_support_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ground_support", 1);
  ground_estimate_pub_ = nh_.advertise<GroundEstimate>("ground_estimate", 1, true);
  odom_sub_ = nh_.subscribe(params_.odom_topic, 200, &MappingRos::odomCallback, this, ros::TransportHints().tcpNoDelay());
  cloud_sub_ = nh_.subscribe(params_.input_cloud_topic, 1, &MappingRos::cloudCallback, this, ros::TransportHints().tcpNoDelay());
  if (yolo_enabled_) {
    boxes_sub_ = nh_.subscribe(boxes_topic_, 1, &MappingRos::boxesCallback, this);
    if (camera_odom_topic_ != params_.odom_topic) {
      camera_odom_sub_ = nh_.subscribe(camera_odom_topic_, 200, &MappingRos::cameraOdomCallback, this);
    }
  }
  // Keep the geometrically valid box component separate from the component
  // accepted by the identity gate.  Without this split, an identity rejection
  // looks exactly like a projection/clustering failure in RViz.
  yolo_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
    "yolo_candidate_points", 1);
  yolo_seed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("yolo_seed_points", 1);
  target_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("target_odom", 1);
  target_path_pub_ = nh_.advertise<nav_msgs::Path>("target_prediction", 1, true);
  target_valid_pub_ = nh_.advertise<std_msgs::Bool>("target_valid", 1, true);
  observation_age_pub_ = nh_.advertise<std_msgs::Float64>("observation_age", 1, true);
  observation_debug_pub_ = nh_.advertise<ObservationDebug>("observation_debug", 10);
  prediction_status_pub_ = nh_.advertise<PredictionStatus>("prediction_status", 1, true);
  target_status_pub_ = nh_.advertise<TargetStatus>("target_status", 1, true);
  fusion_status_pub_ = nh_.advertise<FusionStatus>("fusion_status", 1, true);
  reset_target_service_ = nh_.advertiseService("reset_target", &MappingRos::resetTarget, this);
  target_watchdog_ = nh_.createTimer(ros::Duration(0.1), &MappingRos::targetWatchdog, this);
  if (params_.manual_selection_enabled) {
    clicked_point_sub_ = nh_.subscribe(params_.clicked_point_topic, 10, &MappingRos::clickedPointCallback, this);
    target_pose_sub_ = nh_.subscribe(params_.target_pose_topic, 10, &MappingRos::targetPoseCallback, this);
  }

  ROS_INFO(
    "Ready: cloud=%s odom=%s frame=%s registered=%s voxel=%.2f m mode=%s",
    params_.input_cloud_topic.c_str(), params_.odom_topic.c_str(),
    params_.world_frame.c_str(), params_.cloud_is_registered ? "true" : "false",
    params_.map_resolution,
    yolo_enabled_ && fusion_enabled_ ?
    "LOCKED/SUSPECT YOLO audit + FAPP lidar tracking" :
    (yolo_enabled_ ? "YOLO initialization + lidar tracking" :
    (params_.manual_selection_enabled ? "RViz click-to-select" : "automatic detection")));
  ROS_INFO(
    "Ground-referenced height=%s center_offset=%.2f m search_radius=%.2f m",
    params_.ground_height.enabled ? "enabled" : "disabled",
    params_.ground_height.person_center_height,
    params_.ground_height.search_radius);
  if (params_.manual_selection_enabled) {
    ROS_INFO(
      "Select with RViz Publish Point on %s, or 2D Goal Pose on %s",
      params_.clicked_point_topic.c_str(), params_.target_pose_topic.c_str());
    ROS_INFO( "v1 crossing guard=%s, merge support=%s, private auxiliary tracks=%d",
      params_.crossing_enabled ? "enabled" : "disabled",
      params_.merge_v1_enabled ? "enabled" : "v0 fallback",
      params_.crossing_max_shadow_tracks);
    if (yolo_enabled_) {
      ROS_INFO("Using label '%s' on %s for %s; reset_target allows explicit re-selection",
        target_label_.c_str(), boxes_topic_.c_str(),
        fusion_enabled_ ? "continuous camera-aware semantic audit" :
        "initialization/reacquisition");
    } else {
      ROS_INFO("Background starts from the first scan; initialize with the target outside the scene when possible");
    }
  }
}

MappingRos::~MappingRos()
{
  ROS_INFO( "person tracker stopped");
}

void MappingRos::odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
  if (!appendOdometry(msg, odom_history_)) return;
  odom_position_ = Eigen::Vector3d(
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  odom_orientation_ = Eigen::Quaterniond(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  if (odom_orientation_.norm() < 1e-6) {
    odom_orientation_ = Eigen::Quaterniond::Identity();
  } else {
    odom_orientation_.normalize();
  }
  odom_stamp_ = ros::Time(msg->header.stamp);
  if (odom_stamp_.toNSec() == 0) {
    odom_stamp_ = ros::Time::now();
  }
  has_odom_ = odom_position_.allFinite();
}

void MappingRos::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr msg)
{
  auto normalized_frame = [](std::string frame) {
      while (!frame.empty() && frame.front() == '/') {
        frame.erase(frame.begin());
      }
      return frame;
    };
  if (
    !msg->header.frame_id.empty() && !params_.world_frame.empty() &&
    normalized_frame(msg->header.frame_id) != normalized_frame(params_.world_frame))
  {
    ROS_ERROR( "Clicked point is in frame '%s', but tracker uses '%s'; click with RViz "
      "Fixed Frame set to '%s'",
      msg->header.frame_id.c_str(), params_.world_frame.c_str(), params_.world_frame.c_str());
    return;
  }

  clicked_position_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
  if (!clicked_position_.allFinite()) {
    ROS_WARN( "Ignoring non-finite clicked point");
    return;
  }

  // A new click always starts an explicit re-selection. The next cloud frame
  // resolves this point to the nearest complete DBSCAN cluster.
  trackers_.clear();
  clearCrossingState();
  manual_target_selected_ = false;
  has_last_target_measurement_ = false;
  last_target_measurement_stamp_sec_ = -1.0;
  pending_target_click_ = true;
  fusion_state_machine_.reset();
  fusion_reason_ = "explicit target click; waiting for lidar cluster";
  yolo_armed_ = false;
  yolo_reacquisition_active_ = false;
  yolo_reacquisition_reference_valid_ = false;
  yolo_reacquisition_target_id_ = -1;
  yolo_reacquisition_size_.setZero();
  yolo_semantic_observation_valid_ = false;
  yolo_semantic_stamp_ = ros::Time();
  pending_click_xy_only_ = false;
  ROS_INFO( "Target click received at [%.2f, %.2f, %.2f]; finding nearest cluster",
    clicked_position_.x(), clicked_position_.y(), clicked_position_.z());
}

void MappingRos::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
  auto normalized_frame = [](std::string frame) {
      while (!frame.empty() && frame.front() == '/') {
        frame.erase(frame.begin());
      }
      return frame;
    };
  if (
    !msg->header.frame_id.empty() && !params_.world_frame.empty() &&
    normalized_frame(msg->header.frame_id) != normalized_frame(params_.world_frame))
  {
    ROS_ERROR( "2D target pose is in frame '%s', but tracker uses '%s'",
      msg->header.frame_id.c_str(), params_.world_frame.c_str());
    return;
  }

  clicked_position_ = Eigen::Vector3d(
    msg->pose.position.x, msg->pose.position.y, odom_position_.z());
  if (!clicked_position_.allFinite()) {
    ROS_WARN( "Ignoring non-finite 2D target pose");
    return;
  }

  // SetGoal always intersects the XY grid, so it works even when a sparse
  // PointCloud2 has no renderable point under the mouse cursor.  Z is resolved
  // from the nearest foreground cluster/current vertical column next frame.
  trackers_.clear();
  clearCrossingState();
  manual_target_selected_ = false;
  has_last_target_measurement_ = false;
  last_target_measurement_stamp_sec_ = -1.0;
  pending_target_click_ = true;
  fusion_state_machine_.reset();
  fusion_reason_ = "explicit target pose; waiting for lidar cluster";
  yolo_armed_ = false;
  yolo_reacquisition_active_ = false;
  yolo_reacquisition_reference_valid_ = false;
  yolo_reacquisition_target_id_ = -1;
  yolo_reacquisition_size_.setZero();
  yolo_semantic_observation_valid_ = false;
  yolo_semantic_stamp_ = ros::Time();
  pending_click_xy_only_ = true;
  ROS_INFO( "2D target selection received at XY=[%.2f, %.2f]",
    clicked_position_.x(), clicked_position_.y());
}

void MappingRos::resetTrackingState()
{
  resetGroundHeight();
  scan_history_.clear();
  pending_boxes_.reset();
  yolo_armed_ = true;
  yolo_reacquisition_active_ = false;
  yolo_reacquisition_reference_valid_ = false;
  yolo_reacquisition_target_id_ = -1;
  yolo_reacquisition_size_.setZero();
  yolo_semantic_observation_valid_ = false;
  yolo_semantic_stamp_ = ros::Time();
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
  fusion_reason_ = "time reset; waiting for target";
  target_ever_selected_ = false;
  output_last_observation_stamp_ = ros::Time();
  last_target_id_ = -1;
  trackers_.clear();
  clearCrossingState();
  pending_target_click_ = false;
  pending_click_xy_only_ = false;
  manual_target_selected_ = false;
  has_last_target_measurement_ = false;
  last_target_measurement_stamp_sec_ = -1.0;
  occupancy_map_.clear();
  cloud_frame_index_ = 0;
  background_chunks_.clear();
  background_candidates_.clear();
  background_last_seen_.clear();
  background_first_seen_.clear();
  persistent_background_indices_.clear();
  background_tree_ = std::make_shared<KD_TREE<PointType>>(
    0.3, 0.6, params_.map_resolution);
  next_track_id_ = 0;
  last_cloud_stamp_sec_ = -1.0;
  body_radius_=0.0; body_radius_target_id_=-1; body_radius_stamp_=-1.0;body_center_offset_.setZero();
}

void MappingRos::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (!has_odom_) {
    ROS_WARN_THROTTLE(2.0, "Waiting for valid odometry in the configured world frame");
    return;
  }

  ros::Time cloud_stamp(msg->header.stamp);
  if (cloud_stamp.isZero()) {
    ROS_WARN_THROTTLE(2.0, "Dropping cloud with zero measurement timestamp");
    return;
  }
  const double stamp_sec = cloud_stamp.toSec();
  const double input_age_ms = (ros::Time::now() - cloud_stamp).toSec() * 1000.0;
  if (input_age_ms > max_cloud_age_ * 1000.0 || input_age_ms < -max_cloud_age_ * 1000.0) {
    ROS_WARN_THROTTLE(2.0, "Dropping cloud with invalid age %.1f ms; check sensor clocks", input_age_ms);
    return;
  }
  auto frame_name = [](std::string name) {
    while (!name.empty() && name.front() == '/') name.erase(name.begin());
    return name;
  };
  if (params_.cloud_is_registered && frame_name(msg->header.frame_id) != frame_name(params_.world_frame)) {
    ROS_WARN_THROTTLE(2.0, "Registered cloud frame '%s' differs from world_frame '%s'; transform upstream",
      msg->header.frame_id.c_str(), params_.world_frame.c_str());
    return;
  }
  Eigen::Isometry3d world_from_body;
  if (!poseAt(odom_history_, cloud_stamp, world_from_body)) {
    ROS_WARN_THROTTLE(2.0, "No timestamp-matched odometry for cloud");
    return;
  }
  odom_position_ = world_from_body.translation();
  odom_orientation_ = Eigen::Quaterniond(world_from_body.linear());
  if (last_cloud_stamp_sec_ >= 0.0 && stamp_sec + 1e-6 < last_cloud_stamp_sec_) {
    ROS_WARN( "Time moved backwards; resetting background and tracks");
    resetTrackingState();
  }
  if (last_cloud_stamp_sec_ >= 0 && std::abs(stamp_sec - last_cloud_stamp_sec_) < 1e-6) return;
  expireTarget(cloud_stamp);

  std_msgs::Header output_header = msg->header;
  output_header.stamp = cloud_stamp;
  if (!params_.world_frame.empty()) {
    output_header.frame_id = params_.world_frame;
  }

  const auto start = std::chrono::steady_clock::now();
  PointCloudPtr filtered = preprocessCloud(*msg);
  publishPointCloud(*filtered, output_header, filtered_cloud_pub_);
  cacheScan(filtered, output_header);

  // Tracker observation path: the current voxel observation is rebuilt for
  // every scan, so vanished moving points vanish immediately. The accumulated
  // occupancy map is visualization-only and never feeds clustering/tracking.
  updateVoxelMap(filtered);
  PointCloudPtr occupied;
  const bool publish_occupied = voxel_map_pub_.getNumSubscribers() > 0U;
  const bool publish_inflated = inflated_voxel_map_pub_.getNumSubscribers() > 0U;
  if (publish_occupied || publish_inflated) {
    occupied = extractOccupiedCloud();
  }
  if (publish_occupied) {
    publishPointCloud(*occupied, output_header, voxel_map_pub_);
  }
  if (publish_inflated) {
    PointCloudPtr inflated = buildInflatedCloud(occupied);
    publishPointCloud(*inflated, output_header, inflated_voxel_map_pub_);
  }
  if (detection_voxel_pub_.getNumSubscribers() > 0U) {
    publishPointCloud(*filtered, output_header, detection_voxel_pub_);
  }

  predictTrackers(stamp_sec);
  expireFusionEvidence(cloud_stamp);
  tryInitializeFromYolo(cloud_stamp);
  PointCloudPtr foreground = extractForegroundCloud(filtered);
  if (foreground_cloud_pub_.getNumSubscribers() > 0U) {
    publishPointCloud(*foreground, output_header, foreground_cloud_pub_);
  }
  const auto coarse_clusters = clusterCloud(foreground);
  const auto clusters = refineInteractionClusters(foreground, coarse_clusters);
  publishClusterCloud(foreground, clusters, output_header);

  PointCloud dynamic_points;
  DetectionStats detection_stats;
  auto detections = buildDetections(
    foreground, clusters, &detection_stats, !params_.manual_selection_enabled);
  if (params_.manual_selection_enabled) {
    resetMergeDebug();
    const auto candidates = detections;
    detections = updateManualTarget(candidates, foreground, filtered, stamp_sec);
    updateCleanSupportMemories(candidates, foreground, stamp_sec);
  } else {
    updateTrackers(detections, stamp_sec);
  }

  for (const auto & detection : detections) {
    const PointCloudPtr & source_cloud =
      detection.indices_from_current_cloud ? filtered : foreground;
    for (const int index : detection.indices.indices) {
      if (index >= 0 && static_cast<std::size_t>(index) < source_cloud->size()) {
        dynamic_points.push_back(source_cloud->points[static_cast<std::size_t>(index)]);
      }
    }
  }
  dynamic_points.width = dynamic_points.size();
  dynamic_points.height = 1;
  dynamic_points.is_dense = true;

  publishPointCloud(dynamic_points, output_header, dynamic_cloud_pub_);
  publishMergeDebug(output_header);
  last_cloud_stamp_sec_ = stamp_sec;
  publishTracks(output_header);
  publishMarkers(output_header);
  updateBackground(filtered);
  last_cloud_stamp_sec_ = stamp_sec;

  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  ROS_INFO_THROTTLE(1.0,
    "scan_voxels=%zu foreground=%zu map_cells=%zu clusters=%zu detections=%zu "
    "shape_rejected=%zu dynamic_rejected=%zu tracks=%zu auxiliary=%zu crossing=%s selection=%s "
    "process=%.2f ms input_age=%.1f ms",
    filtered->size(), foreground->size(), occupancy_map_.size(), clusters.size(),
    detections.size(), detection_stats.shape_rejected, detection_stats.dynamic_rejected,
    trackers_.size(), auxiliary_tracks_.size(),
    !identity_hypotheses_.empty() ? "hypotheses" :
    (crossing_occlusion_active_ ? "merged" : "clear"),
    !params_.manual_selection_enabled ? "automatic" :
    (pending_target_click_ ? "waiting_cluster" :
    (manual_target_selected_ ? "tracking" : "waiting_click")), elapsed_ms, input_age_ms);
}

}  // namespace person_tracker
