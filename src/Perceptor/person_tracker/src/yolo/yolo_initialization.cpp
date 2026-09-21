#include "core/mapping_manager.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace person_tracker
{
namespace
{
std::string frameName(std::string name)
{
  while (!name.empty() && name.front() == '/') name.erase(name.begin());
  return name;
}
Eigen::Isometry3d odometryPose(const nav_msgs::Odometry & msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const auto & q = msg.pose.pose.orientation;
  pose.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  const auto & p = msg.pose.pose.position;
  pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return pose;
}
}  // namespace

void MappingRos::loadYoloParameters()
{
  yolo_enabled_ = readParameter<bool>("yolo/enabled", true);
  target_label_ = readParameter<std::string>("target_label", "car");
  boxes_topic_ = readParameter<std::string>("yolo/topic", "/yoloe/plot");
  camera_odom_topic_ = readParameter<std::string>("yolo/camera_odom_topic", params_.odom_topic);
  yolo_min_probability_ = readParameter<double>("yolo/min_probability", 0.4);
  yolo_max_age_ = readParameter<double>("yolo/max_detection_age", 0.8);
  yolo_cloud_slop_ = readParameter<double>("yolo/cloud_sync_slop", 0.12);
  history_duration_ = readParameter<double>("yolo/history_duration", 2.0);
  yolo_confirmation_frames_ = readParameter<int>("yolo/confirmation_frames", 2);
  yolo_confirmation_distance_ = readParameter<double>("yolo/confirmation_distance", 0.5);
  yolo_reacquisition_enabled_ = readParameter<bool>("yolo/reacquisition_enabled", true);
  yolo_reacquisition_max_distance_ = readParameter<double>(
    "yolo/reacquisition_max_distance", 2.5);
  yolo_reacquisition_max_z_difference_ = readParameter<double>(
    "yolo/reacquisition_max_z_difference", 0.8);
  yolo_continuous_validation_enabled_ = readParameter<bool>(
    "yolo/continuous_validation_enabled", true);
  yolo_continuous_validation_max_age_ = readParameter<double>(
    "yolo/continuous_validation_max_age", 0.30);
  yolo_continuous_validation_distance_ = readParameter<double>(
    "yolo/continuous_validation_distance", 0.60);
  fusion_enabled_ = readParameter<bool>(
    "fusion/enabled", yolo_continuous_validation_enabled_);
  const auto policy = readParameter<std::string>("fusion/policy", "legacy");
  if (policy != "legacy" && policy != "lidar_evidence")
    throw std::runtime_error("fusion/policy must be legacy or lidar_evidence");
  lidar_evidence_policy_ = policy == "lidar_evidence";
  fusion_camera_margin_pixels_ = readParameter<double>(
    "fusion/camera_margin_pixels", 8.0);
  fusion_consistent_distance_ = readParameter<double>(
    "fusion/consistent_distance", 0.45);
  fusion_conflict_distance_ = readParameter<double>(
    "fusion/conflict_distance", 0.70);
  fusion_reselection_max_distance_ = readParameter<double>(
    "fusion/reselection_max_distance", yolo_reacquisition_max_distance_);
  const int fusion_conflict_frames = readParameter<int>(
    "fusion/conflict_frames", 3);
  const int fusion_confirmation_frames = readParameter<int>(
    "fusion/relock_confirmation_frames", 2);
  const double fusion_confirmation_radius = readParameter<double>(
    "fusion/relock_confirmation_radius", 0.45);
  fusion_state_machine_ = FusionStateMachine(
    {fusion_conflict_frames, fusion_confirmation_frames,
      fusion_confirmation_radius});
  max_cloud_age_ = readParameter<double>("max_cloud_age", 0.5);
  max_prediction_publish_age_ = readParameter<double>("output/max_prediction_age", 2.0);
  identity_timeout_ = readParameter<double>("output/identity_timeout", 6.0);
  projection_.fx = readParameter<double>("cam_fx", projection_.fx);
  projection_.fy = readParameter<double>("cam_fy", projection_.fy);
  projection_.cx = readParameter<double>("cam_cx", projection_.cx);
  projection_.cy = readParameter<double>("cam_cy", projection_.cy);
  projection_.width = readParameter<int>("cam_width", projection_.width);
  projection_.height = readParameter<int>("cam_height", projection_.height);
  projection_.box_inset = readParameter<double>("yolo/box_inset", 0.05);
  projection_.min_depth = readParameter<double>("yolo/min_depth", 0.3);
  projection_.max_depth = readParameter<double>("yolo/max_depth", 30.0);
  projection_.cluster_tolerance = readParameter<double>("yolo/cluster_tolerance", 0.3);
  projection_.min_points = readParameter<int>("yolo/min_points", 5);
  projection_.min_extent = readParameter<double>("yolo/min_cluster_extent", 0.1);
  projection_.max_extent = readParameter<double>("yolo/max_cluster_extent", 4.0);
  projection_.min_box_coverage = readParameter<double>("yolo/min_box_coverage", 0.02);

  const auto rotation = readParameter<std::vector<double>>("cam2body_R",
    {0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0});
  const auto translation = readParameter<std::vector<double>>("cam2body_p", {0.0, 0.0, 0.1});
  if (rotation.size() != 9 || translation.size() != 3) {
    throw std::runtime_error("cam2body_R must contain 9 values and cam2body_p 3 values");
  }
  body_from_camera_.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.data());
  body_from_camera_.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  const auto & r = body_from_camera_.linear();
  if (!body_from_camera_.matrix().allFinite() ||
      (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() > 1e-3 ||
      std::abs(r.determinant() - 1.0) > 1e-3) {
    throw std::runtime_error("cam2body_R must be an orthonormal rotation with determinant +1");
  }
  const std::vector<double> finite_values{projection_.fx, projection_.fy, projection_.cx,
    projection_.cy, projection_.min_depth, projection_.max_depth, projection_.box_inset,
    projection_.cluster_tolerance, projection_.min_extent, projection_.max_extent,
    projection_.min_box_coverage, yolo_min_probability_, yolo_max_age_, yolo_cloud_slop_,
    history_duration_, yolo_confirmation_distance_, max_cloud_age_,
    max_prediction_publish_age_, identity_timeout_, params_.max_odom_age,
    yolo_reacquisition_max_distance_, yolo_reacquisition_max_z_difference_,
    yolo_continuous_validation_max_age_, yolo_continuous_validation_distance_,
    fusion_camera_margin_pixels_,
    fusion_consistent_distance_, fusion_conflict_distance_,
    fusion_reselection_max_distance_, fusion_confirmation_radius};
  if (std::any_of(finite_values.begin(), finite_values.end(), [](double x) {return !std::isfinite(x);}) ||
      projection_.fx <= 0 || projection_.fy <= 0 || projection_.width <= 0 || projection_.height <= 0 ||
      projection_.min_depth <= 0 || projection_.max_depth <= projection_.min_depth ||
      projection_.min_points < 3 || projection_.cluster_tolerance <= 0 ||
      projection_.box_inset < 0 || projection_.box_inset >= 0.5 ||
      projection_.min_extent < 0 || projection_.max_extent <= projection_.min_extent ||
      projection_.min_box_coverage < 0 || projection_.min_box_coverage > 1 ||
      yolo_min_probability_ < 0 || yolo_min_probability_ > 1 ||
      yolo_max_age_ <= 0 || yolo_cloud_slop_ <= 0 ||
      history_duration_ < yolo_max_age_ + yolo_cloud_slop_ ||
      yolo_confirmation_frames_ < 1 || yolo_confirmation_distance_ <= 0 ||
      yolo_reacquisition_max_distance_ <= 0 || yolo_reacquisition_max_z_difference_ <= 0 ||
      yolo_continuous_validation_max_age_ <= 0 ||
      yolo_continuous_validation_max_age_ > yolo_max_age_ ||
      yolo_continuous_validation_distance_ <= 0 ||
      fusion_camera_margin_pixels_ < 0 || fusion_consistent_distance_ <= 0 ||
      fusion_conflict_distance_ < fusion_consistent_distance_ ||
      fusion_reselection_max_distance_ < fusion_conflict_distance_ ||
      fusion_conflict_frames < 1 || fusion_confirmation_frames < 1 ||
      fusion_confirmation_radius <= 0 ||
      max_cloud_age_ <= 0 || max_prediction_publish_age_ < 0 ||
      identity_timeout_ <= max_prediction_publish_age_ || params_.max_odom_age <= 0 ||
      params_.world_frame.empty() || target_label_.empty()) {
    throw std::runtime_error("Invalid camera, synchronization, initialization or target timeout parameters");
  }
  if (yolo_enabled_) params_.manual_selection_enabled = true;
}

bool MappingRos::appendOdometry(const nav_msgs::Odometry::ConstPtr & msg,
  std::deque<nav_msgs::Odometry::ConstPtr> & history)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (msg->header.stamp.isZero() || frameName(msg->header.frame_id) != frameName(params_.world_frame) ||
      !Eigen::Vector3d(p.x, p.y, p.z).allFinite() || !orientation.coeffs().allFinite() ||
      orientation.norm() < 1e-6) {
    ROS_WARN_THROTTLE(2.0, "Rejecting odometry: require a valid stamped pose in world_frame=%s", params_.world_frame.c_str());
    return false;
  }
  if (!history.empty() && msg->header.stamp < history.back()->header.stamp &&
      (history.back()->header.stamp - msg->header.stamp).toSec() > history_duration_) history.clear();
  auto location = std::lower_bound(history.begin(), history.end(), msg->header.stamp,
    [](const nav_msgs::Odometry::ConstPtr & entry, const ros::Time & stamp) {return entry->header.stamp < stamp;});
  if (location != history.end() && (*location)->header.stamp == msg->header.stamp) *location = msg;
  else history.insert(location, msg);
  while (history.size() > 1000 || (history.size() > 1 &&
    (history.back()->header.stamp - history.front()->header.stamp).toSec() > history_duration_)) history.pop_front();
  return true;
}

bool MappingRos::poseAt(const std::deque<nav_msgs::Odometry::ConstPtr> & history,
  const ros::Time & stamp, Eigen::Isometry3d & pose) const
{
  if (history.empty() || stamp.isZero()) return false;
  const auto after = std::lower_bound(history.begin(), history.end(), stamp,
    [](const nav_msgs::Odometry::ConstPtr & entry, const ros::Time & t) {return entry->header.stamp < t;});
  if (after != history.end() && (*after)->header.stamp == stamp) {
    pose = odometryPose(**after);
    return true;
  }
  if (after == history.begin() || after == history.end()) {
    const auto & nearest = after == history.end() ? history.back() : history.front();
    if (std::abs((stamp - nearest->header.stamp).toSec()) > params_.max_odom_age) return false;
    pose = odometryPose(*nearest);
    return true;
  }
  const auto & before = *(after - 1);
  if ((stamp - before->header.stamp).toSec() > params_.max_odom_age ||
      ((*after)->header.stamp - stamp).toSec() > params_.max_odom_age) return false;
  const double interval = ((*after)->header.stamp - before->header.stamp).toSec();
  const double alpha = (stamp - before->header.stamp).toSec() / interval;
  const auto a = odometryPose(*before), b = odometryPose(**after);
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = (1.0 - alpha) * a.translation() + alpha * b.translation();
  pose.linear() = Eigen::Quaterniond(a.linear()).slerp(alpha, Eigen::Quaterniond(b.linear())).toRotationMatrix();
  return true;
}

void MappingRos::cameraOdomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
  appendOdometry(msg, camera_odom_history_);
}

void MappingRos::boxesCallback(const tracking_detector::BoundingBoxes::ConstPtr & msg)
{
  evidence_boxes_ = msg;
  // Retain only the newest semantic observation. Negative detector evidence
  // is meaningful only if the EKF-predicted target centre is geometrically in
  // the image. OUT_OF_VIEW and UNKNOWN are fail-open for the lidar tracker.
  const bool has_target_box = std::any_of(
    msg->bounding_boxes.begin(), msg->bounding_boxes.end(), [&](const auto & box) {
      return box.Class == target_label_ && std::isfinite(box.probability) &&
        box.probability >= yolo_min_probability_;
    });
  fusion_yolo_target_present_ = has_target_box;
  if (fusion_enabled_ && manual_target_selected_ && !trackers_.empty() &&
    !has_target_box)
  {
    const ros::Time image_stamp = msg->image_header.stamp.isZero() ?
      msg->header.stamp : msg->image_header.stamp;
    const CameraVisibility visibility = cameraVisibility(
      predictedTargetAt(*trackers_.front(), image_stamp), image_stamp);
    fusion_camera_projection_valid_ = visibility.projection_valid;
    fusion_predicted_in_fov_ = visibility.in_fov;
    fusion_yolo_component_valid_ = false;
    fusion_candidate_eligible_ = false;
    clearYoloDebugClouds(image_stamp, true);
    if (!visibility.projection_valid) {
      observeFusion(SemanticEvidence::UNKNOWN, image_stamp,
        "camera pose/projection unavailable; lidar-only fallback");
    } else if (!visibility.in_fov) {
      observeFusion(SemanticEvidence::OUT_OF_VIEW, image_stamp,
        "predicted target is outside camera frustum; YOLO ignored");
    } else {
      observeFusion(SemanticEvidence::UNKNOWN, image_stamp,
        "predicted target is visible but YOLO target is absent; lidar continues");
    }
  }
  if (yolo_enabled_ && !pending_target_click_) {
    pending_boxes_ = msg;
    pending_boxes_received_ = ros::WallTime::now();
  }
}

void MappingRos::cacheScan(const PointCloudPtr & cloud, const std_msgs::Header & header)
{
  const bool continuous_validation = fusion_enabled_ &&
    manual_target_selected_ && !trackers_.empty();
  if (!yolo_enabled_ || (!yolo_armed_ && !continuous_validation)) return;
  scan_history_.push_back({header, cloud, ros::WallTime::now()});
  while (scan_history_.size() > 60 || (scan_history_.size() > 1 &&
    (header.stamp - scan_history_.front().header.stamp).toSec() > history_duration_)) scan_history_.pop_front();
}

void MappingRos::tryInitializeFromYolo(const ros::Time & current_stamp)
{
  const bool continuous_validation = fusion_enabled_ &&
    manual_target_selected_ && !trackers_.empty() && (lidar_evidence_policy_ || !yolo_reacquisition_active_);
  if (!yolo_enabled_ || (!yolo_armed_ && !continuous_validation) || pending_target_click_ ||
      !pending_boxes_ || scan_history_.empty()) return;
  const ros::Time image_stamp = pending_boxes_->image_header.stamp.isZero() ?
    pending_boxes_->header.stamp : pending_boxes_->image_header.stamp;
  if (image_stamp.isZero() || image_stamp <= yolo_processed_stamp_) return;
  if (continuous_validation) {
    const Eigen::Vector3d image_time_prediction =
      predictedTargetAt(*trackers_.front(), image_stamp);
    const CameraVisibility visibility = cameraVisibility(
      image_time_prediction, image_stamp);
    fusion_camera_projection_valid_ = visibility.projection_valid;
    fusion_predicted_in_fov_ = visibility.in_fov;
    if (!visibility.projection_valid || !visibility.in_fov) {
      yolo_processed_stamp_ = image_stamp;
      fusion_yolo_component_valid_ = false;
      fusion_candidate_eligible_ = false;
      clearYoloDebugClouds(image_stamp, true);
      observeFusion(
        visibility.projection_valid ? SemanticEvidence::OUT_OF_VIEW :
        SemanticEvidence::UNKNOWN,
        image_stamp,
        visibility.projection_valid ?
        "predicted target is outside camera frustum; YOLO ignored" :
        "camera pose/projection unavailable; lidar-only fallback");
      pending_boxes_.reset();
      return;
    }
  }
  // Same-clock: pair by message stamps. Dual-clock (Unity camera vs ROS
  // LiDAR): pair the newest box with the nearest scan by wall arrival.
  const bool same_clock = std::abs((current_stamp - image_stamp).toSec()) <= 2.0;
  const double age = same_clock ?
    (current_stamp - image_stamp).toSec() :
    (ros::WallTime::now() - pending_boxes_received_).toSec();
  if (same_clock && age < -yolo_cloud_slop_) return;
  if (age > yolo_max_age_) {
    yolo_confirmation_hits_ = 0;
    if (continuous_validation) {
      fusion_yolo_component_valid_ = false;
      fusion_candidate_eligible_ = false;
      clearYoloDebugClouds(current_stamp, true);
      observeFusion(SemanticEvidence::UNKNOWN, current_stamp,
        "YOLO image is too old; lidar-only fallback");
    }
    pending_boxes_.reset();
    ROS_WARN_THROTTLE(2.0, "YOLO image is too old for initialization; check inference latency and stamps");
    return;
  }
  const CachedScan * scan = nullptr;
  double closest = same_clock ? yolo_cloud_slop_ : yolo_max_age_;
  for (const auto & candidate : scan_history_) {
    const double stamp_diff = std::abs((candidate.header.stamp - image_stamp).toSec());
    const double wall_diff = std::abs((candidate.received - pending_boxes_received_).toSec());
    const double difference = same_clock ? stamp_diff : wall_diff;
    if (difference <= closest) {closest = difference; scan = &candidate;}
  }
  Eigen::Isometry3d world_from_body;
  const auto & camera_history = camera_odom_topic_ == params_.odom_topic ? odom_history_ : camera_odom_history_;
  if (!scan || !poseAt(camera_history, image_stamp, world_from_body)) {
    ROS_WARN_THROTTLE(2.0, same_clock ?
      "Waiting for timestamp-matched cloud and camera odometry for YOLO initialization" :
      "Waiting for wall-time-matched cloud and camera odometry for YOLO initialization");
    return;
  }
  yolo_processed_stamp_ = image_stamp;
  std::vector<tracking_detector::BoundingBox> boxes;
  for (const auto & box : pending_boxes_->bounding_boxes) {
    if (box.Class == target_label_ && std::isfinite(box.probability) &&
        box.probability >= yolo_min_probability_) boxes.push_back(box);
  }
  std::stable_sort(boxes.begin(), boxes.end(), [](const auto & a, const auto & b) {
    return a.probability > b.probability;
  });
  fusion_yolo_target_present_ = !boxes.empty();
  if (continuous_validation && boxes.empty()) {
    fusion_yolo_component_valid_ = false;
    fusion_candidate_eligible_ = false;
    clearYoloDebugClouds(image_stamp, true);
    observeFusion(SemanticEvidence::UNKNOWN, image_stamp,
      "predicted target is visible but YOLO target is absent; lidar continues");
    pending_boxes_.reset();
    return;
  }
  std::optional<ProjectedSeed> seed;
  std::optional<ProjectedSeed> geometric_seed;
  Detection geometric_detection;
  double geometric_best_distance = std::numeric_limits<double>::infinity();
  Detection detection;
  double recovery_best_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d recovery_reference = yolo_reacquisition_reference_;
  bool recovery_reference_valid = yolo_reacquisition_reference_valid_;
  Eigen::Vector3d reference_size = yolo_reacquisition_size_;
  if (continuous_validation) {
    // cloudCallback predicts the EKF to the current scan before this method.
    // The predicted target, not the previous YOLO observation, is the reference
    // used to audit whether camera and lidar still describe the same target.
    recovery_reference = predictedTargetAt(*trackers_.front(), image_stamp);
    reference_size = trackers_.front()->size();
    recovery_reference_valid = true;
  }
  // Lost-track recovery cannot use a frozen old position as a hard XY gate.
  // Keep distance only for candidate ranking; a recovered detection starts
  // a new identity under the lidar-evidence policy.
  const double recovery_distance_limit =
    lidar_evidence_policy_ && yolo_reacquisition_active_ && trackers_.empty() ?
    std::numeric_limits<double>::infinity() : yolo_reacquisition_max_distance_;
  for (const auto & box : boxes) {
    const auto candidate_seed = selectProjectedSeed(scan->cloud,
      {double(box.xmin), double(box.ymin), double(box.xmax), double(box.ymax)},
      world_from_body * body_from_camera_, projection_);
    Detection candidate_detection;
    if (!candidate_seed ||
        !buildDetectionFromIndices(scan->cloud, candidate_seed->indices, candidate_detection)) {
      continue;
    }
    // Publish the closest geometrically supported component even when the
    // frozen identity gate later rejects it.  This makes "no lidar component"
    // and "component belongs to a different/far identity" observable as two
    // different conditions in RViz.
    const double geometric_distance = recovery_reference_valid ?
      (candidate_detection.position.head<2>() - recovery_reference.head<2>()).norm() : 0.0;
    if (!geometric_seed || geometric_distance < geometric_best_distance) {
      geometric_seed = candidate_seed;
      geometric_detection = candidate_detection;
      geometric_best_distance = geometric_distance;
    }
    if (!yolo_reacquisition_active_ && !continuous_validation) {
      seed = candidate_seed;
      detection = candidate_detection;
      break;
    }
    if (!recovery_reference_valid) {
      continue;
    }
    const Eigen::Vector3d offset = candidate_detection.position - recovery_reference;
    const double horizontal_distance = offset.head<2>().norm();
    const double candidate_width = std::max(
      candidate_detection.size.x(), candidate_detection.size.y());
    const double reference_width = std::max(reference_size.x(), reference_size.y());
    const bool size_compatible = params_.selection_fixed_body_size_enabled ?
      (candidate_width <= params_.selection_size_compatibility_max_ratio * reference_width &&
      candidate_detection.size.z() <=
      params_.selection_size_compatibility_max_ratio * reference_size.z()) :
      compatibleTargetSize(candidate_detection.size, reference_size);
    if (horizontal_distance > recovery_distance_limit ||
        (!params_.selection_fixed_body_size_enabled &&
        std::abs(offset.z()) > yolo_reacquisition_max_z_difference_) ||
        !size_compatible) {
      if (yolo_reacquisition_active_) {
        ROS_WARN_THROTTLE(1.0,
          "YOLO recovery candidate rejected by identity gate: xy=%.2f/%.2f m, "
          "z=%.2f/%.2f m, size=[%.2f, %.2f, %.2f]",
          horizontal_distance, recovery_distance_limit, std::abs(offset.z()),
          yolo_reacquisition_max_z_difference_, candidate_detection.size.x(),
          candidate_detection.size.y(), candidate_detection.size.z());
      }
      continue;
    }
    // YOLO says which 2-D region is a person; spatial continuity decides
    // which person keeps the selected lidar identity when several boxes exist.
    if (horizontal_distance < recovery_best_distance) {
      recovery_best_distance = horizontal_distance;
      seed = candidate_seed;
      detection = candidate_detection;
    }
  }
  if (geometric_seed) {
    PointCloud candidate_cloud;
    candidate_cloud.reserve(geometric_seed->indices.size());
    for (const int index : geometric_seed->indices) {
      candidate_cloud.push_back(scan->cloud->points[static_cast<std::size_t>(index)]);
    }
    publishPointCloud(candidate_cloud, scan->header, yolo_candidate_pub_);
  }
  if (!continuous_validation) {
    clearYoloDebugClouds(scan->header.stamp, !geometric_seed.has_value());
  }
  if (continuous_validation) {
    fusion_yolo_component_valid_ = geometric_seed.has_value();
    if (!geometric_seed) {
      fusion_candidate_eligible_ = false;
      clearYoloDebugClouds(image_stamp, true);
      observeFusion(SemanticEvidence::UNKNOWN, image_stamp,
        "YOLO box has no supported lidar component; lidar continues");
      pending_boxes_.reset();
      return;
    }
    // A green/magenta accepted seed exists only while a SUSPECT replacement
    // has passed all confirmations. Clear the previous seed every other frame.
    clearYoloDebugClouds(scan->header.stamp, false);

    if (params_.selection_fixed_body_size_enabled) {
      geometric_detection.position.z() = recovery_reference.z();
    }
    const double distance =
      (geometric_detection.position.head<2>() -
      recovery_reference.head<2>()).norm();
    const double candidate_width = std::max(
      geometric_detection.size.x(), geometric_detection.size.y());
    const double reference_width = std::max(reference_size.x(), reference_size.y());
    const bool size_compatible = params_.selection_fixed_body_size_enabled ?
      (candidate_width <= params_.selection_size_compatibility_max_ratio * reference_width &&
      geometric_detection.size.z() <=
      params_.selection_size_compatibility_max_ratio * reference_size.z()) :
      compatibleTargetSize(geometric_detection.size, reference_size);
    const bool vertical_compatible = params_.selection_fixed_body_size_enabled ||
      std::abs(geometric_detection.position.z() - recovery_reference.z()) <=
      yolo_reacquisition_max_z_difference_;
    if (distance <= fusion_consistent_distance_) {
      fusion_candidate_eligible_ = false;
      observeFusion(SemanticEvidence::CONSISTENT, scan->header.stamp,
        "YOLO-supported lidar component agrees with EKF prediction",
        geometric_detection.position.head<2>(), distance);
      yolo_semantic_observation_valid_ = false;
      yolo_semantic_stamp_ = ros::Time();
    } else if (distance >= fusion_conflict_distance_) {
      const bool inside_identity_bound =
        distance <= fusion_reselection_max_distance_ && size_compatible &&
        vertical_compatible;
      fusion_candidate_eligible_ = inside_identity_bound;
      observeFusion(SemanticEvidence::CONFLICT, scan->header.stamp,
        inside_identity_bound ?
        "YOLO-supported lidar component conflicts with EKF prediction" :
        "conflict failed automatic distance/height/size identity bounds",
        geometric_detection.position.head<2>(), distance);
      yolo_semantic_observation_valid_ = inside_identity_bound;
      if (inside_identity_bound) {
        yolo_semantic_position_ = geometric_detection.position;
        yolo_semantic_size_ = geometric_detection.size;
        yolo_semantic_stamp_ = scan->header.stamp;
      } else {
        yolo_semantic_stamp_ = ros::Time();
      }
      if (inside_identity_bound &&
        fusion_state_machine_.state() == FusionState::SUSPECT &&
        fusion_state_machine_.reselectionReady())
      {
        PointCloud seed_cloud;
        seed_cloud.reserve(geometric_seed->indices.size());
        for (const int index : geometric_seed->indices) {
          seed_cloud.push_back(scan->cloud->points[static_cast<std::size_t>(index)]);
        }
        publishPointCloud(seed_cloud, scan->header, yolo_seed_pub_);
      }
    } else {
      fusion_candidate_eligible_ = false;
      observeFusion(SemanticEvidence::UNKNOWN, scan->header.stamp,
        "camera/lidar distance is inside hysteresis band", std::nullopt, distance);
    }
    if (lidar_evidence_policy_) {
      // Store evidence only; the lidar update must rebuild and validate a
      // current foreground candidate before it can use this search region.
      yolo_semantic_observation_valid_ = size_compatible && vertical_compatible;
      yolo_semantic_position_ = geometric_detection.position;
      yolo_semantic_size_ = geometric_detection.size;
      yolo_semantic_stamp_ = scan->header.stamp;
      fusion_candidate_eligible_ = false;
    }
    pending_boxes_.reset();
    return;
  }
  if (!seed) {
    yolo_confirmation_hits_ = 0;
    if (geometric_seed && recovery_reference_valid) {
      ROS_WARN_THROTTLE(1.0,
        "YOLO lidar component exists (%zu points) but identity gate rejected it: "
        "xy=%.2f/%.2f m; run /demo/run_demo.sh reset only if this is the intended person",
        geometric_seed->indices.size(), geometric_best_distance,
        recovery_distance_limit);
    }
    ROS_DEBUG_THROTTLE(2.0, yolo_reacquisition_active_ ?
      "No identity-gated point cluster inside a matching YOLO box" :
      "No supported point cluster inside a matching YOLO box");
    return;
  }
  if (yolo_reacquisition_active_ &&
      params_.selection_fixed_body_size_enabled) {
    // Sparse torso/leg returns must not create a vertical jump when the same
    // fixed-height person identity is re-anchored.
    detection.position.z() = recovery_reference.z();
  }
  PointCloud seed_cloud;
  for (int index : seed->indices) seed_cloud.push_back(scan->cloud->points[index]);
  publishPointCloud(seed_cloud, scan->header, yolo_seed_pub_);
  // Distinct detector messages matched to the same scan are one observation.
  if (scan->header.stamp <= yolo_candidate_stamp_) return;
  if (yolo_confirmation_hits_ == 0 ||
      (scan->header.stamp - yolo_candidate_stamp_).toSec() > yolo_max_age_ ||
      (detection.position - yolo_candidate_position_).norm() > yolo_confirmation_distance_) {
    yolo_confirmation_hits_ = 1;
  } else ++yolo_confirmation_hits_;
  yolo_candidate_position_ = detection.position;
  yolo_candidate_stamp_ = scan->header.stamp;
  if (yolo_confirmation_hits_ < yolo_confirmation_frames_) return;

  const bool recovered = yolo_reacquisition_active_;
  const int selected_id = recovered && !lidar_evidence_policy_ && yolo_reacquisition_target_id_ >= 0 ?
    yolo_reacquisition_target_id_ : next_track_id_++;
  clearCrossingState();
  anchorTargetHeight(
    detection.position, scan->cloud, scan->header.stamp,
    recovered && recovery_reference_valid ? recovery_reference.z() : detection.position.z(),
    !recovered);
  const auto size = params_.selection_fixed_body_size_enabled ? params_.selection_fixed_body_size : detection.size;
  trackers_.clear();
  trackers_.push_back(std::make_shared<TargetEkf>(selected_id, detection.position, size,
    scan->header.stamp.toSec(), params_.acceleration_noise, params_.measurement_noise,
    !params_.selection_fixed_body_size_enabled && params_.selection_adaptive_size_enabled,
    params_.selection_size_smoothing, params_.selection_size_max_relative_step));
  rememberBodyRadius(detection, selected_id, scan->header.stamp.toSec());
  manual_target_selected_ = true;
  target_ever_selected_ = true;
  fusion_state_machine_.lockTarget();
  fusion_reason_ = recovered ?
    "bounded YOLO+lidar recovery accepted with target ID preserved" :
    "YOLO+lidar initialization confirmed";
  yolo_armed_ = false;
  yolo_reacquisition_active_ = false;
  yolo_reacquisition_reference_valid_ = false;
  yolo_reacquisition_target_id_ = -1;
  yolo_reacquisition_size_.setZero();
  yolo_semantic_observation_valid_ = true;
  yolo_semantic_position_ = detection.position;
  yolo_semantic_size_ = detection.size;
  yolo_semantic_stamp_ = scan->header.stamp;
  has_last_target_measurement_ = true;
  last_target_measurement_position_ = detection.position;
  last_target_measurement_stamp_sec_ = scan->header.stamp.toSec();
  target_observed_size_ = detection.size;
  target_observed_point_count_ = double(seed->indices.size());
  // A stationary target can already be in the very first background scan.
  // Semantic initialization removes ONLY the selected cluster's exact voxels.
  removeSeedFromBackground(scan->cloud, seed->indices);
  if (recovered) {
    ROS_INFO("YOLO+cloud reacquired lidar target %d (%s): %zu points, depth=%.2f m, "
      "reference distance=%.2f m, measurement age=%.3f s",
      trackers_.front()->id(), target_label_.c_str(), seed->indices.size(), seed->depth,
      recovery_best_distance, (current_stamp - scan->header.stamp).toSec());
  } else {
    ROS_INFO("YOLO initialized target %d (%s): %zu points, depth=%.2f m, measurement age=%.3f s",
      trackers_.front()->id(), target_label_.c_str(), seed->indices.size(), seed->depth,
      (current_stamp - scan->header.stamp).toSec());
  }
  pending_boxes_.reset();
  scan_history_.clear();
}

void MappingRos::armYoloReacquisition(const std::string & reason)
{
  if (!yolo_enabled_ || !yolo_reacquisition_enabled_ || pending_target_click_) return;
  if (!yolo_reacquisition_active_) {
    if (!trackers_.empty()) {
      const bool recent_semantic_reference = !lidar_evidence_policy_ && yolo_continuous_validation_enabled_ &&
        yolo_semantic_observation_valid_ && !yolo_semantic_stamp_.isZero() &&
        last_cloud_stamp_sec_ >= 0.0 &&
        std::abs(last_cloud_stamp_sec_ - yolo_semantic_stamp_.toSec()) <=
        yolo_continuous_validation_max_age_;
      yolo_reacquisition_target_id_ = trackers_.front()->id();
      yolo_reacquisition_reference_ = recent_semantic_reference ? yolo_semantic_position_ :
        (has_last_target_measurement_ ? last_target_measurement_position_ :
        trackers_.front()->associationPosition());
      yolo_reacquisition_size_ = trackers_.front()->size();
      yolo_reacquisition_reference_valid_ = true;
    } else if (has_last_target_measurement_) {
      yolo_reacquisition_reference_ = last_target_measurement_position_;
      yolo_reacquisition_size_ = params_.selection_fixed_body_size_enabled ?
        params_.selection_fixed_body_size : target_observed_size_;
      if (!yolo_reacquisition_size_.allFinite() ||
          yolo_reacquisition_size_.minCoeff() <= 0.0) {
        yolo_reacquisition_size_.setConstant(params_.map_resolution);
      }
      yolo_reacquisition_reference_valid_ = true;
    }
    if (!yolo_reacquisition_reference_valid_) return;

    yolo_reacquisition_active_ = true;
    yolo_armed_ = true;
    yolo_confirmation_hits_ = 0;
    yolo_processed_stamp_ = ros::Time();
    yolo_candidate_stamp_ = ros::Time();
    scan_history_.clear();
    ROS_WARN("Lidar target support lost (%s); armed YOLO+cloud semantic reacquisition "
      "for target %d around [%.2f, %.2f, %.2f]",
      reason.c_str(), yolo_reacquisition_target_id_, yolo_reacquisition_reference_.x(),
      yolo_reacquisition_reference_.y(), yolo_reacquisition_reference_.z());
  }
}

void MappingRos::cancelYoloReacquisition()
{
  if (!yolo_reacquisition_active_) return;
  ROS_INFO("Lidar target support recovered before semantic reset; cancelling YOLO assistance");
  yolo_reacquisition_active_ = false;
  yolo_reacquisition_reference_valid_ = false;
  yolo_reacquisition_target_id_ = -1;
  yolo_reacquisition_size_.setZero();
  yolo_armed_ = false;
  yolo_confirmation_hits_ = 0;
  yolo_processed_stamp_ = ros::Time();
  yolo_candidate_stamp_ = ros::Time();
  pending_boxes_.reset();
  if (!yolo_continuous_validation_enabled_) scan_history_.clear();
}

void MappingRos::removeSeedFromBackground(const PointCloudPtr & cloud, const std::vector<int> & indices)
{
  std::unordered_set<VoxelIndex, VoxelIndexHash> seed_voxels;
  PointVector deleted;
  for (int i : indices) {
    const auto & p = cloud->points[i];
    seed_voxels.insert(positionToVoxel(Eigen::Vector3d(p.x, p.y, p.z)));
    // Persistent first-scan voxels may outlive the rolling chunk that stored
    // them. Current observations use the same fixed voxel centres.
    deleted.push_back(p);
  }
  for (auto & chunk : background_chunks_) {
    chunk.erase(std::remove_if(chunk.begin(), chunk.end(), [&](const PointType & p) {
      const auto index = positionToVoxel(Eigen::Vector3d(p.x, p.y, p.z));
      if (!seed_voxels.count(index)) return false;
      deleted.push_back(p);
      return true;
    }), chunk.end());
  }
  for (const auto & index : seed_voxels) {
    persistent_background_indices_.erase(index);
    background_last_seen_.erase(index);
    background_first_seen_.erase(index);
    background_candidates_.erase(index);
  }
  const int removed = deleteBackgroundVoxels(deleted);
  ROS_INFO("YOLO bootstrap selected %zu target voxels and removed %d background points",
    seed_voxels.size(), removed);
}

int MappingRos::deleteBackgroundVoxels(const PointVector & points)
{
  if (points.empty() || !background_tree_->Root_Node) return 0;
  std::unordered_set<VoxelIndex, VoxelIndexHash> unique;
  std::vector<BoxPointType> boxes;
  for (const auto & point : points) {
    const auto index = positionToVoxel(Eigen::Vector3d(point.x, point.y, point.z));
    if (!unique.insert(index).second) continue;
    const Eigen::Vector3d centre = voxelToPosition(index);
    BoxPointType box;
    for (int axis = 0; axis < 3; ++axis) {
      box.vertex_min[axis] = centre[axis] - 0.25 * params_.map_resolution;
      box.vertex_max[axis] = centre[axis] + 0.25 * params_.map_resolution;
    }
    boxes.push_back(box);
  }
  // Equal split-axis coordinates are common on the fixed voxel grid. The
  // vendored ikd-tree's point deletion can miss those ties; range deletion
  // visits both intersecting branches. Each box contains only ONE voxel centre.
  return background_tree_->Delete_Point_Boxes(boxes);
}
}  // namespace person_tracker
