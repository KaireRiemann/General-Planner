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

void MappingRos::resetGroundHeight()
{
  has_filtered_ground_height_ = false;
  filtered_ground_height_ = 0.0;
  latest_ground_estimate_ = GroundHeightEstimate{};
}

void MappingRos::publishGroundEstimate(
  const GroundHeightEstimate & estimate, double target_z,
  const PointCloudPtr & cloud, const ros::Time & stamp)
{
  std_msgs::Header header;
  header.stamp = stamp;
  header.frame_id = params_.world_frame;

  GroundEstimate message;
  message.header = header;
  message.valid = estimate.valid &&
    estimate.confidence >= params_.ground_height_minimum_confidence;
  message.ground_z = estimate.height;
  message.target_z = target_z;
  message.confidence = estimate.confidence;
  message.normal.x = estimate.normal.x();
  message.normal.y = estimate.normal.y();
  message.normal.z = estimate.normal.z();
  message.support_points = static_cast<std::uint32_t>(estimate.support_indices.size());
  ground_estimate_pub_.publish(message);

  if (ground_support_pub_.getNumSubscribers() == 0U) {
    return;
  }
  PointCloud support;
  if (cloud) {
    support.reserve(estimate.support_indices.size());
    for (const int index : estimate.support_indices) {
      if (index >= 0 && static_cast<std::size_t>(index) < cloud->size()) {
        support.push_back(cloud->points[static_cast<std::size_t>(index)]);
      }
    }
  }
  support.width = support.size();
  support.height = 1;
  support.is_dense = true;
  publishPointCloud(support, header, ground_support_pub_);
}

void MappingRos::anchorTargetHeight(
  Eigen::Vector3d & measurement, const PointCloudPtr & ground_cloud,
  const ros::Time & stamp, double reference_center_z, bool initialize)
{
  if (!params_.ground_height.enabled || !measurement.allFinite()) {
    return;
  }

  const double finite_reference = std::isfinite(reference_center_z) ?
    reference_center_z : measurement.z();
  GroundHeightEstimate estimate = ground_height_estimator_.estimate(
    ground_cloud, measurement.head<2>(), finite_reference);
  const bool usable = estimate.valid &&
    estimate.confidence >= params_.ground_height_minimum_confidence;

  double target_z = initialize ? measurement.z() : finite_reference;
  if (usable) {
    if (initialize || !has_filtered_ground_height_) {
      filtered_ground_height_ = estimate.height;
      has_filtered_ground_height_ = true;
    } else {
      const double bounded_change = std::clamp(
        estimate.height - filtered_ground_height_,
        -params_.ground_height_max_step, params_.ground_height_max_step);
      filtered_ground_height_ += params_.ground_height_smoothing * bounded_change;
    }
    target_z = filtered_ground_height_ + params_.ground_height.person_center_height;
  }

  // When ground support disappears, retain the predicted vertical state.  A
  // torso-only lidar centroid must never pull the track upward.
  measurement.z() = target_z;
  latest_ground_estimate_ = estimate;
  publishGroundEstimate(estimate, target_z, ground_cloud, stamp);
}

}  // namespace person_tracker
