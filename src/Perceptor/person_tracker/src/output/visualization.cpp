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

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "common/tracking_geometry.hpp"

namespace person_tracker
{

using detail::clusterColor;

void MappingRos::publishPointCloud(
  const PointCloud & cloud, const std_msgs::Header & header,
  const ros::Publisher & publisher) const
{
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header = header;
  publisher.publish(message);
}

void MappingRos::publishMergeDebug(const std_msgs::Header & header)
{
  if (!params_.merge_v1_debug_enabled || merge_target_support_pub_ == nullptr) {
    return;
  }
  // Publish one empty set after leaving MERGE to clear RViz, but do not spend
  // six PointCloud2 conversions on every ordinary NORMAL frame.
  if (!merge_debug_active_ && !merge_debug_was_published_) {
    return;
  }
  publishPointCloud(merge_debug_target_support_, header, merge_target_support_pub_);
  publishPointCloud(merge_debug_distractor_support_, header, merge_distractor_support_pub_);
  publishPointCloud(merge_debug_merged_, header, merge_cluster_pub_);
  publishPointCloud(merge_debug_target_assigned_, header, merge_target_assigned_pub_);
  publishPointCloud(
    merge_debug_distractor_assigned_, header, merge_distractor_assigned_pub_);
  publishPointCloud(merge_debug_unknown_, header, merge_unknown_pub_);
  merge_debug_was_published_ = merge_debug_active_;
}

void MappingRos::publishClusterCloud(
  const PointCloudPtr & cloud, const std::vector<pcl::PointIndices> & clusters,
  const std_msgs::Header & header) const
{
  pcl::PointCloud<pcl::PointXYZRGB> colored;
  for (std::size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
    const auto color = clusterColor(cluster_index);
    for (const int point_index : clusters[cluster_index].indices) {
      const auto & source = cloud->points[static_cast<std::size_t>(point_index)];
      pcl::PointXYZRGB point;
      point.x = source.x;
      point.y = source.y;
      point.z = source.z;
      point.r = color[0];
      point.g = color[1];
      point.b = color[2];
      colored.push_back(point);
    }
  }
  colored.width = colored.size();
  colored.height = 1;
  colored.is_dense = true;

  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(colored, message);
  message.header = header;
  cluster_cloud_pub_.publish(message);
}

void MappingRos::publishTracks(const std_msgs::Header & header)
{
  TrackedObjectArray message;
  message.header = header;
  for (const auto & tracker : trackers_) {
    const bool confirmed = tracker->hits() >= params_.min_confirmed_hits;
    if (!confirmed && !params_.publish_unconfirmed) {
      continue;
    }
    TrackedObject object;
    object.id = tracker->id();
    object.position.x = tracker->position().x();
    object.position.y = tracker->position().y();
    object.position.z = tracker->position().z();
    object.velocity.x = tracker->velocity().x();
    object.velocity.y = tracker->velocity().y();
    object.velocity.z = tracker->velocity().z();
    object.size.x = tracker->size().x();
    object.size.y = tracker->size().y();
    object.size.z = tracker->size().z();
    object.age = static_cast<std::uint32_t>(tracker->age());
    object.hits = static_cast<std::uint32_t>(tracker->hits());
    object.confirmed = confirmed;
    message.objects.push_back(object);
  }
  tracks_pub_.publish(message);
  publishTargetState(header);
}

void MappingRos::publishMarkers(const std_msgs::Header & header) const
{
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker clear;
  clear.header = header;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);
  // Keep the last target box visible even when FAST-LIO temporarily pauses.
  // The DELETEALL marker at the beginning of the next update still clears it.
  const ros::Duration lifetime(0.0);

  if (params_.manual_selection_enabled && pending_target_click_) {
    visualization_msgs::Marker click;
    click.header = header;
    click.ns = "target_click";
    click.id = 0;
    click.type = visualization_msgs::Marker::SPHERE;
    click.action = visualization_msgs::Marker::ADD;
    click.pose.position.x = clicked_position_.x();
    click.pose.position.y = clicked_position_.y();
    click.pose.position.z = clicked_position_.z();
    click.pose.orientation.w = 1.0;
    click.scale.x = 0.22;
    click.scale.y = 0.22;
    click.scale.z = 0.22;
    click.color.r = 0.1F;
    click.color.g = 0.5F;
    click.color.b = 1.0F;
    click.color.a = 1.0F;
    click.lifetime = lifetime;
    array.markers.push_back(click);
  }

  if (params_.merge_v1_debug_enabled && merge_debug_active_) {
    const std::array<std::pair<Eigen::Vector3d, bool>, 2> predictions{{
      {merge_debug_target_prediction_, true},
      {merge_debug_distractor_prediction_, false}
    }};
    for (std::size_t index = 0; index < predictions.size(); ++index) {
      visualization_msgs::Marker prediction;
      prediction.header = header;
      prediction.ns = predictions[index].second ?
        "merge_v1_predicted_target" : "merge_v1_predicted_auxiliary";
      prediction.id = static_cast<int>(1000U + index);
      prediction.type = visualization_msgs::Marker::SPHERE;
      prediction.action = visualization_msgs::Marker::ADD;
      prediction.pose.position.x = predictions[index].first.x();
      prediction.pose.position.y = predictions[index].first.y();
      prediction.pose.position.z = predictions[index].first.z();
      prediction.pose.orientation.w = 1.0;
      prediction.scale.x = 0.24;
      prediction.scale.y = 0.24;
      prediction.scale.z = 0.24;
      prediction.color.r = predictions[index].second ? 1.0F : 0.1F;
      prediction.color.g = predictions[index].second ? 0.15F : 0.45F;
      prediction.color.b = predictions[index].second ? 0.15F : 1.0F;
      prediction.color.a = 1.0F;
      prediction.lifetime = lifetime;
      array.markers.push_back(prediction);
    }
  }

  for (const auto & tracker : trackers_) {
    const bool confirmed = tracker->hits() >= params_.min_confirmed_hits;
    if (!confirmed && !params_.publish_unconfirmed) {
      continue;
    }
    const Eigen::Vector3d position = tracker->position();
    const Eigen::Vector3d velocity = tracker->velocity();
    const bool crossing_ambiguous = params_.manual_selection_enabled &&
      (crossing_occlusion_active_ || !identity_hypotheses_.empty());
    const bool fusion_suspect = params_.manual_selection_enabled &&
      fusion_state_machine_.state() == FusionState::SUSPECT;

    visualization_msgs::Marker box;
    box.header = header;
    box.ns = params_.manual_selection_enabled ? "target_box" : "person_box";
    box.id = tracker->id() * 3;
    box.type = visualization_msgs::Marker::CUBE;
    box.action = visualization_msgs::Marker::ADD;
    box.pose.position.x = position.x();
    box.pose.position.y = position.y();
    box.pose.position.z = position.z();
    box.pose.orientation.w = 1.0;
    box.scale.x = std::max(0.05, tracker->size().x());
    box.scale.y = std::max(0.05, tracker->size().y());
    box.scale.z = std::max(0.05, tracker->size().z());
    box.color.r = (crossing_ambiguous || fusion_suspect) ? 1.0F :
      (confirmed ? 0.1F : 1.0F);
    box.color.g = crossing_ambiguous ? 0.55F :
      (fusion_suspect ? 0.15F : (confirmed ? 1.0F : 0.65F));
    box.color.b = (crossing_ambiguous || fusion_suspect) ? 0.0F : 0.1F;
    box.color.a = 0.25F;
    box.lifetime = lifetime;
    array.markers.push_back(box);

    if (velocity.norm() > 0.03) {
      visualization_msgs::Marker arrow;
      arrow.header = header;
      arrow.ns = params_.manual_selection_enabled ? "target_velocity" : "person_velocity";
      arrow.id = tracker->id() * 3 + 1;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Point start;
      start.x = position.x();
      start.y = position.y();
      start.z = position.z();
      geometry_msgs::Point end;
      end.x = position.x() + velocity.x();
      end.y = position.y() + velocity.y();
      end.z = position.z() + velocity.z();
      arrow.points = {start, end};
      arrow.scale.x = 0.05;
      arrow.scale.y = 0.10;
      arrow.scale.z = 0.12;
      arrow.color.r = 0.1F;
      arrow.color.g = 1.0F;
      arrow.color.b = 0.2F;
      arrow.color.a = 1.0F;
      arrow.lifetime = lifetime;
      array.markers.push_back(arrow);
    }

    visualization_msgs::Marker label;
    label.header = header;
    label.ns = params_.manual_selection_enabled ? "target_id" : "person_id";
    label.id = tracker->id() * 3 + 2;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.position.x = position.x();
    label.pose.position.y = position.y();
    label.pose.position.z = position.z() + 0.5 * tracker->size().z() + 0.25;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.22;
    label.color.r = 1.0F;
    label.color.g = 1.0F;
    label.color.b = 1.0F;
    label.color.a = 1.0F;
    std::ostringstream text;
    text << (params_.manual_selection_enabled ? "target " : "person ") << tracker->id() << "  " <<
      std::fixed;
    text.precision(2);
    text << velocity.head<2>().norm() << " m/s";
    if (params_.manual_selection_enabled && fusion_enabled_) {
      text << "  " << fusionStateName(fusion_state_machine_.state()) << "/" <<
        semanticEvidenceName(fusion_state_machine_.evidence());
    }
    if (crossing_occlusion_active_) {
      text << "  MERGED";
    } else if (!identity_hypotheses_.empty()) {
      text << "  ID?";
    }
    label.text = text.str();
    label.lifetime = lifetime;
    array.markers.push_back(label);
  }
  markers_pub_.publish(array);
}

}  // namespace person_tracker
