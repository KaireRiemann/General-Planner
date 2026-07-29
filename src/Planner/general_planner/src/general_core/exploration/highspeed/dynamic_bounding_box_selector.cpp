#include <general_core/exploration/highspeed/dynamic_bounding_box_selector.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <thread>
#include <vector>
#include <visualization_msgs/Marker.h>

namespace fast_planner {
namespace {

geometry_msgs::Point toPoint(const Eigen::Vector3f &point) {
  geometry_msgs::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

std::string formatPoint(const Eigen::Vector3f &point) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(2);
  stream << "[" << point.x() << ", " << point.y() << ", " << point.z()
         << "]";
  return stream.str();
}

} // namespace

void DynamicBoundingBoxSelector::init(ros::NodeHandle &nh) {
  nh.param("dynamic_bounding_box/enabled", enabled_, false);
  if (!enabled_) {
    return;
  }

  std::string mode;
  nh.param<std::string>("dynamic_bounding_box/mode", mode, "footprint");
  if (mode == "diagonal_3d") {
    mode_ = SelectionMode::DIAGONAL_3D;
  } else {
    if (mode != "footprint") {
      ROS_WARN_STREAM("[dynamic bbox] unsupported mode='" << mode
                      << "', use footprint");
    }
    mode_ = SelectionMode::FOOTPRINT;
  }

  nh.param<std::string>("dynamic_bounding_box/frame_id", frame_id_, "world");
  nh.param<std::string>("dynamic_bounding_box/goal_topic", goal_topic_,
                        "dynamic_bounding_box/corner_3d");
  nh.param<std::string>("dynamic_bounding_box/reset_topic", reset_topic_,
                        "dynamic_bounding_box/reset");
  nh.param<std::string>("dynamic_bounding_box/marker_topic", marker_topic_,
                        "dynamic_bounding_box/markers");
  nh.param("dynamic_bounding_box/min_z", min_z_, 0.0);
  nh.param("dynamic_bounding_box/max_z", max_z_, 5.0);
  nh.param("dynamic_bounding_box/min_xy_extent", min_xy_extent_, 1.0);
  nh.param("dynamic_bounding_box/min_z_extent", min_z_extent_, 0.5);
  nh.param("dynamic_bounding_box/selection_timeout", selection_timeout_, 0.0);
  nh.param("dynamic_bounding_box/line_width", line_width_, 0.08);

  frame_id_ = normalizedFrame(frame_id_);
  min_xy_extent_ = std::max(0.01, min_xy_extent_);
  min_z_extent_ = std::max(0.01, min_z_extent_);
  selection_timeout_ = std::max(0.0, selection_timeout_);
  line_width_ = std::clamp(line_width_, 0.01, 0.5);
  if (min_z_ > max_z_) {
    std::swap(min_z_, max_z_);
  }

  marker_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
  goal_sub_ =
      nh.subscribe(goal_topic_, 2,
                   &DynamicBoundingBoxSelector::goalCallback, this);
  reset_sub_ =
      nh.subscribe(reset_topic_, 1,
                   &DynamicBoundingBoxSelector::resetCallback, this);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishVisualizationLocked(
        mode_ == SelectionMode::FOOTPRINT
            ? "Select two opposite XY corners with 3D Nav Goal"
            : "Select two opposite 3D corners with 3D Nav Goal");
  }

  ROS_WARN_STREAM(
      "[dynamic bbox] exploration initialization is waiting for two 3D goals on "
      << nh.resolveName(goal_topic_) << " mode="
      << (mode_ == SelectionMode::FOOTPRINT ? "footprint" : "diagonal_3d")
      << " frame=" << frame_id_
      << (mode_ == SelectionMode::FOOTPRINT
              ? " z=[" + std::to_string(min_z_) + ", " +
                    std::to_string(max_z_) + "]"
              : "")
      << ". Reset with: rostopic pub -1 " << nh.resolveName(reset_topic_)
      << " std_msgs/Empty '{}'");
}

bool DynamicBoundingBoxSelector::waitForSelection(Eigen::Vector3f &box_min,
                                                  Eigen::Vector3f &box_max) {
  if (!enabled_) {
    return false;
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();
  const ros::WallTime start = ros::WallTime::now();
  bool selected = false;
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (selection_ready_) {
        box_min = selected_min_;
        box_max = selected_max_;
        selected = true;
        break;
      }
    }

    if (selection_timeout_ > 0.0 &&
        (ros::WallTime::now() - start).toSec() >= selection_timeout_) {
      ROS_ERROR_STREAM("[dynamic bbox] no valid selection within "
                       << selection_timeout_
                       << "s; dynamic mode will stop instead of using the "
                          "fixed YAML exploration bounds");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  spinner.stop();
  return selected;
}

void DynamicBoundingBoxSelector::goalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg) {
    return;
  }

  const std::string incoming_frame = normalizedFrame(msg->header.frame_id);
  if (!incoming_frame.empty() && incoming_frame != frame_id_) {
    ROS_WARN_STREAM("[dynamic bbox] reject point in frame '" << incoming_frame
                    << "'; RViz Fixed Frame must be '" << frame_id_ << "'");
    return;
  }

  const Eigen::Vector3f point(static_cast<float>(msg->pose.position.x),
                             static_cast<float>(msg->pose.position.y),
                             static_cast<float>(msg->pose.position.z));
  if (!point.allFinite()) {
    ROS_WARN("[dynamic bbox] reject non-finite point");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (selection_ready_) {
    return;
  }
  if (!have_first_corner_) {
    first_corner_ = point;
    have_first_corner_ = true;
    publishVisualizationLocked("Corner 1 " + formatPoint(first_corner_) +
                               " selected; select opposite corner");
    ROS_INFO_STREAM("[dynamic bbox] corner 1=" << formatPoint(first_corner_));
    return;
  }

  Eigen::Vector3f candidate_min;
  Eigen::Vector3f candidate_max;
  if (!buildCandidateLocked(point, candidate_min, candidate_max)) {
    ROS_WARN_STREAM(
        "[dynamic bbox] invalid box; required XY extent >= "
        << min_xy_extent_ << "m and Z extent >= " << min_z_extent_
        << "m. The first corner was cleared; select both corners again.");
    have_first_corner_ = false;
    publishVisualizationLocked("Invalid extent; select corner 1 again");
    return;
  }

  selected_min_ = candidate_min;
  selected_max_ = candidate_max;
  selection_ready_ = true;
  publishVisualizationLocked("Dynamic exploration box accepted");
  ROS_INFO_STREAM("[dynamic bbox] accepted min=" << formatPoint(selected_min_)
                                                  << " max="
                                                  << formatPoint(selected_max_));
}

void DynamicBoundingBoxSelector::resetCallback(
    const std_msgs::EmptyConstPtr &) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (selection_ready_) {
    ROS_WARN("[dynamic bbox] reset arrived after acceptance and is ignored");
    return;
  }
  have_first_corner_ = false;
  publishVisualizationLocked("Selection reset; select corner 1");
  ROS_INFO("[dynamic bbox] selection reset");
}

bool DynamicBoundingBoxSelector::buildCandidateLocked(
    const Eigen::Vector3f &second, Eigen::Vector3f &box_min,
    Eigen::Vector3f &box_max) const {
  Eigen::Vector3f first = first_corner_;
  Eigen::Vector3f other = second;
  if (mode_ == SelectionMode::FOOTPRINT) {
    first.z() = static_cast<float>(min_z_);
    other.z() = static_cast<float>(max_z_);
  }
  box_min = first.cwiseMin(other);
  box_max = first.cwiseMax(other);
  const Eigen::Vector3f extent = box_max - box_min;
  return extent.x() >= min_xy_extent_ && extent.y() >= min_xy_extent_ &&
         extent.z() >= min_z_extent_;
}

std::string
DynamicBoundingBoxSelector::normalizedFrame(const std::string &frame) {
  if (!frame.empty() && frame.front() == '/') {
    return frame.substr(1);
  }
  return frame;
}

void DynamicBoundingBoxSelector::publishVisualizationLocked(
    const std::string &status) {
  visualization_msgs::MarkerArray array;
  const ros::Time stamp = ros::Time::now();

  visualization_msgs::Marker clear;
  clear.header.frame_id = frame_id_;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::Marker::DELETEALL;
  array.markers.push_back(clear);

  if (have_first_corner_ && !selection_ready_) {
    visualization_msgs::Marker corner;
    corner.header.frame_id = frame_id_;
    corner.header.stamp = stamp;
    corner.ns = "dynamic_exploration_bbox";
    corner.id = 2;
    corner.type = visualization_msgs::Marker::SPHERE;
    corner.action = visualization_msgs::Marker::ADD;
    corner.pose.position = toPoint(first_corner_);
    corner.pose.orientation.w = 1.0;
    corner.scale.x = corner.scale.y = corner.scale.z = 0.35;
    corner.color.r = 1.0;
    corner.color.g = 0.75;
    corner.color.b = 0.1;
    corner.color.a = 1.0;
    array.markers.push_back(corner);
  }

  Eigen::Vector3f text_position = first_corner_;
  if (selection_ready_) {
    const Eigen::Vector3f draw_min = selected_min_;
    const Eigen::Vector3f draw_max = selected_max_;
    const Eigen::Vector3f center = 0.5f * (draw_min + draw_max);
    const std::array<Eigen::Vector3f, 8> corners = {
        Eigen::Vector3f(draw_min.x(), draw_min.y(), draw_min.z()),
        Eigen::Vector3f(draw_max.x(), draw_min.y(), draw_min.z()),
        Eigen::Vector3f(draw_max.x(), draw_max.y(), draw_min.z()),
        Eigen::Vector3f(draw_min.x(), draw_max.y(), draw_min.z()),
        Eigen::Vector3f(draw_min.x(), draw_min.y(), draw_max.z()),
        Eigen::Vector3f(draw_max.x(), draw_min.y(), draw_max.z()),
        Eigen::Vector3f(draw_max.x(), draw_max.y(), draw_max.z()),
        Eigen::Vector3f(draw_min.x(), draw_max.y(), draw_max.z())};
    constexpr std::array<std::array<int, 2>, 12> edges = {
        std::array<int, 2>{0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5},                    {5, 6}, {6, 7}, {7, 4},
        {0, 4},                    {1, 5}, {2, 6}, {3, 7}};

    // Use an outline only. A large transparent CUBE adds no useful boundary
    // information and makes RViz selection/rendering unnecessarily fragile.
    visualization_msgs::Marker outline;
    outline.header.frame_id = frame_id_;
    outline.header.stamp = stamp;
    outline.ns = "dynamic_exploration_bbox";
    outline.id = 1;
    outline.type = visualization_msgs::Marker::LINE_LIST;
    outline.action = visualization_msgs::Marker::ADD;
    outline.pose.orientation.w = 1.0;
    outline.scale.x = line_width_;
    outline.color.r = 0.1;
    outline.color.g = 0.9;
    outline.color.b = 0.25;
    outline.color.a = 1.0;
    for (const auto &edge : edges) {
      outline.points.push_back(toPoint(corners[edge[0]]));
      outline.points.push_back(toPoint(corners[edge[1]]));
    }
    array.markers.push_back(outline);
    text_position =
        Eigen::Vector3f(center.x(), center.y(), draw_max.z() + 0.6f);
  } else if (have_first_corner_) {
    text_position.z() += 0.6f;
  } else {
    // Before the first click DELETEALL is the only marker. This guarantees
    // that enabling dynamic mode never visualizes the fixed YAML box.
    marker_pub_.publish(array);
    return;
  }

  visualization_msgs::Marker text;
  text.header.frame_id = frame_id_;
  text.header.stamp = stamp;
  text.ns = "dynamic_exploration_bbox";
  text.id = 3;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.pose.position = toPoint(text_position);
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.45;
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.b = 1.0;
  text.color.a = 1.0;
  text.text = status;
  array.markers.push_back(text);

  marker_pub_.publish(array);
}

} // namespace fast_planner
