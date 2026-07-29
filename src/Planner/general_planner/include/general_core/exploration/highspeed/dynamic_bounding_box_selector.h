#pragma once

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

namespace fast_planner {

/**
 * Collects an exploration bounding box before the exploration stack starts.
 *
 * The selector deliberately finishes before TopoGraph, ParallelBubbleAstar,
 * FrontierManager and CoverageGuidanceManager are initialized.  Those modules
 * derive grid origins or allocate arrays from the exploration bounds, so
 * changing the bounds after they start would leave inconsistent indices.
 */
class DynamicBoundingBoxSelector {
public:
  void init(ros::NodeHandle &nh);

  bool enabled() const { return enabled_; }

  /**
   * Wait for two RViz points. Returns true when a dynamic box was selected.
   * Returns false on timeout or ROS shutdown. Dynamic mode never falls back
   * to the fixed YAML bounds.
   */
  bool waitForSelection(Eigen::Vector3f &box_min, Eigen::Vector3f &box_max);

private:
  enum class SelectionMode { FOOTPRINT, DIAGONAL_3D };

  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void resetCallback(const std_msgs::EmptyConstPtr &msg);
  void publishVisualizationLocked(const std::string &status);
  bool buildCandidateLocked(const Eigen::Vector3f &second,
                            Eigen::Vector3f &box_min,
                            Eigen::Vector3f &box_max) const;
  static std::string normalizedFrame(const std::string &frame);

  ros::Subscriber goal_sub_;
  ros::Subscriber reset_sub_;
  ros::Publisher marker_pub_;

  bool enabled_{false};
  SelectionMode mode_{SelectionMode::FOOTPRINT};
  std::string frame_id_{"world"};
  std::string goal_topic_{"dynamic_bounding_box/corner_3d"};
  std::string reset_topic_{"dynamic_bounding_box/reset"};
  std::string marker_topic_{"dynamic_bounding_box/markers"};
  double min_z_{0.0};
  double max_z_{5.0};
  double min_xy_extent_{1.0};
  double min_z_extent_{0.5};
  double selection_timeout_{0.0};
  double line_width_{0.08};

  Eigen::Vector3f first_corner_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f selected_min_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f selected_max_{Eigen::Vector3f::Zero()};
  bool have_first_corner_{false};
  bool selection_ready_{false};
  mutable std::mutex mutex_;
};

} // namespace fast_planner
