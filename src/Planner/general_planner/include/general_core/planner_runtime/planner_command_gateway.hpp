#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#include <general_core/planner_runtime/planner_command_gateway_policy.hpp>
#include <general_core/planner_runtime/planner_status.hpp>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

namespace general_planner::planner_runtime {

class PlannerCommandGateway {
public:
  explicit PlannerCommandGateway(ros::NodeHandle &nh);

  void setAuthorizedOwner(CommandOwner owner, std::uint64_t task_epoch);
  // Atomically transfer command ownership to HOLD and replace its anchor.
  // Lifecycle edges must use this rather than separately authorizing HOLD and
  // reading a callback-queue-local odometry sample: a timer tick between those
  // two operations can otherwise publish the previous task's hold point.
  void setHoldAnchorAndAuthorize(double x, double y, double z, double yaw,
                                 std::uint64_t task_epoch);
  // Used only when the supervisor has no valid current odometry.  Dropping an
  // old anchor is safer than continuing to command a known stale position.
  void clearHoldAnchor();
  void lockHoldAnchorFromOdom();
  void lockHoldAnchor(double x, double y, double z, double yaw);
  bool hasHoldAnchor() const;
  CommandOwner authorizedOwner() const;
  std::uint64_t authorizedEpoch() const;
  // When false, gateway stops publishing /planning/pos_cmd so an external
  // click-demo fsm_node can own the command bus after serial handover.
  void setPublishingEnabled(bool enabled);
  bool publishingEnabled() const;

private:
  void navigationCallback(const quadrotor_msgs::PositionCommandConstPtr &msg);
  void explorationCallback(const quadrotor_msgs::PositionCommandConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void timerCallback(const ros::WallTimerEvent &);
  quadrotor_msgs::PositionCommand makeHoldCommandLocked() const;
  quadrotor_msgs::PositionCommand makeSourceTimeoutHoldCommandLocked(
      CommandOwner owner);
  void clearSourceTimeoutHoldLocked();
  static double yawFromQuat(const geometry_msgs::Quaternion &q);

  ros::NodeHandle nh_;
  ros::Publisher output_pub_;
  ros::Subscriber navigation_sub_;
  ros::Subscriber exploration_sub_;
  ros::Subscriber odom_sub_;
  ros::WallTimer timer_;

  mutable std::mutex mutex_;
  std::string navigation_cmd_topic_;
  std::string exploration_cmd_topic_;
  std::string output_cmd_topic_;
  std::string odometry_topic_;
  double command_timeout_{0.30};
  double publish_rate_{100.0};

  CommandOwner authorized_owner_{CommandOwner::HOLD};
  std::uint64_t authorized_epoch_{0};
  ros::WallTime authorization_time_{ros::WallTime::now()};

  quadrotor_msgs::PositionCommand navigation_cmd_;
  quadrotor_msgs::PositionCommand exploration_cmd_;
  ros::WallTime navigation_rx_time_;
  ros::WallTime exploration_rx_time_;
  bool have_navigation_cmd_{false};
  bool have_exploration_cmd_{false};

  nav_msgs::Odometry odom_;
  bool have_odom_{false};

  bool hold_anchor_valid_{false};
  // When the supervisor cannot verify current odometry it explicitly clears
  // the lifecycle anchor.  In that state HOLD must not fall back to this
  // gateway's asynchronous odom cache, which may belong to an older task.
  bool hold_anchor_required_{false};
  double hold_x_{0.0};
  double hold_y_{0.0};
  double hold_z_{0.0};
  double hold_yaw_{0.0};
  std::uint32_t hold_sequence_{0};
  // A stale command source is not an explicit task hold.  This anchor is
  // captured from the current odometry on timeout and is deliberately kept
  // separate from the task-transition hold anchor above.
  bool source_timeout_hold_active_{false};
  CommandOwner source_timeout_hold_owner_{CommandOwner::HOLD};
  quadrotor_msgs::PositionCommand source_timeout_hold_;
  bool publishing_enabled_{true};
};

} // namespace general_planner::planner_runtime
