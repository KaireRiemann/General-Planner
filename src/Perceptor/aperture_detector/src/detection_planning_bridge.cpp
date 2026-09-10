#include "detector/detection_planning_bridge.hpp"

#include <sstream>

#include <nav_msgs/Odometry.h>

namespace detector
{

DetectionPlanningBridge::DetectionPlanningBridge(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh)
{
  pnh_.param("detector_enable_topic", detector_enable_topic_,
             std::string("/polygon_hole_step_viz/detection_enable"));
  pnh_.param("planning_snapshot_topic", planning_snapshot_topic_,
             std::string("/polygon_hole_step_viz/planning_snapshot_json"));
  pnh_.param("planning_trigger_topic", planning_trigger_topic_,
             std::string("/gate_planner/planning_trigger_odom"));
  pnh_.param("position_command_topic", position_command_topic_,
             std::string("/setpoints_cmd"));
  pnh_.param("frame_id", frame_id_, std::string("world"));
  pnh_.param("initial_detection_enabled", detection_enabled_, true);
  pnh_.param("require_detection_ready", require_detection_ready_, true);
  pnh_.param("snapshot_timeout", snapshot_timeout_, 1.5);
  pnh_.param("planning_start_timeout", planning_start_timeout_, 3.0);
  pnh_.param("command_timeout", command_timeout_, 1.0);

  detector_enable_pub_ = nh_.advertise<std_msgs::Bool>(detector_enable_topic_, 1, true);
  planning_trigger_pub_ = nh_.advertise<nav_msgs::Odometry>(planning_trigger_topic_, 1, false);

  detection_enabled_pub_ = pnh_.advertise<std_msgs::Bool>("detection_enabled", 1, true);
  detection_ready_pub_ = pnh_.advertise<std_msgs::Bool>("detection_ready", 1, true);
  planning_active_pub_ = pnh_.advertise<std_msgs::Bool>("planning_active", 1, true);
  planning_completed_pub_ = pnh_.advertise<std_msgs::Bool>("planning_completed", 1, true);
  state_pub_ = pnh_.advertise<std_msgs::String>("state", 1, true);

  detection_cmd_sub_ = pnh_.subscribe(
    "detection_enable_cmd", 1,
    &DetectionPlanningBridge::detectionEnableCommandCallback, this,
    ros::TransportHints().tcpNoDelay());
  planning_start_sub_ = pnh_.subscribe(
    "planning_start_cmd", 1,
    &DetectionPlanningBridge::planningStartCommandCallback, this,
    ros::TransportHints().tcpNoDelay());
  snapshot_sub_ = nh_.subscribe(
    planning_snapshot_topic_, 1,
    &DetectionPlanningBridge::snapshotCallback, this,
    ros::TransportHints().tcpNoDelay());
  command_sub_ = nh_.subscribe(
    position_command_topic_, 10,
    &DetectionPlanningBridge::positionCommandCallback, this,
    ros::TransportHints().tcpNoDelay());

  status_timer_ = nh_.createTimer(
    ros::Duration(0.2), &DetectionPlanningBridge::statusTimerCallback, this);

  setDetectionEnabled(detection_enabled_);
  publishState("IDLE");
  publishBooleans();

  ROS_INFO_STREAM("[detection_planning_bridge] ready. detector_enable_topic="
                  << detector_enable_topic_
                  << " planning_snapshot_topic=" << planning_snapshot_topic_
                  << " planning_trigger_topic=" << planning_trigger_topic_
                  << " position_command_topic=" << position_command_topic_);
}

void DetectionPlanningBridge::setDetectionEnabled(bool enabled)
{
  detection_enabled_ = enabled;
  if (!enabled)
  {
    detection_ready_ = false;
    last_snapshot_stamp_ = ros::Time(0);
  }

  std_msgs::Bool msg;
  msg.data = enabled;
  detector_enable_pub_.publish(msg);

  publishBooleans();
  publishState(enabled ? "DETECTION_ENABLED" : "DETECTION_DISABLED");
}

bool DetectionPlanningBridge::startPlanning()
{
  updateDetectionFreshness();
  if (require_detection_ready_ && !detection_ready_)
  {
    publishState("PLANNING_REJECTED_NO_VALID_DETECTION");
    ROS_WARN("[detection_planning_bridge] planning start rejected: detection snapshot is not ready.");
    return false;
  }

  nav_msgs::Odometry trigger;
  trigger.header.stamp = ros::Time::now();
  trigger.header.frame_id = frame_id_;
  planning_trigger_pub_.publish(trigger);

  waiting_for_planning_start_ = true;
  planning_active_ = false;
  planning_completed_ = false;
  trigger_stamp_ = ros::Time::now();
  trigger_base_traj_id_ = latest_traj_id_;
  active_traj_id_ = 0;
  publishBooleans();
  publishState("PLANNING_TRIGGERED");
  return true;
}

bool DetectionPlanningBridge::detectionEnabled() const
{
  return detection_enabled_;
}

bool DetectionPlanningBridge::detectionReady() const
{
  return detection_ready_;
}

bool DetectionPlanningBridge::planningActive() const
{
  return planning_active_;
}

bool DetectionPlanningBridge::planningCompleted() const
{
  return planning_completed_;
}

std::string DetectionPlanningBridge::lastState() const
{
  return last_state_;
}

void DetectionPlanningBridge::detectionEnableCommandCallback(const std_msgs::BoolConstPtr &msg)
{
  setDetectionEnabled(msg->data);
}

void DetectionPlanningBridge::planningStartCommandCallback(const std_msgs::EmptyConstPtr &)
{
  startPlanning();
}

void DetectionPlanningBridge::snapshotCallback(const std_msgs::StringConstPtr &msg)
{
  if (!detection_enabled_)
  {
    return;
  }
  latest_snapshot_json_ = msg->data;
  last_snapshot_stamp_ = ros::Time::now();
  detection_ready_ = true;
  publishBooleans();
  publishState("DETECTION_READY");
}

void DetectionPlanningBridge::positionCommandCallback(const quadrotor_msgs::PositionCommandConstPtr &msg)
{
  latest_traj_id_ = msg->trajectory_id;
  latest_trajectory_flag_ = msg->trajectory_flag;
  last_command_stamp_ = ros::Time::now();

  const bool completed =
    msg->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
  const bool ready =
    msg->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

  if (waiting_for_planning_start_ && msg->trajectory_id != trigger_base_traj_id_ && msg->trajectory_id > 0)
  {
    waiting_for_planning_start_ = false;
    planning_active_ = !completed;
    planning_completed_ = completed;
    active_traj_id_ = msg->trajectory_id;
    publishBooleans();
    publishState(completed ? "PLANNING_COMPLETED" : "PLANNING_STARTED");
    return;
  }

  if (active_traj_id_ == 0 || msg->trajectory_id != active_traj_id_)
  {
    return;
  }

  if (completed)
  {
    planning_active_ = false;
    planning_completed_ = true;
    publishBooleans();
    publishState("PLANNING_COMPLETED");
  }
  else if (ready)
  {
    planning_active_ = true;
    planning_completed_ = false;
    publishBooleans();
  }
}

void DetectionPlanningBridge::statusTimerCallback(const ros::TimerEvent &)
{
  updateDetectionFreshness();

  const ros::Time now = ros::Time::now();
  if (waiting_for_planning_start_ &&
      planning_start_timeout_ > 0.0 &&
      (now - trigger_stamp_).toSec() > planning_start_timeout_)
  {
    waiting_for_planning_start_ = false;
    planning_active_ = false;
    planning_completed_ = false;
    publishBooleans();
    publishState("PLANNING_START_TIMEOUT");
  }

  if (planning_active_ &&
      command_timeout_ > 0.0 &&
      !last_command_stamp_.isZero() &&
      (now - last_command_stamp_).toSec() > command_timeout_)
  {
    planning_active_ = false;
    planning_completed_ = false;
    publishBooleans();
    publishState("PLANNING_COMMAND_TIMEOUT");
  }
}

void DetectionPlanningBridge::updateDetectionFreshness()
{
  const bool was_ready = detection_ready_;
  if (!detection_enabled_ || last_snapshot_stamp_.isZero())
  {
    detection_ready_ = false;
  }
  else if (snapshot_timeout_ > 0.0)
  {
    detection_ready_ = (ros::Time::now() - last_snapshot_stamp_).toSec() <= snapshot_timeout_;
  }

  if (was_ready != detection_ready_)
  {
    publishBooleans();
    publishState(detection_ready_ ? "DETECTION_READY" : "DETECTION_NOT_READY");
  }
}

void DetectionPlanningBridge::publishBooleans()
{
  publishBool(detection_enabled_pub_, detection_enabled_);
  publishBool(detection_ready_pub_, detection_ready_);
  publishBool(planning_active_pub_, planning_active_);
  publishBool(planning_completed_pub_, planning_completed_);
}

void DetectionPlanningBridge::publishBool(ros::Publisher &pub, bool value)
{
  std_msgs::Bool msg;
  msg.data = value;
  pub.publish(msg);
}

void DetectionPlanningBridge::publishState(const std::string &state)
{
  last_state_ = state;
  std_msgs::String msg;
  std::ostringstream oss;
  oss << "state=" << state
      << " detection_enabled=" << (detection_enabled_ ? "true" : "false")
      << " detection_ready=" << (detection_ready_ ? "true" : "false")
      << " waiting_for_planning_start=" << (waiting_for_planning_start_ ? "true" : "false")
      << " planning_active=" << (planning_active_ ? "true" : "false")
      << " planning_completed=" << (planning_completed_ ? "true" : "false")
      << " latest_traj_id=" << latest_traj_id_
      << " active_traj_id=" << active_traj_id_
      << " latest_trajectory_flag=" << static_cast<int>(latest_trajectory_flag_);
  msg.data = oss.str();
  state_pub_.publish(msg);
  ROS_INFO_STREAM_THROTTLE(0.5, "[detection_planning_bridge] " << msg.data);
}

}  // namespace detector
