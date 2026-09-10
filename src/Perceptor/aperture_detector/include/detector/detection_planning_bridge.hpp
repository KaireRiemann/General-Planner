#pragma once

#include <cstdint>
#include <string>

#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

namespace detector
{

class DetectionPlanningBridge
{
public:
  DetectionPlanningBridge(ros::NodeHandle nh, ros::NodeHandle pnh);

  void setDetectionEnabled(bool enabled);
  bool startPlanning();

  bool detectionEnabled() const;
  bool detectionReady() const;
  bool planningActive() const;
  bool planningCompleted() const;
  std::string lastState() const;

private:
  void detectionEnableCommandCallback(const std_msgs::BoolConstPtr &msg);
  void planningStartCommandCallback(const std_msgs::EmptyConstPtr &msg);
  void snapshotCallback(const std_msgs::StringConstPtr &msg);
  void positionCommandCallback(const quadrotor_msgs::PositionCommandConstPtr &msg);
  void statusTimerCallback(const ros::TimerEvent &event);

  void updateDetectionFreshness();
  void publishBooleans();
  void publishBool(ros::Publisher &pub, bool value);
  void publishState(const std::string &state);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string detector_enable_topic_;
  std::string planning_snapshot_topic_;
  std::string planning_trigger_topic_;
  std::string position_command_topic_;
  std::string frame_id_;

  bool require_detection_ready_ = true;
  double snapshot_timeout_ = 1.5;
  double planning_start_timeout_ = 3.0;
  double command_timeout_ = 1.0;

  ros::Publisher detector_enable_pub_;
  ros::Publisher planning_trigger_pub_;
  ros::Publisher detection_enabled_pub_;
  ros::Publisher detection_ready_pub_;
  ros::Publisher planning_active_pub_;
  ros::Publisher planning_completed_pub_;
  ros::Publisher state_pub_;

  ros::Subscriber detection_cmd_sub_;
  ros::Subscriber planning_start_sub_;
  ros::Subscriber snapshot_sub_;
  ros::Subscriber command_sub_;
  ros::Timer status_timer_;

  bool detection_enabled_ = true;
  bool detection_ready_ = false;
  bool waiting_for_planning_start_ = false;
  bool planning_active_ = false;
  bool planning_completed_ = false;

  ros::Time last_snapshot_stamp_;
  ros::Time trigger_stamp_;
  ros::Time last_command_stamp_;
  std::string latest_snapshot_json_;
  std::string last_state_;

  uint32_t latest_traj_id_ = 0;
  uint32_t trigger_base_traj_id_ = 0;
  uint32_t active_traj_id_ = 0;
  uint8_t latest_trajectory_flag_ = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
};

}  // namespace detector
