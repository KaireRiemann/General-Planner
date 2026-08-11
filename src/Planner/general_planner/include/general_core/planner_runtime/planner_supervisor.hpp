#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <general_core/planner_runtime/planner_command_gateway.hpp>
#include <general_core/planner_runtime/global_map_runtime.hpp>
#include <general_core/planner_runtime/planner_status.hpp>
#include <general_planner/PlannerModeRequest.h>
#include <general_planner/PlannerStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace general_planner::planner_runtime {

class PlannerSupervisor {
public:
  using MapStatusProvider = std::function<GlobalMapStatus()>;

  PlannerSupervisor(ros::NodeHandle &nh, PlannerCommandGateway &gateway,
                    MapStatusProvider map_status_provider = {});

private:
  void modeRequestCallback(const general_planner::PlannerModeRequestConstPtr &msg);
  void modeRequestTextCallback(const std_msgs::StringConstPtr &msg);
  void navigationGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void explorationTriggerCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  // Shared RViz / click PoseStamped entry: route by active_mode.
  void clickGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  bool acceptNavigationGoalLocked(const geometry_msgs::PoseStamped &msg);
  bool acceptExplorationTriggerLocked(const geometry_msgs::PoseStamped &msg);
  void explorationStatusCallback(const std_msgs::StringConstPtr &msg);
  void navigationStatusCallback(const std_msgs::StringConstPtr &msg);
  void handoverStatusCallback(const std_msgs::StringConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void timerCallback(const ros::TimerEvent &);

  void handleModeRequest(std::uint64_t request_id, PlannerMode mode,
                         const std::string &task_id, const std::string &source);
  void beginTransition(PlannerMode target, std::uint64_t request_id,
                       const std::string &task_id, const std::string &reason);
  void enterStableHold(const std::string &reason, PlannerTaskResult result);
  void activateMode(PlannerMode mode, const std::string &reason);
  void requestAdapterStop(PlannerMode mode, const std::string &reason);
  void resetAdapterTaskState(PlannerMode mode);
  void publishStatus();
  void publishExplorationCommand(const std::string &command);
  void publishNavigationCommand(const std::string &command);
  void publishNavigationTaskMode(const std::string &mode);
  void publishHandoverCommand(const std::string &command);
  void requestExplorationStartLocked(const std::string &reason);
  bool hoverConditionMetLocked() const;
  std::string makeTaskId(PlannerMode mode) const;
  void updatePhaseFromActiveModeLocked();

  ros::NodeHandle nh_;
  PlannerCommandGateway &gateway_;
  MapStatusProvider map_status_provider_;

  ros::Subscriber mode_request_sub_;
  ros::Subscriber mode_request_text_sub_;
  ros::Subscriber navigation_goal_sub_;
  ros::Subscriber exploration_trigger_sub_;
  std::vector<ros::Subscriber> exploration_rviz_trigger_subs_;
  ros::Subscriber exploration_status_sub_;
  ros::Subscriber navigation_status_sub_;
  ros::Subscriber handover_status_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher status_pub_;
  ros::Publisher exploration_command_pub_;
  ros::Publisher navigation_command_pub_;
  ros::Publisher navigation_task_mode_pub_;
  ros::Publisher navigation_goal_pub_;
  ros::Publisher click_demo_goal_pub_;
  ros::Publisher handover_command_pub_;
  ros::Timer timer_;

  mutable std::mutex mutex_;
  PlannerStatusData status_;

  PlannerMode initial_mode_{PlannerMode::HOLD};
  PlannerMode transition_target_{PlannerMode::HOLD};
  bool transition_active_{false};
  bool boot_complete_{false};
  bool hold_anchor_locked_for_transition_{false};
  bool exploration_start_pending_{false};
  bool navigation_enabled_{true};
  bool exploration_enabled_{true};
  // After exploration, kill exploration stack and start a standalone click-demo
  // fsm_node that owns /planning/pos_cmd directly (no parallel nav + gateway).
  bool serial_handover_{true};
  bool serial_state2state_ready_{false};
  bool serial_handover_pending_{false};
  ros::Time hover_satisfied_since_;
  ros::Time last_odom_time_;
  ros::Time last_exploration_start_pub_;
  nav_msgs::Odometry odom_;
  bool have_odom_{false};

  std::string exploration_status_{"IDLE"};
  std::string exploration_status_task_id_;
  // HighSpeedExp publishes its terminal status continuously while PAUSED.
  // Remember the task that already completed so the repeated SUCCEEDED does
  // not restart the supervisor's hover-confirmation transition.
  std::string completed_exploration_task_id_;
  std::string navigation_status_{"INIT"};
  std::uint64_t navigation_status_epoch_{0};
  std::uint64_t navigation_goal_sequence_{0};
  std::uint64_t navigation_goal_sequence_before_dispatch_{0};
  bool navigation_goal_dispatch_pending_{false};

  std::string exploration_command_topic_;
  std::string exploration_status_topic_;
  std::string navigation_command_topic_;
  std::string navigation_status_topic_;
  std::string navigation_task_mode_topic_;
  std::string navigation_goal_out_topic_;
  std::string click_demo_goal_topic_;
  std::string handover_command_topic_;
  std::string handover_status_topic_;
  std::string odometry_topic_;

  double hover_speed_threshold_{0.10};
  double hover_yaw_rate_threshold_{0.10};
  double hover_hold_duration_{0.50};
  double max_odom_age_{0.20};
  double status_rate_{10.0};
  double exploration_start_retry_period_{0.5};
  std::uint64_t next_text_request_id_{1};
};

} // namespace general_planner::planner_runtime
