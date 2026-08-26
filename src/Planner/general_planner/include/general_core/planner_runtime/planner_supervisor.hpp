#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <general_core/planner_runtime/planner_command_gateway.hpp>
#include <general_core/planner_runtime/global_map_runtime.hpp>
#include <general_core/planner_runtime/planner_status.hpp>
#include <general_planner/ExplorationTaskRequest.h>
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
  void navigationGoal3DCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void explorationTriggerCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void explorationTargetGoalCallback(
      const geometry_msgs::PoseStampedConstPtr &msg);
  // Shared RViz / click PoseStamped entry: route by active_mode.
  void clickGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  // `preserve_message_height` selects the FSM's 3D ingress so z is not
  // replaced by the configured 2D click height.
  bool acceptNavigationGoalLocked(const geometry_msgs::PoseStamped &msg,
                                  bool preserve_message_height = false);
  bool acceptExplorationTriggerLocked(const geometry_msgs::PoseStamped &msg);
  bool forwardExplorationTargetGoalLocked(
      const geometry_msgs::PoseStamped &msg, const std::string &source);
  void explorationStatusCallback(const std_msgs::StringConstPtr &msg);
  void navigationStatusCallback(const std_msgs::StringConstPtr &msg);
  // External gate planner lifecycle ingress. The protocol is std_msgs/String:
  // START (or BEGIN/RUNNING) requests external control, END (or DONE/FINISHED)
  // releases it. Both edges are accepted only through the hover verification
  // performed by timerCallback().
  void gateStatusCallback(const std_msgs::StringConstPtr &msg);
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
  void publishExplorationTaskRequestLocked(bool start);
  void publishNavigationCommand(const std::string &command);
  void publishNavigationTaskMode(const std::string &mode);
  void publishHandoverCommand(const std::string &command);
  void requestExplorationStartLocked(const std::string &reason);
  void completeGateExitLocked();
  // A gateway timeout is already a safe current-pose hold.  This completes
  // the recovery protocol by retiring the stalled task instead of letting the
  // supervisor report EXECUTING forever.
  void failStaleCommandSourceLocked(const CommandSourceHealth &health);
  // `gateway_` has its own callback queue, so its cached odometry must never
  // be used as the source of a lifecycle HOLD.  This method runs under
  // `mutex_` and atomically installs the supervisor's validated pose.
  bool authorizeHoldAtCurrentOdomLocked(const std::string &reason);
  bool hoverConditionMetLocked() const;
  std::string makeTaskId(PlannerMode mode) const;
  void updatePhaseFromActiveModeLocked();

  ros::NodeHandle nh_;
  PlannerCommandGateway &gateway_;
  MapStatusProvider map_status_provider_;

  ros::Subscriber mode_request_sub_;
  ros::Subscriber mode_request_text_sub_;
  ros::Subscriber navigation_goal_sub_;
  ros::Subscriber navigation_goal_3d_sub_;
  ros::Subscriber exploration_trigger_sub_;
  ros::Subscriber exploration_target_goal_sub_;
  std::vector<ros::Subscriber> exploration_rviz_trigger_subs_;
  ros::Subscriber exploration_status_sub_;
  ros::Subscriber navigation_status_sub_;
  ros::Subscriber gate_status_sub_;
  ros::Subscriber handover_status_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher status_pub_;
  ros::Publisher exploration_command_pub_;
  ros::Publisher exploration_task_request_pub_;
  ros::Publisher navigation_command_pub_;
  ros::Publisher navigation_task_mode_pub_;
  ros::Publisher navigation_goal_pub_;
  ros::Publisher navigation_goal_3d_pub_;
  ros::Publisher exploration_target_goal_pub_;
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
  // Gate requests may arrive while the regular mode handover is still
  // braking. Keep them latched, but do not release command output until the
  // same verified-hover condition has completed.
  bool gate_start_requested_{false};
  bool gate_executing_{false};
  bool gate_end_requested_{false};
  ros::Time gate_edge_hover_satisfied_since_;
  ros::Time hover_satisfied_since_;
  ros::Time last_odom_time_;
  ros::Time last_exploration_start_pub_;
  nav_msgs::Odometry odom_;
  bool have_odom_{false};

  std::string exploration_status_{"IDLE"};
  std::string exploration_status_task_id_;
  // HighSpeedExp publishes terminal status continuously while PAUSED.  Keep
  // the terminal tuple and the local hold latch so each task can cause at
  // most one command-gateway anchor capture.
  std::string terminal_exploration_task_id_;
  PlannerTaskResult terminal_exploration_result_{PlannerTaskResult::NONE};
  bool exploration_terminal_hold_locked_{false};
  geometry_msgs::PoseStamped exploration_target_goal_;
  bool have_exploration_target_goal_{false};
  std::string navigation_status_{"INIT"};
  std::uint64_t navigation_status_epoch_{0};
  std::uint64_t navigation_goal_sequence_{0};
  std::uint64_t navigation_goal_sequence_before_dispatch_{0};
  bool navigation_goal_dispatch_pending_{false};

  std::string exploration_command_topic_;
  std::string exploration_task_request_topic_;
  std::string exploration_status_topic_;
  std::string navigation_command_topic_;
  std::string navigation_status_topic_;
  std::string gate_status_topic_;
  std::string navigation_task_mode_topic_;
  std::string navigation_goal_out_topic_;
  std::string navigation_goal_3d_in_topic_;
  std::string navigation_goal_3d_out_topic_;
  std::string exploration_target_goal_in_topic_;
  std::string exploration_target_goal_out_topic_;
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
  double source_timeout_abort_duration_{1.0};
  std::uint64_t next_text_request_id_{1};
  std::string runtime_session_id_;
};

} // namespace general_planner::planner_runtime
