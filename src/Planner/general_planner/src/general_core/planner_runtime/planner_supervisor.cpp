#include <general_core/planner_runtime/planner_supervisor.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace general_planner::planner_runtime {

PlannerSupervisor::PlannerSupervisor(ros::NodeHandle &nh,
                                     PlannerCommandGateway &gateway)
    : nh_(nh), gateway_(gateway) {
  std::string initial_mode_text = "hold";
  nh_.param<std::string>("initial_mode", initial_mode_text, initial_mode_text);
  PlannerMode parsed_initial = PlannerMode::HOLD;
  if (!parsePlannerMode(initial_mode_text, parsed_initial)) {
    ROS_WARN_STREAM("[planner_supervisor] unknown initial_mode='"
                    << initial_mode_text << "', fallback to hold");
    parsed_initial = PlannerMode::HOLD;
  }
  initial_mode_ = parsed_initial;

  nh_.param<std::string>("exploration_command_topic", exploration_command_topic_,
                         "/planning/exploration/command");
  nh_.param<std::string>("exploration_status_topic", exploration_status_topic_,
                         "/planning/exploration/status");
  nh_.param<std::string>("navigation_command_topic", navigation_command_topic_,
                         "/planning/navigation/command");
  nh_.param<std::string>("navigation_status_topic", navigation_status_topic_,
                         "/planning/navigation/status");
  nh_.param<std::string>("navigation_task_mode_topic",
                         navigation_task_mode_topic_,
                         "/planning/navigation_task_mode");
  nh_.param<std::string>("navigation_goal_out_topic", navigation_goal_out_topic_,
                         "/planning/click_goal");
  nh_.param<std::string>("click_demo_goal_topic", click_demo_goal_topic_,
                         "/goal");
  nh_.param<std::string>("handover_command_topic", handover_command_topic_,
                         "/planner/handover");
  nh_.param<std::string>("handover_status_topic", handover_status_topic_,
                         "/planner/handover_status");
  nh_.param<std::string>("odometry_topic", odometry_topic_, "/lidar_slam/odom");
  nh_.param("hover_speed_threshold", hover_speed_threshold_, 0.10);
  nh_.param("hover_yaw_rate_threshold", hover_yaw_rate_threshold_, 0.10);
  nh_.param("hover_hold_duration", hover_hold_duration_, 0.50);
  nh_.param("max_odom_age", max_odom_age_, 0.20);
  nh_.param("status_rate", status_rate_, 10.0);
  nh_.param("navigation_enabled", navigation_enabled_, true);
  nh_.param("exploration_enabled", exploration_enabled_, true);
  nh_.param("serial_handover", serial_handover_, true);
  nh_.param("exploration_start_retry_period", exploration_start_retry_period_,
            0.5);
  hover_speed_threshold_ = std::clamp(hover_speed_threshold_, 0.01, 1.0);
  hover_yaw_rate_threshold_ = std::clamp(hover_yaw_rate_threshold_, 0.01, 1.0);
  hover_hold_duration_ = std::clamp(hover_hold_duration_, 0.1, 5.0);
  max_odom_age_ = std::clamp(max_odom_age_, 0.05, 1.0);
  status_rate_ = std::clamp(status_rate_, 1.0, 50.0);
  exploration_start_retry_period_ =
      std::clamp(exploration_start_retry_period_, 0.1, 2.0);

  status_.active_mode = PlannerMode::HOLD;
  status_.requested_mode = initial_mode_;
  status_.phase = PlannerPhase::BOOT;
  status_.mode_state = ModeState::HOLD_IDLE;
  status_.command_owner = CommandOwner::HOLD;
  status_.reason = "boot";

  status_pub_ =
      nh_.advertise<general_planner::PlannerStatus>("/planner/status", 10, true);
  // Latched so late-joining exploration_node still receives the latest START.
  exploration_command_pub_ =
      nh_.advertise<std_msgs::String>(exploration_command_topic_, 10, true);
  navigation_command_pub_ =
      nh_.advertise<std_msgs::String>(navigation_command_topic_, 10, true);
  navigation_task_mode_pub_ =
      nh_.advertise<std_msgs::String>(navigation_task_mode_topic_, 10, true);
  navigation_goal_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(navigation_goal_out_topic_, 10);
  click_demo_goal_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(click_demo_goal_topic_, 10);
  // Latched so a late-starting handover helper still sees the latest request.
  handover_command_pub_ =
      nh_.advertise<std_msgs::String>(handover_command_topic_, 10, true);

  mode_request_sub_ = nh_.subscribe("/planner/mode_request", 10,
                                    &PlannerSupervisor::modeRequestCallback,
                                    this);
  mode_request_text_sub_ =
      nh_.subscribe("/planner/mode_request_text", 10,
                    &PlannerSupervisor::modeRequestTextCallback, this);
  navigation_goal_sub_ =
      nh_.subscribe("/planner/navigation/goal", 10,
                    &PlannerSupervisor::navigationGoalCallback, this);
  exploration_trigger_sub_ =
      nh_.subscribe("/planner/exploration/trigger", 10,
                    &PlannerSupervisor::clickGoalCallback, this);
  // Mode-aware click entry used by planner_runtime.rviz SetGoal.
  exploration_rviz_trigger_subs_.push_back(
      nh_.subscribe("/planner/click_goal", 10,
                    &PlannerSupervisor::clickGoalCallback, this));
  // In serial handover mode the click-demo fsm owns /goal after state2state,
  // so supervisor must not also subscribe there (would create a publish loop).
  if (!serial_handover_) {
    for (const char *topic : {"/goal", "/move_base_simple/goal"}) {
      exploration_rviz_trigger_subs_.push_back(
          nh_.subscribe(topic, 10, &PlannerSupervisor::clickGoalCallback, this));
    }
  } else {
    exploration_rviz_trigger_subs_.push_back(
        nh_.subscribe("/move_base_simple/goal", 10,
                      &PlannerSupervisor::clickGoalCallback, this));
  }
  exploration_status_sub_ =
      nh_.subscribe(exploration_status_topic_, 10,
                    &PlannerSupervisor::explorationStatusCallback, this);
  navigation_status_sub_ =
      nh_.subscribe(navigation_status_topic_, 10,
                    &PlannerSupervisor::navigationStatusCallback, this);
  handover_status_sub_ =
      nh_.subscribe(handover_status_topic_, 10,
                    &PlannerSupervisor::handoverStatusCallback, this);
  odom_sub_ = nh_.subscribe(odometry_topic_, 50,
                            &PlannerSupervisor::odometryCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0 / status_rate_),
                           &PlannerSupervisor::timerCallback, this);

  gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
  // Keep navigation disarmed until an explicit state2state activation.
  if (navigation_enabled_ && !serial_handover_) {
    publishNavigationCommand("CLEAR 0");
  }
  ROS_INFO_STREAM("[planner_supervisor] initial_mode="
                  << toString(initial_mode_)
                  << " navigation_enabled=" << navigation_enabled_
                  << " exploration_enabled=" << exploration_enabled_
                  << " serial_handover=" << serial_handover_
                  << " status=/planner/status");
}

void PlannerSupervisor::modeRequestCallback(
    const general_planner::PlannerModeRequestConstPtr &msg) {
  if (!msg) {
    return;
  }
  PlannerMode mode = PlannerMode::HOLD;
  switch (msg->mode) {
  case general_planner::PlannerModeRequest::MODE_STATE2STATE:
    mode = PlannerMode::STATE2STATE;
    break;
  case general_planner::PlannerModeRequest::MODE_EXPLORATION:
    mode = PlannerMode::EXPLORATION;
    break;
  case general_planner::PlannerModeRequest::MODE_EMERGENCY_STOP:
    mode = PlannerMode::EMERGENCY_STOP;
    break;
  case general_planner::PlannerModeRequest::MODE_HOLD:
  default:
    mode = PlannerMode::HOLD;
    break;
  }
  handleModeRequest(msg->request_id, mode, msg->task_id, "mode_request");
}

void PlannerSupervisor::modeRequestTextCallback(
    const std_msgs::StringConstPtr &msg) {
  if (!msg) {
    return;
  }
  PlannerMode mode = PlannerMode::HOLD;
  if (!parsePlannerMode(msg->data, mode)) {
    ROS_WARN_STREAM("[planner_supervisor] ignore unknown mode_request_text='"
                    << msg->data << "'");
    return;
  }
  const std::uint64_t request_id = next_text_request_id_++;
  handleModeRequest(request_id, mode, "", "mode_request_text");
}

void PlannerSupervisor::handleModeRequest(const std::uint64_t request_id,
                                          const PlannerMode mode,
                                          const std::string &task_id,
                                          const std::string &source) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (mode == PlannerMode::STATE2STATE && !navigation_enabled_ &&
      !serial_handover_) {
    status_.accepted_request_id = request_id;
    status_.phase = PlannerPhase::FAILED;
    status_.reason =
        "navigation fsm disabled (relaunch with enable_navigation:=true or "
        "serial_handover:=true)";
    ROS_WARN_STREAM("[planner_supervisor] reject state2state: "
                    << status_.reason);
    return;
  }
  if (mode == PlannerMode::EXPLORATION && !exploration_enabled_) {
    status_.accepted_request_id = request_id;
    status_.phase = PlannerPhase::FAILED;
    status_.reason =
        "exploration node disabled (relaunch with enable_exploration:=true)";
    ROS_WARN_STREAM("[planner_supervisor] reject exploration: "
                    << status_.reason);
    return;
  }
  if (mode == PlannerMode::EMERGENCY_STOP) {
    status_.accepted_request_id = request_id;
    status_.requested_mode = PlannerMode::HOLD;
    status_.phase = PlannerPhase::EMERGENCY;
    status_.ready_for_new_task = false;
    status_.stable_hover = false;
    status_.task_result = PlannerTaskResult::CANCELED;
    status_.reason = "emergency_stop from " + source;
    ++status_.transition_id;
    ++status_.task_epoch;
    transition_active_ = true;
    transition_target_ = PlannerMode::HOLD;
    hold_anchor_locked_for_transition_ = false;
    hover_satisfied_since_ = ros::Time();
    requestAdapterStop(status_.active_mode, "emergency_stop");
    gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
    gateway_.lockHoldAnchorFromOdom();
    hold_anchor_locked_for_transition_ = true;
    status_.command_owner = CommandOwner::HOLD;
    return;
  }

  if (!boot_complete_) {
    status_.requested_mode = mode;
    if (!task_id.empty()) {
      status_.task_id = task_id;
    }
    status_.accepted_request_id = request_id;
    status_.reason = "queued until boot hover ready (" + source + ")";
    return;
  }

  if (transition_active_) {
    status_.requested_mode = mode;
    status_.accepted_request_id = request_id;
    if (!task_id.empty()) {
      status_.task_id = task_id;
    }
    if (mode != transition_target_) {
      transition_target_ = mode;
      status_.reason = "retarget transition to " + std::string(toString(mode));
      requestAdapterStop(status_.active_mode, "retarget");
    }
    return;
  }

  if (status_.active_mode == mode &&
      (status_.phase == PlannerPhase::WAITING_INPUT ||
       status_.phase == PlannerPhase::STABLE_HOLD ||
       status_.phase == PlannerPhase::PLANNING ||
       status_.phase == PlannerPhase::EXECUTING)) {
    status_.accepted_request_id = request_id;
    status_.requested_mode = mode;
    if (!task_id.empty()) {
      status_.task_id = task_id;
    }
    status_.reason = "mode already active (" + source + ")";
    // Finished exploration can be re-armed explicitly; waiting-for-trigger is
    // already the correct idle state and should not restart a transition.
    if (mode == PlannerMode::EXPLORATION &&
        (status_.phase == PlannerPhase::STABLE_HOLD ||
         exploration_status_ == "PAUSED" ||
         exploration_status_ == "SUCCEEDED") &&
        status_.mode_state == ModeState::EXP_PAUSED) {
      beginTransition(mode, request_id, task_id, "rearm exploration");
    }
    return;
  }

  beginTransition(mode, request_id, task_id, source);
}

void PlannerSupervisor::beginTransition(const PlannerMode target,
                                        const std::uint64_t request_id,
                                        const std::string &task_id,
                                        const std::string &reason) {
  ++status_.transition_id;
  // Invalidate any in-flight plans/commands from the previous task epoch.
  ++status_.task_epoch;
  status_.accepted_request_id = request_id;
  status_.requested_mode = target;
  transition_target_ = target;
  transition_active_ = true;
  hold_anchor_locked_for_transition_ = false;
  hover_satisfied_since_ = ros::Time();
  status_.ready_for_new_task = false;
  status_.stable_hover = false;
  status_.task_result = PlannerTaskResult::NONE;
  status_.phase = PlannerPhase::BRAKING;
  status_.reason = "braking for " + std::string(toString(target)) + " (" +
                   reason + ")";
  if (!task_id.empty()) {
    status_.task_id = task_id;
  } else {
    status_.task_id = makeTaskId(target);
  }

  requestAdapterStop(status_.active_mode, "mode transition");
  // Keep the current flight source during braking so adapters can decelerate;
  // switch to locked HOLD only after the vehicle is slow enough.
  const CommandOwner braking_owner = ownerForMode(status_.active_mode);
  if (braking_owner == CommandOwner::HOLD) {
    gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
    gateway_.lockHoldAnchorFromOdom();
    hold_anchor_locked_for_transition_ = true;
    status_.command_owner = CommandOwner::HOLD;
    status_.phase = PlannerPhase::HOLD_VERIFY;
    status_.reason = "hold verify for " + std::string(toString(target));
  } else {
    gateway_.setAuthorizedOwner(braking_owner, status_.task_epoch);
    status_.command_owner = braking_owner;
  }
}

void PlannerSupervisor::requestAdapterStop(const PlannerMode mode,
                                           const std::string &reason) {
  if (mode == PlannerMode::EXPLORATION) {
    publishExplorationCommand("PAUSE " + status_.task_id);
  }
  if (mode == PlannerMode::STATE2STATE) {
    publishNavigationCommand("PAUSE " + std::to_string(status_.task_epoch));
  }
  ROS_INFO_STREAM("[planner_supervisor] request stop mode="
                  << toString(mode) << " reason=" << reason);
}

void PlannerSupervisor::resetAdapterTaskState(const PlannerMode mode) {
  if (mode == PlannerMode::EXPLORATION) {
    publishExplorationCommand("PAUSE " + status_.task_id);
  }
  if (mode == PlannerMode::STATE2STATE) {
    publishNavigationCommand("CLEAR " + std::to_string(status_.task_epoch));
  }
}

void PlannerSupervisor::enterStableHold(const std::string &reason,
                                        const PlannerTaskResult result) {
  status_.phase = PlannerPhase::STABLE_HOLD;
  status_.stable_hover = true;
  status_.ready_for_new_task = true;
  status_.task_result = result;
  status_.command_owner = CommandOwner::HOLD;
  status_.mode_state = ModeState::HOLD_IDLE;
  status_.reason = reason;
  gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
  if (!hold_anchor_locked_for_transition_) {
    gateway_.lockHoldAnchorFromOdom();
    hold_anchor_locked_for_transition_ = true;
  }
}

void PlannerSupervisor::activateMode(const PlannerMode mode,
                                     const std::string &reason) {
  resetAdapterTaskState(status_.active_mode);
  status_.active_mode = mode;
  status_.requested_mode = mode;
  transition_active_ = false;
  hold_anchor_locked_for_transition_ = true;

  if (mode == PlannerMode::HOLD) {
    enterStableHold(reason, PlannerTaskResult::NONE);
    publishNavigationTaskMode("hold");
    return;
  }

  if (mode == PlannerMode::STATE2STATE) {
    if (!serial_handover_ && !navigation_enabled_) {
      status_.phase = PlannerPhase::FAILED;
      status_.reason =
          "cannot activate state2state without navigation fsm_node";
      return;
    }
    exploration_start_pending_ = false;
    status_.phase = PlannerPhase::WAITING_INPUT;
    status_.mode_state = ModeState::S2S_WAIT_GOAL;
    status_.stable_hover = true;
    status_.command_owner = CommandOwner::HOLD;
    gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
    if (serial_handover_) {
      // Keep publishing HOLD until the click-demo fsm is up, then release the
      // command bus so fsm_node owns /planning/pos_cmd directly.
      serial_state2state_ready_ = false;
      serial_handover_pending_ = true;
      status_.ready_for_new_task = false;
      status_.reason = reason + "; serial handover to click-demo fsm";
      publishExplorationCommand("PAUSE " + status_.task_id);
      publishHandoverCommand("start_state2state");
      ROS_INFO_STREAM(
          "[planner_supervisor] serial handover requested: kill exploration, "
          "start click-demo fsm_node");
      return;
    }
    status_.ready_for_new_task = true;
    status_.reason = reason;
    publishNavigationTaskMode("state2state");
    publishNavigationCommand("ARM " + std::to_string(status_.task_epoch));
    return;
  }

  if (mode == PlannerMode::EXPLORATION) {
    if (!exploration_enabled_) {
      status_.phase = PlannerPhase::FAILED;
      status_.reason = "cannot activate exploration without exploration_node";
      return;
    }
    // Parallel mode: keep navigation fsm alive but disarmed. Serial mode:
    // exploration stack is owned by handover helper / launch.
    exploration_start_pending_ = false;
    serial_state2state_ready_ = false;
    serial_handover_pending_ = false;
    gateway_.setPublishingEnabled(true);
    if (navigation_enabled_ && !serial_handover_) {
      publishNavigationCommand("CLEAR " + std::to_string(status_.task_epoch));
      publishNavigationTaskMode("hold");
    }
    if (serial_handover_) {
      // Stack may still be relaunching; accept triggers only after
      // exploration_ready replaces the latched PAUSE from state2state.
      serial_handover_pending_ = true;
      status_.ready_for_new_task = false;
      status_.reason =
          reason + "; relaunching exploration stack via serial handover";
      publishHandoverCommand("start_exploration");
    } else {
      status_.ready_for_new_task = true;
      status_.reason =
          reason + "; waiting for exploration trigger (/planner/click_goal)";
    }
    status_.phase = PlannerPhase::WAITING_INPUT;
    status_.mode_state = ModeState::EXP_WAIT_TRIGGER;
    status_.stable_hover = true;
    status_.command_owner = CommandOwner::HOLD;
    status_.task_result = PlannerTaskResult::NONE;
    gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
    ROS_INFO_STREAM("[planner_supervisor] exploration armed"
                    << (serial_handover_ ? " (waiting for stack relaunch)"
                                         : " and waiting for trigger"));
    return;
  }
}

void PlannerSupervisor::requestExplorationStartLocked(const std::string &reason) {
  if (status_.task_id.empty()) {
    status_.task_id = makeTaskId(PlannerMode::EXPLORATION);
  }
  exploration_start_pending_ = true;
  last_exploration_start_pub_ = ros::Time::now();
  publishExplorationCommand("START " + status_.task_id);
  ROS_INFO_STREAM("[planner_supervisor] START exploration task_id="
                  << status_.task_id << " reason=" << reason);
}

bool PlannerSupervisor::acceptNavigationGoalLocked(
    const geometry_msgs::PoseStamped &msg) {
  if (!status_.ready_for_new_task ||
      status_.active_mode != PlannerMode::STATE2STATE || transition_active_) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_supervisor] drop navigation goal: not ready "
                      "(ready=%d mode=%s transition=%d)",
                      status_.ready_for_new_task,
                      toString(status_.active_mode), transition_active_);
    return false;
  }
  if (serial_handover_) {
    if (!serial_state2state_ready_) {
      ROS_WARN_THROTTLE(1.0,
                        "[planner_supervisor] drop navigation goal: waiting "
                        "for serial click-demo fsm handover");
      return false;
    }
    // Click-demo fsm owns planning; only forward the goal and keep accepting
    // subsequent clicks (fsm handles busy/replan itself).
    status_.task_result = PlannerTaskResult::NONE;
    status_.phase = PlannerPhase::PLANNING;
    status_.ready_for_new_task = true;
    status_.reason = "forwarded click goal to serial fsm";
    click_demo_goal_pub_.publish(msg);
    ROS_INFO_STREAM("[planner_supervisor] serial click goal -> "
                    << click_demo_goal_topic_ << " p=(" << msg.pose.position.x
                    << "," << msg.pose.position.y << "," << msg.pose.position.z
                    << ")");
    return true;
  }
  status_.task_result = PlannerTaskResult::NONE;
  status_.phase = PlannerPhase::PLANNING;
  status_.ready_for_new_task = false;
  status_.stable_hover = false;
  status_.command_owner = CommandOwner::STATE2STATE;
  status_.reason = "navigation goal accepted";
  gateway_.setAuthorizedOwner(CommandOwner::STATE2STATE, status_.task_epoch);
  navigation_goal_pub_.publish(msg);
  ROS_INFO_STREAM("[planner_supervisor] navigation goal accepted epoch="
                  << status_.task_epoch << " p=(" << msg.pose.position.x << ","
                  << msg.pose.position.y << "," << msg.pose.position.z << ")");
  return true;
}

void PlannerSupervisor::navigationGoalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  acceptNavigationGoalLocked(*msg);
}

void PlannerSupervisor::clickGoalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!boot_complete_ || transition_active_) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_supervisor] drop click goal: boot=%d "
                      "transition=%d mode=%s",
                      boot_complete_, transition_active_,
                      toString(status_.active_mode));
    return;
  }
  if (status_.active_mode == PlannerMode::STATE2STATE) {
    acceptNavigationGoalLocked(*msg);
    return;
  }
  if (status_.active_mode == PlannerMode::EXPLORATION) {
    acceptExplorationTriggerLocked(*msg);
    return;
  }
  ROS_WARN_THROTTLE(1.0,
                    "[planner_supervisor] drop click goal: mode=%s "
                    "(need exploration or state2state)",
                    toString(status_.active_mode));
}

bool PlannerSupervisor::acceptExplorationTriggerLocked(
    const geometry_msgs::PoseStamped &msg) {
  (void)msg;
  if (!boot_complete_ || status_.active_mode != PlannerMode::EXPLORATION ||
      transition_active_ || !status_.ready_for_new_task) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_supervisor] drop exploration trigger: mode=%s "
                      "boot=%d transition=%d ready=%d",
                      toString(status_.active_mode), boot_complete_,
                      transition_active_, status_.ready_for_new_task);
    return false;
  }
  // Only accept a new trigger while idle / waiting / finished. Ignore clicks
  // during active planning/execution to avoid restarting mid-flight.
  if (status_.phase == PlannerPhase::PLANNING ||
      status_.phase == PlannerPhase::EXECUTING ||
      status_.phase == PlannerPhase::BRAKING ||
      status_.phase == PlannerPhase::HOLD_VERIFY) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_supervisor] drop exploration trigger while "
                      "phase=%s",
                      toString(status_.phase));
    return false;
  }

  // RViz 2D Nav Goal (or /planner/exploration/trigger) starts exploration.
  status_.task_result = PlannerTaskResult::NONE;
  status_.phase = PlannerPhase::PLANNING;
  status_.mode_state = ModeState::EXP_PLAN_TRAJ;
  status_.ready_for_new_task = false;
  status_.stable_hover = false;
  status_.command_owner = CommandOwner::HOLD;
  status_.reason = "exploration trigger accepted; starting";
  // A new trigger is a new task even when exploration mode itself did not
  // change.  This makes task ids monotonic and prevents a delayed terminal
  // status from the preceding exploration from affecting the new one.
  ++status_.task_epoch;
  status_.task_id = makeTaskId(PlannerMode::EXPLORATION);
  gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
  requestExplorationStartLocked("rviz/manual trigger");
  return true;
}

void PlannerSupervisor::explorationTriggerCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  acceptExplorationTriggerLocked(*msg);
}

void PlannerSupervisor::explorationStatusCallback(
    const std_msgs::StringConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::istringstream stream(msg->data);
  std::string state;
  stream >> state;
  std::string task_id;
  std::getline(stream, task_id);
  const auto first = task_id.find_first_not_of(" \t");
  task_id = first == std::string::npos ? std::string() : task_id.substr(first);

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_.active_mode == PlannerMode::EXPLORATION &&
      !task_id.empty() && task_id != status_.task_id) {
    ROS_WARN_THROTTLE(
        1.0,
        "[planner_supervisor] ignore stale exploration status task_id=%s "
        "active_task_id=%s state=%s",
        task_id.c_str(), status_.task_id.c_str(), state.c_str());
    return;
  }
  exploration_status_ = state;
  exploration_status_task_id_ = task_id;
  if (status_.active_mode == PlannerMode::EXPLORATION && !transition_active_) {
    status_.mode_state = modeStateFromExplorationString(state);
    if (state == "RUNNING" || state == "PLAN_TRAJ" || state == "EXEC_TRAJ" ||
        state == "REORIENT" || state == "CAUTION") {
      exploration_start_pending_ = false;
      status_.phase = (state == "EXEC_TRAJ" || state == "REORIENT")
                          ? PlannerPhase::EXECUTING
                          : PlannerPhase::PLANNING;
      status_.ready_for_new_task = false;
      status_.stable_hover = false;
      status_.command_owner = CommandOwner::EXPLORATION;
      gateway_.setAuthorizedOwner(CommandOwner::EXPLORATION, status_.task_epoch);
      status_.reason = "exploration " + state;
    } else if (state == "SUCCEEDED" || state == "PAUSED") {
      exploration_start_pending_ = false;
      // FastExplorationFSM publishes this terminal status continuously at its
      // FSM rate.  Only the first terminal notification for this task starts
      // the handover-to-HOLD transition; subsequent messages must preserve
      // STABLE_HOLD/ready_for_new_task.
      if (!task_id.empty() && task_id == completed_exploration_task_id_) {
        return;
      }
      if (!task_id.empty()) {
        completed_exploration_task_id_ = task_id;
      }
      // Exploration finished but remains commandable after stable hold.
      if (!transition_active_) {
        transition_active_ = true;
        transition_target_ = PlannerMode::EXPLORATION;
        status_.phase = PlannerPhase::BRAKING;
        status_.ready_for_new_task = false;
        status_.stable_hover = false;
        status_.task_result = PlannerTaskResult::SUCCEEDED;
        status_.reason = "exploration finished, verifying hover";
        hold_anchor_locked_for_transition_ = false;
        hover_satisfied_since_ = ros::Time();
        gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
      }
    } else if (state == "FAILED") {
      status_.phase = PlannerPhase::FAILED;
      status_.task_result = PlannerTaskResult::FAILED;
      status_.ready_for_new_task = false;
      status_.reason = "exploration failed";
    }
  }
}

void PlannerSupervisor::navigationStatusCallback(
    const std_msgs::StringConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::istringstream stream(msg->data);
  std::string state;
  stream >> state;
  std::lock_guard<std::mutex> lock(mutex_);
  navigation_status_ = state;
  if (serial_handover_) {
    return;
  }
  if (status_.active_mode == PlannerMode::STATE2STATE && !transition_active_) {
    status_.mode_state = modeStateFromNavigationString(state);
    if (state == "FOLLOW_TRAJ" || state == "STATIC_TRACKING" ||
        state == "HOLD_TRACKING" || state == "YAWING") {
      status_.phase = PlannerPhase::EXECUTING;
      status_.ready_for_new_task = false;
      status_.stable_hover = false;
      status_.command_owner = CommandOwner::STATE2STATE;
      gateway_.setAuthorizedOwner(CommandOwner::STATE2STATE, status_.task_epoch);
      status_.reason = "navigation " + state;
    } else if (state == "GENERATE_TRAJ") {
      status_.phase = PlannerPhase::PLANNING;
      status_.ready_for_new_task = false;
      status_.command_owner = CommandOwner::STATE2STATE;
      gateway_.setAuthorizedOwner(CommandOwner::STATE2STATE, status_.task_epoch);
      status_.reason = "navigation planning";
    } else if (state == "WAIT_GOAL") {
      // Do not demote an in-flight goal acceptance: FSM still publishes
      // WAIT_GOAL until the goal callback flips it to GENERATE_TRAJ.
      if (status_.phase == PlannerPhase::PLANNING ||
          status_.phase == PlannerPhase::EXECUTING) {
        return;
      }
      status_.phase = PlannerPhase::WAITING_INPUT;
      status_.ready_for_new_task = true;
      status_.stable_hover = hoverConditionMetLocked();
      status_.command_owner = CommandOwner::HOLD;
      gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
      if (!gateway_.hasHoldAnchor()) {
        gateway_.lockHoldAnchorFromOdom();
      }
      status_.reason = "navigation wait goal";
      status_.task_result = PlannerTaskResult::SUCCEEDED;
    }
  }
}

void PlannerSupervisor::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  odom_ = *msg;
  have_odom_ = true;
  last_odom_time_ = ros::Time::now();
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  const double vz = msg->twist.twist.linear.z;
  status_.speed_mps =
      static_cast<float>(std::sqrt(vx * vx + vy * vy + vz * vz));
  status_.yaw_rate_rps = static_cast<float>(msg->twist.twist.angular.z);
  status_.odom_valid =
      (ros::Time::now() - last_odom_time_).toSec() <= max_odom_age_;
}

bool PlannerSupervisor::hoverConditionMetLocked() const {
  if (!have_odom_) {
    return false;
  }
  const double odom_age = (ros::Time::now() - last_odom_time_).toSec();
  if (odom_age > max_odom_age_) {
    return false;
  }
  return status_.speed_mps <= hover_speed_threshold_ &&
         std::abs(status_.yaw_rate_rps) <= hover_yaw_rate_threshold_;
}

std::string PlannerSupervisor::makeTaskId(const PlannerMode mode) const {
  std::ostringstream oss;
  oss << status_.task_epoch << ":" << toString(mode);
  return oss.str();
}

void PlannerSupervisor::publishExplorationCommand(const std::string &command) {
  std_msgs::String msg;
  msg.data = command;
  exploration_command_pub_.publish(msg);
}

void PlannerSupervisor::publishNavigationCommand(const std::string &command) {
  std_msgs::String msg;
  msg.data = command;
  navigation_command_pub_.publish(msg);
}

void PlannerSupervisor::publishNavigationTaskMode(const std::string &mode) {
  std_msgs::String msg;
  msg.data = mode;
  navigation_task_mode_pub_.publish(msg);
}

void PlannerSupervisor::publishHandoverCommand(const std::string &command) {
  std_msgs::String msg;
  msg.data = command;
  handover_command_pub_.publish(msg);
}

void PlannerSupervisor::handoverStatusCallback(
    const std_msgs::StringConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!serial_handover_) {
    return;
  }
  const std::string &state = msg->data;
  if (state == "state2state_ready") {
    serial_handover_pending_ = false;
    serial_state2state_ready_ = true;
    status_.active_mode = PlannerMode::STATE2STATE;
    status_.requested_mode = PlannerMode::STATE2STATE;
    status_.phase = PlannerPhase::WAITING_INPUT;
    status_.mode_state = ModeState::S2S_WAIT_GOAL;
    status_.ready_for_new_task = true;
    status_.stable_hover = hoverConditionMetLocked();
    status_.command_owner = CommandOwner::STATE2STATE;
    status_.reason =
        "serial click-demo fsm ready; publish goals to /planner/click_goal";
    // Click-demo fsm_node now owns /planning/pos_cmd.
    gateway_.setPublishingEnabled(false);
    ROS_INFO("[planner_supervisor] serial state2state ready; gateway released");
    return;
  }
  if (state == "exploration_ready") {
    serial_handover_pending_ = false;
    serial_state2state_ready_ = false;
    gateway_.setPublishingEnabled(true);
    gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
    // Replace the latched PAUSE left from the previous state2state handover so
    // a freshly started exploration_node does not immediately pause itself.
    publishExplorationCommand("READY");
    if (exploration_start_pending_) {
      requestExplorationStartLocked("reissue after exploration_ready");
    }
    status_.reason =
        "serial exploration stack ready; waiting for /planner/exploration/trigger";
    status_.ready_for_new_task = true;
    status_.phase = PlannerPhase::WAITING_INPUT;
    status_.mode_state = ModeState::EXP_WAIT_TRIGGER;
    status_.active_mode = PlannerMode::EXPLORATION;
    status_.requested_mode = PlannerMode::EXPLORATION;
    ROS_INFO("[planner_supervisor] serial exploration ready; gateway enabled");
    return;
  }
  if (state.rfind("failed", 0) == 0) {
    serial_handover_pending_ = false;
    status_.phase = PlannerPhase::FAILED;
    status_.ready_for_new_task = false;
    status_.reason = "serial handover failed: " + state;
    gateway_.setPublishingEnabled(true);
    gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
    ROS_ERROR_STREAM("[planner_supervisor] " << status_.reason);
  }
}

void PlannerSupervisor::publishStatus() {
  general_planner::PlannerStatus msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id =
        odom_.header.frame_id.empty() ? "world" : odom_.header.frame_id;
    msg.transition_id = status_.transition_id;
    msg.task_epoch = status_.task_epoch;
    msg.world_epoch = status_.world_epoch;
    msg.map_revision = status_.map_revision;
    msg.topo_revision = status_.topo_revision;
    msg.accepted_request_id = status_.accepted_request_id;
    msg.task_id = status_.task_id;
    msg.active_mode = static_cast<std::uint8_t>(status_.active_mode);
    msg.requested_mode = static_cast<std::uint8_t>(status_.requested_mode);
    msg.phase = static_cast<std::uint8_t>(status_.phase);
    msg.mode_state = static_cast<std::uint16_t>(status_.mode_state);
    msg.task_result = static_cast<std::uint8_t>(status_.task_result);
    msg.active_mode_str = toString(status_.active_mode);
    msg.requested_mode_str = toString(status_.requested_mode);
    msg.phase_str = toString(status_.phase);
    msg.mode_state_str = toString(status_.mode_state);
    msg.command_owner_str = toString(status_.command_owner);
    msg.task_result_str = toString(status_.task_result);
    msg.stable_hover = status_.stable_hover;
    msg.ready_for_new_task = status_.ready_for_new_task;
    msg.odom_valid = status_.odom_valid;
    msg.map_ready = status_.map_ready;
    msg.topology_ready = status_.topology_ready;
    msg.command_owner = static_cast<std::uint8_t>(status_.command_owner);
    msg.speed_mps = status_.speed_mps;
    msg.yaw_rate_rps = status_.yaw_rate_rps;
    msg.reason = status_.reason;
  }
  status_pub_.publish(msg);
}

void PlannerSupervisor::timerCallback(const ros::TimerEvent &) {
  PlannerMode boot_activate_mode = PlannerMode::HOLD;
  bool do_boot_activate = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.odom_valid =
        have_odom_ &&
        (ros::Time::now() - last_odom_time_).toSec() <= max_odom_age_;
    // Map/topology readiness are filled by GlobalMapRuntime in M2.
    status_.map_ready = status_.odom_valid;
    status_.topology_ready = status_.odom_valid;

    if (!boot_complete_) {
      if (!status_.odom_valid) {
        status_.phase = PlannerPhase::BOOT;
        status_.reason = "waiting for odometry";
      } else if (!hoverConditionMetLocked()) {
        status_.phase = PlannerPhase::HOLD_VERIFY;
        status_.reason = "boot hover verify";
        gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
        if (!hold_anchor_locked_for_transition_) {
          gateway_.lockHoldAnchorFromOdom();
          hold_anchor_locked_for_transition_ = true;
        }
        hover_satisfied_since_ = ros::Time();
      } else {
        if (hover_satisfied_since_.isZero()) {
          hover_satisfied_since_ = ros::Time::now();
        }
        if ((ros::Time::now() - hover_satisfied_since_).toSec() >=
            hover_hold_duration_) {
          boot_complete_ = true;
          boot_activate_mode = status_.requested_mode;
          do_boot_activate = true;
        }
      }
    } else if (transition_active_) {
      if (!hold_anchor_locked_for_transition_ && hoverConditionMetLocked()) {
        gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
        gateway_.lockHoldAnchorFromOdom();
        hold_anchor_locked_for_transition_ = true;
        status_.command_owner = CommandOwner::HOLD;
        status_.phase = PlannerPhase::HOLD_VERIFY;
        status_.reason = "hold verify";
        hover_satisfied_since_ = ros::Time::now();
      } else if (hold_anchor_locked_for_transition_) {
        if (hoverConditionMetLocked()) {
          if (hover_satisfied_since_.isZero()) {
            hover_satisfied_since_ = ros::Time::now();
          }
          if ((ros::Time::now() - hover_satisfied_since_).toSec() >=
              hover_hold_duration_) {
            const PlannerMode target = transition_target_;
            const PlannerTaskResult result = status_.task_result;
            if (target == PlannerMode::EXPLORATION &&
                result == PlannerTaskResult::SUCCEEDED &&
                status_.active_mode == PlannerMode::EXPLORATION) {
              // Finished exploration stays in exploration mode but idle.
              transition_active_ = false;
              enterStableHold("exploration idle after finish", result);
              status_.active_mode = PlannerMode::EXPLORATION;
              status_.mode_state = ModeState::EXP_PAUSED;
              status_.ready_for_new_task = true;
            } else {
              activateMode(target, "stable hold reached");
            }
          }
        } else {
          hover_satisfied_since_ = ros::Time();
          status_.phase = PlannerPhase::HOLD_VERIFY;
          status_.stable_hover = false;
        }
      } else {
        status_.phase = PlannerPhase::BRAKING;
        status_.stable_hover = false;
      }
    } else {
      updatePhaseFromActiveModeLocked();
      if (exploration_start_pending_ &&
          status_.active_mode == PlannerMode::EXPLORATION &&
          !transition_active_) {
        const double since_pub =
            last_exploration_start_pub_.isZero()
                ? exploration_start_retry_period_
                : (ros::Time::now() - last_exploration_start_pub_).toSec();
        if (since_pub >= exploration_start_retry_period_) {
          requestExplorationStartLocked("start retry");
        }
      }
    }

    if (do_boot_activate) {
      status_.active_mode = PlannerMode::HOLD;
      if (status_.task_id.empty()) {
        status_.task_id = makeTaskId(boot_activate_mode);
      }
      activateMode(boot_activate_mode, "boot complete");
    }
  }

  publishStatus();
}

void PlannerSupervisor::updatePhaseFromActiveModeLocked() {
  if (status_.phase == PlannerPhase::BRAKING ||
      status_.phase == PlannerPhase::HOLD_VERIFY ||
      status_.phase == PlannerPhase::EMERGENCY ||
      status_.phase == PlannerPhase::FAILED) {
    return;
  }
  if (status_.active_mode == PlannerMode::HOLD) {
    status_.stable_hover = hoverConditionMetLocked();
    status_.ready_for_new_task = status_.stable_hover;
    status_.mode_state = ModeState::HOLD_IDLE;
    if (status_.stable_hover) {
      status_.phase = PlannerPhase::STABLE_HOLD;
    }
  }
}

} // namespace general_planner::planner_runtime
