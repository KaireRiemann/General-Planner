#include <general_core/planner_runtime/planner_supervisor.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <utility>

namespace general_planner::planner_runtime {

namespace {

double yawFromQuaternion(const geometry_msgs::Quaternion &q) {
  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sin_yaw, cos_yaw);
}

}  // namespace

PlannerSupervisor::PlannerSupervisor(ros::NodeHandle &nh,
                                     PlannerCommandGateway &gateway,
                                     MapStatusProvider map_status_provider)
    : nh_(nh), gateway_(gateway),
      map_status_provider_(std::move(map_status_provider)) {
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
  nh_.param<std::string>("exploration_task_request_topic",
                         exploration_task_request_topic_,
                         "/planning/exploration/task_request");
  nh_.param<std::string>("exploration_status_topic", exploration_status_topic_,
                         "/planning/exploration/status");
  nh_.param<std::string>("navigation_command_topic", navigation_command_topic_,
                         "/planning/navigation/command");
  nh_.param<std::string>("navigation_status_topic", navigation_status_topic_,
                         "/planning/navigation/status");
  nh_.param<std::string>("gate_status_topic", gate_status_topic_,
                         "/planner/gate/status");
  nh_.param<std::string>("navigation_task_mode_topic",
                         navigation_task_mode_topic_,
                         "/planning/navigation_task_mode");
  nh_.param<std::string>("navigation_goal_out_topic", navigation_goal_out_topic_,
                         "/planning/click_goal");
  nh_.param<std::string>("navigation_goal_3d_in_topic",
                         navigation_goal_3d_in_topic_, "/goal_3d");
  nh_.param<std::string>("navigation_goal_3d_out_topic",
                         navigation_goal_3d_out_topic_,
                         "/planning/click_goal_3d");
  nh_.param<std::string>("exploration_target_goal_in_topic",
                         exploration_target_goal_in_topic_,
                         "/planner/exploration/target_goal");
  nh_.param<std::string>("exploration_target_goal_out_topic",
                         exploration_target_goal_out_topic_,
                         "/planning/exploration/target_goal");
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
  // The request is latched for the same reason as the legacy START command:
  // a late adapter must receive its target and start bit as one unit.
  exploration_task_request_pub_ =
      nh_.advertise<general_planner::ExplorationTaskRequest>(
          exploration_task_request_topic_, 10, true);
  navigation_command_pub_ =
      nh_.advertise<std_msgs::String>(navigation_command_topic_, 10, true);
  navigation_task_mode_pub_ =
      nh_.advertise<std_msgs::String>(navigation_task_mode_topic_, 10, true);
  navigation_goal_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(navigation_goal_out_topic_, 10);
  navigation_goal_3d_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(navigation_goal_3d_out_topic_,
                                                 10);
  exploration_target_goal_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(
          exploration_target_goal_out_topic_, 10);
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
  navigation_goal_3d_sub_ =
      nh_.subscribe(navigation_goal_3d_in_topic_, 10,
                    &PlannerSupervisor::navigationGoal3DCallback, this);
  exploration_trigger_sub_ =
      nh_.subscribe("/planner/exploration/trigger", 10,
                    &PlannerSupervisor::clickGoalCallback, this);
  exploration_target_goal_sub_ =
      nh_.subscribe(exploration_target_goal_in_topic_, 10,
                    &PlannerSupervisor::explorationTargetGoalCallback, this);
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
  gate_status_sub_ = nh_.subscribe(gate_status_topic_, 10,
                                   &PlannerSupervisor::gateStatusCallback,
                                   this);
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
                  << " nav_goal_3d=" << navigation_goal_3d_in_topic_
                  << " -> " << navigation_goal_3d_out_topic_
                  << " target_goal=" << exploration_target_goal_in_topic_
                  << " -> " << exploration_target_goal_out_topic_
                  << " gate_status=" << gate_status_topic_
                  << " task_request=" << exploration_task_request_topic_
                  << " status=/planner/status");
  runtime_session_id_ = std::to_string(ros::WallTime::now().toNSec());
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
  case general_planner::PlannerModeRequest::MODE_TARGET_EXPLORATION:
    mode = PlannerMode::TARGET_EXPLORATION;
    break;
  case general_planner::PlannerModeRequest::MODE_GATE:
    mode = PlannerMode::GATE;
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
  // Gate hands the final command bus to a different planner.  The legacy
  // serial handover starts a standalone fsm_node that also writes that bus,
  // so it cannot provide the single-writer guarantee required here.
  if (mode == PlannerMode::GATE && serial_handover_) {
    status_.accepted_request_id = request_id;
    status_.phase = PlannerPhase::FAILED;
    status_.reason =
        "gate requires serial_handover=false (composed command gateway)";
    ROS_WARN_STREAM("[planner_supervisor] reject gate: " << status_.reason);
    return;
  }
  if (isExplorationMode(mode) && !exploration_enabled_) {
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
    hold_anchor_locked_for_transition_ =
        authorizeHoldAtCurrentOdomLocked("emergency stop");
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

  // Do not let a normal mode request re-enable this runtime's command path
  // while the external gate planner is still flying.  Its END notification is
  // the ownership-release edge; the following stable-hover verification is
  // completed before a navigation/exploration request can take effect.
  if (status_.active_mode == PlannerMode::GATE && gate_executing_ &&
      mode != PlannerMode::GATE) {
    status_.accepted_request_id = request_id;
    status_.requested_mode = PlannerMode::GATE;
    status_.reason = "reject " + std::string(toString(mode)) +
                     ": wait for gate END and stable hover";
    ROS_WARN_STREAM("[planner_supervisor] " << status_.reason);
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
    if (isExplorationMode(mode) &&
        (status_.phase == PlannerPhase::STABLE_HOLD ||
         exploration_status_ == "PAUSED" ||
         exploration_status_ == "SUCCEEDED") &&
        status_.mode_state == ModeState::EXP_PAUSED) {
      beginTransition(mode, request_id, task_id, "rearm exploration");
    }
    if (mode == PlannerMode::GATE &&
        status_.phase == PlannerPhase::STABLE_HOLD &&
        status_.mode_state == ModeState::GATE_COMPLETE) {
      beginTransition(mode, request_id, task_id, "rearm gate");
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
  if (target == PlannerMode::GATE) {
    // A new gate run requires a fresh external START edge.  Clearing these at
    // the beginning of the transition makes a repeated gate request safe.
    gate_start_requested_ = false;
    gate_executing_ = false;
    gate_end_requested_ = false;
    gate_edge_hover_satisfied_since_ = ros::Time();
  }
  if (!task_id.empty()) {
    status_.task_id = task_id;
  } else {
    status_.task_id = makeTaskId(target);
  }

  requestAdapterStop(status_.active_mode, "mode transition");
  // PAUSE/CLEAR stops the active adapter immediately.  If the gateway were
  // left on the old source while it became stale, its fallback would use the
  // hold anchor from a previous task (often the take-off point) and command a
  // return flight.  Always capture the *current* odometry first, then hold it
  // while the existing hover-duration verification runs.  The next adapter is
  // still not armed until that verification completes.
  hold_anchor_locked_for_transition_ =
      authorizeHoldAtCurrentOdomLocked("mode transition");
  status_.command_owner = CommandOwner::HOLD;
  status_.phase = PlannerPhase::HOLD_VERIFY;
  status_.reason = "hold verify for " + std::string(toString(target));
}

void PlannerSupervisor::requestAdapterStop(const PlannerMode mode,
                                           const std::string &reason) {
  if (isExplorationMode(mode)) {
    publishExplorationCommand("PAUSE " + status_.task_id);
  }
  if (mode == PlannerMode::STATE2STATE) {
    publishNavigationCommand("PAUSE " + std::to_string(status_.task_epoch));
  }
  ROS_INFO_STREAM("[planner_supervisor] request stop mode="
                  << toString(mode) << " reason=" << reason);
}

void PlannerSupervisor::resetAdapterTaskState(const PlannerMode mode) {
  if (isExplorationMode(mode)) {
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
  if (!hold_anchor_locked_for_transition_) {
    hold_anchor_locked_for_transition_ =
        authorizeHoldAtCurrentOdomLocked("stable hold");
  } else {
    gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
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
    // A gate run may be canceled before START. Once the normal verified
    // transition has completed, HOLD is again this runtime's command owner.
    gateway_.setPublishingEnabled(true);
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
    if (!serial_handover_) {
      gateway_.setPublishingEnabled(true);
    }
    // The composed runtime keeps the shared map alive, but state2state must
    // not leave the exploration FSM's global topology/visualization timer
    // running on the same callback queue as its command stream.
    publishExplorationCommand("PAUSE");
    // Bind subsequent lifecycle observations to the new supervisor epoch.
    navigation_status_epoch_ = status_.task_epoch;
    // Keep the last observed monotonic goal sequence.  Resetting it would
    // make a delayed terminal status from the previous ARM look newer than a
    // freshly dispatched goal.
    navigation_goal_sequence_before_dispatch_ = navigation_goal_sequence_;
    navigation_goal_dispatch_pending_ = false;
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

  if (mode == PlannerMode::GATE) {
    if (serial_handover_) {
      status_.phase = PlannerPhase::FAILED;
      status_.reason =
          "cannot activate gate while serial_handover is enabled";
      return;
    }

    // Stop both internal task adapters before the command bus is released.
    // GlobalMapRuntime deliberately remains untouched: it continues fusing
    // cloud/odom and updating the persistent global topology while gate owns
    // the vehicle.
    exploration_start_pending_ = false;
    serial_state2state_ready_ = false;
    serial_handover_pending_ = false;
    publishExplorationCommand("PAUSE " + status_.task_id);
    if (navigation_enabled_) {
      publishNavigationCommand("CLEAR " + std::to_string(status_.task_epoch));
      publishNavigationTaskMode("hold");
    }

    // GATE is never an alias for HOLD: HOLD emits a fixed PositionCommand.
    // Set both safeguards before exposing gate readiness. The command gateway
    // policy also suppresses output for CommandOwner::GATE in case this flag
    // is accidentally re-enabled elsewhere.
    gateway_.setAuthorizedOwner(CommandOwner::GATE, status_.task_epoch);
    gateway_.setPublishingEnabled(false);
    status_.phase = PlannerPhase::WAITING_INPUT;
    status_.mode_state = ModeState::GATE_WAIT_START;
    status_.stable_hover = true;
    status_.ready_for_new_task = true;
    status_.command_owner = CommandOwner::GATE;
    status_.task_result = PlannerTaskResult::NONE;
    status_.reason =
        reason + "; gate command handover ready; waiting for START on " +
        gate_status_topic_;
    ROS_INFO_STREAM("[planner_supervisor] gate ready: output command gateway "
                    "suppressed, global map/topology continue");
    return;
  }

  if (isExplorationMode(mode)) {
    if (!exploration_enabled_) {
      status_.phase = PlannerPhase::FAILED;
      status_.reason = "cannot activate exploration without exploration_node";
      return;
    }
    // Parallel mode: keep navigation fsm alive but disarmed. Serial mode:
    // exploration stack is owned by handover helper / launch.
    exploration_start_pending_ = false;
    // This command is accepted only while the exploration FSM is idle or
    // paused, which is exactly the state after the verified handover above.
    // Keep coverage exploration available as the legacy sibling mode.
    publishExplorationCommand(
        std::string("MODE ") +
        (isTargetExplorationMode(mode) ? "target" : "coverage"));
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
    status_.task_id = makeTaskId(status_.active_mode);
  }
  exploration_start_pending_ = true;
  last_exploration_start_pub_ = ros::Time::now();
  publishExplorationTaskRequestLocked(true);
  publishExplorationCommand("START " + status_.task_id);
  ROS_INFO_STREAM("[planner_supervisor] START exploration task_id="
                  << status_.task_id << " reason=" << reason);
}

void PlannerSupervisor::publishExplorationTaskRequestLocked(const bool start) {
  if (!exploration_task_request_pub_) {
    return;
  }
  general_planner::ExplorationTaskRequest request;
  request.header.stamp = ros::Time::now();
  request.task_epoch = status_.task_epoch;
  request.task_id = status_.task_id;
  request.mission_mode = isTargetExplorationMode(status_.active_mode)
                             ? general_planner::ExplorationTaskRequest::MODE_TARGET
                             : general_planner::ExplorationTaskRequest::MODE_COVERAGE;
  request.has_target = isTargetExplorationMode(status_.active_mode) &&
                       have_exploration_target_goal_;
  if (request.has_target) {
    request.target = exploration_target_goal_;
  }
  request.start = start;
  exploration_task_request_pub_.publish(request);
}

bool PlannerSupervisor::acceptNavigationGoalLocked(
    const geometry_msgs::PoseStamped &msg, const bool preserve_message_height) {
  // A state2state goal replaces the current goal within the same navigation
  // task.  ready_for_new_task only governs starting a distinct task; it must
  // not block a rolling replan while this task is executing.
  if (status_.active_mode != PlannerMode::STATE2STATE || transition_active_) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_supervisor] drop navigation goal: inactive "
                      "or transitioning (mode=%s transition=%d)",
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
  status_.reason = preserve_message_height ? "navigation 3d goal accepted"
                                           : "navigation goal accepted";
  navigation_goal_sequence_before_dispatch_ =
      navigation_status_epoch_ == status_.task_epoch
          ? navigation_goal_sequence_ : 0;
  navigation_goal_dispatch_pending_ = true;
  gateway_.setAuthorizedOwner(CommandOwner::STATE2STATE, status_.task_epoch);
  if (preserve_message_height) {
    navigation_goal_3d_pub_.publish(msg);
  } else {
    navigation_goal_pub_.publish(msg);
  }
  ROS_INFO_STREAM("[planner_supervisor] navigation "
                  << (preserve_message_height ? "3d " : "")
                  << "goal accepted epoch="
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

void PlannerSupervisor::navigationGoal3DCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  acceptNavigationGoalLocked(*msg, true);
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
  if (isExplorationMode(status_.active_mode)) {
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
  if (!boot_complete_ || !isExplorationMode(status_.active_mode) ||
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

  if (isTargetExplorationMode(status_.active_mode) &&
      !forwardExplorationTargetGoalLocked(msg, "exploration trigger")) {
    return false;
  }

  // RViz 2D Nav Goal (or /planner/exploration/trigger) starts exploration.
  status_.task_result = PlannerTaskResult::NONE;
  terminal_exploration_task_id_.clear();
  terminal_exploration_result_ = PlannerTaskResult::NONE;
  exploration_terminal_hold_locked_ = false;
  status_.phase = PlannerPhase::PLANNING;
  status_.mode_state = ModeState::EXP_PLAN_TRAJ;
  status_.ready_for_new_task = false;
  status_.stable_hover = false;
  status_.command_owner = CommandOwner::HOLD;
  status_.reason = isTargetExplorationMode(status_.active_mode)
                       ? "target exploration goal accepted; starting"
                       : "exploration trigger accepted; starting";
  // A new trigger is a new task even when exploration mode itself did not
  // change.  This makes task ids monotonic and prevents a delayed terminal
  // status from the preceding exploration from affecting the new one.
  ++status_.task_epoch;
  status_.task_id = makeTaskId(status_.active_mode);
  gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
  requestExplorationStartLocked("rviz/manual trigger");
  return true;
}

bool PlannerSupervisor::forwardExplorationTargetGoalLocked(
    const geometry_msgs::PoseStamped &msg, const std::string &source) {
  if (!isTargetExplorationMode(status_.active_mode) || transition_active_) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_supervisor] drop target exploration goal: "
                      "mode=%s transition=%d",
                      toString(status_.active_mode), transition_active_);
    return false;
  }
  exploration_target_goal_pub_.publish(msg);
  exploration_target_goal_ = msg;
  have_exploration_target_goal_ = true;
  publishExplorationTaskRequestLocked(false);
  ROS_INFO_STREAM("[planner_supervisor] target exploration goal from "
                  << source << " p=(" << msg.pose.position.x << ","
                  << msg.pose.position.y << "," << msg.pose.position.z
                  << ")");
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

void PlannerSupervisor::explorationTargetGoalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  forwardExplorationTargetGoalLocked(*msg, "dedicated target topic");
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
  if (isExplorationMode(status_.active_mode) &&
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
  if (isExplorationMode(status_.active_mode) && !transition_active_) {
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
    } else if (state == "SUCCEEDED") {
      exploration_start_pending_ = false;
      // FastExplorationFSM publishes this terminal status continuously at its
      // FSM rate.  Only the first terminal notification for this task starts
      // the handover-to-HOLD transition; subsequent messages must preserve
      // STABLE_HOLD/ready_for_new_task.
      if (!task_id.empty() && task_id == terminal_exploration_task_id_ &&
          terminal_exploration_result_ == PlannerTaskResult::SUCCEEDED) {
        return;
      }
      if (!task_id.empty()) {
        terminal_exploration_task_id_ = task_id;
        terminal_exploration_result_ = PlannerTaskResult::SUCCEEDED;
      }
      // Exploration finished but remains commandable after stable hold.
      if (!transition_active_) {
        transition_active_ = true;
        transition_target_ = status_.active_mode;
        status_.phase = PlannerPhase::BRAKING;
        status_.ready_for_new_task = false;
        status_.stable_hover = false;
        status_.task_result = PlannerTaskResult::SUCCEEDED;
        status_.reason = "exploration finished, verifying hover";
        hold_anchor_locked_for_transition_ = false;
        hover_satisfied_since_ = ros::Time();
        // Install the terminal pose in the same callback that revokes the
        // exploration source.  Waiting for the next supervisor timer leaves
        // a window in which the gateway can emit the previous task anchor.
        hold_anchor_locked_for_transition_ =
            authorizeHoldAtCurrentOdomLocked("exploration succeeded");
      }
    } else if (state == "BLOCKED") {
      exploration_start_pending_ = false;
      if (!task_id.empty() && task_id == terminal_exploration_task_id_ &&
          terminal_exploration_result_ == PlannerTaskResult::BLOCKED) {
        return;
      }
      terminal_exploration_task_id_ = task_id;
      terminal_exploration_result_ = PlannerTaskResult::BLOCKED;
      status_.phase = PlannerPhase::STABLE_HOLD;
      status_.task_result = PlannerTaskResult::BLOCKED;
      status_.ready_for_new_task = true;
      status_.stable_hover = true;
      status_.command_owner = CommandOwner::HOLD;
      status_.mode_state = ModeState::EXP_PAUSED;
      status_.reason = "target exploration blocked; topo graph retained";
      if (!exploration_terminal_hold_locked_) {
        exploration_terminal_hold_locked_ =
            authorizeHoldAtCurrentOdomLocked("exploration blocked");
      } else {
        gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
      }
    } else if (state == "FAILED") {
      exploration_start_pending_ = false;
      if (!task_id.empty() && task_id == terminal_exploration_task_id_ &&
          terminal_exploration_result_ == PlannerTaskResult::FAILED) {
        return;
      }
      terminal_exploration_task_id_ = task_id;
      terminal_exploration_result_ = PlannerTaskResult::FAILED;
      status_.phase = PlannerPhase::FAILED;
      status_.task_result = PlannerTaskResult::FAILED;
      status_.ready_for_new_task = false;
      status_.stable_hover = false;
      status_.command_owner = CommandOwner::HOLD;
      status_.reason = "exploration failed";
      if (!exploration_terminal_hold_locked_) {
        exploration_terminal_hold_locked_ =
            authorizeHoldAtCurrentOdomLocked("exploration failed");
      } else {
        gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
      }
    }
  }
}

void PlannerSupervisor::navigationStatusCallback(
    const std_msgs::StringConstPtr &msg) {
  if (!msg) {
    return;
  }
  NavigationAdapterStatus adapter_status;
  if (!parseNavigationAdapterStatus(msg->data, adapter_status)) {
    return;
  }
  const std::string &state = adapter_status.state;
  std::lock_guard<std::mutex> lock(mutex_);
  navigation_status_ = state;
  if (serial_handover_) {
    return;
  }
  if (adapter_status.has_lifecycle) {
    // PAUSE/CLEAR can still produce a latched status from the previous task.
    // It must never complete or unlock the current supervisor epoch.
    if (adapter_status.task_epoch != status_.task_epoch) {
      return;
    }
    navigation_status_epoch_ = adapter_status.task_epoch;
    navigation_goal_sequence_ = adapter_status.goal_sequence;
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
      const bool completed_dispatched_goal =
          adapter_status.has_lifecycle &&
          navigation_goal_dispatch_pending_ &&
          adapter_status.goal_sequence >
              navigation_goal_sequence_before_dispatch_ &&
          !adapter_status.goal_active;
      if (completed_dispatched_goal) {
        // Reuse the normal transition path so stale commands are cleared and
        // the next navigation task is accepted only after a verified hover.
        navigation_goal_dispatch_pending_ = false;
        beginTransition(PlannerMode::STATE2STATE,
                        status_.accepted_request_id,
                        "",
                        "navigation goal completed");
        status_.task_result = PlannerTaskResult::SUCCEEDED;
        return;
      }
      // The FSM has accepted this goal but is still in the one-tick
      // WAIT_GOAL-to-GENERATE_TRAJ window.  The old implementation mistook
      // this for a terminal wait; do not expose readiness yet.
      if (navigation_goal_dispatch_pending_) {
        return;
      }
      status_.phase = PlannerPhase::WAITING_INPUT;
      status_.ready_for_new_task = true;
      status_.stable_hover = hoverConditionMetLocked();
      status_.command_owner = CommandOwner::HOLD;
      if (!gateway_.hasHoldAnchor()) {
        authorizeHoldAtCurrentOdomLocked("navigation wait goal");
      } else {
        gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
      }
      status_.reason = "navigation wait goal";
      status_.task_result = PlannerTaskResult::SUCCEEDED;
    }
  }
}

void PlannerSupervisor::gateStatusCallback(const std_msgs::StringConstPtr &msg) {
  if (!msg) {
    return;
  }

  std::string command = msg->data;
  const auto first = command.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return;
  }
  const auto last = command.find_last_not_of(" \t\r\n");
  command = command.substr(first, last - first + 1);
  std::transform(command.begin(), command.end(), command.begin(),
                 [](const unsigned char c) {
                   return static_cast<char>(std::toupper(c));
                 });

  const bool start = command == "START" || command == "BEGIN" ||
                     command == "RUNNING";
  const bool end = command == "END" || command == "DONE" ||
                   command == "FINISHED" || command == "COMPLETE";
  if (!start && !end) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[planner_supervisor] ignore gate status='" << msg->data
                                                           << "' (need START or END)");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // START may be delivered just before the state2state->gate brake has
  // finished. Latch it, then activate it only after activateMode(GATE) has
  // suppressed our command output and timerCallback has re-verified hover.
  const bool gate_requested =
      status_.active_mode == PlannerMode::GATE ||
      status_.requested_mode == PlannerMode::GATE;
  if (!gate_requested) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_supervisor] ignore gate %s: gate mode inactive",
                      command.c_str());
    return;
  }

  if (start) {
    if (status_.active_mode != PlannerMode::GATE || transition_active_) {
      gate_start_requested_ = true;
      ROS_INFO("[planner_supervisor] gate START latched pending safe handover");
      return;
    }
    if (gate_executing_) {
      return;
    }
    if (status_.mode_state != ModeState::GATE_WAIT_START) {
      ROS_WARN("[planner_supervisor] ignore gate START: rearm gate mode first");
      return;
    }
    gate_start_requested_ = true;
    gate_edge_hover_satisfied_since_ = ros::Time();
    status_.phase = PlannerPhase::HOLD_VERIFY;
    status_.mode_state = ModeState::GATE_WAIT_START;
    status_.stable_hover = false;
    status_.ready_for_new_task = false;
    status_.command_owner = CommandOwner::GATE;
    status_.reason = "gate START received; verifying stable hover";
    return;
  }

  if (status_.active_mode != PlannerMode::GATE || !gate_executing_ ||
      transition_active_) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_supervisor] ignore gate END: gate is not executing");
    return;
  }
  if (gate_end_requested_) {
    return;
  }
  // Do not publish a hold command here. The external planner owns the brake
  // through the end of the verification window; only after stable odometry is
  // confirmed do we atomically reclaim the bus with a current-pose hold.
  gate_end_requested_ = true;
  gate_edge_hover_satisfied_since_ = ros::Time();
  status_.phase = PlannerPhase::HOLD_VERIFY;
  status_.mode_state = ModeState::GATE_END_VERIFY;
  status_.stable_hover = false;
  status_.ready_for_new_task = false;
  status_.command_owner = CommandOwner::GATE;
  status_.reason = "gate END received; verifying stable hover";
}

void PlannerSupervisor::completeGateExitLocked() {
  gate_end_requested_ = false;
  gate_start_requested_ = false;
  gate_executing_ = false;
  gate_edge_hover_satisfied_since_ = ros::Time();

  // Keep the first command after the gate at the actual final odometry pose,
  // rather than a transition anchor from before traversing the slit.
  authorizeHoldAtCurrentOdomLocked("gate complete");
  gateway_.setPublishingEnabled(true);
  status_.phase = PlannerPhase::STABLE_HOLD;
  status_.mode_state = ModeState::GATE_COMPLETE;
  status_.stable_hover = true;
  status_.ready_for_new_task = true;
  status_.command_owner = CommandOwner::HOLD;
  status_.task_result = PlannerTaskResult::SUCCEEDED;
  status_.reason =
      "gate complete and stable; ready for state2state or exploration";
  ROS_INFO("[planner_supervisor] gate complete; command gateway reclaimed at final odometry");
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

bool PlannerSupervisor::authorizeHoldAtCurrentOdomLocked(
    const std::string &reason) {
  const bool fresh_odom =
      have_odom_ && !last_odom_time_.isZero() &&
      (ros::Time::now() - last_odom_time_).toSec() <= max_odom_age_;
  if (!fresh_odom) {
    // Do not preserve a hold point from a previous task when the current
    // vehicle pose is unknown.  No position command is safer than a command
    // that can pull the vehicle tens of metres backwards.
    gateway_.clearHoldAnchor();
    gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
    ROS_ERROR_STREAM("[planner_supervisor] cannot authorize HOLD for "
                     << reason << ": current odometry is unavailable");
    return false;
  }

  const auto &pose = odom_.pose.pose;
  gateway_.setHoldAnchorAndAuthorize(
      pose.position.x, pose.position.y, pose.position.z,
      yawFromQuaternion(pose.orientation), status_.task_epoch);
  return true;
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
  oss << runtime_session_id_ << ":" << status_.task_epoch << ":"
      << toString(mode);
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
    const PlannerMode exploration_mode =
        isExplorationMode(status_.requested_mode)
            ? status_.requested_mode
            : PlannerMode::EXPLORATION;
    status_.active_mode = exploration_mode;
    status_.requested_mode = exploration_mode;
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
  const GlobalMapStatus map_status = map_status_provider_
      ? map_status_provider_() : GlobalMapStatus{};
  PlannerMode boot_activate_mode = PlannerMode::HOLD;
  bool do_boot_activate = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool supervisor_odom_valid =
        have_odom_ &&
        (ros::Time::now() - last_odom_time_).toSec() <= max_odom_age_;
    if (map_status_provider_) {
      status_.odom_valid = supervisor_odom_valid && map_status.odom_valid;
      status_.world_epoch = map_status.world_epoch;
      status_.map_revision = map_status.map_revision;
      status_.topo_revision = map_status.topo_revision;
      status_.map_ready = map_status.map_ready;
      status_.topology_ready = map_status.topology_ready;
    } else {
      // Legacy non-composed launch compatibility only. M2 always supplies the
      // provider above, so no map field is inferred from odometry there.
      status_.odom_valid = supervisor_odom_valid;
      status_.map_ready = supervisor_odom_valid;
      status_.topology_ready = supervisor_odom_valid;
    }

    if (!boot_complete_) {
      if (!status_.odom_valid) {
        status_.phase = PlannerPhase::BOOT;
        status_.reason = "waiting for odometry";
      } else if (!hoverConditionMetLocked()) {
        status_.phase = PlannerPhase::HOLD_VERIFY;
        status_.reason = "boot hover verify";
        if (!hold_anchor_locked_for_transition_) {
          hold_anchor_locked_for_transition_ =
              authorizeHoldAtCurrentOdomLocked("boot hover verify");
        } else {
          gateway_.setAuthorizedOwner(CommandOwner::HOLD, status_.task_epoch);
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
        hold_anchor_locked_for_transition_ =
            authorizeHoldAtCurrentOdomLocked("transition hover verify");
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
            if (isExplorationMode(target) &&
                result == PlannerTaskResult::SUCCEEDED &&
                isExplorationMode(status_.active_mode)) {
              // Finished exploration stays in exploration mode but idle.
              transition_active_ = false;
              enterStableHold("exploration idle after finish", result);
              status_.active_mode = target;
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
      if (status_.active_mode == PlannerMode::GATE) {
        // No branch in this block is allowed to authorize HOLD, navigation or
        // exploration output while gate is active. The external planner owns
        // the vehicle until END has remained inside the same hover threshold
        // used for every other runtime handover.
        if (gate_end_requested_) {
          status_.phase = PlannerPhase::HOLD_VERIFY;
          status_.mode_state = ModeState::GATE_END_VERIFY;
          status_.stable_hover = false;
          status_.ready_for_new_task = false;
          status_.command_owner = CommandOwner::GATE;
          if (!hoverConditionMetLocked()) {
            gate_edge_hover_satisfied_since_ = ros::Time();
          } else {
            if (gate_edge_hover_satisfied_since_.isZero()) {
              gate_edge_hover_satisfied_since_ = ros::Time::now();
            }
            if ((ros::Time::now() - gate_edge_hover_satisfied_since_).toSec() >=
                hover_hold_duration_) {
              completeGateExitLocked();
            }
          }
        } else if (gate_start_requested_ && !gate_executing_) {
          status_.phase = PlannerPhase::HOLD_VERIFY;
          status_.mode_state = ModeState::GATE_WAIT_START;
          status_.stable_hover = false;
          status_.ready_for_new_task = false;
          status_.command_owner = CommandOwner::GATE;
          if (!hoverConditionMetLocked()) {
            gate_edge_hover_satisfied_since_ = ros::Time();
          } else {
            if (gate_edge_hover_satisfied_since_.isZero()) {
              gate_edge_hover_satisfied_since_ = ros::Time::now();
            }
            if ((ros::Time::now() - gate_edge_hover_satisfied_since_).toSec() >=
                hover_hold_duration_) {
              gate_executing_ = true;
              gate_start_requested_ = false;
              gate_edge_hover_satisfied_since_ = ros::Time();
              status_.phase = PlannerPhase::EXECUTING;
              status_.mode_state = ModeState::GATE_EXECUTING;
              status_.stable_hover = false;
              status_.ready_for_new_task = false;
              status_.command_owner = CommandOwner::GATE;
              status_.reason = "gate START accepted; external planner owns command bus";
              ROS_INFO("[planner_supervisor] gate START accepted after stable hover");
            }
          }
        } else if (gate_executing_) {
          status_.phase = PlannerPhase::EXECUTING;
          status_.mode_state = ModeState::GATE_EXECUTING;
          status_.stable_hover = false;
          status_.ready_for_new_task = false;
          status_.command_owner = CommandOwner::GATE;
        } else {
          status_.phase = PlannerPhase::WAITING_INPUT;
          status_.mode_state = ModeState::GATE_WAIT_START;
          status_.stable_hover = hoverConditionMetLocked();
          status_.ready_for_new_task = status_.stable_hover;
          status_.command_owner = CommandOwner::GATE;
        }
      } else {
        updatePhaseFromActiveModeLocked();
      }
      if (exploration_start_pending_ &&
          isExplorationMode(status_.active_mode) &&
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
