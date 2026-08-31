#pragma once

#include <cctype>
#include <cstdint>
#include <sstream>
#include <string>

namespace general_planner::planner_runtime {

enum class PlannerMode : std::uint8_t {
  HOLD = 0,
  STATE2STATE = 1,
  EXPLORATION = 2,
  EMERGENCY_STOP = 3,
  // Destination-directed exploration is an explicit runtime task mode.  It
  // uses the same HighSpeedExp adapter and command owner as coverage
  // exploration, but selects safe frontier bridges toward one remote goal.
  TARGET_EXPLORATION = 4,
  // Map-only handover mode.  The runtime keeps the world model/topology
  // alive, but never publishes a PositionCommand: an external gate planner
  // owns the vehicle until it reports END and the vehicle is again stable.
  GATE = 5,
  // Continuous target tracking is executed by the composed FsmRos1 adapter,
  // but is intentionally a distinct runtime owner from click navigation.
  TRACKING = 6
};

enum class PlannerPhase : std::uint8_t {
  BOOT = 0,
  WAITING_INPUT = 1,
  PLANNING = 2,
  EXECUTING = 3,
  BRAKING = 4,
  HOLD_VERIFY = 5,
  STABLE_HOLD = 6,
  FAILED = 7,
  EMERGENCY = 8
};

enum class PlannerTaskResult : std::uint8_t {
  NONE = 0,
  SUCCEEDED = 1,
  FAILED = 2,
  CANCELED = 3,
  // A target-directed task has exhausted all currently safe frontier/topology
  // bridges. This is not coverage completion and not an internal failure:
  // retain the global topo graph for a later explicit navigation task.
  BLOCKED = 4
};

enum class CommandOwner : std::uint8_t {
  HOLD = 0,
  STATE2STATE = 1,
  EXPLORATION = 2,
  // This is deliberately distinct from HOLD.  A HOLD owner publishes a
  // position hold, while GATE suppresses this runtime's command output.
  GATE = 3,
  // Tracking emits on the navigation adapter's input topic, but has an
  // independent authorization identity so status and source timeouts cannot
  // be mistaken for state2state navigation.
  TRACKING = 4
};

enum class ModeState : std::uint16_t {
  UNKNOWN = 0,
  S2S_WAIT_GOAL = 1,
  S2S_GENERATE_TRAJ = 2,
  S2S_FOLLOW_TRAJ = 3,
  S2S_YAWING = 4,
  S2S_EMER_STOP = 5,
  EXP_INIT = 10,
  EXP_WAIT_TRIGGER = 11,
  EXP_PLAN_TRAJ = 12,
  EXP_EXEC_TRAJ = 13,
  EXP_REORIENT = 14,
  EXP_CAUTION = 15,
  EXP_PAUSING = 16,
  EXP_PAUSED = 17,
  EXP_FINISH = 18,
  EXP_WAIT_TARGET = 19,
  HOLD_IDLE = 20,
  GATE_WAIT_START = 30,
  GATE_EXECUTING = 31,
  GATE_END_VERIFY = 32,
  GATE_COMPLETE = 33,
  TRACK_WAIT_TARGET = 40,
  TRACK_GENERATE_TRAJ = 41,
  TRACK_FOLLOW_TRAJ = 42,
  TRACK_STATIC = 43,
  TRACK_HOLD = 44
};

struct PlannerStatusData {
  std::uint64_t transition_id{0};
  std::uint64_t task_epoch{0};
  std::uint64_t world_epoch{0};
  std::uint64_t map_revision{0};
  std::uint64_t topo_revision{0};
  std::uint64_t accepted_request_id{0};

  std::string task_id;
  PlannerMode active_mode{PlannerMode::HOLD};
  PlannerMode requested_mode{PlannerMode::HOLD};
  PlannerPhase phase{PlannerPhase::BOOT};
  ModeState mode_state{ModeState::UNKNOWN};
  PlannerTaskResult task_result{PlannerTaskResult::NONE};

  bool stable_hover{false};
  bool ready_for_new_task{false};
  bool odom_valid{false};
  bool map_ready{false};
  bool topology_ready{false};
  CommandOwner command_owner{CommandOwner::HOLD};
  float speed_mps{0.0f};
  float yaw_rate_rps{0.0f};
  std::string reason;
};

// Wire format published by the state2state FSM on
// /planning/navigation/status:
//   <fsm_state> <task_epoch> <goal_sequence> <ACTIVE|IDLE> <READY|BUSY> [stage=<name>]
//
// The first two fields are retained for compatibility with the original
// status topic.  The lifecycle fields remove the ambiguity of WAIT_GOAL:
// it may mean either "armed and waiting for the first goal" or "the current
// goal has finished".  Supervisor handover must never use the former as a
// completion event. The optional final field is a worker-quiescence fence:
// after a timeout the supervisor must not ARM another state2state task until
// it has observed READY from the adapter. The optional stage field is
// diagnostic-only and identifies the last cooperative cancellation boundary.
struct NavigationAdapterStatus {
  std::string state;
  std::uint64_t task_epoch{0};
  std::uint64_t goal_sequence{0};
  bool goal_active{false};
  bool has_lifecycle{false};
  bool planning_worker_ready{false};
  bool has_worker_readiness{false};
  std::string planning_stage{"unknown"};
  bool has_planning_stage{false};
};

inline bool parseNavigationAdapterStatus(const std::string &text,
                                         NavigationAdapterStatus &status) {
  status = NavigationAdapterStatus{};
  std::istringstream stream(text);
  if (!(stream >> status.state)) {
    return false;
  }
  // A legacy state-only publisher is accepted as best effort, but it never
  // supplies enough information to declare a navigation task complete.
  if (!(stream >> status.task_epoch)) {
    return true;
  }
  std::string lifecycle;
  if (!(stream >> status.goal_sequence >> lifecycle)) {
    return true;
  }
  if (lifecycle == "ACTIVE") {
    status.goal_active = true;
    status.has_lifecycle = true;
  } else if (lifecycle == "IDLE") {
    status.goal_active = false;
    status.has_lifecycle = true;
  }
  std::string worker_state;
  if (stream >> worker_state) {
    if (worker_state == "READY") {
      status.planning_worker_ready = true;
      status.has_worker_readiness = true;
    } else if (worker_state == "BUSY") {
      status.planning_worker_ready = false;
      status.has_worker_readiness = true;
    }
  }
  std::string stage;
  if (stream >> stage) {
    constexpr const char kStagePrefix[] = "stage=";
    if (stage.rfind(kStagePrefix, 0) == 0 &&
        stage.size() > sizeof(kStagePrefix) - 1) {
      status.planning_stage = stage.substr(sizeof(kStagePrefix) - 1);
      status.has_planning_stage = true;
    }
  }
  return true;
}

inline const char *toString(const PlannerMode mode) {
  switch (mode) {
  case PlannerMode::STATE2STATE:
    return "state2state";
  case PlannerMode::EXPLORATION:
    return "exploration";
  case PlannerMode::TARGET_EXPLORATION:
    return "target_exploration";
  case PlannerMode::GATE:
    return "gate";
  case PlannerMode::TRACKING:
    return "tracking";
  case PlannerMode::EMERGENCY_STOP:
    return "emergency_stop";
  case PlannerMode::HOLD:
  default:
    return "hold";
  }
}

inline const char *toString(const PlannerPhase phase) {
  switch (phase) {
  case PlannerPhase::WAITING_INPUT:
    return "waiting_input";
  case PlannerPhase::PLANNING:
    return "planning";
  case PlannerPhase::EXECUTING:
    return "executing";
  case PlannerPhase::BRAKING:
    return "braking";
  case PlannerPhase::HOLD_VERIFY:
    return "hold_verify";
  case PlannerPhase::STABLE_HOLD:
    return "stable_hold";
  case PlannerPhase::FAILED:
    return "failed";
  case PlannerPhase::EMERGENCY:
    return "emergency";
  case PlannerPhase::BOOT:
  default:
    return "boot";
  }
}

inline const char *toString(const CommandOwner owner) {
  switch (owner) {
  case CommandOwner::STATE2STATE:
    return "state2state";
  case CommandOwner::EXPLORATION:
    return "exploration";
  case CommandOwner::GATE:
    return "gate";
  case CommandOwner::TRACKING:
    return "tracking";
  case CommandOwner::HOLD:
  default:
    return "hold";
  }
}

inline const char *toString(const PlannerTaskResult result) {
  switch (result) {
  case PlannerTaskResult::SUCCEEDED:
    return "succeeded";
  case PlannerTaskResult::FAILED:
    return "failed";
  case PlannerTaskResult::CANCELED:
    return "canceled";
  case PlannerTaskResult::BLOCKED:
    return "blocked";
  case PlannerTaskResult::NONE:
  default:
    return "none";
  }
}

inline const char *toString(const ModeState state) {
  switch (state) {
  case ModeState::S2S_WAIT_GOAL:
    return "s2s_wait_goal";
  case ModeState::S2S_GENERATE_TRAJ:
    return "s2s_generate_traj";
  case ModeState::S2S_FOLLOW_TRAJ:
    return "s2s_follow_traj";
  case ModeState::S2S_YAWING:
    return "s2s_yawing";
  case ModeState::S2S_EMER_STOP:
    return "s2s_emer_stop";
  case ModeState::EXP_INIT:
    return "exp_init";
  case ModeState::EXP_WAIT_TRIGGER:
    return "exp_wait_trigger";
  case ModeState::EXP_PLAN_TRAJ:
    return "exp_plan_traj";
  case ModeState::EXP_EXEC_TRAJ:
    return "exp_exec_traj";
  case ModeState::EXP_REORIENT:
    return "exp_reorient";
  case ModeState::EXP_CAUTION:
    return "exp_caution";
  case ModeState::EXP_PAUSING:
    return "exp_pausing";
  case ModeState::EXP_PAUSED:
    return "exp_paused";
  case ModeState::EXP_FINISH:
    return "exp_finish";
  case ModeState::EXP_WAIT_TARGET:
    return "exp_wait_target";
  case ModeState::HOLD_IDLE:
    return "hold_idle";
  case ModeState::GATE_WAIT_START:
    return "gate_wait_start";
  case ModeState::GATE_EXECUTING:
    return "gate_executing";
  case ModeState::GATE_END_VERIFY:
    return "gate_end_verify";
  case ModeState::GATE_COMPLETE:
    return "gate_complete";
  case ModeState::TRACK_WAIT_TARGET:
    return "track_wait_target";
  case ModeState::TRACK_GENERATE_TRAJ:
    return "track_generate_traj";
  case ModeState::TRACK_FOLLOW_TRAJ:
    return "track_follow_traj";
  case ModeState::TRACK_STATIC:
    return "track_static";
  case ModeState::TRACK_HOLD:
    return "track_hold";
  case ModeState::UNKNOWN:
  default:
    return "unknown";
  }
}

inline bool parsePlannerMode(std::string text, PlannerMode &mode) {
  for (char &c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (text == "hold" || text == "wait" || text == "idle") {
    mode = PlannerMode::HOLD;
    return true;
  }
  if (text == "state2state" || text == "navigation" || text == "nav" ||
      text == "s2s") {
    mode = PlannerMode::STATE2STATE;
    return true;
  }
  if (text == "exploration" || text == "explore") {
    mode = PlannerMode::EXPLORATION;
    return true;
  }
  if (text == "target_exploration" || text == "target-exploration" ||
      text == "targetexploration" || text == "target_explore" ||
      text == "target-explore") {
    mode = PlannerMode::TARGET_EXPLORATION;
    return true;
  }
  if (text == "gate") {
    mode = PlannerMode::GATE;
    return true;
  }
  if (text == "tracking" || text == "track") {
    mode = PlannerMode::TRACKING;
    return true;
  }
  if (text == "emergency_stop" || text == "estop" || text == "emergency") {
    mode = PlannerMode::EMERGENCY_STOP;
    return true;
  }
  return false;
}

inline bool isExplorationMode(const PlannerMode mode) {
  return mode == PlannerMode::EXPLORATION ||
         mode == PlannerMode::TARGET_EXPLORATION;
}

inline bool isTargetExplorationMode(const PlannerMode mode) {
  return mode == PlannerMode::TARGET_EXPLORATION;
}

inline bool isNavigationAdapterMode(const PlannerMode mode) {
  return mode == PlannerMode::STATE2STATE || mode == PlannerMode::TRACKING;
}

inline ModeState modeStateFromNavigationString(const std::string &state) {
  if (state == "WAIT_GOAL") {
    return ModeState::S2S_WAIT_GOAL;
  }
  if (state == "GENERATE_TRAJ") {
    return ModeState::S2S_GENERATE_TRAJ;
  }
  if (state == "FOLLOW_TRAJ" || state == "STATIC_TRACKING" ||
      state == "HOLD_TRACKING") {
    return ModeState::S2S_FOLLOW_TRAJ;
  }
  if (state == "YAWING") {
    return ModeState::S2S_YAWING;
  }
  if (state == "EMER_STOP") {
    return ModeState::S2S_EMER_STOP;
  }
  return ModeState::UNKNOWN;
}

inline ModeState modeStateFromTrackingString(const std::string &state) {
  if (state == "WAIT_GOAL" || state == "INIT") {
    return ModeState::TRACK_WAIT_TARGET;
  }
  if (state == "GENERATE_TRAJ") {
    return ModeState::TRACK_GENERATE_TRAJ;
  }
  if (state == "FOLLOW_TRAJ") {
    return ModeState::TRACK_FOLLOW_TRAJ;
  }
  if (state == "STATIC_TRACKING") {
    return ModeState::TRACK_STATIC;
  }
  if (state == "HOLD_TRACKING") {
    return ModeState::TRACK_HOLD;
  }
  return ModeState::UNKNOWN;
}

inline ModeState modeStateFromExplorationString(const std::string &state) {
  if (state == "INIT") {
    return ModeState::EXP_INIT;
  }
  if (state == "WAIT_TRIGGER" || state == "IDLE") {
    return ModeState::EXP_WAIT_TRIGGER;
  }
  if (state == "WAITING_TARGET") {
    return ModeState::EXP_WAIT_TARGET;
  }
  if (state == "BLOCKED") {
    return ModeState::EXP_PAUSED;
  }
  if (state == "PLAN_TRAJ" || state == "RUNNING") {
    return ModeState::EXP_PLAN_TRAJ;
  }
  if (state == "EXEC_TRAJ") {
    return ModeState::EXP_EXEC_TRAJ;
  }
  if (state == "REORIENT") {
    return ModeState::EXP_REORIENT;
  }
  if (state == "CAUTION") {
    return ModeState::EXP_CAUTION;
  }
  if (state == "PAUSING") {
    return ModeState::EXP_PAUSING;
  }
  if (state == "PAUSED" || state == "SUCCEEDED") {
    return ModeState::EXP_PAUSED;
  }
  if (state == "FINISH") {
    return ModeState::EXP_FINISH;
  }
  if (state == "FAILED" || state == "LAND") {
    return ModeState::EXP_FINISH;
  }
  return ModeState::UNKNOWN;
}

inline CommandOwner ownerForMode(const PlannerMode mode) {
  switch (mode) {
  case PlannerMode::STATE2STATE:
    return CommandOwner::STATE2STATE;
  case PlannerMode::TRACKING:
    return CommandOwner::TRACKING;
  case PlannerMode::EXPLORATION:
  case PlannerMode::TARGET_EXPLORATION:
    return CommandOwner::EXPLORATION;
  case PlannerMode::GATE:
    return CommandOwner::GATE;
  case PlannerMode::HOLD:
  case PlannerMode::EMERGENCY_STOP:
  default:
    return CommandOwner::HOLD;
  }
}

} // namespace general_planner::planner_runtime
