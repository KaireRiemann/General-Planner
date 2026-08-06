#pragma once

#include <cctype>
#include <cstdint>
#include <string>

namespace general_planner::planner_runtime {

enum class PlannerMode : std::uint8_t {
  HOLD = 0,
  STATE2STATE = 1,
  EXPLORATION = 2,
  EMERGENCY_STOP = 3
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
  CANCELED = 3
};

enum class CommandOwner : std::uint8_t {
  HOLD = 0,
  STATE2STATE = 1,
  EXPLORATION = 2
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
  HOLD_IDLE = 20
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

inline const char *toString(const PlannerMode mode) {
  switch (mode) {
  case PlannerMode::STATE2STATE:
    return "state2state";
  case PlannerMode::EXPLORATION:
    return "exploration";
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
  case ModeState::HOLD_IDLE:
    return "hold_idle";
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
  if (text == "emergency_stop" || text == "estop" || text == "emergency") {
    mode = PlannerMode::EMERGENCY_STOP;
    return true;
  }
  return false;
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

inline ModeState modeStateFromExplorationString(const std::string &state) {
  if (state == "INIT") {
    return ModeState::EXP_INIT;
  }
  if (state == "WAIT_TRIGGER" || state == "IDLE") {
    return ModeState::EXP_WAIT_TRIGGER;
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
  case PlannerMode::EXPLORATION:
    return CommandOwner::EXPLORATION;
  case PlannerMode::HOLD:
  case PlannerMode::EMERGENCY_STOP:
  default:
    return CommandOwner::HOLD;
  }
}

} // namespace general_planner::planner_runtime
