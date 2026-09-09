#pragma once

#include <general_core/planner_runtime/planner_status.hpp>

namespace general_planner::planner_runtime {

enum class TargetExplorationResult : std::uint8_t {
  RUNNING = 0, SUCCEEDED = 1, FAILED = 2, READY = 3
};

struct TargetExplorationSummary {
  TargetExplorationResult result;
  std::string reason;
};

// Task-level dispatch readiness, not a trajectory execution certificate.
// A blocked task reports FAILED at least once before advertising retry READY.
class TargetExplorationStatusProjector {
public:
  TargetExplorationSummary update(const PlannerStatusData &s,
                                  bool busy, bool hover_now) {
    if (s.task_epoch != epoch_ || s.task_id != task_id_) {
      epoch_ = s.task_epoch;
      task_id_ = s.task_id;
      blocked_reported_ = false;
    }
    if (s.active_mode != PlannerMode::TARGET_EXPLORATION) {
      blocked_reported_ = false;
      return {TargetExplorationResult::FAILED, "target exploration inactive"};
    }
    const bool blocked = s.task_result == PlannerTaskResult::BLOCKED;
    if (s.phase == PlannerPhase::FAILED || s.phase == PlannerPhase::EMERGENCY ||
        s.task_result == PlannerTaskResult::FAILED) {
      return {TargetExplorationResult::FAILED, "recovery required: " + s.reason};
    }
    if (blocked && !blocked_reported_) {
      blocked_reported_ = true;
      return {TargetExplorationResult::FAILED, "target blocked: " + s.reason};
    }
    if (busy) {
      return {TargetExplorationResult::RUNNING, "handover pending: " + s.reason};
    }
    if (!s.odom_valid || !s.map_ready) {
      return {TargetExplorationResult::FAILED,
              !s.odom_valid ? "waiting for odometry" : "waiting for map"};
    }
    const bool idle_phase = s.phase == PlannerPhase::WAITING_INPUT ||
                            s.phase == PlannerPhase::STABLE_HOLD;
    const bool ready = idle_phase && s.ready_for_new_task && s.stable_hover &&
                       hover_now && s.command_owner == CommandOwner::HOLD;
    if (ready) {
      if (s.task_result == PlannerTaskResult::SUCCEEDED) {
        return {TargetExplorationResult::SUCCEEDED, "target reached; next task allowed"};
      }
      return {TargetExplorationResult::READY,
              blocked ? "target blocked; retry or another target allowed"
                      : "ready for a target task"};
    }
    if (blocked) {
      return {TargetExplorationResult::FAILED, "target blocked; waiting for recovery"};
    }
    return {TargetExplorationResult::RUNNING, s.reason};
  }

private:
  std::uint64_t epoch_{0};
  std::string task_id_;
  bool blocked_reported_{false};
};
} // namespace general_planner::planner_runtime
