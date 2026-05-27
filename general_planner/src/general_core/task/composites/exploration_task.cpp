#include "general_core/task/composites/exploration_task.hpp"

#include <algorithm>
#include <iostream>
#include <utility>

namespace general_planner {

ExplorationTask::ExplorationTask(GeneralPlanner::Ptr planner)
        : planner_(std::move(planner)) {}

TaskTickResult ExplorationTask::tick(const TaskContext &ctx) {
    if (!planner_) {
        return makeResult(TaskStatus::FAILED_FATAL,
                          super_utils::FAILED,
                          "exploration planner missing");
    }

    if (ctx.new_task && !new_task_consumed_) {
        planner_->resetExplorationTaskRuntime(false);
        from_rest_ = true;
        last_global_update_wt_ = -1.0;
        new_task_consumed_ = true;
        changeStage(WAIT_OBSERVATION);
    }

    const auto policy = planner_->getExplorationRuntimePolicy();

    for (int step = 0; step < 6; ++step) {
        switch (static_cast<Stage>(stage_)) {
            case WAIT_OBSERVATION: {
                if (!planner_->explorationObservationReady()) {
                    return makeResult(TaskStatus::NOT_READY,
                                      super_utils::NO_NEED,
                                      "waiting for first exploration cloud observation");
                }
                exploration::ExplorationGoal warmed_goal;
                changeStage(planner_->getLatestExplorationGoal(warmed_goal)
                            ? PLAN_LOCAL
                            : UPDATE_GLOBAL);
                continue;
            }

            case UPDATE_GLOBAL: {
                const auto update = planner_->refreshExplorationGlobalPlan();
                last_global_update_wt_ = ctx.now;
                if (!update.ready) {
                    changeStage(WAIT_OBSERVATION);
                    return makeResult(TaskStatus::NOT_READY,
                                      super_utils::NO_NEED,
                                      update.reason.empty()
                                      ? "exploration frontend not ready"
                                      : update.reason);
                }
                if (update.finished) {
                    changeStage(FINISH);
                    return makeResult(TaskStatus::FINISHED,
                                      super_utils::FINISH,
                                      update.reason.empty()
                                      ? "exploration finished"
                                      : update.reason);
                }
                changeStage(PLAN_LOCAL);
                continue;
            }

            case PLAN_LOCAL: {
                const RET_CODE ret = from_rest_
                                     ? planner_->PlanExplorationFromRest(false)
                                     : planner_->ReplanExplorationOnce(false);
                return handlePlanResult(ret, ctx);
            }

            case EXEC_LOCAL: {
                const auto exec = planner_->getExplorationExecutionStatus(ctx.now);
                if (!exec.has_observation) {
                    changeStage(WAIT_OBSERVATION);
                    return makeResult(TaskStatus::NOT_READY,
                                      super_utils::NO_NEED,
                                      "exploration observation is not ready");
                }
                if (!exec.has_active_trajectory || exec.traj_remaining <= 0.02) {
                    from_rest_ = true;
                    changeStage(PLAN_LOCAL);
                    continue;
                }
                if (exec.trajectory_unsafe) {
                    if (exec.collision_time < 0.0 ||
                        exec.collision_time <= policy.collision_replan_time) {
                        const bool truncated =
                                planner_->truncateActiveExplorationTrajectory(policy.stop_traj_time);
                        from_rest_ = true;
                        changeStage(RECOVER);
                        return makeResult(TaskStatus::RUNNING,
                                          super_utils::NEW_TRAJ,
                                          truncated
                                          ? "active exploration trajectory truncated for safety"
                                          : "active exploration trajectory unsafe; request recovery plan");
                    }
                    changeStage(PLAN_LOCAL);
                    continue;
                }

                const bool start_suppression =
                        exec.traj_elapsed < policy.replan_time_after_traj_start &&
                        exec.traj_remaining > policy.replan_time_before_traj_end;
                if (start_suppression) {
                    return makeResult(TaskStatus::KEEP_CURRENT,
                                      super_utils::NO_NEED,
                                      "executing local exploration segment");
                }

                changeStage(PLAN_LOCAL);
                continue;
            }

            case RECOVER: {
                if (!planner_->explorationObservationReady()) {
                    changeStage(WAIT_OBSERVATION);
                    return makeResult(TaskStatus::NOT_READY,
                                      super_utils::NO_NEED,
                                      "waiting for observation before exploration recovery");
                }
                from_rest_ = true;
                changeStage(PLAN_LOCAL);
                continue;
            }

            case FINISH:
            default:
                return makeResult(TaskStatus::FINISHED,
                                  super_utils::FINISH,
                                  "exploration finished");
        }
    }

    return makeResult(TaskStatus::FAILED_RECOVERABLE,
                      super_utils::FAILED,
                      "exploration task stage loop exhausted");
}

void ExplorationTask::reset() {
    CompositeTask::reset();
    from_rest_ = true;
    new_task_consumed_ = false;
    last_global_update_wt_ = -1.0;
}

std::string ExplorationTask::name() const {
    return "exploration";
}

const char *ExplorationTask::stageName(const int stage) {
    switch (stage) {
        case WAIT_OBSERVATION:
            return "WAIT_OBSERVATION";
        case UPDATE_GLOBAL:
            return "UPDATE_GLOBAL";
        case PLAN_LOCAL:
            return "PLAN_LOCAL";
        case EXEC_LOCAL:
            return "EXEC_LOCAL";
        case RECOVER:
            return "RECOVER";
        case FINISH:
            return "FINISH";
    }
    return "UNKNOWN";
}

void ExplorationTask::changeStage(const Stage next) {
    if (stage_ == next) {
        return;
    }
    std::cout << " -- [Task] EXPLORATION_STAGE_CHANGE old="
              << stageName(stage_) << ", new=" << stageName(next) << std::endl;
    stage_ = next;
}

TaskTickResult ExplorationTask::makeResult(const TaskStatus status,
                                           const RET_CODE legacy_ret,
                                           const std::string &reason) const {
    TaskTickResult result;
    result.status = status;
    result.legacy_ret = legacy_ret;
    result.reason = reason;
    return result;
}

TaskTickResult ExplorationTask::handlePlanResult(const RET_CODE ret,
                                                 const TaskContext &ctx) {
    if (ret == super_utils::SUCCESS) {
        from_rest_ = false;
        changeStage(EXEC_LOCAL);
        return TaskTickResult::fromRetCode(ret, "exploration local trajectory committed");
    }
    if (ret == super_utils::FINISH) {
        from_rest_ = true;
        changeStage(FINISH);
        return makeResult(TaskStatus::FINISHED,
                          super_utils::FINISH,
                          "exploration finished");
    }
    if (ret == super_utils::NEW_TRAJ) {
        const auto policy = planner_->getExplorationRuntimePolicy();
        const auto exec = planner_->getExplorationExecutionStatus(ctx.now);
        if (exec.has_active_trajectory &&
            exec.traj_remaining > policy.min_remaining_for_replan) {
            from_rest_ = false;
            changeStage(EXEC_LOCAL);
            return makeResult(TaskStatus::KEEP_CURRENT,
                              super_utils::NO_NEED,
                              "keep current exploration trajectory before retrying continuous replan");
        }
        from_rest_ = true;
        changeStage(PLAN_LOCAL);
        return TaskTickResult::fromRetCode(ret,
                                           "exploration requests plan from rest");
    }
    if (ret == super_utils::NO_NEED) {
        const auto exec = planner_->getExplorationExecutionStatus(ctx.now);
        if (exec.has_active_trajectory && exec.traj_remaining > 0.05) {
            from_rest_ = false;
            changeStage(EXEC_LOCAL);
            return makeResult(TaskStatus::KEEP_CURRENT,
                              super_utils::NO_NEED,
                              "exploration keeps current local trajectory");
        }
        from_rest_ = true;
        changeStage(PLAN_LOCAL);
        return makeResult(TaskStatus::RUNNING,
                          super_utils::NO_NEED,
                          "waiting for reachable exploration guide");
    }
    if (ret == super_utils::EMER) {
        from_rest_ = true;
        changeStage(RECOVER);
        return makeResult(TaskStatus::EMERGENCY,
                          super_utils::EMER,
                          "exploration backend emergency");
    }

    from_rest_ = true;
    changeStage(RECOVER);
    return TaskTickResult::fromRetCode(ret, "exploration planning failed");
}

}  // namespace general_planner
