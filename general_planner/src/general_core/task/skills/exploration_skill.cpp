#include "general_core/task/skills/exploration_skill.hpp"

#include <utility>

namespace general_planner {

ExplorationSkill::ExplorationSkill(GeneralPlanner::Ptr planner)
        : planner_(std::move(planner)) {}

TaskTickResult ExplorationSkill::tick(const TaskContext &ctx) {
    if (!planner_) {
        TaskTickResult result;
        result.status = TaskStatus::FAILED_FATAL;
        result.reason = "exploration planner missing";
        return result;
    }
    if (!planner_->explorationObservationReady()) {
        TaskTickResult result;
        result.status = TaskStatus::NOT_READY;
        result.legacy_ret = super_utils::NO_NEED;
        result.reason = "waiting for first exploration cloud observation";
        return result;
    }
    const RET_CODE ret = from_rest_
                         ? planner_->PlanExplorationFromRest(ctx.new_task)
                         : planner_->ReplanExplorationOnce(ctx.new_task);

    if (ret == super_utils::NEW_TRAJ) {
        from_rest_ = true;
    } else if (ret == super_utils::SUCCESS) {
        from_rest_ = false;
    } else if (ret == super_utils::NO_NEED) {
        const double remaining = planner_->getCommittedTrajectoryRemainingDuration();
        if (remaining > 0.05) {
            from_rest_ = false;
        } else {
            from_rest_ = true;
            TaskTickResult result;
            result.status = TaskStatus::NOT_READY;
            result.legacy_ret = super_utils::NO_NEED;
            result.reason = "waiting for reachable exploration guide";
            return result;
        }
    } else if (ret == super_utils::FINISH) {
        from_rest_ = true;
    }
    return TaskTickResult::fromRetCode(ret, "exploration");
}

void ExplorationSkill::reset() {
    from_rest_ = true;
}

std::string ExplorationSkill::name() const {
    return "exploration";
}

}  // namespace general_planner
