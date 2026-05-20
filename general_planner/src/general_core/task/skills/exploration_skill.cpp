#include "general_core/task/skills/exploration_skill.hpp"

#include <utility>

namespace general_planner {

namespace {
bool retKeepsSkillActive(const RET_CODE ret) {
    return ret == super_utils::SUCCESS ||
           ret == super_utils::NO_NEED ||
           ret == super_utils::FINISH;
}
}  // namespace

ExplorationSkill::ExplorationSkill(GeneralPlanner::Ptr planner)
        : planner_(std::move(planner)) {}

TaskTickResult ExplorationSkill::tick(const TaskContext &ctx) {
    (void)ctx;
    if (!planner_) {
        TaskTickResult result;
        result.status = TaskStatus::FAILED_FATAL;
        result.reason = "exploration planner missing";
        return result;
    }
    const RET_CODE ret = from_rest_
                         ? planner_->PlanExplorationFromRest(ctx.new_task)
                         : planner_->ReplanExplorationOnce(ctx.new_task);
    if (ret == super_utils::NEW_TRAJ) {
        from_rest_ = true;
    }
    if (from_rest_ && retKeepsSkillActive(ret)) {
        from_rest_ = false;
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
