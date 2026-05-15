#include "general_core/task/skills/perching_skill.hpp"

#include <utility>

namespace general_planner {

namespace {
bool retKeepsSkillActive(const RET_CODE ret) {
    return ret == super_utils::SUCCESS ||
           ret == super_utils::NO_NEED ||
           ret == super_utils::FINISH;
}
} // namespace

PerchingSkill::PerchingSkill(GeneralPlanner::Ptr planner)
    : planner_(std::move(planner)) {}

TaskTickResult PerchingSkill::tick(const TaskContext &ctx) {
    if (!planner_) {
        TaskTickResult result;
        result.status = TaskStatus::FAILED_FATAL;
        result.reason = "perching planner missing";
        return result;
    }
    if (!ctx.surface.has_value()) {
        TaskTickResult result;
        result.status = TaskStatus::NOT_READY;
        result.reason = "perching surface missing";
        return result;
    }

    const RET_CODE ret = from_rest_
                             ? planner_->PlanPerchingFromRest(*ctx.surface, ctx.new_task)
                             : planner_->ReplanPerchingOnce(*ctx.surface, ctx.new_task);
    if (from_rest_ && retKeepsSkillActive(ret)) {
        from_rest_ = false;
    }
    return TaskTickResult::fromRetCode(ret, "perching");
}

void PerchingSkill::reset() {
    from_rest_ = true;
}

std::string PerchingSkill::name() const {
    return "perching";
}

} // namespace general_planner
