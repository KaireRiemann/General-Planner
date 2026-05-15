#include "general_core/task/skills/dynamic_takeoff_skill.hpp"

#include <utility>

namespace general_planner {

namespace {
bool retKeepsSkillActive(const RET_CODE ret) {
    return ret == super_utils::SUCCESS ||
           ret == super_utils::NO_NEED ||
           ret == super_utils::FINISH;
}
} // namespace

DynamicTakeoffSkill::DynamicTakeoffSkill(GeneralPlanner::Ptr planner)
    : planner_(std::move(planner)) {}

TaskTickResult DynamicTakeoffSkill::tick(const TaskContext &ctx) {
    if (!planner_) {
        TaskTickResult result;
        result.status = TaskStatus::FAILED_FATAL;
        result.reason = "dynamic_takeoff planner missing";
        return result;
    }
    if (!ctx.surface.has_value()) {
        TaskTickResult result;
        result.status = TaskStatus::NOT_READY;
        result.reason = "dynamic_takeoff surface missing";
        return result;
    }

    const RET_CODE ret = from_rest_
                             ? planner_->PlanDynamicTakeoffFromRest(*ctx.surface, ctx.new_task)
                             : planner_->ReplanDynamicTakeoffOnce(*ctx.surface, ctx.new_task);
    if (from_rest_ && retKeepsSkillActive(ret)) {
        from_rest_ = false;
    }
    return TaskTickResult::fromRetCode(ret, "dynamic_takeoff");
}

void DynamicTakeoffSkill::reset() {
    from_rest_ = true;
}

std::string DynamicTakeoffSkill::name() const {
    return "dynamic_takeoff";
}

} // namespace general_planner
