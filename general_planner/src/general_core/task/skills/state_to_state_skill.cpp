#include "general_core/task/skills/state_to_state_skill.hpp"

#include <cmath>
#include <utility>

namespace general_planner {

namespace {
bool retKeepsSkillActive(const RET_CODE ret) {
    return ret == super_utils::SUCCESS ||
           ret == super_utils::NO_NEED ||
           ret == super_utils::FINISH;
}
} // namespace

StateToStateSkill::StateToStateSkill(GeneralPlanner::Ptr planner)
    : planner_(std::move(planner)) {}

TaskTickResult StateToStateSkill::tick(const TaskContext &ctx) {
    if (!planner_) {
        TaskTickResult result;
        result.status = TaskStatus::FAILED_FATAL;
        result.reason = "state2state planner missing";
        return result;
    }
    if (!ctx.state_goal_p.has_value()) {
        TaskTickResult result;
        result.status = TaskStatus::NOT_READY;
        result.reason = "state2state goal missing";
        return result;
    }

    const double goal_yaw = ctx.state_goal_yaw.value_or(NAN);
    const RET_CODE ret = from_rest_
                             ? planner_->PlanFromRest(*ctx.state_goal_p, goal_yaw, ctx.state_goal_new)
                             : planner_->ReplanOnce(*ctx.state_goal_p, goal_yaw, ctx.state_goal_new);
    if (from_rest_ && retKeepsSkillActive(ret)) {
        from_rest_ = false;
    }
    return TaskTickResult::fromRetCode(ret, "state2state");
}

void StateToStateSkill::reset() {
    from_rest_ = true;
}

std::string StateToStateSkill::name() const {
    return "state2state";
}

} // namespace general_planner
