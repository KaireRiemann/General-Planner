#include "general_core/task/skills/se3_aggressive_skill.hpp"

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

SE3AggressiveSkill::SE3AggressiveSkill(GeneralPlanner::Ptr planner)
    : planner_(std::move(planner)) {}

TaskTickResult SE3AggressiveSkill::tick(const TaskContext &ctx) {
  if (!planner_) {
    TaskTickResult result;
    result.status = TaskStatus::FAILED_FATAL;
    result.reason = "se3 aggressive planner missing";
    return result;
  }
  if (!ctx.state_goal_p.has_value()) {
    TaskTickResult result;
    result.status = TaskStatus::NOT_READY;
    result.reason = "se3 aggressive goal missing";
    return result;
  }

  const double goal_yaw = ctx.state_goal_yaw.value_or(NAN);
  const RET_CODE ret =
      from_rest_
          ? planner_->PlanSE3AggressiveFromRest(*ctx.state_goal_p, goal_yaw, ctx.state_goal_new)
          : planner_->ReplanSE3AggressiveOnce(*ctx.state_goal_p, goal_yaw, ctx.state_goal_new);
  if (from_rest_ && retKeepsSkillActive(ret)) {
    from_rest_ = false;
  }
  return TaskTickResult::fromRetCode(ret, "se3_aggressive");
}

void SE3AggressiveSkill::reset() {
  from_rest_ = true;
}

std::string SE3AggressiveSkill::name() const {
  return "se3_aggressive";
}

} // namespace general_planner
