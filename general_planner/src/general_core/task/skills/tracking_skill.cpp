#include "general_core/task/skills/tracking_skill.hpp"

#include <utility>

namespace general_planner {

namespace {
bool retKeepsSkillActive(const RET_CODE ret) {
    return ret == super_utils::SUCCESS ||
           ret == super_utils::NO_NEED ||
           ret == super_utils::FINISH;
}
} // namespace

TrackingSkill::TrackingSkill(GeneralPlanner::Ptr planner,
                             const bool use_surface_transition)
    : planner_(std::move(planner)),
      use_surface_transition_(use_surface_transition) {}

TaskTickResult TrackingSkill::tick(const TaskContext &ctx) {
    if (!planner_) {
        TaskTickResult result;
        result.status = TaskStatus::FAILED_FATAL;
        result.reason = "tracking planner missing";
        return result;
    }
    if (!ctx.target_prediction.has_value() || ctx.target_prediction->empty()) {
        TaskTickResult result;
        result.status = TaskStatus::NOT_READY;
        result.reason = "tracking prediction missing";
        return result;
    }

    RET_CODE ret = super_utils::FAILED;
    if (from_rest_) {
        ret = planner_->PlanTrackingFromRest(*ctx.target_prediction, ctx.new_task);
        if (retKeepsSkillActive(ret)) {
            from_rest_ = false;
        }
    } else if (use_surface_transition_ && ctx.surface.has_value()) {
        ret = planner_->ReplanTrackingOnce(*ctx.target_prediction, *ctx.surface, ctx.new_task);
    } else {
        ret = planner_->ReplanTrackingOnce(*ctx.target_prediction, ctx.new_task);
    }
    return TaskTickResult::fromRetCode(ret, "tracking");
}

void TrackingSkill::reset() {
    from_rest_ = true;
}

std::string TrackingSkill::name() const {
    return "tracking";
}

void TrackingSkill::setUseSurfaceTransition(const bool enable) {
    use_surface_transition_ = enable;
}

} // namespace general_planner
