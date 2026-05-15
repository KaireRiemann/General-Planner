#include "general_core/task/composites/full_cycle_task.hpp"

#include <iostream>
#include <utility>

namespace general_planner {

FullCycleTask::FullCycleTask(GeneralPlanner::Ptr planner)
    : planner_(std::move(planner)),
      takeoff_skill_(planner_),
      tracking_skill_(planner_),
      tracking_perching_task_(planner_) {}

TaskTickResult FullCycleTask::tick(const TaskContext &ctx) {
    if (!planner_) {
        TaskTickResult result;
        result.status = TaskStatus::FAILED_FATAL;
        result.reason = "full_cycle planner missing";
        return result;
    }

    if (stage_ == TAKEOFF) {
        if (takeoff_committed_ &&
            planner_->getCommittedTrajectoryRemainingDuration() <= 0.05) {
            changeStage(TRACKING);
        } else {
            TaskTickResult result = takeoff_skill_.tick(ctx);
            if (result.legacy_ret == super_utils::SUCCESS ||
                result.legacy_ret == super_utils::NO_NEED ||
                result.legacy_ret == super_utils::FINISH) {
                takeoff_committed_ = true;
            }
            if (!takeoff_committed_ ||
                planner_->getCommittedTrajectoryRemainingDuration() > 0.05) {
                return result;
            }
            changeStage(TRACKING);
        }
    }

    if (stage_ == TRACKING) {
        if (ctx.surface.has_value() &&
            ctx.target_prediction.has_value() &&
            !ctx.target_prediction->empty()) {
            changeStage(TRACKING_PERCHING);
        } else {
            return tracking_skill_.tick(ctx);
        }
    }

    if (stage_ == TRACKING_PERCHING) {
        TaskTickResult result = tracking_perching_task_.tick(ctx);
        if (result.status == TaskStatus::FINISHED ||
            result.legacy_ret == super_utils::FINISH) {
            changeStage(CONTACT);
        }
        return result;
    }

    TaskTickResult result;
    result.status = TaskStatus::FINISHED;
    result.legacy_ret = super_utils::FINISH;
    result.reason = "full_cycle contact";
    return result;
}

void FullCycleTask::reset() {
    CompositeTask::reset();
    takeoff_skill_.reset();
    tracking_skill_.reset();
    tracking_perching_task_.reset();
    takeoff_committed_ = false;
}

std::string FullCycleTask::name() const {
    return "full_cycle";
}

const char *FullCycleTask::stageName(const int stage) {
    switch (stage) {
        case TAKEOFF:
            return "TAKEOFF";
        case TRACKING:
            return "TRACKING";
        case TRACKING_PERCHING:
            return "TRACKING_PERCHING";
        case CONTACT:
            return "CONTACT";
    }
    return "UNKNOWN";
}

void FullCycleTask::changeStage(const Stage next) {
    if (stage_ == next) {
        return;
    }
    std::cout << " -- [Task] FULL_CYCLE_STAGE_CHANGE old="
              << stageName(stage_) << ", new=" << stageName(next) << std::endl;
    stage_ = next;
}

} // namespace general_planner
