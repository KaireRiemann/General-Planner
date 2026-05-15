#include "general_core/task/composites/tracking_perching_task.hpp"

#include <iostream>
#include <utility>

namespace general_planner {

namespace {
bool retKeepsSkillActive(const RET_CODE ret) {
    return ret == super_utils::SUCCESS ||
           ret == super_utils::NO_NEED ||
           ret == super_utils::FINISH;
}

TaskTickResult keepCurrent(const RET_CODE legacy_ret,
                           const std::string &reason) {
    TaskTickResult result;
    result.status = TaskStatus::KEEP_CURRENT;
    result.legacy_ret = legacy_ret;
    result.reason = reason;
    return result;
}
} // namespace

TrackingPerchingTask::TrackingPerchingTask(GeneralPlanner::Ptr planner)
    : planner_(std::move(planner)) {}

TaskTickResult TrackingPerchingTask::tick(const TaskContext &ctx) {
    if (!planner_) {
        TaskTickResult result;
        result.status = TaskStatus::FAILED_FATAL;
        result.reason = "tracking_perching planner missing";
        return result;
    }
    if (planner_->trackingPerchingContactReached()) {
        changeStage(CONTACT);
    }

    if ((stage_ == TRACKING || stage_ == TRY_PERCHING) &&
        (!ctx.target_prediction.has_value() || ctx.target_prediction->empty())) {
        TaskTickResult result;
        result.status = TaskStatus::NOT_READY;
        result.reason = "tracking_perching target prediction missing";
        return result;
    }

    if (stage_ == TRACKING || stage_ == TRY_PERCHING) {
        RET_CODE ret = super_utils::FAILED;
        if (from_rest_) {
            ret = planner_->PlanTrackingFromRest(*ctx.target_prediction, ctx.new_task);
            if (retKeepsSkillActive(ret)) {
                from_rest_ = false;
            }
        } else {
            ret = planner_->ReplanTrackingOnce(*ctx.target_prediction, ctx.new_task);
        }

        if (retKeepsSkillActive(ret) &&
            ctx.surface.has_value() &&
            !planner_->trackingPerchingPerchingActive()) {
            ret = planner_->TryCommitPerchingFromTracking(*ctx.target_prediction,
                                                          *ctx.surface,
                                                          ret);
        }

        std::cout << " -- [Task] TRACKING_PERCHING_TASK tracking stage ret="
                  << legacyRetCodeName(ret) << std::endl;
        if (planner_->trackingPerchingPerchingActive()) {
            std::cout << " -- [Task] TRACKING_PERCHING_TASK perching committed, final maneuver ownership transferred"
                      << std::endl;
            changeStage(PERCHING_EXECUTING);
        } else if (ctx.surface.has_value()) {
            changeStage(TRY_PERCHING);
        }
        return TaskTickResult::fromRetCode(ret, "tracking_perching tracking stage");
    }

    if (stage_ == PERCHING_EXECUTING) {
        if (planner_->trackingPerchingContactReached()) {
            changeStage(CONTACT);
            TaskTickResult result;
            result.status = TaskStatus::FINISHED;
            result.legacy_ret = super_utils::FINISH;
            result.reason = "tracking_perching contact";
            return result;
        }
        if (!ctx.surface.has_value()) {
            return keepCurrent(super_utils::NO_NEED,
                               "tracking_perching keeps committed perching while surface is missing");
        }

        const RET_CODE ret = planner_->ReplanPerchingOnce(*ctx.surface, false);
        std::cout << " -- [Task] TRACKING_PERCHING_TASK final maneuver ret="
                  << legacyRetCodeName(ret) << std::endl;
        if (planner_->trackingPerchingContactReached()) {
            changeStage(CONTACT);
        } else if (ret == super_utils::FAILED && planner_->trackingPerchingPerchingActive()) {
            return keepCurrent(super_utils::NO_NEED,
                               "tracking_perching keeps committed perching after perching replan failed");
        } else if (ret == super_utils::FAILED) {
            changeStage(ABORT);
        }
        return TaskTickResult::fromRetCode(ret, "tracking_perching perching stage");
    }

    if (stage_ == ABORT) {
        from_rest_ = false;
        changeStage(TRACKING);
        TaskTickResult result;
        result.status = TaskStatus::KEEP_CURRENT;
        result.legacy_ret = super_utils::NO_NEED;
        result.reason = "abort to tracking";
        return result;
    }

    TaskTickResult result;
    result.status = TaskStatus::FINISHED;
    result.legacy_ret = super_utils::FINISH;
    result.reason = "tracking_perching contact";
    return result;
}

void TrackingPerchingTask::reset() {
    CompositeTask::reset();
    from_rest_ = true;
}

std::string TrackingPerchingTask::name() const {
    return "tracking_perching";
}

const char *TrackingPerchingTask::stageName(const int stage) {
    switch (stage) {
        case TRACKING:
            return "TRACKING";
        case TRY_PERCHING:
            return "TRY_PERCHING";
        case PERCHING_EXECUTING:
            return "PERCHING_EXECUTING";
        case CONTACT:
            return "CONTACT";
        case ABORT:
            return "ABORT";
    }
    return "UNKNOWN";
}

void TrackingPerchingTask::changeStage(const Stage next) {
    if (stage_ == next) {
        return;
    }
    std::cout << " -- [Task] COMPOSITE_STAGE_CHANGE task=tracking_perching old="
              << stageName(stage_) << ", new=" << stageName(next) << std::endl;
    stage_ = next;
}

} // namespace general_planner
