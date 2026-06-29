#pragma once

#include <string>

#include <general_core/planning_semantics.hpp>
#include <utils/header/type_utils.hpp>

namespace general_planner::architecture {

enum class PlanInvocation {
    PLAN_FROM_REST,
    REPLAN
};

enum class CommitAction {
    COMMIT_CANDIDATE,
    KEEP_OLD_TRAJECTORY,
    HOLD,
    BRAKE,
    EMERGENCY_STOP,
    REQUEST_NEW_INPUT,
    RETRY_PLANNING,
    FINISH_MISSION,
    NOOP
};

struct PlanRequest {
    TaskIdentity identity;
    PlanInvocation invocation{PlanInvocation::PLAN_FROM_REST};
    ExecutionPhase phase{ExecutionPhase::WAITING_INPUT};
    bool new_goal{false};
    bool new_task{false};
    bool perception_trigger{false};
    bool perception_emergency{false};
    std::string reason;
};

struct PlanResult {
    PlanRequest request;
    TaskPlanContext context;
    int ret_code{general_utils::FAILED};
    std::string detail;

    bool missingInput() const {
        return context.missing_input;
    }

    bool handled() const {
        return context.handled;
    }

    bool successful() const {
        return ret_code == general_utils::SUCCESS ||
               ret_code == general_utils::FINISH;
    }

    bool failed() const {
        return ret_code == general_utils::FAILED ||
               ret_code == general_utils::OPT_FAILED ||
               ret_code == general_utils::INIT_ERROR ||
               ret_code == general_utils::NO_PATH ||
               ret_code == general_utils::TIME_OUT;
    }
};

struct CommitDecision {
    CommitAction action{CommitAction::NOOP};
    ExecutionPhase next_phase{ExecutionPhase::WAITING_INPUT};
    bool publish_trajectory{false};
    bool clear_goal{false};
    bool clear_task_new{false};
    bool finish_plan{false};
    bool plan_from_rest_consumed{false};
    std::string reason;
};

inline const char *toString(const PlanInvocation invocation) {
    switch (invocation) {
        case PlanInvocation::REPLAN:
            return "replan";
        case PlanInvocation::PLAN_FROM_REST:
        default:
            return "plan_from_rest";
    }
}

inline const char *toString(const CommitAction action) {
    switch (action) {
        case CommitAction::COMMIT_CANDIDATE:
            return "commit_candidate";
        case CommitAction::KEEP_OLD_TRAJECTORY:
            return "keep_old_trajectory";
        case CommitAction::HOLD:
            return "hold";
        case CommitAction::BRAKE:
            return "brake";
        case CommitAction::EMERGENCY_STOP:
            return "emergency_stop";
        case CommitAction::REQUEST_NEW_INPUT:
            return "request_new_input";
        case CommitAction::RETRY_PLANNING:
            return "retry_planning";
        case CommitAction::FINISH_MISSION:
            return "finish_mission";
        case CommitAction::NOOP:
        default:
            return "noop";
    }
}

} // namespace general_planner::architecture
