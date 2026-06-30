#pragma once

#include <general_core/planning_result.hpp>

namespace general_planner::architecture {

struct CommitGovernorInput {
    PlanResult result;
    bool planner_goal_valid{true};
    bool exploration_task{false};
};

class CommitGovernor {
public:
    CommitDecision decidePlanFromRest(const CommitGovernorInput &input) const {
        const PlanResult &result = input.result;
        CommitDecision decision;
        decision.reason = result.detail;

        if (result.missingInput()) {
            decision.action = CommitAction::REQUEST_NEW_INPUT;
            decision.next_phase = ExecutionPhase::WAITING_INPUT;
            decision.reason = "missing_input";
            return decision;
        }
        if (result.handled()) {
            decision.action = CommitAction::NOOP;
            decision.next_phase = result.request.phase;
            decision.reason = "handled_by_task_plugin";
            return decision;
        }
        if (result.request.identity.goal_like && !input.planner_goal_valid) {
            decision.action = CommitAction::REQUEST_NEW_INPUT;
            decision.next_phase = ExecutionPhase::WAITING_INPUT;
            decision.clear_goal = true;
            decision.finish_plan = true;
            decision.reason = "goal_invalid";
            return decision;
        }
        if (result.ret_code == general_utils::EMER) {
            decision.action = CommitAction::EMERGENCY_STOP;
            decision.next_phase = ExecutionPhase::EMERGENCY;
            decision.reason = "ret_emergency";
            return decision;
        }
        if (result.ret_code == general_utils::NO_NEED &&
            result.request.identity.tracking_like) {
            decision.action = CommitAction::KEEP_OLD_TRAJECTORY;
            decision.next_phase = result.context.tracking_prediction_static
                                      ? ExecutionPhase::HOLDING
                                      : ExecutionPhase::EXECUTING;
            decision.publish_trajectory = true;
            decision.plan_from_rest_consumed = true;
            decision.reason = "tracking_no_need_keep_current";
            return decision;
        }
        if (result.ret_code == general_utils::FINISH && input.exploration_task) {
            decision.action = CommitAction::FINISH_MISSION;
            decision.next_phase = ExecutionPhase::WAITING_INPUT;
            decision.clear_goal = true;
            decision.clear_task_new = true;
            decision.finish_plan = true;
            decision.reason = "exploration_finished";
            return decision;
        }
        if (result.successful()) {
            decision.action = CommitAction::COMMIT_CANDIDATE;
            decision.next_phase = result.context.tracking_prediction_static
                                      ? ExecutionPhase::HOLDING
                                      : ExecutionPhase::EXECUTING;
            decision.publish_trajectory = true;
            decision.clear_goal = true;
            decision.clear_task_new = true;
            decision.plan_from_rest_consumed = true;
            decision.finish_plan = result.ret_code == general_utils::FINISH;
            decision.reason = "ret_success";
            return decision;
        }

        decision.action = CommitAction::RETRY_PLANNING;
        decision.next_phase = ExecutionPhase::PLANNING;
        decision.reason = "plan_from_rest_failed";
        return decision;
    }

    CommitDecision decideReplan(const CommitGovernorInput &input) const {
        const PlanResult &result = input.result;
        CommitDecision decision;
        decision.reason = result.detail;

        if (result.missingInput() || result.handled()) {
            decision.action = CommitAction::NOOP;
            decision.next_phase = result.request.phase;
            decision.reason = result.missingInput() ? "missing_input" : "handled_by_task_plugin";
            return decision;
        }
        if (result.ret_code == general_utils::EMER) {
            decision.action = CommitAction::EMERGENCY_STOP;
            decision.next_phase = ExecutionPhase::EMERGENCY;
            decision.reason = "ret_emergency";
            return decision;
        }
        if (result.failed() && result.request.identity.tracking_like) {
            decision.action = CommitAction::KEEP_OLD_TRAJECTORY;
            decision.next_phase = ExecutionPhase::EXECUTING;
            decision.reason = "tracking_replan_failed_keep_current";
            return decision;
        }
        if (result.failed() && result.request.perception_trigger &&
            result.request.perception_emergency && result.request.identity.goal_like) {
            decision.action = CommitAction::KEEP_OLD_TRAJECTORY;
            decision.next_phase = ExecutionPhase::EXECUTING;
            decision.reason = "perception_replan_failed_keep_current";
            return decision;
        }
        if (result.ret_code == general_utils::NEW_TRAJ) {
            decision.action = CommitAction::RETRY_PLANNING;
            decision.next_phase = ExecutionPhase::PLANNING;
            decision.reason = "new_traj_requires_generate";
            return decision;
        }
        if (result.ret_code == general_utils::NO_NEED &&
            result.request.identity.tracking_like) {
            decision.action = CommitAction::KEEP_OLD_TRAJECTORY;
            decision.next_phase = result.context.tracking_prediction_static
                                      ? ExecutionPhase::HOLDING
                                      : ExecutionPhase::EXECUTING;
            decision.publish_trajectory = true;
            decision.reason = "tracking_no_need_keep_current";
            return decision;
        }
        if (result.ret_code == general_utils::NO_NEED &&
            result.request.identity.goal_like) {
            decision.action = CommitAction::KEEP_OLD_TRAJECTORY;
            decision.next_phase = ExecutionPhase::EXECUTING;
            decision.publish_trajectory = false;
            decision.reason = "state2state_no_need_keep_current";
            return decision;
        }
        if (result.ret_code == general_utils::FINISH && input.exploration_task) {
            decision.action = CommitAction::FINISH_MISSION;
            decision.next_phase = ExecutionPhase::WAITING_INPUT;
            decision.clear_goal = true;
            decision.clear_task_new = true;
            decision.finish_plan = true;
            decision.reason = "exploration_finished";
            return decision;
        }
        if (result.successful()) {
            decision.action = CommitAction::COMMIT_CANDIDATE;
            decision.next_phase = result.context.tracking_prediction_static
                                      ? ExecutionPhase::HOLDING
                                      : ExecutionPhase::EXECUTING;
            decision.publish_trajectory = true;
            decision.clear_goal = true;
            decision.clear_task_new = true;
            decision.reason = "ret_success";
            return decision;
        }

        decision.action = CommitAction::NOOP;
        decision.next_phase = result.request.phase;
        decision.reason = "ret_unhandled";
        return decision;
    }
};

} // namespace general_planner::architecture
