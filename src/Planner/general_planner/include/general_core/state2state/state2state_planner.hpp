#pragma once

#include <string>
#include <utility>

#include <fmt/core.h>

#include <general_core/planner_context.hpp>
#include <general_core/planning_result.hpp>
#include <general_core/state2state/state2state_backend.hpp>
#include <rog_map/rog_map.h>

namespace general_planner::architecture {

struct StateToStateRequest {
    PlanRequest plan_request;
    general_utils::Vec3f goal{general_utils::Vec3f::Zero()};
    double goal_yaw{0.0};
    bool new_goal{false};
};

struct StateToStateResult {
    PlanResult plan_result;
    general_utils::Vec3f resolved_goal{general_utils::Vec3f::Zero()};
    bool goal_projected{false};
};

class StateToStatePlanner {
public:
    explicit StateToStatePlanner(PlannerContext context)
        : context_(std::move(context)) {}

    StateToStateResult plan(const StateToStateRequest &request) const {
        StateToStateResult out;
        out.resolved_goal = request.goal;
        out.plan_result = makeResult(request.plan_request,
                                     dispatchBackend(request,
                                                     PlanInvocation::PLAN_FROM_REST));
        out.plan_result.context.mission_node = "state2state";
        return out;
    }

    StateToStateResult replan(const StateToStateRequest &request) const {
        StateToStateResult out;
        general_utils::Vec3f replan_goal = request.goal;
        out.goal_projected = projectOccupiedGoal(replan_goal);
        out.resolved_goal = replan_goal;
        StateToStateRequest backend_request = request;
        backend_request.goal = replan_goal;
        out.plan_result = makeResult(request.plan_request,
                                     dispatchBackend(backend_request,
                                                     PlanInvocation::REPLAN));
        out.plan_result.context.mission_node = "state2state";
        if (out.goal_projected) {
            out.plan_result.context.decision_detail = "goal_projected_for_replan";
        }
        return out;
    }

private:
    PlanResult makeResult(const PlanRequest &request,
                          const StateToStateBackendResult &backend_result) const {
        PlanResult result;
        result.request = request;
        result.ret_code = backend_result.ret_code;
        result.detail = std::string("backend=") + toString(backend_result.backend);
        if (!backend_result.detail.empty()) {
            result.detail += ";" + backend_result.detail;
        }
        return result;
    }

    StateToStateBackendResult dispatchBackend(const StateToStateRequest &request,
                                              const PlanInvocation invocation) const {
        return StateToStateBackendRouter::makeLegacy(context_).run(
                StateToStateBackendRequest{invocation,
                                           request.goal,
                                           request.goal_yaw,
                                           request.new_goal,
                                           request.plan_request.identity.backend});
    }

    bool projectOccupiedGoal(general_utils::Vec3f &goal) const {
        const auto map_manager = context_.mapManager();
        if (!map_manager ||
            map_manager->getInfGridType(goal) != rog_map::GridType::OCCUPIED) {
            return false;
        }

        const general_utils::Vec3f original_goal = goal;
        general_utils::Vec3f projected_goal = goal;
        if (map_manager->getNearestInfCellNot(rog_map::GridType::OCCUPIED,
                                              original_goal,
                                              projected_goal,
                                              3.0)) {
            context_.recordDiagnostic(
                    "WARN",
                    "goal_projected_for_replan",
                    fmt::format("reason=occupied;original=[{:.3f},{:.3f},{:.3f}];projected=[{:.3f},{:.3f},{:.3f}];distance={:.3f};goal_unified=1",
                                original_goal.x(), original_goal.y(), original_goal.z(),
                                projected_goal.x(), projected_goal.y(), projected_goal.z(),
                                (projected_goal - original_goal).norm()));
            goal = projected_goal;
            return true;
        }

        context_.recordDiagnostic(
                "WARN",
                "goal_projection_failed_for_replan",
                fmt::format("reason=occupied;goal=[{:.3f},{:.3f},{:.3f}]",
                            original_goal.x(), original_goal.y(), original_goal.z()));
        return false;
    }

    PlannerContext context_;
};

} // namespace general_planner::architecture
