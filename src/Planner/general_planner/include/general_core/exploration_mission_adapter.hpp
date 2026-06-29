#pragma once

#include <string>
#include <utility>

#include <general_core/planner_context.hpp>
#include <general_core/planning_result.hpp>

namespace general_planner::architecture {

struct ExplorationPlanRequest {
    PlanRequest plan_request;
    bool new_task{false};
};

class ExplorationMissionAdapter {
public:
    explicit ExplorationMissionAdapter(PlannerContext context)
        : context_(std::move(context)) {}

    PlanResult plan(const ExplorationPlanRequest &request,
                    TaskPlanContext context = {}) const {
        if (!context_.valid()) {
            return makeResult(request.plan_request, general_utils::INIT_ERROR, std::move(context));
        }
        const int ret = context_.planner().PlanExplorationFromRest(request.new_task);
        return makeResult(request.plan_request, ret, std::move(context));
    }

    PlanResult replan(const ExplorationPlanRequest &request,
                      TaskPlanContext context = {}) const {
        if (!context_.valid()) {
            return makeResult(request.plan_request, general_utils::INIT_ERROR, std::move(context));
        }
        const int ret = context_.planner().ReplanExplorationOnce(request.new_task);
        return makeResult(request.plan_request, ret, std::move(context));
    }

private:
    PlanResult makeResult(const PlanRequest &request,
                          const int ret_code,
                          TaskPlanContext context) const {
        context.mission_node = "exploration_mission";

        PlanResult result;
        result.request = request;
        result.context = std::move(context);
        result.ret_code = ret_code;
        result.detail = result.context.mission_node;
        return result;
    }

    PlannerContext context_;
};

} // namespace general_planner::architecture
