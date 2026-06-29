#pragma once

#include <string>
#include <utility>

#include <general_core/planner_context.hpp>
#include <general_core/planning_result.hpp>
#include <traj_opt/perching_surface_state.hpp>

namespace general_planner::architecture {

struct PerchingPlanRequest {
    PlanRequest plan_request;
    traj_opt::PerchingSurfaceState surface;
    bool new_task{false};
};

class PerchingPlanner {
public:
    explicit PerchingPlanner(PlannerContext context)
        : context_(std::move(context)) {}

    PlanResult plan(const PerchingPlanRequest &request,
                    TaskPlanContext context = {}) const {
        if (!context_.valid()) {
            return makeResult(request.plan_request, general_utils::INIT_ERROR, std::move(context));
        }
        const int ret = context_.planner().PlanPerchingFromRest(request.surface,
                                                                request.new_task);
        return makeResult(request.plan_request, ret, std::move(context));
    }

    PlanResult replan(const PerchingPlanRequest &request,
                      TaskPlanContext context = {}) const {
        if (!context_.valid()) {
            return makeResult(request.plan_request, general_utils::INIT_ERROR, std::move(context));
        }
        const int ret = context_.planner().ReplanPerchingOnce(request.surface,
                                                              request.new_task);
        return makeResult(request.plan_request, ret, std::move(context));
    }

private:
    PlanResult makeResult(const PlanRequest &request,
                          const int ret_code,
                          TaskPlanContext context) const {
        context.mission_node = "perching";

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
