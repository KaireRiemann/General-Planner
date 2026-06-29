#pragma once

#include <string>
#include <utility>

#include <general_core/planner_context.hpp>
#include <general_core/planning_result.hpp>
#include <general_core/tracking/tracking_plan_operations.hpp>
#include <traj_opt/tracking_perching_traj_opt.hpp>

namespace general_planner::architecture {

struct TrackingPlanRequest {
    PlanRequest plan_request;
    traj_opt::DynamicTargetStates target_prediction;
    bool new_task{false};
};

class TrackingPlanner {
public:
    explicit TrackingPlanner(PlannerContext context)
        : context_(std::move(context)) {}

    PlanResult plan(const TrackingPlanRequest &request,
                    TaskPlanContext context,
                    std::string mission_node) const {
        if (!context_.valid()) {
            return makeResult(request.plan_request,
                              general_utils::INIT_ERROR,
                              std::move(context),
                              std::move(mission_node));
        }
        auto services = context_.planner().makeTrackingTaskServices();
        auto backend_services = context_.planner().makeTrackingBackendServices();
        const int ret = tracking_task::planFromRest(services,
                                                    backend_services,
                                                    request.target_prediction,
                                                    request.new_task);
        return makeResult(request.plan_request,
                          ret,
                          std::move(context),
                          std::move(mission_node));
    }

    PlanResult replan(const TrackingPlanRequest &request,
                      TaskPlanContext context,
                      std::string mission_node) const {
        if (!context_.valid()) {
            return makeResult(request.plan_request,
                              general_utils::INIT_ERROR,
                              std::move(context),
                              std::move(mission_node));
        }
        auto services = context_.planner().makeTrackingTaskServices();
        auto backend_services = context_.planner().makeTrackingBackendServices();
        const int ret = tracking_task::replanOnce(services,
                                                  backend_services,
                                                  request.target_prediction,
                                                  request.new_task);
        return makeResult(request.plan_request,
                          ret,
                          std::move(context),
                          std::move(mission_node));
    }

    PlanResult replanWithPerchingSurface(const TrackingPlanRequest &request,
                                         const traj_opt::PerchingSurfaceState &surface,
                                         TaskPlanContext context,
                                         std::string mission_node) const {
        if (!context_.valid()) {
            return makeResult(request.plan_request,
                              general_utils::INIT_ERROR,
                              std::move(context),
                              std::move(mission_node));
        }
        auto services = context_.planner().makeTrackingTaskServices();
        auto backend_services = context_.planner().makeTrackingBackendServices();
        const int ret = tracking_task::replanWithPerchingSurface(services,
                                                                 backend_services,
                                                                 request.target_prediction,
                                                                 surface,
                                                                 request.new_task);
        return makeResult(request.plan_request,
                          ret,
                          std::move(context),
                          std::move(mission_node));
    }

    PlanResult tryCommitPerchingFromTracking(const TrackingPlanRequest &request,
                                             const traj_opt::PerchingSurfaceState &surface,
                                             const int tracking_ret,
                                             TaskPlanContext context,
                                             std::string mission_node) const {
        if (!context_.valid()) {
            return makeResult(request.plan_request,
                              general_utils::INIT_ERROR,
                              std::move(context),
                              std::move(mission_node));
        }
        auto services = context_.planner().makeTrackingTaskServices();
        const int ret = tracking_task::tryCommitPerchingFromTracking(
                services,
                request.target_prediction,
                surface,
                static_cast<general_utils::RET_CODE>(tracking_ret));
        return makeResult(request.plan_request,
                          ret,
                          std::move(context),
                          std::move(mission_node));
    }

private:
    PlanResult makeResult(const PlanRequest &request,
                          const int ret_code,
                          TaskPlanContext context,
                          std::string mission_node) const {
        context.mission_node = std::move(mission_node);

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
