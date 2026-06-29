/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/state2state/state2state_se3_backend.hpp>
#include <algorithm>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {
    state2state_task::StateToStateSE3BackendServices GeneralPlanner::makeStateToStateSE3BackendServices() {
        state2state_task::StateToStateSE3BackendServices services{
                cfg_,
                map_manager_,
                ros_ptr_,
                robot_state_,
                se3_aggressive_manager_.get(),
                *state2state_backend_context_,
                *state2state_se3_runtime_
        };
        return services;
    }

    namespace state2state_task {

    RET_CODE planSE3FromRest(StateToStateTaskServices &services,
                             StateToStateSE3BackendServices &se3_services,
                             const Vec3f &goal_p,
                             const double goal_yaw,
                             const bool new_task) {
        (void)new_task;
        TimeConsuming total_t("PlanSE3AggressiveFromRest", false);
        std::lock_guard<std::mutex> guard(services.replan_lock);
        services.latest_replan.reset();
        if (!services.robot_state.rcv) {
            services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);
            services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            services.ros_ptr->warn(" -- [SE3Aggressive] PlanFromRest failed: no odom.");
            services.time_consuming[TOTAL_REPLAN] = total_t.stop();
            return FAILED;
        }
        services.backend_context.setGoalInfo(goal_p, goal_yaw, true, true);
        services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);
        services.ros_ptr->vizGoalPath(vec_Vec3f{goal_p, services.robot_state.p});
        const RET_CODE ret = optimizeSE3AggressiveTask(
                se3_services,
                goal_p,
                goal_yaw,
                true);
        services.latest_replan.setRetCode((ret == SUCCESS || ret == FINISH)
                                              ? GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP
                                              : GENERAL_RET_CODE::GENERAL_UNDEFINED);
        services.time_consuming[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE replanSE3Once(StateToStateTaskServices &services,
                           StateToStateSE3BackendServices &se3_services,
                           const Vec3f &goal_p,
                           const double goal_yaw,
                           const bool new_task) {
        (void)new_task;
        TimeConsuming total_t("ReplanSE3AggressiveOnce", false);
        std::lock_guard<std::mutex> guard(services.replan_lock);
        services.latest_replan.reset();
        if (!services.robot_state.rcv) {
            services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);
            services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            services.ros_ptr->warn(" -- [SE3Aggressive] Replan failed: no odom.");
            services.time_consuming[TOTAL_REPLAN] = total_t.stop();
            return FAILED;
        }
        services.backend_context.setGoalInfo(goal_p, goal_yaw, true, true);
        services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);
        services.ros_ptr->vizGoalPath(vec_Vec3f{goal_p, services.robot_state.p});
        const RET_CODE ret = optimizeSE3AggressiveTask(
                se3_services,
                goal_p,
                goal_yaw,
                false);
        services.latest_replan.setRetCode((ret == SUCCESS || ret == FINISH)
                                              ? GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP
                                              : GENERAL_RET_CODE::GENERAL_UNDEFINED);
        services.time_consuming[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    } // namespace state2state_task

    RET_CODE GeneralPlanner::PlanSE3AggressiveFromRest(const Vec3f &goal_p,
                                                       const double goal_yaw,
                                                       const bool new_task) {
        auto services = makeStateToStateTaskServices();
        auto se3_services = makeStateToStateSE3BackendServices();
        return state2state_task::planSE3FromRest(services, se3_services, goal_p, goal_yaw, new_task);
    }

    RET_CODE GeneralPlanner::ReplanSE3AggressiveOnce(const Vec3f &goal_p,
                                                     const double goal_yaw,
                                                     const bool new_task) {
        auto services = makeStateToStateTaskServices();
        auto se3_services = makeStateToStateSE3BackendServices();
        return state2state_task::replanSE3Once(services, se3_services, goal_p, goal_yaw, new_task);
    }

}
