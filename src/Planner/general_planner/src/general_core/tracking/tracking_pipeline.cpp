/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_plan_operations.hpp>

#include <cmath>
#include <mutex>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {
    namespace {
        bool trackingPerchingPerchingStatus(
                const TrackingPerchingTransitionManager::Status status) {
            return status == TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED ||
                   status == TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING ||
                   status == TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT ||
                   status == TrackingPerchingTransitionManager::Status::CONTACT;
        }
    }

    tracking_task::TrackingTaskServices GeneralPlanner::makeTrackingTaskServices() {
        tracking_task::TrackingTaskServices services{
                replan_lock_,
                latest_replan,
                last_exp_traj_info_,
                robot_state_,
                ros_ptr_,
                tracking_perching_manager_.get(),
                time_consuming_,
                {},
                {},
                {},
                {},
                {}
        };
        services.set_tracking_diagnostic =
                [this](const std::string &phase,
                       const std::string &reason,
                       const std::size_t guide_path_size,
                       const std::size_t sfc_size,
                       const std::size_t target_prediction_size,
                       const double out_traj_duration) {
                    setTrackingDiagnostic(phase,
                                          reason,
                                          guide_path_size,
                                          sfc_size,
                                          target_prediction_size,
                                          out_traj_duration);
                };
        services.set_goal_info =
                [this](const Vec3f &goal, const double yaw, const bool new_task) {
                    gi_.goal_p = goal;
                    gi_.goal_yaw = yaw;
                    gi_.new_goal = new_task;
                };
        services.maybe_reset_tracking_runtime =
                [this](const bool new_task, const std::string &context) {
                    maybeResetTrackingRuntimeForReplan(new_task, context);
                };
        services.optimize_perching_task =
                [this](const traj_opt::PerchingSurfaceState &surface,
                       const bool from_rest) {
                    return optimizePerchingTask(surface, from_rest);
                };
        services.commit_perching_from_tracking =
                [this](const traj_opt::DynamicTargetStates &target_prediction,
                       const traj_opt::PerchingSurfaceState &surface,
                       const RET_CODE tracking_ret) {
                    return tryCommitPerchingFromTracking(target_prediction,
                                                         surface,
                                                         tracking_ret);
                };
        return services;
    }

    tracking_task::TrackingBackendServices GeneralPlanner::makeTrackingBackendServices() {
        tracking_task::TrackingBackendServices services{
                tracking_task::resolveTrackingBackend(cfg_),
                *tracking_backend_runtime_
        };
        return services;
    }

    namespace tracking_task {

        RET_CODE planFromRest(TrackingTaskServices &services,
                              TrackingBackendServices &backend_services,
                              const traj_opt::DynamicTargetStates &target_prediction,
                              const bool new_task) {
            TimeConsuming total_t("PlanTrackingFromRest", false);
            std::lock_guard<std::mutex> guard(services.replan_lock);
            services.latest_replan.reset();
            services.set_tracking_diagnostic("plan_from_rest",
                                             "start",
                                             0,
                                             0,
                                             target_prediction.size(),
                                             0.0);
            if (!services.robot_state.rcv) {
                services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
                services.set_tracking_diagnostic("input",
                                                 "no_odom",
                                                 0,
                                                 0,
                                                 target_prediction.size(),
                                                 0.0);
                services.ros_ptr->warn(" -- [GeneralPlanner] in [PlanTrackingFromRest]: No odom, force return.");
                return FAILED;
            }
            const Vec3f goal = target_prediction.empty()
                                   ? services.robot_state.p
                                   : target_prediction.back().position;
            const double yaw = target_prediction.empty() ? NAN : target_prediction.back().yaw;
            services.latest_replan.setGoal(goal, yaw, services.robot_state);
            services.set_goal_info(goal, yaw, new_task);
            services.last_exp_traj_info.setEmpty();
            services.maybe_reset_tracking_runtime(new_task, "tracking_plan_from_rest");
            if (new_task &&
                services.tracking_perching_manager &&
                !services.tracking_perching_manager->perchingRequested() &&
                !trackingPerchingPerchingStatus(services.tracking_perching_manager->status())) {
                services.tracking_perching_manager->reset();
            }

            const RET_CODE ret = runTrackingBackend(backend_services, target_prediction, true);
            services.time_consuming[TOTAL_REPLAN] = total_t.stop();
            return ret;
        }

        RET_CODE replanOnce(TrackingTaskServices &services,
                            TrackingBackendServices &backend_services,
                            const traj_opt::DynamicTargetStates &target_prediction,
                            const bool new_task) {
            TimeConsuming total_t("ReplanTrackingOnce", false);
            std::lock_guard<std::mutex> guard(services.replan_lock);
            services.latest_replan.reset();
            services.set_tracking_diagnostic("replan",
                                             "start",
                                             0,
                                             0,
                                             target_prediction.size(),
                                             0.0);
            if (!services.robot_state.rcv) {
                services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
                services.set_tracking_diagnostic("input",
                                                 "no_odom",
                                                 0,
                                                 0,
                                                 target_prediction.size(),
                                                 0.0);
                return FAILED;
            }
            if (services.tracking_perching_manager &&
                (services.tracking_perching_manager->status() ==
                     TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED ||
                 services.tracking_perching_manager->status() ==
                     TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING ||
                 services.tracking_perching_manager->status() ==
                     TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT)) {
                services.latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
                services.set_tracking_diagnostic("tracking_perching",
                                                 "perching_owns_committed_trajectory",
                                                 0,
                                                 0,
                                                 target_prediction.size(),
                                                 0.0);
                services.ros_ptr->info(" -- [TrackingPerching] TRACKING_REPLAN_BLOCKED_PERCHING_ACTIVE reason=perching_owns_committed_trajectory");
                services.time_consuming[TOTAL_REPLAN] = total_t.stop();
                return SUCCESS;
            }
            const Vec3f goal = target_prediction.empty()
                                   ? services.robot_state.p
                                   : target_prediction.back().position;
            const double yaw = target_prediction.empty() ? NAN : target_prediction.back().yaw;
            services.latest_replan.setGoal(goal, yaw, services.robot_state);
            services.set_goal_info(goal, yaw, new_task);
            services.maybe_reset_tracking_runtime(new_task, "tracking_replan");

            const RET_CODE ret = runTrackingBackend(backend_services, target_prediction, false);
            services.time_consuming[TOTAL_REPLAN] = total_t.stop();
            return ret;
        }

        RET_CODE replanWithPerchingSurface(
                TrackingTaskServices &services,
                TrackingBackendServices &backend_services,
                const traj_opt::DynamicTargetStates &target_prediction,
                const traj_opt::PerchingSurfaceState &surface,
                const bool new_task) {
            TimeConsuming total_t("ReplanTrackingOnceTrackingPerching", false);
            std::lock_guard<std::mutex> guard(services.replan_lock);
            services.latest_replan.reset();
            services.set_tracking_diagnostic("replan",
                                             "start",
                                             0,
                                             0,
                                             target_prediction.size(),
                                             0.0);
            if (!services.robot_state.rcv) {
                services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
                services.set_tracking_diagnostic("input",
                                                 "no_odom",
                                                 0,
                                                 0,
                                                 target_prediction.size(),
                                                 0.0);
                return FAILED;
            }
            const Vec3f goal = target_prediction.empty()
                                   ? surface.position
                                   : target_prediction.back().position;
            const double yaw = target_prediction.empty()
                                   ? surface.yaw
                                   : target_prediction.back().yaw;
            services.latest_replan.setGoal(goal, yaw, services.robot_state);
            services.set_goal_info(goal, yaw, new_task);

            if (services.tracking_perching_manager &&
                (services.tracking_perching_manager->status() ==
                     TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED ||
                 services.tracking_perching_manager->status() ==
                     TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING ||
                 services.tracking_perching_manager->status() ==
                     TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT)) {
                const RET_CODE ret = services.optimize_perching_task(surface, false);
                services.time_consuming[TOTAL_REPLAN] = total_t.stop();
                return ret;
            }

            services.maybe_reset_tracking_runtime(new_task, "tracking_perching_replan");

            const RET_CODE tracking_ret = runTrackingBackend(backend_services, target_prediction, false);
            const RET_CODE ret = services.commit_perching_from_tracking(target_prediction,
                                                                        surface,
                                                                        tracking_ret);
            services.time_consuming[TOTAL_REPLAN] = total_t.stop();
            return ret;
        }

    } // namespace tracking_task

    RET_CODE GeneralPlanner::PlanTrackingFromRest(
            const traj_opt::DynamicTargetStates &target_prediction,
            const bool &new_task) {
        auto services = makeTrackingTaskServices();
        auto backend_services = makeTrackingBackendServices();
        return tracking_task::planFromRest(services, backend_services, target_prediction, new_task);
    }

    RET_CODE GeneralPlanner::ReplanTrackingOnce(
            const traj_opt::DynamicTargetStates &target_prediction,
            const bool &new_task) {
        auto services = makeTrackingTaskServices();
        auto backend_services = makeTrackingBackendServices();
        return tracking_task::replanOnce(services, backend_services, target_prediction, new_task);
    }

    RET_CODE GeneralPlanner::ReplanTrackingOnce(
            const traj_opt::DynamicTargetStates &target_prediction,
            const traj_opt::PerchingSurfaceState &surface,
            const bool &new_task) {
        auto services = makeTrackingTaskServices();
        auto backend_services = makeTrackingBackendServices();
        return tracking_task::replanWithPerchingSurface(services,
                                                        backend_services,
                                                        target_prediction,
                                                        surface,
                                                        new_task);
    }

    void GeneralPlanner::setTrackingPerchingRequest(const bool request) {
        if (tracking_perching_manager_) {
            tracking_perching_manager_->setPerchingRequest(request);
        }
    }

} // namespace general_planner
