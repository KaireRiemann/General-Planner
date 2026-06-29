/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_plan_operations.hpp>

#include <algorithm>
#include <mutex>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {

    RET_CODE GeneralPlanner::tryCommitPerchingFromTracking(
            const traj_opt::DynamicTargetStates &target_prediction,
            const traj_opt::PerchingSurfaceState &surface,
            const RET_CODE tracking_ret) {
        if (!cfg_.tracking_perching_enable || !tracking_perching_manager_) {
            return tracking_ret;
        }
        if (tracking_ret != SUCCESS && tracking_ret != NO_NEED) {
            return tracking_ret;
        }
        if (cfg_.tracking_perching_require_external_request &&
            !tracking_perching_manager_->perchingRequested()) {
            return tracking_ret;
        }

        Trajectory tracking_pos;
        Trajectory tracking_yaw;
        double tracking_start_wt = 0.0;
        double tracking_total_t = 0.0;
        if (cmd_traj_info_.empty()) {
            ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_KEEP_TRACKING reason=no_committed_tracking");
            return tracking_ret;
        }
        cmd_traj_info_.lock();
        tracking_pos = cmd_traj_info_.posTraj();
        tracking_yaw = cmd_traj_info_.yawTraj();
        tracking_start_wt = cmd_traj_info_.getStartWallTime();
        tracking_total_t = cmd_traj_info_.getTotalDuration();
        cmd_traj_info_.unlock();
        if (tracking_pos.empty()) {
            ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_KEEP_TRACKING reason=empty_committed_tracking");
            return tracking_ret;
        }

        const double now = ros_ptr_->getSimTime();
        const double tracking_local_t =
                std::clamp(now - tracking_start_wt, 0.0, tracking_total_t);
        const bool tracking_active =
                currentTrackingTrajectorySafeAndActive(target_prediction, nullptr);
        const auto readiness =
                tracking_perching_manager_->evaluateReadiness(tracking_active,
                                                              tracking_pos,
                                                              tracking_yaw,
                                                              tracking_local_t,
                                                              surface,
                                                              cfg_);
        if (!readiness.ready) {
            ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_KEEP_TRACKING reason={}, ready_count={}, distance={:.3f}, relative_speed={:.3f}, lateral_speed={:.3f}, estimated_duration={:.3f}",
                           readiness.reason,
                           readiness.ready_count,
                           readiness.distance,
                           readiness.relative_speed,
                           readiness.lateral_speed,
                           readiness.estimated_duration);
            return tracking_ret;
        }

        if (!tracking_to_perching_initializer_) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_FAILED reason=missing_initializer");
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        TrackingToPerchingInitialGuess init_guess;
        if (!tracking_to_perching_initializer_->build(tracking_pos,
                                                      tracking_yaw,
                                                      tracking_local_t,
                                                      surface,
                                                      cfg_,
                                                      init_guess)) {
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        tracking_perching_manager_->onCandidateTesting();

        const PerchingFrontend::Config frontend_cfg = makePerchingFrontendConfig();
        PerchingFrontend frontend(frontend_cfg, map_manager_, astar_ptr_);
        traj_opt::PerchingProblem problem;
        problem.use_tracking_warm_start = true;
        problem.init_total_time = init_guess.total_time;
        problem.init_nu = init_guess.nu_seed;
        problem.init_tau_f = init_guess.tau_f_seed;
        problem.warm_start_guide_path = init_guess.guide_path;
        problem.warm_start_guide_t = init_guess.guide_t;
        problem.warm_start_head_yaw = init_guess.head_yaw;

        if (!frontend.buildProblem(init_guess.head_pvaj,
                                   init_guess.rebased_surface,
                                   problem)) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_FAILED reason=frontend_failed");
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj,
                                      problem.nominal_tail_pvaj, PolytopeVec());

        Trajectory perching_pos;
        TimeConsuming t_opt("tracking_to_perching_opt", false);
        const bool opt_ok = traj_manager_->perchingSnap()->optimize(problem, perching_pos);
        const double opt_t = t_opt.stop();
        time_consuming_[EXP_TRAJ_OPT] += opt_t;
        if (!opt_ok || perching_pos.empty()) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason=optimization_failed");
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }
        ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_OPT_SUCCESS duration={:.3f}, opt_time={:.4f}",
                       perching_pos.getTotalDuration(),
                       opt_t);

        Trajectory perching_yaw;
        if (!buildPerchingYawTrajectoryFromHead(perching_pos,
                                                problem.surface,
                                                init_guess.head_yaw,
                                                perching_yaw)) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason=yaw_generation_failed");
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        const auto candidate_check =
                perching_runtime_manager_
                    ? perching_runtime_manager_->checkCandidate(perching_pos,
                                                                &perching_yaw,
                                                                problem,
                                                                problem.surface)
                    : PerchingRuntimeManager::CheckResult{};
        const bool candidate_accepted =
                perching_runtime_manager_ &&
                perching_runtime_manager_->candidateAccepted(candidate_check);
        if (!candidate_accepted) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason={}, valid={}, safe={}, terminal_sync={}, dynamics_feasible={}, terminal_pos_err={:.3f}, terminal_vel_err={:.3f}, max_thrust={:.3f}, max_omega={:.3f}",
                           candidate_check.reason,
                           candidate_check.valid,
                           candidate_check.safe,
                           candidate_check.terminal_sync,
                           candidate_check.dynamics_feasible,
                           candidate_check.terminal_position_error,
                           candidate_check.terminal_velocity_error,
                           candidate_check.max_thrust,
                           candidate_check.max_omega);
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        if (!commitTrackingToPerchingTrajectory(tracking_pos,
                                                tracking_yaw,
                                                tracking_local_t,
                                                init_guess.handover_delay,
                                                perching_pos,
                                                perching_yaw,
                                                "tracking_to_perching")) {
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        tracking_perching_manager_->onPerchingCommitted();
        return SUCCESS;
    }

    namespace tracking_task {

        RET_CODE tryCommitPerchingFromTracking(
                TrackingTaskServices &services,
                const traj_opt::DynamicTargetStates &target_prediction,
                const traj_opt::PerchingSurfaceState &surface,
                const RET_CODE tracking_ret) {
            TimeConsuming total_t("TryCommitPerchingFromTracking", false);
            std::lock_guard<std::mutex> guard(services.replan_lock);
            const RET_CODE ret =
                    services.commit_perching_from_tracking(target_prediction, surface, tracking_ret);
            services.time_consuming[TOTAL_REPLAN] += total_t.stop();
            return ret;
        }

    } // namespace tracking_task

    RET_CODE GeneralPlanner::TryCommitPerchingFromTracking(
            const traj_opt::DynamicTargetStates &target_prediction,
            const traj_opt::PerchingSurfaceState &surface,
            const RET_CODE tracking_ret) {
        auto services = makeTrackingTaskServices();
        return tracking_task::tryCommitPerchingFromTracking(services,
                                                            target_prediction,
                                                            surface,
                                                            tracking_ret);
    }

}
