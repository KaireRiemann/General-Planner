/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

#include <mutex>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {

    TakeoffFrontend::Config GeneralPlanner::makeTakeoffFrontendConfig() const {
        TakeoffFrontend::Config frontend_cfg;
        frontend_cfg.robot_l = cfg_.takeoff_robot_l;
        frontend_cfg.robot_radius = cfg_.takeoff_robot_radius;
        frontend_cfg.platform_radius = cfg_.takeoff_platform_radius;
        frontend_cfg.platform_clearance = cfg_.takeoff_platform_clearance;
        frontend_cfg.platform_clearance_after_release =
                cfg_.takeoff_platform_clearance_after_release;
        frontend_cfg.release_contact_time = cfg_.takeoff_release_contact_time;
        frontend_cfg.escape_distance = cfg_.takeoff_escape_distance;
        frontend_cfg.escape_height = cfg_.takeoff_escape_height;
        frontend_cfg.reference_speed = cfg_.takeoff_reference_speed;
        frontend_cfg.min_duration = cfg_.takeoff_min_duration;
        frontend_cfg.max_duration = cfg_.takeoff_max_duration;
        frontend_cfg.piece_num = cfg_.takeoff_piece_num;
        frontend_cfg.frontend_astar = cfg_.takeoff_frontend_astar;
        frontend_cfg.safe_distance = cfg_.takeoff_safe_distance;
        frontend_cfg.use_tangent_release_velocity =
                cfg_.takeoff_use_tangent_release_velocity;
        frontend_cfg.thrust_nominal = cfg_.perching_thrust_nominal;
        frontend_cfg.thrust_range = cfg_.perching_thrust_range;
        frontend_cfg.gravity = cfg_.esdf_traj_cfg.grav;
        frontend_cfg.weight_eta = cfg_.takeoff_weight_eta;
        frontend_cfg.weight_tau_f = cfg_.takeoff_weight_tau_f;
        frontend_cfg.rotate_surface_with_yaw_rate =
                cfg_.perching_rotate_surface_with_yaw_rate;
        return frontend_cfg;
    }

    RET_CODE GeneralPlanner::PlanDynamicTakeoffFromRest(
            const traj_opt::PerchingSurfaceState &surface,
            const bool &new_task) {
        TimeConsuming total_t("PlanDynamicTakeoffFromRest", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            ros_ptr_->warn(" -- [GeneralPlanner] in [PlanDynamicTakeoffFromRest]: No odom, force return.");
            return FAILED;
        }
        latest_replan.setGoal(surface.position, surface.yaw, robot_state_);
        gi_.goal_p = surface.position;
        gi_.goal_yaw = surface.yaw;
        gi_.new_goal = new_task;
        last_exp_traj_info_.setEmpty();
        if (takeoff_runtime_manager_) {
            takeoff_runtime_manager_->reset();
        }
        active_takeoff_problem_valid_ = false;

        const RET_CODE ret = optimizeDynamicTakeoffTask(surface, true);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::ReplanDynamicTakeoffOnce(
            const traj_opt::PerchingSurfaceState &surface,
            const bool &new_task) {
        TimeConsuming total_t("ReplanDynamicTakeoffOnce", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            return FAILED;
        }
        latest_replan.setGoal(surface.position, surface.yaw, robot_state_);
        gi_.goal_p = surface.position;
        gi_.goal_yaw = surface.yaw;
        gi_.new_goal = new_task;
        if (!new_task &&
            takeoff_runtime_manager_ &&
            takeoff_runtime_manager_->hasCommittedTakeoff() &&
            !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            const bool has_active_traj = !cmd_traj_info_.posTraj().empty();
            cmd_traj_info_.unlock();
            const double local_t = ros_ptr_->getSimTime() - start_wt;
            if (has_active_traj && local_t >= 0.0 && local_t <= total_dur) {
                latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
                time_consuming_[TOTAL_REPLAN] = total_t.stop();
                return SUCCESS;
            }
        }
        if (new_task && takeoff_runtime_manager_) {
            takeoff_runtime_manager_->reset();
            active_takeoff_problem_valid_ = false;
        }

        const RET_CODE ret = optimizeDynamicTakeoffTask(surface, false);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::optimizeDynamicTakeoffTask(
            const traj_opt::PerchingSurfaceState &surface,
            const bool &from_rest) {
        (void)from_rest;
        if (takeoff_frontend_ == nullptr || takeoff_optimizer_ == nullptr) {
            ros_ptr_->warn(" -- [Takeoff] TAKEOFF_CANDIDATE_REJECTED reason=module_not_initialized");
            return FAILED;
        }

        traj_opt::DynamicTakeoffProblem problem;
        TimeConsuming t_frontend("takeoff_frontend", false);
        if (!takeoff_frontend_->buildProblem(surface, problem)) {
            time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
            ros_ptr_->warn(" -- [Takeoff] TAKEOFF_CANDIDATE_REJECTED reason=frontend_failed");
            return FAILED;
        }
        time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path,
                                      problem.nominal_head_pvaj,
                                      problem.tail_pvaj,
                                      PolytopeVec());

        {
            TimeConsuming t_viz("takeoff_frontend_viz", false);
            ros_ptr_->vizFrontendPath(problem.guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        Trajectory out_traj;
        TimeConsuming t_opt("takeoff_opt", false);
        const bool ok = takeoff_optimizer_->optimize(problem, out_traj);
        time_consuming_[EXP_TRAJ_OPT] = t_opt.stop();
        if (!ok || out_traj.empty()) {
            ros_ptr_->warn(" -- [Takeoff] TAKEOFF_OPT_FAILED");
            return FAILED;
        }

        const auto candidate_check =
                takeoff_runtime_manager_
                    ? takeoff_runtime_manager_->checkCandidate(out_traj, problem)
                    : TakeoffRuntimeManager::CheckResult{};
        const bool accepted =
                takeoff_runtime_manager_
                    ? takeoff_runtime_manager_->decideCommit(candidate_check)
                    : true;
        ros_ptr_->info(" -- [Takeoff] candidate_check valid={}, safe={}, dynamics_feasible={}, platform_clear_after_release={}, terminal_escape_valid={}, max_thrust={:.3f}, max_omega={:.3f}, esdf_min={:.3f}, platform_margin_min={:.3f}, reason={}",
                       candidate_check.valid,
                       candidate_check.safe,
                       candidate_check.dynamics_feasible,
                       candidate_check.platform_clear_after_release,
                       candidate_check.terminal_escape_valid,
                       candidate_check.max_thrust,
                       candidate_check.max_omega,
                       candidate_check.min_esdf_clearance,
                       candidate_check.min_platform_margin_after_release,
                       candidate_check.reason);
        if (!accepted) {
            ros_ptr_->warn(" -- [Takeoff] TAKEOFF_CANDIDATE_REJECTED reason={}",
                           candidate_check.reason);
            return FAILED;
        }

        if (!commitTakeoffTrajectory(out_traj, "dynamic_takeoff")) {
            return FAILED;
        }
        active_takeoff_problem_ = problem;
        active_takeoff_problem_valid_ = true;
        if (takeoff_runtime_manager_) {
            takeoff_runtime_manager_->updateStatusByPosition(out_traj.getPos(0.0), problem);
        }
        ros_ptr_->info(" -- [GeneralPlanner] Dynamic takeoff task success: pieces={}, duration={}.",
                       out_traj.getPieceNum(), out_traj.getTotalDuration());
        return SUCCESS;
    }

}
