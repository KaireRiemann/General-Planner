/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <general_utils/scope_timer.hpp>
#include <utils/geometry/geometry_utils.h>

using namespace general_utils;

namespace general_planner {

    PerchingFrontend::Config GeneralPlanner::makePerchingFrontendConfig() const {
        PerchingFrontend::Config frontend_cfg;
        frontend_cfg.robot_l = cfg_.perching_robot_l;
        frontend_cfg.v_plus = cfg_.perching_v_plus;
        frontend_cfg.pre_contact_distance = cfg_.perching_pre_contact_distance;
        frontend_cfg.terminal_relax_time = cfg_.perching_terminal_relax_time;
        frontend_cfg.safe_distance = cfg_.perching_safe_distance;
        frontend_cfg.platform_radius = cfg_.perching_platform_radius;
        frontend_cfg.robot_radius = cfg_.perching_robot_radius;
        frontend_cfg.platform_clearance = cfg_.perching_platform_clearance;
        frontend_cfg.thrust_nominal = cfg_.perching_thrust_nominal;
        frontend_cfg.thrust_range = cfg_.perching_thrust_range;
        frontend_cfg.weight_nu = cfg_.perching_weight_nu;
        frontend_cfg.weight_tau_f = cfg_.perching_weight_tau_f;
        frontend_cfg.min_duration = cfg_.perching_min_duration;
        frontend_cfg.max_duration = cfg_.perching_max_duration;
        frontend_cfg.reference_speed = cfg_.perching_reference_speed;
        frontend_cfg.max_speed = cfg_.esdf_traj_cfg.max_vel;
        if (cfg_.esdf_traj_cfg.max_acc > 0.0) {
            frontend_cfg.max_acc = cfg_.esdf_traj_cfg.max_acc;
        }
        if (cfg_.esdf_traj_cfg.max_jerk > 0.0) {
            frontend_cfg.max_jerk = cfg_.esdf_traj_cfg.max_jerk;
        }
        if (cfg_.esdf_traj_cfg.max_omg > 0.0) {
            frontend_cfg.max_omega = cfg_.esdf_traj_cfg.max_omg;
        }
        frontend_cfg.relative_z_min = cfg_.perching_relative_z_min;
        frontend_cfg.relative_z_max = cfg_.perching_relative_z_max;
        frontend_cfg.weight_relative_height = cfg_.perching_weight_relative_height;
        frontend_cfg.visual_min_distance = cfg_.perching_visual_min_distance;
        frontend_cfg.visual_activation_distance = cfg_.perching_visual_activation_distance;
        frontend_cfg.visual_fx = cfg_.perching_visual_fx;
        frontend_cfg.visual_fy = cfg_.perching_visual_fy;
        frontend_cfg.gravity = cfg_.esdf_traj_cfg.grav;
        frontend_cfg.searching_horizon = cfg_.planning_horizon;
        frontend_cfg.piece_num = std::max(2, cfg_.esdf_traj_cfg.piece_num);
        frontend_cfg.min_piece_duration = std::max(0.05, cfg_.perching_min_duration /
                                                         static_cast<double>(std::max(2, frontend_cfg.piece_num)));
        frontend_cfg.min_total_duration = cfg_.perching_min_duration;
        frontend_cfg.max_total_duration = cfg_.perching_max_duration;
        frontend_cfg.time_lower_bound_weight = std::max(100.0, std::abs(cfg_.esdf_traj_cfg.penna_t) * 10.0);
        frontend_cfg.time_upper_bound_weight = cfg_.perching_time_upper_bound_weight;
        frontend_cfg.duration_seed_weight = cfg_.perching_duration_seed_weight;
        frontend_cfg.duration_margin = cfg_.perching_duration_margin;
        frontend_cfg.allow_long_standalone = cfg_.perching_allow_long_standalone;
        frontend_cfg.max_piece_duration = cfg_.perching_max_piece_duration;
        frontend_cfg.min_piece_num = cfg_.perching_min_piece_num;
        frontend_cfg.max_piece_num = cfg_.perching_max_piece_num;
        frontend_cfg.multi_point_guide_enable = cfg_.perching_multi_point_guide_enable;
        frontend_cfg.moving_guide_sample_num = cfg_.perching_moving_guide_sample_num;
        frontend_cfg.tau_f_seed_limit = cfg_.perching_tau_f_seed_limit;
        frontend_cfg.reset_surface_time = cfg_.perching_reset_surface_time;
        frontend_cfg.use_astar = cfg_.perching_frontend_astar;
        frontend_cfg.use_dynamics_terminal_accel = cfg_.perching_use_dynamics_terminal_accel;
        frontend_cfg.rotate_surface_with_yaw_rate = cfg_.perching_rotate_surface_with_yaw_rate;
        return frontend_cfg;
    }

    RET_CODE GeneralPlanner::PlanPerchingFromRest(
            const traj_opt::PerchingSurfaceState &surface,
            const bool &new_task) {
        TimeConsuming total_t("PlanPerchingFromRest", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            ros_ptr_->warn(" -- [GeneralPlanner] in [PlanPerchingFromRest]: No odom, force return.");
            return FAILED;
        }
        latest_replan.setGoal(surface.position, surface.yaw, robot_state_);
        gi_.goal_p = surface.position;
        gi_.goal_yaw = surface.yaw;
        gi_.new_goal = new_task;
        last_exp_traj_info_.setEmpty();
        if (perching_runtime_manager_) {
            perching_runtime_manager_->reset();
        }

        const RET_CODE ret = optimizePerchingTask(surface, true);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::ReplanPerchingOnce(
            const traj_opt::PerchingSurfaceState &surface,
            const bool &new_task) {
        TimeConsuming total_t("ReplanPerchingOnce", false);
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
            perching_runtime_manager_ &&
            perching_runtime_manager_->hasCommittedPerching() &&
            !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            const bool has_active_traj = !cmd_traj_info_.posTraj().empty();
            cmd_traj_info_.unlock();
            const double local_t = ros_ptr_->getSimTime() - start_wt;
            const double keep_until =
                    total_dur + std::max(0.0, cfg_.perching_contact_time_margin);
            if (has_active_traj && local_t >= 0.0 && local_t <= keep_until) {
                latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
                time_consuming_[TOTAL_REPLAN] = total_t.stop();
                return SUCCESS;
            }
        }
        if (new_task && perching_runtime_manager_) {
            perching_runtime_manager_->reset();
        }

        const RET_CODE ret = optimizePerchingTask(surface, false);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    bool GeneralPlanner::buildPerchingYawTrajectory(const Trajectory &pos_traj,
                                                    const traj_opt::PerchingSurfaceState &surface,
                                                    Trajectory &yaw_traj) {
        if (pos_traj.empty()) {
            return false;
        }

        Vec4f init_yaw{robot_state_.yaw, 0.0, 0.0, 0.0};
        if (!cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory committed_yaw_traj = cmd_traj_info_.yawTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            const double eval_t = ros_ptr_->getSimTime() - start_wt;
            StatePVAJ yaw_state;
            if (!committed_yaw_traj.empty() && eval_t >= 0.0 && eval_t <= total_dur &&
                committed_yaw_traj.getState(eval_t, yaw_state)) {
                init_yaw = yaw_state.row(0);
            }
        }

        Vec4f terminal_yaw{0.0, 0.0, 0.0, 0.0};
        terminal_yaw[0] = surface.yaw + surface.yaw_rate * pos_traj.getTotalDuration();
        geometry_utils::normalizeNextYaw(init_yaw[0], terminal_yaw[0]);
        terminal_yaw[1] = surface.yaw_rate;

        if (!traj_manager_->yaw()->optimize(init_yaw,
                                            terminal_yaw,
                                            pos_traj,
                                            yaw_traj,
                                            3,
                                            false,
                                            false)) {
            return false;
        }
        yaw_traj.start_WT = pos_traj.start_WT;
        return !yaw_traj.empty();
    }

    bool GeneralPlanner::buildPerchingYawTrajectoryFromHead(
            const Trajectory &pos_traj,
            const traj_opt::PerchingSurfaceState &surface,
            const Eigen::Matrix<double, 1, 2> &head_yaw,
            Trajectory &yaw_traj) {
        if (pos_traj.empty()) {
            return false;
        }

        Vec4f init_yaw{head_yaw(0, 0), head_yaw(0, 1), 0.0, 0.0};
        Vec4f terminal_yaw{0.0, 0.0, 0.0, 0.0};
        terminal_yaw[0] = surface.yaw + surface.yaw_rate * pos_traj.getTotalDuration();
        geometry_utils::normalizeNextYaw(init_yaw[0], terminal_yaw[0]);
        terminal_yaw[1] = surface.yaw_rate;

        if (!traj_manager_->yaw()->optimize(init_yaw,
                                            terminal_yaw,
                                            pos_traj,
                                            yaw_traj,
                                            3,
                                            false,
                                            false)) {
            return false;
        }
        yaw_traj.start_WT = pos_traj.start_WT;
        return !yaw_traj.empty();
    }

    bool GeneralPlanner::commitPerchingTrajectory(const Trajectory &pos_traj,
                                                  const Trajectory &yaw_traj,
                                                  const std::string &traj_ns) {
        if (pos_traj.empty() || yaw_traj.empty()) {
            ros_ptr_->warn(" -- [Perching] PERCHING_CANDIDATE_REJECTED reason=empty_pos_or_yaw");
            return false;
        }

        Trajectory committed_pos = pos_traj;
        Trajectory committed_yaw = yaw_traj;
        const double commit_wt = ros_ptr_->getSimTime();
        committed_pos.start_WT = commit_wt;
        committed_yaw.start_WT = commit_wt;

        ExpTraj perching_exp_traj;
        perching_exp_traj.setGoalConnectedFlag(true);
        perching_exp_traj.setWholeTrajKnownFreeFlag(true);
        perching_exp_traj.setTrajectory(commit_wt, committed_pos, committed_yaw);

        cmd_traj_info_.setTrajectory(perching_exp_traj);
        last_exp_traj_info_ = perching_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        {
            TimeConsuming t_viz("perching_task_viz", false);
            ros_ptr_->vizExpTraj(committed_pos, traj_ns);
            ros_ptr_->vizYawTraj(committed_pos, committed_yaw);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        latest_replan.setExpTraj(committed_pos);
        latest_replan.setExpYawTraj(committed_yaw);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        if (perching_runtime_manager_) {
            perching_runtime_manager_->updateStatusAfterCommit();
        }
        return true;
    }

    RET_CODE GeneralPlanner::optimizePerchingTask(const traj_opt::PerchingSurfaceState &surface,
                                                const bool &from_rest) {
        const PerchingFrontend::Config frontend_cfg = makePerchingFrontendConfig();

        traj_opt::PerchingProblem problem;
        TimeConsuming t_frontend("perching_frontend", false);
        PerchingFrontend frontend(frontend_cfg, map_manager_, astar_ptr_);
        StatePVAJ head_state = makeTaskHeadState(from_rest);
        if (!from_rest &&
            perching_runtime_manager_ &&
            perching_runtime_manager_->hasCommittedPerching() &&
            !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory committed_pos = cmd_traj_info_.posTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();
            const double eval_t = ros_ptr_->getSimTime() - start_wt + cfg_.replan_forward_dt;
            if (!committed_pos.empty() && eval_t >= 0.0 && eval_t <= total_dur) {
                head_state = committed_pos.getState(eval_t);
            }
        }

        if (!frontend.buildProblem(head_state, surface, problem)) {
            time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
            ros_ptr_->warn(" -- [Perching] PERCHING_CANDIDATE_REJECTED reason=frontend_failed");
            return FAILED;
        }
        time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj,
                                      problem.nominal_tail_pvaj, PolytopeVec());
        ros_ptr_->info(" -- [Perching] PERCHING_BUILD_PROBLEM_SUCCESS T0={:.3f}, piece_num={}, guide_size={}, nu_seed=[{:.3f},{:.3f}], tau_f_seed={:.3f}, max_total_duration={:.3f}",
                       problem.initial_guess.total_time,
                       problem.piece_num,
                       problem.guide_path.size(),
                       problem.initial_guess.nu.x(),
                       problem.initial_guess.nu.y(),
                       problem.initial_guess.tau_f,
                       problem.max_total_duration);

        auto checkCurrentPerching = [&](PerchingRuntimeManager::CheckResult &current_check) -> bool {
            if (!perching_runtime_manager_ ||
                !perching_runtime_manager_->hasCommittedPerching() ||
                cmd_traj_info_.empty()) {
                return false;
            }
            cmd_traj_info_.lock();
            const Trajectory current_pos = cmd_traj_info_.posTraj();
            const Trajectory current_yaw = cmd_traj_info_.yawTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            const double local_t = ros_ptr_->getSimTime() - start_wt;
            if (current_pos.empty() || local_t < 0.0 || local_t >= total_dur) {
                return false;
            }
            Trajectory partial_pos;
            Trajectory partial_yaw;
            if (!current_pos.getPartialTrajectoryByTime(local_t, total_dur, partial_pos)) {
                return false;
            }
            const bool has_partial_yaw =
                !current_yaw.empty() &&
                current_yaw.getPartialTrajectoryByTime(local_t,
                                                       std::min(total_dur, current_yaw.getTotalDuration()),
                                                       partial_yaw);
            current_check = perching_runtime_manager_->checkCandidate(
                partial_pos,
                has_partial_yaw ? &partial_yaw : nullptr,
                problem,
                problem.surface);
            return current_check.valid;
        };

        auto failOrKeepCurrent = [&](const std::string &reason) -> RET_CODE {
            PerchingRuntimeManager::CheckResult failed_candidate;
            failed_candidate.reason = reason;
            PerchingRuntimeManager::CheckResult current_check;
            const bool has_current = checkCurrentPerching(current_check);
            const auto decision =
                perching_runtime_manager_
                    ? perching_runtime_manager_->decideCommit(failed_candidate,
                                                              has_current ? &current_check : nullptr)
                    : PerchingRuntimeManager::DecisionType::REJECT;
            if (decision == PerchingRuntimeManager::DecisionType::KEEP_CURRENT_PERCHING) {
                ros_ptr_->info(" -- [Perching] PERCHING_KEEP_CURRENT_TRAJ reason={}, terminal_pos_err={:.3f}, terminal_vel_err={:.3f}, max_thrust={:.3f}, max_omega={:.3f}, esdf_min={:.3f}, platform_margin_min={:.3f}",
                               reason,
                               current_check.terminal_position_error,
                               current_check.terminal_velocity_error,
                               current_check.max_thrust,
                               current_check.max_omega,
                               current_check.min_esdf_clearance,
                               current_check.min_platform_margin);
                latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
                return SUCCESS;
            }
            ros_ptr_->warn(" -- [Perching] PERCHING_CANDIDATE_REJECTED reason={}, has_current={}, current_reason={}",
                           reason,
                           has_current,
                           has_current ? current_check.reason : "none");
            if (perching_runtime_manager_ &&
                reason != "repeat_infeasible_candidate") {
                perching_runtime_manager_->rememberRejectedCandidate(
                    problem,
                    reason,
                    ros_ptr_->getSimTime());
            }
            return FAILED;
        };

        if (perching_runtime_manager_) {
            std::string cached_reason;
            if (perching_runtime_manager_->shouldSkipRejectedCandidate(
                    problem,
                    ros_ptr_->getSimTime(),
                    &cached_reason)) {
                ros_ptr_->warn(" -- [Perching] PERCHING_SKIP_REPEATED_INFEASIBLE_CANDIDATE cached_reason={}",
                               cached_reason);
                return failOrKeepCurrent("repeat_infeasible_candidate");
            }
        }

        {
            TimeConsuming t_viz("perching_frontend_viz", false);
            ros_ptr_->vizFrontendPath(problem.guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        Trajectory out_traj;
        TimeConsuming t_opt("perching_opt", false);
        const bool ok = traj_manager_->perchingSnap()->optimize(problem, out_traj);
        time_consuming_[EXP_TRAJ_OPT] = t_opt.stop();
        if (!ok || out_traj.empty()) {
            ros_ptr_->warn(" -- [Perching] PERCHING_OPT_FAILED");
            return failOrKeepCurrent("optimization_failed");
        }
        ros_ptr_->info(" -- [Perching] PERCHING_OPT_SUCCESS optimized_duration={:.3f}",
                       out_traj.getTotalDuration());

        Trajectory yaw_traj;
        if (!buildPerchingYawTrajectory(out_traj, surface, yaw_traj)) {
            return failOrKeepCurrent("yaw_generation_failed");
        }

        const auto candidate_check =
            perching_runtime_manager_
                ? perching_runtime_manager_->checkCandidate(out_traj, &yaw_traj, problem, problem.surface)
                : PerchingRuntimeManager::CheckResult{};
        PerchingRuntimeManager::CheckResult current_check;
        const bool has_current = !from_rest && checkCurrentPerching(current_check);
        const auto decision =
            perching_runtime_manager_
                ? perching_runtime_manager_->decideCommit(candidate_check,
                                                          has_current ? &current_check : nullptr)
                : PerchingRuntimeManager::DecisionType::COMMIT_CANDIDATE;

        ros_ptr_->info(" -- [Perching] candidate_check valid={}, safe={}, terminal_sync={}, dynamics_feasible={}, contact_imminent={}, terminal_pos_err={:.3f}, terminal_vel_err={:.3f}, max_thrust={:.3f}, max_omega={:.3f}, esdf_min={:.3f}, platform_margin_min={:.3f}, reason={}",
                       candidate_check.valid,
                       candidate_check.safe,
                       candidate_check.terminal_sync,
                       candidate_check.dynamics_feasible,
                       candidate_check.contact_imminent,
                       candidate_check.terminal_position_error,
                       candidate_check.terminal_velocity_error,
                       candidate_check.max_thrust,
                       candidate_check.max_omega,
                       candidate_check.min_esdf_clearance,
                       candidate_check.min_platform_margin,
                       candidate_check.reason);

        if (decision == PerchingRuntimeManager::DecisionType::KEEP_CURRENT_PERCHING) {
            ros_ptr_->info(" -- [Perching] PERCHING_KEEP_CURRENT_TRAJ reason=candidate_rejected_current_contact_imminent");
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        if (decision != PerchingRuntimeManager::DecisionType::COMMIT_CANDIDATE) {
            ros_ptr_->warn(" -- [Perching] PERCHING_CANDIDATE_REJECTED reason={}",
                           candidate_check.reason);
            if (perching_runtime_manager_) {
                perching_runtime_manager_->rememberRejectedCandidate(
                    problem,
                    candidate_check.reason,
                    ros_ptr_->getSimTime());
            }
            return FAILED;
        }

        ros_ptr_->info(" -- [Perching] PERCHING_CANDIDATE_ACCEPTED optimized_duration={:.3f}, terminal_pos_err={:.3f}, terminal_vel_err={:.3f}, max_thrust={:.3f}, max_omega={:.3f}, esdf_min={:.3f}, platform_margin_min={:.3f}",
                       out_traj.getTotalDuration(),
                       candidate_check.terminal_position_error,
                       candidate_check.terminal_velocity_error,
                       candidate_check.max_thrust,
                       candidate_check.max_omega,
                       candidate_check.min_esdf_clearance,
                       candidate_check.min_platform_margin);
        if (candidate_check.contact_imminent) {
            ros_ptr_->info(" -- [Perching] PERCHING_CONTACT_IMMINENT");
        }

        if (!commitPerchingTrajectory(out_traj, yaw_traj, "perching")) {
            return FAILED;
        }
        ros_ptr_->info(" -- [GeneralPlanner] Perching task success: pieces={}, duration={}.",
                       out_traj.getPieceNum(), out_traj.getTotalDuration());
        return SUCCESS;
    }

}
