/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/state2state/state2state_path_utils.hpp>
#include <checker/state2state_checker.hpp>
#include <checker/trajectory_checker.hpp>
#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {
    state2state_task::StateToStateTaskServices GeneralPlanner::makeStateToStateTaskServices() {
        state2state_task::StateToStateTaskServices services{
                replan_lock_,
                latest_replan,
                cfg_,
                map_manager_,
                ros_ptr_,
                robot_state_,
                cmd_traj_info_,
                last_exp_traj_info_,
                local_start_p_,
                robot_on_backup_traj_,
                time_consuming_,
                ft,
                ft_cnt,
                bt,
                bt_cnt,
                *state2state_backend_context_
        };
        return services;
    }

    state2state_task::StateToStateFrontendServices GeneralPlanner::makeStateToStateFrontendServices() {
        state2state_task::StateToStateFrontendServices services{
                cfg_,
                map_manager_,
                ros_ptr_,
                astar_ptr_,
                dynamic_obstacle_layer_.get(),
                local_start_p_,
                gi_.goal_p,
                gi_.goal_valid
        };
        return services;
    }

    state2state_task::StateToStateExpBackendServices GeneralPlanner::makeStateToStateExpBackendServices() {
        state2state_task::StateToStateExpBackendServices services{
                makeStateToStateFrontendServices(),
                RuntimeTrajectorySafetyServices{
                        map_manager_,
                        dynamic_obstacle_layer_.get()
                },
                cfg_,
                map_manager_,
                cg_ptr_,
                ros_ptr_,
                traj_manager_,
                robot_state_,
                cmd_traj_info_,
                last_exp_traj_info_,
                local_start_p_,
                robot_on_backup_traj_,
                time_consuming_,
                shifted_sfc_start_pt_,
                latest_replan,
                gi_.goal_p,
                gi_.goal_yaw,
                gi_.new_goal,
                latest_state2state_z_debug_
        };
        return services;
    }

    state2state_task::StateToStateBackupBackendServices GeneralPlanner::makeStateToStateBackupBackendServices() {
        state2state_task::StateToStateBackupBackendServices services{
                cfg_,
                map_manager_,
                cg_ptr_,
                fov_checker_,
                ros_ptr_,
                traj_manager_,
                robot_state_,
                drone_state_mutex_,
                shifted_sfc_start_pt_,
                gi_.goal_yaw,
                latest_replan,
                cmd_traj_info_,
                time_consuming_
        };
        return services;
    }

    namespace {
        bool backupTrajectoryPlanningEnabled(const Config &cfg) {
            return cfg.backup_traj_en && !cfg.esdf_traj_en && !cfg.plain_traj_en;
        }

        bool goalConnectedExpStopsSafely(const Config &cfg,
                                         const ExpTraj &exp_traj,
                                         const Vec3f &goal_p) {
            if (!cfg.state2state_accept_exp_without_backup_near_goal ||
                exp_traj.empty() ||
                !exp_traj.connectedToGoal()) {
                return false;
            }

            const Trajectory &pos_traj = exp_traj.posTraj();
            if (pos_traj.empty()) {
                return false;
            }
            const double duration = pos_traj.getTotalDuration();
            if (!std::isfinite(duration) || duration <= 1.0e-4) {
                return false;
            }

            const Vec3f end_pos = pos_traj.getPos(duration);
            const Vec3f end_vel = pos_traj.getVel(duration);
            if (!end_pos.allFinite() || !end_vel.allFinite()) {
                return false;
            }

            const double goal_tolerance = std::max(cfg.resolution * 3.0, 0.3);
            const double stop_velocity_tolerance = 0.1;
            return (end_pos - goal_p).norm() <= goal_tolerance &&
                   end_vel.norm() <= stop_velocity_tolerance;
        }

        void logCheckResult(const ros_interface::RosInterface::Ptr &ros_ptr,
                            const std::string &context,
                            const checker::CheckResult &result) {
            if (result.severity == checker::Severity::OK || ros_ptr == nullptr) {
                return;
            }
            const std::string msg = fmt::format(" -- [Checker] {} [{}]: {}",
                                                context,
                                                result.code,
                                                result.message);
            if (result.severity == checker::Severity::WARN) {
                ros_ptr->warn(msg);
            } else {
                ros_ptr->error(msg);
            }
        }

        bool rejectOnCheckFailure(const ros_interface::RosInterface::Ptr &ros_ptr,
                                  const std::string &context,
                                  const checker::CheckResult &result) {
            logCheckResult(ros_ptr, context, result);
            return result.rejected();
        }

        void warnHighSpeedMargin(const ros_interface::RosInterface::Ptr &ros_ptr,
                                 const Config &cfg,
                                 const double speed,
                                 const std::string &context) {
            const auto result = checker::checkHighSpeedSafetyMargin(
                    speed,
                    cfg.exp_traj_cfg.max_acc,
                    cfg.replan_forward_dt,
                    cfg.sensing_horizon,
                    cfg.safe_corridor_line_max_length,
                    cfg.robot_r);
            if (result.severity == checker::Severity::WARN) {
                logCheckResult(ros_ptr, context, result);
            }
        }
    }

    RET_CODE state2state_task::planFromRest(StateToStateTaskServices &services,
                                            StateToStateExpBackendServices &exp_services,
                                            StateToStateBackupBackendServices &backup_services,
                                            const Vec3f &goal_p,
                                            const double goal_yaw,
                                            const bool new_goal) {
        std::lock_guard<std::mutex> guard(services.replan_lock);
        services.latest_replan.reset();
        const auto input_check = checker::checkState2StateInput(goal_p,
                                                                goal_yaw,
                                                                services.robot_state,
                                                                services.map_manager,
                                                                services.ros_ptr->getSimTime());
        if (rejectOnCheckFailure(services.ros_ptr, "PlanFromRest input", input_check)) {
            services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);
            services.latest_replan.setRetCode(input_check.code == "MAP_NOT_READY"
                                              ? GENERAL_RET_CODE::GENERAL_MAP_NOT_READY
                                              : GENERAL_RET_CODE::GENERAL_UNDEFINED);
            return FAILED;
        }
        warnHighSpeedMargin(services.ros_ptr,
                            services.cfg,
                            services.robot_state.v.norm(),
                            "PlanFromRest high-speed margin");
        if (!services.robot_state.rcv) {
            services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);
            services.ros_ptr->warn(" -- [GeneralPlanner] in [PlanFromRest]: No odom, force return.");
            services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            return FAILED;
        }
        services.backend_context.setGoalInfo(goal_p, goal_yaw, new_goal, true);
        services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);
        vec_Vec3f viz_pts{goal_p, services.robot_state.p};

        {
            TimeConsuming t_viz("viz goal path", false);
            services.ros_ptr->vizGoalPath(viz_pts);
            services.time_consuming[VISUALIZATION] += t_viz.stop();
        }

        Vec3f local_star_pt;
        if (!services.map_manager->getNearestInfCellNot(GridType::OCCUPIED,
                                                        services.robot_state.p,
                                                        local_star_pt,
                                                        3.0)) {
            services.ros_ptr->error(
                    " -- [GeneralPlanner] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_START_POINT);
            return FAILED;
        }
        services.latest_replan.setLocalStartP(local_star_pt);

        ExpTraj exp_traj_info;
        BackupTraj back_traj_info;
        services.last_exp_traj_info.setEmpty();
        services.local_start_p = local_star_pt;
        RET_CODE exp_ret_code = generateExpTrajectory(
                exp_services,
                services.last_exp_traj_info,
                exp_traj_info);
        if (exp_ret_code == FAILED) {
            services.ros_ptr->warn(" -- [GeneralPlanner] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                                   RET_CODE_STR[exp_ret_code].c_str());
            return FAILED;
        } else {
            services.ros_ptr->info(" -- [GeneralPlanner] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        if (!backupTrajectoryPlanningEnabled(services.cfg)) {
            if (rejectOnCheckFailure(services.ros_ptr,
                                     "PlanFromRest exp commit",
                                     checker::checkExpTrajectory(exp_traj_info,
                                                                 services.cfg,
                                                                 "plan_from_rest_exp"))) {
                return FAILED;
            }
            services.robot_on_backup_traj = false;
            services.cmd_traj_info.setTrajectory(exp_traj_info);
            services.last_exp_traj_info = exp_traj_info;
            services.backend_context.markGoalConsumed();
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(), -1);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }
            services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }

        if (goalConnectedExpStopsSafely(services.cfg, exp_traj_info, goal_p)) {
            if (rejectOnCheckFailure(services.ros_ptr,
                                     "PlanFromRest goal-connected exp commit",
                                     checker::checkExpTrajectory(
                                             exp_traj_info,
                                             services.cfg,
                                             "plan_from_rest_goal_connected_exp"))) {
                return FAILED;
            }
            services.cmd_traj_info.setTrajectory(exp_traj_info);
            services.last_exp_traj_info = exp_traj_info;
            services.robot_on_backup_traj = false;
            services.backend_context.markGoalConsumed();
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(), -1);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }
            services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP);
            services.ros_ptr->info(
                    " -- [GeneralPlanner] Goal-connected exp trajectory already stops at goal; skip redundant backup.");
            return SUCCESS;
        }

        back_traj_info.setEmpty();
        RET_CODE back_ret_code = generateBackupTrajectory(
                backup_services,
                exp_traj_info,
                back_traj_info);

        if (back_ret_code == SUCCESS) {
            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            if (rejectOnCheckFailure(services.ros_ptr,
                                     "PlanFromRest exp+backup commit",
                                     checker::checkExpBackupCommit(exp_traj_info,
                                                                   back_traj_info,
                                                                   services.cfg,
                                                                   "plan_from_rest_exp_backup"))) {
                return FAILED;
            }
            if (!services.cmd_traj_info.setTrajectory(exp_traj_info, back_traj_info)) {
                services.ros_ptr->error(" -- [Checker] PlanFromRest commit failed: CmdTraj rejected exp+backup trajectory.");
                return FAILED;
            }
            services.last_exp_traj_info = exp_traj_info;
            services.robot_on_backup_traj = false;
            services.backend_context.markGoalConsumed();

            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(),
                                                   services.cmd_traj_info.getBackupTrajStartTT());
                services.time_consuming[VISUALIZATION] += t_viz.stop();
                services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_SUCCESS_WITH_BACKUP);
            }

            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == NO_NEED) {
            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [PlanFromRest] generateBackupTrajectory Finish or NO_NEED.");
            }
            services.robot_on_backup_traj = false;
            if (rejectOnCheckFailure(services.ros_ptr,
                                     "PlanFromRest exp commit",
                                     checker::checkExpTrajectory(exp_traj_info,
                                                                 services.cfg,
                                                                 "plan_from_rest_exp"))) {
                return FAILED;
            }
            services.cmd_traj_info.setTrajectory(exp_traj_info);
            services.last_exp_traj_info = exp_traj_info;
            services.backend_context.markGoalConsumed();

            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(), -1);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }
            services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        services.ros_ptr->warn(" -- [GeneralPlanner] in [PlanFromRest] generateBackupTrajectory return [{}], force return",
                               RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }

    RET_CODE state2state_task::replanOnce(StateToStateTaskServices &services,
                                          StateToStateExpBackendServices &exp_services,
                                          StateToStateBackupBackendServices &backup_services,
                                          const Vec3f &goal_p,
                                          const double goal_yaw,
                                          const bool new_goal) {
        TimeConsuming replan_total_t("ReplanOnce", false);
        std::lock_guard<std::mutex> guard(services.replan_lock);

        const auto input_check = checker::checkState2StateInput(goal_p,
                                                                goal_yaw,
                                                                services.robot_state,
                                                                services.map_manager,
                                                                services.ros_ptr->getSimTime());
        if (rejectOnCheckFailure(services.ros_ptr, "ReplanOnce input", input_check)) {
            services.latest_replan.reset();
            services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);
            services.latest_replan.setRetCode(input_check.code == "MAP_NOT_READY"
                                              ? GENERAL_RET_CODE::GENERAL_MAP_NOT_READY
                                              : GENERAL_RET_CODE::GENERAL_UNDEFINED);
            return FAILED;
        }
        warnHighSpeedMargin(services.ros_ptr,
                            services.cfg,
                            services.robot_state.v.norm(),
                            "ReplanOnce high-speed margin");

        services.backend_context.setGoalInfo(goal_p, goal_yaw, new_goal, true);
        services.latest_replan.reset();
        services.latest_replan.setGoal(goal_p, goal_yaw, services.robot_state);

        vec_Vec3f viz_pts{goal_p, services.robot_state.p};

        {
            TimeConsuming t_viz("tviz", false);
            services.ros_ptr->vizGoalPath(viz_pts);
            services.time_consuming[VISUALIZATION] += t_viz.stop();
        }

        ExpTraj exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        RET_CODE exp_ret_code = generateExpTrajectory(
                exp_services,
                services.last_exp_traj_info,
                exp_traj_info);
        services.time_consuming[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            services.ros_ptr->warn(" -- [GeneralPlanner] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            return FAILED;
        } else if (exp_ret_code == NEW_TRAJ) {
            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [ReplanOnce]: Last epx traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            services.ros_ptr->warn(" -- [GeneralPlanner] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED) {
            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [ReplanOnce]: No need to replan a new exp traj, use last one.");
            }
            services.latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            return NO_NEED;
        }

        {
            TimeConsuming t_viz("tviz", false);
            services.ros_ptr->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            services.time_consuming[VISUALIZATION] += t_viz.stop();
        }

        if (!backupTrajectoryPlanningEnabled(services.cfg)) {
            if (rejectOnCheckFailure(services.ros_ptr,
                                     "ReplanOnce exp commit",
                                     checker::checkExpTrajectory(exp_traj_info,
                                                                 services.cfg,
                                                                 "replan_exp"))) {
                return FAILED;
            }
            services.robot_on_backup_traj = false;
            services.cmd_traj_info.setTrajectory(exp_traj_info);
            services.last_exp_traj_info = exp_traj_info;
            services.backend_context.markGoalConsumed();
            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(), -1);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }
            services.latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }

        if (goalConnectedExpStopsSafely(services.cfg, exp_traj_info, goal_p)) {
            if (rejectOnCheckFailure(services.ros_ptr,
                                     "ReplanOnce goal-connected exp commit",
                                     checker::checkExpTrajectory(
                                             exp_traj_info,
                                             services.cfg,
                                             "replan_goal_connected_exp"))) {
                return FAILED;
            }
            services.cmd_traj_info.setTrajectory(exp_traj_info);
            services.last_exp_traj_info = exp_traj_info;
            services.robot_on_backup_traj = false;
            services.backend_context.markGoalConsumed();
            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(), -1);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }
            services.latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            services.ros_ptr->info(
                    " -- [GeneralPlanner] Goal-connected exp trajectory already stops at goal; skip redundant backup.");
            return SUCCESS;
        }

        BackupTraj back_traj_info;
        TimeConsuming t_back("t_back", false);
        RET_CODE back_ret_code = generateBackupTrajectory(
                backup_services,
                exp_traj_info,
                back_traj_info);
        services.time_consuming[GENERATE_BACK_TRAJ] = t_back.stop();

        {
            services.frontend_time_sum +=
                    services.time_consuming[EPX_TRAJ_FRONTEND] +
                    services.time_consuming[BACK_TRAJ_FRONTEND];
            services.frontend_time_count++;
            services.backend_time_sum +=
                    services.time_consuming[BACK_TRAJ_OPT] +
                    services.time_consuming[EXP_TRAJ_OPT];
            services.backend_time_count++;
        }

        double replan_dt = replan_total_t.stop();
        if (replan_dt > services.cfg.replan_forward_dt * 0.9) {
            services.ros_ptr->warn(" -- [GeneralPlanner] in [ReplanOnce]: Replan overtime, check parameters, replan dt = {}.",
                                   replan_dt);
            return FAILED;
        }

        auto acceptExpWithoutBackupNearGoal = [&]() {
            if (!services.cfg.state2state_accept_exp_without_backup_near_goal ||
                exp_traj_info.empty()) {
                return false;
            }
            const Trajectory &pos_traj = exp_traj_info.posTraj();
            if (pos_traj.empty()) {
                return false;
            }
            const double total_duration = pos_traj.getTotalDuration();
            if (!std::isfinite(total_duration) || total_duration <= 1.0e-4) {
                return false;
            }

            const double near_goal_radius =
                    std::max(services.cfg.resolution * 3.0,
                             services.cfg.state2state_near_goal_radius);
            const double robot_goal_xy =
                    (services.robot_state.p.head<2>() - goal_p.head<2>()).norm();
            const bool near_goal = robot_goal_xy < near_goal_radius ||
                                   exp_traj_info.connectedToGoal();
            if (!near_goal) {
                return false;
            }

            const double now_t =
                    std::clamp(services.ros_ptr->getSimTime() -
                                   exp_traj_info.getStartWallTime(),
                               0.0,
                               total_duration);
            const Vec3f start_pos = pos_traj.getPos(now_t);
            const Vec3f end_pos = pos_traj.getPos(total_duration);
            if (!start_pos.allFinite() || !end_pos.allFinite()) {
                return false;
            }
            const double end_goal_xy = (end_pos.head<2>() - goal_p.head<2>()).norm();
            if (!exp_traj_info.connectedToGoal() &&
                end_goal_xy > std::max(services.cfg.resolution * 3.0, 0.3)) {
                return false;
            }

            const double start_over =
                    state2stateGoalOvershoot(start_pos,
                                             services.local_start_p,
                                             start_pos,
                                             goal_p);
            const double allowed_over =
                    std::max(std::max(0.0, services.cfg.state2state_over_goal_tolerance),
                             start_over +
                                 std::max(0.0, services.cfg.state2state_over_goal_tolerance));
            const double sample_dt = std::max(0.02, services.cfg.sample_traj_dt);
            double max_over = 0.0;
            for (double t = now_t; t < total_duration; t += sample_dt) {
                max_over = std::max(max_over,
                                    state2stateGoalOvershoot(pos_traj.getPos(t),
                                                            services.local_start_p,
                                                            start_pos,
                                                            goal_p));
            }
            max_over = std::max(max_over,
                                state2stateGoalOvershoot(end_pos,
                                                        services.local_start_p,
                                                        start_pos,
                                                        goal_p));
            if (max_over > allowed_over + 1.0e-6) {
                services.ros_ptr->warn(" -- [GeneralPlanner] Reject exp-only near-goal fallback: candidate overshoot {:.2f}m > allowed {:.2f}m.",
                                       max_over,
                                       allowed_over);
                return false;
            }

            if (rejectOnCheckFailure(services.ros_ptr,
                                     "ReplanOnce exp-only near-goal fallback",
                                     checker::checkExpTrajectory(exp_traj_info,
                                                                 services.cfg,
                                                                 "replan_exp_only_near_goal"))) {
                return false;
            }

            services.cmd_traj_info.setTrajectory(exp_traj_info);
            services.last_exp_traj_info = exp_traj_info;
            services.robot_on_backup_traj = false;
            services.backend_context.markGoalConsumed();

            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(), -1);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }

            services.latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            services.ros_ptr->warn(" -- [GeneralPlanner] Backup generation failed near goal; commit checked exp-only trajectory to avoid running old command past goal.");
            return true;
        };

        if (back_ret_code == SUCCESS) {
            if (rejectOnCheckFailure(services.ros_ptr,
                                     "ReplanOnce exp+backup commit",
                                     checker::checkExpBackupCommit(exp_traj_info,
                                                                   back_traj_info,
                                                                   services.cfg,
                                                                   "replan_exp_backup"))) {
                return FAILED;
            }
            if (!services.cmd_traj_info.setTrajectory(exp_traj_info, back_traj_info)) {
                services.ros_ptr->error(" -- [Checker] ReplanOnce commit failed: CmdTraj rejected exp+backup trajectory.");
                return FAILED;
            }
            services.last_exp_traj_info = exp_traj_info;
            services.robot_on_backup_traj = false;
            services.backend_context.markGoalConsumed();

            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(),
                                                   services.cmd_traj_info.getBackupTrajStartTT());
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }

            services.latest_replan.setRetCode(GENERAL_SUCCESS_WITH_BACKUP);
            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            }
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) {
            services.robot_on_backup_traj = false;
            services.last_exp_traj_info = exp_traj_info;
            services.backend_context.markGoalConsumed();

            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(), -1);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }

            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [ReplanOnce]: No need back traj success, all replan success.");
            }
            services.latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            if (rejectOnCheckFailure(services.ros_ptr,
                                     "ReplanOnce exp commit",
                                     checker::checkExpTrajectory(exp_traj_info,
                                                                 services.cfg,
                                                                 "replan_exp"))) {
                return FAILED;
            }
            services.cmd_traj_info.setTrajectory(exp_traj_info);
            services.last_exp_traj_info = exp_traj_info;
            services.robot_on_backup_traj = false;
            services.backend_context.markGoalConsumed();

            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizCommittedTraj(services.cmd_traj_info.posTraj(), -1);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }

            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [ReplanOnce]: No need back traj success, all replan success.");
            }
            services.latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        if (acceptExpWithoutBackupNearGoal()) {
            return SUCCESS;
        }
        services.ros_ptr->warn(" -- [GeneralPlanner] in [ReplanOnce]: generateBackupTrajectory return {}, replan Failed return",
                               RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }

    RET_CODE GeneralPlanner::PlanFromRest(const Vec3f &goal_p,
                                          const double &goal_yaw,
                                          const bool &new_goal) {
        auto services = makeStateToStateTaskServices();
        auto exp_services = makeStateToStateExpBackendServices();
        auto backup_services = makeStateToStateBackupBackendServices();
        return state2state_task::planFromRest(services,
                                              exp_services,
                                              backup_services,
                                              goal_p,
                                              goal_yaw,
                                              new_goal);
    }

    RET_CODE GeneralPlanner::ReplanOnce(const Vec3f &goal_p,
                                        const double &goal_yaw,
                                        const bool &new_goal) {
        auto services = makeStateToStateTaskServices();
        auto exp_services = makeStateToStateExpBackendServices();
        auto backup_services = makeStateToStateBackupBackendServices();
        return state2state_task::replanOnce(services,
                                            exp_services,
                                            backup_services,
                                            goal_p,
                                            goal_yaw,
                                            new_goal);
    }

}
