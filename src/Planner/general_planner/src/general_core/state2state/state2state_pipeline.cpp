/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>
#include <general_core/nhbp/far_goal_reasoner.hpp>
#include <general_core/nhbp/sparse_global_map.hpp>
#include <general_core/nhbp/state2state_nhbp_adapter.hpp>
#include <general_core/nhbp/topological_memory.hpp>
#include <general_core/state2state/state2state_path_utils.hpp>
#include <checker/state2state_checker.hpp>
#include <checker/trajectory_checker.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
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
                *state2state_backend_context_,
                state2state_nhbp_adapter_.get(),
                sparse_global_map_.get(),
                far_goal_reasoner_.get(),
                topological_memory_.get(),
                &sparse_global_map_last_update_wt_,
                &latest_state2state_nhbp_debug_info_
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

        nhbp::SparseCellState sparseStateFromGrid(const rog_map::GridType grid_type) {
            if (grid_type == rog_map::OCCUPIED) {
                return nhbp::SparseCellState::OCCUPIED_BOUNDARY;
            }
            if (grid_type == rog_map::UNKNOWN || grid_type == rog_map::OUT_OF_MAP) {
                return nhbp::SparseCellState::FRONTIER_BOUNDARY;
            }
            return nhbp::SparseCellState::FREE_BOUNDARY;
        }

        void updateSparseGlobalMapFromLocalBoundary(state2state_task::StateToStateTaskServices &services,
                                                    const double stamp) {
            if (!services.cfg.sparse_global_map_enable ||
                services.sparse_global_map == nullptr ||
                services.map_manager == nullptr ||
                !services.map_manager->ready()) {
                return;
            }
            if (services.sparse_global_map_last_update_wt != nullptr &&
                *services.sparse_global_map_last_update_wt >= 0.0 &&
                stamp - *services.sparse_global_map_last_update_wt <
                        std::max(0.0, services.cfg.sparse_global_map_update_period)) {
                return;
            }

            rog_map::Vec3f box_min = rog_map::Vec3f::Zero();
            rog_map::Vec3f box_max = rog_map::Vec3f::Zero();
            services.map_manager->boundBoxByLocalMap(box_min, box_max);

            const double step = std::max({services.cfg.sparse_global_map_sample_resolution,
                                          services.map_manager->getResolution(),
                                          1.0e-3});
            const int max_samples = std::max(0, services.cfg.sparse_global_map_max_boundary_samples);
            int sample_count = 0;

            const auto observe = [&](const rog_map::Vec3f &pos) {
                if (sample_count >= max_samples || !pos.allFinite()) {
                    return;
                }
                if (!services.map_manager->insideLocalMap(pos)) {
                    return;
                }
                const auto grid_type = services.map_manager->getGridType(pos);
                services.sparse_global_map->observeBoundaryCell(pos,
                                                                sparseStateFromGrid(grid_type),
                                                                stamp);
                ++sample_count;
            };

            const auto forRange = [](const double min_v,
                                     const double max_v,
                                     const double step_v,
                                     const std::function<void(double)> &fn) {
                if (max_v < min_v || step_v <= 0.0) {
                    return;
                }
                for (double value = min_v; value <= max_v + 1.0e-6; value += step_v) {
                    fn(std::min(value, max_v));
                }
            };

            forRange(box_min.x(), box_max.x(), step, [&](const double x) {
                forRange(box_min.y(), box_max.y(), step, [&](const double y) {
                    observe(rog_map::Vec3f(x, y, box_min.z()));
                    observe(rog_map::Vec3f(x, y, box_max.z()));
                });
            });
            forRange(box_min.x(), box_max.x(), step, [&](const double x) {
                forRange(box_min.z(), box_max.z(), step, [&](const double z) {
                    observe(rog_map::Vec3f(x, box_min.y(), z));
                    observe(rog_map::Vec3f(x, box_max.y(), z));
                });
            });
            forRange(box_min.y(), box_max.y(), step, [&](const double y) {
                forRange(box_min.z(), box_max.z(), step, [&](const double z) {
                    observe(rog_map::Vec3f(box_min.x(), y, z));
                    observe(rog_map::Vec3f(box_max.x(), y, z));
                });
            });

            if (services.sparse_global_map_last_update_wt != nullptr) {
                *services.sparse_global_map_last_update_wt = stamp;
            }
            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [SparseGlobalMap] boundary samples={}, records={}, frontiers={}.",
                                       sample_count,
                                       services.sparse_global_map->recordCount(),
                                       services.sparse_global_map->frontierCount(stamp));
            }
        }

        struct LongRangeGoalResolution {
            Vec3f planning_goal{Vec3f::Zero()};
            bool using_subgoal{false};
            nhbp::FarGoalDecision decision;
        };

        void setState2StateNhbpDebug(
                state2state_task::StateToStateTaskServices &services,
                const std::string &debug_info) {
            if (services.latest_nhbp_debug_info != nullptr) {
                *services.latest_nhbp_debug_info = debug_info;
            }
        }

        bool state2StateNhbpDebugEmpty(
                const state2state_task::StateToStateTaskServices &services) {
            return services.latest_nhbp_debug_info == nullptr ||
                   services.latest_nhbp_debug_info->empty();
        }

        LongRangeGoalResolution resolveState2StatePlanningGoal(
                state2state_task::StateToStateTaskServices &services,
                const Vec3f &requested_goal,
                const double stamp) {
            LongRangeGoalResolution resolution;
            resolution.planning_goal = requested_goal;

            updateSparseGlobalMapFromLocalBoundary(services, stamp);

            const Vec3f robot_pos = services.robot_state.p;
            if (services.topological_memory != nullptr) {
                services.topological_memory->observePose(robot_pos, stamp);
            }

            const auto observePlanningGoalInTopology = [&]() {
                if (services.topological_memory == nullptr ||
                    !resolution.planning_goal.allFinite() ||
                    services.map_manager == nullptr ||
                    !services.map_manager->ready() ||
                    !services.map_manager->insideLocalMap(resolution.planning_goal)) {
                    return;
                }
                services.topological_memory->observePose(
                        resolution.planning_goal,
                        stamp,
                        resolution.using_subgoal ? nhbp::TopoNodeType::FRONTIER
                                                 : nhbp::TopoNodeType::BRANCH);
                services.topological_memory->observeTransition(robot_pos,
                                                               resolution.planning_goal,
                                                               stamp);
            };

            if (!services.cfg.state2state_far_goal_enable ||
                !services.cfg.far_goal_reasoner_enable ||
                services.far_goal_reasoner == nullptr ||
                !robot_pos.allFinite() ||
                !requested_goal.allFinite()) {
                observePlanningGoalInTopology();
                return resolution;
            }

            const double goal_distance = (requested_goal - robot_pos).norm();
            const bool far_enough =
                    goal_distance >= std::max(0.0, services.cfg.state2state_far_goal_min_distance);
            const bool outside_local =
                    services.map_manager == nullptr ||
                    !services.map_manager->ready() ||
                    !services.map_manager->insideLocalMap(requested_goal);
            if (!far_enough && !outside_local) {
                observePlanningGoalInTopology();
                return resolution;
            }

            resolution.decision =
                    services.far_goal_reasoner->selectSubgoal(robot_pos,
                                                              requested_goal,
                                                              stamp,
                                                              services.sparse_global_map,
                                                              services.topological_memory);
            if (resolution.decision.ready &&
                resolution.decision.type != nhbp::FarGoalDecisionType::DIRECT_GOAL &&
                resolution.decision.local_goal.allFinite() &&
                services.map_manager != nullptr &&
                services.map_manager->ready() &&
                services.map_manager->insideLocalMap(resolution.decision.local_goal)) {
                const auto grid_type = services.map_manager->getInfGridType(resolution.decision.local_goal);
                if (grid_type != rog_map::OCCUPIED && grid_type != rog_map::OUT_OF_MAP) {
                    resolution.planning_goal = resolution.decision.local_goal;
                    resolution.using_subgoal = true;
                }
            }

            observePlanningGoalInTopology();

            if (resolution.using_subgoal && services.cfg.print_log) {
                services.ros_ptr->warn(" -- [FarGoalReasoner] Use local subgoal [{:.2f}, {:.2f}, {:.2f}] for far goal [{:.2f}, {:.2f}, {:.2f}], type={}, reason={}, score={:.3f}.",
                                       resolution.planning_goal.x(),
                                       resolution.planning_goal.y(),
                                       resolution.planning_goal.z(),
                                       requested_goal.x(),
                                       requested_goal.y(),
                                       requested_goal.z(),
                                       nhbp::toString(resolution.decision.type),
                                       resolution.decision.reason,
                                       resolution.decision.score);
            }
            return resolution;
        }

        class ScopedState2StateGoal {
        public:
            ScopedState2StateGoal(state2state_task::StateToStateExpBackendServices &services,
                                  const Vec3f &planning_goal)
                    : services_(services),
                      original_goal_(services.goal_p) {
                if ((planning_goal - services_.goal_p).norm() > 1.0e-5) {
                    services_.goal_p = planning_goal;
                    changed_ = true;
                }
            }

            ~ScopedState2StateGoal() {
                if (changed_) {
                    services_.goal_p = original_goal_;
                }
            }

        private:
            state2state_task::StateToStateExpBackendServices &services_;
            Vec3f original_goal_;
            bool changed_{false};
        };

        RET_CODE generateExpTrajectoryForPlanningGoal(
                state2state_task::StateToStateExpBackendServices &services,
                const Vec3f &planning_goal,
                ExpTraj &last_exp_traj_info,
                ExpTraj &out_exp_traj_info) {
            ScopedState2StateGoal scoped_goal(services, planning_goal);
            return state2state_task::generateExpTrajectory(services,
                                                           last_exp_traj_info,
                                                           out_exp_traj_info);
        }

        bool copyCommittedTrajectory(CmdTraj &cmd_traj_info,
                                     Trajectory &pos_traj,
                                     Trajectory &yaw_traj,
                                     bool &backup_available,
                                     double &backup_start_t) {
            cmd_traj_info.lock();
            const bool empty = cmd_traj_info.empty();
            if (!empty) {
                pos_traj = cmd_traj_info.posTraj();
                yaw_traj = cmd_traj_info.yawTraj();
                backup_available = cmd_traj_info.backupTrajAvilibale();
                backup_start_t = cmd_traj_info.getBackupTrajStartTT();
            }
            cmd_traj_info.unlock();
            return !empty && !pos_traj.empty();
        }

        void recordState2StateNhbpCommit(state2state_task::StateToStateTaskServices &services,
                                         const Vec3f &goal_p,
                                         const std::string &source) {
            if (services.nhbp_adapter == nullptr) {
                return;
            }
            Trajectory committed_pos;
            Trajectory committed_yaw;
            bool backup_available = false;
            double backup_start_t = -1.0;
            if (!copyCommittedTrajectory(services.cmd_traj_info,
                                         committed_pos,
                                         committed_yaw,
                                         backup_available,
                                         backup_start_t)) {
                return;
            }
            services.nhbp_adapter->recordCommitted(committed_pos,
                                                   services.robot_state,
                                                   goal_p,
                                                   services.ros_ptr->getSimTime(),
                                                   source);
            if (state2StateNhbpDebugEmpty(services)) {
                setState2StateNhbpDebug(
                        services,
                        fmt::format(";nhbp_action=COMMIT;nhbp_reason={};{}",
                                    source,
                                    services.nhbp_adapter->diagnosticSummary(
                                            services.ros_ptr->getSimTime())));
            }
            (void)committed_yaw;
            (void)backup_available;
            (void)backup_start_t;
        }

        RET_CODE keepCurrentTrajectoryForNhbp(
                state2state_task::StateToStateTaskServices &services,
                const Vec3f &goal_p,
                const nhbp::State2StateNHBPDecision &decision) {
            Trajectory current_pos;
            Trajectory current_yaw;
            bool backup_available = false;
            double backup_start_t = -1.0;
            if (!copyCommittedTrajectory(services.cmd_traj_info,
                                         current_pos,
                                         current_yaw,
                                         backup_available,
                                         backup_start_t)) {
                return FAILED;
            }
            services.latest_replan.setExpTraj(current_pos);
            services.latest_replan.setExpYawTraj(current_yaw);
            services.latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            services.backend_context.markGoalConsumed();
            if (services.nhbp_adapter != nullptr) {
                setState2StateNhbpDebug(services,
                                        services.nhbp_adapter->formatDecisionDiagnostic(decision));
            }
            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizCommittedTraj(current_pos,
                                                   backup_available ? backup_start_t : -1.0);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }
            if (services.cfg.print_log) {
                services.ros_ptr->warn(" -- [State2StateNHBP] Keep committed trajectory: reason={}, candidate_branch={}, current_branch={}, candidate_score={:.3f}, current_score={:.3f}, remaining={:.3f}, ndo={}.",
                                       decision.reason,
                                       decision.candidate.branch_id,
                                       decision.current.branch_id,
                                       decision.candidate_score,
                                       decision.current_score,
                                       decision.current_remaining,
                                       nhbp::toString(decision.ndo.state));
            }
            if (services.nhbp_adapter != nullptr) {
                services.nhbp_adapter->recordCommitted(current_pos,
                                                       services.robot_state,
                                                       goal_p,
                                                       services.ros_ptr->getSimTime(),
                                                       "keep_current");
            }
            return NO_NEED;
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
        setState2StateNhbpDebug(services, "");
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
        const double planning_stamp = services.ros_ptr->getSimTime();
        const LongRangeGoalResolution goal_resolution =
                resolveState2StatePlanningGoal(services, goal_p, planning_stamp);
        const Vec3f planning_goal_p = goal_resolution.planning_goal;
        vec_Vec3f viz_pts{planning_goal_p, services.robot_state.p};
        if (goal_resolution.using_subgoal) {
            viz_pts.insert(viz_pts.begin(), goal_p);
        }

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
        RET_CODE exp_ret_code = generateExpTrajectoryForPlanningGoal(
                exp_services,
                planning_goal_p,
                services.last_exp_traj_info,
                exp_traj_info);
        if (exp_ret_code == FAILED) {
            services.ros_ptr->warn(" -- [GeneralPlanner] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                                   RET_CODE_STR[exp_ret_code].c_str());
            if (services.topological_memory != nullptr) {
                services.topological_memory->recordFailureNear(planning_goal_p,
                                                               services.ros_ptr->getSimTime());
            }
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
            recordState2StateNhbpCommit(services, goal_p, "plan_from_rest_exp");
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

            recordState2StateNhbpCommit(services, goal_p, "plan_from_rest_exp_backup");
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
            recordState2StateNhbpCommit(services, goal_p, "plan_from_rest_exp_only");
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
        setState2StateNhbpDebug(services, "");

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

        const double planning_stamp = services.ros_ptr->getSimTime();
        const LongRangeGoalResolution goal_resolution =
                resolveState2StatePlanningGoal(services, goal_p, planning_stamp);
        const Vec3f planning_goal_p = goal_resolution.planning_goal;
        vec_Vec3f viz_pts{planning_goal_p, services.robot_state.p};
        if (goal_resolution.using_subgoal) {
            viz_pts.insert(viz_pts.begin(), goal_p);
        }

        {
            TimeConsuming t_viz("tviz", false);
            services.ros_ptr->vizGoalPath(viz_pts);
            services.time_consuming[VISUALIZATION] += t_viz.stop();
        }

        ExpTraj exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        RET_CODE exp_ret_code = generateExpTrajectoryForPlanningGoal(
                exp_services,
                planning_goal_p,
                services.last_exp_traj_info,
                exp_traj_info);
        services.time_consuming[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            services.ros_ptr->warn(" -- [GeneralPlanner] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            if (services.nhbp_adapter != nullptr) {
                services.nhbp_adapter->recordFailure(nullptr,
                                                     services.robot_state,
                                                     goal_p,
                                                     services.ros_ptr->getSimTime(),
                                                     nhbp::FailureReason::OPTIMIZATION_FAIL);
                setState2StateNhbpDebug(
                        services,
                        fmt::format(";nhbp_action=RECORD_FAILURE;nhbp_reason=exp_generation_failed;"
                                    "nhbp_failure={};{}",
                                    nhbp::toString(nhbp::FailureReason::OPTIMIZATION_FAIL),
                                    services.nhbp_adapter->diagnosticSummary(
                                            services.ros_ptr->getSimTime())));
            }
            if (services.topological_memory != nullptr) {
                services.topological_memory->recordFailureNear(planning_goal_p,
                                                               services.ros_ptr->getSimTime());
            }
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
            setState2StateNhbpDebug(
                    services,
                    ";nhbp_action=KEEP_CURRENT;nhbp_reason=exp_no_need");
            return NO_NEED;
        }

        if (exp_ret_code == SUCCESS && services.nhbp_adapter != nullptr) {
            Trajectory current_pos;
            Trajectory current_yaw;
            bool backup_available = false;
            double backup_start_t = -1.0;
            if (copyCommittedTrajectory(services.cmd_traj_info,
                                        current_pos,
                                        current_yaw,
                                        backup_available,
                                        backup_start_t)) {
                const nhbp::State2StateNHBPDecision nhbp_decision =
                        services.nhbp_adapter->evaluateReplan(
                                exp_traj_info.posTraj(),
                                current_pos,
                                backup_available ? backup_start_t : -1.0,
                                services.robot_state,
                                goal_p,
                                services.ros_ptr->getSimTime(),
                                new_goal,
                                exp_services.runtime_safety,
                                services.cfg);
                setState2StateNhbpDebug(
                        services,
                        services.nhbp_adapter->formatDecisionDiagnostic(nhbp_decision));
                if (nhbp_decision.action == nhbp::State2StateGateAction::KEEP_CURRENT) {
                    return keepCurrentTrajectoryForNhbp(services, goal_p, nhbp_decision);
                }
            }
            else {
                setState2StateNhbpDebug(
                        services,
                        ";nhbp_action=SKIP;nhbp_reason=no_committed_current_trajectory");
            }
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
            recordState2StateNhbpCommit(services, goal_p, "replan_exp");
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
            recordState2StateNhbpCommit(services, goal_p, "replan_exp_only_near_goal");
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
            recordState2StateNhbpCommit(services, goal_p, "replan_exp_backup");
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
            recordState2StateNhbpCommit(services, goal_p, "replan_backup_no_need");
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
            recordState2StateNhbpCommit(services, goal_p, "replan_finish_exp");
            return SUCCESS;
        }
        if (acceptExpWithoutBackupNearGoal()) {
            return SUCCESS;
        }
        services.ros_ptr->warn(" -- [GeneralPlanner] in [ReplanOnce]: generateBackupTrajectory return {}, replan Failed return",
                               RET_CODE_STR[back_ret_code].c_str());
        if (services.nhbp_adapter != nullptr) {
            services.nhbp_adapter->recordFailure(&exp_traj_info.posTraj(),
                                                 services.robot_state,
                                                 goal_p,
                                                 services.ros_ptr->getSimTime(),
                                                 nhbp::FailureReason::BACKUP_TRIGGERED);
            setState2StateNhbpDebug(
                    services,
                    fmt::format(";nhbp_action=RECORD_FAILURE;nhbp_reason=backup_generation_failed;"
                                "nhbp_failure={};{}",
                                nhbp::toString(nhbp::FailureReason::BACKUP_TRIGGERED),
                                services.nhbp_adapter->diagnosticSummary(
                                        services.ros_ptr->getSimTime())));
        }
        if (services.topological_memory != nullptr) {
            services.topological_memory->recordFailureNear(planning_goal_p,
                                                           services.ros_ptr->getSimTime());
        }
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
