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

using namespace general_utils;

namespace general_planner {
    namespace {
        double explorationYawDiff(const double target_yaw, const double current_yaw) {
            if (!std::isfinite(target_yaw) || !std::isfinite(current_yaw)) {
                return 0.0;
            }
            return std::atan2(std::sin(target_yaw - current_yaw),
                              std::cos(target_yaw - current_yaw));
        }

        bool explorationGoalIsYawScanRecovery(const ExplorationGoal &goal,
                                              const Vec3f &robot_pos,
                                              const double position_tolerance) {
            if (!goal.valid ||
                !goal.identity.recovery_intent ||
                goal.reason.find("yaw scan") == std::string::npos ||
                !goal.position.allFinite() ||
                !robot_pos.allFinite()) {
                return false;
            }
            return (goal.position - robot_pos).norm() <= std::max(0.05, position_tolerance);
        }

        bool buildExplorationConstantPositionTrajectory(const Vec3f &position,
                                                        const double duration,
                                                        const double start_wt,
                                                        Trajectory &traj) {
            if (!position.allFinite() ||
                !std::isfinite(duration) ||
                duration <= 1.0e-5 ||
                !std::isfinite(start_wt)) {
                return false;
            }
            Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 8);
            coeff.col(7) = position;
            traj.clear();
            traj.emplace_back(duration, coeff);
            traj.start_WT = start_wt;
            return !traj.empty();
        }
    } // namespace

    ExplorationFrontend::Config GeneralPlanner::makeExplorationFrontendConfig() const {
        ExplorationFrontend::Config frontend_cfg;
        frontend_cfg.enable = cfg_.exploration_enable;
        frontend_cfg.print_log = cfg_.exploration_print_log;
        frontend_cfg.frontier_source = cfg_.exploration_frontier_source;
        frontend_cfg.map_resolution = std::max(0.2, cfg_.resolution);
        frontend_cfg.frontier_search_radius = cfg_.exploration_frontier_search_radius;
        frontend_cfg.frontier_cluster_radius = cfg_.exploration_frontier_cluster_radius;
        frontend_cfg.frontier_sample_resolution = cfg_.exploration_frontier_sample_resolution;
        frontend_cfg.frontier_subcluster_size = cfg_.exploration_frontier_subcluster_size;
        frontend_cfg.min_frontier_cluster_size = cfg_.exploration_min_frontier_cluster_size;
        frontend_cfg.min_frontier_area = cfg_.exploration_min_frontier_area;
        frontend_cfg.min_frontier_extent = cfg_.exploration_min_frontier_extent;
        frontend_cfg.min_unknown_neighbor_count = cfg_.exploration_min_unknown_neighbor_count;
        frontend_cfg.max_raw_frontier_points = cfg_.exploration_max_raw_frontier_points;
        frontend_cfg.max_frontier_cells = cfg_.exploration_max_frontier_cells;
        frontend_cfg.max_frontier_clusters = cfg_.exploration_max_frontier_clusters;
        frontend_cfg.max_subclusters_per_cluster = cfg_.exploration_max_subclusters_per_cluster;
        frontend_cfg.frontier_manager_enable = cfg_.exploration_frontier_manager_enable;
        frontend_cfg.frontier_manager_max_records = cfg_.exploration_frontier_manager_max_records;
        frontend_cfg.frontier_manager_match_radius =
                cfg_.exploration_frontier_manager_match_radius;
        frontend_cfg.frontier_manager_stale_time =
                cfg_.exploration_frontier_manager_stale_time;
        frontend_cfg.frontier_manager_dormant_time =
                cfg_.exploration_frontier_manager_dormant_time;
        frontend_cfg.frontier_manager_covered_unknown_radius =
                cfg_.exploration_frontier_manager_covered_unknown_radius;
        frontend_cfg.frontier_manager_min_changed_fraction =
                cfg_.exploration_frontier_manager_min_changed_fraction;
        frontend_cfg.frontier_manager_selection_penalty =
                cfg_.exploration_frontier_manager_selection_penalty;
        frontend_cfg.frontier_manager_recent_selection_penalty =
                cfg_.exploration_frontier_manager_recent_selection_penalty;
        frontend_cfg.frontier_manager_recent_selection_window =
                cfg_.exploration_frontier_manager_recent_selection_window;
        frontend_cfg.frontier_manager_no_view_threshold =
                cfg_.exploration_frontier_manager_no_view_threshold;
        frontend_cfg.viewpoint_min_distance = cfg_.exploration_viewpoint_min_distance;
        frontend_cfg.viewpoint_max_distance = cfg_.exploration_viewpoint_max_distance;
        frontend_cfg.viewpoint_height_offset = cfg_.exploration_viewpoint_height_offset;
        frontend_cfg.viewpoint_safe_distance = cfg_.exploration_viewpoint_safe_distance;
        frontend_cfg.viewpoint_yaw_sample_num = cfg_.exploration_viewpoint_yaw_sample_num;
        frontend_cfg.viewpoint_radius_sample_num = cfg_.exploration_viewpoint_radius_sample_num;
        frontend_cfg.viewpoint_use_normal_sampling = cfg_.exploration_viewpoint_use_normal_sampling;
        frontend_cfg.viewpoint_normal_angle = cfg_.exploration_viewpoint_normal_angle;
        frontend_cfg.viewpoint_z_sample_num = cfg_.exploration_viewpoint_z_sample_num;
        frontend_cfg.viewpoint_z_min = cfg_.exploration_viewpoint_z_min;
        frontend_cfg.viewpoint_z_max = cfg_.exploration_viewpoint_z_max;
        frontend_cfg.min_visible_frontier_cells = cfg_.exploration_min_visible_frontier_cells;
        frontend_cfg.min_visible_frontier_ratio = cfg_.exploration_min_visible_frontier_ratio;
        frontend_cfg.max_candidate_num = cfg_.exploration_max_candidate_num;
        frontend_cfg.max_candidates_per_frontier_cluster =
                cfg_.exploration_max_candidates_per_frontier_cluster;
        frontend_cfg.candidate_separation_distance =
                cfg_.exploration_candidate_separation_distance;
        frontend_cfg.max_astar_checks = cfg_.exploration_max_astar_checks;
        frontend_cfg.min_direct_reachable_before_astar = cfg_.exploration_min_direct_reachable_before_astar;
        frontend_cfg.max_astar_checks_per_frontier = cfg_.exploration_max_astar_checks_per_frontier;
        frontend_cfg.max_reachable_candidate_num = cfg_.exploration_max_reachable_candidate_num;
        frontend_cfg.max_gain_rays = cfg_.exploration_max_gain_rays;
        frontend_cfg.gain_ray_length = cfg_.exploration_gain_ray_length;
        frontend_cfg.gain_ray_step = cfg_.exploration_gain_ray_step;
        frontend_cfg.yaw_policy = cfg_.exploration_yaw_policy;
        frontend_cfg.weight_travel = cfg_.exploration_weight_travel;
        frontend_cfg.weight_yaw = cfg_.exploration_weight_yaw;
        frontend_cfg.weight_curvature = cfg_.exploration_weight_curvature;
        frontend_cfg.weight_info_gain = cfg_.exploration_weight_info_gain;
        frontend_cfg.weight_unknown_risk = cfg_.exploration_weight_unknown_risk;
        frontend_cfg.weight_progress = cfg_.exploration_weight_progress;
        frontend_cfg.weight_short_goal = cfg_.exploration_weight_short_goal;
        frontend_cfg.information_gain_saturation = cfg_.exploration_information_gain_saturation;
        frontend_cfg.min_information_gain = cfg_.exploration_min_information_gain;
        frontend_cfg.min_goal_distance = cfg_.exploration_min_goal_distance;
        frontend_cfg.preferred_goal_distance = cfg_.exploration_preferred_goal_distance;
        frontend_cfg.goal_switch_min_score_improvement = cfg_.exploration_goal_switch_min_score_improvement;
        frontend_cfg.goal_reached_distance = cfg_.exploration_goal_reached_distance;
        frontend_cfg.unknown_as_occupied_for_motion = cfg_.exploration_unknown_as_occupied_for_motion;
        frontend_cfg.require_line_free_to_frontier = cfg_.exploration_require_line_free_to_frontier;
        frontend_cfg.use_astar_cost = cfg_.exploration_use_astar_cost;
        frontend_cfg.astar_time_out = cfg_.exploration_astar_time_out;
        frontend_cfg.astar_total_time_budget_ms = cfg_.exploration_astar_total_time_budget_ms;
        frontend_cfg.astar_failure_cache_ttl = cfg_.exploration_astar_failure_cache_ttl;
        frontend_cfg.use_atsp = cfg_.exploration_use_atsp;
        frontend_cfg.atsp.enable = cfg_.exploration_use_atsp;
        frontend_cfg.atsp.solver = cfg_.exploration_atsp_solver;
        frontend_cfg.atsp.work_dir = cfg_.exploration_atsp_work_dir;
        frontend_cfg.atsp.external_command = cfg_.exploration_atsp_external_command;
        frontend_cfg.atsp.time_budget_ms = cfg_.exploration_atsp_time_budget_ms;
        frontend_cfg.atsp.cost_scale = cfg_.exploration_atsp_cost_scale;
        frontend_cfg.atsp.max_candidate_num = cfg_.exploration_atsp_max_candidate_num;
        return frontend_cfg;
    }

    bool GeneralPlanner::validateExplorationRecoveryGoal(const ExplorationGoal &goal,
                                                         const Vec3f &robot_pos,
                                                         std::string *reason) {
        const auto fail = [reason](const std::string &msg) {
            if (reason != nullptr) {
                *reason = msg;
            }
            return false;
        };

        if (!goal.valid) {
            return fail("goal_invalid");
        }
        if (!goal.position.allFinite() || !robot_pos.allFinite()) {
            return fail("non_finite_position");
        }
        if (map_manager_ == nullptr || !map_manager_->ready()) {
            return fail("map_not_ready");
        }
        if (!map_manager_->insideLocalMap(robot_pos)) {
            return fail("robot_outside_local_map");
        }
        if (!map_manager_->insideLocalMap(goal.position)) {
            return fail("goal_outside_local_map");
        }
        if (exploration_manager_ != nullptr &&
            !exploration_manager_->insideTaskBox(goal.position)) {
            return fail("goal_outside_exploration_box");
        }

        const rog_map::GridType grid_type = map_manager_->getGridType(goal.position);
        if (grid_type == rog_map::GridType::OCCUPIED ||
            grid_type == rog_map::GridType::OUT_OF_MAP) {
            return fail("goal_prob_occupied_or_out");
        }
        if (cfg_.exploration_unknown_as_occupied_for_motion &&
            grid_type != rog_map::GridType::KNOWN_FREE) {
            return fail("goal_not_known_free");
        }

        const rog_map::GridType inf_type = map_manager_->getInfGridType(goal.position);
        if (inf_type == rog_map::GridType::OCCUPIED ||
            inf_type == rog_map::GridType::OUT_OF_MAP) {
            return fail("goal_inflated_occupied_or_out");
        }

        if (map_manager_->hasESDF() && cfg_.exploration_viewpoint_safe_distance > 0.0) {
            double dist = 0.0;
            Vec3f grad = Vec3f::Zero();
            if (!map_manager_->evaluateESDF(goal.position, dist, grad) ||
                !std::isfinite(dist) ||
                dist < cfg_.exploration_viewpoint_safe_distance) {
                return fail("goal_esdf_unsafe");
            }
        }

        const bool inflated_line_free =
                map_manager_->isLineFree(robot_pos, goal.position, true, false);
        const bool known_line_free =
                !cfg_.exploration_unknown_as_occupied_for_motion ||
                map_manager_->isLineFree(robot_pos, goal.position, false, true);
        if (inflated_line_free && known_line_free) {
            if (reason != nullptr) {
                *reason = "direct_line_free";
            }
            return true;
        }

        if (!cfg_.exploration_use_astar_cost || astar_ptr_ == nullptr) {
            return fail("direct_line_blocked_no_astar");
        }

        vec_E<Vec3f> path;
        const int astar_flag =
                cfg_.exploration_unknown_as_occupied_for_motion
                        ? (path_search::ON_PROB_MAP |
                           path_search::UNKNOWN_AS_OCCUPIED |
                           path_search::DONT_USE_INF_NEIGHBOR)
                        : (path_search::ON_INF_MAP |
                           path_search::UNKNOWN_AS_FREE);
        const double distance = (goal.position - robot_pos).norm();
        const double search_horizon =
                std::max(cfg_.exploration_frontier_search_radius * 1.5,
                         distance * 1.8 + 2.0);
        const RET_CODE ret = astar_ptr_->pointToPointPathSearch(
                robot_pos,
                goal.position,
                astar_flag,
                search_horizon,
                path,
                std::max(0.005, cfg_.exploration_astar_time_out));
        if (ret != REACH_GOAL || path.empty()) {
            return fail("astar_not_reached");
        }

        const double resolution = std::max(0.05, map_manager_->getResolution());
        if ((path.front() - robot_pos).norm() > resolution) {
            path.insert(path.begin(), robot_pos);
        }
        if ((path.back() - goal.position).norm() > resolution) {
            path.push_back(goal.position);
        }

        for (size_t i = 1; i < path.size(); ++i) {
            const bool segment_inflated_free =
                    map_manager_->isLineFree(path[i - 1], path[i], true, false);
            const bool segment_known_free =
                    !cfg_.exploration_unknown_as_occupied_for_motion ||
                    map_manager_->isLineFree(path[i - 1], path[i], false, true);
            if (!segment_inflated_free || !segment_known_free) {
                return fail("astar_segment_blocked");
            }
        }

        if (reason != nullptr) {
            *reason = "astar_reachable";
        }
        return true;
    }

    bool GeneralPlanner::selectValidatedExplorationRecoveryGoal(const Vec3f &robot_pos,
                                                                const double stamp,
                                                                ExplorationGoal &goal,
                                                                std::string *reason) {
        const auto set_reason = [reason](const std::string &msg) {
            if (reason != nullptr) {
                *reason = msg;
            }
        };

        const int max_attempts =
                std::clamp(cfg_.exploration_max_astar_checks / 2, 3, 8);
        std::string last_reject_reason = "no_recovery_goal";
        if (exploration_runtime_manager_ != nullptr) {
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                ExplorationGoal candidate;
                if (!exploration_runtime_manager_->hasRecoveryGoal(robot_pos, stamp, candidate)) {
                    break;
                }

                std::string validation_reason;
                if (validateExplorationRecoveryGoal(candidate, robot_pos, &validation_reason)) {
                    goal = candidate;
                    set_reason(validation_reason);
                    return true;
                }

                const nhbp::FailureReason failure_reason =
                        validation_reason.find("astar") != std::string::npos ||
                        validation_reason.find("line") != std::string::npos
                                ? nhbp::FailureReason::ASTAR_FAIL
                                : nhbp::FailureReason::VIEWPOINT_UNSAFE;
                exploration_runtime_manager_->recordFailure(candidate, failure_reason, stamp);
                last_reject_reason = validation_reason;
                if (exploration_manager_ != nullptr) {
                    exploration_manager_->recordFailure(candidate, stamp);
                }
                if (cfg_.exploration_print_log) {
                    ros_ptr_->warn(" -- [Exploration] Reject memory recovery goal before backend: reason={}, candidate_id={}, frontier_id={}, recovery_reason={}.",
                                   validation_reason,
                                   candidate.candidate_id,
                                   candidate.frontier_id,
                                   candidate.reason);
                }
            }
        }

        if (exploration_manager_ != nullptr) {
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                ExplorationGoal candidate;
                if (!exploration_manager_->recoverGoal(robot_pos, robot_state_.yaw, stamp, candidate)) {
                    break;
                }

                std::string validation_reason;
                if (validateExplorationRecoveryGoal(candidate, robot_pos, &validation_reason)) {
                    goal = candidate;
                    set_reason("manager_" + validation_reason);
                    return true;
                }

                exploration_manager_->recordFailure(candidate, stamp);
                last_reject_reason = validation_reason;
                if (cfg_.exploration_print_log) {
                    ros_ptr_->warn(" -- [Exploration] Reject manager recovery goal before backend: reason={}, candidate_id={}, frontier_id={}, recovery_reason={}.",
                                   validation_reason,
                                   candidate.candidate_id,
                                   candidate.frontier_id,
                                   candidate.reason);
                }
            }
        }

        set_reason(last_reject_reason);
        return false;
    }

    bool GeneralPlanner::commitExplorationYawScanTrajectory(const ExplorationGoal &goal) {
        if (!robot_state_.rcv || !robot_state_.p.allFinite()) {
            ros_ptr_->warn(" -- [Exploration] Yaw-scan recovery commit failed: invalid robot state.");
            return false;
        }
        const double position_tolerance = std::max(0.15, cfg_.resolution * 2.0);
        if (!explorationGoalIsYawScanRecovery(goal, robot_state_.p, position_tolerance)) {
            return false;
        }

        const double yaw_delta =
                std::abs(explorationYawDiff(goal.yaw, robot_state_.yaw));
        const double yaw_rate =
                std::max(0.5, std::isfinite(cfg_.yaw_dot_max) ? cfg_.yaw_dot_max : 3.0);
        const double duration =
                std::clamp(yaw_delta / yaw_rate + 0.25, 0.6, 2.5);
        const double commit_wt = ros_ptr_->getSimTime();

        Trajectory hold_pos_traj;
        if (!buildExplorationConstantPositionTrajectory(robot_state_.p,
                                                        duration,
                                                        commit_wt,
                                                        hold_pos_traj)) {
            ros_ptr_->warn(" -- [Exploration] Yaw-scan recovery commit failed: cannot build hold trajectory.");
            return false;
        }

        latest_replan.setGoal(goal.position, goal.yaw, robot_state_);
        const bool committed =
                commitTaskTrajectory(hold_pos_traj,
                                     goal.yaw,
                                     true,
                                     "exploration_yaw_scan");
        if (!committed) {
            ros_ptr_->warn(" -- [Exploration] Yaw-scan recovery commit failed: task trajectory commit rejected.");
            return false;
        }

        ros_ptr_->info(" -- [Exploration] Yaw-scan recovery committed: yaw_delta={:.3f}, duration={:.3f}, reason={}.",
                       yaw_delta,
                       duration,
                       goal.reason);
        return true;
    }

    RET_CODE GeneralPlanner::PlanExplorationFromRest(const bool &new_task) {
        TimeConsuming total_t("PlanExplorationFromRest", false);
        ExplorationGoal goal;
        {
            std::lock_guard<std::mutex> guard(replan_lock_);
            latest_replan.reset();
            if (!robot_state_.rcv) {
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
                ros_ptr_->warn(" -- [Exploration] PlanFromRest failed: no odom.");
                return FAILED;
            }
            if (map_manager_ == nullptr || !map_manager_->ready()) {
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_MAP_NOT_READY);
                ros_ptr_->warn(" -- [Exploration] PlanFromRest failed: map is not ready.");
                return FAILED;
            }
            if (exploration_frontend_ == nullptr) {
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                ros_ptr_->warn(" -- [Exploration] PlanFromRest failed: frontend is not initialized.");
                return FAILED;
            }
            if (exploration_runtime_manager_ == nullptr) {
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                ros_ptr_->warn(" -- [Exploration] PlanFromRest failed: runtime manager is not initialized.");
                return FAILED;
            }
            if (new_task) {
                exploration_frontend_->reset();
                if (exploration_manager_ != nullptr) {
                    exploration_manager_->reset();
                }
                exploration_runtime_manager_->reset();
            }
            exploration_runtime_manager_->onSelectingGoal();

            const double now = ros_ptr_->getSimTime();
            const StatePVAJ head_state = makeTaskHeadState(true);
            if (!exploration_frontend_->planNextGoal(head_state, robot_state_.yaw, goal, now)) {
                latest_replan.setGoal(robot_state_.p, robot_state_.yaw, robot_state_);
                ExplorationGoal recovery_goal;
                std::string recovery_validation;
                if (exploration_frontend_->isExplorationFinished() &&
                    exploration_runtime_manager_->shouldDelayFinish(now) &&
                    selectValidatedExplorationRecoveryGoal(robot_state_.p, now, recovery_goal, &recovery_validation)) {
                    goal = recovery_goal;
                    exploration_runtime_manager_->onGoalSelected(goal);
                    ros_ptr_->info(" -- [Exploration] Delay finish and use memory recovery goal: candidate_id={}, frontier_id={}, reason={}, validation={}.",
                                   goal.candidate_id,
                                   goal.frontier_id,
                                   goal.reason,
                                   recovery_validation);
                } else if (selectValidatedExplorationRecoveryGoal(robot_state_.p, now, recovery_goal, &recovery_validation)) {
                    goal = recovery_goal;
                    exploration_runtime_manager_->onGoalSelected(goal);
                    ros_ptr_->info(" -- [Exploration] Use memory recovery goal after frontend failure: candidate_id={}, frontier_id={}, reason={}, validation={}.",
                                   goal.candidate_id,
                                   goal.frontier_id,
                                   goal.reason,
                                   recovery_validation);
                } else {
                    ExplorationGoal latest_goal;
                    std::string latest_validation;
                    const bool has_latest_goal =
                            exploration_runtime_manager_->getLatestGoal(latest_goal) &&
                            latest_goal.valid &&
                            latest_goal.position.allFinite() &&
                            (latest_goal.position - robot_state_.p).norm() >
                                    std::max(0.5, cfg_.exploration_goal_reached_distance);
                    if (has_latest_goal &&
                        validateExplorationRecoveryGoal(latest_goal,
                                                        robot_state_.p,
                                                        &latest_validation)) {
                        goal = latest_goal;
                        goal.reason += " plan_from_rest_latest_goal_recovery";
                        exploration_runtime_manager_->onGoalSelected(goal);
                        ros_ptr_->info(" -- [Exploration] Reuse validated latest goal after frontend failure: candidate_id={}, frontier_id={}, validation={}.",
                                       goal.candidate_id,
                                       goal.frontier_id,
                                       latest_validation);
                    } else if (has_latest_goal) {
                        const nhbp::FailureReason failure_reason =
                                latest_validation.find("astar") != std::string::npos ||
                                latest_validation.find("line") != std::string::npos
                                        ? nhbp::FailureReason::ASTAR_FAIL
                                        : nhbp::FailureReason::VIEWPOINT_UNSAFE;
                        exploration_runtime_manager_->recordFailure(latest_goal,
                                                                    failure_reason,
                                                                    now);
                        if (exploration_manager_ != nullptr) {
                            exploration_manager_->recordFailure(latest_goal, now);
                        }
                        ros_ptr_->warn(" -- [Exploration] Reject latest goal after frontend failure: reason={}, candidate_id={}, frontier_id={}.",
                                       latest_validation,
                                       latest_goal.candidate_id,
                                       latest_goal.frontier_id);
                    }
                    if (!goal.valid) {
                        if (exploration_frontend_->isExplorationFinished()) {
                            exploration_runtime_manager_->onFinished(goal);
                            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_EXPLORATION_FINISH);
                            ros_ptr_->info(" -- [Exploration] Exploration finished: {}.", goal.reason);
                            time_consuming_[TOTAL_REPLAN] = total_t.stop();
                            return FINISH;
                        }
                        exploration_runtime_manager_->onTemporaryFailure(goal);
                        latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                        ros_ptr_->warn(" -- [Exploration] Failed to select goal: {}.", goal.reason);
                        time_consuming_[TOTAL_REPLAN] = total_t.stop();
                        return FAILED;
                    }
                }
            }
            const ExplorationRuntimeManager::SelectionDecision decision =
                    exploration_runtime_manager_->stabilizeCandidate(goal,
                                                                     robot_state_.p,
                                                                     0.0,
                                                                     now,
                                                                     new_task);
            if (!decision.ready) {
                ExplorationGoal recovery_goal;
                std::string recovery_validation;
                if (selectValidatedExplorationRecoveryGoal(robot_state_.p, now, recovery_goal, &recovery_validation)) {
                    goal = recovery_goal;
                    exploration_runtime_manager_->onGoalSelected(goal);
                    ros_ptr_->info(" -- [Exploration] Use memory recovery goal after initial NHBP decision: reject_reason={}, recovery_requested={}, candidate_id={}, frontier_id={}, recovery_reason={}, validation={}.",
                                   decision.reason,
                                   static_cast<int>(decision.recovery_requested),
                                   recovery_goal.candidate_id,
                                   recovery_goal.frontier_id,
                                   recovery_goal.reason,
                                   recovery_validation);
                } else if ((decision.allow_candidate_fallback ||
                            goal.reason.find("bootstrap") != std::string::npos) &&
                           goal.valid) {
                    goal.reason += " nhbp_recovery_unavailable=" + decision.reason;
                    exploration_runtime_manager_->onGoalSelected(goal);
                    ros_ptr_->warn(" -- [Exploration] NHBP requested recovery but no validated recovery goal was found, fallback to frontend candidate: reason={}, candidate_id={}, frontier_id={}.",
                                   decision.reason,
                                   goal.candidate_id,
                                   goal.frontier_id);
                } else {
                    exploration_runtime_manager_->onTemporaryFailure(goal);
                    latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                    ros_ptr_->warn(" -- [Exploration] NHBP rejected initial goal: reason={}, ndo={}.",
                                   decision.reason,
                                   nhbp::toString(decision.ndo.state));
                    time_consuming_[TOTAL_REPLAN] = total_t.stop();
                    return FAILED;
                }
            } else {
                goal = decision.goal;
                exploration_runtime_manager_->onGoalSelected(goal);
            }
        }

        {
            std::lock_guard<std::mutex> guard(replan_lock_);
            if (map_manager_ != nullptr) {
                robot_state_ = map_manager_->getRobotState();
            }
        }
        const bool yaw_scan_recovery =
                explorationGoalIsYawScanRecovery(goal,
                                                 robot_state_.p,
                                                 std::max(0.15, cfg_.resolution * 2.0));
        const RET_CODE ret =
                yaw_scan_recovery
                        ? (commitExplorationYawScanTrajectory(goal) ? SUCCESS : FAILED)
                        : PlanFromRest(goal.position, goal.yaw, true);
        {
            std::lock_guard<std::mutex> guard(replan_lock_);
            if (exploration_runtime_manager_ != nullptr) {
                const double now = ros_ptr_->getSimTime();
                if (ret == SUCCESS || ret == FINISH || ret == NO_NEED) {
                    if (exploration_frontend_ != nullptr) {
                        exploration_frontend_->recordGoalCommitted(goal, now, true);
                    }
                    if (exploration_manager_ != nullptr) {
                        exploration_manager_->recordCommitted(goal, robot_state_.p, now);
                    }
                    exploration_runtime_manager_->recordDecision(goal, robot_state_.p, now);
                    exploration_runtime_manager_->onCommitted(goal);
                } else {
                    if (exploration_frontend_ != nullptr) {
                        exploration_frontend_->recordGoalFailed(goal, now);
                    }
                    if (exploration_manager_ != nullptr) {
                        exploration_manager_->recordFailure(goal, now);
                    }
                    exploration_runtime_manager_->recordFailure(goal,
                                                               nhbp::FailureReason::OPTIMIZATION_FAIL,
                                                               now);
                    exploration_runtime_manager_->onTemporaryFailure(ExplorationGoal{});
                    if (cfg_.exploration_print_log) {
                        ros_ptr_->warn(" -- [Exploration] PlanFromRest failure memory summary: {}.",
                                       exploration_runtime_manager_->diagnosticSummary(now));
                    }
                }
            }
        }
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::ReplanExplorationOnce(const bool &new_task) {
        TimeConsuming total_t("ReplanExplorationOnce", false);
        ExplorationGoal selected_goal;
        bool goal_switched = new_task;
        bool selected_goal_ready = false;
        {
            std::lock_guard<std::mutex> guard(replan_lock_);
            latest_replan.reset();
            if (!robot_state_.rcv) {
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
                ros_ptr_->warn(" -- [Exploration] Replan failed: no odom.");
                return FAILED;
            }
            if (map_manager_ == nullptr || !map_manager_->ready()) {
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_MAP_NOT_READY);
                ros_ptr_->warn(" -- [Exploration] Replan failed: map is not ready.");
                return FAILED;
            }
            if (exploration_frontend_ == nullptr) {
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                ros_ptr_->warn(" -- [Exploration] Replan failed: frontend is not initialized.");
                return FAILED;
            }
            if (exploration_runtime_manager_ == nullptr) {
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                ros_ptr_->warn(" -- [Exploration] Replan failed: runtime manager is not initialized.");
                return FAILED;
            }
            if (new_task) {
                exploration_frontend_->reset();
                if (exploration_manager_ != nullptr) {
                    exploration_manager_->reset();
                }
                exploration_runtime_manager_->reset();
            }
            exploration_runtime_manager_->onSelectingGoal();

            const double remaining = getCommittedTrajectoryRemainingDuration();
            const double now = ros_ptr_->getSimTime();
            ExplorationGoal candidate;
            const StatePVAJ head_state = makeTaskHeadState(false);
            if (!exploration_frontend_->planNextGoal(head_state, robot_state_.yaw, candidate, now)) {
                latest_replan.setGoal(robot_state_.p, robot_state_.yaw, robot_state_);
                if (exploration_frontend_->isExplorationFinished()) {
                    ExplorationGoal recovery_goal;
                    std::string recovery_validation;
                    if (exploration_runtime_manager_->shouldDelayFinish(now) &&
                        selectValidatedExplorationRecoveryGoal(robot_state_.p, now, recovery_goal, &recovery_validation)) {
                        selected_goal = recovery_goal;
                        goal_switched = true;
                        selected_goal_ready = true;
                        exploration_runtime_manager_->onGoalSelected(selected_goal);
                        ros_ptr_->info(" -- [Exploration] Delay finish and use memory recovery goal: candidate_id={}, frontier_id={}, reason={}, validation={}.",
                                       selected_goal.candidate_id,
                                       selected_goal.frontier_id,
                                       selected_goal.reason,
                                       recovery_validation);
                    } else {
                        exploration_runtime_manager_->onFinished(candidate);
                        latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_EXPLORATION_FINISH);
                        ros_ptr_->info(" -- [Exploration] Exploration finished: {}.", candidate.reason);
                        time_consuming_[TOTAL_REPLAN] = total_t.stop();
                        return FINISH;
                    }
                }
                if (!selected_goal_ready &&
                    exploration_runtime_manager_->shouldReuseLatestGoal(robot_state_.p, remaining, new_task) &&
                    exploration_runtime_manager_->getLatestGoal(selected_goal)) {
                    goal_switched = false;
                    selected_goal_ready = true;
                    exploration_runtime_manager_->onKeepCurrentGoal();
                    if (cfg_.exploration_print_log) {
                        ros_ptr_->info(" -- [Exploration] Reuse current goal after temporary frontend failure: reason={}, remaining={:.3f}.",
                                       candidate.reason,
                                       remaining);
                    }
                } else if (!selected_goal_ready) {
                    ExplorationGoal recovery_goal;
                    std::string recovery_validation;
                    if (selectValidatedExplorationRecoveryGoal(robot_state_.p, now, recovery_goal, &recovery_validation)) {
                        selected_goal = recovery_goal;
                        goal_switched = true;
                        selected_goal_ready = true;
                        exploration_runtime_manager_->onGoalSelected(selected_goal);
                        ros_ptr_->info(" -- [Exploration] Use memory recovery goal after frontend failure: candidate_id={}, frontier_id={}, reason={}, validation={}.",
                                       selected_goal.candidate_id,
                                       selected_goal.frontier_id,
                                       selected_goal.reason,
                                       recovery_validation);
                    } else {
                        exploration_runtime_manager_->onTemporaryFailure(candidate);
                        latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                        ros_ptr_->warn(" -- [Exploration] Failed to select replan goal: {}.", candidate.reason);
                        time_consuming_[TOTAL_REPLAN] = total_t.stop();
                        return FAILED;
                    }
                }
            } else {
                const ExplorationRuntimeManager::SelectionDecision decision =
                        exploration_runtime_manager_->stabilizeCandidate(candidate,
                                                                         robot_state_.p,
                                                                         remaining,
                                                                         now,
                                                                         new_task);
                if (decision.ready && decision.keep_current) {
                    selected_goal = decision.goal;
                    goal_switched = false;
                    exploration_runtime_manager_->onKeepCurrentGoal();
                    if (cfg_.exploration_print_log) {
                        ros_ptr_->info(" -- [Exploration] NHBP keep current goal: reason={}, ndo={}, remaining={:.3f}, current_score={:.3f}, candidate_score={:.3f}.",
                                       decision.reason,
                                       nhbp::toString(decision.ndo.state),
                                       remaining,
                                       selected_goal.score,
                                       candidate.score);
                    }
                } else if (decision.ready) {
                    selected_goal = decision.goal;
                    goal_switched = true;
                    exploration_runtime_manager_->onGoalSelected(selected_goal);
                    if (cfg_.exploration_print_log) {
                        ros_ptr_->info(" -- [Exploration] NHBP accept goal: reason={}, ndo={}, candidate_id={}, frontier_id={}.",
                                       decision.reason,
                                       nhbp::toString(decision.ndo.state),
                                       selected_goal.candidate_id,
                                       selected_goal.frontier_id);
                    }
                } else {
                    ExplorationGoal recovery_goal;
                    std::string recovery_validation;
                    if (selectValidatedExplorationRecoveryGoal(robot_state_.p, now, recovery_goal, &recovery_validation)) {
                        selected_goal = recovery_goal;
                        goal_switched = true;
                        exploration_runtime_manager_->onGoalSelected(selected_goal);
                        ros_ptr_->info(" -- [Exploration] Use memory recovery goal after NHBP decision: reject_reason={}, recovery_requested={}, candidate_id={}, frontier_id={}, recovery_candidate_id={}, recovery_frontier_id={}, validation={}.",
                                       decision.reason,
                                       static_cast<int>(decision.recovery_requested),
                                       candidate.candidate_id,
                                       candidate.frontier_id,
                                       selected_goal.candidate_id,
                                       selected_goal.frontier_id,
                                       recovery_validation);
                    } else if ((decision.allow_candidate_fallback ||
                                candidate.reason.find("bootstrap") != std::string::npos) &&
                               candidate.valid) {
                        selected_goal = candidate;
                        selected_goal.reason += " nhbp_recovery_unavailable=" + decision.reason;
                        goal_switched = true;
                        exploration_runtime_manager_->onGoalSelected(selected_goal);
                        ros_ptr_->warn(" -- [Exploration] NHBP requested recovery but no validated recovery goal was found, fallback to frontend candidate: reason={}, candidate_id={}, frontier_id={}.",
                                       decision.reason,
                                       selected_goal.candidate_id,
                                       selected_goal.frontier_id);
                    } else {
                        exploration_runtime_manager_->onTemporaryFailure(candidate);
                        latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                        ros_ptr_->warn(" -- [Exploration] NHBP rejected replan goal: reason={}, ndo={}, candidate_id={}, frontier_id={}.",
                                       decision.reason,
                                       nhbp::toString(decision.ndo.state),
                                       candidate.candidate_id,
                                       candidate.frontier_id);
                        time_consuming_[TOTAL_REPLAN] = total_t.stop();
                        return FAILED;
                    }
                }
                selected_goal_ready = true;
            }
        }

        if (!selected_goal_ready) {
            return FAILED;
        }
        {
            std::lock_guard<std::mutex> guard(replan_lock_);
            if (map_manager_ != nullptr) {
                robot_state_ = map_manager_->getRobotState();
            }
        }
        const bool yaw_scan_recovery =
                explorationGoalIsYawScanRecovery(selected_goal,
                                                 robot_state_.p,
                                                 std::max(0.15, cfg_.resolution * 2.0));
        const RET_CODE ret =
                yaw_scan_recovery
                        ? (commitExplorationYawScanTrajectory(selected_goal) ? SUCCESS : FAILED)
                        : ReplanOnce(selected_goal.position, selected_goal.yaw, goal_switched);
        {
            std::lock_guard<std::mutex> guard(replan_lock_);
            if (exploration_runtime_manager_ != nullptr) {
                const double now = ros_ptr_->getSimTime();
                if (ret == SUCCESS || ret == FINISH || ret == NO_NEED) {
                    if (exploration_frontend_ != nullptr) {
                        exploration_frontend_->recordGoalCommitted(selected_goal,
                                                                   now,
                                                                   goal_switched);
                    }
                    if (exploration_manager_ != nullptr) {
                        exploration_manager_->recordCommitted(selected_goal, robot_state_.p, now);
                    }
                    exploration_runtime_manager_->recordDecision(selected_goal, robot_state_.p, now);
                    exploration_runtime_manager_->onCommitted(selected_goal);
                } else {
                    if (exploration_frontend_ != nullptr) {
                        exploration_frontend_->recordGoalFailed(selected_goal, now);
                    }
                    if (exploration_manager_ != nullptr) {
                        exploration_manager_->recordFailure(selected_goal, now);
                    }
                    exploration_runtime_manager_->recordFailure(selected_goal,
                                                               nhbp::FailureReason::OPTIMIZATION_FAIL,
                                                               now);
                    exploration_runtime_manager_->onTemporaryFailure(ExplorationGoal{});
                    if (cfg_.exploration_print_log) {
                        ros_ptr_->warn(" -- [Exploration] Replan failure memory summary: {}.",
                                       exploration_runtime_manager_->diagnosticSummary(now));
                    }
                }
            }
        }
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    bool GeneralPlanner::getLatestExplorationGoal(ExplorationGoal &goal) const {
        std::lock_guard<std::mutex> guard(replan_lock_);
        return exploration_runtime_manager_ != nullptr &&
               exploration_runtime_manager_->getLatestGoal(goal);
    }

}
