/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

#include <algorithm>
#include <mutex>
#include <general_utils/scope_timer.hpp>

using namespace general_utils;

namespace general_planner {

    ExplorationFrontend::Config GeneralPlanner::makeExplorationFrontendConfig() const {
        ExplorationFrontend::Config frontend_cfg;
        frontend_cfg.enable = cfg_.exploration_enable;
        frontend_cfg.print_log = cfg_.exploration_print_log;
        frontend_cfg.map_resolution = std::max(0.2, cfg_.resolution);
        frontend_cfg.frontier_search_radius = cfg_.exploration_frontier_search_radius;
        frontend_cfg.frontier_cluster_radius = cfg_.exploration_frontier_cluster_radius;
        frontend_cfg.min_frontier_cluster_size = cfg_.exploration_min_frontier_cluster_size;
        frontend_cfg.viewpoint_min_distance = cfg_.exploration_viewpoint_min_distance;
        frontend_cfg.viewpoint_max_distance = cfg_.exploration_viewpoint_max_distance;
        frontend_cfg.viewpoint_height_offset = cfg_.exploration_viewpoint_height_offset;
        frontend_cfg.viewpoint_safe_distance = cfg_.exploration_viewpoint_safe_distance;
        frontend_cfg.viewpoint_yaw_sample_num = cfg_.exploration_viewpoint_yaw_sample_num;
        frontend_cfg.viewpoint_radius_sample_num = cfg_.exploration_viewpoint_radius_sample_num;
        frontend_cfg.max_candidate_num = cfg_.exploration_max_candidate_num;
        frontend_cfg.weight_travel = cfg_.exploration_weight_travel;
        frontend_cfg.weight_yaw = cfg_.exploration_weight_yaw;
        frontend_cfg.weight_curvature = cfg_.exploration_weight_curvature;
        frontend_cfg.weight_info_gain = cfg_.exploration_weight_info_gain;
        frontend_cfg.weight_unknown_risk = cfg_.exploration_weight_unknown_risk;
        frontend_cfg.min_information_gain = cfg_.exploration_min_information_gain;
        frontend_cfg.goal_switch_min_score_improvement = cfg_.exploration_goal_switch_min_score_improvement;
        frontend_cfg.goal_reached_distance = cfg_.exploration_goal_reached_distance;
        frontend_cfg.unknown_as_occupied_for_motion = cfg_.exploration_unknown_as_occupied_for_motion;
        frontend_cfg.require_line_free_to_frontier = cfg_.exploration_require_line_free_to_frontier;
        frontend_cfg.use_astar_cost = cfg_.exploration_use_astar_cost;
        return frontend_cfg;
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
                exploration_runtime_manager_->reset();
            }
            exploration_runtime_manager_->onSelectingGoal();

            const StatePVAJ head_state = makeTaskHeadState(true);
            if (!exploration_frontend_->planNextGoal(head_state, robot_state_.yaw, goal)) {
                latest_replan.setGoal(robot_state_.p, robot_state_.yaw, robot_state_);
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
            exploration_runtime_manager_->onGoalSelected(goal);
        }

        const RET_CODE ret = PlanFromRest(goal.position, goal.yaw, true);
        {
            std::lock_guard<std::mutex> guard(replan_lock_);
            if (exploration_runtime_manager_ != nullptr) {
                if (ret == SUCCESS || ret == FINISH || ret == NO_NEED) {
                    exploration_runtime_manager_->onCommitted(goal);
                } else {
                    exploration_runtime_manager_->onTemporaryFailure(goal);
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
                exploration_runtime_manager_->reset();
            }
            exploration_runtime_manager_->onSelectingGoal();

            const double remaining = getCommittedTrajectoryRemainingDuration();
            ExplorationGoal candidate;
            const StatePVAJ head_state = makeTaskHeadState(false);
            if (!exploration_frontend_->planNextGoal(head_state, robot_state_.yaw, candidate)) {
                latest_replan.setGoal(robot_state_.p, robot_state_.yaw, robot_state_);
                if (exploration_frontend_->isExplorationFinished()) {
                    exploration_runtime_manager_->onFinished(candidate);
                    latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_EXPLORATION_FINISH);
                    ros_ptr_->info(" -- [Exploration] Exploration finished: {}.", candidate.reason);
                    time_consuming_[TOTAL_REPLAN] = total_t.stop();
                    return FINISH;
                }
                if (exploration_runtime_manager_->shouldReuseLatestGoal(robot_state_.p, remaining, new_task) &&
                    exploration_runtime_manager_->getLatestGoal(selected_goal)) {
                    goal_switched = false;
                    selected_goal_ready = true;
                    exploration_runtime_manager_->onKeepCurrentGoal();
                    if (cfg_.exploration_print_log) {
                        ros_ptr_->info(" -- [Exploration] Reuse current goal after temporary frontend failure: reason={}, remaining={:.3f}.",
                                       candidate.reason,
                                       remaining);
                    }
                } else {
                    exploration_runtime_manager_->onTemporaryFailure(candidate);
                    latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_UNDEFINED);
                    ros_ptr_->warn(" -- [Exploration] Failed to select replan goal: {}.", candidate.reason);
                    time_consuming_[TOTAL_REPLAN] = total_t.stop();
                    return FAILED;
                }
            } else {
                if (exploration_runtime_manager_->shouldKeepCurrentGoal(candidate, robot_state_.p, remaining, new_task) &&
                    exploration_runtime_manager_->getLatestGoal(selected_goal)) {
                    goal_switched = false;
                    exploration_runtime_manager_->onKeepCurrentGoal();
                    if (cfg_.exploration_print_log) {
                        ros_ptr_->info(" -- [Exploration] Keep current goal: remaining={:.3f}, current_score={:.3f}, candidate_score={:.3f}.",
                                       remaining,
                                       selected_goal.score,
                                       candidate.score);
                    }
                } else {
                    selected_goal = candidate;
                    goal_switched = true;
                    exploration_runtime_manager_->onGoalSelected(candidate);
                }
                selected_goal_ready = true;
            }
        }

        if (!selected_goal_ready) {
            return FAILED;
        }
        const RET_CODE ret = ReplanOnce(selected_goal.position, selected_goal.yaw, goal_switched);
        {
            std::lock_guard<std::mutex> guard(replan_lock_);
            if (exploration_runtime_manager_ != nullptr) {
                if (ret == SUCCESS || ret == FINISH || ret == NO_NEED) {
                    exploration_runtime_manager_->onCommitted(selected_goal);
                } else {
                    exploration_runtime_manager_->onTemporaryFailure(selected_goal);
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
