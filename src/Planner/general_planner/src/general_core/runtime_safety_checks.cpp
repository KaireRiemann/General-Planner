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

namespace general_planner {

    bool GeneralPlanner::checkPositionTrajectorySafety(
            const Trajectory &traj,
            const double now_wt,
            const double horizon,
            const double dt,
            const int consecutive_hits,
            const bool unknown_as_occupied,
            CommittedTrajectorySafetyReport *report) const {
        CommittedTrajectorySafetyReport local_report;
        local_report.safe = true;
        local_report.valid = false;

        const auto fill_report = [&]() {
            if (report != nullptr) {
                *report = local_report;
            }
        };

        if (map_manager_ == nullptr || !map_manager_->ready()) {
            local_report.reason = "map_not_ready";
            fill_report();
            return true;
        }

        if (traj.empty()) {
            local_report.safe = false;
            local_report.reason = "empty_trajectory";
            fill_report();
            return false;
        }

        const double total_duration = traj.getTotalDuration();
        if (!std::isfinite(total_duration) || total_duration <= 1.0e-6 ||
            !std::isfinite(traj.start_WT)) {
            local_report.safe = false;
            local_report.reason = "invalid_trajectory_time";
            fill_report();
            return false;
        }

        const double current_t = std::clamp(now_wt - traj.start_WT, 0.0, total_duration);
        const double remaining = std::max(0.0, total_duration - current_t);
        local_report.valid = true;
        local_report.check_start_t = current_t;
        local_report.check_horizon = horizon > 0.0 ? std::min(horizon, remaining) : remaining;

        if (remaining <= 1.0e-3 || local_report.check_horizon <= 1.0e-3) {
            local_report.reason = "trajectory_finished_or_horizon_empty";
            fill_report();
            return true;
        }

        const double sample_dt = std::max(0.02, dt);
        const int required_hits = std::max(1, consecutive_hits);
        Vec3f last_pos = traj.getPos(current_t);
        if (!last_pos.allFinite()) {
            local_report.safe = false;
            local_report.reason = "invalid_start_position";
            local_report.collision_t = current_t;
            local_report.time_to_collision = 0.0;
            fill_report();
            return false;
        }

        int hit_streak = 0;
        double streak_start_t = current_t;
        Vec3f streak_start_pos = last_pos;
        rog_map::GridType streak_grid = rog_map::GridType::KNOWN_FREE;
        std::string streak_reason;

        const auto unsafe_grid = [unknown_as_occupied](const rog_map::GridType grid_type) {
            return grid_type == rog_map::GridType::OCCUPIED ||
                   grid_type == rog_map::GridType::OUT_OF_MAP ||
                   (unknown_as_occupied && grid_type == rog_map::GridType::UNKNOWN);
        };
        const bool line_unknown_as_occupied =
                unknown_as_occupied &&
                map_manager_ != nullptr &&
                map_manager_->ready() &&
                map_manager_->getMapConfig().unk_inflation_en;
        const bool dynamic_layer_active =
                dynamic_obstacle_layer_ != nullptr && dynamic_obstacle_layer_->enabled();

        for (double offset = 0.0;
             offset <= local_report.check_horizon + 1.0e-6;
             offset += sample_dt) {
            const double t = std::min(total_duration, current_t + offset);
            const Vec3f pos = traj.getPos(t);
            Vec3f unsafe_pos = pos;
            bool unsafe = false;
            rog_map::GridType grid_type = rog_map::GridType::KNOWN_FREE;
            std::string reason;

            if (!pos.allFinite()) {
                unsafe = true;
                reason = "invalid_sample_position";
            } else if (!map_manager_->insideLocalMap(pos)) {
                unsafe = true;
                grid_type = rog_map::GridType::OUT_OF_MAP;
                reason = "out_of_local_map";
            } else {
                grid_type = map_manager_->getInfGridType(pos);
                if (unsafe_grid(grid_type)) {
                    unsafe = true;
                    if (grid_type == rog_map::GridType::OCCUPIED) {
                        reason = "occupied_inflated_cell";
                    } else if (grid_type == rog_map::GridType::UNKNOWN) {
                        reason = "unknown_inflated_cell";
                    } else {
                        reason = "out_of_map_cell";
                    }
                }
                if (!unsafe && dynamic_layer_active) {
                    DynamicObstacleLayer::QueryResult dynamic_query;
                    if (dynamic_obstacle_layer_->pointOccupied(pos, now_wt, &dynamic_query)) {
                        unsafe = true;
                        grid_type = rog_map::GridType::OCCUPIED;
                        unsafe_pos = dynamic_query.hit_pos;
                        reason = "dynamic_obstacle_cell";
                    }
                }
                if (!unsafe &&
                    (pos - last_pos).norm() > 1.0e-4 &&
                    dynamic_layer_active) {
                    DynamicObstacleLayer::QueryResult dynamic_query;
                    if (dynamic_obstacle_layer_->lineOccupied(last_pos, pos, now_wt, &dynamic_query)) {
                        unsafe = true;
                        grid_type = rog_map::GridType::OCCUPIED;
                        unsafe_pos = dynamic_query.hit_pos;
                        reason = "dynamic_obstacle_line";
                    }
                }
                if (!unsafe &&
                    (pos - last_pos).norm() > 1.0e-4 &&
                    !map_manager_->isLineFree(last_pos, pos, true, line_unknown_as_occupied)) {
                    unsafe = true;
                    reason = "line_collision";
                }
            }

            if (unsafe) {
                if (hit_streak == 0) {
                    streak_start_t = t;
                    streak_start_pos = unsafe_pos;
                    streak_grid = grid_type;
                    streak_reason = reason;
                }
                ++hit_streak;
                if (hit_streak >= required_hits) {
                    local_report.safe = false;
                    local_report.collision_t = streak_start_t;
                    local_report.time_to_collision = std::max(0.0, streak_start_t - current_t);
                    local_report.collision_pos = streak_start_pos;
                    local_report.grid_type = static_cast<int>(streak_grid);
                    local_report.hit_count = hit_streak;
                    local_report.reason = streak_reason;
                    fill_report();
                    return false;
                }
            } else {
                hit_streak = 0;
            }

            last_pos = pos;
        }

        local_report.reason = "safe";
        fill_report();
        return true;
    }

    bool GeneralPlanner::checkCommittedPositionTrajectorySafety(
            const double horizon,
            const double dt,
            const int consecutive_hits,
            const bool unknown_as_occupied,
            CommittedTrajectorySafetyReport *report) {
        cmd_traj_info_.lock();
        const Trajectory traj = cmd_traj_info_.posTraj();
        const bool backup_available = cmd_traj_info_.backupTrajAvilibale();
        const double backup_start_t = cmd_traj_info_.getBackupTrajStartTT();
        cmd_traj_info_.unlock();
        const double now_wt = ros_ptr_->getSimTime();
        double effective_horizon = horizon;
        if (backup_available && !traj.empty() && std::isfinite(backup_start_t)) {
            const double current_t = std::clamp(now_wt - traj.start_WT,
                                                0.0,
                                                traj.getTotalDuration());
            if (current_t >= backup_start_t - 1.0e-3) {
                if (report != nullptr) {
                    report->valid = true;
                    report->safe = true;
                    report->check_start_t = current_t;
                    report->check_horizon = 0.0;
                    report->reason = "on_backup_trajectory";
                }
                return true;
            }
            effective_horizon = std::min(effective_horizon,
                                         std::max(0.0, backup_start_t - current_t));
            if (effective_horizon <= 1.0e-3) {
                if (report != nullptr) {
                    report->valid = true;
                    report->safe = true;
                    report->check_start_t = current_t;
                    report->check_horizon = 0.0;
                    report->reason = "backup_boundary_reached";
                }
                return true;
            }
        }
        return checkPositionTrajectorySafety(traj,
                                             now_wt,
                                             effective_horizon,
                                             dt,
                                             consecutive_hits,
                                             unknown_as_occupied,
                                             report);
    }

}
