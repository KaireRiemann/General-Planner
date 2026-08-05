#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <checker/check_result.hpp>
#include <checker/common_checker.hpp>
#include <general_core/config.hpp>
#include <map_manager/map_manager.hpp>
#include <rog_map/rog_map.h>

namespace general_planner::checker {
    inline CheckResult checkState2StateConfig(const Config &cfg) {
        if (!std::isfinite(cfg.resolution) || cfg.resolution <= 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_RESOLUTION", "rog_map/resolution must be positive");
        }
        if (!std::isfinite(cfg.replan_forward_dt) || cfg.replan_forward_dt <= 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_REPLAN_FORWARD_DT", "replan_forward_dt must be positive");
        }
        if (!std::isfinite(cfg.planning_horizon) || cfg.planning_horizon <= cfg.resolution) {
            return CheckResult::Fatal("CONFIG_BAD_PLANNING_HORIZON", "planning_horizon is too small");
        }
        if (!std::isfinite(cfg.robot_r) || cfg.robot_r <= 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_ROBOT_RADIUS", "robot_r must be positive");
        }
        if (!std::isfinite(cfg.exp_traj_cfg.max_vel) || cfg.exp_traj_cfg.max_vel <= 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_MAX_VEL", "max_vel must be positive");
        }
        if (!std::isfinite(cfg.exp_traj_cfg.max_acc) || cfg.exp_traj_cfg.max_acc <= 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_MAX_ACC", "max_acc must be positive");
        }
        if (!std::isfinite(cfg.exp_traj_cfg.max_jerk) || cfg.exp_traj_cfg.max_jerk <= 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_MAX_JERK", "max_jerk must be positive");
        }
        if (!std::isfinite(cfg.sample_traj_dt) || cfg.sample_traj_dt <= 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_SAMPLE_DT", "sample_traj_dt must be positive");
        }
        if (cfg.esdf_traj_en && cfg.plain_traj_en) {
            return CheckResult::Fatal("CONFIG_EXP_MODE_CONFLICT", "esdf_traj_en and plain_traj_en cannot both be true");
        }
        if (cfg.backup_traj_en && (cfg.esdf_traj_en || cfg.plain_traj_en)) {
            return CheckResult::Warn("CONFIG_BACKUP_DISABLED_BY_EXP_MODE",
                                     "backup_traj_en is ignored in ESDF/plain mode");
        }
        if (!std::isfinite(cfg.state2state_altitude_band) ||
            cfg.state2state_altitude_band < 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_STATE2STATE_ALTITUDE_BAND",
                                      "state2state altitude_band must be non-negative");
        }
        if (!std::isfinite(cfg.state2state_altitude_escape_band) ||
            cfg.state2state_altitude_escape_band < cfg.state2state_altitude_band) {
            return CheckResult::Fatal("CONFIG_BAD_STATE2STATE_ALTITUDE_ESCAPE_BAND",
                                      "state2state altitude_escape_band must be >= altitude_band");
        }
        if (!std::isfinite(cfg.state2state_over_goal_tolerance) ||
            cfg.state2state_over_goal_tolerance < 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_STATE2STATE_OVER_GOAL_TOLERANCE",
                                      "state2state over_goal_tolerance must be non-negative");
        }
        if (!std::isfinite(cfg.state2state_near_goal_radius) ||
            cfg.state2state_near_goal_radius <= 0.0) {
            return CheckResult::Fatal("CONFIG_BAD_STATE2STATE_NEAR_GOAL_RADIUS",
                                      "state2state near_goal_radius must be positive");
        }
        return CheckResult::Ok();
    }

    inline CheckResult checkState2StateInput(const Vec3f &goal_p,
                                             const double goal_yaw,
                                             const rog_map::RobotState &robot_state,
                                             const MapManager::Ptr &map_manager,
                                             const double now,
                                             const double max_odom_age = 0.5) {
        if (!goal_p.allFinite()) {
            return CheckResult::Reject("GOAL_NON_FINITE", "state2state goal contains NaN or Inf");
        }
        if (!yawValueValid(goal_yaw)) {
            return CheckResult::Reject("GOAL_YAW_INVALID", "state2state goal yaw is Inf");
        }
        if (map_manager == nullptr || !map_manager->ready()) {
            return CheckResult::Reject("MAP_NOT_READY", "map manager is not ready");
        }
        if (!robot_state.rcv) {
            return CheckResult::Reject("ODOM_NOT_RECEIVED", "robot odom has not been received");
        }
        if (robot_state.rcv_time > 0.0 && std::isfinite(now) &&
            now - robot_state.rcv_time > max_odom_age) {
            return CheckResult::Reject("ODOM_STALE", "robot odom is stale");
        }
        if (!robot_state.p.allFinite() || !robot_state.v.allFinite() ||
            !std::isfinite(robot_state.yaw)) {
            return CheckResult::Reject("ROBOT_STATE_NON_FINITE", "robot state contains NaN or Inf");
        }
        if (!std::isfinite(robot_state.q.w()) || !std::isfinite(robot_state.q.x()) ||
            !std::isfinite(robot_state.q.y()) || !std::isfinite(robot_state.q.z()) ||
            robot_state.q.norm() < 1.0e-6) {
            return CheckResult::Reject("ROBOT_QUAT_INVALID", "robot quaternion is invalid");
        }
        return CheckResult::Ok();
    }

    inline CheckResult checkGuidePath(const vec_Vec3f &path,
                                      const std::vector<double> &stamps,
                                      const double resolution,
                                      const std::string &name) {
        if (path.size() < 2) {
            return CheckResult::Reject(name + "_GUIDE_TOO_SHORT", name + " guide path has fewer than 2 points");
        }
        if (path.size() != stamps.size()) {
            return CheckResult::Reject(name + "_GUIDE_STAMP_SIZE_MISMATCH",
                                       name + " guide path and stamp sizes differ");
        }
        double last_stamp = -1.0e-9;
        for (std::size_t i = 0; i < path.size(); ++i) {
            if (!path[i].allFinite()) {
                return CheckResult::Reject(name + "_GUIDE_POINT_NON_FINITE",
                                           name + " guide point is NaN/Inf at " + std::to_string(i));
            }
            if (!std::isfinite(stamps[i]) || stamps[i] + 1.0e-8 < last_stamp) {
                return CheckResult::Reject(name + "_GUIDE_STAMP_INVALID",
                                           name + " guide stamp is invalid at " + std::to_string(i));
            }
            last_stamp = stamps[i];
        }
        if ((path.back() - path.front()).norm() < std::max(1.0e-3, resolution)) {
            return CheckResult::Reject(name + "_GUIDE_TOO_SHORT_DISTANCE",
                                       name + " guide path length is too short");
        }
        return CheckResult::Ok();
    }

    inline CheckResult checkHighSpeedSafetyMargin(const double speed,
                                                  const double max_acc,
                                                  const double replan_forward_dt,
                                                  const double sensing_horizon,
                                                  const double safe_corridor_line_max_length,
                                                  const double robot_radius) {
        if (!std::isfinite(speed) || speed < 0.0 ||
            !std::isfinite(max_acc) || max_acc <= 1.0e-6) {
            return CheckResult::Reject("SPEED_MARGIN_INPUT_INVALID", "speed margin input is invalid");
        }
        const double stopping_distance = speed * speed / (2.0 * max_acc);
        const double reaction_distance = speed * std::max(0.0, replan_forward_dt);
        bool has_visible_distance = false;
        double visible_distance = std::numeric_limits<double>::infinity();
        if (std::isfinite(sensing_horizon) && sensing_horizon > 0.0) {
            visible_distance = sensing_horizon;
            has_visible_distance = true;
        }
        if (std::isfinite(safe_corridor_line_max_length) && safe_corridor_line_max_length > 0.0) {
            visible_distance = has_visible_distance
                               ? std::min(visible_distance, safe_corridor_line_max_length)
                               : safe_corridor_line_max_length;
            has_visible_distance = true;
        }
        const double required_distance = stopping_distance + reaction_distance + std::max(0.0, robot_radius);
        if (!has_visible_distance) {
            return CheckResult::Warn("HIGH_SPEED_MARGIN_UNKNOWN",
                                     "no positive sensing_horizon or safe_corridor_line_max_length configured");
        }
        if (required_distance > visible_distance) {
            return CheckResult::Warn(
                    "HIGH_SPEED_MARGIN_LOW",
                    "required_stop_margin=" + std::to_string(required_distance) +
                    ", visible_distance=" + std::to_string(visible_distance) +
                    ", speed=" + std::to_string(speed));
        }
        return CheckResult::Ok();
    }
}
