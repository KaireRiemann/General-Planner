/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/state2state/state2state_exp_backup_backend.hpp>

#include <fmt/color.h>
#include <utils/header/type_utils.hpp>
#include <data_structure/base/polytope.h>

namespace general_planner {
using namespace general_utils;
using namespace geometry_utils;
}

#include <data_structure/cmd_traj.h>
#include <data_structure/exp_traj.h>
#include <checker/state2state_checker.hpp>
#include <checker/trajectory_checker.hpp>
#include <general_core/config.hpp>
#include <general_core/corridor_generator.h>
#include <map_manager/map_manager.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <traj_opt/traj_manager.h>
#include <general_core/log_utils.hpp>
#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <limits>
#include <general_utils/scope_timer.hpp>
#include <vector>

using namespace general_utils;

namespace general_planner {
    namespace {
        struct LocalZSummary {
            bool valid{false};
            double start{0.0};
            double end{0.0};
            double min{0.0};
            double max{0.0};
        };

        LocalZSummary summarizePathZ(const vec_Vec3f &path) {
            LocalZSummary out;
            if (path.empty()) {
                return out;
            }
            bool initialized = false;
            for (const auto &point : path) {
                if (!point.allFinite() || !std::isfinite(point.z())) {
                    continue;
                }
                if (!initialized) {
                    out.start = point.z();
                    out.min = point.z();
                    out.max = point.z();
                    initialized = true;
                }
                out.end = point.z();
                out.min = std::min(out.min, point.z());
                out.max = std::max(out.max, point.z());
            }
            out.valid = initialized;
            return out;
        }

        LocalZSummary summarizeTrajectoryZ(const Trajectory &traj, double sample_dt) {
            LocalZSummary out;
            if (traj.empty()) {
                return out;
            }
            const double duration = traj.getTotalDuration();
            if (!std::isfinite(duration) || duration < 0.0) {
                return out;
            }
            sample_dt = std::max(0.02, sample_dt);
            auto addSample = [&](const double t) {
                const Vec3f pos = traj.getPos(std::clamp(t, 0.0, duration));
                if (!pos.allFinite() || !std::isfinite(pos.z())) {
                    return;
                }
                if (!out.valid) {
                    out.start = pos.z();
                    out.min = pos.z();
                    out.max = pos.z();
                    out.valid = true;
                }
                out.end = pos.z();
                out.min = std::min(out.min, pos.z());
                out.max = std::max(out.max, pos.z());
            };
            addSample(0.0);
            for (double t = sample_dt; t < duration; t += sample_dt) {
                addSample(t);
            }
            addSample(duration);
            return out;
        }

        struct GuideZLowerGuardReport {
            bool enabled{false};
            bool valid{true};
            int samples{0};
            double min_margin{0.0};
            double max_violation{0.0};

            bool passed() const {
                return valid && max_violation <= 1.0e-6;
            }
        };

        bool interpolateGuideZ(const vec_Vec3f &guide_path,
                               const std::vector<double> &guide_stamp,
                               double query_t,
                               double &z) {
            if (guide_path.size() < 2 || guide_path.size() != guide_stamp.size() ||
                !std::isfinite(query_t)) {
                return false;
            }
            const double start_t = guide_stamp.front();
            const double end_t = guide_stamp.back();
            if (!std::isfinite(start_t) || !std::isfinite(end_t) || end_t <= start_t) {
                return false;
            }
            const double clamped_t = std::clamp(query_t, start_t, end_t);
            for (std::size_t i = 0; i + 1 < guide_path.size(); ++i) {
                const double t0 = guide_stamp[i];
                const double t1 = guide_stamp[i + 1];
                if (!std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0 ||
                    !guide_path[i].allFinite() || !guide_path[i + 1].allFinite()) {
                    continue;
                }
                if (clamped_t < t0 || (clamped_t > t1 && i + 2 < guide_path.size())) {
                    continue;
                }
                const double alpha = std::clamp((clamped_t - t0) / (t1 - t0), 0.0, 1.0);
                z = guide_path[i].z() + alpha * (guide_path[i + 1].z() - guide_path[i].z());
                return std::isfinite(z);
            }
            return false;
        }

        GuideZLowerGuardReport checkGuideZLowerBound(const Trajectory &traj,
                                                      const vec_Vec3f &guide_path,
                                                      const std::vector<double> &guide_stamp,
                                                      bool enabled,
                                                      double tolerance,
                                                      double sample_dt) {
            GuideZLowerGuardReport report;
            report.enabled = enabled;
            if (!enabled) {
                return report;
            }
            tolerance = std::max(0.0, tolerance);
            sample_dt = std::max(0.01, sample_dt);
            const double duration = traj.getTotalDuration();
            if (traj.empty() || !std::isfinite(duration) || duration < 0.0) {
                report.valid = false;
                return report;
            }
            report.min_margin = std::numeric_limits<double>::infinity();
            auto evaluate = [&](const double t) {
                double guide_z = 0.0;
                const Vec3f pos = traj.getPos(std::clamp(t, 0.0, duration));
                if (!pos.allFinite() || !interpolateGuideZ(guide_path, guide_stamp, t, guide_z)) {
                    report.valid = false;
                    return;
                }
                const double margin = pos.z() - (guide_z - tolerance);
                report.min_margin = std::min(report.min_margin, margin);
                report.max_violation = std::max(report.max_violation, std::max(0.0, -margin));
                ++report.samples;
            };
            evaluate(0.0);
            for (double t = sample_dt; t < duration; t += sample_dt) {
                evaluate(t);
            }
            evaluate(duration);
            if (report.samples == 0 || !std::isfinite(report.min_margin)) {
                report.valid = false;
            }
            return report;
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

        bool currentTrajectorySafeForNoNeed(state2state_task::StateToStateExpBackendServices &services,
                                            const Trajectory &traj,
                                            const double start_t) {
            if (traj.empty()) {
                return false;
            }
            const double total_duration = traj.getTotalDuration();
            if (!std::isfinite(total_duration)) {
                return false;
            }
            if (start_t >= total_duration) {
                return true;
            }
            CommittedTrajectorySafetyReport report;
            const double now_wt = traj.start_WT + std::clamp(start_t, 0.0, total_duration);
            double horizon = std::max(0.0, total_duration - start_t);
            const double backup_start_t = services.cmd_traj_info.getBackupTrajStartTT();
            if (std::isfinite(backup_start_t) && backup_start_t > 0.0 && backup_start_t < total_duration) {
                if (start_t >= backup_start_t - 1.0e-3) {
                    return false;
                }
                horizon = std::min(horizon, std::max(0.0, backup_start_t - start_t));
                if (horizon <= 1.0e-3) {
                    return false;
                }
            }
            const double dt = std::max(services.cfg.sample_traj_dt, services.cfg.resolution);
            const bool safe = checkPositionTrajectorySafety(services.runtime_safety,
                                                            traj,
                                                            now_wt,
                                                            horizon,
                                                            dt,
                                                            1,
                                                            false,
                                                            &report);
            if (!safe && services.cfg.print_log && services.ros_ptr != nullptr) {
                services.ros_ptr->warn(" -- [GeneralPlanner] Dynamic safety guard blocks NO_NEED: reason={}, ttc={:.3f}, pos=({:.2f},{:.2f},{:.2f}).",
                                       report.reason,
                                       report.time_to_collision,
                                       report.collision_pos.x(),
                                       report.collision_pos.y(),
                                       report.collision_pos.z());
            }
            return safe;
        }
    }

    namespace state2state_task {

    RET_CODE generateExpTrajectory(StateToStateExpBackendServices &services,
                                   ExpTraj &last_exp_traj_info,
                                   ExpTraj &out_exp_traj_info) {
        TimeConsuming t_exp_frontend("t_exp_frontend", false);
        services.z_debug = State2StateZDebug{};
        services.z_debug.goal_z = services.goal_p.z();
        services.z_debug.robot_z = services.robot_state.p.z();
        services.z_debug.exp_mode = services.cfg.plain_traj_en
                                               ? "plain"
                                               : (services.cfg.esdf_traj_en ? "esdf" : "corridor");

        StatePVAJ pos_init_state, pos_fina_state;
        PolytopeVec sfc;
        vec_Vec3f guide_path;
        std::vector<double> guide_stamp;
        double guide_path_end_vel{0.0};
        int reserve_size = services.cfg.planning_horizon / services.cfg.resolution * 1.2;
        guide_path.reserve(reserve_size);
        guide_stamp.reserve(reserve_size);

        Vec4f init_yaw{services.robot_state.yaw, 0, 0, 0};
        Vec4f fina_yaw{0, 0, 0, 0};

        Trajectory guide_pos_traj, guide_yaw_traj, last_exp_traj;

        const double replan_process_start_WT = services.ros_ptr->getSimTime();
        double replan_process_start_TT, replan_state_TT;
        const bool planning_from_rest = last_exp_traj_info.empty();
        const bool use_plain_exp_traj = services.cfg.plain_traj_en;
        const bool use_esdf_exp_traj = services.cfg.esdf_traj_en && !use_plain_exp_traj;
        const bool use_distance_field_exp_traj = use_plain_exp_traj || use_esdf_exp_traj;
        const bool reuse_command_guide_prefix =
                !planning_from_rest && services.cfg.state2state_replan_use_command_guide;

        if (planning_from_rest) {
            pos_init_state.setZero();
            pos_init_state.col(0) = services.local_start_p;
            replan_process_start_TT = -1;
            replan_state_TT = -1;
        } else {
            guide_pos_traj = services.cmd_traj_info.posTraj();
            guide_yaw_traj = services.cmd_traj_info.yawTraj();
            last_exp_traj = last_exp_traj_info.posTraj();

            replan_process_start_TT = replan_process_start_WT - last_exp_traj.start_WT;
            replan_state_TT = replan_process_start_TT + services.cfg.replan_forward_dt;
            std::vector<TimePosPair> last_exp_traj_time_pos;
            std::vector<double> last_exp_traj_vel;

            const double cmd_total_duration = services.cmd_traj_info.getTotalDuration();
            if (replan_state_TT >= cmd_total_duration) {
                out_exp_traj_info = last_exp_traj_info;

                if (services.robot_on_backup_traj) {
                    if (services.cfg.print_log) {
                        services.ros_ptr->warn(
                                " -- [GeneralPlanner] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    }
                    return FAILED;
                }

                if (services.cfg.print_log) {
                    services.ros_ptr->warn(
                            " -- [generateExpTraj] replan_state_TT >= services.cmd_traj_info.pos_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                }

                return NO_NEED;
	            }

            if (!last_exp_traj_info.empty()) {
                const double last_exp_total_duration = last_exp_traj.getTotalDuration();
                if (replan_state_TT >= last_exp_total_duration) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (services.robot_on_backup_traj) {
                        if (services.cfg.print_log) {
                            services.ros_ptr->warn(
                                    " -- [GeneralPlanner] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        }
                        return FAILED;
                    }
                    if (services.cfg.print_log) {
                        services.ros_ptr->warn(
                                " -- [generateExpTraj] replan_state_TT >= last_exp_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                    }

                    return NO_NEED;
                }

                if (!services.new_goal &&
                    last_exp_traj_info.getSFCSize() == 1 &&
                    last_exp_traj_info.connectedToGoal() &&
                    currentTrajectorySafeForNoNeed(services, guide_pos_traj, replan_state_TT)) {
                    if (services.cfg.print_log) {
                        services.ros_ptr->warn(
                                " -- [GeneralPlanner] Replan, last exp have only one corridor and connected to goal return NONEED.");
                    }

                    out_exp_traj_info = last_exp_traj_info;
                    if (services.robot_on_backup_traj) {
                        if (services.cfg.print_log) {
                            services.ros_ptr->warn(
                                    " -- [GeneralPlanner] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        }
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                if (!services.new_goal &&
                    (services.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < services.cfg.resolution * 3 &&
                    currentTrajectorySafeForNoNeed(services, guide_pos_traj, replan_state_TT)) {
                    out_exp_traj_info = last_exp_traj_info;
                    out_exp_traj_info.setGoalConnectedFlag(true);

                    services.ros_ptr->warn(" -- [GeneralPlanner] Replan, close to goal and return NONEED.");
                    if (services.robot_on_backup_traj) {
                        services.ros_ptr->warn(
                                " -- [GeneralPlanner] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }
            }
            out_exp_traj_info.setGoalConnectedFlag(false);

            double eval_t = replan_state_TT;
            double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();

            Vec3f temp_pt, last_sample_pt;
            last_exp_traj_time_pos.clear();
            last_exp_traj_info.setWholeTrajKnownFreeFlag(true);
            last_sample_pt = guide_pos_traj.getPos(eval_t);
            eval_t += services.cfg.sample_traj_dt;
            int replan_id = -1;
            for (; eval_t < guide_pos_traj_total_time; eval_t += services.cfg.sample_traj_dt) {
                temp_pt = guide_pos_traj.getPos(eval_t);
                if ((temp_pt - last_sample_pt).norm() < services.cfg.resolution * 0.8) {
                    continue;
                }

                rog_map::GridType temp_grid = services.map_manager->getInfGridType(temp_pt);

                if (temp_grid == rog_map::GridType::OCCUPIED || temp_grid == rog_map::GridType::OUT_OF_MAP) {
                    last_exp_traj_info.setWholeTrajKnownFreeFlag(false);
                    break;
                }
                if (eval_t > replan_state_TT && replan_id == -1) {
                    replan_id = last_exp_traj_time_pos.size();
                }
                last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
                last_exp_traj_vel.emplace_back(guide_pos_traj.getVel(eval_t).norm());
                last_sample_pt = temp_pt;
            }

            if (!services.new_goal &&
                use_distance_field_exp_traj &&
                last_exp_traj_info.connectedToGoal() &&
                last_exp_traj_info.wholeTrajKnownFree() &&
                currentTrajectorySafeForNoNeed(services, guide_pos_traj, replan_state_TT)) {
                out_exp_traj_info = last_exp_traj_info;
                if (services.robot_on_backup_traj) {
                    if (services.cfg.print_log) {
                        services.ros_ptr->warn(
                                " -- [GeneralPlanner] Distance-field replan, emergency stop, return FAILED and wait for plan form rest.");
                    }
                    return FAILED;
                }
                return NO_NEED;
            }

            if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {
                services.ros_ptr->warn(" -- [GeneralPlanner] Invalid traj or eval t");
                return FAILED;
            }
            guide_stamp.clear();
            guide_path.clear();
            if (!reuse_command_guide_prefix) {
                // The previous command is a candidate, not a trusted height
                // reference. Start a fresh map-valid guide from the exact
                // future replan state so a z overshoot cannot feed itself into
                // the following optimization cycle.
                guide_path.push_back(pos_init_state.col(0));
                guide_stamp.push_back(0.0);
                guide_path_end_vel = pos_init_state.col(1).norm();
            } else {
                double split_dis = services.cfg.receding_dis;
                if (last_exp_traj_info.wholeTrajKnownFree() && !services.new_goal && services.cfg.receding_dis > 0.0) {
                    split_dis = std::numeric_limits<double>::max();
                }
                if (split_dis <= 0 || last_exp_traj_time_pos.empty()) {
                    guide_path.push_back(pos_init_state.col(0));
                    guide_stamp.push_back(0.0);
                    last_exp_traj_time_pos.clear();
                    last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                    guide_path_end_vel = services.robot_state.v.norm();
                } else {
                    temp_pt = last_exp_traj_time_pos.back().second;
                    while (services.map_manager->isOccupiedInflate(temp_pt) ||
                           (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
                        last_exp_traj_time_pos.pop_back();
                        last_exp_traj_vel.pop_back();
                        if (last_exp_traj_time_pos.empty()) {
                            services.ros_ptr->warn(" -- [GeneralPlanner] WARN, all traj is collide in INF2");
                            break;
                        }
                        temp_pt = last_exp_traj_time_pos.back().second;
                    }
                    if (!last_exp_traj_time_pos.empty()) {
                        for (long unsigned int i = 0; i < last_exp_traj_time_pos.size(); i++) {
                            guide_path.push_back(last_exp_traj_time_pos[i].second);
                            guide_stamp.push_back(last_exp_traj_time_pos[i].first - last_exp_traj_time_pos.front().first);
                            guide_path_end_vel = last_exp_traj_vel[i];
                        }
                    } else {
                        guide_path.push_back(pos_init_state.col(0));
                        guide_stamp.push_back(0.0);
                        last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                        guide_path_end_vel = services.robot_state.v.norm();
                    }
                }
            }
        }

        double guide_path_length = geometry_utils::computePathLength(guide_path);
        double temp_horizon = services.cfg.planning_horizon - guide_path_length;

        std::vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        std::vector<int> sfc_waypoint_ids;

        if (guide_path.empty() ||
            ((guide_path.front() - pos_init_state.col(0)).norm() > 1e-2)) {
            guide_path.insert(guide_path.begin(), pos_init_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }

        if (temp_horizon > services.cfg.resolution * 2) {
            if ((guide_path.back() - services.goal_p).norm() < services.cfg.resolution * 5) {
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - services.goal_p).norm() / services.cfg.exp_traj_cfg.max_vel);
                guide_path.push_back(services.goal_p);
            } else {
                vec_Vec3f new_path;
                if (!pathSearch(services.frontend, guide_path.back(),
                                services.goal_p, temp_horizon, new_path)) {
                    services.ros_ptr->warn(" -- [GeneralPlanner] PathSearch for new path failed");
                    return FAILED;
                } else if (new_path.size() < 2) {
                    services.ros_ptr->warn(" -- [GeneralPlanner] PathSearch for new path failed");
                    return FAILED;
                } else {
                    double total_dis{0.0};
                    std::vector<double> dis(new_path.size());
                    Vec3f last_p = new_path.back();
                    for (int i = new_path.size() - 2; i >= 0; i--) {
                        auto d = (new_path[i] - last_p).norm();
                        total_dis += d;
                        dis[i + 1] = total_dis;
                        last_p = new_path[i];
                    }
                    total_dis += (new_path.front() - guide_path.back()).norm();
                    dis[0] = total_dis;
                    std::vector<double> stamps(new_path.size(), 0);
                    std::vector<double> dt(new_path.size(), 0);
                    double last_stamp = 0;
                    for (int i = dis.size() - 1; i >= 0; i--) {
                        double vel;
                        if (!geometry_utils::simplePMTimeAllocator(services.cfg.exp_traj_cfg.max_acc,
                                                                   services.cfg.exp_traj_cfg.max_vel,
                                                                   guide_path_end_vel,
                                                                   total_dis,
                                                                   dis[i],
                                                                   stamps[i],
                                                                   vel)) {
                            services.ros_ptr->warn(" -- [GeneralPlanner] Guide time allocation failed: total_dis={:.6f}, cur_dis={:.6f}, end_vel={:.6f}, max_vel={:.6f}, max_acc={:.6f}.",
                                           total_dis,
                                           dis[i],
                                           guide_path_end_vel,
                                           services.cfg.exp_traj_cfg.max_vel,
                                           services.cfg.exp_traj_cfg.max_acc);
                            return FAILED;
                        }
                        const double stamp_dt = stamps[i] - last_stamp;
                        if (!std::isfinite(stamp_dt) || stamp_dt < -1.0e-8) {
                            services.ros_ptr->warn(" -- [GeneralPlanner] Guide time allocation produced invalid dt: index={}, stamp={:.6f}, last_stamp={:.6f}, dt={:.6f}.",
                                           i,
                                           stamps[i],
                                           last_stamp,
                                           stamp_dt);
                            return FAILED;
                        }
                        dt[i] = std::max(0.0, stamp_dt);
                        last_stamp = stamps[i];
                    }
                    double time_stamp = guide_stamp.back();

                    for (long unsigned int i = 1; i < new_path.size(); i++) {
                        double t = dt[i];
                        time_stamp += t;
                        guide_path.emplace_back(new_path[i]);
                        guide_stamp.emplace_back(time_stamp);
                    }
                }
            }
        }

        if (!prepareESDFGuideEndpoint(services.frontend, guide_path, guide_stamp)) {
            services.ros_ptr->warn(" -- [GeneralPlanner] Failed to prepare ESDF rolling local endpoint.");
            return FAILED;
        }

        const bool connected_goal = (guide_path.back().head(2) - services.goal_p.head(2)).norm() < services.cfg.resolution * 2;
        out_exp_traj_info.setGoalConnectedFlag(connected_goal);

        if (rejectOnCheckFailure(services.ros_ptr,
                                 "generateExpTraj guide",
                                 checker::checkGuidePath(guide_path,
                                                         guide_stamp,
                                                         services.cfg.resolution,
                                                         "state2state_exp"))) {
            return FAILED;
        }
        services.latest_replan.setGuidePath(guide_path);

        sfc.clear();
        {
            TimeConsuming t_viz("tviz", false);
            services.ros_ptr->vizFrontendPath(guide_path);
            services.time_consuming[VISUALIZATION] += t_viz.stop();
        }

        if (use_esdf_exp_traj && !services.map_manager->hasESDF()) {
            services.ros_ptr->warn(" -- [GeneralPlanner] ESDF exp traj is enabled, but ROGMap ESDF is unavailable.");
            return FAILED;
        }

        services.shifted_sfc_start_pt = Vec3f(9999, 9999, 9999);
        if (!use_esdf_exp_traj && !use_plain_exp_traj) {
            bool bool_ret_code = services.corridor_generator->SearchPolytopeOnPath(guide_path, sfc, services.shifted_sfc_start_pt, services.cfg.use_fov_cut);

            if (!bool_ret_code) {
                services.ros_ptr->warn(" -- [GeneralPlanner] SearchPolytopeOnPath for new path failed");
                return FAILED;
            }
            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizExpSfc(sfc);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }
        }

        services.time_consuming[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();

        pos_fina_state.setZero();
        pos_fina_state.col(0) = guide_path.back();
        const bool local_endpoint_is_global_goal =
                (pos_fina_state.col(0) - services.goal_p).norm() < services.cfg.resolution * 2;
        if (services.cfg.goal_vel_en && (services.goal_p - services.robot_state.p).norm() > services.cfg.planning_horizon / 2) {
            pos_fina_state.col(1) = (services.goal_p - services.robot_state.p).normalized() * services.cfg.exp_traj_cfg.max_vel / 2;
        }
        if (local_endpoint_is_global_goal) {
            pos_fina_state.col(0) = services.goal_p;
            pos_fina_state.col(1).setZero();
        }
        auto copyZSummary = [](const LocalZSummary &src, State2StateZSummary &dst) {
            dst.valid = src.valid;
            dst.start = src.start;
            dst.end = src.end;
            dst.min = src.min;
            dst.max = src.max;
        };
        services.z_debug.valid = true;
        services.z_debug.guide_size = static_cast<int>(guide_path.size());
        copyZSummary(summarizePathZ(guide_path), services.z_debug.guide);
        services.z_debug.goal_z = services.goal_p.z();
        services.z_debug.robot_z = pos_init_state(2, 0);
        services.z_debug.local_target_z = pos_fina_state(2, 0);
        services.z_debug.local_target_goal_dist = (pos_fina_state.col(0) - services.goal_p).norm();
        services.z_debug.local_target_goal_xy_dist =
                (pos_fina_state.col(0).head<2>() - services.goal_p.head<2>()).norm();
        services.z_debug.local_target_goal_z_err = pos_fina_state(2, 0) - services.goal_p.z();
        services.z_debug.local_target_is_global_goal = local_endpoint_is_global_goal;
        services.z_debug.guide_reused_command_prefix = reuse_command_guide_prefix;
        services.z_debug.z_lower_guard_enabled =
                services.cfg.state2state_z_lower_guard_enable &&
                !use_distance_field_exp_traj;
        services.z_debug.z_lower_guard_tolerance =
                services.cfg.state2state_z_lower_guard_tolerance;

        bool temp_ret;
        Trajectory out_traj;
        TimeConsuming t_exp_opt("t_exp_opt", false);
        services.traj_manager->setSwarmCurrentWallTime(replan_process_start_WT);
        if (use_esdf_exp_traj) {
            temp_ret = services.traj_manager->esdf()->optimize(pos_init_state,
                                                       pos_fina_state,
                                                       guide_path,
                                                       guide_stamp,
                                                       out_traj);
            if (!temp_ret) {
                if (!planning_from_rest &&
                    last_exp_traj_info.wholeTrajKnownFree() &&
                    currentTrajectorySafeForNoNeed(services, guide_pos_traj, replan_state_TT)) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (services.cfg.print_log) {
                        services.ros_ptr->warn(" -- [GeneralPlanner] ESDF candidate optimization failed, keep current safe trajectory.");
                    }
                    return NO_NEED;
                }
                services.ros_ptr->warn(" -- [GeneralPlanner] ESDF optimization failed.");
                return FAILED;
            }
        } else if (use_plain_exp_traj) {
            temp_ret = services.traj_manager->plain()->optimize(pos_init_state,
                                                        pos_fina_state,
                                                        guide_path,
                                                        guide_stamp,
                                                        out_traj);
            if (!temp_ret) {
                if (!planning_from_rest &&
                    last_exp_traj_info.wholeTrajKnownFree() &&
                    currentTrajectorySafeForNoNeed(services, guide_pos_traj, replan_state_TT)) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (services.cfg.print_log) {
                        services.ros_ptr->warn(" -- [GeneralPlanner] Plain candidate optimization failed, keep current safe trajectory.");
                    }
                    return NO_NEED;
                }
                services.ros_ptr->warn(" -- [GeneralPlanner] Plain optimization failed.");
                return FAILED;
            }
        } else {
            temp_ret = services.traj_manager->exp()->optimize(pos_init_state,
                                                      pos_fina_state,
                                                      guide_path,
                                                      guide_stamp,
                                                      sfc,
                                                      out_traj);
        }
        services.time_consuming[EXP_TRAJ_OPT] = t_exp_opt.stop();
        copyZSummary(summarizeTrajectoryZ(out_traj, services.cfg.sample_traj_dt),
                     services.z_debug.optimized);
        if (services.z_debug.optimized.valid) {
            services.z_debug.opt_end_local_target_z_err =
                    services.z_debug.optimized.end -
                    services.z_debug.local_target_z;
        }
        if (use_esdf_exp_traj) {
            VecDf init_ts;
            vec_Vec3f init_ps;
            services.traj_manager->esdf()->getInitValue(init_ts, init_ps);
            services.latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        } else if (use_plain_exp_traj) {
            VecDf init_ts;
            vec_Vec3f init_ps;
            services.traj_manager->plain()->getInitValue(init_ts, init_ps);
            services.latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        } else {
            VecDf init_ts;
            vec_Vec3f init_ps;
            services.traj_manager->exp()->getInitValue(init_ts, init_ps);
            services.latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        }
        if (!temp_ret) {
            services.ros_ptr->warn(" -- [GeneralPlanner] OptimizationExpTraj for new path failed");
            return FAILED;
        }
        const GuideZLowerGuardReport z_guard = checkGuideZLowerBound(
                out_traj,
                guide_path,
                guide_stamp,
                services.z_debug.z_lower_guard_enabled,
                services.cfg.state2state_z_lower_guard_tolerance,
                services.cfg.state2state_z_lower_guard_sample_dt);
        services.z_debug.z_lower_guard_passed = z_guard.passed();
        services.z_debug.z_lower_guard_samples = z_guard.samples;
        services.z_debug.z_lower_guard_min_margin = z_guard.min_margin;
        services.z_debug.z_lower_guard_max_violation = z_guard.max_violation;
        if (!z_guard.passed()) {
            services.ros_ptr->warn(
                    " -- [GeneralPlanner] Reject z-lower candidate: valid={}, samples={}, min_margin={:.3f}, max_violation={:.3f}, tolerance={:.3f}.",
                    z_guard.valid,
                    z_guard.samples,
                    z_guard.min_margin,
                    z_guard.max_violation,
                    services.cfg.state2state_z_lower_guard_tolerance);
            if (!planning_from_rest && last_exp_traj_info.wholeTrajKnownFree() &&
                currentTrajectorySafeForNoNeed(services, guide_pos_traj, replan_state_TT)) {
                out_exp_traj_info = last_exp_traj_info;
                return NO_NEED;
            }
            return FAILED;
        }
        double replan_total_t = (services.ros_ptr->getSimTime() - replan_process_start_WT);
        if (replan_total_t > services.cfg.replan_forward_dt) {
            if (!planning_from_rest) {
                services.ros_ptr->warn(" -- [GeneralPlanner] Replan over time({})!!!! Return FAILED", replan_total_t);
                return FAILED;
            }
            services.ros_ptr->warn(" -- [GeneralPlanner] PlanFromRest over realtime budget({}), accept initial trajectory.", replan_total_t);
        }

        {
            TimeConsuming t_viz("tviz", false);
            services.ros_ptr->vizExpTraj(out_traj,
                                 use_plain_exp_traj ? "plain_traj" : (use_esdf_exp_traj ? "esdf_traj" : "exp_traj"));
            services.time_consuming[VISUALIZATION] += t_viz.stop();
        }

        double new_traj_WT = replan_process_start_WT;

        replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;
        Trajectory temp_exp_traj;
        if (!services.last_exp_traj_info.empty() &&
            !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       temp_exp_traj)) {
            services.ros_ptr->error(" -- [GeneralPlanner] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.setSFC(sfc);
        temp_exp_traj = temp_exp_traj + out_traj;
        temp_exp_traj.start_WT = new_traj_WT;
        copyZSummary(summarizeTrajectoryZ(temp_exp_traj, services.cfg.sample_traj_dt),
                     services.z_debug.exp_full);

        if (!last_exp_traj_info.empty()) {
            StatePVAJ yaw_replan_state;
            if (!guide_yaw_traj.getState(replan_state_TT, yaw_replan_state)) {
                services.ros_ptr->warn(" -- [GeneralPlanner] Invalid traj or eval t");
                return FAILED;
            }
            init_yaw = yaw_replan_state.row(0);
        }

        bool free_end{true};
        if (services.cfg.goal_yaw_en && !isnan(services.goal_yaw) && connected_goal) {
            free_end = false;
            fina_yaw[0] = services.goal_yaw;
        }
        Trajectory new_traj, old_traj;

        if (!services.traj_manager->yaw()->optimize(init_yaw, fina_yaw, out_traj, new_traj, 3, false, free_end)) {
            services.ros_ptr->error(" -- [GeneralPlanner] in [generateExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        if (!last_exp_traj_info.empty()) {
            if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                           old_traj)) {
                services.ros_ptr->error(" -- [GeneralPlanner] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
        }

        Trajectory temp_yaw_traj = old_traj + new_traj;
        double on_backup_end_TT{-1}, on_backup_start_TT{-1};
        if (!last_exp_traj_info.empty() && replan_state_TT > services.cmd_traj_info.getBackupTrajStartTT()) {
            on_backup_start_TT = services.cmd_traj_info.getBackupTrajStartTT() - replan_process_start_TT;
            on_backup_end_TT = replan_state_TT - replan_process_start_TT;
        }
        out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj, on_backup_start_TT,
                                        on_backup_end_TT);

        const checker::CheckResult output_check =
                checker::checkExpTrajectory(out_exp_traj_info,
                                            services.cfg,
                                            "state2state_exp_output");
        if (output_check.rejected()) {
            logCheckResult(services.ros_ptr,
                           "generateExpTraj output",
                           output_check);
            out_exp_traj_info.setEmpty();
            return FAILED;
        }

        if (out_exp_traj_info.empty()) {
            out_exp_traj_info.setEmpty();
            return FAILED;
        }

        services.latest_replan.setExpYawTraj(temp_yaw_traj);
        services.latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

} // namespace state2state_task
} // namespace general_planner
