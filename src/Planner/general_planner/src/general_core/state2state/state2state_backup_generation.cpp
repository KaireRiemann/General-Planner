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

#include <data_structure/backup_traj.h>
#include <data_structure/cmd_traj.h>
#include <data_structure/exp_traj.h>
#include <checker/trajectory_checker.hpp>
#include <general_core/config.hpp>
#include <general_core/corridor_generator.h>
#include <general_core/fov_checker.h>
#include <general_core/general_ret_code.hpp>
#include <map_manager/map_manager.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <traj_opt/traj_manager.h>
#include <general_core/log_utils.hpp>
#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <general_utils/scope_timer.hpp>
#include <utils/optimization/polynomial_interpolation.h>
#include <vector>

using namespace general_utils;
using namespace geometry_utils;

namespace general_planner {
    namespace {
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

        bool buildYawBrakeTrajectory(const Vec4f &yaw_state,
                                     const double duration,
                                     const double start_wt,
                                     Trajectory &yaw_traj) {
            if (!yaw_state.allFinite() ||
                !std::isfinite(duration) ||
                duration <= 1.0e-5 ||
                !std::isfinite(start_wt)) {
                return false;
            }

            Eigen::Matrix<double, 1, 2> init_state;
            Eigen::Matrix<double, 1, 2> goal_state;
            const double yaw0 = yaw_state(0);
            const double yaw_rate0 = yaw_state(1);
            init_state << yaw0, yaw_rate0;
            goal_state << yaw0 + 0.5 * yaw_rate0 * duration, 0.0;

            Eigen::Matrix<double, 1, -1> waypoints(1, 0);
            VecDf times(1);
            times(0) = duration;
            yaw_traj = poly_interpo::minimumAccInterpolation<1>(init_state,
                                                                goal_state,
                                                                waypoints,
                                                                times);
            yaw_traj.start_WT = start_wt;
            return !yaw_traj.empty();
        }
    }

    namespace state2state_task {

    RET_CODE generateBackupTrajectory(StateToStateBackupBackendServices &services,
                                      ExpTraj &ref_exp_traj,
                                      BackupTraj &back_traj_info) {
        services.drone_state_mutex.lock();
        back_traj_info.setRobotPos(services.robot_state.p);
        services.drone_state_mutex.unlock();
        TimeConsuming t_back_frontend("t_back_frontend", false);

        if (rejectOnCheckFailure(services.ros_ptr,
                                 "generateBackupTrajectory input exp",
                                 checker::checkExpTrajectory(ref_exp_traj,
                                                             services.cfg,
                                                             "backup_ref_exp"))) {
            back_traj_info.setEmpty();
            return FAILED;
        }

        if (!services.cfg.backup_traj_en || services.cfg.esdf_traj_en || services.cfg.plain_traj_en) {
            back_traj_info.setEmpty();
            services.time_consuming[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
            return FINISH;
        }

        double total_dur = ref_exp_traj.getTotalDuration();
        double start_t = services.ros_ptr->getSimTime() - ref_exp_traj.getStartWallTime();

        if (start_t > total_dur - 0.01) {
            if (services.cfg.print_log) {
                services.ros_ptr->info(" -- [GeneralPlanner] in [generateBackupTrajectory]: start_t > total_dur, return NO_NEED");
            }
            return NO_NEED;
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        std::vector<double> min_stop_dis;
        std::vector<TimePosPair> eval_ps;
        Vec3f temp_vel;

        Vec3f last_pos = ref_exp_traj.getPos(start_t);
        for (out_t = start_t; out_t < total_dur; out_t += services.cfg.sample_traj_dt) {
            temp_point = ref_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < services.cfg.resolution * 0.8) {
                continue;
            }
            last_pos = temp_point;
            temp_vel = ref_exp_traj.getVel(out_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / services.cfg.exp_traj_cfg.max_acc);
            eval_ps.push_back(std::pair<double, Vec3f>(out_t, temp_point));
            const double min_dis =
                    services.cfg.sensing_horizon > 0 ? std::min(services.cfg.sensing_horizon, services.cfg.safe_corridor_line_max_length)
                                             : services.cfg.safe_corridor_line_max_length;
            if (!services.map_manager->isLineFree(back_traj_info.getRobotPos(),
                                          temp_point,
                                          min_dis,
                                          services.cfg.seed_line_neighbour)) {
                all_traj_visible = false;
                break;
            }
        }

        if (eval_ps.empty()) {
            services.ros_ptr->warn(" -- [Checker] generateBackupTrajectory has no sampled exp point, return NO_NEED.");
            back_traj_info.setEmpty();
            return NO_NEED;
        }

        if (all_traj_visible) {
            back_traj_info.setEmpty();
            {
                double dur = ref_exp_traj.getTotalDuration();
                Vec3f seed_pt = ref_exp_traj.getPos(dur);
                Line line{back_traj_info.getRobotPos(), seed_pt};
                Polytope temp_poly;
                if (services.corridor_generator->GeneratePolytopeFromLine(line, temp_poly)) {
                    back_traj_info.setSFC(temp_poly);
                    {
                        TimeConsuming t_viz("tviz", false);
                        services.ros_ptr->vizBackupSfc(temp_poly);
                        services.time_consuming[VISUALIZATION] += t_viz.stop();
                    }
                }
            }
            return FINISH;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= services.cfg.sample_traj_dt;
            Vec3f out_p = ref_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > services.cfg.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);
        Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);

        Vec3f shifted_robot_p = services.shifted_sfc_start_pt.norm() > 999 ? services.robot_state.p : services.shifted_sfc_start_pt;
        if (!services.map_manager->getNearestCellNot(GridType::OCCUPIED, shifted_robot_p, shifted_robot_p, 3.0)) {
            services.ros_ptr->error(
                    " -- [GeneralPlanner] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            services.latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_START_POINT);
            return FAILED;
        }

        Line line{shifted_robot_p, seed_point};
        Polytope temp_poly;
        if (!services.corridor_generator->GeneratePolytopeFromLine(line, temp_poly)) {
            services.ros_ptr->warn(" -- [GeneralPlanner] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
            services.ros_ptr->warn(" -- [GeneralPlanner] Cannot generate feasible backup sfc, force return");
            return FAILED;
        }

        if (services.cfg.use_fov_cut) {
            if (!services.fov_checker->cutPolyByFov(services.robot_state.p, services.robot_state.q, seed_point,
                                            temp_poly)) {
                services.ros_ptr->warn(" -- [GeneralPlanner] cutPolyByFov failed, force return");
                return FAILED;
            }
        }
        if (services.cfg.sensing_horizon > 0 &&
            !services.fov_checker->cutPolyBySensingHorizon(services.robot_state.p, seed_point, services.cfg.sensing_horizon,
                                                   temp_poly)) {
            services.ros_ptr->warn(" -- [GeneralPlanner] cutPolyBySensingHorizon failed, force return");
            return FAILED;
        }

        back_traj_info.setSFC(temp_poly);

        {
            TimeConsuming t_viz("tviz", false);
            services.ros_ptr->vizBackupSfc(temp_poly);
            services.time_consuming[VISUALIZATION] += t_viz.stop();
        }

        double eval_t = eval_ps.back().first + services.cfg.sample_traj_dt;
        last_pos = eval_ps.back().second;
        while (temp_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = ref_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < services.cfg.resolution * 0.8) {
                eval_t += services.cfg.sample_traj_dt;
                continue;
            }
            temp_vel = ref_exp_traj.getVel(eval_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / services.cfg.exp_traj_cfg.max_acc);
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += services.cfg.sample_traj_dt;
        }
        if (!eval_ps.empty()) {
            eval_ps.pop_back();
        }
        if (eval_ps.empty()) {
            services.ros_ptr->warn(" -- [Checker] generateBackupTrajectory backup seed samples are empty after trimming.");
            back_traj_info.setEmpty();
            return NO_NEED;
        }
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        double t0 = services.ros_ptr->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        if (!std::isfinite(t0) || !std::isfinite(te) || t0 < -1.0e-6 || te <= t0 + 1.0e-6 ||
            te > ref_exp_traj.getTotalDuration() + 1.0e-6) {
            services.ros_ptr->warn(" -- [Checker] generateBackupTrajectory invalid backup time window: t0={}, te={}, exp_dur={}.",
                           t0, te, ref_exp_traj.getTotalDuration());
            back_traj_info.setEmpty();
            return NO_NEED;
        }
        double vel_e_n = ref_exp_traj.getVel(te).norm();
        double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / services.cfg.back_traj_cfg.max_acc);
        double heu_dur = te - heu_ts;
        Vec3f heu_p = seed_point;
        services.time_consuming[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        Trajectory temp_pos_traj;
        bool temp_ret = services.traj_manager->backup()->optimize(ref_exp_traj.posTraj(),
                                                          t0,
                                                          te,
                                                          heu_ts,
                                                          heu_p,
                                                          heu_dur,
                                                          back_traj_info.getSFC(),
                                                          temp_pos_traj,
                                                          opt_ts);
        services.time_consuming[BACK_TRAJ_OPT] = t_back_opt.stop();

        double init_ts;
        VecDf init_times;
        vec_Vec3f init_ps;
        services.traj_manager->backup()->getInitValue(init_ts, init_times, init_ps);
        services.latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                         t0, te,
                                         back_traj_info.getSFC());

        if (!temp_ret) {
            services.ros_ptr->warn(" -- [GeneralPlanner] OptimizationBakTrajInPolytopes failed, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        } else {
            if (!std::isfinite(opt_ts) || opt_ts < t0 - 1.0e-6 || opt_ts > te + 1.0e-6) {
                services.ros_ptr->error(" -- [Checker] generateBackupTrajectory invalid opt_ts={}, t0={}, te={}.",
                                opt_ts, t0, te);
                return OPT_FAILED;
            }
            Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
            Vec4f yaw_goal{0, 0, 0, 0};
            bool free_end{true};
            if (services.cfg.goal_yaw_en) {
                if (!isnan(services.goal_yaw)) {
                    free_end = false;
                    yaw_goal[0] = services.goal_yaw;
                }
            }
            Trajectory temp_yaw_traj;
            if (!services.traj_manager->yaw()->optimize(yaw_init_vec, yaw_goal, temp_pos_traj,
                                                temp_yaw_traj, 3, false, free_end)) {
                services.ros_ptr->error(" -- [GeneralPlanner] in [generateBackupTrajectory] YawTrajOpt FAILD.");
                return OPT_FAILED;
            }

            if (opt_ts < t0) {
                services.ros_ptr->error(" -- [GeneralPlanner] opt_ts {} < t0 {}", opt_ts, t0);
                return OPT_FAILED;
            }
            double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
            const auto &committed_ts_WT = services.cmd_traj_info.getBackupTrajStartTT();
            if (committed_ts_WT < services.cmd_traj_info.getTotalDuration() && new_ts_WT < committed_ts_WT) {
                services.ros_ptr->error(" -- [GeneralPlanner] new_ts_WT {} < committed_ts_WT {}", new_ts_WT, committed_ts_WT);
                return OPT_FAILED;
            }

            {
                TimeConsuming t_viz("tviz", false);
                services.ros_ptr->vizBackupTraj(temp_pos_traj);
                services.time_consuming[VISUALIZATION] += t_viz.stop();
            }

            back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
            const auto backup_check = checker::checkBackupTrajectory(back_traj_info,
                                                                     services.cfg,
                                                                     "state2state_backup_output");
            if (backup_check.rejected()) {
                const bool yaw_rate_limited =
                        backup_check.code.find("YAW_RATE_LIMIT") != std::string::npos;
                if (yaw_rate_limited) {
                    Trajectory yaw_brake_traj;
                    if (buildYawBrakeTrajectory(yaw_init_vec,
                                                temp_pos_traj.getTotalDuration(),
                                                new_ts_WT,
                                                yaw_brake_traj)) {
                        back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, yaw_brake_traj);
                        const auto yaw_brake_check = checker::checkBackupTrajectory(
                                back_traj_info,
                                services.cfg,
                                "state2state_backup_yaw_brake_output");
                        if (!yaw_brake_check.rejected()) {
                            services.ros_ptr->warn(" -- [GeneralPlanner] Normal backup yaw rejected [{}], use yaw-brake fallback.",
                                           backup_check.code);
                            services.latest_replan.setBackupTraj(temp_pos_traj);
                            services.latest_replan.setBackupYawTraj(yaw_brake_traj);
                            return SUCCESS;
                        }
                        logCheckResult(services.ros_ptr,
                                       "generateBackupTrajectory yaw-brake output",
                                       yaw_brake_check);
                    } else {
                        services.ros_ptr->warn(" -- [GeneralPlanner] Failed to build yaw-brake fallback for backup trajectory.");
                    }
                }
                logCheckResult(services.ros_ptr, "generateBackupTrajectory output", backup_check);
                back_traj_info.setEmpty();
                return OPT_FAILED;
            }
            services.latest_replan.setBackupTraj(temp_pos_traj);
            services.latest_replan.setBackupYawTraj(temp_yaw_traj);
            return SUCCESS;
        }
        services.ros_ptr->warn(" -- [GeneralPlanner] Cannot find backup traj start point.");
        return FAILED;
    }

} // namespace state2state_task
} // namespace general_planner
