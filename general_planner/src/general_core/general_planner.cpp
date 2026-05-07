/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <general_core/general_planner.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <super_utils/scope_timer.hpp>
#include <utils/optimization/polynomial_interpolation.h>
#include <fmt/color.h>

using namespace super_utils;

namespace general_planner {
    namespace {
        traj_opt::DynamicTargetState interpolateTargetPrediction(const traj_opt::DynamicTargetStates &prediction,
                                                                 const double &t) {
            if (prediction.empty()) {
                return {};
            }
            if (prediction.size() == 1 || t <= prediction.front().t) {
                return prediction.front();
            }
            if (t >= prediction.back().t) {
                return prediction.back();
            }

            const auto it = std::lower_bound(prediction.begin(),
                                             prediction.end(),
                                             t,
                                             [](const traj_opt::DynamicTargetState &state, double query_t) {
                                                 return state.t < query_t;
                                             });
            const int idx = static_cast<int>(std::distance(prediction.begin(), it));
            const auto &left = prediction[static_cast<std::size_t>(idx - 1)];
            const auto &right = prediction[static_cast<std::size_t>(idx)];
            const double alpha = (t - left.t) / std::max(1.0e-9, right.t - left.t);

            traj_opt::DynamicTargetState out;
            out.t = t;
            out.position = left.position + alpha * (right.position - left.position);
            out.velocity = left.velocity + alpha * (right.velocity - left.velocity);
            out.acceleration = left.acceleration + alpha * (right.acceleration - left.acceleration);
            out.yaw = left.yaw + alpha * (right.yaw - left.yaw);
            out.yaw_rate = left.yaw_rate + alpha * (right.yaw_rate - left.yaw_rate);
            return out;
        }

        double yawFacingTarget(const Trajectory &pos_traj,
                               const traj_opt::DynamicTargetStates &target_prediction,
                               const double &t,
                               const double &last_yaw) {
            const double eval_t = std::clamp(t, 0.0, pos_traj.getTotalDuration());
            const Vec3f tracker_p = pos_traj.getPos(eval_t);
            const Vec3f target_p = interpolateTargetPrediction(target_prediction, eval_t).position;
            const Vec3f face_dir = target_p - tracker_p;
            double yaw = last_yaw;
            if (face_dir.head<2>().norm() > 1.0e-4) {
                yaw = std::atan2(face_dir.y(), face_dir.x());
                geometry_utils::normalizeNextYaw(last_yaw, yaw);
            }
            return yaw;
        }
    }

    GeneralPlanner::GeneralPlanner
            (const std::string &cfg_path,
             const ros_interface::RosInterface::Ptr &ros_ptr,
             const rog_map::ROGMapROS::Ptr &map_ptr
            ) : cfg_(Config(cfg_path)),
                map_manager_(std::make_shared<MapManager>(map_ptr)),
                ros_ptr_(ros_ptr) {

        ros_ptr_->setResolution(cfg_.resolution);
        ros_ptr_->setVisualizationEn(cfg_.visualization_en);
        traj_manager_ = std::make_shared<traj_opt::TrajManager>(cfg_.exp_traj_cfg,
                                                                cfg_.esdf_traj_cfg,
                                                                cfg_.plain_traj_cfg,
                                                                cfg_.back_traj_cfg,
                                                                cfg_.yaw_dot_max,
                                                                cfg_.esdf_safe_distance,
                                                                ros_ptr_,
                                                                map_manager_);
        traj_opt::SwarmPenaltyConfig swarm_config;
        swarm_config.enable = cfg_.swarm_enable;
        swarm_config.self_id = cfg_.swarm_drone_id;
        swarm_config.weight = cfg_.swarm_weight;
        swarm_config.clearance = cfg_.swarm_clearance;
        swarm_config.des_clearance = cfg_.swarm_des_clearance;
        swarm_config.horizontal_scale = cfg_.swarm_horizontal_scale;
        swarm_config.vertical_scale = cfg_.swarm_vertical_scale;
        swarm_config.activation_scale = cfg_.swarm_activation_scale;
        swarm_config.time_horizon = cfg_.swarm_time_horizon;
        swarm_config.stale_timeout = cfg_.swarm_stale_timeout;
        swarm_trajs_ = std::make_shared<traj_opt::SwarmTrajectories>();
        traj_manager_->setSwarmConfig(swarm_config);
        traj_manager_->setSwarmTrajectories(swarm_trajs_);

        const auto rog_map_cfg = map_manager_->getMapConfig();
        astar_ptr_ = std::make_shared<path_search::Astar>(cfg_path, ros_ptr_, map_manager_);
        if (traj_manager_->plain()) {
            traj_manager_->plain()->setLocalAstar(astar_ptr_);
        }
        cg_ptr_ = std::make_shared<CorridorGenerator>(ros_ptr_, map_manager_, cfg_.corridor_bound_dis,
                                                      cfg_.corridor_line_max_length,
                                                      cfg_.resolution, rog_map_cfg.virtual_ground_height,
                                                      rog_map_cfg.virtual_ceil_height,
                                                      cfg_.robot_r,
                                                      cfg_.obs_skip_num,
                                                      cfg_.iris_iter_num);
        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);


        time_consuming_.resize(8);

        robot_state_.rcv = false;
        planner_process_start_WT_ = ros_ptr_->getSimTime();
        fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI,
                                                    -1.0,
                                                    -35.0,
                                                    35.0);

        const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);
        astar_ptr_->setFineInfNeighbors(neighbor_step);
    }

    void GeneralPlanner::setSwarmTrajectories(const traj_opt::SwarmTrajectories &trajectories) {
        auto snapshot = std::make_shared<traj_opt::SwarmTrajectories>(trajectories);
        std::lock_guard<std::mutex> lock(swarm_traj_mutex_);
        swarm_trajs_ = snapshot;
        if (traj_manager_) {
            traj_manager_->setSwarmTrajectories(swarm_trajs_);
        }
    }

    bool GeneralPlanner::prepareESDFGuideEndpoint(vec_Vec3f &guide_path,
                                                std::vector<double> &guide_stamp) {
        if (!cfg_.esdf_traj_en || cfg_.plain_traj_en || map_manager_ == nullptr || !map_manager_->hasESDF()) {
            return true;
        }
        if (guide_path.size() < 2 || guide_path.size() != guide_stamp.size()) {
            return false;
        }

        const double hard_required_dist = 0.5 * cfg_.esdf_safe_distance;
        auto is_inflated_free = [&](const Vec3f &pos) {
            const auto inf_type = map_manager_->getInfGridType(pos);
            return inf_type != rog_map::GridType::OCCUPIED &&
                   inf_type != rog_map::GridType::OUT_OF_MAP;
        };
        auto is_esdf_hard_safe = [&](const Vec3f &pos, double &dist) {
            Vec3f grad = Vec3f::Zero();
            if (!map_manager_->evaluateESDF(pos, dist, grad)) {
                dist = -1.0;
                return false;
            }
            return std::isfinite(dist) && dist >= hard_required_dist;
        };
        auto is_guide_point_safe = [&](const Vec3f &pos, double &dist) {
            return is_inflated_free(pos) && is_esdf_hard_safe(pos, dist);
        };
        auto refresh_guide_stamp = [&]() {
            if (guide_path.size() != guide_stamp.size()) {
                guide_stamp.resize(guide_path.size(), 0.0);
            }
            for (size_t i = 1; i < guide_path.size(); ++i) {
                const double dt = (guide_path[i] - guide_path[i - 1]).norm() /
                                  std::max(1.0e-3, cfg_.exp_traj_cfg.max_vel);
                guide_stamp[i] = guide_stamp[i - 1] + std::max(0.05, dt);
            }
        };

        const double repair_radius = std::clamp(2.0 * cfg_.esdf_safe_distance, 0.8, 1.5);
        int repaired_mid_points = 0;
        int unresolved_mid_points = 0;
        for (size_t i = 1; i + 1 < guide_path.size(); ++i) {
            double dist = 0.0;
            if (is_guide_point_safe(guide_path[i], dist)) {
                continue;
            }

            Vec3f shifted_point = guide_path[i];
            if (map_manager_->findNearestESDFSafe(guide_path[i],
                                                  hard_required_dist,
                                                  shifted_point,
                                                  repair_radius)) {
                guide_path[i] = shifted_point;
                ++repaired_mid_points;
            } else {
                ++unresolved_mid_points;
            }
        }

        int repaired_segments = 0;
        int unresolved_segments = 0;
        const int max_segment_repair_iter = 3;
        const size_t max_guide_points = 96;
        for (int iter = 0; iter < max_segment_repair_iter && guide_path.size() < max_guide_points; ++iter) {
            bool inserted = false;
            for (size_t i = 0; i + 1 < guide_path.size() && guide_path.size() < max_guide_points; ++i) {
                if (map_manager_->isLineFree(guide_path[i], guide_path[i + 1], true, false)) {
                    continue;
                }

                const Vec3f mid_point = 0.5 * (guide_path[i] + guide_path[i + 1]);
                Vec3f safe_mid = mid_point;
                if (map_manager_->findNearestESDFSafe(mid_point,
                                                      hard_required_dist,
                                                      safe_mid,
                                                      repair_radius)) {
                    guide_path.insert(guide_path.begin() + static_cast<long>(i + 1), safe_mid);
                    guide_stamp.insert(guide_stamp.begin() + static_cast<long>(i + 1), guide_stamp[i]);
                    ++repaired_segments;
                    inserted = true;
                    ++i;
                } else {
                    ++unresolved_segments;
                }
            }
            if (!inserted) {
                break;
            }
        }
        if (repaired_mid_points > 0 || repaired_segments > 0) {
            refresh_guide_stamp();
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] ESDF guide repaired: mid_points={}, segments={}, unresolved_mid={}, unresolved_segments={}, guide_size={}.",
                               repaired_mid_points,
                               repaired_segments,
                               unresolved_mid_points,
                               unresolved_segments,
                               guide_path.size());
            }
        } else if ((unresolved_mid_points > 0 || unresolved_segments > 0) && cfg_.print_log) {
            ros_ptr_->warn(" -- [GeneralPlanner] ESDF guide has unresolved unsafe samples: mid_points={}, segments={}.",
                           unresolved_mid_points,
                           unresolved_segments);
        }

        double endpoint_dist = 0.0;
        const bool endpoint_inflated_free = is_inflated_free(guide_path.back());
        const bool endpoint_esdf_ready = is_esdf_hard_safe(guide_path.back(), endpoint_dist);
        if (endpoint_inflated_free && endpoint_esdf_ready) {
            return true;
        }

        const bool connected_global_goal =
                (guide_path.back() - gi_.goal_p).norm() < 2.0 * cfg_.resolution;
        if (connected_global_goal && endpoint_inflated_free) {
            return true;
        }

        if (endpoint_inflated_free && endpoint_esdf_ready) {
            return true;
        }

        const Vec3f original_endpoint = guide_path.back();
        Vec3f shifted_endpoint = original_endpoint;
        if (map_manager_->findNearestESDFSafe(original_endpoint, hard_required_dist, shifted_endpoint, 0.8)) {
            const Vec3f prev_pt = guide_path[guide_path.size() - 2];
            if (map_manager_->isLineFree(prev_pt, shifted_endpoint, true, false)) {
                double shifted_dist = 0.0;
                Vec3f shifted_grad = Vec3f::Zero();
                map_manager_->evaluateESDF(shifted_endpoint, shifted_dist, shifted_grad);
                guide_path.back() = shifted_endpoint;
                guide_stamp.back() = guide_stamp[guide_stamp.size() - 2] +
                                     std::max(0.05, (shifted_endpoint - prev_pt).norm() / cfg_.exp_traj_cfg.max_vel);
                ros_ptr_->warn(" -- [GeneralPlanner] ESDF local endpoint hard clearance {} < {}, shift local endpoint from [{}, {}, {}] to [{}, {}, {}], shifted_dist={}.",
                               endpoint_dist,
                               hard_required_dist,
                               original_endpoint.x(), original_endpoint.y(), original_endpoint.z(),
                               shifted_endpoint.x(), shifted_endpoint.y(), shifted_endpoint.z(),
                               shifted_dist);
                return true;
            }
        }

        const int original_size = static_cast<int>(guide_path.size());
        for (int i = static_cast<int>(guide_path.size()) - 2; i >= 1; --i) {
            double dist = 0.0;
            if (!is_inflated_free(guide_path[i]) || !is_esdf_hard_safe(guide_path[i], dist)) {
                continue;
            }
            guide_path.resize(i + 1);
            guide_stamp.resize(i + 1);
            ros_ptr_->warn(" -- [GeneralPlanner] ESDF local endpoint is unavailable or too close: dist={}, required={}. Truncate guide {} -> {}, new endpoint=[{}, {}, {}], new_dist={}.",
                           endpoint_dist,
                           hard_required_dist,
                           original_size,
                           guide_path.size(),
                           guide_path.back().x(), guide_path.back().y(), guide_path.back().z(),
                           dist);
            return (guide_path.back() - guide_path.front()).norm() > cfg_.resolution * 2.0;
        }

        ros_ptr_->warn(" -- [GeneralPlanner] Failed to find a valid ESDF local endpoint for the rolling guide. endpoint_dist={}, required={}.",
                       endpoint_dist,
                       hard_required_dist);
        return false;
    }

    RET_CODE
    GeneralPlanner::PlanFromRest(const Vec3f &goal_p,
                               const double &goal_yaw,
                               const bool &new_goal) {
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (robot_state_.rcv == false) {
            latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
            ros_ptr_->warn(" -- [GeneralPlanner] in [PlanFromRest]: No odom, force return.");
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            return FAILED;
        }
        gi_.goal_valid = true;
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("viz goal path", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) First, shift the start_point to free space.
        Vec3f local_star_pt;
        if (!map_manager_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0)) {
            ros_ptr_->error(
                    " -- [GeneralPlanner] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_START_POINT);
            return FAILED;
        }
        latest_replan.setLocalStartP(local_star_pt);

        /// 2) Generate Exp traj
        ExpTraj exp_traj_info;
        BackupTraj back_traj_info;
        last_exp_traj_info_.setEmpty();
        local_start_p_ = local_star_pt;
        RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
        //GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        if (exp_ret_code == FAILED) {
            ros_ptr_->warn(" -- [GeneralPlanner] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                           RET_CODE_STR[exp_ret_code].c_str());
            return FAILED;
        } else {
            ros_ptr_->info(" -- [GeneralPlanner] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }

        back_traj_info.setEmpty();
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);;

        if (back_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [GeneralPlanner] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;

            // For visualization
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
                latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_SUCCESS_WITH_BACKUP);
            }

            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == NO_NEED) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [GeneralPlanner] in [PlanFromRest] generateBackupTrajectory Finish or NO_NEED.");
            }
            robot_on_backup_traj_ = false;
            cmd_traj_info_.setTrajectory(exp_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;

            // For visualization
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            {
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [GeneralPlanner] in [PlanFromRest] generateBackupTrajectory return [{}], force return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }


    RET_CODE
    GeneralPlanner::ReplanOnce(const Vec3f &goal_p,
                             const double &goal_yaw,
                             const bool &new_goal) {
        TimeConsuming replan_total_t("ReplanOnce", false);
        std::lock_guard<std::mutex> guard(replan_lock_);

        gi_.goal_valid = true;
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);

        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) Replan EXP traj
        ExpTraj exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
        time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            ros_ptr_->warn(" -- [GeneralPlanner] in [ReplanOnce]: GenerateExpTrajectory failed, force return");
            return FAILED;
        } else if (exp_ret_code == NEW_TRAJ) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [GeneralPlanner] in [ReplanOnce]: Last epx traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            ros_ptr_->warn(" -- [GeneralPlanner] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [GeneralPlanner] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED) {
            if (cfg_.print_log)
                ros_ptr_->info(" -- [GeneralPlanner] in [ReplanOnce]: No need to replan a new exp traj, use last one.");
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        BackupTraj back_traj_info;
        // 2）生成back轨迹
        TimeConsuming t_back("t_back", false);
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);
        time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

        {
            ft += time_consuming_[EPX_TRAJ_FRONTEND] + time_consuming_[BACK_TRAJ_FRONTEND];
            ft_cnt++;
            bt += time_consuming_[BACK_TRAJ_OPT] + time_consuming_[EXP_TRAJ_OPT];
            bt_cnt++;
        }

        double replan_dt = replan_total_t.stop();
        if (replan_dt > cfg_.replan_forward_dt * 0.9) {
            ros_ptr_->warn(" -- [GeneralPlanner] in [ReplanOnce]: Replan overtime, check parameters, replan dt = {}.", replan_dt);
            return FAILED;
        }

        if (back_ret_code == SUCCESS) {
            cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;

            {
                // For visualization
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            latest_replan.setRetCode(GENERAL_SUCCESS_WITH_BACKUP);
            if (cfg_.print_log)
                ros_ptr_->info(" -- [GeneralPlanner] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) {
            // 这次生成backup轨迹的点没有意义,
            robot_on_backup_traj_ = false;
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();

            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [GeneralPlanner] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            // Which means the exp traj is all in known free, no need for backup traj
            cmd_traj_info_.setTrajectory(exp_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;

            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [GeneralPlanner] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [GeneralPlanner] in [ReplanOnce]: generateBackupTrajectory return {}, replan Failed return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }

    StatePVAJ GeneralPlanner::makeTaskHeadState(const bool &from_rest) {
        StatePVAJ head = StatePVAJ::Zero();
        head.col(0) = robot_state_.p;
        if (!from_rest) {
            head.col(1) = robot_state_.v;
            head.col(2) = robot_state_.a;
            head.col(3) = robot_state_.j;
        }

        if (!from_rest && !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory pos_traj = cmd_traj_info_.posTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            if (!pos_traj.empty()) {
                const double eval_t = ros_ptr_->getSimTime() - start_wt + cfg_.replan_forward_dt;
                if (eval_t >= 0.0 && eval_t <= total_dur) {
                    return pos_traj.getState(eval_t);
                }
            }
        }
        return head;
    }

    bool GeneralPlanner::commitTaskTrajectory(const Trajectory &pos_traj,
                                            const double &terminal_yaw,
                                            const bool &fix_terminal_yaw,
                                            const std::string &traj_ns) {
        if (pos_traj.empty()) {
            ros_ptr_->warn(" -- [GeneralPlanner] Task trajectory is empty, cannot commit.");
            return false;
        }

        Vec4f init_yaw{robot_state_.yaw, 0.0, 0.0, 0.0};
        if (!cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory yaw_traj = cmd_traj_info_.yawTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            const double eval_t = ros_ptr_->getSimTime() - start_wt;
            StatePVAJ yaw_state;
            if (!yaw_traj.empty() && eval_t >= 0.0 && eval_t <= total_dur &&
                yaw_traj.getState(eval_t, yaw_state)) {
                init_yaw = yaw_state.row(0);
            }
        }

        Vec4f fina_yaw{0.0, 0.0, 0.0, 0.0};
        bool free_end = true;
        if (fix_terminal_yaw && std::isfinite(terminal_yaw)) {
            fina_yaw[0] = terminal_yaw;
            free_end = false;
        }

        Trajectory yaw_traj;
        if (!traj_manager_->yaw()->optimize(init_yaw, fina_yaw, pos_traj, yaw_traj, 3, false, free_end)) {
            ros_ptr_->warn(" -- [GeneralPlanner] Task yaw optimization failed.");
            return false;
        }

        ExpTraj task_exp_traj;
        task_exp_traj.setGoalConnectedFlag(true);
        task_exp_traj.setWholeTrajKnownFreeFlag(true);
        task_exp_traj.setTrajectory(ros_ptr_->getSimTime(), pos_traj, yaw_traj);

        cmd_traj_info_.setTrajectory(task_exp_traj);
        last_exp_traj_info_ = task_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        {
            TimeConsuming t_viz("task_viz", false);
            ros_ptr_->vizExpTraj(pos_traj, traj_ns);
            ros_ptr_->vizYawTraj(pos_traj, yaw_traj);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        latest_replan.setExpTraj(pos_traj);
        latest_replan.setExpYawTraj(yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        return true;
    }

    bool GeneralPlanner::buildTrackingTargetYawTrajectory(const Trajectory &pos_traj,
                                                        const traj_opt::DynamicTargetStates &target_prediction,
                                                        Trajectory &yaw_traj) {
        if (pos_traj.empty() || target_prediction.empty()) {
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

        VecDf times;
        traj_manager_->yaw()->getYawTimeAllocation(pos_traj.getTotalDuration(), times);
        if (times.size() == 0 || !times.allFinite()) {
            return false;
        }

        VecDf way_pts;
        way_pts.resize(std::max<Eigen::Index>(0, times.size() - 1));
        double eval_t = 0.0;
        double last_yaw = init_yaw[0];
        for (Eigen::Index i = 0; i < way_pts.size(); ++i) {
            eval_t += times(i);
            const double yaw = yawFacingTarget(pos_traj, target_prediction, eval_t, last_yaw);
            way_pts(i) = yaw;
            last_yaw = yaw;
        }

        Vec4f goal_yaw{0.0, 0.0, 0.0, 0.0};
        goal_yaw[0] = yawFacingTarget(pos_traj, target_prediction, pos_traj.getTotalDuration(), last_yaw);
        if (way_pts.size() == 0) {
            geometry_utils::normalizeNextYaw(init_yaw[0], goal_yaw[0]);
        } else {
            geometry_utils::normalizeNextYaw(way_pts(way_pts.size() - 1), goal_yaw[0]);
        }

        const Vec2f init_state = init_yaw.head(2);
        const Vec2f goal_state = goal_yaw.head(2);
        yaw_traj = poly_interpo::minimumAccInterpolation<1>(init_state,
                                                            goal_state,
                                                            way_pts,
                                                            times);
        yaw_traj.start_WT = pos_traj.start_WT;
        return !yaw_traj.empty();
    }

    bool GeneralPlanner::commitTrackingTrajectory(const Trajectory &pos_traj,
                                                const Trajectory &optimized_yaw_traj,
                                                const traj_opt::DynamicTargetStates &target_prediction,
                                                const std::string &traj_ns) {
        if (pos_traj.empty()) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking trajectory is empty, cannot commit.");
            return false;
        }

        Trajectory yaw_traj = optimized_yaw_traj;
        if (yaw_traj.empty() && !buildTrackingTargetYawTrajectory(pos_traj, target_prediction, yaw_traj)) {
            double terminal_yaw = target_prediction.empty() ? robot_state_.yaw : target_prediction.back().yaw;
            if (!target_prediction.empty()) {
                const Vec3f face_dir = target_prediction.back().position - pos_traj.getPos(pos_traj.getTotalDuration());
                if (face_dir.head<2>().norm() > 1.0e-3) {
                    terminal_yaw = std::atan2(face_dir.y(), face_dir.x());
                }
            }
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking target yaw generation failed, fallback to terminal yaw.");
            return commitTaskTrajectory(pos_traj, terminal_yaw, true, traj_ns);
        }

        ExpTraj task_exp_traj;
        task_exp_traj.setGoalConnectedFlag(true);
        task_exp_traj.setWholeTrajKnownFreeFlag(true);
        task_exp_traj.setTrajectory(ros_ptr_->getSimTime(), pos_traj, yaw_traj);

        cmd_traj_info_.setTrajectory(task_exp_traj);
        last_exp_traj_info_ = task_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        {
            TimeConsuming t_viz("tracking_task_viz", false);
            ros_ptr_->vizExpTraj(pos_traj, traj_ns);
            ros_ptr_->vizYawTraj(pos_traj, yaw_traj);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        latest_replan.setExpTraj(pos_traj);
        latest_replan.setExpYawTraj(yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        return true;
    }

    RET_CODE GeneralPlanner::optimizeTrackingTask(const traj_opt::DynamicTargetStates &target_prediction,
                                                const bool &from_rest) {
        if (target_prediction.empty()) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking task has no target prediction.");
            return FAILED;
        }

        TrackingFrontend::Config frontend_cfg;
        frontend_cfg.tracking_distance = cfg_.tracking_distance;
        frontend_cfg.distance_tolerance = cfg_.tracking_distance_tolerance;
        frontend_cfg.height_offset = cfg_.tracking_height_offset;
        frontend_cfg.height_tolerance = cfg_.tracking_height_tolerance;
        frontend_cfg.safe_distance = cfg_.tracking_safe_distance;
        frontend_cfg.visibility_safe_distance = cfg_.tracking_visibility_safe_distance;
        frontend_cfg.visibility_cone_ratio = cfg_.tracking_visibility_cone_ratio;
        frontend_cfg.searching_horizon = cfg_.planning_horizon;
        frontend_cfg.candidate_angle_step = cfg_.tracking_candidate_angle_step;
        frontend_cfg.candidate_radius_num = cfg_.tracking_candidate_radius_num;
        frontend_cfg.visibility_samples = cfg_.tracking_visibility_samples;
        frontend_cfg.unknown_as_occupied = cfg_.tracking_unknown_as_occupied;
        frontend_cfg.use_astar = cfg_.tracking_frontend_astar;

        traj_opt::TrackingProblem problem;
        TimeConsuming t_frontend("tracking_frontend", false);
        TrackingFrontend frontend(frontend_cfg, map_manager_, astar_ptr_);
        if (!frontend.buildProblem(makeTaskHeadState(from_rest), target_prediction, problem)) {
            time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking frontend failed.");
            return FAILED;
        }
        time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj, problem.tail_pvaj, PolytopeVec());

        problem.head_yaw << robot_state_.yaw, 0.0;
        if (!from_rest && !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const Trajectory yaw_traj = cmd_traj_info_.yawTraj();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            const double eval_t = ros_ptr_->getSimTime() - start_wt + cfg_.replan_forward_dt;
            StatePVAJ yaw_state;
            if (!yaw_traj.empty() && eval_t >= 0.0 && eval_t <= total_dur &&
                yaw_traj.getState(eval_t, yaw_state)) {
                problem.head_yaw = yaw_state.row(0).head<2>();
            }
        }
        double terminal_yaw = target_prediction.back().yaw;
        const Vec3f face_dir = target_prediction.back().position - problem.tail_pvaj.col(0);
        if (face_dir.head<2>().norm() > 1.0e-3) {
            terminal_yaw = std::atan2(face_dir.y(), face_dir.x());
            geometry_utils::normalizeNextYaw(problem.head_yaw(0, 0), terminal_yaw);
        }
        problem.tail_yaw << terminal_yaw, target_prediction.back().yaw_rate;
        problem.weight_od_near = cfg_.tracking_weight_od_near;
        problem.weight_od_far = cfg_.tracking_weight_od_far;
        problem.weight_od_vertical = cfg_.tracking_weight_od_vertical;
        problem.weight_oa = cfg_.tracking_weight_oa;
        problem.weight_oe = cfg_.tracking_weight_oe;
        problem.weight_visibility = cfg_.tracking_weight_oe;
        problem.weight_relative_velocity = cfg_.tracking_weight_relative_velocity;
        problem.weight_tangent_velocity = cfg_.tracking_weight_tangent_velocity;
        problem.weight_viewpoint_attractor = cfg_.tracking_weight_viewpoint_attractor;

        {
            TimeConsuming t_viz("tracking_frontend_viz", false);
            ros_ptr_->vizFrontendPath(problem.guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        Trajectory out_traj;
        Trajectory out_yaw_traj;
        TimeConsuming t_opt("tracking_opt", false);
        const bool ok = cfg_.tracking_use_snap
                            ? traj_manager_->trackingSnap()->optimize(problem, out_traj, &out_yaw_traj)
                            : traj_manager_->trackingJerk()->optimize(problem, out_traj, &out_yaw_traj);
        time_consuming_[EXP_TRAJ_OPT] = t_opt.stop();
        if (!ok || out_traj.empty()) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking optimization failed.");
            return FAILED;
        }

        if (!commitTrackingTrajectory(out_traj, out_yaw_traj, target_prediction, cfg_.tracking_use_snap ? "tracking_snap" : "tracking_jerk")) {
            return FAILED;
        }
        ros_ptr_->info(" -- [GeneralPlanner] Tracking task success: pieces={}, duration={}.",
                       out_traj.getPieceNum(), out_traj.getTotalDuration());
        return SUCCESS;
    }

    RET_CODE GeneralPlanner::optimizePerchingTask(const traj_opt::PerchingSurfaceState &surface,
                                                const bool &from_rest) {
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
        frontend_cfg.searching_horizon = cfg_.planning_horizon;
        frontend_cfg.use_astar = cfg_.perching_frontend_astar;
        frontend_cfg.use_dynamics_terminal_accel = cfg_.perching_use_dynamics_terminal_accel;

        traj_opt::PerchingProblem problem;
        TimeConsuming t_frontend("perching_frontend", false);
        PerchingFrontend frontend(frontend_cfg, map_manager_, astar_ptr_);
        if (!frontend.buildProblem(makeTaskHeadState(from_rest), surface, problem)) {
            time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
            ros_ptr_->warn(" -- [GeneralPlanner] Perching frontend failed.");
            return FAILED;
        }
        time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj,
                                      problem.nominal_tail_pvaj, PolytopeVec());

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
            ros_ptr_->warn(" -- [GeneralPlanner] Perching optimization failed.");
            return FAILED;
        }

        if (!commitTaskTrajectory(out_traj, surface.yaw, true, "perching_snap")) {
            return FAILED;
        }
        ros_ptr_->info(" -- [GeneralPlanner] Perching task success: pieces={}, duration={}.",
                       out_traj.getPieceNum(), out_traj.getTotalDuration());
        return SUCCESS;
    }

    RET_CODE GeneralPlanner::PlanTrackingFromRest(const traj_opt::DynamicTargetStates &target_prediction,
                                                const bool &new_task) {
        TimeConsuming total_t("PlanTrackingFromRest", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            ros_ptr_->warn(" -- [GeneralPlanner] in [PlanTrackingFromRest]: No odom, force return.");
            return FAILED;
        }
        const Vec3f goal = target_prediction.empty() ? robot_state_.p : target_prediction.back().position;
        latest_replan.setGoal(goal, target_prediction.empty() ? NAN : target_prediction.back().yaw, robot_state_);
        gi_.goal_p = goal;
        gi_.goal_yaw = target_prediction.empty() ? NAN : target_prediction.back().yaw;
        gi_.new_goal = new_task;
        last_exp_traj_info_.setEmpty();

        const RET_CODE ret = optimizeTrackingTask(target_prediction, true);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::ReplanTrackingOnce(const traj_opt::DynamicTargetStates &target_prediction,
                                              const bool &new_task) {
        TimeConsuming total_t("ReplanTrackingOnce", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            return FAILED;
        }
        const Vec3f goal = target_prediction.empty() ? robot_state_.p : target_prediction.back().position;
        latest_replan.setGoal(goal, target_prediction.empty() ? NAN : target_prediction.back().yaw, robot_state_);
        gi_.goal_p = goal;
        gi_.goal_yaw = target_prediction.empty() ? NAN : target_prediction.back().yaw;
        gi_.new_goal = new_task;

        const RET_CODE ret = optimizeTrackingTask(target_prediction, false);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::PlanPerchingFromRest(const traj_opt::PerchingSurfaceState &surface,
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

        const RET_CODE ret = optimizePerchingTask(surface, true);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::ReplanPerchingOnce(const traj_opt::PerchingSurfaceState &surface,
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

        const RET_CODE ret = optimizePerchingTask(surface, false);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    void GeneralPlanner::getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish) {
        double eval_t = (ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime());
        traj_finish = false;
        double total_dur = cmd_traj_info_.getTotalDuration();
        if (eval_t > total_dur) {
            traj_finish = true;
            eval_t = total_dur;
        }
        start_WT_pos = cmd_traj_info_.getStartWallTime();
        if (cmd_traj_info_.backupTrajAvilibale() && eval_t > cmd_traj_info_.getBackupTrajStartTT()) {
            robot_on_backup_traj_ = true;
        } else {
            robot_on_backup_traj_ = false;
        }
    }

    Trajectory GeneralPlanner::getCommittedPositionTrajectory() {
        return cmd_traj_info_.posTraj();
    }

    Trajectory GeneralPlanner::getCommittedYawTrajectory() {
        return cmd_traj_info_.yawTraj();
    }


    void GeneralPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                             double &yaw,
                                             double &yaw_dot,
                                             bool &on_backup_traj,
                                             bool &traj_finish) {
        cmd_traj_info_.lock();
        const double &cur_t = ros_ptr_->getSimTime();
        const double &cmd_start_WT = cmd_traj_info_.getStartWallTime();
//        const bool &backup_avilibale = cmd_traj_info_.backupTrajAvilibale();
//        const double &backup_start_TT = cmd_traj_info_.getBackupTrajStartTT();
        const double &total_dur = cmd_traj_info_.getTotalDuration();

        traj_finish = (cur_t - cmd_start_WT) > total_dur;
        const double &eval_t = traj_finish ? total_dur : (cur_t - cmd_start_WT);

//        bool last_round_robot_on_backup_traj = robot_on_backup_traj_;
        robot_on_backup_traj_ = cmd_traj_info_.isTTOnBackupTraj(eval_t);
        on_backup_traj = robot_on_backup_traj_;

        pvaj = cmd_traj_info_.posTraj().getState(eval_t);


        /// Get Yaw planning
        static double last_yaw = robot_state_.yaw;

        yaw = cmd_traj_info_.getYaw((eval_t))[0];
        yaw_dot = cmd_traj_info_.getYawRate((eval_t))[0];

        if (isnan(yaw)) {
            yaw = last_yaw;
            yaw_dot = 0;
        } else {
            last_yaw = yaw;
        }
        if (isnan(yaw_dot)) {
            yaw_dot = 0;
        }

//        if (last_round_robot_on_backup_traj != robot_on_backup_traj_) {
//            if (last_round_robot_on_backup_traj) {
//                ros_ptr_->info(" -- [CMD] Emergency Stop End ========================");
//            } else {
//                ros_ptr_->info(" -- [CMD] Emergency Stop Start ========================");
//            }
//        }

//        double cur_yaw = geometry_utils::get_yaw_from_quaternion(robot_state_.q);
        cmd_traj_info_.unlock();
    }


    void GeneralPlanner::getModuleTimeConsuming(vector<double> &time) {
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }


    RET_CODE GeneralPlanner::generateExpTraj(ExpTraj &last_exp_traj_info, ExpTraj &out_exp_traj_info) {
        /* 1) Log the exp traj frontend time*/
        TimeConsuming t_exp_frontend("t_exp_frontend", false);

        // use hot init or not, just prepare a guide path, a guide t, init and fina state and sfc for exp traj opt
        StatePVAJ pos_init_state, pos_fina_state;
        PolytopeVec sfc;
        vec_Vec3f guide_path;
        // the guide_stamp saves a TT
        vector<double> guide_stamp;
        double guide_path_end_vel{0.0};
        int reserve_size = cfg_.planning_horizon / cfg_.resolution * 1.2;
        guide_path.reserve(reserve_size);
        guide_stamp.reserve(reserve_size);

        Vec4f init_yaw{robot_state_.yaw, 0, 0, 0};
        Vec4f fina_yaw{0, 0, 0, 0};


        // alias for last_exp_traj_info
        Trajectory guide_pos_traj, guide_yaw_traj, last_exp_traj;

        // record the wall time (WT) and the trajectory time (TT) at the start of the replan.
        const double replan_process_start_WT = ros_ptr_->getSimTime();
        double replan_process_start_TT, replan_state_TT;
        const bool planning_from_rest = last_exp_traj_info.empty();
        const bool use_plain_exp_traj = cfg_.plain_traj_en;
        const bool use_esdf_exp_traj = cfg_.esdf_traj_en && !use_plain_exp_traj;
        const bool use_distance_field_exp_traj = use_plain_exp_traj || use_esdf_exp_traj;

        /* 2) Check last exp traj */
        if (planning_from_rest) {
            /* 2.1) Perform rest2rest exp traj generation */
            // just skip the first part of the guide trajectory
            pos_init_state.setZero();
            pos_init_state.col(0) = local_start_p_;
            replan_process_start_TT = -1;
            replan_state_TT = -1;
        } else {
            guide_pos_traj = cmd_traj_info_.posTraj(); // last_exp_traj;
            guide_yaw_traj = cmd_traj_info_.yawTraj(); //last_exp_traj_info.exp_yaw_traj;
            last_exp_traj = last_exp_traj_info.posTraj();

            replan_process_start_TT = replan_process_start_WT - last_exp_traj.start_WT;
            replan_state_TT = replan_process_start_TT + cfg_.replan_forward_dt;
            /* 2.2) Perform collision check on last exp traj*/
            vector<TimePosPair> last_exp_traj_time_pos;
            vector<double> last_exp_traj_vel;


            // check early exit condition
            // 1) if the replan state is beyond the last cmd traj, return NO_NEED
            if (replan_state_TT >= cmd_traj_info_.getTotalDuration()) {
                out_exp_traj_info = last_exp_traj_info;

                if (robot_on_backup_traj_) {
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [GeneralPlanner] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                }

                if (cfg_.print_log) {
                    ros_ptr_->warn(
                            " -- [generateExpTraj] replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                }
                return NO_NEED;
            }

            if (!last_exp_traj_info.empty()) {
                if (replan_state_TT >= last_exp_traj.getTotalDuration()) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [generateExpTraj] replan_state_TT >= last_exp_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [GeneralPlanner] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                /// 1) Check a series of early termination conditions.
                if (!gi_.new_goal && last_exp_traj_info.getSFCSize() == 1 && last_exp_traj_info.connectedToGoal()) {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(
                                " -- [GeneralPlanner] Replan, last exp have only one corridor and connected to goal return NONEED.");
                    }

                    out_exp_traj_info = last_exp_traj_info;
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [GeneralPlanner] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                if (!gi_.new_goal &&
                    (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3) {
                    // Return if the traj close to goal
                    out_exp_traj_info = last_exp_traj_info;
                    out_exp_traj_info.setGoalConnectedFlag(true);

                    ros_ptr_->warn(" -- [GeneralPlanner] Replan, close to goal and return NONEED.");
                    if (robot_on_backup_traj_) {
                        ros_ptr_->warn(
                                " -- [GeneralPlanner] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }
            }
            /// Ready for replan.
            out_exp_traj_info.setGoalConnectedFlag(false);

            // * 2) Check if in backup trajectory. While in backup trajectory,
            // *    the guide trajectory should be a part of cmd trajectory.
            // TODO: Why cannot directly replan on cmd traj? 241121

            // * 3) Perform collision check on the guide trajectory.
            // TODO 0929 critical change for hot init.
            double eval_t = replan_state_TT; //replan_process_start_TT;
            double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();

            Vec3f temp_pt, last_sample_pt;
            last_exp_traj_time_pos.clear();
            last_exp_traj_info.setWholeTrajKnownFreeFlag(true);
            last_sample_pt = guide_pos_traj.getPos(eval_t);
            eval_t += cfg_.sample_traj_dt;
            // * 4) 记录replan点在evaluated_pts上的id
            int replan_id = -1;
            for (; eval_t < guide_pos_traj_total_time; eval_t += cfg_.sample_traj_dt) {
                temp_pt = guide_pos_traj.getPos(eval_t);
                if ((temp_pt - last_sample_pt).norm() < cfg_.resolution * 0.8) {
                    continue;
                }

                rog_map::GridType temp_grid = map_manager_->getInfGridType(temp_pt);

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

            if (!gi_.new_goal &&
                use_distance_field_exp_traj &&
                last_exp_traj_info.connectedToGoal() &&
                last_exp_traj_info.wholeTrajKnownFree()) {
                out_exp_traj_info = last_exp_traj_info;
                if (robot_on_backup_traj_) {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(
                                " -- [GeneralPlanner] Distance-field replan, emergency stop, return FAILED and wait for plan form rest.");
                    }
                    return FAILED;
                }
                return NO_NEED;
            }

            // * 6) Decide where to split the original exp trajecory and re-plan a new one with an A*,
            // *    If the whole trajectory if free,  the whole trajectory should be receding and if not, or a new goal
            // *    is given, we should only receiding a small distance and replan new trajectory ASAP
            double split_dis = cfg_.receding_dis;
            if (last_exp_traj_info.wholeTrajKnownFree() && !gi_.new_goal && cfg_.receding_dis > 0.0) {
                split_dis = std::numeric_limits<double>::max();
            }


            // * 7）Begin replan process, first get the replan state from the committed trajectory.
            if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {
                ros_ptr_->warn(" -- [GeneralPlanner] Invalid traj or eval t");
                return FAILED;
            }
            // * Generate guide path with time stampe, for hot trajectory initialization
            // * the guide stamp is time from the replan start t
            guide_stamp.clear();
            guide_path.clear();
            if (split_dis <= 0 || last_exp_traj_time_pos.empty()) {
                /// No need receding, just path search.
                guide_path.push_back(pos_init_state.col(0));
                guide_stamp.push_back(0.0);
                last_exp_traj_time_pos.clear();
                last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
                guide_path_end_vel = robot_state_.v.norm();
            } else {
                temp_pt = last_exp_traj_time_pos.back().second;
                // * 8) Pop all evaluated pts after the sampled point.
                while (map_manager_->isOccupiedInflate(temp_pt) ||
                       (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
                    last_exp_traj_time_pos.pop_back();
                    last_exp_traj_vel.pop_back();
                    if (last_exp_traj_time_pos.empty()) {
                        ros_ptr_->warn(" -- [GeneralPlanner] WARN, all traj is collide in INF2");
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
                    guide_path_end_vel = robot_state_.v.norm();
                }
            }
        }

        // second, geometry part of the guide path
        ///=================The Second Part of Guide Path ================================================

        double guide_path_length = geometry_utils::computePathLength(guide_path);
        double temp_horizon = cfg_.planning_horizon - guide_path_length;

        vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        vector<int> sfc_waypoint_ids;

        if (guide_path.empty() ||
            ((guide_path.front() - pos_init_state.col(0)).norm() > 1e-2)) {
            guide_path.insert(guide_path.begin(), pos_init_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }

        // if need a geometry path
        if (temp_horizon > cfg_.resolution * 2) {
            /// start point TT + exp_traj start_WT
//            double path_search_start_point_WT = guide_stamp.back() + guide_pos_traj.start_WT;
            // if the goal is close to the last point of the guide path, just add the goal to the guide path
            if ((guide_path.back() - gi_.goal_p).norm() < cfg_.resolution * 5) {
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - gi_.goal_p).norm() / cfg_.exp_traj_cfg.max_vel);
                guide_path.push_back(gi_.goal_p);
                // NO NEED
            } else {
                vec_Vec3f new_path;
                // project goal within the planning horizon
//                const Vec3f dir = (gi_.goal_p - robot_state_.p).normalized();
//                const double dis2goal = (gi_.goal_p - robot_state_.p).norm();
//                Vec3f cadi_p = gi_.goal_p;
//                if(dis2goal > cfg_.planning_horizon) {
//                    double proj_l = cfg_.planning_horizon;
//                    Vec3f cadi_p = robot_state_.p + dir * proj_l;
//                    int max_iter = 100;
//                    while(map_manager_->isOccupiedInflate(cadi_p) && max_iter-- > 0) {
//                        if(map_manager_->getNearestInfCellNot(OCCUPIED, cadi_p, cadi_p, 1.0)) {
//                            break;
//                        }
//                        proj_l -= 2.0;
//                        if(proj_l < 1){
//                            ros_ptr_->warn(" -- [GeneralPlanner] Project goal failed");
//                            gi_.goal_valid = false;
//                            return FAILED;
//                        }
//                        cadi_p = robot_state_.p + dir * proj_l;
//                    }
//                    if(max_iter <= 0) {
//                        ros_ptr_->warn(" -- [GeneralPlanner] Project goal failed");
//                        gi_.goal_valid = false;
//                        return FAILED;
//                    }
//                }
                if (!PathSearch(guide_path.back(), gi_.goal_p, temp_horizon, new_path)) {
                    ros_ptr_->warn(" -- [GeneralPlanner] PathSearch for new path failed");
                    return FAILED;
                } else if (new_path.size() < 2) {
                    ros_ptr_->warn(" -- [GeneralPlanner] PathSearch for new path failed");
                    return FAILED;
                } else {

                    // compute total dis
                    // backward compute dis for all points
                    double total_dis{0.0};
                    vector<double> dis(new_path.size());
                    Vec3f last_p = new_path.back();
                    for (int i = new_path.size() - 2; i >= 0; i--) {
                        auto d = (new_path[i] - last_p).norm();
                        total_dis += d;
                        dis[i+1] = total_dis;
                        last_p = new_path[i];
                    }
                    total_dis += (new_path.front() - guide_path.back()).norm();
                    dis[0] = total_dis;
//                for (int i = 0; i < dis.size(); i++) {
//                    cout << dis[i] << " ";
//                }
//                cout << endl;
                    vector<double> stamps(new_path.size(), 0);
                    vector<double> dt(new_path.size(), 0);
                    double last_stamp = 0;
                    for (int i = dis.size() - 1; i >= 0; i--) {
                        double vel;
                        geometry_utils::simplePMTimeAllocator(cfg_.exp_traj_cfg.max_acc, cfg_.exp_traj_cfg.max_vel,
                                                              guide_path_end_vel,
                                                              total_dis,
                                                              dis[i], stamps[i], vel);
                        dt[i] = stamps[i] - last_stamp;
                        last_stamp = stamps[i];
                    }
                    double time_stamp = guide_stamp.back();

//                for (int i = 0; i < stamps.size(); i++) {
//                    cout << stamps[i] << " ";
//                }
//                cout << endl;
//
//                for (int i = 0; i < dt.size(); i++) {
//                    cout << dt[i] << " ";
//                }
//                cout << endl;

                    for (long unsigned int i = 1; i < new_path.size(); i++) {
                        double t = dt[i];
                        time_stamp += t;
                        guide_path.emplace_back(new_path[i]);
                        guide_stamp.emplace_back(time_stamp);
                    }
                }
            }
	        }

            if (!prepareESDFGuideEndpoint(guide_path, guide_stamp)) {
                ros_ptr_->warn(" -- [GeneralPlanner] Failed to prepare ESDF rolling local endpoint.");
                return FAILED;
            }

	        const bool connected_goal = (guide_path.back().head(2) - gi_.goal_p.head(2)).norm() < cfg_.resolution * 2;
	        out_exp_traj_info.setGoalConnectedFlag(connected_goal);

        sfc.clear();
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizFrontendPath(guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        if (use_esdf_exp_traj && !map_manager_->hasESDF()) {
            ros_ptr_->warn(" -- [GeneralPlanner] ESDF exp traj is enabled, but ROGMap ESDF is unavailable.");
            return FAILED;
        }

        shifted_sfc_start_pt_ = Vec3f(9999,9999,9999);
        if (!use_esdf_exp_traj && !use_plain_exp_traj) {
            bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, shifted_sfc_start_pt_, cfg_.use_fov_cut);

            if (!bool_ret_code) {
                ros_ptr_->warn(" -- [GeneralPlanner] SearchPolytopeOnPath for new path failed");
                return FAILED;
            }
            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizExpSfc(sfc);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
        }

        time_consuming_[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();


        pos_fina_state.setZero();
        pos_fina_state.col(0) = guide_path.back();
        const bool local_endpoint_is_global_goal =
                (pos_fina_state.col(0) - gi_.goal_p).norm() < cfg_.resolution * 2;
        if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
            pos_fina_state.col(1) = (gi_.goal_p - robot_state_.p).normalized() * cfg_.exp_traj_cfg.max_vel / 2;
        }
        if (local_endpoint_is_global_goal) {
            pos_fina_state.col(1).setZero();
            pos_fina_state.col(0) = gi_.goal_p;
        }

        // optimize and update exp traj
        bool temp_ret;
        Trajectory out_traj;
        TimeConsuming t_exp_opt("t_exp_opt", false);
        traj_manager_->setSwarmCurrentWallTime(replan_process_start_WT);
        if (use_esdf_exp_traj) {
            temp_ret = traj_manager_->esdf()->optimize(pos_init_state,
                                                pos_fina_state,
                                                guide_path,
                                                guide_stamp,
                                                out_traj);
            if (!temp_ret) {
                if (!planning_from_rest && last_exp_traj_info.wholeTrajKnownFree()) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [GeneralPlanner] ESDF candidate optimization failed, keep current safe trajectory.");
                    }
                    return NO_NEED;
                }
                ros_ptr_->warn(" -- [GeneralPlanner] ESDF optimization failed.");
                return FAILED;
            }
        } else if (use_plain_exp_traj) {
            temp_ret = traj_manager_->plain()->optimize(pos_init_state,
                                                 pos_fina_state,
                                                 guide_path,
                                                 guide_stamp,
                                                 out_traj);
            if (!temp_ret) {
                if (!planning_from_rest && last_exp_traj_info.wholeTrajKnownFree()) {
                    out_exp_traj_info = last_exp_traj_info;
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [GeneralPlanner] Plain candidate optimization failed, keep current safe trajectory.");
                    }
                    return NO_NEED;
                }
                ros_ptr_->warn(" -- [GeneralPlanner] Plain optimization failed.");
                return FAILED;
            }
        } else {
            temp_ret = traj_manager_->exp()->optimize(pos_init_state,
                                               pos_fina_state,
                                               guide_path,
                                               guide_stamp,
                                               sfc,
                                               out_traj);
        }
        time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();
        if (use_esdf_exp_traj || use_plain_exp_traj) {
            latest_replan.setExpCondition(VecDf(), vec_Vec3f(), pos_init_state, pos_fina_state, sfc);
        } else {
            VecDf init_ts;
            vec_Vec3f init_ps;
            traj_manager_->exp()->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        }
        if (!temp_ret) {
            ros_ptr_->warn(" -- [GeneralPlanner] OptimizationExpTraj for new path failed");
            return FAILED;
        }
        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);
        if (replan_total_t > cfg_.replan_forward_dt) {
            if (!planning_from_rest) {
                ros_ptr_->warn(" -- [GeneralPlanner] Replan over time({})!!!! Return FAILED", replan_total_t);
                return FAILED;
            }
            ros_ptr_->warn(" -- [GeneralPlanner] PlanFromRest over realtime budget({}), accept initial trajectory.", replan_total_t);
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpTraj(out_traj,
                                 use_plain_exp_traj ? "plain_traj" : (use_esdf_exp_traj ? "esdf_traj" : "exp_traj"));
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        double new_traj_WT = replan_process_start_WT;

        replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;
        Trajectory temp_exp_traj;
        if (!last_exp_traj_info_.empty() &&
            !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       temp_exp_traj)) {
            ros_ptr_->error(" -- [GeneralPlanner] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.setSFC(sfc);
        temp_exp_traj = temp_exp_traj + out_traj;
        temp_exp_traj.start_WT = new_traj_WT; //last_exp_traj_info.replan_start_WT ;

        if (!last_exp_traj_info.empty()) {
            StatePVAJ yaw_replan_state;
            if (!guide_yaw_traj.getState(replan_state_TT, yaw_replan_state)) {
                ros_ptr_->warn(" -- [GeneralPlanner] Invalid traj or eval t");
                return FAILED;
            }
            init_yaw = yaw_replan_state.row(0);
        }


        bool free_end{true};
        if (cfg_.goal_yaw_en && !isnan(gi_.goal_yaw) && connected_goal) {
            free_end = false;
            fina_yaw[0] = gi_.goal_yaw;
        }
        Trajectory new_traj, old_traj;

        if (!traj_manager_->yaw()->optimize(init_yaw, fina_yaw, out_traj, new_traj, 3, false, free_end)) {
            ros_ptr_->error(" -- [GeneralPlanner] in [generateExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        if (!last_exp_traj_info.empty()) {
            if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                           old_traj)) {
                ros_ptr_->error(" -- [GeneralPlanner] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
        }

        const auto temp_yaw_traj = old_traj + new_traj;
        // check if part of the exp on last backup
        double on_backup_end_TT{-1}, on_backup_start_TT{-1};
        if (!last_exp_traj_info.empty() && replan_state_TT > cmd_traj_info_.getBackupTrajStartTT()) {
            on_backup_start_TT = cmd_traj_info_.getBackupTrajStartTT() - replan_process_start_TT;
            on_backup_end_TT = replan_state_TT - replan_process_start_TT;
        }
        out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj, on_backup_start_TT,
                                        on_backup_end_TT);

        latest_replan.setExpYawTraj(temp_yaw_traj);
        latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

    RET_CODE GeneralPlanner::generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info) {
        drone_state_mutex_.lock();
        back_traj_info.setRobotPos(robot_state_.p);
        drone_state_mutex_.unlock();
        TimeConsuming t_back_frontend("t_back_frontend", false);

        if (!cfg_.backup_traj_en || cfg_.esdf_traj_en || cfg_.plain_traj_en) {
            back_traj_info.setEmpty();
            time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
            return FINISH;
        }

        double total_dur = ref_exp_traj.getTotalDuration();
        double start_t = ros_ptr_->getSimTime() - ref_exp_traj.getStartWallTime();


        if (start_t > total_dur - 0.01) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [GeneralPlanner] in [generateBackupTrajectory]: start_t > total_dur, return NO_NEED");
            }
            return NO_NEED;
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        // 同时记录每一个点的刹车时间和刹车距离
        vector<double> min_stop_dis;
        vector<TimePosPair> eval_ps;
        Vec3f temp_vel;

        // 记录当前时刻到最远时刻的所有可视部分
        Vec3f last_pos = ref_exp_traj.getPos(start_t);
        for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt) {
            temp_point = ref_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) {
                continue;
            }
            last_pos = temp_point;
            temp_vel = ref_exp_traj.getVel(out_t);
            // Compute initial
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.push_back(std::pair<double, Vec3f>(out_t, temp_point));
            const double min_dis =
                    cfg_.sensing_horizon > 0 ? std::min(cfg_.sensing_horizon, cfg_.safe_corridor_line_max_length)
                                             : cfg_.safe_corridor_line_max_length;
            if (!map_manager_->isLineFree(back_traj_info.getRobotPos(),
                                      temp_point,
                                      min_dis,
                                      cfg_.seed_line_neighbour)) {
                all_traj_visible = false;
                break;
            }
        }

        if (all_traj_visible) {
            back_traj_info.setEmpty();
            {
                double dur = ref_exp_traj.getTotalDuration();
                Vec3f seed_pt = ref_exp_traj.getPos(dur);
                Line line{back_traj_info.getRobotPos(), seed_pt};
                Polytope temp_poly;
                if (cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
                    back_traj_info.setSFC(temp_poly);
                    {
                        TimeConsuming t_viz("tviz", false);
                        ros_ptr_->vizBackupSfc(temp_poly);
                        time_consuming_[VISUALIZATION] += t_viz.stop();
                    }
                }
            }
            return FINISH;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= cfg_.sample_traj_dt;
            Vec3f out_p = ref_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > cfg_.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);

        // TODO check this logic, comment on Dec. 13
        // if
        // 1) last exp traj has a backup traj
        // 2) last backup WT is larger than this term
        // 3) last exp is collision free
        // if (ref_exp_traj.back_traj_start_TT > 0 &&
        // seed_point_t < ref_exp_traj.back_traj_start_TT) {
        // return NO_NEED;
        // }


        Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);

        Vec3f shifted_robot_p = shifted_sfc_start_pt_.norm()> 999?robot_state_.p:shifted_sfc_start_pt_;
        if (!map_manager_->getNearestCellNot(GridType::OCCUPIED, shifted_robot_p, shifted_robot_p, 3.0)) {
            ros_ptr_->error(
                    " -- [GeneralPlanner] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_START_POINT);
            return FAILED;
        }

        Line line{shifted_robot_p, seed_point};
        Polytope temp_poly;
        if (!cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
            ros_ptr_->warn(" -- [GeneralPlanner] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        Eigen::Matrix3Xd vPoly;
        if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
            ros_ptr_->warn(" -- [GeneralPlanner] Cannot generate feasible backup sfc, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        if (cfg_.use_fov_cut) {
            if (!fov_checker_->cutPolyByFov(robot_state_.p, robot_state_.q, seed_point,
                                            temp_poly)) {
                ros_ptr_->warn(" -- [GeneralPlanner] cutPolyByFov failed, force return");
                return FAILED;
            }
        }
        // cut by sensing horizon
        if (cfg_.sensing_horizon > 0 &&
            !fov_checker_->cutPolyBySensingHorizon(robot_state_.p, seed_point, cfg_.sensing_horizon,
                                                   temp_poly)) {
            ros_ptr_->warn(" -- [GeneralPlanner] cutPolyBySensingHorizon failed, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        back_traj_info.setSFC(temp_poly);

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizBackupSfc(temp_poly);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

//        Vec3f out_p = temp_point;
//        double t_R = 0.0;
        double eval_t = eval_ps.back().first + cfg_.sample_traj_dt;
        last_pos = eval_ps.back().second;
        while (temp_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = ref_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
                eval_t += cfg_.sample_traj_dt;
                continue;
            }
            temp_vel = ref_exp_traj.getVel(out_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt;
        }
        eval_ps.pop_back();
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        //        bool use_new{true};
        //        if (use_new) {
        double t0 = ros_ptr_->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        //            cout << "t0: " << t0 << endl;
        //            cout << "te: " << te << endl;
        //            cout << "exp_traj_dur: " << ref_exp_traj.optimized_exp_traj.getTotalDuration() << endl;
        double vel_e_n = ref_exp_traj.getVel(te).norm();
        double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
        double heu_dur = te - heu_ts;
        Vec3f heu_p = seed_point;
        time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        Trajectory temp_pos_traj;
        auto sfc0 = back_traj_info.getSFC();
        bool temp_ret = traj_manager_->backup()->optimize(ref_exp_traj.posTraj(),
                                                 t0,
                                                 te,
                                                 heu_ts,
                                                 heu_p,
                                                 heu_dur,
                                                 back_traj_info.getSFC(),
                                                 temp_pos_traj,
                                                 opt_ts);
        time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();

        {
            double init_ts;
            VecDf init_times;
            vec_Vec3f init_ps;
            traj_manager_->backup()->getInitValue(init_ts, init_times, init_ps);
            latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                             t0, te,
                                             back_traj_info.getSFC());
            Trajectory traj;
            double out_ts;
            traj_manager_->backup()->optimize(ref_exp_traj.posTraj(),
                                     t0,
                                     te,
                                     init_ts,
                                     sfc0,
                                     init_times,
                                     init_ps,
                                     traj,
                                     out_ts
            );

        }

        if (!temp_ret) {
            ros_ptr_->warn(" -- [GeneralPlanner] OptimizationBakTrajInPolytopes failed, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        } else {
            Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
            Vec4f yaw_goal{0, 0, 0, 0};
            bool free_end{true};
            if (cfg_.goal_yaw_en) {
                if (!isnan(gi_.goal_yaw)) {
                    free_end = false;
                    yaw_goal[0] = gi_.goal_yaw;
                }
            }
            Trajectory temp_yaw_traj;
            if (!traj_manager_->yaw()->optimize(yaw_init_vec, yaw_goal, temp_pos_traj,
                                         temp_yaw_traj, 3, false, free_end)) {
                ros_ptr_->error(" -- [GeneralPlanner] in [generateBackupTrajectory] YawTrajOpt FAILD.");
                return OPT_FAILED;
            }


            if (opt_ts < t0) {
                ros_ptr_->error(" -- [GeneralPlanner] opt_ts {} < t0 {}", opt_ts, t0);
                return OPT_FAILED;
            }
            double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
            const auto &committed_ts_WT = cmd_traj_info_.getBackupTrajStartTT();
            if (committed_ts_WT < cmd_traj_info_.getTotalDuration() && new_ts_WT < committed_ts_WT) {
                ros_ptr_->error(" -- [GeneralPlanner] new_ts_WT {} < committed_ts_WT {}", new_ts_WT, committed_ts_WT);
                return OPT_FAILED;
            }


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizBackupTraj(temp_pos_traj);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
            latest_replan.setBackupTraj(temp_pos_traj);
            latest_replan.setBackupYawTraj(temp_yaw_traj);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [GeneralPlanner] Cannot find backup traj start point.");
        return FAILED;
    }

    int GeneralPlanner::getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt) {
        if (goals.size() == 1) {
            return 0;
        }
        Vec3f a = start_pt, b;
        int min_id = 0;
        double min_dis = 1e10;
        for (long unsigned int i = 0; i < goals.size() - 1; i++) {
            b = goals[i];
            double dis = geometry_utils::pointLineSegmentDistance(start_pt, a, b);
            if (dis < min_dis) {
                min_dis = dis;
                min_id = i;
            }
            a = b;
        }
        return min_id;
    }

    bool
    GeneralPlanner::PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                             const double &searching_horizon,
                             vec_Vec3f &path) {
        using namespace path_search;
        if (searching_horizon <= 0.0) {
            ros_ptr_->error(" -- [GeneralPlanner] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        rog_map::GridType start_type;
        start_type = map_manager_->getGridType(start_pt);

        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (start_type == rog_map::GridType::OCCUPIED ||
            start_type == rog_map::GridType::OUT_OF_MAP) {
            ros_ptr_->warn(
                    " -- [GeneralPlanner] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
        vec_Vec3f out_path;
        RET_CODE ret_es = astar_ptr_->escapePathSearch(start_pt, flag_es, out_path);
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                ros_ptr_->error(
                        " -- [GeneralPlanner] Escape path search failed with [{}], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
        //            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) | DONT_USE_INF_NEIGHBOR;

        RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                               path);

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        const bool distance_field_frontend = cfg_.esdf_traj_en || cfg_.plain_traj_en;
        if (ret_code == NO_PATH && !distance_field_frontend) {
            flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                          path);
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        } else if (ret_code == NO_PATH && distance_field_frontend) {
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map in distance-field mode; skip prob-map fallback.\n");
        }
        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            ros_ptr_->error(
                    " -- [GeneralPlanner] Path search failed with [{}], force return.\n", RET_CODE_STR[ret_code].c_str());
            return false;
        }
        if (!start_point_escape_path.empty()) {
            path.insert(path.begin(), start_point_escape_path.begin(),
                        start_point_escape_path.end());
        }

        if (path.empty()) {
            ros_ptr_->warn(
                    " -- [GeneralPlanner] Path search failed with empty segments, force return.");
            return false;
        }
        path.insert(path.begin(), start_pt);
        if (ret_code == REACH_GOAL) {
            path.push_back(goal);
        }
        return true;
    }


    void GeneralPlanner::getRobotState(rog_map::RobotState &out) {
        robot_state_ = map_manager_->getRobotState();
        out = robot_state_;
    }
}
