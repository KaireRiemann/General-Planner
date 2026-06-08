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
#include <checker/state2state_checker.hpp>
#include <checker/trajectory_checker.hpp>
#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <limits>
#include <memory>
#include <super_utils/scope_timer.hpp>
#include <utils/optimization/polynomial_interpolation.h>
#include <fmt/color.h>
#include <fmt/format.h>

using namespace super_utils;

namespace general_planner {
    namespace {
        void setFailureReason(std::string *out, const std::string &reason) {
            if (out != nullptr) {
                *out = reason;
            }
        }

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

        void makeHoldCommandFromRobotState(const rog_map::RobotState &robot_state,
                                           StatePVAJ &pvaj,
                                           double &yaw,
                                           double &yaw_dot,
                                           bool &on_backup_traj,
                                           bool &traj_finish) {
            pvaj.setZero();
            if (robot_state.rcv && robot_state.p.allFinite()) {
                pvaj.col(0) = robot_state.p;
            }
            yaw = std::isfinite(robot_state.yaw) ? robot_state.yaw : 0.0;
            yaw_dot = 0.0;
            on_backup_traj = false;
            traj_finish = true;
        }

        bool trackingPerchingPerchingStatus(
                const TrackingPerchingTransitionManager::Status status) {
            return status == TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED ||
                   status == TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING ||
                   status == TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT ||
                   status == TrackingPerchingTransitionManager::Status::CONTACT;
        }

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

        bool buildYawPrefixFromSamples(const Trajectory &yaw_traj,
                                       const double sample_start_t,
                                       const double sample_end_t,
                                       const double prefix_duration,
                                       Trajectory &prefix_yaw) {
            if (yaw_traj.empty() ||
                prefix_duration <= 1.0e-5 ||
                sample_start_t < -1.0e-6 ||
                sample_end_t < sample_start_t - 1.0e-6) {
                return false;
            }

            const double total_duration = yaw_traj.getTotalDuration();
            if (sample_start_t > total_duration + 1.0e-6) {
                return false;
            }

            const double clamped_start = std::clamp(sample_start_t, 0.0, total_duration);
            const double clamped_end = std::clamp(sample_end_t, clamped_start, total_duration);
            StatePVAJ start_state;
            StatePVAJ end_state;
            if (!yaw_traj.getState(clamped_start, start_state) ||
                !yaw_traj.getState(clamped_end, end_state)) {
                return false;
            }

            Eigen::Matrix<double, 1, 2> init_state;
            Eigen::Matrix<double, 1, 2> goal_state;
            init_state << start_state(0, 0), start_state(0, 1);
            goal_state << end_state(0, 0), end_state(0, 1);
            geometry_utils::normalizeNextYaw(init_state(0, 0), goal_state(0, 0));

            Eigen::Matrix<double, 1, -1> waypoints(1, 0);
            VecDf times(1);
            times(0) = prefix_duration;
            prefix_yaw = poly_interpo::minimumAccInterpolation<1>(init_state,
                                                                  goal_state,
                                                                  waypoints,
                                                                  times);
            prefix_yaw.start_WT = yaw_traj.start_WT + clamped_start;
            return !prefix_yaw.empty();
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

        bool extractYawPrefixForStitching(const Trajectory &tracking_yaw,
                                          const double prefix_start,
                                          const double prefix_duration,
                                          Trajectory &prefix_yaw,
                                          bool &used_sampled_fallback) {
            used_sampled_fallback = false;
            if (tracking_yaw.empty() || prefix_duration <= 1.0e-5) {
                return false;
            }

            const double yaw_total = tracking_yaw.getTotalDuration();
            if (prefix_start < -1.0e-6 || prefix_start > yaw_total + 1.0e-6) {
                return false;
            }

            const double prefix_end = prefix_start + prefix_duration;
            const double sample_end = std::min(prefix_end, yaw_total);
            const double query_t = std::clamp(prefix_start, 0.0, std::max(0.0, yaw_total - 1.0e-7));
            double local_query_t = query_t;
            const int piece_idx = tracking_yaw.locatePieceIdx(local_query_t);
            const int degree = tracking_yaw[piece_idx].getDegree();
            if ((degree == 5 || degree == 7) &&
                sample_end > prefix_start + 1.0e-5 &&
                tracking_yaw.getPartialTrajectoryByTime(prefix_start, sample_end, prefix_yaw)) {
                if (std::abs(prefix_yaw.getTotalDuration() - prefix_duration) > 1.0e-4) {
                    return buildYawPrefixFromSamples(tracking_yaw,
                                                     prefix_start,
                                                     sample_end,
                                                     prefix_duration,
                                                     prefix_yaw);
                }
                return true;
            }

            used_sampled_fallback = true;
            return buildYawPrefixFromSamples(tracking_yaw,
                                             prefix_start,
                                             sample_end,
                                             prefix_duration,
                                             prefix_yaw);
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

        void appendGuideUnique(const Vec3f &point, vec_Vec3f &path) {
            if (!point.allFinite()) {
                return;
            }
            if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
                path.emplace_back(point);
            }
        }

        void appendGuideTimedUnique(const Vec3f &point,
                                    const double stamp,
                                    vec_Vec3f &path,
                                    std::vector<double> &path_t) {
            if (!point.allFinite() || !std::isfinite(stamp)) {
                return;
            }
            if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
                path.emplace_back(point);
                path_t.emplace_back(stamp);
            } else if (!path_t.empty()) {
                path_t.back() = stamp;
            }
        }

        double interpolateSegmentStamp(const std::vector<double> &times,
                                       const int left_id,
                                       const double alpha,
                                       const double fallback_start_t,
                                       const double fallback_end_t) {
            if (times.size() > static_cast<std::size_t>(left_id + 1) &&
                std::isfinite(times[static_cast<std::size_t>(left_id)]) &&
                std::isfinite(times[static_cast<std::size_t>(left_id + 1)])) {
                const double left_t = times[static_cast<std::size_t>(left_id)];
                const double right_t = std::max(left_t, times[static_cast<std::size_t>(left_id + 1)]);
                return left_t + alpha * (right_t - left_t);
            }
            return fallback_start_t + alpha * std::max(0.0, fallback_end_t - fallback_start_t);
        }

        int validVisibleRegionCount(const traj_opt::TrackingProblem &problem) {
            return static_cast<int>(std::count_if(problem.visible_regions.begin(),
                                                  problem.visible_regions.end(),
                                                  [](const traj_opt::TrackingVisibleRegion &region) {
                                                      return region.valid;
                                                  }));
        }

        bool staticTargetPrediction(const traj_opt::DynamicTargetStates &prediction,
                                    const double position_epsilon,
                                    const double velocity_epsilon) {
            if (prediction.empty()) {
                return false;
            }
            const Vec3f ref = prediction.front().position;
            double max_span = 0.0;
            double max_vel = 0.0;
            for (const auto &state : prediction) {
                max_span = std::max(max_span, (state.position - ref).norm());
                max_vel = std::max(max_vel, state.velocity.norm());
            }
            return max_span <= std::max(0.0, position_epsilon) &&
                   max_vel <= std::max(0.0, velocity_epsilon);
        }

        Vec3f trackingTargetDirection(const traj_opt::DynamicTargetStates &prediction,
                                      const double speed_threshold) {
            if (prediction.empty()) {
                return Vec3f::UnitX();
            }

            Vec3f dir = prediction.front().velocity;
            dir.z() = 0.0;
            if (dir.norm() > speed_threshold) {
                return dir.normalized();
            }

            if (prediction.size() >= 2) {
                dir = prediction.back().position - prediction.front().position;
                dir.z() = 0.0;
                if (dir.norm() > 1.0e-4) {
                    return dir.normalized();
                }
            }

            return Vec3f::UnitX();
        }

        double trackingDistanceError(const Vec3f &tracker,
                                     const Vec3f &target,
                                     const double desired_distance,
                                     const double desired_height) {
            if (!tracker.allFinite() || !target.allFinite()) {
                return std::numeric_limits<double>::infinity();
            }
            const Vec3f rel = tracker - target;
            const double h_err = std::abs(rel.head<2>().norm() - desired_distance);
            const double z_err = std::abs(rel.z() - desired_height);
            return h_err + 0.5 * z_err;
        }
    }

    GeneralPlanner::GeneralPlanner
            (const std::string &cfg_path,
             const ros_interface::RosInterface::Ptr &ros_ptr,
             const rog_map::ROGMapROS::Ptr &map_ptr
            ) : cfg_(Config(cfg_path)),
                map_manager_(std::make_shared<MapManager>(map_ptr)),
                ros_ptr_(ros_ptr) {

        const auto config_check = checker::checkState2StateConfig(cfg_);
        logCheckResult(ros_ptr_, "state2state config", config_check);
        if (config_check.rejected()) {
            throw std::invalid_argument("state2state config invalid: " + config_check.code +
                                        " " + config_check.message);
        }
        ros_ptr_->setResolution(cfg_.resolution);
        ros_ptr_->setVisualizationEn(cfg_.visualization_en);
        tracking_runtime_manager_ = std::make_unique<TrackingRuntimeManager>(cfg_, map_manager_);
        perching_runtime_manager_ = std::make_unique<PerchingRuntimeManager>(cfg_, map_manager_);
        takeoff_runtime_manager_ = std::make_unique<TakeoffRuntimeManager>(cfg_, map_manager_);
        tracking_perching_manager_ = std::make_unique<TrackingPerchingTransitionManager>();
        tracking_to_perching_initializer_ = std::make_unique<TrackingToPerchingInitializer>();
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
        takeoff_frontend_ = std::make_unique<TakeoffFrontend>(
                makeTakeoffFrontendConfig(), map_manager_, astar_ptr_);
        takeoff_optimizer_ =
                std::make_unique<traj_opt::DynamicTakeoffSnapTrajOpt>(cfg_.esdf_traj_cfg, ros_ptr_);
        takeoff_optimizer_->setMapManager(map_manager_);
        takeoff_optimizer_->setSafeDistance(cfg_.esdf_safe_distance);
        exploration_frontend_ = std::make_unique<ExplorationFrontend>(
                makeExplorationFrontendConfig(), map_manager_, astar_ptr_);
        exploration_runtime_manager_ = std::make_unique<ExplorationRuntimeManager>(cfg_);
        const auto ellipsoid_optimizer_config =
                optimization_utils::EllipsoidOptimizer::makeConfig(cfg_.ellipsoid_optimizer,
                                                                   cfg_.ellipsoid_optimizer_fallback);
        cg_ptr_ = std::make_shared<CorridorGenerator>(ros_ptr_, map_manager_, cfg_.corridor_bound_dis,
                                                      cfg_.corridor_line_max_length,
                                                      cfg_.resolution, rog_map_cfg.virtual_ground_height,
                                                      rog_map_cfg.virtual_ceil_height,
                                                      cfg_.robot_r,
                                                      cfg_.obs_skip_num,
                                                      cfg_.iris_iter_num,
                                                      ellipsoid_optimizer_config);
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
        const auto input_check = checker::checkState2StateInput(goal_p,
                                                                goal_yaw,
                                                                robot_state_,
                                                                map_manager_,
                                                                ros_ptr_->getSimTime());
        if (rejectOnCheckFailure(ros_ptr_, "PlanFromRest input", input_check)) {
            latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
            latest_replan.setRetCode(input_check.code == "MAP_NOT_READY"
                                     ? GENERAL_RET_CODE::GENERAL_MAP_NOT_READY
                                     : GENERAL_RET_CODE::GENERAL_UNDEFINED);
            return FAILED;
        }
        warnHighSpeedMargin(ros_ptr_, cfg_, robot_state_.v.norm(), "PlanFromRest high-speed margin");
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
        if (!map_manager_->getNearestInfCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0)) {
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

        if (!backupTrajectoryPlanningEnabled(cfg_)) {
            if (rejectOnCheckFailure(ros_ptr_,
                                     "PlanFromRest exp commit",
                                     checker::checkExpTrajectory(exp_traj_info, cfg_, "plan_from_rest_exp"))) {
                return FAILED;
	            }
	            robot_on_backup_traj_ = false;
	            cmd_traj_info_.setTrajectory(exp_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }

        back_traj_info.setEmpty();
        RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);;

        if (back_ret_code == SUCCESS) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [GeneralPlanner] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            if (rejectOnCheckFailure(ros_ptr_,
                                     "PlanFromRest exp+backup commit",
                                     checker::checkExpBackupCommit(exp_traj_info,
                                                                   back_traj_info,
                                                                   cfg_,
                                                                   "plan_from_rest_exp_backup"))) {
                return FAILED;
	            }
	            if (!cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info)) {
	                ros_ptr_->error(" -- [Checker] PlanFromRest commit failed: CmdTraj rejected exp+backup trajectory.");
	                return FAILED;
	            }
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
	            if (rejectOnCheckFailure(ros_ptr_,
                                     "PlanFromRest exp commit",
                                     checker::checkExpTrajectory(exp_traj_info, cfg_, "plan_from_rest_exp"))) {
                return FAILED;
            }
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

        const auto input_check = checker::checkState2StateInput(goal_p,
                                                                goal_yaw,
                                                                robot_state_,
                                                                map_manager_,
                                                                ros_ptr_->getSimTime());
        if (rejectOnCheckFailure(ros_ptr_, "ReplanOnce input", input_check)) {
            latest_replan.reset();
            latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
            latest_replan.setRetCode(input_check.code == "MAP_NOT_READY"
                                     ? GENERAL_RET_CODE::GENERAL_MAP_NOT_READY
                                     : GENERAL_RET_CODE::GENERAL_UNDEFINED);
            return FAILED;
        }
        warnHighSpeedMargin(ros_ptr_, cfg_, robot_state_.v.norm(), "ReplanOnce high-speed margin");

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

        if (!backupTrajectoryPlanningEnabled(cfg_)) {
            if (rejectOnCheckFailure(ros_ptr_,
                                     "ReplanOnce exp commit",
                                     checker::checkExpTrajectory(exp_traj_info, cfg_, "replan_exp"))) {
                return FAILED;
	            }
	            robot_on_backup_traj_ = false;
	            cmd_traj_info_.setTrajectory(exp_traj_info);
            last_exp_traj_info_ = exp_traj_info;
            gi_.new_goal = false;
            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            return SUCCESS;
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
            if (rejectOnCheckFailure(ros_ptr_,
                                     "ReplanOnce exp+backup commit",
                                     checker::checkExpBackupCommit(exp_traj_info,
                                                                   back_traj_info,
                                                                   cfg_,
                                                                   "replan_exp_backup"))) {
                return FAILED;
	            }
	            if (!cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info)) {
	                ros_ptr_->error(" -- [Checker] ReplanOnce commit failed: CmdTraj rejected exp+backup trajectory.");
	                return FAILED;
	            }
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
            if (rejectOnCheckFailure(ros_ptr_,
                                     "ReplanOnce exp commit",
                                     checker::checkExpTrajectory(exp_traj_info, cfg_, "replan_exp"))) {
                return FAILED;
            }
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

    bool GeneralPlanner::commitTakeoffTrajectory(const Trajectory &pos_traj,
                                                 const std::string &traj_ns) {
        const bool committed = commitTaskTrajectory(pos_traj, NAN, false, traj_ns);
        if (committed && takeoff_runtime_manager_) {
            takeoff_runtime_manager_->updateStatusAfterCommit();
        }
        return committed;
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

    bool GeneralPlanner::commitTrackingToPerchingTrajectory(
            const Trajectory &tracking_pos,
            const Trajectory &tracking_yaw,
            const double current_tracking_local_t,
            const double handover_delay,
            const Trajectory &perching_pos,
            const Trajectory &perching_yaw,
            const std::string &traj_ns) {
        if (perching_pos.empty() || perching_yaw.empty()) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason=empty_perching_suffix");
            return false;
        }

        const double commit_wt = ros_ptr_->getSimTime();
        Trajectory committed_pos = perching_pos;
        Trajectory committed_yaw = perching_yaw;
        bool stitched = false;
        const bool use_prefix =
                cfg_.tracking_to_perching_stitch_prefix &&
                handover_delay > 1.0e-4 &&
                !tracking_pos.empty() &&
                !tracking_yaw.empty();
        if (use_prefix) {
            const double prefix_start = current_tracking_local_t;
            const double prefix_end =
                    std::min(current_tracking_local_t + handover_delay,
                             tracking_pos.getTotalDuration());
            const double prefix_duration = prefix_end - prefix_start;
            Trajectory prefix_pos;
            Trajectory prefix_yaw;
            bool used_sampled_yaw_prefix = false;
            const bool prefix_pos_ok =
                    prefix_end > prefix_start + 1.0e-4 &&
                    tracking_pos.getPartialTrajectoryByTime(prefix_start, prefix_end, prefix_pos);
            const bool prefix_yaw_ok =
                    prefix_pos_ok &&
                    extractYawPrefixForStitching(tracking_yaw,
                                                 prefix_start,
                                                 prefix_duration,
                                                 prefix_yaw,
                                                 used_sampled_yaw_prefix);
            if (prefix_pos_ok && prefix_yaw_ok) {
                committed_pos = prefix_pos + perching_pos;
                committed_yaw = prefix_yaw + perching_yaw;
                stitched = true;

                const StatePVAJ prefix_tail = prefix_pos.getState(prefix_pos.getTotalDuration());
                const StatePVAJ suffix_head = perching_pos.getState(0.0);
                ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_STITCHED_COMMIT prefix_dt={:.3f}, suffix_dt={:.3f}, pos_jump={:.4f}, vel_jump={:.4f}, sampled_yaw_prefix={}",
                               prefix_pos.getTotalDuration(),
                               perching_pos.getTotalDuration(),
                               (prefix_tail.col(0) - suffix_head.col(0)).norm(),
                               (prefix_tail.col(1) - suffix_head.col(1)).norm(),
                               used_sampled_yaw_prefix);
            } else {
                ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason=prefix_extract_failed, pos_ok={}, yaw_ok={}, prefix_start={:.3f}, prefix_end={:.3f}, tracking_pos_dur={:.3f}, tracking_yaw_dur={:.3f}",
                               prefix_pos_ok,
                               prefix_yaw_ok,
                               prefix_start,
                               prefix_end,
                               tracking_pos.getTotalDuration(),
                               tracking_yaw.getTotalDuration());
                return false;
            }
        }

        committed_pos.start_WT = commit_wt;
        committed_yaw.start_WT = commit_wt;

        ExpTraj task_exp_traj;
        task_exp_traj.setGoalConnectedFlag(true);
        task_exp_traj.setWholeTrajKnownFreeFlag(true);
        task_exp_traj.setTrajectory(commit_wt, committed_pos, committed_yaw);

        cmd_traj_info_.setTrajectory(task_exp_traj);
        last_exp_traj_info_ = task_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        {
            TimeConsuming t_viz("tracking_perching_task_viz", false);
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
        ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_COMMIT_SUCCESS stitched={}, total_duration={:.3f}, handover_delay={:.3f}",
                       stitched,
                       committed_pos.getTotalDuration(),
                       handover_delay);
        return true;
    }

    bool GeneralPlanner::commitTrackingTrajectory(const Trajectory &pos_traj,
                                                const Trajectory &optimized_yaw_traj,
                                                const traj_opt::DynamicTargetStates &target_prediction,
                                                const std::string &traj_ns) {
        if (pos_traj.empty()) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking trajectory is empty, cannot commit.");
            return false;
        }

        const double commit_wt = ros_ptr_->getSimTime();
        bool has_old_cmd = false;
        Trajectory old_pos_traj;
        Trajectory old_yaw_traj;
        double old_start_wt = 0.0;
        double old_total_dur = 0.0;
        if (!cmd_traj_info_.empty()) {
            has_old_cmd = true;
            cmd_traj_info_.lock();
            old_pos_traj = cmd_traj_info_.posTraj();
            old_yaw_traj = cmd_traj_info_.yawTraj();
            old_start_wt = cmd_traj_info_.getStartWallTime();
            old_total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();
        }

        auto keepOldFromSnapshot = [&](const std::string &reason) -> bool {
            if (!has_old_cmd || old_pos_traj.empty()) {
                return false;
            }
            latest_replan.setExpTraj(old_pos_traj);
            latest_replan.setExpYawTraj(old_yaw_traj);
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [Tracking] TRACKING_KEEP_OLD_ACTIVE reason={}, keep_old_count={}, reject_count={}",
                               reason,
                               tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld() : tracking_consecutive_keep_old_,
                               tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject() : tracking_consecutive_reject_);
            }
            return true;
        };

        auto decisionTypeName = [](const TrackingRuntimeManager::DecisionType type) -> const char * {
            switch (type) {
                case TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE:
                    return "COMMIT_CANDIDATE";
                case TrackingRuntimeManager::DecisionType::KEEP_OLD:
                    return "KEEP_OLD";
                case TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE:
                    return "FORCE_COMMIT_CANDIDATE";
                case TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL:
                    return "REJECT_AND_FAIL";
            }
            return "UNKNOWN";
        };

        auto logRuntimeDecision =
                [&](const TrackingRuntimeManager::Decision &decision,
                    const Trajectory &candidate,
                    const bool anti_rollback_pass,
                    const std::string &tag) {
            if (!cfg_.print_log) {
                return;
            }
            const double guard_h =
                    std::min(cfg_.tracking_no_motion_check_horizon,
                             candidate.getTotalDuration());
            const double candidate_disp =
                    guard_h > 1.0e-6
                        ? (candidate.getPos(guard_h) -
                           candidate.getPos(0.0)).head<2>().norm()
                        : 0.0;
            const double candidate_speed0 =
                    candidate.empty() ? 0.0 : candidate.getVel(0.0).head<2>().norm();
            const char *log_name = "TRACKING_MANAGER_DECISION_REJECT";
            if (decision.type == TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE) {
                log_name = "TRACKING_MANAGER_DECISION_COMMIT";
            } else if (decision.type ==
                       TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE) {
                log_name = "TRACKING_MANAGER_DECISION_FORCE_COMMIT";
            } else if (decision.type == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
                log_name = "TRACKING_MANAGER_DECISION_KEEP_OLD";
            }
            ros_ptr_->info(" -- [Tracking] {} tag={}, decision={}, reason={}, candidate_safe={}, candidate_commandable={}, anti_rollback_pass={}, bypass_anti_rollback={}, candidate_duration={:.3f}, candidate_disp_0p35s={:.3f}, candidate_speed0={:.3f}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}, keep_old_count={}, reject_count={}",
                           log_name,
                           tag,
                           decisionTypeName(decision.type),
                           decision.reason,
                           decision.candidate_safe,
                           decision.candidate_commandable,
                           anti_rollback_pass,
                           decision.bypass_anti_rollback,
                           candidate.getTotalDuration(),
                           candidate_disp,
                           candidate_speed0,
                           decision.old_activity.remaining,
                           decision.old_activity.speed0,
                           decision.old_activity.displacement,
                           decision.old_activity.progress,
                           decision.old_activity.expected_progress,
                           decision.old_activity.avg_tracking_error,
                           tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld() : tracking_consecutive_keep_old_,
                           tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject() : tracking_consecutive_reject_);
            if (decision.candidate_safe && !decision.candidate_commandable) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_NO_MOTION reason={}, candidate_duration={:.3f}, candidate_disp_0p35s={:.3f}, candidate_speed0={:.3f}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               decision.reason,
                               candidate.getTotalDuration(),
                               candidate_disp,
                               candidate_speed0,
                               decision.old_activity.remaining,
                               decision.old_activity.speed0,
                               decision.old_activity.displacement,
                               decision.old_activity.progress,
                               decision.old_activity.expected_progress,
                               decision.old_activity.avg_tracking_error);
            }
            if (!anti_rollback_pass &&
                decision.type == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_ANTI_ROLLBACK reason={}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               decision.reason,
                               decision.old_activity.remaining,
                               decision.old_activity.speed0,
                               decision.old_activity.displacement,
                               decision.old_activity.progress,
                               decision.old_activity.expected_progress,
                               decision.old_activity.avg_tracking_error);
            }
        };

        auto applyRuntimeDecision =
                [&](const Trajectory &candidate,
                    const std::string &tag) -> TrackingRuntimeManager::DecisionType {
            if (!cfg_.tracking_runtime_manager_enable || !tracking_runtime_manager_) {
                return trackingCommitPassesAntiRollback(candidate, target_prediction, commit_wt)
                           ? TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE
                           : TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
            }

            const bool has_old_tracking =
                    has_old_cmd &&
                    tracking_runtime_manager_->hasCommittedTracking() &&
                    !old_pos_traj.empty();
            const double old_local_t =
                    has_old_tracking
                        ? std::clamp(commit_wt - old_start_wt, 0.0, old_total_dur)
                        : 0.0;
            std::string candidate_safe_reason;
            const bool candidate_safe =
                    tracking_runtime_manager_->trajectorySafe(
                            candidate,
                            0.0,
                            std::min(cfg_.tracking_keep_old_horizon,
                                     candidate.getTotalDuration()),
                            cfg_.tracking_keep_old_safety_dt,
                            &candidate_safe_reason);
            const bool anti_rollback_pass =
                    has_old_tracking
                        ? trackingCommitPassesAntiRollback(candidate,
                                                           target_prediction,
                                                           commit_wt)
                        : true;
            auto decision =
                    tracking_runtime_manager_->decide(has_old_tracking ? &old_pos_traj : nullptr,
                                                      old_local_t,
                                                      candidate,
                                                      target_prediction,
                                                      candidate_safe,
                                                      anti_rollback_pass);
            if (!candidate_safe && !candidate_safe_reason.empty()) {
                decision.reason = decision.reason.empty()
                                      ? candidate_safe_reason
                                      : decision.reason + ": " + candidate_safe_reason;
            }
            logRuntimeDecision(decision, candidate, anti_rollback_pass, tag);

            switch (decision.type) {
                case TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE:
                case TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE:
                    return decision.type;
                case TrackingRuntimeManager::DecisionType::KEEP_OLD:
                    tracking_runtime_manager_->onKeepOld();
                    if (keepOldFromSnapshot(decision.reason)) {
                        return TrackingRuntimeManager::DecisionType::KEEP_OLD;
                    }
                    tracking_runtime_manager_->onRejected();
                    return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
                case TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL:
                    tracking_runtime_manager_->onRejected();
                    return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
            }
            tracking_runtime_manager_->onRejected();
            return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
        };

        Trajectory yaw_traj = optimized_yaw_traj;
        if (yaw_traj.empty() && !buildTrackingTargetYawTrajectory(pos_traj, target_prediction, yaw_traj)) {
            const auto decision = applyRuntimeDecision(pos_traj, "yaw_fallback");
            if (decision == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
                return true;
            }
            if (decision == TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL) {
                return false;
            }
            double terminal_yaw = target_prediction.empty() ? robot_state_.yaw : target_prediction.back().yaw;
            if (!target_prediction.empty()) {
                const Vec3f face_dir = target_prediction.back().position - pos_traj.getPos(pos_traj.getTotalDuration());
                if (face_dir.head<2>().norm() > 1.0e-3) {
                    terminal_yaw = std::atan2(face_dir.y(), face_dir.x());
                }
            }
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking target yaw generation failed, fallback to terminal yaw.");
            const bool committed = commitTaskTrajectory(pos_traj, terminal_yaw, true, traj_ns);
            if (committed) {
                if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                    tracking_runtime_manager_->onCommitted();
                }
                resetTrackingCommitCounters();
            }
            return committed;
        }

        Trajectory committed_pos_traj = pos_traj;
        Trajectory committed_yaw_traj = yaw_traj;
        committed_pos_traj.start_WT = commit_wt;
        committed_yaw_traj.start_WT = commit_wt;
        if (has_old_cmd) {
            const double prefix_start_t = commit_wt - old_start_wt;
            const double prefix_end_t = prefix_start_t + std::max(0.0, cfg_.replan_forward_dt);
            const bool prefix_window_valid =
                    !old_pos_traj.empty() &&
                    !old_yaw_traj.empty() &&
                    prefix_start_t >= 0.0 &&
                    prefix_end_t > prefix_start_t + 1.0e-4 &&
                    prefix_end_t <= old_total_dur + 1.0e-6;

            if (prefix_window_valid) {
                Trajectory prefix_pos_traj;
                Trajectory prefix_yaw_traj;
                if (old_pos_traj.getPartialTrajectoryByTime(prefix_start_t,
                                                            std::min(prefix_end_t, old_total_dur),
                                                            prefix_pos_traj) &&
                    old_yaw_traj.getPartialTrajectoryByTime(prefix_start_t,
                                                            std::min(prefix_end_t, old_yaw_traj.getTotalDuration()),
                                                            prefix_yaw_traj)) {
                    committed_pos_traj = prefix_pos_traj + pos_traj;
                    committed_yaw_traj = prefix_yaw_traj + yaw_traj;
                    committed_pos_traj.start_WT = commit_wt;
                    committed_yaw_traj.start_WT = commit_wt;

                    if (cfg_.print_log) {
                        const StatePVAJ old_tail = prefix_pos_traj.getState(prefix_pos_traj.getTotalDuration());
                        const StatePVAJ new_head = pos_traj.getState(0.0);
                        const double pos_jump = (old_tail.col(0) - new_head.col(0)).norm();
                        const double vel_jump = (old_tail.col(1) - new_head.col(1)).norm();
                        ros_ptr_->info(" -- [GeneralPlanner] Tracking replan stitched old prefix: dt={:.3f}s, pos_jump={:.4f}, vel_jump={:.4f}.",
                                       prefix_pos_traj.getTotalDuration(),
                                       pos_jump,
                                       vel_jump);
                    }
                } else if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [GeneralPlanner] Tracking replan prefix extraction failed; commit raw tracking trajectory.");
                }
            } else if (cfg_.print_log && prefix_start_t >= 0.0) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking replan prefix unavailable: start_t={:.3f}, end_t={:.3f}, total={:.3f}.",
                               prefix_start_t,
                               prefix_end_t,
                               old_total_dur);
            }
        }
        committed_pos_traj.start_WT = commit_wt;
        committed_yaw_traj.start_WT = commit_wt;

        const auto runtime_decision = applyRuntimeDecision(committed_pos_traj, "final_commit");
        if (runtime_decision == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
            return true;
        }
        if (runtime_decision == TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL) {
            return false;
        }

        ExpTraj task_exp_traj;
        task_exp_traj.setGoalConnectedFlag(true);
        task_exp_traj.setWholeTrajKnownFreeFlag(true);
        task_exp_traj.setTrajectory(commit_wt, committed_pos_traj, committed_yaw_traj);

        cmd_traj_info_.setTrajectory(task_exp_traj);
        last_exp_traj_info_ = task_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        {
            TimeConsuming t_viz("tracking_task_viz", false);
            ros_ptr_->vizExpTraj(committed_pos_traj, traj_ns);
            ros_ptr_->vizYawTraj(committed_pos_traj, committed_yaw_traj);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        latest_replan.setExpTraj(committed_pos_traj);
        latest_replan.setExpYawTraj(committed_yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            tracking_runtime_manager_->onCommitted();
        }
        resetTrackingCommitCounters();

        if (cfg_.print_log) {
            const double guard_h =
                    std::min(cfg_.tracking_no_motion_check_horizon,
                             committed_pos_traj.getTotalDuration());
            const double candidate_disp =
                    guard_h > 1.0e-6
                        ? (committed_pos_traj.getPos(guard_h) -
                           committed_pos_traj.getPos(0.0)).head<2>().norm()
                        : 0.0;
            const double speed0 = committed_pos_traj.getVel(0.0).head<2>().norm();
            const double now_minus_start_wt = ros_ptr_->getSimTime() - committed_pos_traj.start_WT;
            ros_ptr_->info(" -- [Tracking] TRACKING_CANDIDATE_COMMITTED candidate_duration={:.3f}, candidate_disp_0p35s={:.3f}, candidate_speed0={:.3f}, now_minus_start_WT={:.3f}, committed_total_duration={:.3f}",
                           committed_pos_traj.getTotalDuration(),
                           candidate_disp,
                           speed0,
                           now_minus_start_wt,
                           committed_pos_traj.getTotalDuration());

            const auto target0 = target_prediction.empty()
                                     ? traj_opt::DynamicTargetState()
                                     : target_prediction.front();
            const bool target_moving =
                    !target_prediction.empty() &&
                    target0.velocity.head<2>().norm() >
                    cfg_.tracking_no_motion_target_speed_threshold;
            const double short_h = std::min(0.15, committed_pos_traj.getTotalDuration());
            const double short_disp =
                    short_h > 1.0e-6
                        ? (committed_pos_traj.getPos(short_h) -
                           committed_pos_traj.getPos(0.0)).head<2>().norm()
                        : 0.0;
            if (target_moving &&
                short_disp < cfg_.tracking_no_motion_min_displacement &&
                speed0 < cfg_.tracking_keep_old_min_speed) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_COMMITTED_BUT_NO_MOTION start_WT={:.3f}, now={:.3f}, duration={:.3f}, now_minus_start_WT={:.3f}, speed={:.3f}, displacement={:.3f}",
                               committed_pos_traj.start_WT,
                               ros_ptr_->getSimTime(),
                               committed_pos_traj.getTotalDuration(),
                               now_minus_start_wt,
                               speed0,
                               short_disp);
            }
            if (std::abs(now_minus_start_wt) > cfg_.tracking_commit_start_time_tolerance) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_COMMIT_START_TIME_OFFSET now_minus_start_WT={:.3f}, tolerance={:.3f}, start_WT={:.3f}, now={:.3f}",
                               now_minus_start_wt,
                               cfg_.tracking_commit_start_time_tolerance,
                               committed_pos_traj.start_WT,
                               ros_ptr_->getSimTime());
            }
        }
        return true;
    }

    bool GeneralPlanner::trackingTrajectorySafeForHorizon(const Trajectory &traj,
                                                          const double start_t,
                                                          const double horizon,
                                                          const double dt) const {
        if (traj.empty()) {
            return false;
        }
        if (horizon <= 1.0e-6) {
            return true;
        }
        const double total_dur = traj.getTotalDuration();
        if (!std::isfinite(start_t) ||
            start_t < -1.0e-6 ||
            start_t + horizon > total_dur + 1.0e-6) {
            return false;
        }
        if (map_manager_ == nullptr || !map_manager_->ready()) {
            return true;
        }

        const double safe_dt = std::max(0.05, dt);
        Vec3f last = traj.getPos(std::clamp(start_t, 0.0, total_dur));
        if (!last.allFinite()) {
            return false;
        }

        for (double offset = 0.0; offset <= horizon + 1.0e-6; offset += safe_dt) {
            const double t = std::clamp(start_t + offset, 0.0, total_dur);
            const Vec3f pos = traj.getPos(t);
            if (!pos.allFinite() || !map_manager_->insideLocalMap(pos)) {
                return false;
            }
            const auto grid_type = map_manager_->getInfGridType(pos);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP) {
                return false;
            }
            if (cfg_.tracking_unknown_as_occupied &&
                (grid_type == rog_map::GridType::UNKNOWN ||
                 grid_type == rog_map::GridType::UNDEFINED ||
                 grid_type == rog_map::GridType::FRONTIER)) {
                return false;
            }
            if (map_manager_->hasESDF()) {
                double dist = 0.0;
                Vec3f grad = Vec3f::Zero();
                if (map_manager_->evaluateESDF(pos, dist, grad) &&
                    dist < cfg_.tracking_safe_distance) {
                    return false;
                }
            }
            if ((pos - last).norm() > 1.0e-4 &&
                !map_manager_->isLineFree(last, pos, true, cfg_.tracking_unknown_as_occupied)) {
                return false;
            }
            last = pos;
        }
        return true;
    }

    bool GeneralPlanner::currentTrackingTrajectorySafeForHorizon(const double horizon) {
        if (cmd_traj_info_.empty()) {
            return false;
        }

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const double old_start_wt = cmd_traj_info_.getStartWallTime();
        cmd_traj_info_.unlock();

        const double start_t = ros_ptr_->getSimTime() - old_start_wt;
        return trackingTrajectorySafeForHorizon(old_pos_traj,
                                                start_t,
                                                std::max(0.0, horizon),
                                                cfg_.tracking_keep_old_safety_dt);
    }

    bool GeneralPlanner::keepOldTrackingTrajectory(const std::string &reason) {
        const double keep_horizon = std::max(0.0, cfg_.tracking_keep_old_horizon);
        if (!currentTrackingTrajectorySafeForHorizon(keep_horizon)) {
            return false;
        }

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const Trajectory old_yaw_traj = cmd_traj_info_.yawTraj();
        cmd_traj_info_.unlock();

        latest_replan.setExpTraj(old_pos_traj);
        latest_replan.setExpYawTraj(old_yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        ros_ptr_->info(" -- [GeneralPlanner] {}; keep old tracking trajectory for {:.2f}s.",
                       reason,
                       keep_horizon);
        return true;
    }

    GeneralPlanner::TrackingTrajectoryActivity
    GeneralPlanner::evaluateTrackingTrajectoryActivity(
            const Trajectory &traj,
            const double local_start_t,
            const traj_opt::DynamicTargetStates &target_prediction,
            const double horizon,
            const double dt) const {
        TrackingTrajectoryActivity out;

        if (traj.empty() || target_prediction.empty()) {
            out.reason = "empty trajectory or target prediction";
            return out;
        }

        const double total_dur = traj.getTotalDuration();
        if (!std::isfinite(local_start_t) ||
            local_start_t < -1.0e-6 ||
            local_start_t > total_dur + 1.0e-6) {
            out.reason = "local_start_t outside duration";
            return out;
        }

        out.valid = true;
        out.remaining = std::max(0.0, total_dur - local_start_t);
        if (out.remaining < cfg_.tracking_keep_old_min_remaining) {
            out.reason = "remaining time too short";
            return out;
        }

        const double eval_horizon = std::min(std::max(0.0, horizon), out.remaining);
        const double safe_dt = std::max(0.03, dt);
        const auto target0 = interpolateTargetPrediction(target_prediction, 0.0);
        out.target_moving =
                target0.velocity.head<2>().norm() >
                cfg_.tracking_no_motion_target_speed_threshold;
        const Vec3f target_dir =
                trackingTargetDirection(target_prediction,
                                        cfg_.tracking_no_motion_target_speed_threshold);

        Vec3f last_p = traj.getPos(std::clamp(local_start_t, 0.0, total_dur));
        if (!last_p.allFinite()) {
            out.reason = "non-finite initial point";
            return out;
        }

        out.speed0 = traj.getVel(std::clamp(local_start_t, 0.0, total_dur)).head<2>().norm();
        out.safe = true;
        double total_error = 0.0;
        int sample_count = 0;

        for (double s = 0.0; s <= eval_horizon + 1.0e-6; s += safe_dt) {
            const double traj_t = std::min(total_dur, local_start_t + s);
            const Vec3f p = traj.getPos(traj_t);
            if (!p.allFinite()) {
                out.safe = false;
                out.reason = "non-finite sample";
                return out;
            }

            if (map_manager_ != nullptr && map_manager_->ready()) {
                if (!map_manager_->insideLocalMap(p)) {
                    out.safe = false;
                    out.reason = "outside local map";
                    return out;
                }

                const auto grid_type = map_manager_->getInfGridType(p);
                if (grid_type == rog_map::GridType::OCCUPIED ||
                    grid_type == rog_map::GridType::OUT_OF_MAP) {
                    out.safe = false;
                    out.reason = "occupied or out-of-map";
                    return out;
                }

                if (cfg_.tracking_unknown_as_occupied &&
                    (grid_type == rog_map::GridType::UNKNOWN ||
                     grid_type == rog_map::GridType::UNDEFINED ||
                     grid_type == rog_map::GridType::FRONTIER)) {
                    out.safe = false;
                    out.reason = "unknown treated as occupied";
                    return out;
                }

                if ((p - last_p).norm() > 1.0e-4 &&
                    !map_manager_->isLineFree(last_p, p, true, cfg_.tracking_unknown_as_occupied)) {
                    out.safe = false;
                    out.reason = "segment not line-free";
                    return out;
                }
            }

            const auto target = interpolateTargetPrediction(target_prediction, s);
            total_error += trackingDistanceError(p,
                                                 target.position,
                                                 cfg_.tracking_distance,
                                                 cfg_.tracking_height_offset);
            ++sample_count;

            if (s > 1.0e-6) {
                const Vec3f dp = p - last_p;
                out.displacement += dp.head<2>().norm();
                out.progress += dp.head<2>().dot(target_dir.head<2>());
            }
            last_p = p;
        }

        out.avg_tracking_error =
                sample_count > 0 ? total_error / static_cast<double>(sample_count) : 0.0;
        out.tracking_error =
                trackingDistanceError(traj.getPos(std::clamp(local_start_t, 0.0, total_dur)),
                                      target0.position,
                                      cfg_.tracking_distance,
                                      cfg_.tracking_height_offset);

        if (out.target_moving) {
            if (out.speed0 < cfg_.tracking_keep_old_min_speed) {
                out.reason = "speed too small";
                return out;
            }

            if (out.displacement < cfg_.tracking_keep_old_min_displacement) {
                out.reason = "displacement too small";
                return out;
            }

            out.expected_progress =
                    target0.velocity.head<2>().norm() *
                    eval_horizon *
                    cfg_.tracking_keep_old_min_progress_ratio;
            if (out.progress < out.expected_progress) {
                out.reason = "insufficient target-direction progress";
                return out;
            }

            const double max_err =
                    cfg_.tracking_keep_old_max_tracking_error_scale *
                    std::max(0.1, cfg_.tracking_distance_tolerance);
            if (out.avg_tracking_error > max_err) {
                out.reason = "tracking error too large";
                return out;
            }
        }

        out.active = true;
        out.reason = "safe and active";
        return out;
    }

    bool GeneralPlanner::currentTrackingTrajectorySafeAndActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            TrackingTrajectoryActivity *activity) const {
        if (cmd_traj_info_.empty()) {
            if (activity) {
                activity->reason = "empty committed trajectory";
            }
            return false;
        }

        Trajectory old_pos_traj;
        double old_start_wt = 0.0;
        double total_dur = 0.0;
        auto &mutable_cmd_traj = const_cast<CmdTraj &>(cmd_traj_info_);
        mutable_cmd_traj.lock();
        old_pos_traj = cmd_traj_info_.posTraj();
        old_start_wt = cmd_traj_info_.getStartWallTime();
        total_dur = cmd_traj_info_.getTotalDuration();
        mutable_cmd_traj.unlock();

        const double now = ros_ptr_->getSimTime();
        const double cur_t = std::clamp(now - old_start_wt, 0.0, total_dur);
        TrackingTrajectoryActivity local_activity =
                evaluateTrackingTrajectoryActivity(old_pos_traj,
                                                   cur_t,
                                                   target_prediction,
                                                   cfg_.tracking_keep_old_horizon,
                                                   cfg_.tracking_keep_old_safety_dt);
        if (activity) {
            *activity = local_activity;
        }

        return local_activity.valid &&
               local_activity.safe &&
               local_activity.active;
    }

    bool GeneralPlanner::candidateTrackingTrajectoryCommandable(
            const Trajectory &candidate_pos_traj,
            const traj_opt::DynamicTargetStates &target_prediction,
            std::string *reason) const {
        if (!cfg_.tracking_no_motion_guard_enable) {
            return true;
        }
        if (candidate_pos_traj.empty() || target_prediction.empty()) {
            if (reason) {
                *reason = "empty candidate or target prediction";
            }
            return false;
        }

        const auto target0 = target_prediction.front();
        const bool target_moving =
                target0.velocity.head<2>().norm() >
                cfg_.tracking_no_motion_target_speed_threshold;
        if (!target_moving) {
            return true;
        }

        const double h =
                std::min(cfg_.tracking_no_motion_check_horizon,
                         candidate_pos_traj.getTotalDuration());
        if (h < 1.0e-3) {
            if (reason) {
                *reason = "candidate duration too short";
            }
            return false;
        }

        const Vec3f p0 = candidate_pos_traj.getPos(0.0);
        const Vec3f p1 = candidate_pos_traj.getPos(h);
        const Vec3f v0 = candidate_pos_traj.getVel(0.0);
        if (!p0.allFinite() || !p1.allFinite() || !v0.allFinite()) {
            if (reason) {
                *reason = "candidate contains non-finite state";
            }
            return false;
        }

        const double disp = (p1 - p0).head<2>().norm();
        const double speed = v0.head<2>().norm();
        if (disp < cfg_.tracking_no_motion_min_displacement &&
            speed < cfg_.tracking_keep_old_min_speed) {
            if (reason) {
                *reason = fmt::format("candidate no-motion: disp={:.3f}, speed={:.3f}",
                                      disp,
                                      speed);
            }
            return false;
        }

        return true;
    }

    bool GeneralPlanner::keepOldTrackingTrajectoryIfActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            const std::string &reason) {
        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            if (cmd_traj_info_.empty() ||
                !tracking_runtime_manager_->hasCommittedTracking()) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_INACTIVE reason={}, activity_reason=no committed tracking trajectory, keep_old_count={}, reject_count={}",
                                   reason,
                                   tracking_runtime_manager_->consecutiveKeepOld(),
                                   tracking_runtime_manager_->consecutiveReject());
                }
                return false;
            }

            Trajectory old_pos_traj;
            Trajectory old_yaw_traj;
            double old_start_wt = 0.0;
            double old_total_dur = 0.0;
            cmd_traj_info_.lock();
            old_pos_traj = cmd_traj_info_.posTraj();
            old_yaw_traj = cmd_traj_info_.yawTraj();
            old_start_wt = cmd_traj_info_.getStartWallTime();
            old_total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();

            const double now = ros_ptr_->getSimTime();
            const double old_local_t =
                    std::clamp(now - old_start_wt, 0.0, old_total_dur);
            const auto activity =
                    tracking_runtime_manager_->evaluateActivity(old_pos_traj,
                                                                 old_local_t,
                                                                 target_prediction,
                                                                 cfg_.tracking_keep_old_horizon,
                                                                 cfg_.tracking_keep_old_safety_dt);
            if (!activity.active) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_INACTIVE reason={}, activity_reason={}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}, keep_old_count={}, reject_count={}",
                                   reason,
                                   activity.reason,
                                   activity.remaining,
                                   activity.speed0,
                                   activity.displacement,
                                   activity.progress,
                                   activity.expected_progress,
                                   activity.avg_tracking_error,
                                   tracking_runtime_manager_->consecutiveKeepOld(),
                                   tracking_runtime_manager_->consecutiveReject());
                }
                return false;
            }

            tracking_runtime_manager_->onKeepOld();
            latest_replan.setExpTraj(old_pos_traj);
            latest_replan.setExpYawTraj(old_yaw_traj);
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [Tracking] TRACKING_KEEP_OLD_ACTIVE reason={}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}, keep_old_count={}, reject_count={}",
                               reason,
                               activity.remaining,
                               activity.speed0,
                               activity.displacement,
                               activity.progress,
                               activity.expected_progress,
                               activity.avg_tracking_error,
                               tracking_runtime_manager_->consecutiveKeepOld(),
                               tracking_runtime_manager_->consecutiveReject());
            }
            return true;
        }

        TrackingTrajectoryActivity activity;
        const bool active = currentTrackingTrajectorySafeAndActive(target_prediction, &activity);
        if (!active) {
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_INACTIVE reason={}, activity_reason={}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               reason,
                               activity.reason,
                               activity.remaining,
                               activity.speed0,
                               activity.displacement,
                               activity.progress,
                               activity.expected_progress,
                               activity.avg_tracking_error);
            }
            return false;
        }

        ++tracking_consecutive_keep_old_;
        if (cfg_.print_log) {
            ros_ptr_->info(" -- [Tracking] TRACKING_KEEP_OLD_ACTIVE reason={}, keep_old_count={}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                           reason,
                           tracking_consecutive_keep_old_,
                           activity.remaining,
                           activity.speed0,
                           activity.displacement,
                           activity.progress,
                           activity.expected_progress,
                           activity.avg_tracking_error);
        }

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const Trajectory old_yaw_traj = cmd_traj_info_.yawTraj();
        cmd_traj_info_.unlock();
        latest_replan.setExpTraj(old_pos_traj);
        latest_replan.setExpYawTraj(old_yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        return true;
    }

    bool GeneralPlanner::trackingCandidateSafeForCommit(const Trajectory &candidate_pos_traj) const {
        if (candidate_pos_traj.empty()) {
            return false;
        }
        const double horizon =
                std::min(std::max(0.0, cfg_.tracking_keep_old_horizon),
                         candidate_pos_traj.getTotalDuration());
        return trackingTrajectorySafeForHorizon(candidate_pos_traj,
                                                0.0,
                                                horizon,
                                                cfg_.tracking_keep_old_safety_dt);
    }

    void GeneralPlanner::resetTrackingCommitCounters() {
        tracking_consecutive_keep_old_ = 0;
        tracking_consecutive_reject_ = 0;
        last_tracking_commit_wt_ = ros_ptr_ ? ros_ptr_->getSimTime() : -1.0;
    }

    double GeneralPlanner::trackingViewpointErrorScore(const Vec3f &tracker,
                                                       const Vec3f &target) const {
        return trackingDistanceError(tracker,
                                     target,
                                     cfg_.tracking_distance,
                                     cfg_.tracking_height_offset);
    }

    bool GeneralPlanner::trackingCommitPassesAntiRollback(
            const Trajectory &candidate_pos_traj,
            const traj_opt::DynamicTargetStates &target_prediction,
            const double commit_wt) {
        if (!cfg_.tracking_anti_rollback_enable) {
            return true;
        }
        if (candidate_pos_traj.empty() || target_prediction.empty()) {
            return false;
        }
        if (cmd_traj_info_.empty()) {
            return true;
        }

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const double old_start_wt = cmd_traj_info_.getStartWallTime();
        const double old_total_dur = cmd_traj_info_.getTotalDuration();
        cmd_traj_info_.unlock();

        const double old_eval_t = commit_wt - old_start_wt;
        if (old_pos_traj.empty() || old_eval_t < -1.0e-6) {
            return true;
        }

        const double dt = std::max(0.05, cfg_.tracking_anti_rollback_dt);
        const double horizon =
                std::min({std::max(0.0, cfg_.tracking_anti_rollback_horizon),
                          std::max(0.0, old_total_dur - old_eval_t),
                          candidate_pos_traj.getTotalDuration(),
                          std::max(0.0, target_prediction.back().t)});
        if (horizon < dt + 1.0e-6) {
            return true;
        }
        if (!trackingTrajectorySafeForHorizon(old_pos_traj, old_eval_t, horizon, dt)) {
            return true;
        }

        const double margin = std::max(0.0, cfg_.tracking_anti_rollback_margin);
        int worse_count = 0;
        double max_regression = 0.0;
        for (double offset = dt; offset <= horizon + 1.0e-6; offset += dt) {
            const Vec3f old_pos =
                    old_pos_traj.getPos(std::clamp(old_eval_t + offset,
                                                   0.0,
                                                   old_pos_traj.getTotalDuration()));
            const Vec3f new_pos =
                    candidate_pos_traj.getPos(std::clamp(offset,
                                                         0.0,
                                                         candidate_pos_traj.getTotalDuration()));
            const Vec3f target_pos = interpolateTargetPrediction(target_prediction, offset).position;
            const double old_score = trackingViewpointErrorScore(old_pos, target_pos);
            const double new_score = trackingViewpointErrorScore(new_pos, target_pos);
            const double regression = new_score - old_score;
            if (regression > margin) {
                ++worse_count;
                max_regression = std::max(max_regression, regression);
            }
        }

        if (worse_count >= 2 || max_regression > 2.0 * margin) {
            TrackingTrajectoryActivity old_activity =
                    evaluateTrackingTrajectoryActivity(old_pos_traj,
                                                       std::clamp(old_eval_t, 0.0, old_total_dur),
                                                       target_prediction,
                                                       cfg_.tracking_keep_old_horizon,
                                                       cfg_.tracking_keep_old_safety_dt);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_ANTI_ROLLBACK reject_count={}, keep_old_count={}, worse_count={}, max_regression={:.3f}, horizon={:.2f}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject() : tracking_consecutive_reject_,
                               tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld() : tracking_consecutive_keep_old_,
                               worse_count,
                               max_regression,
                               horizon,
                               old_activity.remaining,
                               old_activity.speed0,
                               old_activity.displacement,
                               old_activity.progress,
                               old_activity.expected_progress,
                               old_activity.avg_tracking_error);
            } else {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking commit rejected by anti-rollback gate: worse_count={}, max_regression={:.3f}, horizon={:.2f}s.",
                               worse_count,
                               max_regression,
                               horizon);
            }
            return false;
        }
        return true;
    }

    bool GeneralPlanner::trackingGuidePointSafe(const Vec3f &point) const {
        if (!point.allFinite()) {
            return false;
        }
        if (map_manager_ == nullptr || !map_manager_->ready()) {
            return true;
        }
        if (!map_manager_->insideLocalMap(point)) {
            return false;
        }
        const auto grid_type = map_manager_->getInfGridType(point);
        if (grid_type == rog_map::GridType::OCCUPIED ||
            grid_type == rog_map::GridType::OUT_OF_MAP) {
            return false;
        }
        if (cfg_.tracking_unknown_as_occupied &&
            (grid_type == rog_map::GridType::UNKNOWN ||
             grid_type == rog_map::GridType::UNDEFINED ||
             grid_type == rog_map::GridType::FRONTIER)) {
            return false;
        }
        return true;
    }

    bool GeneralPlanner::densifyTrackingGuideForCorridor(const vec_Vec3f &guide_path,
                                                         const std::vector<double> &guide_t,
                                                         vec_Vec3f &dense_path,
                                                         std::vector<double> &dense_t) const {
        dense_path.clear();
        dense_t.clear();
        if (guide_path.size() < 2) {
            return false;
        }

        double max_step = 0.8 * cfg_.corridor_line_max_length;
        if (!std::isfinite(max_step) || max_step <= 1.0e-3) {
            max_step = map_manager_ != nullptr ? 4.0 * std::max(0.05, map_manager_->getResolution()) : 0.5;
        }
        max_step = std::clamp(max_step, 0.2, std::max(0.2, cfg_.corridor_line_max_length));

        if (!trackingGuidePointSafe(guide_path.front())) {
            return false;
        }
        const bool has_valid_times = guide_t.size() == guide_path.size();
        double fallback_stamp = has_valid_times && std::isfinite(guide_t.front()) ? guide_t.front() : 0.0;
        appendGuideTimedUnique(guide_path.front(), fallback_stamp, dense_path, dense_t);

        for (int i = 1; i < static_cast<int>(guide_path.size()); ++i) {
            const Vec3f start = dense_path.back();
            const Vec3f goal = guide_path[static_cast<std::size_t>(i)];
            if (!trackingGuidePointSafe(goal)) {
                dense_path.clear();
                dense_t.clear();
                return false;
            }

            const double segment_len = (goal - start).norm();
            if (!std::isfinite(segment_len)) {
                dense_path.clear();
                dense_t.clear();
                return false;
            }
            const int segment_num = std::max(1, static_cast<int>(std::ceil(segment_len / max_step)));
            Vec3f last = start;
            const double fallback_start_t = fallback_stamp;
            fallback_stamp += std::max(0.05, segment_len / 2.0);
            for (int seg = 1; seg <= segment_num; ++seg) {
                const double alpha = static_cast<double>(seg) / static_cast<double>(segment_num);
                Vec3f point = start + alpha * (goal - start);
                if (seg == segment_num) {
                    point = goal;
                }
                if (!trackingGuidePointSafe(point)) {
                    dense_path.clear();
                    dense_t.clear();
                    return false;
                }
                if (map_manager_ != nullptr && map_manager_->ready() &&
                    !map_manager_->isLineFree(last, point, true, cfg_.tracking_unknown_as_occupied)) {
                    dense_path.clear();
                    dense_t.clear();
                    return false;
                }
                const double stamp = interpolateSegmentStamp(guide_t,
                                                             i - 1,
                                                             alpha,
                                                             fallback_start_t,
                                                             fallback_stamp);
                appendGuideTimedUnique(point, stamp, dense_path, dense_t);
                last = point;
            }
        }

        return dense_path.size() >= 2 && dense_path.size() == dense_t.size();
    }

    void GeneralPlanner::refreshTrackingGuideTiming(traj_opt::TrackingProblem &problem) const {
        problem.guide_t.clear();
        problem.guide_t.reserve(problem.guide_path.size());
        double stamp = 0.0;
        problem.guide_t.emplace_back(stamp);
        for (int i = 1; i < static_cast<int>(problem.guide_path.size()); ++i) {
            stamp += std::max(0.1, (problem.guide_path[i] - problem.guide_path[i - 1]).norm() / 2.0);
            problem.guide_t.emplace_back(stamp);
        }

        problem.tail_pvaj.col(0) = problem.guide_path.back();
        if (!problem.target_prediction.empty()) {
            Vec3f tail_vel = problem.target_prediction.back().velocity;
            if (problem.static_tracking_mode ||
                !tail_vel.allFinite() ||
                tail_vel.norm() < cfg_.tracking_static_tail_speed_epsilon) {
                tail_vel.setZero();
            }
            problem.tail_pvaj.col(1) = tail_vel;
            problem.min_total_duration = std::max(0.6, problem.target_prediction.back().t);
        } else {
            problem.tail_pvaj.col(1).setZero();
            problem.min_total_duration = std::max(0.6, problem.guide_t.back());
        }
    }

    void GeneralPlanner::refreshTrackingGuideEndpoint(traj_opt::TrackingProblem &problem) const {
        if (problem.guide_path.empty()) {
            return;
        }
        problem.tail_pvaj.col(0) = problem.guide_path.back();
        if (!problem.target_prediction.empty()) {
            Vec3f tail_vel = problem.target_prediction.back().velocity;
            if (problem.static_tracking_mode ||
                !tail_vel.allFinite() ||
                tail_vel.norm() < cfg_.tracking_static_tail_speed_epsilon) {
                tail_vel.setZero();
            }
            problem.tail_pvaj.col(1) = tail_vel;
            problem.min_total_duration = std::max(0.6, problem.target_prediction.back().t);
        } else if (!problem.guide_t.empty()) {
            problem.tail_pvaj.col(1).setZero();
            problem.min_total_duration = std::max(0.6, problem.guide_t.back());
        }
    }

    bool GeneralPlanner::tryGenerateTrackingCorridor(const vec_Vec3f &guide_path,
                                                     PolytopeVec &sfcs,
                                                     std::string *failure_reason) {
        sfcs.clear();
        if (cg_ptr_ == nullptr) {
            setFailureReason(failure_reason, "corridor_generator_null");
            return false;
        }
        if (guide_path.size() < 2) {
            setFailureReason(failure_reason,
                             fmt::format("guide_path_too_short(size={})", guide_path.size()));
            return false;
        }
        for (std::size_t i = 0; i < guide_path.size(); ++i) {
            const auto &point = guide_path[i];
            if (!trackingGuidePointSafe(point)) {
                setFailureReason(failure_reason,
                                 fmt::format("unsafe_guide_point(index={}, p=[{:.3f},{:.3f},{:.3f}])",
                                             i, point.x(), point.y(), point.z()));
                return false;
            }
        }

        Vec3f shifted_start_pt = Vec3f(9999, 9999, 9999);
        bool ok = false;
        try {
            ok = cg_ptr_->SearchPolytopeOnPath(guide_path, sfcs, shifted_start_pt, false);
        } catch (const std::exception &e) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking SFC generation threw exception: {}", e.what());
            setFailureReason(failure_reason, fmt::format("SearchPolytopeOnPath_exception({})", e.what()));
            sfcs.clear();
            return false;
        }
        if (!ok) {
            setFailureReason(failure_reason,
                             fmt::format("SearchPolytopeOnPath_returned_false(guide_size={})",
                                         guide_path.size()));
            sfcs.clear();
            return false;
        }
        if (sfcs.empty()) {
            setFailureReason(failure_reason, "SearchPolytopeOnPath_returned_empty_sfc");
            sfcs.clear();
            return false;
        }

        for (std::size_t i = 0; i < sfcs.size(); ++i) {
            const auto &poly = sfcs[i];
            const auto planes = poly.GetPlanes();
            if (planes.rows() == 0 || !std::isfinite(planes.sum())) {
                setFailureReason(failure_reason,
                                 fmt::format("invalid_sfc_poly(index={}, rows={}, finite={})",
                                             i, planes.rows(), std::isfinite(planes.sum())));
                sfcs.clear();
                return false;
            }
        }

        for (std::size_t i = 0; i < guide_path.size(); ++i) {
            const auto &point = guide_path[i];
            bool covered = false;
            for (const auto &poly: sfcs) {
                if (poly.PointIsInside(point, 0.05)) {
                    covered = true;
                    break;
                }
            }
            if (!covered) {
                setFailureReason(failure_reason,
                                 fmt::format("guide_point_not_covered_by_sfc(index={}, p=[{:.3f},{:.3f},{:.3f}], sfc_count={})",
                                             i, point.x(), point.y(), point.z(), sfcs.size()));
                sfcs.clear();
                return false;
            }
        }
        return true;
    }

    bool GeneralPlanner::repairTrackingGuideWithAstar(const vec_Vec3f &guide_path,
                                                      const std::vector<double> &guide_t,
                                                      vec_Vec3f &repaired_path,
                                                      std::vector<double> &repaired_t,
                                                      std::string *failure_reason) {
        repaired_path.clear();
        repaired_t.clear();
        if (guide_path.size() < 2) {
            setFailureReason(failure_reason,
                             fmt::format("astar_repair_guide_too_short(size={})", guide_path.size()));
            return false;
        }
        if (astar_ptr_ == nullptr) {
            setFailureReason(failure_reason, "astar_repair_astar_null");
            return false;
        }
        if (!trackingGuidePointSafe(guide_path.front())) {
            const auto &p = guide_path.front();
            setFailureReason(failure_reason,
                             fmt::format("astar_repair_start_unsafe(p=[{:.3f},{:.3f},{:.3f}])",
                                         p.x(), p.y(), p.z()));
            return false;
        }

        const bool has_valid_times = guide_t.size() == guide_path.size();
        double fallback_stamp = has_valid_times && std::isfinite(guide_t.front()) ? guide_t.front() : 0.0;
        appendGuideTimedUnique(guide_path.front(), fallback_stamp, repaired_path, repaired_t);
        const int astar_flag = path_search::ON_INF_MAP |
                               (cfg_.tracking_unknown_as_occupied
                                    ? path_search::UNKNOWN_AS_OCCUPIED
                                    : path_search::UNKNOWN_AS_FREE) |
                               path_search::USE_INF_NEIGHBOR;

        bool full_repair = true;
        for (int i = 1; i < static_cast<int>(guide_path.size()); ++i) {
            const Vec3f start = repaired_path.back();
            const Vec3f goal = guide_path[i];
            if (!trackingGuidePointSafe(goal)) {
                setFailureReason(failure_reason,
                                 fmt::format("astar_repair_goal_unsafe(segment={}, goal=[{:.3f},{:.3f},{:.3f}])",
                                             i, goal.x(), goal.y(), goal.z()));
                full_repair = false;
                break;
            }

            if (map_manager_ == nullptr || !map_manager_->ready() ||
                map_manager_->isLineFree(start, goal, true, cfg_.tracking_unknown_as_occupied)) {
                const double stamp = has_valid_times ? guide_t[static_cast<std::size_t>(i)]
                                                     : fallback_stamp + std::max(0.05, (goal - start).norm() / 2.0);
                appendGuideTimedUnique(goal, stamp, repaired_path, repaired_t);
                fallback_stamp = stamp;
                continue;
            }

            vec_Vec3f astar_path;
            const RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(start,
                                                                         goal,
                                                                         astar_flag,
                                                                         cfg_.planning_horizon,
                                                                         astar_path,
                                                                         0.08);
            if ((ret_code != SUCCESS && ret_code != REACH_GOAL) || astar_path.empty()) {
                setFailureReason(failure_reason,
                                 fmt::format("astar_repair_search_failed(segment={}, ret={}, path_size={}, start=[{:.3f},{:.3f},{:.3f}], goal=[{:.3f},{:.3f},{:.3f}])",
                                             i, ret_code, astar_path.size(),
                                             start.x(), start.y(), start.z(),
                                             goal.x(), goal.y(), goal.z()));
                full_repair = false;
                break;
            }

            Vec3f last = start;
            for (std::size_t path_id = 0; path_id < astar_path.size(); ++path_id) {
                const auto &point = astar_path[path_id];
                if (!trackingGuidePointSafe(point)) {
                    setFailureReason(failure_reason,
                                     fmt::format("astar_repair_path_point_unsafe(segment={}, path_index={}, p=[{:.3f},{:.3f},{:.3f}])",
                                                 i, path_id, point.x(), point.y(), point.z()));
                    full_repair = false;
                    break;
                }
                if (map_manager_ != nullptr && map_manager_->ready() &&
                    !map_manager_->isLineFree(last, point, true, cfg_.tracking_unknown_as_occupied)) {
                    setFailureReason(failure_reason,
                                     fmt::format("astar_repair_path_segment_blocked(segment={}, path_index={}, from=[{:.3f},{:.3f},{:.3f}], to=[{:.3f},{:.3f},{:.3f}])",
                                                 i, path_id,
                                                 last.x(), last.y(), last.z(),
                                                 point.x(), point.y(), point.z()));
                    full_repair = false;
                    break;
                }
                last = point;
            }
            if (!full_repair) {
                break;
            }
            if (map_manager_ != nullptr && map_manager_->ready() &&
                !map_manager_->isLineFree(repaired_path.back(), goal, true, cfg_.tracking_unknown_as_occupied)) {
                setFailureReason(failure_reason,
                                 fmt::format("astar_repair_final_segment_blocked(segment={}, from=[{:.3f},{:.3f},{:.3f}], goal=[{:.3f},{:.3f},{:.3f}])",
                                             i,
                                             repaired_path.back().x(), repaired_path.back().y(), repaired_path.back().z(),
                                             goal.x(), goal.y(), goal.z()));
                full_repair = false;
                break;
            }

            vec_Vec3f segment_path;
            appendGuideUnique(start, segment_path);
            for (const auto &point: astar_path) {
                appendGuideUnique(point, segment_path);
            }
            appendGuideUnique(goal, segment_path);

            std::vector<double> segment_accum(segment_path.size(), 0.0);
            for (int sid = 1; sid < static_cast<int>(segment_path.size()); ++sid) {
                segment_accum[static_cast<std::size_t>(sid)] =
                        segment_accum[static_cast<std::size_t>(sid - 1)] +
                        (segment_path[static_cast<std::size_t>(sid)] -
                         segment_path[static_cast<std::size_t>(sid - 1)]).norm();
            }
            const double start_t = repaired_t.empty() ? fallback_stamp : repaired_t.back();
            const double goal_t = has_valid_times
                                      ? std::max(start_t, guide_t[static_cast<std::size_t>(i)])
                                      : start_t + std::max(0.05, segment_accum.back() / 2.0);
            for (int sid = 1; sid < static_cast<int>(segment_path.size()); ++sid) {
                const double ratio = segment_accum.back() > 1.0e-6
                                         ? segment_accum[static_cast<std::size_t>(sid)] / segment_accum.back()
                                         : 1.0;
                appendGuideTimedUnique(segment_path[static_cast<std::size_t>(sid)],
                                       start_t + ratio * (goal_t - start_t),
                                       repaired_path,
                                       repaired_t);
            }
            fallback_stamp = goal_t;
        }

        if (!full_repair) {
            if (failure_reason != nullptr && failure_reason->empty()) {
                *failure_reason = "astar_repair_failed";
            }
            return false;
        }
        if (repaired_path.size() < 2 || repaired_path.size() != repaired_t.size()) {
            setFailureReason(failure_reason,
                             fmt::format("astar_repair_invalid_output(path_size={}, time_size={})",
                                         repaired_path.size(), repaired_t.size()));
            return false;
        }
        return true;
    }

    bool GeneralPlanner::truncateTrackingProblemForCorridor(traj_opt::TrackingProblem &problem,
                                                            const vec_Vec3f &candidate_guide,
                                                            const std::vector<double> &candidate_guide_t,
                                                            PolytopeVec &sfcs,
                                                            std::string *failure_reason) {
        if (candidate_guide.size() < 2) {
            setFailureReason(failure_reason,
                             fmt::format("truncate_candidate_too_short(size={})", candidate_guide.size()));
            return false;
        }
        if (candidate_guide.size() != candidate_guide_t.size()) {
            setFailureReason(failure_reason,
                             fmt::format("truncate_candidate_time_size_mismatch(path_size={}, time_size={})",
                                         candidate_guide.size(), candidate_guide_t.size()));
            return false;
        }
        if (problem.viewpoints.empty()) {
            setFailureReason(failure_reason, "truncate_no_viewpoints");
            return false;
        }

        const double match_tol = std::max(1.0e-3, 0.25 * cfg_.resolution);
        std::string last_prefix_reason;
        for (int view_id = static_cast<int>(problem.viewpoints.size()) - 1; view_id >= 0; --view_id) {
            const Vec3f &viewpoint = problem.viewpoints[static_cast<std::size_t>(view_id)];
            int end_id = -1;
            for (int guide_id = static_cast<int>(candidate_guide.size()) - 1; guide_id >= 1; --guide_id) {
                if ((candidate_guide[static_cast<std::size_t>(guide_id)] - viewpoint).norm() <= match_tol) {
                    end_id = guide_id;
                    break;
                }
            }
            if (end_id < 1) {
                last_prefix_reason = fmt::format("viewpoint_not_found_in_candidate(view_id={}, viewpoint=[{:.3f},{:.3f},{:.3f}])",
                                                 view_id, viewpoint.x(), viewpoint.y(), viewpoint.z());
                continue;
            }
            if (!trackingGuidePointSafe(candidate_guide[static_cast<std::size_t>(end_id)])) {
                const auto &p = candidate_guide[static_cast<std::size_t>(end_id)];
                last_prefix_reason = fmt::format("truncate_endpoint_unsafe(view_id={}, end_id={}, p=[{:.3f},{:.3f},{:.3f}])",
                                                 view_id, end_id, p.x(), p.y(), p.z());
                continue;
            }

            vec_Vec3f prefix(candidate_guide.begin(), candidate_guide.begin() + end_id + 1);
            std::vector<double> prefix_t(candidate_guide_t.begin(), candidate_guide_t.begin() + end_id + 1);
            std::string prefix_reason;
            if (!tryGenerateTrackingCorridor(prefix, sfcs, &prefix_reason)) {
                last_prefix_reason = fmt::format("truncate_prefix_sfc_failed(view_id={}, end_id={}, reason={})",
                                                 view_id, end_id, prefix_reason);
                continue;
            }

            problem.guide_path = std::move(prefix);
            problem.guide_t = std::move(prefix_t);
            const std::size_t keep_count = static_cast<std::size_t>(view_id + 1);
            problem.viewpoints.resize(keep_count);
            problem.target_sample_times.resize(keep_count);
            problem.target_prediction.resize(keep_count);
            if (problem.visible_regions.size() > keep_count) {
                problem.visible_regions.resize(keep_count);
            }
            refreshTrackingGuideEndpoint(problem);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking guide truncated to safe local SFC endpoint, kept {} target samples.",
                               keep_count);
            }
            return true;
        }
        setFailureReason(failure_reason,
                         last_prefix_reason.empty()
                             ? "truncate_no_safe_prefix_found"
                             : last_prefix_reason);
        return false;
    }

    bool GeneralPlanner::buildTrackingGuideCorridor(traj_opt::TrackingProblem &problem,
                                                    std::string *failure_reason) {
        problem.sfcs.clear();
        problem.use_corridor = false;

        if (cg_ptr_ != nullptr && !problem.guide_path.empty()) {
            double guide_length = 0.0;
            for (int i = 1; i < static_cast<int>(problem.guide_path.size()); ++i) {
                guide_length += (problem.guide_path[static_cast<std::size_t>(i)] -
                                 problem.guide_path[static_cast<std::size_t>(i - 1)]).norm();
            }
            if (guide_length < 1.0e-4) {
                const Vec3f hover_point = problem.guide_path.front();
                if (!trackingGuidePointSafe(hover_point)) {
                    setFailureReason(failure_reason,
                                     fmt::format("hover_guide_point_unsafe(p=[{:.3f},{:.3f},{:.3f}])",
                                                 hover_point.x(), hover_point.y(), hover_point.z()));
                    return false;
                }
                Polytope hover_sfc;
                if (!cg_ptr_->GeneratePolytopeFromPoint(hover_point, hover_sfc)) {
                    setFailureReason(failure_reason,
                                     fmt::format("hover_GeneratePolytopeFromPoint_failed(p=[{:.3f},{:.3f},{:.3f}])",
                                                 hover_point.x(), hover_point.y(), hover_point.z()));
                    return false;
                }
                problem.guide_path.clear();
                problem.guide_path.emplace_back(hover_point);
                if (problem.guide_t.empty()) {
                    const double hover_t = !problem.target_prediction.empty()
                                               ? problem.target_prediction.back().t
                                               : std::max(0.6, problem.min_total_duration);
                    problem.guide_t.emplace_back(std::max(0.6, hover_t));
                } else {
                    problem.guide_t.resize(1);
                    problem.guide_t.front() = std::max(0.6, problem.guide_t.front());
                }
                problem.sfcs.emplace_back(hover_sfc);
                problem.use_corridor = true;
                refreshTrackingGuideEndpoint(problem);
                return true;
            }
        }

        PolytopeVec sfcs;
        vec_Vec3f dense_guide;
        std::vector<double> dense_guide_t;
        std::string dense_reason;
        const bool dense_ok =
                densifyTrackingGuideForCorridor(problem.guide_path, problem.guide_t, dense_guide, dense_guide_t);
        if (!dense_ok) {
            dense_reason = fmt::format("densify_original_guide_failed(guide_size={}, guide_t_size={})",
                                       problem.guide_path.size(), problem.guide_t.size());
        } else if (!tryGenerateTrackingCorridor(dense_guide, sfcs, &dense_reason)) {
            dense_reason = "original_dense_sfc_failed:" + dense_reason;
        } else {
            problem.guide_path = std::move(dense_guide);
            problem.guide_t = std::move(dense_guide_t);
            refreshTrackingGuideEndpoint(problem);
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            return true;
        }

        vec_Vec3f astar_repaired;
        std::vector<double> astar_repaired_t;
        std::string astar_reason;
        const bool full_astar_repair =
                repairTrackingGuideWithAstar(problem.guide_path,
                                             problem.guide_t,
                                             astar_repaired,
                                             astar_repaired_t,
                                             &astar_reason);
        vec_Vec3f dense_astar_repaired;
        std::vector<double> dense_astar_repaired_t;
        std::string dense_astar_reason;
        if (full_astar_repair) {
            const bool dense_astar_ok =
                    densifyTrackingGuideForCorridor(astar_repaired,
                                                    astar_repaired_t,
                                                    dense_astar_repaired,
                                                    dense_astar_repaired_t);
            if (!dense_astar_ok) {
                dense_astar_reason = fmt::format("densify_astar_repair_failed(path_size={}, time_size={})",
                                                 astar_repaired.size(), astar_repaired_t.size());
            } else if (!tryGenerateTrackingCorridor(dense_astar_repaired, sfcs, &dense_astar_reason)) {
                dense_astar_reason = "astar_dense_sfc_failed:" + dense_astar_reason;
            } else {
            problem.guide_path = std::move(dense_astar_repaired);
            problem.guide_t = std::move(dense_astar_repaired_t);
            refreshTrackingGuideEndpoint(problem);
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking SFC built after A* guide repair.");
            }
            return true;
            }
        }

        std::string truncate_astar_reason;
        if (!dense_astar_repaired.empty() &&
            truncateTrackingProblemForCorridor(problem,
                                               dense_astar_repaired,
                                               dense_astar_repaired_t,
                                               sfcs,
                                               &truncate_astar_reason)) {
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            return true;
        }
        std::string truncate_dense_reason;
        if (!dense_guide.empty() &&
            truncateTrackingProblemForCorridor(problem,
                                               dense_guide,
                                               dense_guide_t,
                                               sfcs,
                                               &truncate_dense_reason)) {
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            return true;
        }

        problem.sfcs.clear();
        problem.use_corridor = false;
        setFailureReason(failure_reason,
                         fmt::format("guide_size={}, target_samples={}, dense={}, astar={}, dense_astar={}, truncate_astar={}, truncate_dense={}",
                                     problem.guide_path.size(),
                                     problem.target_prediction.size(),
                                     dense_reason.empty() ? "ok-but-unused" : dense_reason,
                                     full_astar_repair ? "ok" : astar_reason,
                                     dense_astar_reason.empty() ? "not_attempted_or_ok" : dense_astar_reason,
                                     truncate_astar_reason.empty() ? "not_attempted" : truncate_astar_reason,
                                     truncate_dense_reason.empty() ? "not_attempted" : truncate_dense_reason));
        return false;
    }

    RET_CODE GeneralPlanner::optimizeTrackingTask(const traj_opt::DynamicTargetStates &target_prediction,
                                                const bool &from_rest) {
        if (target_prediction.empty()) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking task has no target prediction.");
            return FAILED;
        }

        auto failOrKeepOld = [this, &target_prediction](const std::string &reason) -> RET_CODE {
            if (keepOldTrackingTrajectoryIfActive(target_prediction, reason)) {
                return NO_NEED;
            }
            if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                tracking_runtime_manager_->onRejected();
            }
            ros_ptr_->warn(" -- [GeneralPlanner] {}", reason);
            return FAILED;
        };

        const bool static_tracking =
                staticTargetPrediction(target_prediction,
                                       0.05,
                                       cfg_.tracking_static_tail_speed_epsilon);
        const double static_distance_tolerance =
                std::max(0.05,
                         cfg_.tracking_distance_tolerance *
                         std::clamp(cfg_.tracking_static_distance_tolerance_scale, 0.05, 1.0));
        const double static_height_tolerance =
                std::max(0.05,
                         cfg_.tracking_height_tolerance *
                         std::clamp(cfg_.tracking_static_height_tolerance_scale, 0.05, 1.0));

        TrackingFrontend::Config frontend_cfg;
        frontend_cfg.tracking_distance = cfg_.tracking_distance;
        frontend_cfg.distance_tolerance = static_tracking ? static_distance_tolerance
                                                          : cfg_.tracking_distance_tolerance;
        frontend_cfg.height_offset = cfg_.tracking_height_offset;
        frontend_cfg.height_tolerance = static_tracking ? static_height_tolerance
                                                        : cfg_.tracking_height_tolerance;
        frontend_cfg.safe_distance = cfg_.tracking_safe_distance;
        frontend_cfg.visibility_safe_distance = cfg_.tracking_visibility_safe_distance;
        frontend_cfg.visibility_cone_ratio = cfg_.tracking_visibility_cone_ratio;
        frontend_cfg.visibility_angle_clearance = cfg_.tracking_visibility_angle_clearance;
        frontend_cfg.reacquire_distance = cfg_.tracking_reacquire_distance;
        frontend_cfg.searching_horizon = cfg_.planning_horizon;
        frontend_cfg.low_speed_velocity_threshold = cfg_.tracking_low_speed_velocity_threshold;
        frontend_cfg.angular_hysteresis = cfg_.tracking_angular_hysteresis;
        frontend_cfg.candidate_angle_step = cfg_.tracking_candidate_angle_step;
        frontend_cfg.candidate_radius_num = cfg_.tracking_candidate_radius_num;
        frontend_cfg.visibility_samples = cfg_.tracking_visibility_samples;
        frontend_cfg.fallback_relax_enable = cfg_.tracking_fallback_relax_enable;
        frontend_cfg.fallback_distance_tolerance_scale = cfg_.tracking_fallback_distance_tolerance_scale;
        frontend_cfg.fallback_height_tolerance_scale = cfg_.tracking_fallback_height_tolerance_scale;
        frontend_cfg.fallback_candidate_radius_extra = cfg_.tracking_fallback_candidate_radius_extra;
        frontend_cfg.fallback_candidate_angle_step_scale = cfg_.tracking_fallback_candidate_angle_step_scale;
        frontend_cfg.fallback_search_horizon_scale = cfg_.tracking_fallback_search_horizon_scale;
        frontend_cfg.elastic_guide_enable = cfg_.tracking_frontend_elastic_enable;
        frontend_cfg.elastic_distance_tolerance_scale = cfg_.tracking_frontend_elastic_distance_tolerance_scale;
        frontend_cfg.elastic_height_tolerance_scale = cfg_.tracking_frontend_elastic_height_tolerance_scale;
        frontend_cfg.partial_guide_enable = cfg_.tracking_frontend_partial_guide_enable;
        frontend_cfg.partial_guide_min_duration = cfg_.tracking_frontend_partial_min_duration;
        frontend_cfg.partial_guide_min_samples = cfg_.tracking_frontend_partial_min_samples;
        frontend_cfg.unknown_as_occupied = cfg_.tracking_unknown_as_occupied;
        frontend_cfg.use_astar = cfg_.tracking_frontend_astar;
        frontend_cfg.use_visible_region = cfg_.tracking_use_visible_region;
        frontend_cfg.print_log = cfg_.print_log;

        traj_opt::TrackingProblem problem;
        TimeConsuming t_frontend("tracking_frontend", false);
        TrackingFrontend frontend(frontend_cfg, map_manager_, astar_ptr_);
        if (!frontend.buildProblem(makeTaskHeadState(from_rest), target_prediction, problem)) {
            time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
            return failOrKeepOld("Tracking frontend failed.");
        }
        problem.static_tracking_mode = static_tracking;
        const double tracking_frontend_t = t_frontend.stop();
        time_consuming_[EPX_TRAJ_FRONTEND] = tracking_frontend_t;

        TimeConsuming t_tracking_sfc("tracking_sfc", false);
        if (!buildTrackingGuideCorridor(problem)) {
            time_consuming_[EPX_TRAJ_FRONTEND] += t_tracking_sfc.stop();
            return failOrKeepOld("Tracking SFC generation failed.");
        }
        time_consuming_[EPX_TRAJ_FRONTEND] += t_tracking_sfc.stop();
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj, problem.tail_pvaj, problem.sfcs);
        const traj_opt::DynamicTargetStates &active_target_prediction =
                problem.target_prediction.empty() ? target_prediction : problem.target_prediction;

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
        double terminal_yaw = active_target_prediction.back().yaw;
        const Vec3f face_dir = active_target_prediction.back().position - problem.tail_pvaj.col(0);
        if (face_dir.head<2>().norm() > 1.0e-3) {
            terminal_yaw = std::atan2(face_dir.y(), face_dir.x());
            geometry_utils::normalizeNextYaw(problem.head_yaw(0, 0), terminal_yaw);
        }
        problem.tail_yaw << terminal_yaw, static_tracking ? 0.0 : active_target_prediction.back().yaw_rate;
        problem.weight_od_near = cfg_.tracking_weight_od_near;
        problem.weight_od_far = cfg_.tracking_weight_od_far;
        problem.weight_od_vertical = cfg_.tracking_weight_od_vertical;
        problem.weight_oa = cfg_.tracking_weight_oa;
        problem.weight_oe = cfg_.tracking_weight_oe;
        problem.weight_visibility = cfg_.tracking_weight_oe;
        problem.weight_relative_velocity = cfg_.tracking_weight_relative_velocity;
        problem.weight_tangent_velocity = cfg_.tracking_weight_tangent_velocity;
        problem.weight_viewpoint_attractor = cfg_.tracking_weight_viewpoint_attractor;
        problem.weight_visible_region = cfg_.tracking_weight_visible_region;
        if (problem.reacquire_mode) {
            problem.weight_visible_region *= 0.25;
        }
        if (static_tracking) {
            problem.distance_tolerance = static_distance_tolerance;
            problem.height_tolerance = static_height_tolerance;
            problem.od_h_lower = std::max(0.05, cfg_.tracking_distance - static_distance_tolerance);
            problem.od_h_upper = std::max(problem.od_h_lower + 0.05,
                                          cfg_.tracking_distance + static_distance_tolerance);
            problem.od_v_lower = cfg_.tracking_height_offset - static_height_tolerance;
            problem.od_v_upper = cfg_.tracking_height_offset + static_height_tolerance;
            problem.tail_pvaj.col(1).setZero();
            problem.tail_pvaj.col(2).setZero();
            problem.tail_pvaj.col(3).setZero();
            for (auto &state : problem.target_prediction) {
                state.velocity.setZero();
                state.acceleration.setZero();
                state.yaw_rate = 0.0;
            }
            problem.weight_tangent_velocity *=
                    std::max(1.0, cfg_.tracking_static_tangent_weight_scale);
        }
        const int valid_visible_regions = validVisibleRegionCount(problem);
        problem.use_visible_region = cfg_.tracking_use_visible_region && valid_visible_regions > 0;
        problem.visibility_angle_clearance = cfg_.tracking_visibility_angle_clearance;
        if (cfg_.print_log && cfg_.tracking_use_visible_region) {
            ros_ptr_->info(" -- [GeneralPlanner] Tracking visible-region soft prior: valid={}/{}, enabled={}, reacquire={}.",
                           valid_visible_regions,
                           problem.visible_regions.size(),
                           problem.use_visible_region,
                           problem.reacquire_mode);
        }
        if (static_tracking && cfg_.print_log) {
            ros_ptr_->info(" -- [GeneralPlanner] Static tracking mode: OD_h=[{:.2f},{:.2f}], OD_v=[{:.2f},{:.2f}], tangent_weight={:.2f}.",
                           problem.od_h_lower,
                           problem.od_h_upper,
                           problem.od_v_lower,
                           problem.od_v_upper,
                           problem.weight_tangent_velocity);
        }

        {
            TimeConsuming t_viz("tracking_frontend_viz", false);
            ros_ptr_->vizFrontendPath(problem.guide_path);
            ros_ptr_->vizExpSfc(problem.sfcs);
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
            return failOrKeepOld("Tracking optimization failed.");
        }
        if (out_traj.getTotalDuration() < cfg_.tracking_min_commit_duration) {
            return failOrKeepOld(fmt::format("Tracking trajectory too short ({:.3f}s < {:.3f}s).",
                                             out_traj.getTotalDuration(),
                                             cfg_.tracking_min_commit_duration));
        }

        const double candidate_guard_h =
                std::min(cfg_.tracking_no_motion_check_horizon,
                         out_traj.getTotalDuration());
        const double candidate_disp =
                candidate_guard_h > 1.0e-6
                    ? (out_traj.getPos(candidate_guard_h) -
                       out_traj.getPos(0.0)).head<2>().norm()
                    : 0.0;
        const double candidate_speed0 = out_traj.getVel(0.0).head<2>().norm();
        double old_remaining = 0.0;
        double old_speed0 = 0.0;
        double old_displacement = 0.0;
        double old_progress = 0.0;
        double old_expected_progress = 0.0;
        double old_avg_tracking_error = 0.0;
        if (cfg_.tracking_runtime_manager_enable &&
            tracking_runtime_manager_ &&
            tracking_runtime_manager_->hasCommittedTracking() &&
            !cmd_traj_info_.empty()) {
            Trajectory old_pos_traj;
            double old_start_wt = 0.0;
            double old_total_dur = 0.0;
            cmd_traj_info_.lock();
            old_pos_traj = cmd_traj_info_.posTraj();
            old_start_wt = cmd_traj_info_.getStartWallTime();
            old_total_dur = cmd_traj_info_.getTotalDuration();
            cmd_traj_info_.unlock();
            const auto old_activity =
                    tracking_runtime_manager_->evaluateActivity(
                            old_pos_traj,
                            std::clamp(ros_ptr_->getSimTime() - old_start_wt,
                                       0.0,
                                       old_total_dur),
                            active_target_prediction,
                            cfg_.tracking_keep_old_horizon,
                            cfg_.tracking_keep_old_safety_dt);
            old_remaining = old_activity.remaining;
            old_speed0 = old_activity.speed0;
            old_displacement = old_activity.displacement;
            old_progress = old_activity.progress;
            old_expected_progress = old_activity.expected_progress;
            old_avg_tracking_error = old_activity.avg_tracking_error;
        } else if (!cfg_.tracking_runtime_manager_enable) {
            TrackingTrajectoryActivity old_activity;
            currentTrackingTrajectorySafeAndActive(active_target_prediction, &old_activity);
            old_remaining = old_activity.remaining;
            old_speed0 = old_activity.speed0;
            old_displacement = old_activity.displacement;
            old_progress = old_activity.progress;
            old_expected_progress = old_activity.expected_progress;
            old_avg_tracking_error = old_activity.avg_tracking_error;
        }
        if (cfg_.print_log) {
            ros_ptr_->info(" -- [Tracking] TRACKING_CANDIDATE_OPT_SUCCESS candidate_duration={:.3f}, candidate_disp_0p35s={:.3f}, candidate_speed0={:.3f}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                           out_traj.getTotalDuration(),
                           candidate_disp,
                           candidate_speed0,
                           old_remaining,
                           old_speed0,
                           old_displacement,
                           old_progress,
                           old_expected_progress,
                           old_avg_tracking_error);
        }

        std::string commandable_reject_reason;
        if (!cfg_.tracking_runtime_manager_enable &&
            !candidateTrackingTrajectoryCommandable(out_traj,
                                                    active_target_prediction,
                                                    &commandable_reject_reason)) {
            TrackingTrajectoryActivity old_activity;
            currentTrackingTrajectorySafeAndActive(active_target_prediction, &old_activity);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_NO_MOTION reject_reason={}, candidate_duration={:.3f}, candidate_disp_0p35s={:.3f}, candidate_speed0={:.3f}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               commandable_reject_reason,
                               out_traj.getTotalDuration(),
                               candidate_disp,
                               candidate_speed0,
                               old_activity.remaining,
                               old_activity.speed0,
                               old_activity.displacement,
                               old_activity.progress,
                               old_activity.expected_progress,
                               old_activity.avg_tracking_error);
            }

            if (old_activity.active &&
                keepOldTrackingTrajectoryIfActive(active_target_prediction,
                                                  "tracking candidate rejected by no-motion guard")) {
                return NO_NEED;
            }

            if (old_activity.valid &&
                !old_activity.safe &&
                trackingCandidateSafeForCommit(out_traj)) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] no-motion guard allows safe candidate because old trajectory is unsafe. old_reason={}",
                                   old_activity.reason);
                }
            } else {
                return FAILED;
            }
        }

        {
            TimeConsuming t_viz("tracking_fov_viz", false);
            const double fov_range = cfg_.tracking_fov_range > 0.0
                                         ? cfg_.tracking_fov_range
                                         : cfg_.tracking_distance + cfg_.tracking_distance_tolerance;
            ros_ptr_->vizTrackingFov(out_traj,
                                     out_yaw_traj,
                                     cfg_.tracking_fov_horizontal_deg,
                                     cfg_.tracking_fov_vertical_deg,
                                     fov_range);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        if (!commitTrackingTrajectory(out_traj,
                                      out_yaw_traj,
                                      active_target_prediction,
                                      cfg_.tracking_use_snap ? "tracking_snap" : "tracking_jerk")) {
            if (keepOldTrackingTrajectoryIfActive(active_target_prediction,
                                                  "tracking trajectory commit rejected")) {
                return NO_NEED;
            }
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking trajectory commit rejected.");
            return FAILED;
        }
        ros_ptr_->info(" -- [GeneralPlanner] Tracking task success: pieces={}, duration={}.",
                       out_traj.getPieceNum(), out_traj.getTotalDuration());
        return SUCCESS;
    }

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

    TakeoffFrontend::Config GeneralPlanner::makeTakeoffFrontendConfig() const {
        TakeoffFrontend::Config frontend_cfg;
        frontend_cfg.robot_l = cfg_.takeoff_robot_l;
        frontend_cfg.robot_radius = cfg_.takeoff_robot_radius;
        frontend_cfg.platform_radius = cfg_.takeoff_platform_radius;
        frontend_cfg.platform_clearance = cfg_.takeoff_platform_clearance;
        frontend_cfg.platform_clearance_after_release =
                cfg_.takeoff_platform_clearance_after_release;
        frontend_cfg.release_contact_time = cfg_.takeoff_release_contact_time;
        frontend_cfg.escape_distance = cfg_.takeoff_escape_distance;
        frontend_cfg.escape_height = cfg_.takeoff_escape_height;
        frontend_cfg.reference_speed = cfg_.takeoff_reference_speed;
        frontend_cfg.min_duration = cfg_.takeoff_min_duration;
        frontend_cfg.max_duration = cfg_.takeoff_max_duration;
        frontend_cfg.piece_num = cfg_.takeoff_piece_num;
        frontend_cfg.frontend_astar = cfg_.takeoff_frontend_astar;
        frontend_cfg.safe_distance = cfg_.takeoff_safe_distance;
        frontend_cfg.use_tangent_release_velocity =
                cfg_.takeoff_use_tangent_release_velocity;
        frontend_cfg.thrust_nominal = cfg_.perching_thrust_nominal;
        frontend_cfg.thrust_range = cfg_.perching_thrust_range;
        frontend_cfg.gravity = cfg_.esdf_traj_cfg.grav;
        frontend_cfg.weight_eta = cfg_.takeoff_weight_eta;
        frontend_cfg.weight_tau_f = cfg_.takeoff_weight_tau_f;
        frontend_cfg.rotate_surface_with_yaw_rate =
                cfg_.perching_rotate_surface_with_yaw_rate;
        return frontend_cfg;
    }

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

    RET_CODE GeneralPlanner::optimizeDynamicTakeoffTask(
            const traj_opt::PerchingSurfaceState &surface,
            const bool &from_rest) {
        (void)from_rest;
        if (takeoff_frontend_ == nullptr || takeoff_optimizer_ == nullptr) {
            ros_ptr_->warn(" -- [Takeoff] TAKEOFF_CANDIDATE_REJECTED reason=module_not_initialized");
            return FAILED;
        }

        traj_opt::DynamicTakeoffProblem problem;
        TimeConsuming t_frontend("takeoff_frontend", false);
        if (!takeoff_frontend_->buildProblem(surface, problem)) {
            time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
            ros_ptr_->warn(" -- [Takeoff] TAKEOFF_CANDIDATE_REJECTED reason=frontend_failed");
            return FAILED;
        }
        time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path,
                                      problem.nominal_head_pvaj,
                                      problem.tail_pvaj,
                                      PolytopeVec());

        {
            TimeConsuming t_viz("takeoff_frontend_viz", false);
            ros_ptr_->vizFrontendPath(problem.guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        Trajectory out_traj;
        TimeConsuming t_opt("takeoff_opt", false);
        const bool ok = takeoff_optimizer_->optimize(problem, out_traj);
        time_consuming_[EXP_TRAJ_OPT] = t_opt.stop();
        if (!ok || out_traj.empty()) {
            ros_ptr_->warn(" -- [Takeoff] TAKEOFF_OPT_FAILED");
            return FAILED;
        }

        const auto candidate_check =
                takeoff_runtime_manager_
                    ? takeoff_runtime_manager_->checkCandidate(out_traj, problem)
                    : TakeoffRuntimeManager::CheckResult{};
        const bool accepted =
                takeoff_runtime_manager_
                    ? takeoff_runtime_manager_->decideCommit(candidate_check)
                    : true;
        ros_ptr_->info(" -- [Takeoff] candidate_check valid={}, safe={}, dynamics_feasible={}, platform_clear_after_release={}, terminal_escape_valid={}, max_thrust={:.3f}, max_omega={:.3f}, esdf_min={:.3f}, platform_margin_min={:.3f}, reason={}",
                       candidate_check.valid,
                       candidate_check.safe,
                       candidate_check.dynamics_feasible,
                       candidate_check.platform_clear_after_release,
                       candidate_check.terminal_escape_valid,
                       candidate_check.max_thrust,
                       candidate_check.max_omega,
                       candidate_check.min_esdf_clearance,
                       candidate_check.min_platform_margin_after_release,
                       candidate_check.reason);
        if (!accepted) {
            ros_ptr_->warn(" -- [Takeoff] TAKEOFF_CANDIDATE_REJECTED reason={}",
                           candidate_check.reason);
            return FAILED;
        }

        if (!commitTakeoffTrajectory(out_traj, "dynamic_takeoff")) {
            return FAILED;
        }
        active_takeoff_problem_ = problem;
        active_takeoff_problem_valid_ = true;
        if (takeoff_runtime_manager_) {
            takeoff_runtime_manager_->updateStatusByPosition(out_traj.getPos(0.0), problem);
        }
        ros_ptr_->info(" -- [GeneralPlanner] Dynamic takeoff task success: pieces={}, duration={}.",
                       out_traj.getPieceNum(), out_traj.getTotalDuration());
        return SUCCESS;
    }

    RET_CODE GeneralPlanner::tryCommitPerchingFromTracking(
            const traj_opt::DynamicTargetStates &target_prediction,
            const traj_opt::PerchingSurfaceState &surface,
            const RET_CODE tracking_ret) {
        if (!cfg_.tracking_perching_enable || !tracking_perching_manager_) {
            return tracking_ret;
        }
        if (tracking_ret != SUCCESS && tracking_ret != NO_NEED) {
            return tracking_ret;
        }
        if (cfg_.tracking_perching_require_external_request &&
            !tracking_perching_manager_->perchingRequested()) {
            return tracking_ret;
        }

        Trajectory tracking_pos;
        Trajectory tracking_yaw;
        double tracking_start_wt = 0.0;
        double tracking_total_t = 0.0;
        if (cmd_traj_info_.empty()) {
            ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_KEEP_TRACKING reason=no_committed_tracking");
            return tracking_ret;
        }
        cmd_traj_info_.lock();
        tracking_pos = cmd_traj_info_.posTraj();
        tracking_yaw = cmd_traj_info_.yawTraj();
        tracking_start_wt = cmd_traj_info_.getStartWallTime();
        tracking_total_t = cmd_traj_info_.getTotalDuration();
        cmd_traj_info_.unlock();
        if (tracking_pos.empty()) {
            ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_KEEP_TRACKING reason=empty_committed_tracking");
            return tracking_ret;
        }

        const double now = ros_ptr_->getSimTime();
        const double tracking_local_t =
                std::clamp(now - tracking_start_wt, 0.0, tracking_total_t);
        const bool tracking_active =
                currentTrackingTrajectorySafeAndActive(target_prediction, nullptr);
        const auto readiness =
                tracking_perching_manager_->evaluateReadiness(tracking_active,
                                                              tracking_pos,
                                                              tracking_yaw,
                                                              tracking_local_t,
                                                              surface,
                                                              cfg_);
        if (!readiness.ready) {
            ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_KEEP_TRACKING reason={}, ready_count={}, distance={:.3f}, relative_speed={:.3f}, lateral_speed={:.3f}, estimated_duration={:.3f}",
                           readiness.reason,
                           readiness.ready_count,
                           readiness.distance,
                           readiness.relative_speed,
                           readiness.lateral_speed,
                           readiness.estimated_duration);
            return tracking_ret;
        }

        if (!tracking_to_perching_initializer_) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_FAILED reason=missing_initializer");
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        TrackingToPerchingInitialGuess init_guess;
        if (!tracking_to_perching_initializer_->build(tracking_pos,
                                                      tracking_yaw,
                                                      tracking_local_t,
                                                      surface,
                                                      cfg_,
                                                      init_guess)) {
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        tracking_perching_manager_->onCandidateTesting();

        const PerchingFrontend::Config frontend_cfg = makePerchingFrontendConfig();
        PerchingFrontend frontend(frontend_cfg, map_manager_, astar_ptr_);
        traj_opt::PerchingProblem problem;
        problem.use_tracking_warm_start = true;
        problem.init_total_time = init_guess.total_time;
        problem.init_nu = init_guess.nu_seed;
        problem.init_tau_f = init_guess.tau_f_seed;
        problem.warm_start_guide_path = init_guess.guide_path;
        problem.warm_start_guide_t = init_guess.guide_t;
        problem.warm_start_head_yaw = init_guess.head_yaw;

        if (!frontend.buildProblem(init_guess.head_pvaj,
                                   init_guess.rebased_surface,
                                   problem)) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_TO_PERCHING_INIT_FAILED reason=frontend_failed");
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }
        latest_replan.setGuidePath(problem.guide_path);
        latest_replan.setExpCondition(VecDf(), problem.guide_path, problem.head_pvaj,
                                      problem.nominal_tail_pvaj, PolytopeVec());

        Trajectory perching_pos;
        TimeConsuming t_opt("tracking_to_perching_opt", false);
        const bool opt_ok = traj_manager_->perchingSnap()->optimize(problem, perching_pos);
        const double opt_t = t_opt.stop();
        time_consuming_[EXP_TRAJ_OPT] += opt_t;
        if (!opt_ok || perching_pos.empty()) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason=optimization_failed");
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }
        ros_ptr_->info(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_OPT_SUCCESS duration={:.3f}, opt_time={:.4f}",
                       perching_pos.getTotalDuration(),
                       opt_t);

        Trajectory perching_yaw;
        if (!buildPerchingYawTrajectoryFromHead(perching_pos,
                                                problem.surface,
                                                init_guess.head_yaw,
                                                perching_yaw)) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason=yaw_generation_failed");
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        const auto candidate_check =
                perching_runtime_manager_
                    ? perching_runtime_manager_->checkCandidate(perching_pos,
                                                                &perching_yaw,
                                                                problem,
                                                                problem.surface)
                    : PerchingRuntimeManager::CheckResult{};
        const bool candidate_accepted =
                perching_runtime_manager_ &&
                perching_runtime_manager_->candidateAccepted(candidate_check);
        if (!candidate_accepted) {
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_CANDIDATE_REJECTED reason={}, valid={}, safe={}, terminal_sync={}, dynamics_feasible={}, terminal_pos_err={:.3f}, terminal_vel_err={:.3f}, max_thrust={:.3f}, max_omega={:.3f}",
                           candidate_check.reason,
                           candidate_check.valid,
                           candidate_check.safe,
                           candidate_check.terminal_sync,
                           candidate_check.dynamics_feasible,
                           candidate_check.terminal_position_error,
                           candidate_check.terminal_velocity_error,
                           candidate_check.max_thrust,
                           candidate_check.max_omega);
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        if (!commitTrackingToPerchingTrajectory(tracking_pos,
                                                tracking_yaw,
                                                tracking_local_t,
                                                init_guess.handover_delay,
                                                perching_pos,
                                                perching_yaw,
                                                "tracking_to_perching")) {
            tracking_perching_manager_->onPerchingRejected();
            return tracking_ret;
        }

        tracking_perching_manager_->onPerchingCommitted();
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
        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            tracking_runtime_manager_->reset();
        }
        if (new_task &&
            tracking_perching_manager_ &&
            !tracking_perching_manager_->perchingRequested()) {
            tracking_perching_manager_->reset();
        }

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
        if (tracking_perching_manager_ &&
            (tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED ||
             tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING ||
             tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT)) {
            const bool contact_imminent =
                    perching_runtime_manager_ &&
                    perching_runtime_manager_->status() ==
                        PerchingRuntimeManager::Status::CONTACT_IMMINENT;
            if (!cfg_.perching_abort_to_tracking_enable || contact_imminent) {
                latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
                time_consuming_[TOTAL_REPLAN] = total_t.stop();
                return SUCCESS;
            }
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_PERCHING_ABORT_TO_TRACKING reason=surface_unavailable_before_contact");
            tracking_perching_manager_->onAbortToTracking();
            if (perching_runtime_manager_) {
                perching_runtime_manager_->reset();
            }
        }
        const Vec3f goal = target_prediction.empty() ? robot_state_.p : target_prediction.back().position;
        latest_replan.setGoal(goal, target_prediction.empty() ? NAN : target_prediction.back().yaw, robot_state_);
        gi_.goal_p = goal;
        gi_.goal_yaw = target_prediction.empty() ? NAN : target_prediction.back().yaw;
        gi_.new_goal = new_task;
        if (new_task && cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            tracking_runtime_manager_->reset();
        }

        const RET_CODE ret = optimizeTrackingTask(target_prediction, false);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::ReplanTrackingOnce(
            const traj_opt::DynamicTargetStates &target_prediction,
            const traj_opt::PerchingSurfaceState &surface,
            const bool &new_task) {
        TimeConsuming total_t("ReplanTrackingOnceTrackingPerching", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            return FAILED;
        }
        latest_replan.setGoal(target_prediction.empty() ? surface.position : target_prediction.back().position,
                              target_prediction.empty() ? surface.yaw : target_prediction.back().yaw,
                              robot_state_);
        gi_.goal_p = target_prediction.empty() ? surface.position : target_prediction.back().position;
        gi_.goal_yaw = target_prediction.empty() ? surface.yaw : target_prediction.back().yaw;
        gi_.new_goal = new_task;

        if (tracking_perching_manager_ &&
            (tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED ||
             tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING ||
             tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT)) {
            const RET_CODE ret = optimizePerchingTask(surface, false);
            time_consuming_[TOTAL_REPLAN] = total_t.stop();
            return ret;
        }

        if (new_task && cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            tracking_runtime_manager_->reset();
        }

        const RET_CODE tracking_ret = optimizeTrackingTask(target_prediction, false);
        const RET_CODE ret = tryCommitPerchingFromTracking(target_prediction, surface, tracking_ret);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    void GeneralPlanner::setTrackingPerchingRequest(const bool request) {
        if (tracking_perching_manager_) {
            tracking_perching_manager_->setPerchingRequest(request);
        }
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
        if (perching_runtime_manager_) {
            perching_runtime_manager_->reset();
        }

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

    RET_CODE GeneralPlanner::PlanDynamicTakeoffFromRest(
            const traj_opt::PerchingSurfaceState &surface,
            const bool &new_task) {
        TimeConsuming total_t("PlanDynamicTakeoffFromRest", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            ros_ptr_->warn(" -- [GeneralPlanner] in [PlanDynamicTakeoffFromRest]: No odom, force return.");
            return FAILED;
        }
        latest_replan.setGoal(surface.position, surface.yaw, robot_state_);
        gi_.goal_p = surface.position;
        gi_.goal_yaw = surface.yaw;
        gi_.new_goal = new_task;
        last_exp_traj_info_.setEmpty();
        if (takeoff_runtime_manager_) {
            takeoff_runtime_manager_->reset();
        }
        active_takeoff_problem_valid_ = false;

        const RET_CODE ret = optimizeDynamicTakeoffTask(surface, true);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::ReplanDynamicTakeoffOnce(
            const traj_opt::PerchingSurfaceState &surface,
            const bool &new_task) {
        TimeConsuming total_t("ReplanDynamicTakeoffOnce", false);
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
            takeoff_runtime_manager_ &&
            takeoff_runtime_manager_->hasCommittedTakeoff() &&
            !cmd_traj_info_.empty()) {
            cmd_traj_info_.lock();
            const double start_wt = cmd_traj_info_.getStartWallTime();
            const double total_dur = cmd_traj_info_.getTotalDuration();
            const bool has_active_traj = !cmd_traj_info_.posTraj().empty();
            cmd_traj_info_.unlock();
            const double local_t = ros_ptr_->getSimTime() - start_wt;
            if (has_active_traj && local_t >= 0.0 && local_t <= total_dur) {
                latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
                time_consuming_[TOTAL_REPLAN] = total_t.stop();
                return SUCCESS;
            }
        }
        if (new_task && takeoff_runtime_manager_) {
            takeoff_runtime_manager_->reset();
            active_takeoff_problem_valid_ = false;
        }

        const RET_CODE ret = optimizeDynamicTakeoffTask(surface, false);
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

    double GeneralPlanner::getCommittedTrajectoryRemainingDuration() {
        cmd_traj_info_.lock();
        if (cmd_traj_info_.empty()) {
            cmd_traj_info_.unlock();
            return 0.0;
        }
        const double remaining = cmd_traj_info_.getTotalDuration() -
                                 (ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime());
        cmd_traj_info_.unlock();
        return std::max(0.0, remaining);
    }

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

        for (double offset = 0.0;
             offset <= local_report.check_horizon + 1.0e-6;
             offset += sample_dt) {
            const double t = std::min(total_duration, current_t + offset);
            const Vec3f pos = traj.getPos(t);
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
                if (!unsafe &&
                    (pos - last_pos).norm() > 1.0e-4 &&
                    !map_manager_->isLineFree(last_pos, pos, true, unknown_as_occupied)) {
                    unsafe = true;
                    reason = "line_collision";
                }
            }

            if (unsafe) {
                if (hit_streak == 0) {
                    streak_start_t = t;
                    streak_start_pos = pos;
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

    bool GeneralPlanner::state2stateCurrentTrajectorySafeForNoNeed(
            const Trajectory &traj,
            const double start_t) const {
        if (traj.empty()) {
            return false;
        }
        const double total_duration = traj.getTotalDuration();
        if (!std::isfinite(total_duration) || start_t >= total_duration) {
            return true;
        }
        CommittedTrajectorySafetyReport report;
        const double now_wt = traj.start_WT + std::clamp(start_t, 0.0, total_duration);
        double horizon = std::max(0.0, total_duration - start_t);
        const double backup_start_t = cmd_traj_info_.getBackupTrajStartTT();
        if (std::isfinite(backup_start_t) && backup_start_t > 0.0 && backup_start_t < total_duration) {
            if (start_t >= backup_start_t - 1.0e-3) {
                return false;
            }
            horizon = std::min(horizon, std::max(0.0, backup_start_t - start_t));
            if (horizon <= 1.0e-3) {
                return false;
            }
        }
        const double dt = std::max(cfg_.sample_traj_dt, cfg_.resolution);
        const bool safe = checkPositionTrajectorySafety(traj,
                                                        now_wt,
                                                        horizon,
                                                        dt,
                                                        1,
                                                        false,
                                                        &report);
        if (!safe && cfg_.print_log) {
            ros_ptr_->warn(" -- [GeneralPlanner] Dynamic safety guard blocks NO_NEED: reason={}, ttc={:.3f}, pos=({:.2f},{:.2f},{:.2f}).",
                           report.reason,
                           report.time_to_collision,
                           report.collision_pos.x(),
                           report.collision_pos.y(),
                           report.collision_pos.z());
        }
        return safe;
    }

    bool GeneralPlanner::trackingPerchingPerchingActive() const {
        return tracking_perching_manager_ &&
               trackingPerchingPerchingStatus(tracking_perching_manager_->status());
    }

    void GeneralPlanner::markTrackingPerchingContact() {
        if (tracking_perching_manager_ &&
            trackingPerchingPerchingStatus(tracking_perching_manager_->status())) {
            tracking_perching_manager_->onContact();
        }
    }


    void GeneralPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                             double &yaw,
                                             double &yaw_dot,
                                             bool &on_backup_traj,
                                             bool &traj_finish) {
        cmd_traj_info_.lock();
        if (cmd_traj_info_.empty()) {
            cmd_traj_info_.unlock();
            ros_ptr_->warn(" -- [Checker] getOneCommandFromTraj called with empty committed trajectory.");
            makeHoldCommandFromRobotState(robot_state_, pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            return;
        }
        const double cur_t = ros_ptr_->getSimTime();
        const double cmd_start_WT = cmd_traj_info_.getStartWallTime();
//        const bool &backup_avilibale = cmd_traj_info_.backupTrajAvilibale();
//        const double &backup_start_TT = cmd_traj_info_.getBackupTrajStartTT();
        const double total_dur = cmd_traj_info_.getTotalDuration();
        if (!std::isfinite(cur_t) || !std::isfinite(cmd_start_WT) ||
            !std::isfinite(total_dur) || total_dur <= 1.0e-6) {
            cmd_traj_info_.unlock();
            ros_ptr_->warn(" -- [Checker] getOneCommandFromTraj has invalid timing: cur_t={}, start_WT={}, duration={}.",
                           cur_t, cmd_start_WT, total_dur);
            makeHoldCommandFromRobotState(robot_state_, pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            return;
        }

        traj_finish = (cur_t - cmd_start_WT) > total_dur;
        const double eval_t = traj_finish ? total_dur : std::clamp(cur_t - cmd_start_WT, 0.0, total_dur);

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
        if (checker::checkStateFinite(pvaj, "cmd_pvaj").rejected() ||
            !std::isfinite(yaw) || !std::isfinite(yaw_dot)) {
            cmd_traj_info_.unlock();
            ros_ptr_->warn(" -- [Checker] getOneCommandFromTraj sampled invalid command at eval_t={}.", eval_t);
            makeHoldCommandFromRobotState(robot_state_, pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            return;
        }
        if (takeoff_runtime_manager_ && active_takeoff_problem_valid_) {
            takeoff_runtime_manager_->updateStatusByPosition(pvaj.col(0),
                                                             active_takeoff_problem_);
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
                if (!gi_.new_goal &&
                    last_exp_traj_info.getSFCSize() == 1 &&
                    last_exp_traj_info.connectedToGoal() &&
                    state2stateCurrentTrajectorySafeForNoNeed(guide_pos_traj, replan_state_TT)) {
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
                    (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3 &&
                    state2stateCurrentTrajectorySafeForNoNeed(guide_pos_traj, replan_state_TT)) {
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

        if (rejectOnCheckFailure(ros_ptr_,
                                 "generateExpTraj guide",
                                 checker::checkGuidePath(guide_path,
                                                         guide_stamp,
                                                         cfg_.resolution,
                                                         "state2state_exp"))) {
            return FAILED;
        }
        latest_replan.setGuidePath(guide_path);

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
        if (use_esdf_exp_traj) {
            VecDf init_ts;
            vec_Vec3f init_ps;
            traj_manager_->esdf()->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        } else if (use_plain_exp_traj) {
            VecDf init_ts;
            vec_Vec3f init_ps;
            traj_manager_->plain()->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
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

        if (rejectOnCheckFailure(ros_ptr_,
                                 "generateExpTraj output",
                                 checker::checkExpTrajectory(out_exp_traj_info,
                                                             cfg_,
                                                             "state2state_exp_output"))) {
            out_exp_traj_info.setEmpty();
            return FAILED;
        }

        latest_replan.setExpYawTraj(temp_yaw_traj);
        latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

    RET_CODE GeneralPlanner::generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info) {
        drone_state_mutex_.lock();
        back_traj_info.setRobotPos(robot_state_.p);
        drone_state_mutex_.unlock();
        TimeConsuming t_back_frontend("t_back_frontend", false);

        if (rejectOnCheckFailure(ros_ptr_,
                                 "generateBackupTrajectory input exp",
                                 checker::checkExpTrajectory(ref_exp_traj,
                                                             cfg_,
                                                             "backup_ref_exp"))) {
            back_traj_info.setEmpty();
            return FAILED;
        }

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

        if (eval_ps.empty()) {
            ros_ptr_->warn(" -- [Checker] generateBackupTrajectory has no sampled exp point, return NO_NEED.");
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
            temp_vel = ref_exp_traj.getVel(eval_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt;
        }
        if (!eval_ps.empty()) {
            eval_ps.pop_back();
        }
        if (eval_ps.empty()) {
            ros_ptr_->warn(" -- [Checker] generateBackupTrajectory backup seed samples are empty after trimming.");
            back_traj_info.setEmpty();
            return NO_NEED;
        }
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        //        bool use_new{true};
        //        if (use_new) {
        double t0 = ros_ptr_->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        if (!std::isfinite(t0) || !std::isfinite(te) || t0 < -1.0e-6 || te <= t0 + 1.0e-6 ||
            te > ref_exp_traj.getTotalDuration() + 1.0e-6) {
            ros_ptr_->warn(" -- [Checker] generateBackupTrajectory invalid backup time window: t0={}, te={}, exp_dur={}.",
                           t0, te, ref_exp_traj.getTotalDuration());
            back_traj_info.setEmpty();
            return NO_NEED;
        }
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

        double init_ts;
        VecDf init_times;
        vec_Vec3f init_ps;
        traj_manager_->backup()->getInitValue(init_ts, init_times, init_ps);
        latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                         t0, te,
                                         back_traj_info.getSFC());

        if (!temp_ret) {
            ros_ptr_->warn(" -- [GeneralPlanner] OptimizationBakTrajInPolytopes failed, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        } else {
            if (!std::isfinite(opt_ts) || opt_ts < t0 - 1.0e-6 || opt_ts > te + 1.0e-6) {
                ros_ptr_->error(" -- [Checker] generateBackupTrajectory invalid opt_ts={}, t0={}, te={}.",
                                opt_ts, t0, te);
                return OPT_FAILED;
            }
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
            const auto backup_check = checker::checkBackupTrajectory(back_traj_info,
                                                                     cfg_,
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
                                cfg_,
                                "state2state_backup_yaw_brake_output");
	                        if (!yaw_brake_check.rejected()) {
	                            ros_ptr_->warn(" -- [GeneralPlanner] Normal backup yaw rejected [{}], use yaw-brake fallback.",
	                                           backup_check.code);
	                            latest_replan.setBackupTraj(temp_pos_traj);
	                            latest_replan.setBackupYawTraj(yaw_brake_traj);
	                            return SUCCESS;
                        }
                        logCheckResult(ros_ptr_,
                                       "generateBackupTrajectory yaw-brake output",
                                       yaw_brake_check);
                    } else {
                        ros_ptr_->warn(" -- [GeneralPlanner] Failed to build yaw-brake fallback for backup trajectory.");
                    }
                }
                logCheckResult(ros_ptr_, "generateBackupTrajectory output", backup_check);
	                back_traj_info.setEmpty();
	                return OPT_FAILED;
	            }
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

    namespace {
        void appendPathPointUnique(const Vec3f &point, vec_Vec3f &path) {
            if (!point.allFinite()) {
                return;
            }
            if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
                path.emplace_back(point);
            }
        }

        double pathForwardProgress(const vec_Vec3f &path, const Vec3f &origin, const Vec3f &dir) {
            if (path.empty() || dir.norm() < 1.0e-6) {
                return 0.0;
            }
            return (path.back() - origin).dot(dir.normalized());
        }
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

        const bool goal_inside_local_map = map_manager_->insideLocalMap(goal);
        const rog_map::GridType goal_inf_type =
                goal_inside_local_map ? map_manager_->getInfGridType(goal) : OUT_OF_MAP;
        const bool hidden_unknown_goal = cfg_.unknown_goal_reveal_en &&
                                         goal_inside_local_map &&
                                         goal_inf_type == UNKNOWN;
        const bool unknown_as_occupied_for_frontend = cfg_.frontend_in_known_free || hidden_unknown_goal;
        if (hidden_unknown_goal && cfg_.print_log) {
            ros_ptr_->info(" -- [GeneralPlanner] Click goal is unknown; search a reveal/frontier waypoint first.");
        }

        int flag = ON_INF_MAP |
                   (unknown_as_occupied_for_frontend ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   DONT_USE_INF_NEIGHBOR;

        vec_Vec3f normal_path;
        RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point,
                                                               goal,
                                                               flag,
                                                               temp_plannning_horizon,
                                                               normal_path,
                                                               cfg_.frontend_astar_time_out);

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        const bool distance_field_frontend = cfg_.esdf_traj_en || cfg_.plain_traj_en;
        if (ret_code == NO_PATH && !distance_field_frontend) {
            flag = ON_PROB_MAP |
                   (unknown_as_occupied_for_frontend ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                          normal_path, cfg_.frontend_astar_time_out);
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

        auto pointUsable = [&](const Vec3f &point) {
            if (!point.allFinite() || !map_manager_->insideLocalMap(point)) {
                return false;
            }
            const rog_map::GridType inf_type = map_manager_->getInfGridType(point);
            if (inf_type == OCCUPIED || inf_type == OUT_OF_MAP) {
                return false;
            }
            return !(unknown_as_occupied_for_frontend && inf_type == UNKNOWN);
        };

        auto lineUsable = [&](const Vec3f &a, const Vec3f &b) {
            return pointUsable(a) &&
                   pointUsable(b) &&
                   map_manager_->isLineFree(a, b, true, unknown_as_occupied_for_frontend);
        };

        auto blockedSpanOnDirectLine = [&]() {
            const Vec3f delta = goal - temp_start_point;
            const double len = std::min(delta.norm(), std::max(0.0, searching_horizon));
            if (len < 1.0e-4) {
                return 0.0;
            }
            const Vec3f dir = delta / std::max(1.0e-6, delta.norm());
            const double step = std::max(0.2, map_manager_->getInfResolution());
            double blocked_span = 0.0;
            double current_span = 0.0;
            for (double s = 0.0; s <= len + 1.0e-6; s += step) {
                const Vec3f sample = temp_start_point + dir * std::min(s, len);
                const bool blocked = !pointUsable(sample);
                if (blocked) {
                    current_span += step;
                    blocked_span = std::max(blocked_span, current_span);
                } else {
                    current_span = 0.0;
                }
            }
            return blocked_span;
        };

        auto buildOverWallCandidate = [&](vec_Vec3f &candidate, RET_CODE &candidate_ret) {
            candidate.clear();
            candidate_ret = FAILED;
            if (!cfg_.over_wall_search_en) {
                return false;
            }

            const Vec3f delta = goal - temp_start_point;
            const double goal_dist = delta.norm();
            if (goal_dist < std::max(0.5, cfg_.resolution * 5.0)) {
                return false;
            }
            const Vec3f horizontal_delta(delta.x(), delta.y(), 0.0);
            const double horizontal_dist = horizontal_delta.norm();
            if (horizontal_dist < 1.0e-3) {
                return false;
            }
            const double blocked_span = blockedSpanOnDirectLine();
            if (blocked_span < std::max(0.0, cfg_.over_wall_min_blocked_span) &&
                ret_code == REACH_GOAL) {
                return false;
            }

            const Vec3f dir_xy = horizontal_delta / horizontal_dist;
            const double max_climb = std::max(0.0, cfg_.over_wall_max_climb);
            const double height_step = std::max(map_manager_->getInfResolution(), cfg_.over_wall_height_step);
            const double base_z = std::max(temp_start_point.z(), goal.z());
            const double forward_ratio = std::clamp(cfg_.over_wall_forward_ratio, 0.2, 1.0);

            for (double climb = height_step; climb <= max_climb + 1.0e-6; climb += height_step) {
                const double level_z = base_z + climb;
                const Vec3f elevated_start(temp_start_point.x(), temp_start_point.y(), level_z);
                if (!lineUsable(temp_start_point, elevated_start)) {
                    continue;
                }

                const double goal_detour = (elevated_start - temp_start_point).norm() +
                                           (Vec3f(goal.x(), goal.y(), level_z) - elevated_start).norm() +
                                           std::abs(level_z - goal.z());
                if (goal_detour <= searching_horizon * 1.15) {
                    const Vec3f elevated_goal(goal.x(), goal.y(), level_z);
                    if (lineUsable(elevated_start, elevated_goal) &&
                        lineUsable(elevated_goal, goal)) {
                        appendPathPointUnique(temp_start_point, candidate);
                        appendPathPointUnique(elevated_start, candidate);
                        appendPathPointUnique(elevated_goal, candidate);
                        appendPathPointUnique(goal, candidate);
                        candidate_ret = REACH_GOAL;
                        return true;
                    }
                }

                double forward_dist = std::min(horizontal_dist,
                                               std::max(0.0, searching_horizon - climb) * forward_ratio);
                while (forward_dist > std::max(0.5, cfg_.resolution * 5.0)) {
                    const Vec3f elevated_forward = elevated_start + dir_xy * forward_dist;
                    if (lineUsable(elevated_start, elevated_forward)) {
                        appendPathPointUnique(temp_start_point, candidate);
                        appendPathPointUnique(elevated_start, candidate);
                        appendPathPointUnique(elevated_forward, candidate);
                        candidate_ret = REACH_HORIZON;
                        return true;
                    }
                    forward_dist -= std::max(0.5, 2.0 * map_manager_->getInfResolution());
                }
            }
            return false;
        };

        vec_Vec3f selected_path = normal_path;
        RET_CODE selected_ret = ret_code;
        vec_Vec3f over_wall_path;
        RET_CODE over_wall_ret = FAILED;
        if (buildOverWallCandidate(over_wall_path, over_wall_ret)) {
            const Vec3f goal_dir = (goal - temp_start_point).norm() > 1.0e-6
                                       ? (goal - temp_start_point).normalized()
                                       : Vec3f::Zero();
            const double normal_progress = pathForwardProgress(normal_path, temp_start_point, goal_dir);
            const double over_wall_progress = pathForwardProgress(over_wall_path, temp_start_point, goal_dir);
            const bool normal_failed = ret_code != REACH_HORIZON && ret_code != REACH_GOAL;
            const bool over_reaches_goal = over_wall_ret == REACH_GOAL && ret_code != REACH_GOAL;
            const bool progress_better =
                    over_wall_progress > normal_progress + std::max(0.0, cfg_.over_wall_min_progress_gain);
            if (normal_failed || over_reaches_goal || (ret_code == REACH_HORIZON && progress_better)) {
                selected_path = over_wall_path;
                selected_ret = over_wall_ret;
                if (cfg_.print_log) {
                    ros_ptr_->info(" -- [GeneralPlanner] Use over-wall frontend candidate: ret={}, progress {:.2f}->{:.2f}.",
                                   RET_CODE_STR[over_wall_ret],
                                   normal_progress,
                                   over_wall_progress);
                }
            }
        }

        if (selected_ret != REACH_HORIZON && selected_ret != REACH_GOAL) {
            ros_ptr_->error(
                    " -- [GeneralPlanner] Path search failed with [{}], force return.\n",
                    RET_CODE_STR[ret_code].c_str());
            return false;
        }
        if (!start_point_escape_path.empty()) {
            selected_path.insert(selected_path.begin(), start_point_escape_path.begin(),
                                 start_point_escape_path.end());
        }

        if (selected_path.empty()) {
            ros_ptr_->warn(
                    " -- [GeneralPlanner] Path search failed with empty segments, force return.");
            return false;
        }
        path = selected_path;
        path.insert(path.begin(), start_pt);
        if (selected_ret == REACH_GOAL) {
            path.push_back(goal);
        }
        return true;
    }


    void GeneralPlanner::getRobotState(rog_map::RobotState &out) {
        robot_state_ = map_manager_->getRobotState();
        out = robot_state_;
    }
}
