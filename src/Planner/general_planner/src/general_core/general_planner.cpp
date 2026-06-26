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
#include <sstream>
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

        Vec3f state2stateGoalAxis(const Vec3f &axis_start,
                                  const Vec3f &fallback_start,
                                  const Vec3f &goal);

        double state2stateGoalOvershoot(const Vec3f &point,
                                        const Vec3f &axis_start,
                                        const Vec3f &fallback_start,
                                        const Vec3f &goal);

        double state2stateMaxGoalOvershoot(const vec_Vec3f &path,
                                           const Vec3f &axis_start,
                                           const Vec3f &fallback_start,
                                           const Vec3f &goal);

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

        bool buildConstantPositionTrajectory(const Vec3f &position,
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

        bool buildConstantYawTrajectory(const double yaw,
                                        const double duration,
                                        const double start_wt,
                                        Trajectory &traj) {
            if (!std::isfinite(yaw) ||
                !std::isfinite(duration) ||
                duration <= 1.0e-5 ||
                !std::isfinite(start_wt)) {
                return false;
            }

            Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 8);
            coeff(0, 7) = yaw;
            traj.clear();
            traj.emplace_back(duration, coeff);
            traj.start_WT = start_wt;
            return !traj.empty();
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
            if ((degree == 3 || degree == 5 || degree == 7) &&
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

        Vec3f interpolatePointOnTimedGuide(const vec_Vec3f &path,
                                           const std::vector<double> &path_t,
                                           const double query_t) {
            if (path.empty()) {
                return Vec3f::Zero();
            }
            if (path_t.size() != path.size()) {
                return path.back();
            }
            if (path.size() == 1 || query_t <= path_t.front()) {
                return path.front();
            }
            if (query_t >= path_t.back()) {
                return path.back();
            }
            const auto it = std::lower_bound(path_t.begin(), path_t.end(), query_t);
            const std::size_t right = static_cast<std::size_t>(std::distance(path_t.begin(), it));
            if (right == 0 || right >= path.size()) {
                return path.back();
            }
            const std::size_t left = right - 1;
            const double dt = std::max(1.0e-9, path_t[right] - path_t[left]);
            const double alpha = std::clamp((query_t - path_t[left]) / dt, 0.0, 1.0);
            return path[left] + alpha * (path[right] - path[left]);
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

        double trackingHardSafeDistance(const Config &cfg) {
            return std::max(cfg.tracking_hard_safe_distance, cfg.robot_r + 0.02);
        }

        const char *gridTypeName(const rog_map::GridType type) {
            switch (type) {
                case rog_map::GridType::UNKNOWN:
                    return "UNKNOWN";
                case rog_map::GridType::OCCUPIED:
                    return "OCCUPIED";
                case rog_map::GridType::KNOWN_FREE:
                    return "KNOWN_FREE";
                case rog_map::GridType::OUT_OF_MAP:
                    return "OUT_OF_MAP";
                case rog_map::GridType::UNDEFINED:
                    return "UNDEFINED";
                case rog_map::GridType::FRONTIER:
                    return "FRONTIER";
            }
            return "UNKNOWN_GRID_TYPE";
        }

        std::string formatVec3Compact(const Vec3f &v) {
            if (!v.allFinite()) {
                return "[nan,nan,nan]";
            }
            return fmt::format("[{:.3f},{:.3f},{:.3f}]", v.x(), v.y(), v.z());
        }

        Vec3f trackingTargetDirection(const traj_opt::DynamicTargetStates &prediction,
                                      const double speed_threshold,
                                      const double vertical_threshold = 0.12,
                                      const bool motion_3d_enable = false) {
            if (prediction.empty()) {
                return Vec3f::UnitX();
            }

            Vec3f dir = prediction.front().velocity;
            if (!motion_3d_enable) {
                dir.z() = 0.0;
            }
            const double threshold =
                    motion_3d_enable ? std::min(speed_threshold, vertical_threshold)
                                     : speed_threshold;
            if (dir.norm() > threshold) {
                return dir.normalized();
            }

            if (prediction.size() >= 2) {
                dir = prediction.back().position - prediction.front().position;
                if (!motion_3d_enable) {
                    dir.z() = 0.0;
                }
                if (dir.norm() > 1.0e-4) {
                    return dir.normalized();
                }
            }

            return Vec3f::UnitX();
        }

        struct TrackingMotionMetrics {
            double speed_xy{0.0};
            double speed_z{0.0};
            double speed_3d{0.0};
            double displacement_xy{0.0};
            double displacement_z{0.0};
            double displacement_3d{0.0};
            double progress_xy{0.0};
            double progress_3d{0.0};
            double target_speed_xy{0.0};
            double target_speed_z{0.0};
            double target_speed_3d{0.0};
            bool target_vertical_moving{false};
            bool target_moving{false};
        };

        TrackingMotionMetrics computeTrackingMotionMetrics(
                const Trajectory &traj,
                const traj_opt::DynamicTargetStates &target_prediction,
                const Config &cfg,
                const double candidate_eval_start_t,
                const double target_eval_start_t,
                const double horizon) {
            TrackingMotionMetrics metrics;
            if (traj.empty() || target_prediction.empty()) {
                return metrics;
            }
            const double total = traj.getTotalDuration();
            const double start_t = std::clamp(candidate_eval_start_t, 0.0, total);
            const double target_start = std::max(0.0, target_eval_start_t);
            const double eval_horizon =
                    std::min({std::max(0.0, horizon),
                              std::max(0.0, total - start_t),
                              std::max(0.0, target_prediction.back().t - target_start)});
            const double end_t = std::clamp(start_t + eval_horizon, 0.0, total);
            const Vec3f p0 = traj.getPos(start_t);
            const Vec3f p1 = traj.getPos(end_t);
            const Vec3f v0 = traj.getVel(start_t);
            if (p0.allFinite() && p1.allFinite()) {
                const Vec3f dp = p1 - p0;
                metrics.displacement_xy = dp.head<2>().norm();
                metrics.displacement_z = std::abs(dp.z());
                metrics.displacement_3d = dp.norm();
                const Vec3f target_dir =
                        trackingTargetDirection(target_prediction,
                                                cfg.tracking_no_motion_target_speed_threshold,
                                                cfg.tracking_vertical_motion_threshold,
                                                cfg.tracking_motion_3d_enable);
                metrics.progress_xy = dp.head<2>().dot(target_dir.head<2>());
                metrics.progress_3d = dp.dot(target_dir);
            }
            if (v0.allFinite()) {
                metrics.speed_xy = v0.head<2>().norm();
                metrics.speed_z = std::abs(v0.z());
                metrics.speed_3d = v0.norm();
            }
            const auto target0 = interpolateTargetPrediction(target_prediction, target_start);
            metrics.target_speed_xy = target0.velocity.head<2>().norm();
            metrics.target_speed_z = std::abs(target0.velocity.z());
            metrics.target_speed_3d = target0.velocity.norm();
            double target_span_3d = 0.0;
            double target_span_z = 0.0;
            if (target_prediction.size() >= 2) {
                const auto target1 =
                        interpolateTargetPrediction(target_prediction,
                                                    std::min(target_prediction.back().t,
                                                             target_start + eval_horizon));
                const Vec3f target_dp = target1.position - target0.position;
                target_span_3d = target_dp.norm();
                target_span_z = std::abs(target_dp.z());
            }
            metrics.target_vertical_moving =
                    metrics.target_speed_z > cfg.tracking_vertical_motion_threshold ||
                    target_span_z > cfg.tracking_no_motion_min_displacement_z;
            metrics.target_moving =
                    metrics.target_speed_xy > cfg.tracking_no_motion_target_speed_threshold ||
                    metrics.target_vertical_moving ||
                    target_span_3d > std::max(cfg.tracking_no_motion_min_displacement,
                                              cfg.tracking_no_motion_min_displacement_z);
            return metrics;
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

        double trackingAdaptiveFovRange(const double configured_range,
                                        const double tracking_distance,
                                        const double distance_upper_tolerance,
                                        const double distance_tolerance,
                                        const double height_offset,
                                        const double height_tolerance,
                                        const double horizontal_fov_deg,
                                        const double vertical_fov_deg) {
            constexpr double kPi = 3.14159265358979323846;
            constexpr double kDegToRad = kPi / 180.0;
            const double horizontal_upper =
                    std::max(0.05,
                             tracking_distance +
                                     std::max({0.0,
                                               distance_upper_tolerance,
                                               distance_tolerance}));
            const double vertical_upper =
                    std::max(0.0, std::abs(height_offset) + std::max(0.0, height_tolerance));
            const double base_range = configured_range > 0.0 ? configured_range : horizontal_upper;
            const double half_h =
                    std::clamp(0.5 * std::max(1.0, horizontal_fov_deg) * kDegToRad,
                               kPi / 180.0,
                               0.5 * kPi - 1.0e-3);
            const double half_v =
                    std::clamp(0.5 * std::max(1.0, vertical_fov_deg) * kDegToRad,
                               kPi / 180.0,
                               0.5 * kPi - 1.0e-3);
            const double footprint_scale = std::hypot(std::tan(half_h), std::tan(half_v));
            const double geometry_range = std::hypot(horizontal_upper, vertical_upper);
            const double footprint_range = horizontal_upper + vertical_upper * footprint_scale;
            return std::max({0.05, base_range, geometry_range, footprint_range});
        }

        double trackingAdaptiveFovRange(const Config &cfg) {
            return trackingAdaptiveFovRange(cfg.tracking_fov_range,
                                            cfg.tracking_distance,
                                            cfg.tracking_distance_upper_tolerance,
                                            cfg.tracking_distance_tolerance,
                                            cfg.tracking_height_offset,
                                            cfg.tracking_height_tolerance,
                                            cfg.tracking_fov_horizontal_deg,
                                            cfg.tracking_fov_vertical_deg);
        }

        struct TrackingFovSampleStatus {
            bool inside{false};
            double h_violation{0.0};
            double v_violation{0.0};
            double range_violation{0.0};
            double front_violation{0.0};
            double distance{0.0};
            double qx{0.0};
        };

        TrackingFovSampleStatus evaluateYawOnlyTrackingFov(
                const Vec3f &tracker,
                const Vec3f &target,
                const double yaw,
                const double horizontal_fov_deg,
                const double vertical_fov_deg,
                const double range,
                const double range_margin,
                const double front_margin) {
            TrackingFovSampleStatus out;
            if (!tracker.allFinite() || !target.allFinite() || !std::isfinite(yaw)) {
                out.h_violation = std::numeric_limits<double>::infinity();
                out.v_violation = std::numeric_limits<double>::infinity();
                out.range_violation = std::numeric_limits<double>::infinity();
                out.front_violation = std::numeric_limits<double>::infinity();
                return out;
            }

            constexpr double kPi = 3.14159265358979323846;
            constexpr double kDegToRad = kPi / 180.0;
            const double half_h =
                    std::clamp(0.5 * std::max(1.0, horizontal_fov_deg) * kDegToRad,
                               kPi / 180.0,
                               0.5 * kPi - 1.0e-3);
            const double half_v =
                    std::clamp(0.5 * std::max(1.0, vertical_fov_deg) * kDegToRad,
                               kPi / 180.0,
                               0.5 * kPi - 1.0e-3);
            const double effective_range =
                    range > 0.0 ? std::max(0.05, range - std::max(0.0, range_margin)) : -1.0;
            const double min_forward = std::max(1.0e-3, front_margin);

            const Vec3f rel = target - tracker;
            const double c = std::cos(yaw);
            const double sn = std::sin(yaw);
            const Vec3f q(c * rel.x() + sn * rel.y(),
                          -sn * rel.x() + c * rel.y(),
                          rel.z());
            out.qx = q.x();
            out.distance = q.norm();

            const double h_angle = q.x() > 1.0e-6
                                       ? std::atan2(std::abs(q.y()), q.x())
                                       : kPi;
            const double v_angle = q.x() > 1.0e-6
                                       ? std::atan2(std::abs(q.z()), q.x())
                                       : kPi;
            out.h_violation = h_angle - half_h;
            out.v_violation = v_angle - half_v;
            out.range_violation =
                    effective_range > 0.0 ? out.distance - effective_range : 0.0;
            out.front_violation = min_forward - q.x();
            out.inside =
                    out.h_violation <= 1.0e-6 &&
                    out.v_violation <= 1.0e-6 &&
                    out.range_violation <= 1.0e-6 &&
                    out.front_violation <= 1.0e-6;
            return out;
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
        se3_aggressive_manager_ =
                std::make_unique<SE3AggressiveManager>(cfg_, ros_ptr_, map_manager_, astar_ptr_, cg_ptr_);


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

    std::string GeneralPlanner::getLatestState2StateZDebugInfo() const {
        if (!latest_state2state_z_debug_.valid) {
            return "";
        }
        const auto &d = latest_state2state_z_debug_;
        std::ostringstream oss;
        oss << ";z_debug_valid=1"
            << ";exp_mode=" << d.exp_mode
            << ";goal_z=" << d.goal_z
            << ";robot_z=" << d.robot_z
            << ";guide_size=" << d.guide_size
            << ";guide_z_valid=" << static_cast<int>(d.guide.valid)
            << ";guide_z_start=" << d.guide.start
            << ";guide_z_end=" << d.guide.end
            << ";guide_z_min=" << d.guide.min
            << ";guide_z_max=" << d.guide.max
            << ";local_target_z=" << d.local_target_z
            << ";local_target_goal_z_err=" << d.local_target_goal_z_err
            << ";local_target_goal_dist=" << d.local_target_goal_dist
            << ";local_target_goal_xy_dist=" << d.local_target_goal_xy_dist
            << ";local_target_is_goal=" << static_cast<int>(d.local_target_is_global_goal)
            << ";opt_z_valid=" << static_cast<int>(d.optimized.valid)
            << ";opt_z_start=" << d.optimized.start
            << ";opt_z_end=" << d.optimized.end
            << ";opt_z_min=" << d.optimized.min
            << ";opt_z_max=" << d.optimized.max
            << ";opt_end_local_target_z_err=" << d.opt_end_local_target_z_err
            << ";exp_full_z_valid=" << static_cast<int>(d.exp_full.valid)
            << ";exp_full_z_start=" << d.exp_full.start
            << ";exp_full_z_end=" << d.exp_full.end
            << ";exp_full_z_min=" << d.exp_full.min
            << ";exp_full_z_max=" << d.exp_full.max;
        return oss.str();
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

        auto acceptExpWithoutBackupNearGoal = [&]() {
            if (!cfg_.state2state_accept_exp_without_backup_near_goal ||
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
                    std::max(cfg_.resolution * 3.0, cfg_.state2state_near_goal_radius);
            const double robot_goal_xy = (robot_state_.p.head<2>() - gi_.goal_p.head<2>()).norm();
            const bool near_goal = robot_goal_xy < near_goal_radius ||
                                   exp_traj_info.connectedToGoal();
            if (!near_goal) {
                return false;
            }

            const double now_t =
                    std::clamp(ros_ptr_->getSimTime() - exp_traj_info.getStartWallTime(),
                               0.0,
                               total_duration);
            const Vec3f start_pos = pos_traj.getPos(now_t);
            const Vec3f end_pos = pos_traj.getPos(total_duration);
            if (!start_pos.allFinite() || !end_pos.allFinite()) {
                return false;
            }
            const double end_goal_xy = (end_pos.head<2>() - gi_.goal_p.head<2>()).norm();
            if (!exp_traj_info.connectedToGoal() &&
                end_goal_xy > std::max(cfg_.resolution * 3.0, 0.3)) {
                return false;
            }

            const double start_over =
                    state2stateGoalOvershoot(start_pos,
                                             local_start_p_,
                                             start_pos,
                                             gi_.goal_p);
            const double allowed_over =
                    std::max(std::max(0.0, cfg_.state2state_over_goal_tolerance),
                             start_over + std::max(0.0, cfg_.state2state_over_goal_tolerance));
            const double sample_dt = std::max(0.02, cfg_.sample_traj_dt);
            double max_over = 0.0;
            for (double t = now_t; t < total_duration; t += sample_dt) {
                max_over = std::max(max_over,
                                    state2stateGoalOvershoot(pos_traj.getPos(t),
                                                            local_start_p_,
                                                            start_pos,
                                                            gi_.goal_p));
            }
            max_over = std::max(max_over,
                                state2stateGoalOvershoot(end_pos,
                                                        local_start_p_,
                                                        start_pos,
                                                        gi_.goal_p));
            if (max_over > allowed_over + 1.0e-6) {
                ros_ptr_->warn(" -- [GeneralPlanner] Reject exp-only near-goal fallback: candidate overshoot {:.2f}m > allowed {:.2f}m.",
                               max_over,
                               allowed_over);
                return false;
            }

            if (rejectOnCheckFailure(ros_ptr_,
                                     "ReplanOnce exp-only near-goal fallback",
                                     checker::checkExpTrajectory(exp_traj_info,
                                                                 cfg_,
                                                                 "replan_exp_only_near_goal"))) {
                return false;
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

            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            ros_ptr_->warn(" -- [GeneralPlanner] Backup generation failed near goal; commit checked exp-only trajectory to avoid running old command past goal.");
            return true;
        };

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
        if (acceptExpWithoutBackupNearGoal()) {
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

    RET_CODE GeneralPlanner::PlanSE3AggressiveFromRest(const Vec3f &goal_p,
                                                       double goal_yaw,
                                                       bool new_task) {
        (void)new_task;
        TimeConsuming total_t("PlanSE3AggressiveFromRest", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            ros_ptr_->warn(" -- [SE3Aggressive] PlanFromRest failed: no odom.");
            time_consuming_[TOTAL_REPLAN] = total_t.stop();
            return FAILED;
        }
        gi_.goal_valid = true;
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = true;
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
        ros_ptr_->vizGoalPath(vec_Vec3f{goal_p, robot_state_.p});
        const RET_CODE ret = optimizeSE3AggressiveTask(goal_p, goal_yaw, true);
        latest_replan.setRetCode((ret == SUCCESS || ret == FINISH) ? GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP
                                                                   : GENERAL_RET_CODE::GENERAL_UNDEFINED);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::ReplanSE3AggressiveOnce(const Vec3f &goal_p,
                                                     double goal_yaw,
                                                     bool new_task) {
        (void)new_task;
        TimeConsuming total_t("ReplanSE3AggressiveOnce", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        if (!robot_state_.rcv) {
            latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            ros_ptr_->warn(" -- [SE3Aggressive] Replan failed: no odom.");
            time_consuming_[TOTAL_REPLAN] = total_t.stop();
            return FAILED;
        }
        gi_.goal_valid = true;
        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = true;
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
        ros_ptr_->vizGoalPath(vec_Vec3f{goal_p, robot_state_.p});
        const RET_CODE ret = optimizeSE3AggressiveTask(goal_p, goal_yaw, false);
        latest_replan.setRetCode((ret == SUCCESS || ret == FINISH) ? GENERAL_RET_CODE::GENERAL_SUCCESS_NO_BACKUP
                                                                   : GENERAL_RET_CODE::GENERAL_UNDEFINED);
        time_consuming_[TOTAL_REPLAN] = total_t.stop();
        return ret;
    }

    StatePVAJ GeneralPlanner::makeTaskHeadState(const bool &from_rest,
                                                const double eval_wall_time) {
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
                const double eval_t = (std::isfinite(eval_wall_time)
                                           ? eval_wall_time
                                           : ros_ptr_->getSimTime() + cfg_.replan_forward_dt) -
                                      start_wt;
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

    bool GeneralPlanner::commitSE3AggressiveTrajectory(const Trajectory &pos_traj,
                                                       const std::string &traj_ns) {
        static bool warning_printed = false;
        if (!warning_printed) {
            ros_ptr_->warn(" -- [SE3Aggressive] SE3 trajectory requires a flatness-aware controller for reliable execution.");
            warning_printed = true;
        }
        const bool fix_terminal_yaw = cfg_.se3_use_yaw && std::isfinite(gi_.goal_yaw);
        return commitTaskTrajectory(pos_traj, gi_.goal_yaw, fix_terminal_yaw, traj_ns);
    }

    RET_CODE GeneralPlanner::optimizeSE3AggressiveTask(const Vec3f &goal_p,
                                                       double goal_yaw,
                                                       const bool &from_rest) {
        if (!se3_aggressive_manager_) {
            ros_ptr_->warn(" -- [SE3Aggressive] Manager is not initialized.");
            return FAILED;
        }

        StatePVAJ head_state = makeTaskHeadState(from_rest);
        if (from_rest && map_manager_) {
            Vec3f shifted_start = head_state.col(0);
            if (map_manager_->getNearestCellNot(GridType::OCCUPIED,
                                                head_state.col(0),
                                                shifted_start,
                                                3.0)) {
                head_state.col(0) = shifted_start;
            }
        }

        StatePVAJ tail_state = StatePVAJ::Zero();
        tail_state.col(0) = goal_p;
        if (cfg_.goal_vel_en && !from_rest) {
            tail_state.col(1).setZero();
        }

        const double finish_thresh = std::max(0.08, 2.0 * cfg_.resolution);
        if ((goal_p - robot_state_.p).norm() < finish_thresh) {
            return FINISH;
        }

        gi_.goal_p = goal_p;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = false;

        Trajectory pos_traj;
        std::string reason;
        if (!se3_aggressive_manager_->plan(head_state, tail_state, pos_traj, &reason, goal_yaw, 0.0)) {
            ros_ptr_->warn(" -- [SE3Aggressive] Plan failed: {}.", reason);
            return FAILED;
        }

        if (!commitSE3AggressiveTrajectory(pos_traj, "se3_aggressive")) {
            ros_ptr_->warn(" -- [SE3Aggressive] Commit failed.");
            return FAILED;
        }
        return SUCCESS;
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
                                                const std::string &traj_ns,
                                                const double candidate_head_wt,
                                                const bool allow_reacquire_fov_relax,
                                                const bool allow_old_prefix) {
        if (trackingPerchingPerchingActive()) {
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            setTrackingCommitRejectInfo("perching owns committed trajectory",
                                        "failure=perching_owns_committed_trajectory");
            ros_ptr_->warn(" -- [TrackingPerching] TRACKING_COMMIT_BLOCKED_PERCHING_ACTIVE reason=perching_owns_committed_trajectory");
            return false;
        }
        if (pos_traj.empty()) {
            setTrackingCommitRejectInfo("empty tracking trajectory",
                                        "failure=empty_tracking_trajectory");
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking trajectory is empty, cannot commit.");
            return false;
        }
        clearTrackingCommitRejectInfo();

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

        const bool runtime_has_committed_tracking =
                cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_
                    ? tracking_runtime_manager_->hasCommittedTracking()
                    : has_old_cmd;
        const double old_local_t_raw = has_old_cmd
                                           ? commit_wt - old_start_wt
                                           : std::numeric_limits<double>::quiet_NaN();
        const bool old_time_valid =
                has_old_cmd &&
                std::isfinite(old_start_wt) &&
                std::isfinite(old_total_dur) &&
                old_total_dur > 1.0e-6;
        const bool old_currently_active =
                old_time_valid &&
                old_local_t_raw >= -std::max(0.0, cfg_.tracking_commit_start_time_tolerance) &&
                old_local_t_raw < old_total_dur - 1.0e-3;
        const bool old_tracking_active_for_prefix =
                has_old_cmd &&
                runtime_has_committed_tracking &&
                old_currently_active &&
                !old_pos_traj.empty() &&
                !old_yaw_traj.empty();
        const bool should_stitch_old_prefix =
                allow_old_prefix && old_tracking_active_for_prefix;

        if (has_old_cmd && !should_stitch_old_prefix && cfg_.print_log) {
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking old prefix disabled: allow_old_prefix={}, runtime_has_committed_tracking={}, old_currently_active={}, old_local_t={:.3f}, old_total_dur={:.3f}, candidate_head_wt={:.3f}, commit_wt={:.3f}.",
                           allow_old_prefix,
                           runtime_has_committed_tracking,
                           old_currently_active,
                           std::isfinite(old_local_t_raw) ? old_local_t_raw : 0.0,
                           old_total_dur,
                           candidate_head_wt,
                           commit_wt);
        }

        auto keepOldFromSnapshot = [&](const std::string &reason) -> bool {
            if (!has_old_cmd ||
                !runtime_has_committed_tracking ||
                !old_currently_active ||
                old_pos_traj.empty() ||
                old_yaw_traj.empty()) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_INACTIVE reason={}, activity_reason=no active committed tracking snapshot, runtime_has_committed_tracking={}, old_currently_active={}, old_local_t={:.3f}, old_total_dur={:.3f}",
                                   reason,
                                   runtime_has_committed_tracking,
                                   old_currently_active,
                                   std::isfinite(old_local_t_raw) ? old_local_t_raw : 0.0,
                                   old_total_dur);
                }
                return false;
            }

            const double old_local_t =
                    std::clamp(commit_wt - old_start_wt, 0.0, old_total_dur);
            const auto activity =
                    evaluateTrackingTrajectoryActivity(old_pos_traj,
                                                       old_local_t,
                                                       target_prediction,
                                                       cfg_.tracking_keep_old_horizon,
                                                       cfg_.tracking_keep_old_safety_dt);
            if (!activity.valid || !activity.safe || !activity.active) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_INACTIVE reason={}, activity_reason={}, old_local_t={:.3f}, old_remaining={:.3f}, old_speed0={:.3f}, old_displacement={:.3f}, old_progress={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                                   reason,
                                   activity.reason,
                                   old_local_t,
                                   activity.remaining,
                                   activity.speed0,
                                   activity.displacement,
                                   activity.progress,
                                   activity.expected_progress,
                                   activity.avg_tracking_error);
                }
                return false;
            }

            std::string old_fov_reason;
            if (!trackingSnapshotSatisfiesFovForKeepOld(old_pos_traj,
                                                        old_yaw_traj,
                                                        old_local_t,
                                                        target_prediction,
                                                        &old_fov_reason)) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_FOV reason={}, fov_reason={}, old_local_t={:.3f}, old_remaining={:.3f}",
                                   reason,
                                   old_fov_reason,
                                   old_local_t,
                                   activity.remaining);
                }
                return false;
            }

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
                    const bool candidate_fov_ok,
                    const double prefix_duration,
                    const double runtime_eval_start,
                    const int anti_rollback_worse_count,
                    const double anti_rollback_max_regression,
                    const std::string &candidate_safe_reason,
                    const std::string &tag) {
            if (!cfg_.print_log) {
                return;
            }
            const double guard_h =
                    std::min(cfg_.tracking_no_motion_check_horizon,
                             std::max(0.0, candidate.getTotalDuration() -
                                            std::clamp(runtime_eval_start,
                                                       0.0,
                                                       candidate.getTotalDuration())));
            const TrackingMotionMetrics candidate_metrics =
                    computeTrackingMotionMetrics(candidate,
                                                 target_prediction,
                                                 cfg_,
                                                 runtime_eval_start,
                                                 runtime_eval_start,
                                                 guard_h);
            const char *log_name = "TRACKING_MANAGER_DECISION_REJECT";
            if (decision.type == TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE) {
                log_name = "TRACKING_MANAGER_DECISION_COMMIT";
            } else if (decision.type ==
                       TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE) {
                log_name = "TRACKING_MANAGER_DECISION_FORCE_COMMIT";
            } else if (decision.type == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
                log_name = "TRACKING_MANAGER_DECISION_KEEP_OLD";
            }
            ros_ptr_->info(" -- [Tracking] {} tag={}, decision={}, reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, candidate_safe={}, candidate_safe_reason={}, candidate_commandable={}, candidate_fov_ok={}, anti_rollback_pass={}, anti_rollback_worse_count={}, anti_rollback_max_regression={:.3f}, bypass_anti_rollback={}, candidate_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, target_speed_xy={:.3f}, target_speed_z={:.3f}, target_speed_3d={:.3f}, old_remaining={:.3f}, old_activity_reason={}, old_speed_xy={:.3f}, old_speed_z={:.3f}, old_speed_3d={:.3f}, old_disp_xy={:.3f}, old_disp_z={:.3f}, old_disp_3d={:.3f}, old_progress_xy={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}, keep_old_count={}, reject_count={}",
                           log_name,
                           tag,
                           decisionTypeName(decision.type),
                           decision.reason,
                           last_tracking_diag_guide_path_size_,
                           last_tracking_diag_sfc_size_,
                           last_tracking_diag_target_prediction_size_,
                           last_tracking_diag_out_traj_duration_,
                           decision.candidate_safe,
                           candidate_safe_reason,
                           decision.candidate_commandable,
                           candidate_fov_ok,
                           anti_rollback_pass,
                           anti_rollback_worse_count,
                           anti_rollback_max_regression,
                           decision.bypass_anti_rollback,
                           candidate.getTotalDuration(),
                           prefix_duration,
                           runtime_eval_start,
                           candidate_metrics.displacement_xy,
                           candidate_metrics.displacement_z,
                           candidate_metrics.displacement_3d,
                           candidate_metrics.speed_xy,
                           candidate_metrics.speed_z,
                           candidate_metrics.speed_3d,
                           candidate_metrics.target_speed_xy,
                           candidate_metrics.target_speed_z,
                           candidate_metrics.target_speed_3d,
                           decision.old_activity.remaining,
                           decision.old_activity.reason,
                           decision.old_activity.speed_xy,
                           decision.old_activity.speed_z,
                           decision.old_activity.speed_3d,
                           decision.old_activity.displacement_xy,
                           decision.old_activity.displacement_z,
                           decision.old_activity.displacement_3d,
                           decision.old_activity.progress_xy,
                           decision.old_activity.progress_3d,
                           decision.old_activity.expected_progress,
                           decision.old_activity.avg_tracking_error,
                           tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld() : tracking_consecutive_keep_old_,
                           tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject() : tracking_consecutive_reject_);
            if (decision.candidate_safe && !decision.candidate_commandable) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_NO_MOTION reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, candidate_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, target_speed_z={:.3f}, old_remaining={:.3f}, old_activity_reason={}, old_speed_3d={:.3f}, old_disp_3d={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               decision.reason,
                               last_tracking_diag_guide_path_size_,
                               last_tracking_diag_sfc_size_,
                               last_tracking_diag_target_prediction_size_,
                               last_tracking_diag_out_traj_duration_,
                               candidate.getTotalDuration(),
                               prefix_duration,
                               runtime_eval_start,
                               candidate_metrics.displacement_xy,
                               candidate_metrics.displacement_z,
                               candidate_metrics.displacement_3d,
                               candidate_metrics.speed_xy,
                               candidate_metrics.speed_z,
                               candidate_metrics.speed_3d,
                               candidate_metrics.target_speed_z,
                               decision.old_activity.remaining,
                               decision.old_activity.reason,
                               decision.old_activity.speed_3d,
                               decision.old_activity.displacement_3d,
                               decision.old_activity.progress_3d,
                               decision.old_activity.expected_progress,
                               decision.old_activity.avg_tracking_error);
            }
            if (!decision.candidate_safe) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_UNSAFE reason={}, candidate_safe_reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, candidate_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, old_remaining={:.3f}, old_activity_reason={}",
                               decision.reason,
                               candidate_safe_reason,
                               last_tracking_diag_guide_path_size_,
                               last_tracking_diag_sfc_size_,
                               last_tracking_diag_target_prediction_size_,
                               last_tracking_diag_out_traj_duration_,
                               candidate.getTotalDuration(),
                               prefix_duration,
                               runtime_eval_start,
                               candidate_metrics.displacement_xy,
                               candidate_metrics.displacement_z,
                               candidate_metrics.displacement_3d,
                               decision.old_activity.remaining,
                               decision.old_activity.reason);
            }
            if (!anti_rollback_pass &&
                decision.type == TrackingRuntimeManager::DecisionType::KEEP_OLD) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_ANTI_ROLLBACK reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_fov_ok={}, worse_count={}, max_regression={:.3f}, candidate_disp_3d={:.3f}, candidate_progress_3d={:.3f}, old_remaining={:.3f}, old_activity_reason={}, old_speed_3d={:.3f}, old_disp_3d={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               decision.reason,
                               last_tracking_diag_guide_path_size_,
                               last_tracking_diag_sfc_size_,
                               last_tracking_diag_target_prediction_size_,
                               last_tracking_diag_out_traj_duration_,
                               prefix_duration,
                               runtime_eval_start,
                               candidate_fov_ok,
                               anti_rollback_worse_count,
                               anti_rollback_max_regression,
                               candidate_metrics.displacement_3d,
                               candidate_metrics.progress_3d,
                               decision.old_activity.remaining,
                               decision.old_activity.reason,
                               decision.old_activity.speed_3d,
                               decision.old_activity.displacement_3d,
                               decision.old_activity.progress_3d,
                               decision.old_activity.expected_progress,
                               decision.old_activity.avg_tracking_error);
            }
        };

        auto applyRuntimeDecision =
                [&](const Trajectory &candidate,
                    const std::string &tag,
                    const double candidate_eval_start_t,
                    const double target_eval_start_t,
                    const bool candidate_fov_ok) -> TrackingRuntimeManager::DecisionType {
            if (!cfg_.tracking_runtime_manager_enable || !tracking_runtime_manager_) {
                int worse_count = 0;
                double max_regression = 0.0;
                std::string anti_reason;
                const bool pass =
                        trackingCommitPassesAntiRollback(candidate,
                                                         target_prediction,
                                                         commit_wt,
                                                         candidate_eval_start_t,
                                                         target_eval_start_t,
                                                         true,
                                                         candidate_fov_ok,
                                                         &worse_count,
                                                         &max_regression,
                                                         &anti_reason);
                if (!pass) {
                    setTrackingCommitRejectInfo(
                            anti_reason.empty() ? "anti-rollback rejected candidate" : anti_reason,
                            fmt::format(
                                    "decision=REJECT_AND_FAIL|failure=anti_rollback|anti_rollback_pass=0|anti_rollback_reason={}|worse_count={}|max_regression={:.3f}|candidate_eval_start_t={:.3f}|target_eval_start_t={:.3f}|candidate_duration={:.3f}",
                                    anti_reason.empty() ? "none" : anti_reason,
                                    worse_count,
                                    max_regression,
                                    candidate_eval_start_t,
                                    target_eval_start_t,
                                    candidate.getTotalDuration()));
                }
                return pass ? TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE
                            : TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
            }

            const bool has_old_tracking =
                    allow_old_prefix &&
                    old_tracking_active_for_prefix;
            const double old_local_t =
                    has_old_tracking
                        ? std::clamp(commit_wt - old_start_wt, 0.0, old_total_dur)
                        : 0.0;
            std::string candidate_safe_reason;
            std::string candidate_safe_detail;
            const double candidate_total = candidate.getTotalDuration();
            const double candidate_safety_start =
                    std::clamp(candidate_eval_start_t, 0.0, candidate_total);
            const double candidate_safety_horizon =
                    std::min(cfg_.tracking_keep_old_horizon,
                             std::max(0.0, candidate_total - candidate_safety_start));
            const bool candidate_safe =
                    trackingTrajectorySafeForHorizonDetailed(
                            candidate,
                            candidate_safety_start,
                            candidate_safety_horizon,
                            cfg_.tracking_keep_old_safety_dt,
                            &candidate_safe_reason,
                            &candidate_safe_detail);
            int anti_rollback_worse_count = 0;
            double anti_rollback_max_regression = 0.0;
            std::string anti_rollback_reason;
            const bool anti_rollback_pass =
                    has_old_tracking
                        ? trackingCommitPassesAntiRollback(candidate,
                                                           target_prediction,
                                                           commit_wt,
                                                           candidate_eval_start_t,
                                                           target_eval_start_t,
                                                           candidate_safe,
                                                           candidate_fov_ok,
                                                           &anti_rollback_worse_count,
                                                           &anti_rollback_max_regression,
                                                           &anti_rollback_reason)
                        : true;
            auto decision =
                    tracking_runtime_manager_->decide(has_old_tracking ? &old_pos_traj : nullptr,
                                                      old_local_t,
                                                      candidate,
                                                      target_prediction,
                                                      candidate_safe,
                                                      anti_rollback_pass,
                                                      candidate_eval_start_t,
                                                      target_eval_start_t);
            if (!candidate_safe && !candidate_safe_reason.empty()) {
                decision.reason = decision.reason.empty()
                                      ? candidate_safe_reason
                                      : decision.reason + ": " + candidate_safe_reason;
            }
            if (!anti_rollback_pass && !anti_rollback_reason.empty()) {
                decision.reason = decision.reason.empty()
                                      ? anti_rollback_reason
                                      : decision.reason + ": " + anti_rollback_reason;
            }
            const std::string runtime_decision_detail = fmt::format(
                    "tag={}|decision={}|has_old_tracking={}|candidate_safe={}|candidate_commandable={}|candidate_fov_ok={}|bypass_anti_rollback={}|candidate_safe_reason={}|candidate_safe_detail={}|anti_rollback_pass={}|anti_rollback_reason={}|anti_rollback_worse_count={}|anti_rollback_max_regression={:.3f}|candidate_eval_start_t={:.3f}|target_eval_start_t={:.3f}|candidate_duration={:.3f}|candidate_safety_start={:.3f}|candidate_safety_horizon={:.3f}|old_local_t={:.3f}",
                    tag,
                    decisionTypeName(decision.type),
                    static_cast<int>(has_old_tracking),
                    static_cast<int>(candidate_safe),
                    static_cast<int>(decision.candidate_commandable),
                    static_cast<int>(candidate_fov_ok),
                    static_cast<int>(decision.bypass_anti_rollback),
                    candidate_safe_reason.empty() ? "none" : candidate_safe_reason,
                    candidate_safe_detail.empty() ? "none" : candidate_safe_detail,
                    static_cast<int>(anti_rollback_pass),
                    anti_rollback_reason.empty() ? "none" : anti_rollback_reason,
                    anti_rollback_worse_count,
                    anti_rollback_max_regression,
                    candidate_eval_start_t,
                    target_eval_start_t,
                    candidate.getTotalDuration(),
                    candidate_safety_start,
                    candidate_safety_horizon,
                    old_local_t);
            logRuntimeDecision(decision,
                               candidate,
                               anti_rollback_pass,
                               candidate_fov_ok,
                               candidate_eval_start_t,
                               candidate_eval_start_t,
                               anti_rollback_worse_count,
                               anti_rollback_max_regression,
                               candidate_safe_reason,
                               tag);

            switch (decision.type) {
                case TrackingRuntimeManager::DecisionType::COMMIT_CANDIDATE:
                case TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE:
                    return decision.type;
                case TrackingRuntimeManager::DecisionType::KEEP_OLD:
                    if (keepOldFromSnapshot(decision.reason)) {
                        tracking_runtime_manager_->onKeepOld();
                        return TrackingRuntimeManager::DecisionType::KEEP_OLD;
                    }
                    if (decision.candidate_safe && decision.candidate_commandable) {
                        if (cfg_.print_log) {
                            ros_ptr_->warn(" -- [Tracking] TRACKING_FORCE_COMMIT_SAFE_RECOVERY reason=old_tracking_not_legally_keepable, original_decision={}, original_reason={}, prefix_duration={:.3f}, runtime_eval_start={:.3f}",
                                           decisionTypeName(decision.type),
                                           decision.reason,
                                           candidate_eval_start_t,
                                           candidate_eval_start_t);
                        }
                        return TrackingRuntimeManager::DecisionType::FORCE_COMMIT_CANDIDATE;
                    }
                    setTrackingCommitRejectInfo(decision.reason, runtime_decision_detail);
                    tracking_runtime_manager_->onRejected();
                    return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
                case TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL:
                    setTrackingCommitRejectInfo(decision.reason, runtime_decision_detail);
                    tracking_runtime_manager_->onRejected();
                    return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
            }
            setTrackingCommitRejectInfo("runtime decision fell through", runtime_decision_detail);
            tracking_runtime_manager_->onRejected();
            return TrackingRuntimeManager::DecisionType::REJECT_AND_FAIL;
        };

        Trajectory yaw_traj = optimized_yaw_traj;
        if (yaw_traj.empty() && !buildTrackingTargetYawTrajectory(pos_traj, target_prediction, yaw_traj)) {
            setTrackingCommitRejectInfo(
                    "yaw generation failed",
                    fmt::format("failure=yaw_generation_failed|candidate_duration={:.3f}|target_prediction_size={}",
                                pos_traj.getTotalDuration(),
                                target_prediction.size()));
            ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_FOV reason=yaw_generation_failed");
            if (keepOldFromSnapshot("tracking yaw generation failed")) {
                if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                    tracking_runtime_manager_->onKeepOld();
                }
                return true;
            }
            if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                tracking_runtime_manager_->onRejected();
            }
            return false;
        }

        Trajectory committed_pos_traj = pos_traj;
        Trajectory committed_yaw_traj = yaw_traj;
        committed_pos_traj.start_WT = commit_wt;
        committed_yaw_traj.start_WT = commit_wt;
        double stitched_prefix_duration = 0.0;
        if (should_stitch_old_prefix) {
            const bool fixed_head_time_valid =
                    std::isfinite(candidate_head_wt) &&
                    candidate_head_wt > commit_wt + 1.0e-4;
            const double prefix_end_wt =
                    fixed_head_time_valid
                        ? candidate_head_wt
                        : commit_wt + std::max(0.0, cfg_.replan_forward_dt);
            const double prefix_start_t = commit_wt - old_start_wt;
            const double prefix_end_t = prefix_end_wt - old_start_wt;
            const bool prefix_window_valid =
                    !old_pos_traj.empty() &&
                    !old_yaw_traj.empty() &&
                    fixed_head_time_valid &&
                    prefix_start_t >= 0.0 &&
                    prefix_end_t > prefix_start_t + 1.0e-4 &&
                    prefix_end_t <= old_total_dur + 1.0e-6;

            if (prefix_window_valid) {
                Trajectory prefix_pos_traj;
                Trajectory prefix_yaw_traj;
                const double clipped_prefix_end_t = std::min(prefix_end_t, old_total_dur);
                const double prefix_duration = clipped_prefix_end_t - prefix_start_t;
                bool used_sampled_yaw_prefix = false;
                const bool prefix_pos_ok =
                        old_pos_traj.getPartialTrajectoryByTime(prefix_start_t,
                                                                clipped_prefix_end_t,
                                                                prefix_pos_traj);
                const bool prefix_yaw_ok =
                        prefix_pos_ok &&
                        extractYawPrefixForStitching(old_yaw_traj,
                                                     prefix_start_t,
                                                     prefix_duration,
                                                     prefix_yaw_traj,
                                                     used_sampled_yaw_prefix);
                if (prefix_pos_ok && prefix_yaw_ok) {
                    committed_pos_traj = prefix_pos_traj + pos_traj;
                    committed_yaw_traj = prefix_yaw_traj + yaw_traj;
                    stitched_prefix_duration = prefix_pos_traj.getTotalDuration();
                    committed_pos_traj.start_WT = commit_wt;
                    committed_yaw_traj.start_WT = commit_wt;

                    if (cfg_.print_log) {
                        const StatePVAJ old_tail = prefix_pos_traj.getState(prefix_pos_traj.getTotalDuration());
                        const StatePVAJ new_head = pos_traj.getState(0.0);
                        const double pos_jump = (old_tail.col(0) - new_head.col(0)).norm();
                        const double vel_jump = (old_tail.col(1) - new_head.col(1)).norm();
                        const double acc_jump = (old_tail.col(2) - new_head.col(2)).norm();
                        const double jerk_jump = (old_tail.col(3) - new_head.col(3)).norm();
                        ros_ptr_->info(" -- [GeneralPlanner] Tracking replan stitched old prefix: dt={:.3f}s, start_t={:.3f}, end_t={:.3f}, candidate_head_wt={:.3f}, commit_wt={:.3f}, pos_jump={:.4f}, vel_jump={:.4f}, acc_jump={:.4f}, jerk_jump={:.4f}, sampled_yaw_prefix={}.",
                                       prefix_pos_traj.getTotalDuration(),
                                       prefix_start_t,
                                       clipped_prefix_end_t,
                                       candidate_head_wt,
                                       commit_wt,
                                       pos_jump,
                                       vel_jump,
                                       acc_jump,
                                       jerk_jump,
                                       used_sampled_yaw_prefix);
                    }
                } else {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [GeneralPlanner] Tracking replan prefix extraction failed: pos_ok={}, yaw_ok={}, start_t={:.3f}, end_t={:.3f}, candidate_head_wt={:.3f}, commit_wt={:.3f}.",
                                       prefix_pos_ok,
                                       prefix_yaw_ok,
                                       prefix_start_t,
                                       clipped_prefix_end_t,
                                       candidate_head_wt,
                                       commit_wt);
                    }
                    if (keepOldFromSnapshot("tracking replan prefix extraction failed")) {
                        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                            tracking_runtime_manager_->onKeepOld();
                        }
                        return true;
                    }
                    setTrackingCommitRejectInfo(
                            "tracking replan prefix extraction failed",
                            fmt::format("failure=prefix_extract_failed|pos_ok={}|yaw_ok={}|prefix_start_t={:.3f}|prefix_end_t={:.3f}|candidate_head_wt={:.3f}|commit_wt={:.3f}",
                                        static_cast<int>(prefix_pos_ok),
                                        static_cast<int>(prefix_yaw_ok),
                                        prefix_start_t,
                                        clipped_prefix_end_t,
                                        candidate_head_wt,
                                        commit_wt));
                    return false;
                }
            } else {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [GeneralPlanner] Tracking replan prefix unavailable: start_t={:.3f}, end_t={:.3f}, total={:.3f}, fixed_head_time_valid={}, candidate_head_wt={:.3f}, commit_wt={:.3f}.",
                                   prefix_start_t,
                                   prefix_end_t,
                                   old_total_dur,
                                   fixed_head_time_valid,
                                   candidate_head_wt,
                                   commit_wt);
                }
                if (keepOldFromSnapshot(fixed_head_time_valid
                                            ? "tracking replan prefix unavailable"
                                            : "tracking replan head time stale")) {
                    if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                        tracking_runtime_manager_->onKeepOld();
                    }
                    return true;
                }
                setTrackingCommitRejectInfo(
                        fixed_head_time_valid
                            ? "tracking replan prefix unavailable"
                            : "tracking replan head time stale",
                        fmt::format("failure=prefix_unavailable|prefix_start_t={:.3f}|prefix_end_t={:.3f}|old_total_dur={:.3f}|fixed_head_time_valid={}|candidate_head_wt={:.3f}|commit_wt={:.3f}",
                                    prefix_start_t,
                                    prefix_end_t,
                                    old_total_dur,
                                    static_cast<int>(fixed_head_time_valid),
                                    candidate_head_wt,
                                    commit_wt));
                return false;
            }
        }
        committed_pos_traj.start_WT = commit_wt;
        committed_yaw_traj.start_WT = commit_wt;

        const bool should_check_fov =
                cfg_.tracking_fov_commit_check_enable &&
                cfg_.tracking_fov_check_strict &&
                (cfg_.tracking_fov_check_first_commit || runtime_has_committed_tracking);
        bool candidate_fov_ok_for_commit = true;
        if (should_check_fov) {
            std::string fov_reject_reason;
            const double fov_start_t = 0.0;
            const double fov_horizon =
                    std::min({std::max(0.0, cfg_.tracking_keep_old_horizon),
                              committed_pos_traj.getTotalDuration(),
                              target_prediction.empty() ? 0.0
                                                        : std::max(0.0, target_prediction.back().t)});
            if (!trackingTrajectorySatisfiesFov(committed_pos_traj,
                                                committed_yaw_traj,
                                                target_prediction,
                                                fov_start_t,
                                                fov_horizon,
                                                cfg_.tracking_fov_check_dt,
                                                0.0,
                                                &fov_reject_reason,
                                                false,
                                                allow_reacquire_fov_relax)) {
                Trajectory target_yaw_traj;
                std::string rebuilt_yaw_fov_reason;
                const bool rebuilt_yaw_generated =
                        buildTrackingTargetYawTrajectory(committed_pos_traj,
                                                         target_prediction,
                                                         target_yaw_traj);
                const bool rebuilt_yaw_ok =
                        rebuilt_yaw_generated &&
                        trackingTrajectorySatisfiesFov(committed_pos_traj,
                                                       target_yaw_traj,
                                                       target_prediction,
                                                       fov_start_t,
                                                       fov_horizon,
                                                       cfg_.tracking_fov_check_dt,
                                                       0.0,
                                                       &rebuilt_yaw_fov_reason,
                                                       false,
                                                       allow_reacquire_fov_relax);
                if (rebuilt_yaw_ok) {
                    committed_yaw_traj = target_yaw_traj;
                    committed_yaw_traj.start_WT = commit_wt;
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_YAW_REBUILT_FOR_FOV reason={}, horizon={:.3f}, reacquire_fov_relax={}",
                                       fov_reject_reason,
                                       fov_horizon,
                                       allow_reacquire_fov_relax);
                    }
                } else if (allow_reacquire_fov_relax) {
                    candidate_fov_ok_for_commit = false;
                    if (rebuilt_yaw_generated && !target_yaw_traj.empty()) {
                        committed_yaw_traj = target_yaw_traj;
                        committed_yaw_traj.start_WT = commit_wt;
                    }
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_FOV_DEGRADED_ACCEPT reason={}, target_yaw_fallback={}, start_t={:.3f}, horizon={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}",
                                       fov_reject_reason,
                                       rebuilt_yaw_generated
                                           ? (rebuilt_yaw_fov_reason.empty()
                                                  ? "failed_without_reason"
                                                  : rebuilt_yaw_fov_reason)
                                           : "generation_failed",
                                       fov_start_t,
                                       fov_horizon,
                                       stitched_prefix_duration,
                                       stitched_prefix_duration,
                                       last_tracking_diag_guide_path_size_,
                                       last_tracking_diag_sfc_size_,
                                       last_tracking_diag_target_prediction_size_,
                                       last_tracking_diag_out_traj_duration_);
                    }
                } else {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_FOV reason={}, start_t={:.3f}, horizon={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, reacquire_fov_relax={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}",
                                       fov_reject_reason,
                                       fov_start_t,
                                       fov_horizon,
                                       stitched_prefix_duration,
                                       stitched_prefix_duration,
                                       allow_reacquire_fov_relax,
                                       last_tracking_diag_guide_path_size_,
                                       last_tracking_diag_sfc_size_,
                                       last_tracking_diag_target_prediction_size_,
                                       last_tracking_diag_out_traj_duration_);
                    }
                    setTrackingCommitRejectInfo(
                            "candidate FOV rejected: " + fov_reject_reason +
                            "; target_yaw_fallback=" +
                            (rebuilt_yaw_fov_reason.empty() ? "failed" : rebuilt_yaw_fov_reason),
                            fmt::format(
                                    "failure=fov_rejected|fov_start_t={:.3f}|fov_horizon={:.3f}|prefix_duration={:.3f}|reacquire_fov_relax={}|candidate_duration={:.3f}|target_prediction_size={}|target_yaw_fallback_reason={}",
                                    fov_start_t,
                                    fov_horizon,
                                    stitched_prefix_duration,
                                    static_cast<int>(allow_reacquire_fov_relax),
                                    committed_pos_traj.getTotalDuration(),
                                    target_prediction.size(),
                                    rebuilt_yaw_fov_reason.empty() ? "failed" : rebuilt_yaw_fov_reason));
                    if (keepOldFromSnapshot("tracking candidate rejected by FOV: " + fov_reject_reason)) {
                        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                            tracking_runtime_manager_->onKeepOld();
                        }
                        return true;
                    }
                    if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                        tracking_runtime_manager_->onRejected();
                    }
                    return false;
                }
            }
        }

        const auto runtime_decision =
                applyRuntimeDecision(committed_pos_traj,
                                     "final_commit",
                                     cfg_.tracking_anti_rollback_eval_after_prefix
                                         ? stitched_prefix_duration
                                         : 0.0,
                                     cfg_.tracking_anti_rollback_eval_after_prefix
                                         ? stitched_prefix_duration
                                         : 0.0,
                                     candidate_fov_ok_for_commit);
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
                             std::max(0.0, committed_pos_traj.getTotalDuration() -
                                            stitched_prefix_duration));
            const TrackingMotionMetrics committed_metrics =
                    computeTrackingMotionMetrics(committed_pos_traj,
                                                 target_prediction,
                                                 cfg_,
                                                 stitched_prefix_duration,
                                                 stitched_prefix_duration,
                                                 guard_h);
            const double now_minus_start_wt = ros_ptr_->getSimTime() - committed_pos_traj.start_WT;
            ros_ptr_->info(" -- [Tracking] TRACKING_CANDIDATE_COMMITTED candidate_duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, candidate_head_wt={:.3f}, commit_wt={:.3f}, head_lag={:.3f}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, target_speed_z={:.3f}, now_minus_start_WT={:.3f}, committed_total_duration={:.3f}",
                           committed_pos_traj.getTotalDuration(),
                           stitched_prefix_duration,
                           stitched_prefix_duration,
                           candidate_head_wt,
                           commit_wt,
                           candidate_head_wt - commit_wt,
                           committed_metrics.displacement_xy,
                           committed_metrics.displacement_z,
                           committed_metrics.displacement_3d,
                           committed_metrics.speed_xy,
                           committed_metrics.speed_z,
                           committed_metrics.speed_3d,
                           committed_metrics.target_speed_z,
                           now_minus_start_wt,
                           committed_pos_traj.getTotalDuration());

            const double short_h = std::min(0.15, committed_pos_traj.getTotalDuration());
            const TrackingMotionMetrics short_metrics =
                    computeTrackingMotionMetrics(committed_pos_traj,
                                                 target_prediction,
                                                 cfg_,
                                                 stitched_prefix_duration,
                                                 stitched_prefix_duration,
                                                 short_h);
            if (short_metrics.target_moving &&
                short_metrics.displacement_3d < cfg_.tracking_no_motion_min_displacement &&
                short_metrics.displacement_z < cfg_.tracking_no_motion_min_displacement_z &&
                short_metrics.speed_3d < cfg_.tracking_keep_old_min_speed) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_COMMITTED_BUT_NO_MOTION start_WT={:.3f}, now={:.3f}, duration={:.3f}, prefix_duration={:.3f}, runtime_eval_start={:.3f}, now_minus_start_WT={:.3f}, speed_3d={:.3f}, displacement_3d={:.3f}, displacement_z={:.3f}",
                               committed_pos_traj.start_WT,
                               ros_ptr_->getSimTime(),
                               committed_pos_traj.getTotalDuration(),
                               stitched_prefix_duration,
                               stitched_prefix_duration,
                               now_minus_start_wt,
                               short_metrics.speed_3d,
                               short_metrics.displacement_3d,
                               short_metrics.displacement_z);
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
                    dist < trackingHardSafeDistance(cfg_)) {
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
        const Vec3f target_dir =
                trackingTargetDirection(target_prediction,
                                        cfg_.tracking_no_motion_target_speed_threshold,
                                        cfg_.tracking_vertical_motion_threshold,
                                        cfg_.tracking_motion_3d_enable);

        Vec3f last_p = traj.getPos(std::clamp(local_start_t, 0.0, total_dur));
        if (!last_p.allFinite()) {
            out.reason = "non-finite initial point";
            return out;
        }

        const TrackingMotionMetrics initial_metrics =
                computeTrackingMotionMetrics(traj,
                                             target_prediction,
                                             cfg_,
                                             local_start_t,
                                             0.0,
                                             eval_horizon);
        out.target_moving = initial_metrics.target_moving;
        out.target_vertical_moving = initial_metrics.target_vertical_moving;
        out.speed_xy = initial_metrics.speed_xy;
        out.speed_z = initial_metrics.speed_z;
        out.speed_3d = initial_metrics.speed_3d;
        out.speed0 = out.speed_xy;
        out.target_speed_xy = initial_metrics.target_speed_xy;
        out.target_speed_z = initial_metrics.target_speed_z;
        out.target_speed_3d = initial_metrics.target_speed_3d;
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
                out.displacement_xy += dp.head<2>().norm();
                out.displacement_z += std::abs(dp.z());
                out.displacement_3d += dp.norm();
                out.progress_xy += dp.head<2>().dot(target_dir.head<2>());
                out.progress_3d += dp.dot(target_dir);
            }
            last_p = p;
        }
        out.displacement = out.displacement_xy;
        out.progress = out.progress_xy;

        out.avg_tracking_error =
                sample_count > 0 ? total_error / static_cast<double>(sample_count) : 0.0;
        out.tracking_error =
                trackingDistanceError(traj.getPos(std::clamp(local_start_t, 0.0, total_dur)),
                                      target0.position,
                                      cfg_.tracking_distance,
                                      cfg_.tracking_height_offset);

        if (out.target_moving) {
            const bool use_3d_motion =
                    cfg_.tracking_motion_3d_enable || out.target_vertical_moving;
            const double active_speed =
                    use_3d_motion ? out.speed_3d : out.speed_xy;
            const double active_displacement =
                    use_3d_motion ? out.displacement_3d : out.displacement_xy;
            const double active_progress =
                    use_3d_motion ? out.progress_3d : out.progress_xy;
            const double target_speed =
                    use_3d_motion ? out.target_speed_3d : out.target_speed_xy;
            const double progress_ratio =
                    use_3d_motion ? cfg_.tracking_keep_old_min_progress_3d_ratio
                                  : cfg_.tracking_keep_old_min_progress_ratio;

            if (active_speed < cfg_.tracking_keep_old_min_speed) {
                out.reason = "speed too small";
                return out;
            }

            if (active_displacement < cfg_.tracking_keep_old_min_displacement &&
                (!out.target_vertical_moving ||
                 out.displacement_z < cfg_.tracking_no_motion_min_displacement_z)) {
                out.reason = "displacement too small";
                return out;
            }

            out.expected_progress =
                    target_speed *
                    eval_horizon *
                    progress_ratio;
            if (active_progress < out.expected_progress) {
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
            const double candidate_eval_start_t,
            const double target_eval_start_t,
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

        const double h =
                std::min(cfg_.tracking_no_motion_check_horizon,
                         std::max(0.0, candidate_pos_traj.getTotalDuration() -
                                        std::clamp(candidate_eval_start_t,
                                                   0.0,
                                                   candidate_pos_traj.getTotalDuration())));
        if (h < 1.0e-3) {
            if (reason) {
                *reason = "candidate duration too short";
            }
            return false;
        }

        const double start_t =
                std::clamp(candidate_eval_start_t, 0.0, candidate_pos_traj.getTotalDuration());
        const Vec3f p0 = candidate_pos_traj.getPos(start_t);
        const Vec3f p1 = candidate_pos_traj.getPos(start_t + h);
        const Vec3f v0 = candidate_pos_traj.getVel(start_t);
        if (!p0.allFinite() || !p1.allFinite() || !v0.allFinite()) {
            if (reason) {
                *reason = "candidate contains non-finite state";
            }
            return false;
        }

        const TrackingMotionMetrics metrics =
                computeTrackingMotionMetrics(candidate_pos_traj,
                                             target_prediction,
                                             cfg_,
                                             start_t,
                                             target_eval_start_t,
                                             h);
        if (!metrics.target_moving) {
            return true;
        }

        const bool use_3d_motion =
                cfg_.tracking_motion_3d_enable || metrics.target_vertical_moving;
        const bool commandable =
                use_3d_motion
                    ? (metrics.displacement_3d >= cfg_.tracking_no_motion_min_displacement ||
                       metrics.displacement_z >= cfg_.tracking_no_motion_min_displacement_z ||
                       metrics.speed_3d >= cfg_.tracking_keep_old_min_speed)
                    : (metrics.displacement_xy >= cfg_.tracking_no_motion_min_displacement ||
                       metrics.speed_xy >= cfg_.tracking_keep_old_min_speed);
        if (!commandable) {
            if (reason) {
                *reason = fmt::format("candidate no-motion: disp_xy={:.3f}, disp_z={:.3f}, disp_3d={:.3f}, speed_xy={:.3f}, speed_z={:.3f}, speed_3d={:.3f}, target_speed_xy={:.3f}, target_speed_z={:.3f}, target_speed_3d={:.3f}",
                                      metrics.displacement_xy,
                                      metrics.displacement_z,
                                      metrics.displacement_3d,
                                      metrics.speed_xy,
                                      metrics.speed_z,
                                      metrics.speed_3d,
                                      metrics.target_speed_xy,
                                      metrics.target_speed_z,
                                      metrics.target_speed_3d);
            }
            return false;
        }

        return true;
    }

    bool GeneralPlanner::keepOldTrackingTrajectoryIfActive(
            const traj_opt::DynamicTargetStates &target_prediction,
            const std::string &reason) {
        auto keepOldReason = [this](const std::string &status) {
            return fmt::format("{};trigger_phase={};trigger_reason={}",
                               status,
                               last_tracking_diag_phase_,
                               last_tracking_diag_reason_);
        };
        auto tryShortSafetyGrace =
                [&](const Trajectory &old_pos_traj,
                    const Trajectory &old_yaw_traj,
                    const double old_local_t,
                    const auto &activity,
                    const bool runtime_managed) -> bool {
            if (!cfg_.tracking_keep_old_short_safety_grace_enable ||
                old_pos_traj.empty() ||
                target_prediction.empty()) {
                return false;
            }
            if (activity.remaining < cfg_.tracking_keep_old_min_remaining) {
                return false;
            }
            const int consecutive_keep_old =
                    runtime_managed && tracking_runtime_manager_
                        ? tracking_runtime_manager_->consecutiveKeepOld()
                        : tracking_consecutive_keep_old_;
            if (cfg_.tracking_max_consecutive_keep_old > 0 &&
                consecutive_keep_old >= cfg_.tracking_max_consecutive_keep_old) {
                return false;
            }

            const double grace_horizon =
                    std::min({std::max(0.0, cfg_.tracking_keep_old_short_safety_grace_horizon),
                              std::max(0.0, activity.remaining),
                              std::max(0.0, old_pos_traj.getTotalDuration() - old_local_t),
                              std::max(0.0, target_prediction.back().t)});
            if (grace_horizon < 1.0e-3) {
                return false;
            }
            if (!trackingTrajectorySafeForHorizon(old_pos_traj,
                                                  old_local_t,
                                                  grace_horizon,
                                                  cfg_.tracking_keep_old_safety_dt)) {
                return false;
            }

            std::string old_fov_reason;
            const bool old_fov_ok =
                    !cfg_.tracking_keep_old_requires_fov ||
                    (!old_yaw_traj.empty() &&
                     trackingTrajectorySatisfiesFov(old_pos_traj,
                                                    old_yaw_traj,
                                                    target_prediction,
                                                    old_local_t,
                                                    grace_horizon,
                                                    cfg_.tracking_fov_check_dt,
                                                    0.0,
                                                    &old_fov_reason,
                                                    true));
            const bool fov_degraded_grace =
                    !old_fov_ok &&
                    cfg_.tracking_reacquire_fov_relax_enable;
            if (!old_fov_ok && !fov_degraded_grace) {
                return false;
            }

            if (runtime_managed && tracking_runtime_manager_) {
                tracking_runtime_manager_->onKeepOld();
            } else {
                ++tracking_consecutive_keep_old_;
            }
            setTrackingDiagnostic("keep_old",
                                  keepOldReason(fmt::format("{}:{};grace_horizon={:.3f};old_fov_ok={};old_fov_reason={}",
                                                            fov_degraded_grace
                                                                ? "short_safety_grace_fov_degraded"
                                                                : "short_safety_grace",
                                                            activity.reason,
                                                            grace_horizon,
                                                            static_cast<int>(old_fov_ok),
                                                            old_fov_reason.empty() ? "none" : old_fov_reason)),
                                  last_tracking_diag_guide_path_size_,
                                  last_tracking_diag_sfc_size_,
                                  target_prediction.size(),
                                  old_pos_traj.getTotalDuration());
            latest_replan.setExpTraj(old_pos_traj);
            latest_replan.setExpYawTraj(old_yaw_traj);
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] {} reason={}, activity_reason={}, grace_horizon={:.3f}, old_remaining={:.3f}, old_fov_ok={}, old_fov_reason={}, keep_old_count={}, reject_count={}",
                               fov_degraded_grace
                                   ? "TRACKING_KEEP_OLD_SHORT_SAFETY_GRACE_FOV_DEGRADED"
                                   : "TRACKING_KEEP_OLD_SHORT_SAFETY_GRACE",
                               reason,
                               activity.reason,
                               grace_horizon,
                               activity.remaining,
                               old_fov_ok,
                               old_fov_reason.empty() ? "none" : old_fov_reason,
                               runtime_managed && tracking_runtime_manager_
                                   ? tracking_runtime_manager_->consecutiveKeepOld()
                                   : tracking_consecutive_keep_old_,
                               runtime_managed && tracking_runtime_manager_
                                   ? tracking_runtime_manager_->consecutiveReject()
                                   : tracking_consecutive_reject_);
            }
            return true;
        };

        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            if (cmd_traj_info_.empty() ||
                !tracking_runtime_manager_->hasCommittedTracking()) {
                setTrackingDiagnostic("keep_old",
                                      keepOldReason("inactive:no_committed_tracking_trajectory"),
                                      last_tracking_diag_guide_path_size_,
                                      last_tracking_diag_sfc_size_,
                                      target_prediction.size(),
                                      last_tracking_diag_out_traj_duration_);
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
                if (tryShortSafetyGrace(old_pos_traj,
                                        old_yaw_traj,
                                        old_local_t,
                                        activity,
                                        true)) {
                    return true;
                }
                setTrackingDiagnostic("keep_old",
                                      keepOldReason("inactive:" + activity.reason),
                                      last_tracking_diag_guide_path_size_,
                                      last_tracking_diag_sfc_size_,
                                      target_prediction.size(),
                                      last_tracking_diag_out_traj_duration_);
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

            std::string old_fov_reason;
            if (!trackingSnapshotSatisfiesFovForKeepOld(old_pos_traj,
                                                        old_yaw_traj,
                                                        old_local_t,
                                                        target_prediction,
                                                        &old_fov_reason)) {
                if (tryShortSafetyGrace(old_pos_traj,
                                        old_yaw_traj,
                                        old_local_t,
                                        activity,
                                        true)) {
                    return true;
                }
                setTrackingDiagnostic("keep_old",
                                      keepOldReason("fov_rejected:" + old_fov_reason),
                                      last_tracking_diag_guide_path_size_,
                                      last_tracking_diag_sfc_size_,
                                      target_prediction.size(),
                                      last_tracking_diag_out_traj_duration_);
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_FOV reason={}, fov_reason={}, old_local_t={:.3f}, old_remaining={:.3f}, keep_old_count={}, reject_count={}",
                                   reason,
                                   old_fov_reason,
                                   old_local_t,
                                   activity.remaining,
                                   tracking_runtime_manager_->consecutiveKeepOld(),
                                   tracking_runtime_manager_->consecutiveReject());
                }
                return false;
            }

            tracking_runtime_manager_->onKeepOld();
            setTrackingDiagnostic("keep_old",
                                  keepOldReason("active:" + reason),
                                  last_tracking_diag_guide_path_size_,
                                  last_tracking_diag_sfc_size_,
                                  target_prediction.size(),
                                  old_pos_traj.getTotalDuration());
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
            if (!cmd_traj_info_.empty()) {
                cmd_traj_info_.lock();
                const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
                const Trajectory old_yaw_traj = cmd_traj_info_.yawTraj();
                const double old_start_wt = cmd_traj_info_.getStartWallTime();
                const double old_total_dur = cmd_traj_info_.getTotalDuration();
                cmd_traj_info_.unlock();
                const double old_local_t =
                        std::clamp(ros_ptr_->getSimTime() - old_start_wt,
                                   0.0,
                                   old_total_dur);
                if (tryShortSafetyGrace(old_pos_traj,
                                        old_yaw_traj,
                                        old_local_t,
                                        activity,
                                        false)) {
                    return true;
                }
            }
            setTrackingDiagnostic("keep_old",
                                  keepOldReason("inactive:" + activity.reason),
                                  last_tracking_diag_guide_path_size_,
                                  last_tracking_diag_sfc_size_,
                                  target_prediction.size(),
                                  last_tracking_diag_out_traj_duration_);
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

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const Trajectory old_yaw_traj = cmd_traj_info_.yawTraj();
        const double old_start_wt = cmd_traj_info_.getStartWallTime();
        const double old_total_dur = cmd_traj_info_.getTotalDuration();
        cmd_traj_info_.unlock();

        const double old_local_t =
                std::clamp(ros_ptr_->getSimTime() - old_start_wt,
                           0.0,
                           old_total_dur);
        std::string old_fov_reason;
        if (!trackingSnapshotSatisfiesFovForKeepOld(old_pos_traj,
                                                    old_yaw_traj,
                                                    old_local_t,
                                                    target_prediction,
                                                    &old_fov_reason)) {
            if (tryShortSafetyGrace(old_pos_traj,
                                    old_yaw_traj,
                                    old_local_t,
                                    activity,
                                    false)) {
                return true;
            }
            setTrackingDiagnostic("keep_old",
                                  keepOldReason("fov_rejected:" + old_fov_reason),
                                  last_tracking_diag_guide_path_size_,
                                  last_tracking_diag_sfc_size_,
                                  target_prediction.size(),
                                  last_tracking_diag_out_traj_duration_);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_KEEP_OLD_REJECTED_FOV reason={}, fov_reason={}, old_local_t={:.3f}, old_remaining={:.3f}",
                               reason,
                               old_fov_reason,
                               old_local_t,
                               activity.remaining);
            }
            return false;
        }

        ++tracking_consecutive_keep_old_;
        setTrackingDiagnostic("keep_old",
                              keepOldReason("active:" + reason),
                              last_tracking_diag_guide_path_size_,
                              last_tracking_diag_sfc_size_,
                              target_prediction.size(),
                              old_pos_traj.getTotalDuration());
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

        latest_replan.setExpTraj(old_pos_traj);
        latest_replan.setExpYawTraj(old_yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        return true;
    }

    bool GeneralPlanner::trackingTrajectorySafeForHorizonDetailed(
            const Trajectory &traj,
            const double start_t,
            const double horizon,
            const double dt,
            std::string *reason,
            std::string *detail) const {
        setFailureReason(reason, "");
        setFailureReason(detail, "");

        const auto set_reject =
                [&](const std::string &reject_reason,
                    const double sample_dt,
                    const double eval_t,
                    const Vec3f &pos,
                    const Vec3f &prev,
                    const rog_map::GridType grid_type,
                    const bool inside_local_map,
                    const bool line_free,
                    const double esdf_dist) -> bool {
            setFailureReason(reason, reject_reason);
            if (detail != nullptr) {
                *detail = fmt::format(
                        "check_start_t={:.3f}|check_horizon={:.3f}|sample_dt={:.3f}|fail_offset={:.3f}|fail_t={:.3f}|pos={}|prev={}|segment_len={:.3f}|inside_local_map={}|grid={}|unknown_as_occupied={}|esdf_dist={:.3f}|line_free={}",
                        start_t,
                        horizon,
                        sample_dt,
                        eval_t - start_t,
                        eval_t,
                        formatVec3Compact(pos),
                        formatVec3Compact(prev),
                        (pos - prev).norm(),
                        static_cast<int>(inside_local_map),
                        gridTypeName(grid_type),
                        static_cast<int>(cfg_.tracking_unknown_as_occupied),
                        esdf_dist,
                        static_cast<int>(line_free));
            }
            return false;
        };

        if (traj.empty()) {
            setFailureReason(reason, "empty trajectory");
            if (detail != nullptr) {
                *detail = fmt::format("check_start_t={:.3f}|check_horizon={:.3f}|failure=empty_trajectory",
                                      start_t,
                                      horizon);
            }
            return false;
        }
        if (horizon <= 1.0e-6) {
            return true;
        }
        const double total_dur = traj.getTotalDuration();
        if (!std::isfinite(start_t) ||
            start_t < -1.0e-6 ||
            start_t + horizon > total_dur + 1.0e-6) {
            setFailureReason(reason, "invalid safety horizon");
            if (detail != nullptr) {
                *detail = fmt::format(
                        "check_start_t={:.3f}|check_horizon={:.3f}|total_duration={:.3f}|failure=invalid_safety_horizon",
                        start_t,
                        horizon,
                        total_dur);
            }
            return false;
        }
        if (map_manager_ == nullptr || !map_manager_->ready()) {
            return true;
        }

        const double safe_dt = std::max(0.05, dt);
        Vec3f last = traj.getPos(std::clamp(start_t, 0.0, total_dur));
        if (!last.allFinite()) {
            setFailureReason(reason, "non-finite initial point");
            if (detail != nullptr) {
                *detail = fmt::format(
                        "check_start_t={:.3f}|check_horizon={:.3f}|sample_dt={:.3f}|failure=non_finite_initial_point|pos={}",
                        start_t,
                        horizon,
                        safe_dt,
                        formatVec3Compact(last));
            }
            return false;
        }

        for (double offset = 0.0; offset <= horizon + 1.0e-6; offset += safe_dt) {
            const double t = std::clamp(start_t + offset, 0.0, total_dur);
            const Vec3f pos = traj.getPos(t);
            if (!pos.allFinite()) {
                return set_reject("non-finite sample",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  rog_map::GridType::UNDEFINED,
                                  true,
                                  true,
                                  std::numeric_limits<double>::quiet_NaN());
            }
            if (!map_manager_->insideLocalMap(pos)) {
                return set_reject("outside local map",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  rog_map::GridType::OUT_OF_MAP,
                                  false,
                                  true,
                                  std::numeric_limits<double>::quiet_NaN());
            }

            const auto grid_type = map_manager_->getInfGridType(pos);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP) {
                return set_reject("occupied or out-of-map",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  grid_type,
                                  true,
                                  true,
                                  std::numeric_limits<double>::quiet_NaN());
            }
            if (cfg_.tracking_unknown_as_occupied &&
                (grid_type == rog_map::GridType::UNKNOWN ||
                 grid_type == rog_map::GridType::UNDEFINED ||
                 grid_type == rog_map::GridType::FRONTIER)) {
                return set_reject("unknown treated as occupied",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  grid_type,
                                  true,
                                  true,
                                  std::numeric_limits<double>::quiet_NaN());
            }

            double esdf_dist = std::numeric_limits<double>::quiet_NaN();
            if (map_manager_->hasESDF()) {
                Vec3f grad = Vec3f::Zero();
                if (map_manager_->evaluateESDF(pos, esdf_dist, grad) &&
                    esdf_dist < trackingHardSafeDistance(cfg_)) {
                    return set_reject("inside tracking hard safe distance",
                                      safe_dt,
                                      t,
                                      pos,
                                      last,
                                      grid_type,
                                      true,
                                      true,
                                      esdf_dist);
                }
            }

            if ((pos - last).norm() > 1.0e-4 &&
                !map_manager_->isLineFree(last, pos, true, cfg_.tracking_unknown_as_occupied)) {
                return set_reject("segment not line-free",
                                  safe_dt,
                                  t,
                                  pos,
                                  last,
                                  grid_type,
                                  true,
                                  false,
                                  esdf_dist);
            }
            last = pos;
        }
        return true;
    }

    bool GeneralPlanner::trackingCandidateSafeForCommit(const Trajectory &candidate_pos_traj,
                                                        std::string *reason,
                                                        std::string *detail) const {
        if (candidate_pos_traj.empty()) {
            setFailureReason(reason, "empty trajectory");
            setFailureReason(detail, "failure=empty_trajectory");
            return false;
        }
        const double horizon =
                std::min(std::max(0.0, cfg_.tracking_keep_old_horizon),
                         candidate_pos_traj.getTotalDuration());
        return trackingTrajectorySafeForHorizonDetailed(candidate_pos_traj,
                                                        0.0,
                                                        horizon,
                                                        cfg_.tracking_keep_old_safety_dt,
                                                        reason,
                                                        detail);
    }

    bool GeneralPlanner::trackingSnapshotSatisfiesFovForKeepOld(
            const Trajectory &pos_traj,
            const Trajectory &yaw_traj,
            const double local_start_t,
            const traj_opt::DynamicTargetStates &target_prediction,
            std::string *reason) const {
        if (!cfg_.tracking_keep_old_requires_fov) {
            return true;
        }
        if (pos_traj.empty() || yaw_traj.empty()) {
            setFailureReason(reason, "old trajectory has empty position or yaw trajectory");
            return false;
        }

        const double total = std::min(pos_traj.getTotalDuration(), yaw_traj.getTotalDuration());
        const double begin = std::clamp(local_start_t, 0.0, total);
        const double horizon =
                std::min({std::max(0.0, cfg_.tracking_keep_old_horizon),
                          std::max(0.0, total - begin),
                          target_prediction.empty() ? 0.0
                                                    : std::max(0.0, target_prediction.back().t)});
        return trackingTrajectorySatisfiesFov(pos_traj,
                                              yaw_traj,
                                              target_prediction,
                                              begin,
                                              horizon,
                                              cfg_.tracking_fov_check_dt,
                                              0.0,
                                              reason,
                                              true);
    }

    bool GeneralPlanner::trackingTrajectorySatisfiesFov(
            const Trajectory &pos_traj,
            const Trajectory &yaw_traj,
            const traj_opt::DynamicTargetStates &target_prediction,
            const double start_t,
            const double horizon,
            const double dt,
            const double target_start_t,
            std::string *reason,
            const bool allow_keep_old_grace,
            const bool allow_reacquire_range_grace) const {
        if (!cfg_.tracking_fov_commit_check_enable) {
            return true;
        }
        if (!cfg_.tracking_fov_check_strict) {
            return true;
        }
        if (pos_traj.empty() || yaw_traj.empty() || target_prediction.empty()) {
            setFailureReason(reason, "empty trajectory, yaw trajectory, or target prediction");
            return false;
        }

        const double pos_total = pos_traj.getTotalDuration();
        const double yaw_total = yaw_traj.getTotalDuration();
        const double total = std::min(pos_total, yaw_total);
        if (total <= 1.0e-6) {
            setFailureReason(reason, "trajectory duration too short");
            return false;
        }

        constexpr double kPi = 3.14159265358979323846;
        constexpr double kDegToRad = kPi / 180.0;
        const double range = trackingAdaptiveFovRange(cfg_);
        const double begin = std::clamp(start_t, 0.0, total);
        const double target_begin = std::max(0.0, target_start_t);
        const double eval_horizon =
                std::min({std::max(0.0, horizon),
                          std::max(0.0, total - begin),
                          std::max(0.0, target_prediction.back().t - target_begin)});
        const double safe_dt = std::max(0.01, dt);
        const double reacquire_entry_distance =
                std::max({range,
                          std::max(0.0, cfg_.tracking_reacquire_distance),
                          std::max(0.0, cfg_.tracking_reacquire_fov_entry_distance)});
        const bool deferred_reacquire_strict =
                allow_reacquire_range_grace &&
                cfg_.tracking_reacquire_fov_relax_enable &&
                cfg_.tracking_reacquire_fov_deferred_strict_enable;
        const double reacquire_angular_grace_rad =
                std::max(0.0, cfg_.tracking_reacquire_fov_angular_grace_deg) * kDegToRad;

        int total_sample_count = 0;
        int total_violation_count = 0;
        double total_max_h_violation = 0.0;
        double total_max_v_violation = 0.0;
        double total_max_range_violation = 0.0;
        double total_max_front_violation = 0.0;
        double initial_distance = std::numeric_limits<double>::infinity();
        double best_distance = std::numeric_limits<double>::infinity();
        double final_distance = std::numeric_limits<double>::infinity();

        int strict_sample_count = 0;
        int strict_violation_count = 0;
        double strict_max_h_violation = 0.0;
        double strict_max_v_violation = 0.0;
        double strict_max_range_violation = 0.0;
        double strict_max_front_violation = 0.0;
        bool strict_phase_started = !deferred_reacquire_strict;
        double strict_phase_start_t = begin;

        for (double s = 0.0; s <= eval_horizon + 1.0e-6; s += safe_dt) {
            const double t = std::min(total, begin + s);
            const Vec3f p = pos_traj.getPos(t);
            StatePVAJ yaw_state;
            if (!p.allFinite() ||
                !yaw_traj.getState(std::min(t, yaw_total), yaw_state)) {
                setFailureReason(reason, "non-finite FOV sample");
                return false;
            }

            const double yaw = yaw_state(0, 0);
            const Vec3f target =
                    interpolateTargetPrediction(target_prediction, target_begin + s).position;
            const auto fov =
                    evaluateYawOnlyTrackingFov(p,
                                               target,
                                               yaw,
                                               cfg_.tracking_fov_horizontal_deg,
                                               cfg_.tracking_fov_vertical_deg,
                                               range,
                                               cfg_.tracking_fov_range_margin,
                                               cfg_.tracking_fov_front_margin);
            const bool violated = !fov.inside;
            if (total_sample_count == 0) {
                initial_distance = fov.distance;
            }
            best_distance = std::min(best_distance, fov.distance);
            final_distance = fov.distance;
            if (violated) {
                ++total_violation_count;
                total_max_h_violation = std::max(total_max_h_violation, fov.h_violation);
                total_max_v_violation = std::max(total_max_v_violation, fov.v_violation);
                total_max_range_violation = std::max(total_max_range_violation, fov.range_violation);
                total_max_front_violation = std::max(total_max_front_violation, fov.front_violation);
            }
            ++total_sample_count;

            if (deferred_reacquire_strict && !strict_phase_started) {
                if (fov.distance > reacquire_entry_distance + 1.0e-6) {
                    continue;
                }
                strict_phase_started = true;
                strict_phase_start_t = t;
            }

            ++strict_sample_count;
            if (violated) {
                ++strict_violation_count;
                strict_max_h_violation = std::max(strict_max_h_violation, fov.h_violation);
                strict_max_v_violation = std::max(strict_max_v_violation, fov.v_violation);
                strict_max_range_violation = std::max(strict_max_range_violation, fov.range_violation);
                strict_max_front_violation = std::max(strict_max_front_violation, fov.front_violation);
            }
        }

        if (deferred_reacquire_strict && strict_sample_count == 0) {
            const double required_progress =
                    std::max(std::max(0.0, cfg_.tracking_reacquire_min_progress_distance),
                             std::max(0.0, cfg_.tracking_reacquire_min_progress_ratio) *
                             std::max(0.0, initial_distance - reacquire_entry_distance));
            const bool progress_ok =
                    std::isfinite(initial_distance) &&
                    std::isfinite(best_distance) &&
                    std::isfinite(final_distance) &&
                    initial_distance > reacquire_entry_distance + 1.0e-6 &&
                    best_distance <= initial_distance - required_progress &&
                    final_distance <= initial_distance - required_progress;
            const bool orientation_ok =
                    total_max_h_violation <= reacquire_angular_grace_rad &&
                    total_max_v_violation <= reacquire_angular_grace_rad &&
                    total_max_front_violation <= 1.0e-6;
            const Vec3f transit_start = pos_traj.getPos(begin);
            const Vec3f transit_end = pos_traj.getPos(std::min(total, begin + eval_horizon));
            const Vec3f target_start = interpolateTargetPrediction(target_prediction, target_begin).position;
            Vec3f chase_dir = target_start - transit_start;
            if (!cfg_.tracking_motion_3d_enable) {
                chase_dir.z() = 0.0;
            }
            const bool chase_dir_valid = chase_dir.allFinite() && chase_dir.norm() > 1.0e-4;
            const Vec3f transit_dp = transit_end - transit_start;
            const double chase_progress =
                    chase_dir_valid && transit_dp.allFinite()
                        ? transit_dp.dot(chase_dir.normalized())
                        : 0.0;
            const TrackingMotionMetrics transit_metrics =
                    computeTrackingMotionMetrics(pos_traj,
                                                 target_prediction,
                                                 cfg_,
                                                 begin,
                                                 target_begin,
                                                 eval_horizon);
            const bool use_3d_motion =
                    cfg_.tracking_motion_3d_enable || transit_metrics.target_vertical_moving;
            const double transit_displacement =
                    use_3d_motion ? transit_metrics.displacement_3d
                                  : transit_metrics.displacement_xy;
            const double transit_speed =
                    use_3d_motion ? transit_metrics.speed_3d
                                  : transit_metrics.speed_xy;
            const double transit_target_speed =
                    use_3d_motion ? transit_metrics.target_speed_3d
                                  : transit_metrics.target_speed_xy;
            const double required_chase_progress =
                    std::max({0.15,
                              std::max(0.0, cfg_.tracking_keep_old_min_progress_3d_ratio) *
                                      transit_target_speed * eval_horizon,
                              std::max(0.0, cfg_.tracking_no_motion_min_displacement)});
            const bool chase_progress_ok =
                    cfg_.tracking_detour_grace_enable &&
                    chase_dir_valid &&
                    std::isfinite(chase_progress) &&
                    chase_progress >= required_chase_progress &&
                    transit_displacement >= std::max(0.0, cfg_.tracking_no_motion_min_displacement) &&
                    transit_speed >= std::max(0.0, cfg_.tracking_keep_old_min_speed);
            if ((progress_ok || chase_progress_ok) && orientation_ok) {
                return true;
            }
            setFailureReason(reason,
                             fmt::format("reacquire transit without FOV entry: init_dist={:.2f}, best_dist={:.2f}, final_dist={:.2f}, entry_dist={:.2f}, required_progress={:.2f}, chase_progress={:.2f}, required_chase_progress={:.2f}, transit_disp={:.2f}, transit_speed={:.2f}, max_h={:.1f}deg, max_v={:.1f}deg, max_front={:.2f}",
                                         initial_distance,
                                         best_distance,
                                         final_distance,
                                         reacquire_entry_distance,
                                         required_progress,
                                         chase_progress,
                                         required_chase_progress,
                                         transit_displacement,
                                         transit_speed,
                                         total_max_h_violation / kDegToRad,
                                         total_max_v_violation / kDegToRad,
                                         total_max_front_violation));
            return false;
        }

        const int sample_count =
                deferred_reacquire_strict ? strict_sample_count : total_sample_count;
        const int violation_count =
                deferred_reacquire_strict ? strict_violation_count : total_violation_count;
        const double max_h_violation =
                deferred_reacquire_strict ? strict_max_h_violation : total_max_h_violation;
        const double max_v_violation =
                deferred_reacquire_strict ? strict_max_v_violation : total_max_v_violation;
        const double max_range_violation =
                deferred_reacquire_strict ? strict_max_range_violation : total_max_range_violation;
        const double max_front_violation =
                deferred_reacquire_strict ? strict_max_front_violation : total_max_front_violation;

        const bool severe =
                max_h_violation > 10.0 * kDegToRad ||
                max_v_violation > 10.0 * kDegToRad ||
                max_range_violation > 0.20 ||
                max_front_violation > 0.2;
        const bool persistent =
                violation_count > std::max(1, sample_count / 4);
        const bool range_only_grace =
                cfg_.tracking_fov_range_grace_enable &&
                max_range_violation > 0.0 &&
                max_range_violation <= std::max(0.0, cfg_.tracking_fov_range_grace) &&
                max_h_violation <= 1.0e-6 &&
                max_v_violation <= 1.0e-6 &&
                max_front_violation <= 1.0e-6;
        const double keep_old_angular_grace_rad =
                std::max(0.0, cfg_.tracking_fov_keep_old_angular_grace_deg) * kDegToRad;
        const double keep_old_violation_ratio =
                std::clamp(cfg_.tracking_fov_keep_old_violation_ratio_grace, 0.0, 1.0);
        const int keep_old_violation_limit =
                std::max(1, static_cast<int>(std::ceil(sample_count * keep_old_violation_ratio)));
        const bool keep_old_angular_grace_ok =
                allow_keep_old_grace &&
                violation_count <= keep_old_violation_limit &&
                max_h_violation <= keep_old_angular_grace_rad &&
                max_v_violation <= keep_old_angular_grace_rad &&
                max_range_violation <= 1.0e-6 &&
                max_front_violation <= 1.0e-6;
        const bool reacquire_range_grace_ok =
                allow_reacquire_range_grace &&
                cfg_.tracking_reacquire_fov_relax_enable &&
                max_range_violation > 0.0 &&
                max_range_violation <= std::max(0.0, cfg_.tracking_reacquire_fov_range_grace) &&
                max_h_violation <= reacquire_angular_grace_rad &&
                max_v_violation <= reacquire_angular_grace_rad &&
                max_front_violation <= 1.0e-6;
        if (violation_count > 0 &&
            !range_only_grace &&
            !keep_old_angular_grace_ok &&
            !reacquire_range_grace_ok &&
            (cfg_.tracking_fov_check_strict || severe || persistent)) {
            if (deferred_reacquire_strict) {
                setFailureReason(reason,
                                 fmt::format("FOV violation samples={}/{}, range={:.2f}, max_h={:.1f}deg, max_v={:.1f}deg, max_range={:.2f}, max_front={:.2f}, strict_start_t={:.2f}, entry_dist={:.2f}",
                                             violation_count,
                                             sample_count,
                                             range,
                                             max_h_violation / kDegToRad,
                                             max_v_violation / kDegToRad,
                                             max_range_violation,
                                             max_front_violation,
                                             strict_phase_start_t,
                                             reacquire_entry_distance));
            } else {
                setFailureReason(reason,
                                 fmt::format("FOV violation samples={}/{}, range={:.2f}, max_h={:.1f}deg, max_v={:.1f}deg, max_range={:.2f}, max_front={:.2f}",
                                             violation_count,
                                             sample_count,
                                             range,
                                             max_h_violation / kDegToRad,
                                             max_v_violation / kDegToRad,
                                             max_range_violation,
                                             max_front_violation));
            }
            return false;
        }

        return true;
    }

    void GeneralPlanner::resetTrackingCommitCounters() {
        tracking_consecutive_keep_old_ = 0;
        tracking_consecutive_reject_ = 0;
        last_tracking_commit_wt_ = ros_ptr_ ? ros_ptr_->getSimTime() : -1.0;
    }

    void GeneralPlanner::resetTrackingRuntimeDecision(const std::string &reason) {
        last_tracking_runtime_reset_ = false;
        last_tracking_runtime_preserved_ = false;
        last_tracking_runtime_reason_ = reason;
    }

    void GeneralPlanner::maybeResetTrackingRuntimeForReplan(
            const bool new_task,
            const std::string &context) {
        resetTrackingRuntimeDecision(new_task ? "new_task_runtime_check" : "not_new_task");
        if (!cfg_.tracking_runtime_manager_enable || !tracking_runtime_manager_) {
            last_tracking_runtime_reason_ = context + ":runtime_manager_disabled";
            return;
        }
        if (!new_task) {
            last_tracking_runtime_reason_ = context + ":not_new_task";
            return;
        }

        const double committed_remaining = getCommittedTrajectoryRemainingDuration();
        const bool has_committed_tracking =
                tracking_runtime_manager_->hasCommittedTracking();
        if (has_committed_tracking && committed_remaining > 1.0e-3) {
            last_tracking_runtime_preserved_ = true;
            last_tracking_runtime_reason_ =
                    fmt::format("{}:prediction_update_preserve_committed,remaining={:.3f},has_committed_tracking=1",
                                context,
                                committed_remaining);
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [Tracking] TRACKING_RUNTIME_PRESERVED reason={}",
                               last_tracking_runtime_reason_);
            }
            return;
        }

        tracking_runtime_manager_->reset();
        last_tracking_runtime_reset_ = true;
        last_tracking_runtime_reason_ =
                fmt::format("{}:hard_reset_no_committed_tracking,remaining={:.3f},has_committed_tracking={}",
                            context,
                            committed_remaining,
                            static_cast<int>(has_committed_tracking));
        if (cfg_.print_log) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_RUNTIME_RESET reason={}",
                           last_tracking_runtime_reason_);
        }
    }

    void GeneralPlanner::clearTrackingCommitRejectInfo() {
        last_tracking_commit_reject_reason_.clear();
        last_tracking_commit_reject_detail_.clear();
    }

    void GeneralPlanner::setTrackingCommitRejectInfo(const std::string &reason,
                                                     const std::string &detail) {
        last_tracking_commit_reject_reason_ = reason;
        last_tracking_commit_reject_detail_ = detail;
    }

    void GeneralPlanner::setTrackingDiagnostic(const std::string &phase,
                                               const std::string &reason,
                                               const std::size_t guide_path_size,
                                               const std::size_t sfc_size,
                                               const std::size_t target_prediction_size,
                                               const double out_traj_duration) {
        last_tracking_diag_phase_ = phase;
        last_tracking_diag_reason_ = reason;
        last_tracking_diag_guide_path_size_ = guide_path_size;
        last_tracking_diag_sfc_size_ = sfc_size;
        last_tracking_diag_target_prediction_size_ = target_prediction_size;
        last_tracking_diag_out_traj_duration_ = out_traj_duration;
    }

    void GeneralPlanner::setTrackingDiagnostic(const std::string &phase,
                                               const std::string &reason,
                                               const traj_opt::TrackingProblem &problem,
                                               const double out_traj_duration) {
        setTrackingDiagnostic(phase,
                              reason,
                              problem.guide_path.size(),
                              problem.sfcs.size(),
                              problem.target_prediction.size(),
                              out_traj_duration);
    }

    GeneralPlanner::TrackingDiagnosticSnapshot
    GeneralPlanner::getLatestTrackingDiagnosticSnapshot() {
        TrackingDiagnosticSnapshot snapshot;
        snapshot.phase = last_tracking_diag_phase_;
        snapshot.reason = last_tracking_diag_reason_;
        snapshot.guide_path_size = last_tracking_diag_guide_path_size_;
        snapshot.sfc_size = last_tracking_diag_sfc_size_;
        snapshot.target_prediction_size = last_tracking_diag_target_prediction_size_;
        snapshot.out_traj_duration = last_tracking_diag_out_traj_duration_;
        snapshot.consecutive_keep_old =
                tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld()
                                          : tracking_consecutive_keep_old_;
        snapshot.consecutive_reject =
                tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject()
                                          : tracking_consecutive_reject_;
        snapshot.last_commit_wt = last_tracking_commit_wt_;
        snapshot.last_commit_reject_reason = last_tracking_commit_reject_reason_;
        snapshot.last_commit_reject_detail = last_tracking_commit_reject_detail_;
        snapshot.runtime_manager_enabled = cfg_.tracking_runtime_manager_enable &&
                                           static_cast<bool>(tracking_runtime_manager_);
        snapshot.has_committed_tracking =
                tracking_runtime_manager_ ? tracking_runtime_manager_->hasCommittedTracking()
                                          : !cmd_traj_info_.empty();
        snapshot.committed_remaining = getCommittedTrajectoryRemainingDuration();
        snapshot.runtime_reset = last_tracking_runtime_reset_;
        snapshot.runtime_preserved = last_tracking_runtime_preserved_;
        snapshot.runtime_reason = last_tracking_runtime_reason_;
        return snapshot;
    }

    std::string GeneralPlanner::getTrackingConfigSummary() const {
        return fmt::format("tracking_distance={:.3f};distance_tolerance={:.3f};"
                           "distance_lower_tolerance={:.3f};distance_upper_tolerance={:.3f};"
                           "height_offset={:.3f};height_tolerance={:.3f};safe_distance={:.3f};"
                           "hard_safe_distance={:.3f};fov_h_deg={:.3f};fov_v_deg={:.3f};"
                           "fov_range={:.3f};fov_range_effective={:.3f};"
                           "fov_check_strict={};fov_commit_check_enable={};"
                           "frontend_fov_feasibility_enable={};runtime_manager_enable={};"
                           "keep_old_horizon={:.3f};keep_old_safety_dt={:.3f};"
                           "keep_old_requires_fov={};short_safety_grace_enable={};"
                           "anti_rollback_enable={};reacquire_distance={:.3f};"
                           "reacquire_min_progress_distance={:.3f};reacquire_min_progress_ratio={:.3f};"
                           "reacquire_fov_relax_enable={};reacquire_fov_deferred_strict_enable={};"
                           "optimizer_commit_safety_precheck_enable={};"
                           "detour_grace_enable={};detour_grace_horizon={:.3f};"
                           "frontend_astar={};use_visible_region={};max_vel={:.3f};max_acc={:.3f}",
                           cfg_.tracking_distance,
                           cfg_.tracking_distance_tolerance,
                           cfg_.tracking_distance_lower_tolerance,
                           cfg_.tracking_distance_upper_tolerance,
                           cfg_.tracking_height_offset,
                           cfg_.tracking_height_tolerance,
                           cfg_.tracking_safe_distance,
                           cfg_.tracking_hard_safe_distance,
                           cfg_.tracking_fov_horizontal_deg,
                           cfg_.tracking_fov_vertical_deg,
                           cfg_.tracking_fov_range,
                           trackingAdaptiveFovRange(cfg_),
                           static_cast<int>(cfg_.tracking_fov_check_strict),
                           static_cast<int>(cfg_.tracking_fov_commit_check_enable),
                           static_cast<int>(cfg_.tracking_frontend_fov_feasibility_enable &&
                                            cfg_.tracking_fov_check_strict),
                           static_cast<int>(cfg_.tracking_runtime_manager_enable),
                           cfg_.tracking_keep_old_horizon,
                           cfg_.tracking_keep_old_safety_dt,
                           static_cast<int>(cfg_.tracking_keep_old_requires_fov &&
                                            cfg_.tracking_fov_check_strict),
                           static_cast<int>(cfg_.tracking_keep_old_short_safety_grace_enable),
                           static_cast<int>(cfg_.tracking_anti_rollback_enable),
                           cfg_.tracking_reacquire_distance,
                           cfg_.tracking_reacquire_min_progress_distance,
                           cfg_.tracking_reacquire_min_progress_ratio,
                           static_cast<int>(cfg_.tracking_reacquire_fov_relax_enable),
                           static_cast<int>(cfg_.tracking_reacquire_fov_deferred_strict_enable),
                           static_cast<int>(cfg_.tracking_optimizer_commit_safety_precheck_enable),
                           static_cast<int>(cfg_.tracking_detour_grace_enable),
                           cfg_.tracking_detour_grace_horizon,
                           static_cast<int>(cfg_.tracking_frontend_astar),
                           static_cast<int>(cfg_.tracking_use_visible_region),
                           cfg_.exp_traj_cfg.max_vel,
                           cfg_.exp_traj_cfg.max_acc);
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
            const double commit_wt,
            const double candidate_eval_start_t,
            const double target_eval_start_t,
            const bool candidate_safe,
            const bool candidate_fov_ok,
            int *worse_count_out,
            double *max_regression_out,
            std::string *reason) {
        if (worse_count_out != nullptr) {
            *worse_count_out = 0;
        }
        if (max_regression_out != nullptr) {
            *max_regression_out = 0.0;
        }
        if (!cfg_.tracking_anti_rollback_enable) {
            return true;
        }
        if (candidate_pos_traj.empty() || target_prediction.empty()) {
            setFailureReason(reason, "empty candidate or target prediction");
            return false;
        }
        if (cmd_traj_info_.empty()) {
            return true;
        }

        cmd_traj_info_.lock();
        const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
        const Trajectory old_yaw_traj = cmd_traj_info_.yawTraj();
        const double old_start_wt = cmd_traj_info_.getStartWallTime();
        const double old_total_dur = cmd_traj_info_.getTotalDuration();
        cmd_traj_info_.unlock();

        const double old_eval_t = commit_wt - old_start_wt;
        if (old_pos_traj.empty() || old_eval_t < -1.0e-6) {
            return true;
        }

        const double dt = std::max(0.05, cfg_.tracking_anti_rollback_dt);
        const double candidate_start =
                std::clamp(candidate_eval_start_t, 0.0, candidate_pos_traj.getTotalDuration());
        const double target_start = std::max(0.0, target_eval_start_t);
        const double horizon =
                std::min({std::max(0.0, cfg_.tracking_anti_rollback_horizon),
                          std::max(0.0, old_total_dur - old_eval_t),
                          std::max(0.0, candidate_pos_traj.getTotalDuration() - candidate_start),
                          std::max(0.0, target_prediction.back().t - target_start)});
        if (horizon < dt + 1.0e-6) {
            return true;
        }
        if (!trackingTrajectorySafeForHorizon(old_pos_traj, old_eval_t, horizon, dt)) {
            return true;
        }

        const double margin = std::max(0.0, cfg_.tracking_anti_rollback_margin);
        int worse_count = 0;
        double max_regression = 0.0;
        double max_new_score = 0.0;
        for (double offset = dt; offset <= horizon + 1.0e-6; offset += dt) {
            const Vec3f old_pos =
                    old_pos_traj.getPos(std::clamp(old_eval_t + offset,
                                                   0.0,
                                                   old_pos_traj.getTotalDuration()));
            const Vec3f new_pos =
                    candidate_pos_traj.getPos(std::clamp(candidate_start + offset,
                                                         0.0,
                                                         candidate_pos_traj.getTotalDuration()));
            const Vec3f target_pos =
                    interpolateTargetPrediction(target_prediction, target_start + offset).position;
            const double old_score = trackingViewpointErrorScore(old_pos, target_pos);
            const double new_score = trackingViewpointErrorScore(new_pos, target_pos);
            max_new_score = std::max(max_new_score, new_score);
            const double regression = new_score - old_score;
            if (regression > margin) {
                ++worse_count;
                max_regression = std::max(max_regression, regression);
            }
        }
        if (worse_count_out != nullptr) {
            *worse_count_out = worse_count;
        }
        if (max_regression_out != nullptr) {
            *max_regression_out = max_regression;
        }

        if (worse_count >= 2 || max_regression > 2.0 * margin) {
            TrackingTrajectoryActivity old_activity =
                    evaluateTrackingTrajectoryActivity(old_pos_traj,
                                                       std::clamp(old_eval_t, 0.0, old_total_dur),
                                                       target_prediction,
                                                       cfg_.tracking_keep_old_horizon,
                                                       cfg_.tracking_keep_old_safety_dt);
            std::string old_fov_reason;
            const bool old_fov_ok =
                    old_yaw_traj.empty()
                        ? false
                        : trackingSnapshotSatisfiesFovForKeepOld(old_pos_traj,
                                                                 old_yaw_traj,
                                                                 std::clamp(old_eval_t, 0.0, old_total_dur),
                                                                 target_prediction,
                                                                 &old_fov_reason);
            const double grace_horizon =
                    std::min({std::max(0.0, cfg_.tracking_detour_grace_horizon),
                              std::max(0.0, candidate_pos_traj.getTotalDuration() - candidate_start),
                              std::max(0.0, target_prediction.back().t - target_start)});
            const TrackingMotionMetrics candidate_metrics =
                    computeTrackingMotionMetrics(candidate_pos_traj,
                                                 target_prediction,
                                                 cfg_,
                                                 candidate_start,
                                                 target_start,
                                                 grace_horizon);
            const double min_positive_progress =
                    std::max(0.02,
                             candidate_metrics.target_speed_3d *
                             grace_horizon *
                             std::max(0.0, cfg_.tracking_keep_old_min_progress_3d_ratio));
            const bool positive_3d_progress =
                    candidate_metrics.progress_3d >= min_positive_progress ||
                    candidate_metrics.displacement_3d >= cfg_.tracking_no_motion_min_displacement ||
                    candidate_metrics.displacement_z >= cfg_.tracking_no_motion_min_displacement_z;
            const bool old_unusable =
                    !old_activity.active ||
                    old_activity.remaining < cfg_.tracking_keep_old_min_remaining ||
                    !old_fov_ok;
            const double detour_error_limit =
                    cfg_.tracking_detour_max_tracking_error_scale *
                    std::max(0.1, cfg_.tracking_distance_tolerance);
            if (cfg_.tracking_detour_grace_enable &&
                candidate_safe &&
                candidate_fov_ok &&
                positive_3d_progress &&
                old_unusable &&
                max_new_score <= detour_error_limit) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_FORCE_COMMIT_SAFE_RECOVERY reason=detour_grace, worse_count={}, max_regression={:.3f}, max_new_error={:.3f}, error_limit={:.3f}, candidate_safe={}, candidate_fov_ok={}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_progress_3d={:.3f}, old_active={}, old_remaining={:.3f}, old_activity_reason={}, old_fov_ok={}, old_fov_reason={}, prefix_duration={:.3f}, target_eval_start={:.3f}",
                                   worse_count,
                                   max_regression,
                                   max_new_score,
                                   detour_error_limit,
                                   candidate_safe,
                                   candidate_fov_ok,
                                   candidate_metrics.displacement_xy,
                                   candidate_metrics.displacement_z,
                                   candidate_metrics.displacement_3d,
                                   candidate_metrics.progress_3d,
                                   old_activity.active,
                                   old_activity.remaining,
                                   old_activity.reason,
                                   old_fov_ok,
                                   old_fov_reason,
                                   candidate_start,
                                   target_start);
                }
                return true;
            }
            setFailureReason(reason,
                             fmt::format("anti-rollback regression: worse_count={}, max_regression={:.3f}, max_new_error={:.3f}, old_activity={}, old_fov_ok={}",
                                         worse_count,
                                         max_regression,
                                         max_new_score,
                                         old_activity.reason,
                                         old_fov_ok));
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_ANTI_ROLLBACK reject_count={}, keep_old_count={}, worse_count={}, max_regression={:.3f}, max_new_error={:.3f}, horizon={:.2f}, prefix_duration={:.3f}, target_eval_start={:.3f}, candidate_safe={}, candidate_fov_ok={}, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_progress_3d={:.3f}, old_remaining={:.3f}, old_activity_reason={}, old_fov_ok={}, old_fov_reason={}, old_speed_xy={:.3f}, old_speed_z={:.3f}, old_speed_3d={:.3f}, old_disp_xy={:.3f}, old_disp_z={:.3f}, old_disp_3d={:.3f}, old_progress_xy={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveReject() : tracking_consecutive_reject_,
                               tracking_runtime_manager_ ? tracking_runtime_manager_->consecutiveKeepOld() : tracking_consecutive_keep_old_,
                               worse_count,
                               max_regression,
                               max_new_score,
                               horizon,
                               candidate_start,
                               target_start,
                               candidate_safe,
                               candidate_fov_ok,
                               candidate_metrics.displacement_xy,
                               candidate_metrics.displacement_z,
                               candidate_metrics.displacement_3d,
                               candidate_metrics.progress_3d,
                               old_activity.remaining,
                               old_activity.reason,
                               old_fov_ok,
                               old_fov_reason,
                               old_activity.speed_xy,
                               old_activity.speed_z,
                               old_activity.speed_3d,
                               old_activity.displacement_xy,
                               old_activity.displacement_z,
                               old_activity.displacement_3d,
                               old_activity.progress_xy,
                               old_activity.progress_3d,
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

    bool GeneralPlanner::findTrackingViewpointReference(
            const traj_opt::DynamicTargetStates &target_prediction,
            Vec3f &reference_viewpoint,
            traj_opt::DynamicTargetState &reference_target) const {
        if (target_prediction.empty() ||
            last_tracking_frontend_prediction_.empty() ||
            last_tracking_frontend_viewpoints_.empty() ||
            last_tracking_frontend_prediction_.size() != last_tracking_frontend_viewpoints_.size()) {
            return false;
        }

        const Vec3f &target0 = target_prediction.front().position;
        double best_score = std::numeric_limits<double>::infinity();
        std::size_t best_idx = 0;
        for (std::size_t i = 0; i < last_tracking_frontend_prediction_.size(); ++i) {
            const auto &old_target = last_tracking_frontend_prediction_[i];
            const auto &old_viewpoint = last_tracking_frontend_viewpoints_[i];
            if (!old_target.position.allFinite() || !old_viewpoint.allFinite()) {
                continue;
            }
            const double score = (old_target.position - target0).norm();
            if (score < best_score) {
                best_score = score;
                best_idx = i;
            }
        }

        const double max_match_dist =
                std::max({1.0,
                          cfg_.tracking_distance,
                          cfg_.tracking_distance + cfg_.tracking_distance_upper_tolerance});
        if (!std::isfinite(best_score) || best_score > max_match_dist) {
            return false;
        }

        reference_viewpoint = last_tracking_frontend_viewpoints_[best_idx];
        reference_target = last_tracking_frontend_prediction_[best_idx];
        return trackingGuidePointSafe(reference_viewpoint);
    }

    void GeneralPlanner::rememberTrackingViewpointReference(
            const traj_opt::TrackingProblem &problem) {
        if (problem.target_prediction.empty() ||
            problem.viewpoints.empty() ||
            problem.target_prediction.size() != problem.viewpoints.size()) {
            last_tracking_frontend_prediction_.clear();
            last_tracking_frontend_viewpoints_.clear();
            return;
        }
        last_tracking_frontend_prediction_ = problem.target_prediction;
        last_tracking_frontend_viewpoints_ = problem.viewpoints;
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

        auto applyViewpointTruncation =
                [&](const vec_Vec3f &prefix,
                    const std::vector<double> &prefix_t,
                    const std::size_t keep_count) {
                    problem.guide_path = prefix;
                    problem.guide_t = prefix_t;
                    problem.viewpoints.resize(keep_count);
                    problem.target_sample_times.resize(keep_count);
                    problem.target_prediction.resize(keep_count);
                    if (problem.visible_regions.size() > keep_count) {
                        problem.visible_regions.resize(keep_count);
                    }
                    refreshTrackingGuideEndpoint(problem);
                };

        auto applyTimeAlignedTruncation =
                [&](const vec_Vec3f &prefix,
                    const std::vector<double> &prefix_t) -> bool {
                    if (prefix.size() < 2 ||
                        prefix_t.size() != prefix.size() ||
                        problem.target_prediction.empty()) {
                        return false;
                    }
                    const double end_t = prefix_t.back();
                    if (!std::isfinite(end_t)) {
                        return false;
                    }

                    traj_opt::DynamicTargetStates truncated_target_prediction;
                    std::vector<double> truncated_target_sample_times;
                    vec_Vec3f truncated_viewpoints;
                    super_utils::vec_E<traj_opt::TrackingVisibleRegion> truncated_visible_regions;

                    auto appendTimedTrackingSample =
                            [&](const double sample_t,
                                const traj_opt::DynamicTargetState &state,
                                const traj_opt::TrackingVisibleRegion *source_region) {
                                if (!std::isfinite(sample_t)) {
                                    return;
                                }
                                const Vec3f viewpoint =
                                        interpolatePointOnTimedGuide(prefix, prefix_t, sample_t);
                                truncated_target_prediction.emplace_back(state);
                                truncated_target_prediction.back().t = sample_t;
                                truncated_target_sample_times.emplace_back(sample_t);
                                truncated_viewpoints.emplace_back(viewpoint);
                                if (!problem.visible_regions.empty()) {
                                    traj_opt::TrackingVisibleRegion region;
                                    if (source_region != nullptr) {
                                        region = *source_region;
                                    } else {
                                        region.valid = false;
                                        region.confidence = 0.0;
                                        region.theta = 0.0;
                                    }
                                    region.t = sample_t;
                                    region.target_position = state.position;
                                    region.visible_point = viewpoint;
                                    truncated_visible_regions.emplace_back(region);
                                }
                            };

                    const bool has_explicit_sample_times =
                            problem.target_sample_times.size() == problem.target_prediction.size();
                    for (std::size_t i = 0; i < problem.target_prediction.size(); ++i) {
                        const double sample_t =
                                has_explicit_sample_times
                                    ? problem.target_sample_times[i]
                                    : problem.target_prediction[i].t;
                        if (sample_t <= end_t + 1.0e-6) {
                            const traj_opt::TrackingVisibleRegion *source_region =
                                    i < problem.visible_regions.size()
                                        ? &problem.visible_regions[i]
                                        : nullptr;
                            appendTimedTrackingSample(sample_t,
                                                      problem.target_prediction[i],
                                                      source_region);
                        }
                    }

                    if (truncated_target_prediction.empty()) {
                        const double first_t =
                                has_explicit_sample_times
                                    ? problem.target_sample_times.front()
                                    : problem.target_prediction.front().t;
                        appendTimedTrackingSample(std::min(first_t, end_t),
                                                  problem.target_prediction.front(),
                                                  problem.visible_regions.empty()
                                                      ? nullptr
                                                      : &problem.visible_regions.front());
                    }

                    if (truncated_target_prediction.empty() ||
                        truncated_viewpoints.size() != truncated_target_prediction.size() ||
                        truncated_target_sample_times.size() != truncated_target_prediction.size()) {
                        return false;
                    }

                    if (end_t > truncated_target_sample_times.back() + 1.0e-4) {
                        const auto horizon_state =
                                interpolateTargetPrediction(problem.target_prediction, end_t);
                        appendTimedTrackingSample(end_t, horizon_state, nullptr);
                    }

                    if (truncated_target_prediction.empty() ||
                        truncated_viewpoints.size() != truncated_target_prediction.size() ||
                        truncated_target_sample_times.size() != truncated_target_prediction.size()) {
                        return false;
                    }

                    problem.guide_path = prefix;
                    problem.guide_t = prefix_t;
                    problem.target_prediction = std::move(truncated_target_prediction);
                    problem.target_sample_times = std::move(truncated_target_sample_times);
                    problem.viewpoints = std::move(truncated_viewpoints);
                    if (!problem.visible_regions.empty()) {
                        problem.visible_regions = std::move(truncated_visible_regions);
                    } else {
                        problem.visible_regions.clear();
                    }
                    refreshTrackingGuideEndpoint(problem);
                    return true;
                };

        const double match_tol = std::max({0.05,
                                           1.5 * std::max(1.0e-3, cfg_.resolution),
                                           0.25 * std::max(0.2, cfg_.corridor_line_max_length)});
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

            const std::size_t keep_count = static_cast<std::size_t>(view_id + 1);
            applyViewpointTruncation(prefix, prefix_t, keep_count);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking guide truncated to safe local SFC endpoint, kept {} target samples.",
                               keep_count);
            }
            return true;
        }

        std::string last_time_prefix_reason;
        for (int end_id = static_cast<int>(candidate_guide.size()) - 1; end_id >= 1; --end_id) {
            vec_Vec3f prefix(candidate_guide.begin(), candidate_guide.begin() + end_id + 1);
            std::vector<double> prefix_t(candidate_guide_t.begin(), candidate_guide_t.begin() + end_id + 1);
            std::string prefix_reason;
            if (!tryGenerateTrackingCorridor(prefix, sfcs, &prefix_reason)) {
                last_time_prefix_reason =
                        fmt::format("time_prefix_sfc_failed(end_id={}, reason={})",
                                    end_id,
                                    prefix_reason);
                continue;
            }
            if (!applyTimeAlignedTruncation(prefix, prefix_t)) {
                last_time_prefix_reason =
                        fmt::format("time_prefix_apply_failed(end_id={}, end_t={:.3f})",
                                    end_id,
                                    prefix_t.empty() ? 0.0 : prefix_t.back());
                continue;
            }
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking guide truncated by timed prefix fallback, kept {} target samples.",
                               problem.target_prediction.size());
            }
            return true;
        }

        if (!last_prefix_reason.empty() && !last_time_prefix_reason.empty()) {
            setFailureReason(failure_reason,
                             last_prefix_reason + "; " + last_time_prefix_reason);
        } else {
            setFailureReason(failure_reason,
                             !last_prefix_reason.empty()
                                 ? last_prefix_reason
                                 : (!last_time_prefix_reason.empty()
                                        ? last_time_prefix_reason
                                        : "truncate_no_safe_prefix_found"));
        }
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

        std::string truncate_original_reason;
        if (!problem.guide_path.empty() &&
            truncateTrackingProblemForCorridor(problem,
                                               problem.guide_path,
                                               problem.guide_t,
                                               sfcs,
                                               &truncate_original_reason)) {
            problem.sfcs = std::move(sfcs);
            problem.use_corridor = true;
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [GeneralPlanner] Tracking SFC fallback accepted truncated original guide prefix.");
            }
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
        std::string truncate_repaired_reason;
        if (full_astar_repair) {
            if (truncateTrackingProblemForCorridor(problem,
                                                  astar_repaired,
                                                  astar_repaired_t,
                                                  sfcs,
                                                  &truncate_repaired_reason)) {
                problem.sfcs = std::move(sfcs);
                problem.use_corridor = true;
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [GeneralPlanner] Tracking SFC fallback accepted truncated A* repaired guide prefix.");
                }
                return true;
            }
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
                         fmt::format("guide_size={}, target_samples={}, dense={}, truncate_original={}, astar={}, truncate_repaired={}, dense_astar={}, truncate_astar={}, truncate_dense={}",
                                     problem.guide_path.size(),
                                     problem.target_prediction.size(),
                                     dense_reason.empty() ? "ok-but-unused" : dense_reason,
                                     truncate_original_reason.empty() ? "not_attempted_or_failed" : truncate_original_reason,
                                     full_astar_repair ? "ok" : astar_reason,
                                     truncate_repaired_reason.empty() ? "not_attempted_or_failed" : truncate_repaired_reason,
                                     dense_astar_reason.empty() ? "not_attempted_or_ok" : dense_astar_reason,
                                     truncate_astar_reason.empty() ? "not_attempted" : truncate_astar_reason,
                                     truncate_dense_reason.empty() ? "not_attempted" : truncate_dense_reason));
        return false;
    }

    bool GeneralPlanner::applyTrackingNarrowPassageSoftDistance(
            traj_opt::TrackingProblem &problem,
            std::string *reason) const {
        if (!cfg_.tracking_narrow_passage_enable ||
            problem.guide_path.empty() ||
            map_manager_ == nullptr ||
            !map_manager_->ready() ||
            !map_manager_->hasESDF()) {
            return false;
        }

        double min_clearance = std::numeric_limits<double>::infinity();
        for (const Vec3f &p : problem.guide_path) {
            double dist = 0.0;
            Vec3f grad = Vec3f::Zero();
            if (p.allFinite() && map_manager_->evaluateESDF(p, dist, grad)) {
                min_clearance = std::min(min_clearance, dist);
            }
        }
        if (!std::isfinite(min_clearance) ||
            min_clearance >= cfg_.tracking_narrow_passage_clearance_threshold) {
            return false;
        }

        const double old_safe_distance = problem.safe_distance;
        const double hard_distance = trackingHardSafeDistance(cfg_);
        const double scaled_soft =
                cfg_.tracking_safe_distance *
                std::clamp(cfg_.tracking_narrow_passage_soft_safe_distance_scale, 0.1, 1.0);
        problem.safe_distance = std::max(hard_distance, scaled_soft);
        setFailureReason(reason,
                         fmt::format("narrow_passage min_clearance={:.3f}, optimizer_safe_distance {:.3f}->{:.3f}, hard_safe_distance={:.3f}",
                                     min_clearance,
                                     old_safe_distance,
                                     problem.safe_distance,
                                     hard_distance));
        return true;
    }

    bool GeneralPlanner::optimizeTrackingProblemWithRetries(
            const traj_opt::TrackingProblem &normal_problem,
            const traj_opt::DynamicTargetStates &active_target_prediction,
            Trajectory &out_traj,
            Trajectory &out_yaw_traj,
            traj_opt::DynamicTargetStates *accepted_target_prediction,
            bool *accepted_reacquire_fov_relax,
            std::string *failure_reason) {
        out_traj = Trajectory();
        out_yaw_traj = Trajectory();
        if (accepted_target_prediction != nullptr) {
            *accepted_target_prediction = active_target_prediction;
        }
        if (accepted_reacquire_fov_relax != nullptr) {
            *accepted_reacquire_fov_relax = false;
        }
        std::vector<std::string> attempt_failures;

        auto shouldFovPostcheckOptimization = [&]() -> bool {
            if (!cfg_.tracking_adaptive_occlusion_postcheck_enable ||
                !cfg_.tracking_fov_commit_check_enable ||
                !cfg_.tracking_fov_check_strict) {
                return false;
            }
            const bool has_committed_tracking =
                    cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_
                        ? tracking_runtime_manager_->hasCommittedTracking()
                        : !cmd_traj_info_.empty();
            return cfg_.tracking_fov_check_first_commit || has_committed_tracking;
        };

        auto candidatePassesOptimizationFovPostcheck =
                [&](const Trajectory &candidate_pos,
                    Trajectory &candidate_yaw,
                    const traj_opt::DynamicTargetStates &check_target_prediction,
                    const bool allow_reacquire_fov_relax,
                    std::string *reject_reason) -> bool {
            if (!shouldFovPostcheckOptimization()) {
                return true;
            }
            if (candidate_pos.empty() || check_target_prediction.empty()) {
                setFailureReason(reject_reason, "empty candidate or target prediction");
                return false;
            }
            if (candidate_yaw.empty() &&
                !buildTrackingTargetYawTrajectory(candidate_pos,
                                                  check_target_prediction,
                                                  candidate_yaw)) {
                setFailureReason(reject_reason, "yaw generation failed for FOV postcheck");
                return false;
            }
            const double fov_horizon =
                    std::min({std::max(0.0, cfg_.tracking_keep_old_horizon),
                              candidate_pos.getTotalDuration(),
                              check_target_prediction.empty()
                                  ? 0.0
                                  : std::max(0.0, check_target_prediction.back().t)});
            std::string optimized_yaw_reason;
            if (trackingTrajectorySatisfiesFov(candidate_pos,
                                               candidate_yaw,
                                               check_target_prediction,
                                               0.0,
                                               fov_horizon,
                                               cfg_.tracking_fov_check_dt,
                                               0.0,
                                               &optimized_yaw_reason,
                                               false,
                                               allow_reacquire_fov_relax)) {
                return true;
            }

            Trajectory target_yaw_traj;
            std::string target_yaw_reason;
            const bool target_yaw_generated =
                    buildTrackingTargetYawTrajectory(candidate_pos,
                                                     check_target_prediction,
                                                     target_yaw_traj);
            if (target_yaw_generated &&
                trackingTrajectorySatisfiesFov(candidate_pos,
                                               target_yaw_traj,
                                               check_target_prediction,
                                               0.0,
                                               fov_horizon,
                                               cfg_.tracking_fov_check_dt,
                                               0.0,
                                               &target_yaw_reason,
                                               false,
                                               allow_reacquire_fov_relax)) {
                candidate_yaw = target_yaw_traj;
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_YAW_REBUILT_FOR_FOV reason={}, horizon={:.3f}, reacquire_fov_relax={}",
                                   optimized_yaw_reason,
                                   fov_horizon,
                                   allow_reacquire_fov_relax);
                }
                return true;
            }

            if (allow_reacquire_fov_relax) {
                if (target_yaw_generated && !target_yaw_traj.empty()) {
                    candidate_yaw = target_yaw_traj;
                }
                const std::string fallback_reason =
                        target_yaw_generated
                            ? (target_yaw_reason.empty()
                                   ? "failed_without_reason"
                                   : target_yaw_reason)
                            : "generation_failed";
                setFailureReason(reject_reason,
                                 optimized_yaw_reason +
                                 "; target_yaw_fallback=" +
                                 fallback_reason +
                                 "; degraded_fov_accepted=1");
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_FOV_DEGRADED_ACCEPT reason={}, target_yaw_fallback={}, horizon={:.3f}, reacquire_fov_relax={}",
                                   optimized_yaw_reason,
                                   fallback_reason,
                                   fov_horizon,
                                   allow_reacquire_fov_relax);
                }
                return true;
            }

            setFailureReason(reject_reason,
                             optimized_yaw_reason +
                             "; target_yaw_fallback=" +
                             (target_yaw_generated
                                  ? (target_yaw_reason.empty()
                                         ? "failed_without_reason"
                                         : target_yaw_reason)
                                  : "generation_failed"));
            return false;
        };

        auto hasCommittedTrackingSnapshot = [&]() -> bool {
            if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
                return tracking_runtime_manager_->hasCommittedTracking() && !cmd_traj_info_.empty();
            }
            return !cmd_traj_info_.empty();
        };

        auto candidatePassesOptimizationCommitSafety =
                [&](const Trajectory &candidate_pos,
                    std::string *reject_reason) -> bool {
            if (!cfg_.tracking_optimizer_commit_safety_precheck_enable) {
                return true;
            }
            if (candidate_pos.empty()) {
                setFailureReason(reject_reason, "empty candidate trajectory");
                return false;
            }

            Trajectory committed_candidate = candidate_pos;
            const double commit_wt = ros_ptr_->getSimTime();
            double stitched_prefix_duration = 0.0;
            if (!cmd_traj_info_.empty()) {
                cmd_traj_info_.lock();
                const Trajectory old_pos_traj = cmd_traj_info_.posTraj();
                const double old_start_wt = cmd_traj_info_.getStartWallTime();
                const double old_total_dur = cmd_traj_info_.getTotalDuration();
                cmd_traj_info_.unlock();

                const double prefix_start_t = commit_wt - old_start_wt;
                const double prefix_end_t =
                        prefix_start_t + std::max(0.0, cfg_.replan_forward_dt);
                const bool prefix_window_valid =
                        !old_pos_traj.empty() &&
                        prefix_start_t >= 0.0 &&
                        prefix_end_t > prefix_start_t + 1.0e-4 &&
                        prefix_end_t <= old_total_dur + 1.0e-6;
                if (prefix_window_valid) {
                    Trajectory prefix_pos_traj;
                    if (old_pos_traj.getPartialTrajectoryByTime(prefix_start_t,
                                                                std::min(prefix_end_t, old_total_dur),
                                                                prefix_pos_traj)) {
                        committed_candidate = prefix_pos_traj + candidate_pos;
                        stitched_prefix_duration = prefix_pos_traj.getTotalDuration();
                        committed_candidate.start_WT = commit_wt;
                    }
                }
            }

            const double safety_start =
                    cfg_.tracking_anti_rollback_eval_after_prefix
                        ? stitched_prefix_duration
                        : 0.0;
            const double safety_horizon =
                    std::min(std::max(0.0, cfg_.tracking_keep_old_horizon),
                             std::max(0.0,
                                      committed_candidate.getTotalDuration() -
                                      std::clamp(safety_start,
                                                 0.0,
                                                 committed_candidate.getTotalDuration())));
            std::string safety_reason;
            std::string safety_detail;
            if (!trackingTrajectorySafeForHorizonDetailed(committed_candidate,
                                                          safety_start,
                                                          safety_horizon,
                                                          cfg_.tracking_keep_old_safety_dt,
                                                          &safety_reason,
                                                          &safety_detail)) {
                setFailureReason(reject_reason,
                                 fmt::format(
                                         "{}; commit_candidate_duration={:.3f}; stitched_prefix_duration={:.3f}; safety_start={:.3f}; safety_horizon={:.3f}; {}",
                                         safety_reason.empty() ? "commit safety precheck failed"
                                                               : safety_reason,
                                         committed_candidate.getTotalDuration(),
                                         stitched_prefix_duration,
                                         safety_start,
                                         safety_horizon,
                                         safety_detail.empty() ? "detail=none"
                                                               : safety_detail));
                return false;
            }
            return true;
        };

        auto estimateReacquireTransitHorizon =
                [&](const traj_opt::TrackingProblem &problem) -> double {
            const double base_h =
                    std::max(0.3, cfg_.tracking_reacquire_recovery_horizon);
            if (problem.target_prediction.empty()) {
                return base_h;
            }
            const double initial_horizontal_dist =
                    (problem.head_pvaj.col(0) - problem.target_prediction.front().position)
                            .head<2>()
                            .norm();
            const double reacquire_band =
                    std::max(problem.tracking_distance + problem.distance_tolerance,
                             cfg_.tracking_reacquire_distance);
            const double gap = std::max(0.0, initial_horizontal_dist - reacquire_band);
            if (gap <= 1.0e-6) {
                return base_h;
            }
            const double speed_cap =
                    std::max(0.8, cfg_.esdf_traj_cfg.max_vel);
            const double scaled_h =
                    gap * std::max(0.1, cfg_.tracking_reacquire_transit_horizon_scale) / speed_cap;
            return std::clamp(std::max(base_h, scaled_h),
                              base_h,
                              std::max(base_h, cfg_.tracking_reacquire_transit_max_horizon));
        };

        auto runOptimization = [&](const std::string &attempt_name,
                                   traj_opt::TrackingProblem problem,
                                   const bool recovery_attempt) -> bool {
            std::string narrow_reason;
            const bool narrow_adjusted = applyTrackingNarrowPassageSoftDistance(problem, &narrow_reason);
            if (cfg_.print_log && narrow_adjusted) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_NARROW_PASSAGE_SOFT_CLEARANCE attempt={}, reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}",
                               attempt_name,
                               narrow_reason,
                               problem.guide_path.size(),
                               problem.sfcs.size(),
                               problem.target_prediction.size());
            }

            Trajectory candidate_pos;
            Trajectory candidate_yaw;
            const bool ok = cfg_.tracking_use_snap
                                ? traj_manager_->trackingSnap()->optimize(problem, candidate_pos, &candidate_yaw)
                                : traj_manager_->trackingJerk()->optimize(problem, candidate_pos, &candidate_yaw);
            if (ok && !candidate_pos.empty()) {
                const traj_opt::DynamicTargetStates &check_target_prediction =
                        problem.target_prediction.empty()
                            ? active_target_prediction
                            : problem.target_prediction;
                const double initial_horizontal_dist =
                        check_target_prediction.empty()
                            ? 0.0
                            : (problem.head_pvaj.col(0) - check_target_prediction.front().position)
                                      .head<2>()
                                      .norm();
                const bool recovery_like_reacquire =
                        recovery_attempt &&
                        (initial_horizontal_dist >
                                 problem.tracking_distance + problem.distance_tolerance ||
                         !hasCommittedTrackingSnapshot());
                const bool allow_reacquire_fov_relax =
                        (problem.reacquire_mode || recovery_like_reacquire) &&
                        cfg_.tracking_reacquire_fov_relax_enable;
                std::string fov_postcheck_reason;
                if (!candidatePassesOptimizationFovPostcheck(candidate_pos,
                                                             candidate_yaw,
                                                             check_target_prediction,
                                                             allow_reacquire_fov_relax,
                                                             &fov_postcheck_reason)) {
                    attempt_failures.emplace_back(
                            fmt::format("{} rejected_by_fov_postcheck(reason={}, guide={}, sfc={}, target={}, duration={:.3f}, safe_distance={:.3f}, use_corridor={})",
                                        attempt_name,
                                        fov_postcheck_reason,
                                        problem.guide_path.size(),
                                        problem.sfcs.size(),
                                        problem.target_prediction.size(),
                                        candidate_pos.getTotalDuration(),
                                        problem.safe_distance,
                                        problem.use_corridor));
                    ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_RECOVERY_FAILED attempt={}, reason=fov_postcheck_failed:{}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, safe_distance={:.3f}, use_corridor={}, narrow_adjusted={}",
                                   attempt_name,
                                   fov_postcheck_reason,
                                   problem.guide_path.size(),
                                   problem.sfcs.size(),
                                   problem.target_prediction.size(),
                                   candidate_pos.getTotalDuration(),
                                   problem.safe_distance,
                                   problem.use_corridor,
                                   narrow_adjusted);
                    return false;
                }
                std::string commit_safety_precheck_reason;
                if (!candidatePassesOptimizationCommitSafety(candidate_pos,
                                                             &commit_safety_precheck_reason)) {
                    attempt_failures.emplace_back(
                            fmt::format("{} rejected_by_commit_safety_precheck(reason={}, guide={}, sfc={}, target={}, duration={:.3f}, safe_distance={:.3f}, use_corridor={})",
                                        attempt_name,
                                        commit_safety_precheck_reason,
                                        problem.guide_path.size(),
                                        problem.sfcs.size(),
                                        problem.target_prediction.size(),
                                        candidate_pos.getTotalDuration(),
                                        problem.safe_distance,
                                        problem.use_corridor));
                    if (recovery_attempt) {
                        ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_RECOVERY_FAILED attempt={}, reason=commit_safety_precheck_failed:{}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, safe_distance={:.3f}, use_corridor={}, narrow_adjusted={}",
                                       attempt_name,
                                       commit_safety_precheck_reason,
                                       problem.guide_path.size(),
                                       problem.sfcs.size(),
                                       problem.target_prediction.size(),
                                       candidate_pos.getTotalDuration(),
                                       problem.safe_distance,
                                       problem.use_corridor,
                                       narrow_adjusted);
                    }
                    return false;
                }
                out_traj = candidate_pos;
                out_yaw_traj = candidate_yaw;
                if (accepted_target_prediction != nullptr) {
                    *accepted_target_prediction = check_target_prediction;
                }
                if (accepted_reacquire_fov_relax != nullptr) {
                    *accepted_reacquire_fov_relax = allow_reacquire_fov_relax;
                }
                if (recovery_attempt) {
                    ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_RECOVERY_SUCCESS attempt={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, safe_distance={:.3f}, use_corridor={}, narrow_adjusted={}",
                                   attempt_name,
                                   problem.guide_path.size(),
                                   problem.sfcs.size(),
                                   problem.target_prediction.size(),
                                   out_traj.getTotalDuration(),
                                   problem.safe_distance,
                                   problem.use_corridor,
                                   narrow_adjusted);
                }
                return true;
            }

            attempt_failures.emplace_back(
                    fmt::format("{} failed(ok={}, empty={}, guide={}, sfc={}, target={}, safe_distance={:.3f}, use_corridor={})",
                                attempt_name,
                                ok,
                                candidate_pos.empty(),
                                problem.guide_path.size(),
                                problem.sfcs.size(),
                                problem.target_prediction.size(),
                                problem.safe_distance,
                                problem.use_corridor));
            if (recovery_attempt) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_RECOVERY_FAILED attempt={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, safe_distance={:.3f}, use_corridor={}, narrow_adjusted={}",
                               attempt_name,
                               problem.guide_path.size(),
                               problem.sfcs.size(),
                               problem.target_prediction.size(),
                               candidate_pos.empty() ? 0.0 : candidate_pos.getTotalDuration(),
                               problem.safe_distance,
                               problem.use_corridor,
                               narrow_adjusted);
            }
            return false;
        };

        traj_opt::TrackingProblem normal = normal_problem;
        normal.safe_distance = std::max(0.0, normal.safe_distance);
        if (runOptimization("normal", normal, false)) {
            return true;
        }

        if (!cfg_.tracking_recovery_enable) {
            setFailureReason(failure_reason,
                             attempt_failures.empty() ? "normal optimization failed"
                                                      : attempt_failures.back());
            return false;
        }

        auto truncateRecoveryProblem = [&](traj_opt::TrackingProblem &problem,
                                           const double horizon) {
            const double h = std::max(0.3, horizon);
            if (!problem.target_prediction.empty() &&
                problem.target_prediction.back().t > h) {
                traj_opt::DynamicTargetStates truncated;
                for (const auto &state : problem.target_prediction) {
                    if (state.t <= h + 1.0e-6) {
                        truncated.emplace_back(state);
                    }
                }
                const auto horizon_state = interpolateTargetPrediction(problem.target_prediction, h);
                if (truncated.empty() || std::abs(truncated.back().t - h) > 1.0e-4) {
                    truncated.emplace_back(horizon_state);
                }
                problem.target_prediction = std::move(truncated);
            }

            if (problem.guide_path.size() >= 2 &&
                problem.guide_t.size() == problem.guide_path.size() &&
                problem.guide_t.back() > h) {
                vec_Vec3f guide;
                std::vector<double> guide_t;
                guide.reserve(problem.guide_path.size());
                guide_t.reserve(problem.guide_t.size());
                appendGuideTimedUnique(problem.guide_path.front(), problem.guide_t.front(), guide, guide_t);
                for (std::size_t i = 1; i < problem.guide_path.size(); ++i) {
                    if (problem.guide_t[i] <= h + 1.0e-6) {
                        appendGuideTimedUnique(problem.guide_path[i], problem.guide_t[i], guide, guide_t);
                        continue;
                    }
                    const double t0 = problem.guide_t[i - 1];
                    const double t1 = problem.guide_t[i];
                    if (h > t0 + 1.0e-6 && t1 > t0 + 1.0e-6) {
                        const double alpha = std::clamp((h - t0) / (t1 - t0), 0.0, 1.0);
                        const Vec3f p = problem.guide_path[i - 1] +
                                        alpha * (problem.guide_path[i] - problem.guide_path[i - 1]);
                        appendGuideTimedUnique(p, h, guide, guide_t);
                    }
                    break;
                }
                if (guide.size() >= 2 && guide.size() == guide_t.size()) {
                    problem.guide_path = std::move(guide);
                    problem.guide_t = std::move(guide_t);
                    refreshTrackingGuideEndpoint(problem);
                }
            }

            if (!problem.target_prediction.empty()) {
                problem.min_total_duration =
                        std::max(0.6,
                                 std::min(problem.min_total_duration,
                                          problem.target_prediction.back().t));
            }
        };

        traj_opt::TrackingProblem recovery = normal_problem;
        truncateRecoveryProblem(recovery, cfg_.tracking_recovery_horizon);
        recovery.distance_tolerance *= std::max(1.0, cfg_.tracking_recovery_distance_tolerance_scale);
        recovery.height_tolerance *= std::max(1.0, cfg_.tracking_recovery_height_tolerance_scale);
        recovery.od_h_lower =
                std::max(0.05,
                         recovery.tracking_distance - recovery.distance_tolerance);
        recovery.od_h_upper =
                std::max(recovery.od_h_lower + 0.05,
                         recovery.tracking_distance + recovery.distance_tolerance);
        recovery.od_v_lower = recovery.height_offset - recovery.height_tolerance;
        recovery.od_v_upper = recovery.height_offset + recovery.height_tolerance;
        recovery.min_total_duration =
                std::max(0.6,
                         recovery.min_total_duration *
                         std::max(1.0, cfg_.tracking_recovery_time_scale));
        if (!recovery.target_prediction.empty()) {
            recovery.min_total_duration =
                    std::max(recovery.min_total_duration,
                             recovery.target_prediction.back().t);
        }
        recovery.weight_visible_region *=
                std::clamp(cfg_.tracking_recovery_reduce_visible_region_weight, 0.0, 1.0);
        recovery.weight_target_forward *=
                std::clamp(cfg_.tracking_recovery_reduce_target_forward_weight, 0.0, 1.0);

        if (runOptimization("recovery_relaxed", recovery, true)) {
            return true;
        }

        auto applyAdaptiveOcclusionRecovery = [&](traj_opt::TrackingProblem &problem) {
            problem.adaptive_occlusion_enable = true;
            problem.weight_oe *=
                    std::max(1.0, cfg_.tracking_adaptive_occlusion_recovery_oe_scale);
            problem.weight_visibility = problem.weight_oe;
            problem.weight_od_far *=
                    std::max(1.0, cfg_.tracking_adaptive_occlusion_od_far_weight_scale);
            problem.visibility_samples =
                    std::max(problem.visibility_samples,
                             std::max(7, 2 * cfg_.tracking_visibility_samples - 1));
            if (problem.joint_sample_dt > 0.0) {
                problem.joint_sample_dt =
                        std::min(problem.joint_sample_dt,
                                 std::max(0.02, cfg_.tracking_fov_check_dt));
            }
            const double adaptive_upper =
                    std::max(problem.od_h_lower + 0.05,
                             std::max(problem.adaptive_occlusion_min_horizontal_upper,
                                      problem.tracking_distance *
                                      std::clamp(problem.adaptive_occlusion_distance_upper_scale,
                                                 0.1,
                                                 1.0)));
            problem.od_h_upper = std::min(problem.od_h_upper, adaptive_upper);
            problem.weight_viewpoint_attractor *= 0.65;
            problem.weight_visible_region *= 0.8;
        };

        traj_opt::TrackingProblem adaptive_recovery = recovery;
        if (cfg_.tracking_adaptive_occlusion_enable) {
            applyAdaptiveOcclusionRecovery(adaptive_recovery);
            if (runOptimization("recovery_adaptive_occlusion", adaptive_recovery, true)) {
                return true;
            }
        }

        traj_opt::TrackingProblem obstacle_recovery =
                cfg_.tracking_adaptive_occlusion_enable ? adaptive_recovery : recovery;
        obstacle_recovery.safe_distance =
                std::max(trackingHardSafeDistance(cfg_),
                         cfg_.tracking_safe_distance *
                         std::clamp(cfg_.tracking_narrow_passage_soft_safe_distance_scale, 0.1, 1.0));
        if (runOptimization("recovery_narrow_corridor", obstacle_recovery, true)) {
            return true;
        }

        if (cfg_.tracking_retry_without_corridor_enable && normal_problem.use_corridor) {
            traj_opt::TrackingProblem no_corridor = obstacle_recovery;
            no_corridor.use_corridor = false;
            no_corridor.sfcs.clear();
            if (runOptimization("recovery_without_corridor", no_corridor, true)) {
                return true;
            }
        }

        if (cfg_.tracking_reacquire_recovery_enable) {
            traj_opt::TrackingProblem reacquire_recovery = obstacle_recovery;
            reacquire_recovery.reacquire_mode = true;
            const double dynamic_reacquire_horizon =
                    estimateReacquireTransitHorizon(reacquire_recovery);
            truncateRecoveryProblem(reacquire_recovery, dynamic_reacquire_horizon);
            const double visible_scale =
                    std::clamp(cfg_.tracking_reacquire_visible_region_weight_scale, 0.0, 1.0);
            reacquire_recovery.weight_visible_region *= visible_scale;
            if (visible_scale <= 1.0e-6) {
                reacquire_recovery.use_visible_region = false;
            }
            if (runOptimization("recovery_reacquire_short", reacquire_recovery, true)) {
                return true;
            }

            if (cfg_.tracking_reacquire_transit_enable) {
                traj_opt::TrackingProblem reacquire_transit = reacquire_recovery;
                reacquire_transit.use_corridor = false;
                reacquire_transit.sfcs.clear();
                reacquire_transit.use_visible_region = false;
                reacquire_transit.weight_visible_region = 0.0;
                reacquire_transit.weight_target_forward *= 0.35;
                reacquire_transit.weight_viewpoint_attractor *= 1.35;
                reacquire_transit.distance_tolerance *= 1.35;
                reacquire_transit.height_tolerance *= 1.35;
                reacquire_transit.od_h_lower =
                        std::max(0.05,
                                 reacquire_transit.tracking_distance -
                                         reacquire_transit.distance_tolerance);
                reacquire_transit.od_h_upper =
                        std::max(reacquire_transit.od_h_lower + 0.05,
                                 reacquire_transit.tracking_distance +
                                         reacquire_transit.distance_tolerance);
                reacquire_transit.od_v_lower =
                        reacquire_transit.height_offset -
                        reacquire_transit.height_tolerance;
                reacquire_transit.od_v_upper =
                        reacquire_transit.height_offset +
                        reacquire_transit.height_tolerance;
                reacquire_transit.min_total_duration =
                        std::max(reacquire_transit.min_total_duration,
                                 dynamic_reacquire_horizon);
                if (runOptimization("recovery_reacquire_transit",
                                    reacquire_transit,
                                    true)) {
                    return true;
                }
            }
        }

        std::string joined;
        for (const std::string &failure : attempt_failures) {
            if (!joined.empty()) {
                joined += "; ";
            }
            joined += failure;
        }
        setFailureReason(failure_reason,
                         joined.empty() ? "all tracking optimization attempts failed" : joined);
        ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_FAILED reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, prefix_duration=0.000, runtime_eval_start=0.000",
                       failure_reason != nullptr ? *failure_reason : joined,
                       normal_problem.guide_path.size(),
                       normal_problem.sfcs.size(),
                       normal_problem.target_prediction.size(),
                       out_traj.empty() ? 0.0 : out_traj.getTotalDuration());
        return false;
    }

    RET_CODE GeneralPlanner::optimizeTrackingTask(const traj_opt::DynamicTargetStates &target_prediction,
                                                const bool &from_rest) {
        if (target_prediction.empty()) {
            setTrackingDiagnostic("input",
                                  "empty_target_prediction",
                                  0,
                                  0,
                                  0,
                                  0.0);
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking task has no target prediction.");
            return FAILED;
        }

        auto failOrKeepOld = [this, &target_prediction, &from_rest](const std::string &reason) -> RET_CODE {
            if (keepOldTrackingTrajectoryIfActive(target_prediction, reason)) {
                return NO_NEED;
            }
            const bool lost_or_from_rest =
                    from_rest || getCommittedTrajectoryRemainingDuration() <= 1.0e-3;
            if (lost_or_from_rest &&
                commitTrackingHoldTrajectory("tracking recovery hold after failure: " + reason)) {
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
        frontend_cfg.distance_lower_tolerance =
                static_tracking ? static_distance_tolerance
                                : cfg_.tracking_distance_lower_tolerance;
        frontend_cfg.distance_upper_tolerance =
                static_tracking ? static_distance_tolerance
                                : cfg_.tracking_distance_upper_tolerance;
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
        frontend_cfg.fov_feasibility_enable =
                cfg_.tracking_frontend_fov_feasibility_enable &&
                cfg_.tracking_fov_check_strict;
        frontend_cfg.yaw_rate_feasibility_enable = cfg_.tracking_frontend_yaw_rate_feasibility_enable;
        const double effective_tracking_fov_range = trackingAdaptiveFovRange(cfg_);
        frontend_cfg.fov_horizontal_deg = cfg_.tracking_fov_horizontal_deg;
        frontend_cfg.fov_vertical_deg = cfg_.tracking_fov_vertical_deg;
        frontend_cfg.fov_range = effective_tracking_fov_range;
        frontend_cfg.fov_range_margin = cfg_.tracking_frontend_fov_range_margin;
        frontend_cfg.fov_front_margin = cfg_.tracking_fov_front_margin;
        frontend_cfg.max_yaw_rate = cfg_.yaw_dot_max;
        frontend_cfg.yaw_rate_margin = cfg_.tracking_frontend_yaw_rate_margin;
        frontend_cfg.obstacle_recovery_enable = cfg_.tracking_frontend_obstacle_recovery_enable;
        frontend_cfg.grid_neighbor_mode = cfg_.tracking_frontend_grid_neighbor_mode;
        frontend_cfg.over_wall_enable = cfg_.tracking_frontend_over_wall_enable;
        frontend_cfg.over_wall_max_climb = cfg_.tracking_frontend_over_wall_max_climb;
        frontend_cfg.side_pass_enable = cfg_.tracking_frontend_side_pass_enable;
        frontend_cfg.side_pass_width = cfg_.tracking_frontend_side_pass_width;
        frontend_cfg.reacquire_relax_yaw_rate = cfg_.tracking_frontend_reacquire_relax_yaw_rate;
        frontend_cfg.unknown_as_occupied = cfg_.tracking_unknown_as_occupied;
        frontend_cfg.use_astar = cfg_.tracking_frontend_astar;
        frontend_cfg.use_visible_region = cfg_.tracking_use_visible_region;
        frontend_cfg.print_log = cfg_.print_log;

        traj_opt::TrackingProblem problem;
        TimeConsuming t_frontend("tracking_frontend", false);
        const double tracking_plan_start_wt = ros_ptr_->getSimTime();
        const double tracking_candidate_head_wt =
                tracking_plan_start_wt + std::max(0.0, cfg_.replan_forward_dt);
        const StatePVAJ head_state = makeTaskHeadState(from_rest, tracking_candidate_head_wt);
        const bool has_committed_tracking_for_frontend =
                cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_
                    ? tracking_runtime_manager_->hasCommittedTracking()
                    : !cmd_traj_info_.empty();
        const double initial_target_distance =
                (head_state.col(0) - target_prediction.front().position).head<2>().norm();
        const double frontend_fov_range = effective_tracking_fov_range;
        const double frontend_reacquire_entry_distance =
                std::max({frontend_fov_range,
                          std::max(0.0, cfg_.tracking_reacquire_distance),
                          std::max(0.0, cfg_.tracking_reacquire_fov_entry_distance)});
        const bool frontend_reacquire_mode =
                cfg_.tracking_reacquire_fov_relax_enable &&
                (from_rest ||
                 !has_committed_tracking_for_frontend ||
                 initial_target_distance > frontend_reacquire_entry_distance);
        if (frontend_reacquire_mode && frontend_cfg.fov_feasibility_enable) {
            frontend_cfg.fov_feasibility_enable = false;
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_FRONTEND_FOV_FEASIBILITY_DISABLED reason=reacquire_or_uncommitted, from_rest={}, has_committed_tracking={}, initial_dist={:.3f}, entry_dist={:.3f}",
                               from_rest,
                               has_committed_tracking_for_frontend,
                               initial_target_distance,
                               frontend_reacquire_entry_distance);
            }
        }
        TrackingFrontend frontend(frontend_cfg, map_manager_, astar_ptr_);
        Vec3f reference_viewpoint = Vec3f::Zero();
        traj_opt::DynamicTargetState reference_target;
        const bool has_viewpoint_reference =
                findTrackingViewpointReference(target_prediction,
                                               reference_viewpoint,
                                               reference_target);
        if (!frontend.buildProblem(head_state,
                                   target_prediction,
                                   problem,
                                   has_viewpoint_reference ? &reference_viewpoint : nullptr,
                                   has_viewpoint_reference ? &reference_target : nullptr)) {
            time_consuming_[EPX_TRAJ_FRONTEND] = t_frontend.stop();
            setTrackingDiagnostic("frontend",
                                  "frontend_failed",
                                  problem,
                                  0.0);
            ros_ptr_->warn(" -- [Tracking] TRACKING_FRONTEND_FAILED guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration=0.000, prefix_duration=0.000, runtime_eval_start=0.000, old_remaining=0.000, old_activity.reason=frontend_failed",
                           problem.guide_path.size(),
                           problem.sfcs.size(),
                           problem.target_prediction.size());
            return failOrKeepOld("Tracking frontend failed.");
        }
        problem.static_tracking_mode = static_tracking;
        const double tracking_frontend_t = t_frontend.stop();
        time_consuming_[EPX_TRAJ_FRONTEND] = tracking_frontend_t;

        TimeConsuming t_tracking_sfc("tracking_sfc", false);
        std::string tracking_sfc_failure_reason;
        if (!buildTrackingGuideCorridor(problem, &tracking_sfc_failure_reason)) {
            time_consuming_[EPX_TRAJ_FRONTEND] += t_tracking_sfc.stop();
            setTrackingDiagnostic("sfc",
                                  tracking_sfc_failure_reason,
                                  problem,
                                  0.0);
            ros_ptr_->warn(" -- [Tracking] TRACKING_SFC_FAILED reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration=0.000, prefix_duration=0.000, runtime_eval_start=0.000",
                           tracking_sfc_failure_reason,
                           problem.guide_path.size(),
                           problem.sfcs.size(),
                           problem.target_prediction.size());
            return failOrKeepOld("Tracking SFC generation failed: " + tracking_sfc_failure_reason);
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

            const double eval_t = tracking_candidate_head_wt - start_wt;
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
        problem.weight_fov = cfg_.tracking_weight_fov;
        problem.weight_target_forward = cfg_.tracking_weight_target_forward;
        problem.adaptive_occlusion_enable = cfg_.tracking_adaptive_occlusion_enable;
        problem.adaptive_occlusion_activation_distance = cfg_.tracking_adaptive_occlusion_activation_distance;
        problem.adaptive_occlusion_max_weight_scale = cfg_.tracking_adaptive_occlusion_max_weight_scale;
        problem.adaptive_occlusion_od_far_weight_scale = cfg_.tracking_adaptive_occlusion_od_far_weight_scale;
        problem.adaptive_occlusion_distance_upper_scale = cfg_.tracking_adaptive_occlusion_distance_upper_scale;
        problem.adaptive_occlusion_min_horizontal_upper = cfg_.tracking_adaptive_occlusion_min_horizontal_upper;
        constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
        problem.fov_horizontal = std::max(1.0, cfg_.tracking_fov_horizontal_deg) * kDegToRad;
        problem.fov_vertical = std::max(1.0, cfg_.tracking_fov_vertical_deg) * kDegToRad;
        problem.fov_range = effective_tracking_fov_range;
        problem.fov_range_margin = cfg_.tracking_fov_range_margin;
        problem.fov_front_margin = cfg_.tracking_fov_front_margin;
        problem.target_front_margin = cfg_.tracking_target_front_margin;
        problem.target_motion_speed_threshold =
                std::max(0.0, cfg_.tracking_no_motion_target_speed_threshold);
        problem.joint_sample_dt = cfg_.tracking_joint_sample_dt;
        problem.dense_joint_sample_enable = cfg_.tracking_dense_joint_sample_enable;
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
        traj_opt::DynamicTargetStates accepted_target_prediction = active_target_prediction;
        bool accepted_reacquire_fov_relax = false;
        TimeConsuming t_opt("tracking_opt", false);
        std::string tracking_opt_failure_reason;
        const bool ok = optimizeTrackingProblemWithRetries(problem,
                                                           active_target_prediction,
                                                           out_traj,
                                                           out_yaw_traj,
                                                           &accepted_target_prediction,
                                                           &accepted_reacquire_fov_relax,
                                                           &tracking_opt_failure_reason);
        time_consuming_[EXP_TRAJ_OPT] = t_opt.stop();
        if (!ok || out_traj.empty()) {
            setTrackingDiagnostic("optimizer",
                                  tracking_opt_failure_reason,
                                  problem,
                                  out_traj.empty() ? 0.0 : out_traj.getTotalDuration());
            ros_ptr_->warn(" -- [Tracking] TRACKING_OPT_FAILED reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, out_traj_duration={:.3f}, prefix_duration=0.000, runtime_eval_start=0.000",
                           tracking_opt_failure_reason,
                           problem.guide_path.size(),
                           problem.sfcs.size(),
                           problem.target_prediction.size(),
                           out_traj.empty() ? 0.0 : out_traj.getTotalDuration());
            const std::string failure_message =
                    "Tracking optimization failed: " + tracking_opt_failure_reason;
            return failOrKeepOld(failure_message);
        }
        setTrackingDiagnostic("candidate",
                              "optimizer_success",
                              problem,
                              out_traj.getTotalDuration());
        if (out_traj.getTotalDuration() < cfg_.tracking_min_commit_duration) {
            setTrackingDiagnostic("commit_guard",
                                  fmt::format("trajectory_too_short:{:.3f}<{}",
                                              out_traj.getTotalDuration(),
                                              cfg_.tracking_min_commit_duration),
                                  problem,
                                  out_traj.getTotalDuration());
            return failOrKeepOld(fmt::format("Tracking trajectory too short ({:.3f}s < {:.3f}s).",
                                             out_traj.getTotalDuration(),
                                             cfg_.tracking_min_commit_duration));
        }

        const double candidate_guard_h =
                std::min(cfg_.tracking_no_motion_check_horizon,
                         out_traj.getTotalDuration());
        const TrackingMotionMetrics candidate_metrics =
                computeTrackingMotionMetrics(out_traj,
                                             accepted_target_prediction,
                                             cfg_,
                                             0.0,
                                             0.0,
                                             candidate_guard_h);
        double old_remaining = 0.0;
        double old_speed0 = 0.0;
        double old_speed_z = 0.0;
        double old_speed_3d = 0.0;
        double old_displacement = 0.0;
        double old_displacement_z = 0.0;
        double old_displacement_3d = 0.0;
        double old_progress = 0.0;
        double old_progress_3d = 0.0;
        double old_expected_progress = 0.0;
        double old_avg_tracking_error = 0.0;
        std::string old_activity_reason = "none";
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
                            accepted_target_prediction,
                            cfg_.tracking_keep_old_horizon,
                            cfg_.tracking_keep_old_safety_dt);
            old_remaining = old_activity.remaining;
            old_speed0 = old_activity.speed0;
            old_speed_z = old_activity.speed_z;
            old_speed_3d = old_activity.speed_3d;
            old_displacement = old_activity.displacement;
            old_displacement_z = old_activity.displacement_z;
            old_displacement_3d = old_activity.displacement_3d;
            old_progress = old_activity.progress;
            old_progress_3d = old_activity.progress_3d;
            old_expected_progress = old_activity.expected_progress;
            old_avg_tracking_error = old_activity.avg_tracking_error;
            old_activity_reason = old_activity.reason;
        } else if (!cfg_.tracking_runtime_manager_enable) {
            TrackingTrajectoryActivity old_activity;
            currentTrackingTrajectorySafeAndActive(accepted_target_prediction, &old_activity);
            old_remaining = old_activity.remaining;
            old_speed0 = old_activity.speed0;
            old_speed_z = old_activity.speed_z;
            old_speed_3d = old_activity.speed_3d;
            old_displacement = old_activity.displacement;
            old_displacement_z = old_activity.displacement_z;
            old_displacement_3d = old_activity.displacement_3d;
            old_progress = old_activity.progress;
            old_progress_3d = old_activity.progress_3d;
            old_expected_progress = old_activity.expected_progress;
            old_avg_tracking_error = old_activity.avg_tracking_error;
            old_activity_reason = old_activity.reason;
        }
        if (cfg_.print_log) {
            ros_ptr_->info(" -- [Tracking] TRACKING_CANDIDATE_OPT_SUCCESS guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, candidate_duration={:.3f}, prefix_duration=0.000, runtime_eval_start=0.000, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, target_speed_z={:.3f}, old_remaining={:.3f}, old_activity.reason={}, old_speed_xy={:.3f}, old_speed_z={:.3f}, old_speed_3d={:.3f}, old_displacement_xy={:.3f}, old_displacement_z={:.3f}, old_displacement_3d={:.3f}, old_progress_xy={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                           problem.guide_path.size(),
                           problem.sfcs.size(),
                           problem.target_prediction.size(),
                           out_traj.getTotalDuration(),
                           candidate_metrics.displacement_xy,
                           candidate_metrics.displacement_z,
                           candidate_metrics.displacement_3d,
                           candidate_metrics.speed_xy,
                           candidate_metrics.speed_z,
                           candidate_metrics.speed_3d,
                           candidate_metrics.target_speed_z,
                           old_remaining,
                           old_activity_reason,
                           old_speed0,
                           old_speed_z,
                           old_speed_3d,
                           old_displacement,
                           old_displacement_z,
                           old_displacement_3d,
                           old_progress,
                           old_progress_3d,
                           old_expected_progress,
                           old_avg_tracking_error);
        }

        std::string commandable_reject_reason;
        if (!cfg_.tracking_runtime_manager_enable &&
            !candidateTrackingTrajectoryCommandable(out_traj,
                                                    accepted_target_prediction,
                                                    0.0,
                                                    0.0,
                                                    &commandable_reject_reason)) {
            TrackingTrajectoryActivity old_activity;
            currentTrackingTrajectorySafeAndActive(accepted_target_prediction, &old_activity);
            if (cfg_.print_log) {
                ros_ptr_->warn(" -- [Tracking] TRACKING_CANDIDATE_REJECTED_NO_MOTION reject_reason={}, guide_path.size()={}, problem.sfcs.size()={}, problem.target_prediction.size()={}, candidate_duration={:.3f}, prefix_duration=0.000, runtime_eval_start=0.000, candidate_disp_xy={:.3f}, candidate_disp_z={:.3f}, candidate_disp_3d={:.3f}, candidate_speed_xy={:.3f}, candidate_speed_z={:.3f}, candidate_speed_3d={:.3f}, old_remaining={:.3f}, old_activity.reason={}, old_speed_3d={:.3f}, old_displacement_3d={:.3f}, old_progress_3d={:.3f}, old_expected_progress={:.3f}, old_avg_tracking_error={:.3f}",
                               commandable_reject_reason,
                               problem.guide_path.size(),
                               problem.sfcs.size(),
                               problem.target_prediction.size(),
                               out_traj.getTotalDuration(),
                               candidate_metrics.displacement_xy,
                               candidate_metrics.displacement_z,
                               candidate_metrics.displacement_3d,
                               candidate_metrics.speed_xy,
                               candidate_metrics.speed_z,
                               candidate_metrics.speed_3d,
                               old_activity.remaining,
                               old_activity.reason,
                               old_activity.speed_3d,
                               old_activity.displacement_3d,
                               old_activity.progress_3d,
                               old_activity.expected_progress,
                               old_activity.avg_tracking_error);
            }

            if (old_activity.active &&
                keepOldTrackingTrajectoryIfActive(active_target_prediction,
                                                  "tracking candidate rejected by no-motion guard")) {
                return NO_NEED;
            }

            std::string candidate_safe_reason;
            std::string candidate_safe_detail;
            const bool candidate_safe_for_commit =
                    trackingCandidateSafeForCommit(out_traj,
                                                  &candidate_safe_reason,
                                                  &candidate_safe_detail);
            if (old_activity.valid &&
                !old_activity.safe &&
                candidate_safe_for_commit) {
                if (cfg_.print_log) {
                    ros_ptr_->warn(" -- [Tracking] no-motion guard allows safe candidate because old trajectory is unsafe. old_reason={}",
                                   old_activity.reason);
                }
            } else {
                setTrackingCommitRejectInfo(
                        "candidate rejected by no-motion guard: " + commandable_reject_reason,
                        fmt::format(
                                "failure=no_motion_guard|candidate_safe_for_commit={}|candidate_safe_reason={}|candidate_safe_detail={}|old_activity_valid={}|old_activity_safe={}|old_activity_active={}|old_activity_reason={}|candidate_duration={:.3f}",
                                static_cast<int>(candidate_safe_for_commit),
                                candidate_safe_reason.empty() ? "none" : candidate_safe_reason,
                                candidate_safe_detail.empty() ? "none" : candidate_safe_detail,
                                static_cast<int>(old_activity.valid),
                                static_cast<int>(old_activity.safe),
                                static_cast<int>(old_activity.active),
                                old_activity.reason.empty() ? "none" : old_activity.reason,
                                out_traj.getTotalDuration()));
                setTrackingDiagnostic("commit_guard",
                                      "candidate_rejected_no_motion:" + commandable_reject_reason,
                                      problem,
                                      out_traj.getTotalDuration());
                return failOrKeepOld("Tracking candidate rejected by no-motion guard: " +
                                     commandable_reject_reason);
            }
        }

        {
            TimeConsuming t_viz("tracking_fov_viz", false);
            ros_ptr_->vizTrackingFov(out_traj,
                                     out_yaw_traj,
                                     cfg_.tracking_fov_horizontal_deg,
                                     cfg_.tracking_fov_vertical_deg,
                                     effective_tracking_fov_range);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        if (!commitTrackingTrajectory(out_traj,
                                      out_yaw_traj,
                                      accepted_target_prediction,
                                      cfg_.tracking_use_snap ? "tracking_snap" : "tracking_jerk",
                                      tracking_candidate_head_wt,
                                      accepted_reacquire_fov_relax,
                                      !from_rest)) {
            const std::string commit_reason =
                    last_tracking_commit_reject_reason_.empty()
                        ? "unknown"
                        : last_tracking_commit_reject_reason_;
            setTrackingDiagnostic("commit",
                                  commit_reason,
                                  problem,
                                  out_traj.getTotalDuration());
            if (keepOldTrackingTrajectoryIfActive(active_target_prediction,
                                                  "tracking trajectory commit rejected: " + commit_reason)) {
                return NO_NEED;
            }
            ros_ptr_->warn(" -- [GeneralPlanner] Tracking trajectory commit rejected: reason={}.", commit_reason);
            return failOrKeepOld("Tracking trajectory commit rejected: " + commit_reason);
        }
        rememberTrackingViewpointReference(problem);
        setTrackingDiagnostic("success",
                              "candidate_committed",
                              problem,
                              out_traj.getTotalDuration());
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

    RET_CODE GeneralPlanner::TryCommitPerchingFromTracking(
            const traj_opt::DynamicTargetStates &target_prediction,
            const traj_opt::PerchingSurfaceState &surface,
            const RET_CODE tracking_ret) {
        TimeConsuming total_t("TryCommitPerchingFromTracking", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        const RET_CODE ret =
                tryCommitPerchingFromTracking(target_prediction, surface, tracking_ret);
        time_consuming_[TOTAL_REPLAN] += total_t.stop();
        return ret;
    }

    RET_CODE GeneralPlanner::PlanTrackingFromRest(const traj_opt::DynamicTargetStates &target_prediction,
                                                const bool &new_task) {
        TimeConsuming total_t("PlanTrackingFromRest", false);
        std::lock_guard<std::mutex> guard(replan_lock_);
        latest_replan.reset();
        setTrackingDiagnostic("plan_from_rest",
                              "start",
                              0,
                              0,
                              target_prediction.size(),
                              0.0);
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            setTrackingDiagnostic("input",
                                  "no_odom",
                                  0,
                                  0,
                                  target_prediction.size(),
                                  0.0);
            ros_ptr_->warn(" -- [GeneralPlanner] in [PlanTrackingFromRest]: No odom, force return.");
            return FAILED;
        }
        const Vec3f goal = target_prediction.empty() ? robot_state_.p : target_prediction.back().position;
        latest_replan.setGoal(goal, target_prediction.empty() ? NAN : target_prediction.back().yaw, robot_state_);
        gi_.goal_p = goal;
        gi_.goal_yaw = target_prediction.empty() ? NAN : target_prediction.back().yaw;
        gi_.new_goal = new_task;
        last_exp_traj_info_.setEmpty();
        maybeResetTrackingRuntimeForReplan(new_task, "tracking_plan_from_rest");
        if (new_task &&
            tracking_perching_manager_ &&
            !tracking_perching_manager_->perchingRequested() &&
            !trackingPerchingPerchingStatus(tracking_perching_manager_->status())) {
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
        setTrackingDiagnostic("replan",
                              "start",
                              0,
                              0,
                              target_prediction.size(),
                              0.0);
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            setTrackingDiagnostic("input",
                                  "no_odom",
                                  0,
                                  0,
                                  target_prediction.size(),
                                  0.0);
            return FAILED;
        }
        if (tracking_perching_manager_ &&
            (tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::PERCHING_COMMITTED ||
             tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::PERCHING_EXECUTING ||
             tracking_perching_manager_->status() ==
                 TrackingPerchingTransitionManager::Status::CONTACT_IMMINENT)) {
            latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
            setTrackingDiagnostic("tracking_perching",
                                  "perching_owns_committed_trajectory",
                                  0,
                                  0,
                                  target_prediction.size(),
                                  0.0);
            ros_ptr_->info(" -- [TrackingPerching] TRACKING_REPLAN_BLOCKED_PERCHING_ACTIVE reason=perching_owns_committed_trajectory");
            time_consuming_[TOTAL_REPLAN] = total_t.stop();
            return SUCCESS;
        }
        const Vec3f goal = target_prediction.empty() ? robot_state_.p : target_prediction.back().position;
        latest_replan.setGoal(goal, target_prediction.empty() ? NAN : target_prediction.back().yaw, robot_state_);
        gi_.goal_p = goal;
        gi_.goal_yaw = target_prediction.empty() ? NAN : target_prediction.back().yaw;
        gi_.new_goal = new_task;
        maybeResetTrackingRuntimeForReplan(new_task, "tracking_replan");

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
        setTrackingDiagnostic("replan",
                              "start",
                              0,
                              0,
                              target_prediction.size(),
                              0.0);
        if (!robot_state_.rcv) {
            latest_replan.setRetCode(GENERAL_RET_CODE::GENERAL_NO_ODOM);
            setTrackingDiagnostic("input",
                                  "no_odom",
                                  0,
                                  0,
                                  target_prediction.size(),
                                  0.0);
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

        maybeResetTrackingRuntimeForReplan(new_task, "tracking_perching_replan");

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

    bool GeneralPlanner::commitTrackingHoldTrajectory(const std::string &reason,
                                                      const double duration) {
        if (!robot_state_.rcv || !robot_state_.p.allFinite()) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMIT_FAILED reason={}, robot_state_valid=0",
                           reason);
            return false;
        }

        const double commit_wt = ros_ptr_->getSimTime();
        const double hold_duration =
                std::max(0.2,
                         std::isfinite(duration) && duration > 1.0e-5
                             ? duration
                             : std::max(0.8, cfg_.tracking_min_commit_duration));
        Trajectory hold_pos_traj;
        Trajectory hold_yaw_traj;
        const double hold_yaw =
                std::isfinite(robot_state_.yaw) ? robot_state_.yaw : 0.0;
        if (!buildConstantPositionTrajectory(robot_state_.p,
                                             hold_duration,
                                             commit_wt,
                                             hold_pos_traj) ||
            !buildConstantYawTrajectory(hold_yaw,
                                        hold_duration,
                                        commit_wt,
                                        hold_yaw_traj)) {
            ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMIT_FAILED reason={}, build_constant_traj=0",
                           reason);
            return false;
        }

        std::string safety_reason;
        std::string safety_detail;
        const double safety_horizon =
                std::min(std::max(0.0, cfg_.tracking_keep_old_horizon),
                         hold_pos_traj.getTotalDuration());
        const bool hold_safe =
                trackingTrajectorySafeForHorizonDetailed(hold_pos_traj,
                                                         0.0,
                                                         safety_horizon,
                                                         cfg_.tracking_keep_old_safety_dt,
                                                         &safety_reason,
                                                         &safety_detail);

        ExpTraj hold_exp_traj;
        hold_exp_traj.setGoalConnectedFlag(false);
        hold_exp_traj.setWholeTrajKnownFreeFlag(hold_safe);
        hold_exp_traj.setTrajectory(commit_wt, hold_pos_traj, hold_yaw_traj);

        cmd_traj_info_.setTrajectory(hold_exp_traj);
        last_exp_traj_info_ = hold_exp_traj;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;

        latest_replan.setExpTraj(hold_pos_traj);
        latest_replan.setExpYawTraj(hold_yaw_traj);
        latest_replan.setRetCode(GENERAL_SUCCESS_NO_BACKUP);
        setTrackingDiagnostic("recovery_hold",
                              fmt::format("reason={};hold_safe={};safety_reason={};safety_detail={}",
                                          reason,
                                          static_cast<int>(hold_safe),
                                          safety_reason.empty() ? "none" : safety_reason,
                                          safety_detail.empty() ? "none" : safety_detail),
                              0,
                              0,
                              0,
                              hold_duration);

        if (cfg_.tracking_runtime_manager_enable && tracking_runtime_manager_) {
            tracking_runtime_manager_->onCommitted();
        }
        resetTrackingCommitCounters();
        clearTrackingCommitRejectInfo();

        {
            TimeConsuming t_viz("tracking_hold_viz", false);
            ros_ptr_->vizExpTraj(hold_pos_traj, "tracking_hold");
            ros_ptr_->vizYawTraj(hold_pos_traj, hold_yaw_traj);
            ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1.0);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        ros_ptr_->warn(" -- [Tracking] TRACKING_HOLD_COMMITTED reason={}, duration={:.3f}, hold_safe={}, safety_reason={}, pos={}",
                       reason,
                       hold_duration,
                       hold_safe,
                       safety_reason.empty() ? "none" : safety_reason,
                       robot_state_.p);
        return true;
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

    bool GeneralPlanner::trackingPerchingContactReached() const {
        return tracking_perching_manager_ &&
               tracking_perching_manager_->status() ==
                   TrackingPerchingTransitionManager::Status::CONTACT;
    }

    TrackingPerchingTransitionManager::Status GeneralPlanner::trackingPerchingStatus() const {
        return tracking_perching_manager_
                   ? tracking_perching_manager_->status()
                   : TrackingPerchingTransitionManager::Status::TRACKING_ONLY;
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
        latest_state2state_z_debug_ = State2StateZDebug{};
        latest_state2state_z_debug_.goal_z = gi_.goal_p.z();
        latest_state2state_z_debug_.robot_z = robot_state_.p.z();
        latest_state2state_z_debug_.exp_mode = cfg_.plain_traj_en
                                               ? "plain"
                                               : (cfg_.esdf_traj_en ? "esdf" : "corridor");

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
                        if (!geometry_utils::simplePMTimeAllocator(cfg_.exp_traj_cfg.max_acc,
                                                                   cfg_.exp_traj_cfg.max_vel,
                                                                   guide_path_end_vel,
                                                                   total_dis,
                                                                   dis[i],
                                                                   stamps[i],
                                                                   vel)) {
                            ros_ptr_->warn(" -- [GeneralPlanner] Guide time allocation failed: total_dis={:.6f}, cur_dis={:.6f}, end_vel={:.6f}, max_vel={:.6f}, max_acc={:.6f}.",
                                           total_dis,
                                           dis[i],
                                           guide_path_end_vel,
                                           cfg_.exp_traj_cfg.max_vel,
                                           cfg_.exp_traj_cfg.max_acc);
                            return FAILED;
                        }
                        const double stamp_dt = stamps[i] - last_stamp;
                        if (!std::isfinite(stamp_dt) || stamp_dt < -1.0e-8) {
                            ros_ptr_->warn(" -- [GeneralPlanner] Guide time allocation produced invalid dt: index={}, stamp={:.6f}, last_stamp={:.6f}, dt={:.6f}.",
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
        auto copyZSummary = [](const LocalZSummary &src, ZDebugSummary &dst) {
            dst.valid = src.valid;
            dst.start = src.start;
            dst.end = src.end;
            dst.min = src.min;
            dst.max = src.max;
        };
        latest_state2state_z_debug_.valid = true;
        latest_state2state_z_debug_.guide_size = static_cast<int>(guide_path.size());
        copyZSummary(summarizePathZ(guide_path), latest_state2state_z_debug_.guide);
        latest_state2state_z_debug_.goal_z = gi_.goal_p.z();
        latest_state2state_z_debug_.robot_z = pos_init_state(2, 0);
        latest_state2state_z_debug_.local_target_z = pos_fina_state(2, 0);
        latest_state2state_z_debug_.local_target_goal_dist = (pos_fina_state.col(0) - gi_.goal_p).norm();
        latest_state2state_z_debug_.local_target_goal_xy_dist =
                (pos_fina_state.col(0).head<2>() - gi_.goal_p.head<2>()).norm();
        latest_state2state_z_debug_.local_target_goal_z_err = pos_fina_state(2, 0) - gi_.goal_p.z();
        latest_state2state_z_debug_.local_target_is_global_goal = local_endpoint_is_global_goal;

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
        copyZSummary(summarizeTrajectoryZ(out_traj, cfg_.sample_traj_dt),
                     latest_state2state_z_debug_.optimized);
        if (latest_state2state_z_debug_.optimized.valid) {
            latest_state2state_z_debug_.opt_end_local_target_z_err =
                    latest_state2state_z_debug_.optimized.end -
                    latest_state2state_z_debug_.local_target_z;
        }
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
        copyZSummary(summarizeTrajectoryZ(temp_exp_traj, cfg_.sample_traj_dt),
                     latest_state2state_z_debug_.exp_full);

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

        Vec3f state2stateGoalAxis(const Vec3f &axis_start,
                                  const Vec3f &fallback_start,
                                  const Vec3f &goal) {
            Vec3f axis(goal.x() - axis_start.x(), goal.y() - axis_start.y(), 0.0);
            if (axis.norm() < 1.0e-3) {
                axis = Vec3f(goal.x() - fallback_start.x(), goal.y() - fallback_start.y(), 0.0);
            }
            if (axis.norm() < 1.0e-3) {
                axis = goal - fallback_start;
            }
            if (axis.norm() < 1.0e-6) {
                return Vec3f::Zero();
            }
            return axis.normalized();
        }

        double state2stateGoalOvershoot(const Vec3f &point,
                                        const Vec3f &axis_start,
                                        const Vec3f &fallback_start,
                                        const Vec3f &goal) {
            const Vec3f axis = state2stateGoalAxis(axis_start, fallback_start, goal);
            if (axis.norm() < 1.0e-6) {
                return 0.0;
            }
            return (point - goal).dot(axis);
        }

        double state2stateMaxGoalOvershoot(const vec_Vec3f &path,
                                           const Vec3f &axis_start,
                                           const Vec3f &fallback_start,
                                           const Vec3f &goal) {
            double max_over = 0.0;
            for (const auto &point : path) {
                max_over = std::max(max_over,
                                    state2stateGoalOvershoot(point,
                                                            axis_start,
                                                            fallback_start,
                                                            goal));
            }
            return max_over;
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

        auto buildDirectLineCandidate = [&](vec_Vec3f &candidate, RET_CODE &candidate_ret) {
            candidate.clear();
            candidate_ret = FAILED;
            if (!cfg_.state2state_direct_line_frontend_enable) {
                return false;
            }
            const Vec3f delta = goal - temp_start_point;
            const double dist = delta.norm();
            if (dist < 1.0e-4) {
                appendPathPointUnique(goal, candidate);
                candidate_ret = REACH_GOAL;
                return true;
            }
            const double usable_horizon = std::max(0.0, searching_horizon);
            const bool reaches_goal = dist <= usable_horizon + std::max(1.0e-3, cfg_.resolution);
            const Vec3f direct_end = reaches_goal
                                         ? goal
                                         : temp_start_point + delta / dist * usable_horizon;
            if (!lineUsable(temp_start_point, direct_end)) {
                return false;
            }
            appendPathPointUnique(direct_end, candidate);
            candidate_ret = reaches_goal ? REACH_GOAL : REACH_HORIZON;
            return true;
        };

        int flag = ON_INF_MAP |
                   (unknown_as_occupied_for_frontend ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   DONT_USE_INF_NEIGHBOR;

        vec_Vec3f normal_path;
        RET_CODE ret_code = FAILED;
        const bool direct_line_frontend =
                buildDirectLineCandidate(normal_path, ret_code);
        if (!direct_line_frontend) {
            ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point,
                                                          goal,
                                                          flag,
                                                          temp_plannning_horizon,
                                                          normal_path,
                                                          cfg_.frontend_astar_time_out);
        } else if (cfg_.print_log) {
            ros_ptr_->info(" -- [GeneralPlanner] Use direct-line frontend candidate: ret={}.",
                           RET_CODE_STR[ret_code]);
        }

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        const bool distance_field_frontend = cfg_.esdf_traj_en || cfg_.plain_traj_en;
        if (!direct_line_frontend && ret_code == NO_PATH && !distance_field_frontend) {
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
        } else if (!direct_line_frontend && ret_code == NO_PATH && distance_field_frontend) {
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map in distance-field mode; skip prob-map fallback.\n");
        }

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

        auto assembleSelectedPath = [&]() {
            vec_Vec3f assembled;
            appendPathPointUnique(start_pt, assembled);
            for (const auto &point : start_point_escape_path) {
                appendPathPointUnique(point, assembled);
            }
            for (const auto &point : selected_path) {
                appendPathPointUnique(point, assembled);
            }
            if (selected_ret == REACH_GOAL) {
                appendPathPointUnique(goal, assembled);
            }
            return assembled;
        };

        path = assembleSelectedPath();
        if (path.size() < 2) {
            ros_ptr_->warn(
                    " -- [GeneralPlanner] Path search failed with empty segments, force return.");
            return false;
        }

        if (cfg_.state2state_over_goal_guard_enable) {
            const double near_goal_radius = std::max(cfg_.resolution * 3.0,
                                                     cfg_.state2state_near_goal_radius);
            const double near_goal_xy = (temp_start_point.head<2>() - goal.head<2>()).norm();
            const double start_over =
                    state2stateGoalOvershoot(temp_start_point,
                                             local_start_p_,
                                             temp_start_point,
                                             goal);
            const double max_allowed_over =
                    std::max(std::max(0.0, cfg_.state2state_over_goal_tolerance),
                             start_over + std::max(0.0, cfg_.state2state_over_goal_tolerance));
            const double max_path_over =
                    state2stateMaxGoalOvershoot(path,
                                                local_start_p_,
                                                temp_start_point,
                                                goal);
            if (near_goal_xy < near_goal_radius &&
                max_path_over > max_allowed_over + 1.0e-6) {
                vec_Vec3f direct_path;
                RET_CODE direct_ret = FAILED;
                if (buildDirectLineCandidate(direct_path, direct_ret) &&
                    direct_ret == REACH_GOAL) {
                    selected_path = direct_path;
                    selected_ret = direct_ret;
                    path = assembleSelectedPath();
                    ros_ptr_->warn(" -- [GeneralPlanner] Near-goal frontend path overshoots goal by {:.2f}m; clamp to direct goal segment.",
                                   max_path_over);
                } else {
                    ros_ptr_->warn(" -- [GeneralPlanner] Near-goal frontend path overshoots goal by {:.2f}m but direct goal segment is not usable; keep A* path.",
                                   max_path_over);
                }
            }
        }

        if (path.size() >= 2) {
            const double max_segment =
                    std::max(cfg_.resolution,
                             0.8 * std::max(cfg_.resolution, cfg_.corridor_line_max_length));
            if (std::isfinite(max_segment) && max_segment > 1.0e-3) {
                vec_Vec3f dense_path;
                appendPathPointUnique(path.front(), dense_path);
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const Vec3f a = dense_path.back();
                    const Vec3f b = path[i];
                    const double len = (b - a).norm();
                    const int pieces = std::max(1, static_cast<int>(std::ceil(len / max_segment)));
                    for (int k = 1; k <= pieces; ++k) {
                        const double alpha = static_cast<double>(k) / static_cast<double>(pieces);
                        appendPathPointUnique(a + alpha * (b - a), dense_path);
                    }
                }
                path = dense_path;
            }
        }
        return true;
    }


    void GeneralPlanner::getRobotState(rog_map::RobotState &out) {
        robot_state_ = map_manager_->getRobotState();
        out = robot_state_;
    }
}
