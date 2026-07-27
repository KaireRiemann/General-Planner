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


#ifdef USE_ROS1

#ifndef SRC_FSM_ROS1_HPP
#define SRC_FSM_ROS1_HPP

#include "fsm/fsm.h"
#include "ros_interface/ros_adapter_contract.hpp"
#include <map_manager/topology_graph_ros1.hpp>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/SO3Command.h"
#include "std_msgs/String.h"
#include "utils/geometry/quadrotor_flatness.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <utility>


namespace fsm {
    class FsmRos1 : public Fsm {
        ros::NodeHandle nh_;
        ros::Subscriber goal_sub_;
        ros::Subscriber goal_3d_sub_;
        ros::Subscriber task_mode_sub_;
        ros::Subscriber tracking_target_sub_;
        ros::Subscriber tracking_prediction_sub_;
        ros::Subscriber perching_surface_sub_;
        ros::Subscriber swarm_broadcast_traj_sub_;
        ros::Subscriber swarm_state_sub_;
        ros::Subscriber swarm_formation_reference_sub_;
        ros::Subscriber dynamic_obstacle_cloud_sub_;
        ros::Publisher cmd_pub, so3_cmd_pub_, mpc_cmd_pub_, path_pub_;
        ros::Publisher swarm_traj_pub_, swarm_state_pub_;
        ros::Publisher diagnostic_event_pub_;
        ros::Timer execution_timer_, replan_timer_, cmd_timer_, perception_safety_timer_;
        quadrotor_msgs::PositionCommand pid_cmd_;
        rog_map::ROGMapROS::Ptr map_ptr_;
        general_planner::TopologyGraphROS1::Ptr topology_graph_ros1_;
        quadrotor_msgs::PositionCommand latest_cmd;
        nav_msgs::Path path;
        std::vector<ros::Subscriber> swarm_traj_subs_;
        std::map<int, traj_opt::SwarmTrajectory> swarm_traj_buffer_;
        std::map<int, nav_msgs::Odometry> swarm_state_buffer_;
        ros_interface::RosAdapterContract ros_adapter_contract_;
        unsigned int traj_seq_{0};
        int last_cmd_backup_flag_{-1};
        ros::Time last_tracking_prediction_path_time_;

        struct CommandLogEntry {
            quadrotor_msgs::PositionCommand cmd;
            uint64_t replan_id{0};
            unsigned int traj_seq{0};
        };

        vector<CommandLogEntry> cmd_logs_;
        vector<CommandLogEntry> tracking_cmd_logs_;

        void appendCommandLog(const quadrotor_msgs::PositionCommand &cmd) {
            CommandLogEntry entry;
            entry.cmd = cmd;
            entry.replan_id = active_replan_id_;
            entry.traj_seq = traj_seq_;
            if (useTrackingLogStream()) {
                tracking_cmd_logs_.push_back(entry);
            } else {
                cmd_logs_.push_back(entry);
            }
        }

        void publishDiagnosticEvent(const DiagnosticEvent &event) override {
            if (!cfg_.diagnostic_log_en || !diagnostic_event_pub_) {
                return;
            }
            std_msgs::String msg;
            msg.data = diagnosticEventToString(event);
            diagnostic_event_pub_.publish(msg);
        }

        void resetVisualizedPath() override {
            path.poses.clear();
        }

        void publishCurPoseToPath() override {
            path.header.frame_id = "world";
            path.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = robot_state_.p(0);
            pose.pose.position.y = robot_state_.p(1);
            pose.pose.position.z = robot_state_.p(2);
            pose.pose.orientation.x = robot_state_.q.x();
            pose.pose.orientation.y = robot_state_.q.y();
            pose.pose.orientation.z = robot_state_.q.z();
            pose.pose.orientation.w = robot_state_.q.w();
            path.poses.push_back(pose);
            path_pub_.publish(path);
        }

        void publishPolyTraj() override {
            quadrotor_msgs::PolynomialTrajectory cmd_traj;
            ++traj_seq_;
            getCommittedTrajectory(cmd_traj);
            mpc_cmd_pub_.publish(cmd_traj);
            if (cfg_.swarm_enable && cfg_.swarm_broadcast_enable) {
                swarm_traj_pub_.publish(cmd_traj);
            }
            double duration = 0.0;
            for (const auto &piece_duration: cmd_traj.time_pos) {
                duration += piece_duration;
            }
            recordDiagnosticEvent("INFO",
                                  "trajectory_published",
                                  fmt::format("piece_num_pos={};duration={:.3f};debug_info={}",
                                              cmd_traj.piece_num_pos,
                                              duration,
                                              cmd_traj.debug_info),
                                  -1,
                                  static_cast<int>(traj_seq_));
        }

        void getOneHeartBeatMsg(quadrotor_msgs::PolynomialTrajectory &heartbeat, bool &traj_finish) {
            heartbeat.type = quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
            heartbeat.header.stamp = ros::Time::now();
            heartbeat.header.frame_id = "world";
            double swt;
            planner_ptr_->getOneHeartbeatTime(swt, traj_finish);
            heartbeat.start_WT_pos = ros::Time(swt);
        }

        void getCommittedTrajectory(quadrotor_msgs::PolynomialTrajectory &cmd_traj) {
            cmd_traj.header.stamp = ros::Time::now();
            cmd_traj.header.frame_id = "world";
            cmd_traj.trajectory_id = traj_seq_;
            cmd_traj.debug_info = makeSwarmDebugInfo();
            cmd_traj.type = quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ |
                            quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
            planner_ptr_->lockCommittedTraj();
            const Trajectory pos_traj = planner_ptr_->getCommittedPositionTrajectory();
            const Trajectory yaw_traj = planner_ptr_->getCommittedYawTrajectory();
            planner_ptr_->unlockCommittedTraj();

            cmd_traj.start_WT_pos = ros::Time(pos_traj.start_WT);
            cmd_traj.debug_info += makeCommittedZDebugInfo(pos_traj);

            cmd_traj.piece_num_pos = pos_traj.getPieceNum();
            cmd_traj.order_pos = 7;
            cmd_traj.time_pos.resize(pos_traj.getPieceNum());
            cmd_traj.coef_pos_x.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.coef_pos_y.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.coef_pos_z.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.start_WT_pos = ros::Time(pos_traj.start_WT);

            if (!yaw_traj.empty()) {
                cmd_traj.type = cmd_traj.type |
                                quadrotor_msgs::PolynomialTrajectory::YAW_TRAJ;
                cmd_traj.piece_num_yaw = yaw_traj.getPieceNum();
                cmd_traj.order_yaw = 7;
                double col_size = cmd_traj.order_yaw + 1;
                cmd_traj.coef_yaw.resize(cmd_traj.piece_num_yaw * col_size);
                cmd_traj.time_yaw.resize(cmd_traj.piece_num_yaw);
                for (int i = 0; i < cmd_traj.piece_num_yaw; i++) {
                    Eigen::VectorXd yaw_coef = yaw_traj[i].getCoeffMat().row(0);
                    Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_yaw[col_size * i], col_size) = yaw_coef;
                    cmd_traj.time_yaw[i] = yaw_traj[i].getDuration();
                }
                cmd_traj.start_WT_yaw = ros::Time(yaw_traj.start_WT);
            }

            for (int i = 0; i < cmd_traj.piece_num_pos; i++) {
                Eigen::Matrix<double, 3, 8> coef = pos_traj[i].getCoeffMat();
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_x[8 * i], 8) = coef.row(0);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_y[8 * i], 8) = coef.row(1);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_z[8 * i], 8) = coef.row(2);
                cmd_traj.time_pos[i] = pos_traj[i].getDuration();
            }
        }

        std::string makeCommittedZDebugInfo(const Trajectory &pos_traj) const {
            if (pos_traj.empty()) {
                return ";committed_z_valid=0";
            }
            const double duration = pos_traj.getTotalDuration();
            if (!std::isfinite(duration) || duration < 0.0) {
                return ";committed_z_valid=0";
            }

            bool valid = false;
            double start_z = 0.0;
            double end_z = 0.0;
            double min_z = 0.0;
            double max_z = 0.0;
            const double sample_dt = 0.05;
            auto addSample = [&](const double t) {
                const Vec3f pos = pos_traj.getPos(std::clamp(t, 0.0, duration));
                if (!pos.allFinite() || !std::isfinite(pos.z())) {
                    return;
                }
                if (!valid) {
                    valid = true;
                    start_z = pos.z();
                    min_z = pos.z();
                    max_z = pos.z();
                }
                end_z = pos.z();
                min_z = std::min(min_z, pos.z());
                max_z = std::max(max_z, pos.z());
            };
            addSample(0.0);
            for (double t = sample_dt; t < duration; t += sample_dt) {
                addSample(t);
            }
            addSample(duration);

            std::ostringstream oss;
            oss << ";committed_z_valid=" << static_cast<int>(valid)
                << ";committed_z_start=" << start_z
                << ";committed_z_end=" << end_z
                << ";committed_z_min=" << min_z
                << ";committed_z_max=" << max_z;
            return oss.str();
        }

        std::string makeSwarmDebugInfo() const {
            const double corridor_time = planner_ptr_->getLatestCorridorTime();
            const std::string state2state_z_debug =
                    state2stateMode() ? planner_ptr_->getLatestState2StateZDebugInfo() : "";
            const char *task_phase = "state_to_state";
            if (perchingMode()) {
                task_phase = "perching";
            } else if (explorationMode()) {
                task_phase = "exploration";
            } else if (se3AggressiveMode()) {
                task_phase = "se3_aggressive";
            } else if (trackingPerchingMode()) {
                task_phase = trackingPerchingPerchingActive()
                                 ? "tracking_perching_perching"
                                 : "tracking_perching_tracking";
            } else if (trackingPerchingPerchingActive()) {
                task_phase = "tracking_perching_perching";
            } else if (trackingMode()) {
                task_phase = "tracking";
            }
            std::ostringstream oss;
            oss << "drone_id=" << cfg_.swarm_drone_id
                << ";des_clearance=" << cfg_.swarm_des_clearance
                << ";task_mode=" << cfg_.task_mode_str
                << ";task_phase=" << task_phase
                << ";replan_id=" << active_replan_id_
                << ";traj_seq=" << traj_seq_
                << ";ellipsoid_optimizer=" << planner_ptr_->getEllipsoidOptimizerName()
                << ";corridor_time_ms=" << (corridor_time >= 0.0 ? corridor_time * 1000.0 : -1.0)
                << ";mvie_lbfgs_iterations=" << planner_ptr_->getLatestMvieLbfgsIterations()
                << ";exp_frontend_time_ms=" << planner_ptr_->getLatestExpFrontendTime() * 1000.0
                << ";exp_opt_time_ms=" << planner_ptr_->getLatestExpOptimizationTime() * 1000.0
                << ";backup_frontend_time_ms=" << planner_ptr_->getLatestBackupFrontendTime() * 1000.0
                << ";backup_opt_time_ms=" << planner_ptr_->getLatestBackupOptimizationTime() * 1000.0
                << ";total_replan_time_ms=" << planner_ptr_->getLatestTotalReplanTime() * 1000.0
                << ";trajectory_optimization_success=1"
                << ";optimization_time_ms=" << planner_ptr_->getLatestOptimizationTime() * 1000.0
                << state2state_z_debug;
            return oss.str();
        }

        static bool parseKeyValueDebugInfo(const std::string &debug_info,
                                           const std::string &key,
                                           std::string &value) {
            const std::string needle = key + "=";
            const size_t begin = debug_info.find(needle);
            if (begin == std::string::npos) {
                return false;
            }
            const size_t value_begin = begin + needle.size();
            const size_t value_end = debug_info.find(';', value_begin);
            value = debug_info.substr(value_begin,
                                      value_end == std::string::npos ? std::string::npos : value_end - value_begin);
            return !value.empty();
        }

        static bool parseSwarmDebugInfo(const std::string &debug_info,
                                        int &drone_id,
                                        double &des_clearance) {
            std::string value;
            bool parsed = false;
            if (parseKeyValueDebugInfo(debug_info, "drone_id", value)) {
                try {
                    drone_id = std::stoi(value);
                    parsed = true;
                } catch (const std::exception &) {
                    return false;
                }
            }
            if (parseKeyValueDebugInfo(debug_info, "des_clearance", value)) {
                try {
                    des_clearance = std::stod(value);
                } catch (const std::exception &) {
                    return false;
                }
            }
            return parsed;
        }

        static bool polynomialMsgToTrajectory(const quadrotor_msgs::PolynomialTrajectory &msg,
                                              Trajectory &traj) {
            if ((msg.type & quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ) == 0 ||
                msg.piece_num_pos <= 0 || msg.order_pos < 1) {
                return false;
            }
            const int coeff_num = msg.order_pos + 1;
            if (static_cast<int>(msg.time_pos.size()) < msg.piece_num_pos ||
                static_cast<int>(msg.coef_pos_x.size()) < msg.piece_num_pos * coeff_num ||
                static_cast<int>(msg.coef_pos_y.size()) < msg.piece_num_pos * coeff_num ||
                static_cast<int>(msg.coef_pos_z.size()) < msg.piece_num_pos * coeff_num) {
                return false;
            }

            traj.clear();
            traj.reserve(msg.piece_num_pos);
            for (int i = 0; i < msg.piece_num_pos; ++i) {
                const double duration = msg.time_pos[i];
                if (duration <= 1.0e-6) {
                    return false;
                }
                Eigen::MatrixXd coeff(3, coeff_num);
                for (int j = 0; j < coeff_num; ++j) {
                    const int offset = i * coeff_num + j;
                    coeff(0, j) = msg.coef_pos_x[offset];
                    coeff(1, j) = msg.coef_pos_y[offset];
                    coeff(2, j) = msg.coef_pos_z[offset];
                }
                traj.emplace_back(duration, coeff);
            }
            traj.start_WT = msg.start_WT_pos.toSec();
            return traj.start_WT > 0.0 && !traj.empty();
        }

        void swarmTrajCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg,
                               int drone_id) {
            double des_clearance = cfg_.swarm_des_clearance;
            if (drone_id < 0 && !parseSwarmDebugInfo(msg->debug_info, drone_id, des_clearance)) {
                return;
            }
            if (!cfg_.swarm_enable || drone_id == cfg_.swarm_drone_id || planner_ptr_ == nullptr) {
                return;
            }

            Trajectory pos_traj;
            if (!polynomialMsgToTrajectory(*msg, pos_traj)) {
                return;
            }
            const auto existing = swarm_traj_buffer_.find(drone_id);
            if (existing != swarm_traj_buffer_.end() &&
                pos_traj.start_WT <= existing->second.start_wall_time + 1.0e-6) {
                ROS_WARN_STREAM_THROTTLE(1.0, " -- [Fsm] Ignore old swarm traj from drone "
                                                  << drone_id << ", start_WT=" << pos_traj.start_WT
                                                  << ", buffered=" << existing->second.start_wall_time);
                return;
            }
            const double now_wt = ros::Time::now().toSec();
            const double time_diff = now_wt - pos_traj.start_WT;
            if (std::abs(time_diff) > 10.0) {
                ROS_ERROR_STREAM(" -- [Fsm] Ignore swarm traj from drone " << drone_id
                                 << " because swarm time is not synchronized, diff=" << time_diff);
                return;
            }
            if (std::abs(time_diff) > 0.25) {
                ROS_WARN_STREAM_THROTTLE(1.0, " -- [Fsm] Swarm traj time diff from drone "
                                                  << drone_id << " is " << time_diff << "s");
            }

            traj_opt::SwarmTrajectory swarm_traj;
            swarm_traj.drone_id = drone_id;
            swarm_traj.traj_id = msg->trajectory_id;
            swarm_traj.start_wall_time = pos_traj.start_WT;
            swarm_traj.duration = pos_traj.getTotalDuration();
            swarm_traj.clearance = des_clearance;
            swarm_traj.traj = pos_traj;
            if (!swarm_traj.valid()) {
                return;
            }

            swarm_traj_buffer_[drone_id] = swarm_traj;
            traj_opt::SwarmTrajectories snapshot;
            snapshot.reserve(swarm_traj_buffer_.size());
            for (const auto &kv : swarm_traj_buffer_) {
                snapshot.emplace_back(kv.second);
            }
            planner_ptr_->setSwarmTrajectories(snapshot);
            ROS_INFO_STREAM_THROTTLE(1.0, " -- [Fsm] Swarm traj update from drone "
                                              << drone_id << ", traj_id=" << swarm_traj.traj_id
                                              << ", duration=" << swarm_traj.duration
                                              << ", buffer=" << snapshot.size());
        }

        void swarmFormationReferenceCallback(const nav_msgs::PathConstPtr &msg) {
            if (!cfg_.swarm_enable || !cfg_.swarm_formation_reference_enable || planner_ptr_ == nullptr) {
                return;
            }
            if (msg->poses.size() < 2) {
                ROS_WARN_THROTTLE(1.0, " -- [Fsm] Invalid formation reference: need at least 2 poses.");
                return;
            }
            const auto &start_msg = msg->poses.front().pose.position;
            const auto &end_msg = msg->poses.back().pose.position;
            Vec3f start(start_msg.x, start_msg.y, start_msg.z);
            Vec3f end(end_msg.x, end_msg.y, end_msg.z);
            if (!start.allFinite() || !end.allFinite() ||
                (end - start).squaredNorm() < 1.0e-8) {
                ROS_WARN_THROTTLE(1.0, " -- [Fsm] Invalid formation reference geometry.");
                return;
            }
            planner_ptr_->setSwarmFormationReference(start, end);
            ROS_INFO_STREAM_THROTTLE(1.0, " -- [Fsm] Formation reference update: start=["
                                              << start.transpose() << "], end=[" << end.transpose() << "]");
        }

        void swarmStateCallback(const nav_msgs::OdometryConstPtr &msg) {
            if (!cfg_.swarm_enable) {
                return;
            }
            int drone_id = -1;
            const std::string &frame = msg->child_frame_id;
            const std::string prefix = "drone_";
            const size_t pos = frame.find(prefix);
            if (pos != std::string::npos) {
                try {
                    drone_id = std::stoi(frame.substr(pos + prefix.size()));
                } catch (const std::exception &) {
                    drone_id = -1;
                }
            }
            if (drone_id < 0 || drone_id == cfg_.swarm_drone_id) {
                return;
            }
            swarm_state_buffer_[drone_id] = *msg;
        }

        void publishSwarmState() {
            if (!cfg_.swarm_enable || !cfg_.swarm_broadcast_enable || !robot_state_.rcv) {
                return;
            }
            nav_msgs::Odometry state;
            state.header.stamp = ros::Time::now();
            state.header.frame_id = "world";
            state.child_frame_id = "drone_" + std::to_string(cfg_.swarm_drone_id);
            state.pose.pose.position.x = robot_state_.p.x();
            state.pose.pose.position.y = robot_state_.p.y();
            state.pose.pose.position.z = robot_state_.p.z();
            state.pose.pose.orientation.x = robot_state_.q.x();
            state.pose.pose.orientation.y = robot_state_.q.y();
            state.pose.pose.orientation.z = robot_state_.q.z();
            state.pose.pose.orientation.w = robot_state_.q.w();
            state.twist.twist.linear.x = robot_state_.v.x();
            state.twist.twist.linear.y = robot_state_.v.y();
            state.twist.twist.linear.z = robot_state_.v.z();
            swarm_state_pub_.publish(state);
        }

        bool getOneSO3Command(const quadrotor_msgs::PositionCommand &pos_cmd,
                              quadrotor_msgs::SO3Command &so3_cmd) const {
            flatness::FlatnessMap flatmap;
            flatmap.reset(cfg_.flatness_mass,
                          cfg_.flatness_grav,
                          cfg_.flatness_dh,
                          cfg_.flatness_dv,
                          cfg_.flatness_cp,
                          cfg_.flatness_v_eps);

            const Eigen::Vector3d vel(pos_cmd.velocity.x, pos_cmd.velocity.y, pos_cmd.velocity.z);
            const Eigen::Vector3d acc(pos_cmd.acceleration.x, pos_cmd.acceleration.y, pos_cmd.acceleration.z);
            const Eigen::Vector3d jerk(pos_cmd.jerk.x, pos_cmd.jerk.y, pos_cmd.jerk.z);

            double thrust = 0.0;
            Eigen::Vector4d quat_vec = Eigen::Vector4d::Zero();
            Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
            flatmap.forward(vel,
                            acc,
                            jerk,
                            pos_cmd.yaw,
                            pos_cmd.yaw_dot,
                            thrust,
                            quat_vec,
                            body_rate);
            if (!std::isfinite(thrust) || !quat_vec.allFinite()) {
                return false;
            }

            Eigen::Quaterniond quat(quat_vec(0), quat_vec(1), quat_vec(2), quat_vec(3));
            if (!std::isfinite(quat.norm()) || quat.norm() < 1.0e-9) {
                return false;
            }
            quat.normalize();

            const Eigen::Vector3d force = quat * Eigen::Vector3d::UnitZ() * thrust;
            if (!force.allFinite()) {
                return false;
            }

            so3_cmd.header = pos_cmd.header;
            so3_cmd.force.x = force.x();
            so3_cmd.force.y = force.y();
            so3_cmd.force.z = force.z();
            so3_cmd.orientation.x = quat.x();
            so3_cmd.orientation.y = quat.y();
            so3_cmd.orientation.z = quat.z();
            so3_cmd.orientation.w = quat.w();
            for (int i = 0; i < 3; ++i) {
                so3_cmd.kR[i] = cfg_.so3_kR[static_cast<std::size_t>(i)];
                so3_cmd.kOm[i] = cfg_.so3_kOm[static_cast<std::size_t>(i)];
            }
            so3_cmd.aux.current_yaw = pos_cmd.yaw;
            so3_cmd.aux.kf_correction = 0.0;
            so3_cmd.aux.angle_corrections[0] = 0.0;
            so3_cmd.aux.angle_corrections[1] = 0.0;
            so3_cmd.aux.enable_motors = true;
            so3_cmd.aux.use_external_yaw = false;
            return true;
        }

        void getOnePositionCommand(quadrotor_msgs::PositionCommand &pos_cmd, bool &traj_finish) {
            pos_cmd.trajectory_flag = 0;
            StatePVAJ pvaj;
            double yaw, yaw_dot;
            bool on_backup_traj;
            planner_ptr_->getOneCommandFromTraj(pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            pos_cmd.header.stamp = ros::Time::now();
            pos_cmd.header.frame_id = "world";
            pos_cmd.trajectory_id = traj_seq_;
            pos_cmd.position.x = pvaj(0, 0);
            pos_cmd.position.y = pvaj(1, 0);
            pos_cmd.position.z = pvaj(2, 0);
            pos_cmd.velocity.x = pvaj(0, 1);
            pos_cmd.velocity.y = pvaj(1, 1);
            pos_cmd.velocity.z = pvaj(2, 1);
            pos_cmd.acceleration.x = pvaj(0, 2);
            pos_cmd.acceleration.y = pvaj(1, 2);
            pos_cmd.acceleration.z = pvaj(2, 2);
            pos_cmd.jerk.x = pvaj(0, 3);
            pos_cmd.jerk.y = pvaj(1, 3);
            pos_cmd.jerk.z = pvaj(2, 3);
            pos_cmd.yaw = yaw;
            pos_cmd.yaw_dot = yaw_dot;
            pos_cmd.trajectory_flag = on_backup_traj ? 2 : 1;
            pos_cmd.vel_norm = pvaj.col(1).norm();
            pos_cmd.acc_norm = pvaj.col(2).norm();
            Vec3f rpy, omg;
            double aT;
            geometry_utils::convertFlatOutputToAttAndOmg(pvaj.col(0), pvaj.col(1), pvaj.col(2), pvaj.col(3), yaw,
                                                         yaw_dot, rpy, omg, aT);
            if (!rpy.allFinite() || !omg.allFinite() || !std::isfinite(aT)) {
                recordDiagnosticEvent("WARN",
                                      "cmd_flat_output_invalid",
                                      fmt::format("trajectory_id={}", traj_seq_),
                                      -1,
                                      static_cast<int>(traj_seq_),
                                      on_backup_traj);
                rpy.setZero();
                omg.setZero();
                aT = 0.0;
            }
            pos_cmd.attitude.x = rpy(0);
            pos_cmd.attitude.y = rpy(1);
            pos_cmd.attitude.z = rpy(2);
            pos_cmd.angular_velocity.x = omg(0);
            pos_cmd.angular_velocity.y = omg(1);
            pos_cmd.angular_velocity.z = omg(2);
            pos_cmd.thrust.z = aT;
            latest_cmd = pos_cmd;
            appendCommandLog(latest_cmd);
            const int backup_flag = on_backup_traj ? 1 : 0;
            if (last_cmd_backup_flag_ != backup_flag) {
                last_cmd_backup_flag_ = backup_flag;
                recordDiagnosticEvent(on_backup_traj ? "WARN" : "INFO",
                                      on_backup_traj ? "cmd_enter_backup_traj" : "cmd_use_exp_traj",
                                      fmt::format("traj_finish={};trajectory_id={}",
                                                  static_cast<int>(traj_finish),
                                                  pos_cmd.trajectory_id),
                                      -1,
                                      static_cast<int>(traj_seq_),
                                      on_backup_traj);
            }
        }

    public:
        FsmRos1() = default;

        ~FsmRos1() {
            stop = true;
            try {
                execution_timer_.stop();
                replan_timer_.stop();
                cmd_timer_.stop();
            } catch (const std::exception &e) {
                fmt::print(stderr, " -- [Fsm] Failed to stop ROS timers: {}\n", e.what());
            } catch (...) {
                fmt::print(stderr, " -- [Fsm] Failed to stop ROS timers: unknown exception\n");
            }
            try {
                saveReplanLogToFile("general_latest_log");
            } catch (const std::exception &e) {
                fmt::print(stderr, " -- [Fsm] Failed to save final replan log: {}\n", e.what());
            } catch (...) {
                fmt::print(stderr, " -- [Fsm] Failed to save final replan log: unknown exception\n");
            }
        };

        typedef std::shared_ptr<FsmRos1> Ptr;

        void saveReplanLogToFile(const string &name = "") {
            const auto makeBaseName = [&](const bool tracking_stream) -> std::string {
                if (name.empty()) {
                    return BinaryFileHandler<int>::getCurrentTimeStr();
                }
                if (!tracking_stream) {
                    return name;
                }
                if (name == "general_latest_log") {
                    return "tracking_latest_log";
                }
                return "tracking_" + name;
            };

            const auto saveCommandCsv =
                    [&](const std::string &csv_path,
                        const vector<CommandLogEntry> &cmd_logs) {
                if (cmd_logs.empty()) {
                    return;
                }
                if (!ensureLogParentDirectory(csv_path)) {
                    fmt::print(stderr, " -- [Fsm] Failed to create cmd log directory for {}\n", csv_path);
                    return;
                }
                std::ofstream csv_writer(csv_path, std::ios::out | std::ios::trunc);
                if (!csv_writer.is_open()) {
                    fmt::print(stderr, " -- [Fsm] Failed to open cmd log file {}\n", csv_path);
                    return;
                }
                csv_writer
                        << "time,replan_id,traj_seq,posi_x,posi_y,posi_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,jerk_x,jerk_y,jerk_z,yaw,yaw_rate,backup"
                        << std::endl;
                csv_writer << std::fixed << std::setprecision(15);
                for (const auto &entry: cmd_logs) {
                    const auto &cmd = entry.cmd;
                    csv_writer << cmd.header.stamp.toSec() - system_start_time_ << ","
                               << entry.replan_id << "," << entry.traj_seq << ","
                               << cmd.position.x << "," << cmd.position.y << ","
                               << cmd.position.z << ","
                               << cmd.velocity.x << "," << cmd.velocity.y << "," << cmd.velocity.z << ","
                               << cmd.acceleration.x << "," << cmd.acceleration.y << "," << cmd.acceleration.z << ","
                               << cmd.jerk.x << "," << cmd.jerk.y << "," << cmd.jerk.z << ","
                               << cmd.yaw << "," << cmd.yaw_dot << "," << static_cast<int>(cmd.trajectory_flag)
                               << std::endl;
                }
            };

            const auto saveStream =
                    [&](const std::string &stream_name,
                        const bool tracking_stream,
                        const vector<LogOneReplan> &replan_logs,
                        const vector<CommandLogEntry> &cmd_logs,
                        const bool has_diagnostic_events) {
                if (replan_logs.empty() && cmd_logs.empty() && !has_diagnostic_events) {
                    return;
                }

                double total_length{0.0};
                int total_replan_num{0};
                double average_compt_t{0.0};
                Vec3f cur_p{0, 0, 0};
                for (auto rp: replan_logs) {
                    if (rp.getRetCode() > 0) {
                        if (cur_p.norm() < 1e-6) {
                            cur_p = rp.getRobotP();
                        } else {
                            total_length += (rp.getRobotP() - cur_p).norm();
                            cur_p = rp.getRobotP();
                        }
                        total_replan_num++;
                        average_compt_t += rp.getTotalCompT();
                    }
                }
                const double average_computation_ms =
                        average_compt_t /
                        static_cast<double>(total_replan_num == 0 ? 1 : total_replan_num) *
                        1000.0;
                if (!tracking_stream && state2stateMode() &&
                    cfg_.backend_type == general_planner::architecture::BackendType::CORRIDOR) {
                    const auto timing = planner_ptr_->getCumulativeExpTimingReport();
                    const double total_optimization_ms = timing.optimization_seconds * 1000.0;
                    const double dense_share =
                            timing.optimization_seconds > 0.0
                            ? timing.dense_integral_seconds / timing.optimization_seconds * 100.0
                            : 0.0;
                    const double control_point_share =
                            timing.optimization_seconds > 0.0
                            ? timing.control_point_seconds / timing.optimization_seconds * 100.0
                            : 0.0;
                    fmt::print("[{}] Total replan num: {}, total length: {}, average computation time: {} ms, total optimization time: {} ms, dense sampling share: {}%, control point share: {}%\n",
                               stream_name,
                               total_replan_num,
                               total_length,
                               average_computation_ms,
                               total_optimization_ms,
                               dense_share,
                               control_point_share);
                } else {
                    fmt::print("[{}] Total replan num: {}, total length: {}, average computation time: {} ms\n",
                               stream_name,
                               total_replan_num,
                               total_length,
                               average_computation_ms);
                }

                const std::string base_name = makeBaseName(tracking_stream);
                const std::string replan_dir =
                        tracking_stream ? "tracking_replan_logs/" : "replan_logs/";
                const std::string cmd_dir =
                        tracking_stream ? "tracking_cmd_logs/" : "cmd_logs/";
                const std::string save_path = LOG_FILE_DIR(replan_dir + base_name + ".bin");
                const std::string csv_path = LOG_FILE_DIR(cmd_dir + base_name + ".csv");

                if (!replan_logs.empty()) {
                    if (ensureLogParentDirectory(save_path)) {
                        BinaryFileHandler<vector<LogOneReplan>>::save(save_path, replan_logs);
                    } else {
                        fmt::print(stderr, " -- [Fsm] Failed to create replan log directory for {}\n", save_path);
                    }
                }

                if (tracking_stream) {
                    saveTrackingDiagnosticLogToFile(name.empty() ? "" : base_name + "_events");
                } else {
                    saveDiagnosticLogToFile(name.empty() ? "" : base_name + "_events");
                }
                saveCommandCsv(csv_path, cmd_logs);
            };

            const auto general_replan_logs = snapshotReplanLogs();
            const auto tracking_replan_logs = snapshotTrackingReplanLogs();
            const bool has_general_diagnostics = !snapshotDiagnosticEvents().empty();
            const bool has_tracking_diagnostics = !snapshotTrackingDiagnosticEvents().empty();

            saveStream("general",
                       false,
                       general_replan_logs,
                       cmd_logs_,
                       has_general_diagnostics);
            saveStream("tracking",
                       true,
                       tracking_replan_logs,
                       tracking_cmd_logs_,
                       has_tracking_diagnostics);
        }

        bool getPoseFromTraj(general_utils::Pose &pose) {
            if (machine_state_ != FOLLOW_TRAJ &&
                machine_state_ != STATIC_TRACKING) {
                cout << YELLOW << "[Fsm] Not in trajectory execution state, can't get pose from traj." << RESET << endl;
                return false;
            }
            getOnePositionCommand(pid_cmd_, traj_finish_);
            if (traj_finish_) {
                const auto finish_result = handleExecutedTrajectoryFinished(
                        "getPoseFromTraj",
                        pid_cmd_.trajectory_id,
                        static_cast<int>(traj_seq_),
                        pid_cmd_.trajectory_flag == 2,
                        false,
                        false);
                if (finish_result.tracking_unfinished) {
                    traj_finish_ = false;
                }
            }
            pose.first = Vec3f{pid_cmd_.position.x, pid_cmd_.position.y, pid_cmd_.position.z};
            pose.second = eulerToQuaternion(pid_cmd_.attitude.x, pid_cmd_.attitude.y, pid_cmd_.attitude.z);


            /// for checking the trajectory continuty
            static double max_delta_v{0.0};
            static double last_v = pid_cmd_.vel_norm;
            double delta_v = std::abs(pid_cmd_.vel_norm - last_v);
            last_v = pid_cmd_.vel_norm;
            if (delta_v > max_delta_v) {
                max_delta_v = delta_v;
            }
            fmt::print(" -- [Fsm] Cur vel: {}, delta_v: {}, max_delta_v: {}\n", pid_cmd_.vel_norm, delta_v,
                       max_delta_v);
            return true;
        }

        void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            general_utils::Vec3f goal_p = Vec3f{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
            general_utils::Quatf goal_q = general_utils::Quatf{msg->pose.orientation.w, msg->pose.orientation.x,
                                                           msg->pose.orientation.y, msg->pose.orientation.z};
            setGoalPosiAndYaw(goal_p, goal_q);
        }

        void goal3DCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            general_utils::Vec3f goal_p = Vec3f{msg->pose.position.x, msg->pose.position.y,
                                                msg->pose.position.z};
            general_utils::Quatf goal_q = general_utils::Quatf{msg->pose.orientation.w,
                                                                msg->pose.orientation.x,
                                                                msg->pose.orientation.y,
                                                                msg->pose.orientation.z};
            setGoalPosiAndYaw(goal_p, goal_q, GoalHeightMode::MESSAGE_HEIGHT);
        }

        bool subscribe3DGoal(const uint32_t queue_size,
                             const bool fixed_height_goal_active) {
            if (!cfg_.click_goal_3d_en) {
                return false;
            }
            if (cfg_.click_goal_3d_topic.empty()) {
                ROS_WARN(" -- [Fsm] 3D click goal is enabled but its topic is empty; skip it.");
                return false;
            }
            if (fixed_height_goal_active &&
                nh_.resolveName(cfg_.click_goal_3d_topic) ==
                nh_.resolveName(cfg_.click_goal_topic)) {
                ROS_WARN_STREAM(" -- [Fsm] 3D click goal topic resolves to the 2D goal topic "
                                << nh_.resolveName(cfg_.click_goal_topic)
                                << "; skip the ambiguous 3D subscriber.");
                return false;
            }
            goal_3d_sub_ = nh_.subscribe(cfg_.click_goal_3d_topic, queue_size,
                                         &FsmRos1::goal3DCallback, this);
            cout << YELLOW << " -- [Fsm] 3D CLICK GOAL ENABLED: "
                 << nh_.resolveName(cfg_.click_goal_3d_topic)
                 << " (message z is preserved)." << RESET << endl;
            return true;
        }

        void dynamicObstacleCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
            if (!cfg_.dynamic_obstacle_layer_enable || planner_ptr_ == nullptr) {
                return;
            }

            rog_map::RobotState robot_state;
            planner_ptr_->getRobotState(robot_state);
            const double now = ros_ptr_ != nullptr ? ros_ptr_->getSimTime() : ros::Time::now().toSec();
            if (!robot_state.rcv ||
                now - robot_state.rcv_time > std::max(0.0, cfg_.dynamic_obstacle_layer_odom_timeout)) {
                ROS_WARN_THROTTLE(1.0,
                                  " -- [Fsm] Dynamic obstacle cloud skipped: odom not ready or stale.");
                return;
            }

            rog_map::PointCloud cloud;
            pcl::fromROSMsg(*msg, cloud);
            planner_ptr_->updateDynamicObstacleCloud(cloud, robot_state.p, now);
        }

        static double yawFromMsgQuat(const geometry_msgs::Quaternion &q_msg) {
            const double w = q_msg.w;
            const double x = q_msg.x;
            const double y = q_msg.y;
            const double z = q_msg.z;
            return std::atan2(2.0 * (w * z + x * y),
                              1.0 - 2.0 * (y * y + z * z));
        }

        bool trackingPredictionStateValid(const Vec3f &p, const Vec3f &v) const {
            if (cfg_.tracking_prediction_vmax > 0.0 &&
                v.norm() > cfg_.tracking_prediction_vmax) {
                return false;
            }
            if (map_ptr_ == nullptr) {
                return true;
            }
            if (!map_ptr_->insideLocalMap(p)) {
                return true;
            }
            const auto inf_grid_type = map_ptr_->getInfGridType(p);
            return inf_grid_type != general_utils::GridType::OCCUPIED &&
                   inf_grid_type != general_utils::GridType::OUT_OF_MAP;
        }

        void buildConstantVelocityTrackingPrediction(const Vec3f &p,
                                                     const Vec3f &v,
                                                     const double pose_yaw,
                                                     traj_opt::DynamicTargetStates &prediction) const {
            const Vec3f a = Vec3f::Zero();
            const double base_yaw = v.head<2>().norm() > 1.0e-3 ? std::atan2(v.y(), v.x()) : pose_yaw;
            const double dt = std::max(0.05, cfg_.tracking_prediction_dt);
            const double horizon = std::max(dt, cfg_.tracking_prediction_horizon);
            const int sample_num = std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1);

            prediction.clear();
            prediction.reserve(sample_num);
            for (int i = 0; i < sample_num; ++i) {
                const double t = static_cast<double>(i) * dt;
                traj_opt::DynamicTargetState target;
                target.t = t;
                target.position = p + v * t + 0.5 * a * t * t;
                target.velocity = v + a * t;
                target.acceleration = a;
                target.yaw = base_yaw;
                target.yaw_rate = 0.0;
                prediction.emplace_back(target);
            }
        }

        bool buildKinodynamicTrackingPrediction(const Vec3f &p,
                                                const Vec3f &v,
                                                const double pose_yaw,
                                                traj_opt::DynamicTargetStates &prediction) const {
            const double dt = std::max(0.05, cfg_.tracking_prediction_dt);
            const double horizon = std::max(dt, cfg_.tracking_prediction_horizon);
            const double acc = std::max(0.0, cfg_.tracking_prediction_accel);
            if (acc <= 1.0e-6) {
                return false;
            }

            struct PredictNode {
                Vec3f p{Vec3f::Zero()};
                Vec3f v{Vec3f::Zero()};
                Vec3f a{Vec3f::Zero()};
                double t{0.0};
                double score{0.0};
                int parent{-1};
            };

            std::vector<PredictNode> nodes;
            nodes.reserve(512);
            nodes.push_back(PredictNode{p, v, Vec3f::Zero(), 0.0, 0.0, -1});

            const Vec3f nominal_end = p + v * horizon;
            auto heuristic = [&](const PredictNode &node) {
                return 0.001 * (node.p - nominal_end).norm();
            };
            using QueueEntry = std::pair<double, int>;
            std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open_set;

            int cur = 0;
            const double dt2_2 = 0.5 * dt * dt;
            const double max_wall_time = std::max(0.001, cfg_.tracking_prediction_max_time);
            const ros::WallTime start_wall = ros::WallTime::now();
            const int max_nodes = 1 << 16;
            const std::array<double, 3> acc_samples{-acc, 0.0, acc};

            while (nodes[static_cast<std::size_t>(cur)].t + 0.5 * dt < horizon) {
                const PredictNode &cur_node = nodes[static_cast<std::size_t>(cur)];
                for (const double ax : acc_samples) {
                    for (const double ay : acc_samples) {
                        Vec3f input(ax, ay, 0.0);
                        const Vec3f next_p = cur_node.p + cur_node.v * dt + input * dt2_2;
                        const Vec3f next_v = cur_node.v + input * dt;
                        if (!trackingPredictionStateValid(next_p, next_v)) {
                            continue;
                        }
                        if (static_cast<int>(nodes.size()) >= max_nodes) {
                            return false;
                        }
                        if ((ros::WallTime::now() - start_wall).toSec() > max_wall_time) {
                            return false;
                        }
                        PredictNode next;
                        next.p = next_p;
                        next.v = next_v;
                        next.a = input;
                        next.t = cur_node.t + dt;
                        next.score = cur_node.score + cfg_.tracking_prediction_rho_accel * input.norm();
                        next.parent = cur;
                        nodes.emplace_back(next);
                        const int next_id = static_cast<int>(nodes.size()) - 1;
                        open_set.emplace(next.score + heuristic(next), next_id);
                    }
                }
                if (open_set.empty()) {
                    return false;
                }
                cur = open_set.top().second;
                open_set.pop();
            }

            std::vector<int> ids;
            for (int id = cur; id >= 0; id = nodes[static_cast<std::size_t>(id)].parent) {
                ids.emplace_back(id);
            }
            std::reverse(ids.begin(), ids.end());
            if (ids.size() < 2) {
                return false;
            }

            prediction.clear();
            prediction.reserve(ids.size());
            double last_yaw = pose_yaw;
            for (const int id : ids) {
                const PredictNode &node = nodes[static_cast<std::size_t>(id)];
                traj_opt::DynamicTargetState target;
                target.t = node.t;
                target.position = node.p;
                target.velocity = node.v;
                target.acceleration = node.a;
                if (node.v.head<2>().norm() > 1.0e-3) {
                    target.yaw = std::atan2(node.v.y(), node.v.x());
                    target.yaw = last_yaw + std::atan2(std::sin(target.yaw - last_yaw),
                                                       std::cos(target.yaw - last_yaw));
                } else {
                    target.yaw = last_yaw;
                }
                target.yaw_rate = 0.0;
                last_yaw = target.yaw;
                prediction.emplace_back(target);
            }
            for (std::size_t i = 1; i < prediction.size(); ++i) {
                const double local_dt = std::max(0.05, prediction[i].t - prediction[i - 1].t);
                prediction[i - 1].yaw_rate = (prediction[i].yaw - prediction[i - 1].yaw) / local_dt;
            }
            prediction.back().yaw_rate = prediction.size() > 1
                                             ? prediction[prediction.size() - 2].yaw_rate
                                             : 0.0;
            return true;
        }

        void trackingTargetCallback(const nav_msgs::OdometryConstPtr &msg) {
            if (cfg_.tracking_use_target_prediction_path &&
                !cfg_.tracking_target_prediction_topic.empty() &&
                !last_tracking_prediction_path_time_.isZero() &&
                (ros::Time::now() - last_tracking_prediction_path_time_).toSec() < 0.5) {
                return;
            }
            const Vec3f p(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
            const Vec3f v(msg->twist.twist.linear.x,
                          msg->twist.twist.linear.y,
                          msg->twist.twist.linear.z);
            const double pose_yaw = yawFromMsgQuat(msg->pose.pose.orientation);

            traj_opt::DynamicTargetStates prediction;
            bool used_kinodynamic_prediction = false;
            if (!cfg_.tracking_prediction_use_kinodynamic ||
                !(used_kinodynamic_prediction =
                          buildKinodynamicTrackingPrediction(p, v, pose_yaw, prediction))) {
                buildConstantVelocityTrackingPrediction(p, v, pose_yaw, prediction);
            }
            setTrackingTargetPrediction(prediction);
            traj_opt::DynamicTargetStates accepted_prediction;
            if (getTrackingTargetPrediction(accepted_prediction)) {
                const double source_stamp = msg->header.stamp.isZero()
                                            ? -1.0
                                            : msg->header.stamp.toSec();
                recordTrackingTargetInput(used_kinodynamic_prediction
                                              ? "target_odom_kinodynamic"
                                              : "target_odom_constant_velocity",
                                          accepted_prediction,
                                          source_stamp,
                                          prediction.size());
            }
        }

        static Vec3f poseMsgPosition(const geometry_msgs::PoseStamped &pose) {
            return Vec3f(pose.pose.position.x,
                         pose.pose.position.y,
                         pose.pose.position.z);
        }

        void trackingPredictionPathCallback(const nav_msgs::PathConstPtr &msg) {
            if (!cfg_.tracking_use_target_prediction_path || msg->poses.size() < 2) {
                if (useTrackingLogStream()) {
                    recordDiagnosticEvent("WARN",
                                          "tracking_target_input_rejected",
                                          fmt::format("source=target_prediction_path;reason={};raw_samples={}",
                                                      cfg_.tracking_use_target_prediction_path
                                                          ? "insufficient_path_samples"
                                                          : "prediction_path_disabled",
                                                      msg->poses.size()));
                }
                return;
            }

            const double dt = std::max(0.05, cfg_.tracking_prediction_dt);
            const double horizon = std::max(dt, cfg_.tracking_prediction_horizon);
            const std::size_t max_samples =
                static_cast<std::size_t>(std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1));
            const std::size_t sample_num = std::min(max_samples, msg->poses.size());

            std::vector<Vec3f> positions;
            positions.reserve(sample_num);
            for (std::size_t i = 0; i < sample_num; ++i) {
                positions.emplace_back(poseMsgPosition(msg->poses[i]));
            }

            traj_opt::DynamicTargetStates prediction;
            prediction.reserve(sample_num);
            for (std::size_t i = 0; i < sample_num; ++i) {
                Vec3f velocity = Vec3f::Zero();
                if (i + 1 < sample_num) {
                    velocity = (positions[i + 1] - positions[i]) / dt;
                } else if (i > 0) {
                    velocity = (positions[i] - positions[i - 1]) / dt;
                }

                Vec3f acceleration = Vec3f::Zero();
                if (i > 0 && i + 1 < sample_num) {
                    acceleration = (positions[i + 1] - 2.0 * positions[i] + positions[i - 1]) / (dt * dt);
                }

                traj_opt::DynamicTargetState target;
                target.t = static_cast<double>(i) * dt;
                target.position = positions[i];
                target.velocity = velocity;
                target.acceleration = acceleration;
                const double pose_yaw = yawFromMsgQuat(msg->poses[i].pose.orientation);
                target.yaw = velocity.head<2>().norm() > 1.0e-3
                                 ? std::atan2(velocity.y(), velocity.x())
                                 : pose_yaw;
                target.yaw_rate = 0.0;
                prediction.emplace_back(target);
            }

            last_tracking_prediction_path_time_ = ros::Time::now();
            setTrackingTargetPrediction(prediction);
            traj_opt::DynamicTargetStates accepted_prediction;
            if (getTrackingTargetPrediction(accepted_prediction)) {
                double source_stamp = msg->header.stamp.isZero()
                                      ? -1.0
                                      : msg->header.stamp.toSec();
                if (source_stamp < 0.0 && !msg->poses.front().header.stamp.isZero()) {
                    source_stamp = msg->poses.front().header.stamp.toSec();
                }
                recordTrackingTargetInput("target_prediction_path",
                                          accepted_prediction,
                                          source_stamp,
                                          msg->poses.size());
            }
        }

        void perchingSurfaceCallback(const nav_msgs::OdometryConstPtr &msg) {
            traj_opt::PerchingSurfaceState surface;
            surface.t = 0.0;
            surface.position = Vec3f(msg->pose.pose.position.x,
                                     msg->pose.pose.position.y,
                                     msg->pose.pose.position.z);
            surface.velocity = Vec3f(msg->twist.twist.linear.x,
                                     msg->twist.twist.linear.y,
                                     msg->twist.twist.linear.z);
            surface.acceleration.setZero();

            Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                                 msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z);
            if (q.norm() < 1.0e-6) {
                q.setIdentity();
            } else {
                q.normalize();
            }
            const Eigen::Matrix3d R = q.toRotationMatrix();
            surface.surface_x = R.col(0);
            surface.surface_y = R.col(1);
            surface.surface_z = R.col(2);
            surface.yaw = yawFromMsgQuat(msg->pose.pose.orientation);
            surface.yaw_rate = msg->twist.twist.angular.z;
            setPerchingSurface(surface);
        }

        void taskModeCallback(const std_msgs::StringConstPtr &msg) {
            setTaskModeFromString(msg->data);
        }

        void applyLaunchOverrides() {
            int swarm_drone_id = cfg_.swarm_drone_id;
            bool swarm_id_overridden = nh_.getParam("swarm_drone_id", swarm_drone_id);
            if (!swarm_id_overridden) {
                swarm_id_overridden = nh_.getParam("general_planner/swarm/drone_id", swarm_drone_id);
            }
            if (swarm_id_overridden) {
                cfg_.swarm_drone_id = swarm_drone_id;
                cout << YELLOW << " -- [Fsm] LAUNCH OVERRIDE: swarm_drone_id="
                     << cfg_.swarm_drone_id << RESET << endl;
            }
        }

        void init(const ros::NodeHandle &nh, const std::string &cfg_path) {
            // 初始化参数读取
            nh_ = nh;
            cfg_ = Config(cfg_path);
            applyLaunchOverrides();
            ros_adapter_contract_.adapter_name = "ros1";
            ros_adapter_contract_.cloud_topic = cfg_.dynamic_obstacle_layer_cloud_topic;
            ros_adapter_contract_.target_topic = cfg_.tracking_target_odom_topic;
            ros_adapter_contract_.command_topic = cfg_.cmd_topic;
            ros_adapter_contract_.trajectory_topic = cfg_.mpc_cmd_topic;
            ros_adapter_contract_.mission = cfg_.mission_mode;
            map_ptr_ = std::make_shared<rog_map::ROGMapROS>(nh, cfg_path);
            // 初始化Planner
            auto ros1_ptr = std::make_shared<ros_interface::Ros1Interface>(nh_);
            ros_ptr_ = ros1_ptr;
            planner_ptr_ = std::make_shared<GeneralPlanner>(cfg_path, ros_ptr_, map_ptr_);
            planner_ptr_->setSwarmDroneId(cfg_.swarm_drone_id);
            topology_graph_ros1_ = std::make_shared<general_planner::TopologyGraphROS1>(
                    nh_, planner_ptr_->getMapManager(), "topology",
                    [this]() { return state2stateMode(); });
            if (cfg_.dynamic_obstacle_layer_enable) {
                dynamic_obstacle_cloud_sub_ =
                        nh_.subscribe<sensor_msgs::PointCloud2>(
                                cfg_.dynamic_obstacle_layer_cloud_topic,
                                std::max(1, cfg_.dynamic_obstacle_layer_cloud_queue_size),
                                &FsmRos1::dynamicObstacleCloudCallback,
                                this,
                                ros::TransportHints().tcpNoDelay());
                cout << YELLOW << " -- [Fsm] DYNAMIC OBSTACLE LAYER ENABLE: cloud "
                     << cfg_.dynamic_obstacle_layer_cloud_topic
                     << ", queue " << std::max(1, cfg_.dynamic_obstacle_layer_cloud_queue_size)
                     << RESET << endl;
            }
            cmd_pub = nh_.advertise<quadrotor_msgs::PositionCommand>(cfg_.cmd_topic, 10);
            if (cfg_.publish_so3_cmd) {
                so3_cmd_pub_ = nh_.advertise<quadrotor_msgs::SO3Command>(cfg_.so3_cmd_topic, 10);
            }
            mpc_cmd_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>(cfg_.mpc_cmd_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>("fsm/path", 100);
            if (cfg_.diagnostic_log_en) {
                diagnostic_event_pub_ = nh_.advertise<std_msgs::String>(cfg_.diagnostic_event_topic, 100);
            }

            int cmd_cnt = 0;

            if (cfg_.swarm_enable) {
                if (cfg_.swarm_broadcast_enable) {
                    swarm_traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>(
                        cfg_.swarm_traj_broadcast_topic, 20);
                    swarm_state_pub_ = nh_.advertise<nav_msgs::Odometry>(
                        cfg_.swarm_state_broadcast_topic, 50);
                    swarm_broadcast_traj_sub_ =
                        nh_.subscribe<quadrotor_msgs::PolynomialTrajectory>(
                            cfg_.swarm_traj_broadcast_topic, 50,
                            [this](const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg) {
                                this->swarmTrajCallback(msg, -1);
                            });
                    swarm_state_sub_ = nh_.subscribe(cfg_.swarm_state_broadcast_topic, 50,
                                                     &FsmRos1::swarmStateCallback, this);
                    cout << YELLOW << " -- [Fsm] SWARM BROADCAST ENABLE: traj "
                         << cfg_.swarm_traj_broadcast_topic << ", state "
                         << cfg_.swarm_state_broadcast_topic << RESET << endl;
                }
                if (cfg_.swarm_formation_reference_enable) {
                    swarm_formation_reference_sub_ =
                        nh_.subscribe(cfg_.swarm_formation_reference_topic, 10,
                                      &FsmRos1::swarmFormationReferenceCallback, this,
                                      ros::TransportHints().tcpNoDelay());
                    cout << YELLOW << " -- [Fsm] SWARM FORMATION REFERENCE SUB: "
                         << cfg_.swarm_formation_reference_topic << RESET << endl;
                }
                for (size_t i = 0; i < cfg_.swarm_traj_topics.size(); ++i) {
                    const std::string &topic = cfg_.swarm_traj_topics[i];
                    const int drone_id = i < cfg_.swarm_traj_ids.size()
                                             ? cfg_.swarm_traj_ids[i]
                                             : static_cast<int>(i);
                    if (topic.empty() || drone_id == cfg_.swarm_drone_id) {
                        continue;
                    }
                    swarm_traj_subs_.emplace_back(
                        nh_.subscribe<quadrotor_msgs::PolynomialTrajectory>(
                            topic, 20,
                            [this, drone_id](const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg) {
                                this->swarmTrajCallback(msg, drone_id);
                            }));
                    cout << YELLOW << " -- [Fsm] SWARM TRAJ SUB: drone " << drone_id
                         << ", topic: " << topic << RESET << endl;
                }
            }

            if (cfg_.task_planner_en) {
                task_mode_sub_ = nh_.subscribe(cfg_.task_mode_topic, 10,
                                               &FsmRos1::taskModeCallback, this);
                goal_sub_ = nh_.subscribe(cfg_.click_goal_topic, 10,
                                          &FsmRos1::goalCallback, this);
                subscribe3DGoal(10, true);
                tracking_target_sub_ = nh_.subscribe(cfg_.tracking_target_odom_topic, 10,
                                                     &FsmRos1::trackingTargetCallback, this);
                if (cfg_.tracking_use_target_prediction_path && !cfg_.tracking_target_prediction_topic.empty()) {
                    tracking_prediction_sub_ =
                        nh_.subscribe(cfg_.tracking_target_prediction_topic, 10,
                                      &FsmRos1::trackingPredictionPathCallback, this);
                }
                perching_surface_sub_ = nh_.subscribe(cfg_.perching_surface_odom_topic, 10,
                                                      &FsmRos1::perchingSurfaceCallback, this);
                cout << YELLOW << " -- [Fsm] TASK PLANNER ENABLE: mode topic "
                     << cfg_.task_mode_topic << ", state2state goal "
                     << cfg_.click_goal_topic << ", tracking target "
                     << cfg_.tracking_target_odom_topic << ", perching surface "
                     << cfg_.perching_surface_odom_topic << RESET << endl;
                if (cfg_.tracking_use_target_prediction_path && !cfg_.tracking_target_prediction_topic.empty()) {
                    cout << YELLOW << " -- [Fsm] TRACKING PREDICTION PATH: "
                         << cfg_.tracking_target_prediction_topic << RESET << endl;
                }
                cmd_cnt++;
            } else if ((state2stateMode() || se3AggressiveMode()) &&
                       (cfg_.click_goal_en || cfg_.click_goal_3d_en)) {
                bool goal_input_ready = false;
                if (cfg_.click_goal_en) {
                    goal_sub_ = nh_.subscribe(cfg_.click_goal_topic, 1,
                                              &FsmRos1::goalCallback, this);
                    cout << YELLOW << " -- [Fsm] 2D CLICK GOAL ENABLED: "
                         << nh_.resolveName(cfg_.click_goal_topic)
                         << " (z falls back to click_height=" << cfg_.click_height
                         << ")." << RESET << endl;
                    goal_input_ready = true;
                }
                goal_input_ready = subscribe3DGoal(1, cfg_.click_goal_en) || goal_input_ready;
                if (goal_input_ready) {
                    cmd_cnt++;
                }
            } else if (trackingMode() || trackingPerchingMode()) {
                tracking_target_sub_ = nh_.subscribe(cfg_.tracking_target_odom_topic, 10,
                                                     &FsmRos1::trackingTargetCallback, this);
                if (cfg_.tracking_use_target_prediction_path && !cfg_.tracking_target_prediction_topic.empty()) {
                    tracking_prediction_sub_ =
                        nh_.subscribe(cfg_.tracking_target_prediction_topic, 10,
                                      &FsmRos1::trackingPredictionPathCallback, this);
                }
                cout << YELLOW << " -- [Fsm] TRACKING TASK ENABLE, target odom: "
                     << cfg_.tracking_target_odom_topic << RESET << endl;
                if (cfg_.tracking_perching_enable || trackingPerchingMode()) {
                    task_mode_sub_ = nh_.subscribe(cfg_.task_mode_topic, 10,
                                                   &FsmRos1::taskModeCallback, this);
                    perching_surface_sub_ = nh_.subscribe(cfg_.perching_surface_odom_topic, 10,
                                                          &FsmRos1::perchingSurfaceCallback, this);
                    cout << YELLOW << " -- [Fsm] TRACKING-PERCHING ENABLE, request mode topic: "
                         << cfg_.task_mode_topic << ", surface odom: "
                         << cfg_.perching_surface_odom_topic << RESET << endl;
                }
                if (cfg_.tracking_use_target_prediction_path && !cfg_.tracking_target_prediction_topic.empty()) {
                    cout << YELLOW << " -- [Fsm] TRACKING PREDICTION PATH: "
                         << cfg_.tracking_target_prediction_topic << RESET << endl;
                }
                cmd_cnt++;
            } else if (perchingMode()) {
                task_mode_sub_ = nh_.subscribe(cfg_.task_mode_topic, 10,
                                               &FsmRos1::taskModeCallback, this);
                perching_surface_sub_ = nh_.subscribe(cfg_.perching_surface_odom_topic, 10,
                                                      &FsmRos1::perchingSurfaceCallback, this);
                cout << YELLOW << " -- [Fsm] PERCHING TASK ENABLE, surface odom: "
                     << cfg_.perching_surface_odom_topic
                     << ", mode switch topic: " << cfg_.task_mode_topic << RESET << endl;
                cmd_cnt++;
            } else if (dynamicTakeoffMode()) {
                task_mode_sub_ = nh_.subscribe(cfg_.task_mode_topic, 10,
                                               &FsmRos1::taskModeCallback, this);
                perching_surface_sub_ = nh_.subscribe(cfg_.perching_surface_odom_topic, 10,
                                                      &FsmRos1::perchingSurfaceCallback, this);
                cout << YELLOW << " -- [Fsm] DYNAMIC TAKEOFF TASK ENABLE, surface odom: "
                     << cfg_.perching_surface_odom_topic
                     << ", mode switch topic: " << cfg_.task_mode_topic << RESET << endl;
                cmd_cnt++;
            } else if (explorationMode()) {
                cout << YELLOW << " -- [Fsm] EXPLORATION TASK ENABLE." << RESET << endl;
                cmd_cnt++;
            }

            if (cmd_cnt != 1) {
                cout << YELLOW << " -- [Fsm] CMD INPUT ERROR." << RESET << endl;
                exit(0);
            }

            if (cfg_.timer_en) {
                execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FsmRos1::mainFsmTimerCallback, this); // 100Hz
                cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &FsmRos1::pubCmdTimerCallback, this); // 100Hz
                replan_timer_ = nh_.createTimer(ros::Duration(1.0 / cfg_.replan_rate), &FsmRos1::replanTimerCallback,
                                                this); // 10Hz
                if ((cfg_.perception_replan_check_en || cfg_.dynamic_obstacle_layer_enable) &&
                    cfg_.perception_replan_check_rate > 1.0e-3) {
                    perception_safety_timer_ = nh_.createTimer(
                            ros::Duration(1.0 / cfg_.perception_replan_check_rate),
                            &FsmRos1::perceptionSafetyTimerCallback,
                            this);
                }
            }

            std::string time_log_file;
            if (useTrackingLogStream()) {
                time_log_file = "tracking_time_consuming.csv";
            } else if (state2stateMode() &&
                       cfg_.backend_type == general_planner::architecture::BackendType::CORRIDOR) {
                const auto timing_report = planner_ptr_->getLatestExpTimingReport();
                time_log_file = "time_consuming_" + timing_report.mode + ".csv";
            } else {
                time_log_file = "time_consuming.csv";
            }
            write_time_.open(DEBUG_FILE_DIR(time_log_file), std::ios::out | std::ios::trunc);
            log_module_time.resize(9);
            const bool write_exp_timing =
                    state2stateMode() &&
                    cfg_.backend_type == general_planner::architecture::BackendType::CORRIDOR;
            for (int i = 0; i < 9; i++) {
                write_time_ << log_time_str[i];
                if (i != 8 || write_exp_timing) {
                    write_time_ << ",";
                }
            }
            if (write_exp_timing) {
                write_time_ << "EXP_COST_MODE,EXP_EVALUATIONS,EXP_LBFGS_ITERATIONS,"
                               "EXP_LINE_SEARCH_EVALUATIONS,EXP_AVG_LINE_SEARCH_EVALS,"
                               "EXP_MAX_LINE_SEARCH_EVALS,EXP_AVG_ACCEPTED_STEP,"
                               "EXP_MIN_ACCEPTED_STEP,EXP_POLYNOMIAL_PIECES,"
                               "EXP_DENSE_NODES_PER_EVAL,EXP_HULL_CONTROL_CHECKS_PER_EVAL,"
                               "EXP_SCALAR_CONSTRAINT_CHECKS,"
                               "EXP_ALM_CONSTRAINTS,EXP_ALM_OUTER_ITERATIONS,"
                               "EXP_ALM_INNER_SOLVES,EXP_ALM_TOPOLOGY_CHANGES,"
                               "EXP_ADAPTIVE_COARSE_SEGMENTS,EXP_ADAPTIVE_FINE_SEGMENTS,"
                               "EXP_ALM_MAX_VIOLATION,EXP_ALM_CERTIFIED,EXP_ALM_WARM_START_MS,"
                               "EXP_DENSE_INTEGRAL_MS,EXP_CONTROL_POINT_FUNCTIONAL_MS,"
                               "EXP_HULL_TRANSFORM_MS,EXP_HULL_HODOGRAPH_MS,"
                               "EXP_HULL_POSITION_RESIDUAL_MS,EXP_HULL_DERIVATIVE_RESIDUAL_MS,"
                               "EXP_HULL_REVERSE_HODOGRAPH_MS,EXP_HULL_BACKWARD_ADD_MS,"
                               "EXP_HULL_DISCRETE_ATTRACTOR_MS,"
                               "EXP_MINCO_EVALUATION_MS,EXP_LBFGS_MS,"
                               "EXP_DENSE_SHARE_MINCO_PERCENT,EXP_CONTROL_POINT_SHARE_MINCO_PERCENT,"
                               "EXP_DENSE_SHARE_OPT_PERCENT,"
                               "EXP_LBFGS_SHARE_MODULE_PERCENT,EXP_MODULE_SHARE_REPLAN_PERCENT,"
                               "EXP_FAST_STOP,EXP_FAST_STOP_CHECKS,"
                               "EXP_FAST_COST_PASSES,EXP_FAST_STEP_PASSES,"
                               "EXP_FAST_PENALTY_CHANGE_PASSES,EXP_FAST_TIME_PASSES,"
                               "EXP_FAST_WAYPOINT_PASSES,EXP_FAST_GRADIENT_PASSES,"
                               "EXP_FAST_VIOLATION_PASSES,EXP_FAST_NONSTALL_PASSES,"
                               "EXP_FAST_BASE_RULE_PASSES,EXP_FAST_GUARDED_RULE_PASSES,"
                               "EXP_WARM_START_STATUS,EXP_WARM_START_ATTEMPTED,"
                               "EXP_WARM_START_ACCEPTED,EXP_WARM_START_RESAMPLED,"
                               "EXP_WARM_START_COMPARE_EVALS,"
                               "EXP_WARM_START_MS,EXP_WARM_START_BASELINE_COST,"
                               "EXP_WARM_START_CANDIDATE_COST,"
                               "EXP_WARM_START_BASELINE_GRADIENT,"
                               "EXP_WARM_START_CANDIDATE_GRADIENT,"
                               "EXP_WARM_START_BASELINE_PENALTY,"
                               "EXP_WARM_START_CANDIDATE_PENALTY,"
                               "EXP_WARM_START_MAX_WAYPOINT_SHIFT,"
                               "EXP_CONTINUOUS_FEASIBLE,EXP_ROBUSTLY_CERTIFIED,"
                               "EXP_HAS_CERTIFIED_INCUMBENT,EXP_MAX_NORMALIZED_VIOLATION,"
                               "EXP_MIN_POSITION_MARGIN,EXP_PRIMAL_RESIDUAL,EXP_DUAL_RESIDUAL,"
                               "EXP_COMPLEMENTARITY_RESIDUAL,EXP_STATIONARITY_RESIDUAL,"
                               "EXP_PHASE2_TRIGGERED,EXP_PHASE2_PACKED_CONSTRAINTS,"
                               "EXP_JERK_CERTIFICATE_ENABLED";
            }
            write_time_ << endl;
            machine_state_ = INIT;
            system_start_time_ = ros_ptr_->getSimTime();
            openDiagnosticLogFile(LOG_FILE_DIR("diagnostic_events/general_runtime.csv"));
            openTrackingDiagnosticLogFile(LOG_FILE_DIR("tracking_diagnostic_events/tracking_runtime.csv"));
            recordDiagnosticEvent("INFO",
                                  "fsm_initialized",
                                  fmt::format("task_mode={};mission={};task={};backend={};adapter={};replan_rate={:.3f};perception_replan_check_en={};perception_replan_check_rate={:.3f};dynamic_obstacle_layer_enable={};dynamic_obstacle_cloud_topic={};target_topic={};cmd_topic={};mpc_cmd_topic={}",
                                              cfg_.task_mode_str,
                                              general_planner::architecture::toString(cfg_.mission_mode),
                                              general_planner::architecture::toString(cfg_.task_type),
                                              general_planner::architecture::toString(cfg_.backend_type),
                                              ros_adapter_contract_.adapter_name,
                                              cfg_.replan_rate,
                                              static_cast<int>(cfg_.perception_replan_check_en),
                                              cfg_.perception_replan_check_rate,
                                              static_cast<int>(cfg_.dynamic_obstacle_layer_enable),
                                              cfg_.dynamic_obstacle_layer_cloud_topic,
                                              ros_adapter_contract_.target_topic,
                                              cfg_.cmd_topic,
                                              cfg_.mpc_cmd_topic),
                                  -1,
                                  -1,
                                  false,
                                  -1,
                                  0);
            if (useTrackingLogStream()) {
                recordDiagnosticEvent("INFO",
                                      "tracking_config_snapshot",
                                      fmt::format("target_odom_topic={};target_prediction_topic={};use_prediction_path={};"
                                                  "prediction_horizon={:.3f};prediction_dt={:.3f};prediction_kinodynamic={};"
                                                  "task_timeout={:.3f};cmd_topic={};mpc_cmd_topic={};planner_config={}",
                                                  cfg_.tracking_target_odom_topic,
                                                  cfg_.tracking_target_prediction_topic,
                                                  static_cast<int>(cfg_.tracking_use_target_prediction_path),
                                                  cfg_.tracking_prediction_horizon,
                                                  cfg_.tracking_prediction_dt,
                                                  static_cast<int>(cfg_.tracking_prediction_use_kinodynamic),
                                                  cfg_.task_timeout,
                                                  cfg_.cmd_topic,
                                                  cfg_.mpc_cmd_topic,
                                                  planner_ptr_ ? planner_ptr_->getTrackingConfigSummary()
                                                               : "planner_missing"),
                                      -1,
                                      -1,
                                      false,
                                      -1,
                                      0);
            }
            if (cfg_.auto_start) {
                started_ = true;
                if (explorationMode()) {
                    task_new_ = true;
                }
                cout << YELLOW << " -- [Fsm] AUTO START ENABLE." << RESET << endl;
            }

            pid_cmd_.kx[0] = 5.7;
            pid_cmd_.kx[1] = 5.7;
            pid_cmd_.kx[2] = 4.2;

            pid_cmd_.kv[0] = 3.4;
            pid_cmd_.kv[1] = 3.4;
            pid_cmd_.kv[2] = 4.0;
        }

        void pubCmdTimerCallback(const ros::TimerEvent &event) {
            if (stop) {
                return;
            }
            if (machine_state_ != FOLLOW_TRAJ &&
                machine_state_ != STATIC_TRACKING &&
                machine_state_ != EMER_STOP) {
                return;
            }

            quadrotor_msgs::PolynomialTrajectory heartbeat;
            getOneHeartBeatMsg(heartbeat, traj_finish_);
            getOnePositionCommand(pid_cmd_, traj_finish_);
            publishSwarmState();
            mpc_cmd_pub_.publish(heartbeat);
            cmd_pub.publish(pid_cmd_);
            if (cfg_.publish_so3_cmd) {
                quadrotor_msgs::SO3Command so3_cmd;
                if (getOneSO3Command(pid_cmd_, so3_cmd)) {
                    so3_cmd_pub_.publish(so3_cmd);
                }
            }
            if (traj_finish_) {
                const auto finish_result = handleExecutedTrajectoryFinished(
                        "PubCmdCallback",
                        pid_cmd_.trajectory_id,
                        static_cast<int>(traj_seq_),
                        pid_cmd_.trajectory_flag == 2,
                        true,
                        true);
                if (finish_result.tracking_unfinished) {
                    traj_finish_ = false;
                    return;
                }
            }
        }

        void replanTimerCallback(const ros::TimerEvent &event) {
            callReplanOnce();
        }

        void perceptionSafetyTimerCallback(const ros::TimerEvent &event) {
            callPerceptionSafetyCheckOnce();
        }

        void mainFsmTimerCallback(const ros::TimerEvent &event) {
            callMainFsmOnce();
        }

    };
}

#endif //SRC_FSM_ROS1_HPP

#endif
