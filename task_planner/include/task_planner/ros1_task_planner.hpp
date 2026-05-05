#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include "task_planner/config.hpp"
#include "utils/color_msg_utils.hpp"

namespace task_planner {
    class TaskPlanner {
    private:
        TaskPlannerConfig cfg_;
        ros::NodeHandle nh_;
        ros::Publisher task_mode_pub_;
        ros::Publisher goal_pub_;
        ros::Publisher tracking_target_pub_;
        ros::Publisher tracking_target_path_pub_;
        ros::Publisher tracking_prediction_pub_;
        ros::Publisher perching_surface_pub_;
        ros::Publisher path_pub_;
        ros::Publisher mkr_pub_;
        ros::Subscriber click_sub_;
        ros::Subscriber mavros_sub_;
        ros::Subscriber odom_sub_;
        ros::Timer task_timer_;

        Eigen::Vector3d cur_position_{Eigen::Vector3d::Zero()};
        bool had_odom_{false};
        bool triggered_{false};
        bool trigger_once_{false};
        bool new_task_{true};
        int task_id_{0};
        double odom_rcv_time_{0.0};
        double system_start_time_{0.0};
        double task_start_time_{0.0};
        double last_pub_time_{0.0};

        void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
            had_odom_ = true;
            odom_rcv_time_ = ros::Time::now().toSec();
            cur_position_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z);
        }

        void rvizClickCallback(const geometry_msgs::PoseStampedConstPtr &) {
            if (!had_odom_) {
                return;
            }
            triggered_ = true;
            switchToTask(0);
            std::cout << YELLOW << " -- [TASK_PLANNER] Rviz triggered." << RESET << std::endl;
        }

        void mavrosRcCallback(const mavros_msgs::RCInConstPtr &msg) {
            if (!had_odom_ || msg->channels.size() <= 9) {
                return;
            }
            static int last_ch_10 = 1000;
            const int ch_10 = msg->channels[9];
            if (last_ch_10 > 1500 && ch_10 < 1500) {
                triggered_ = true;
                switchToTask(0);
                std::cout << YELLOW << " -- [TASK_PLANNER] Mavros triggered." << RESET << std::endl;
            }
            last_ch_10 = ch_10;
        }

        static geometry_msgs::Quaternion quatFromRpy(double roll, double pitch, double yaw) {
            const Eigen::Quaterniond q =
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
            geometry_msgs::Quaternion msg;
            msg.w = q.w();
            msg.x = q.x();
            msg.y = q.y();
            msg.z = q.z();
            return msg;
        }

        static std::vector<double> allocatePathTime(const vec_E<Vec3f> &path, double speed) {
            std::vector<double> times(path.size(), 0.0);
            speed = std::max(0.2, speed);
            for (int i = 1; i < static_cast<int>(path.size()); ++i) {
                times[static_cast<size_t>(i)] =
                    times[static_cast<size_t>(i - 1)] +
                    std::max(0.05, (path[i] - path[i - 1]).norm() / speed);
            }
            return times;
        }

        static Vec3f interpolatePath(const vec_E<Vec3f> &path,
                                     const std::vector<double> &times,
                                     const double t,
                                     Vec3f &velocity) {
            velocity.setZero();
            if (path.empty()) {
                return Vec3f::Zero();
            }
            if (path.size() == 1 || t <= times.front()) {
                return path.front();
            }
            if (t >= times.back()) {
                return path.back();
            }
            const auto upper = std::lower_bound(times.begin(), times.end(), t);
            const int idx = static_cast<int>(std::distance(times.begin(), upper));
            const double t0 = times[static_cast<size_t>(idx - 1)];
            const double t1 = times[static_cast<size_t>(idx)];
            const double alpha = (t - t0) / std::max(1.0e-6, t1 - t0);
            velocity = (path[static_cast<size_t>(idx)] - path[static_cast<size_t>(idx - 1)]) /
                       std::max(1.0e-6, t1 - t0);
            return path[static_cast<size_t>(idx - 1)] +
                   alpha * (path[static_cast<size_t>(idx)] - path[static_cast<size_t>(idx - 1)]);
        }

        static double wrapPathTime(const double t, const double total_t, const bool loop) {
            if (total_t <= 1.0e-6) {
                return 0.0;
            }
            if (!loop) {
                return std::min(std::max(0.0, t), total_t);
            }
            double wrapped = std::fmod(t, total_t);
            if (wrapped < 0.0) {
                wrapped += total_t;
            }
            return wrapped;
        }

        static void sampleTrackingTask(const ManagedTask &task,
                                       const double elapsed,
                                       Vec3f &p,
                                       Vec3f &v,
                                       Vec3f &a) {
            p = task.position;
            v = task.velocity;
            a = task.acceleration;
            if (task.waypoints.size() >= 2) {
                const auto times = allocatePathTime(task.waypoints, task.speed);
                const double total_t = std::max(0.05, times.back());
                const double path_t = wrapPathTime(elapsed, total_t, task.loop);
                p = interpolatePath(task.waypoints, times, path_t, v);
                a.setZero();
                return;
            }
            p = task.position + task.velocity * elapsed + 0.5 * task.acceleration * elapsed * elapsed;
            v = task.velocity + task.acceleration * elapsed;
        }

        geometry_msgs::PoseStamped makePose(const Vec3f &p, const Vec3f &v, const Vec3f &rpy, const double yaw) const {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = cfg_.frame_id;
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();
            const double pose_yaw = std::isfinite(yaw)
                                        ? yaw
                                        : (v.head<2>().norm() > 1.0e-3 ? std::atan2(v.y(), v.x()) : rpy.z());
            pose.pose.orientation = quatFromRpy(rpy.x(), rpy.y(), pose_yaw);
            return pose;
        }

        bool closeToTaskGoal(const ManagedTask &task) const {
            return (task.position - cur_position_).norm() < task.switch_dis;
        }

        double trackingPathDuration(const ManagedTask &task) const {
            if (task.waypoints.size() < 2) {
                return std::max(0.0, task.hold_duration);
            }
            const auto times = allocatePathTime(task.waypoints, task.speed);
            return times.empty() ? 0.0 : times.back();
        }

        bool taskShouldAdvance(const ManagedTask &task, const double elapsed) const {
            if (task.mode == ManagedTaskMode::STATE_TO_STATE) {
                return closeToTaskGoal(task);
            }
            if (task.mode == ManagedTaskMode::TRACKING) {
                if (task.loop || task.hold_duration < 0.0) {
                    return false;
                }
                return elapsed > trackingPathDuration(task) + task.hold_duration;
            }
            if (task.mode == ManagedTaskMode::PERCHING) {
                return task.hold_duration > 0.0 && elapsed > task.hold_duration;
            }
            return false;
        }

        void switchToTask(const int id) {
            task_id_ = id;
            task_start_time_ = ros::Time::now().toSec();
            last_pub_time_ = 0.0;
            new_task_ = true;
            if (task_id_ >= 0 && task_id_ < static_cast<int>(cfg_.tasks.size())) {
                const auto &task = cfg_.tasks[static_cast<size_t>(task_id_)];
                std::cout << GREEN << " -- [TASK_PLANNER] Switch to task " << task_id_
                          << " [" << task.name << "], mode=" << taskModeToString(task.mode)
                          << RESET << std::endl;
            }
            visualizeTasks();
        }

        void advanceTask() {
            const int next_id = task_id_ + 1;
            if (next_id >= static_cast<int>(cfg_.tasks.size())) {
                triggered_ = false;
                new_task_ = false;
                std::cout << GREEN << " -- [TASK_PLANNER] Mission finished." << RESET << std::endl;
                visualizeTasks();
                return;
            }
            switchToTask(next_id);
        }

        void publishTaskMode(const ManagedTask &task) {
            std_msgs::String msg;
            msg.data = taskModeToString(task.mode);
            task_mode_pub_.publish(msg);
        }

        void publishStateToStateGoal(const ManagedTask &task) {
            geometry_msgs::PoseStamped goal;
            goal.header.frame_id = cfg_.frame_id;
            goal.header.stamp = ros::Time::now();
            goal.pose.position.x = task.position.x();
            goal.pose.position.y = task.position.y();
            goal.pose.position.z = task.position.z();
            const double yaw = std::isfinite(task.yaw) ? task.yaw : task.rpy.z();
            goal.pose.orientation = quatFromRpy(task.rpy.x(), task.rpy.y(), yaw);
            goal_pub_.publish(goal);
        }

        void fillOdom(nav_msgs::Odometry &odom,
                      const std::string &child_frame_id,
                      const Vec3f &p,
                      const Vec3f &v,
                      const geometry_msgs::Quaternion &q) const {
            odom.header.frame_id = cfg_.frame_id;
            odom.header.stamp = ros::Time::now();
            odom.child_frame_id = child_frame_id;
            odom.pose.pose.position.x = p.x();
            odom.pose.pose.position.y = p.y();
            odom.pose.pose.position.z = p.z();
            odom.pose.pose.orientation = q;
            odom.twist.twist.linear.x = v.x();
            odom.twist.twist.linear.y = v.y();
            odom.twist.twist.linear.z = v.z();
        }

        void publishTrackingTarget(const ManagedTask &task, double elapsed) {
            Vec3f p;
            Vec3f v;
            Vec3f a;
            sampleTrackingTask(task, elapsed, p, v, a);
            const double yaw = std::isfinite(task.yaw)
                                   ? task.yaw
                                   : (v.head<2>().norm() > 1.0e-3 ? std::atan2(v.y(), v.x()) : task.rpy.z());
            nav_msgs::Odometry odom;
            fillOdom(odom, task.name, p, v, quatFromRpy(task.rpy.x(), task.rpy.y(), yaw));
            tracking_target_pub_.publish(odom);
            publishTrackingPath(task);
            publishTrackingPrediction(task, elapsed);
        }

        void publishTrackingPath(const ManagedTask &task) {
            if (task.waypoints.empty()) {
                return;
            }
            nav_msgs::Path path;
            path.header.frame_id = cfg_.frame_id;
            path.header.stamp = ros::Time::now();
            for (const auto &pt: task.waypoints) {
                Vec3f zero_v = Vec3f::Zero();
                path.poses.push_back(makePose(pt, zero_v, task.rpy, task.yaw));
                path.poses.back().header = path.header;
            }
            tracking_target_path_pub_.publish(path);
            path_pub_.publish(path);
        }

        void publishTrackingPrediction(const ManagedTask &task, const double elapsed) {
            const double dt = std::max(0.05, cfg_.tracking_prediction_dt);
            const double horizon = std::max(dt, cfg_.tracking_prediction_horizon);
            const int sample_num = std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1);
            nav_msgs::Path prediction;
            prediction.header.frame_id = cfg_.frame_id;
            prediction.header.stamp = ros::Time::now();
            prediction.poses.reserve(static_cast<size_t>(sample_num));
            for (int i = 0; i < sample_num; ++i) {
                Vec3f p;
                Vec3f v;
                Vec3f a;
                sampleTrackingTask(task, elapsed + static_cast<double>(i) * dt, p, v, a);
                prediction.poses.push_back(makePose(p, v, task.rpy, task.yaw));
                prediction.poses.back().header = prediction.header;
            }
            tracking_prediction_pub_.publish(prediction);
        }

        void publishPerchingSurface(const ManagedTask &task, const double elapsed) {
            const Vec3f p = task.position + task.velocity * elapsed + 0.5 * task.acceleration * elapsed * elapsed;
            const Vec3f v = task.velocity + task.acceleration * elapsed;
            const double yaw = std::isfinite(task.yaw) ? task.yaw : task.rpy.z();
            nav_msgs::Odometry odom;
            fillOdom(odom, task.name, p, v, quatFromRpy(task.rpy.x(), task.rpy.y(), yaw));
            perching_surface_pub_.publish(odom);
        }

        void publishPath(const ManagedTask &task, const double elapsed) {
            if (task.waypoints.empty()) {
                return;
            }
            nav_msgs::Path path;
            path.header.frame_id = cfg_.frame_id;
            path.header.stamp = ros::Time::now();
            for (const auto &pt: task.waypoints) {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = pt.x();
                pose.pose.position.y = pt.y();
                pose.pose.position.z = pt.z();
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
            path_pub_.publish(path);
        }

        void publishTask(const ManagedTask &task, const double elapsed) {
            publishTaskMode(task);
            if (task.mode == ManagedTaskMode::STATE_TO_STATE) {
                publishStateToStateGoal(task);
            } else if (task.mode == ManagedTaskMode::TRACKING) {
                publishTrackingTarget(task, elapsed);
            } else if (task.mode == ManagedTaskMode::PERCHING) {
                publishPerchingSurface(task, elapsed);
            }
        }

        void taskTimerCallback(const ros::TimerEvent &) {
            const int last_mkr_sub_num = mkr_pub_.getNumSubscribers();
            if (last_mkr_sub_num > 0) {
                visualizeTasks();
            }

            if (cfg_.start_trigger_type == 2 && !trigger_once_) {
                const double cur_t = ros::Time::now().toSec();
                if (cur_t - system_start_time_ < cfg_.start_program_delay) {
                    return;
                }
                triggered_ = true;
                trigger_once_ = true;
                switchToTask(0);
                std::cout << YELLOW << " -- [TASK_PLANNER] Auto start delay passed." << RESET << std::endl;
            }
            if (!triggered_ || cfg_.tasks.empty()) {
                return;
            }

            const double cur_t = ros::Time::now().toSec();
            if (!had_odom_ || cur_t - odom_rcv_time_ > cfg_.odom_timeout) {
                static double last_print_t = 0.0;
                if (cur_t - last_print_t > 1.0) {
                    last_print_t = cur_t;
                    std::cout << YELLOW << " -- [TASK_PLANNER] Odom timeout." << RESET << std::endl;
                }
                return;
            }

            if (task_id_ < 0 || task_id_ >= static_cast<int>(cfg_.tasks.size())) {
                triggered_ = false;
                return;
            }

            const ManagedTask &task = cfg_.tasks[static_cast<size_t>(task_id_)];
            const double elapsed = std::max(0.0, cur_t - task_start_time_);
            if (taskShouldAdvance(task, elapsed)) {
                std::cout << YELLOW << " -- [TASK_PLANNER] Task " << task_id_
                          << " reached/expired, switch to next." << RESET << std::endl;
                advanceTask();
                return;
            }

            const double task_publish_dt = std::max(0.02, task.publish_dt);
            if (!new_task_ && cur_t - last_pub_time_ < task_publish_dt) {
                return;
            }
            publishTask(task, elapsed);
            last_pub_time_ = cur_t;
            new_task_ = false;
        }

    public:
        TaskPlanner() = default;

        explicit TaskPlanner(const ros::NodeHandle &nh) {
            init(nh);
        }

        void init(const ros::NodeHandle &nh) {
            nh_ = nh;
#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/" + name))
            std::string dft_cfg_path = CONFIG_FILE_DIR("task_planner.yaml");
            std::string cfg_path, cfg_name;
            if (nh_.param("config_path", cfg_path, dft_cfg_path)) {
                std::cout << " -- [TaskPlanner] Load config from: " << cfg_path << std::endl;
            } else if (nh_.param("config_name", cfg_name, std::string("task_planner.yaml"))) {
                cfg_path = CONFIG_FILE_DIR(cfg_name);
                std::cout << " -- [TaskPlanner] Load config by file name: " << cfg_name << std::endl;
            }
#define DATA_FILE_DIR(name) (string(string(ROOT_DIR) + "data/" + name))
            std::string dft_data_path = DATA_FILE_DIR("benchmark_dense.txt");
            std::string data_path, data_name;
            if (nh_.param("data_path", data_path, dft_data_path)) {
                std::cout << " -- [TaskPlanner] Load data from: " << data_path << std::endl;
            } else if (nh_.param("data_name", data_name, std::string("benchmark_dense.txt"))) {
                data_path = DATA_FILE_DIR(data_name);
                std::cout << " -- [TaskPlanner] Load data by file name: " << data_path << std::endl;
            }

            cfg_ = TaskPlannerConfig(cfg_path);
            if (cfg_.tasks.empty()) {
                cfg_.loadLegacyWaypointFile(data_path);
            }
            if (cfg_.tasks.empty()) {
                std::cout << RED << " -- [TASK_PLANNER] No task loaded, shutdown." << RESET << std::endl;
                ros::shutdown();
                return;
            }

            odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &TaskPlanner::odomCallback, this);
            task_mode_pub_ = nh_.advertise<std_msgs::String>(cfg_.task_mode_topic, 10, true);
            goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.goal_pub_topic, 10);
            tracking_target_pub_ = nh_.advertise<nav_msgs::Odometry>(cfg_.tracking_target_odom_topic, 10);
            tracking_target_path_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.tracking_target_path_topic, 1, true);
            tracking_prediction_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.tracking_target_prediction_topic, 10);
            perching_surface_pub_ = nh_.advertise<nav_msgs::Odometry>(cfg_.perching_surface_odom_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>(cfg_.path_pub_topic, 1, true);
            mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(cfg_.marker_topic, 1, true);

            if (cfg_.start_trigger_type == 0) {
                click_sub_ = nh_.subscribe("/goal", 10, &TaskPlanner::rvizClickCallback, this);
            } else if (cfg_.start_trigger_type == 1) {
                mavros_sub_ = nh_.subscribe("/mavros/rc/in", 10, &TaskPlanner::mavrosRcCallback, this);
            }

            system_start_time_ = ros::Time::now().toSec();
            task_start_time_ = system_start_time_;
            task_timer_ = nh_.createTimer(ros::Duration(0.01), &TaskPlanner::taskTimerCallback, this);
            visualizeTasks();
        }

        void visualizeTasks() {
            visualization_msgs::MarkerArray mkr_arr;
            addPathToMarkerArray(mkr_arr, taskPositions(), Color::SteelBlue(), "managed_tasks", 0.35, 0.16);
            for (size_t i = 0; i < cfg_.tasks.size(); ++i) {
                const auto &task = cfg_.tasks[i];
                Color color = i == static_cast<size_t>(task_id_) && triggered_
                                  ? Color::Orange()
                                  : Color::SteelBlue();
                color.a = 0.8;
                visualizePoint(mkr_arr, task.position, color, taskModeToString(task.mode),
                               i == static_cast<size_t>(task_id_) ? 0.65 : 0.45, static_cast<int>(i));
                visualizeText(mkr_arr, "task_id", std::to_string(i) + ":" + task.name,
                              task.position + Vec3f(0.0, 0.0, 0.55), Color::Black(), 0.45, static_cast<int>(i));
            }
            mkr_pub_.publish(mkr_arr);
        }

        vec_E<Vec3f> taskPositions() const {
            vec_E<Vec3f> points;
            for (const auto &task: cfg_.tasks) {
                points.push_back(task.position);
            }
            return points;
        }

        static void visualizePoint(visualization_msgs::MarkerArray &mkr_arr,
                                   const Vec3f &pt,
                                   Color color,
                                   const std::string &ns,
                                   const double size,
                                   const int id) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = ns;
            marker.id = id;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = size;
            marker.scale.y = size;
            marker.scale.z = size;
            marker.color = color;
            marker.pose.position.x = pt.x();
            marker.pose.position.y = pt.y();
            marker.pose.position.z = pt.z();
            mkr_arr.markers.push_back(marker);
        }

        static void visualizeText(visualization_msgs::MarkerArray &mkr_arr,
                                  const std::string &ns,
                                  const std::string &text,
                                  const Vec3f &position,
                                  const Color &color,
                                  const double size,
                                  const int id) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = ns;
            marker.id = id;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = size;
            marker.color = color;
            marker.text = text;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z();
            mkr_arr.markers.push_back(marker);
        }

        static void addPathToMarkerArray(visualization_msgs::MarkerArray &mkr_arr,
                                         const vec_E<Vec3f> &path,
                                         Color color,
                                         const std::string &ns,
                                         const double pt_size,
                                         const double line_size) {
            if (path.empty()) {
                return;
            }
            for (size_t i = 0; i < path.size(); ++i) {
                visualizePoint(mkr_arr, path[i], color, ns + "_point", pt_size, static_cast<int>(i));
                if (i == 0) {
                    continue;
                }
                visualization_msgs::Marker line;
                line.header.frame_id = "world";
                line.header.stamp = ros::Time::now();
                line.ns = ns + "_line";
                line.id = static_cast<int>(i);
                line.action = visualization_msgs::Marker::ADD;
                line.pose.orientation.w = 1.0;
                line.type = visualization_msgs::Marker::LINE_LIST;
                line.scale.x = line_size;
                line.color = color;
                geometry_msgs::Point p0;
                p0.x = path[i - 1].x();
                p0.y = path[i - 1].y();
                p0.z = path[i - 1].z();
                geometry_msgs::Point p1;
                p1.x = path[i].x();
                p1.y = path[i].y();
                p1.z = path[i].z();
                line.points.push_back(p0);
                line.points.push_back(p1);
                mkr_arr.markers.push_back(line);
            }
        }
    };
}
