#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "path_search/astar.h"
#include "rog_map/rog_map.h"
#include "rog_map_ros/rog_map_ros1.hpp"
#include "ros_interface/ros1/ros1_interface.hpp"
#include "general_core/map_manager.hpp"
#include "utils/header/color_msg_utils.hpp"

#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/" + name))

namespace
{

using super_utils::Vec3f;

geometry_msgs::Quaternion quatFromYaw(const double yaw)
{
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    geometry_msgs::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}

geometry_msgs::Quaternion quatFromRpy(const double roll, const double pitch, const double yaw)
{
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

std::vector<double> allocatePathTime(const super_utils::vec_E<Vec3f> &path, double speed)
{
    std::vector<double> times(path.size(), 0.0);
    speed = std::max(0.2, speed);
    for (int i = 1; i < static_cast<int>(path.size()); ++i)
    {
        times[static_cast<std::size_t>(i)] =
            times[static_cast<std::size_t>(i - 1)] +
            std::max(0.05, (path[i] - path[i - 1]).norm() / speed);
    }
    return times;
}

std::string normalizedMode(std::string mode)
{
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return mode;
}

super_utils::vec_E<Vec3f> makeRuntimePath(const super_utils::vec_E<Vec3f> &path,
                                          const bool loop,
                                          const std::string &loop_mode)
{
    if (!loop || normalizedMode(loop_mode) == "teleport" || path.size() < 2)
    {
        return path;
    }

    super_utils::vec_E<Vec3f> runtime_path = path;
    for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i)
    {
        if ((runtime_path.back() - path[static_cast<std::size_t>(i)]).norm() > 1.0e-6)
        {
            runtime_path.emplace_back(path[static_cast<std::size_t>(i)]);
        }
    }
    return runtime_path;
}

double normalizePathTime(const double t,
                         const double total_t,
                         const bool loop)
{
    if (total_t <= 1.0e-6)
    {
        return 0.0;
    }
    if (!loop)
    {
        return std::clamp(t, 0.0, total_t);
    }
    double wrapped = std::fmod(t, total_t);
    if (wrapped < 0.0)
    {
        wrapped += total_t;
    }
    return wrapped;
}

Vec3f interpolatePath(const super_utils::vec_E<Vec3f> &path,
                      const std::vector<double> &times,
                      const double t,
                      Vec3f &velocity)
{
    velocity.setZero();
    if (path.empty())
    {
        return Vec3f::Zero();
    }
    if (path.size() == 1 || t <= times.front())
    {
        return path.front();
    }
    if (t >= times.back())
    {
        return path.back();
    }

    const auto upper = std::lower_bound(times.begin(), times.end(), t);
    const int idx = static_cast<int>(std::distance(times.begin(), upper));
    const double t0 = times[static_cast<std::size_t>(idx - 1)];
    const double t1 = times[static_cast<std::size_t>(idx)];
    const double alpha = (t - t0) / std::max(1.0e-6, t1 - t0);
    velocity = (path[static_cast<std::size_t>(idx)] - path[static_cast<std::size_t>(idx - 1)]) /
               std::max(1.0e-6, t1 - t0);
    return path[static_cast<std::size_t>(idx - 1)] +
           alpha * (path[static_cast<std::size_t>(idx)] - path[static_cast<std::size_t>(idx - 1)]);
}

geometry_msgs::PoseStamped makePose(const std::string &frame_id,
                                    const ros::Time &stamp,
                                    const Vec3f &position,
                                    const Vec3f &velocity)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();
    const double yaw = velocity.head<2>().norm() > 1.0e-3 ? std::atan2(velocity.y(), velocity.x()) : 0.0;
    pose.pose.orientation = quatFromYaw(yaw);
    return pose;
}

nav_msgs::Path buildPathMsg(const super_utils::vec_E<Vec3f> &path,
                            const std::vector<double> &times,
                            const std::string &frame_id)
{
    nav_msgs::Path msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    msg.poses.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        Vec3f velocity = Vec3f::Zero();
        if (i + 1 < path.size())
        {
            const double dt = std::max(1.0e-6, times[i + 1] - times[i]);
            velocity = (path[i + 1] - path[i]) / dt;
        }
        else if (i > 0)
        {
            const double dt = std::max(1.0e-6, times[i] - times[i - 1]);
            velocity = (path[i] - path[i - 1]) / dt;
        }
        msg.poses.emplace_back(makePose(frame_id, msg.header.stamp, path[i], velocity));
    }
    return msg;
}

nav_msgs::Path buildPredictionMsg(const super_utils::vec_E<Vec3f> &path,
                                  const std::vector<double> &times,
                                  const double current_t,
                                  const double total_t,
                                  const bool loop,
                                  const double horizon,
                                  const double dt,
                                  const std::string &frame_id)
{
    nav_msgs::Path msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    const int sample_num = std::max(2, static_cast<int>(std::ceil(std::max(dt, horizon) / dt)) + 1);
    msg.poses.reserve(static_cast<std::size_t>(sample_num));
    for (int i = 0; i < sample_num; ++i)
    {
        const double future_t = normalizePathTime(current_t + static_cast<double>(i) * dt, total_t, loop);
        Vec3f velocity = Vec3f::Zero();
        const Vec3f position = interpolatePath(path, times, future_t, velocity);
        msg.poses.emplace_back(makePose(frame_id,
                                        msg.header.stamp + ros::Duration(static_cast<double>(i) * dt),
                                        position,
                                        velocity));
    }
    return msg;
}

super_utils::vec_E<Vec3f> buildTrackingPath(const path_search::Astar::Ptr &astar,
                                            const Vec3f &start,
                                            const Vec3f &goal,
                                            const double horizon,
                                            const std::string &planner)
{
    super_utils::vec_E<Vec3f> path;
    auto tryAstar = [&](const int flag, const std::string &label) {
        super_utils::vec_E<Vec3f> astar_path;
        const auto ret = astar->pointToPointPathSearch(start, goal, flag, horizon, astar_path, 0.2);
        if ((ret == super_utils::SUCCESS || ret == super_utils::REACH_GOAL) && astar_path.size() >= 2)
        {
            path = astar_path;
            ROS_INFO("Task target source %s A* guide: %zu waypoints.", label.c_str(), path.size());
            return true;
        }
        ROS_WARN("Task target source %s A* failed with ret=%d.", label.c_str(), ret);
        return false;
    };

    const int prob_flag = path_search::ON_PROB_MAP |
                          path_search::UNKNOWN_AS_FREE |
                          path_search::DONT_USE_INF_NEIGHBOR;
    const int inf_flag = path_search::ON_INF_MAP |
                         path_search::UNKNOWN_AS_FREE |
                         path_search::USE_INF_NEIGHBOR;
    if (planner == "inflated_astar")
    {
        tryAstar(inf_flag, "inflated-map") || tryAstar(prob_flag, "prob-map");
    }
    else
    {
        tryAstar(prob_flag, "prob-map") || tryAstar(inf_flag, "inflated-map");
    }

    if (path.size() < 2)
    {
        path.clear();
        path.emplace_back(start);
        path.emplace_back(Vec3f(-1.6, 0.25, start.z()));
        path.emplace_back(Vec3f(0.55, 1.95, start.z()));
        path.emplace_back(Vec3f(2.4, 0.25, start.z()));
        path.emplace_back(goal);
        ROS_WARN("Task target source uses deterministic fallback guide.");
    }
    return path;
}

void fillOdom(nav_msgs::Odometry &odom,
              const std::string &frame_id,
              const std::string &child_frame_id,
              const Vec3f &p,
              const Vec3f &v,
              const geometry_msgs::Quaternion &q)
{
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation = q;
    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_target_source");
    ros::NodeHandle nh("~");

    std::string cfg_name;
    std::string cfg_path;
    std::string mode;
    nh.param<std::string>("config_name", cfg_name, "tracking_perching_tracking_ros1.yaml");
    nh.param<std::string>("config_path", cfg_path, CONFIG_FILE_DIR(cfg_name));
    nh.param<std::string>("mode", mode, "tracking");

    auto ros_ptr = std::make_shared<ros_interface::Ros1Interface>(nh);
    ros_ptr->setVisualizationEn(true);
    ros_ptr->setResolution(0.1);
    auto rog_map = std::make_shared<rog_map::ROGMapROS>(nh, cfg_path);
    auto map_manager = std::make_shared<general_planner::MapManager>(rog_map);
    auto astar = std::make_shared<path_search::Astar>(cfg_path, ros_ptr, map_manager);

    double rate_hz = 30.0;
    nh.param<double>("rate", rate_hz, 30.0);
    ros::Rate rate(std::max(1.0, rate_hz));

    if (mode == "tracking")
    {
        std::string odom_topic;
        std::string path_topic;
        std::string prediction_topic;
        std::string target_planner;
        std::string loop_mode;
        bool loop = true;
        double target_speed = 1.2;
        double search_horizon = 10.0;
        double prediction_horizon = 4.0;
        double prediction_dt = 0.25;
        Vec3f target_start(-2.2, -2.3, 1.25);
        Vec3f target_goal(4.2, 2.15, 1.25);
        nh.param<std::string>("target_odom_topic", odom_topic, "/tracking/target_odom");
        nh.param<std::string>("target_path_topic", path_topic, "/tracking/target_path");
        nh.param<std::string>("target_prediction_topic", prediction_topic, "/tracking/target_prediction");
        nh.param<std::string>("target_planner", target_planner, "astar");
        nh.param<std::string>("loop_mode", loop_mode, "pingpong");
        nh.param<bool>("loop", loop, true);
        nh.param<double>("target_speed", target_speed, 1.2);
        nh.param<double>("search_horizon", search_horizon, 10.0);
        nh.param<double>("prediction_horizon", prediction_horizon, 4.0);
        nh.param<double>("prediction_dt", prediction_dt, 0.25);
        nh.param<double>("target_start_x", target_start.x(), target_start.x());
        nh.param<double>("target_start_y", target_start.y(), target_start.y());
        nh.param<double>("target_start_z", target_start.z(), target_start.z());
        nh.param<double>("target_goal_x", target_goal.x(), target_goal.x());
        nh.param<double>("target_goal_y", target_goal.y(), target_goal.y());
        nh.param<double>("target_goal_z", target_goal.z(), target_goal.z());

        const auto astar_path = buildTrackingPath(astar, target_start, target_goal, search_horizon, target_planner);
        const auto path = makeRuntimePath(astar_path, loop, loop_mode);
        const auto times = allocatePathTime(path, target_speed);
        const double total_t = std::max(0.1, times.back());
        const ros::Time start_time = ros::Time::now();
        ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
        ros::Publisher path_pub = nh.advertise<nav_msgs::Path>(path_topic, 1, true);
        ros::Publisher prediction_pub = nh.advertise<nav_msgs::Path>(prediction_topic, 10);

        const auto path_msg = buildPathMsg(path, times, "world");
        path_pub.publish(path_msg);
        ROS_INFO("Tracking target source publishes odom %s and prediction %s, path waypoints %zu, duration %.3f s.",
                 odom_topic.c_str(),
                 prediction_topic.c_str(),
                 path.size(),
                 total_t);
        while (ros::ok())
        {
            const double t = normalizePathTime((ros::Time::now() - start_time).toSec(), total_t, loop);

            Vec3f v;
            const Vec3f p = interpolatePath(path, times, t, v);
            const double yaw = v.head<2>().norm() > 1.0e-3 ? std::atan2(v.y(), v.x()) : 0.0;
            nav_msgs::Odometry odom;
            fillOdom(odom, "world", "tracking_target", p, v, quatFromYaw(yaw));
            odom_pub.publish(odom);
            prediction_pub.publish(buildPredictionMsg(path,
                                                      times,
                                                      t,
                                                      total_t,
                                                      loop,
                                                      prediction_horizon,
                                                      std::max(0.05, prediction_dt),
                                                      "world"));
            path_pub.publish(path_msg);
            ros_ptr->vizFrontendPath(path);
            ros::spinOnce();
            rate.sleep();
        }
        return 0;
    }

    std::string surface_topic;
    Vec3f p0(3.6, 0.4, 0.95);
    Vec3f v(0.12, 0.0, 0.0);
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double oscillation_amp = 0.0;
    double oscillation_freq = 0.4;
    nh.param<std::string>("surface_odom_topic", surface_topic, "/perching/surface_odom");
    nh.param<double>("surface_x", p0.x(), p0.x());
    nh.param<double>("surface_y", p0.y(), p0.y());
    nh.param<double>("surface_z", p0.z(), p0.z());
    nh.param<double>("surface_vx", v.x(), v.x());
    nh.param<double>("surface_vy", v.y(), v.y());
    nh.param<double>("surface_vz", v.z(), v.z());
    nh.param<double>("surface_yaw", yaw, yaw);
    nh.param<double>("surface_pitch", pitch, pitch);
    nh.param<double>("surface_roll", roll, roll);
    nh.param<double>("oscillation_amp", oscillation_amp, oscillation_amp);
    nh.param<double>("oscillation_freq", oscillation_freq, oscillation_freq);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(surface_topic, 10);
    const ros::Time start_time = ros::Time::now();
    ROS_INFO("Perching surface source publishes %s.", surface_topic.c_str());
    while (ros::ok())
    {
        const double t = (ros::Time::now() - start_time).toSec();
        Vec3f p = p0 + v * t;
        p.y() += oscillation_amp * std::sin(oscillation_freq * t);
        Vec3f vel = v;
        vel.y() += oscillation_amp * oscillation_freq * std::cos(oscillation_freq * t);

        nav_msgs::Odometry odom;
        fillOdom(odom, "world", "perching_surface", p, vel, quatFromRpy(roll, pitch, yaw));
        odom_pub.publish(odom);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
