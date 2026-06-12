#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

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

std_msgs::ColorRGBA colorRgba(const double r, const double g, const double b, const double a)
{
    std_msgs::ColorRGBA color;
    color.r = static_cast<float>(r);
    color.g = static_cast<float>(g);
    color.b = static_cast<float>(b);
    color.a = static_cast<float>(a);
    return color;
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

bool isTrackingPerchingMode(const std::string &mode)
{
    const std::string normalized = normalizedMode(mode);
    return normalized == "tracking_perching" ||
           normalized == "tracking-perching" ||
           normalized == "trackingperching" ||
           normalized == "joint";
}

bool debugInfoIndicatesPerchingTrajectory(const std::string &debug_info)
{
    return debug_info.find("task_phase=perching") != std::string::npos ||
           debug_info.find("task_phase=tracking_perching_perching") != std::string::npos;
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
              const geometry_msgs::Quaternion &q,
              const Vec3f &omega = Vec3f::Zero())
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
    odom.twist.twist.angular.x = omega.x();
    odom.twist.twist.angular.y = omega.y();
    odom.twist.twist.angular.z = omega.z();
}

geometry_msgs::Point pointMsg(const Vec3f &p)
{
    geometry_msgs::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    return point;
}

struct PerchingSurfaceSample
{
    Vec3f position{Vec3f::Zero()};
    Vec3f velocity{Vec3f::Zero()};
    Vec3f omega{Vec3f::Zero()};
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
};

quadrotor_msgs::PositionCommand makeTakeoffContactCommand(const PerchingSurfaceSample &sample,
                                                          const Eigen::Quaterniond &q,
                                                          const double robot_l,
                                                          const std::string &frame_id)
{
    const Eigen::Matrix3d R = q.toRotationMatrix();
    const Vec3f offset = std::max(0.0, robot_l) * R.col(2);
    const Vec3f position = sample.position + offset;
    const Vec3f velocity = sample.velocity + sample.omega.cross(offset);
    const Vec3f acceleration = 9.80 * R.col(2) - Vec3f(0.0, 0.0, 9.80);

    quadrotor_msgs::PositionCommand cmd;
    cmd.header.frame_id = frame_id;
    cmd.header.stamp = ros::Time::now();
    cmd.trajectory_flag = 1;
    cmd.position.x = position.x();
    cmd.position.y = position.y();
    cmd.position.z = position.z();
    cmd.velocity.x = velocity.x();
    cmd.velocity.y = velocity.y();
    cmd.velocity.z = velocity.z();
    cmd.acceleration.x = acceleration.x();
    cmd.acceleration.y = acceleration.y();
    cmd.acceleration.z = acceleration.z();
    cmd.jerk.x = 0.0;
    cmd.jerk.y = 0.0;
    cmd.jerk.z = 0.0;
    cmd.angular_velocity.x = sample.omega.x();
    cmd.angular_velocity.y = sample.omega.y();
    cmd.angular_velocity.z = sample.omega.z();
    cmd.attitude.x = sample.roll;
    cmd.attitude.y = sample.pitch;
    cmd.attitude.z = sample.yaw;
    cmd.yaw = sample.yaw;
    cmd.yaw_dot = sample.omega.z();
    return cmd;
}

visualization_msgs::Marker makeArrowMarker(const std::string &frame_id,
                                           const ros::Time &stamp,
                                           const std::string &ns,
                                           const int id,
                                           const Vec3f &start,
                                           const Vec3f &vec,
                                           const std_msgs::ColorRGBA &color,
                                           const double shaft_diameter,
                                           const double head_diameter)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.push_back(pointMsg(start));
    marker.points.push_back(pointMsg(start + vec));
    marker.scale.x = shaft_diameter;
    marker.scale.y = head_diameter;
    marker.scale.z = head_diameter * 1.4;
    marker.color = color;
    marker.lifetime = ros::Duration(0.2);
    return marker;
}

visualization_msgs::MarkerArray makeSurfaceMarkers(const Vec3f &p,
                                                   const Eigen::Matrix3d &R,
                                                   const geometry_msgs::Quaternion &q_msg,
                                                   const double platform_radius,
                                                   const double normal_length,
                                                   const std::string &frame_id)
{
    const ros::Time stamp = ros::Time::now();
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker disk;
    disk.header.frame_id = frame_id;
    disk.header.stamp = stamp;
    disk.ns = "perching_surface";
    disk.id = 0;
    disk.type = visualization_msgs::Marker::CYLINDER;
    disk.action = visualization_msgs::Marker::ADD;
    disk.pose.position = pointMsg(p);
    disk.pose.orientation = q_msg;
    disk.scale.x = std::max(0.05, 2.0 * platform_radius);
    disk.scale.y = std::max(0.05, 2.0 * platform_radius);
    disk.scale.z = 0.035;
    disk.color = colorRgba(0.1, 0.6, 1.0, 0.42);
    disk.lifetime = ros::Duration(0.2);
    markers.markers.push_back(disk);

    const Vec3f x_axis = R.col(0) * (platform_radius * 0.95);
    const Vec3f y_axis = R.col(1) * (platform_radius * 0.95);
    const Vec3f z_axis = R.col(2) * normal_length;
    markers.markers.push_back(makeArrowMarker(frame_id, stamp, "perching_surface", 1, p, x_axis,
                                              colorRgba(1.0, 0.15, 0.1, 0.95), 0.025, 0.06));
    markers.markers.push_back(makeArrowMarker(frame_id, stamp, "perching_surface", 2, p, y_axis,
                                              colorRgba(0.1, 0.85, 0.2, 0.95), 0.025, 0.06));
    markers.markers.push_back(makeArrowMarker(frame_id, stamp, "perching_surface", 3, p, z_axis,
                                              colorRgba(1.0, 0.9, 0.05, 1.0), 0.04, 0.095));

    visualization_msgs::Marker text;
    text.header.frame_id = frame_id;
    text.header.stamp = stamp;
    text.ns = "perching_surface";
    text.id = 4;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position = pointMsg(p + z_axis * 1.08);
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.16;
    text.color = colorRgba(1.0, 1.0, 1.0, 0.9);
    text.text = "perching surface";
    text.lifetime = ros::Duration(0.2);
    markers.markers.push_back(text);

    return markers;
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
    mode = normalizedMode(mode);
    const bool tracking_perching_mode = isTrackingPerchingMode(mode);

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
    std::string surface_marker_topic;
    std::string tracking_target_odom_topic;
    std::string tracking_target_path_topic;
    std::string tracking_target_prediction_topic;
    std::string target_loop_mode;
    Vec3f p0(3.6, 0.4, 0.95);
    Vec3f v(0.12, 0.0, 0.0);
    Vec3f target_start(3.0, 0.0, 1.0);
    Vec3f target_goal(15.0, 0.0, 1.0);
    bool target_loop = false;
    double target_speed = 1.2;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double yaw_rate = 0.0;
    double pitch_rate = 0.0;
    double roll_rate = 0.0;
    double oscillation_amp = 0.0;
    double oscillation_freq = 0.4;
    double yaw_oscillation_amp = 0.0;
    double yaw_oscillation_freq = 0.4;
    double pitch_oscillation_amp = 0.0;
    double pitch_oscillation_freq = 0.4;
    double roll_oscillation_amp = 0.0;
    double roll_oscillation_freq = 0.4;
    double platform_radius = 0.35;
    double normal_length = 0.9;
    bool publish_tracking_target = tracking_perching_mode;
    double tracking_prediction_horizon = 4.0;
    double tracking_prediction_dt = 0.25;
    double tracking_target_normal_offset = 0.0;
    bool publish_takeoff_contact_cmd = false;
    std::string takeoff_contact_cmd_topic{"/planning/pos_cmd"};
    double takeoff_contact_robot_l = 0.28;
    double takeoff_contact_cmd_duration = 0.0;
    std::string surface_traj_topic;
    bool stop_surface_on_traj_arrival = false;
    bool stop_surface_require_perching_trajectory = false;
    double surface_stop_margin = 0.05;
    nh.param<std::string>("surface_odom_topic", surface_topic, "/perching/surface_odom");
    nh.param<std::string>("surface_marker_topic", surface_marker_topic, "/perching/surface_markers");
    nh.param<std::string>("tracking_target_odom_topic", tracking_target_odom_topic, "/tracking/target_odom");
    nh.param<std::string>("tracking_target_path_topic", tracking_target_path_topic, "/tracking/target_path");
    nh.param<std::string>("tracking_target_prediction_topic", tracking_target_prediction_topic, "/tracking/target_prediction");
    nh.param<std::string>("surface_traj_topic", surface_traj_topic, "/planning_cmd/poly_traj");
    nh.param<std::string>("target_loop_mode", target_loop_mode, "teleport");
    nh.param<bool>("target_loop", target_loop, target_loop);
    nh.param<double>("target_speed", target_speed, target_speed);
    nh.param<double>("target_start_x", target_start.x(), target_start.x());
    nh.param<double>("target_start_y", target_start.y(), target_start.y());
    nh.param<double>("target_start_z", target_start.z(), target_start.z());
    nh.param<double>("target_goal_x", target_goal.x(), target_goal.x());
    nh.param<double>("target_goal_y", target_goal.y(), target_goal.y());
    nh.param<double>("target_goal_z", target_goal.z(), target_goal.z());
    nh.param<double>("surface_x", p0.x(), p0.x());
    nh.param<double>("surface_y", p0.y(), p0.y());
    nh.param<double>("surface_z", p0.z(), p0.z());
    nh.param<double>("surface_vx", v.x(), v.x());
    nh.param<double>("surface_vy", v.y(), v.y());
    nh.param<double>("surface_vz", v.z(), v.z());
    nh.param<double>("surface_yaw", yaw, yaw);
    nh.param<double>("surface_pitch", pitch, pitch);
    nh.param<double>("surface_roll", roll, roll);
    nh.param<double>("surface_yaw_rate", yaw_rate, yaw_rate);
    nh.param<double>("surface_pitch_rate", pitch_rate, pitch_rate);
    nh.param<double>("surface_roll_rate", roll_rate, roll_rate);
    nh.param<double>("oscillation_amp", oscillation_amp, oscillation_amp);
    nh.param<double>("oscillation_freq", oscillation_freq, oscillation_freq);
    nh.param<double>("yaw_oscillation_amp", yaw_oscillation_amp, yaw_oscillation_amp);
    nh.param<double>("yaw_oscillation_freq", yaw_oscillation_freq, yaw_oscillation_freq);
    nh.param<double>("pitch_oscillation_amp", pitch_oscillation_amp, pitch_oscillation_amp);
    nh.param<double>("pitch_oscillation_freq", pitch_oscillation_freq, pitch_oscillation_freq);
    nh.param<double>("roll_oscillation_amp", roll_oscillation_amp, roll_oscillation_amp);
    nh.param<double>("roll_oscillation_freq", roll_oscillation_freq, roll_oscillation_freq);
    nh.param<double>("platform_radius", platform_radius, platform_radius);
    nh.param<double>("normal_length", normal_length, normal_length);
    nh.param<bool>("publish_tracking_target", publish_tracking_target, publish_tracking_target);
    publish_tracking_target = publish_tracking_target || tracking_perching_mode;
    nh.param<double>("tracking_prediction_horizon", tracking_prediction_horizon, tracking_prediction_horizon);
    nh.param<double>("tracking_prediction_dt", tracking_prediction_dt, tracking_prediction_dt);
    nh.param<double>("tracking_target_normal_offset", tracking_target_normal_offset, tracking_target_normal_offset);
    nh.param<bool>("publish_takeoff_contact_cmd", publish_takeoff_contact_cmd, publish_takeoff_contact_cmd);
    nh.param<std::string>("takeoff_contact_cmd_topic", takeoff_contact_cmd_topic, takeoff_contact_cmd_topic);
    nh.param<double>("takeoff_contact_robot_l", takeoff_contact_robot_l, takeoff_contact_robot_l);
    nh.param<double>("takeoff_contact_cmd_duration", takeoff_contact_cmd_duration, takeoff_contact_cmd_duration);
    nh.param<bool>("stop_surface_on_traj_arrival",
                   stop_surface_on_traj_arrival,
                   stop_surface_on_traj_arrival);
    nh.param<bool>("stop_surface_require_perching_trajectory",
                   stop_surface_require_perching_trajectory,
                   stop_surface_require_perching_trajectory);
    nh.param<double>("surface_stop_margin", surface_stop_margin, surface_stop_margin);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(surface_topic, 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>(surface_marker_topic, 10);
    ros::Publisher tracking_odom_pub;
    ros::Publisher tracking_path_pub;
    ros::Publisher tracking_prediction_pub;
    ros::Publisher takeoff_contact_cmd_pub;
    if (publish_tracking_target)
    {
        tracking_odom_pub = nh.advertise<nav_msgs::Odometry>(tracking_target_odom_topic, 10);
        tracking_path_pub = nh.advertise<nav_msgs::Path>(tracking_target_path_topic, 10);
        tracking_prediction_pub = nh.advertise<nav_msgs::Path>(tracking_target_prediction_topic, 10);
    }
    if (publish_takeoff_contact_cmd)
    {
        takeoff_contact_cmd_pub =
            nh.advertise<quadrotor_msgs::PositionCommand>(takeoff_contact_cmd_topic, 10);
    }
    const ros::Time start_time = ros::Time::now();
    bool stop_scheduled = false;
    bool surface_frozen = false;
    uint32_t scheduled_traj_id = 0;
    ros::Time scheduled_stop_time;
    PerchingSurfaceSample frozen_surface;
    super_utils::vec_E<Vec3f> joint_path;
    std::vector<double> joint_times;
    double joint_total_t = 0.1;
    nav_msgs::Path joint_path_msg;

    if (tracking_perching_mode)
    {
        p0 = target_start;
        Vec3f displacement = target_goal - target_start;
        double travel_distance = displacement.norm();
        Vec3f direction = Vec3f::UnitX();
        if (travel_distance > 1.0e-6)
        {
            direction = displacement / travel_distance;
        }
        if (v.norm() > 1.0e-6)
        {
            direction = v.normalized();
            target_speed = v.norm();
        }
        target_speed = std::max(0.2, target_speed);
        travel_distance = std::max(travel_distance,
                                   std::max(6.0, target_speed * std::max(6.0, tracking_prediction_horizon)));
        target_goal = target_start + direction * travel_distance;
        super_utils::vec_E<Vec3f> base_path;
        base_path.emplace_back(target_start);
        base_path.emplace_back(target_goal);
        joint_path = makeRuntimePath(base_path, target_loop, target_loop_mode);
        joint_times = allocatePathTime(joint_path, target_speed);
        joint_total_t = std::max(0.1, joint_times.back());
        joint_path_msg = buildPathMsg(joint_path, joint_times, "world");
        ROS_INFO("Tracking-perching source uses one moving platform from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f), speed %.2f, duration %.2f s.",
                 target_start.x(),
                 target_start.y(),
                 target_start.z(),
                 target_goal.x(),
                 target_goal.y(),
                 target_goal.z(),
                 target_speed,
                 joint_total_t);
    }

    auto sampleSurface = [&](const double t) {
        PerchingSurfaceSample sample;
        if (tracking_perching_mode)
        {
            const double path_t = normalizePathTime(t,
                                                    joint_total_t,
                                                    target_loop);
            sample.position = interpolatePath(joint_path, joint_times, path_t, sample.velocity);
        }
        else
        {
            sample.position = p0 + v * t;
            sample.velocity = v;
        }
        sample.position.y() += oscillation_amp * std::sin(oscillation_freq * t);
        sample.velocity.y() += oscillation_amp * oscillation_freq * std::cos(oscillation_freq * t);
        sample.roll = roll + roll_rate * t +
                      roll_oscillation_amp * std::sin(roll_oscillation_freq * t);
        sample.pitch = pitch + pitch_rate * t +
                       pitch_oscillation_amp * std::sin(pitch_oscillation_freq * t);
        sample.yaw = yaw + yaw_rate * t +
                     yaw_oscillation_amp * std::sin(yaw_oscillation_freq * t);
        sample.omega = Vec3f(roll_rate + roll_oscillation_amp * roll_oscillation_freq *
                                             std::cos(roll_oscillation_freq * t),
                             pitch_rate + pitch_oscillation_amp * pitch_oscillation_freq *
                                              std::cos(pitch_oscillation_freq * t),
                             yaw_rate + yaw_oscillation_amp * yaw_oscillation_freq *
                                            std::cos(yaw_oscillation_freq * t));
        return sample;
    };

    auto surfaceQuaternion = [](const PerchingSurfaceSample &sample) {
        return Eigen::Quaterniond(Eigen::AngleAxisd(sample.yaw, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(sample.pitch, Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(sample.roll, Eigen::Vector3d::UnitX()));
    };

    auto trackingTargetYaw = [](const PerchingSurfaceSample &sample) {
        return sample.velocity.head<2>().norm() > 1.0e-3
                   ? std::atan2(sample.velocity.y(), sample.velocity.x())
                   : sample.yaw;
    };

    auto trackingTargetState = [&](const PerchingSurfaceSample &sample,
                                   Vec3f &target_p,
                                   Vec3f &target_v) {
        target_p = sample.position;
        target_v = sample.velocity;
        if (std::abs(tracking_target_normal_offset) > 1.0e-6)
        {
            const Vec3f offset =
                tracking_target_normal_offset * surfaceQuaternion(sample).toRotationMatrix().col(2);
            target_p += offset;
            target_v += sample.omega.cross(offset);
        }
    };

    auto buildSurfaceTrackingPrediction = [&](const double t0) {
        nav_msgs::Path msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();
        const double dt = std::max(0.05, tracking_prediction_dt);
        const double horizon = std::max(dt, tracking_prediction_horizon);
        const int sample_num = std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1);
        msg.poses.reserve(static_cast<std::size_t>(sample_num));
        for (int i = 0; i < sample_num; ++i)
        {
            const PerchingSurfaceSample future = surface_frozen
                                                     ? frozen_surface
                                                     : sampleSurface(t0 + static_cast<double>(i) * dt);
            Vec3f target_p;
            Vec3f target_v;
            trackingTargetState(future, target_p, target_v);
            auto pose = makePose("world",
                                 msg.header.stamp + ros::Duration(static_cast<double>(i) * dt),
                                 target_p,
                                 target_v);
            if (target_v.head<2>().norm() <= 1.0e-3)
            {
                pose.pose.orientation = quatFromYaw(trackingTargetYaw(future));
            }
            msg.poses.emplace_back(pose);
        }
        return msg;
    };

    ros::Subscriber traj_sub;
    if (stop_surface_on_traj_arrival)
    {
        traj_sub = nh.subscribe<quadrotor_msgs::PolynomialTrajectory>(
            surface_traj_topic,
            10,
            [&](const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg) {
                if (surface_frozen)
                {
                    return;
                }
                if ((msg->type & quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ) == 0 ||
                    msg->piece_num_pos <= 0)
                {
                    return;
                }
                if (stop_surface_require_perching_trajectory &&
                    !debugInfoIndicatesPerchingTrajectory(msg->debug_info))
                {
                    return;
                }
                const int piece_num = std::min<int>(msg->piece_num_pos,
                                                    static_cast<int>(msg->time_pos.size()));
                if (piece_num <= 0)
                {
                    return;
                }

                double duration = 0.0;
                for (int i = 0; i < piece_num; ++i)
                {
                    duration += std::max(0.0, msg->time_pos[static_cast<std::size_t>(i)]);
                }
                if (duration <= 1.0e-3)
                {
                    return;
                }

                ros::Time traj_start = msg->start_WT_pos;
                if (traj_start.isZero())
                {
                    traj_start = ros::Time::now();
                }
                const ros::Time next_stop_time =
                    traj_start + ros::Duration(duration + std::max(0.0, surface_stop_margin));
                if (stop_scheduled &&
                    std::abs((next_stop_time - scheduled_stop_time).toSec()) < 1.0e-3)
                {
                    return;
                }
                scheduled_stop_time = next_stop_time;
                stop_scheduled = true;
                scheduled_traj_id = msg->trajectory_id;
                ROS_INFO("Perching surface will stop at planned arrival. traj_id=%u, duration=%.3f, stop_in=%.3f.",
                         scheduled_traj_id,
                         duration,
                         (scheduled_stop_time - ros::Time::now()).toSec());
            });
    }
    ROS_INFO("Perching surface source publishes %s and markers %s. p0=(%.2f, %.2f, %.2f), v=(%.2f, %.2f, %.2f), rpy=(%.2f, %.2f, %.2f), yaw_rate=%.2f.",
             surface_topic.c_str(),
             surface_marker_topic.c_str(),
             p0.x(),
             p0.y(),
             p0.z(),
             v.x(),
             v.y(),
             v.z(),
             roll,
             pitch,
             yaw,
             yaw_rate);
    if (publish_tracking_target)
    {
        ROS_INFO("Perching surface source also publishes tracking target %s and prediction %s from the same moving platform.",
                 tracking_target_odom_topic.c_str(),
                 tracking_target_prediction_topic.c_str());
        if (tracking_perching_mode)
        {
            tracking_path_pub.publish(joint_path_msg);
        }
    }
    if (publish_takeoff_contact_cmd)
    {
        ROS_INFO("Perching surface source publishes takeoff contact commands on %s for %.2f s with robot_l=%.2f.",
                 takeoff_contact_cmd_topic.c_str(),
                 std::max(0.0, takeoff_contact_cmd_duration),
                 takeoff_contact_robot_l);
    }
    bool takeoff_contact_cmd_stop_logged = false;
    while (ros::ok())
    {
        const ros::Time now = ros::Time::now();
        if (stop_scheduled && !surface_frozen && now >= scheduled_stop_time)
        {
            const double stop_t = std::max(0.0, (scheduled_stop_time - start_time).toSec());
            frozen_surface = sampleSurface(stop_t);
            frozen_surface.velocity.setZero();
            frozen_surface.omega.setZero();
            surface_frozen = true;
            stop_scheduled = false;
            ROS_INFO("Perching surface stopped at planned arrival. traj_id=%u, p=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f).",
                     scheduled_traj_id,
                     frozen_surface.position.x(),
                     frozen_surface.position.y(),
                     frozen_surface.position.z(),
                     frozen_surface.roll,
                     frozen_surface.pitch,
                     frozen_surface.yaw);
        }

        const double t = std::max(0.0, (now - start_time).toSec());
        const PerchingSurfaceSample sample = surface_frozen ? frozen_surface : sampleSurface(t);
        const Eigen::Quaterniond q =
            Eigen::AngleAxisd(sample.yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(sample.pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(sample.roll, Eigen::Vector3d::UnitX());
        geometry_msgs::Quaternion q_msg;
        q_msg.w = q.w();
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();

        nav_msgs::Odometry odom;
        fillOdom(odom, "world", "perching_surface", sample.position, sample.velocity, q_msg, sample.omega);
        odom_pub.publish(odom);
        marker_pub.publish(makeSurfaceMarkers(sample.position,
                                               q.toRotationMatrix(),
                                               q_msg,
                                               std::max(0.05, platform_radius),
                                               std::max(0.1, normal_length),
                                               "world"));
        if (publish_takeoff_contact_cmd)
        {
            const bool command_active =
                takeoff_contact_cmd_duration <= 0.0 ||
                t <= std::max(0.0, takeoff_contact_cmd_duration);
            if (command_active)
            {
                takeoff_contact_cmd_pub.publish(makeTakeoffContactCommand(
                    sample,
                    q,
                    takeoff_contact_robot_l,
                    "world"));
            }
            else if (!takeoff_contact_cmd_stop_logged)
            {
                takeoff_contact_cmd_stop_logged = true;
                ROS_INFO("Takeoff contact command phase finished after %.2f s.",
                         takeoff_contact_cmd_duration);
            }
        }
        if (publish_tracking_target)
        {
            Vec3f target_p;
            Vec3f target_v;
            trackingTargetState(sample, target_p, target_v);
            nav_msgs::Odometry target_odom;
            fillOdom(target_odom,
                     "world",
                     "tracking_target",
                     target_p,
                     target_v,
                     quatFromYaw(trackingTargetYaw(sample)));
            tracking_odom_pub.publish(target_odom);
            const auto prediction = buildSurfaceTrackingPrediction(t);
            tracking_prediction_pub.publish(prediction);
            tracking_path_pub.publish(tracking_perching_mode ? joint_path_msg : prediction);
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
