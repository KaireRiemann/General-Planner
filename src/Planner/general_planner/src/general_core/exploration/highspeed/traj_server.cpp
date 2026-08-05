#include <data_structure/base/trajectory.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <traj_utils/PolyTraj.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace
{
ros::Publisher pos_cmd_pub;
ros::Publisher cmd_vis_pub;
ros::Publisher traj_pub;
std::shared_ptr<geometry_utils::Trajectory> traj;
std::shared_ptr<geometry_utils::Trajectory> yaw_traj;
quadrotor_msgs::PositionCommand cmd;
std::vector<Eigen::Vector3d> traj_cmd;
std::vector<Eigen::Vector3d> traj_real;
Eigen::Vector3d last_pos = Eigen::Vector3d::Zero();
double replan_time = 0.1;
double heartbeat_timeout = 1.5;
double traj_duration = 0.0;
double yaw_traj_duration = 0.0;
double last_yaw = 0.0;
ros::Time start_time;
ros::Time heartbeat_time(0);
int traj_id = 0;
bool receive_traj = false;
bool position_hold_active = false;
bool yaw_hold_active = false;
bool execution_enabled = true;
Eigen::Vector3d hold_pos = Eigen::Vector3d::Zero();
double hold_yaw = 0.0;

std::pair<double, double> getYaw(double current_yaw, double t_cur)
{
  if (!yaw_traj || yaw_traj->getPieceNum() <= 0)
  {
    return {current_yaw, 0.0};
  }
  t_cur = std::min(t_cur, yaw_traj->getTotalDuration());
  const Eigen::Vector3d yaw_pos = yaw_traj->getPos(t_cur);
  const Eigen::Vector3d yaw_vel = yaw_traj->getVel(t_cur);
  double next_yaw = yaw_pos.x();
  double d_yaw = next_yaw - current_yaw;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2.0 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2.0 * M_PI;
  }
  next_yaw = current_yaw + d_yaw;
  return {next_yaw, yaw_vel.x()};
}

void heartbeatCallback(const std_msgs::EmptyConstPtr &)
{
  heartbeat_time = ros::Time::now();
}

void executionEnabledCallback(const std_msgs::BoolConstPtr &msg)
{
  if (!msg)
  {
    return;
  }
  execution_enabled = msg->data;
  if (!execution_enabled)
  {
    ROS_INFO("[highspeed_traj_server] exploration command output disabled");
  }
}

void drawCmd(const Eigen::Vector3d &pos,
             const Eigen::Vector3d &vec,
             const int id,
             const Eigen::Vector4d &color)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.2;
  marker.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos.x();
  pt.y = pos.y();
  pt.z = pos.z();
  marker.points.push_back(pt);
  pt.x = pos.x() + vec.x();
  pt.y = pos.y() + vec.y();
  pt.z = pos.z() + vec.z();
  marker.points.push_back(pt);

  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();
  marker.color.a = color.w();
  cmd_vis_pub.publish(marker);
}

void polyTrajCallback(const traj_utils::PolyTrajConstPtr &msg)
{
  if (msg->order != 7)
  {
    ROS_ERROR("[highspeed_traj_server] Only support trajectory order 7.");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size() ||
      msg->coef_x.size() != msg->coef_y.size() ||
      msg->coef_x.size() != msg->coef_z.size())
  {
    ROS_ERROR("[highspeed_traj_server] Invalid position trajectory coefficients.");
    return;
  }

  const int piece_num = static_cast<int>(msg->duration.size());
  auto parsed_traj = std::make_shared<geometry_utils::Trajectory>();
  for (int i = 0; i < piece_num; ++i)
  {
    const int base = i * 8;
    Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 8);
    for (int j = 0; j < 8; ++j)
    {
      coeff(0, j) = msg->coef_x[base + j];
      coeff(1, j) = msg->coef_y[base + j];
      coeff(2, j) = msg->coef_z[base + j];
    }
    parsed_traj->emplace_back(std::max(1.0e-4, static_cast<double>(msg->duration[i])), coeff);
  }

  traj = parsed_traj;
  start_time = msg->start_time;
  traj_duration = traj->getTotalDuration();
  traj_id = msg->traj_id;
  last_pos = traj->getPos(0.0);
  hold_pos = last_pos;
  position_hold_active = false;
  receive_traj = true;
}

void polyYawTrajCallback(const traj_utils::PolyTrajConstPtr &msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[highspeed_traj_server] Only support yaw trajectory order 5.");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size() ||
      msg->coef_x.size() != msg->coef_y.size() ||
      msg->coef_x.size() != msg->coef_z.size())
  {
    ROS_ERROR("[highspeed_traj_server] Invalid yaw trajectory coefficients.");
    return;
  }

  const int piece_num = static_cast<int>(msg->duration.size());
  auto parsed_traj = std::make_shared<geometry_utils::Trajectory>();
  for (int i = 0; i < piece_num; ++i)
  {
    const int base = i * 6;
    Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(3, 6);
    for (int j = 0; j < 6; ++j)
    {
      coeff(0, j) = msg->coef_x[base + j];
      coeff(1, j) = msg->coef_y[base + j];
      coeff(2, j) = msg->coef_z[base + j];
    }
    parsed_traj->emplace_back(std::max(1.0e-4, static_cast<double>(msg->duration[i])), coeff);
  }
  yaw_traj = parsed_traj;
  yaw_traj_duration = yaw_traj->getTotalDuration();
  yaw_hold_active = false;
}

void publishCmd(const Eigen::Vector3d &pos,
                const Eigen::Vector3d &vel,
                const Eigen::Vector3d &acc,
                const Eigen::Vector3d &jerk,
                const double yaw,
                const double yaw_dot)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;
  cmd.position.x = pos.x();
  cmd.position.y = pos.y();
  cmd.position.z = pos.z();
  cmd.velocity.x = vel.x();
  cmd.velocity.y = vel.y();
  cmd.velocity.z = vel.z();
  cmd.acceleration.x = acc.x();
  cmd.acceleration.y = acc.y();
  cmd.acceleration.z = acc.z();
  cmd.jerk.x = jerk.x();
  cmd.jerk.y = jerk.y();
  cmd.jerk.z = jerk.z();
  cmd.yaw = yaw;
  cmd.yaw_dot = yaw_dot;
  cmd.vel_norm = vel.norm();
  cmd.acc_norm = acc.norm();
  pos_cmd_pub.publish(cmd);
}

void cmdTimerCallback(const ros::TimerEvent &)
{
  if (!execution_enabled || heartbeat_time.toSec() <= 1.0e-5 ||
      !receive_traj || !traj)
  {
    return;
  }
  const ros::Time now = ros::Time::now();
  if ((now - heartbeat_time).toSec() > heartbeat_timeout)
  {
    ROS_ERROR_THROTTLE(1.0, "[highspeed_traj_server] Lost planner heartbeat.");
  }

  const double t_cur = (now - start_time).toSec();
  Eigen::Vector3d pos = last_pos;
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  if (t_cur >= 0.0 && t_cur < traj_duration)
  {
    position_hold_active = false;
    pos = traj->getPos(t_cur);
    vel = traj->getVel(t_cur);
    acc = traj->getAcc(t_cur);
    jerk = traj->getJer(t_cur);
  }
  else if (t_cur >= traj_duration)
  {
    if (!position_hold_active)
    {
      // Sample the mathematical endpoint once.  Reusing last_pos here leaves
      // the command one timer tick short of the endpoint and makes the hold
      // location dependent on callback jitter.
      hold_pos = traj->getPos(traj_duration);
      const double terminal_speed = traj->getVel(traj_duration).norm();
      const double terminal_acc = traj->getAcc(traj_duration).norm();
      position_hold_active = true;
      ROS_WARN_THROTTLE(
          1.0,
          "[highspeed_traj_server] trajectory %d exhausted at %.3f/%.3f s; "
          "entering endpoint HOLD (terminal |v|=%.3f m/s, |a|=%.3f m/s^2).",
          traj_id, t_cur, traj_duration, terminal_speed, terminal_acc);
    }
    pos = hold_pos;
  }

  std::pair<double, double> yaw(last_yaw, 0.0);
  if (t_cur >= 0.0 && t_cur < yaw_traj_duration)
  {
    yaw_hold_active = false;
    yaw = getYaw(last_yaw, t_cur);
  }
  else if (t_cur >= yaw_traj_duration && yaw_traj && yaw_traj->getPieceNum() > 0)
  {
    if (!yaw_hold_active)
    {
      // Preserve angle unwrapping relative to the last command while taking
      // the exact yaw endpoint, then hold it with zero yaw rate.
      hold_yaw = getYaw(last_yaw, yaw_traj_duration).first;
      yaw_hold_active = true;
    }
    yaw = std::make_pair(hold_yaw, 0.0);
  }
  last_yaw = yaw.first;
  last_pos = pos;
  publishCmd(pos, vel, acc, jerk, yaw.first, yaw.second);

  if (traj_cmd.empty() || (traj_cmd.back() - pos).norm() > 0.02)
  {
    traj_cmd.emplace_back(pos);
  }
  drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O")
  {
    return;
  }
  const Eigen::Vector3d pos(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
  if (traj_real.empty() || (traj_real.back() - pos).norm() > 0.1)
  {
    traj_real.emplace_back(pos);
  }
  if (traj_real.size() > 100000)
  {
    traj_real.erase(traj_real.begin(), traj_real.begin() + 1000);
  }
}

void displayExecutedTrajectory(const std::vector<Eigen::Vector3d> &path)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.ns = "";
  marker.id = 0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.points.reserve(path.size());
  for (const auto &p : path)
  {
    geometry_msgs::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    marker.points.push_back(pt);
  }
  traj_pub.publish(marker);
}

void visTimerCallback(const ros::TimerEvent &)
{
  displayExecutedTrajectory(traj_cmd);
}

void replanCallback(const std_msgs::EmptyConstPtr &)
{
  constexpr double timeout = 0.1;
  const double stop_t = (ros::Time::now() - start_time).toSec() + replan_time + timeout;
  traj_duration = std::min(traj_duration, stop_t);
}

void newCallback(const std_msgs::EmptyConstPtr &)
{
  traj_cmd.clear();
  traj_real.clear();
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "highspeed_traj_server");
  ros::NodeHandle nh("~");

  std::string odom_topic;
  std::string pos_cmd_topic;
  std::string execution_enabled_topic;
  nh.param<std::string>("odometry_topic", odom_topic, "/lidar_slam/odom");
  nh.param<std::string>("pos_cmd_topic", pos_cmd_topic, "/planning/pos_cmd");
  nh.param<std::string>("execution_enabled_topic", execution_enabled_topic,
                        "/planning/exploration/command_enabled");
  nh.param("replan_time", replan_time, 0.1);
  nh.param("heartbeat_timeout", heartbeat_timeout, 1.5);
  heartbeat_timeout = std::max(0.5, heartbeat_timeout);

  ros::Subscriber poly_traj_sub = nh.subscribe("/planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber poly_yaw_traj_sub = nh.subscribe("/planning/yaw_trajectory", 10, polyYawTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("/planning/heartbeat", 10, heartbeatCallback);
  ros::Subscriber execution_enabled_sub;
  if (!execution_enabled_topic.empty())
  {
    execution_enabled_sub = nh.subscribe(execution_enabled_topic, 1,
                                         executionEnabledCallback);
  }
  ros::Subscriber odom_sub = nh.subscribe(odom_topic, 50, odomCallback);
  ros::Subscriber replan_sub = nh.subscribe("/planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = nh.subscribe("/planning/new", 10, newCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>(pos_cmd_topic, 50);
  cmd_vis_pub = nh.advertise<visualization_msgs::Marker>("/planning/position_cmd_vis", 10);
  traj_pub = nh.advertise<visualization_msgs::Marker>("/planning/travel_traj", 10);

  ros::Timer vis_timer = nh.createTimer(ros::Duration(0.25), visTimerCallback);
  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdTimerCallback);

  ROS_INFO("[highspeed_traj_server] ready. odom=%s pos_cmd=%s enabled_topic=%s",
           odom_topic.c_str(), pos_cmd_topic.c_str(),
           execution_enabled_topic.c_str());
  ros::spin();
  return 0;
}
