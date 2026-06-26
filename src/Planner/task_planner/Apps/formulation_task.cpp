#include <algorithm>
#include <cmath>
#include <exception>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <ros/ros.h>

namespace
{
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Vec3 &operator+=(const Vec3 &other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  Vec3 operator+(const Vec3 &other) const
  {
    return {x + other.x, y + other.y, z + other.z};
  }

  Vec3 operator-(const Vec3 &other) const
  {
    return {x - other.x, y - other.y, z - other.z};
  }

  Vec3 operator*(double scale) const
  {
    return {x * scale, y * scale, z * scale};
  }

  Vec3 operator/(double value) const
  {
    return {x / value, y / value, z / value};
  }

  bool finite() const
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

Vec3 fromPoint(const geometry_msgs::Point &point)
{
  return {point.x, point.y, point.z};
}

double normXY(const Vec3 &value)
{
  return std::sqrt(value.x * value.x + value.y * value.y);
}

geometry_msgs::PoseStamped makePose(const Vec3 &position,
                                    const std::string &frame_id,
                                    const ros::Time &stamp)
{
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = position.x;
  pose.pose.position.y = position.y;
  pose.pose.position.z = position.z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool parseKeyValue(const std::string &debug_info,
                   const std::string &key,
                   std::string &value)
{
  const std::string needle = key + "=";
  const size_t begin = debug_info.find(needle);
  if (begin == std::string::npos)
  {
    return false;
  }
  const size_t value_begin = begin + needle.size();
  const size_t value_end = debug_info.find(';', value_begin);
  value = debug_info.substr(value_begin,
                            value_end == std::string::npos ? std::string::npos
                                                           : value_end - value_begin);
  return !value.empty();
}

bool parseDroneId(const std::string &debug_info, int &drone_id)
{
  std::string value;
  if (!parseKeyValue(debug_info, "drone_id", value))
  {
    return false;
  }
  try
  {
    drone_id = std::stoi(value);
  }
  catch (const std::exception &)
  {
    return false;
  }
  return drone_id >= 0;
}

class FormulationTask
{
public:
  explicit FormulationTask(const ros::NodeHandle &pnh) : pnh_(pnh)
  {
    pnh_.param("input_goal_topic", input_goal_topic_, std::string("/swarm/goal"));
    pnh_.param("formation_reference_topic", formation_reference_topic_,
               std::string("/swarm/formation_reference"));
    pnh_.param("traj_broadcast_topic", traj_broadcast_topic_, std::string("/swarm/trajectory"));
    pnh_.param("odom_timeout", odom_timeout_, 1.0);
    pnh_.param("publish_repeat", publish_repeat_, 3);
    pnh_.param("publish_period", publish_period_, 0.05);
    pnh_.param("use_clicked_z", use_clicked_z_, false);
    pnh_.param("goal_z", goal_z_, 1.5);
    pnh_.param("sequential_start_enable", sequential_start_enable_, true);
    pnh_.param("sequential_timeout", sequential_timeout_, 5.0);
    pnh_.param("min_segment_length", min_segment_length_, 0.2);

    goal_topics_ = {"/swarm0/goal", "/swarm1/goal"};
    odom_topics_ = {"/swarm0/lidar_slam/odom", "/swarm1/lidar_slam/odom"};
    pnh_.getParam("goal_topics", goal_topics_);
    pnh_.getParam("odom_topics", odom_topics_);

    std::vector<double> flat_offsets;
    pnh_.getParam("formation_offsets", flat_offsets);

    const size_t agent_num = std::min(goal_topics_.size(), odom_topics_.size());
    goal_topics_.resize(agent_num);
    odom_topics_.resize(agent_num);
    odoms_.resize(agent_num);
    odom_received_.assign(agent_num, false);
    odom_stamps_.assign(agent_num, ros::Time(0));
    latest_traj_start_.assign(agent_num, 0.0);
    latest_traj_id_.assign(agent_num, 0);
    formation_offsets_.assign(agent_num, Vec3{});

    if (flat_offsets.size() < agent_num * 3)
    {
      ROS_WARN_STREAM("[formulation_task] formation_offsets size=" << flat_offsets.size()
                                                                   << ", expected at least "
                                                                   << agent_num * 3
                                                                   << ". Missing offsets use zero.");
    }
    for (size_t i = 0; i < agent_num; ++i)
    {
      const size_t base = i * 3;
      if (flat_offsets.size() >= base + 3)
      {
        formation_offsets_[i] = {flat_offsets[base], flat_offsets[base + 1], flat_offsets[base + 2]};
      }
    }

    for (size_t i = 0; i < agent_num; ++i)
    {
      goal_pubs_.emplace_back(pnh_.advertise<geometry_msgs::PoseStamped>(goal_topics_[i], 1, true));
      odom_subs_.emplace_back(
          pnh_.subscribe<nav_msgs::Odometry>(
              odom_topics_[i],
              10,
              [this, i](const nav_msgs::OdometryConstPtr &msg) { this->odomCallback(msg, i); },
              ros::VoidPtr(),
              ros::TransportHints().tcpNoDelay()));
      ROS_INFO_STREAM("[formulation_task] agent " << i << ": odom " << odom_topics_[i]
                                                  << ", goal " << goal_topics_[i]
                                                  << ", offset [" << formation_offsets_[i].x
                                                  << ", " << formation_offsets_[i].y
                                                  << ", " << formation_offsets_[i].z << "]");
    }

    formation_reference_pub_ = pnh_.advertise<nav_msgs::Path>(formation_reference_topic_, 1, true);
    traj_sub_ = pnh_.subscribe(traj_broadcast_topic_,
                              100,
                              &FormulationTask::trajectoryCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
    goal_sub_ = pnh_.subscribe(input_goal_topic_,
                              10,
                              &FormulationTask::goalCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
    ROS_INFO_STREAM("[formulation_task] input goal: " << input_goal_topic_
                                                      << ", formation reference: "
                                                      << formation_reference_topic_
                                                      << ", trajectory broadcast: "
                                                      << traj_broadcast_topic_
                                                      << ", sequential="
                                                      << sequential_start_enable_);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg, size_t index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odoms_[index] = *msg;
    odom_received_[index] = true;
    odom_stamps_[index] = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  }

  void trajectoryCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg)
  {
    if ((msg->type & quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ) == 0)
    {
      return;
    }
    int drone_id = -1;
    if (!parseDroneId(msg->debug_info, drone_id))
    {
      return;
    }
    if (drone_id < 0 || drone_id >= static_cast<int>(latest_traj_start_.size()))
    {
      return;
    }
    const double start_wt = msg->start_WT_pos.toSec();
    if (start_wt <= 0.0)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (start_wt > latest_traj_start_[drone_id])
    {
      latest_traj_start_[drone_id] = start_wt;
      latest_traj_id_[drone_id] = msg->trajectory_id;
    }
  }

  bool getFormationCenter(Vec3 &center) const
  {
    center = {};
    const ros::Time now = ros::Time::now();
    for (size_t i = 0; i < odoms_.size(); ++i)
    {
      if (!odom_received_[i])
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[formulation_task] waiting odom: " << odom_topics_[i]);
        return false;
      }
      if (odom_timeout_ > 0.0 && (now - odom_stamps_[i]).toSec() > odom_timeout_)
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[formulation_task] stale odom: " << odom_topics_[i]);
        return false;
      }
      center += {odoms_[i].pose.pose.position.x,
                 odoms_[i].pose.pose.position.y,
                 odoms_[i].pose.pose.position.z};
    }
    center = center / static_cast<double>(odoms_.size());
    return center.finite();
  }

  Vec3 meanFormationOffset() const
  {
    Vec3 mean;
    for (const auto &offset : formation_offsets_)
    {
      mean += offset;
    }
    return formation_offsets_.empty() ? mean : mean / static_cast<double>(formation_offsets_.size());
  }

  Vec3 rotateOffset(const Vec3 &offset, const Vec3 &axis, const Vec3 &lateral) const
  {
    return axis * offset.x + lateral * offset.y + Vec3{0.0, 0.0, offset.z};
  }

  void publishFormationReference(const nav_msgs::Path &reference)
  {
    const int repeats = std::max(1, publish_repeat_);
    const ros::Duration period(std::max(0.0, publish_period_));
    for (int rep = 0; rep < repeats && ros::ok(); ++rep)
    {
      formation_reference_pub_.publish(reference);
      if (rep + 1 < repeats && period.toSec() > 0.0)
      {
        period.sleep();
      }
    }
  }

  void publishGoal(size_t index, const geometry_msgs::PoseStamped &goal)
  {
    const int repeats = std::max(1, publish_repeat_);
    const ros::Duration period(std::max(0.0, publish_period_));
    for (int rep = 0; rep < repeats && ros::ok(); ++rep)
    {
      goal_pubs_[index].publish(goal);
      if (rep + 1 < repeats && period.toSec() > 0.0)
      {
        period.sleep();
      }
    }
  }

  bool waitForTrajectory(size_t index, double min_start_wt) const
  {
    const ros::Time deadline = ros::Time::now() + ros::Duration(std::max(0.1, sequential_timeout_));
    ros::Rate rate(50.0);
    while (ros::ok())
    {
      {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (index < latest_traj_start_.size() && latest_traj_start_[index] > min_start_wt)
        {
          return true;
        }
      }
      if (ros::Time::now() >= deadline)
      {
        return false;
      }
      rate.sleep();
    }
    return false;
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    std::unique_lock<std::mutex> dispatch_lock(dispatch_mutex_, std::try_to_lock);
    if (!dispatch_lock.owns_lock())
    {
      ROS_WARN("[formulation_task] previous formulation dispatch is still running, ignore new click.");
      return;
    }

    std::vector<geometry_msgs::PoseStamped> goals;
    nav_msgs::Path reference;
    Vec3 center;
    Vec3 target_center;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (odoms_.empty())
      {
        ROS_ERROR("[formulation_task] no swarm agents configured.");
        return;
      }
      if (!getFormationCenter(center))
      {
        return;
      }

      target_center = fromPoint(msg->pose.position);
      if (!use_clicked_z_)
      {
        target_center.z = goal_z_;
      }
      if (!target_center.finite())
      {
        ROS_ERROR("[formulation_task] invalid clicked target.");
        return;
      }

      Vec3 axis = target_center - center;
      axis.z = 0.0;
      const double axis_norm = normXY(axis);
      if (axis_norm < std::max(1.0e-3, min_segment_length_))
      {
        axis = last_axis_valid_ ? last_axis_ : Vec3{1.0, 0.0, 0.0};
      }
      else
      {
        axis = axis / axis_norm;
        last_axis_ = axis;
        last_axis_valid_ = true;
      }
      const Vec3 lateral{-axis.y, axis.x, 0.0};
      const Vec3 mean_offset = meanFormationOffset();
      const Vec3 start_anchor = center - rotateOffset(mean_offset, axis, lateral);
      const Vec3 end_anchor = target_center - rotateOffset(mean_offset, axis, lateral);

      const ros::Time stamp = ros::Time::now();
      const std::string frame_id = msg->header.frame_id.empty() ? std::string("world") : msg->header.frame_id;
      reference.header.stamp = stamp;
      reference.header.frame_id = frame_id;
      reference.poses.push_back(makePose(start_anchor, frame_id, stamp));
      reference.poses.push_back(makePose(end_anchor, frame_id, stamp));

      goals.reserve(odoms_.size());
      for (size_t i = 0; i < odoms_.size(); ++i)
      {
        const Vec3 agent_goal = end_anchor + rotateOffset(formation_offsets_[i], axis, lateral);
        geometry_msgs::PoseStamped out = makePose(agent_goal, frame_id, stamp);
        out.pose.orientation = msg->pose.orientation;
        goals.emplace_back(out);
      }

      ROS_INFO_STREAM("[formulation_task] click center [" << target_center.x << ", "
                                                          << target_center.y << ", "
                                                          << target_center.z << "] current center ["
                                                          << center.x << ", " << center.y << ", "
                                                          << center.z << "], anchor start ["
                                                          << start_anchor.x << ", "
                                                          << start_anchor.y << ", "
                                                          << start_anchor.z << "], anchor end ["
                                                          << end_anchor.x << ", "
                                                          << end_anchor.y << ", "
                                                          << end_anchor.z << "], agents="
                                                          << goals.size());
    }

    const double dispatch_start_wt = ros::Time::now().toSec() - 0.01;
    publishFormationReference(reference);
    if (!sequential_start_enable_)
    {
      for (size_t i = 0; i < goals.size(); ++i)
      {
        publishGoal(i, goals[i]);
      }
      return;
    }

    for (size_t i = 0; i < goals.size() && ros::ok(); ++i)
    {
      publishGoal(i, goals[i]);
      if (waitForTrajectory(i, dispatch_start_wt))
      {
        ROS_INFO_STREAM("[formulation_task] agent " << i << " accepted formation goal.");
      }
      else
      {
        ROS_WARN_STREAM("[formulation_task] timeout waiting trajectory from agent " << i
                                                                                   << ", continue dispatch.");
      }
    }
  }

  ros::NodeHandle pnh_;
  std::string input_goal_topic_;
  std::string formation_reference_topic_;
  std::string traj_broadcast_topic_;
  std::vector<std::string> goal_topics_;
  std::vector<std::string> odom_topics_;
  std::vector<Vec3> formation_offsets_;
  std::vector<ros::Publisher> goal_pubs_;
  std::vector<ros::Subscriber> odom_subs_;
  ros::Publisher formation_reference_pub_;
  ros::Subscriber traj_sub_;
  ros::Subscriber goal_sub_;

  mutable std::mutex mutex_;
  mutable std::mutex traj_mutex_;
  std::mutex dispatch_mutex_;
  std::vector<nav_msgs::Odometry> odoms_;
  std::vector<bool> odom_received_;
  std::vector<ros::Time> odom_stamps_;
  std::vector<double> latest_traj_start_;
  std::vector<unsigned int> latest_traj_id_;

  double odom_timeout_{1.0};
  int publish_repeat_{3};
  double publish_period_{0.05};
  bool use_clicked_z_{false};
  double goal_z_{1.5};
  bool sequential_start_enable_{true};
  double sequential_timeout_{5.0};
  double min_segment_length_{0.2};
  Vec3 last_axis_{1.0, 0.0, 0.0};
  bool last_axis_valid_{false};
};
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "formulation_task");
  ros::NodeHandle pnh("~");
  FormulationTask task(pnh);
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
