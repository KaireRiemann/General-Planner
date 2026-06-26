#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
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

  Vec3 operator/(double value) const
  {
    return {x / value, y / value, z / value};
  }

  Vec3 operator-(const Vec3 &other) const
  {
    return {x - other.x, y - other.y, z - other.z};
  }

  Vec3 operator+(const Vec3 &other) const
  {
    return {x + other.x, y + other.y, z + other.z};
  }
};

class SwarmGoalRelay
{
public:
  explicit SwarmGoalRelay(const ros::NodeHandle &pnh) : pnh_(pnh)
  {
    pnh_.param("input_goal_topic", input_goal_topic_, std::string("/swarm/goal"));
    pnh_.param("odom_timeout", odom_timeout_, 1.0);
    pnh_.param("publish_repeat", publish_repeat_, 3);
    pnh_.param("publish_period", publish_period_, 0.05);
    pnh_.param("use_clicked_z", use_clicked_z_, false);
    pnh_.param("goal_z", goal_z_, 1.5);

    goal_topics_ = {"/swarm0/goal", "/swarm1/goal"};
    odom_topics_ = {"/swarm0/lidar_slam/odom", "/swarm1/lidar_slam/odom"};
    pnh_.getParam("goal_topics", goal_topics_);
    pnh_.getParam("odom_topics", odom_topics_);

    const size_t agent_num = std::min(goal_topics_.size(), odom_topics_.size());
    goal_topics_.resize(agent_num);
    odom_topics_.resize(agent_num);
    odoms_.resize(agent_num);
    odom_received_.assign(agent_num, false);
    odom_stamps_.assign(agent_num, ros::Time(0));

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
      ROS_INFO_STREAM("[swarm_goal_relay] agent " << i << ": odom " << odom_topics_[i]
                                                  << ", goal " << goal_topics_[i]);
    }

    goal_sub_ = pnh_.subscribe(input_goal_topic_,
                              10,
                              &SwarmGoalRelay::goalCallback,
                              this,
                              ros::TransportHints().tcpNoDelay());
    ROS_INFO_STREAM("[swarm_goal_relay] input goal topic: " << input_goal_topic_
                                                           << ", agents=" << agent_num);
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr &msg, size_t index)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odoms_[index] = *msg;
    odom_received_[index] = true;
    odom_stamps_[index] = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  }

  bool getFormationCenter(Vec3 &center) const
  {
    center = {};
    const ros::Time now = ros::Time::now();
    for (size_t i = 0; i < odoms_.size(); ++i)
    {
      if (!odom_received_[i])
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[swarm_goal_relay] waiting odom: " << odom_topics_[i]);
        return false;
      }
      if (odom_timeout_ > 0.0 && (now - odom_stamps_[i]).toSec() > odom_timeout_)
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[swarm_goal_relay] stale odom: " << odom_topics_[i]);
        return false;
      }
      center += {odoms_[i].pose.pose.position.x,
                 odoms_[i].pose.pose.position.y,
                 odoms_[i].pose.pose.position.z};
    }
    center = center / static_cast<double>(odoms_.size());
    return true;
  }

  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    std::vector<geometry_msgs::PoseStamped> goals;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (odoms_.empty())
      {
        ROS_ERROR("[swarm_goal_relay] no swarm agents configured.");
        return;
      }

      Vec3 center;
      if (!getFormationCenter(center))
      {
        return;
      }

      Vec3 target_center{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
      if (!use_clicked_z_)
      {
        target_center.z = goal_z_;
      }
      const Vec3 movement = target_center - center;

      goals.reserve(odoms_.size());
      const ros::Time stamp = ros::Time::now();
      for (size_t i = 0; i < odoms_.size(); ++i)
      {
        const Vec3 start{odoms_[i].pose.pose.position.x,
                         odoms_[i].pose.pose.position.y,
                         odoms_[i].pose.pose.position.z};
        const Vec3 agent_goal = start + movement;

        geometry_msgs::PoseStamped out;
        out.header.stamp = stamp;
        out.header.frame_id = msg->header.frame_id.empty() ? std::string("world") : msg->header.frame_id;
        out.pose.position.x = agent_goal.x;
        out.pose.position.y = agent_goal.y;
        out.pose.position.z = agent_goal.z;
        out.pose.orientation = msg->pose.orientation;
        goals.emplace_back(out);
      }

      ROS_INFO_STREAM("[swarm_goal_relay] click center [" << target_center.x << ", "
                                                         << target_center.y << ", "
                                                         << target_center.z << "] from formation center ["
                                                         << center.x << ", " << center.y << ", "
                                                         << center.z << "], dispatch agents="
                                                         << goals.size());
    }

    const int repeats = std::max(1, publish_repeat_);
    const ros::Duration period(std::max(0.0, publish_period_));
    for (int rep = 0; rep < repeats && ros::ok(); ++rep)
    {
      for (size_t i = 0; i < goals.size(); ++i)
      {
        goal_pubs_[i].publish(goals[i]);
      }
      if (rep + 1 < repeats && period.toSec() > 0.0)
      {
        period.sleep();
      }
    }
  }

  ros::NodeHandle pnh_;
  std::string input_goal_topic_;
  std::vector<std::string> goal_topics_;
  std::vector<std::string> odom_topics_;
  std::vector<ros::Publisher> goal_pubs_;
  std::vector<ros::Subscriber> odom_subs_;
  ros::Subscriber goal_sub_;

  mutable std::mutex mutex_;
  std::vector<nav_msgs::Odometry> odoms_;
  std::vector<bool> odom_received_;
  std::vector<ros::Time> odom_stamps_;

  double odom_timeout_{1.0};
  int publish_repeat_{3};
  double publish_period_{0.05};
  bool use_clicked_z_{false};
  double goal_z_{1.5};
};
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_goal_relay");
  ros::NodeHandle pnh("~");
  SwarmGoalRelay relay(pnh);
  ros::spin();
  return 0;
}
