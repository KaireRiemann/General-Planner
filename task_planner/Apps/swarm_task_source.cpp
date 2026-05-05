#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace
{
geometry_msgs::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_task_source");
  ros::NodeHandle pnh("~");

  std::vector<std::string> goal_topics{"/drone_0/goal", "/drone_1/goal"};
  std::vector<double> goals{2.5, 0.0, 1.5, -2.5, 0.0, 1.5};
  std::vector<double> yaws{0.0, 3.141592653589793};
  std::vector<double> delays{0.0, 1.5};
  pnh.getParam("goal_topics", goal_topics);
  pnh.getParam("goals", goals);
  pnh.getParam("yaws", yaws);
  pnh.getParam("delays", delays);

  double start_delay = 2.0;
  double publish_period = 0.25;
  double publish_duration = 5.0;
  double hold_duration = 2.0;
  bool repeat = false;
  bool wait_for_subscribers = true;
  pnh.param("start_delay", start_delay, start_delay);
  pnh.param("publish_period", publish_period, publish_period);
  pnh.param("publish_duration", publish_duration, publish_duration);
  pnh.param("hold_duration", hold_duration, hold_duration);
  pnh.param("repeat", repeat, repeat);
  pnh.param("wait_for_subscribers", wait_for_subscribers, wait_for_subscribers);

  const size_t goal_count = std::min(goal_topics.size(), goals.size() / 3);
  if (goal_count == 0)
  {
    ROS_ERROR("[swarm_task_source] No valid goal topics/goals.");
    return 1;
  }
  yaws.resize(goal_count, 0.0);
  delays.resize(goal_count, 0.0);

  std::vector<ros::Publisher> pubs;
  pubs.reserve(goal_count);
  for (size_t i = 0; i < goal_count; ++i)
  {
    pubs.emplace_back(pnh.advertise<geometry_msgs::PoseStamped>(goal_topics[i], 1, true));
    ROS_INFO_STREAM("[swarm_task_source] goal " << i << " -> " << goal_topics[i]
                                                << " at [" << goals[3 * i] << ", "
                                                << goals[3 * i + 1] << ", "
                                                << goals[3 * i + 2] << "], delay="
                                                << start_delay + delays[i]);
  }

  const double max_delay = *std::max_element(delays.begin(), delays.end());
  const double period = std::max(0.02, publish_period);
  ros::Rate rate(1.0 / period);
  const ros::Time start = ros::Time::now();
  std::vector<bool> sent(goal_count, false);
  while (ros::ok())
  {
    const double elapsed = (ros::Time::now() - start).toSec();
    bool active = false;
    for (size_t i = 0; i < goal_count; ++i)
    {
      const double begin_t = start_delay + delays[i];
      const double end_t = begin_t + publish_duration;
      if (elapsed < begin_t || elapsed > end_t)
      {
        continue;
      }
      if (!repeat && sent[i])
      {
        continue;
      }
      if (!repeat && wait_for_subscribers &&
          pubs[i].getNumSubscribers() == 0 && elapsed < end_t)
      {
        continue;
      }
      active = true;
      geometry_msgs::PoseStamped goal;
      goal.header.stamp = ros::Time::now();
      goal.header.frame_id = "world";
      goal.pose.position.x = goals[3 * i];
      goal.pose.position.y = goals[3 * i + 1];
      goal.pose.position.z = goals[3 * i + 2];
      goal.pose.orientation = yawToQuat(yaws[i]);
      pubs[i].publish(goal);
      sent[i] = true;
    }

    const bool all_sent = std::all_of(sent.begin(), sent.end(), [](bool value) { return value; });
    if ((repeat && elapsed > start_delay + max_delay + publish_duration + 0.5 && !active) ||
        (!repeat && all_sent && elapsed > start_delay + max_delay + hold_duration))
    {
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
