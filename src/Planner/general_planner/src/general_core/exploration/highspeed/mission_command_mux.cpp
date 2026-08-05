#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <mutex>
#include <string>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace {

enum class CommandSource { HOLD, NAVIGATION, EXPLORATION };

class MissionCommandMux {
public:
  explicit MissionCommandMux(ros::NodeHandle &nh) : nh_(nh) {
    nh_.param<std::string>("navigation_cmd_topic", navigation_cmd_topic_,
                           "/planning/navigation/pos_cmd");
    nh_.param<std::string>("exploration_cmd_topic", exploration_cmd_topic_,
                           "/planning/exploration/pos_cmd");
    nh_.param<std::string>("output_cmd_topic", output_cmd_topic_,
                           "/planning/pos_cmd");
    nh_.param<std::string>("task_mode_topic", task_mode_topic_,
                           "/mission/task_mode");
    nh_.param<std::string>("odometry_topic", odometry_topic_,
                           "/lidar_slam/odom");
    nh_.param("command_timeout", command_timeout_, 0.30);
    nh_.param("publish_rate", publish_rate_, 100.0);
    command_timeout_ = std::clamp(command_timeout_, 0.05, 2.0);
    publish_rate_ = std::clamp(publish_rate_, 20.0, 200.0);

    output_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(
        output_cmd_topic_, 50);
    navigation_sub_ = nh_.subscribe(navigation_cmd_topic_, 50,
                                    &MissionCommandMux::navigationCallback,
                                    this);
    exploration_sub_ = nh_.subscribe(exploration_cmd_topic_, 50,
                                     &MissionCommandMux::explorationCallback,
                                     this);
    task_mode_sub_ = nh_.subscribe(task_mode_topic_, 10,
                                   &MissionCommandMux::taskModeCallback,
                                   this);
    odom_sub_ = nh_.subscribe(odometry_topic_, 50,
                              &MissionCommandMux::odometryCallback, this);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / publish_rate_),
                                 &MissionCommandMux::timerCallback, this);

    ROS_INFO_STREAM("[mission command mux] nav=" << navigation_cmd_topic_
                    << " exploration=" << exploration_cmd_topic_
                    << " output=" << output_cmd_topic_
                    << " mode=" << task_mode_topic_);
  }

private:
  static CommandSource sourceForMode(std::string mode) {
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode == "exploration" || mode == "explore") {
      return CommandSource::EXPLORATION;
    }
    if (mode == "wait" || mode == "hold" || mode == "idle") {
      return CommandSource::HOLD;
    }
    return CommandSource::NAVIGATION;
  }

  static const char *sourceName(const CommandSource source) {
    switch (source) {
    case CommandSource::EXPLORATION:
      return "exploration";
    case CommandSource::NAVIGATION:
      return "navigation";
    case CommandSource::HOLD:
    default:
      return "hold";
    }
  }

  void navigationCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
    if (!msg) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    navigation_cmd_ = *msg;
    navigation_rx_time_ = ros::WallTime::now();
    have_navigation_cmd_ = true;
  }

  void explorationCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
    if (!msg) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    exploration_cmd_ = *msg;
    exploration_rx_time_ = ros::WallTime::now();
    have_exploration_cmd_ = true;
  }

  void taskModeCallback(const std_msgs::StringConstPtr &msg) {
    if (!msg) {
      return;
    }
    const CommandSource requested = sourceForMode(msg->data);
    std::lock_guard<std::mutex> lock(mutex_);
    if (requested == selected_source_) {
      return;
    }
    selected_source_ = requested;
    selection_time_ = ros::WallTime::now();
    ++hold_sequence_;
    ROS_INFO_STREAM("[mission command mux] select "
                    << sourceName(selected_source_));
  }

  void odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    if (!msg) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    odom_ = *msg;
    have_odom_ = true;
  }

  quadrotor_msgs::PositionCommand makeHoldCommand() const {
    quadrotor_msgs::PositionCommand hold;
    hold.header.stamp = ros::Time::now();
    hold.header.frame_id = odom_.header.frame_id.empty() ? "world" : odom_.header.frame_id;
    hold.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    hold.trajectory_id = hold_sequence_;
    hold.position = odom_.pose.pose.position;
    hold.velocity.x = hold.velocity.y = hold.velocity.z = 0.0;
    hold.acceleration.x = hold.acceleration.y = hold.acceleration.z = 0.0;
    hold.jerk.x = hold.jerk.y = hold.jerk.z = 0.0;
    const auto &q = odom_.pose.pose.orientation;
    const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
    const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    hold.yaw = std::atan2(sin_yaw, cos_yaw);
    hold.yaw_dot = 0.0;
    hold.vel_norm = 0.0;
    hold.acc_norm = 0.0;
    return hold;
  }

  void timerCallback(const ros::WallTimerEvent &) {
    quadrotor_msgs::PositionCommand output;
    bool publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const ros::WallTime now = ros::WallTime::now();
      const bool navigation_fresh = have_navigation_cmd_ &&
          navigation_rx_time_ >= selection_time_ &&
          (now - navigation_rx_time_).toSec() <= command_timeout_;
      const bool exploration_fresh = have_exploration_cmd_ &&
          exploration_rx_time_ >= selection_time_ &&
          (now - exploration_rx_time_).toSec() <= command_timeout_;
      if (selected_source_ == CommandSource::NAVIGATION && navigation_fresh) {
        output = navigation_cmd_;
        publish = true;
      } else if (selected_source_ == CommandSource::EXPLORATION &&
                 exploration_fresh) {
        output = exploration_cmd_;
        publish = true;
      } else if (have_odom_) {
        output = makeHoldCommand();
        publish = true;
      }
    }
    if (!publish) {
      ROS_WARN_THROTTLE(1.0,
                        "[mission command mux] no odometry yet; cannot publish hold");
      return;
    }
    output.header.stamp = ros::Time::now();
    output_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::Publisher output_pub_;
  ros::Subscriber navigation_sub_;
  ros::Subscriber exploration_sub_;
  ros::Subscriber task_mode_sub_;
  ros::Subscriber odom_sub_;
  ros::WallTimer timer_;
  std::mutex mutex_;

  std::string navigation_cmd_topic_;
  std::string exploration_cmd_topic_;
  std::string output_cmd_topic_;
  std::string task_mode_topic_;
  std::string odometry_topic_;
  double command_timeout_{0.30};
  double publish_rate_{100.0};

  CommandSource selected_source_{CommandSource::HOLD};
  ros::WallTime selection_time_{ros::WallTime::now()};
  quadrotor_msgs::PositionCommand navigation_cmd_;
  quadrotor_msgs::PositionCommand exploration_cmd_;
  ros::WallTime navigation_rx_time_;
  ros::WallTime exploration_rx_time_;
  bool have_navigation_cmd_{false};
  bool have_exploration_cmd_{false};
  nav_msgs::Odometry odom_;
  bool have_odom_{false};
  std::uint32_t hold_sequence_{0};
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "mission_command_mux");
  ros::NodeHandle nh("~");
  MissionCommandMux mux(nh);
  ros::spin();
  return 0;
}
