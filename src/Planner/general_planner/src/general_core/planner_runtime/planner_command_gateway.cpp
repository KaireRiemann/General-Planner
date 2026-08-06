#include <general_core/planner_runtime/planner_command_gateway.hpp>

#include <algorithm>
#include <cmath>

namespace general_planner::planner_runtime {

PlannerCommandGateway::PlannerCommandGateway(ros::NodeHandle &nh) : nh_(nh) {
  nh_.param<std::string>("navigation_cmd_topic", navigation_cmd_topic_,
                         "/planning/navigation/pos_cmd");
  nh_.param<std::string>("exploration_cmd_topic", exploration_cmd_topic_,
                         "/planning/exploration/pos_cmd");
  nh_.param<std::string>("output_cmd_topic", output_cmd_topic_,
                         "/planning/pos_cmd");
  nh_.param<std::string>("odometry_topic", odometry_topic_,
                         "/lidar_slam/odom");
  nh_.param("command_timeout", command_timeout_, 0.30);
  nh_.param("publish_rate", publish_rate_, 100.0);
  command_timeout_ = std::clamp(command_timeout_, 0.05, 2.0);
  publish_rate_ = std::clamp(publish_rate_, 20.0, 200.0);

  output_pub_ =
      nh_.advertise<quadrotor_msgs::PositionCommand>(output_cmd_topic_, 50);
  navigation_sub_ = nh_.subscribe(navigation_cmd_topic_, 50,
                                  &PlannerCommandGateway::navigationCallback,
                                  this);
  exploration_sub_ = nh_.subscribe(exploration_cmd_topic_, 50,
                                   &PlannerCommandGateway::explorationCallback,
                                   this);
  odom_sub_ = nh_.subscribe(odometry_topic_, 50,
                            &PlannerCommandGateway::odometryCallback, this);
  timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / publish_rate_),
                               &PlannerCommandGateway::timerCallback, this);

  ROS_INFO_STREAM("[planner_command_gateway] nav=" << navigation_cmd_topic_
                  << " exploration=" << exploration_cmd_topic_
                  << " output=" << output_cmd_topic_);
}

void PlannerCommandGateway::setAuthorizedOwner(const CommandOwner owner,
                                               const std::uint64_t task_epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (authorized_owner_ == owner && authorized_epoch_ == task_epoch) {
    return;
  }
  authorized_owner_ = owner;
  authorized_epoch_ = task_epoch;
  authorization_time_ = ros::WallTime::now();
  if (owner == CommandOwner::HOLD) {
    ++hold_sequence_;
  }
  ROS_INFO_STREAM("[planner_command_gateway] authorize owner="
                  << toString(owner) << " epoch=" << task_epoch);
}

void PlannerCommandGateway::lockHoldAnchorFromOdom() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_odom_) {
    ROS_WARN("[planner_command_gateway] cannot lock hold anchor without odom");
    return;
  }
  hold_x_ = odom_.pose.pose.position.x;
  hold_y_ = odom_.pose.pose.position.y;
  hold_z_ = odom_.pose.pose.position.z;
  hold_yaw_ = yawFromQuat(odom_.pose.pose.orientation);
  hold_anchor_valid_ = true;
  ++hold_sequence_;
  ROS_INFO_STREAM("[planner_command_gateway] lock hold anchor x="
                  << hold_x_ << " y=" << hold_y_ << " z=" << hold_z_
                  << " yaw=" << hold_yaw_);
}

void PlannerCommandGateway::lockHoldAnchor(const double x, const double y,
                                           const double z, const double yaw) {
  std::lock_guard<std::mutex> lock(mutex_);
  hold_x_ = x;
  hold_y_ = y;
  hold_z_ = z;
  hold_yaw_ = yaw;
  hold_anchor_valid_ = true;
  ++hold_sequence_;
}

bool PlannerCommandGateway::hasHoldAnchor() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return hold_anchor_valid_;
}

CommandOwner PlannerCommandGateway::authorizedOwner() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return authorized_owner_;
}

std::uint64_t PlannerCommandGateway::authorizedEpoch() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return authorized_epoch_;
}

void PlannerCommandGateway::navigationCallback(
    const quadrotor_msgs::PositionCommandConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  navigation_cmd_ = *msg;
  navigation_rx_time_ = ros::WallTime::now();
  have_navigation_cmd_ = true;
}

void PlannerCommandGateway::explorationCallback(
    const quadrotor_msgs::PositionCommandConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  exploration_cmd_ = *msg;
  exploration_rx_time_ = ros::WallTime::now();
  have_exploration_cmd_ = true;
}

void PlannerCommandGateway::odometryCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  odom_ = *msg;
  have_odom_ = true;
}

double PlannerCommandGateway::yawFromQuat(const geometry_msgs::Quaternion &q) {
  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sin_yaw, cos_yaw);
}

quadrotor_msgs::PositionCommand
PlannerCommandGateway::makeHoldCommandLocked() const {
  quadrotor_msgs::PositionCommand hold;
  hold.header.stamp = ros::Time::now();
  hold.header.frame_id =
      odom_.header.frame_id.empty() ? "world" : odom_.header.frame_id;
  hold.trajectory_flag =
      quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  hold.trajectory_id = hold_sequence_;
  if (hold_anchor_valid_) {
    hold.position.x = hold_x_;
    hold.position.y = hold_y_;
    hold.position.z = hold_z_;
    hold.yaw = hold_yaw_;
  } else if (have_odom_) {
    hold.position = odom_.pose.pose.position;
    hold.yaw = yawFromQuat(odom_.pose.pose.orientation);
  }
  hold.velocity.x = hold.velocity.y = hold.velocity.z = 0.0;
  hold.acceleration.x = hold.acceleration.y = hold.acceleration.z = 0.0;
  hold.jerk.x = hold.jerk.y = hold.jerk.z = 0.0;
  hold.yaw_dot = 0.0;
  hold.vel_norm = 0.0;
  hold.acc_norm = 0.0;
  return hold;
}

void PlannerCommandGateway::timerCallback(const ros::WallTimerEvent &) {
  quadrotor_msgs::PositionCommand output;
  bool publish = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::WallTime now = ros::WallTime::now();
    const bool navigation_fresh =
        have_navigation_cmd_ && navigation_rx_time_ >= authorization_time_ &&
        (now - navigation_rx_time_).toSec() <= command_timeout_;
    const bool exploration_fresh =
        have_exploration_cmd_ && exploration_rx_time_ >= authorization_time_ &&
        (now - exploration_rx_time_).toSec() <= command_timeout_;

    if (authorized_owner_ == CommandOwner::STATE2STATE && navigation_fresh) {
      output = navigation_cmd_;
      publish = true;
    } else if (authorized_owner_ == CommandOwner::EXPLORATION &&
               exploration_fresh) {
      output = exploration_cmd_;
      publish = true;
    } else if (have_odom_ || hold_anchor_valid_) {
      // Stale or unauthorized source commands never reach the controller.
      output = makeHoldCommandLocked();
      publish = true;
    }
  }
  if (!publish) {
    ROS_WARN_THROTTLE(1.0,
                      "[planner_command_gateway] waiting for odometry before "
                      "publishing hold");
    return;
  }
  output.header.stamp = ros::Time::now();
  output_pub_.publish(output);
}

} // namespace general_planner::planner_runtime
