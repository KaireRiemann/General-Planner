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
  clearSourceTimeoutHoldLocked();
  if (owner == CommandOwner::HOLD) {
    ++hold_sequence_;
  }
  ROS_INFO_STREAM("[planner_command_gateway] authorize owner="
                  << toString(owner) << " epoch=" << task_epoch);
}

void PlannerCommandGateway::setHoldAnchorAndAuthorize(
    const double x, const double y, const double z, const double yaw,
    const std::uint64_t task_epoch) {
  std::lock_guard<std::mutex> lock(mutex_);
  authorized_owner_ = CommandOwner::HOLD;
  authorized_epoch_ = task_epoch;
  authorization_time_ = ros::WallTime::now();
  clearSourceTimeoutHoldLocked();

  hold_x_ = x;
  hold_y_ = y;
  hold_z_ = z;
  hold_yaw_ = yaw;
  hold_anchor_valid_ = true;
  hold_anchor_required_ = false;
  ++hold_sequence_;

  ROS_INFO_STREAM("[planner_command_gateway] authorize owner=hold epoch="
                  << task_epoch << " and lock hold anchor x=" << hold_x_
                  << " y=" << hold_y_ << " z=" << hold_z_
                  << " yaw=" << hold_yaw_);
}

void PlannerCommandGateway::clearHoldAnchor() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!hold_anchor_valid_ && hold_anchor_required_) {
    return;
  }
  hold_anchor_valid_ = false;
  hold_anchor_required_ = true;
  ++hold_sequence_;
  clearSourceTimeoutHoldLocked();
  ROS_WARN("[planner_command_gateway] cleared stale hold anchor");
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
  hold_anchor_required_ = false;
  ++hold_sequence_;
  clearSourceTimeoutHoldLocked();
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
  hold_anchor_required_ = false;
  ++hold_sequence_;
  clearSourceTimeoutHoldLocked();
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

void PlannerCommandGateway::setPublishingEnabled(const bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (publishing_enabled_ == enabled) {
    return;
  }
  publishing_enabled_ = enabled;
  ROS_INFO_STREAM("[planner_command_gateway] publishing_enabled="
                  << (enabled ? "true" : "false"));
}

bool PlannerCommandGateway::publishingEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return publishing_enabled_;
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

quadrotor_msgs::PositionCommand
PlannerCommandGateway::makeSourceTimeoutHoldCommandLocked(
    const CommandOwner owner) {
  if (!source_timeout_hold_active_ || source_timeout_hold_owner_ != owner) {
    // Never use hold_x_/hold_y_/hold_z_ here.  Those are task-transition
    // anchors and can belong to the take-off point of a previous task.  A
    // transient source timeout must instead stop at the latest observed pose.
    source_timeout_hold_ = quadrotor_msgs::PositionCommand();
    source_timeout_hold_.header.stamp = ros::Time::now();
    source_timeout_hold_.header.frame_id =
        odom_.header.frame_id.empty() ? "world" : odom_.header.frame_id;
    source_timeout_hold_.trajectory_flag =
        quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    source_timeout_hold_.trajectory_id = ++hold_sequence_;
    source_timeout_hold_.position = odom_.pose.pose.position;
    source_timeout_hold_.yaw = yawFromQuat(odom_.pose.pose.orientation);
    source_timeout_hold_.velocity.x = source_timeout_hold_.velocity.y =
        source_timeout_hold_.velocity.z = 0.0;
    source_timeout_hold_.acceleration.x = source_timeout_hold_.acceleration.y =
        source_timeout_hold_.acceleration.z = 0.0;
    source_timeout_hold_.jerk.x = source_timeout_hold_.jerk.y =
        source_timeout_hold_.jerk.z = 0.0;
    source_timeout_hold_.yaw_dot = 0.0;
    source_timeout_hold_.vel_norm = 0.0;
    source_timeout_hold_.acc_norm = 0.0;
    source_timeout_hold_active_ = true;
    source_timeout_hold_owner_ = owner;
  }
  return source_timeout_hold_;
}

void PlannerCommandGateway::clearSourceTimeoutHoldLocked() {
  source_timeout_hold_active_ = false;
  source_timeout_hold_owner_ = CommandOwner::HOLD;
}

void PlannerCommandGateway::timerCallback(const ros::WallTimerEvent &) {
  quadrotor_msgs::PositionCommand output;
  bool publish = false;
  bool entered_source_timeout_hold = false;
  bool resumed_source_after_timeout = false;
  CommandOwner event_owner = CommandOwner::HOLD;
  double source_age = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!publishing_enabled_) {
      return;
    }
    const ros::WallTime now = ros::WallTime::now();
    const bool navigation_fresh =
        have_navigation_cmd_ && navigation_rx_time_ >= authorization_time_ &&
        (now - navigation_rx_time_).toSec() <= command_timeout_;
    const bool exploration_fresh =
        have_exploration_cmd_ && exploration_rx_time_ >= authorization_time_ &&
        (now - exploration_rx_time_).toSec() <= command_timeout_;

    const GatewayOutputMode output_mode = selectGatewayOutputMode(
        authorized_owner_, navigation_fresh, exploration_fresh);
    if (output_mode == GatewayOutputMode::EXTERNAL_GATE_SUPPRESSED) {
      // Gate mode is a command-bus handover.  In particular, never fall
      // through to makeHoldCommandLocked(): doing so would race the external
      // gate planner on /planning/pos_cmd.
      return;
    } else if (output_mode == GatewayOutputMode::NAVIGATION) {
      resumed_source_after_timeout = source_timeout_hold_active_;
      event_owner = authorized_owner_;
      clearSourceTimeoutHoldLocked();
      output = navigation_cmd_;
      publish = true;
    } else if (output_mode == GatewayOutputMode::EXPLORATION) {
      resumed_source_after_timeout = source_timeout_hold_active_;
      event_owner = authorized_owner_;
      clearSourceTimeoutHoldLocked();
      output = exploration_cmd_;
      publish = true;
    } else if (output_mode == GatewayOutputMode::SOURCE_TIMEOUT_HOLD &&
               have_odom_) {
      entered_source_timeout_hold = !source_timeout_hold_active_;
      event_owner = authorized_owner_;
      if (authorized_owner_ == CommandOwner::STATE2STATE &&
          have_navigation_cmd_) {
        source_age = (now - navigation_rx_time_).toSec();
      } else if (authorized_owner_ == CommandOwner::EXPLORATION &&
                 have_exploration_cmd_) {
        source_age = (now - exploration_rx_time_).toSec();
      }
      output = makeSourceTimeoutHoldCommandLocked(authorized_owner_);
      publish = true;
    } else if ((have_odom_ || hold_anchor_valid_) &&
               !(authorized_owner_ == CommandOwner::HOLD &&
                 hold_anchor_required_)) {
      clearSourceTimeoutHoldLocked();
      // This is an explicit lifecycle HOLD, not a source timeout.
      output = makeHoldCommandLocked();
      publish = true;
    }
  }
  if (entered_source_timeout_hold) {
    ROS_WARN_STREAM("[planner_command_gateway] source timeout: owner="
                    << toString(event_owner) << " age=" << source_age
                    << "s; holding current odometry until source resumes");
  } else if (resumed_source_after_timeout) {
    ROS_INFO_STREAM("[planner_command_gateway] source resumed safely: owner="
                    << toString(event_owner));
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
