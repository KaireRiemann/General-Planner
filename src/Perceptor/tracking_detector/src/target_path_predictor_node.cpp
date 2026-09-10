#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double clampValue(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
  const Eigen::Quaterniond quaternion(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  geometry_msgs::Quaternion message;
  message.w = quaternion.w();
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  return message;
}

struct TargetStateSample {
  ros::Time stamp;
  Eigen::Vector2d velocity;
};

class TargetPathPredictor {
 public:
  TargetPathPredictor() : nh_("~") {
    nh_.param("prediction_horizon", prediction_horizon_, 4.0);
    nh_.param("prediction_dt", prediction_dt_, 0.25);
    nh_.param("publish_rate", publish_rate_, 20.0);
    nh_.param("input_timeout", input_timeout_, 0.4);
    nh_.param("heading_window", heading_window_, 0.5);
    nh_.param("turn_rate_filter", turn_rate_filter_, 0.35);
    nh_.param("turn_rate_decay", turn_rate_decay_, 0.8);
    nh_.param("maximum_speed", maximum_speed_, 4.0);
    nh_.param("maximum_turn_rate", maximum_turn_rate_, 1.2);
    nh_.param("minimum_speed", minimum_speed_, 0.15);
    nh_.param("turn_rate_deadband", turn_rate_deadband_, 0.04);
    nh_.param<std::string>("frame_id", fallback_frame_id_, "world");

    prediction_horizon_ = std::max(prediction_horizon_, 0.05);
    prediction_dt_ = std::max(prediction_dt_, 0.05);
    publish_rate_ = std::max(publish_rate_, 1.0);
    input_timeout_ = std::max(input_timeout_, 0.05);
    heading_window_ = std::max(heading_window_, 0.05);
    turn_rate_filter_ = clampValue(turn_rate_filter_, 0.0, 1.0);
    turn_rate_decay_ = std::max(turn_rate_decay_, 0.05);
    minimum_speed_ = std::max(minimum_speed_, 0.01);
    maximum_speed_ = std::max(maximum_speed_, minimum_speed_);
    maximum_turn_rate_ = std::max(maximum_turn_rate_, 0.01);
    turn_rate_deadband_ = std::max(turn_rate_deadband_, 0.0);

    target_sub_ = nh_.subscribe("target_odom", 20, &TargetPathPredictor::targetCallback, this,
                                ros::TransportHints().tcpNoDelay());
    prediction_pub_ = nh_.advertise<nav_msgs::Path>("prediction", 10);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &TargetPathPredictor::publishTimerCallback, this);

    ROS_INFO("Target path predictor ready: CTRV horizon %.2f s, dt %.2f s.",
             prediction_horizon_, prediction_dt_);
  }

 private:
  void targetCallback(const nav_msgs::OdometryConstPtr& message) {
    const ros::Time stamp = message->header.stamp.isZero() ? ros::Time::now() : message->header.stamp;
    const Eigen::Vector2d velocity(message->twist.twist.linear.x, message->twist.twist.linear.y);

    std::lock_guard<std::mutex> lock(mutex_);
    latest_target_ = *message;
    latest_target_receipt_stamp_ = ros::Time::now();
    has_target_ = true;

    if (!history_.empty() && stamp <= history_.back().stamp) {
      return;
    }
    history_.push_back({stamp, velocity});
    while (history_.size() > 2 &&
           (history_.back().stamp - history_.front().stamp).toSec() > 2.0 * heading_window_) {
      history_.pop_front();
    }
    updateTurnRateLocked();
  }

  void updateTurnRateLocked() {
    if (history_.size() < 2) {
      return;
    }

    const TargetStateSample& newest = history_.back();
    if (newest.velocity.norm() < minimum_speed_) {
      return;
    }

    int reference_index = -1;
    for (int index = static_cast<int>(history_.size()) - 2; index >= 0; --index) {
      if (history_[index].velocity.norm() < minimum_speed_) {
        continue;
      }
      const double dt = (newest.stamp - history_[index].stamp).toSec();
      if (dt >= heading_window_) {
        reference_index = index;
        break;
      }
    }
    if (reference_index < 0) {
      return;
    }

    const TargetStateSample& reference = history_[reference_index];
    const double dt = (newest.stamp - reference.stamp).toSec();
    if (dt <= 1e-3) {
      return;
    }
    const double current_heading = std::atan2(newest.velocity.y(), newest.velocity.x());
    const double reference_heading = std::atan2(reference.velocity.y(), reference.velocity.x());
    const double raw_turn_rate = wrapAngle(current_heading - reference_heading) / dt;
    const double bounded_turn_rate =
        clampValue(raw_turn_rate, -maximum_turn_rate_, maximum_turn_rate_);
    turn_rate_ = (1.0 - turn_rate_filter_) * turn_rate_ + turn_rate_filter_ * bounded_turn_rate;
    last_turn_rate_update_stamp_ = newest.stamp;
  }

  void publishTimerCallback(const ros::TimerEvent&) {
    nav_msgs::Odometry target;
    double turn_rate = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_target_ ||
          (ros::Time::now() - latest_target_receipt_stamp_).toSec() > input_timeout_) {
        return;
      }
      target = latest_target_;
      turn_rate = turn_rate_;
      if (!last_turn_rate_update_stamp_.isZero()) {
        const double time_without_turn_update =
            std::max(0.0, (target.header.stamp - last_turn_rate_update_stamp_).toSec());
        turn_rate *= std::exp(-time_without_turn_update / turn_rate_decay_);
      }
    }

    const Eigen::Vector3d start_position(target.pose.pose.position.x,
                                         target.pose.pose.position.y,
                                         target.pose.pose.position.z);
    Eigen::Vector3d velocity(target.twist.twist.linear.x,
                             target.twist.twist.linear.y,
                             target.twist.twist.linear.z);
    double speed = velocity.head<2>().norm();
    if (speed > maximum_speed_) {
      velocity.head<2>() *= maximum_speed_ / speed;
      speed = maximum_speed_;
    }
    const double heading = speed >= minimum_speed_ ? std::atan2(velocity.y(), velocity.x()) : 0.0;
    if (speed < minimum_speed_ || std::abs(turn_rate) < turn_rate_deadband_) {
      turn_rate = 0.0;
    }

    nav_msgs::Path prediction;
    prediction.header.stamp = ros::Time::now();
    prediction.header.frame_id = target.header.frame_id.empty() ? fallback_frame_id_ : target.header.frame_id;
    const int sample_count = static_cast<int>(std::ceil(prediction_horizon_ / prediction_dt_));
    prediction.poses.reserve(sample_count + 1);
    for (int index = 0; index <= sample_count; ++index) {
      const double time_from_start = std::min(prediction_horizon_, index * prediction_dt_);
      Eigen::Vector3d position = start_position;
      double yaw = heading;
      if (turn_rate == 0.0) {
        position.x() += velocity.x() * time_from_start;
        position.y() += velocity.y() * time_from_start;
      } else {
        yaw = heading + turn_rate * time_from_start;
        position.x() += speed / turn_rate * (std::sin(yaw) - std::sin(heading));
        position.y() -= speed / turn_rate * (std::cos(yaw) - std::cos(heading));
      }
      position.z() += velocity.z() * time_from_start;

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = prediction.header.frame_id;
      pose.header.stamp = prediction.header.stamp + ros::Duration(time_from_start);
      pose.pose.position.x = position.x();
      pose.pose.position.y = position.y();
      pose.pose.position.z = position.z();
      pose.pose.orientation = quaternionFromYaw(yaw);
      prediction.poses.push_back(pose);
    }
    prediction_pub_.publish(prediction);
  }

  ros::NodeHandle nh_;
  ros::Subscriber target_sub_;
  ros::Publisher prediction_pub_;
  ros::Timer publish_timer_;

  std::mutex mutex_;
  std::deque<TargetStateSample> history_;
  nav_msgs::Odometry latest_target_;
  ros::Time latest_target_receipt_stamp_;
  ros::Time last_turn_rate_update_stamp_;
  bool has_target_ = false;
  double turn_rate_ = 0.0;

  double prediction_horizon_;
  double prediction_dt_;
  double publish_rate_;
  double input_timeout_;
  double heading_window_;
  double turn_rate_filter_;
  double turn_rate_decay_;
  double maximum_speed_;
  double maximum_turn_rate_;
  double minimum_speed_;
  double turn_rate_deadband_;
  std::string fallback_frame_id_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_path_predictor");
  TargetPathPredictor predictor;
  ros::spin();
  return 0;
}
