#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <ros/ros.h>

#include "data_structure/base/trajectory.h"

namespace
{
using geometry_utils::Trajectory;

geometry_msgs::Quaternion quatFromYaw(const double yaw)
{
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
}

double yawFromQuat(const geometry_msgs::Quaternion &q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double yawFromVelocity(const Eigen::Vector3d &vel, const double fallback_yaw)
{
    return vel.head<2>().norm() > 1.0e-3 ? std::atan2(vel.y(), vel.x()) : fallback_yaw;
}

geometry_msgs::PoseStamped makePose(const std::string &frame_id,
                                    const ros::Time &stamp,
                                    const Eigen::Vector3d &pos,
                                    const Eigen::Vector3d &vel,
                                    const double fallback_yaw)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = pos.x();
    pose.pose.position.y = pos.y();
    pose.pose.position.z = pos.z();
    pose.pose.orientation = quatFromYaw(yawFromVelocity(vel, fallback_yaw));
    return pose;
}

bool polynomialMsgToTrajectory(const quadrotor_msgs::PolynomialTrajectory &msg,
                               const ros::Time &receive_time,
                               Trajectory &traj)
{
    if ((msg.type & quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ) == 0 ||
        msg.piece_num_pos <= 0 || msg.order_pos < 1)
    {
        return false;
    }

    const int piece_num = static_cast<int>(msg.piece_num_pos);
    const int coeff_num = msg.order_pos + 1;
    if (static_cast<int>(msg.time_pos.size()) < piece_num ||
        static_cast<int>(msg.coef_pos_x.size()) < piece_num * coeff_num ||
        static_cast<int>(msg.coef_pos_y.size()) < piece_num * coeff_num ||
        static_cast<int>(msg.coef_pos_z.size()) < piece_num * coeff_num)
    {
        return false;
    }

    traj.clear();
    traj.reserve(piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
        const double duration = msg.time_pos[i];
        if (duration <= 1.0e-6)
        {
            traj.clear();
            return false;
        }

        Eigen::MatrixXd coeff(3, coeff_num);
        for (int j = 0; j < coeff_num; ++j)
        {
            const int offset = i * coeff_num + j;
            coeff(0, j) = msg.coef_pos_x[offset];
            coeff(1, j) = msg.coef_pos_y[offset];
            coeff(2, j) = msg.coef_pos_z[offset];
        }
        traj.emplace_back(duration, coeff);
    }

    traj.start_WT = msg.start_WT_pos.toSec();
    if (traj.start_WT <= 0.0 && !msg.header.stamp.isZero())
    {
        traj.start_WT = msg.header.stamp.toSec();
    }
    if (traj.start_WT <= 0.0)
    {
        traj.start_WT = receive_time.toSec();
    }
    return !traj.empty();
}

class TrackingTargetRelay
{
  public:
    explicit TrackingTargetRelay(const ros::NodeHandle &nh) : nh_(nh)
    {
        nh_.param<std::string>("target_odom_topic", target_odom_topic_, "/drone_0/lidar_slam/odom");
        nh_.param<std::string>("target_traj_topic", target_traj_topic_, "/drone_0/planning_cmd/poly_traj");
        nh_.param<std::string>("tracking_target_odom_topic", tracking_target_odom_topic_, "/tracking/target_odom");
        nh_.param<std::string>("tracking_prediction_topic", tracking_prediction_topic_, "/tracking/target_prediction");
        nh_.param<std::string>("tracking_target_path_topic", tracking_target_path_topic_, "/tracking/target_path");
        nh_.param<std::string>("frame_id", frame_id_, "world");
        nh_.param<std::string>("target_child_frame_id", target_child_frame_id_, "tracking_target");
        nh_.param<double>("prediction_horizon", prediction_horizon_, 4.0);
        nh_.param<double>("prediction_dt", prediction_dt_, 0.25);
        nh_.param<double>("path_dt", path_dt_, 0.1);
        nh_.param<double>("publish_rate", publish_rate_, 30.0);
        nh_.param<double>("prediction_publish_rate", prediction_publish_rate_, 8.0);
        nh_.param<double>("trajectory_finish_hold", trajectory_finish_hold_, 1.0);
        nh_.param<double>("trajectory_log_period", trajectory_log_period_, 1.0);
        nh_.param<bool>("publish_before_trajectory", publish_before_trajectory_, false);

        target_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(tracking_target_odom_topic_, 10);
        prediction_pub_ = nh_.advertise<nav_msgs::Path>(tracking_prediction_topic_, 10);
        target_path_pub_ = nh_.advertise<nav_msgs::Path>(tracking_target_path_topic_, 1, true);
        odom_sub_ = nh_.subscribe(target_odom_topic_, 20, &TrackingTargetRelay::odomCallback, this);
        traj_sub_ = nh_.subscribe(target_traj_topic_, 20, &TrackingTargetRelay::trajCallback, this);

        const double timer_dt = 1.0 / std::max(1.0, publish_rate_);
        pub_timer_ = nh_.createTimer(ros::Duration(timer_dt), &TrackingTargetRelay::timerCallback, this);

        ROS_INFO("Tracking target relay: odom %s -> %s, traj %s -> %s.",
                 target_odom_topic_.c_str(),
                 tracking_target_odom_topic_.c_str(),
                 target_traj_topic_.c_str(),
                 tracking_prediction_topic_.c_str());
    }

  private:
    void odomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_odom_ = *msg;
        latest_odom_time_ = ros::Time::now();
        has_odom_ = true;
    }

    void trajCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg)
    {
        Trajectory parsed_traj;
        if (!polynomialMsgToTrajectory(*msg, ros::Time::now(), parsed_traj))
        {
            return;
        }

        nav_msgs::Path target_path;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_traj_ = parsed_traj;
            has_traj_ = true;
            last_traj_id_ = msg->trajectory_id;
            last_traj_time_ = ros::Time::now();
            target_path = buildTrajectoryPathLocked(last_traj_time_);
        }

        if (!target_path.poses.empty())
        {
            target_path_pub_.publish(target_path);
        }
        const ros::Time now = ros::Time::now();
        if (last_traj_log_time_.isZero() ||
            (now - last_traj_log_time_).toSec() >= std::max(0.0, trajectory_log_period_))
        {
            last_traj_log_time_ = now;
            ROS_INFO("Tracking target relay received target trajectory id=%u, pieces=%d, duration=%.3f s.",
                     static_cast<unsigned int>(msg->trajectory_id),
                     parsed_traj.getPieceNum(),
                     parsed_traj.getTotalDuration());
        }
    }

    bool trajectoryUsableLocked(const ros::Time &now) const
    {
        if (!has_traj_ || target_traj_.empty())
        {
            return false;
        }
        const double total_t = target_traj_.getTotalDuration();
        if (total_t <= 1.0e-6)
        {
            return false;
        }
        const double current_t = now.toSec() - target_traj_.start_WT;
        return current_t >= -0.5 && current_t <= total_t + std::max(0.0, trajectory_finish_hold_);
    }

    double latestOdomYawLocked() const
    {
        return has_odom_ ? yawFromQuat(latest_odom_.pose.pose.orientation) : 0.0;
    }

    Eigen::Vector3d latestOdomPositionLocked() const
    {
        if (!has_odom_)
        {
            return Eigen::Vector3d::Zero();
        }
        return Eigen::Vector3d(latest_odom_.pose.pose.position.x,
                               latest_odom_.pose.pose.position.y,
                               latest_odom_.pose.pose.position.z);
    }

    Eigen::Vector3d latestOdomVelocityLocked() const
    {
        if (!has_odom_)
        {
            return Eigen::Vector3d::Zero();
        }
        return Eigen::Vector3d(latest_odom_.twist.twist.linear.x,
                               latest_odom_.twist.twist.linear.y,
                               latest_odom_.twist.twist.linear.z);
    }

    nav_msgs::Path buildTrajectoryPathLocked(const ros::Time &stamp) const
    {
        nav_msgs::Path path;
        path.header.frame_id = frame_id_;
        path.header.stamp = stamp;
        if (!has_traj_ || target_traj_.empty())
        {
            return path;
        }

        const double total_t = target_traj_.getTotalDuration();
        const double dt = std::max(0.02, path_dt_);
        const int sample_num = std::max(2, static_cast<int>(std::ceil(total_t / dt)) + 1);
        path.poses.reserve(static_cast<std::size_t>(sample_num));
        const double fallback_yaw = latestOdomYawLocked();
        for (int i = 0; i < sample_num; ++i)
        {
            const double t = std::min(total_t, static_cast<double>(i) * dt);
            const Eigen::Vector3d pos = target_traj_.getPos(t);
            const Eigen::Vector3d vel = target_traj_.getVel(t);
            path.poses.emplace_back(makePose(frame_id_, stamp + ros::Duration(t), pos, vel, fallback_yaw));
        }
        return path;
    }

    nav_msgs::Path buildPredictionLocked(const ros::Time &stamp) const
    {
        nav_msgs::Path prediction;
        prediction.header.frame_id = frame_id_;
        prediction.header.stamp = stamp;

        const double dt = std::max(0.05, prediction_dt_);
        const double horizon = std::max(dt, prediction_horizon_);
        const int sample_num = std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1);
        prediction.poses.reserve(static_cast<std::size_t>(sample_num));
        const double fallback_yaw = latestOdomYawLocked();

        if (trajectoryUsableLocked(stamp))
        {
            const double total_t = target_traj_.getTotalDuration();
            const double now_traj_t = stamp.toSec() - target_traj_.start_WT;
            for (int i = 0; i < sample_num; ++i)
            {
                const double future_dt = static_cast<double>(i) * dt;
                const double sample_t = std::clamp(now_traj_t + future_dt, 0.0, total_t);
                const Eigen::Vector3d pos = target_traj_.getPos(sample_t);
                const Eigen::Vector3d vel = target_traj_.getVel(sample_t);
                prediction.poses.emplace_back(
                    makePose(frame_id_, stamp + ros::Duration(future_dt), pos, vel, fallback_yaw));
            }
            return prediction;
        }

        if (!has_odom_ || (!publish_before_trajectory_ && !has_traj_))
        {
            return prediction;
        }

        const Eigen::Vector3d p0 = latestOdomPositionLocked();
        const Eigen::Vector3d v0 = latestOdomVelocityLocked();
        for (int i = 0; i < sample_num; ++i)
        {
            const double future_dt = static_cast<double>(i) * dt;
            prediction.poses.emplace_back(
                makePose(frame_id_, stamp + ros::Duration(future_dt), p0 + v0 * future_dt, v0, fallback_yaw));
        }
        return prediction;
    }

    nav_msgs::Odometry buildTargetOdomLocked(const ros::Time &stamp) const
    {
        nav_msgs::Odometry odom;
        if (has_odom_ && (publish_before_trajectory_ || has_traj_))
        {
            odom = latest_odom_;
            odom.header.stamp = stamp;
            odom.header.frame_id = frame_id_;
            odom.child_frame_id = target_child_frame_id_;
            return odom;
        }

        if (trajectoryUsableLocked(stamp))
        {
            const double total_t = target_traj_.getTotalDuration();
            const double sample_t = std::clamp(stamp.toSec() - target_traj_.start_WT, 0.0, total_t);
            const Eigen::Vector3d pos = target_traj_.getPos(sample_t);
            const Eigen::Vector3d vel = target_traj_.getVel(sample_t);
            odom.header.frame_id = frame_id_;
            odom.header.stamp = stamp;
            odom.child_frame_id = target_child_frame_id_;
            odom.pose.pose.position.x = pos.x();
            odom.pose.pose.position.y = pos.y();
            odom.pose.pose.position.z = pos.z();
            odom.pose.pose.orientation = quatFromYaw(yawFromVelocity(vel, 0.0));
            odom.twist.twist.linear.x = vel.x();
            odom.twist.twist.linear.y = vel.y();
            odom.twist.twist.linear.z = vel.z();
        }
        return odom;
    }

    void timerCallback(const ros::TimerEvent &)
    {
        nav_msgs::Odometry target_odom;
        nav_msgs::Path prediction;
        bool publish_odom = false;
        bool publish_prediction = false;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            const ros::Time stamp = ros::Time::now();
            publish_odom = (has_odom_ && (publish_before_trajectory_ || has_traj_)) ||
                           trajectoryUsableLocked(stamp);
            if (publish_odom)
            {
                target_odom = buildTargetOdomLocked(stamp);
            }
            const double prediction_interval =
                prediction_publish_rate_ > 1.0e-6 ? 1.0 / prediction_publish_rate_ : 0.0;
            if (last_prediction_pub_time_.isZero() ||
                prediction_interval <= 0.0 ||
                (stamp - last_prediction_pub_time_).toSec() >= prediction_interval)
            {
                prediction = buildPredictionLocked(stamp);
                publish_prediction = prediction.poses.size() >= 2;
                if (publish_prediction)
                {
                    last_prediction_pub_time_ = stamp;
                }
            }
        }

        if (publish_odom)
        {
            target_odom_pub_.publish(target_odom);
        }
        if (publish_prediction)
        {
            prediction_pub_.publish(prediction);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber traj_sub_;
    ros::Publisher target_odom_pub_;
    ros::Publisher prediction_pub_;
    ros::Publisher target_path_pub_;
    ros::Timer pub_timer_;

    mutable std::mutex mutex_;
    nav_msgs::Odometry latest_odom_;
    ros::Time latest_odom_time_;
    Trajectory target_traj_;
    bool has_odom_{false};
    bool has_traj_{false};
    uint32_t last_traj_id_{0};
    ros::Time last_traj_time_;
    ros::Time last_traj_log_time_;
    ros::Time last_prediction_pub_time_;

    std::string target_odom_topic_;
    std::string target_traj_topic_;
    std::string tracking_target_odom_topic_;
    std::string tracking_prediction_topic_;
    std::string tracking_target_path_topic_;
    std::string frame_id_;
    std::string target_child_frame_id_;
    double prediction_horizon_{4.0};
    double prediction_dt_{0.25};
    double path_dt_{0.1};
    double publish_rate_{30.0};
    double prediction_publish_rate_{8.0};
    double trajectory_finish_hold_{1.0};
    double trajectory_log_period_{1.0};
    bool publish_before_trajectory_{false};
};
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_target_relay");
    ros::NodeHandle nh("~");
    TrackingTargetRelay relay(nh);
    ros::spin();
    return 0;
}
