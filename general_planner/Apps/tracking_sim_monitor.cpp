#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace
{

Eigen::Vector3d positionOf(const nav_msgs::Odometry &odom)
{
    return Eigen::Vector3d(odom.pose.pose.position.x,
                           odom.pose.pose.position.y,
                           odom.pose.pose.position.z);
}

Eigen::Vector3d velocityOf(const nav_msgs::Odometry &odom)
{
    return Eigen::Vector3d(odom.twist.twist.linear.x,
                           odom.twist.twist.linear.y,
                           odom.twist.twist.linear.z);
}

class TrackingSimMonitor
{
public:
    explicit TrackingSimMonitor(const ros::NodeHandle &nh)
        : nh_(nh)
    {
        nh_.param<std::string>("drone_odom_topic", drone_odom_topic_, "/lidar_slam/odom");
        nh_.param<std::string>("target_odom_topic", target_odom_topic_, "/tracking/target_odom");
        nh_.param<double>("desired_distance", desired_distance_, 2.2);
        nh_.param<double>("distance_tolerance", distance_tolerance_, 0.8);
        nh_.param<double>("height_offset", height_offset_, 0.7);
        nh_.param<double>("warmup_time", warmup_time_, 3.0);
        nh_.param<double>("sample_rate", sample_rate_, 20.0);
        nh_.param<std::string>("log_path", log_path_, std::string(ROOT_DIR) + "log/tracking_monitor_latest.csv");

        openLog();

        drone_sub_ = nh_.subscribe(drone_odom_topic_, 20, &TrackingSimMonitor::droneCallback, this);
        target_sub_ = nh_.subscribe(target_odom_topic_, 20, &TrackingSimMonitor::targetCallback, this);
        sample_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, sample_rate_)),
                                        &TrackingSimMonitor::sampleTimerCallback,
                                        this);
        start_time_ = ros::Time::now();
        ROS_INFO("Tracking sim monitor: drone=%s target=%s desired_distance=%.3f height_offset=%.3f log=%s",
                 drone_odom_topic_.c_str(),
                 target_odom_topic_.c_str(),
                 desired_distance_,
                 height_offset_,
                 log_path_.c_str());
    }

    void printSummary() const
    {
        if (sample_count_ == 0)
        {
            ROS_WARN("Tracking sim monitor summary: no valid samples after %.2f s warmup.", warmup_time_);
            return;
        }

        ROS_INFO("Tracking sim monitor summary: samples=%zu avg_dist_err=%.3f max_dist_err=%.3f "
                 "avg_height_err=%.3f max_height_err=%.3f min_horizontal_dist=%.3f max_horizontal_dist=%.3f",
                 sample_count_,
                 sum_dist_error_ / static_cast<double>(sample_count_),
                 max_dist_error_,
                 sum_height_error_ / static_cast<double>(sample_count_),
                 max_height_error_,
                 min_horizontal_distance_,
                 max_horizontal_distance_);
    }

private:
    void openLog()
    {
        if (log_path_.empty())
        {
            return;
        }
        log_.open(log_path_, std::ios::out | std::ios::trunc);
        if (!log_.good())
        {
            ROS_WARN("Tracking sim monitor cannot open log file: %s", log_path_.c_str());
            return;
        }
        log_ << "time,elapsed,drone_x,drone_y,drone_z,target_x,target_y,target_z,"
                "drone_speed,target_speed,horizontal_dist,dist_error,height_error,"
                "closing_speed,dist_rate,rel_vx,rel_vy,rel_vz,in_distance_band,moving_closer\n";
    }

    void droneCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        latest_drone_ = *msg;
        has_drone_ = true;
    }

    void targetCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        latest_target_ = *msg;
        has_target_ = true;
    }

    void sampleTimerCallback(const ros::TimerEvent &)
    {
        if (!has_drone_ || !has_target_)
        {
            return;
        }
        const double elapsed = (ros::Time::now() - start_time_).toSec();
        if (elapsed < warmup_time_)
        {
            return;
        }

        const Eigen::Vector3d drone_p = positionOf(latest_drone_);
        const Eigen::Vector3d target_p = positionOf(latest_target_);
        const Eigen::Vector3d drone_v = velocityOf(latest_drone_);
        const Eigen::Vector3d target_v = velocityOf(latest_target_);
        const Eigen::Vector2d rel_xy = (drone_p - target_p).head<2>();
        const double horizontal_dist = rel_xy.norm();
        const double dist_error = std::abs(horizontal_dist - desired_distance_);
        const double height_error = std::abs((drone_p.z() - target_p.z()) - height_offset_);
        const Eigen::Vector3d rel_v = drone_v - target_v;
        double dist_rate = 0.0;
        if (horizontal_dist > 1.0e-6)
        {
            dist_rate = rel_xy.dot(rel_v.head<2>()) / horizontal_dist;
        }
        const double closing_speed = -dist_rate;
        const bool in_distance_band =
            std::abs(horizontal_dist - desired_distance_) <= std::max(0.0, distance_tolerance_);

        ++sample_count_;
        sum_dist_error_ += dist_error;
        max_dist_error_ = std::max(max_dist_error_, dist_error);
        sum_height_error_ += height_error;
        max_height_error_ = std::max(max_height_error_, height_error);
        min_horizontal_distance_ = std::min(min_horizontal_distance_, horizontal_dist);
        max_horizontal_distance_ = std::max(max_horizontal_distance_, horizontal_dist);

        if (log_.good())
        {
            log_ << std::fixed << std::setprecision(6)
                 << ros::Time::now().toSec() << ','
                 << elapsed << ','
                 << drone_p.x() << ',' << drone_p.y() << ',' << drone_p.z() << ','
                 << target_p.x() << ',' << target_p.y() << ',' << target_p.z() << ','
                 << drone_v.norm() << ','
                 << target_v.norm() << ','
                 << horizontal_dist << ','
                 << dist_error << ','
                 << height_error << ','
                 << closing_speed << ','
                 << dist_rate << ','
                 << rel_v.x() << ',' << rel_v.y() << ',' << rel_v.z() << ','
                 << (in_distance_band ? 1 : 0) << ','
                 << (closing_speed > 0.0 ? 1 : 0) << '\n';
        }

        ROS_INFO_STREAM_THROTTLE(2.0, "Tracking monitor: horizontal_dist=" << horizontal_dist
                                      << ", dist_err=" << dist_error
                                      << ", height_err=" << height_error
                                      << ", closing_speed=" << closing_speed
                                      << ", samples=" << sample_count_);
        ROS_INFO_STREAM_THROTTLE(5.0, "Tracking sim monitor running summary: samples=" << sample_count_
                                      << ", avg_dist_err="
                                      << sum_dist_error_ / static_cast<double>(sample_count_)
                                      << ", max_dist_err=" << max_dist_error_
                                      << ", avg_height_err="
                                      << sum_height_error_ / static_cast<double>(sample_count_)
                                      << ", max_height_err=" << max_height_error_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber drone_sub_;
    ros::Subscriber target_sub_;
    ros::Timer sample_timer_;
    nav_msgs::Odometry latest_drone_;
    nav_msgs::Odometry latest_target_;
    ros::Time start_time_;

    std::string drone_odom_topic_;
    std::string target_odom_topic_;
    std::string log_path_;
    std::ofstream log_;
    double desired_distance_{2.2};
    double distance_tolerance_{0.8};
    double height_offset_{0.7};
    double warmup_time_{3.0};
    double sample_rate_{20.0};
    bool has_drone_{false};
    bool has_target_{false};

    std::size_t sample_count_{0};
    double sum_dist_error_{0.0};
    double max_dist_error_{0.0};
    double sum_height_error_{0.0};
    double max_height_error_{0.0};
    double min_horizontal_distance_{std::numeric_limits<double>::infinity()};
    double max_horizontal_distance_{0.0};
};

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_sim_monitor");
    ros::NodeHandle nh("~");
    TrackingSimMonitor monitor(nh);
    ros::spin();
    monitor.printSummary();
    return 0;
}
