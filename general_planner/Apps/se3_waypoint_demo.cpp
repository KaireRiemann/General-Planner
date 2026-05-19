#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>

#include "data_structure/base/trajectory.h"
#include "traj_opt/flatness/se3_flatness_map.hpp"

namespace {

struct Waypoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

double normalizeAngle(double a) {
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

Eigen::Quaterniond quatFromRpy(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  return Eigen::Quaterniond(rz * ry * rx);
}

Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond &q) {
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  Eigen::Vector3d rpy;
  rpy.x() = std::atan2(R(2, 1), R(2, 2));
  rpy.y() = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  rpy.z() = std::atan2(R(1, 0), R(0, 0));
  return rpy;
}

geometry_msgs::Quaternion toMsg(const Eigen::Quaterniond &q) {
  geometry_msgs::Quaternion msg;
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

geometry_msgs::Point toPoint(const Eigen::Vector3d &p) {
  geometry_msgs::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  return point;
}

std_msgs::ColorRGBA rgba(double r, double g, double b, double a) {
  std_msgs::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = static_cast<float>(a);
  return color;
}

bool xmlToDouble(const XmlRpc::XmlRpcValue &value, double &out) {
  try {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      out = static_cast<double>(value);
      return true;
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
      out = static_cast<int>(value);
      return true;
    }
    if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
      out = std::stod(static_cast<std::string>(value));
      return true;
    }
  } catch (const std::exception &) {
  }
  return false;
}

double memberDouble(const XmlRpc::XmlRpcValue &value,
                    const std::string &key,
                    double fallback) {
  if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      !value.hasMember(key)) {
    return fallback;
  }
  double out = fallback;
  return xmlToDouble(value[key], out) ? out : fallback;
}

bool polynomialMsgToTrajectory(const quadrotor_msgs::PolynomialTrajectory &msg,
                               const ros::Time &receive_time,
                               geometry_utils::Trajectory &traj) {
  if ((msg.type & quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ) == 0 ||
      msg.piece_num_pos <= 0 || msg.order_pos < 1) {
    return false;
  }

  const int piece_num = static_cast<int>(msg.piece_num_pos);
  const int coeff_num = static_cast<int>(msg.order_pos) + 1;
  if (static_cast<int>(msg.time_pos.size()) < piece_num ||
      static_cast<int>(msg.coef_pos_x.size()) < piece_num * coeff_num ||
      static_cast<int>(msg.coef_pos_y.size()) < piece_num * coeff_num ||
      static_cast<int>(msg.coef_pos_z.size()) < piece_num * coeff_num) {
    return false;
  }

  traj.clear();
  traj.reserve(piece_num);
  for (int i = 0; i < piece_num; ++i) {
    const double duration = msg.time_pos[static_cast<std::size_t>(i)];
    if (duration <= 1.0e-6) {
      traj.clear();
      return false;
    }
    Eigen::MatrixXd coeff(3, coeff_num);
    for (int j = 0; j < coeff_num; ++j) {
      const int offset = i * coeff_num + j;
      coeff(0, j) = msg.coef_pos_x[static_cast<std::size_t>(offset)];
      coeff(1, j) = msg.coef_pos_y[static_cast<std::size_t>(offset)];
      coeff(2, j) = msg.coef_pos_z[static_cast<std::size_t>(offset)];
    }
    traj.emplace_back(duration, coeff);
  }

  traj.start_WT = msg.start_WT_pos.toSec();
  if (traj.start_WT <= 0.0 && !msg.header.stamp.isZero()) {
    traj.start_WT = msg.header.stamp.toSec();
  }
  if (traj.start_WT <= 0.0) {
    traj.start_WT = receive_time.toSec();
  }
  return !traj.empty();
}

class SE3WaypointDemo {
public:
  explicit SE3WaypointDemo(const ros::NodeHandle &nh) : nh_(nh) {
    nh_.param<std::string>("frame_id", frame_id_, "world");
    nh_.param<std::string>("odom_topic", odom_topic_, "/se3_demo/odom");
    nh_.param<std::string>("goal_topic", goal_topic_, "/se3_demo/goal");
    nh_.param<std::string>("traj_topic", traj_topic_, "/se3_demo/poly_traj");
    nh_.param<std::string>("cloud_topic", cloud_topic_, "/se3_demo/cloud");
    nh_.param<std::string>("marker_topic", marker_topic_, "/se3_demo/markers");
    nh_.param<double>("gravity", gravity_, 9.81);
    nh_.param<double>("start_delay", start_delay_, 2.0);
    nh_.param<double>("odom_rate", odom_rate_, 100.0);
    nh_.param<double>("goal_check_rate", goal_check_rate_, 10.0);
    nh_.param<double>("goal_reissue_period", goal_reissue_period_, 1.0);
    nh_.param<double>("advance_distance", advance_distance_, 0.25);
    nh_.param<double>("final_distance", final_distance_, 0.08);
    nh_.param<double>("settle_time", settle_time_, 0.3);
    nh_.param<bool>("advance_on_traj_finish", advance_on_traj_finish_, true);
    nh_.param<bool>("publish_empty_cloud", publish_empty_cloud_, true);

    if (!loadWaypoints()) {
      makeDefaultWaypoints();
    }
    if (waypoints_.size() < 2) {
      ROS_ERROR("SE3 waypoint demo needs at least two waypoints.");
      valid_ = false;
      return;
    }

    current_p_ = waypoints_.front().p;
    current_q_ = quatFromRpy(waypoints_.front().roll,
                             waypoints_.front().pitch,
                             waypoints_.front().yaw);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    if (publish_empty_cloud_) {
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 1);
    }
    traj_sub_ = nh_.subscribe(traj_topic_, 10, &SE3WaypointDemo::trajCallback, this);

    odom_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, odom_rate_)),
                                  &SE3WaypointDemo::odomTimer, this);
    goal_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, goal_check_rate_)),
                                  &SE3WaypointDemo::goalTimer, this);
    marker_timer_ = nh_.createTimer(ros::Duration(0.5),
                                    &SE3WaypointDemo::markerTimer, this);

    start_time_ = ros::Time::now();
    publishMarkers();
    ROS_INFO("SE3_WAYPOINT_DEMO_READY waypoints=%zu odom=%s goal=%s traj=%s",
             waypoints_.size(),
             odom_topic_.c_str(),
             goal_topic_.c_str(),
             traj_topic_.c_str());
  }

  bool valid() const { return valid_; }

private:
  bool loadWaypoints() {
    XmlRpc::XmlRpcValue list;
    if (!nh_.getParam("waypoints", list) ||
        list.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        list.size() <= 0) {
      return false;
    }

    waypoints_.clear();
    for (int i = 0; i < list.size(); ++i) {
      const auto &item = list[i];
      if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        continue;
      }
      Waypoint wp;
      wp.p.x() = memberDouble(item, "x", 0.0);
      wp.p.y() = memberDouble(item, "y", 0.0);
      wp.p.z() = memberDouble(item, "z", 1.0);
      wp.roll = memberDouble(item, "roll", 0.0);
      wp.pitch = memberDouble(item, "pitch", 0.0);
      wp.yaw = memberDouble(item, "yaw", 0.0);
      waypoints_.push_back(wp);
    }
    return waypoints_.size() >= 2;
  }

  void makeDefaultWaypoints() {
    waypoints_.clear();
    waypoints_.push_back({Eigen::Vector3d(0.0, 0.0, 1.2), 0.0, 0.0, 0.0});
    waypoints_.push_back({Eigen::Vector3d(2.0, 0.0, 1.4), 0.0, 0.0, 0.0});
    waypoints_.push_back({Eigen::Vector3d(4.0, 1.0, 1.8), 0.0, 0.2, 0.65});
    waypoints_.push_back({Eigen::Vector3d(6.0, -0.8, 1.2), 0.25, 0.0, -0.75});
    waypoints_.push_back({Eigen::Vector3d(8.0, 0.0, 1.5), 0.0, 0.0, 0.0});
  }

  void publishGoal(int idx, bool announce) {
    if (idx <= 0 || idx >= static_cast<int>(waypoints_.size())) {
      return;
    }
    current_goal_idx_ = idx;
    last_goal_pub_time_ = ros::Time::now();

    const Waypoint &wp = waypoints_[static_cast<std::size_t>(idx)];
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = frame_id_;
    goal.header.stamp = last_goal_pub_time_;
    goal.pose.position = toPoint(wp.p);
    goal.pose.orientation = toMsg(quatFromRpy(wp.roll, wp.pitch, wp.yaw));
    goal_pub_.publish(goal);

    if (announce) {
      ROS_INFO("SE3_DEMO_GOAL idx=%d pos=[%.2f %.2f %.2f] rpy=[%.2f %.2f %.2f]deg",
               idx,
               wp.p.x(), wp.p.y(), wp.p.z(),
               wp.roll * 180.0 / M_PI,
               wp.pitch * 180.0 / M_PI,
               wp.yaw * 180.0 / M_PI);
    }
  }

  void trajCallback(const quadrotor_msgs::PolynomialTrajectoryConstPtr &msg) {
    geometry_utils::Trajectory traj;
    if (!polynomialMsgToTrajectory(*msg, ros::Time::now(), traj)) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    active_traj_ = traj;
    active_traj_id_ = static_cast<int>(msg->trajectory_id);
    active_goal_idx_ = current_goal_idx_;
    has_active_traj_ = true;
    ROS_INFO("SE3_DEMO_TRAJ_RECEIVED goal_idx=%d traj_id=%d pieces=%d duration=%.3f",
             active_goal_idx_,
             active_traj_id_,
             active_traj_.getPieceNum(),
             active_traj_.getTotalDuration());
  }

  bool reachedCurrentGoal() const {
    if (current_goal_idx_ <= 0 ||
        current_goal_idx_ >= static_cast<int>(waypoints_.size())) {
      return false;
    }

    const double pos_err =
        (current_p_ - waypoints_[static_cast<std::size_t>(current_goal_idx_)].p).norm();
    const bool final_goal =
        current_goal_idx_ == static_cast<int>(waypoints_.size()) - 1;
    if (pos_err <= (final_goal ? final_distance_ : advance_distance_)) {
      return true;
    }

    if (final_goal || !advance_on_traj_finish_ || !has_active_traj_ ||
        active_goal_idx_ != current_goal_idx_) {
      return false;
    }

    const double elapsed = ros::Time::now().toSec() - active_traj_.start_WT;
    return elapsed >= active_traj_.getTotalDuration() + settle_time_;
  }

  void logReached() const {
    const Waypoint &wp = waypoints_[static_cast<std::size_t>(current_goal_idx_)];
    const Eigen::Quaterniond desired_q = quatFromRpy(wp.roll, wp.pitch, wp.yaw);
    Eigen::Quaterniond dq = desired_q.conjugate() * current_q_;
    dq.normalize();
    double angle = Eigen::AngleAxisd(dq).angle();
    if (angle > M_PI) {
      angle = 2.0 * M_PI - angle;
    }

    const Eigen::Vector3d rpy = rpyFromQuat(current_q_);
    const double yaw_err = normalizeAngle(rpy.z() - wp.yaw);
    ROS_INFO("SE3_DEMO_WAYPOINT_REACHED idx=%d pos_err=%.3f yaw_err=%.2fdeg attitude_err=%.2fdeg actual_rpy=[%.2f %.2f %.2f]deg",
             current_goal_idx_,
             (current_p_ - wp.p).norm(),
             yaw_err * 180.0 / M_PI,
             angle * 180.0 / M_PI,
             rpy.x() * 180.0 / M_PI,
             rpy.y() * 180.0 / M_PI,
             rpy.z() * 180.0 / M_PI);
  }

  void goalTimer(const ros::TimerEvent &) {
    if (!valid_ || done_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    if (!started_) {
      if ((now - start_time_).toSec() >= start_delay_) {
        started_ = true;
        publishGoal(1, true);
      }
      return;
    }

    if (reachedCurrentGoal()) {
      logReached();
      const int next_idx = current_goal_idx_ + 1;
      if (next_idx >= static_cast<int>(waypoints_.size())) {
        ROS_INFO("SE3_WAYPOINT_DEMO_DONE waypoints=%zu", waypoints_.size());
        done_ = true;
        return;
      }
      publishGoal(next_idx, true);
      return;
    }

    const bool has_traj_for_current =
        has_active_traj_ && active_goal_idx_ == current_goal_idx_;
    if (!has_traj_for_current &&
        (now - last_goal_pub_time_).toSec() >= goal_reissue_period_) {
      publishGoal(current_goal_idx_, false);
    }
  }

  void odomTimer(const ros::TimerEvent &) {
    const ros::Time now = ros::Time::now();
    Eigen::Vector3d p = current_p_;
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    Eigen::Vector3d j = Eigen::Vector3d::Zero();
    Eigen::Vector3d s = Eigen::Vector3d::Zero();
    double yaw = waypoints_.front().yaw;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (has_active_traj_) {
        const double total = active_traj_.getTotalDuration();
        const double elapsed = now.toSec() - active_traj_.start_WT;
        const double t = std::clamp(elapsed, 0.0, total);
        p = active_traj_.getPos(t);
        v = active_traj_.getVel(t);
        a = active_traj_.getAcc(t);
        j = active_traj_.getJer(t);
        s = active_traj_.getSnap(t);
        if (active_goal_idx_ > 0 &&
            active_goal_idx_ < static_cast<int>(waypoints_.size())) {
          yaw = waypoints_[static_cast<std::size_t>(active_goal_idx_)].yaw;
        }
      }
    }

    traj_opt::SE3FlatnessMap flatness;
    flatness.setYawMode(true, false);
    traj_opt::SE3FlatnessOutput flat;
    if (flatness.forward(v, a, j, s, yaw, 0.0, gravity_, flat)) {
      current_q_ = Eigen::Quaterniond(flat.R);
      current_q_.normalize();
    }
    current_p_ = p;
    current_v_ = v;

    nav_msgs::Odometry odom;
    odom.header.frame_id = frame_id_;
    odom.header.stamp = now;
    odom.child_frame_id = "se3_demo_body";
    odom.pose.pose.position = toPoint(current_p_);
    odom.pose.pose.orientation = toMsg(current_q_);
    odom.twist.twist.linear.x = current_v_.x();
    odom.twist.twist.linear.y = current_v_.y();
    odom.twist.twist.linear.z = current_v_.z();
    odom_pub_.publish(odom);

    if (publish_empty_cloud_) {
      publishEmptyCloud(now);
    }
  }

  void publishEmptyCloud(const ros::Time &stamp) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = 0;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(0);
    cloud_pub_.publish(cloud);
  }

  void markerTimer(const ros::TimerEvent &) { publishMarkers(); }

  void publishMarkers() {
    visualization_msgs::MarkerArray array;
    const ros::Time stamp = ros::Time::now();

    visualization_msgs::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = stamp;
    line.ns = "se3_waypoints";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.scale.x = 0.04;
    line.color = rgba(0.0, 0.75, 1.0, 0.9);
    for (const auto &wp : waypoints_) {
      line.points.push_back(toPoint(wp.p));
    }
    array.markers.push_back(line);

    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      const Waypoint &wp = waypoints_[i];
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = frame_id_;
      sphere.header.stamp = stamp;
      sphere.ns = "se3_waypoint_points";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::Marker::SPHERE;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position = toPoint(wp.p);
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.18;
      sphere.scale.y = 0.18;
      sphere.scale.z = 0.18;
      sphere.color = i == 0 ? rgba(0.2, 1.0, 0.2, 0.9) : rgba(1.0, 0.8, 0.1, 0.9);
      array.markers.push_back(sphere);

      visualization_msgs::Marker arrow;
      arrow.header.frame_id = frame_id_;
      arrow.header.stamp = stamp;
      arrow.ns = "se3_waypoint_attitudes";
      arrow.id = static_cast<int>(i);
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.pose.position = toPoint(wp.p);
      arrow.pose.orientation = toMsg(quatFromRpy(wp.roll, wp.pitch, wp.yaw));
      arrow.scale.x = 0.55;
      arrow.scale.y = 0.05;
      arrow.scale.z = 0.09;
      arrow.color = rgba(1.0, 0.25, 0.1, 0.9);
      array.markers.push_back(arrow);

      visualization_msgs::Marker text;
      text.header.frame_id = frame_id_;
      text.header.stamp = stamp;
      text.ns = "se3_waypoint_labels";
      text.id = static_cast<int>(i);
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position = toPoint(wp.p + Eigen::Vector3d(0.0, 0.0, 0.28));
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.22;
      text.color = rgba(1.0, 1.0, 1.0, 0.95);
      text.text = "wp" + std::to_string(i);
      array.markers.push_back(text);
    }
    marker_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  bool valid_{true};
  bool started_{false};
  bool done_{false};
  bool has_active_traj_{false};
  bool advance_on_traj_finish_{true};
  bool publish_empty_cloud_{true};
  std::string frame_id_{"world"};
  std::string odom_topic_{"/se3_demo/odom"};
  std::string goal_topic_{"/se3_demo/goal"};
  std::string traj_topic_{"/se3_demo/poly_traj"};
  std::string cloud_topic_{"/se3_demo/cloud"};
  std::string marker_topic_{"/se3_demo/markers"};
  double gravity_{9.81};
  double start_delay_{2.0};
  double odom_rate_{100.0};
  double goal_check_rate_{10.0};
  double goal_reissue_period_{1.0};
  double advance_distance_{0.25};
  double final_distance_{0.08};
  double settle_time_{0.3};
  ros::Time start_time_;
  ros::Time last_goal_pub_time_;
  int current_goal_idx_{0};
  int active_goal_idx_{0};
  int active_traj_id_{-1};
  Eigen::Vector3d current_p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_v_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond current_q_{Eigen::Quaterniond::Identity()};
  std::vector<Waypoint, Eigen::aligned_allocator<Waypoint>> waypoints_;
  geometry_utils::Trajectory active_traj_;
  mutable std::mutex mutex_;
  ros::Publisher odom_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher cloud_pub_;
  ros::Subscriber traj_sub_;
  ros::Timer odom_timer_;
  ros::Timer goal_timer_;
  ros::Timer marker_timer_;
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "se3_waypoint_demo");
  ros::NodeHandle nh("~");
  SE3WaypointDemo demo(nh);
  if (!demo.valid()) {
    return 1;
  }
  ros::spin();
  return 0;
}
