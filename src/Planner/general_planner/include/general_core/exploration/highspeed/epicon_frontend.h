#pragma once
#include <Eigen/Core>
#include <memory>
#include <cstdint>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

namespace fast_planner {
// No occupancy-map or General Planner goal-selection types cross this boundary.
class EpiconFrontend {
public:
  enum class Result { WAITING_MAP, NO_FRONTIER, BLOCKED, DISCONNECTED, FAIL, SUCCEED };
  EpiconFrontend();
  ~EpiconFrontend();
  void init(ros::NodeHandle &runtime_nh);
  void ingestCloud(const sensor_msgs::PointCloud2ConstPtr &cloud,
                   const nav_msgs::OdometryConstPtr &odom);
  void setOdometry(const Eigen::Vector3f &position, const Eigen::Vector3f &velocity, float yaw);
  void resetTask();
  Result update(bool select_goal, bool full_audit = false);
  void deferCurrentGoal(double cooldown_seconds, double radius);
  bool hasSelectedGoal() const;
  bool lastUpdateAudited() const;
  std::uint64_t cloudRevision() const;
  ros::Time cloudStamp() const;
  Result pathToGoal(std::vector<Eigen::Vector3f> &path);
  const std::vector<Eigen::Vector3f> &tour() const;
  double goalYaw() const;
  bool ready() const;
  bool hasCloud() const;
  std::size_t clusterCount() const;
  std::size_t historyCount() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
