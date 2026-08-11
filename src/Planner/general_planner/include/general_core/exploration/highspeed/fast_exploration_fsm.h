/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-21 21:31:51
 * @LastEditTime: 2024-03-06 10:28:56
 * @Description:
 * @
 * @Copyright (c) 2023 by ning-zelin, All Rights Reserved.
 */

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdint>
#include <geometry_msgs/PoseStamped.h>
#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <iostream>
#include <mutex>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <general_core/exploration/exploration_utils/pointcloud_topo/graph_visualizer.hpp>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE {
  INIT,
  WAIT_TRIGGER,
  PLAN_TRAJ,
  CAUTION,
  EXEC_TRAJ,
  REORIENT,
  PAUSING,
  PAUSED,
  FINISH,
  LAND
};

struct FinishGate {
  int no_frontier_count{0};
  ros::Time first_no_frontier_time;
  ros::Time last_force_recheck_time;
  int no_reachable_count{0};
  bool force_recheck_requested{false};
};

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;
  FinishGate finish_gate_;
  ros::Time last_plan_traj_global_update_time_;
  ros::Time last_near_stationary_hold_time_;
  int last_near_stationary_hold_traj_id_{-1};
  ros::WallTime last_global_callback_wall_time_;
  ros::Time last_topology_update_time_;
  ros::Time last_historical_topology_update_time_;
  Eigen::Vector3f last_historical_topology_update_pos_{
      Eigen::Vector3f::Zero()};
  bool historical_topology_update_initialized_{false};
  std::uint64_t topology_map_revision_{0};
  std::uint64_t topology_applied_revision_{0};
  bool topology_revision_applied_{false};

  bool classic_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, global_path_update_timer_;
  ros::Subscriber trigger_sub_, nav_goal_trigger_sub_, map_update_sub_,
      battary_sub_, raw_odom_sub_, latest_cloud_sub_, task_command_sub_;
  ros::Publisher stop_pub_, new_pub_, replan_pub_, poly_traj_pub_, heartbeat_pub_, time_cost_pub_, poly_yaw_traj_pub_, static_pub_, state_pub_,
  speed_pub_, land_pub_, task_status_pub_, execution_enabled_pub_;
  bool task_control_enable_{false};
  bool pause_stop_issued_{false};
  bool completion_pending_{false};
  bool execution_enabled_published_{false};
  bool last_execution_enabled_{true};
  // In M2 GlobalMapRuntime owns the only cloud/odom subscriptions.  This FSM
  // still receives every accepted frame, but never fuses LIO/ROG itself.
  bool external_sensor_ingress_{false};
  std::string active_task_id_;
  std::string task_command_topic_;
  std::string task_status_topic_;
  std::string execution_enabled_topic_;
  double handover_slow_speed_{0.10};
  double total_time_;

  /*cloud odom callback*/
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  SynchronizerCloudOdom sync_cloud_odom_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  nav_msgs::OdometryConstPtr latest_odom_msg_;
  ros::WallTime latest_odom_receive_wall_time_;
  std::mutex latest_odom_mutex_;
  void CloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &msg, const nav_msgs::Odometry::ConstPtr &odom_);
  void latestCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call, bool red = false);
  void battaryCallback(const sensor_msgs::BatteryStateConstPtr &msg);
  void resetFinishGate(const string &reason);
  void requestFrontierRecheck(const string &reason);
  bool finishGateSatisfied(const string &reason) const;
  void handleNoFrontierResult(const string &source);
  bool handleGoalReached();
  bool trajectoryEnded() const;
  /* ROS functions */
  void FSMCallback(const ros::TimerEvent &e);
  // void PlannerDebugFSMCallback(const ros::TimerEvent &e);
  void safetyCallback(const ros::TimerEvent &e);
  void updateTopoAndGlobalPath();
  void globalPathUpdateCallback(const ros::TimerEvent &e);
  void triggerCallback(const nav_msgs::PathConstPtr &msg);
  void navGoalTriggerCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void taskCommandCallback(const std_msgs::StringConstPtr &msg);
  void acceptManualTrigger(const string &source);
  void startExplorationTask(const std::string &task_id,
                            const std::string &source);
  void beginPause(const std::string &reason, bool completed);
  bool handoverSafe() const;
  void publishTaskStatus();
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void stopTraj(const string &reason);

  // void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void visualize();
  void pubState();

public:
  FastExplorationFSM(/* args */) {}

  ~FastExplorationFSM();

  void init(ros::NodeHandle &nh, FastExplorationManager::Ptr &explorer,
            bool external_sensor_ingress = false);

  /** Feed one already-accepted global-map frame (M2 composed runtime). */
  void ingestCloudOdom(const sensor_msgs::PointCloud2ConstPtr &msg,
                       const nav_msgs::OdometryConstPtr &odom);
  /** Feed raw odometry while GlobalMapRuntime owns the ROS subscription. */
  void ingestOdometry(const nav_msgs::OdometryConstPtr &msg);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner
