#pragma once

#include <Eigen/Eigen>
#include <general_core/exploration/exploration_utils/pointcloud_topo/graph.h>
#include <ros/ros.h>
#include <traj_utils/PolyTraj.h>
#include <vector>
using Eigen::Vector3d;
using std::vector;
using namespace std;

namespace fast_planner {
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_, emergency_replan_,
      use_bubble_a_star_, half_resolution, auto_triggered_;
  bool reorientation_required_, reorientation_stop_requested_;
  int consecutive_plan_failures_;
  vector<string> state_str_;
  int bb_astar_fail_cnt_, fast_search_fial_cnt_;
  double bb_astar_time_out, fast_search_time_out;
  Eigen::Vector3f odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;
  float odom_yaw_;
  ros::Time first_odom_time_;
  ros::Time last_odom_receive_time_;
  ros::Time reorientation_start_time_;
  ros::Time reorientation_last_stop_request_time_;
  ros::Time next_plan_retry_time_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
  vector<Eigen::Vector3d> start_poss;
  traj_utils::PolyTraj newest_traj_;
  traj_utils::PolyTraj newest_yaw_traj_;
};

struct FSMParam {
  double replan_thresh_;
  double replan_time_after_traj_start_;
  double replan_time_before_traj_end_;
  double replan_time_; // second
  double emergency_replan_control_error;
  double bubble_a_star_resolution;
  double path_densify_step_;
  int finish_no_frontier_min_count_;
  double finish_no_frontier_min_duration_;
  bool finish_require_vehicle_slow_;
  double finish_slow_speed_;
  bool finish_recheck_after_goal_reached_;
  bool auto_trigger_enable_;
  double auto_trigger_delay_;
  double global_path_update_min_interval_;
  int cloud_subscriber_queue_;
  int odom_subscriber_queue_;
  int sync_queue_;
  double max_cloud_age_;
  double reorient_exit_speed_;
  double reorient_timeout_;
  double reorient_stop_retry_interval_;
  double max_odom_age_;
  bool controlled_reorientation_enable_;
  double plan_failure_retry_delay_;
  int plan_failure_refresh_count_;
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<TopoNode::Ptr> local_tour_;
  vector<Eigen::Vector3f> global_tour_;
  TopoNode::Ptr first_root_, second_root_;

  Eigen::Vector3f next_goal_;
  TopoNode::Ptr next_goal_node_;
  vector<Eigen::Vector3f> path_next_goal_;
  bool has_goal_lock_{false};
  int locked_goal_cluster_id_{-1};
  Eigen::Vector3f locked_goal_{Eigen::Vector3f::Zero()};
  double locked_goal_yaw_{0.0};
  ros::Time locked_goal_time_;
  double locked_goal_cost_{0.0};

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;
  Eigen::Vector3f tsp_end_node_;
};

struct ExplorationParam {
  // params
  int local_viewpoint_num_, global_viewpoint_num_;
  int viewpoint_connection_num_;
  double a_avg_, v_max_, yaw_v_max_, viewpoint_gian_lambda_;
  double w_vdir_, w_yawdir_;
  bool goal_lock_enable_;
  double goal_switch_min_interval_;
  double goal_switch_min_improvement_;
  double goal_switch_high_speed_multiplier_;
  double goal_keep_cost_ratio_;
  double goal_lock_match_radius_;
  double goal_reached_radius_;
  bool original_frontend_compatibility_;
  bool use_lkh_;
  bool view_graph_;
  string tsp_dir_; // Per-process writable directory used by the LKH solver.
};

} // namespace fast_planner
