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
  int stationary_failure_refreshes_;
  bool caution_force_relocation_;
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
  ros::Time caution_last_stop_request_time_;
  ros::Time caution_last_recovery_attempt_time_;
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
  bool adaptive_tight_path_horizon_enable_;
  double tight_path_turn_threshold_;
  double tight_path_min_horizon_;
  int finish_no_frontier_min_count_;
  double finish_no_frontier_min_duration_;
  bool finish_require_vehicle_slow_;
  double finish_slow_speed_;
  bool finish_recheck_after_goal_reached_;
  bool auto_trigger_enable_;
  double auto_trigger_delay_;
  string trigger_topic_;
  string target_goal_topic_;
  bool trigger_goal_sets_target_;
  bool target_goal_start_on_receive_;
  string legacy_trigger_topic_;
  double global_path_update_min_interval_;
  int cloud_subscriber_queue_;
  int odom_subscriber_queue_;
  int sync_queue_;
  string cloud_odom_mode_;
  double latest_odom_timeout_;
  double max_cloud_age_;
  double reorient_exit_speed_;
  double reorient_timeout_;
  double reorient_stop_retry_interval_;
  double caution_stop_retry_interval_;
  double caution_recovery_retry_interval_;
  double max_odom_age_;
  bool controlled_reorientation_enable_;
  double controlled_stop_min_speed_;
  double handover_slow_speed_;
  double stationary_hold_retry_interval_;
  double plan_failure_retry_delay_;
  int plan_failure_refresh_count_;
  int stationary_relocation_refresh_count_;
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
  bool locked_goal_is_coverage_{false};
  bool locked_goal_is_mission_{false};
  int locked_goal_cluster_id_{-1};
  std::uint64_t locked_goal_coverage_id_{0};
  Eigen::Vector3f locked_goal_{Eigen::Vector3f::Zero()};
  double locked_goal_yaw_{0.0};
  ros::Time locked_goal_time_;
  double locked_goal_cost_{0.0};

  // The mission target outlives individual frontier/viewpoint selections.
  // In target-directed mode it is only used to rank safe frontier candidates
  // until the endpoint itself has been observed and can be inserted safely.
  bool has_mission_goal_{false};
  Eigen::Vector3f mission_goal_{Eigen::Vector3f::Zero()};
  Eigen::Vector3f mission_start_{Eigen::Vector3f::Zero()};
  ros::Time mission_goal_time_;
  bool mission_goal_needs_initialization_{false};

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
  bool epic_simple_global_cost_;
  bool composite_candidate_cost_enable_;
  double candidate_travel_weight_;
  double candidate_turn_brake_weight_;
  double candidate_future_return_weight_;
  double candidate_information_gain_weight_;
  double candidate_wait_weight_;
  double candidate_debt_weight_;
  double candidate_gain_saturation_;
  double candidate_wait_saturation_;
  double candidate_debt_saturation_;
  double candidate_return_cost_cap_;
  int candidate_return_horizon_;
  double frontier_pass_radius_;
  double frontier_pass_exit_margin_;
  double frontier_pass_cooldown_;
  double frontier_pass_debt_increment_;
  double frontier_pass_debt_max_;
  double failed_goal_cooldown_;
  double failed_goal_penalty_;
  bool target_directed_mode_{false};
  bool target_goal_use_message_z_{false};
  double target_heuristic_weight_{3.0};
  double target_lateral_weight_{0.15};
  double target_vertical_weight_{1.0};
  double target_frontier_min_progress_{0.25};
  double target_reached_radius_{0.50};
  double target_direct_retry_delay_{8.0};
  int target_cluster_shortlist_{8};
  double target_goal_lock_remaining_margin_{1.0};
  // Persistent MapManager topology is an advisory long-range layer.  The
  // local Bubble-Topo / safety map remains the only execution authority.
  bool target_topology_guidance_enable_{true};
  double target_topology_query_interval_{1.0};
  double target_topology_local_prefix_length_{12.0};
  double target_topology_goal_change_tolerance_{0.5};
  double target_topology_anchor_min_progress_{0.5};
  double target_topology_anchor_progress_weight_{1.0};
  double target_topology_anchor_lateral_penalty_{0.15};
  double target_topology_anchor_radial_penalty_{0.10};
  double target_topology_anchor_remaining_penalty_{0.05};
  double target_topology_anchor_expansion_bonus_{2.0};
  int target_topology_anchor_candidate_count_{8};
  int target_topology_anchor_query_attempts_{4};
  double target_topology_anchor_min_route_length_{1.0};
  double target_topology_frontier_bias_{0.80};
  bool target_topology_direct_prefix_enable_{true};
  // Local preflight may reject a bridge that topology still scores as
  // reachable.  Defer only after repeated failures on the same cluster so a
  // single narrow-map cycle cannot empty the shortlist.
  int target_local_preflight_fail_limit_{3};
  // Extra detour bridges kept beside progressing ones as an escape hatch.
  int target_progress_detour_budget_{3};
  // Preflight checks a short executable step, not the full cruise horizon.
  double target_preflight_horizon_{8.0};
  // Soft cooldown applied to every bridge that failed a preflight pass so the
  // planner does not reselect the same dead shortlist at FSM rate.
  double target_preflight_soft_defer_{1.5};
  // Minimum wait after the shortlist is fully deferred before unlocking the
  // oldest cooled bridge (never unlock the whole set at once).
  double target_empty_pool_escape_wait_{2.5};
  int target_empty_pool_unlock_count_{1};
  // Detours weaker than this progress (metres toward the mission) are never
  // committed.  Prevents large backward flights when the forward map is thin.
  double target_escape_min_progress_{-1.0};
  // Inside this remaining distance, only non-regressing bridges may be used.
  double target_near_goal_remaining_{15.0};
  // A target task may wait briefly for the local map to expose a bridge, but
  // it must not retain command ownership and rebuild topology indefinitely
  // when the vehicle is already stationary.  Expiry reports BLOCKED while
  // preserving the accumulated map for the next target.
  double target_no_progress_timeout_{12.0};
  // Cluster IDs churn; blacklist failed bridge centers by geometry.
  double target_spatial_blacklist_radius_{2.5};
  double target_spatial_blacklist_duration_{8.0};
  bool use_lkh_;
  bool view_graph_;
  string tsp_dir_; // Per-process writable directory used by the LKH solver.
};

} // namespace fast_planner
