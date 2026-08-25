/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-25 15:00:51
 * @LastEditTime: 2024-03-12 22:15:11
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <general_core/exploration/highspeed/target_directed_exploration.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <general_core/exploration/highspeed/planner_manager.h>
#include <sstream>
#include <system_error>
#include <tf/tf.h>
#include <unordered_set>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
using namespace std;
using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {}

void FastExplorationManager::initialize(
    ros::NodeHandle &nh, FrontierManager::Ptr frt_manager,
    FastPlannerManager::Ptr planner_manager) {

  frontier_manager_ptr_ = frt_manager;
  planner_manager_ = planner_manager;

  // The coordinator is a strict runtime bypass when disabled.  Keeping it
  // outside FrontierManager/CoverageGuidance preserves byte-for-byte task
  // generation and all single-UAV planning costs.
  swarm_coordinator_ = std::make_shared<SwarmExplorationCoordinator>();
  swarm_coordinator_->init(nh);

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);
  ed_->next_goal_node_ = make_shared<TopoNode>();

  ep_->a_avg_ = tan(planner_manager_->gcopter_config_->maxTiltAngle) *
                planner_manager_->gcopter_config_->gravAcc;
  ep_->v_max_ = planner_manager_->gcopter_config_->maxVelMag;
  ep_->yaw_v_max_ = planner_manager_->gcopter_config_->yaw_max_vel;
  nh.param("viewpoint_param/global_viewpoint_num",
           ep_->global_viewpoint_num_, 16);
  nh.getParam("view_graph", ep_->view_graph_);
  nh.param("viewpoint_param/local_viewpoint_num", ep_->local_viewpoint_num_, 8);
  nh.getParam("global_planning/w_vdir", ep_->w_vdir_);
  nh.getParam("global_planning/w_yawdir", ep_->w_yawdir_);
  nh.param("GoalLockEnable", ep_->goal_lock_enable_, true);
  nh.param("TaskManager/GoalLockEnable", ep_->goal_lock_enable_,
           ep_->goal_lock_enable_);
  nh.param("GoalSwitchMinInterval", ep_->goal_switch_min_interval_, 1.2);
  nh.param("TaskManager/GoalSwitchMinInterval",
           ep_->goal_switch_min_interval_, ep_->goal_switch_min_interval_);
  nh.param("GoalSwitchMinImprovement", ep_->goal_switch_min_improvement_, 2.0);
  nh.param("TaskManager/GoalSwitchMinImprovement",
           ep_->goal_switch_min_improvement_,
           ep_->goal_switch_min_improvement_);
  nh.param("GoalSwitchHighSpeedMultiplier",
           ep_->goal_switch_high_speed_multiplier_, 1.8);
  nh.param("TaskManager/GoalSwitchHighSpeedMultiplier",
           ep_->goal_switch_high_speed_multiplier_,
           ep_->goal_switch_high_speed_multiplier_);
  nh.param("GoalKeepCostRatio", ep_->goal_keep_cost_ratio_, 1.35);
  nh.param("TaskManager/GoalKeepCostRatio", ep_->goal_keep_cost_ratio_,
           ep_->goal_keep_cost_ratio_);
  nh.param("GoalLockMatchRadius", ep_->goal_lock_match_radius_, 1.0);
  nh.param("TaskManager/GoalLockMatchRadius", ep_->goal_lock_match_radius_,
           ep_->goal_lock_match_radius_);
  nh.param("GoalReachedRadius", ep_->goal_reached_radius_, 0.35);
  nh.param("TaskManager/GoalReachedRadius", ep_->goal_reached_radius_,
           ep_->goal_reached_radius_);
  nh.param("exploration/original_frontend_compatibility",
           ep_->original_frontend_compatibility_, true);
  nh.param("global_planning/epic_simple_cost",
           ep_->epic_simple_global_cost_, true);
  nh.param("global_planning/composite_candidate_cost_enable",
           ep_->composite_candidate_cost_enable_, true);
  nh.param("global_planning/candidate_travel_weight",
           ep_->candidate_travel_weight_, 1.0);
  nh.param("global_planning/candidate_turn_brake_weight",
           ep_->candidate_turn_brake_weight_, 1.0);
  nh.param("global_planning/candidate_future_return_weight",
           ep_->candidate_future_return_weight_, 0.55);
  nh.param("global_planning/candidate_information_gain_weight",
           ep_->candidate_information_gain_weight_, 2.0);
  nh.param("global_planning/candidate_wait_weight",
           ep_->candidate_wait_weight_, 1.5);
  nh.param("global_planning/candidate_debt_weight",
           ep_->candidate_debt_weight_, 2.5);
  nh.param("global_planning/candidate_gain_saturation",
           ep_->candidate_gain_saturation_, 30.0);
  nh.param("global_planning/candidate_wait_saturation",
           ep_->candidate_wait_saturation_, 18.0);
  nh.param("global_planning/candidate_debt_saturation",
           ep_->candidate_debt_saturation_, 3.0);
  nh.param("global_planning/candidate_return_cost_cap",
           ep_->candidate_return_cost_cap_, 12.0);
  nh.param("global_planning/candidate_return_horizon",
           ep_->candidate_return_horizon_, 4);
  nh.param("global_planning/frontier_pass_radius",
           ep_->frontier_pass_radius_, 10.0);
  nh.param("global_planning/frontier_pass_exit_margin",
           ep_->frontier_pass_exit_margin_, 2.0);
  nh.param("global_planning/frontier_pass_cooldown",
           ep_->frontier_pass_cooldown_, 4.0);
  nh.param("global_planning/frontier_pass_debt_increment",
           ep_->frontier_pass_debt_increment_, 1.0);
  nh.param("global_planning/frontier_pass_debt_max",
           ep_->frontier_pass_debt_max_, 4.0);
  nh.param("global_planning/failed_goal_cooldown",
           ep_->failed_goal_cooldown_, 30.0);
  nh.param("global_planning/failed_goal_penalty",
           ep_->failed_goal_penalty_, 2000.0);
  std::string mission_mode{"coverage"};
  nh.param("exploration/mission_mode", mission_mode, mission_mode);
  std::transform(mission_mode.begin(), mission_mode.end(),
                 mission_mode.begin(), [](const unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  if (mission_mode == "target" || mission_mode == "target_directed") {
    ep_->target_directed_mode_ = true;
  } else if (mission_mode != "coverage") {
    ROS_WARN_STREAM("[target exploration] invalid mission_mode='"
                    << mission_mode << "'; fall back to coverage");
  }
  nh.param("exploration/target_goal_use_message_z",
           ep_->target_goal_use_message_z_, ep_->target_goal_use_message_z_);
  nh.param("exploration/target_heuristic_weight",
           ep_->target_heuristic_weight_, ep_->target_heuristic_weight_);
  nh.param("exploration/target_lateral_weight",
           ep_->target_lateral_weight_, ep_->target_lateral_weight_);
  nh.param("exploration/target_vertical_weight",
           ep_->target_vertical_weight_, ep_->target_vertical_weight_);
  nh.param("exploration/target_frontier_min_progress",
           ep_->target_frontier_min_progress_,
           ep_->target_frontier_min_progress_);
  nh.param("exploration/target_reached_radius",
           ep_->target_reached_radius_, ep_->target_reached_radius_);
  nh.param("exploration/target_direct_retry_delay",
           ep_->target_direct_retry_delay_,
           ep_->target_direct_retry_delay_);
  nh.param("global_planning/frontier_progress_watchdog_enable",
           frontier_progress_watchdog_enable_,
           frontier_progress_watchdog_enable_);
  nh.param("global_planning/frontier_progress_timeout",
           frontier_progress_timeout_, frontier_progress_timeout_);
  nh.param("global_planning/frontier_progress_min_cost_drop",
           frontier_progress_min_cost_drop_,
           frontier_progress_min_cost_drop_);
  nh.param("global_planning/frontier_progress_min_distance_drop",
           frontier_progress_min_distance_drop_,
           frontier_progress_min_distance_drop_);
  ep_->failed_goal_cooldown_ =
      std::clamp(ep_->failed_goal_cooldown_, 1.0, 120.0);
  ep_->failed_goal_penalty_ = std::max(0.0, ep_->failed_goal_penalty_);
  ep_->target_heuristic_weight_ =
      std::clamp(ep_->target_heuristic_weight_, 0.0, 20.0);
  ep_->target_lateral_weight_ =
      std::clamp(ep_->target_lateral_weight_, 0.0, 10.0);
  ep_->target_vertical_weight_ =
      std::clamp(ep_->target_vertical_weight_, 0.0, 10.0);
  ep_->target_frontier_min_progress_ =
      std::clamp(ep_->target_frontier_min_progress_, 0.0, 20.0);
  ep_->target_reached_radius_ =
      std::clamp(ep_->target_reached_radius_, 0.10, 5.0);
  ep_->target_direct_retry_delay_ =
      std::clamp(ep_->target_direct_retry_delay_, 0.5, 120.0);
  nh.param("exploration/target_cluster_shortlist",
           ep_->target_cluster_shortlist_, ep_->target_cluster_shortlist_);
  nh.param("exploration/target_goal_lock_remaining_margin",
           ep_->target_goal_lock_remaining_margin_,
           ep_->target_goal_lock_remaining_margin_);
  nh.param("exploration/target_topology_guidance_enable",
           ep_->target_topology_guidance_enable_,
           ep_->target_topology_guidance_enable_);
  nh.param("exploration/target_topology_query_interval",
           ep_->target_topology_query_interval_,
           ep_->target_topology_query_interval_);
  nh.param("exploration/target_topology_local_prefix_length",
           ep_->target_topology_local_prefix_length_,
           ep_->target_topology_local_prefix_length_);
  nh.param("exploration/target_topology_goal_change_tolerance",
           ep_->target_topology_goal_change_tolerance_,
           ep_->target_topology_goal_change_tolerance_);
  nh.param("exploration/target_topology_anchor_min_progress",
           ep_->target_topology_anchor_min_progress_,
           ep_->target_topology_anchor_min_progress_);
  nh.param("exploration/target_topology_anchor_progress_weight",
           ep_->target_topology_anchor_progress_weight_,
           ep_->target_topology_anchor_progress_weight_);
  nh.param("exploration/target_topology_anchor_lateral_penalty",
           ep_->target_topology_anchor_lateral_penalty_,
           ep_->target_topology_anchor_lateral_penalty_);
  nh.param("exploration/target_topology_anchor_radial_penalty",
           ep_->target_topology_anchor_radial_penalty_,
           ep_->target_topology_anchor_radial_penalty_);
  nh.param("exploration/target_topology_anchor_remaining_penalty",
           ep_->target_topology_anchor_remaining_penalty_,
           ep_->target_topology_anchor_remaining_penalty_);
  nh.param("exploration/target_topology_anchor_expansion_bonus",
           ep_->target_topology_anchor_expansion_bonus_,
           ep_->target_topology_anchor_expansion_bonus_);
  nh.param("exploration/target_topology_anchor_candidate_count",
           ep_->target_topology_anchor_candidate_count_,
           ep_->target_topology_anchor_candidate_count_);
  nh.param("exploration/target_topology_anchor_query_attempts",
           ep_->target_topology_anchor_query_attempts_,
           ep_->target_topology_anchor_query_attempts_);
  nh.param("exploration/target_topology_anchor_min_route_length",
           ep_->target_topology_anchor_min_route_length_,
           ep_->target_topology_anchor_min_route_length_);
  nh.param("exploration/target_topology_frontier_bias",
           ep_->target_topology_frontier_bias_,
           ep_->target_topology_frontier_bias_);
  nh.param("exploration/target_topology_direct_prefix_enable",
           ep_->target_topology_direct_prefix_enable_,
           ep_->target_topology_direct_prefix_enable_);
  nh.param("exploration/target_local_preflight_fail_limit",
           ep_->target_local_preflight_fail_limit_,
           ep_->target_local_preflight_fail_limit_);
  nh.param("exploration/target_progress_detour_budget",
           ep_->target_progress_detour_budget_,
           ep_->target_progress_detour_budget_);
  nh.param("exploration/target_preflight_horizon",
           ep_->target_preflight_horizon_, ep_->target_preflight_horizon_);
  nh.param("exploration/target_preflight_soft_defer",
           ep_->target_preflight_soft_defer_,
           ep_->target_preflight_soft_defer_);
  nh.param("exploration/target_empty_pool_escape_wait",
           ep_->target_empty_pool_escape_wait_,
           ep_->target_empty_pool_escape_wait_);
  nh.param("exploration/target_empty_pool_unlock_count",
           ep_->target_empty_pool_unlock_count_,
           ep_->target_empty_pool_unlock_count_);
  nh.param("exploration/target_escape_min_progress",
           ep_->target_escape_min_progress_,
           ep_->target_escape_min_progress_);
  nh.param("exploration/target_near_goal_remaining",
           ep_->target_near_goal_remaining_,
           ep_->target_near_goal_remaining_);
  nh.param("exploration/target_spatial_blacklist_radius",
           ep_->target_spatial_blacklist_radius_,
           ep_->target_spatial_blacklist_radius_);
  nh.param("exploration/target_spatial_blacklist_duration",
           ep_->target_spatial_blacklist_duration_,
           ep_->target_spatial_blacklist_duration_);
  ep_->target_cluster_shortlist_ =
      std::clamp(ep_->target_cluster_shortlist_, 2, 16);
  ep_->target_goal_lock_remaining_margin_ =
      std::clamp(ep_->target_goal_lock_remaining_margin_, 0.1, 10.0);
  ep_->target_topology_query_interval_ =
      std::clamp(ep_->target_topology_query_interval_, 0.0, 30.0);
  ep_->target_topology_local_prefix_length_ =
      std::clamp(ep_->target_topology_local_prefix_length_, 1.0, 50.0);
  ep_->target_topology_goal_change_tolerance_ =
      std::clamp(ep_->target_topology_goal_change_tolerance_, 0.05, 10.0);
  ep_->target_topology_anchor_min_progress_ =
      std::clamp(ep_->target_topology_anchor_min_progress_, 0.0, 20.0);
  ep_->target_topology_anchor_progress_weight_ =
      std::clamp(ep_->target_topology_anchor_progress_weight_, 0.0, 20.0);
  ep_->target_topology_anchor_lateral_penalty_ =
      std::clamp(ep_->target_topology_anchor_lateral_penalty_, 0.0, 10.0);
  ep_->target_topology_anchor_radial_penalty_ =
      std::clamp(ep_->target_topology_anchor_radial_penalty_, 0.0, 10.0);
  ep_->target_topology_anchor_remaining_penalty_ =
      std::clamp(ep_->target_topology_anchor_remaining_penalty_, 0.0, 10.0);
  ep_->target_topology_anchor_expansion_bonus_ =
      std::clamp(ep_->target_topology_anchor_expansion_bonus_, 0.0, 50.0);
  ep_->target_topology_anchor_candidate_count_ =
      std::clamp(ep_->target_topology_anchor_candidate_count_, 1, 32);
  ep_->target_topology_anchor_query_attempts_ =
      std::clamp(ep_->target_topology_anchor_query_attempts_, 1, 16);
  ep_->target_topology_anchor_min_route_length_ =
      std::clamp(ep_->target_topology_anchor_min_route_length_, 0.0, 20.0);
  ep_->target_topology_frontier_bias_ =
      std::clamp(ep_->target_topology_frontier_bias_, 0.0, 20.0);
  ep_->target_local_preflight_fail_limit_ =
      std::clamp(ep_->target_local_preflight_fail_limit_, 1, 10);
  ep_->target_progress_detour_budget_ =
      std::clamp(ep_->target_progress_detour_budget_, 0, 8);
  ep_->target_preflight_horizon_ =
      std::clamp(ep_->target_preflight_horizon_, 2.0, 20.0);
  ep_->target_preflight_soft_defer_ =
      std::clamp(ep_->target_preflight_soft_defer_, 0.3, 10.0);
  ep_->target_empty_pool_escape_wait_ =
      std::clamp(ep_->target_empty_pool_escape_wait_, 0.5, 30.0);
  ep_->target_empty_pool_unlock_count_ =
      std::clamp(ep_->target_empty_pool_unlock_count_, 1, 4);
  ep_->target_escape_min_progress_ =
      std::clamp(ep_->target_escape_min_progress_, -20.0, 5.0);
  ep_->target_near_goal_remaining_ =
      std::clamp(ep_->target_near_goal_remaining_, 2.0, 80.0);
  ep_->target_spatial_blacklist_radius_ =
      std::clamp(ep_->target_spatial_blacklist_radius_, 0.5, 20.0);
  ep_->target_spatial_blacklist_duration_ =
      std::clamp(ep_->target_spatial_blacklist_duration_, 1.0, 60.0);
  target_topology_guidance_.configure(targetTopologyGuidanceConfig());
  frontier_progress_timeout_ =
      std::clamp(frontier_progress_timeout_, 4.0, 60.0);
  frontier_progress_min_cost_drop_ =
      std::clamp(frontier_progress_min_cost_drop_, 0.1, 5.0);
  frontier_progress_min_distance_drop_ =
      std::clamp(frontier_progress_min_distance_drop_, 0.1, 5.0);
  nh.param("coverage_guidance/recovery_target_cooldown",
           coverage_recovery_cooldown_, coverage_recovery_cooldown_);
  nh.param("coverage_guidance/recovery_target_timeout",
           coverage_recovery_timeout_, coverage_recovery_timeout_);
  nh.param("coverage_guidance/recovery_target_match_radius",
           coverage_recovery_match_radius_,
           coverage_recovery_match_radius_);
  nh.param("coverage_guidance/recovery_reached_radius",
           coverage_recovery_reached_radius_,
           coverage_recovery_reached_radius_);
  nh.param("coverage_guidance/finish_plateau_duration",
           coverage_finish_plateau_duration_,
           coverage_finish_plateau_duration_);
  nh.param("coverage_guidance/finish_min_progress_voxels",
           coverage_finish_min_progress_voxels_,
           coverage_finish_min_progress_voxels_);
  nh.param("coverage_guidance/recovery_min_gain_voxels",
           coverage_recovery_min_gain_voxels_,
           coverage_recovery_min_gain_voxels_);
  nh.param("coverage_guidance/recovery_max_no_gain_attempts",
           coverage_recovery_max_no_gain_attempts_,
           coverage_recovery_max_no_gain_attempts_);
  nh.param("coverage_guidance/recovery_max_failure_attempts",
           coverage_recovery_max_failure_attempts_,
           coverage_recovery_max_failure_attempts_);
  nh.param("coverage_guidance/terminal_retry_enable",
           coverage_terminal_retry_enable_,
           coverage_terminal_retry_enable_);
  nh.param("coverage_guidance/terminal_retry_interval",
           coverage_terminal_retry_interval_,
           coverage_terminal_retry_interval_);
  nh.param("coverage_guidance/executable_candidate_enable",
           coverage_executable_candidate_enable_,
           coverage_executable_candidate_enable_);
  nh.param("coverage_guidance/executable_candidate_max_count",
           coverage_executable_candidate_max_count_,
           coverage_executable_candidate_max_count_);
  nh.param("coverage_guidance/executable_empty_min_count",
           coverage_executable_empty_min_count_,
           coverage_executable_empty_min_count_);
  nh.param("coverage_guidance/executable_empty_min_duration",
           coverage_executable_empty_min_duration_,
           coverage_executable_empty_min_duration_);
  nh.param("coverage_guidance/executable_candidate_max_speed",
           coverage_executable_candidate_max_speed_,
           coverage_executable_candidate_max_speed_);
  nh.param("coverage_guidance/moving_handoff_enable",
           coverage_moving_handoff_enable_,
           coverage_moving_handoff_enable_);
  nh.param("coverage_guidance/route_rank_weight",
           coverage_route_rank_weight_,
           coverage_route_rank_weight_);
  nh.param("coverage_guidance/floor_priority_enable",
           coverage_floor_priority_enable_,
           coverage_floor_priority_enable_);
  nh.param("coverage_guidance/floor_priority_min_z",
           coverage_floor_priority_min_z_,
           coverage_floor_priority_min_z_);
  nh.param("coverage_guidance/floor_transition_rank_window",
           coverage_floor_transition_rank_window_,
           coverage_floor_transition_rank_window_);
  nh.param("coverage_guidance/executable_candidate_bonus",
           coverage_executable_candidate_bonus_,
           coverage_executable_candidate_bonus_);
  coverage_recovery_cooldown_ =
      std::clamp(coverage_recovery_cooldown_, 5.0, 300.0);
  coverage_recovery_timeout_ =
      std::clamp(coverage_recovery_timeout_, 5.0, 120.0);
  coverage_recovery_match_radius_ =
      std::clamp(coverage_recovery_match_radius_, 0.3, 5.0);
  coverage_recovery_reached_radius_ =
      std::clamp(coverage_recovery_reached_radius_, 0.3, 2.5);
  coverage_finish_plateau_duration_ =
      std::clamp(coverage_finish_plateau_duration_, 3.0, 300.0);
  coverage_finish_min_progress_voxels_ =
      std::max(1, coverage_finish_min_progress_voxels_);
  coverage_recovery_min_gain_voxels_ =
      std::max(1, coverage_recovery_min_gain_voxels_);
  coverage_recovery_max_no_gain_attempts_ =
      std::max(1, coverage_recovery_max_no_gain_attempts_);
  coverage_recovery_max_failure_attempts_ =
      std::max(1, coverage_recovery_max_failure_attempts_);
  coverage_terminal_retry_interval_ =
      std::clamp(coverage_terminal_retry_interval_, 0.5,
                 coverage_recovery_cooldown_);
  coverage_executable_candidate_max_count_ =
      std::clamp(coverage_executable_candidate_max_count_, 1, 16);
  coverage_executable_empty_min_count_ =
      std::clamp(coverage_executable_empty_min_count_, 1, 20);
  coverage_executable_empty_min_duration_ =
      std::clamp(coverage_executable_empty_min_duration_, 0.0, 10.0);
  coverage_executable_candidate_max_speed_ =
      std::clamp(coverage_executable_candidate_max_speed_, 0.1, 2.0);
  coverage_route_rank_weight_ =
      std::clamp(coverage_route_rank_weight_, 0.0, 2.0);
  coverage_floor_priority_min_z_ =
      std::clamp(coverage_floor_priority_min_z_, -20.0, 50.0);
  coverage_floor_transition_rank_window_ =
      std::clamp(coverage_floor_transition_rank_window_, 1, 16);
  coverage_executable_candidate_bonus_ =
      std::clamp(coverage_executable_candidate_bonus_, 0.0, 20.0);
  nh.param("exploration/use_lkh", ep_->use_lkh_, true);

  // FALCON-style global coverage guidance is deliberately a separate data
  // layer. It remembers raw ROG free/occupied evidence globally but can only
  // rank frontier IDs produced by the existing HighSpeedExp frontend.
  double coverage_resolution = 0.6;
  nh.param("coverage_guidance/voxel_resolution", coverage_resolution,
           coverage_resolution);
  coverage_resolution = std::max(0.2, coverage_resolution);
  CoverageMapSpec coverage_spec;
  if (planner_manager_->lidar_map_interface_ &&
      planner_manager_->lidar_map_interface_->lp_) {
    const auto &lio = planner_manager_->lidar_map_interface_->lp_;
    coverage_spec.min = lio->global_box_min_boundary_.cast<double>();
    coverage_spec.max = lio->global_box_max_boundary_.cast<double>();
    coverage_spec.resolution = coverage_resolution;
    coverage_spec.dims =
        ((coverage_spec.max - coverage_spec.min) / coverage_resolution)
            .array()
            .ceil()
            .cast<int>()
            .matrix()
            .cwiseMax(Eigen::Vector3i::Ones());
    for (int i = 0;
         i < static_cast<int>(lio->global_box_min_boundary_vec_.size()) &&
         i < static_cast<int>(lio->global_box_max_boundary_vec_.size());
         ++i) {
      coverage_spec.valid_boxes.push_back(
          {lio->global_box_min_boundary_vec_[i].cast<double>(),
           lio->global_box_max_boundary_vec_[i].cast<double>()});
    }
    for (int i = 0;
         i < static_cast<int>(lio->dead_area_min_boundary_vec_.size()) &&
         i < static_cast<int>(lio->dead_area_max_boundary_vec_.size());
         ++i) {
      coverage_spec.dead_boxes.push_back(
          {lio->dead_area_min_boundary_vec_[i].cast<double>(),
           lio->dead_area_max_boundary_vec_[i].cast<double>()});
    }
  }
  coverage_guidance_ = std::make_shared<CoverageGuidanceManager>();
  coverage_guidance_->initialize(nh, coverage_spec);

  string tsp_base_dir;
  nh.param("exploration/tsp_dir", tsp_base_dir,
           std::filesystem::temp_directory_path().string());
  if (ep_->use_lkh_) {
    const string process_dir_name =
        "highspeed_exp_lkh_u" +
        std::to_string(static_cast<unsigned long long>(::getuid())) + "_p" +
        std::to_string(static_cast<long long>(::getpid()));
    std::filesystem::path process_dir =
        std::filesystem::path(tsp_base_dir) / process_dir_name;
    std::error_code ec;
    std::filesystem::create_directories(process_dir, ec);
    if (ec) {
      ROS_WARN_STREAM("[global tour] cannot create configured LKH directory "
                      << process_dir << ": " << ec.message()
                      << "; retry under the system temporary directory");
      ec.clear();
      process_dir = std::filesystem::temp_directory_path(ec) / process_dir_name;
      if (!ec) {
        std::filesystem::create_directories(process_dir, ec);
      }
    }
    if (!ec) {
      ep_->tsp_dir_ = std::filesystem::absolute(process_dir).string();
      ofstream par_file(ep_->tsp_dir_ + "/single.par",
                        std::ios::out | std::ios::trunc);
      if (!par_file.is_open()) {
        ROS_ERROR_STREAM("[global tour] cannot write LKH parameter file in "
                         << ep_->tsp_dir_
                         << "; use deterministic fallback solver");
        ep_->use_lkh_ = false;
      } else {
        par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
        par_file << "GAIN23 = NO\n";
        par_file << "MOVE_TYPE = 2\n";
        par_file << "OUTPUT_TOUR_FILE = " << ep_->tsp_dir_
                 << "/single.txt\n";
        par_file << "RUNS = 10\n";
      }
    } else {
      ROS_ERROR_STREAM("[global tour] cannot create a writable LKH directory: "
                       << ec.message()
                       << "; use deterministic fallback solver");
      ep_->use_lkh_ = false;
    }
  }
  ROS_INFO_STREAM("[frontend compatibility] original_behavior="
                  << ep_->original_frontend_compatibility_
                  << " epic_simple_cost=" << ep_->epic_simple_global_cost_
                  << " composite_candidate_cost="
                  << ep_->composite_candidate_cost_enable_
                  << " lkh=" << ep_->use_lkh_
                  << (ep_->use_lkh_ ? " work_dir=" + ep_->tsp_dir_ : ""));
  ROS_INFO_STREAM("[target exploration] mission_mode="
                  << (ep_->target_directed_mode_ ? "target" : "coverage")
                  << " heuristic=" << ep_->target_heuristic_weight_
                  << " lateral=" << ep_->target_lateral_weight_
                  << " min_frontier_progress="
                  << ep_->target_frontier_min_progress_
                  << " cluster_shortlist=" << ep_->target_cluster_shortlist_
                  << " use_message_z=" << ep_->target_goal_use_message_z_);
}

void FastExplorationManager::updateCoverageGuidance(
    const Eigen::Vector3d &pos) {
  if (targetDirectedModeConfigured()) {
    return;
  }
  if (!coverage_guidance_ || !coverage_guidance_->samplingDue() ||
      !frontier_manager_ptr_ || !planner_manager_) {
    return;
  }
  CoverageMapDelta delta;
  delta.version = ++coverage_map_version_;
  if (!planner_manager_->sampleCoverageMap(coverage_guidance_->mapSpec(),
                                           delta)) {
    --coverage_map_version_;
    ROS_WARN_STREAM_THROTTLE(
        2.0, "[coverage guidance] raw ROG map snapshot unavailable; keep "
             "legacy frontier selection");
    return;
  }

  std::vector<CoverageFrontier> frontiers;
  frontiers.reserve(frontier_manager_ptr_->cluster_list_.size());
  const ros::Time now = ros::Time::now();
  for (const ClusterInfo::Ptr &cluster :
       frontier_manager_ptr_->cluster_list_) {
    if (!cluster || cluster->state_ == FrontierState::VISITED ||
        cluster->state_ == FrontierState::BLACKLISTED ||
        cluster->state_ == FrontierState::SUSPENDED) {
      continue;
    }
    CoverageFrontier frontier;
    frontier.cluster_id = cluster->id_;
    frontier.position =
        (!cluster->candidate_vps_.empty()
             ? cluster->candidate_vps_.front()
             : cluster->center_)
            .cast<double>();
    frontier.yaw = !cluster->candidate_yaws_.empty()
                       ? cluster->candidate_yaws_.front()
                       : cluster->best_vp_yaw_;
    frontier.information_gain =
        std::max(cluster->last_visible_gain_, cluster->stable_visible_gain_);
    frontier.wait_age =
        cluster->first_reachable_time_.isZero()
            ? 0.0
            : std::max(0.0,
                       (now - cluster->first_reachable_time_).toSec());
    frontier.pass_debt = std::max(0.0, cluster->pass_debt_);
    frontiers.emplace_back(frontier);
  }
  coverage_guidance_->submit(std::move(delta), std::move(frontiers), pos);
}

CoverageFinishStatus FastExplorationManager::coverageFinishStatus() {
  CoverageFinishStatus status;
  if (targetDirectedModeConfigured() || !coverage_guidance_ ||
      !planner_manager_) {
    return status;
  }
  status.guard_enabled = coverage_guidance_->finishGuardEnabled();
  if (!status.guard_enabled) {
    return status;
  }

  const CoveragePlan::Ptr plan = coverage_guidance_->latestUsablePlan();
  if (!plan || !plan->valid) {
    return status;
  }
  status.plan_valid = true;
  status.observed_voxels = plan->observed_voxel_count;
  status.valid_voxels = plan->valid_voxel_count;
  status.coverage_ratio = plan->coverage_ratio;

  const ros::Time now = ros::Time::now();
  if (coverage_finish_progress_observed_voxels_ < 0 ||
      coverage_finish_last_progress_time_.isZero()) {
    coverage_finish_progress_observed_voxels_ =
        plan->observed_voxel_count;
    coverage_finish_last_progress_time_ = now;
  } else if (plan->observed_voxel_count >=
             coverage_finish_progress_observed_voxels_ +
                 coverage_finish_min_progress_voxels_) {
    coverage_finish_progress_observed_voxels_ =
        plan->observed_voxel_count;
    coverage_finish_last_progress_time_ = now;
  }
  status.plateau_duration =
      std::max(0.0, (now - coverage_finish_last_progress_time_).toSec());
  status.plateau_reached =
      status.plateau_duration >= coverage_finish_plateau_duration_;

  // Cooldown is deliberately ignored here. A target is exhausted only after
  // a terminal outcome (unsafe/disconnected/occluded) or enough no-gain/
  // planning-failure attempts. This prevents a rotating set of 45-second
  // cooldowns from blocking FINISH forever.
  const auto targets = coverage_guidance_->unknownApproachTargets(
      planner_manager_->local_data_.curr_pos_, 160, 0.0);
  status.actionable_targets = static_cast<int>(targets.size());
  double next_retry_duration = std::numeric_limits<double>::infinity();
  for (const CoverageTarget &raw_target : targets) {
    CoverageTarget target = raw_target;
    // Recovery outcomes are recorded against the executable approach chosen
    // from approach_candidates. Canonicalize before querying the registry so
    // FINISH observes the same identity as planGlobalPath. A raw regenerated
    // center must not make an already exhausted action look eligible again.
    if (!coverageRecoveryExhausted(target)) {
      // This function is reached only after the frontend/full-reachability
      // audit has produced NO_FRONTIER.  If none of a component's approach
      // candidates is safe/visible, record that terminal outcome here;
      // otherwise it would be counted as eligible on every finish-gate tick
      // even though planGlobalPath can never promote it.
      selectSafeCoverageApproach(target, true);
    }
    if (coverageRecoveryExhausted(target)) {
      rememberCoverageRecoveryAlias(target);
      ++status.exhausted_targets;
      continue;
    }
    double cooling_remaining = 0.0;
    if (coverageRecoveryCooling(target, now, &cooling_remaining)) {
      ++status.cooling_targets;
      double retry_after = cooling_remaining;
      coverageTerminalRetryReady(target, now, &retry_after);
      next_retry_duration = std::min(next_retry_duration, retry_after);
    } else {
      ++status.eligible_targets;
    }
  }
  status.next_retry_duration =
      std::isfinite(next_retry_duration)
          ? std::max(0.0, next_retry_duration)
          : 0.0;
  status.targets_exhausted =
      status.actionable_targets == status.exhausted_targets;
  return status;
}

EdgeSafetyCost FastExplorationManager::getPathEdgeCost(
    TopoNode::Ptr &n1, const Eigen::Vector3d &v1, const float yaw1,
    TopoNode::Ptr &n2, const float yaw2) {
  EdgeSafetyCost edge;
  vector<Eigen::Vector3f> path;
  const int result =
      planner_manager_->fast_searcher_->topoSearch(n1, n2, 1e-2, path);
  if (result == BubbleAstar::NO_PATH || result == BubbleAstar::START_FAIL ||
      result == BubbleAstar::END_FAIL || path.size() < 2) {
    const double unreachable = 2e3 + (n1->center_ - n2->center_).norm();
    edge.total_cost = unreachable;
    edge.time_cost = unreachable;
    edge.backup_penalty = unreachable;
    edge.backup_feasible = false;
    return edge;
  }
  return planner_manager_->estimateHighSpeedEdgeCost(path, v1, yaw1, yaw2);
}

void FastExplorationManager::goalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg) {
    return;
  }
  setMissionGoal(*msg);
  // 提取四元数
  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);

  // 将四元数转换为Euler角
  tf::Matrix3x3(quat).getRPY(roll, pitch, goal_yaw);
}

bool FastExplorationManager::targetDirectedModeConfigured() const {
  return ep_ && ep_->target_directed_mode_;
}

bool FastExplorationManager::targetDirectedModeActive() const {
  return targetDirectedModeConfigured() && ed_ && ed_->has_mission_goal_;
}

void FastExplorationManager::setMissionGoal(
    const geometry_msgs::PoseStamped &msg) {
  if (!ed_ || !ep_) {
    return;
  }
  Eigen::Vector3d goal(msg.pose.position.x, msg.pose.position.y,
                       msg.pose.position.z);
  if (!goal.allFinite()) {
    ROS_WARN("[target exploration] ignore non-finite mission goal");
    return;
  }
  if (!ep_->target_goal_use_message_z_ && planner_manager_) {
    goal.z() = planner_manager_->local_data_.curr_pos_.z();
  }
  if (!goal.allFinite()) {
    ROS_WARN("[target exploration] ignore mission goal without valid height");
    return;
  }

  ed_->mission_goal_ = goal.cast<float>();
  ed_->mission_start_ = Eigen::Vector3f::Zero();
  if (planner_manager_) {
    ed_->mission_start_ =
        planner_manager_->local_data_.curr_pos_.cast<float>();
  }
  ed_->mission_goal_time_ = ros::Time::now();
  ed_->has_mission_goal_ = true;
  // A Nav Goal can arrive before the first odometry message. Resolve the
  // start (and the implicit 2D-goal height) from the first planning pose so a
  // click during startup never creates a ground-level endpoint for a UAV.
  ed_->mission_goal_needs_initialization_ = true;
  mission_goal_direct_retry_after_ = ros::Time(0);
  ed_->has_goal_lock_ = false;
  ed_->locked_goal_is_coverage_ = false;
  ed_->locked_goal_is_mission_ = false;
  ed_->locked_goal_cluster_id_ = -1;
  ed_->locked_goal_coverage_id_ = 0;
  resetNormalGoalProgress();
  target_topology_guidance_.reset();
  ROS_INFO_STREAM("[target exploration] set mission goal=("
                  << ed_->mission_goal_.transpose() << ") start=("
                  << ed_->mission_start_.transpose() << ")");
}

bool FastExplorationManager::setMissionMode(const std::string &mode) {
  if (!ed_ || !ep_) {
    return false;
  }
  std::string normalized = mode;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](const unsigned char value) {
                   return static_cast<char>(std::tolower(value));
                 });
  const bool target = normalized == "target" ||
                      normalized == "target_directed" ||
                      normalized == "target_exploration";
  if (!target && normalized != "coverage") {
    ROS_WARN_STREAM("[target exploration] ignore invalid mission mode='"
                    << mode << "' (expected target or coverage)");
    return false;
  }
  if (ep_->target_directed_mode_ == target) {
    return true;
  }

  ep_->target_directed_mode_ = target;
  mission_goal_direct_retry_after_ = ros::Time(0);
  ed_->has_goal_lock_ = false;
  ed_->locked_goal_is_coverage_ = false;
  ed_->locked_goal_is_mission_ = false;
  ed_->locked_goal_cluster_id_ = -1;
  ed_->locked_goal_coverage_id_ = 0;
  resetNormalGoalProgress();
  target_topology_guidance_.reset();
  if (!target) {
    ed_->has_mission_goal_ = false;
    ed_->mission_goal_needs_initialization_ = false;
    ed_->mission_goal_ = Eigen::Vector3f::Zero();
    ed_->mission_start_ = Eigen::Vector3f::Zero();
  }
  ROS_INFO_STREAM("[target exploration] runtime mission mode switched to "
                  << (target ? "target" : "coverage"));
  return true;
}

std::string FastExplorationManager::missionMode() const {
  return targetDirectedModeConfigured() ? "target" : "coverage";
}

double FastExplorationManager::missionGoalDistance(
    const Eigen::Vector3d &position) const {
  if (!targetDirectedModeActive() || !position.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  Eigen::Vector3d delta = position - ed_->mission_goal_.cast<double>();
  delta.z() *= ep_->target_vertical_weight_;
  return delta.norm();
}

bool FastExplorationManager::missionGoalReached(
    const Eigen::Vector3d &position) const {
  return ep_ && missionGoalDistance(position) <= ep_->target_reached_radius_;
}

bool FastExplorationManager::missionGoalDirectCandidateReady() const {
  if (!targetDirectedModeActive() || !planner_manager_ ||
      !planner_manager_->gcopter_config_ || !planner_manager_->topo_graph_ ||
      (!mission_goal_direct_retry_after_.isZero() &&
       ros::Time::now() < mission_goal_direct_retry_after_)) {
    return false;
  }
  const Eigen::Vector3d goal = ed_->mission_goal_.cast<double>();
  const double required_clearance = std::max(
      planner_manager_->topo_graph_->bubble_min_radius_,
      planner_manager_->gcopter_config_->commitKnownFreeSafeDistance);
  return planner_manager_->querySafetyState(goal) ==
             MapVoxelState::KNOWN_FREE &&
         planner_manager_->safetyDistanceToOcc(goal) >= required_clearance;
}

TargetDirectedExplorationConfig
FastExplorationManager::targetGuidanceConfig() const {
  TargetDirectedExplorationConfig config;
  if (ep_) {
    config.heuristic_weight = ep_->target_heuristic_weight_;
    config.lateral_weight = ep_->target_lateral_weight_;
    config.vertical_weight = ep_->target_vertical_weight_;
    config.nominal_speed = std::max(0.5, ep_->v_max_ / 2.0);
  }
  return config;
}

TargetTopologyGuidanceConfig
FastExplorationManager::targetTopologyGuidanceConfig() const {
  TargetTopologyGuidanceConfig config;
  if (!ep_) {
    return config;
  }
  config.enabled = ep_->target_topology_guidance_enable_;
  config.query_interval = ep_->target_topology_query_interval_;
  config.local_prefix_length = ep_->target_topology_local_prefix_length_;
  config.goal_change_tolerance =
      ep_->target_topology_goal_change_tolerance_;
  config.minimum_progress = ep_->target_topology_anchor_min_progress_;
  config.progress_weight = ep_->target_topology_anchor_progress_weight_;
  config.lateral_penalty = ep_->target_topology_anchor_lateral_penalty_;
  config.radial_penalty = ep_->target_topology_anchor_radial_penalty_;
  config.remaining_penalty = ep_->target_topology_anchor_remaining_penalty_;
  config.expansion_bonus = ep_->target_topology_anchor_expansion_bonus_;
  config.anchor_candidate_count =
      ep_->target_topology_anchor_candidate_count_;
  config.anchor_query_attempts = ep_->target_topology_anchor_query_attempts_;
  config.minimum_anchor_route_length =
      ep_->target_topology_anchor_min_route_length_;
  config.vertical_weight = ep_->target_vertical_weight_;
  return config;
}

const TargetTopologyGuide &FastExplorationManager::updateTargetTopologyGuide(
    const Eigen::Vector3d &pos) {
  target_topology_guidance_.configure(targetTopologyGuidanceConfig());
  const std::shared_ptr<general_planner::MapManager> map_manager =
      planner_manager_ ? planner_manager_->sharedMapManager() : nullptr;
  Eigen::Vector3d mission_goal = Eigen::Vector3d::Zero();
  if (ed_ && ed_->has_mission_goal_) {
    mission_goal = ed_->mission_goal_.cast<double>();
  }
  const TargetTopologyGuide &guide = target_topology_guidance_.update(
      map_manager, pos, mission_goal, ros::Time::now().toSec());
  if (guide.valid) {
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[target topology] "
                 << targetTopologyGuideStatusName(guide.status)
                 << " route=" << guide.route_id
                 << " topo_rev=" << guide.topology_revision
                 << " points=" << guide.route.size() << " anchor=("
                 << guide.anchor.transpose() << ") prefix=("
                 << guide.local_prefix_point.transpose() << ")");
  }
  return guide;
}

double FastExplorationManager::topologyGuidePenalty(
    const Eigen::Vector3d &candidate) const {
  if (!ep_ || !candidate.allFinite()) {
    return 0.0;
  }
  const TargetTopologyGuide &guide = target_topology_guidance_.guide();
  if (!guide.valid || !guide.local_prefix_point.allFinite()) {
    return 0.0;
  }
  Eigen::Vector3d delta =
      candidate - guide.local_prefix_point.cast<double>();
  delta.z() *= ep_->target_vertical_weight_;
  const double distance = delta.norm();
  if (!std::isfinite(distance)) {
    return 0.0;
  }
  return ep_->target_topology_frontier_bias_ * distance /
         std::max(0.5, ep_->v_max_ / 2.0);
}

bool FastExplorationManager::appendTopologyGuideCandidate(
    const Eigen::Vector3d &pos, const float curr_yaw,
    vector<TopoNode::Ptr> &viewpoints) const {
  if (!ep_ || !ep_->target_topology_direct_prefix_enable_ ||
      !planner_manager_ || !planner_manager_->topo_graph_ ||
      !planner_manager_->gcopter_config_) {
    return false;
  }
  const TargetTopologyGuide &guide = target_topology_guidance_.guide();
  if (!guide.valid || !guide.local_prefix_point.allFinite()) {
    return false;
  }
  const Eigen::Vector3f prefix = guide.local_prefix_point;
  if ((prefix.cast<double>() - pos).norm() <=
      std::max(0.5, ep_->target_reached_radius_)) {
    return false;
  }
  if (!planner_manager_->topo_graph_->hasRegionForPoint(prefix)) {
    return false;
  }
  const double required_clearance = std::max(
      planner_manager_->topo_graph_->bubble_min_radius_,
      planner_manager_->gcopter_config_->commitKnownFreeSafeDistance);
  const Eigen::Vector3d prefix_d = prefix.cast<double>();
  if (planner_manager_->querySafetyState(prefix_d) !=
          MapVoxelState::KNOWN_FREE ||
      planner_manager_->safetyDistanceToOcc(prefix_d) < required_clearance) {
    return false;
  }
  // A global edge is not permission to cross an unseen portion of the local
  // rolling map. Directly inject only a fully known-free local prefix; all
  // other guide information remains a frontier-ranking bias.
  const RaycastSafetyInfo local_line = planner_manager_->raycastSafety(
      pos, prefix_d, true, required_clearance,
      planner_manager_->gcopter_config_->safetyMapQueryStep);
  if (!local_line.all_known_free) {
    return false;
  }
  TopoNode::Ptr guide_node = std::make_shared<TopoNode>();
  guide_node->is_viewpoint_ = true;
  guide_node->frontier_cluster_id_ = -1;
  guide_node->center_ = prefix;
  const Eigen::Vector3d direction = prefix_d - pos;
  guide_node->yaw_ = std::hypot(direction.x(), direction.y()) > 1.0e-3
                         ? std::atan2(direction.y(), direction.x())
                         : curr_yaw;
  viewpoints.emplace_back(guide_node);
  ROS_INFO_STREAM_THROTTLE(
      1.0, "[target topology] add locally verified route prefix ("
               << prefix.transpose() << ") route=" << guide.route_id);
  return true;
}

std::unordered_set<int> FastExplorationManager::preferredTargetClusterIds(
    const Eigen::Vector3d &pos) {
  std::unordered_set<int> preferred;
  if (!targetDirectedModeActive() || !frontier_manager_ptr_ || !ed_ || !ep_) {
    return preferred;
  }
  std::vector<TargetClusterCandidate> clusters;
  clusters.reserve(frontier_manager_ptr_->cluster_list_.size());
  int deferred_skipped = 0;
  for (const ClusterInfo::Ptr &cluster :
       frontier_manager_ptr_->cluster_list_) {
    if (!cluster || cluster->id_ < 0 ||
        cluster->state_ == FrontierState::VISITED ||
        cluster->state_ == FrontierState::BLACKLISTED ||
        cluster->state_ == FrontierState::SUSPENDED) {
      continue;
    }
    const Eigen::Vector3f viewpoint =
        (!cluster->candidate_vps_.empty() ? cluster->candidate_vps_.front()
                                          : cluster->center_);
    // Never re-inject cooled bridges as "preferred"; that recreates the empty
    // shortlist death spiral after an escape unlock.
    if (isGoalCurrentlyDeferred(cluster->id_, viewpoint) ||
        isSpatiallyBlacklisted(viewpoint)) {
      ++deferred_skipped;
      continue;
    }
    TargetClusterCandidate candidate;
    candidate.id = cluster->id_;
    candidate.position = viewpoint.cast<double>();
    clusters.emplace_back(candidate);
  }
  // When many bridges are cooling, pull a wider destination-ranked set so a
  // fresh detour can replace the deferred shortlist.
  int shortlist = ep_->target_cluster_shortlist_;
  if (deferred_skipped > 0) {
    shortlist = std::min(
        16, shortlist + std::min(8, deferred_skipped + 2));
  }
  const TargetTopologyGuide &topology_guide = updateTargetTopologyGuide(pos);
  std::vector<int> ranked;
  if (!topology_guide.valid) {
    ranked = selectPreferredTargetClusterIds(
        pos, ed_->mission_goal_.cast<double>(), clusters,
        targetGuidanceConfig(), ep_->target_frontier_min_progress_, shortlist);
  } else {
    struct RankedCluster {
      int id{-1};
      TargetDirectedExplorationScore score;
      double cost{std::numeric_limits<double>::infinity()};
    };
    std::vector<RankedCluster> progressing;
    std::vector<RankedCluster> detours;
    progressing.reserve(clusters.size());
    detours.reserve(clusters.size());
    for (const auto &cluster : clusters) {
      const auto score = scoreTargetDirectedViewpoint(
          pos, ed_->mission_goal_.cast<double>(), cluster.position,
          targetGuidanceConfig());
      if (!score.valid) {
        continue;
      }
      RankedCluster candidate;
      candidate.id = cluster.id;
      candidate.score = score;
      candidate.cost = score.cost + topologyGuidePenalty(cluster.position);
      if (hasSufficientTargetProgress(score,
                                      ep_->target_frontier_min_progress_)) {
        progressing.emplace_back(candidate);
      } else {
        detours.emplace_back(candidate);
      }
    }
    const auto compare = [](const RankedCluster &lhs,
                            const RankedCluster &rhs) {
      if (lhs.cost != rhs.cost) {
        return lhs.cost < rhs.cost;
      }
      if (lhs.score.remaining_distance != rhs.score.remaining_distance) {
        return lhs.score.remaining_distance < rhs.score.remaining_distance;
      }
      return lhs.id < rhs.id;
    };
    std::stable_sort(progressing.begin(), progressing.end(), compare);
    std::stable_sort(detours.begin(), detours.end(), compare);
    const auto &primary = progressing.empty() ? detours : progressing;
    ranked.reserve(std::min<int>(shortlist, static_cast<int>(primary.size())));
    for (const auto &candidate : primary) {
      if (static_cast<int>(ranked.size()) >= shortlist) {
        break;
      }
      ranked.emplace_back(candidate.id);
    }
  }
  preferred.insert(ranked.begin(), ranked.end());
  if (!ranked.empty()) {
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[target exploration] inject " << ranked.size()
                                            << " destination-ranked frontier "
                                               "clusters into the viewpoint "
                                               "shortlist (skipped_deferred="
                                            << deferred_skipped << ")");
  }
  return preferred;
}

bool FastExplorationManager::targetBridgeLocallyExecutable(
    const TopoNode::Ptr &candidate, const Eigen::Vector3d &pos,
    const Eigen::Vector3d &vel, const float curr_yaw,
    SegmentSafetyInfo *safety_out) {
  if (!targetDirectedModeActive() || !candidate || !planner_manager_ ||
      !planner_manager_->topo_graph_ || !planner_manager_->fast_searcher_ ||
      !planner_manager_->gcopter_config_ ||
      !planner_manager_->topo_graph_->odom_node_) {
    return true;
  }
  // Search the already-inserted viewpoint/bridge node directly.  Do not rewrite
  // next_goal_node_ through updateGoalNode() here: that remeshes the virtual
  // goal with OpenMP on a shared ParallelBubbleAstar and mutates topology
  // adjacency.  Multi-candidate preflight used to invoke it many times per
  // plan cycle and could abort with glibc "corrupted double-linked list".
  (void)pos;
  vector<Eigen::Vector3f> local_path;
  const int search_result = planner_manager_->fast_searcher_->search(
      planner_manager_->topo_graph_->odom_node_, vel.cast<float>(), candidate,
      0.2, local_path);
  SegmentSafetyInfo safety;
  bool executable =
      search_result == BubbleAstar::REACH_END && local_path.size() >= 2;
  if (executable) {
    vector<Eigen::Vector3f> horizon_path;
    horizon_path.reserve(local_path.size());
    horizon_path.emplace_back(local_path.front());
    const double known_free_short = std::max(
        0.5, planner_manager_->gcopter_config_->knownFreeShortLength);
    const double configured_horizon =
        ep_ ? ep_->target_preflight_horizon_ : (2.0 * known_free_short);
    const double horizon = std::clamp(
        std::min(std::max(0.5, planner_manager_->max_traj_len_),
                 configured_horizon),
        known_free_short, 12.0);
    double accumulated = 0.0;
    for (std::size_t i = 1; i < local_path.size(); ++i) {
      const Eigen::Vector3f &next = local_path[i];
      const Eigen::Vector3f &previous = horizon_path.back();
      const double segment_length = (next - previous).norm();
      if (segment_length < 1.0e-4) {
        continue;
      }
      if (accumulated + segment_length > horizon) {
        const float fraction = static_cast<float>(std::clamp(
            (horizon - accumulated) / segment_length, 0.0, 1.0));
        horizon_path.emplace_back(previous + fraction * (next - previous));
        break;
      }
      horizon_path.emplace_back(next);
      accumulated += segment_length;
    }
    if (horizon_path.size() < 2) {
      executable = false;
    } else {
      float end_yaw = curr_yaw;
      const Eigen::Vector3f tail =
          horizon_path.back() - horizon_path[horizon_path.size() - 2];
      if (std::hypot(tail.x(), tail.y()) > 1.0e-3f) {
        end_yaw = std::atan2(tail.y(), tail.x());
      }
      // Reuse the topology edge evaluator so backup/clearance match the score
      // that admitted this bridge into the reachable set.
      const EdgeSafetyCost edge = planner_manager_->estimateHighSpeedEdgeCost(
          horizon_path, vel, curr_yaw, end_yaw);
      safety.path_length = edge.path_length;
      safety.known_free_length = edge.known_free_length;
      safety.min_clearance = edge.min_clearance;
      safety.turn_angle = edge.turn_angle;
      safety.initial_heading_delta = edge.initial_heading_delta;
      safety.current_speed = vel.norm();
      safety.backup_feasible = edge.backup_feasible;
      const double required_clearance = std::max(
          planner_manager_->topo_graph_->bubble_min_radius_,
          planner_manager_->gcopter_config_->commitKnownFreeSafeDistance);
      executable = safety.backup_feasible &&
                   std::isfinite(safety.min_clearance) &&
                   safety.min_clearance + 0.05 >= required_clearance;
    }
  }
  if (safety_out) {
    *safety_out = safety;
  }
  if (!executable) {
    ROS_INFO_STREAM_THROTTLE(
        0.5, "[target exploration] skip locally unsafe frontier "
                 "bridge before commit: cluster="
                 << candidate->frontier_cluster_id_
                 << " search_result=" << search_result
                 << " known_free=" << safety.known_free_length
                 << " min_clearance=" << safety.min_clearance
                 << " backup=" << safety.backup_feasible);
  }
  return executable;
}

int FastExplorationManager::noteLocalPreflightFailure(
    const TopoNode::Ptr &viewpoint) {
  if (!viewpoint || viewpoint->is_mission_goal_target_ ||
      viewpoint->is_coverage_target_) {
    return 0;
  }
  const ros::Time now = ros::Time::now();
  local_preflight_failures_.erase(
      std::remove_if(local_preflight_failures_.begin(),
                     local_preflight_failures_.end(),
                     [&](const LocalPreflightFailure &entry) {
                       return entry.last_failure.isZero() ||
                              (now - entry.last_failure).toSec() > 30.0;
                     }),
      local_preflight_failures_.end());
  LocalPreflightFailure *matched = nullptr;
  for (auto &entry : local_preflight_failures_) {
    const bool same_cluster =
        entry.cluster_id >= 0 &&
        entry.cluster_id == viewpoint->frontier_cluster_id_;
    const bool same_position =
        (entry.position - viewpoint->center_).norm() <=
        std::max(0.2, ep_ ? ep_->goal_lock_match_radius_ : 1.0);
    if (same_cluster || same_position) {
      matched = &entry;
      break;
    }
  }
  if (!matched) {
    if (local_preflight_failures_.size() >= 32U) {
      local_preflight_failures_.erase(local_preflight_failures_.begin());
    }
    local_preflight_failures_.push_back({});
    matched = &local_preflight_failures_.back();
    matched->consecutive_failures = 0;
  }
  matched->cluster_id = viewpoint->frontier_cluster_id_;
  matched->position = viewpoint->center_;
  matched->last_failure = now;
  matched->consecutive_failures =
      std::min(matched->consecutive_failures + 1, 100);
  return matched->consecutive_failures;
}

void FastExplorationManager::clearLocalPreflightFailure(
    const TopoNode::Ptr &viewpoint) {
  if (!viewpoint || local_preflight_failures_.empty()) {
    return;
  }
  local_preflight_failures_.erase(
      std::remove_if(local_preflight_failures_.begin(),
                     local_preflight_failures_.end(),
                     [&](const LocalPreflightFailure &entry) {
                       const bool same_cluster =
                           entry.cluster_id >= 0 &&
                           entry.cluster_id == viewpoint->frontier_cluster_id_;
                       const bool same_position =
                           (entry.position - viewpoint->center_).norm() <=
                           std::max(0.2, ep_ ? ep_->goal_lock_match_radius_
                                             : 1.0);
                       return same_cluster || same_position;
                     }),
      local_preflight_failures_.end());
}

bool FastExplorationManager::isGoalCurrentlyDeferred(
    const int cluster_id, const Eigen::Vector3f &position) const {
  if (!ep_ || deferred_goals_.empty()) {
    return false;
  }
  const ros::Time now = ros::Time::now();
  const double match_radius = std::max(0.2, ep_->goal_lock_match_radius_);
  for (const auto &goal : deferred_goals_) {
    if (goal.until.isZero() || now >= goal.until) {
      continue;
    }
    if ((goal.cluster_id >= 0 && goal.cluster_id == cluster_id) ||
        (goal.position - position).norm() <= match_radius) {
      return true;
    }
  }
  return false;
}

bool FastExplorationManager::isSpatiallyBlacklisted(
    const Eigen::Vector3f &position) const {
  if (!ep_ || target_spatial_blacklist_.empty() || !position.allFinite()) {
    return false;
  }
  const ros::Time now = ros::Time::now();
  const double radius = std::max(0.5, ep_->target_spatial_blacklist_radius_);
  for (const auto &entry : target_spatial_blacklist_) {
    if (entry.until.isZero() || now >= entry.until) {
      continue;
    }
    if ((entry.position - position).norm() <= radius) {
      return true;
    }
  }
  return false;
}

void FastExplorationManager::rememberSpatialBlacklist(
    const Eigen::Vector3f &position, const double duration_sec) {
  if (!ep_ || !position.allFinite() || duration_sec <= 0.0) {
    return;
  }
  const ros::Time now = ros::Time::now();
  const double radius = std::max(0.5, ep_->target_spatial_blacklist_radius_);
  target_spatial_blacklist_.erase(
      std::remove_if(target_spatial_blacklist_.begin(),
                     target_spatial_blacklist_.end(),
                     [&](const SpatialBlacklistEntry &entry) {
                       return entry.until.isZero() || entry.until <= now;
                     }),
      target_spatial_blacklist_.end());
  SpatialBlacklistEntry *matched = nullptr;
  for (auto &entry : target_spatial_blacklist_) {
    if ((entry.position - position).norm() <= radius) {
      matched = &entry;
      break;
    }
  }
  if (!matched) {
    if (target_spatial_blacklist_.size() >= 64U) {
      target_spatial_blacklist_.erase(target_spatial_blacklist_.begin());
    }
    target_spatial_blacklist_.push_back({});
    matched = &target_spatial_blacklist_.back();
  }
  matched->position = position;
  const ros::Time until = now + ros::Duration(duration_sec);
  if (matched->until.isZero() || until > matched->until) {
    matched->until = until;
  }
}

void FastExplorationManager::clearSpatialBlacklistAround(
    const Eigen::Vector3f &position) {
  if (target_spatial_blacklist_.empty() || !position.allFinite() || !ep_) {
    return;
  }
  const double radius = std::max(0.5, ep_->target_spatial_blacklist_radius_);
  target_spatial_blacklist_.erase(
      std::remove_if(target_spatial_blacklist_.begin(),
                     target_spatial_blacklist_.end(),
                     [&](const SpatialBlacklistEntry &entry) {
                       return (entry.position - position).norm() <= radius;
                     }),
      target_spatial_blacklist_.end());
}

void FastExplorationManager::deferGoalIdentity(const int cluster_id,
                                               const Eigen::Vector3f &position,
                                               const double cooldown_sec) {
  if (!ep_ || cooldown_sec <= 0.0) {
    return;
  }
  const ros::Time now = ros::Time::now();
  deferred_goals_.erase(
      std::remove_if(deferred_goals_.begin(), deferred_goals_.end(),
                     [&](const DeferredGoal &goal) {
                       return goal.until.isZero() || goal.until <= now;
                     }),
      deferred_goals_.end());
  DeferredGoal *matched = nullptr;
  const double match_radius = std::max(0.2, ep_->goal_lock_match_radius_);
  for (auto &goal : deferred_goals_) {
    if ((goal.cluster_id >= 0 && goal.cluster_id == cluster_id) ||
        (goal.position - position).norm() <= match_radius) {
      matched = &goal;
      break;
    }
  }
  if (!matched) {
    if (deferred_goals_.size() >= 64U) {
      deferred_goals_.erase(deferred_goals_.begin());
    }
    deferred_goals_.push_back({cluster_id, position, ros::Time(0)});
    matched = &deferred_goals_.back();
  }
  matched->cluster_id = cluster_id;
  matched->position = position;
  const ros::Time until = now + ros::Duration(cooldown_sec);
  if (matched->until.isZero() || until > matched->until) {
    matched->until = until;
  }
}

int FastExplorationManager::unlockOldestDeferredGoals(const int max_count) {
  if (deferred_goals_.empty() || max_count <= 0) {
    return 0;
  }
  const ros::Time now = ros::Time::now();
  std::vector<std::size_t> active_idx;
  active_idx.reserve(deferred_goals_.size());
  for (std::size_t i = 0; i < deferred_goals_.size(); ++i) {
    if (!deferred_goals_[i].until.isZero() && deferred_goals_[i].until > now) {
      active_idx.emplace_back(i);
    }
  }
  if (active_idx.empty()) {
    return 0;
  }
  std::stable_sort(active_idx.begin(), active_idx.end(),
                   [&](const std::size_t lhs, const std::size_t rhs) {
                     return deferred_goals_[lhs].until <
                            deferred_goals_[rhs].until;
                   });
  const int unlock_n =
      std::min(max_count, static_cast<int>(active_idx.size()));
  for (int i = 0; i < unlock_n; ++i) {
    auto &goal = deferred_goals_[active_idx[static_cast<std::size_t>(i)]];
    goal.until = now;
    clearSpatialBlacklistAround(goal.position);
  }
  return unlock_n;
}

int FastExplorationManager::commitTargetDirectedTour(
    const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
    const float curr_yaw, vector<TopoNode::Ptr> &viewpoints,
    vector<TopoNode::Ptr> &viewpoint_reachable,
    vector<double> &viewpoint_reachable_distance,
    vector<EdgeSafetyCost> &viewpoint_reachable_edges) {
  if (viewpoint_reachable.empty()) {
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    last_plan_empty_frontier_ = false;
    last_plan_no_reachable_ = false;
    return FAIL;
  }

  const TargetDirectedExplorationConfig config = targetGuidanceConfig();
  vector<TargetDirectedExplorationScore> scores(viewpoint_reachable.size());
  vector<double> selection_costs(viewpoint_reachable.size(),
                                 std::numeric_limits<double>::infinity());
  int best_idx = 0;
  for (int i = 0; i < static_cast<int>(viewpoint_reachable.size()); ++i) {
    const TopoNode::Ptr &viewpoint = viewpoint_reachable[i];
    scores[i] = scoreTargetDirectedViewpoint(
        pos, ed_->mission_goal_.cast<double>(),
        viewpoint->center_.cast<double>(), config);
    const double travel = i < static_cast<int>(viewpoint_reachable_edges.size())
                              ? viewpoint_reachable_edges[i].time_cost
                              : viewpoint_reachable_distance[i];
    selection_costs[i] = targetDirectedSelectionCost(scores[i], travel) +
                         topologyGuidePenalty(
                             viewpoint->center_.cast<double>());
    if (selection_costs[i] < selection_costs[best_idx] - 1.0e-6) {
      best_idx = i;
    } else if (std::fabs(selection_costs[i] - selection_costs[best_idx]) <=
                   1.0e-6 &&
               scores[i].remaining_distance + 1.0e-6 <
                   scores[best_idx].remaining_distance) {
      best_idx = i;
    }
  }

  int locked_idx = -1;
  if (ed_->has_goal_lock_) {
    double locked_match = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(viewpoint_reachable.size()); ++i) {
      const TopoNode::Ptr &viewpoint = viewpoint_reachable[i];
      const bool same_cluster =
          ed_->locked_goal_cluster_id_ >= 0 &&
          !viewpoint->is_mission_goal_target_ &&
          viewpoint->frontier_cluster_id_ == ed_->locked_goal_cluster_id_;
      const double distance =
          (viewpoint->center_ - ed_->locked_goal_).norm();
      if ((same_cluster && distance <= ep_->goal_lock_match_radius_) ||
          distance < locked_match) {
        if (same_cluster && distance <= ep_->goal_lock_match_radius_) {
          locked_idx = i;
          locked_match = distance;
          break;
        }
        if (distance <= ep_->goal_lock_match_radius_ &&
            distance < locked_match) {
          locked_idx = i;
          locked_match = distance;
        }
      }
    }
  }

  int chosen_idx = best_idx;
  if (locked_idx >= 0 && locked_idx != best_idx &&
      shouldRetainTargetGoalLock(scores[locked_idx], scores[best_idx],
                                 ep_->target_frontier_min_progress_,
                                 ep_->target_goal_lock_remaining_margin_)) {
    chosen_idx = locked_idx;
    ROS_INFO_STREAM_THROTTLE(
        0.5, "[target exploration] keep locked frontier bridge cluster="
                 << viewpoint_reachable[chosen_idx]->frontier_cluster_id_
                 << " remaining=" << scores[chosen_idx].remaining_distance
                 << " candidate_remaining="
                 << scores[best_idx].remaining_distance);
  }

  const double escape_min_progress =
      ep_ ? ep_->target_escape_min_progress_ : -1.0;
  const double near_goal_remaining =
      ep_ ? ep_->target_near_goal_remaining_ : 15.0;
  const double mission_remaining_now = missionGoalDistance(pos);
  const bool near_goal = std::isfinite(mission_remaining_now) &&
                         mission_remaining_now <= near_goal_remaining;
  const double commit_min_progress =
      near_goal ? 0.0 : escape_min_progress;
  auto isCommitEligible = [&](const int idx) {
    if (idx < 0 || idx >= static_cast<int>(viewpoint_reachable.size())) {
      return false;
    }
    const TopoNode::Ptr &vp = viewpoint_reachable[idx];
    if (!vp) {
      return false;
    }
    if (vp->is_mission_goal_target_) {
      return true;
    }
    if (isSpatiallyBlacklisted(vp->center_)) {
      return false;
    }
    if (!scores[idx].valid) {
      return false;
    }
    return scores[idx].progress_distance + 1.0e-6 >= commit_min_progress;
  };

  // Prefer the locked/best bridge, then try cheaper destination-ranked
  // alternatives.  Large regressions are never attempted: hovering while the
  // map grows is safer than flying back tens of metres.
  vector<int> try_order;
  try_order.reserve(viewpoint_reachable.size());
  if (isCommitEligible(chosen_idx)) {
    try_order.emplace_back(chosen_idx);
  }
  vector<int> ranked_rest;
  ranked_rest.reserve(viewpoint_reachable.size());
  for (int i = 0; i < static_cast<int>(viewpoint_reachable.size()); ++i) {
    if (i == chosen_idx || !isCommitEligible(i)) {
      continue;
    }
    ranked_rest.emplace_back(i);
  }
  std::stable_sort(ranked_rest.begin(), ranked_rest.end(),
                   [&](const int lhs, const int rhs) {
                     if (selection_costs[lhs] != selection_costs[rhs]) {
                       return selection_costs[lhs] < selection_costs[rhs];
                     }
                     return scores[lhs].remaining_distance <
                            scores[rhs].remaining_distance;
                   });
  try_order.insert(try_order.end(), ranked_rest.begin(), ranked_rest.end());
  if (try_order.empty()) {
    ROS_WARN_STREAM_THROTTLE(
        0.5, "[target exploration] no non-regressing bridge to commit "
                 "(escape_min_progress="
                 << escape_min_progress << " near_goal=" << near_goal
                 << " remaining=" << mission_remaining_now
                 << "); wait for map/forward frontier");
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    return FAIL;
  }

  int executable_idx = -1;
  vector<int> search_failed_idx;
  search_failed_idx.reserve(try_order.size());
  int backup_skipped = 0;
  bool any_edge_backup = false;
  for (const int idx : try_order) {
    if (idx < static_cast<int>(viewpoint_reachable_edges.size()) &&
        viewpoint_reachable_edges[idx].backup_feasible) {
      any_edge_backup = true;
      break;
    }
  }
  for (const int idx : try_order) {
    // Prefer bridges the edge layer already certified.  If none exist, still
    // attempt local preflight: early map growth often has short known-free
    // runways that the short-horizon check accepts.  Never soft-defer solely
    // because edge backup=0 — that emptied the shortlist every tick.
    const bool edge_backup_ok =
        idx >= static_cast<int>(viewpoint_reachable_edges.size()) ||
        viewpoint_reachable_edges[idx].backup_feasible;
    if (any_edge_backup && !edge_backup_ok) {
      ++backup_skipped;
      continue;
    }
    if (targetBridgeLocallyExecutable(viewpoint_reachable[idx], pos, vel,
                                      curr_yaw, nullptr)) {
      executable_idx = idx;
      break;
    }
    search_failed_idx.emplace_back(idx);
  }
  if (executable_idx < 0) {
    const TopoNode::Ptr &primary = viewpoint_reachable[try_order.front()];
    const int fail_limit =
        ep_ ? ep_->target_local_preflight_fail_limit_ : 3;
    const double soft_defer =
        ep_ ? ep_->target_preflight_soft_defer_ : 1.5;
    const double blacklist_duration =
        ep_ ? ep_->target_spatial_blacklist_duration_ : 8.0;
    int max_fails = 0;
    for (const int idx : search_failed_idx) {
      const TopoNode::Ptr &vp = viewpoint_reachable[idx];
      if (!vp || vp->is_mission_goal_target_) {
        continue;
      }
      const int fails = noteLocalPreflightFailure(vp);
      max_fails = std::max(max_fails, fails);
      deferGoalIdentity(vp->frontier_cluster_id_, vp->center_, soft_defer);
      rememberSpatialBlacklist(vp->center_, blacklist_duration);
      if (fails >= fail_limit) {
        deferGoalIdentity(vp->frontier_cluster_id_, vp->center_,
                          ep_->failed_goal_cooldown_);
        clearLocalPreflightFailure(vp);
      }
    }
    ROS_WARN_STREAM_THROTTLE(
        0.5, "[target exploration] no locally executable non-regressing "
                 "bridge; primary cluster="
                 << primary->frontier_cluster_id_
                 << " tried=" << try_order.size()
                 << " backup_skipped=" << backup_skipped
                 << " search_failed=" << search_failed_idx.size()
                 << " max_consecutive_fails=" << max_fails << "/" << fail_limit);
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    return FAIL;
  }
  if (executable_idx != chosen_idx) {
    ROS_INFO_STREAM_THROTTLE(
        0.5, "[target exploration] promote next-best locally executable "
                 "bridge cluster="
                 << viewpoint_reachable[executable_idx]->frontier_cluster_id_
                 << " after rejecting cluster="
                 << (isCommitEligible(chosen_idx)
                         ? viewpoint_reachable[chosen_idx]->frontier_cluster_id_
                         : -1));
  }
  chosen_idx = executable_idx;
  const TopoNode::Ptr selected = viewpoint_reachable[chosen_idx];
  clearLocalPreflightFailure(selected);
  clearSpatialBlacklistAround(selected->center_);

  if (updateNormalGoalProgress(selected, viewpoint_reachable_distance[chosen_idx],
                               pos, scores[chosen_idx].remaining_distance)) {
    ed_->has_goal_lock_ = true;
    ed_->locked_goal_is_coverage_ = false;
    ed_->locked_goal_is_mission_ = false;
    ed_->locked_goal_cluster_id_ = selected->frontier_cluster_id_;
    ed_->locked_goal_coverage_id_ = 0;
    ed_->locked_goal_ = selected->center_;
    ed_->locked_goal_yaw_ = selected->yaw_;
    deferCurrentGoalAfterPlanningFailure();
    if (selected) {
      rememberSpatialBlacklist(
          selected->center_,
          ep_ ? ep_->target_spatial_blacklist_duration_ : 8.0);
    }
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    return planGlobalPath(pos, vel);
  }

  if (has_active_coverage_goal_) {
    deferCoverageRecovery(active_coverage_target_,
                          CoverageRecoveryOutcome::FRONTIER_RESUMED);
  }

  const bool same_identity =
      ed_->has_goal_lock_ && !ed_->locked_goal_is_coverage_ &&
      !ed_->locked_goal_is_mission_ &&
      selected->frontier_cluster_id_ == ed_->locked_goal_cluster_id_ &&
      (selected->center_ - ed_->locked_goal_).norm() <=
          ep_->goal_lock_match_radius_;
  ed_->has_goal_lock_ = true;
  ed_->locked_goal_is_mission_ = selected->is_mission_goal_target_;
  ed_->locked_goal_is_coverage_ = false;
  ed_->locked_goal_cluster_id_ = selected->is_mission_goal_target_
                                     ? -1
                                     : selected->frontier_cluster_id_;
  ed_->locked_goal_coverage_id_ = 0;
  ed_->locked_goal_ = selected->center_;
  ed_->locked_goal_yaw_ = selected->yaw_;
  ed_->locked_goal_cost_ = selection_costs[chosen_idx];
  if (!same_identity) {
    ed_->locked_goal_time_ = ros::Time::now();
    if (frontier_manager_ptr_ && !selected->is_mission_goal_target_ &&
        selected->frontier_cluster_id_ >= 0) {
      frontier_manager_ptr_->markClusterGoalSelected(
          selected->frontier_cluster_id_, selected->center_,
          ep_->goal_lock_match_radius_);
    }
  }

  ROS_INFO_STREAM_THROTTLE(
      0.5, "[target exploration] select frontier bridge cluster="
               << selected->frontier_cluster_id_ << " J="
               << selection_costs[chosen_idx] << " travel="
               << (chosen_idx < static_cast<int>(viewpoint_reachable_edges.size())
                       ? viewpoint_reachable_edges[chosen_idx].time_cost
                       : viewpoint_reachable_distance[chosen_idx])
               << " remaining=" << scores[chosen_idx].remaining_distance
               << " progress=" << scores[chosen_idx].progress_distance
               << " lateral=" << scores[chosen_idx].lateral_distance
               << " goal=(" << selected->center_.transpose() << ")");

  ed_->global_tour_.clear();
  ed_->global_tour_.emplace_back(pos.cast<float>());
  ed_->global_tour_.emplace_back(selected->center_);
  planner_manager_->local_data_.end_yaw_ = selected->yaw_;
  planner_manager_->topo_graph_->removeNodes(viewpoints);
  planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED,
                                               "global");
  updateGoalNode();
  target_empty_pool_since_ = ros::Time(0);
  return SUCCEED;
}

double FastExplorationManager::getPathCost(TopoNode::Ptr &n1,
                                           Eigen::Vector3d v1, float &yaw1,
                                           TopoNode::Ptr &n2, float &yaw2) {
  auto estimateCost = [&](TopoNode::Ptr &n1, Eigen::Vector3d v1, float &yaw1,
                          TopoNode::Ptr &n2, float &yaw2, int res,
                          vector<Eigen::Vector3f> &path) -> double {
    double len_cost, yaw_cost, dir_cost;
    len_cost = yaw_cost = dir_cost = 0.0;
    if (res == BubbleAstar::NO_PATH)
      return 2e3 + (n1->center_ - n2->center_)
                       .norm(); // 使用一个大的时间值表示无法到达
    if (res == BubbleAstar::START_FAIL || res == BubbleAstar::END_FAIL)
      return 2e3 +
             (n1->center_ - n2->center_).norm(); // 同上，用于不同的错误情况

    if (ep_->epic_simple_global_cost_) {
      // Match EPIC's global routing objective: use only estimated travel time.
      // Direction changes are handled by the local reorientation/safety layer;
      // charging them again here systematically postpones nearby side rooms.
      for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Eigen::Vector3f delta = path[i + 1] - path[i];
        len_cost += delta.norm() + 0.5 * std::fabs(delta.z());
      }
      len_cost /= std::max(0.1, ep_->v_max_ / 2.0);
    } else {
      const EdgeSafetyCost edge_cost =
          planner_manager_->estimateHighSpeedEdgeCost(path, v1, yaw1, yaw2);
      len_cost = edge_cost.total_cost;
    }

    // if (v1.norm() > 1e-3) {
    //   Eigen::Vector3f dir = n2->center_ - n1->center_;
    //   dir.normalize();
    //   Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    //   float yaw1 = atan2(dir.y(), dir.x());
    //   float yaw2 = atan2(v_dir.y(), v_dir.x());
    //   float diff = yaw1 - yaw2;
    //   while (diff > M_PI)
    //     diff -= 2.0 * M_PI;
    //   while (diff < -M_PI)
    //     diff += 2.0 * M_PI;
    //   dir_cost = ep_->w_vdir_ * (fabs(diff) /
    //   planner_manager_->gcopter_config_->yaw_max_vel);
    // }

    // if (path.size() >= 2) {
    //   planner_manager_->calculateTimelb(path, yaw1, yaw2, yaw_cost);
    //   yaw_cost *= ep_->w_yawdir_;
    // }

    return len_cost + dir_cost;
    // return len_cost + dir_cost;
  };
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->fast_searcher_->topoSearch(n1, n2, 1e-2, path);
  return estimateCost(n1, v1, yaw1, n2, yaw2, res, path);
}

double FastExplorationManager::getPathCostWithoutTopo(TopoNode::Ptr &n1,
                                                      Eigen::Vector3d v1,
                                                      float &yaw1,
                                                      TopoNode::Ptr &n2,
                                                      float &yaw2) {
  vector<Eigen::Vector3f> path;
  int res = planner_manager_->parallel_path_finder_->search(
      n1->center_, n2->center_, path, 1.0, false);
  if (res != ParallelBubbleAstar::REACH_END)
    return 2e3;
  if (ep_->epic_simple_global_cost_) {
    double path_cost = 0.0;
    planner_manager_->parallel_path_finder_->calculatePathCost(path, path_cost);
    return path_cost;
  }
  return planner_manager_->estimateHighSpeedEdgeCost(path, v1, yaw1, yaw2)
      .total_cost;
}

int FastExplorationManager::selectStableGoalIndex(
    const vector<TopoNode::Ptr> &viewpoints,
    const vector<double> &distance_odom2vp, const int candidate_idx,
    const Eigen::Vector3d &vel) {
  if (!ep_->goal_lock_enable_ || viewpoints.empty() || candidate_idx < 0 ||
      candidate_idx >= static_cast<int>(viewpoints.size())) {
    return candidate_idx;
  }

  const ros::Time now = ros::Time::now();
  int locked_idx = -1;
  double locked_match_distance = std::numeric_limits<double>::max();
  if (ed_->has_goal_lock_) {
    if (ed_->locked_goal_is_mission_) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        if (viewpoints[i]->is_mission_goal_target_) {
          locked_idx = i;
          locked_match_distance =
              (viewpoints[i]->center_ - ed_->locked_goal_).norm();
          break;
        }
      }
    } else if (ed_->locked_goal_is_coverage_ &&
        ed_->locked_goal_coverage_id_ != 0) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        if (viewpoints[i]->is_coverage_target_ &&
            viewpoints[i]->coverage_target_id_ ==
                ed_->locked_goal_coverage_id_) {
          locked_idx = i;
          locked_match_distance =
              (viewpoints[i]->center_ - ed_->locked_goal_).norm();
          break;
        }
      }
    } else if (ed_->locked_goal_cluster_id_ >= 0) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        if (!viewpoints[i]->is_coverage_target_ &&
            viewpoints[i]->frontier_cluster_id_ ==
            ed_->locked_goal_cluster_id_) {
          const double distance =
              (viewpoints[i]->center_ - ed_->locked_goal_).norm();
          // A globally cached cluster may retain its numeric ID while its best
          // observation point moves to another doorway/side of a room.  Do not
          // force that spatially different target through the old goal lock.
          if (distance <= ep_->goal_lock_match_radius_) {
            locked_idx = i;
            locked_match_distance = distance;
            break;
          }
        }
      }
    }
    if (locked_idx < 0) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        const double distance =
            (viewpoints[i]->center_ - ed_->locked_goal_).norm();
        if (distance < locked_match_distance) {
          locked_match_distance = distance;
          locked_idx = i;
        }
      }
      if (locked_match_distance > ep_->goal_lock_match_radius_) {
        locked_idx = -1;
      }
    }
  }

  int chosen_idx = candidate_idx;
  if (locked_idx >= 0 && locked_idx != candidate_idx) {
    const double candidate_cost =
        candidate_idx < static_cast<int>(distance_odom2vp.size())
            ? distance_odom2vp[candidate_idx]
            : 0.0;
    const double locked_cost =
        locked_idx < static_cast<int>(distance_odom2vp.size())
            ? distance_odom2vp[locked_idx]
            : ed_->locked_goal_cost_;
    const double since_lock = (now - ed_->locked_goal_time_).toSec();
    // Goal switching becomes expensive before the exact high-speed threshold.
    // Waiting until v >= MaxVelMag made the high-speed multiplier practically
    // unreachable in normal flight (the logged odometry usually stays just
    // below the configured maximum).
    const bool high_speed =
        vel.norm() >=
        planner_manager_->gcopter_config_->highSpeedModeExitThreshold;
    const double high_speed_multiplier =
        high_speed ? std::max(1.0, ep_->goal_switch_high_speed_multiplier_) : 1.0;
    const double min_improvement =
        ep_->goal_switch_min_improvement_ * high_speed_multiplier;
    const double candidate_improvement = locked_cost - candidate_cost;
    const bool in_cooldown =
        since_lock < ep_->goal_switch_min_interval_ * high_speed_multiplier;
    const double relative_margin =
        std::max(0.0, ep_->goal_keep_cost_ratio_ - 1.0) *
        std::max(1.0, std::fabs(candidate_cost));
    const bool old_goal_cost_ok =
        locked_cost <= candidate_cost + relative_margin + min_improvement;

    if (in_cooldown || old_goal_cost_ok ||
        candidate_improvement < min_improvement) {
      chosen_idx = locked_idx;
      ROS_INFO_STREAM_THROTTLE(
          0.5,
          "[goal lock] keep goal idx=" << locked_idx
                                      << " candidate_idx=" << candidate_idx
                                      << " locked_cost=" << locked_cost
                                      << " candidate_cost=" << candidate_cost
                                      << " since=" << since_lock
                                      << " high_speed=" << high_speed);
    }
  }

  // Frontier IDs are regenerated during reclustering.  A nearby replacement
  // is the same logical goal even if its numeric ID changed.
  const bool chosen_is_mission =
      viewpoints[chosen_idx]->is_mission_goal_target_;
  const bool chosen_is_coverage =
      !chosen_is_mission && viewpoints[chosen_idx]->is_coverage_target_;
  const bool same_identity =
      ed_->has_goal_lock_ &&
      chosen_is_mission == ed_->locked_goal_is_mission_ &&
      chosen_is_coverage == ed_->locked_goal_is_coverage_ &&
      (chosen_is_mission
           ? true
           : chosen_is_coverage
           ? viewpoints[chosen_idx]->coverage_target_id_ ==
                 ed_->locked_goal_coverage_id_
           : viewpoints[chosen_idx]->frontier_cluster_id_ ==
                     ed_->locked_goal_cluster_id_ &&
                 (viewpoints[chosen_idx]->center_ - ed_->locked_goal_).norm() <=
                     ep_->goal_lock_match_radius_);
  const bool new_lock =
      !same_identity &&
      (!ed_->has_goal_lock_ ||
       (viewpoints[chosen_idx]->center_ - ed_->locked_goal_).norm() >
           ep_->goal_lock_match_radius_);
  ed_->has_goal_lock_ = true;
  ed_->locked_goal_is_mission_ = chosen_is_mission;
  ed_->locked_goal_is_coverage_ = chosen_is_coverage;
  ed_->locked_goal_cluster_id_ =
      (chosen_is_coverage || chosen_is_mission)
          ? -1
          : viewpoints[chosen_idx]->frontier_cluster_id_;
  ed_->locked_goal_coverage_id_ =
      chosen_is_coverage ? viewpoints[chosen_idx]->coverage_target_id_ : 0;
  ed_->locked_goal_ = viewpoints[chosen_idx]->center_;
  ed_->locked_goal_yaw_ = viewpoints[chosen_idx]->yaw_;
  ed_->locked_goal_cost_ =
      chosen_idx < static_cast<int>(distance_odom2vp.size())
          ? distance_odom2vp[chosen_idx]
          : 0.0;
  if (new_lock) {
    ed_->locked_goal_time_ = now;
    if (frontier_manager_ptr_ && !chosen_is_coverage && !chosen_is_mission) {
      frontier_manager_ptr_->markClusterGoalSelected(
          viewpoints[chosen_idx]->frontier_cluster_id_,
          viewpoints[chosen_idx]->center_, ep_->goal_lock_match_radius_);
    }
  }
  return chosen_idx;
}

void FastExplorationManager::deferCurrentGoalAfterPlanningFailure() {
  if (planner_manager_ && planner_manager_->fast_searcher_) {
    planner_manager_->fast_searcher_->clearPathCache();
  }
  if (has_active_coverage_goal_) {
    deferCoverageRecovery(active_coverage_target_,
                          CoverageRecoveryOutcome::TRAJECTORY_FAILURE);
  }
  if (!ed_ || !ep_ || !ed_->has_goal_lock_) {
    return;
  }
  if (ed_->locked_goal_is_coverage_) {
    ed_->has_goal_lock_ = false;
    ed_->locked_goal_is_coverage_ = false;
    ed_->locked_goal_coverage_id_ = 0;
    return;
  }
  const ros::Time now = ros::Time::now();
  if (ed_->locked_goal_is_mission_) {
    mission_goal_direct_retry_after_ =
        now + ros::Duration(ep_->target_direct_retry_delay_);
    ed_->has_goal_lock_ = false;
    ed_->locked_goal_is_mission_ = false;
    ROS_WARN_STREAM("[target exploration] known-free endpoint failed local "
                    "planning; resume frontier bridge for "
                    << ep_->target_direct_retry_delay_ << "s");
    return;
  }
  deferred_goals_.erase(
      std::remove_if(deferred_goals_.begin(), deferred_goals_.end(),
                     [&](const DeferredGoal &goal) {
                       return goal.until.isZero() || goal.until <= now;
                     }),
      deferred_goals_.end());
  DeferredGoal *matched = nullptr;
  for (auto &goal : deferred_goals_) {
    if ((goal.cluster_id >= 0 &&
         goal.cluster_id == ed_->locked_goal_cluster_id_) ||
        (goal.position - ed_->locked_goal_).norm() <=
            std::max(0.2, ep_->goal_lock_match_radius_)) {
      matched = &goal;
      break;
    }
  }
  if (!matched) {
    if (deferred_goals_.size() >= 64U) {
      deferred_goals_.erase(deferred_goals_.begin());
    }
    deferred_goals_.push_back(
        {ed_->locked_goal_cluster_id_, ed_->locked_goal_, ros::Time(0)});
    matched = &deferred_goals_.back();
  }
  matched->cluster_id = ed_->locked_goal_cluster_id_;
  matched->position = ed_->locked_goal_;
  matched->until = now + ros::Duration(ep_->failed_goal_cooldown_);
  ROS_WARN_STREAM("[plan recovery] temporarily defer failed goal cluster="
                  << matched->cluster_id << " goal=("
                  << matched->position.transpose() << ") cooldown="
                  << ep_->failed_goal_cooldown_ << "s penalty="
                  << ep_->failed_goal_penalty_
                  << " active_deferred=" << deferred_goals_.size());
  resetNormalGoalProgress();
}

void FastExplorationManager::resetNormalGoalProgress() {
  normal_goal_progress_ = NormalGoalProgress();
}

bool FastExplorationManager::updateNormalGoalProgress(
    const TopoNode::Ptr &viewpoint, const double route_cost,
    const Eigen::Vector3d &robot_position, const double mission_remaining) {
  if (!frontier_progress_watchdog_enable_ || !viewpoint ||
      viewpoint->is_coverage_target_ || viewpoint->is_mission_goal_target_ ||
      viewpoint->frontier_cluster_id_ < 0 || !std::isfinite(route_cost)) {
    if (viewpoint && (viewpoint->is_coverage_target_ ||
                      viewpoint->is_mission_goal_target_)) {
      resetNormalGoalProgress();
    }
    return false;
  }

  const ros::Time now = ros::Time::now();
  const double goal_distance =
      (viewpoint->center_.cast<double>() - robot_position).norm();
  const bool sample_gap =
      normal_goal_progress_.valid &&
      !normal_goal_progress_.last_sample_time.isZero() &&
      (now - normal_goal_progress_.last_sample_time).toSec() >
          std::max(3.0, 0.5 * frontier_progress_timeout_);
  if (sample_gap) {
    resetNormalGoalProgress();
  }
  const bool same_cluster =
      normal_goal_progress_.valid &&
      normal_goal_progress_.cluster_id == viewpoint->frontier_cluster_id_;
  const bool nearby_reclustered_goal =
      normal_goal_progress_.valid &&
      (normal_goal_progress_.goal - viewpoint->center_).norm() <=
          std::max(0.2, ep_->goal_lock_match_radius_);
  if (!same_cluster && !nearby_reclustered_goal) {
    normal_goal_progress_.valid = true;
    normal_goal_progress_.cluster_id = viewpoint->frontier_cluster_id_;
    normal_goal_progress_.goal = viewpoint->center_;
    normal_goal_progress_.best_route_cost = route_cost;
    normal_goal_progress_.best_goal_distance = goal_distance;
    normal_goal_progress_.best_remaining_distance = mission_remaining;
    normal_goal_progress_.last_progress_time = now;
    normal_goal_progress_.last_sample_time = now;
    normal_goal_progress_.samples = 1;
    return false;
  }

  normal_goal_progress_.cluster_id = viewpoint->frontier_cluster_id_;
  normal_goal_progress_.goal = viewpoint->center_;
  normal_goal_progress_.last_sample_time = now;
  ++normal_goal_progress_.samples;
  const bool remaining_progress =
      std::isfinite(mission_remaining) &&
      std::isfinite(normal_goal_progress_.best_remaining_distance) &&
      mission_remaining <= normal_goal_progress_.best_remaining_distance -
                               frontier_progress_min_distance_drop_;
  const bool route_progress =
      route_cost <= normal_goal_progress_.best_route_cost -
                        frontier_progress_min_cost_drop_;
  const bool spatial_progress =
      !std::isfinite(mission_remaining) &&
      goal_distance <= normal_goal_progress_.best_goal_distance -
                           frontier_progress_min_distance_drop_;
  if (remaining_progress || route_progress || spatial_progress) {
    normal_goal_progress_.best_route_cost =
        std::min(normal_goal_progress_.best_route_cost, route_cost);
    normal_goal_progress_.best_goal_distance =
        std::min(normal_goal_progress_.best_goal_distance, goal_distance);
    if (std::isfinite(mission_remaining)) {
      normal_goal_progress_.best_remaining_distance =
          std::min(normal_goal_progress_.best_remaining_distance,
                   mission_remaining);
    }
    normal_goal_progress_.last_progress_time = now;
    return false;
  }

  if (normal_goal_progress_.last_progress_time.isZero()) {
    normal_goal_progress_.last_progress_time = now;
    return false;
  }
  const double stalled_duration =
      (now - normal_goal_progress_.last_progress_time).toSec();
  if (normal_goal_progress_.samples < 4 ||
      stalled_duration < frontier_progress_timeout_) {
    return false;
  }

  ROS_WARN_STREAM(
      "[frontier progress] defer non-progressing ordinary frontier: cluster="
      << viewpoint->frontier_cluster_id_ << " goal=("
      << viewpoint->center_.transpose() << ") stalled=" << stalled_duration
      << "s route=" << route_cost
      << " best_route=" << normal_goal_progress_.best_route_cost
      << " distance=" << goal_distance
      << " best_distance=" << normal_goal_progress_.best_goal_distance
      << " remaining=" << mission_remaining
      << " best_remaining=" << normal_goal_progress_.best_remaining_distance
      << " samples=" << normal_goal_progress_.samples);
  return true;
}

bool FastExplorationManager::selectSafeCoverageApproach(
    CoverageTarget &target, const bool record_terminal_failure) {
  std::vector<Eigen::Vector3d> candidates = target.approach_candidates;
  if (candidates.empty() && target.has_approach) {
    candidates.emplace_back(target.approach_position);
  }
  const double required_clearance =
      std::max(planner_manager_->topo_graph_->bubble_min_radius_,
               std::max(
                   planner_manager_->gcopter_config_
                       ->commitKnownFreeSafeDistance,
                   planner_manager_->gcopter_config_->dilateRadiusSoft +
                       planner_manager_->gcopter_config_
                           ->safetyClearanceTolerance));
  bool saw_occlusion = false;
  for (const Eigen::Vector3d &approach : candidates) {
    if (!approach.allFinite() ||
        !planner_manager_->lidar_map_interface_->IsInBox(
            approach.cast<float>()) ||
        planner_manager_->safetyDistanceToOcc(approach) <
            required_clearance) {
      continue;
    }
    const RaycastSafetyInfo observation_ray = planner_manager_->raycastSafety(
        approach, target.position, false, 0.05, 0.15);
    if (observation_ray.blocked_by_occupied ||
        observation_ray.first_blocked_state == MapVoxelState::OUT_OF_MAP) {
      saw_occlusion = true;
      continue;
    }
    target.approach_position = approach;
    target.has_approach = true;
    return true;
  }
  if (record_terminal_failure) {
    deferCoverageRecovery(
        target, saw_occlusion ? CoverageRecoveryOutcome::OCCLUDED
                              : CoverageRecoveryOutcome::UNSAFE);
  }
  return false;
}

bool FastExplorationManager::coverageRecoveryDeferred(
    const CoverageTarget &target, const ros::Time &now) const {
  return std::any_of(
      deferred_coverage_goals_.begin(), deferred_coverage_goals_.end(),
      [&](const DeferredCoverageGoal &goal) {
        return (goal.exhausted || goal.until > now) &&
               coverageRecoveryIdentityMatches(
                   goal.identity, target, coverage_recovery_match_radius_);
      });
}

bool FastExplorationManager::coverageRecoveryExhausted(
    const CoverageTarget &target) const {
  return std::any_of(
      deferred_coverage_goals_.begin(), deferred_coverage_goals_.end(),
      [&](const DeferredCoverageGoal &goal) {
        return goal.exhausted &&
               coverageRecoveryIdentityMatches(
                   goal.identity, target, coverage_recovery_match_radius_);
      });
}

void FastExplorationManager::rememberCoverageRecoveryAlias(
    const CoverageTarget &target) {
  for (DeferredCoverageGoal &goal : deferred_coverage_goals_) {
    if (!coverageRecoveryIdentityMatches(
            goal.identity, target, coverage_recovery_match_radius_)) {
      continue;
    }
    rememberCoverageStableId(goal.identity, target.stable_id);
    return;
  }
}

bool FastExplorationManager::coverageRecoveryCooling(
    const CoverageTarget &target, const ros::Time &now,
    double *remaining) const {
  for (const DeferredCoverageGoal &goal : deferred_coverage_goals_) {
    if (!coverageRecoveryIdentityMatches(
            goal.identity, target, coverage_recovery_match_radius_) ||
        goal.exhausted ||
        goal.until.isZero() || goal.until <= now) {
      continue;
    }
    if (remaining) {
      *remaining = std::max(0.0, (goal.until - now).toSec());
    }
    return true;
  }
  if (remaining) {
    *remaining = 0.0;
  }
  return false;
}

bool FastExplorationManager::coverageTerminalRetryReady(
    const CoverageTarget &target, const ros::Time &now,
    double *retry_after) const {
  for (const DeferredCoverageGoal &goal : deferred_coverage_goals_) {
    if (!coverageRecoveryIdentityMatches(
            goal.identity, target, coverage_recovery_match_radius_) ||
        goal.exhausted ||
        goal.until.isZero() || goal.until <= now) {
      continue;
    }
    const ros::Time normal_attempt_time =
        goal.until - ros::Duration(coverage_recovery_cooldown_);
    const ros::Time terminal_retry_time =
        normal_attempt_time +
        ros::Duration(coverage_terminal_retry_interval_);
    const double remaining =
        std::max(0.0, (terminal_retry_time - now).toSec());
    if (retry_after) {
      *retry_after = remaining;
    }
    return coverage_terminal_retry_enable_ && remaining <= 1.0e-6;
  }
  if (retry_after) {
    *retry_after = 0.0;
  }
  return false;
}

void FastExplorationManager::pruneDeferredCoverageGoals(
    const ros::Time &now) {
  // Preserve records that carry an attempt count or permanent exhaustion.
  // Only outcome-free entries (for example a target superseded when a normal
  // frontier resumed) may disappear after their cooldown.
  deferred_coverage_goals_.erase(
      std::remove_if(
          deferred_coverage_goals_.begin(), deferred_coverage_goals_.end(),
          [&](const DeferredCoverageGoal &goal) {
            return !goal.exhausted && goal.no_gain_attempts == 0 &&
                   goal.failure_attempts == 0 &&
                   (goal.until.isZero() || goal.until <= now);
          }),
      deferred_coverage_goals_.end());
}

int FastExplorationManager::latestCoverageObservedVoxels() const {
  if (!coverage_guidance_) {
    return -1;
  }
  const CoveragePlan::Ptr plan = coverage_guidance_->latestUsablePlan();
  return plan ? plan->observed_voxel_count : -1;
}

const char *FastExplorationManager::coverageRecoveryOutcomeName(
    CoverageRecoveryOutcome outcome) {
  switch (outcome) {
    case CoverageRecoveryOutcome::REACHED:
      return "observation point reached";
    case CoverageRecoveryOutcome::TIMEOUT:
      return "target timeout";
    case CoverageRecoveryOutcome::TRAJECTORY_FAILURE:
      return "trajectory failure";
    case CoverageRecoveryOutcome::UNSAFE:
      return "unsafe observation point";
    case CoverageRecoveryOutcome::DISCONNECTED:
      return "topology disconnected";
    case CoverageRecoveryOutcome::OCCLUDED:
      return "observation ray occluded";
    case CoverageRecoveryOutcome::FRONTIER_RESUMED:
      return "frontend frontier resumed";
  }
  return "unknown";
}

void FastExplorationManager::deferCoverageRecovery(
    const CoverageTarget &target_in, CoverageRecoveryOutcome outcome) {
  // Callers commonly pass active_coverage_target_. Keep a value copy because
  // this function clears that member before emitting its outcome log.
  const CoverageTarget target = target_in;
  const ros::Time now = ros::Time::now();
  pruneDeferredCoverageGoals(now);
  DeferredCoverageGoal *matched = nullptr;
  for (auto &goal : deferred_coverage_goals_) {
    if (coverageRecoveryIdentityMatches(
            goal.identity, target, coverage_recovery_match_radius_)) {
      matched = &goal;
      break;
    }
  }
  if (!matched) {
    if (deferred_coverage_goals_.size() >= 256U) {
      const auto removable = std::find_if(
          deferred_coverage_goals_.begin(), deferred_coverage_goals_.end(),
          [](const DeferredCoverageGoal &goal) {
            return !goal.exhausted;
          });
      if (removable != deferred_coverage_goals_.end()) {
        deferred_coverage_goals_.erase(removable);
      } else {
        ROS_ERROR_THROTTLE(
            1.0, "[coverage recovery] exhausted-goal registry full");
        return;
      }
    }
    DeferredCoverageGoal goal;
    rememberCoverageStableId(goal.identity, target.stable_id);
    goal.identity.approach = target.approach_position;
    goal.identity.has_approach = target.has_approach;
    goal.unknown_position = target.position;
    goal.voxel_count = target.voxel_count;
    deferred_coverage_goals_.push_back(goal);
    matched = &deferred_coverage_goals_.back();
  }
  // A regenerated coverage component can carry a new stable id while
  // selecting the same canonical executable approach. Preserve the primary
  // id and remember the new id as an alias; overwriting the id here caused two
  // aliases at one disconnected approach to resurrect each other forever.
  rememberCoverageStableId(matched->identity, target.stable_id);
  if (!matched->identity.has_approach && target.has_approach) {
    matched->identity.approach = target.approach_position;
    matched->identity.has_approach = true;
  }
  matched->unknown_position = target.position;
  matched->voxel_count = target.voxel_count;
  const int current_observed = latestCoverageObservedVoxels();
  const bool same_active_id =
      target.stable_id != 0 &&
      active_coverage_target_.stable_id == target.stable_id;
  const bool matches_active =
      has_active_coverage_goal_ &&
      (same_active_id ||
       (active_coverage_target_.approach_position -
        target.approach_position).norm() <= coverage_recovery_match_radius_);
  const int observed_gain =
      matches_active && active_coverage_observed_voxels_ >= 0 &&
              current_observed >= active_coverage_observed_voxels_
          ? current_observed - active_coverage_observed_voxels_
          : 0;

  switch (outcome) {
    case CoverageRecoveryOutcome::REACHED:
      if (observed_gain < coverage_recovery_min_gain_voxels_) {
        ++matched->no_gain_attempts;
      } else {
        matched->no_gain_attempts = 0;
      }
      break;
    case CoverageRecoveryOutcome::TIMEOUT:
    case CoverageRecoveryOutcome::TRAJECTORY_FAILURE:
      ++matched->failure_attempts;
      break;
    case CoverageRecoveryOutcome::DISCONNECTED:
      // A skeleton/map update may make this approach reachable later. Apply
      // the normal cooldown and require repeated failures before exhaustion.
      ++matched->failure_attempts;
      break;
    case CoverageRecoveryOutcome::UNSAFE:
    case CoverageRecoveryOutcome::OCCLUDED:
      matched->exhausted = true;
      break;
    case CoverageRecoveryOutcome::FRONTIER_RESUMED:
      break;
  }
  matched->exhausted =
      matched->exhausted ||
      matched->no_gain_attempts >=
          coverage_recovery_max_no_gain_attempts_ ||
      matched->failure_attempts >=
          coverage_recovery_max_failure_attempts_;
  matched->until =
      matched->exhausted
          ? ros::TIME_MAX
          : now + ros::Duration(coverage_recovery_cooldown_);
  has_active_coverage_goal_ = false;
  active_coverage_target_ = CoverageTarget();
  active_coverage_goal_start_ = ros::Time(0);
  active_coverage_observed_voxels_ = -1;
  ROS_WARN_STREAM("[coverage recovery] defer observation goal: id="
                  << target.stable_id << " approach=("
                  << target.approach_position.transpose() << ") reason="
                  << coverageRecoveryOutcomeName(outcome)
                  << " observed_gain=" << observed_gain
                  << " no_gain_attempts=" << matched->no_gain_attempts
                  << " failure_attempts=" << matched->failure_attempts
                  << " exhausted=" << matched->exhausted
                  << " cooldown=" << coverage_recovery_cooldown_
                  << "s active_deferred="
                  << deferred_coverage_goals_.size());
}

bool FastExplorationManager::completeActiveCoverageGoalIfReached(
    const Eigen::Vector3d &pos) {
  if (!has_active_coverage_goal_ ||
      (pos - active_coverage_target_.approach_position).norm() >
          coverage_recovery_reached_radius_) {
    return false;
  }
  const CoverageTarget reached_target = active_coverage_target_;
  deferCoverageRecovery(reached_target, CoverageRecoveryOutcome::REACHED);
  ROS_INFO_STREAM("[coverage recovery] observation target reached: id="
                  << reached_target.stable_id << " position=("
                  << reached_target.approach_position.transpose() << ")");
  return true;
}

double FastExplorationManager::failedGoalPenalty(
    const TopoNode::Ptr &viewpoint) const {
  if (!viewpoint || viewpoint->is_coverage_target_ || !ep_ ||
      deferred_goals_.empty()) {
    return 0.0;
  }
  const ros::Time now = ros::Time::now();
  for (const auto &goal : deferred_goals_) {
    if (goal.until.isZero() || now >= goal.until) {
      continue;
    }
    const bool same_cluster = goal.cluster_id >= 0 &&
        viewpoint->frontier_cluster_id_ == goal.cluster_id;
    const bool same_position =
        (viewpoint->center_ - goal.position).norm() <=
        std::max(0.2, ep_->goal_lock_match_radius_);
    if (same_cluster || same_position) {
      return ep_->failed_goal_penalty_;
    }
  }
  return 0.0;
}

int FastExplorationManager::planGlobalPath(const Eigen::Vector3d &pos,
                                           const Eigen::Vector3d &vel) {
  if (swarm_coordinator_ && swarm_coordinator_->enabled()) {
    swarm_coordinator_->updateRobotState(pos, vel);
  }
  last_plan_empty_frontier_ = false;
  last_plan_no_reachable_ = false;
  last_plan_requires_reorientation_ = false;
  if (targetDirectedModeConfigured() && !targetDirectedModeActive()) {
    ROS_WARN_THROTTLE(
        1.0,
        "[target exploration] waiting for a mission target; publish a "
        "PoseStamped goal before starting target-directed exploration");
    return TARGET_UNREACHABLE;
  }
  const bool target_directed = targetDirectedModeActive();
  if (target_directed && ed_->mission_goal_needs_initialization_) {
    ed_->mission_start_ = pos.cast<float>();
    if (!ep_->target_goal_use_message_z_) {
      ed_->mission_goal_.z() = static_cast<float>(pos.z());
    }
    ed_->mission_goal_needs_initialization_ = false;
    ROS_INFO_STREAM("[target exploration] initialize target frame start=("
                    << ed_->mission_start_.transpose() << ") goal=("
                    << ed_->mission_goal_.transpose() << ")");
  }
  // The frontier and local Bubble-Topo modules share a finite internal index
  // domain.  A destination outside it can never become a valid direct target
  // or frontier bridge; continuing to rank frontiers would look like arbitrary
  // wandering.  Fail the task explicitly so the operator can enlarge the
  // automatic capacity (or correct an accidental click) without confusing it
  // with a map/topology failure.
  if (target_directed && planner_manager_ &&
      planner_manager_->lidar_map_interface_ &&
      !planner_manager_->lidar_map_interface_->IsInBox(
          ed_->mission_goal_)) {
    ROS_ERROR_STREAM("[target exploration] mission goal outside the active "
                     "exploration capacity: goal=("
                     << ed_->mission_goal_.transpose()
                     << "). Increase target_exploration/auto_workspace/"
                        "half_extent_xy before launch, or use a target inside "
                        "the configured legacy coverage boxes.");
    return TARGET_UNREACHABLE;
  }
  if (target_directed && missionGoalReached(pos)) {
    return TARGET_REACHED;
  }
  const double current_speed = vel.norm();
  if (ep_->original_frontend_compatibility_) {
    // The source highspeedExp frontend uses the instantaneous threshold.  Keep
    // that behavior for migration parity; the hysteresis remains available in
    // the enhanced mode.
    high_speed_mode_active_ =
        current_speed >=
        planner_manager_->gcopter_config_->highSpeedModeThreshold;
  } else if (high_speed_mode_active_) {
    if (current_speed <=
        planner_manager_->gcopter_config_->highSpeedModeExitThreshold) {
      high_speed_mode_active_ = false;
    }
  } else if (current_speed >=
             planner_manager_->gcopter_config_->highSpeedModeThreshold) {
    high_speed_mode_active_ = true;
  }
  const bool moving = current_speed > 0.5;
  bool bm_without_topo = false;
  auto estimiateVdirCost = [&](const TopoNode::Ptr &n1,
                               const Eigen::Vector3d &v1,
                               const TopoNode::Ptr &n2) -> double {
    Eigen::Vector3f dir = n2->center_ - n1->center_;
    dir.normalize();
    Eigen::Vector3f v_dir = v1.normalized().cast<float>();
    float yaw1 = atan2(dir.y(), dir.x());
    float yaw2 = atan2(v_dir.y(), v_dir.x());
    float diff = yaw1 - yaw2;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    return ep_->w_vdir_ *
           (fabs(diff) / planner_manager_->gcopter_config_->yaw_max_vel);
  };
  ros::Time start = ros::Time::now();
  vector<TopoNode::Ptr> viewpoints;
  float curr_yaw = (float)planner_manager_->local_data_.curr_yaw_;
  HighSpeedViewScoreContext view_ctx;
  // The composite selector requires safe, kinematically meaningful viewpoint
  // representatives even when the legacy edge-cost compatibility switch is
  // enabled.  Information gain is still carried separately into the global
  // next-goal objective below, so this does not double-count it.
  view_ctx.enabled =
      ep_->composite_candidate_cost_enable_ || !ep_->epic_simple_global_cost_;
  view_ctx.log = planner_manager_->gcopter_config_->velocityLogEnable;
  view_ctx.curr_pos = pos.cast<float>();
  view_ctx.curr_vel = vel.cast<float>();
  view_ctx.curr_yaw = curr_yaw;
  view_ctx.high_speed_threshold = ep_->original_frontend_compatibility_
                                      ? planner_manager_->gcopter_config_
                                            ->highSpeedModeThreshold
                                      : (high_speed_mode_active_
                                             ? 0.0
                                             : planner_manager_->gcopter_config_
                                                   ->highSpeedModeThreshold);
  view_ctx.gain_weight =
      planner_manager_->gcopter_config_->viewScoreGainWeight;
  view_ctx.progress_weight =
      planner_manager_->gcopter_config_->viewScoreProgressWeight;
  view_ctx.velocity_align_weight =
      planner_manager_->gcopter_config_->viewScoreVelocityAlignWeight;
  view_ctx.known_free_weight =
      planner_manager_->gcopter_config_->viewScoreKnownFreeWeight;
  view_ctx.clearance_weight =
      planner_manager_->gcopter_config_->viewScoreClearanceWeight;
  view_ctx.yaw_weight =
      planner_manager_->gcopter_config_->viewScoreYawWeight;
  view_ctx.turn_weight =
      planner_manager_->gcopter_config_->viewScoreTurnWeight;
  view_ctx.backup_penalty =
      planner_manager_->gcopter_config_->viewScoreBackupPenalty;
  view_ctx.known_free_max_len =
      planner_manager_->gcopter_config_->viewScoreKnownFreeMaxLen;
  view_ctx.backup_required_len =
      planner_manager_->gcopter_config_->knownFreeShortLength;
  view_ctx.min_clearance =
      planner_manager_->gcopter_config_->commitKnownFreeSafeDistance;
  view_ctx.query_step =
      planner_manager_->gcopter_config_->safetyMapQueryStep;
  view_ctx.hard_gate_enable =
      planner_manager_->gcopter_config_->viewScoreHardGateEnable;
  view_ctx.hard_gate_min_known_free_ratio =
      planner_manager_->gcopter_config_->viewScoreHardGateMinKnownFreeRatio;
  view_ctx.hard_gate_max_turn_angle =
      planner_manager_->gcopter_config_->viewScoreHardGateMaxTurnAngle;
  view_ctx.hard_gate_max_yaw_delta =
      planner_manager_->gcopter_config_->viewScoreHardGateMaxYawDelta;
  view_ctx.hard_gate_min_clearance =
      planner_manager_->gcopter_config_->viewScoreHardGateMinClearance;
  view_ctx.top_viewpoint_num =
      planner_manager_->gcopter_config_->viewScoreTopCandidateNum;
  view_ctx.corridor_cruise_enable =
      planner_manager_->gcopter_config_->corridorCruiseEnable &&
      (ep_->original_frontend_compatibility_ || moving);
  view_ctx.corridor_known_free_len =
      planner_manager_->gcopter_config_->corridorCruiseKnownFreeLength;
  view_ctx.corridor_min_alignment =
      planner_manager_->gcopter_config_->corridorCruiseMinAlignment;
  view_ctx.corridor_forward_weight =
      planner_manager_->gcopter_config_->corridorCruiseForwardWeight;
  view_ctx.corridor_lateral_penalty =
      planner_manager_->gcopter_config_->corridorCruiseLateralPenalty;
  view_ctx.forward_known_free =
      [pm = planner_manager_](const Eigen::Vector3d &start,
                              const Eigen::Vector3d &dir,
                              double max_len,
                              double safe_distance,
                              double step) {
        return pm->forwardKnownFreeLength(start, dir, max_len, safe_distance,
                                          step);
      };
  view_ctx.clearance = [pm = planner_manager_](const Eigen::Vector3d &p) {
    return pm->safetyDistanceToOcc(p);
  };
  frontier_manager_ptr_->setHighSpeedViewScoreContext(view_ctx);
  frontier_manager_ptr_->updateExplorationDebt(
      pos.cast<float>(),
      ed_->has_goal_lock_ ? ed_->locked_goal_cluster_id_ : -1,
      ed_->has_goal_lock_ ? ed_->locked_goal_ : Eigen::Vector3f::Zero(),
      ep_->goal_lock_match_radius_,
      ep_->frontier_pass_radius_, ep_->frontier_pass_exit_margin_,
      ep_->frontier_pass_cooldown_, ep_->frontier_pass_debt_increment_,
      ep_->frontier_pass_debt_max_);
  planner_manager_->printSafetyMapSummary();
  // Destination tasks inject geometrically useful frontier clusters instead of
  // coverage-path preferred IDs.  Coverage guidance stays a coverage-only
  // planner and is not sampled or visualized here.
  std::unordered_set<int> preferred_clusters;
  if (target_directed) {
    preferred_clusters = preferredTargetClusterIds(pos);
  } else if (coverage_guidance_) {
    preferred_clusters = coverage_guidance_->preferredClusterIds();
  }
  frontier_manager_ptr_->generateTSPViewpoints(
      planner_manager_->topo_graph_->odom_node_->center_, viewpoints,
      preferred_clusters);
  if (target_directed) {
    // The guide point is injected only when its immediate straight segment is
    // known-free in the current rolling map. Otherwise the guide remains a
    // scoring preference and ordinary frontier bridges continue expanding the
    // map safely toward it.
    appendTopologyGuideCandidate(pos, curr_yaw, viewpoints);
  }
  if (!target_directed && coverage_guidance_) {
    coverage_guidance_->publishVisualization();
  }

  // A failed-goal cooldown must remove the target from the executable set,
  // not merely add a cost.  With only one remaining frontier, a soft 2000
  // penalty still selects the same target forever (notably an upstairs robot
  // repeatedly trying an unsafe path through the floor to a downstairs
  // frontier).  An empty set below activates the persistent-coverage recovery
  // path and gives other rooms/floors a chance.
  //
  // Target mode must NOT immediately expire every cooldown and restore the
  // same bridges: that recreates a hover-fail death spiral.  Wait for the
  // preferred-cluster injector to surface non-deferred detours, and only
  // unlock the oldest cooled bridge after a sustained empty-pool wait.
  const std::size_t viewpoint_count_before_defer = viewpoints.size();
  const vector<TopoNode::Ptr> viewpoints_before_defer = viewpoints;
  viewpoints.erase(
      std::remove_if(viewpoints.begin(), viewpoints.end(),
                     [&](const TopoNode::Ptr &viewpoint) {
                       return failedGoalPenalty(viewpoint) > 0.0;
                     }),
      viewpoints.end());
  const bool topology_forced_coverage = force_coverage_fallback_once_;
  force_coverage_fallback_once_ = false;
  if (topology_forced_coverage) {
    viewpoints.clear();
  }
  if (!topology_forced_coverage && viewpoint_count_before_defer > 0 &&
      viewpoints.empty()) {
    if (target_directed) {
      const ros::Time now = ros::Time::now();
      if (target_empty_pool_since_.isZero()) {
        target_empty_pool_since_ = now;
      }
      const double empty_for = (now - target_empty_pool_since_).toSec();
      const double escape_wait =
          ep_ ? ep_->target_empty_pool_escape_wait_ : 2.5;
      const int unlock_budget =
          ep_ ? ep_->target_empty_pool_unlock_count_ : 1;
      const bool unlock_ready =
          empty_for >= escape_wait &&
          (last_target_pool_unlock_time_.isZero() ||
           (now - last_target_pool_unlock_time_).toSec() >= escape_wait);
      if (unlock_ready) {
        const int unlocked = unlockOldestDeferredGoals(unlock_budget);
        last_target_pool_unlock_time_ = now;
        if (unlocked > 0) {
          // Re-admit only the unlocked identities; leave the rest cooling so
          // the planner can try a single fresh option instead of the whole
          // failed shortlist.
          for (const auto &viewpoint : viewpoints_before_defer) {
            if (!viewpoint) {
              continue;
            }
            if (failedGoalPenalty(viewpoint) <= 0.0) {
              viewpoints.emplace_back(viewpoint);
            }
          }
          if (!viewpoints.empty()) {
            // Do NOT clear target_empty_pool_since_ here.  Unlock-only refills
            // used to reset empty_for every 2.5 s and keep recycling the same
            // cooled bridges.  Clear only after a successful tour commit.
          }
          ROS_WARN_STREAM_THROTTLE(
              1.0, "[target exploration] empty deferred shortlist for "
                       << empty_for << "s; unlock oldest " << unlocked
                       << " cooled bridge(s) and keep remaining cooldowns");
        } else {
          ROS_WARN_STREAM_THROTTLE(
              1.0, "[target exploration] all "
                       << viewpoint_count_before_defer
                       << " frontier viewpoints are in failed-goal cooldown; "
                          "wait for map/new clusters (empty_for="
                       << empty_for << "s)");
        }
      } else {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[target exploration] all "
                     << viewpoint_count_before_defer
                     << " frontier viewpoints are in failed-goal cooldown; "
                        "wait for cooldown/map (empty_for="
                     << empty_for << "/" << escape_wait << "s)");
      }
    } else {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[plan recovery] all " << viewpoint_count_before_defer
                                      << " frontier viewpoints are in "
                                         "failed-goal cooldown; try coverage "
                                         "recovery");
    }
  } else if (target_directed) {
    target_empty_pool_since_ = ros::Time(0);
  }

  const ros::Time coverage_now = ros::Time::now();
  pruneDeferredCoverageGoals(coverage_now);
  if (has_active_coverage_goal_ &&
      !active_coverage_goal_start_.isZero() &&
      (coverage_now - active_coverage_goal_start_).toSec() >=
          coverage_recovery_timeout_) {
    deferCoverageRecovery(active_coverage_target_,
                          CoverageRecoveryOutcome::TIMEOUT);
  }

  const int active_clusters = frontier_manager_ptr_->activeClusterCount();
  const int reachable_clusters =
      frontier_manager_ptr_->reachableClusterCount();
  // Coverage is a strict fallback phase.  Base the handoff on the executable
  // set after failed-goal cooldown filtering, not on raw cached cluster counts:
  // a reachable-but-deferred cluster cannot produce a trajectory.  Conversely,
  // never mix a coverage target with an executable frontend viewpoint.
  const bool no_executable_frontier = viewpoints.empty();
  if (!no_executable_frontier) {
    coverage_executable_empty_count_ = 0;
    coverage_executable_empty_since_ = ros::Time(0);
  } else {
    if (coverage_executable_empty_since_.isZero()) {
      coverage_executable_empty_since_ = coverage_now;
      coverage_executable_empty_count_ = 1;
    } else {
      ++coverage_executable_empty_count_;
    }
  }
  const double executable_empty_duration =
      coverage_executable_empty_since_.isZero()
          ? 0.0
          : (coverage_now - coverage_executable_empty_since_).toSec();
  const bool executable_empty_stable =
      no_executable_frontier &&
      coverage_executable_empty_count_ >=
          coverage_executable_empty_min_count_ &&
      executable_empty_duration >=
          coverage_executable_empty_min_duration_;
  const bool moving_handoff_ready =
      coverage_moving_handoff_enable_ &&
      planner_manager_->hasCommittedTrajectory();
  const bool promote_coverage_candidates =
      !target_directed && coverage_guidance_ &&
      coverage_executable_candidate_enable_ &&
      executable_empty_stable &&
      (moving_handoff_ready ||
       current_speed <= coverage_executable_candidate_max_speed_) &&
      no_executable_frontier;
  const bool coverage_handoff_pending =
      !target_directed && coverage_guidance_ &&
      coverage_executable_candidate_enable_ &&
      no_executable_frontier && !has_active_coverage_goal_ &&
      (!executable_empty_stable ||
       (!moving_handoff_ready &&
        current_speed > coverage_executable_candidate_max_speed_));
  if (coverage_handoff_pending) {
    ROS_INFO_STREAM_THROTTLE(
        0.5, "[coverage handoff] wait for stable executable-frontier-empty "
                 "count="
                 << coverage_executable_empty_count_ << "/"
                 << coverage_executable_empty_min_count_ << " duration="
                 << executable_empty_duration << "/"
                 << coverage_executable_empty_min_duration_
                 << "s raw_active=" << active_clusters
                 << " raw_reachable=" << reachable_clusters
                 << " speed=" << current_speed << "/"
                 << coverage_executable_candidate_max_speed_
                 << " moving_handoff=" << moving_handoff_ready
                 << " blocker="
                 << (!executable_empty_stable ? "empty_debounce"
                                              : "vehicle_speed"));
  }
  bool priority_floor_active = false;
  bool ascending_to_priority_floor = false;
  int first_priority_floor_rank = std::numeric_limits<int>::max();
  auto isPriorityFloorTarget = [&](const CoverageTarget &target) {
    return coverage_floor_priority_enable_ &&
           target.position.z() >= coverage_floor_priority_min_z_;
  };
  if (promote_coverage_candidates) {
    auto coverage_targets =
        coverage_guidance_->unknownApproachTargets(pos, 160, 0.8);
    // Canonicalize the executable approach before consulting recovery state.
    // The raw approach_position is only a component hint; the selected entry
    // from approach_candidates is the action that is actually inserted into
    // the topology graph. Checking cooldown/exhaustion before this step let
    // two regenerated ids mapped to one disconnected approach bypass each
    // other's terminal record indefinitely.
    std::vector<CoverageTarget> canonical_coverage_targets;
    canonical_coverage_targets.reserve(coverage_targets.size());
    for (CoverageTarget target : coverage_targets) {
      if (coverageRecoveryExhausted(target)) {
        continue;
      }
      if (!selectSafeCoverageApproach(target, false)) {
        continue;
      }
      if (coverageRecoveryExhausted(target)) {
        rememberCoverageRecoveryAlias(target);
        continue;
      }
      canonical_coverage_targets.emplace_back(std::move(target));
    }
    coverage_targets.swap(canonical_coverage_targets);
    const bool active_goal_is_priority =
        has_active_coverage_goal_ &&
        isPriorityFloorTarget(active_coverage_target_);
    // Once ordinary frontiers are exhausted in a multi-floor scene, finish
    // the executable upper-floor pool before returning to lower-floor
    // perimeter cleanup. Preserve an already active lower-floor goal, then
    // switch floors at the next handoff.
    priority_floor_active =
        coverage_floor_priority_enable_ &&
        (!has_active_coverage_goal_ || active_goal_is_priority) &&
        std::any_of(
            coverage_targets.begin(), coverage_targets.end(),
            [&](const CoverageTarget &target) {
              return isPriorityFloorTarget(target) &&
                     !coverageRecoveryDeferred(target, coverage_now);
            });
    if (priority_floor_active) {
      for (const CoverageTarget &target : coverage_targets) {
        if (isPriorityFloorTarget(target) &&
            !coverageRecoveryDeferred(target, coverage_now)) {
          first_priority_floor_rank =
              std::min(first_priority_floor_rank, target.route_rank);
        }
      }
      ascending_to_priority_floor =
          pos.z() < coverage_floor_priority_min_z_ - 0.4 &&
          first_priority_floor_rank != std::numeric_limits<int>::max();
    }
    // When the robot is still below the priority floor, retain the short CP
    // prefix immediately preceding its first upper-floor observation. Those
    // lower-z nodes describe the staircase/doorway transition in the free-zone
    // graph. A pure z filter discarded them and asked MINCO to connect
    // directly to scattered upper-floor endpoints.
    auto isFloorPhaseTarget = [&](const CoverageTarget &target) {
      if (!priority_floor_active) {
        return true;
      }
      if (!ascending_to_priority_floor) {
        return isPriorityFloorTarget(target);
      }
      const int transition_rank_begin =
          std::max(0, first_priority_floor_rank -
                          coverage_floor_transition_rank_window_);
      return target.route_rank >= transition_rank_begin &&
             target.route_rank <= first_priority_floor_rank;
    };
    // The persistent CP route remains a long-horizon guide, but execution is
    // receding-horizon: expose several nearby/high-gain observations to the
    // real topology cost instead of blindly taking the first two CP nodes.
    auto localExecutionScore = [&](const CoverageTarget &target) {
      const double distance =
          target.has_approach
              ? (target.approach_position - pos).norm()
              : std::numeric_limits<double>::infinity();
      const double bounded_rank =
          std::min(40.0, static_cast<double>(std::max(0, target.route_rank)));
      const double bounded_gain =
          std::min(2.0, 0.35 * std::log1p(std::max(0, target.voxel_count)));
      return distance + coverage_route_rank_weight_ * bounded_rank -
             bounded_gain;
    };
    std::stable_sort(
        coverage_targets.begin(), coverage_targets.end(),
        [&](const CoverageTarget &first, const CoverageTarget &second) {
          return localExecutionScore(first) < localExecutionScore(second);
        });
    if (has_active_coverage_goal_) {
      std::stable_sort(
          coverage_targets.begin(), coverage_targets.end(),
          [&](const CoverageTarget &first, const CoverageTarget &second) {
            const bool first_active =
                first.stable_id != 0 &&
                first.stable_id == active_coverage_target_.stable_id;
            const bool second_active =
                second.stable_id != 0 &&
                second.stable_id == active_coverage_target_.stable_id;
            return first_active && !second_active;
          });
    }
    auto appendCoverageViewpoint = [&](const CoverageTarget &target) {
      if (!target.has_approach || !target.approach_position.allFinite()) {
        return false;
      }
      TopoNode::Ptr viewpoint = std::make_shared<TopoNode>();
      viewpoint->is_viewpoint_ = true;
      viewpoint->is_coverage_target_ = true;
      viewpoint->frontier_cluster_id_ = -1;
      viewpoint->coverage_target_id_ = target.stable_id;
      viewpoint->center_ = target.approach_position.cast<float>();
      viewpoint->coverage_unknown_ = target.position.cast<float>();
      viewpoint->coverage_voxel_count_ = target.voxel_count;
      viewpoint->coverage_route_rank_ = target.route_rank;
      viewpoint->frontier_information_gain_ =
          static_cast<double>(std::max(0, target.voxel_count));
      const Eigen::Vector3d observe_direction =
          target.position - target.approach_position;
      viewpoint->yaw_ =
          std::hypot(observe_direction.x(), observe_direction.y()) > 1.0e-3
              ? std::atan2(observe_direction.y(), observe_direction.x())
              : curr_yaw;
      viewpoints.emplace_back(viewpoint);
      return true;
    };

    int promoted = 0;
    std::vector<CoverageTarget> promoted_identities;
    promoted_identities.reserve(coverage_executable_candidate_max_count_);
    auto alreadyPromoted = [&](const CoverageTarget &target) {
      return std::any_of(
          promoted_identities.begin(), promoted_identities.end(),
          [&](const CoverageTarget &accepted) {
            return sameCoverageExecutionTarget(
                accepted, target, coverage_recovery_match_radius_);
          });
    };
    for (CoverageTarget target : coverage_targets) {
      if (promoted >= coverage_executable_candidate_max_count_) {
        break;
      }
      if (!isFloorPhaseTarget(target)) {
        continue;
      }
      if (coverageRecoveryExhausted(target)) {
        rememberCoverageRecoveryAlias(target);
        continue;
      }
      const bool is_active =
          has_active_coverage_goal_ && target.stable_id != 0 &&
          target.stable_id == active_coverage_target_.stable_id;
      if (!is_active && coverageRecoveryDeferred(target, coverage_now)) {
        rememberCoverageRecoveryAlias(target);
        continue;
      }
      if (alreadyPromoted(target)) {
        continue;
      }
      if (appendCoverageViewpoint(target)) {
        ++promoted;
        promoted_identities.emplace_back(target);
      }
    }

    // Preserve the normal 45 s room/floor rotation while another target is
    // executable. Once both the frontend and normal coverage pool are empty,
    // however, waiting for the coverage plateau before shortening cooldown is
    // dead time: the current zero-terminal trajectory can end long before a
    // retry is exposed. The retry remains bounded by the per-target attempt
    // counters and terminal_retry_interval, so it cannot spin forever.
    int terminal_retry_promoted = 0;
    int terminal_eligible_promoted = 0;
    int cooling_pending = 0;
    double next_terminal_retry =
        std::numeric_limits<double>::infinity();
    if (promoted == 0) {
      auto terminal_targets =
          coverage_guidance_->unknownApproachTargets(pos, 160, 0.0);
      std::stable_sort(
          terminal_targets.begin(), terminal_targets.end(),
          [&](const CoverageTarget &first, const CoverageTarget &second) {
            return localExecutionScore(first) < localExecutionScore(second);
          });
      for (CoverageTarget target : terminal_targets) {
        if (promoted >= coverage_executable_candidate_max_count_) {
          break;
        }
        // The terminal set includes low-gain and cooling targets omitted from
        // the preferred pool, so it must repeat the same canonicalization.
        // Terminal failures are recorded here because there is no remaining
        // normal executable action to make progress on them later.
        if (coverageRecoveryExhausted(target)) {
          continue;
        }
        if (!selectSafeCoverageApproach(target, true)) {
          continue;
        }
        if (coverageRecoveryExhausted(target)) {
          rememberCoverageRecoveryAlias(target);
          continue;
        }
        if (!isFloorPhaseTarget(target)) {
          continue;
        }
        if (alreadyPromoted(target)) {
          continue;
        }
        const bool is_active =
            has_active_coverage_goal_ && target.stable_id != 0 &&
            target.stable_id == active_coverage_target_.stable_id;
        if (is_active ||
            !coverageRecoveryDeferred(target, coverage_now)) {
          if (appendCoverageViewpoint(target)) {
            ++promoted;
            ++terminal_eligible_promoted;
            promoted_identities.emplace_back(target);
          }
          continue;
        }
        rememberCoverageRecoveryAlias(target);
        double cooling_remaining = 0.0;
        if (!coverageRecoveryCooling(target, coverage_now,
                                     &cooling_remaining)) {
          continue;
        }
        ++cooling_pending;
        double retry_after = cooling_remaining;
        if (!coverage_terminal_retry_enable_ ||
            !coverageTerminalRetryReady(target, coverage_now, &retry_after)) {
          next_terminal_retry =
              std::min(next_terminal_retry, retry_after);
          continue;
        }
        if (appendCoverageViewpoint(target)) {
          ++promoted;
          ++terminal_retry_promoted;
          promoted_identities.emplace_back(target);
        }
      }
    }
    if (terminal_retry_promoted > 0) {
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[coverage terminal retry] promoted="
                   << terminal_retry_promoted
                   << " after normal eligible set drained; cooling="
                   << cooling_pending
                   << " retry_interval="
                   << coverage_terminal_retry_interval_ << "s");
    } else if (terminal_eligible_promoted > 0) {
      ROS_INFO_STREAM_THROTTLE(
          0.5, "[coverage terminal drain] promoted low-gain actionable="
                   << terminal_eligible_promoted
                   << " after preferred execution pool drained");
    } else if (promoted == 0 && cooling_pending > 0) {
      ROS_INFO_STREAM_THROTTLE(
          0.5, "[coverage terminal retry] wait for bounded retry: cooling="
                   << cooling_pending << " retry_after="
                   << (std::isfinite(next_terminal_retry)
                           ? std::max(0.0, next_terminal_retry)
                           : coverage_terminal_retry_interval_)
                   << "s");
    }
    if (promoted > 0) {
      ROS_INFO_STREAM_THROTTLE(
          0.5, "[coverage candidate] promoted=" << promoted
                                                << " frontend_active="
                                                << active_clusters
                                                << " executable_frontiers="
                                                << viewpoints.size() - promoted
                                                << " speed=" << current_speed);
    }
  }

  // Do not ask the local planner to cross unknown space to the remote goal.
  // Once perception has made the endpoint known free, however, insert it as a
  // regular temporary topology node. This is the final handoff from
  // exploration to an exact point target and retains all normal graph and
  // MINCO feasibility checks.
  if (target_directed && missionGoalDirectCandidateReady()) {
    Eigen::Vector3f endpoint_center = ed_->mission_goal_;
    auto endpoint_is_safe = [&](const Eigen::Vector3f &candidate) {
      const Eigen::Vector3d candidate_d = candidate.cast<double>();
      const double required_clearance = std::max(
          planner_manager_->topo_graph_->bubble_min_radius_,
          planner_manager_->gcopter_config_->commitKnownFreeSafeDistance);
      return planner_manager_->querySafetyState(candidate_d) ==
                 MapVoxelState::KNOWN_FREE &&
             planner_manager_->safetyDistanceToOcc(candidate_d) >=
                 required_clearance;
    };
    if (!planner_manager_->topo_graph_->hasRegionForPoint(endpoint_center)) {
      // The top face of a configured box is a valid user target but maps to
      // the first non-existent region under floor indexing. Use a known-free
      // approach point inside the target tolerance when possible; reaching it
      // satisfies missionGoalReached() without inserting outside the graph.
      const Eigen::Vector3d goal = ed_->mission_goal_.cast<double>();
      const Eigen::Vector3d from_goal = pos - goal;
      const double distance = from_goal.norm();
      const double approach_distance =
          std::max(0.05, 0.5 * ep_->target_reached_radius_);
      if (distance > 1.0e-3) {
        const Eigen::Vector3f approach =
            (goal + from_goal / distance * approach_distance).cast<float>();
        if (planner_manager_->topo_graph_->hasRegionForPoint(approach) &&
            endpoint_is_safe(approach)) {
          endpoint_center = approach;
          ROS_INFO_STREAM_THROTTLE(
              1.0, "[target exploration] use safe in-region approach ("
                       << endpoint_center.transpose() << ") for boundary "
                       "goal=(" << ed_->mission_goal_.transpose() << ")");
        } else {
          ROS_WARN_STREAM_THROTTLE(
              1.0, "[target exploration] exact target is outside initialized "
                       "topology regions and no known-free in-region approach "
                       "exists; continue through frontier bridges");
          endpoint_center = Eigen::Vector3f::Constant(
              std::numeric_limits<float>::quiet_NaN());
        }
      } else {
        endpoint_center = Eigen::Vector3f::Constant(
            std::numeric_limits<float>::quiet_NaN());
      }
    }
    if (endpoint_center.allFinite()) {
      TopoNode::Ptr endpoint = std::make_shared<TopoNode>();
      endpoint->is_viewpoint_ = true;
      endpoint->is_mission_goal_target_ = true;
      endpoint->frontier_cluster_id_ = -1;
      endpoint->center_ = endpoint_center;
      const Eigen::Vector3d approach =
          endpoint_center.cast<double>() - pos;
      endpoint->yaw_ = std::hypot(approach.x(), approach.y()) > 1.0e-3
                           ? std::atan2(approach.y(), approach.x())
                           : curr_yaw;
      viewpoints.emplace_back(endpoint);
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[target exploration] endpoint is known free; add final goal "
                   "handoff at ("
                   << endpoint->center_.transpose() << ")");
    }
  }

  if (viewpoints.empty()) {
    if (coverage_handoff_pending) {
      // This is neither an executable frontier nor a confirmed terminal
      // NO_FRONTIER state. Keep the committed trajectory/controlled stop while
      // the handoff debounce settles; otherwise the FSM starts an expensive
      // global finish audit on every transient empty frontend cycle.
      last_plan_empty_frontier_ = false;
      last_plan_no_reachable_ = false;
      return FAIL;
    }
    if (target_directed) {
      // An empty target-directed pool is normally transient: all viewpoints
      // may be in a failed-goal cooldown, the local rolling map may be
      // catching up, or the endpoint may become known-free on the next cloud.
      // Do not convert that condition into a terminal BLOCKED task and throw
      // away the opportunity to retry with the retained global topology.
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[target exploration] no executable frontier remains before "
                   "mission goal=("
                   << ed_->mission_goal_.transpose()
                   << "); wait for map/cooldown update and retry with "
                      "retained topology");
      last_plan_empty_frontier_ = false;
      last_plan_no_reachable_ = false;
      return FAIL;
    }
    last_plan_empty_frontier_ = active_clusters == 0 && reachable_clusters == 0;
    last_plan_no_reachable_ = active_clusters > 0 && reachable_clusters == 0;
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    if (last_plan_no_reachable_) {
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[frontier gate] viewpoint generation empty but frontier "
               "clusters remain: active="
                   << active_clusters << " reachable=" << reachable_clusters);
      // These clusters have survived a forced global refresh but still have no
      // safe/reachable viewpoint.  Treat this as an executable-frontier-empty
      // observation and let the FSM's count/time debounce decide FINISH.
      // Returning FAIL here retries forever because an inactive raw cluster
      // cannot become a trajectory target without a map change.
      return NO_FRONTIER;
    }
    if (!last_plan_empty_frontier_) {
      return FAIL;
    }
    return NO_FRONTIER;
  }

  ros::Time t1 = ros::Time::now();
  planner_manager_->topo_graph_->insertNodes(viewpoints, false);
  updateGoalNode();
  ros::Time t2 = ros::Time::now();
  cout << "insert viewpoint to graph time: " << (t2 - t1).toSec() * 1000
       << " ms" << endl;
  vector<double> distance_odom2vp(viewpoints.size(), 2e3);
  vector<EdgeSafetyCost> edge_odom2vp(viewpoints.size());
  ros::Time t_start_cvp_1 = ros::Time::now();
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 0; i < viewpoints.size(); ++i) {
    edge_odom2vp[i] = getPathEdgeCost(
        planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
        viewpoints[i], viewpoints[i]->yaw_);
    distance_odom2vp[i] = edge_odom2vp[i].total_cost;
  }
  ros::Time t_end_cvp_1 = ros::Time::now();
  if (bm_without_topo) {
    omp_set_num_threads(4);
    // clang-format off
    #pragma omp parallel for
    // clang-format on
    for (int i = 0; i < viewpoints.size(); ++i) {
      distance_odom2vp[i] = getPathCostWithoutTopo(
          planner_manager_->topo_graph_->odom_node_, vel, curr_yaw,
          viewpoints[i], viewpoints[i]->yaw_);
    }
    ros::Time t_end_cvp_2 = ros::Time::now();
    double cost_mat_with_topo = (t_end_cvp_1 - t_start_cvp_1).toSec() * 1000;
    double cost_mat_without_topo = (t_end_cvp_2 - t_end_cvp_1).toSec() * 1000;
    cout << "cost mat topo: " << cost_mat_with_topo << "ms" << endl;
    cout << "cost mat point cloud: " << cost_mat_without_topo << "ms" << endl;
  }

  vector<TopoNode::Ptr> viewpoint_reachable;
  vector<double> viewpoint_reachable_distance;
  vector<EdgeSafetyCost> viewpoint_reachable_edges;
  vector<int> reversal_indices;
  const double reversal_angle =
      planner_manager_->gcopter_config_->reorientationHeadingAngle;
  for (int i = 0; i < static_cast<int>(distance_odom2vp.size()); ++i) {
    if (distance_odom2vp[i] > 2e3)
      continue;
    if (moving &&
        edge_odom2vp[i].initial_heading_delta > reversal_angle) {
      reversal_indices.emplace_back(i);
      continue;
    }
    viewpoint_reachable_distance.emplace_back(distance_odom2vp[i]);
    viewpoint_reachable_edges.emplace_back(edge_odom2vp[i]);
    viewpoint_reachable.emplace_back(viewpoints[i]);
  }
  const bool odom_topology_connected =
      planner_manager_->topo_graph_ &&
      planner_manager_->topo_graph_->odom_node_ &&
      !planner_manager_->topo_graph_->odom_node_->neighbors_.empty();
  if (viewpoint_reachable.empty()) {
    last_plan_no_reachable_ = true;
    if (!odom_topology_connected) {
      // A missing odom-to-skeleton edge is transient. Do not enter the
      // coverage-only recursion below: it would classify every promoted
      // approach as permanently disconnected before the root reconnects.
      last_plan_empty_frontier_ = false;
      last_plan_requires_reorientation_ = false;
      planner_manager_->topo_graph_->removeNodes(viewpoints);
      planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED,
                                                   "global");
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[topology gate] odom root is disconnected; preserve frontier "
               "and coverage recovery state until topology reconnects. "
               "candidates="
                   << viewpoints.size());
      return FAIL;
    }
    if (moving && !reversal_indices.empty()) {
      last_plan_requires_reorientation_ = true;
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[candidate gate] all reachable frontiers require reversal; "
               "keep current command and brake before reorientation. speed="
                   << vel.norm()
                   << " rejected=" << reversal_indices.size());
    } else {
      if (target_directed) {
        planner_manager_->topo_graph_->removeNodes(viewpoints);
        planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED,
                                                     "global");
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[target exploration] every frontier bridge to mission "
                     "goal is currently disconnected; wait for topology/map "
                     "update before retrying");
        last_plan_empty_frontier_ = false;
        return FAIL;
      }
      const bool evaluated_frontend_only =
          !viewpoints.empty() &&
          std::none_of(viewpoints.begin(), viewpoints.end(),
                       [](const TopoNode::Ptr &viewpoint) {
                         return viewpoint && viewpoint->is_coverage_target_;
                       });
      if (!target_directed && evaluated_frontend_only && coverage_guidance_ &&
          coverage_executable_candidate_enable_) {
        // generateTSPViewpoints can produce a geometrically valid viewpoint
        // whose topology edge is nevertheless NO_PATH. That candidate is not
        // executable and must not block coverage fallback. Re-enter this
        // single planning call once with a forced coverage-only pool; the
        // one-shot flag and the coverage-target check above bound recursion to
        // one level.
        planner_manager_->topo_graph_->removeNodes(viewpoints);
        coverage_executable_empty_count_ =
            coverage_executable_empty_min_count_;
        coverage_executable_empty_since_ =
            coverage_now -
            ros::Duration(coverage_executable_empty_min_duration_);
        force_coverage_fallback_once_ = true;
        last_plan_empty_frontier_ = false;
        last_plan_no_reachable_ = false;
        ROS_WARN_STREAM_THROTTLE(
            0.5, "[coverage handoff] frontend viewpoints failed topology "
                 "reachability; retry with a strict coverage-only pool count="
                     << viewpoints.size());
        return planGlobalPath(pos, vel);
      }
      for (const TopoNode::Ptr &viewpoint : viewpoints) {
        if (!viewpoint || !viewpoint->is_coverage_target_) {
          continue;
        }
        CoverageTarget disconnected;
        disconnected.stable_id = viewpoint->coverage_target_id_;
        disconnected.position =
            viewpoint->coverage_unknown_.cast<double>();
        disconnected.voxel_count = viewpoint->coverage_voxel_count_;
        disconnected.route_rank = viewpoint->coverage_route_rank_;
        disconnected.approach_position =
            viewpoint->center_.cast<double>();
        disconnected.has_approach = true;
        deferCoverageRecovery(disconnected,
                              CoverageRecoveryOutcome::DISCONNECTED);
      }
    }
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    return FAIL;
  }

  // A known-free *endpoint* is not by itself a point-to-point navigation
  // certificate.  In a target exploration task, a topology path may still
  // cross unknown space between the vehicle and that endpoint.  Treating such
  // an endpoint as an unconditional mission handoff bypasses the very
  // frontier bridges that are meant to reveal that route; it also makes the
  // local planner repeatedly receive a long, partially-observed path.
  //
  // Therefore the endpoint becomes a direct handoff only after the actual
  // local executable route is known-free and safety-feasible.  Otherwise
  // remove the temporary endpoint from this candidate pool and keep planning
  // ordinary frontier bridges.  The endpoint is re-inserted on the next
  // planning pass, so this is a state decision rather than a permanent
  // rejection.
  if (target_directed) {
    const auto mission_it = std::find_if(
        viewpoint_reachable.begin(), viewpoint_reachable.end(),
        [](const TopoNode::Ptr &viewpoint) {
          return viewpoint && viewpoint->is_mission_goal_target_;
        });
    if (mission_it != viewpoint_reachable.end()) {
      const int mission_idx = static_cast<int>(
          std::distance(viewpoint_reachable.begin(), mission_it));
      const TopoNode::Ptr &mission = viewpoint_reachable[mission_idx];
      const EdgeSafetyCost &topology_edge =
          viewpoint_reachable_edges[mission_idx];
      const double required_clearance = std::max(
          planner_manager_->topo_graph_->bubble_min_radius_,
          planner_manager_->gcopter_config_->commitKnownFreeSafeDistance);
      // The global topology and the local safety map update asynchronously.
      // A path that merely touches the shared clearance boundary can change
      // classification by one safety-map sample before the FSM consumes it.
      // Reserve one query step for the direct handoff; narrower routes remain
      // valid frontier bridges and are not rejected by this extra condition.
      const double direct_handoff_clearance =
          required_clearance + std::max(
                                   0.10,
                                   planner_manager_->gcopter_config_
                                       ->safetyMapQueryStep);
      // The candidate topology edge is useful for global ordering, but it is
      // not the path ultimately handed to the local FSM.  updateGoalNode()
      // creates a virtual endpoint and FastSearcher::search() then chooses
      // the executable local/topological route.  A previous implementation
      // certified the former only; in forest it reported 29 m known-free
      // while the actual FSM path had only 11 m known-free and failed the
      // backup gate.  Preflight that exact route before taking ownership of
      // the mission goal.
      ed_->global_tour_.clear();
      ed_->global_tour_.emplace_back(pos.cast<float>());
      ed_->global_tour_.emplace_back(mission->center_);
      updateGoalNode();
      vector<Eigen::Vector3f> local_handoff_path;
      const int local_handoff_result = planner_manager_->fast_searcher_->search(
          planner_manager_->topo_graph_->odom_node_, vel.cast<float>(),
          ed_->next_goal_node_, 0.2, local_handoff_path);
      EdgeSafetyCost local_handoff_edge;
      const bool local_route_found =
          local_handoff_result == BubbleAstar::REACH_END &&
          local_handoff_path.size() >= 2;
      if (local_route_found) {
        local_handoff_edge = planner_manager_->estimateHighSpeedEdgeCost(
            local_handoff_path, vel, curr_yaw, mission->yaw_);
      }
      const bool route_length_valid =
          local_route_found && std::isfinite(local_handoff_edge.path_length) &&
          local_handoff_edge.path_length > 1.0e-3;
      const bool route_fully_known =
          route_length_valid &&
          std::isfinite(local_handoff_edge.known_free_length) &&
          local_handoff_edge.known_free_length + 0.10 >=
              local_handoff_edge.path_length;
      const bool route_safe =
          std::isfinite(local_handoff_edge.min_clearance) &&
          local_handoff_edge.min_clearance + 1.0e-3 >=
              direct_handoff_clearance &&
          local_handoff_edge.backup_feasible;
      if (route_fully_known && route_safe) {
        ed_->has_goal_lock_ = true;
        ed_->locked_goal_is_mission_ = true;
        ed_->locked_goal_is_coverage_ = false;
        ed_->locked_goal_cluster_id_ = -1;
        ed_->locked_goal_coverage_id_ = 0;
        ed_->locked_goal_ = mission->center_;
        ed_->locked_goal_yaw_ = mission->yaw_;
        ed_->locked_goal_cost_ = viewpoint_reachable_distance[mission_idx];
        ed_->locked_goal_time_ = ros::Time::now();
        resetNormalGoalProgress();
        ed_->global_tour_.clear();
        ed_->global_tour_.emplace_back(pos.cast<float>());
        ed_->global_tour_.emplace_back(mission->center_);
        planner_manager_->local_data_.end_yaw_ = mission->yaw_;
        planner_manager_->topo_graph_->removeNodes(viewpoints);
        planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_,
                                                     VizColor::RED, "global");
        updateGoalNode();
        ROS_INFO_STREAM_THROTTLE(
            0.5, "[target exploration] select fully-known mission handoff "
                     "after local preflight: topology_cost="
                     << viewpoint_reachable_distance[mission_idx]
                     << " local_len=" << local_handoff_edge.path_length
                     << " local_known_free="
                     << local_handoff_edge.known_free_length
                     << " local_min_clearance="
                     << local_handoff_edge.min_clearance
                     << " goal=(" << mission->center_.transpose() << ")");
        return SUCCEED;
      }

      // Do not leave a virtual mission endpoint connected while this pass
      // chooses a frontier bridge.  The selected bridge will install its own
      // goal node below, and clearing here prevents a stale direct route from
      // being consumed if the FSM is triggered by another callback first.
      ed_->global_tour_.clear();
      updateGoalNode();
      ROS_INFO_STREAM_THROTTLE(
          0.5, "[target exploration] defer known-free endpoint until its "
                   "local executable route is fully observed; use frontier "
                   "bridges: topo_len=" << topology_edge.path_length
                   << " topo_known_free=" << topology_edge.known_free_length
                   << " local_result=" << local_handoff_result
                   << " local_len=" << local_handoff_edge.path_length
                   << " local_known_free="
                   << local_handoff_edge.known_free_length
                   << " local_min_clearance="
                   << local_handoff_edge.min_clearance << " local_backup="
                   << local_handoff_edge.backup_feasible
                   << " direct_handoff_clearance="
                   << direct_handoff_clearance);
      viewpoint_reachable.erase(viewpoint_reachable.begin() + mission_idx);
      viewpoint_reachable_distance.erase(
          viewpoint_reachable_distance.begin() + mission_idx);
      viewpoint_reachable_edges.erase(
          viewpoint_reachable_edges.begin() + mission_idx);
      if (viewpoint_reachable.empty()) {
        planner_manager_->topo_graph_->removeNodes(viewpoints);
        planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED,
                                                     "global");
        last_plan_empty_frontier_ = false;
        last_plan_no_reachable_ = false;
        return FAIL;
      }
    }
  }

  // In the frontier-bridge phase, prefer viewpoints that make material
  // progress toward the destination.  Large regressions are never kept as an
  // "escape hatch": flying backward from x≈49 to x≈-16 was worse than waiting
  // for the forward map to open.
  TargetDirectedExplorationConfig target_config;
  target_config.heuristic_weight = ep_->target_heuristic_weight_;
  target_config.lateral_weight = ep_->target_lateral_weight_;
  target_config.vertical_weight = ep_->target_vertical_weight_;
  target_config.nominal_speed = std::max(0.5, ep_->v_max_ / 2.0);
  if (target_directed) {
    const double escape_min_progress =
        ep_ ? ep_->target_escape_min_progress_ : -1.0;
    const double near_goal_remaining =
        ep_ ? ep_->target_near_goal_remaining_ : 15.0;
    const double mission_remaining_now = missionGoalDistance(pos);
    const bool near_goal = std::isfinite(mission_remaining_now) &&
                           mission_remaining_now <= near_goal_remaining;
    const double detour_min_progress =
        near_goal ? 0.0 : escape_min_progress;
    std::vector<bool> progressing(viewpoint_reachable.size(), false);
    std::vector<TargetDirectedExplorationScore> bridge_scores(
        viewpoint_reachable.size());
    bool has_progressing_bridge = false;
    for (std::size_t i = 0; i < viewpoint_reachable.size(); ++i) {
      const TopoNode::Ptr &viewpoint = viewpoint_reachable[i];
      if (!viewpoint || viewpoint->is_mission_goal_target_) {
        continue;
      }
      bridge_scores[i] = scoreTargetDirectedViewpoint(
          pos, ed_->mission_goal_.cast<double>(),
          viewpoint->center_.cast<double>(), target_config);
      progressing[i] = hasSufficientTargetProgress(
          bridge_scores[i], ep_->target_frontier_min_progress_);
      has_progressing_bridge = has_progressing_bridge || progressing[i];
    }
    vector<int> progressing_idx;
    vector<int> detour_idx;
    vector<int> mission_idx;
    progressing_idx.reserve(viewpoint_reachable.size());
    detour_idx.reserve(viewpoint_reachable.size());
    mission_idx.reserve(viewpoint_reachable.size());
    vector<double> detour_cost(viewpoint_reachable.size(),
                               std::numeric_limits<double>::infinity());
    vector<double> detour_remaining(viewpoint_reachable.size(),
                                    std::numeric_limits<double>::infinity());
    for (std::size_t i = 0; i < viewpoint_reachable.size(); ++i) {
      const TopoNode::Ptr &viewpoint = viewpoint_reachable[i];
      if (!viewpoint) {
        continue;
      }
      if (viewpoint->is_mission_goal_target_) {
        mission_idx.emplace_back(static_cast<int>(i));
        continue;
      }
      if (isSpatiallyBlacklisted(viewpoint->center_)) {
        continue;
      }
      if (progressing[i]) {
        progressing_idx.emplace_back(static_cast<int>(i));
        continue;
      }
      if (!bridge_scores[i].valid ||
          bridge_scores[i].progress_distance + 1.0e-6 < detour_min_progress) {
        continue;
      }
      detour_cost[i] = bridge_scores[i].cost;
      detour_remaining[i] = bridge_scores[i].remaining_distance;
      detour_idx.emplace_back(static_cast<int>(i));
    }
    std::stable_sort(detour_idx.begin(), detour_idx.end(),
                     [&](const int lhs, const int rhs) {
                       if (detour_cost[lhs] != detour_cost[rhs]) {
                         return detour_cost[lhs] < detour_cost[rhs];
                       }
                       return detour_remaining[lhs] < detour_remaining[rhs];
                     });
    const int detour_budget =
        ep_ ? ep_->target_progress_detour_budget_ : 3;
    if (static_cast<int>(detour_idx.size()) > detour_budget) {
      detour_idx.resize(static_cast<std::size_t>(detour_budget));
    }

    if (!has_progressing_bridge && progressing_idx.empty() &&
        detour_idx.empty() && mission_idx.empty()) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[target exploration] no forward/non-regressing frontier "
                   "bridge (min_detour_progress="
                   << detour_min_progress << " near_goal=" << near_goal
                   << " remaining=" << mission_remaining_now
                   << "); wait for map update");
    } else {
      vector<TopoNode::Ptr> filtered_viewpoints;
      vector<double> filtered_distances;
      vector<EdgeSafetyCost> filtered_edges;
      filtered_viewpoints.reserve(mission_idx.size() + progressing_idx.size() +
                                  detour_idx.size());
      filtered_distances.reserve(mission_idx.size() + progressing_idx.size() +
                                 detour_idx.size());
      filtered_edges.reserve(mission_idx.size() + progressing_idx.size() +
                             detour_idx.size());
      auto append_idx = [&](const int idx) {
        filtered_viewpoints.emplace_back(viewpoint_reachable[idx]);
        filtered_distances.emplace_back(viewpoint_reachable_distance[idx]);
        filtered_edges.emplace_back(viewpoint_reachable_edges[idx]);
      };
      for (const int idx : mission_idx) {
        append_idx(idx);
      }
      for (const int idx : progressing_idx) {
        append_idx(idx);
      }
      for (const int idx : detour_idx) {
        append_idx(idx);
      }
      if (has_progressing_bridge) {
        ROS_INFO_STREAM_THROTTLE(
            0.5, "[target exploration] retain " << progressing_idx.size()
                                                   << " progressing + "
                                                   << detour_idx.size()
                                                   << " mild-detour bridges "
                                                      "from "
                                                   << viewpoint_reachable.size()
                                                   << " (min progress="
                                                   << ep_->target_frontier_min_progress_
                                                   << ")");
      } else {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[target exploration] no forward frontier bridge with "
                     "minimum target progress="
                     << ep_->target_frontier_min_progress_
                     << "; retain " << detour_idx.size()
                     << " mild detours (progress>=" << detour_min_progress
                     << ")");
      }
      viewpoint_reachable.swap(filtered_viewpoints);
      viewpoint_reachable_distance.swap(filtered_distances);
      viewpoint_reachable_edges.swap(filtered_edges);
    }
  }
  if (target_directed) {
    return commitTargetDirectedTour(
        pos, vel, curr_yaw, viewpoints, viewpoint_reachable,
        viewpoint_reachable_distance, viewpoint_reachable_edges);
  }

  std::vector<double> swarm_candidate_penalties(viewpoint_reachable.size(),
                                                 0.0);
  if (swarm_coordinator_ && swarm_coordinator_->enabled()) {
    std::vector<SwarmCandidate> swarm_candidates;
    swarm_candidates.reserve(viewpoint_reachable.size());
    for (std::size_t i = 0; i < viewpoint_reachable.size(); ++i) {
      SwarmCandidate candidate;
      candidate.position = viewpoint_reachable[i]->center_.cast<double>();
      candidate.information_gain =
          viewpoint_reachable[i]->is_coverage_target_
              ? static_cast<double>(
                    std::max(0, viewpoint_reachable[i]->coverage_voxel_count_))
              : viewpoint_reachable[i]->frontier_information_gain_;
      candidate.travel_cost = viewpoint_reachable_distance[i];
      candidate.coverage = viewpoint_reachable[i]->is_coverage_target_;
      swarm_candidates.emplace_back(candidate);
    }
    swarm_candidate_penalties =
        swarm_coordinator_->candidatePenalties(swarm_candidates);
  }

  auto activateSelectedGoal = [&](const TopoNode::Ptr &selected) {
    if (!selected) {
      return;
    }
    if (selected->is_mission_goal_target_) {
      resetNormalGoalProgress();
      return;
    }
    if (!selected->is_coverage_target_) {
      if (has_active_coverage_goal_) {
        deferCoverageRecovery(active_coverage_target_,
                              CoverageRecoveryOutcome::FRONTIER_RESUMED);
      }
      return;
    }
    resetNormalGoalProgress();
    CoverageTarget target;
    target.type = CoverageTargetType::REACHABLE_UNKNOWN;
    target.stable_id = selected->coverage_target_id_;
    target.position = selected->coverage_unknown_.cast<double>();
    target.voxel_count = selected->coverage_voxel_count_;
    target.route_rank = selected->coverage_route_rank_;
    target.approach_position = selected->center_.cast<double>();
    target.has_approach = true;
    target.approach_candidates.emplace_back(target.approach_position);
    const bool same_active =
        has_active_coverage_goal_ && target.stable_id != 0 &&
        target.stable_id == active_coverage_target_.stable_id;
    active_coverage_target_ = target;
    has_active_coverage_goal_ = true;
    if (!same_active) {
      active_coverage_goal_start_ = coverage_now;
      active_coverage_observed_voxels_ = latestCoverageObservedVoxels();
    }
  };
  auto deferStalledNormalGoal =
      [&](const TopoNode::Ptr &selected, const double route_cost) {
        if (!updateNormalGoalProgress(selected, route_cost, pos)) {
          return false;
        }
        // selectStableGoalIndex() has already populated the lock with this
        // selected ordinary frontier, so the existing cooldown mechanism can
        // defer exactly that goal.  Clear every continuity owner before the
        // replacement planning pass.
        ed_->has_goal_lock_ = true;
        ed_->locked_goal_is_coverage_ = false;
        ed_->locked_goal_is_mission_ = false;
        ed_->locked_goal_cluster_id_ = selected->frontier_cluster_id_;
        ed_->locked_goal_coverage_id_ = 0;
        ed_->locked_goal_ = selected->center_;
        ed_->locked_goal_yaw_ = selected->yaw_;
        deferCurrentGoalAfterPlanningFailure();
        ed_->has_goal_lock_ = false;
        ed_->locked_goal_cluster_id_ = -1;
        ed_->locked_goal_is_coverage_ = false;
        ed_->locked_goal_is_mission_ = false;
        ed_->locked_goal_coverage_id_ = 0;
        return true;
      };

  struct CandidateCostBreakdown {
    double travel{0.0};
    double turn_brake{0.0};
    double future_return{0.0};
    double gain_norm{0.0};
    double wait_norm{0.0};
    double debt_norm{0.0};
    double coverage{0.0};
    double target_guidance{0.0};
    double target_remaining{0.0};
    double target_progress{0.0};
    double target_lateral{0.0};
    double failed_goal{0.0};
    double swarm{0.0};
    double total{0.0};
  };
  vector<CandidateCostBreakdown> candidate_terms(viewpoint_reachable.size());
  auto fillStaticTerms = [&](const int i) {
    CandidateCostBreakdown &terms = candidate_terms[i];
    const auto &edge = viewpoint_reachable_edges[i];
    const auto &viewpoint = viewpoint_reachable[i];
    terms.travel = edge.time_cost;
    terms.turn_brake = edge.turn_penalty + edge.yaw_penalty +
                       edge.known_free_penalty + edge.backup_penalty;
    if (viewpoint->is_mission_goal_target_) {
      terms.gain_norm = 0.0;
      terms.wait_norm = 0.0;
      terms.debt_norm = 0.0;
      terms.coverage = 0.0;
    } else if (viewpoint->is_coverage_target_) {
      const double gain =
          static_cast<double>(std::max(0, viewpoint->coverage_voxel_count_));
      terms.gain_norm = gain / (60.0 + gain);
      terms.wait_norm = 0.0;
      terms.debt_norm = 0.0;
      const double rank_bonus =
          viewpoint->coverage_route_rank_ >= 0
              ? 1.0 / (1.0 + viewpoint->coverage_route_rank_)
              : 0.0;
      const double bounded_rank = std::min(
          40.0, static_cast<double>(
                    std::max(0, viewpoint->coverage_route_rank_)));
      terms.coverage = -coverage_executable_candidate_bonus_ - rank_bonus +
                       coverage_route_rank_weight_ * bounded_rank;
      if (priority_floor_active &&
          (!ascending_to_priority_floor ||
           (viewpoint->coverage_route_rank_ >=
                std::max(0, first_priority_floor_rank -
                                coverage_floor_transition_rank_window_) &&
            viewpoint->coverage_route_rank_ <= first_priority_floor_rank))) {
        terms.coverage -= coverage_executable_candidate_bonus_;
      }
    } else {
      terms.gain_norm =
          viewpoint->frontier_information_gain_ /
          (std::max(1.0e-3, ep_->candidate_gain_saturation_) +
           viewpoint->frontier_information_gain_);
      terms.wait_norm = std::clamp(
          viewpoint->frontier_wait_age_ /
              std::max(1.0, ep_->candidate_wait_saturation_),
          0.0, 1.0);
      terms.debt_norm = std::clamp(
          viewpoint->frontier_pass_debt_ /
              std::max(1.0, ep_->candidate_debt_saturation_),
          0.0, 1.0);
      terms.coverage = !target_directed && coverage_guidance_
                           ? coverage_guidance_->clusterPenalty(
                                 viewpoint->frontier_cluster_id_,
                                 viewpoint->center_.cast<double>())
                           : 0.0;
    }
    terms.failed_goal = failedGoalPenalty(viewpoint);
    terms.swarm = swarm_candidate_penalties[i];
  };
  auto finishCompositeCost = [&](const int i) {
    CandidateCostBreakdown &terms = candidate_terms[i];
    if (!ep_->composite_candidate_cost_enable_) {
      terms.total = viewpoint_reachable_distance[i] + terms.coverage +
                    terms.target_guidance +
                    terms.failed_goal + terms.swarm;
      return;
    }
    terms.total =
        ep_->candidate_travel_weight_ * terms.travel +
        ep_->candidate_turn_brake_weight_ * terms.turn_brake +
        ep_->candidate_future_return_weight_ * terms.future_return -
        ep_->candidate_information_gain_weight_ * terms.gain_norm -
        ep_->candidate_wait_weight_ * terms.wait_norm -
        ep_->candidate_debt_weight_ * terms.debt_norm + terms.coverage +
        terms.target_guidance +
        terms.failed_goal + terms.swarm;
  };

  for (int i = 0; i < static_cast<int>(candidate_terms.size()); ++i) {
    fillStaticTerms(i);
  }

  if (viewpoint_reachable.size() == 1) {
    finishCompositeCost(0);
    vector<double> composite_costs{candidate_terms[0].total};
    const int goal_idx =
        selectStableGoalIndex(viewpoint_reachable, composite_costs,
                              0, vel);
    if (deferStalledNormalGoal(
            viewpoint_reachable[goal_idx],
            candidate_terms[goal_idx].travel)) {
      planner_manager_->topo_graph_->removeNodes(viewpoints);
      return planGlobalPath(pos, vel);
    }
    activateSelectedGoal(viewpoint_reachable[goal_idx]);
    if (swarm_coordinator_ && swarm_coordinator_->enabled()) {
      SwarmCandidate claim;
      claim.position = viewpoint_reachable[goal_idx]->center_.cast<double>();
      claim.information_gain =
          viewpoint_reachable[goal_idx]->is_coverage_target_
              ? static_cast<double>(std::max(
                    0, viewpoint_reachable[goal_idx]->coverage_voxel_count_))
              : viewpoint_reachable[goal_idx]->frontier_information_gain_;
      claim.travel_cost = candidate_terms[goal_idx].travel;
      claim.coverage = viewpoint_reachable[goal_idx]->is_coverage_target_;
      swarm_coordinator_->claimTask(claim);
    }
    ROS_INFO_STREAM_THROTTLE(
        0.5, "[candidate cost] chosen_cluster="
                 << viewpoint_reachable[goal_idx]->frontier_cluster_id_
                 << " coverage_id="
                 << viewpoint_reachable[goal_idx]->coverage_target_id_
                 << " total=" << candidate_terms[goal_idx].total
                 << " travel=" << candidate_terms[goal_idx].travel
                 << " turn_brake=" << candidate_terms[goal_idx].turn_brake
                 << " future_return=0 gain="
                 << candidate_terms[goal_idx].gain_norm
                 << " wait=" << candidate_terms[goal_idx].wait_norm
                 << " debt=" << candidate_terms[goal_idx].debt_norm
                 << " coverage=" << candidate_terms[goal_idx].coverage
                 << " target=" << candidate_terms[goal_idx].target_guidance
                 << " target_remaining="
                 << candidate_terms[goal_idx].target_remaining
                 << " target_progress="
                 << candidate_terms[goal_idx].target_progress
                 << " failed_goal="
                 << candidate_terms[goal_idx].failed_goal
                 << " swarm=" << candidate_terms[goal_idx].swarm);
    ed_->global_tour_.clear();
    ed_->global_tour_.emplace_back(pos.cast<float>());
    ed_->global_tour_.emplace_back(viewpoint_reachable[goal_idx]->center_);
    planner_manager_->local_data_.end_yaw_ = viewpoint_reachable[goal_idx]->yaw_;
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_,
                                                 VizColor::RED, "global");
    updateGoalNode();
    return SUCCEED;
  }

  int dim = viewpoint_reachable.size() + 1;
  Eigen::MatrixXd mat;
  mat.resize(dim, dim);
  mat.setZero();
  for (int i = 1; i < dim; ++i) {
    mat(0, i) = viewpoint_reachable_distance[i - 1];
  }

  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (int i = 1; i < dim; i++) {
    for (int j = i + 1; j < dim; j++) {
      EdgeSafetyCost forward = getPathEdgeCost(
          viewpoint_reachable[i - 1], Eigen::Vector3d::Zero(),
          viewpoint_reachable[i - 1]->yaw_, viewpoint_reachable[j - 1],
          viewpoint_reachable[j - 1]->yaw_);
      EdgeSafetyCost reverse = getPathEdgeCost(
          viewpoint_reachable[j - 1], Eigen::Vector3d::Zero(),
          viewpoint_reachable[j - 1]->yaw_, viewpoint_reachable[i - 1],
          viewpoint_reachable[i - 1]->yaw_);
      mat(i, j) = forward.total_cost;
      mat(j, i) = reverse.total_cost;
    }
  }

  // Estimate the cost of skipping a currently cheap/high-value frontier and
  // returning to it after visiting candidate i.  This is evaluated only for
  // next-goal selection; adding a fixed node reward to an all-node TSP would be
  // a constant and could not change visit order.
  const int return_horizon = std::clamp(
      ep_->candidate_return_horizon_, 1,
      std::max(1, static_cast<int>(viewpoint_reachable.size()) - 1));
  for (int i = 0; i < static_cast<int>(viewpoint_reachable.size()); ++i) {
    struct ReturnAlternative {
      double priority;
      double extra;
    };
    vector<ReturnAlternative> alternatives;
    alternatives.reserve(viewpoint_reachable.size() - 1);
    for (int k = 0; k < static_cast<int>(viewpoint_reachable.size()); ++k) {
      if (k == i) {
        continue;
      }
      const double extra = std::max(
          0.0, mat(i + 1, k + 1) - candidate_terms[k].travel);
      if (extra <= 1.0e-6) {
        continue;
      }
      const double priority = 1.0 + candidate_terms[k].gain_norm +
                              candidate_terms[k].wait_norm +
                              candidate_terms[k].debt_norm;
      alternatives.push_back({priority, extra});
    }
    std::stable_sort(
        alternatives.begin(), alternatives.end(),
        [](const ReturnAlternative &a, const ReturnAlternative &b) {
          return a.priority * a.extra > b.priority * b.extra;
        });
    double weighted_extra = 0.0;
    double weight_sum = 0.0;
    for (int k = 0;
         k < std::min(return_horizon, static_cast<int>(alternatives.size()));
         ++k) {
      weighted_extra += alternatives[k].priority * alternatives[k].extra;
      weight_sum += alternatives[k].priority;
    }
    candidate_terms[i].future_return =
        std::min(std::max(0.0, ep_->candidate_return_cost_cap_),
                 weight_sum > 1.0e-6 ? weighted_extra / weight_sum : 0.0);
    finishCompositeCost(i);
  }

  vector<double> composite_costs(candidate_terms.size(), 0.0);
  int candidate_goal_idx = 0;
  auto deterministicCandidateLess = [&](const int first, const int second) {
    constexpr double kCostTieEpsilon = 1.0e-6;
    const double delta =
        candidate_terms[first].total - candidate_terms[second].total;
    if (std::fabs(delta) > kCostTieEpsilon) {
      return delta < 0.0;
    }
    const Eigen::Vector3f &first_center =
        viewpoint_reachable[first]->center_;
    const Eigen::Vector3f &second_center =
        viewpoint_reachable[second]->center_;
    if (first_center.x() != second_center.x())
      return first_center.x() < second_center.x();
    if (first_center.y() != second_center.y())
      return first_center.y() < second_center.y();
    if (first_center.z() != second_center.z())
      return first_center.z() < second_center.z();
    if (viewpoint_reachable[first]->is_coverage_target_ !=
        viewpoint_reachable[second]->is_coverage_target_) {
      return !viewpoint_reachable[first]->is_coverage_target_;
    }
    return viewpoint_reachable[first]->frontier_cluster_id_ <
           viewpoint_reachable[second]->frontier_cluster_id_;
  };
  for (int i = 0; i < static_cast<int>(candidate_terms.size()); ++i) {
    composite_costs[i] = candidate_terms[i].total;
    if (deterministicCandidateLess(i, candidate_goal_idx)) {
      candidate_goal_idx = i;
    }
  }
  const int stable_goal_idx = selectStableGoalIndex(
      viewpoint_reachable, composite_costs, candidate_goal_idx, vel);
  if (deferStalledNormalGoal(
          viewpoint_reachable[stable_goal_idx],
          candidate_terms[stable_goal_idx].travel)) {
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    return planGlobalPath(pos, vel);
  }
  activateSelectedGoal(viewpoint_reachable[stable_goal_idx]);
  if (swarm_coordinator_ && swarm_coordinator_->enabled()) {
    SwarmCandidate claim;
    claim.position =
        viewpoint_reachable[stable_goal_idx]->center_.cast<double>();
    claim.information_gain =
        viewpoint_reachable[stable_goal_idx]->is_coverage_target_
            ? static_cast<double>(std::max(
                  0, viewpoint_reachable[stable_goal_idx]
                         ->coverage_voxel_count_))
            : viewpoint_reachable[stable_goal_idx]
                  ->frontier_information_gain_;
    claim.travel_cost = candidate_terms[stable_goal_idx].travel;
    claim.coverage =
        viewpoint_reachable[stable_goal_idx]->is_coverage_target_;
    swarm_coordinator_->claimTask(claim);
  }

  vector<int> debug_order(candidate_terms.size());
  for (int i = 0; i < static_cast<int>(debug_order.size()); ++i) {
    debug_order[i] = i;
  }
  std::stable_sort(debug_order.begin(), debug_order.end(),
                   deterministicCandidateLess);
  std::ostringstream cost_log;
  cost_log << "[candidate cost] chosen_cluster="
           << viewpoint_reachable[stable_goal_idx]->frontier_cluster_id_
           << " chosen_coverage="
           << viewpoint_reachable[stable_goal_idx]->coverage_target_id_
           << " raw_best_cluster="
           << viewpoint_reachable[candidate_goal_idx]->frontier_cluster_id_
           << " raw_best_coverage="
           << viewpoint_reachable[candidate_goal_idx]->coverage_target_id_;
  for (int rank = 0;
       rank < std::min(3, static_cast<int>(debug_order.size())); ++rank) {
    const int i = debug_order[rank];
    const auto &term = candidate_terms[i];
    cost_log << " | #" << rank << " c="
             << viewpoint_reachable[i]->frontier_cluster_id_
             << "/cp=" << viewpoint_reachable[i]->coverage_target_id_
             << " J=" << term.total << " T=" << term.travel
             << " TB=" << term.turn_brake << " R=" << term.future_return
             << " G=" << term.gain_norm << " W=" << term.wait_norm
             << " D=" << term.debt_norm << " C=" << term.coverage
             << " M=" << term.target_guidance
             << " Mr=" << term.target_remaining
             << " Mp=" << term.target_progress;
    if (std::fabs(term.swarm) > 1.0e-9) {
      cost_log << " S=" << term.swarm;
    }
    if (term.failed_goal > 0.0) {
      cost_log << " F=" << term.failed_goal;
    }
  }
  ROS_INFO_STREAM_THROTTLE(0.5, cost_log.str());

  // Convert the cycle into an open route and force the separately scored next
  // goal to be first.  The old far-end return-edge trick systematically
  // postponed side rooms and is intentionally removed.
  Eigen::MatrixXd route_mat = mat;
  constexpr double kForceFirstPenalty = 1e3;
  for (int i = 1; i < dim; ++i) {
    route_mat(i, 0) = 0.0;
    if (i != stable_goal_idx + 1) {
      route_mat(0, i) += kForceFirstPenalty;
    }
  }
  vector<int> indices;
  indices.reserve(dim);
  ros::Time start_tsp = ros::Time::now();
  cout << "calculate tsp cost matrix cost " << (start_tsp - t2).toSec() * 1000
       << "ms" << endl;
  solveTour(route_mat, indices);
  ros::Time end_tsp = ros::Time::now();
  cout << "tour solver cost: " << (end_tsp - start_tsp).toSec() * 1000 << "ms"
       << endl;
  if (indices.size() < 2 || indices[1] != stable_goal_idx + 1) {
    // Deterministic greedy fallback with the already selected first goal.  A
    // route-solver failure must not discard an otherwise valid next target.
    indices.clear();
    indices.emplace_back(0);
    indices.emplace_back(stable_goal_idx + 1);
    vector<bool> used(dim, false);
    used[0] = true;
    used[stable_goal_idx + 1] = true;
    int current = stable_goal_idx + 1;
    while (static_cast<int>(indices.size()) < dim) {
      int best = -1;
      for (int next = 1; next < dim; ++next) {
        if (!used[next] &&
            (best < 0 || mat(current, next) < mat(current, best))) {
          best = next;
        }
      }
      if (best < 0) {
        break;
      }
      used[best] = true;
      indices.emplace_back(best);
      current = best;
    }
  }
  ed_->global_tour_.clear();
  ed_->global_tour_.push_back(planner_manager_->topo_graph_->odom_node_->center_);
  ed_->global_tour_.emplace_back(viewpoint_reachable[stable_goal_idx]->center_);
  for (auto &i : indices) {
    if (i <= 0)
      continue;
    const int viewpoint_idx = i - 1;
    if (viewpoint_idx == stable_goal_idx)
      continue;
    if (viewpoint_idx >= 0 &&
        viewpoint_idx < static_cast<int>(viewpoint_reachable.size())) {
      ed_->global_tour_.emplace_back(viewpoint_reachable[viewpoint_idx]->center_);
    }
  }
  ros::Time end = ros::Time::now();
  planner_manager_->topo_graph_->removeNodes(viewpoints);
  planner_manager_->graph_visualizer_->vizTour(ed_->global_tour_, VizColor::RED,
                                               "global");

  planner_manager_->local_data_.end_yaw_ =
      viewpoint_reachable[stable_goal_idx]->yaw_;
  updateGoalNode();
  return SUCCEED;
}

void FastExplorationManager::solveTour(Eigen::MatrixXd &cost_mat,
                                       vector<int> &indices) {
  indices.clear();
  const int dimension = cost_mat.rows();
  if (dimension <= 0 || cost_mat.cols() != dimension)
    return;
  if (dimension == 1) {
    indices.emplace_back(0);
    return;
  }

  if (ep_->use_lkh_ && solveLKH(cost_mat, indices)) {
    return;
  }

  if (ep_->use_lkh_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[global tour] LKH failed; use deterministic ATSP fallback");
  }
  solveFallbackTour(cost_mat, indices);
}

bool FastExplorationManager::solveLKH(const Eigen::MatrixXd &cost_mat,
                                      vector<int> &indices) {
  indices.clear();
  const int dimension = cost_mat.rows();
  if (dimension < 3 || cost_mat.cols() != dimension || ep_->tsp_dir_.empty()) {
    return false;
  }

  const string problem_file = ep_->tsp_dir_ + "/single.tsp";
  const string parameter_file = ep_->tsp_dir_ + "/single.par";
  const string result_file = ep_->tsp_dir_ + "/single.txt";
  ofstream problem(problem_file, std::ios::out | std::ios::trunc);
  if (!problem.is_open()) {
    return false;
  }

  problem << "NAME : single\n"
          << "TYPE : ATSP\n"
          << "DIMENSION : " << dimension << "\n"
          << "EDGE_WEIGHT_TYPE : EXPLICIT\n"
          << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
          << "EDGE_WEIGHT_SECTION\n";
  constexpr double kScale = 100.0;
  constexpr int kMaxLkhCost = std::numeric_limits<int>::max() / 8;
  for (int row = 0; row < dimension; ++row) {
    for (int col = 0; col < dimension; ++col) {
      const double cost = cost_mat(row, col);
      if (!std::isfinite(cost)) {
        problem.close();
        return false;
      }
      const long long scaled = std::llround(cost * kScale);
      problem << std::clamp<long long>(scaled, 0, kMaxLkhCost) << " ";
    }
    problem << "\n";
  }
  problem << "EOF\n";
  problem.close();
  if (!problem) {
    return false;
  }

  // Do not accept a tour left by an earlier failed invocation.
  std::remove(result_file.c_str());
  if (solveTSPLKH(parameter_file.c_str()) != EXIT_SUCCESS) {
    return false;
  }

  ifstream result(result_file);
  if (!result.is_open()) {
    return false;
  }
  string line;
  bool in_tour_section = false;
  vector<int> raw_tour;
  raw_tour.reserve(dimension);
  while (std::getline(result, line)) {
    if (!in_tour_section) {
      if (line == "TOUR_SECTION") {
        in_tour_section = true;
      }
      continue;
    }
    std::istringstream line_stream(line);
    int id = 0;
    if (!(line_stream >> id)) {
      continue;
    }
    if (id == -1) {
      break;
    }
    raw_tour.emplace_back(id - 1);
  }
  if (static_cast<int>(raw_tour.size()) != dimension) {
    return false;
  }

  vector<bool> seen(dimension, false);
  for (const int node : raw_tour) {
    if (node < 0 || node >= dimension || seen[node]) {
      return false;
    }
    seen[node] = true;
  }
  const auto depot = std::find(raw_tour.begin(), raw_tour.end(), 0);
  if (depot == raw_tour.end()) {
    return false;
  }
  indices.insert(indices.end(), depot, raw_tour.end());
  indices.insert(indices.end(), raw_tour.begin(), depot);
  return static_cast<int>(indices.size()) == dimension && indices.front() == 0;
}

void FastExplorationManager::solveFallbackTour(
    const Eigen::MatrixXd &cost_mat, vector<int> &indices) const {
  indices.clear();
  const int dimension = cost_mat.rows();
  if (dimension <= 0 || cost_mat.cols() != dimension) {
    return;
  }
  if (dimension == 1) {
    indices.emplace_back(0);
    return;
  }

  constexpr int kExactMaxDimension = 16;
  const double inf = std::numeric_limits<double>::infinity();
  if (dimension <= kExactMaxDimension) {
    const int node_num = dimension - 1;
    const int state_num = 1 << node_num;
    vector<double> dp(static_cast<std::size_t>(state_num) * node_num, inf);
    vector<int> parent(static_cast<std::size_t>(state_num) * node_num, -1);
    const auto offset = [node_num](const int mask, const int node) {
      return static_cast<std::size_t>(mask) * node_num + node;
    };

    for (int node = 0; node < node_num; ++node) {
      dp[offset(1 << node, node)] = cost_mat(0, node + 1);
    }
    for (int mask = 1; mask < state_num; ++mask) {
      for (int node = 0; node < node_num; ++node) {
        if ((mask & (1 << node)) == 0)
          continue;
        const int previous_mask = mask ^ (1 << node);
        if (previous_mask == 0)
          continue;
        for (int previous = 0; previous < node_num; ++previous) {
          if ((previous_mask & (1 << previous)) == 0)
            continue;
          const double candidate =
              dp[offset(previous_mask, previous)] +
              cost_mat(previous + 1, node + 1);
          if (candidate < dp[offset(mask, node)]) {
            dp[offset(mask, node)] = candidate;
            parent[offset(mask, node)] = previous;
          }
        }
      }
    }

    const int full_mask = state_num - 1;
    int last_node = -1;
    double best_cost = inf;
    for (int node = 0; node < node_num; ++node) {
      const double cycle_cost =
          dp[offset(full_mask, node)] + cost_mat(node + 1, 0);
      if (cycle_cost < best_cost) {
        best_cost = cycle_cost;
        last_node = node;
      }
    }
    if (last_node >= 0 && std::isfinite(best_cost)) {
      vector<int> reversed;
      int mask = full_mask;
      while (last_node >= 0) {
        reversed.push_back(last_node + 1);
        const int previous = parent[offset(mask, last_node)];
        mask ^= 1 << last_node;
        last_node = previous;
      }
      indices.push_back(0);
      indices.insert(indices.end(), reversed.rbegin(), reversed.rend());
      return;
    }
  }

  // Directed cheapest insertion keeps the depot-closing edge in the objective.
  // For the current open-route matrix every frontier-to-depot edge is zero,
  // while depot departure still forces the separately selected first goal.
  vector<bool> visited(dimension, false);
  vector<int> route{0};
  visited[0] = true;
  while (static_cast<int>(route.size()) < dimension) {
    int best_node = -1;
    int best_insert_after = -1;
    double best_delta = inf;
    for (int node = 1; node < dimension; ++node) {
      if (visited[node]) {
        continue;
      }
      for (int pos = 0; pos < static_cast<int>(route.size()); ++pos) {
        const int from = route[pos];
        const int to = pos + 1 < static_cast<int>(route.size())
                           ? route[pos + 1]
                           : 0;
        const double delta = cost_mat(from, node) + cost_mat(node, to) -
                             cost_mat(from, to);
        if (std::isfinite(delta) && delta < best_delta) {
          best_delta = delta;
          best_node = node;
          best_insert_after = pos;
        }
      }
    }
    if (best_node < 0) {
      indices.clear();
      return;
    }
    route.insert(route.begin() + best_insert_after + 1, best_node);
    visited[best_node] = true;
  }

  const auto cycle_cost = [&cost_mat](const vector<int> &tour) {
    double cost = 0.0;
    for (int i = 0; i < static_cast<int>(tour.size()); ++i) {
      cost += cost_mat(tour[i],
                       i + 1 < static_cast<int>(tour.size()) ? tour[i + 1] : 0);
    }
    return cost;
  };
  double best_cost = cycle_cost(route);
  // A few deterministic relocate passes cheaply remove bad insertions while
  // retaining directed edge costs.
  for (int pass = 0; pass < 4; ++pass) {
    bool improved = false;
    for (int from = 1; from < dimension && !improved; ++from) {
      for (int insert_at = 1; insert_at < dimension && !improved;
           ++insert_at) {
        vector<int> candidate = route;
        const int node = candidate[from];
        candidate.erase(candidate.begin() + from);
        candidate.insert(candidate.begin() + insert_at, node);
        if (candidate == route) {
          continue;
        }
        const double candidate_cost = cycle_cost(candidate);
        if (candidate_cost + 1.0e-9 < best_cost) {
          route.swap(candidate);
          best_cost = candidate_cost;
          improved = true;
        }
      }
    }
    if (!improved) {
      break;
    }
  }
  indices.swap(route);
}

void FastExplorationManager::updateGoalNode() {
  if (!ed_->next_goal_node_)
    return;

  // next_goal_node_ is a reusable virtual graph node.  It is connected to the
  // persistent topology but is not owned by a topology region.  Disconnect it
  // explicitly on every update; otherwise an update with no newly reachable
  // neighbours leaves edges and paths pointing to the previous goal.
  const vector<TopoNode::Ptr> old_neighbors(
      ed_->next_goal_node_->neighbors_.begin(),
      ed_->next_goal_node_->neighbors_.end());
  for (const auto &neighbor : old_neighbors) {
    if (!neighbor) {
      continue;
    }
    neighbor->neighbors_.erase(ed_->next_goal_node_);
    neighbor->paths_.erase(ed_->next_goal_node_);
    neighbor->weight_.erase(ed_->next_goal_node_);
    neighbor->unreachable_nbrs_.erase(ed_->next_goal_node_);
  }
  ed_->next_goal_node_->neighbors_.clear();
  ed_->next_goal_node_->paths_.clear();
  ed_->next_goal_node_->weight_.clear();
  ed_->next_goal_node_->unreachable_nbrs_.clear();

  if (ed_->global_tour_.size() < 2) {
    ed_->next_goal_node_->is_viewpoint_ = false;
    return;
  }
  const Eigen::Vector3f goal = ed_->global_tour_[1];
  ed_->next_goal_node_->center_ = goal;
  ed_->next_goal_node_->is_viewpoint_ = true;

  struct PairPtrHash {
    std::size_t
    operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
      return std::hash<TopoNode::Ptr>()(p.first) ^
             std::hash<TopoNode::Ptr>()(p.second);
    }
  };

  Eigen::Vector3i idx;
  planner_manager_->topo_graph_->getIndex(goal, idx);
  vector<TopoNode::Ptr> pre_nbrs;
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i tmp_idx = idx;
        tmp_idx(0) = idx(0) + i;
        tmp_idx(1) = idx(1) + j;
        tmp_idx(2) = idx(2) + k;
        auto region = planner_manager_->topo_graph_->getRegionNode(tmp_idx);
        if (region) {
          for (auto &topo : region->topo_nodes_) {
            if (topo == ed_->next_goal_node_)
              continue;
            pre_nbrs.emplace_back(topo);
          }
        }
      }
  std::unordered_map<std::pair<TopoNode::Ptr, TopoNode::Ptr>,
                     vector<Eigen::Vector3f>, PairPtrHash>
      edge2insert;
  mutex edge2insert_mtx;
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &nbr : pre_nbrs) {
    vector<Eigen::Vector3f> path;
    int res = planner_manager_->topo_graph_->parallel_bubble_astar_->search(
        goal, nbr->center_, path, 1e-3);
    if (res == ParallelBubbleAstar::REACH_END &&
        planner_manager_->topo_graph_->parallel_bubble_astar_
            ->collisionCheck_shortenPath(path)) {
      edge2insert_mtx.lock();
      edge2insert.insert({std::make_pair(ed_->next_goal_node_, nbr), path});
      edge2insert_mtx.unlock();
    }
  }
  if (edge2insert.size() > 0) {
    for (auto &edge : edge2insert) {
      ed_->next_goal_node_->neighbors_.insert(edge.first.second);
      ed_->next_goal_node_->paths_.insert({edge.first.second, edge.second});
      double cost;
      planner_manager_->topo_graph_->parallel_bubble_astar_->calculatePathCost(
          edge.second, cost);
      ed_->next_goal_node_->weight_[edge.first.second] = cost;
      auto nbr = edge.first.second;
      nbr->neighbors_.insert(ed_->next_goal_node_);
      nbr->weight_[ed_->next_goal_node_] = cost;
      vector<Eigen::Vector3f> path = edge.second;
      std::reverse(path.begin(), path.end());
      nbr->paths_[ed_->next_goal_node_] = path;
    }
  }
}
} // namespace fast_planner
