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
#include <lkh_tsp_solver/lkh_interface.h>
#include <algorithm>
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
}

void FastExplorationManager::updateCoverageGuidance(
    const Eigen::Vector3d &pos) {
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
  if (!coverage_guidance_ || !planner_manager_) {
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
  for (const CoverageTarget &target : targets) {
    if (coverageRecoveryExhausted(target)) {
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
  // 提取四元数
  double roll, pitch;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.orientation, quat);

  // 将四元数转换为Euler角
  tf::Matrix3x3(quat).getRPY(roll, pitch, goal_yaw);
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
    if (ed_->locked_goal_is_coverage_ &&
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
  const bool chosen_is_coverage = viewpoints[chosen_idx]->is_coverage_target_;
  const bool same_identity =
      ed_->has_goal_lock_ &&
      chosen_is_coverage == ed_->locked_goal_is_coverage_ &&
      (chosen_is_coverage
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
  ed_->locked_goal_is_coverage_ = chosen_is_coverage;
  ed_->locked_goal_cluster_id_ =
      chosen_is_coverage ? -1 : viewpoints[chosen_idx]->frontier_cluster_id_;
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
    if (frontier_manager_ptr_ && !chosen_is_coverage) {
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
    const Eigen::Vector3d &robot_position) {
  if (!frontier_progress_watchdog_enable_ || !viewpoint ||
      viewpoint->is_coverage_target_ ||
      viewpoint->frontier_cluster_id_ < 0 || !std::isfinite(route_cost)) {
    if (viewpoint && viewpoint->is_coverage_target_) {
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
    normal_goal_progress_.last_progress_time = now;
    normal_goal_progress_.last_sample_time = now;
    normal_goal_progress_.samples = 1;
    return false;
  }

  normal_goal_progress_.cluster_id = viewpoint->frontier_cluster_id_;
  normal_goal_progress_.goal = viewpoint->center_;
  normal_goal_progress_.last_sample_time = now;
  ++normal_goal_progress_.samples;
  const bool route_progress =
      route_cost <= normal_goal_progress_.best_route_cost -
                        frontier_progress_min_cost_drop_;
  const bool spatial_progress =
      goal_distance <= normal_goal_progress_.best_goal_distance -
                           frontier_progress_min_distance_drop_;
  if (route_progress || spatial_progress) {
    normal_goal_progress_.best_route_cost =
        std::min(normal_goal_progress_.best_route_cost, route_cost);
    normal_goal_progress_.best_goal_distance =
        std::min(normal_goal_progress_.best_goal_distance, goal_distance);
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
      << " samples=" << normal_goal_progress_.samples);
  return true;
}

bool FastExplorationManager::coverageRecoveryDeferred(
    const CoverageTarget &target, const ros::Time &now) const {
  return std::any_of(
      deferred_coverage_goals_.begin(), deferred_coverage_goals_.end(),
      [&](const DeferredCoverageGoal &goal) {
        const bool same_id =
            target.stable_id != 0 && goal.stable_id == target.stable_id;
        return (goal.exhausted || goal.until > now) &&
               (same_id ||
                (goal.approach - target.approach_position).norm() <=
                    coverage_recovery_match_radius_);
      });
}

bool FastExplorationManager::coverageRecoveryExhausted(
    const CoverageTarget &target) const {
  return std::any_of(
      deferred_coverage_goals_.begin(), deferred_coverage_goals_.end(),
      [&](const DeferredCoverageGoal &goal) {
        const bool same_id =
            target.stable_id != 0 && goal.stable_id == target.stable_id;
        return goal.exhausted &&
               (same_id ||
                (goal.approach - target.approach_position).norm() <=
                    coverage_recovery_match_radius_);
      });
}

bool FastExplorationManager::coverageRecoveryCooling(
    const CoverageTarget &target, const ros::Time &now,
    double *remaining) const {
  for (const DeferredCoverageGoal &goal : deferred_coverage_goals_) {
    const bool same_id =
        target.stable_id != 0 && goal.stable_id == target.stable_id;
    const bool same_position =
        (goal.approach - target.approach_position).norm() <=
        coverage_recovery_match_radius_;
    if ((!same_id && !same_position) || goal.exhausted ||
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
    const bool same_id =
        target.stable_id != 0 && goal.stable_id == target.stable_id;
    const bool same_position =
        (goal.approach - target.approach_position).norm() <=
        coverage_recovery_match_radius_;
    if ((!same_id && !same_position) || goal.exhausted ||
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
    const bool same_id =
        target.stable_id != 0 && goal.stable_id == target.stable_id;
    if (same_id ||
        (goal.approach - target.approach_position).norm() <=
            coverage_recovery_match_radius_) {
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
    goal.stable_id = target.stable_id;
    goal.approach = target.approach_position;
    goal.unknown_position = target.position;
    goal.voxel_count = target.voxel_count;
    deferred_coverage_goals_.push_back(goal);
    matched = &deferred_coverage_goals_.back();
  }
  matched->stable_id = target.stable_id;
  matched->approach = target.approach_position;
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
    case CoverageRecoveryOutcome::UNSAFE:
    case CoverageRecoveryOutcome::DISCONNECTED:
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
  last_plan_empty_frontier_ = false;
  last_plan_no_reachable_ = false;
  last_plan_requires_reorientation_ = false;
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
  const std::unordered_set<int> coverage_preferred =
      coverage_guidance_ ? coverage_guidance_->preferredClusterIds()
                         : std::unordered_set<int>();
  frontier_manager_ptr_->generateTSPViewpoints(
      planner_manager_->topo_graph_->odom_node_->center_, viewpoints,
      coverage_preferred);
  if (coverage_guidance_) {
    coverage_guidance_->publishVisualization();
  }

  // A failed-goal cooldown must remove the target from the executable set,
  // not merely add a cost.  With only one remaining frontier, a soft 2000
  // penalty still selects the same target forever (notably an upstairs robot
  // repeatedly trying an unsafe path through the floor to a downstairs
  // frontier).  An empty set below activates the persistent-coverage recovery
  // path and gives other rooms/floors a chance.
  const std::size_t viewpoint_count_before_defer = viewpoints.size();
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
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[plan recovery] all " << viewpoint_count_before_defer
                                    << " frontier viewpoints are in failed-goal "
                                       "cooldown; try coverage recovery");
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

  auto selectSafeCoverageApproach =
      [&](CoverageTarget &target, bool record_terminal_failure) {
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
          const RaycastSafetyInfo observation_ray =
              planner_manager_->raycastSafety(
                  approach, target.position, false, 0.05, 0.15);
          if (observation_ray.blocked_by_occupied ||
              observation_ray.first_blocked_state ==
                  MapVoxelState::OUT_OF_MAP) {
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
      };

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
      coverage_guidance_ && coverage_executable_candidate_enable_ &&
      executable_empty_stable &&
      (moving_handoff_ready ||
       current_speed <= coverage_executable_candidate_max_speed_) &&
      no_executable_frontier;
  const bool coverage_handoff_pending =
      coverage_guidance_ && coverage_executable_candidate_enable_ &&
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
    auto appendCoverageViewpoint = [&](CoverageTarget target) {
      if (!selectSafeCoverageApproach(target, viewpoints.empty())) {
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
    for (CoverageTarget target : coverage_targets) {
      if (promoted >= coverage_executable_candidate_max_count_) {
        break;
      }
      if (!isFloorPhaseTarget(target)) {
        continue;
      }
      const bool is_active =
          has_active_coverage_goal_ && target.stable_id != 0 &&
          target.stable_id == active_coverage_target_.stable_id;
      if (!is_active && coverageRecoveryDeferred(target, coverage_now)) {
        continue;
      }
      if (appendCoverageViewpoint(target)) {
        ++promoted;
      }
    }

    // Preserve the normal 45 s room/floor rotation while coverage is still
    // changing. A short retry is a terminal audit mechanism only: enabling it
    // immediately after every temporary empty pool made the vehicle drain
    // perimeter/occluded goals instead of following the multi-floor route.
    int terminal_retry_promoted = 0;
    int terminal_eligible_promoted = 0;
    int cooling_pending = 0;
    double next_terminal_retry =
        std::numeric_limits<double>::infinity();
    const CoverageFinishStatus terminal_status =
        promoted == 0 ? coverageFinishStatus() : CoverageFinishStatus();
    if (promoted == 0 && terminal_status.plateau_reached) {
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
        if (coverageRecoveryExhausted(target)) {
          continue;
        }
        if (!isFloorPhaseTarget(target)) {
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
          }
          continue;
        }
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
    } else if (promoted == 0 && !terminal_status.plateau_reached) {
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[coverage terminal retry] wait for coverage plateau before "
                   "short cooldown retry: plateau="
                   << terminal_status.plateau_duration << "/"
                   << coverage_finish_plateau_duration_ << "s");
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
  if (viewpoint_reachable.empty()) {
    last_plan_no_reachable_ = true;
    if (moving && !reversal_indices.empty()) {
      last_plan_requires_reorientation_ = true;
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[candidate gate] all reachable frontiers require reversal; "
               "keep current command and brake before reorientation. speed="
                   << vel.norm()
                   << " rejected=" << reversal_indices.size());
    } else {
      const bool evaluated_frontend_only =
          !viewpoints.empty() &&
          std::none_of(viewpoints.begin(), viewpoints.end(),
                       [](const TopoNode::Ptr &viewpoint) {
                         return viewpoint && viewpoint->is_coverage_target_;
                       });
      if (evaluated_frontend_only && coverage_guidance_ &&
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

  auto activateSelectedGoal = [&](const TopoNode::Ptr &selected) {
    if (!selected) {
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
        ed_->locked_goal_cluster_id_ = selected->frontier_cluster_id_;
        ed_->locked_goal_coverage_id_ = 0;
        ed_->locked_goal_ = selected->center_;
        ed_->locked_goal_yaw_ = selected->yaw_;
        deferCurrentGoalAfterPlanningFailure();
        ed_->has_goal_lock_ = false;
        ed_->locked_goal_cluster_id_ = -1;
        ed_->locked_goal_is_coverage_ = false;
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
    double failed_goal{0.0};
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
    if (viewpoint->is_coverage_target_) {
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
      terms.coverage = coverage_guidance_
                           ? coverage_guidance_->clusterPenalty(
                                 viewpoint->frontier_cluster_id_,
                                 viewpoint->center_.cast<double>())
                           : 0.0;
    }
    terms.failed_goal = failedGoalPenalty(viewpoint);
  };
  auto finishCompositeCost = [&](const int i) {
    CandidateCostBreakdown &terms = candidate_terms[i];
    if (!ep_->composite_candidate_cost_enable_) {
      terms.total = viewpoint_reachable_distance[i] + terms.coverage +
                    terms.failed_goal;
      return;
    }
    terms.total =
        ep_->candidate_travel_weight_ * terms.travel +
        ep_->candidate_turn_brake_weight_ * terms.turn_brake +
        ep_->candidate_future_return_weight_ * terms.future_return -
        ep_->candidate_information_gain_weight_ * terms.gain_norm -
        ep_->candidate_wait_weight_ * terms.wait_norm -
        ep_->candidate_debt_weight_ * terms.debt_norm + terms.coverage +
        terms.failed_goal;
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
                 << " failed_goal="
                 << candidate_terms[goal_idx].failed_goal);
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
             << " D=" << term.debt_norm << " C=" << term.coverage;
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
