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
  nh.param("exploration/use_lkh", ep_->use_lkh_, true);

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
    if (ed_->locked_goal_cluster_id_ >= 0) {
      for (int i = 0; i < static_cast<int>(viewpoints.size()); ++i) {
        if (viewpoints[i]->frontier_cluster_id_ ==
            ed_->locked_goal_cluster_id_) {
          locked_idx = i;
          locked_match_distance =
              (viewpoints[i]->center_ - ed_->locked_goal_).norm();
          break;
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
  const bool new_lock =
      !ed_->has_goal_lock_ ||
      (viewpoints[chosen_idx]->frontier_cluster_id_ !=
           ed_->locked_goal_cluster_id_ &&
       (viewpoints[chosen_idx]->center_ - ed_->locked_goal_).norm() >
           ep_->goal_lock_match_radius_);
  ed_->has_goal_lock_ = true;
  ed_->locked_goal_cluster_id_ =
      viewpoints[chosen_idx]->frontier_cluster_id_;
  ed_->locked_goal_ = viewpoints[chosen_idx]->center_;
  ed_->locked_goal_yaw_ = viewpoints[chosen_idx]->yaw_;
  ed_->locked_goal_cost_ =
      chosen_idx < static_cast<int>(distance_odom2vp.size())
          ? distance_odom2vp[chosen_idx]
          : 0.0;
  if (new_lock) {
    ed_->locked_goal_time_ = now;
    if (frontier_manager_ptr_) {
      frontier_manager_ptr_->markClusterGoalSelected(
          viewpoints[chosen_idx]->frontier_cluster_id_,
          viewpoints[chosen_idx]->center_, ep_->goal_lock_match_radius_);
    }
  }
  return chosen_idx;
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
  frontier_manager_ptr_->generateTSPViewpoints(
      planner_manager_->topo_graph_->odom_node_->center_, viewpoints);

  if (viewpoints.empty()) {
    const int active_clusters = frontier_manager_ptr_->activeClusterCount();
    const int reachable_clusters = frontier_manager_ptr_->reachableClusterCount();
    last_plan_empty_frontier_ = active_clusters == 0 && reachable_clusters == 0;
    last_plan_no_reachable_ = !last_plan_empty_frontier_;
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    if (!last_plan_empty_frontier_) {
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[frontier gate] viewpoint generation empty but frontier "
               "clusters remain: active="
                   << active_clusters << " reachable=" << reachable_clusters);
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
    }
    planner_manager_->topo_graph_->removeNodes(viewpoints);
    planner_manager_->graph_visualizer_->vizTour({}, VizColor::RED, "global");
    return FAIL;
  }

  struct CandidateCostBreakdown {
    double travel{0.0};
    double turn_brake{0.0};
    double future_return{0.0};
    double gain_norm{0.0};
    double wait_norm{0.0};
    double debt_norm{0.0};
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
  };
  auto finishCompositeCost = [&](const int i) {
    CandidateCostBreakdown &terms = candidate_terms[i];
    if (!ep_->composite_candidate_cost_enable_) {
      terms.total = viewpoint_reachable_distance[i];
      return;
    }
    terms.total =
        ep_->candidate_travel_weight_ * terms.travel +
        ep_->candidate_turn_brake_weight_ * terms.turn_brake +
        ep_->candidate_future_return_weight_ * terms.future_return -
        ep_->candidate_information_gain_weight_ * terms.gain_norm -
        ep_->candidate_wait_weight_ * terms.wait_norm -
        ep_->candidate_debt_weight_ * terms.debt_norm;
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
    ROS_INFO_STREAM_THROTTLE(
        0.5, "[candidate cost] chosen_cluster="
                 << viewpoint_reachable[goal_idx]->frontier_cluster_id_
                 << " total=" << candidate_terms[goal_idx].total
                 << " travel=" << candidate_terms[goal_idx].travel
                 << " turn_brake=" << candidate_terms[goal_idx].turn_brake
                 << " future_return=0 gain="
                 << candidate_terms[goal_idx].gain_norm
                 << " wait=" << candidate_terms[goal_idx].wait_norm
                 << " debt=" << candidate_terms[goal_idx].debt_norm);
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
  for (int i = 0; i < static_cast<int>(candidate_terms.size()); ++i) {
    composite_costs[i] = candidate_terms[i].total;
    if (candidate_terms[i].total < candidate_terms[candidate_goal_idx].total) {
      candidate_goal_idx = i;
    }
  }
  const int stable_goal_idx = selectStableGoalIndex(
      viewpoint_reachable, composite_costs, candidate_goal_idx, vel);

  vector<int> debug_order(candidate_terms.size());
  for (int i = 0; i < static_cast<int>(debug_order.size()); ++i) {
    debug_order[i] = i;
  }
  std::stable_sort(debug_order.begin(), debug_order.end(), [&](int a, int b) {
    return candidate_terms[a].total < candidate_terms[b].total;
  });
  std::ostringstream cost_log;
  cost_log << "[candidate cost] chosen_cluster="
           << viewpoint_reachable[stable_goal_idx]->frontier_cluster_id_
           << " raw_best_cluster="
           << viewpoint_reachable[candidate_goal_idx]->frontier_cluster_id_;
  for (int rank = 0;
       rank < std::min(3, static_cast<int>(debug_order.size())); ++rank) {
    const int i = debug_order[rank];
    const auto &term = candidate_terms[i];
    cost_log << " | #" << rank << " c="
             << viewpoint_reachable[i]->frontier_cluster_id_
             << " J=" << term.total << " T=" << term.travel
             << " TB=" << term.turn_brake << " R=" << term.future_return
             << " G=" << term.gain_norm << " W=" << term.wait_norm
             << " D=" << term.debt_norm;
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
