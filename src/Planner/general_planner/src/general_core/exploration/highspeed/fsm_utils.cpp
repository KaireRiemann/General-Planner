#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/fast_exploration_fsm.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <tf/tf.h>

namespace {
float segmentAngle(const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
  if (a.norm() < 1.0e-4f || b.norm() < 1.0e-4f) {
    return 0.0f;
  }
  const float c = std::clamp(a.normalized().dot(b.normalized()), -1.0f, 1.0f);
  return std::acos(c);
}

void conditionHighSpeedPath(vector<Eigen::Vector3f> &path) {
  if (path.size() < 3) {
    return;
  }

  vector<Eigen::Vector3f> compact;
  compact.reserve(path.size());
  compact.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    if ((path[i] - compact.back()).norm() >= 0.20f ||
        i + 1 == path.size()) {
      compact.push_back(path[i]);
    }
  }

  if (compact.size() < 3) {
    path.swap(compact);
    return;
  }

  vector<Eigen::Vector3f> conditioned;
  conditioned.reserve(compact.size());
  conditioned.push_back(compact.front());
  for (std::size_t i = 1; i + 1 < compact.size(); ++i) {
    const Eigen::Vector3f a = compact[i] - conditioned.back();
    const Eigen::Vector3f b = compact[i + 1] - compact[i];
    const float angle = segmentAngle(a, b);
    const bool nearly_collinear = angle < 0.12f && a.norm() < 5.0f;
    const bool tight_backtrack = angle > 2.35f && a.norm() < 0.8f;
    if (!nearly_collinear && !tight_backtrack) {
      conditioned.push_back(compact[i]);
    }
  }
  conditioned.push_back(compact.back());
  path.swap(conditioned);
}

vector<Eigen::Vector3d> truncatePathHorizon(
    const vector<Eigen::Vector3f> &path, double max_length) {
  vector<Eigen::Vector3d> out;
  if (path.empty()) {
    return out;
  }
  out.reserve(path.size());
  out.emplace_back(path.front().cast<double>());
  const double limit = max_length > 0.1
                           ? max_length
                           : std::numeric_limits<double>::infinity();
  double accumulated = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d from = path[i - 1].cast<double>();
    const Eigen::Vector3d to = path[i].cast<double>();
    const double segment_length = (to - from).norm();
    if (segment_length < 1.0e-6) {
      continue;
    }
    if (accumulated + segment_length > limit) {
      const double ratio =
          std::clamp((limit - accumulated) / segment_length, 0.0, 1.0);
      out.emplace_back(from + ratio * (to - from));
      break;
    }
    out.emplace_back(to);
    accumulated += segment_length;
    if (accumulated >= limit) {
      break;
    }
  }
  return out;
}
}  // namespace

void FastExplorationFSM::pubState() {
  std_msgs::Empty heartbeat_msg;
  heartbeat_pub_.publish(heartbeat_msg);
  std_msgs::Bool msg;
  msg.data = fd_->static_state_;
  static_pub_.publish(msg);
  Marker state_marker;
  state_marker.type = Marker::TEXT_VIEW_FACING;
  state_marker.pose.position.x = fd_->odom_pos_.x();
  state_marker.pose.position.y = fd_->odom_pos_.y();
  state_marker.pose.position.z = fd_->odom_pos_.z();
  state_marker.pose.orientation.w = 1.0;
  state_marker.scale.x = state_marker.scale.y = state_marker.scale.z = 0.5;
  state_marker.action = Marker::ADD;
  state_marker.color.r = 1.0;
  state_marker.color.a = 1.0;
  state_marker.text = fd_->state_str_[int(state_)];
  state_marker.header.frame_id = "world";
  state_marker.header.stamp = ros::Time::now();

  state_pub_.publish(state_marker);

  std::ostringstream speed_text;
  speed_text << "Speed: " << std::fixed << std::setprecision(2)
             << fd_->odom_vel_.norm() << " m/s";

  Marker speed_marker;
  speed_marker.type = Marker::TEXT_VIEW_FACING;
  speed_marker.pose.position.x = fd_->odom_pos_.x();
  speed_marker.pose.position.y = fd_->odom_pos_.y();
  speed_marker.pose.position.z = fd_->odom_pos_.z() + 0.6;
  speed_marker.pose.orientation.w = 1.0;
  speed_marker.scale.x = speed_marker.scale.y = speed_marker.scale.z = 0.45;
  speed_marker.action = Marker::ADD;
  speed_marker.color.g = 1.0;
  speed_marker.color.b = 1.0;
  speed_marker.color.a = 1.0;
  speed_marker.text = speed_text.str();
  speed_marker.header.frame_id = "world";
  speed_marker.header.stamp = state_marker.header.stamp;

  speed_pub_.publish(speed_marker);
}

void FastExplorationFSM::resetFinishGate(const string &reason) {
  if (finish_gate_.no_frontier_count > 0 ||
      finish_gate_.no_reachable_count > 0 ||
      finish_gate_.force_recheck_requested) {
    ROS_INFO_STREAM("[finish gate] reset by " << reason);
  }
  finish_gate_ = FinishGate();
}

bool FastExplorationFSM::trajectoryEnded() const {
  if (!planner_manager_->hasCommittedTrajectory()) {
    return true;
  }
  const double remaining = planner_manager_->committedTrajectoryRemainingTime();
  return remaining <= std::max(0.05, fp_->replan_time_before_traj_end_);
}

bool FastExplorationFSM::finishGateSatisfied(const string &reason) const {
  if (expl_manager_->last_plan_no_reachable_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[finish gate] blocked by non-empty but unreachable/gated "
             "frontier after "
                 << reason);
    return false;
  }
  if (!expl_manager_->last_plan_empty_frontier_) {
    return false;
  }
  if (finish_gate_.no_frontier_count <
      fp_->finish_no_frontier_min_count_) {
    return false;
  }
  if (finish_gate_.first_no_frontier_time.isZero()) {
    return false;
  }
  const double no_frontier_duration =
      (ros::Time::now() - finish_gate_.first_no_frontier_time).toSec();
  if (no_frontier_duration < fp_->finish_no_frontier_min_duration_) {
    return false;
  }
  const bool vehicle_slow =
      fd_->odom_vel_.norm() <= fp_->finish_slow_speed_;
  const bool traj_ended = trajectoryEnded();
  if (fp_->finish_require_vehicle_slow_ && !vehicle_slow && !traj_ended) {
    return false;
  }
  if (expl_manager_->frontier_manager_ptr_) {
    const int active_clusters =
        expl_manager_->frontier_manager_ptr_->activeClusterCount();
    const int reachable_clusters =
        expl_manager_->frontier_manager_ptr_->reachableClusterCount();
    if (active_clusters > 0 || reachable_clusters > 0) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[finish gate] blocked by unresolved clusters after "
                   << reason << " active=" << active_clusters
                   << " reachable=" << reachable_clusters);
      return false;
    }
  }
  return true;
}

void FastExplorationFSM::requestFrontierRecheck(const string &reason) {
  if (!fd_->have_odom_ || !expl_manager_->frontier_manager_ptr_) {
    return;
  }
  const ros::Time now = ros::Time::now();
  if (!finish_gate_.last_force_recheck_time.isZero() &&
      (now - finish_gate_.last_force_recheck_time).toSec() < 1.0) {
    return;
  }
  finish_gate_.last_force_recheck_time = now;
  finish_gate_.force_recheck_requested = true;
  vector<ClusterInfo::Ptr> new_clusters;
  vector<int> cluster_removed;
  expl_manager_->frontier_manager_ptr_->forceGlobalRefresh(new_clusters,
                                                           cluster_removed);
  const int odom_id =
      planner_manager_->topo_graph_->history_odom_nodes_.empty()
          ? 0
          : static_cast<int>(
                planner_manager_->topo_graph_->history_odom_nodes_.size()) -
                1;
  for (auto &cls : new_clusters) {
    cls->odom_id_ = odom_id;
  }
  finish_gate_.force_recheck_requested = false;
  ROS_WARN_STREAM_THROTTLE(
      1.0, "[finish gate] force frontier refresh by "
               << reason << " new=" << new_clusters.size()
               << " removed=" << cluster_removed.size()
               << " active="
               << expl_manager_->frontier_manager_ptr_->activeClusterCount()
               << " reachable="
               << expl_manager_->frontier_manager_ptr_->reachableClusterCount());
}

void FastExplorationFSM::handleNoFrontierResult(const string &source) {
  const ros::Time now = ros::Time::now();
  if (finish_gate_.no_frontier_count == 0 ||
      finish_gate_.first_no_frontier_time.isZero()) {
    finish_gate_.first_no_frontier_time = now;
  }
  finish_gate_.no_frontier_count++;
  if (expl_manager_->last_plan_no_reachable_) {
    finish_gate_.no_reachable_count++;
  }
  requestFrontierRecheck(source);

  if (finishGateSatisfied(source)) {
    fd_->static_state_ = true;
    transitState(FINISH, source + ": finish gate satisfied");
    return;
  }

  fd_->static_state_ = true;
  ROS_WARN_STREAM_THROTTLE(
      1.0, "[finish gate] hold exploration after "
               << source << " no_frontier_count="
               << finish_gate_.no_frontier_count
               << " no_reachable_count="
               << finish_gate_.no_reachable_count);
  if (state_ != PLAN_TRAJ && state_ != WAIT_TRIGGER) {
    transitState(PLAN_TRAJ, source + ": no frontier gated", true);
  }
}

bool FastExplorationFSM::handleGoalReached() {
  if (expl_manager_->ed_->global_tour_.size() < 2) {
    return false;
  }
  const Eigen::Vector3f goal = expl_manager_->ed_->global_tour_[1];
  const double reached_radius =
      std::max(0.05, expl_manager_->ep_->goal_reached_radius_);
  if ((goal - fd_->odom_pos_).norm() > reached_radius) {
    return false;
  }

  const float visited_radius = static_cast<float>(
      std::max(expl_manager_->ep_->goal_lock_match_radius_,
               2.0 * reached_radius));
  const bool marked = expl_manager_->frontier_manager_ptr_->markClusterVisitedNear(
      goal, visited_radius);
  expl_manager_->ed_->has_goal_lock_ = false;
  expl_manager_->ed_->locked_goal_cluster_id_ = -1;
  if (expl_manager_->ep_->goal_lock_enable_) {
    ROS_INFO_STREAM("[goal reached] goal=(" << goal.x() << ", " << goal.y()
                                           << ", " << goal.z()
                                           << ") marked=" << marked);
  }

  if (fp_->finish_recheck_after_goal_reached_) {
    requestFrontierRecheck("goal reached");
  }

  if (finishGateSatisfied("goal reached")) {
    fd_->static_state_ = true;
    transitState(FINISH, "goal reached: finish gate satisfied");
  } else {
    expl_manager_->ed_->global_tour_.clear();
    expl_manager_->ed_->path_next_goal_.clear();
    expl_manager_->updateGoalNode();
    expl_manager_->last_plan_empty_frontier_ = false;
    expl_manager_->last_plan_no_reachable_ = false;
    if (state_ != PLAN_TRAJ) {
      transitState(PLAN_TRAJ, "goal reached: recheck frontier");
    }
  }
  return true;
}

int FastExplorationFSM::callExplorationPlanner() {
  // if (planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_) < planner_manager_->gcopter_config_->dilateRadiusHard)
  //   return START_FAIL;
  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
    return START_FAIL;
  if (expl_manager_->ed_->global_tour_.size() < 2)
    return NO_FRONTIER;
  fd_->reorientation_required_ = false;

  // debug
  if (planner_manager_->lidar_map_interface_->getDisToOcc(expl_manager_->ed_->next_goal_node_->center_) <
      planner_manager_->topo_graph_->bubble_min_radius_) { // TODO:
    cout << "410:  next goal in occ, update it" << endl;
    updateTopoAndGlobalPath();
    return FAIL;
  }
  vector<Eigen::Vector3f> path_next_goal;

  int res = planner_manager_->fast_searcher_->search(planner_manager_->topo_graph_->odom_node_, fd_->odom_vel_, expl_manager_->ed_->next_goal_node_,
                                                     0.2, path_next_goal);
  if (res == ParallelBubbleAstar::NO_PATH) {
    ROS_ERROR("ExplorationPlanner: No path to goal");
    return FAIL;

  } else if (res == ParallelBubbleAstar::START_FAIL) {
    ROS_ERROR("ExplorationPlanner: Start point in occ");
    return START_FAIL;
  } else if (res == ParallelBubbleAstar::END_FAIL) {
    ROS_ERROR("ExplorationPlanner: End point in occ");
    return FAIL;
  } else if (res == ParallelBubbleAstar::TIME_OUT) {
    ROS_ERROR("ExplorationPlanner: Time out");
    return FAIL;
  }

  auto info = &planner_manager_->local_data_;

  // The A* path is rooted at current odometry.  The General adapter is the
  // single owner of the future replan head and trims this path at its switch
  // state.  Prepending that future point here creates [future,current,goal]
  // and therefore a fake reverse segment at high speed.
  const std::size_t raw_path_size = path_next_goal.size();
  conditionHighSpeedPath(path_next_goal);
  if (path_next_goal.size() >= 2 &&
      planner_manager_->gcopter_config_->corridorCruiseEnable) {
    const Eigen::Vector3d start = path_next_goal.front().cast<double>();
    const Eigen::Vector3d goal = path_next_goal.back().cast<double>();
    Eigen::Vector3d direct = goal - start;
    Eigen::Vector3d direct_xy = direct;
    direct_xy.z() = 0.0;
    Eigen::Vector3d heading(std::cos(planner_manager_->local_data_.curr_yaw_),
                            std::sin(planner_manager_->local_data_.curr_yaw_),
                            0.0);
    Eigen::Vector3d vel_dir = planner_manager_->local_data_.curr_vel_;
    vel_dir.z() = 0.0;
    if (vel_dir.norm() > 0.5) {
      heading = vel_dir.normalized();
    }
    const double align =
        direct_xy.norm() > 1.0e-3
            ? std::clamp(direct_xy.normalized().dot(heading), -1.0, 1.0)
            : 1.0;
    const double direct_len = direct.norm();
    const double safe_distance =
        std::max(0.05, planner_manager_->gcopter_config_->commitKnownFreeSafeDistance);
    const double query_step =
        std::max(0.05, planner_manager_->gcopter_config_->safetyMapQueryStep);
    const RaycastSafetyInfo direct_safety = planner_manager_->raycastSafety(
        start, goal, true, safe_distance, query_step);
    const bool direct_known_free =
        direct_safety.all_known_free &&
        direct_safety.known_free_length + 1.0e-3 >= direct_len;
    if (direct_known_free &&
        direct_safety.known_free_length >=
            planner_manager_->gcopter_config_->knownFreeMediumLength &&
        align >= planner_manager_->gcopter_config_->corridorCruiseMinAlignment) {
      vector<Eigen::Vector3f> straight_path;
      straight_path.push_back(path_next_goal.front());
      const double step =
          std::max(4.0, planner_manager_->gcopter_config_->knownFreeShortLength);
      const int samples =
          std::max(1, static_cast<int>(std::floor(direct_len / step)));
      for (int s = 1; s < samples; ++s) {
        const double ratio = static_cast<double>(s) / static_cast<double>(samples);
        straight_path.push_back((start + ratio * direct).cast<float>());
      }
      straight_path.push_back(path_next_goal.back());
      path_next_goal.swap(straight_path);
      if (planner_manager_->gcopter_config_->velocityLogEnable) {
        ROS_INFO_STREAM("[corridor cruise path] straighten direct known-free path:"
                        << " len=" << direct_len
                        << " align=" << align
                        << " known_free=" << direct_safety.known_free_length
                        << " pts=" << path_next_goal.size());
      }
    }
  }
  vector<Eigen::Vector3f> path_next_goal_tmp;
  path_next_goal_tmp.push_back(path_next_goal[0]);

  const float path_step = static_cast<float>(std::max(0.5, fp_->path_densify_step_));
  for (int i = 1; i < static_cast<int>(path_next_goal.size());) {
    Eigen::Vector3f end_pt = path_next_goal_tmp.back();
    if ((path_next_goal[i] - end_pt).norm() > path_step) {
      Eigen::Vector3f dir = (path_next_goal[i] - end_pt).normalized();
      path_next_goal_tmp.push_back(end_pt + path_step * dir);
    } else if ((path_next_goal[i] - end_pt).norm() < 0.01) {
      i++;
    } else {
      path_next_goal_tmp.push_back(path_next_goal[i]);
      i++;
    }
  }
  expl_manager_->ed_->path_next_goal_.swap(path_next_goal_tmp);
  vector<Eigen::Vector3d> path_d = truncatePathHorizon(
      expl_manager_->ed_->path_next_goal_, planner_manager_->max_traj_len_);
  if (path_d.size() < 2) {
    return FAIL;
  }
  double horizon_end_yaw = planner_manager_->local_data_.curr_yaw_;
  Eigen::Vector3d horizon_tail = path_d.back() - path_d[path_d.size() - 2];
  if (std::hypot(horizon_tail.x(), horizon_tail.y()) > 1.0e-3) {
    horizon_end_yaw = std::atan2(horizon_tail.y(), horizon_tail.x());
  }
  const auto safety = planner_manager_->evaluatePathSegmentSafety(
      path_d, planner_manager_->local_data_.curr_yaw_,
      horizon_end_yaw);
  const auto limit = planner_manager_->computeSegmentVelocityLimit(safety);
  const double current_speed = planner_manager_->local_data_.curr_vel_.norm();
  if (expl_manager_->ep_->original_frontend_compatibility_) {
    expl_manager_->high_speed_mode_active_ =
        current_speed >=
        planner_manager_->gcopter_config_->highSpeedModeThreshold;
  } else if (expl_manager_->high_speed_mode_active_) {
    if (current_speed <=
        planner_manager_->gcopter_config_->highSpeedModeExitThreshold) {
      expl_manager_->high_speed_mode_active_ = false;
    }
  } else if (current_speed >=
             planner_manager_->gcopter_config_->highSpeedModeThreshold) {
    expl_manager_->high_speed_mode_active_ = true;
  }
  const bool high_speed = expl_manager_->high_speed_mode_active_;
  const bool hard_gate_enabled =
      planner_manager_->gcopter_config_->viewScoreHardGateEnable;
  const double required_clearance =
      std::max(planner_manager_->gcopter_config_->commitKnownFreeSafeDistance,
               planner_manager_->gcopter_config_->viewScoreHardGateMinClearance);
  const bool safety_rejected =
      !safety.backup_feasible ||
      safety.min_clearance + 0.05 < required_clearance;
  const bool reversal_rejected =
      safety.initial_heading_delta >
      planner_manager_->gcopter_config_->reorientationHeadingAngle;
  const bool geometry_rejected =
      safety.max_local_turn >
          planner_manager_->gcopter_config_->viewScoreHardGateMaxTurnAngle ||
      safety.yaw_delta >
          planner_manager_->gcopter_config_->viewScoreHardGateMaxYawDelta ||
      reversal_rejected;
  const bool moving_for_reorientation =
      current_speed > fp_->reorient_exit_speed_;
  const bool reorientation_required =
      fp_->controlled_reorientation_enable_ && reversal_rejected &&
      moving_for_reorientation;
  const bool reject_path =
      hard_gate_enabled &&
      (safety_rejected || reorientation_required ||
       (high_speed && geometry_rejected));
  if (reject_path) {
    ROS_WARN_STREAM_THROTTLE(
        0.5, "[path gate] reject target before optimization:"
                 << " len=" << safety.path_length
                 << " known_free=" << safety.known_free_length
                 << " min_clearance=" << safety.min_clearance
                 << " turn=" << safety.turn_angle
                 << " max_local_turn=" << safety.max_local_turn
                 << " initial_heading=" << safety.initial_heading_delta
                 << " yaw_delta=" << safety.yaw_delta
                 << " backup_feasible=" << safety.backup_feasible
                 << " sched_v=" << limit.final_limit
                 << " reason=" << limit.reason
                 << " high_speed=" << high_speed
                 << " reversal=" << reversal_rejected
                 << " geometry_rejected=" << geometry_rejected
                 << " safety_rejected=" << safety_rejected
                 << " speed=" << current_speed);
    // Keep the selected goal stable while braking. Clearing the lock here made
    // every retry choose another frontier and was a direct source of ping-pong
    // turnarounds near the end of the committed trajectory.
    fd_->reorientation_required_ =
        reorientation_required && !safety_rejected;
    expl_manager_->ed_->path_next_goal_.clear();
    return FAIL;
  }
  if (planner_manager_->gcopter_config_->velocityLogEnable) {
    ROS_INFO_STREAM(
        "[path condition] raw_pts=" << raw_path_size
                                    << " conditioned_pts=" << path_next_goal.size()
                                    << " dense_pts="
                                    << expl_manager_->ed_->path_next_goal_.size()
                                    << " len=" << safety.path_length
                                    << " known_free="
                                    << safety.known_free_length
                                    << " min_clearance="
                                    << safety.min_clearance
                                    << " turn=" << safety.turn_angle
                                    << " max_local_turn="
                                    << safety.max_local_turn
                                    << " initial_heading="
                                    << safety.initial_heading_delta
                                    << " backup_feasible="
                                    << safety.backup_feasible
                                    << " sched_v=" << limit.final_limit
                                    << " reason=" << limit.reason);
  }
  if (planner_manager_->planExploreTraj(expl_manager_->ed_->path_next_goal_, fd_->static_state_)) {
    traj_utils::PolyTraj poly_traj_msg;
    planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
    fd_->newest_traj_ = poly_traj_msg;
    traj_utils::PolyTraj poly_yaw_traj_msg;
    planner_manager_->polyYawTraj2ROSMsg(poly_yaw_traj_msg, info->start_time_);
    fd_->newest_yaw_traj_ = poly_yaw_traj_msg;
    return SUCCEED;
  } else {
    return FAIL;
  }
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg) {
  if (!msg || msg->poses.empty()) {
    ROS_WARN("[exploration trigger] ignore empty legacy waypoint path");
    return;
  }
  if (msg->poses.front().pose.position.z < -0.1)
    return;

  acceptManualTrigger("legacy waypoint path");
}

void FastExplorationFSM::navGoalTriggerCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg) {
    return;
  }
  ROS_INFO_STREAM("[exploration trigger] received 2D Nav Goal at ["
                  << msg->pose.position.x << ", " << msg->pose.position.y
                  << ", " << msg->pose.position.z
                  << "]; position is used only as a start trigger");
  acceptManualTrigger("2D Nav Goal");
}

void FastExplorationFSM::acceptManualTrigger(const string &source) {
  if (state_ != INIT && state_ != WAIT_TRIGGER) {
    ROS_WARN_STREAM("[exploration trigger] ignore " << source
                    << " while state=" << fd_->state_str_[state_]);
    return;
  }
  fd_->trigger_ = true;
  if (state_ == INIT) {
    ROS_INFO_STREAM("[exploration trigger] queued " << source
                    << "; waiting for first odometry sample");
    return;
  }

  ROS_INFO_STREAM("[exploration trigger] accepted " << source);
  total_time_ = ros::Time::now().toSec();
  resetFinishGate(source);
  transitState(PLAN_TRAJ, source);
}

void FastExplorationFSM::odometryCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  if (!msg) {
    return;
  }

  // Keep the complete message for latest_odom mode.  Wall time deliberately
  // measures local transport freshness and is independent of /clock or of the
  // timestamp convention used by an external simulator.
  {
    std::lock_guard<std::mutex> lock(latest_odom_mutex_);
    latest_odom_msg_ = msg;
    latest_odom_receive_wall_time_ = ros::WallTime::now();
  }

  fd_->odom_pos_ = Eigen::Vector3f(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z);
  fd_->odom_vel_ = Eigen::Vector3f(msg->twist.twist.linear.x,
                                  msg->twist.twist.linear.y,
                                  msg->twist.twist.linear.z);
  fd_->odom_orient_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                        msg->pose.pose.orientation.x,
                                        msg->pose.pose.orientation.y,
                                        msg->pose.pose.orientation.z);
  fd_->odom_yaw_ = static_cast<float>(tf::getYaw(msg->pose.pose.orientation));
  fd_->last_odom_receive_time_ = ros::Time::now();

  if (!fd_->have_odom_) {
    fd_->first_odom_time_ = fd_->last_odom_receive_time_;
  }
  fd_->have_odom_ = true;

  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  planner_manager_->local_data_.curr_yaw_ = fd_->odom_yaw_;
  if (planner_manager_->topo_graph_ &&
      planner_manager_->topo_graph_->odom_node_) {
    planner_manager_->topo_graph_->odom_node_->center_ = fd_->odom_pos_;
  }
}

void FastExplorationFSM::latestCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {
  if (!msg) {
    return;
  }

  nav_msgs::OdometryConstPtr odom;
  ros::WallTime odom_receive_time;
  {
    std::lock_guard<std::mutex> lock(latest_odom_mutex_);
    odom = latest_odom_msg_;
    odom_receive_time = latest_odom_receive_wall_time_;
  }

  if (!odom || odom_receive_time.isZero()) {
    ROS_WARN_THROTTLE(
        1.0, "[cloud input] latest_odom mode: no odometry received yet");
    return;
  }

  const double odom_receive_age =
      (ros::WallTime::now() - odom_receive_time).toSec();
  if (fp_->latest_odom_timeout_ > 0.0 &&
      odom_receive_age > fp_->latest_odom_timeout_) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[cloud input] latest_odom mode: odometry receive timeout age="
                 << odom_receive_age << "s max="
                 << fp_->latest_odom_timeout_ << "s");
    return;
  }

  if (!msg->header.stamp.isZero() && !odom->header.stamp.isZero()) {
    const double stamp_delta =
        std::abs((msg->header.stamp - odom->header.stamp).toSec());
    if (stamp_delta > 0.1) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[cloud input] latest_odom mode tolerating header stamp delta="
                   << stamp_delta << "s");
    }
  }

  CloudOdomCallback(msg, odom);
}

void FastExplorationFSM::CloudOdomCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    const nav_msgs::Odometry::ConstPtr &odom_) {
  if (!msg || !odom_) {
    ROS_WARN_THROTTLE(1.0, "[cloud input] null cloud or odometry message");
    return;
  }

  const ros::Time now = ros::Time::now();
  const bool valid_age = !now.isZero() && !msg->header.stamp.isZero();
  const double cloud_age =
      valid_age ? (now - msg->header.stamp).toSec() : -1.0;
  const std::uint64_t point_count =
      static_cast<std::uint64_t>(msg->width) *
      static_cast<std::uint64_t>(msg->height);
  static std::uint64_t dropped_stale_clouds = 0;
  const bool enforce_header_age =
      fp_->cloud_odom_mode_ == "approximate_sync";
  if (enforce_header_age && fp_->max_cloud_age_ > 0.0 && valid_age &&
      cloud_age > fp_->max_cloud_age_) {
    ++dropped_stale_clouds;
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[cloud input] drop stale synchronized cloud: age="
                 << cloud_age << "s max=" << fp_->max_cloud_age_
                 << "s points=" << point_count
                 << " dropped=" << dropped_stale_clouds);
    return;
  }
  ROS_INFO_STREAM_THROTTLE(
      1.0, "[cloud input] mode=" << fp_->cloud_odom_mode_
                                   << " points=" << point_count
                                   << " age="
                                   << (valid_age ? cloud_age : -1.0)
                                   << "s stamp_valid=" << valid_age);

  ros::Time t1 = ros::Time::now();
  planner_manager_->lidar_map_interface_->updateCloudMapOdometry(msg, odom_);
  planner_manager_->updateRogMap(msg, odom_);
  double collision_time;
  bool safe = planner_manager_->checkTrajCollision(collision_time);
  if (!safe) {
    transitState(PLAN_TRAJ, "safetyCallback: not safe, time:" + to_string(collision_time), true);
    if (collision_time < fp_->replan_time_ + 0.2)
      stopTraj();
  }
  ros::Time t2 = ros::Time::now();
  ros::Time t3 = ros::Time::now();

  if (planner_manager_->lidar_map_interface_->ld_->lidar_cloud_.points.empty())
    return;
  // Do not overwrite current FSM state with the odometry selected for this map
  // update. odometryCallback owns the live vehicle state; this callback owns
  // only the map/frontier update.
  vector<ClusterInfo::Ptr> new_clusters;
  vector<int> cluster_removed;
  expl_manager_->frontier_manager_ptr_->updateFrontierClusters(new_clusters, cluster_removed);
  const int odom_id =
      planner_manager_->topo_graph_->history_odom_nodes_.empty()
          ? 0
          : static_cast<int>(
                planner_manager_->topo_graph_->history_odom_nodes_.size()) -
                1;
  for (auto &cls : new_clusters) {
    cls->odom_id_ = odom_id;
  }
  ros::Time t4 = ros::Time::now();

  ROS_INFO_STREAM_THROTTLE(1.0, "cloud odom callback cost: " << "map update:" << (t2 - t1).toSec() * 1000 << "ms  "
                                                             << "update frontier clusters: " << (t4 - t3).toSec() * 1000 << "ms  "
                                                             << "total: " << (t4 - t1).toSec() * 1000 << "ms" << endl);
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call, bool red) {
  int pre_s = int(state_);
  state_ = new_state;
  if (!red) {
    cout << "\033[32m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  } else {
    cout << "\033[31m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  }
}

void FastExplorationFSM::stopTraj() {
  // A replan notification only shortens the polynomial in traj_server.  Its
  // mathematical endpoint can still carry several m/s of velocity, after which
  // traj_server switches directly to position HOLD.  Commit and publish an
  // actual dynamically feasible braking polynomial first.
  if (planner_manager_->planControlledStopTrajectory()) {
    traj_utils::PolyTraj stop_pos_msg;
    traj_utils::PolyTraj stop_yaw_msg;
    auto *info = &planner_manager_->local_data_;
    planner_manager_->polyTraj2ROSMsg(stop_pos_msg, info->start_time_);
    planner_manager_->polyYawTraj2ROSMsg(stop_yaw_msg, info->start_time_);
    if (!stop_pos_msg.duration.empty() && !stop_yaw_msg.duration.empty()) {
      fd_->newest_traj_ = stop_pos_msg;
      fd_->newest_yaw_traj_ = stop_yaw_msg;
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[controlled stop] published braking trajectory id="
                   << info->traj_id_ << " duration=" << info->duration_);
      return;
    }
  }

  // Retain the legacy emergency path only as a last resort when no collision-
  // free braking trajectory can be constructed.  REORIENT in known-free space
  // should never take this branch.
  ROS_ERROR_THROTTLE(
      1.0, "[controlled stop] braking trajectory generation failed; use "
           "legacy emergency truncation");
  replan_pub_.publish(std_msgs::Empty());
  ros::Time time_now = ros::Time::now();
  ros::Time start_time = planner_manager_->local_data_.start_time_;
  double curr_dur = planner_manager_->local_data_.duration_;
  planner_manager_->local_data_.duration_ = min(curr_dur, (time_now - start_time).toSec() + fp_->replan_time_);
  if (planner_manager_->local_data_.duration_ <= (time_now - start_time).toSec())
    fd_->static_state_ = true;
}
