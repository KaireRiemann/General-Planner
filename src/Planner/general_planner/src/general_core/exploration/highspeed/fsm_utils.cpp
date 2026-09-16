// Parse map headers before legacy exploration headers export global enums.
#include <map_manager/map_manager.hpp>
#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/fast_exploration_fsm.h>
#include <general_core/exploration/highspeed/target_directed_exploration.h>
#include <algorithm>
#include <cctype>
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
  if (execution_enabled_pub_) {
    // A PAUSED exploration node must keep mapping alive but must no longer
    // compete for the vehicle command topic.  The publisher is latched, so
    // transmit only transitions rather than the same Bool at the 100 Hz FSM
    // rate; this also keeps the trajectory-server log readable.
    const bool execution_enabled = state_ != PAUSED;
    if (!execution_enabled_published_ ||
        execution_enabled != last_execution_enabled_) {
      std_msgs::Bool msg;
      msg.data = execution_enabled;
      execution_enabled_pub_.publish(msg);
      execution_enabled_published_ = true;
      last_execution_enabled_ = execution_enabled;
    }
  }
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
  if ((state_ == PAUSED || state_ == PAUSING || state_ == FINISH) && !coverage_result_.empty()) {
    std::ostringstream result;
    result << (coverage_result_=="CONVERGED" ? "FINISH / COVERAGE_" : "COVERAGE_") << coverage_result_ << " " << std::fixed << std::setprecision(1)
           << 100.0*coverage_result_ratio_ << "%";
    state_marker.text=result.str();
  }
  if ((state_ == PLAN_TRAJ || state_ == EXEC_TRAJ) &&
      finish_gate_.no_frontier_count > 0 &&
      (expl_manager_->last_plan_empty_frontier_ ||
       expl_manager_->last_plan_no_reachable_)) {
    // During this state no new motion is expected: the planner is auditing
    // persistent coverage targets and the plateau timer. Expose that fact in
    // RViz instead of looking indistinguishable from a stalled PLAN_TRAJ.
    state_marker.text = "TERMINAL_AUDIT";
  }
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
  publishTaskStatus();
}

void FastExplorationFSM::resetFinishGate(const string &reason) {
  if (finish_gate_.no_frontier_count > 0 ||
      finish_gate_.no_reachable_count > 0 ||
      finish_gate_.force_recheck_requested) {
    ROS_INFO_STREAM("[finish gate] reset by " << reason);
  }
  finish_gate_ = FinishGate();
  if (expl_manager_ && expl_manager_->swarm_coordinator_ &&
      expl_manager_->swarm_coordinator_->enabled()) {
    expl_manager_->swarm_coordinator_->setLocalConverged(false);
  }
}

bool FastExplorationFSM::trajectoryEnded() const {
  if (!planner_manager_->hasCommittedTrajectory()) {
    return true;
  }
  const double remaining = planner_manager_->committedTrajectoryRemainingTime();
  return remaining <= std::max(0.05, fp_->replan_time_before_traj_end_);
}

bool FastExplorationFSM::finishGateSatisfied(const string &reason, bool require_stopped) const {
  if (!fd_->have_odom_ || fd_->last_odom_receive_time_.isZero() ||
      (ros::Time::now()-fd_->last_odom_receive_time_).toSec()>fp_->max_odom_age_ ||
      !expl_manager_->frontier_manager_ptr_) return false;
  const bool no_raw_frontier = expl_manager_->last_plan_empty_frontier_;
  const bool no_executable_frontier = expl_manager_->last_plan_no_reachable_;
  if (!no_raw_frontier && !no_executable_frontier) {
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
  if (expl_manager_->frontier_manager_ptr_) {
    if (!expl_manager_->frontier_manager_ptr_->frontierAuditReady()) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[finish gate] wait for full frontier reachability audit after "
                   << reason);
      return false;
    }
    const int active_clusters =
        expl_manager_->frontier_manager_ptr_->activeClusterCount();
    const int reachable_clusters =
        expl_manager_->frontier_manager_ptr_->reachableClusterCount();
    if (reachable_clusters > 0) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[finish gate] blocked by unresolved clusters after "
                   << reason << " active=" << active_clusters
                   << " reachable=" << reachable_clusters);
      return false;
    }
    if (active_clusters > 0 && no_executable_frontier) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[finish gate] accept stable non-executable raw frontiers after "
                   << reason << " active=" << active_clusters
                   << " reachable=0 confirmations="
                   << finish_gate_.no_reachable_count);
    }
  }
  const CoverageFinishStatus coverage_finish =
      expl_manager_->coverageFinishStatus();
  if (!coverage_finish.ready()) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[finish gate] wait for coverage convergence after "
                 << reason << ": plan_valid="
                 << coverage_finish.plan_valid
                 << " coverage=" << std::fixed << std::setprecision(4)
                 << coverage_finish.coverage_ratio
                 << " observed=" << coverage_finish.observed_voxels
                 << "/" << coverage_finish.valid_voxels
                 << " plateau="
                 << coverage_finish.plateau_reached
                 << " plateau_duration="
                 << std::setprecision(1)
                 << coverage_finish.plateau_duration
                 << "s targets_exhausted="
                 << coverage_finish.targets_exhausted
                 << " active_target_pending=" << coverage_finish.active_target_pending
                 << " exhausted="
                 << coverage_finish.exhausted_targets << "/"
                 << coverage_finish.actionable_targets
                 << " eligible=" << coverage_finish.eligible_targets
                 << " cooling=" << coverage_finish.cooling_targets
                 << " retry_after="
                 << std::setprecision(1)
                 << coverage_finish.next_retry_duration << "s");
    return false;
  }
  if (require_stopped &&
      ((fp_->finish_require_vehicle_slow_ && fd_->odom_vel_.norm() > fp_->finish_slow_speed_) ||
       !trajectoryEnded())) return false;
  if (require_stopped && coverage_finish.guard_enabled) {
    ROS_WARN_STREAM("[finish gate] " << "frontier and coverage audit converged after "
                    << reason << ": coverage=" << std::fixed
                    << std::setprecision(4)
                    << coverage_finish.coverage_ratio
                    << " plateau_duration=" << std::setprecision(1)
                    << coverage_finish.plateau_duration
                    << "s exhausted="
                    << coverage_finish.exhausted_targets << "/"
                    << coverage_finish.actionable_targets);
  }
  return true;
}

bool FastExplorationFSM::recordCoverageTermination(const std::string &source,
                                                  const std::string &blocked_reason) {
  if (!expl_manager_->coverageMotionEnabled()) return false;
  const auto status=expl_manager_->coverageFinishStatus();
  // Empty reason is used only after finishGateSatisfied(). Explicit execution
  // failures (liveness / escape deadline) remain BLOCKED, independently of
  // the raw unknown-volume statistic.
  coverage_blocked_pending_=!blocked_reason.empty();
  coverage_result_=coverage_blocked_pending_ ? "BLOCKED" : "CONVERGED";
  coverage_result_ratio_=status.coverage_ratio;
  std::ostringstream result;
  result << "{\"result\":\"" << coverage_result_ << "\",\"reason\":\""
         << (!blocked_reason.empty() ? blocked_reason :
             "no_reachable_frontier_after_full_audit")
         << "\",\"coverage_ratio\":" << status.coverage_ratio
         << ",\"observed_voxels\":" << status.observed_voxels
         << ",\"valid_voxels\":" << status.valid_voxels
         << ",\"unobserved_voxels\":" << std::max(0, status.valid_voxels-status.observed_voxels)
         << ",\"unresolved_unknown_groups\":" << status.unresolved_unknown_groups
         << ",\"exhausted_targets\":" << status.exhausted_targets
         << ",\"actionable_targets\":" << status.actionable_targets << "}";
  std_msgs::String msg; msg.data=result.str(); coverage_result_pub_.publish(msg);
  if (coverage_blocked_pending_) {
    ROS_WARN_STREAM("[coverage result] BLOCKED: " << result.str());
    beginPause(source+": remaining coverage could not be executed",false);
    return true;
  }
  return false;
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
  // A NO_FRONTIER result invalidates the previous navigation goal. Keeping
  // the old tour here lets PLAN_TRAJ optimize that stale path successfully on
  // the next tick, which resets the finish gate before its count/duration can
  // ever be satisfied.
  expl_manager_->ed_->global_tour_.clear();
  expl_manager_->ed_->path_next_goal_.clear();
  expl_manager_->ed_->has_goal_lock_ = false;
  expl_manager_->ed_->locked_goal_cluster_id_ = -1;
  expl_manager_->ed_->locked_goal_is_coverage_ = false;
  expl_manager_->ed_->locked_goal_is_mission_ = false;
  expl_manager_->ed_->locked_goal_coverage_id_ = 0;
  if (finish_gate_.no_frontier_count == 0 ||
      finish_gate_.first_no_frontier_time.isZero()) {
    finish_gate_.first_no_frontier_time = now;
  }
  finish_gate_.no_frontier_count++;
  if (expl_manager_->last_plan_no_reachable_) {
    finish_gate_.no_reachable_count++;
  }
  // A force refresh starts a two-phase audit: rebuild every cached frontier
  // cluster now, then validate viewpoints/reachability in the following global
  // planning pass. Do not restart that audit once the current revision has
  // already been fully validated.
  if (!expl_manager_->frontier_manager_ptr_->frontierAuditReady()) {
    requestFrontierRecheck(source);
  }

  if (expl_manager_->coverageTerminalAuditPending() &&
      !expl_manager_->hasActiveCoverageRecoveryGoal() &&
      !planner_manager_->hasCommittedStopTrajectory() &&
      fd_->odom_vel_.norm()>fp_->controlled_stop_min_speed_ &&
      finishGateSatisfied(source, false)) {
    // Full audit and repeated empty results must precede the stop command.
    // A request to refresh frontiers is not itself proof of completion.
    stopTraj("coverage terminal audit: settle before completion");
  }

  if (finishGateSatisfied(source)) {
    if (recordCoverageTermination(source)) return;
    if (expl_manager_->swarm_coordinator_ &&
        expl_manager_->swarm_coordinator_->enabled()) {
      const CoverageFinishStatus coverage =
          expl_manager_->coverageFinishStatus();
      const bool audit_ready =
          !expl_manager_->frontier_manager_ptr_ ||
          expl_manager_->frontier_manager_ptr_->frontierAuditReady();
      expl_manager_->swarm_coordinator_->setLocalConverged(
          true, audit_ready,
          !coverage.guard_enabled || coverage.plateau_reached,
          coverage.ready());
    }
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

void FastExplorationFSM::resetCoverageMotion() {
  coverage_retained_path_.clear();
  coverage_retained_time_ = ros::Time(0);
  coverage_passage_ = {};
  coverage_sequence_.clear();coverage_sequence_passed_=0;
}

bool FastExplorationFSM::handleGoalReached() {
  if (expl_manager_->missionGoalReached(fd_->odom_pos_.cast<double>())) {
    beginTargetArrivalVerification("target exploration: mission target radius entered");
    return true;
  }
  // A known-route prefix is not a frontier visit or coverage completion.
  if (expl_manager_->targetRouteSelected()) return false;
  if (expl_manager_->ed_->global_tour_.size() < 2) {
    return false;
  }
  const Eigen::Vector3f goal = expl_manager_->ed_->global_tour_[1];
  const double reached_radius =
      std::max(0.05, expl_manager_->ep_->goal_reached_radius_);
  const bool observed_passage = expl_manager_->coverageMotionEnabled() &&
      coverage_passage_.active && coverage_passage_.passed &&
      (goal.cast<double>() - coverage_passage_.goal).norm() <= reached_radius;
  // Recovery has its own configured observation radius (1 m in house).
  // The ordinary frontier's 0.35 m gate used to mask that check completely.
  bool coverage_reached = expl_manager_->coverageMotionEnabled() &&
      expl_manager_->completeActiveCoverageGoalIfReached(fd_->odom_pos_.cast<double>());
  if (!observed_passage && !coverage_reached && (goal - fd_->odom_pos_).norm() > reached_radius) {
    return false;
  }
  if (expl_manager_->coverageRouteEnabled() && !coverage_reached) {
    // Real position, orientation and a fresh accepted cloud are required.
    // Coverage itself is confirmed later by a newer frontier/map snapshot.
    const bool fresh=!coverage_cloud_received_.isZero() &&
        (ros::Time::now()-coverage_cloud_received_).toSec()<=.5;
    if (!observed_passage && (!fresh || coverage_route::yawDistance(
        fd_->odom_yaw_,expl_manager_->ed_->locked_goal_yaw_)>.75)) return false;
    if (observed_passage && coverage_sequence_.size()>1 &&
        expl_manager_->ed_->global_tour_.size()>2) {
      // Advance within the already validated and submitted observation prefix.
      // The ordinary safety timer can still invalidate it immediately.
      coverage_sequence_.erase(coverage_sequence_.begin());
      if (coverage_sequence_passed_) --coverage_sequence_passed_;
      auto &tour=expl_manager_->ed_->global_tour_;tour.erase(tour.begin()+1);
      const auto &next=coverage_sequence_.front();
      coverage_passage_={};coverage_passage_.active=true;coverage_passage_.goal=next.goal;
      coverage_passage_.radius=next.radius;coverage_passage_.passed=coverage_sequence_passed_>0;
      expl_manager_->ed_->locked_goal_=next.goal.cast<float>();
      expl_manager_->ed_->locked_goal_cluster_id_=next.cluster;
      expl_manager_->ed_->locked_goal_yaw_=next.yaw;
      expl_manager_->ed_->locked_goal_time_=ros::Time::now();
      planner_manager_->local_data_.end_yaw_=next.yaw;
      expl_manager_->updateGoalNode();
      return true;
    }
  }

  if (!expl_manager_->coverageMotionEnabled())
    coverage_reached = expl_manager_->completeActiveCoverageGoalIfReached(
          fd_->odom_pos_.cast<double>());
  if (expl_manager_->swarm_coordinator_ &&
      expl_manager_->swarm_coordinator_->enabled()) {
    expl_manager_->swarm_coordinator_->completeTaskAt(goal.cast<double>());
  }
  bool marked = false;
  if (!coverage_reached && !expl_manager_->coverageRouteEnabled()) {
    const float visited_radius = static_cast<float>(
        std::max(expl_manager_->ep_->goal_lock_match_radius_,
                 2.0 * reached_radius));
    marked = expl_manager_->frontier_manager_ptr_->markClusterVisitedNear(
        goal, visited_radius);
  }
  if (observed_passage) {
    expl_manager_->retainCoverageContinuation(planner_manager_->committedTrajectoryRemainingTime()-0.8);
    ROS_INFO("[coverage motion] measured observation passage; hand off while moving");
  }
  resetCoverageMotion();
  expl_manager_->ed_->has_goal_lock_ = false;
  expl_manager_->ed_->locked_goal_cluster_id_ = -1;
  expl_manager_->ed_->locked_goal_is_coverage_ = false;
  expl_manager_->ed_->locked_goal_is_mission_ = false;
  expl_manager_->ed_->locked_goal_coverage_id_ = 0;
  if (expl_manager_->ep_->goal_lock_enable_) {
    ROS_INFO_STREAM("[goal reached] goal=(" << goal.x() << ", " << goal.y()
                                           << ", " << goal.z()
                                           << ") coverage=" << coverage_reached
                                           << " marked=" << marked);
  }

  // A moving handoff has fresh sensor updates and a known continuation.
  // Defer the expensive full audit until the ordinary candidate pool is empty.
  if (fp_->finish_recheck_after_goal_reached_ && !observed_passage) {
    requestFrontierRecheck("goal reached");
  }

  if (finishGateSatisfied("goal reached")) {
    if (recordCoverageTermination("goal reached")) return true;
    if (expl_manager_->swarm_coordinator_ &&
        expl_manager_->swarm_coordinator_->enabled()) {
      const CoverageFinishStatus coverage =
          expl_manager_->coverageFinishStatus();
      const bool audit_ready =
          !expl_manager_->frontier_manager_ptr_ ||
          expl_manager_->frontier_manager_ptr_->frontierAuditReady();
      expl_manager_->swarm_coordinator_->setLocalConverged(
          true, audit_ready,
          !coverage.guard_enabled || coverage.plateau_reached,
          coverage.ready());
    }
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

void FastExplorationFSM::beginTargetArrivalVerification(
    const string &source) {
  if (target_arrival_verification_pending_ || !expl_manager_ ||
      !expl_manager_->targetDirectedModeActive()) {
    return;
  }

  // Standalone exploration has no task-result consumer or supervisor handover
  // contract. Preserve its historical FINISH behavior; the composed runtime
  // below owns the stricter task-completion semantics.
  if (!task_control_enable_) {
    expl_manager_->ed_->global_tour_.clear();
    expl_manager_->ed_->path_next_goal_.clear();
    expl_manager_->ed_->has_goal_lock_ = false;
    expl_manager_->ed_->locked_goal_is_coverage_ = false;
    expl_manager_->ed_->locked_goal_is_mission_ = false;
    expl_manager_->ed_->locked_goal_cluster_id_ = -1;
    expl_manager_->ed_->locked_goal_coverage_id_ = 0;
    fd_->static_state_ = true;
    transitState(FINISH, source);
    return;
  }

  target_arrival_verification_pending_ = true;
  const double target_error = expl_manager_->missionGoalDistance(
      fd_->odom_pos_.cast<double>());
  ROS_INFO_STREAM("[target exploration] enter terminal arrival verification: "
                  << "error=" << target_error
                  << " radius=" << expl_manager_->ep_->target_reached_radius_
                  << " speed=" << fd_->odom_vel_.norm()
                  << " position=(" << fd_->odom_pos_.transpose() << ")");
  // Do not mark the task completed here. beginPause sends a dynamically safe
  // brake, then PAUSING validates the stopped odometry against the mission
  // tolerance before exposing SUCCEEDED to PlannerSupervisor.
  beginPause(source + "; controlled terminal stop", false);
}

void FastExplorationFSM::resumeTargetArrivalCorrection() {
  target_arrival_verification_pending_ = false;
  completion_pending_ = false;
  target_unreachable_pending_ = false;
  pause_stop_issued_ = false;
  ++target_arrival_correction_count_;
  fd_->trigger_ = true;
  fd_->auto_triggered_ = true;
  fd_->static_state_ = true;
  fd_->next_plan_retry_time_ = ros::Time(0);
  expl_manager_->ed_->global_tour_.clear();
  expl_manager_->ed_->path_next_goal_.clear();
  expl_manager_->ed_->has_goal_lock_ = false;
  expl_manager_->ed_->locked_goal_is_coverage_ = false;
  expl_manager_->ed_->locked_goal_is_mission_ = false;
  expl_manager_->ed_->locked_goal_cluster_id_ = -1;
  expl_manager_->ed_->locked_goal_coverage_id_ = 0;
  resetFinishGate("target terminal correction");
  transitState(PLAN_TRAJ,
               "target terminal hold outside tolerance; replan correction",
               true);
}

double FastExplorationFSM::coverageReplanLead() const {
  return expl_manager_->coverageMotionEnabled() ?
      coverage_planning_budget_.lead(fp_->replan_time_before_traj_end_,
          planner_manager_->gcopter_config_->controlLatency) : fp_->replan_time_before_traj_end_;
}

int FastExplorationFSM::callExplorationPlanner() {
  refreshRuntimeOdometry();
  if (expl_manager_->coverageMotionEnabled() && !coverage_emergency_stop_until_.isZero()) {
    // Emergency truncation invalidated the stored polynomial. Wait for the
    // server's stop deadline AND fresh measured rest before seeding from odom.
    if (ros::Time::now()<coverage_emergency_stop_until_ ||
        fd_->last_odom_receive_time_.isZero() ||
        (ros::Time::now()-fd_->last_odom_receive_time_).toSec()>fp_->max_odom_age_ ||
        fd_->odom_vel_.norm()>fp_->controlled_stop_min_speed_) {
      planner_manager_->coverage_failure_.kind=CoverageFailureKind::HEAD;
      return START_FAIL;
    }
    planner_manager_->clearReleasedTrajectory();
    coverage_emergency_stop_until_=ros::Time(0);
  }
  if (expl_manager_->coverageMotionEnabled() && planner_manager_->hasCommittedTrajectory())
    fd_->static_state_=false;
  // if (planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_) < planner_manager_->gcopter_config_->dilateRadiusHard)
  //   return START_FAIL;
  last_plan_used_target_route_ = prepared_target_route_.ready();
  const bool known_route = last_plan_used_target_route_;
  const bool coverage_motion_enabled = expl_manager_->coverageMotionEnabled();
  planner_manager_->coverage_failure_ = {};
  if (!coverage_motion_enabled) resetCoverageMotion();
  if (!known_route && (!planner_manager_->topo_graph_->odom_node_ ||
      planner_manager_->topo_graph_->odom_node_->neighbors_.empty())) {
    if (coverage_motion_enabled) planner_manager_->coverage_failure_.kind=CoverageFailureKind::HEAD;
    return START_FAIL;
  }
  if (!known_route && expl_manager_->ed_->global_tour_.size() < 2)
    return NO_FRONTIER;
  fd_->reorientation_required_ = false;

  const auto &motion = expl_manager_->ep_->coverage_motion_;
  // Raw observed-free evidence is required for new shortcuts/continuations.
  // The legacy LIO fallback is deliberately not used to certify these paths.
  const coverage_motion::SegmentFree observed_free =
      [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
        const auto map = planner_manager_->sharedMapManager();
        const double step = map ? std::clamp(0.5 * map->getResolution(), 0.02, 0.10) : 0.10;
        const int samples = std::max(1, static_cast<int>(std::ceil((b - a).norm() / step)));
        for (int i = 0; i <= samples; ++i)
          if (!planner_manager_->isObservedLocalKnownFree(a + (b - a) *
              (static_cast<double>(i) / samples))) return false;
        return true;
      };
  coverage_motion::Path retained;
  if (coverage_motion_enabled && !coverage_retained_path_.empty() &&
      !coverage_retained_time_.isZero() &&
      (ros::Time::now() - coverage_retained_time_).toSec() <= 3.0 &&
      (coverage_retained_goal_ - expl_manager_->ed_->global_tour_[1].cast<double>()).norm() <= motion.path_match_radius) {
    auto adjusted = coverage_retained_path_;
    adjusted.back() = expl_manager_->ed_->global_tour_[1].cast<double>();
    retained = coverage_motion::reconnect(adjusted,
        fd_->odom_pos_.cast<double>(), motion.path_match_radius, observed_free);
  }

  // debug
  if (!known_route && planner_manager_->lidar_map_interface_->getDisToOcc(expl_manager_->ed_->next_goal_node_->center_) <
      planner_manager_->topo_graph_->bubble_min_radius_) { // TODO:
    cout << "410:  next goal in occ, update it" << endl;
    if (expl_manager_->coverageRouteEnabled()) planner_manager_->coverage_failure_.kind=CoverageFailureKind::SPATIAL;
    else updateTopoAndGlobalPath();
    return FAIL;
  }
  vector<Eigen::Vector3f> path_next_goal;
  if (known_route) {
    path_next_goal = prepared_target_route_.path;
  } else {
  int res = BubbleAstar::NO_PATH;
  if (expl_manager_->coverageRouteEnabled()) {
    const auto &observations=expl_manager_->coverageRouteObservations();
    for (const auto &task:observations) {
      if ((task.position-expl_manager_->ed_->global_tour_[1].cast<double>()).norm()>.1) continue;
      for (const auto &point:task.path_from_previous) path_next_goal.push_back(point.cast<float>());
      if (planner_manager_->prepareCoveragePath(path_next_goal,fd_->static_state_,planner_manager_->max_traj_len_)) {
        res=BubbleAstar::REACH_END;
        ROS_INFO_STREAM_THROTTLE(.5,"[coverage route] reuse topology prefix map=" << task.map_version);
      } else path_next_goal.clear();
      break;
    }
  }
  if (res != BubbleAstar::REACH_END) res = planner_manager_->fast_searcher_->search(
      planner_manager_->topo_graph_->odom_node_, fd_->odom_vel_, expl_manager_->ed_->next_goal_node_,
      expl_manager_->coverageRouteEnabled() ? .05 : .2, path_next_goal);
  if (res != BubbleAstar::REACH_END && coverage_motion_enabled &&
      expl_manager_->coveragePreflightPath(expl_manager_->ed_->global_tour_[1],path_next_goal)) {
    res = BubbleAstar::REACH_END;
    ROS_INFO_THROTTLE(0.5,"[coverage prefix] local search failed; reuse candidate preflight");
  }
  if (res != BubbleAstar::REACH_END && !retained.empty()) {
    path_next_goal.clear();
    for (const auto &p : retained) path_next_goal.push_back(p.cast<float>());
    res = BubbleAstar::REACH_END;
    ROS_INFO_THROTTLE(0.5, "[coverage motion] search failed; reuse revalidated route");
  }
  if (res != BubbleAstar::REACH_END && coverage_motion_enabled)
    planner_manager_->coverage_failure_.kind = res==ParallelBubbleAstar::START_FAIL ?
        CoverageFailureKind::HEAD : CoverageFailureKind::PATH;
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
  }
  if (path_next_goal.size() < 2) return FAIL;

  if (coverage_motion_enabled) {
    coverage_motion::Path candidate;
    for (const auto &p : path_next_goal) candidate.push_back(p.cast<double>());
    candidate = coverage_motion::shortcut(candidate, motion.shortcut_distance, observed_free);
    const Eigen::Vector3d heading = fd_->odom_vel_.norm() > 0.5
        ? Eigen::Vector3d(fd_->odom_vel_.cast<double>())
        : Eigen::Vector3d(std::cos(fd_->odom_yaw_), std::sin(fd_->odom_yaw_), 0.0);
    const bool removes_reversal = retained.size() >= 2 && candidate.size() >= 2 &&
        coverage_motion::angle(retained[1] - retained[0], heading) >
            planner_manager_->gcopter_config_->reorientationHeadingAngle &&
        coverage_motion::angle(candidate[1] - candidate[0], heading) < 1.0;
    const bool reuse = !removes_reversal &&
        coverage_motion::preferRetained(retained, candidate, motion);
    const auto &chosen = reuse ? retained : candidate;
    path_next_goal.clear();
    for (const auto &p : chosen) path_next_goal.push_back(p.cast<float>());
    ROS_INFO_STREAM_THROTTLE(0.5, "[coverage motion] route=" << (reuse ? "retained" : "new")
        << " length=" << coverage_motion::length(chosen)
        << " alternative=" << coverage_motion::length(candidate));
  }
  const vector<Eigen::Vector3f> selected_observation_path = path_next_goal;

  auto info = &planner_manager_->local_data_;

  // The A* path is rooted at current odometry.  The General adapter is the
  // single owner of the future replan head and trims this path at its switch
  // state.  Prepending that future point here creates [future,current,goal]
  // and therefore a fake reverse segment at high speed.
  const std::size_t raw_path_size = path_next_goal.size();
  if (!known_route && !coverage_motion_enabled) conditionHighSpeedPath(path_next_goal);
  if (!known_route && path_next_goal.size() >= 2 &&
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
    if (direct_known_free && (!coverage_motion_enabled || observed_free(start, goal)) &&
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

  const double corridor_step = std::max(0.1,
      0.9 * planner_manager_->gcopter_config_->corridorLineMaxLength);
  const float path_step = static_cast<float>(coverage_motion_enabled
      ? std::min(std::max(0.1, fp_->path_densify_step_), corridor_step)
      : std::max(0.5, fp_->path_densify_step_));
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
  const Eigen::Vector3d requested_goal = coverage_motion_enabled
      ? expl_manager_->ed_->global_tour_[1].cast<double>()
      : expl_manager_->ed_->path_next_goal_.back().cast<double>();
  bool needs_coverage_prefix = coverage_motion_enabled;
  bool prepared_coverage_prefix = false;
  if (coverage_motion_enabled) {
    vector<Eigen::Vector3d> existing_path;
    for (const auto &p:expl_manager_->ed_->path_next_goal_) existing_path.push_back(p.cast<double>());
    const auto existing_safety=planner_manager_->evaluatePathSegmentSafety(
        existing_path,planner_manager_->local_data_.curr_yaw_,planner_manager_->local_data_.curr_yaw_);
    const double required=std::max(planner_manager_->gcopter_config_->commitKnownFreeSafeDistance,
                                  planner_manager_->gcopter_config_->viewScoreHardGateMinClearance);
    // Preserve already executable paths. Requiring every ordinary task to
    // use raw-free prefixes changed the observation route and slowed coverage.
    needs_coverage_prefix=!existing_safety.backup_feasible || existing_safety.min_clearance+0.05<required;
    if (expl_manager_->coverageRouteEnabled())
      needs_coverage_prefix=needs_coverage_prefix || !coverage_motion::pathFree(existing_path,observed_free);
  }
  if (needs_coverage_prefix && (expl_manager_->coverageRouteEnabled() || fd_->odom_vel_.norm() <= 0.20)) {
    // Prefer the remaining ordinary observations over a partial excursion to
    // a currently unexecutable one. The strict recovery pool (or last ordinary
    // target) may advance by an observed prefix when alternatives are gone.
    if (!expl_manager_->coverageRouteEnabled() && !expl_manager_->hasActiveCoverageRecoveryGoal() &&
        expl_manager_->ed_->global_tour_.size()>2) {
      planner_manager_->coverage_failure_.kind=CoverageFailureKind::PATH;
      ROS_INFO_THROTTLE(0.5,"[coverage prefix] prefer another ordinary observation before partial advance");
      return FAIL;
    }
    if (!planner_manager_->prepareCoveragePath(expl_manager_->ed_->path_next_goal_,
                                               fd_->static_state_, planner_manager_->max_traj_len_)) {
      if (planner_manager_->coverage_failure_.kind != CoverageFailureKind::HEAD)
        planner_manager_->coverage_failure_.kind = CoverageFailureKind::PATH;
      ROS_WARN_THROTTLE(0.5, "[coverage prefix] no observed-free prefix with braking room at switch state");
      return FAIL;
    }
    if ((expl_manager_->ed_->path_next_goal_.back().cast<double>()-requested_goal).norm()>0.10)
      ROS_INFO_STREAM_THROTTLE(0.5, "[coverage prefix] advance through observed free space; retain remote goal=("
          << requested_goal.transpose() << ") endpoint=("
          << expl_manager_->ed_->path_next_goal_.back().transpose() << ")");
    prepared_coverage_prefix = true;
  }
  vector<Eigen::Vector3d> path_d = truncatePathHorizon(
      expl_manager_->ed_->path_next_goal_, planner_manager_->max_traj_len_);
  if (path_d.size() < 2) {
    return FAIL;
  }
  bool rolling_horizon =
      (path_d.back() - requested_goal).norm() > 0.10;
  bool truncated_before_reversal = false;
  // Topological paths can contain a local out-and-back loop when the current
  // odom node is attached to both sides of the same skeleton branch. Collapse
  // the spur if the returning leg reaches the point before the reversal. The
  // previous implementation always truncated at the spur tip; for a 2--3 m
  // spur that produced a two-point zero-duration MINCO trajectory and retried
  // the same goal forever.
  for (std::size_t i = 1; !known_route && i + 1 < path_d.size(); ++i) {
    const Eigen::Vector3d incoming = path_d[i] - path_d[i - 1];
    const Eigen::Vector3d outgoing = path_d[i + 1] - path_d[i];
    if (incoming.norm() < 0.20 || outgoing.norm() < 0.20) {
      continue;
    }
    const double angle = std::acos(std::clamp(
        incoming.normalized().dot(outgoing.normalized()), -1.0, 1.0));
    if (angle > 2.60 && (path_d[i] - path_d.front()).norm() > 0.75) {
      const double return_radius =
          std::max(0.75, 0.60 * static_cast<double>(path_step));
      std::size_t return_index = path_d.size();
      for (std::size_t j = i + 1; j < path_d.size(); ++j) {
        if ((path_d[j] - path_d[i - 1]).norm() <= return_radius) {
          return_index = j;
          break;
        }
        // A real hairpin does not return to the incoming branch. Limit the
        // loop search so it cannot erase a large intentional detour.
        if ((path_d[j] - path_d[i]).norm() >
            2.5 * incoming.norm() + return_radius) {
          break;
        }
      }
      if (return_index < path_d.size()) {
        vector<Eigen::Vector3d> collapsed;
        collapsed.reserve(path_d.size() - (return_index - i));
        collapsed.insert(collapsed.end(), path_d.begin(), path_d.begin() + i);
        for (std::size_t j = return_index; j < path_d.size(); ++j) {
          if (collapsed.empty() ||
              (path_d[j] - collapsed.back()).norm() > 0.10) {
            collapsed.push_back(path_d[j]);
          }
        }
        if (collapsed.size() >= 2) {
          ROS_WARN_STREAM_THROTTLE(
              0.5, "[path horizon] collapse local out-and-back loop: angle="
                       << angle << " removed_pts="
                       << (return_index - i + 1)
                       << " remaining_pts=" << collapsed.size());
          path_d.swap(collapsed);
          // Re-scan because a topological path may contain adjacent spurs.
          i = 0;
          continue;
        }
      }

      path_d.resize(i + 1);
      // A prefix ending immediately before a reversal is a deliberate stop
      // boundary, not a receding-horizon continuation. Giving this endpoint
      // a non-zero velocity would recreate the out-and-back oscillation that
      // this guard is meant to remove.
      truncated_before_reversal = true;
      rolling_horizon = false;
      expl_manager_->ed_->path_next_goal_.clear();
      expl_manager_->ed_->path_next_goal_.reserve(path_d.size());
      for (const auto &point : path_d) {
        expl_manager_->ed_->path_next_goal_.push_back(point.cast<float>());
      }
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[path horizon] truncate before local reversal: angle="
                   << angle << " prefix_pts=" << path_d.size()
                   << " displacement="
                   << (path_d.back() - path_d.front()).norm());
      break;
    }
  }
  // Keep the frontend path consistent with either a collapsed loop or a
  // truncated prefix before handing it to the adapter.
  expl_manager_->ed_->path_next_goal_.clear();
  expl_manager_->ed_->path_next_goal_.reserve(path_d.size());
  for (const auto &point : path_d) {
    expl_manager_->ed_->path_next_goal_.push_back(point.cast<float>());
  }
  auto pathEndYaw = [&](const vector<Eigen::Vector3d> &candidate) {
    double yaw = planner_manager_->local_data_.curr_yaw_;
    if (candidate.size() >= 2) {
      const Eigen::Vector3d tail =
          candidate.back() - candidate[candidate.size() - 2];
      if (std::hypot(tail.x(), tail.y()) > 1.0e-3) {
        yaw = std::atan2(tail.y(), tail.x());
      }
    }
    return yaw;
  };
  double horizon_end_yaw = pathEndYaw(path_d);
  auto safety = planner_manager_->evaluatePathSegmentSafety(
      path_d, planner_manager_->local_data_.curr_yaw_, horizon_end_yaw);
  if (fp_->adaptive_tight_path_horizon_enable_ &&
      (safety.turn_angle > fp_->tight_path_turn_threshold_ ||
       safety.max_local_turn > 0.5 * fp_->tight_path_turn_threshold_)) {
    const double original_horizon = planner_manager_->max_traj_len_;
    const double turn_excess =
        std::max(0.0, safety.turn_angle - fp_->tight_path_turn_threshold_);
    const double minimum_horizon =
        std::min(fp_->tight_path_min_horizon_, original_horizon);
    const double tight_horizon = std::clamp(
        original_horizon / (1.0 + 0.35 * turn_excess),
        minimum_horizon, original_horizon);
    vector<Eigen::Vector3d> tight_path = truncatePathHorizon(
        expl_manager_->ed_->path_next_goal_, tight_horizon);
    if (tight_path.size() >= 2 && tight_path.size() < path_d.size()) {
      path_d.swap(tight_path);
      if (!truncated_before_reversal &&
          (path_d.back() - requested_goal).norm() > 0.10) {
        rolling_horizon = true;
      }
      horizon_end_yaw = pathEndYaw(path_d);
      safety = planner_manager_->evaluatePathSegmentSafety(
          path_d, planner_manager_->local_data_.curr_yaw_, horizon_end_yaw);
      expl_manager_->ed_->path_next_goal_.clear();
      expl_manager_->ed_->path_next_goal_.reserve(path_d.size());
      for (const auto &point : path_d) {
        expl_manager_->ed_->path_next_goal_.push_back(point.cast<float>());
      }
      ROS_INFO_STREAM_THROTTLE(
          0.5, "[path horizon] shorten high-curvature local path: horizon="
                   << tight_horizon << "m pts=" << path_d.size()
                   << " turn=" << safety.turn_angle
                   << " max_local_turn=" << safety.max_local_turn);
    }
  }
  CoverageObservationContext observation;
  CoverageMotionConfig observation_motion=motion;
  // A route-selected corner may be negotiated at a lower transit speed.
  // Keep the raw-free guide and backend dynamic/yaw checks as hard gates.
  if (expl_manager_->coverageRouteEnabled())
    observation_motion.extension_max_turn=std::max(motion.extension_max_turn,1.2);
  const vector<Eigen::Vector3f> stopped_goal_path = expl_manager_->ed_->path_next_goal_;
  const bool stopped_goal_rolling = rolling_horizon;
  if (coverage_motion_enabled && motion.continuous_observation &&
      !expl_manager_->hasActiveCoverageRecoveryGoal() && !truncated_before_reversal &&
      !rolling_horizon && (expl_manager_->coverageRouteEnabled() || expl_manager_->ed_->global_tour_.size() >= 3) &&
      (path_d.back() - expl_manager_->ed_->global_tour_[1].cast<double>()).norm() <= 0.10) {
    const Eigen::Vector3d goal = path_d.back();
    // Extend toward the next ordered task only. Never invent a forward
    // excursion beyond a terminal viewpoint simply to maintain speed.
    const auto &tasks=expl_manager_->coverageRouteObservations();
    std::size_t current=0;
    while (current<tasks.size() && (tasks[current].position-goal).norm()>.1) ++current;
    const auto &continuation=current+1<tasks.size() ? tasks[current+1].path_from_previous :
        expl_manager_->coverageRouteExitPath();
    const bool extended=expl_manager_->coverageRouteEnabled() ?
        (current<tasks.size() && coverage_motion::appendPathContinuation(path_d,
            continuation,planner_manager_->max_traj_len_,observation_motion,observed_free)) :
        coverage_motion::appendContinuation(path_d,expl_manager_->ed_->global_tour_[2].cast<double>(),
            planner_manager_->max_traj_len_,observation_motion,observed_free);
    if (extended) {
      observation.enabled = true;
      observation.goal = goal;
      observation.radius = std::max(0.05, expl_manager_->ep_->goal_reached_radius_);
      if (expl_manager_->coverageRouteEnabled()) {
        if (current+2<tasks.size() && (path_d.back()-tasks[current+1].position).norm()<.1)
          coverage_motion::appendPathContinuation(path_d,tasks[current+2].path_from_previous,
              planner_manager_->max_traj_len_,observation_motion,observed_free);
        for (std::size_t task_index=current;task_index<tasks.size();++task_index) {
          const auto &task=tasks[task_index];
          if (observation.gates.size()>=3 || task.cluster<0) break;
          double distance=std::numeric_limits<double>::infinity();
          for (std::size_t k=1;k<path_d.size();++k) distance=std::min(distance,
              coverage_motion::pointSegmentDistance(task.position,path_d[k-1],path_d[k]));
          if (distance>observation.radius) break;
          CoverageObservationGate gate;gate.goal=task.position;gate.yaw=task.yaw;
          gate.yaw_tolerance=.75;gate.radius=observation.radius;
          gate.identity=task.identity;gate.cluster=task.cluster;
          observation.gates.push_back(gate);
        }
      }
      expl_manager_->ed_->path_next_goal_.clear();
      expl_manager_->ed_->path_next_goal_.push_back(path_d.front().cast<float>());
      for (std::size_t i = 1; i < path_d.size(); ++i) {
        const int pieces = std::max(1, static_cast<int>(std::ceil(
            (path_d[i] - path_d[i - 1]).norm() / path_step)));
        for (int j = 1; j <= pieces; ++j) {
          const Eigen::Vector3d p = path_d[i - 1] + (path_d[i] - path_d[i - 1]) *
              (static_cast<double>(j) / pieces);
          expl_manager_->ed_->path_next_goal_.push_back(p.cast<float>());
        }
      }
      horizon_end_yaw = pathEndYaw(path_d);
      if (!observation.gates.empty()) horizon_end_yaw=observation.gates.back().yaw;
      safety = planner_manager_->evaluatePathSegmentSafety(
          path_d, planner_manager_->local_data_.curr_yaw_, horizon_end_yaw);
      // This prefix may end at rest; the observation is now in its interior.
      // Do not relax the adapter's existing nonstop/backup requirements.
      rolling_horizon = expl_manager_->coverageRouteEnabled() ?
          (current+1>=tasks.size() || (path_d.back()-tasks[current+1].position).norm()>.1) :
          (path_d.back()-expl_manager_->ed_->global_tour_[2].cast<double>()).norm()>.1;
    }
  }
  // A full ordinary observation endpoint has the same yaw/evidence contract
  // as an interior observation. Otherwise the backend's path-heading yaw
  // could make arrival impossible to verify, even while standing at the goal.
  if (expl_manager_->coverageRouteEnabled() && !observation.enabled && !rolling_horizon &&
      !expl_manager_->hasActiveCoverageRecoveryGoal() &&
      !expl_manager_->coverageRouteObservations().empty() &&
      (path_d.back()-requested_goal).norm()<.1) {
    const auto &task=expl_manager_->coverageRouteObservations().front();
    if ((task.position-requested_goal).norm()<.1) {
      observation.enabled=true;observation.goal=task.position;
      observation.radius=std::max(.05,expl_manager_->ep_->goal_reached_radius_);
      CoverageObservationGate gate;gate.goal=task.position;gate.yaw=task.yaw;
      gate.radius=observation.radius;gate.yaw_tolerance=.75;gate.identity=task.identity;gate.cluster=task.cluster;
      observation.gates={gate};
    }
  }
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
  if (prepared_coverage_prefix && safety.known_free_length + 0.05 >= safety.path_length) {
    const auto &cfg=*planner_manager_->gcopter_config_;
    const double stop_length=current_speed*(cfg.plannerLatency+cfg.controlLatency)+
        current_speed*current_speed/(2.0*std::max(1.0,cfg.brakeAccel))+cfg.safetyBrakeMargin;
    // A fully observed local prefix can end at rest without a four-metre
    // cruise runway. Actual derivative and braking checks remain in MINCO.
    safety.backup_feasible=current_speed<=0.20 || safety.path_length>=stop_length;
  }
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
    if (reorientation_required && !safety_rejected && expl_manager_->coverageContinuationActive()) {
      double collision_time=0.0;
      if (planner_manager_->checkTrajCollision(collision_time,true)) {
        planner_manager_->coverage_failure_.kind=CoverageFailureKind::HEAD;
        return FAIL;
      }
    }
    if (coverage_motion_enabled) planner_manager_->coverage_failure_.kind =
        safety.min_clearance+.05<required_clearance ? CoverageFailureKind::SPATIAL :
        (!safety.backup_feasible ? CoverageFailureKind::PATH : CoverageFailureKind::DYNAMICS);
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
                                    << " rolling_horizon="
                                    << rolling_horizon
                                    << " sched_v=" << limit.final_limit
                                    << " reason=" << limit.reason);
  }
  if (known_route) rolling_horizon = false;
  const auto optimization_started=ros::WallTime::now();
  bool planned = planner_manager_->planExploreTraj(expl_manager_->ed_->path_next_goal_,
                                        fd_->static_state_, false,
                                        rolling_horizon,
                                        known_route ? prepared_target_route_.context
                                                    : TargetRouteExecutionContext{}, observation,
                                        CoverageExecutionContext{coverage_motion_enabled,false});
  const double backend_elapsed=(ros::WallTime::now()-optimization_started).toSec();
  const bool route_retry_budget = !coverage_motion_enabled ||
      (backend_elapsed<0.18 && planner_manager_->coverage_failure_.kind!=CoverageFailureKind::BUDGET &&
       (fd_->odom_vel_.norm()<=.20 || planner_manager_->committedTrajectoryRemainingTime()>
          backend_elapsed+planner_manager_->gcopter_config_->controlLatency+.1));
  bool backend_retried=false;
  if (!planned && observation.enabled && route_retry_budget &&
      (!coverage_motion_enabled ||
       planner_manager_->coverage_failure_.kind!=CoverageFailureKind::HEAD)) {
    backend_retried=true;
    // One bounded fallback through the unchanged stopped-viewpoint pipeline.
    CoverageObservationContext stopped_observation;
    if (expl_manager_->coverageRouteEnabled() && !observation.gates.empty() &&
        !stopped_goal_rolling && (stopped_goal_path.back().cast<double>()-observation.goal).norm()<.1) {
      stopped_observation=observation;stopped_observation.gates.resize(1);
    }
    observation = stopped_observation;
    expl_manager_->ed_->path_next_goal_ = stopped_goal_path;
    planned = planner_manager_->planExploreTraj(stopped_goal_path,
        fd_->static_state_, false, stopped_goal_rolling, {}, observation,
        CoverageExecutionContext{coverage_motion_enabled,false});
    ROS_INFO_STREAM("[coverage motion] continuation fallback success=" << planned);
  }
  const bool stationary_recovery = coverage_motion_enabled && fd_->odom_vel_.norm() <= 0.20;
  if (!planned && coverage_motion_enabled && !backend_retried && route_retry_budget &&
      (stationary_recovery || planner_manager_->coverage_failure_.kind==CoverageFailureKind::SPATIAL) &&
      !fd_->reorientation_required_ &&
      planner_manager_->coverage_failure_.kind != CoverageFailureKind::HEAD) {
    // One geometric retry at the same remote goal, rather than four speeds
    // through the same blocked corner or immediately cooling the whole task.
    const Eigen::Vector3d repair_position=planner_manager_->coverage_failure_.kind==CoverageFailureKind::SPATIAL
        ? planner_manager_->coverage_failure_.position
        : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    auto repair_path = stopped_goal_path;
    if (planner_manager_->prepareCoveragePath(repair_path, fd_->static_state_, 8.0)) {
      observation = {};
      const bool repair_rolling = (repair_path.back().cast<double>() - requested_goal).norm() > 0.10;
      planned=planner_manager_->planExploreTraj(repair_path,fd_->static_state_,false,repair_rolling,{}, {},
                                               CoverageExecutionContext{true,true,repair_position});
      if (planned) expl_manager_->ed_->path_next_goal_=repair_path;
      ROS_INFO_STREAM("[coverage repair] constrained seed corridor success=" << planned);
    }
  }
  if (planned) {
    if (coverage_motion_enabled) {
      coverage_retained_path_.clear();
      for (const auto &p : selected_observation_path) coverage_retained_path_.push_back(p.cast<double>());
      coverage_retained_goal_ = expl_manager_->ed_->global_tour_[1].cast<double>();
      coverage_retained_time_ = ros::Time::now();
      if (observation.enabled) {
        if (expl_manager_->coverageRouteEnabled()) {
          coverage_sequence_=observation.orderedGates();coverage_sequence_passed_=0;
          if (coverage_sequence_.size()>1) {
            auto &tour=expl_manager_->ed_->global_tour_;
            tour.resize(1);
            for (const auto &gate:coverage_sequence_) tour.push_back(gate.goal.cast<float>());
          }
        }
        if (!coverage_passage_.active ||
            (coverage_passage_.goal - observation.goal).norm() > 0.10) coverage_passage_ = {};
        coverage_passage_.active = true;
        coverage_passage_.goal = observation.goal;
        coverage_passage_.radius = observation.radius;
        if (!expl_manager_->coverageRouteEnabled())
          coverage_passage_.observe(fd_->odom_pos_.cast<double>(),
              fd_->last_odom_receive_time_.toSec(), planner_manager_->gcopter_config_->maxVelMag);
        ROS_INFO_STREAM("[coverage motion] transit observation=(" << observation.goal.transpose()
                        << ") continuation=(" << path_d.back().transpose() << ")");
      } else {
        coverage_passage_ = {};
        coverage_sequence_.clear();coverage_sequence_passed_=0;
      }
    }
    traj_utils::PolyTraj poly_traj_msg;
    planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
    fd_->newest_traj_ = poly_traj_msg;
    traj_utils::PolyTraj poly_yaw_traj_msg;
    planner_manager_->polyYawTraj2ROSMsg(poly_yaw_traj_msg, info->start_time_);
    fd_->newest_yaw_traj_ = poly_yaw_traj_msg;
    return SUCCEED;
  } else {
    // A rejected replacement does not revoke observation progress belonging
    // to the command still being executed. Explicit stop/task changes reset it.
    if (!coverage_motion_enabled || !planner_manager_->hasCommittedTrajectory())
      resetCoverageMotion();
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
  if (fp_->trigger_goal_sets_target_ && expl_manager_ &&
      expl_manager_->targetDirectedModeConfigured()) {
    expl_manager_->setMissionGoal(*msg);
    ROS_INFO_STREAM("[target exploration] received Nav Goal at ["
                    << msg->pose.position.x << ", " << msg->pose.position.y
                    << ", " << msg->pose.position.z
                    << "]; use it as the remote mission destination");
  } else {
    ROS_INFO_STREAM("[exploration trigger] received 2D Nav Goal at ["
                    << msg->pose.position.x << ", " << msg->pose.position.y
                    << ", " << msg->pose.position.z
                    << "]; position is used only as a start trigger");
  }
  acceptManualTrigger("2D Nav Goal");
}

void FastExplorationFSM::missionGoalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  if (!msg || !expl_manager_) {
    return;
  }
  expl_manager_->setMissionGoal(*msg);
  ROS_INFO_STREAM("[target exploration] received dedicated target at ["
                  << msg->pose.position.x << ", " << msg->pose.position.y
                  << ", " << msg->pose.position.z << "]");
  if (!pending_target_task_id_.empty()) {
    const std::string task_id = pending_target_task_id_;
    startExplorationTask(task_id, "legacy target arrived after START");
    return;
  }
  if (fp_->target_goal_start_on_receive_) {
    acceptManualTrigger("dedicated mission target");
  }
}

void FastExplorationFSM::taskRequestCallback(
    const general_planner::ExplorationTaskRequestConstPtr &msg) {
  if (!task_control_enable_ || !msg || !expl_manager_) {
    return;
  }
  if (msg->task_id.empty()) {
    ROS_WARN("[exploration task] ignore atomic request without task id");
    return;
  }
  if (msg->start && msg->task_id == active_task_id_ && pending_target_task_id_.empty() &&
      fd_->trigger_ && (state_ == INIT || state_ == WAIT_TRIGGER || state_ == PLAN_TRAJ ||
                       state_ == EXEC_TRAJ || state_ == REORIENT || state_ == CAUTION)) {
    // A retry is an acknowledgement request, not a new goal/world generation.
    // In particular it must not reset route progress/cooldowns while START's
    // initial status is still in transport.
    publishTaskStatus();
    return;
  }

  const std::string mode =
      msg->mission_mode == general_planner::ExplorationTaskRequest::MODE_TARGET
          ? "target"
          : "coverage";
  if (!expl_manager_->setMissionMode(mode)) {
    ROS_WARN_STREAM("[exploration task] reject atomic request task_id="
                    << msg->task_id << " mode=" << mode);
    return;
  }
  if (msg->has_target) {
    expl_manager_->setMissionGoal(msg->target);
  }
  if (msg->start) {
    startExplorationTask(msg->task_id, "atomic task request");
  }
}

void FastExplorationFSM::acceptManualTrigger(const string &source) {
  startExplorationTask("manual", source);
}

void FastExplorationFSM::waitForMissionTarget(
    const std::string &task_id, const std::string &source) {
  active_task_id_ = task_id;
  resetCoverageMotion();
  coverage_blocked_pending_=false;
  coverage_result_.clear();
  expl_manager_->resetCoverageRecovery();
  if (expl_manager_->coverageMotionEnabled()) {
    std_msgs::String msg; msg.data="{\"result\":\"RUNNING\"}";
    coverage_result_pub_.publish(msg);
  }
  pending_target_task_id_ = task_id;
  completion_pending_ = false;
  target_arrival_verification_pending_ = false;
  target_arrival_correction_count_ = 0;
  target_unreachable_pending_ = false;
  pause_stop_issued_ = false;
  fd_->trigger_ = false;
  fd_->auto_triggered_ = false;
  fd_->static_state_ = true;
  expl_manager_->ed_->global_tour_.clear();
  expl_manager_->ed_->path_next_goal_.clear();
  expl_manager_->ed_->has_goal_lock_ = false;
  expl_manager_->ed_->locked_goal_is_mission_ = false;
  if (state_ != INIT && state_ != WAIT_TRIGGER) {
    transitState(WAIT_TRIGGER, source + ": waiting for mission target");
  }
  ROS_INFO_STREAM("[target exploration] task_id=" << task_id
                  << " is waiting for its mission target (" << source
                  << ")");
  publishTaskStatus();
}

void FastExplorationFSM::taskCommandCallback(
    const std_msgs::StringConstPtr &msg) {
  if (!task_control_enable_ || !msg) {
    return;
  }

  std::istringstream stream(msg->data);
  std::string command;
  stream >> command;
  std::string task_id;
  std::getline(stream, task_id);
  const auto first = task_id.find_first_not_of(" \t");
  task_id = first == std::string::npos ? std::string() : task_id.substr(first);
  std::transform(command.begin(), command.end(), command.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

  if (command == "MODE") {
    if (task_id.empty()) {
      ROS_WARN("[exploration task] ignore MODE without target or coverage");
      return;
    }
    if (state_ == PLAN_TRAJ || state_ == CAUTION || state_ == EXEC_TRAJ ||
        state_ == REORIENT) {
      ROS_WARN_STREAM("[exploration task] ignore MODE while state="
                      << fd_->state_str_[state_]);
      return;
    }
    if (!expl_manager_->setMissionMode(task_id)) {
      ROS_WARN_STREAM("[exploration task] failed to switch mission mode='"
                      << task_id << "'");
    }
    return;
  }

  if (command == "START" || command == "RESUME") {
    if (task_id.empty()) {
      ROS_WARN("[exploration task] ignore START/RESUME without task id");
      return;
    }
    startExplorationTask(task_id, "task command " + command);
    return;
  }

  if (command == "PAUSE" || command == "CANCEL") {
    if (!task_id.empty() && !active_task_id_.empty() &&
        task_id != active_task_id_) {
      ROS_WARN_STREAM("[exploration task] ignore " << command
                      << " for inactive task_id=" << task_id);
      return;
    }
    target_arrival_verification_pending_ = false;
    beginPause("task command " + command, false);
    return;
  }

  ROS_WARN_STREAM("[exploration task] ignore unknown command='" << command
                  << "'");
}

void FastExplorationFSM::startExplorationTask(const std::string &task_id,
                                              const std::string &source) {
  if (task_id.empty()) {
    return;
  }
  if (state_ == LAND || state_ == PAUSING) {
    ROS_WARN_STREAM("[exploration task] cannot start task_id=" << task_id
                    << " while state=" << fd_->state_str_[state_]);
    return;
  }

  const bool already_running = pending_target_task_id_.empty() &&
      active_task_id_ == task_id &&
      (state_ == WAIT_TRIGGER || state_ == PLAN_TRAJ || state_ == CAUTION ||
       state_ == EXEC_TRAJ || state_ == REORIENT);
  if (already_running) {
    return;
  }

  // A completed task must stay PAUSED/SUCCEEDED until the supervisor issues a
  // new task_id. Repeated START with the same id is a common race with
  // periodic mission publishers and must not reopen planning.
  if (state_ == PAUSED && completion_pending_ && active_task_id_ == task_id) {
    ROS_INFO_STREAM("[exploration task] ignore START for completed task_id="
                    << task_id << " source=" << source);
    publishTaskStatus();
    return;
  }

  if (expl_manager_->targetDirectedModeConfigured() &&
      !expl_manager_->targetDirectedModeActive()) {
    waitForMissionTarget(task_id, source);
    return;
  }

  // Another controller may have moved the vehicle since PAUSED. Its old
  // exploration command is no longer an execution seed for the next task.
  if (expl_manager_->coverageMotionEnabled() && state_==PAUSED) {
    planner_manager_->clearReleasedTrajectory();
    coverage_emergency_stop_until_=ros::Time(0);
  }
  active_task_id_ = task_id;
  resetCoverageMotion();
  coverage_blocked_pending_=false;
  coverage_result_.clear();
  expl_manager_->resetCoverageRecovery();
  if (expl_manager_->coverageMotionEnabled()) {
    std_msgs::String msg; msg.data="{\"result\":\"RUNNING\"}";
    coverage_result_pub_.publish(msg);
  }

  pending_target_task_id_.clear();
  completion_pending_ = false;
  task_command_started_ = false;
  last_plan_used_target_route_ = false;
  prepared_target_route_ = {};
  expl_manager_->resetTargetRoute();
  target_last_motion_time_ = ros::WallTime::now();
  target_last_motion_pos_ = fd_->odom_pos_;
  target_arrival_verification_pending_ = false;
  target_arrival_correction_count_ = 0;
  target_unreachable_pending_ = false;
  pause_stop_issued_ = false;
  fd_->trigger_ = true;
  fd_->auto_triggered_ = true;
  fd_->static_state_ = true;
  fd_->next_plan_retry_time_ = ros::Time(0);
  fd_->consecutive_plan_failures_ = 0;
  fd_->stationary_failure_refreshes_ = 0;
  expl_manager_->ed_->global_tour_.clear();
  expl_manager_->ed_->path_next_goal_.clear();
  expl_manager_->ed_->has_goal_lock_ = false;
  expl_manager_->ed_->locked_goal_is_mission_ = false;
  resetFinishGate(source);
  total_time_ = ros::Time::now().toSec();
  global_path_update_timer_.start();

  if (state_ == INIT) {
    ROS_INFO_STREAM("[exploration task] queued task_id=" << task_id
                    << "; waiting for first odometry sample");
    return;
  }

  ROS_INFO_STREAM("[exploration task] start task_id=" << task_id
                  << " source=" << source);
  transitState(PLAN_TRAJ, source);
}

void FastExplorationFSM::beginPause(const std::string &reason,
                                    const bool completed) {
  resetCoverageMotion();
  if (state_ == LAND) {
    return;
  }
  completion_pending_ = completion_pending_ || completed;
  expl_manager_->resetTargetRoute();
  prepared_target_route_ = {};
  pending_target_task_id_.clear();
  fd_->trigger_ = false;
  // A paused exploration task keeps its world map, but must not continue the
  // expensive global topology/path/visualization loop while another runtime
  // mode owns the command path.
  global_path_update_timer_.stop();
  expl_manager_->ed_->global_tour_.clear();
  expl_manager_->ed_->path_next_goal_.clear();
  expl_manager_->ed_->has_goal_lock_ = false;
  expl_manager_->ed_->locked_goal_is_mission_ = false;

  if (state_ == PAUSED || state_ == PAUSING) {
    return;
  }
  pause_stop_issued_ = false;
  transitState(PAUSING, reason);
}

bool FastExplorationFSM::handoverSafe() const {
  if (!fd_->have_odom_ || fd_->last_odom_receive_time_.isZero()) {
    return false;
  }
  const double odom_age =
      (ros::Time::now() - fd_->last_odom_receive_time_).toSec();
  return odom_age <= fp_->max_odom_age_ &&
         fd_->odom_vel_.norm() <= handover_slow_speed_ && trajectoryEnded();
}

void FastExplorationFSM::publishTaskStatus() {
  if (!task_control_enable_ || !task_status_pub_ || active_task_id_.empty()) {
    return;
  }
  std::string state_name;
  if (state_ == PAUSED && completion_pending_) {
    state_name = "SUCCEEDED";
  } else if (!pending_target_task_id_.empty()) {
    state_name = "WAITING_TARGET";
  } else if (state_ == PAUSED && (target_unreachable_pending_ || coverage_blocked_pending_)) {
    state_name = "BLOCKED";
  } else if (state_ == PAUSED) {
    state_name = "PAUSED";
  } else if (state_ == PAUSING || state_ == FINISH) {
    state_name = "PAUSING";
  } else if (state_ == LAND) {
    state_name = "FAILED";
  } else if (state_ == INIT || state_ == WAIT_TRIGGER) {
    state_name = "IDLE";
  } else if (expl_manager_->targetDirectedModeConfigured() && !task_command_started_) {
    state_name = "WAITING_LOCAL_PLAN";
  } else {
    state_name = "RUNNING";
  }
  std_msgs::String msg;
  msg.data = state_name + " " + active_task_id_;
  task_status_pub_.publish(msg);
}

void FastExplorationFSM::odometryCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  if (odom_spinner_) {
    if (!msg) return;
    // Only copy the snapshot on this queue. All FSM, map and planner state
    // stays on the original callback thread.
    std::lock_guard<std::mutex> lock(latest_odom_mutex_);
    latest_odom_msg_=msg;
    latest_odom_receive_time_=ros::Time::now();
    latest_odom_receive_wall_time_=ros::WallTime::now();
    return;
  }
  if (external_sensor_ingress_) {
    refreshRuntimeOdometry();
    return;
  }
  applyOdometry(msg, ros::Time::now());
}

void FastExplorationFSM::refreshRuntimeOdometry() {
  if (!planner_manager_) return;
  if (!external_sensor_ingress_) {
    if (!odom_spinner_) return;
    nav_msgs::OdometryConstPtr msg;
    ros::Time received;
    {
      std::lock_guard<std::mutex> lock(latest_odom_mutex_);
      msg=latest_odom_msg_; received=latest_odom_receive_time_;
    }
    if (msg) applyOdometry(msg,received);
    return;
  }
  const auto manager = planner_manager_->sharedMapManager();
  if (!manager) return;
  const auto state = manager->getRobotState();
  if (!state.rcv) return;
  nav_msgs::OdometryPtr msg(new nav_msgs::Odometry);
  msg->pose.pose.position.x = state.p.x();
  msg->pose.pose.position.y = state.p.y();
  msg->pose.pose.position.z = state.p.z();
  msg->pose.pose.orientation.w = state.q.w();
  msg->pose.pose.orientation.x = state.q.x();
  msg->pose.pose.orientation.y = state.q.y();
  msg->pose.pose.orientation.z = state.q.z();
  msg->twist.twist.linear.x = state.v.x();
  msg->twist.twist.linear.y = state.v.y();
  msg->twist.twist.linear.z = state.v.z();
  ros::Time received;
  received.fromSec(state.rcv_time);
  msg->header.stamp = received;
  applyOdometry(msg, received);
}

void FastExplorationFSM::applyOdometry(
    const nav_msgs::OdometryConstPtr &msg, const ros::Time &received) {
  if (!msg) {
    return;
  }

  // Keep the complete message for latest_odom mode.  Wall time deliberately
  // measures local transport freshness and is independent of /clock or of the
  // timestamp convention used by an external simulator.
  if (!external_sensor_ingress_ && !odom_spinner_) {
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
  fd_->last_odom_receive_time_ = received;
  if (expl_manager_->coverageMotionEnabled()) {
    if (expl_manager_->coverageRouteEnabled() && !coverage_sequence_.empty()) {
      if (!coverage_cloud_received_.isZero() && (received-coverage_cloud_received_).toSec()<=.5) {
        while (coverage_sequence_passed_<coverage_sequence_.size()) {
          const auto &gate=coverage_sequence_[coverage_sequence_passed_];
          if ((fd_->odom_pos_.cast<double>()-gate.goal).norm()>gate.radius ||
              coverage_route::yawDistance(fd_->odom_yaw_,gate.yaw)>gate.yaw_tolerance) break;
          expl_manager_->notifyCoverageRoutePassage(gate.identity);++coverage_sequence_passed_;
        }
        coverage_passage_.passed=coverage_sequence_passed_>0;
      }
    } else coverage_passage_.observe(fd_->odom_pos_.cast<double>(), received.toSec(),
                                    planner_manager_->gcopter_config_->maxVelMag);
  }

  if (!fd_->have_odom_) {
    fd_->first_odom_time_ = fd_->last_odom_receive_time_;
  }
  fd_->have_odom_ = true;

  planner_manager_->local_data_.curr_pos_ = fd_->odom_pos_.cast<double>();
  planner_manager_->local_data_.curr_vel_ = fd_->odom_vel_.cast<double>();
  if (expl_manager_->swarm_coordinator_ &&
      expl_manager_->swarm_coordinator_->enabled()) {
    expl_manager_->swarm_coordinator_->updateRobotState(
        fd_->odom_pos_.cast<double>(), fd_->odom_vel_.cast<double>());
  }
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
  if (external_sensor_ingress_) {
    // GlobalMapRuntime already updated the shared LIO + ROG maps exactly once
    // before dispatching this callback.  Only mark the revision as visible to
    // exploration's local frontier/Bubble consumers.
    planner_manager_->notifyGlobalMapUpdated();
  } else {
    planner_manager_->lidar_map_interface_->updateCloudMapOdometry(msg, odom_);
    planner_manager_->updateRogMap(msg, odom_);
  }
  // The topology timer runs faster than the point-cloud input. Track accepted
  // map updates so the same cloud does not rebuild the skeleton and historical
  // graph two or three times.
  ++topology_map_revision_;
  coverage_cloud_received_=now;
  double collision_time;
  bool safe = planner_manager_->checkTrajCollision(collision_time, expl_manager_->coverageMotionEnabled());
  if (!safe) {
    transitState(PLAN_TRAJ, "safetyCallback: not safe, time:" + to_string(collision_time), true);
    if (collision_time < fp_->replan_time_ + 0.2)
      stopTraj("cloud-map collision update");
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
  // Copy a rate-limited raw-ROG delta and the just-updated frontier snapshot.
  // The coverage graph is built on its own worker; this sensor callback never
  // waits for global coverage optimization.
  expl_manager_->updateCoverageGuidance(fd_->odom_pos_.cast<double>());
  ros::Time t4 = ros::Time::now();

  ROS_INFO_STREAM_THROTTLE(1.0, "cloud odom callback cost: " << "map update:" << (t2 - t1).toSec() * 1000 << "ms  "
                                                             << "update frontier clusters: " << (t4 - t3).toSec() * 1000 << "ms  "
                                                             << "total: " << (t4 - t1).toSec() * 1000 << "ms" << endl);
}

void FastExplorationFSM::ingestCloudOdom(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    const nav_msgs::OdometryConstPtr &odom) {
  CloudOdomCallback(msg, odom);
}

void FastExplorationFSM::ingestOdometry(
    const nav_msgs::OdometryConstPtr &msg) {
  odometryCallback(msg);
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call, bool red) {
  int pre_s = int(state_);
  if (new_state == CAUTION && state_ != CAUTION) {
    fd_->caution_last_stop_request_time_ = ros::Time(0);
    fd_->caution_last_recovery_attempt_time_ = ros::Time(0);
  }
  state_ = new_state;
  if (!red) {
    cout << "\033[32m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  } else {
    cout << "\033[31m[" + pos_call + "]\033[0m: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
  }
}

void FastExplorationFSM::stopTraj(const string &reason) {
  refreshRuntimeOdometry();
  const ros::Time now = ros::Time::now();
  if (expl_manager_->coverageMotionEnabled() && !coverage_emergency_stop_until_.isZero())
    return;  // Do not republish the invalidated command or postpone its stop.
  const double speed = fd_->odom_vel_.norm();
  const double odom_age =
      fd_->last_odom_receive_time_.isZero()
          ? std::numeric_limits<double>::infinity()
          : (now - fd_->last_odom_receive_time_).toSec();
  const bool odom_fresh = odom_age <= fp_->max_odom_age_;

  // At near-zero speed a new braking polynomial has no safety benefit. It can
  // instead overwrite a valid replanned trajectory a few milliseconds after
  // publication, producing the repeated stop/no-trajectory chatter observed
  // in narrow rooms. Truncate the unsafe command and hold the current pose.
  if (!expl_manager_->coverageMotionEnabled() && odom_fresh && speed <= fp_->controlled_stop_min_speed_) {
    const int current_traj_id = planner_manager_->local_data_.traj_id_;
    const bool hold_retry_due =
        current_traj_id != last_near_stationary_hold_traj_id_ ||
        last_near_stationary_hold_time_.isZero() ||
        (now - last_near_stationary_hold_time_).toSec() >=
            fp_->stationary_hold_retry_interval_;
    if (hold_retry_due) {
      replan_pub_.publish(std_msgs::Empty());
      last_near_stationary_hold_time_ = now;
      last_near_stationary_hold_traj_id_ = current_traj_id;
      const ros::Time start_time = planner_manager_->local_data_.start_time_;
      const double elapsed =
          std::max(0.0, (now - start_time).toSec());
      planner_manager_->local_data_.duration_ =
          std::min(planner_manager_->local_data_.duration_, elapsed);
      fd_->static_state_ = true;
      ROS_WARN_STREAM(
          "[controlled stop] near-stationary hold without braking trajectory: "
          "reason="
          << reason << " speed=" << speed
          << " traj_id=" << current_traj_id
          << " threshold=" << fp_->controlled_stop_min_speed_
          << " odom_age=" << odom_age);
    } else {
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[controlled stop] suppress duplicate near-stationary hold: "
                   "reason="
                   << reason << " speed=" << speed
                   << " traj_id=" << current_traj_id);
    }
    return;
  }

  // A replan notification only shortens the polynomial in traj_server.  Its
  // mathematical endpoint can still carry several m/s of velocity, after which
  // traj_server switches directly to position HOLD.  Commit and publish an
  // actual dynamically feasible braking polynomial first.
  if (planner_manager_->planControlledStopTrajectory(expl_manager_->coverageMotionEnabled())) {
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
      task_command_started_ = true;
      publishTaskStatus();
      fd_->static_state_ = false;
      ROS_WARN_STREAM_THROTTLE(
          0.5, "[controlled stop] published braking trajectory id="
                   << info->traj_id_ << " duration=" << info->duration_
                   << " reason=" << reason << " odom_speed=" << speed);
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
  if (expl_manager_->coverageMotionEnabled())
    coverage_emergency_stop_until_=ros::Time::now()+ros::Duration(
        fp_->replan_time_+0.1+std::max(0.05,planner_manager_->gcopter_config_->controlLatency));
  ros::Time time_now = ros::Time::now();
  ros::Time start_time = planner_manager_->local_data_.start_time_;
  double curr_dur = planner_manager_->local_data_.duration_;
  planner_manager_->local_data_.duration_ = min(curr_dur, (time_now - start_time).toSec() + fp_->replan_time_);
  if (planner_manager_->local_data_.duration_ <= (time_now - start_time).toSec())
    fd_->static_state_ = true;
}
