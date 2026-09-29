#include <general_core/exploration/highspeed/fast_exploration_fsm.h>
#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/epicon_frontend.h>

bool FastExplorationFSM::usingEpicon() const {
  return epicon_frontend_ && !expl_manager_->targetDirectedModeConfigured();
}

void FastExplorationFSM::resetEpiconExecution() {
  node_.param("epicon/execution/max_cloud_age",epicon_finish_cloud_age_,1.0);
  node_.param("epicon/execution/audit_period",epicon_audit_period_,1.0);
  node_.param("epicon/execution/goal_cooldown",epicon_goal_cooldown_,15.0);
  node_.param("epicon/execution/goal_exclusion_radius",epicon_goal_exclusion_radius_,0.8);
  node_.param("epicon/execution/stall_reselect",epicon_stall_reselect_,8.0);
  node_.param("epicon/execution/stall_timeout",epicon_stall_timeout_,40.0);
  node_.param("epicon/execution/goal_failure_limit",epicon_goal_failure_limit_,3);
  epicon_finish_cloud_age_=std::clamp(epicon_finish_cloud_age_,0.1,2.0);
  epicon_audit_period_=std::max(0.2,epicon_audit_period_);
  epicon_stall_reselect_=std::max(2.0,epicon_stall_reselect_);
  epicon_stall_timeout_=std::max(epicon_stall_reselect_*2,epicon_stall_timeout_);
  epicon_goal_failure_limit_=std::max(1,epicon_goal_failure_limit_);
  epicon_finish_.reset(); epicon_recheck_pending_=false;
  epicon_last_audit_=epicon_last_deferral_=ros::WallTime();
  epicon_last_motion_=ros::WallTime::now(); epicon_motion_anchor_=fd_->odom_pos_;
  ROS_INFO_STREAM("[EPICON] execution policy: audit_period=" << epicon_audit_period_
      << " goal_failure_limit=" << epicon_goal_failure_limit_
      << " stall_reselect=" << epicon_stall_reselect_ << " stall_timeout=" << epicon_stall_timeout_);
}

void FastExplorationFSM::finishEpiconTask(bool completed, const std::string &reason) {
  coverage_blocked_pending_=!completed;
  completion_pending_=completed;
  coverage_result_=completed ? "COMPLETE" : "BLOCKED";
  std_msgs::String msg;
  msg.data="{\"backend\":\"epicon_pointcloud\",\"result\":\""+coverage_result_+
      "\",\"reason\":\""+reason+"\",\"verified_frames\":"+std::to_string(epicon_finish_.count())+"}";
  coverage_result_pub_.publish(msg);
  ROS_WARN_STREAM("[EPICON] " << coverage_result_ << ": " << reason);
  if(task_control_enable_ || !completed) beginPause(reason,completed);
  else transitState(FINISH,reason);
}

void FastExplorationFSM::updateEpiconGlobalPath() {
  if (!fd_->have_odom_ || !(state_==WAIT_TRIGGER || state_==PLAN_TRAJ || state_==EXEC_TRAJ ||
                          state_==CAUTION || (state_==FINISH && !task_control_enable_))) return;
  if (!fd_->last_odom_receive_time_.isZero() &&
      (ros::Time::now()-fd_->last_odom_receive_time_).toSec()>fp_->max_odom_age_) return;
  epicon_frontend_->setOdometry(fd_->odom_pos_,fd_->odom_vel_,fd_->odom_yaw_);
  auto &info=planner_manager_->local_data_;
  double collision_time=0.0;
  const bool safe=planner_manager_->checkTrajCollision(collision_time);
  const double elapsed=(ros::Time::now()-info.start_time_).toSec();
  const bool recent=planner_manager_->hasCommittedTrajectory() && safe &&
      elapsed<fp_->replan_time_after_traj_start_ &&
      planner_manager_->committedTrajectoryRemainingTime()>fp_->replan_time_before_traj_end_;
  const bool select=fd_->trigger_ && state_!=WAIT_TRIGGER && (!recent || epicon_recheck_pending_);
  const auto wall_now=ros::WallTime::now();
  const bool audit=select && epicon_recheck_pending_ &&
      (epicon_last_audit_.isZero() || (wall_now-epicon_last_audit_).toSec()>=epicon_audit_period_);
  if(audit) epicon_last_audit_=wall_now;
  const auto result=epicon_frontend_->update(select,audit);
  if (!select) return;
  refreshRuntimeOdometry();
  const auto now=ros::Time::now();
  const double cloud_age=(now-epicon_frontend_->cloudStamp()).toSec();
  const bool fresh=!epicon_frontend_->cloudStamp().isZero() &&
      cloud_age>=0.0 && cloud_age<=epicon_finish_cloud_age_ &&
      (now-fd_->last_odom_receive_time_).toSec()<=fp_->max_odom_age_;
  const bool settled=fd_->odom_vel_.norm()<=handover_slow_speed_ &&
      (!planner_manager_->hasCommittedTrajectory() ||
       planner_manager_->committedTrajectoryRemainingTime()<=0.03);
  if (result==fast_planner::EpiconFrontend::Result::NO_FRONTIER) {
    epicon_recheck_pending_=true;
    if(!safe) {
      epicon_finish_.reset();
      stopTraj("EPICON: unsafe remaining trajectory during finish verification");
      return;
    }
    // Keep the safe committed motion and the global timer alive. In particular,
    // do not brake a newly committed long trajectory on one empty selection.
    if(!settled || !fresh) epicon_finish_.reset();
    if(audit && epicon_finish_.observe(now.toSec(),settled,fresh,true,
        epicon_frontend_->cloudRevision(),epicon_frontend_->cloudStamp().toSec(),
        fp_->finish_no_frontier_min_count_,fp_->finish_no_frontier_min_duration_)) {
      if(state_!=FINISH) finishEpiconTask(true,"fresh point-cloud audits found no actionable frontier after settling");
      return;
    }
    ROS_INFO_STREAM_THROTTLE(1.0,"[EPICON] finish pending: settled=" << settled
        << " fresh=" << fresh << " audited_frames=" << epicon_finish_.count()
        << " remaining=" << planner_manager_->committedTrajectoryRemainingTime());
  } else if(result==fast_planner::EpiconFrontend::Result::SUCCEED) {
    if(state_==FINISH) {
      completion_pending_=false; coverage_result_.clear();
      resetEpiconExecution();
    }
    epicon_finish_.reset(); epicon_recheck_pending_=false;
    expl_manager_->ed_->global_tour_=epicon_frontend_->tour(); // visualization/task compatibility only
    if(state_!=CAUTION && now>=fd_->next_plan_retry_time_)
      transitState(PLAN_TRAJ,"EPICON: global tour ready");
  } else {
    epicon_finish_.reset(); epicon_recheck_pending_=true;
    if(result==fast_planner::EpiconFrontend::Result::DISCONNECTED &&
       (!safe || planner_manager_->committedTrajectoryRemainingTime()<=0.03))
      transitState(CAUTION,"EPICON: odometry disconnected",true);
  }
}

int FastExplorationFSM::callEpiconPlanner() {
  epicon_frontend_->setOdometry(fd_->odom_pos_,fd_->odom_vel_,fd_->odom_yaw_);
  std::vector<Eigen::Vector3f> path;
  const auto result=epicon_frontend_->pathToGoal(path);
  if(result==fast_planner::EpiconFrontend::Result::DISCONNECTED) return START_FAIL;
  if(result==fast_planner::EpiconFrontend::Result::NO_FRONTIER) return NO_FRONTIER;
  if(result!=fast_planner::EpiconFrontend::Result::SUCCEED) return FAIL;
  planner_manager_->local_data_.end_yaw_=epicon_frontend_->goalYaw();
  expl_manager_->ed_->path_next_goal_=path;
  // No ROG-derived coverage goals, observation constraints, path extensions,
  // or legacy coverage costs are inserted before our trajectory backend.
  if(!planner_manager_->planExploreTraj(path,fd_->static_state_,false,false,{},{},{},true)) return FAIL;
  planner_manager_->polyTraj2ROSMsg(fd_->newest_traj_,planner_manager_->local_data_.start_time_);
  planner_manager_->polyYawTraj2ROSMsg(fd_->newest_yaw_traj_,planner_manager_->local_data_.start_time_);
  return SUCCEED;
}

bool FastExplorationFSM::epiconFSMCallback() {
  // Pause/cancel/handover and landing remain framework states. All autonomous
  // exploration transitions below replace the previous coverage state machine.
  if(state_==PAUSING || state_==PAUSED || state_==LAND) return false;
  const auto now=ros::Time::now();
  if(fd_->have_odom_) epicon_frontend_->setOdometry(fd_->odom_pos_,fd_->odom_vel_,fd_->odom_yaw_);
  if(fd_->trigger_ && state_!=FINISH) {
    const auto wall_now=ros::WallTime::now();
    if(epicon_last_motion_.isZero() || (fd_->odom_pos_-epicon_motion_anchor_).norm()>=0.35f) {
      epicon_motion_anchor_=fd_->odom_pos_; epicon_last_motion_=wall_now;
    }
    const double stationary=(wall_now-epicon_last_motion_).toSec();
    if(stationary>=epicon_stall_timeout_) {
      finishEpiconTask(false,"no measured motion after bounded frontier and trajectory recovery");
      return true;
    }
    if(stationary>=epicon_stall_reselect_ && epicon_frontend_->hasSelectedGoal() &&
       (epicon_last_deferral_.isZero() || (wall_now-epicon_last_deferral_).toSec()>=epicon_stall_reselect_)) {
      epicon_frontend_->deferCurrentGoal(epicon_goal_cooldown_,epicon_goal_exclusion_radius_);
      epicon_last_deferral_=wall_now; epicon_recheck_pending_=true;
      epicon_finish_.reset(); fd_->consecutive_plan_failures_=0;
      transitState(CAUTION,"EPICON: no measured progress; relocate before retry",true);
    }
  }
  if(fd_->trigger_ && (fd_->last_odom_receive_time_.isZero() ||
      (now-fd_->last_odom_receive_time_).toSec()>fp_->max_odom_age_)) return true;
  switch(state_) {
  case INIT:
    if(fd_->have_odom_) transitState(WAIT_TRIGGER,"EPICON: odometry ready");
    break;
  case WAIT_TRIGGER:
    if(fd_->trigger_) { global_path_update_timer_.start(); transitState(PLAN_TRAJ,"EPICON: trigger"); }
    else if(fp_->auto_trigger_enable_ && !fd_->auto_triggered_ && fd_->have_odom_ && epicon_frontend_->ready() &&
            (now-fd_->first_odom_time_).toSec()>=fp_->auto_trigger_delay_)
      startExplorationTask("auto","EPICON: auto trigger");
    break;
  case PLAN_TRAJ: {
    if(!fd_->trigger_ || !epicon_frontend_->hasCloud() || now<fd_->next_plan_retry_time_) break;
    if(!epicon_frontend_->ready() || !epicon_frontend_->hasSelectedGoal()) break;
    exec_timer_.stop();
    const auto started=ros::WallTime::now();
    const int result=callEpiconPlanner();
    exec_timer_.start();
    ROS_INFO("[EPICON] trajectory backend result=%d ms=%.1f",result,(ros::WallTime::now()-started).toSec()*1000.0);
    if(result==SUCCEED) {
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_=false; task_command_started_=true;
      fd_->consecutive_plan_failures_=0;
      transitState(EXEC_TRAJ,"EPICON: General trajectory committed");
    } else {
      ++fd_->consecutive_plan_failures_;
      const bool relocate=fd_->consecutive_plan_failures_>=epicon_goal_failure_limit_ &&
          fd_->odom_vel_.norm()<0.20;
      if(fd_->consecutive_plan_failures_>=epicon_goal_failure_limit_) {
        epicon_frontend_->deferCurrentGoal(epicon_goal_cooldown_,epicon_goal_exclusion_radius_);
        epicon_last_deferral_=ros::WallTime::now(); epicon_recheck_pending_=true;
        epicon_finish_.reset(); fd_->consecutive_plan_failures_=0;
      }
      fd_->next_plan_retry_time_=ros::Time::now()+ros::Duration(fp_->plan_failure_retry_delay_);
      double collision_time=0.0;
      const bool safe=planner_manager_->checkTrajCollision(collision_time);
      // General's committed safe prefix stays executable during a failed
      // optimization. This is the backend's ownership contract.
      if(!safe || planner_manager_->committedTrajectoryRemainingTime()<=0.05)
        stopTraj("EPICON: local planning failed without safe remaining trajectory");
      if(result==START_FAIL || relocate)
        transitState(CAUTION,"EPICON: repeated local failure; recover the start region",true);
      else transitState(EXEC_TRAJ,"EPICON: local retry pending",true);
    }
    break;
  }
  case EXEC_TRAJ: {
    double collision_time=0.0;
    const bool safe=planner_manager_->checkTrajCollision(collision_time);
    if(!safe) {
      if(collision_time<fp_->replan_time_+0.2) stopTraj("EPICON: imminent collision");
      transitState(PLAN_TRAJ,"EPICON: collision replan",true);
    } else if(!planner_manager_->checkTrajVelocity() ||
              planner_manager_->committedTrajectoryRemainingTime()<=fp_->replan_time_before_traj_end_) {
      if(now>=fd_->next_plan_retry_time_) transitState(PLAN_TRAJ,"EPICON: trajectory replan");
    }
    break;
  }
  case CAUTION: {
    if(fd_->caution_last_stop_request_time_.isZero() ||
        (now-fd_->caution_last_stop_request_time_).toSec()>=fp_->caution_stop_retry_interval_) {
      stopTraj("EPICON: caution"); fd_->caution_last_stop_request_time_=now;
    }
    if(!fd_->caution_last_recovery_attempt_time_.isZero() &&
       (now-fd_->caution_last_recovery_attempt_time_).toSec()<fp_->caution_recovery_retry_interval_) break;
    fd_->caution_last_recovery_attempt_time_=now;
    if(planner_manager_->flyToSafeRegion(fd_->static_state_,true)) {
      planner_manager_->polyTraj2ROSMsg(fd_->newest_traj_,planner_manager_->local_data_.start_time_);
      planner_manager_->polyYawTraj2ROSMsg(fd_->newest_yaw_traj_,planner_manager_->local_data_.start_time_);
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_); poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_=false;
      // Execute the certified relocation instead of braking it again on the
      // next CAUTION stop-retry tick.
      transitState(EXEC_TRAJ,"EPICON: execute topology reconnection");
      break;
    }
    if(epicon_frontend_->ready() && planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_)>
        planner_manager_->gcopter_config_->dilateRadiusSoft) transitState(PLAN_TRAJ,"EPICON: safe region");
    break;
  }
  case FINISH:
    // Unmanaged EPICON FINISH remains reversible through the global timer.
    break;
  case REORIENT:
    transitState(PLAN_TRAJ,"EPICON: replan from backend stop");
    break;
  default: break;
  }
  return true;
}
