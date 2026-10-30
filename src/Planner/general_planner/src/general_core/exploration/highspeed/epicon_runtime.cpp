#include <general_core/exploration/highspeed/fast_exploration_fsm.h>
#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/epicon_frontend.h>

bool FastExplorationFSM::usingEpicon() const {
  return epicon_frontend_ && !expl_manager_->targetDirectedModeConfigured();
}
bool FastExplorationFSM::pointcloudExplorationActive() const {
  return usingEpicon() && fd_->trigger_ &&
      (state_==PLAN_TRAJ || state_==EXEC_TRAJ || state_==CAUTION);
}

void FastExplorationFSM::resetEpiconExecution() {
  ROS_INFO("[exploration] native EPICON execution policy; General MINCO backend");
}

void FastExplorationFSM::finishEpiconTask(bool completed, const std::string &reason) {
  coverage_blocked_pending_=!completed;
  completion_pending_=completed;
  coverage_result_=completed ? "COMPLETE" : "BLOCKED";
  std_msgs::String msg;
  msg.data="{\"backend\":\"epicon_pointcloud\",\"result\":\""+coverage_result_+
      "\",\"reason\":\""+reason+"\",\"completion_scope\":\"native_frontier_selection\"}";
  coverage_result_pub_.publish(msg);
  if(task_control_enable_ || !completed) beginPause(reason,completed);
  else transitState(FINISH,reason);
}

void FastExplorationFSM::updateEpiconGlobalPath() {
  if (!fd_->have_odom_ || !(state_==WAIT_TRIGGER || state_==PLAN_TRAJ || state_==EXEC_TRAJ ||
                          (state_==FINISH && !task_control_enable_))) return;
  epicon_frontend_->setOdometry(fd_->odom_pos_,fd_->odom_vel_,fd_->odom_yaw_);
  double collision_time=0.0;
  const bool safe=planner_manager_->checkTrajCollision(collision_time);
  const double elapsed=(ros::Time::now()-planner_manager_->local_data_.start_time_).toSec();
  const bool recent=planner_manager_->hasCommittedTrajectory() && safe &&
      elapsed<fp_->replan_time_after_traj_start_ &&
      planner_manager_->committedTrajectoryRemainingTime()>fp_->replan_time_before_traj_end_;
  const bool select=!recent;
  const auto result=epicon_frontend_->update(select);
  if(result==fast_planner::EpiconFrontend::Result::DISCONNECTED ||
     result==fast_planner::EpiconFrontend::Result::OUTSIDE_BOX) {
    if(!planner_manager_->hasCommittedTrajectory() || !safe)
      transitState(CAUTION,"exploration: odometry node disconnected",true);
    return;
  }
  if(!select || state_==WAIT_TRIGGER || !fd_->trigger_) return;
  if(result==fast_planner::EpiconFrontend::Result::NO_FRONTIER) {
    if(state_!=FINISH) finishEpiconTask(true,"native EPICON: no frontier");
  } else if(result==fast_planner::EpiconFrontend::Result::SUCCEED) {
    expl_manager_->ed_->global_tour_=epicon_frontend_->tour();
    transitState(PLAN_TRAJ,"exploration: global tour ready");
  }
}

int FastExplorationFSM::callEpiconPlanner() {
  epicon_frontend_->setOdometry(fd_->odom_pos_,fd_->odom_vel_,fd_->odom_yaw_);
  if (!epicon_frontend_->ready()) return START_FAIL;
  if (!epicon_frontend_->hasSelectedGoal()) return NO_FRONTIER;
  std::vector<Eigen::Vector3f> path;
  const auto result=epicon_frontend_->pathToGoal(path);
  if(result==fast_planner::EpiconFrontend::Result::DISCONNECTED) return START_FAIL;
  if(result!=fast_planner::EpiconFrontend::Result::SUCCEED) return FAIL;
  planner_manager_->local_data_.end_yaw_=epicon_frontend_->goalYaw();
  expl_manager_->ed_->path_next_goal_=path;
  if(!planner_manager_->planEpiconTraj(path,fd_->static_state_)) return FAIL;
  planner_manager_->polyTraj2ROSMsg(fd_->newest_traj_,planner_manager_->local_data_.start_time_);
  planner_manager_->polyYawTraj2ROSMsg(fd_->newest_yaw_traj_,planner_manager_->local_data_.start_time_);
  return SUCCEED;
}

bool FastExplorationFSM::epiconFSMCallback() {
  // Task cancellation and handover remain outside autonomous exploration.
  if(state_==PAUSING || state_==PAUSED || state_==LAND) return false;
  const auto now=ros::Time::now();
  if(fd_->have_odom_) epicon_frontend_->setOdometry(fd_->odom_pos_,fd_->odom_vel_,fd_->odom_yaw_);
  if(fd_->trigger_ && !epicon_frontend_->hasCloud()) return true;
  if(fd_->trigger_ && (fd_->last_odom_receive_time_.isZero() ||
      (now-fd_->last_odom_receive_time_).toSec()>fp_->max_odom_age_)) return true;
  switch(state_) {
  case INIT:
    if(fd_->have_odom_) transitState(WAIT_TRIGGER,"exploration: odometry ready");
    break;
  case WAIT_TRIGGER:
    if(fd_->trigger_) { global_path_update_timer_.start(); transitState(PLAN_TRAJ,"exploration: trigger"); }
    else if(fp_->auto_trigger_enable_ && !fd_->auto_triggered_ && fd_->have_odom_ && epicon_frontend_->ready() &&
            (now-fd_->first_odom_time_).toSec()>=fp_->auto_trigger_delay_)
      startExplorationTask("auto","exploration: auto trigger");
    break;
  case PLAN_TRAJ: {
    if(!fd_->trigger_ || !epicon_frontend_->ready()) break;
    // Task reset clears the old tour. Select before interpreting an empty tour
    // as NO_FRONTIER; the 100 Hz FSM can precede the global timer after START.
    if(!epicon_frontend_->hasSelectedGoal()) {
      updateEpiconGlobalPath();
      if(state_!=PLAN_TRAJ || !epicon_frontend_->hasSelectedGoal()) break;
    }
    const auto &tour=epicon_frontend_->tour();
    if(tour.size()==2 && (tour[1]-fd_->odom_pos_).norm()<.1f) {
      finishEpiconTask(true,"native EPICON: final tour goal reached");
      break;
    }
    exec_timer_.stop();
    const auto started=ros::WallTime::now();
    const int result=callEpiconPlanner();
    exec_timer_.start();
    ROS_INFO("[exploration] trajectory backend result=%d ms=%.1f",result,(ros::WallTime::now()-started).toSec()*1000.0);
    if(result==SUCCEED) {
      publishCommittedTaskTrajectory();
      fd_->static_state_=false; task_command_started_=true;
      transitState(EXEC_TRAJ,"exploration: General trajectory published");
    } else if(result==NO_FRONTIER) {
      finishEpiconTask(true,"native EPICON: no frontier");
    } else if(result==START_FAIL) {
      transitState(CAUTION,"exploration: start failed",true);
    } else {
      stopTraj("exploration: local planning failed");
      transitState(PLAN_TRAJ,"exploration: local retry",true);
    }
    break;
  }
  case EXEC_TRAJ: {
    double collision_time=0.0;
    if(!planner_manager_->checkTrajCollision(collision_time)) {
      if(collision_time<fp_->replan_time_+0.2) stopTraj("exploration: imminent collision");
      transitState(PLAN_TRAJ,"exploration: collision replan",true);
    } else if(!planner_manager_->checkTrajVelocity()) {
      transitState(PLAN_TRAJ,"exploration: velocity replan",true);
    }
    break;
  }
  case CAUTION: {
    stopTraj("exploration: caution");
    exec_timer_.stop();
    if(planner_manager_->flyToSafeRegion(fd_->static_state_,false)) {
      planner_manager_->polyTraj2ROSMsg(fd_->newest_traj_,planner_manager_->local_data_.start_time_);
      planner_manager_->polyYawTraj2ROSMsg(fd_->newest_yaw_traj_,planner_manager_->local_data_.start_time_);
      publishCommittedTaskTrajectory();
      task_command_started_=true; fd_->static_state_=false;
      ros::Duration(.2).sleep();
    }
    exec_timer_.start();
    if(epicon_frontend_->clearance(fd_->odom_pos_.cast<double>())>
       planner_manager_->gcopter_config_->dilateRadiusSoft)
      transitState(PLAN_TRAJ,"exploration: safe region");
    break;
  }
  case FINISH: {
    double collision_time=0.0;
    if(!planner_manager_->checkTrajCollision(collision_time)) stopTraj("exploration: finished collision");
    break;
  }
  case REORIENT:
    transitState(PLAN_TRAJ,"exploration: replan");
    break;
  default: break;
  }
  return true;
}
