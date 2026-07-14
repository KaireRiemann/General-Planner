/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2024-02-29 16:54:46
 * @LastEditTime: 2024-03-11 13:22:44
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/fast_exploration_fsm.h>
#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <general_core/exploration/highspeed/planner_manager.h>
#include <algorithm>
#include <limits>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <traj_utils/planning_visualization.h>
using Eigen::Vector3d;
using Eigen::Vector4d;
bool debug_planner;
typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;

FastExplorationFSM::~FastExplorationFSM() {
  exec_timer_.stop();
  global_path_update_timer_.stop();

  trigger_sub_.shutdown();
  nav_goal_trigger_sub_.shutdown();
  map_update_sub_.shutdown();
  battary_sub_.shutdown();
  raw_odom_sub_.shutdown();
  latest_cloud_sub_.shutdown();

  if (cloud_sub_)
    cloud_sub_->unsubscribe();
  if (odom_sub_)
    odom_sub_->unsubscribe();

  sync_cloud_odom_.reset();
  cloud_sub_.reset();
  odom_sub_.reset();
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e) {
  pubState();
  switch (state_) {
  case INIT: {
    if (!fd_->have_odom_) {
      ROS_WARN_THROTTLE(1.0, "no odom.");
      return;
    }
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    // A 2D Nav Goal may arrive before the first odometry sample. In that case
    // acceptManualTrigger() latches fd_->trigger_ while the FSM is still INIT,
    // and exploration starts as soon as INIT advances to WAIT_TRIGGER.
    if (fd_->trigger_) {
      total_time_ = ros::Time::now().toSec();
      resetFinishGate("queued manual trigger");
      transitState(PLAN_TRAJ, "queued manual trigger");
      break;
    }
    if (fp_->auto_trigger_enable_ && fd_->have_odom_ &&
        !fd_->auto_triggered_) {
      if (fd_->first_odom_time_.isZero()) {
        fd_->first_odom_time_ = ros::Time::now();
      }
      const double wait_time =
          (ros::Time::now() - fd_->first_odom_time_).toSec();
      const bool topo_ready =
          planner_manager_->topo_graph_->odom_node_ &&
          !planner_manager_->topo_graph_->odom_node_->neighbors_.empty();
      if (wait_time >= fp_->auto_trigger_delay_ && topo_ready) {
        fd_->auto_triggered_ = true;
        fd_->trigger_ = true;
        total_time_ = ros::Time::now().toSec();
        resetFinishGate("auto trigger");
        transitState(PLAN_TRAJ, "auto trigger");
        break;
      }
      ROS_INFO_THROTTLE(1.0,
                        "auto trigger armed, waiting %.2fs/topology ready=%d",
                        std::max(0.0, fp_->auto_trigger_delay_ - wait_time),
                        topo_ready);
    }
    ROS_WARN_THROTTLE(1.0, "wait for trigger.");
    break;
  }

  case FINISH: {
    // stopTraj();
    double collision_time = 0.0;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      stopTraj();
    }
    ROS_WARN_THROTTLE(1.0, "Finished.");
    break;
  }

  case PLAN_TRAJ: {
    if (!fd_->trigger_)
      return;
    const ros::Time plan_now = ros::Time::now();
    if (!fd_->next_plan_retry_time_.isZero() &&
        plan_now < fd_->next_plan_retry_time_) {
      return;
    }
    if (!planner_manager_->topo_graph_->odom_node_ ||
        planner_manager_->topo_graph_->odom_node_->neighbors_.empty())
      return;
    if (expl_manager_->ed_->global_tour_.size() < 2) {
      const ros::Time now = ros::Time::now();
      const double min_update_interval =
          std::max(0.02, fp_->global_path_update_min_interval_);
      if (last_plan_traj_global_update_time_.isZero() ||
          (now - last_plan_traj_global_update_time_).toSec() >=
              min_update_interval) {
        last_plan_traj_global_update_time_ = now;
        updateTopoAndGlobalPath();
      }
      if (state_ != PLAN_TRAJ) {
        return;
      }
      if (expl_manager_->ed_->global_tour_.size() < 2) {
        return;
      }
    }
    // 要报min-step的case
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double time_to_end = info->duration_ - t_cur;
    (void)time_to_end;
    if (handleGoalReached()) {
      return;
    }
    ros::Time tplan = ros::Time::now();
    exec_timer_.stop();
    int res = callExplorationPlanner();
    exec_timer_.start();
    ROS_INFO("\033[31m call planner \033[0m: %.3f",
             (ros::Time::now() - tplan).toSec() * 1000.0);

    if (res == SUCCEED) {
      fd_->consecutive_plan_failures_ = 0;
      fd_->next_plan_retry_time_ = ros::Time(0);
      resetFinishGate("PLAN_TRAJ succeed");
      poly_yaw_traj_pub_.publish(fd_->newest_yaw_traj_);
      poly_traj_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      if (fd_->use_bubble_a_star_) {
        transitState(EXEC_TRAJ,
                     "ParallelBubbleAstar plan success: new traj pub");
      } else {
        transitState(EXEC_TRAJ, "plan success: new traj pub");
      }
      fd_->use_bubble_a_star_ = false;
      fd_->half_resolution = false;

    } else if (res == NO_FRONTIER) {
      handleNoFrontierResult("PLAN_TRAJ: no frontier");
    } else if (res == FAIL) {
      ++fd_->consecutive_plan_failures_;
      const double retry_delay = fp_->plan_failure_retry_delay_ *
          std::min(3, fd_->consecutive_plan_failures_);
      fd_->next_plan_retry_time_ = ros::Time::now() +
          ros::Duration(retry_delay);
      if (fp_->controlled_reorientation_enable_ &&
          fd_->reorientation_required_) {
        fd_->reorientation_start_time_ = ros::Time::now();
        fd_->reorientation_stop_requested_ = false;
        fd_->reorientation_last_stop_request_time_ = ros::Time(0);
        transitState(REORIENT, "PLAN_TRAJ: brake before large turn", true);
        break;
      }
      double collision_time = 0.0;
      const bool safe = planner_manager_->checkTrajCollision(collision_time);
      const double remaining = planner_manager_->committedTrajectoryRemainingTime();
      const bool have_safe_committed =
          safe && planner_manager_->hasCommittedTrajectory() && remaining > 0.08;
      const bool may_refresh_goal =
          !have_safe_committed &&
          fd_->consecutive_plan_failures_ >= fp_->plan_failure_refresh_count_ &&
          fd_->odom_vel_.norm() <= fp_->reorient_exit_speed_;
      if (may_refresh_goal) {
        expl_manager_->ed_->has_goal_lock_ = false;
        expl_manager_->ed_->locked_goal_cluster_id_ = -1;
        expl_manager_->ed_->global_tour_.clear();
        expl_manager_->ed_->path_next_goal_.clear();
        expl_manager_->updateGoalNode();
        fd_->consecutive_plan_failures_ = 0;
        ROS_WARN_STREAM("[plan recovery] refresh goal after repeated failures; "
                        << "retry_delay=" << retry_delay
                        << " safe_committed=" << have_safe_committed);
      }
      if (have_safe_committed) {
        transitState(EXEC_TRAJ,
                     planner_manager_->hasCommittedBackup()
                         ? "PLAN_TRAJ: plan failed, keep committed backup"
                         : "PLAN_TRAJ: plan failed, keep safe terminal-stop trajectory",
                     true);
      } else {
        if (fd_->odom_vel_.norm() > fp_->reorient_exit_speed_) {
          stopTraj();
        }
        transitState(PLAN_TRAJ, "PLAN_TRAJ: plan failed", true);
      }

    } else if (res == START_FAIL) {
      transitState(CAUTION, "PLAN_TRAJ: start failed", true);
    } else {
      cout << "330?" << endl;
    }
    break;
  }

  case EXEC_TRAJ: {
    // collision check
    double collision_time;
    bool safe = planner_manager_->checkTrajCollision(collision_time);
    if (!safe) {
      transitState(
          PLAN_TRAJ,
          "safetyCallback: not safe, time:" + to_string(collision_time), true);
      if (collision_time < fp_->replan_time_ + 0.2)
        stopTraj();
    } else if (!planner_manager_->checkTrajVelocity()) {
      transitState(PLAN_TRAJ, "velocity too fast", true);
    } else if (planner_manager_->committedTrajectoryRemainingTime() <=
               fp_->replan_time_before_traj_end_) {
      transitState(PLAN_TRAJ, "EXEC_TRAJ: plan before committed trajectory end");
    }

    break;
  }

  case REORIENT: {
    if (!fp_->controlled_reorientation_enable_) {
      fd_->reorientation_required_ = false;
      fd_->reorientation_stop_requested_ = false;
      fd_->reorientation_last_stop_request_time_ = ros::Time(0);
      transitState(PLAN_TRAJ,
                   "REORIENT disabled by configuration");
      break;
    }
    const ros::Time now = ros::Time::now();
    const double speed = fd_->odom_vel_.norm();
    const double elapsed = fd_->reorientation_start_time_.isZero()
                               ? 0.0
                               : (now - fd_->reorientation_start_time_)
                                     .toSec();
    const double odom_age = fd_->last_odom_receive_time_.isZero()
                                ? std::numeric_limits<double>::infinity()
                                : (now - fd_->last_odom_receive_time_).toSec();
    const bool odom_fresh = odom_age <= fp_->max_odom_age_;
    double collision_time = 0.0;
    const bool safe = planner_manager_->checkTrajCollision(collision_time);
    const bool backup_braking =
        safe &&
        (planner_manager_->hasCommittedBackup() ||
         planner_manager_->hasCommittedStopTrajectory()) &&
        planner_manager_->committedTrajectoryRemainingTime() > 0.05;

    // A stale non-zero velocity used to trap the FSM here forever. Conversely,
    // a stale zero must not be treated as proof that the vehicle has stopped.
    // Only a fresh, independently received odometry sample may release the
    // controlled-stop gate.
    if (odom_fresh && speed <= fp_->reorient_exit_speed_) {
      fd_->static_state_ = true;
      fd_->reorientation_required_ = false;
      fd_->reorientation_stop_requested_ = false;
      fd_->reorientation_last_stop_request_time_ = ros::Time(0);
      expl_manager_->ed_->path_next_goal_.clear();
      transitState(PLAN_TRAJ, "REORIENT: vehicle stopped");
      break;
    }

    const bool stop_retry_due =
        fd_->reorientation_last_stop_request_time_.isZero() ||
        (now - fd_->reorientation_last_stop_request_time_).toSec() >=
            fp_->reorient_stop_retry_interval_;
    if (!backup_braking && stop_retry_due) {
      const bool retry = fd_->reorientation_stop_requested_;
      stopTraj();
      fd_->reorientation_stop_requested_ = true;
      fd_->reorientation_last_stop_request_time_ = now;
      ROS_WARN_STREAM("[reorient] request controlled stop"
                      << (retry ? "/retry" : "")
                      << ": speed=" << speed
                      << " elapsed=" << elapsed
                      << " backup_braking=" << backup_braking
                      << " odom_age=" << odom_age);
    }
    if (!odom_fresh) {
      ROS_ERROR_STREAM_THROTTLE(
          1.0, "[reorient] odometry stale; hold stopped trajectory and wait "
                   "for fresh odometry: age="
                   << odom_age << "s max=" << fp_->max_odom_age_ << "s");
    }
    ROS_WARN_STREAM_THROTTLE(
        0.5, "[reorient] braking before goal reversal: speed="
                 << speed << " elapsed=" << elapsed
                 << " backup_braking=" << backup_braking
                 << " odom_age=" << odom_age
                 << " odom_fresh=" << odom_fresh);
    break;
  }

  case CAUTION: {
    stopTraj();
    exec_timer_.stop();
    bool success = planner_manager_->flyToSafeRegion(fd_->static_state_);
    if (success) {
      traj_utils::PolyTraj poly_traj_msg;
      auto info = &planner_manager_->local_data_;
      planner_manager_->polyTraj2ROSMsg(poly_traj_msg, info->start_time_);
      fd_->newest_traj_ = poly_traj_msg;
      poly_traj_pub_.publish(fd_->newest_traj_);
      ros::Duration(0.2).sleep();
    }
    exec_timer_.start();
    double dis2occ =
        planner_manager_->lidar_map_interface_->getDisToOcc(fd_->odom_pos_);
    if (dis2occ > planner_manager_->gcopter_config_->dilateRadiusSoft)
      transitState(PLAN_TRAJ, "safe now");
    break;
  }
  case LAND: {
    stopTraj();
    exec_timer_.stop();
    global_path_update_timer_.stop();
    // 没电了！！再飞就会炸鸡，降落！！！
    while (1) {
      quadrotor_msgs::TakeoffLand land_msg;
      land_msg.takeoff_land_cmd = land_msg.LAND;
      land_pub_.publish(land_msg);
      ros::Duration(0.2).sleep();
      ROS_WARN_THROTTLE(1.0, "NO POWER. LAND!!");
    }

    break;
  }
  }
}

void FastExplorationFSM::init(ros::NodeHandle &nh,
                              FastExplorationManager::Ptr &explorer) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan", fp_->replan_thresh_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("bubble_astar/resolution_astar", fp_->bubble_a_star_resolution, 0.1);
  nh.param("fsm/debug_planner", debug_planner, false);
  nh.param("fsm/emergency_replan_control_error",
           fp_->emergency_replan_control_error, 0.3);
  nh.param("fsm/replan_time_after_traj_start",
           fp_->replan_time_after_traj_start_, 0.5);
  nh.param("fsm/replan_time_before_traj_end", fp_->replan_time_before_traj_end_,
           0.5);
  nh.param("fsm/path_densify_step", fp_->path_densify_step_, 1.0);
  nh.param("FinishNoFrontierMinCount", fp_->finish_no_frontier_min_count_, 4);
  nh.param("fsm/FinishNoFrontierMinCount",
           fp_->finish_no_frontier_min_count_,
           fp_->finish_no_frontier_min_count_);
  nh.param("FinishNoFrontierMinDuration",
           fp_->finish_no_frontier_min_duration_, 1.5);
  nh.param("fsm/FinishNoFrontierMinDuration",
           fp_->finish_no_frontier_min_duration_,
           fp_->finish_no_frontier_min_duration_);
  nh.param("FinishRequireVehicleSlow", fp_->finish_require_vehicle_slow_,
           true);
  nh.param("fsm/FinishRequireVehicleSlow", fp_->finish_require_vehicle_slow_,
           fp_->finish_require_vehicle_slow_);
  nh.param("FinishSlowSpeed", fp_->finish_slow_speed_, 0.5);
  nh.param("fsm/FinishSlowSpeed", fp_->finish_slow_speed_,
           fp_->finish_slow_speed_);
  nh.param("FinishRecheckAfterGoalReached",
           fp_->finish_recheck_after_goal_reached_, true);
  nh.param("fsm/FinishRecheckAfterGoalReached",
           fp_->finish_recheck_after_goal_reached_,
           fp_->finish_recheck_after_goal_reached_);
  nh.param("fsm/auto_trigger", fp_->auto_trigger_enable_, false);
  nh.param("fsm/auto_trigger_delay", fp_->auto_trigger_delay_, 2.0);
  nh.param<string>("fsm/trigger_topic", fp_->trigger_topic_,
                   "/move_base_simple/goal");
  nh.param<string>("fsm/legacy_trigger_topic", fp_->legacy_trigger_topic_,
                   "/waypoint_generator/waypoints");
  nh.param("fsm/global_path_update_min_interval",
           fp_->global_path_update_min_interval_, 0.2);
  nh.param("fsm/cloud_subscriber_queue", fp_->cloud_subscriber_queue_, 1);
  nh.param("fsm/odom_subscriber_queue", fp_->odom_subscriber_queue_, 50);
  nh.param("fsm/sync_queue", fp_->sync_queue_, 20);
  nh.param<string>("fsm/cloud_odom_mode", fp_->cloud_odom_mode_,
                   "approximate_sync");
  nh.param("fsm/latest_odom_timeout", fp_->latest_odom_timeout_, 0.5);
  nh.param("fsm/max_cloud_age", fp_->max_cloud_age_, 0.5);
  fp_->cloud_subscriber_queue_ = std::max(1, fp_->cloud_subscriber_queue_);
  fp_->odom_subscriber_queue_ = std::max(1, fp_->odom_subscriber_queue_);
  fp_->sync_queue_ = std::max(1, fp_->sync_queue_);
  fp_->latest_odom_timeout_ = std::max(0.0, fp_->latest_odom_timeout_);
  if (fp_->cloud_odom_mode_ != "approximate_sync" &&
      fp_->cloud_odom_mode_ != "latest_odom") {
    ROS_WARN_STREAM("[cloud input] unsupported cloud_odom_mode='"
                    << fp_->cloud_odom_mode_
                    << "'; fall back to approximate_sync");
    fp_->cloud_odom_mode_ = "approximate_sync";
  }
  nh.param("fsm/reorient_exit_speed", fp_->reorient_exit_speed_, 0.45);
  nh.param("fsm/reorient_timeout", fp_->reorient_timeout_, 6.0);
  nh.param("fsm/reorient_stop_retry_interval",
           fp_->reorient_stop_retry_interval_, 0.8);
  nh.param("fsm/max_odom_age", fp_->max_odom_age_, 0.5);
  nh.param("fsm/controlled_reorientation_enable",
           fp_->controlled_reorientation_enable_, true);
  nh.param("fsm/plan_failure_retry_delay",
           fp_->plan_failure_retry_delay_, 0.20);
  nh.param("fsm/plan_failure_refresh_count",
           fp_->plan_failure_refresh_count_, 3);
  fp_->reorient_exit_speed_ = std::max(0.05, fp_->reorient_exit_speed_);
  fp_->reorient_timeout_ = std::max(1.0, fp_->reorient_timeout_);
  fp_->reorient_stop_retry_interval_ =
      std::clamp(fp_->reorient_stop_retry_interval_, 0.2, 5.0);
  fp_->max_odom_age_ = std::clamp(fp_->max_odom_age_, 0.1, 5.0);
  fp_->plan_failure_retry_delay_ =
      std::clamp(fp_->plan_failure_retry_delay_, 0.05, 2.0);
  fp_->plan_failure_refresh_count_ =
      std::max(1, fp_->plan_failure_refresh_count_);
  /* Initialize main modules */
  // expl_manager_.reset(new FastExplorationManager);
  // expl_manager_->initialize(nh);
  expl_manager_ = explorer;
  planner_manager_ = expl_manager_->planner_manager_;

  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = {"INIT",       "WAIT_TRIGGER", "PLAN_TRAJ", "CAUTION",
                     "EXEC_TRAJ",  "REORIENT",     "FINISH",    "LAND"};
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->auto_triggered_ = false;
  fd_->reorientation_required_ = false;
  fd_->reorientation_stop_requested_ = false;
  fd_->consecutive_plan_failures_ = 0;
  fd_->first_odom_time_ = ros::Time(0);
  fd_->last_odom_receive_time_ = ros::Time(0);
  fd_->reorientation_start_time_ = ros::Time(0);
  fd_->reorientation_last_stop_request_time_ = ros::Time(0);
  fd_->next_plan_retry_time_ = ros::Time(0);
  fd_->use_bubble_a_star_ = false;
  last_plan_traj_global_update_time_ = ros::Time(0);
  last_global_callback_wall_time_ = ros::WallTime();
  latest_odom_msg_.reset();
  latest_odom_receive_wall_time_ = ros::WallTime();
  battary_sub_ =
      nh.subscribe("/mavros/battery", 10, &FastExplorationFSM::battaryCallback,
                   this, ros::TransportHints().tcpNoDelay());

  /* Ros sub, pub and timer */
  // if (debug_planner) {
  //   exec_timer_ = nh.createTimer(ros::Duration(0.01),
  //   &FastExplorationFSM::PlannerDebugFSMCallback, this);
  // } else {
  exec_timer_ = nh.createTimer(ros::Duration(0.01),
                               &FastExplorationFSM::FSMCallback, this);
  // }
  global_path_update_timer_ = nh.createTimer(
      ros::Duration(std::max(0.02, fp_->global_path_update_min_interval_)),
      &FastExplorationFSM::globalPathUpdateCallback, this);
  if (!fp_->legacy_trigger_topic_.empty()) {
    trigger_sub_ = nh.subscribe(fp_->legacy_trigger_topic_, 1,
                                &FastExplorationFSM::triggerCallback, this);
  }
  if (!fp_->trigger_topic_.empty()) {
    nav_goal_trigger_sub_ =
        nh.subscribe(fp_->trigger_topic_, 1,
                     &FastExplorationFSM::navGoalTriggerCallback, this);
  }
  ROS_INFO_STREAM("[exploration trigger] auto=" << fp_->auto_trigger_enable_
                  << " nav_goal_topic=" << fp_->trigger_topic_
                  << " legacy_path_topic=" << fp_->legacy_trigger_topic_);
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);

  heartbeat_pub_ = nh.advertise<std_msgs::Empty>("/planning/heartbeat", 10);
  land_pub_ =
      nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);

  poly_traj_pub_ =
      nh.advertise<traj_utils::PolyTraj>("/planning/trajectory", 10);
  poly_yaw_traj_pub_ =
      nh.advertise<traj_utils::PolyTraj>("/planning/yaw_trajectory", 10);
  time_cost_pub_ = nh.advertise<std_msgs::Float32>("/time_cost", 10);
  static_pub_ = nh.advertise<std_msgs::Bool>("/planning/static", 10);
  state_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/state", 10);
  speed_pub_ = nh.advertise<visualization_msgs::Marker>("/planning/speed", 10);
  string odom_topic, cloud_topic;
  nh.getParam("odometry_topic", odom_topic);
  nh.getParam("cloud_topic", cloud_topic);
  // Keep current vehicle state independent of point-cloud synchronization.
  // The synchronized callback can legitimately pause while pairing or while a
  // stale cloud is dropped; braking and FSM transitions must still see every
  // new odometry message.
  raw_odom_sub_ = nh.subscribe(
      odom_topic, fp_->odom_subscriber_queue_,
      &FastExplorationFSM::odometryCallback, this,
      ros::TransportHints().tcpNoDelay());
  if (fp_->cloud_odom_mode_ == "latest_odom") {
    latest_cloud_sub_ = nh.subscribe(
        cloud_topic, fp_->cloud_subscriber_queue_,
        &FastExplorationFSM::latestCloudCallback, this,
        ros::TransportHints().tcpNoDelay());
    ROS_INFO_STREAM("[cloud input] mode=latest_odom cloud_topic="
                    << cloud_topic << " odom_topic=" << odom_topic
                    << " cloud_queue=" << fp_->cloud_subscriber_queue_
                    << " odom_queue=" << fp_->odom_subscriber_queue_
                    << " latest_odom_timeout=" << fp_->latest_odom_timeout_
                    << "s header_age_gate=disabled");
  } else {
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        nh, cloud_topic, fp_->cloud_subscriber_queue_));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh, odom_topic, fp_->odom_subscriber_queue_));
    sync_cloud_odom_.reset(
        new message_filters::Synchronizer<SyncPolicyCloudOdom>(
            SyncPolicyCloudOdom(fp_->sync_queue_), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(
        boost::bind(&FastExplorationFSM::CloudOdomCallback, this, _1, _2));
    ROS_INFO_STREAM("[cloud sync] mode=approximate_sync cloud_topic="
                    << cloud_topic << " odom_topic=" << odom_topic
                    << " cloud_queue=" << fp_->cloud_subscriber_queue_
                    << " odom_queue=" << fp_->odom_subscriber_queue_
                    << " sync_queue=" << fp_->sync_queue_
                    << " max_cloud_age=" << fp_->max_cloud_age_ << "s"
                    << " independent_odom=1"
                    << " max_odom_age=" << fp_->max_odom_age_ << "s");
  }
}

void FastExplorationFSM::battaryCallback(
    const sensor_msgs::BatteryStateConstPtr &msg) {
  // if(msg->voltage < 21.0){
  //   transitState(LAND, "battary low");
  // }
}

void FastExplorationFSM::updateTopoAndGlobalPath() {
  if (!(state_ == WAIT_TRIGGER || state_ == PLAN_TRAJ || state_ == EXEC_TRAJ ||
        state_ == REORIENT || state_ == FINISH)) {
    global_path_update_timer_.stop();
    // expl_manager_->frontier_manager_ptr_->viz_pocc();
    expl_manager_->frontier_manager_ptr_->visfrtcluster();
    global_path_update_timer_.start();
    return;
  }
  static int cnt = 0;
  cnt++;

  global_path_update_timer_.stop();
  ros::Time t2 = ros::Time::now();
  planner_manager_->topo_graph_->getRegionsToUpdate();
  // cout << "getRegionsToUpdate time cost:" << (ros::Time::now() - t2).toSec()
  // * 1000 << "ms" << endl;
  planner_manager_->topo_graph_->updateSkeleton();

  ros::Time t3 = ros::Time::now();
  planner_manager_->topo_graph_->updateOdomNode(fd_->odom_pos_, fd_->odom_yaw_);
  planner_manager_->topo_graph_->updateHistoricalOdoms();

  if (state_ == WAIT_TRIGGER) {
    planner_manager_->graph_visualizer_->vizBox(planner_manager_->topo_graph_);
    expl_manager_->frontier_manager_ptr_->viz_pocc();
    expl_manager_->frontier_manager_ptr_->visfrtcluster();
    global_path_update_timer_.start();
    return;
  }

  if (planner_manager_->topo_graph_->odom_node_->neighbors_.empty()) {
    double time;
    if (planner_manager_->hasCommittedTrajectory()) {
      bool safe = planner_manager_->checkTrajCollision(time);
      if (!safe) {
        transitState(CAUTION, "odom_node no nbrs");
      } else {
        global_path_update_timer_.start();
        return;
      }
    } else {
      transitState(CAUTION, "odom_node no nbrs");
    }
    global_path_update_timer_.start();
    return;
  }
  if (planner_manager_->hasCommittedTrajectory()) {

    double curr_time =
        (ros::Time::now() - planner_manager_->local_data_.start_time_).toSec();
    double time;
    bool safe = planner_manager_->checkTrajCollision(time);
    double total_time = planner_manager_->local_data_.duration_;
    double time2end = total_time - curr_time;

    if (safe && curr_time < fp_->replan_time_after_traj_start_ &&
        time2end > fp_->replan_time_before_traj_end_) {
      global_path_update_timer_.start();
      return;
    }
  }
  cout << endl << endl;
  cout << "\033[1;33m------------- <" << cnt
       << "> Plan Global Path start---------------" << "\033[0m" << endl;
  planner_manager_->topo_graph_->log << "<" << cnt << ">" << endl;
  ros::Time t4 = ros::Time::now();
  // cout << "updateSkeleton time cost:" << (t3 - t2).toSec() * 1000 << "ms" <<
  // endl; if( (t3 - t1).toSec() * 1000 > 100){
  //   ROS_ERROR("time too long");
  //   exit(0);
  // }
  ROS_INFO("update topo skeleton cost: %fms, update odom vertex cost:%fms ",
           (t3 - t2).toSec() * 1000, (t4 - t3).toSec() * 1000);
  Eigen::Vector3d vel = fd_->odom_vel_.cast<double>();
  Eigen::Vector3d odom = fd_->odom_pos_.cast<double>();
  int res = expl_manager_->planGlobalPath(odom, vel);
  ros::Time t5 = ros::Time::now();

  cout << "\033[1;33m-------------Plan Global Path end-----------------"
       << "\033[0m" << endl
       << endl;

  planner_manager_->graph_visualizer_->vizBox(planner_manager_->topo_graph_);
  if(expl_manager_->ep_->view_graph_)
    planner_manager_->graph_visualizer_->vizGraph(planner_manager_->topo_graph_);
  std_msgs::Float32 time_cost;
  double time_cost_now = (t5 - t2).toSec() * 1000;
  time_cost.data = time_cost_now;
  time_cost_pub_.publish(time_cost);

  cout << "total time cost: " << time_cost_now << "ms" << endl;
  if (res == NO_FRONTIER && state_ != WAIT_TRIGGER) {
    handleNoFrontierResult("planGlobalPath: no frontier");
  } else if (fp_->controlled_reorientation_enable_ && res == FAIL &&
             expl_manager_->last_plan_requires_reorientation_ &&
             state_ != WAIT_TRIGGER && state_ != REORIENT) {
    fd_->reorientation_required_ = true;
    fd_->reorientation_start_time_ = ros::Time::now();
    fd_->reorientation_stop_requested_ = false;
    fd_->reorientation_last_stop_request_time_ = ros::Time(0);
    transitState(REORIENT, "planGlobalPath: brake before goal reversal", true);
  } else if (res == SUCCEED && state_ != WAIT_TRIGGER && state_ != REORIENT) {
    resetFinishGate("planGlobalPath succeed");
    transitState(PLAN_TRAJ, "planGlobalPath: succeed");
  } else if (res == FAIL && state_ == FINISH &&
             expl_manager_->frontier_manager_ptr_ &&
             expl_manager_->frontier_manager_ptr_->activeClusterCount() > 0) {
    resetFinishGate("finish recovery: frontier pending");
    transitState(PLAN_TRAJ, "finish recovery: frontier pending", true);
  }

  expl_manager_->frontier_manager_ptr_->viz_pocc();
  expl_manager_->frontier_manager_ptr_->visfrtcluster();
  static ros::Time t_p = ros::Time::now();
  if ((ros::Time::now() - t_p).toSec() > 5.0) {
    expl_manager_->frontier_manager_ptr_->printMemoryCost();
    t_p = ros::Time::now();
  }
  global_path_update_timer_.start();
  cout << "viz&&print cost:" << (ros::Time::now() - t5).toSec() * 1000 << "ms"
       << endl;
}

void FastExplorationFSM::globalPathUpdateCallback(const ros::TimerEvent &e) {
  const ros::WallTime now = ros::WallTime::now();
  if (!last_global_callback_wall_time_.isZero()) {
    ROS_INFO_STREAM_THROTTLE(
        1.0, "[global update] actual callback interval="
                 << (now - last_global_callback_wall_time_).toSec() * 1000.0
                 << "ms configured="
                 << fp_->global_path_update_min_interval_ * 1000.0 << "ms");
  }
  last_global_callback_wall_time_ = now;
  updateTopoAndGlobalPath();
}
