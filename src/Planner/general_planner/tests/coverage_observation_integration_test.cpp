#include <general_core/exploration/highspeed/expl_data.h>
#include <general_core/exploration/highspeed/fast_exploration_manager.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <stdexcept>

void require(bool condition, const char *reason) {
  if (!condition) throw std::runtime_error(reason);
}

void verifyCommittedContinuity(fast_planner::FastPlannerManager &planner) {
  using namespace fast_planner;
  auto straight=[](const Eigen::Vector3d &from, const Eigen::Vector3d &to) {
    std::vector<Eigen::Vector3f> result;
    const int steps=std::max(1,static_cast<int>(std::ceil((to-from).norm()/.5)));
    for (int i=0;i<=steps;++i) result.push_back((from+(to-from)*i/steps).cast<float>());
    return result;
  };
  auto path=straight({0,0,1.5},{7,0,1.5});
  require(planner.planExploreTraj(path,true,false,false), "continuity fixture failed to commit");
  auto old=planner.local_data_.minco_traj_;
  double old_time=std::min(1.2,planner.local_data_.duration_*.4);
  require(old.getVel(old_time).norm()>.5, "continuity fixture was not moving");
  ros::Time::setNow(planner.local_data_.start_time_+ros::Duration(old_time));
  planner.local_data_.curr_pos_=old.getPos(old_time);
  planner.local_data_.curr_vel_=old.getVel(old_time);
  path=straight(planner.local_data_.curr_pos_,{7,0,1.5});
  // REORIENT used to leave this flag true when its failed replacement retained
  // the accelerating command. The committed state must take precedence.
  require(planner.planExploreTraj(path,true,false,false,{}, {},{true,false}),
          "moving command could not replace a stale static flag");
  auto check_prefix=[&](double t, double old_t) {
    const auto &next=planner.local_data_.minco_traj_;
    require((next.getPos(t)-old.getPos(old_t)).norm()<1e-5, "committed position jumped");
    require((next.getVel(t)-old.getVel(old_t)).norm()<1e-5, "committed velocity jumped");
    require((next.getAcc(t)-old.getAcc(old_t)).norm()<1e-4, "committed acceleration jumped");
  };
  check_prefix(0,old_time); check_prefix(.1,old_time+.1);

  ros::Time::setNow(planner.local_data_.start_time_+ros::Duration(.25));
  planner.local_data_.curr_pos_=planner.local_data_.minco_traj_.getPos(.25);
  planner.local_data_.curr_vel_=planner.local_data_.minco_traj_.getVel(.25);
  require(planner.planControlledStopTrajectory(true), "coverage fixture could not commit a validated shorter brake");
  old=planner.local_data_.minco_traj_;
  const double stop_duration=planner.local_data_.duration_;
  old_time=stop_duration-.30;
  ros::Time::setNow(planner.local_data_.start_time_+ros::Duration(old_time));
  // Reproduce the bag: less than the 0.45 s switch budget remains and odometry
  // is 200 ms behind. Preserve the real brake tail, then its stopped HOLD.
  planner.local_data_.curr_pos_=old.getPos(old_time-.20);
  planner.local_data_.curr_vel_=old.getVel(old_time-.20);
  path=straight(old.getPos(old_time),{8,0,1.5});
  require(planner.planExploreTraj(path,true,false,false,{}, {},{true,false}),
          "short brake tail could not replan through its terminal hold");
  check_prefix(0,old_time); check_prefix(.20,old_time+.20);
  require((planner.local_data_.minco_traj_.getPos(.305)-old.getPos(stop_duration)).norm()<1e-5 &&
          planner.local_data_.minco_traj_.getVel(.305).norm()<1e-5,
          "short brake tail was extrapolated instead of holding its stopped endpoint");
  const int committed=planner.local_data_.traj_id_;
  require(!planner.planExploreTraj({},true,false,false,{}, {},{true,false}) &&
          planner.local_data_.traj_id_==committed,
          "failed replacement revoked the still executable command");

  old=planner.local_data_.minco_traj_;
  old_time=planner.local_data_.duration_;
  ros::Time::setNow(planner.local_data_.start_time_+ros::Duration(old_time+.15));
  planner.local_data_.curr_pos_=old.getPos(old_time);
  planner.local_data_.curr_vel_.setZero();
  path=straight(planner.local_data_.curr_pos_,{2,0,1.5});
  require(planner.planExploreTraj(path,true,false,false,{}, {},{true,false}),
          "expired stopped command could not restart from its endpoint");
  check_prefix(0,old_time);
  planner.clearReleasedTrajectory();
  require(!planner.hasCommittedTrajectory(), "released task retained execution ownership");
  planner.local_data_.curr_pos_=Eigen::Vector3d(0,2,1.5);
  planner.local_data_.curr_vel_.setZero();
  path=straight(planner.local_data_.curr_pos_,{4,2,1.5});
  require(planner.planExploreTraj(path,true,false,false,{}, {},{true,false}) &&
          (planner.local_data_.minco_traj_.getPos(0)-planner.local_data_.curr_pos_).norm()<1e-5,
          "new coverage task reused the preceding task's endpoint after handover");
  CoverageObservationContext endpoint_observation;
  endpoint_observation.enabled=true;endpoint_observation.goal=path.back().cast<double>();
  CoverageObservationGate gate;gate.goal=endpoint_observation.goal;gate.yaw=1.2;gate.yaw_tolerance=.1;
  endpoint_observation.gates={gate};
  require(planner.planExploreTraj(path,true,false,false,{},endpoint_observation,{true,false}),
          "terminal yaw fixture could not commit");
  const auto old_yaw=planner.local_data_.minco_yaw_traj_;
  old_time=planner.local_data_.duration_;
  require(old_yaw.getVel(old_time).norm()<1e-3 && old_yaw.getAcc(old_time).norm()>.01,
          "terminal yaw fixture did not exercise unconstrained angular acceleration");
  ros::Time::setNow(planner.local_data_.start_time_+ros::Duration(old_time+.2));
  planner.local_data_.curr_pos_=planner.local_data_.minco_traj_.getPos(old_time);
  planner.local_data_.curr_yaw_=old_yaw.getPos(old_time).x();
  path=straight(planner.local_data_.curr_pos_,{0,2,1.5});
  require(planner.planExploreTraj(path,true,false,false,{}, {},{true,false}) &&
          (planner.local_data_.minco_yaw_traj_.getPos(0)-old_yaw.getPos(old_time)).norm()<1e-5 &&
          planner.local_data_.minco_yaw_traj_.getVel(0).norm()<1e-5,
          "valid terminal yaw acceleration trapped stopped coverage replanning");
  std::cout << "coverage committed continuity PASS: moving static flag, short brake tail, expired hold, rejection retention, task handover, terminal yaw hold\n";
}

struct FrontierObservationEvidenceTestAccess {
  static void run() {
    FrontierManager frontier;
    const Eigen::Vector3i cell(1,2,3);ByteArrayRaw key;frontier.idx2bytes(cell,key);
    frontier.frtd_.label_map_[key]=DENSE;
    require(frontier.observedBoundaryFraction({cell})==0,
            "heuristic DENSE / discarded frontier was mistaken for measured observation");
    frontier.observation_evidence_.insert(key);
    require(frontier.observedBoundaryFraction({cell,Eigen::Vector3i(2,2,3)})==.5,
            "measured boundary evidence did not stay separate from task labels");
    // Reproduce the full-launch failure: a small dormant cluster can still
    // supply a valid viewpoint, so the finish inventory must count it too.
    auto dormant=std::make_shared<ClusterInfo>();
    dormant->id_=1;
    dormant->state_=FrontierState::SUSPENDED;
    dormant->is_dormant_=true;
    dormant->is_reachable_=true;
    dormant->needs_revalidation_=false;
    frontier.cluster_list_.push_back(dormant);
    require(dormant->canGenerateViewpoint() && frontier.activeClusterCount()==1 &&
            frontier.reachableClusterCount()==1,
            "executable dormant frontier disappeared from the finish inventory");
    frontier.requestGlobalRecluster();
    require(dormant->needs_revalidation_ && frontier.reachableClusterCount()==0,
            "cached reachability survived the start of a full audit");
    frontier.finishGlobalAuditIfComplete();
    require(!frontier.frontierAuditReady(),"full audit skipped an unvalidated dormant frontier");
    dormant->needs_revalidation_=false;
    frontier.finishGlobalAuditIfComplete();
    require(frontier.frontierAuditReady() && frontier.reachableClusterCount()==1,
            "validated dormant viewpoint did not veto completion");
    dormant->needs_revalidation_=true;
    require(!frontier.frontierAuditReady(),
            "a rebuilt cluster reused an old audit with the same semantic revision");
    dormant->needs_revalidation_=false;
    dormant->state_=FrontierState::VISITED;
    require(!dormant->canGenerateViewpoint() && frontier.activeClusterCount()==0 &&
            frontier.reachableClusterCount()==0,"visited frontier returned to the executable inventory");
  }
};

namespace fast_planner {
struct CoverageRecoveryTestAccess {
  static void run(FastExplorationManager &manager, FastPlannerManager &planner) {
    manager.planner_manager_=std::shared_ptr<FastPlannerManager>(&planner,[](FastPlannerManager *){});
    CoverageTarget region;
    region.stable_id=42; region.has_approach=true; region.voxel_count=100;
    region.approach_position=Eigen::Vector3d(1,0,1.5);
    region.approach_candidates={region.approach_position,Eigen::Vector3d(4,0,1.5)};
    auto action=region; action.approach_candidates.clear();
    for (int i=0; i<2; ++i) manager.deferCoverageRecovery(action,
        FastExplorationManager::CoverageRecoveryOutcome::TRAJECTORY_FAILURE);
    require(manager.coverageRecoveryExhausted(action),"local attempt budget did not bound repeated failure");
    require(!manager.coverageRecoveryExhausted(region),"one entrance failure exhausted another entrance");
    action.approach_position=region.approach_candidates[1];
    for (int i=0; i<2; ++i) manager.deferCoverageRecovery(action,
        FastExplorationManager::CoverageRecoveryOutcome::TRAJECTORY_FAILURE);
    require(manager.coverageRecoveryExhausted(region),"all failed entrances not recognized as search exhausted");
    const auto old=planner.local_data_.curr_pos_;
    planner.local_data_.curr_pos_.x()+=3.5;
    require(manager.coverageRecoveryExhausted(region),"moving the execution origin reset an unchanged action budget");
    for (int i=0; i<2; ++i) manager.deferCoverageRecovery(action,
        FastExplorationManager::CoverageRecoveryOutcome::TRAJECTORY_FAILURE);
    planner.local_data_.curr_pos_=old;
    require(manager.coverageRecoveryExhausted(region),"A->B->A overwrote the previous origin's failed attempts");
    planner.local_data_.curr_pos_.x()+=7.0;
    require(manager.coverageRecoveryExhausted(action),"unchanged map allowed an unlimited sequence of new origin retries");
    auto observed_region=action;observed_region.voxel_count=40;
    require(!manager.coverageRecoveryExhausted(observed_region),"new evidence in the actual target did not reopen its action");
    planner.local_data_.curr_pos_=old;
    manager.resetCoverageRecovery();
    // Two differently sized unknown components may share one approach. A
    // component switch is not measured shrinkage and cannot replenish the
    // trajectory-failure budget of that unchanged executable approach.
    action=region; action.approach_candidates.clear();
    manager.deferCoverageRecovery(action,
        FastExplorationManager::CoverageRecoveryOutcome::TRAJECTORY_FAILURE);
    auto smaller_neighbor=action;
    smaller_neighbor.stable_id=43; smaller_neighbor.voxel_count=30;
    require(manager.coverageRecoveryCooling(smaller_neighbor,ros::Time::now(),nullptr),
            "a smaller neighboring component bypassed the failed approach cooldown");
    manager.deferCoverageRecovery(smaller_neighbor,
        FastExplorationManager::CoverageRecoveryOutcome::TRAJECTORY_FAILURE);
    require(manager.coverageRecoveryExhausted(action) &&
            manager.coverageRecoveryExhausted(smaller_neighbor),
            "alternating component sizes reset the same approach failure budget");
    for (int i=0; i<3; ++i) {
      manager.deferCoverageRecovery(action,
          FastExplorationManager::CoverageRecoveryOutcome::TRAJECTORY_FAILURE);
      require(manager.coverageRecoveryExhausted(smaller_neighbor),
              "an aliased component was compared with another component's voxel count");
    }
    observed_region=action; observed_region.voxel_count=40;
    require(!manager.coverageRecoveryExhausted(observed_region),
            "same-component measured shrinkage no longer reopened its approach");
    manager.resetCoverageRecovery();
    // Reproduce the tail: no frontend for 20 s, an active bounded CP action,
    // and successive valid CP snapshots. New CP snapshots must not postpone
    // audit, while a reachable frontier (even cooling) must reopen exploration.
    manager.coverage_finish_last_progress_time_=ros::Time::now()-ros::Duration(25);
    manager.coverage_finish_progress_observed_voxels_=20000;
    manager.has_active_coverage_goal_=true;
    manager.updateCoverageCompletion(0,true,false,20060);
    require(!manager.coverage_terminal_audit_pending_,"stale plan started completion audit");
    manager.updateCoverageCompletion(0,true,true,20060);
    require(manager.coverage_terminal_audit_pending_ && manager.has_active_coverage_goal_,
            "completion failed to drain the active action or canceled it early");
    manager.updateCoverageCompletion(0,true,true,20072);
    require(manager.coverage_terminal_audit_pending_,"CP snapshot churn canceled completion audit");
    manager.updateCoverageProgress(20072,35519);
    require(manager.coverage_terminal_audit_pending_ &&
            manager.coverage_terminal_audit_observed_voxels_==20060,
            "old boundary gains crossed the previous credit threshold and reopened the audit");
    manager.updateCoverageProgress(21000,35519);
    require(!manager.coverage_terminal_audit_pending_,"large CP observation gain was ignored by completion");
    manager.coverage_finish_last_progress_time_=ros::Time::now()-ros::Duration(25);
    manager.updateCoverageCompletion(0,true,true,21000);
    manager.updateCoverageCompletion(1,true,true,21000);
    require(!manager.coverage_terminal_audit_pending_ &&
            (ros::Time::now()-manager.coverage_finish_last_progress_time_).toSec()<.1,
            "reachable frontier in cooldown was treated as coverage completion");
    manager.resetCoverageRecovery();
    manager.coverage_route_config_.enabled=true;
    manager.coverage_route_tasks_[71].deferred=6;
    manager.coverage_route_tasks_[72].deferred=6;
    manager.coverage_route_observations_.clear();
    manager.notifyCoverageRoutePassage(71);
    require(manager.coverage_route_tasks_[71].verifying &&
            manager.coverage_route_tasks_[71].deferred==0 &&
            !manager.coverage_route_tasks_[72].verifying &&
            manager.coverage_route_tasks_[72].deferred==6,
            "submitted passage lost identity after candidate refresh or verified another task");
    manager.coverage_route_tasks_[71].evidence_version=123;
    manager.notifyCoverageRoutePassage(71);
    require(manager.coverage_route_tasks_[71].evidence_version==123,
            "duplicate passage reset the observation evidence epoch");
    manager.notifyCoverageRoutePassage(999);
    require(manager.coverage_route_tasks_.size()==2,"expired passage recreated a task");
    manager.coverage_route_tasks_.clear();
    manager.ep_->goal_lock_match_radius_=.75;
    manager.ep_->failed_goal_penalty_=2000;
    manager.deferred_goals_.push_back({7,Eigen::Vector3f(0,0,1.5),ros::Time::now()+ros::Duration(30)});
    auto second_entrance=std::make_shared<TopoNode>();
    second_entrance->frontier_cluster_id_=7;second_entrance->center_=Eigen::Vector3f(3,0,1.5);
    require(manager.failedGoalPenalty(second_entrance)==0,"one failed coverage viewpoint cooled all alternatives");
    second_entrance->center_.x()=.1;
    require(manager.failedGoalPenalty(second_entrance)>0,"failed coverage entrance immediately retried");
    manager.coverage_route_config_.enabled=false;second_entrance->center_.x()=3;
    require(manager.failedGoalPenalty(second_entrance)>0,"coverage entrance policy changed default cooldown semantics");
    manager.deferred_goals_.clear();
    manager.deferCoverageRecovery(action,FastExplorationManager::CoverageRecoveryOutcome::OCCLUDED);
    require(manager.coverageRecoveryExhausted(action),"occluded observation was not recorded");
    planner.local_data_.curr_pos_.x()+=3.5;
    require(manager.coverageRecoveryExhausted(action),"moving the origin resurrected an unchanged occluded sight ray");
    planner.local_data_.curr_pos_=old;
    action.stable_id=43;
    require(!manager.coverageRecoveryExhausted(action),"occluded sight ray exhausted a different observation direction");
    manager.active_coverage_target_=action;
    manager.active_coverage_target_.stable_id=99;
    manager.active_coverage_target_.approach_position=Eigen::Vector3d(2,2,1.5);
    manager.has_active_coverage_goal_=true;
    manager.active_coverage_goal_start_=ros::Time::now();
    const auto began=manager.active_coverage_goal_start_;
    manager.deferCoverageRecovery(action,FastExplorationManager::CoverageRecoveryOutcome::OCCLUDED);
    require(manager.has_active_coverage_goal_ && manager.active_coverage_target_.stable_id==99 &&
            manager.active_coverage_goal_start_==began,
            "another candidate's failure canceled the active coverage action");
    manager.ed_=std::make_shared<ExplorationData>();
    manager.ep_->goal_lock_enable_=true;
    manager.ed_->has_goal_lock_=true;
    manager.ed_->locked_goal_is_coverage_=true;
    manager.ed_->locked_goal_coverage_id_=99;
    manager.ed_->locked_goal_=manager.active_coverage_target_.approach_position.cast<float>();
    manager.ed_->locked_goal_time_=ros::Time::now()-ros::Duration(30);
    auto alternative=std::make_shared<TopoNode>();
    alternative->is_coverage_target_=true; alternative->coverage_target_id_=100;
    alternative->center_=Eigen::Vector3f(5,2,1.5);
    auto held=std::make_shared<TopoNode>();
    held->is_coverage_target_=true; held->coverage_target_id_=99; held->center_=manager.ed_->locked_goal_;
    std::vector<TopoNode::Ptr> choices{alternative,held};
    require(manager.selectStableGoalIndex(choices,{1.0,100.0},0,Eigen::Vector3d::Zero())==1,
            "coverage cost update replaced an active observation before arrival or timeout");
    manager.active_coverage_goal_start_=ros::Time::now()-ros::Duration(100);
    require(manager.selectStableGoalIndex(choices,{1.0,100.0},0,Eigen::Vector3d::Zero())==0,
            "coverage action commitment ignored its timeout");
    manager.coverage_recovery_reached_radius_=1.0;
    const Eigen::Vector3d observation=manager.active_coverage_target_.approach_position;
    require(!manager.completeActiveCoverageGoalIfReached(observation+Eigen::Vector3d(1.1,0,0)),
            "coverage observation completed outside its radius");
    require(manager.completeActiveCoverageGoalIfReached(observation+Eigen::Vector3d(.75,0,0)) &&
            !manager.has_active_coverage_goal_,
            "configured recovery radius did not complete the active observation");
    manager.resetCoverageRecovery();
  }
};
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "coverage_observation_integration_test");
  ros::NodeHandle nh("~");
  try {
    using namespace fast_planner;
    FrontierObservationEvidenceTestAccess::run();
    auto lio = std::make_shared<LIOInterface>();
    auto search = std::make_shared<ParallelBubbleAstar>();
    auto graph = std::make_shared<TopoGraph>();
    FastPlannerManager planner;
    lio->init(nh);
    graph->init(nh, lio, search);
    search->init(nh, lio);
    planner.initPlanModules(nh, search, graph);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    auto point = [&](double x, double y, double z) {
      pcl::PointXYZI p; p.x=x; p.y=y; p.z=z; p.intensity=1; cloud.push_back(p);
    };
    // Convex room: every supplied surface point is a valid first return from
    // the origin. Dense repeated rays establish actual raw ROG free evidence.
    for (double x=-10; x<=10; x+=.15) for (double y=-5; y<=5; y+=.15) {
      point(x,y,0); point(x,y,4);
    }
    for (double x=-10; x<=10; x+=.15) for (double z=0; z<=4; z+=.15) {
      point(x,-5,z); point(x,5,z);
    }
    for (double y=-5; y<=5; y+=.15) for (double z=0; z<=4; z+=.15) {
      point(-10,y,z); point(10,y,z);
    }
    auto msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(cloud, *msg); msg->header.frame_id="world";
    auto odom = boost::make_shared<nav_msgs::Odometry>();
    odom->header.frame_id="world";
    odom->pose.pose.position.z=1.5; odom->pose.pose.orientation.w=1;
    for (int i=0; i<12; ++i) {
      msg->header.stamp=odom->header.stamp=ros::Time::now();
      lio->updateCloudMapOdometry(msg,odom);
      require(planner.updateRogMap(msg,odom), "ROG update failed");
    }
    planner.local_data_.curr_pos_ = Eigen::Vector3d(0,0,1.5);
    planner.local_data_.curr_vel_.setZero();
    planner.local_data_.curr_yaw_ = planner.local_data_.end_yaw_ = 0;
    for (double x=0; x<=7; x+=.1)
      require(planner.isObservedLocalKnownFree(Eigen::Vector3d(x,0,1.5)),
              "test route not supported by raw observed free evidence");
    if (argc>1 && std::string(argv[1])=="--continuity") {
      verifyCommittedContinuity(planner);
      return 0;
    }
    std::vector<Eigen::Vector3f> path;
    for (int x=0; x<=7; ++x) path.emplace_back(x,0,1.5);
    CoverageObservationContext observation;
    observation.enabled=true; observation.goal=Eigen::Vector3d(3,0,1.5);
    observation.radius=.35;
    require(planner.planExploreTraj(path,true,false,false,{},observation),
            "continuous observation did not commit through real backend");
    double best=1e9, passage_speed=0;
    for (double t=0; t<=planner.local_data_.duration_; t+=.005) {
      const double d=(planner.local_data_.minco_traj_.getPos(t)-observation.goal).norm();
      if (d<best) { best=d; passage_speed=planner.local_data_.minco_traj_.getVel(t).norm(); }
    }
    require(best<observation.radius && passage_speed>.5,
            "observation was skipped or became a stationary intermediate point");
    require(planner.local_data_.minco_traj_.getVel(planner.local_data_.duration_).norm()<.1,
            "test continuation lost its safe stopped endpoint");
    const int committed=planner.local_data_.traj_id_;
    observation.goal.y()=3;
    require(!planner.planExploreTraj(path,true,false,false,{},observation),
            "optimizer accepted path missing requested observation");
    require(planner.local_data_.traj_id_==committed, "rejection replaced active command");

    observation.goal=Eigen::Vector3d(2,0,1.5);
    CoverageObservationGate first_gate;first_gate.goal=observation.goal;first_gate.yaw_tolerance=.5;
    CoverageObservationGate second_gate=first_gate;second_gate.goal.x()=4;
    observation.gates={first_gate,second_gate};
    require(planner.planExploreTraj(path,true,false,false,{},observation),
            "ordered multiple observations did not commit");
    const int sequence_committed=planner.local_data_.traj_id_;
    observation.gates[0].yaw=M_PI;observation.gates[0].yaw_tolerance=.1;
    require(!planner.planExploreTraj(path,true,false,false,{},observation),
            "position-only pass accepted incompatible observation yaw");
    require(planner.local_data_.traj_id_==sequence_committed,"failed yaw validation replaced command");
    observation.gates={second_gate,first_gate};
    require(!planner.planExploreTraj(path,true,false,false,{},observation),
            "reversed observations accepted on a forward-only guide");
    observation.gates.clear();

    // Real room with an intermediate corner that unconstrained MINCO can cut.
    // The original straight-only test could not exercise the bag's 8/8 misses.
    path={{0,0,1.5},{1,1,1.5},{2,2,1.5},{3,2,1.5},{4,2,1.5},{5,2,1.5},{6,2,1.5}};
    observation.goal=Eigen::Vector3d(2,2,1.5);
    require(planner.planExploreTraj(path,true,false,false,{},observation),
            "corner observation corridor constraint did not commit");
    double corner_distance=1e9,corner_speed=0;
    for (double t=0; t<=planner.local_data_.duration_; t+=.005) {
      const double distance=(planner.local_data_.minco_traj_.getPos(t)-observation.goal).norm();
      if (distance<corner_distance) {corner_distance=distance;corner_speed=planner.local_data_.minco_traj_.getVel(t).norm();}
    }
    require(corner_distance<observation.radius && corner_speed>.3,
            "corner observation missed or became a full stop");
    require(planner.planExploreTraj(path,true,false,false),
            "default optimizer invocation failed after ordered observation");
    require(planner.local_data_.minco_traj_.getPieceNum()==1,
            "ordered observation corridor leaked into a subsequent default invocation");
    // Independent stopped-endpoint case; an active replacement deliberately
    // contains the preceding command prefix (covered by --continuity).
    planner.clearReleasedTrajectory();
    observation.goal=path.back().cast<double>();
    CoverageObservationGate endpoint;endpoint.goal=observation.goal;endpoint.yaw=.8;endpoint.yaw_tolerance=.1;
    observation.gates={endpoint};
    require(planner.planExploreTraj(path,true,false,false,{},observation,CoverageExecutionContext{true,false}),
            "terminal observation yaw did not commit");
    require(planner.local_data_.minco_traj_.getPieceNum()==1,
            "endpoint-only observation unnecessarily disabled corridor simplification");
    observation.gates.clear();
    require(planner.prepareCoveragePath(path,true),"observed repair seed was not executable");
    require(planner.planExploreTraj(path,true,false,false,{}, {},CoverageExecutionContext{true,true}),
            "bounded geometric repair failed through the real backend");
    for (double t=0; t<planner.local_data_.duration_; t+=.01)
      require(planner.isObservedLocalKnownFree(planner.local_data_.minco_traj_.getPos(t)),
              "geometric repair left raw observed free space");
    double collision_time=0;
    require(planner.checkTrajCollision(collision_time,true),"live coverage check rejected the certified room trajectory");
    require(lio->setSingleExplorationBox(Eigen::Vector3f(-12,-6,-.2),Eigen::Vector3f(3,6,4.2)),
            "could not update live exploration bounds");
    require(!planner.checkTrajCollision(collision_time,true) && collision_time>0,
            "LIO free sphere hid an invalidated coverage trajectory");
    require(planner.checkTrajCollision(collision_time),"coverage-only live check changed default mode behavior");
    require(lio->setSingleExplorationBox(Eigen::Vector3f(-12,-6,-.2),Eigen::Vector3f(12,6,4.2)),
            "could not restore room bounds");

    CoverageFinishStatus finish;
    finish.guard_enabled=finish.plan_valid=finish.plateau_reached=finish.targets_exhausted=true;
    finish.unresolved_unknown_groups=143;
    finish.valid_voxels=35519; finish.observed_voxels=29613;
    require(finish.ready(),"residual unknown volume vetoed a converged frontier/coverage audit");
    finish.unresolved_unknown_groups=0;
    require(finish.ready(),"fine unknown voxels were mistaken for reachable frontier tasks");
    finish.observed_voxels=finish.valid_voxels;
    require(finish.ready(),"fully observed valid volume cannot converge");
    finish.plan_valid=false;
    require(!finish.ready(),"stale coverage plan produced a terminal result");
    finish.plan_valid=true; finish.plateau_reached=false;
    require(!finish.ready(),"ongoing observation progress prematurely converged");
    finish.plateau_reached=true; finish.targets_exhausted=false;
    require(finish.ready(),"untried speculative CP cleanup vetoed a converged frontier audit");
    finish.active_target_pending=true;
    require(!finish.ready(),"active coverage observation prematurely converged");

    FastExplorationManager manager;
    manager.ep_=std::make_shared<ExplorationParam>();
    manager.ep_->coverage_motion_.enabled=true;
    require(manager.coverageMotionEnabled(), "coverage policy did not activate");
    CoverageRecoveryTestAccess::run(manager,planner);
    manager.ep_->target_directed_mode_=true;
    require(!manager.coverageMotionEnabled(), "coverage policy leaked into target exploration");
    lio->setTargetNavigation(true);
    observation.goal.y()=0;
    require(!planner.planExploreTraj(path,true,false,false,{},observation),
            "coverage observation context accepted in target mode");
    std::cout << "coverage observation integration PASS distance=" << best
              << " passage_speed=" << passage_speed << " corner_distance=" << corner_distance
              << " corner_speed=" << corner_speed << '\n';
  } catch (const std::exception &e) {
    std::cerr << "coverage observation integration FAILED: " << e.what() << '\n';
    return 1;
  }
}
