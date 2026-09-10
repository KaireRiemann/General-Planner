#include <general_core/gate/gate_runtime.hpp>
#include <general_core/planner_runtime/planner_supervisor.hpp>
#include <general_core/planner_runtime/topology_maintenance_policy.hpp>
#include <std_msgs/Bool.h>
#include <cstdlib>
#include <iostream>
#include <set>
#include <thread>
#include <deque>

// An ideal tracking plant and injected map probe exercise the real supervisor,
// frontend, optimizer, validation and gateway without sending vehicle commands.
int main(int argc, char **argv) {
  const char *master=std::getenv("ROS_MASTER_URI");
  if (!master || std::string(master)!="http://127.0.0.1:11381") {
    std::cerr << "requires isolated ROS_MASTER_URI=http://127.0.0.1:11381\n"; return 2;
  }
  const std::string scenario=argc>1 ? argv[1] : "success";
  ros::init(argc,argv,"internal_gate_integration_test");
  ros::NodeHandle nh("~");
  nh.setParam("initial_mode","hold"); nh.setParam("serial_handover",false);
  nh.setParam("odometry_topic","/gate_test/odom");
  nh.setParam("gate/observation_timeout",scenario=="timeout" ? 1.0 : 10.0);
  nh.setParam("gate/planning_timeout",12.0);
  const bool delayed=scenario.rfind("delayed",0)==0;
  nh.setParam("gate/feedback_max_delay",delayed ? .20 : 0.0);
  nh.setParam("gate/restore_start_height",delayed);
  const bool corridor_only=delayed || scenario=="corridor_only" || scenario=="corridor_tracking_error" || scenario=="corridor_unknown";
  nh.setParam("gate/validation_policy",corridor_only ? "corridor_only" : "corridor_and_map");
  int map_probes=0;
  using namespace general_planner;
  using namespace general_planner::planner_runtime;
  PlannerCommandGateway gateway(nh);
  gate::Environment environment;
  environment.ready=[] { return true; };
  if(scenario!="unknown" && scenario!="corridor_unknown") environment.body_clear=[&](const Eigen::Vector3d &,const Eigen::Matrix3d &,const Eigen::Vector3d &) {
    if (scenario=="cancel") std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ++map_probes;
    return scenario!="blocked" && !corridor_only;
  };
  MapManager::Ptr test_map;
  if(scenario=="unknown" || scenario=="corridor_unknown") {
    auto rog=std::make_shared<rog_map::ROGMapROS>(nh,std::string(ROOT_DIR)+"tests/gate_unknown_map.yaml");
    test_map=std::make_shared<MapManager>(rog);
  }
  auto runtime=std::make_shared<gate::Runtime>(nh,gateway,test_map,environment);
  std::uint64_t revisions=0; bool topo_enabled=true;
  PlannerSupervisor supervisor(nh,gateway,[&] {
    GlobalMapStatus s; s.odom_valid=s.map_ready=s.topology_ready=true;
    s.map_revision=s.topo_revision=++revisions; return s;
  },[&](bool enabled) { topo_enabled=topo_enabled && enabled; },runtime);
  PlannerStatus latest;
  std::set<std::string> phases;
  auto status_sub=nh.subscribe<PlannerStatus>("/planner/status",20,[&](const PlannerStatusConstPtr &m) {
    latest=*m; if(m->active_mode_str=="gate") phases.insert(m->mode_state_str);
  });
  nav_msgs::Odometry odom;
  odom.header.frame_id="world"; odom.pose.pose.orientation.w=1; odom.pose.pose.position.z=1.2;
  if(delayed) odom.pose.pose.position.z=1.5;
  bool follow=true;
  std::deque<std::pair<ros::WallTime,quadrotor_msgs::PositionCommand>> commands;
  ros::WallTime last_feedback;
  auto apply_command=[&](const quadrotor_msgs::PositionCommand &m) {
    odom.pose.pose.position=m.position;
    odom.twist.twist.linear=m.velocity;
    Eigen::Quaterniond q(Eigen::AngleAxisd(m.attitude.z,Eigen::Vector3d::UnitZ())*
                         Eigen::AngleAxisd(m.attitude.y,Eigen::Vector3d::UnitY())*
                         Eigen::AngleAxisd(m.attitude.x,Eigen::Vector3d::UnitX()));
    if(scenario=="delayed_attitude" && m.position.x>1.6)
      q=Eigen::Quaterniond(Eigen::AngleAxisd(1.57079632679,Eigen::Vector3d::UnitX()));
    odom.pose.pose.orientation.w=q.w(); odom.pose.pose.orientation.x=q.x();
    odom.pose.pose.orientation.y=q.y(); odom.pose.pose.orientation.z=q.z();
    if(scenario=="delayed_cross_track" && m.position.x>1.5) odom.pose.pose.position.y+=.07;
  };
  auto command_sub=nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd",1,
      [&](const quadrotor_msgs::PositionCommandConstPtr &m) {
    if (!follow) return;
    if(delayed) commands.emplace_back(ros::WallTime::now(),*m);
    else apply_command(*m);
  });
  bool detection=false;
  auto enable_sub=nh.subscribe<std_msgs::Bool>("/polygon_hole_step_viz/detection_enable",1,
      [&](const std_msgs::BoolConstPtr &m) { detection=m->data; });
  auto odom_pub=nh.advertise<nav_msgs::Odometry>("/gate_test/odom",1);
  auto observation_pub=nh.advertise<aperture_detector::ApertureObservation>("/polygon_hole_step_viz/observation",1);
  auto request_pub=nh.advertise<std_msgs::String>("/planner/mode_request_text",1);
  auto external_pub=nh.advertise<std_msgs::String>("/planner/gate/status",1);
  aperture_detector::ApertureObservation obs;
  obs.header.frame_id="world"; obs.geometry_valid=true;
  obs.center.x=2; obs.center.z=1.2; obs.normal.x=1; obs.hole_area=1.44;
  for(const auto &yz: std::vector<std::pair<float,float>>{{-.6f,.6f},{.6f,.6f},{.6f,1.8f},{-.6f,1.8f}}) {
    geometry_msgs::Point32 p; p.x=2; p.y=yz.first; p.z=yz.second; obs.boundary.points.push_back(p);
  }
  if(delayed) {
    obs.center.x=1.896382; obs.center.y=-.293176; obs.center.z=.877086;
    obs.boundary.points.clear(); obs.hole_area=.705*.330;
    for(const auto &yz:std::vector<std::pair<float,float>>{{-.645f,.712f},{.059f,.712f},{.059f,1.042f},{-.645f,1.042f}}) {
      geometry_msgs::Point32 v;v.x=obs.center.x;v.y=yz.first;v.z=yz.second;obs.boundary.points.push_back(v);
    }
  }
  // Geometry is validated independently from the ROS observation admission.
  gate::Plan plan; gate::Config cfg; std::string reason;
  if (!gate::buildProblem(obs,{0,0,1.2},0,cfg,plan,reason)) { std::cerr<<reason; return 1; }
  auto malformed=obs; malformed.boundary.points[1].x+=.2;
  if (gate::buildProblem(malformed,{0,0,1.2},0,cfg,plan,reason)) return 1;
  auto wait=[&](const std::function<bool()> &predicate,double seconds) {
    const auto begin=ros::WallTime::now();
    while(ros::ok() && (ros::WallTime::now()-begin).toSec()<seconds) {
      const auto now=ros::WallTime::now();
      if(!delayed || last_feedback.isZero() || (now-last_feedback).toSec()>=.12) {
        while(!commands.empty() && (now-commands.front().first).toSec()>=.08) {
          if(scenario!="delayed_stall" || odom.pose.pose.position.x<.8) apply_command(commands.front().second);
          commands.pop_front();
        }
        odom.header.stamp=ros::Time::now(); odom_pub.publish(odom); last_feedback=now;
      }
      if(detection && scenario!="timeout" && scenario!="external" && scenario!="detector") {
        obs.header.stamp=ros::Time::now();
        if(scenario=="stale") obs.header.stamp-=ros::Duration(30);
        observation_pub.publish(obs);
      }
      ros::spinOnce(); if(predicate()) return true;
      ros::WallDuration(.01).sleep();
    }
    return false;
  };
  auto check=[&](bool ok,const char *label) {
    if(!ok) std::cerr << label << ": " << latest.phase_str << "/" <<latest.mode_state_str<<" "<<latest.reason<<"\n";
    return ok;
  };
  if(!check(wait([&]{return latest.ready_for_new_task && request_pub.getNumSubscribers();},5),"boot")) return 1;
  std_msgs::String request; request.data="gate"; request_pub.publish(request);
  if(!check(wait([&]{return latest.mode_state==PlannerStatus::MODE_STATE_GATE_WAIT_OBSERVATION;},5),"gate activation")) return 1;
  const auto epoch=latest.task_epoch; const auto initial_revision=latest.map_revision;
  quadrotor_msgs::PositionCommand bad;
  if(gateway.submitGateCommand(bad,epoch-1)) return 1;
  if(scenario=="external") {
    std_msgs::String ext; ext.data="START"; external_pub.publish(ext);
    wait([]{return false;},.3); ext.data="END"; external_pub.publish(ext); wait([]{return false;},.3);
    if(latest.mode_state!=PlannerStatus::MODE_STATE_GATE_WAIT_OBSERVATION || latest.command_owner!=0) return 1;
    request.data="hold"; request_pub.publish(request);
    if(!check(wait([&]{return latest.active_mode_str=="hold" && latest.ready_for_new_task;},5),"external/cancel")) return 1;
  } else if(scenario=="cancel") {
    if(!check(wait([&]{return latest.mode_state==PlannerStatus::MODE_STATE_GATE_PLANNING;},5),"planning")) return 1;
    request.data="hold"; request_pub.publish(request);
    if(!check(wait([&]{return latest.active_mode_str=="hold" && latest.ready_for_new_task;},5),"cancel")) return 1;
    wait([]{return false;},1);
    if(latest.command_owner!=0 || runtime->status().phase!=gate::Phase::IDLE || gateway.submitGateCommand(bad,epoch)) return 1;
  } else if(scenario=="blocked" || scenario=="timeout" || scenario=="stale" || scenario=="unknown") {
    if(!check(wait([&]{return latest.task_result_str=="failed";},20),"expected failure")) return 1;
    if(latest.command_owner!=0 || !gateway.publishingEnabled()) return 1;
    if(scenario=="unknown" && latest.reason.find("MAP_CENTER_NOT_FREE")==std::string::npos) return 1;
    if(scenario=="blocked" && latest.reason.rfind("MAP_COLLISION_OR_UNKNOWN",0)!=0) return 1;
  } else {
    if(!check(wait([&]{return latest.mode_state==PlannerStatus::MODE_STATE_GATE_EXECUTING || latest.task_result_str=="failed";},20),"solve")) return 1;
    if(!check(latest.task_result_str!="failed","solve rejected")) return 1;
    if(gateway.submitGateCommand(bad,epoch-1)) return 1;
    if(scenario=="tracking_error" || scenario=="corridor_tracking_error") follow=false;
    const bool expected_success=follow && scenario!="delayed_cross_track" && scenario!="delayed_stall" && scenario!="delayed_attitude";
    if(!check(wait([&]{return latest.task_result_str==(expected_success ? "succeeded" : "failed");},25),"execution")) return 1;
    if(latest.command_owner!=0 || !gateway.publishingEnabled()) return 1;
    if(!check(!expected_success || (odom.pose.pose.position.x>=3.3 && phases.count("gate_end_verify")), "measured completion")) return 1;
  }
  if(scenario=="delayed_cross_track" && latest.reason.find("GATE_LATERAL_TRACKING_ERROR")!=0) return 1;
  if(scenario=="delayed_stall" && latest.reason.find("TRACKING_ERROR")!=0) return 1;
  if(scenario=="delayed_attitude" && latest.reason.find("MEASURED_CORRIDOR_VIOLATION")!=0) return 1;
  if(!check(topo_enabled && latest.map_revision>initial_revision && shouldMaintainTopology(), "map/topology continuity")) return 1;
  if(corridor_only && map_probes!=0) { std::cerr<<"corridor_only queried voxel map"; return 1; }
  std::cout<<"internal gate "<<scenario<<" passed; map/topo provider advanced, final="<<latest.reason<<"\n";
  return 0;
}
