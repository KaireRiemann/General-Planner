#include <general_core/exploration/highspeed/epicon_frontend.h>
#include <general_core/exploration/highspeed/planner_manager.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc,char **argv) {
  ros::init(argc,argv,"epicon_backend_replay_test");
  ros::NodeHandle nh("~");
  try {
    if(argc<2) throw std::runtime_error("expected snapshot manifest");
    nh.setParam("RogMapEnable",false);
    auto lio=std::make_shared<fast_planner::LIOInterface>();
    auto search=std::make_shared<ParallelBubbleAstar>();
    auto graph=std::make_shared<TopoGraph>();
    lio->init(nh); graph->init(nh,lio,search); search->init(nh,lio);
    fast_planner::FastPlannerManager planner;
    planner.initPlanModules(nh,search,graph);
    auto frontend=std::make_shared<fast_planner::EpiconFrontend>();
    frontend->init(nh); planner.attachEpiconFrontend(frontend);
    if (std::string(argv[1])=="--recovery") {
      auto odom=boost::make_shared<nav_msgs::Odometry>();
      // Inside the configured exploration buffer, above its physical floor.
      const double start_x=planner.gcopter_config_->dilateRadiusHard-.01;
      odom->pose.pose.position.x=start_x; odom->pose.pose.position.z=1.5;
      odom->pose.pose.orientation.w=1;
      pcl::PointCloud<pcl::PointXYZ> wall;
      for (double y=-2;y<=2;y+=.05) for(double z=0;z<=3;z+=.05)
        wall.emplace_back(0,y,z);
      auto cloud=boost::make_shared<sensor_msgs::PointCloud2>(); pcl::toROSMsg(wall,*cloud);
      cloud->header.stamp=odom->header.stamp=ros::Time::now(); cloud->header.frame_id="world";
      const Eigen::Vector3d start(start_x,0,1.5);
      frontend->setOdometry(start.cast<float>(),Eigen::Vector3f::Zero(),0);
      frontend->ingestCloud(cloud,odom);
      planner.local_data_.curr_pos_=start; planner.local_data_.curr_vel_.setZero();
      planner.local_data_.curr_yaw_=0; planner.local_data_.end_yaw_=0;
      Eigen::Vector3d recovery_goal;
      const bool has_recovery=frontend->recoveryPoint(start,recovery_goal);
      std::cout<<"RECOVERY_INPUT clearance="<<frontend->clearance(start)
               <<" cloud="<<frontend->hasCloud()<<" contains="<<frontend->contains(start)
               <<" radius="<<planner.gcopter_config_->corridorRobotRadius
               <<" tolerance="<<planner.gcopter_config_->safetyClearanceTolerance
               <<" goal="<<has_recovery<<std::endl;
      if (!planner.flyToSafeRegion(true,false)) throw std::runtime_error("thin-buffer recovery could not commit");
      double collision_time=0;
      if (!planner.checkTrajCollision(collision_time)) throw std::runtime_error("live checker immediately rejected certified recovery");
      const auto goal=planner.local_data_.minco_traj_.getPos(planner.local_data_.duration_);
      if (frontend->clearance(goal)<.6 || std::abs(goal.z()-start.z())>.05)
        throw std::runtime_error("recovery did not restore clearance at the same height");
      planner.clearReleasedTrajectory(); planner.local_data_.curr_pos_=Eigen::Vector3d(.1,0,1.5);
      if (planner.flyToSafeRegion(true,false)) throw std::runtime_error("recovery bypassed physical body clearance");
      std::cout<<"EPICON_RECOVERY PASS: thin buffer escape, live certification, level goal, physical floor\n";
      return 0;
    }
    std::ifstream input(argv[1]);
    if(!input) throw std::runtime_error("cannot open manifest");
    std::string pcd; int frames=0,selected=0,success=0,replans=0,continuity=0;
    while(input>>pcd) {
      auto odom=boost::make_shared<nav_msgs::Odometry>();
      auto &p=odom->pose.pose.position; auto &q=odom->pose.pose.orientation; auto &v=odom->twist.twist.linear;
      input>>p.x>>p.y>>p.z>>q.x>>q.y>>q.z>>q.w>>v.x>>v.y>>v.z;
      if(!input) throw std::runtime_error("truncated manifest");
      pcl::PointCloud<pcl::PointXYZ> points;
      if(pcl::io::loadPCDFile(pcd,points)!=0) throw std::runtime_error("cannot load snapshot");
      auto cloud=boost::make_shared<sensor_msgs::PointCloud2>(); pcl::toROSMsg(points,*cloud);
      cloud->header.stamp=odom->header.stamp=ros::Time::now(); cloud->header.frame_id="world";
      const Eigen::Vector3f position(p.x,p.y,p.z);
      frontend->setOdometry(position,Eigen::Vector3f::Zero(),tf::getYaw(q));
      frontend->ingestCloud(cloud,odom); ++frames;
      if(frontend->update(true)!=fast_planner::EpiconFrontend::Result::SUCCEED) continue;
      ++selected; planner.clearReleasedTrajectory();
      planner.local_data_.curr_pos_=position.cast<double>(); planner.local_data_.curr_vel_.setZero();
      planner.local_data_.curr_yaw_=tf::getYaw(q); planner.local_data_.end_yaw_=frontend->goalYaw();
      const bool ok=planner.planExploreTraj(frontend->tour(),true,false,false,{},{},{},true);
      std::cout<<"EPICON_BACKEND_FRAME frame="<<frames<<" success="<<ok
               <<" failure="<<static_cast<int>(planner.coverage_failure_.kind)<<std::endl;
      if(!ok) continue;
      ++success;
      const auto old=planner.local_data_.minco_traj_;
      const auto old_yaw=planner.local_data_.minco_yaw_traj_;
      const auto start=planner.local_data_.start_time_;
      const auto duration=planner.local_data_.duration_;
      for(double t=0;t<duration;t+=.02) {
        const auto point=old.getPos(t);
        if(!frontend->contains(point) || frontend->clearance(point)<planner.gcopter_config_->dilateRadiusHard-.02)
          throw std::runtime_error("committed point-cloud trajectory violates physical clearance or task bounds");
      }
      // Same scene, a moving execution origin, real prefix composition.
      const double t=std::min(.7,duration*.2);
      const double wait=t-(ros::Time::now()-start).toSec();
      if (wait>0) ros::WallDuration(wait).sleep();
      const double execution_time=std::clamp((ros::Time::now()-start).toSec(),0.0,duration);
      planner.local_data_.curr_pos_=old.getPos(execution_time);
      planner.local_data_.curr_vel_=old.getVel(execution_time);
      frontend->setOdometry(planner.local_data_.curr_pos_.cast<float>(),planner.local_data_.curr_vel_.cast<float>(),tf::getYaw(q));
      ++replans;
      if(planner.planExploreTraj(frontend->tour(),false,false,false,{},{},{},true)) {
        ++continuity;
        const double offset=(planner.local_data_.start_time_-start).toSec();
        if ((old.getPos(offset)-planner.local_data_.minco_traj_.getPos(0)).norm()>1e-5 ||
            (old.getVel(offset)-planner.local_data_.minco_traj_.getVel(0)).norm()>1e-4 ||
            (old.getAcc(offset)-planner.local_data_.minco_traj_.getAcc(0)).norm()>1e-3 ||
            (old_yaw.getPos(offset)-planner.local_data_.minco_yaw_traj_.getPos(0)).norm()>1e-5 ||
            (old_yaw.getVel(offset)-planner.local_data_.minco_yaw_traj_.getVel(0)).norm()>1e-4)
          throw std::runtime_error("position/velocity/acceleration/yaw discontinuity at command replacement");
      }
    }
    std::cout<<"EPICON_BACKEND_REPLAY frames="<<frames<<" selected="<<selected<<" success="<<success
             <<" replans="<<replans<<" continuous="<<continuity<<" occupancy_map=absent"<<std::endl;
    if(selected<3 || success<3 || success<.75*selected || continuity<2 || continuity<.85*replans)
      throw std::runtime_error("point-cloud backend replay success/continuity below regression threshold");
    return 0;
  } catch(const std::exception &error) { std::cerr<<error.what()<<std::endl; return 1; }
}
