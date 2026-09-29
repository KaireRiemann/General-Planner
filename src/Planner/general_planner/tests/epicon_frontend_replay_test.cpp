#include <general_core/exploration/highspeed/epicon_frontend.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <tf/tf.h>

int main(int argc,char **argv) {
  ros::init(argc,argv,"epicon_frontend_replay_test");
  ros::NodeHandle nh("~");
  try {
    if(argc<2) throw std::runtime_error("expected snapshot manifest");
    const bool shifted = std::string(argv[1])=="--shifted-box";
    if (shifted) {
      nh.setParam("box_0/down",std::vector<double>{100.0,100.0,10.0});
      nh.setParam("box_0/up",std::vector<double>{110.0,110.0,15.0});
    }
    fast_planner::EpiconFrontend frontend;
    frontend.init(nh);
    if(frontend.update(true)!=fast_planner::EpiconFrontend::Result::WAITING_MAP)
      throw std::runtime_error("frontend finished before receiving a point cloud");
    if (shifted) {
      frontend.resetTask();
      if(frontend.ready() || !frontend.tour().empty()) throw std::runtime_error("shifted-box reset retained a goal");
      std::cout<<"EPICON_SHIFTED_BOX PASS"<<std::endl;
      return 0;
    }
    std::ifstream input(argv[1]);
    if(!input) throw std::runtime_error("cannot open manifest");
    std::string pcd; int frames=0, selected=0, executable=0; size_t max_clusters=0;
    double max_update_ms=0.0;
    while(input>>pcd) {
      auto odom=boost::make_shared<nav_msgs::Odometry>();
      auto &p=odom->pose.pose.position; auto &q=odom->pose.pose.orientation; auto &v=odom->twist.twist.linear;
      input>>p.x>>p.y>>p.z>>q.x>>q.y>>q.z>>q.w>>v.x>>v.y>>v.z;
      if(!input) throw std::runtime_error("truncated snapshot manifest");
      pcl::PointCloud<pcl::PointXYZ> points;
      if(pcl::io::loadPCDFile(pcd,points)!=0) throw std::runtime_error("cannot load point cloud");
      auto cloud=boost::make_shared<sensor_msgs::PointCloud2>(); pcl::toROSMsg(points,*cloud);
      cloud->header.stamp=odom->header.stamp=ros::Time::now(); cloud->header.frame_id="world";
      const Eigen::Vector3f position(p.x,p.y,p.z);
      frontend.setOdometry(position,Eigen::Vector3f(v.x,v.y,v.z),tf::getYaw(q));
      frontend.ingestCloud(cloud,odom);
      const auto started=ros::WallTime::now();
      const auto result=frontend.update(true);
      max_update_ms=std::max(max_update_ms,(ros::WallTime::now()-started).toSec()*1000.0);
      ++frames; max_clusters=std::max(max_clusters,frontend.clusterCount());
      if(result==fast_planner::EpiconFrontend::Result::SUCCEED) {
        ++selected;
        if(frontend.tour().size()<2) throw std::runtime_error("successful tour has no goal");
        for(const auto &x:frontend.tour()) if(!x.allFinite()) throw std::runtime_error("non-finite tour");
        std::vector<Eigen::Vector3f> path;
        if(frontend.pathToGoal(path)==fast_planner::EpiconFrontend::Result::SUCCEED) {
          ++executable;
          if(path.size()<2 || (path.front()-position).norm()>.1f)
            throw std::runtime_error("local path is disconnected from odometry");
          for(size_t j=1;j<path.size();++j)
            if(!path[j].allFinite() || (path[j]-path[j-1]).norm()>1.01f)
              throw std::runtime_error("invalid local path resampling");
        }
        if(selected==1) {
          const auto revision=frontend.cloudRevision();
          frontend.ingestCloud(cloud,odom);
          if(frontend.cloudRevision()!=revision) throw std::runtime_error("duplicate scan counted as new finish evidence");
          frontend.deferCurrentGoal(30.0,100.0);
          if(frontend.hasSelectedGoal()) throw std::runtime_error("deferred goal still executable");
          if(frontend.update(true,true)!=fast_planner::EpiconFrontend::Result::BLOCKED)
            throw std::runtime_error("excluded goals were reported as coverage completion");
          if(!frontend.lastUpdateAudited()) throw std::runtime_error("audit was skipped");
          frontend.resetTask();
          if(frontend.update(true,true)!=fast_planner::EpiconFrontend::Result::SUCCEED)
            throw std::runtime_error("fresh task could not recover excluded candidates");
        }
      }
      ros::spinOnce();
    }
    frontend.setOdometry(Eigen::Vector3f(10000.f,0.f,1.f),Eigen::Vector3f::Zero(),0.f);
    if(frontend.update(true)!=fast_planner::EpiconFrontend::Result::DISCONNECTED)
      throw std::runtime_error("out-of-box odometry was accepted");
    frontend.resetTask();
    if(!frontend.tour().empty()) throw std::runtime_error("task reset retained old goal");
    std::cout<<"EPICON_REPLAY frames="<<frames<<" max_clusters="<<max_clusters<<" selected="<<selected
             <<" executable="<<executable<<" max_update_ms="<<max_update_ms<<" occupancy_map=absent"<<std::endl;
    if(frames<3 || selected<2 || executable<2) throw std::runtime_error("replay did not exercise native frontier path selection");
    return 0;
  } catch(const std::exception &error) { std::cerr<<error.what()<<std::endl; return 1; }
}
