#include <general_core/exploration/highspeed/epicon_frontend.h>
#include <epicon_native/pointcloud_topo/parallel_bubble_astar.h>
#include <epicon_native/pointcloud_topo/graph.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <tf/tf.h>
#include <std_msgs/String.h>

int main(int argc,char **argv) {
  ros::init(argc,argv,"epicon_frontend_replay_test");
  ros::NodeHandle nh("~");
  try {
    if(argc<2) throw std::runtime_error("expected snapshot manifest");
    const bool diagnose = std::string(argv[1])=="--diagnose";
    if (diagnose && argc<3) throw std::runtime_error("--diagnose requires a snapshot manifest");
    std::string status_json = "{}";
    auto status_sub = nh.subscribe<std_msgs::String>("/planning/exploration/epicon_status", 1,
        [&](const std_msgs::String::ConstPtr &message) { status_json=message->data; });
    const bool shifted = std::string(argv[1])=="--shifted-box";
    if (shifted) {
      nh.setParam("box_0/down",std::vector<double>{100.0,100.0,10.0});
      nh.setParam("box_0/up",std::vector<double>{110.0,110.0,15.0});
    }
    fast_planner::EpiconFrontend frontend;
    frontend.init(nh);
    if(frontend.update(true)!=fast_planner::EpiconFrontend::Result::WAITING_MAP)
      throw std::runtime_error("frontend finished before receiving a point cloud");
    if (std::string(argv[1])=="--clearance") {
      ros::NodeHandle native(nh,"epicon");
      auto map=std::make_shared<epicon_native::fast_planner::LIOInterface>();
      map->init(native);
      auto search_ptr=std::make_shared<epicon_native::ParallelBubbleAstar>();
      auto &search=*search_ptr; search.init(native,map);
      auto odom=boost::make_shared<nav_msgs::Odometry>();
      odom->pose.pose.position.x=.65; odom->pose.pose.position.y=-.8;
      odom->pose.pose.position.z=1.5; odom->pose.pose.orientation.w=1;
      pcl::PointCloud<pcl::PointXYZ> walls;
      for(double z=0;z<=3;z+=.05) {
        for(double y=-3;y<=3;y+=.05) walls.emplace_back(0,y,z);
        for(double x=0;x<=1.3;x+=.05) walls.emplace_back(x,0,z);
      }
      auto cloud=boost::make_shared<sensor_msgs::PointCloud2>(); pcl::toROSMsg(walls,*cloud);
      cloud->header.stamp=odom->header.stamp=ros::Time::now(); cloud->header.frame_id="world";
      map->updateCloudMapOdometry(cloud,odom);
      std::vector<Eigen::Vector3f> path;
      const Eigen::Vector3f start(.65,-.8,1.5),goal(.65,.8,1.5);
      if(search.search(start,goal,path,.5,true)==epicon_native::ParallelBubbleAstar::REACH_END &&
          search.collisionCheck_shortenPath(path)) {
        for(const auto &point:path) if(map->getDisToOcc(point)<.6)
          throw std::runtime_error("detour violated physical search clearance");
      } else throw std::runtime_error("65 cm root could not take a safe detour around a wall");
      if(search.search(Eigen::Vector3f(.3,-.8,1.5),goal,path,.1,true)==epicon_native::ParallelBubbleAstar::REACH_END)
        throw std::runtime_error("sub-clearance root was accepted");
      if(std::abs(frontend.planningClearance()-.6)>1e-6 ||
         std::abs(frontend.executionClearance()-.48)>1e-6)
        throw std::runtime_error("planning and execution clearances were conflated");
      epicon_native::TopoGraph graph; graph.init(native,map,search_ptr);
      auto node=std::make_shared<epicon_native::TopoNode>();
      node->center_=Eigen::Vector3f(2.5,-.8,1.5);
      Eigen::Vector3i idx; graph.getIndex(node->center_,idx);
      auto region=graph.getRegionNode(idx);
      if(!region) throw std::runtime_error("missing synthetic graph region");
      region->topo_nodes_.insert(node);
      Eigen::Vector3f root(.8,-.8,1.5); float yaw=0;
      graph.updateOdomNode(root,yaw);
      if(!graph.odom_node_->neighbors_.count(node)) throw std::runtime_error("initial root connection failed");
      root.x()=1.; graph.updateOdomNode(root,yaw);
      if(graph.odom_reused_connections_!=1 ||
         (graph.odom_node_->paths_.at(node).front()-root).norm()>1e-6)
        throw std::runtime_error("root path was not recertified from current odometry");
      walls.emplace_back(node->center_.x(),node->center_.y(),node->center_.z());
      pcl::toROSMsg(walls,*cloud); cloud->header.stamp=ros::Time::now();
      map->updateCloudMapOdometry(cloud,odom);
      graph.updateOdomNode(root,yaw);
      if(!graph.odom_node_->neighbors_.empty() || !graph.odom_node_->paths_.empty())
        throw std::runtime_error("stale root edge survived a new obstacle");
      // The endpoints are safe, but a 2 cm gap must not be crossed by a
      // direct connector, grid edge, or path-shortening shortcut.
      map->lp_->box_num_=2;
      map->lp_->global_box_min_boundary_vec_={Eigen::Vector3f(-2,-3,0),Eigen::Vector3f(-2,.01,0)};
      map->lp_->global_box_max_boundary_vec_={Eigen::Vector3f(4,-.01,3),Eigen::Vector3f(4,3,3)};
      const Eigen::Vector3f left(3.5,-.6,1.5),right(3.5,.6,1.5);
      path={left,right};
      if(search.collisionCheck_shortenPath(path) ||
         search.search(left,right,path,.1,true)==epicon_native::ParallelBubbleAstar::REACH_END)
        throw std::runtime_error("connector crossed a gap between task boxes");
      map->lp_->global_box_max_boundary_vec_[0].y()=.1;
      if(search.search(left,right,path,.1,true)!=epicon_native::ParallelBubbleAstar::REACH_END)
        throw std::runtime_error("overlapping task boxes were treated as disconnected");
      std::cout<<"EPICON_CLEARANCE PASS: near-wall detour, unsafe-root rejection, clearance policy, root reuse/invalidation, box union\n";
      return 0;
    }
    if (shifted) {
      frontend.resetTask();
      if(frontend.ready() || !frontend.tour().empty()) throw std::runtime_error("shifted-box reset retained a goal");
      std::cout<<"EPICON_SHIFTED_BOX PASS"<<std::endl;
      return 0;
    }
    std::ifstream input(argv[diagnose ? 2 : 1]);
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
      const auto result=frontend.update(true,diagnose);
      const double update_ms=(ros::WallTime::now()-started).toSec()*1000.0;
      max_update_ms=std::max(max_update_ms,update_ms);
      bool frame_executable=false;
      ++frames; max_clusters=std::max(max_clusters,frontend.clusterCount());
      if(result==fast_planner::EpiconFrontend::Result::SUCCEED) {
        ++selected;
        if(frontend.tour().size()<2) throw std::runtime_error("successful tour has no goal");
        for(const auto &x:frontend.tour()) if(!x.allFinite()) throw std::runtime_error("non-finite tour");
        std::vector<Eigen::Vector3f> path;
        if(frontend.pathToGoal(path)==fast_planner::EpiconFrontend::Result::SUCCEED) {
          ++executable;
          frame_executable=true;
          if(path.size()<2 || (path.front()-position).norm()>.1f)
            throw std::runtime_error("local path is disconnected from odometry");
          for(size_t j=1;j<path.size();++j)
            if(!path[j].allFinite() || (path[j]-path[j-1]).norm()>1.01f)
              throw std::runtime_error("invalid local path resampling");
          for(size_t j=1;j<path.size();++j) for(int k=0;k<=20;++k)
            if(!frontend.contains((path[j-1]+(path[j]-path[j-1])*(float(k)/20)).cast<double>()))
              throw std::runtime_error("topological path left the task-box union");
          if(diagnose && executable==1) {
            std::vector<Eigen::Vector3d> guide;
            for(const auto &point:path) guide.push_back(point.cast<double>());
            fast_planner::EpiconCorridor corridor;
            const bool bounded=frontend.corridor(guide,3.5,frontend.planningClearance(),corridor);
            std::cout<<"EPICON_GEOMETRY bounded="<<bounded<<" truncated="<<corridor.truncated
                     <<" failure="<<corridor.failure<<" guide=";
            for(const auto &point:guide) std::cout<<" ["<<point.transpose()<<"]";
            std::cout<<" corridor_path=";
            for(const auto &point:corridor.path) std::cout<<" ["<<point.transpose()<<"]";
            std::cout<<std::endl;
          }
        }
        if(selected==1 && !diagnose) {
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
      if (diagnose) ros::WallDuration(.01).sleep();
      ros::spinOnce();
      if (diagnose) std::cout<<"EPICON_FRAME {\"frame\":"<<frames
          <<",\"update_ms\":"<<update_ms<<",\"executable\":"<<(frame_executable ? "true" : "false")
          <<",\"status\":"<<status_json<<"}"<<std::endl;
    }
    frontend.setOdometry(Eigen::Vector3f(10000.f,0.f,1.f),Eigen::Vector3f::Zero(),0.f);
    if(frontend.update(true)!=fast_planner::EpiconFrontend::Result::DISCONNECTED)
      throw std::runtime_error("out-of-box odometry was accepted");
    frontend.resetTask();
    if(!frontend.tour().empty()) throw std::runtime_error("task reset retained old goal");
    std::cout<<"EPICON_REPLAY frames="<<frames<<" max_clusters="<<max_clusters<<" selected="<<selected
             <<" executable="<<executable<<" max_update_ms="<<max_update_ms<<" occupancy_map=absent"<<std::endl;
    if(frames<3 || (!diagnose && (selected<2 || executable<2)))
      throw std::runtime_error("replay did not exercise native frontier path selection");
    return 0;
  } catch(const std::exception &error) { std::cerr<<error.what()<<std::endl; return 1; }
}
