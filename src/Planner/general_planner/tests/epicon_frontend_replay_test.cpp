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
    const bool cache_test=std::string(argv[1])=="--cache-invalidation";
    if(cache_test && argc<3) throw std::runtime_error("cache test requires manifest");
    const bool incremental = std::string(argv[1])=="--incremental";
    const bool diagnose = std::string(argv[1])=="--diagnose" || incremental;
    if (diagnose && argc<3) throw std::runtime_error("--diagnose requires a snapshot manifest");
    std::string status_json = "{}";
    auto status_sub = nh.subscribe<std_msgs::String>("/planning/exploration/epicon_status", 1,
        [&](const std_msgs::String::ConstPtr &message) { status_json=message->data; });
    const bool shifted = std::string(argv[1])=="--shifted-box";
    if (shifted) {
      nh.setParam("box_0/down",std::vector<double>{100.0,100.0,10.0});
      nh.setParam("box_0/up",std::vector<double>{110.0,110.0,15.0});
    }
    if(std::string(argv[1])=="--invalid-policy") {
      nh.setParam("DilateRadiusHard",.6);nh.setParam("DilateRadiusSoft",.5);
      bool rejected=false;
      try { fast_planner::EpiconFrontend invalid; invalid.init(nh); }
      catch(const std::invalid_argument &) { rejected=true; }
      if(!rejected) throw std::runtime_error("soft below physical hard margin was silently accepted");
      std::cout<<"INVALID_POLICY PASS"<<std::endl;return 0;
    }
    if(std::string(argv[1])=="--parameter-inheritance") {
      nh.deleteParam("epicon");
      nh.setParam("box_num",1);
      nh.setParam("box_0/down",std::vector<double>{-10,-10,-1});
      nh.setParam("box_0/up",std::vector<double>{10,10,8});
      nh.setParam("view_graph",false);
      nh.setParam("max_update_region_num",32);
      nh.setParam("bubble_astar/safe_distance",.45);
      nh.setParam("bubble_astar/resolution_astar",.25);
      nh.setParam("epicon/bubble_astar/resolution_astar",.15);
      nh.setParam("lidar_perception/fov_up",52.);
      nh.setParam("lidar_perception/fov_down",-7.);
      nh.setParam("epicon/lidar_perception/fov_up",48.);
      nh.setParam("DilateRadiusHard",.45);
      nh.setParam("DilateRadiusSoft",.6);
      nh.setParam("CommitKnownFreeSafeDistance",.45);
      fast_planner::EpiconFrontend configured; configured.init(nh);
      bool graph=true; int budget=0; double clearance=0,resolution=0,up=0,down=0;
      nh.getParam("epicon/view_graph",graph);
      nh.getParam("epicon/max_update_region_num",budget);
      nh.getParam("epicon/bubble_astar/safe_distance",clearance);
      nh.getParam("epicon/bubble_astar/resolution_astar",resolution);
      nh.getParam("epicon/lidar_perception/fov_up",up);
      nh.getParam("epicon/lidar_perception/fov_down",down);
      if(graph || budget!=32 || std::abs(clearance-.45)>1e-6 ||
         std::abs(resolution-.15)>1e-6 || up!=48 || down!=-7)
        throw std::runtime_error("scene/native leaf precedence failed");
      std::cout<<"PARAMETER_INHERITANCE PASS"<<std::endl; return 0;
    }
    fast_planner::EpiconFrontend frontend;
    frontend.init(nh);
    if(frontend.update(true)!=fast_planner::EpiconFrontend::Result::WAITING_MAP)
      throw std::runtime_error("frontend finished before receiving a point cloud");
    if(std::string(argv[1])=="--door-check") {
      if(argc<3) throw std::runtime_error("--door-check requires PCD");
      ros::NodeHandle native(nh,"epicon");
      auto map=std::make_shared<epicon_native::fast_planner::LIOInterface>(); map->init(native);
      auto search=std::make_shared<epicon_native::ParallelBubbleAstar>(); search->init(native,map);
      pcl::PointCloud<pcl::PointXYZ> points;
      if(pcl::io::loadPCDFile(argv[2],points)!=0) throw std::runtime_error("PCD load failed");
      auto cloud=boost::make_shared<sensor_msgs::PointCloud2>();pcl::toROSMsg(points,*cloud);
      auto odom=boost::make_shared<nav_msgs::Odometry>(); odom->pose.pose.position.x=31;
      odom->pose.pose.position.z=1.93;odom->pose.pose.orientation.w=1;
      cloud->header.stamp=odom->header.stamp=ros::Time::now();map->updateCloudMapOdometry(cloud,odom);
      frontend.ingestCloud(cloud,odom);
      for(double margin:{.6,.505}) {
        fast_planner::EpiconCorridor corridor;
        const bool ok=frontend.corridor({Eigen::Vector3d(31,0,1.93),Eigen::Vector3d(33,0,1.93)},3.5,margin,corridor);
        std::cout<<"DOOR_CORRIDOR margin="<<margin<<" success="<<ok<<" truncated="<<corridor.truncated
                 <<" failure="<<corridor.failure<<std::endl;
        if(margin<.6 && (!ok || corridor.truncated)) throw std::runtime_error("hard-safe doorway corridor failed");
      }
      for(double goal_y:{0.,.4}) for(double clearance:{.6,.5}) for(double resolution:{.2,.05}) for(double budget:{.01,.1,1.0}) {
        search->safe_distance_=clearance;search->resolution_=resolution;search->inv_resolution_=1/resolution;
        std::vector<Eigen::Vector3f> path;
        const auto begin=ros::WallTime::now();
        const int result=search->search(Eigen::Vector3f(31,0,1.93),Eigen::Vector3f(33,goal_y,1.93),path,budget,true);
        double minimum=100;
        for(size_t n=1;n<path.size();++n) for(int k=0;k<=100;++k)
          minimum=std::min(minimum,double(map->getDisToOcc((path[n-1]+(path[n]-path[n-1])*(float(k)/100)).eval())));
        std::cout<<"DOOR_CHECK goal_y="<<goal_y<<" clearance="<<clearance<<" resolution="<<resolution<<" budget="<<budget
                 <<" result="<<result<<" ms="<<(ros::WallTime::now()-begin).toSec()*1000
                 <<" min_clearance="<<minimum<<" points="<<path.size()<<std::endl;
        if(result==epicon_native::ParallelBubbleAstar::REACH_END && minimum<clearance-1e-4)
          throw std::runtime_error("door path violated requested clearance");
      }
      return 0;
    }
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
      auto viewpoint=std::make_shared<epicon_native::TopoNode>(); viewpoint->center_=goal;
      if(graph.connectViewpointToRoot(viewpoint,.5)!=epicon_native::ParallelBubbleAstar::REACH_END)
        throw std::runtime_error("viewpoint retry could not connect around the wall");
      const auto certified=graph.odom_node_->paths_.at(viewpoint);
      if(certified.size()<3 || (certified.front()-root).norm()>1e-6 ||
         (certified.back()-goal).norm()>1e-6)
        throw std::runtime_error("viewpoint connector did not preserve its detour");
      graph.removeNode(viewpoint);
      if(graph.odom_node_->neighbors_.count(viewpoint)) throw std::runtime_error("temporary viewpoint leaked root edge");
      viewpoint->center_=Eigen::Vector3f(.3,-.8,1.5);
      if(graph.connectViewpointToRoot(viewpoint,.1)==epicon_native::ParallelBubbleAstar::REACH_END)
        throw std::runtime_error("viewpoint retry accepted insufficient clearance");
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
      // A 1.21 m corridor has >0.6 m clearance on its centreline, but
      // endpoint free spheres cannot overlap across a 0.2 m grid edge.
      auto narrow_map=std::make_shared<epicon_native::fast_planner::LIOInterface>();
      narrow_map->init(native);
      epicon_native::ParallelBubbleAstar narrow_search;narrow_search.init(native,narrow_map);
      pcl::PointCloud<pcl::PointXYZ> corridor_walls;
      for(double z=0;z<=3;z+=.04) for(double y=-2;y<=2;y+=.04) {
        corridor_walls.emplace_back(0,y,z);corridor_walls.emplace_back(1.21,y,z);
      }
      pcl::toROSMsg(corridor_walls,*cloud);cloud->header.stamp=ros::Time::now();
      narrow_map->updateCloudMapOdometry(cloud,odom);
      if(narrow_search.search(Eigen::Vector3f(.605,-.8,1.5),Eigen::Vector3f(.605,.8,1.5),path,.1,true)!=
         epicon_native::ParallelBubbleAstar::REACH_END || !narrow_search.collisionCheck_shortenPath(path))
        throw std::runtime_error("safe narrow corridor rejected by endpoint sphere overlap");
      for(int k=0;k<=200;++k) {
        const Eigen::Vector3f point(.605,-.8+1.6*k/200.,1.5);
        if(narrow_map->getDisToOcc(point)<.6) throw std::runtime_error("narrow test violates clearance");
      }
      path={Eigen::Vector3f(-.7,0,1.5),Eigen::Vector3f(.605,0,1.5)};
      if(narrow_search.collisionCheck_shortenPath(path))
        throw std::runtime_error("segment refinement allowed crossing a wall");
      std::cout<<"EPICON_CLEARANCE PASS: near-wall detour, unsafe-root rejection, clearance policy, root reuse/invalidation, box union\n";
      return 0;
    }
    if (shifted) {
      frontend.resetTask();
      if(frontend.ready() || !frontend.tour().empty()) throw std::runtime_error("shifted-box reset retained a goal");
      std::cout<<"EPICON_SHIFTED_BOX PASS"<<std::endl;
      return 0;
    }
    std::ifstream input(argv[(diagnose || cache_test) ? 2 : 1]);
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
      if(incremental) frontend.update(false,false);
      const auto result=frontend.update(true,diagnose && (!incremental || frames%4==0));
      if(result==fast_planner::EpiconFrontend::Result::NO_FRONTIER && frontend.clusterCount()>0)
        throw std::runtime_error("unresolved frontier was treated as coverage completion");
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
          if(cache_test) {
            std::vector<Eigen::Vector3f> moving;
            if(frontend.pathFrom(position.cast<double>(),Eigen::Vector3d::Zero(),tf::getYaw(q),moving)!=
                fast_planner::EpiconFrontend::Result::SUCCEED || moving.size()<2)
              throw std::runtime_error("reanchoring lost the selected route");
            const Eigen::Vector3f obstacle=(moving.front()+moving[1])*.5f;
            points.emplace_back(obstacle.x(),obstacle.y(),obstacle.z());
            pcl::toROSMsg(points,*cloud);cloud->header.frame_id="world";
            cloud->header.stamp=ros::Time::now()+ros::Duration(.001);
            frontend.ingestCloud(cloud,odom);
            if(frontend.clearance(obstacle.cast<double>())>.05)
              throw std::runtime_error("cache test obstacle was not inserted into the map");
            std::vector<Eigen::Vector3f> replanned;
            const auto changed=frontend.pathFrom(position.cast<double>(),Eigen::Vector3d::Zero(),tf::getYaw(q),replanned);
            if(changed==fast_planner::EpiconFrontend::Result::SUCCEED) {
              for(size_t n=1;n<replanned.size();++n) for(int k=0;k<=100;++k) {
                const Eigen::Vector3d point=(replanned[n-1]+(replanned[n]-replanned[n-1])*(float(k)/100)).cast<double>();
                if(frontend.clearance(point)<.49)
                  throw std::runtime_error("cached route bypassed a newly observed obstacle");
              }
            }
            frontend.resetTask();
            if(frontend.hasSelectedGoal() || !frontend.executionPath().empty())
              throw std::runtime_error("route cache survived task reset");
            std::cout<<"CACHE_INVALIDATION PASS: reanchor, new obstacle, task reset"<<std::endl;
            return 0;
          }
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
    if(frontend.update(true)!=fast_planner::EpiconFrontend::Result::OUTSIDE_BOX)
      throw std::runtime_error("out-of-box odometry was not distinguished from topology disconnection");
    if(frontend.hasSelectedGoal() || !frontend.tour().empty())
      throw std::runtime_error("out-of-box odometry retained a selected goal");
    std::vector<Eigen::Vector3f> outside_path;
    if(frontend.pathFrom(Eigen::Vector3d(10000,0,1),Eigen::Vector3d::Zero(),0,outside_path)!=
        fast_planner::EpiconFrontend::Result::OUTSIDE_BOX)
      throw std::runtime_error("out-of-box execution head lost its failure reason");
    frontend.resetTask();
    if(!frontend.tour().empty()) throw std::runtime_error("task reset retained old goal");
    std::cout<<"EPICON_REPLAY frames="<<frames<<" max_clusters="<<max_clusters<<" selected="<<selected
             <<" executable="<<executable<<" max_update_ms="<<max_update_ms<<" occupancy_map=absent"<<std::endl;
    if(frames<3 || (!diagnose && (selected<2 || executable<2)))
      throw std::runtime_error("replay did not exercise native frontier path selection");
    return 0;
  } catch(const std::exception &error) { std::cerr<<error.what()<<std::endl; return 1; }
}
