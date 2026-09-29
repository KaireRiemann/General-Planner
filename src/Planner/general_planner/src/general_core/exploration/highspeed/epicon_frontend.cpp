#include <general_core/exploration/highspeed/epicon_frontend.h>
#include <epicon_native/epicon_planner/fast_exploration_manager.h>
#include <epicon_native/epicon_planner/expl_data.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdexcept>
#include <pcl/filters/voxel_grid.h>

namespace fast_planner {
namespace {
template <typename T> void defaultParam(ros::NodeHandle &nh, const std::string &key, const T &value) {
  if (!nh.hasParam(key)) nh.setParam(key, value);
}
void configureNativeParameters(ros::NodeHandle &parent, ros::NodeHandle &nh) {
  // EPICON unity.yaml exploration defaults. Scene and sensor geometry come
  // from the existing runtime; native algorithm overrides live under epicon/.
  defaultParam(nh, "global_planning/w_vdir", 1.2);
  defaultParam(nh, "global_planning/w_yawdir", 0.0);
  defaultParam(nh, "global_planning/w_firstnode", 0.2);
  defaultParam(nh, "view_graph", true);
  defaultParam(nh, "FrontierManager/view_cluster", true);
  defaultParam(nh, "FrontierManager/view_frt", true);
  defaultParam(nh, "bubble_topo/min_x", 0.6);
  defaultParam(nh, "bubble_topo/min_y", 0.6);
  defaultParam(nh, "bubble_topo/min_z", 0.3);
  defaultParam(nh, "bubble_topo/bubble_min_radius", 0.61);
  defaultParam(nh, "bubble_topo/init_region_size_x", 5.0);
  defaultParam(nh, "bubble_topo/init_region_size_y", 5.0);
  defaultParam(nh, "bubble_topo/init_region_size_z", 4.0);
  defaultParam(nh, "bubble_topo/cube_discrete_size", 0.25);
  defaultParam(nh, "bubble_topo/odom_node_distance", 3.0);
  defaultParam(nh, "bubble_astar/safe_distance", 0.6);
  defaultParam(nh, "max_update_region_num", 99999999);
  defaultParam(nh, "lidar_perception/fov_up", 30.0);
  defaultParam(nh, "lidar_perception/fov_down", -30.0);
  defaultParam(nh, "lidar_perception/lidar_pitch", 0.0);
  defaultParam(nh, "lidar_perception/fov_viewpoint_up", 28.0);
  defaultParam(nh, "lidar_perception/fov_viewpoint_down", -28.0);
  defaultParam(nh, "lidar_perception/max_ray_length", 60);
  defaultParam(nh, "parallel_astar/update_connection_timeout", 0.002);
  defaultParam(nh, "parallel_astar/insert_node_timeout", 0.002);
  defaultParam(nh, "FrontierManager/cell_size", 0.4);
  defaultParam(nh, "FrontierManager/noise_cell_range", 1);
  defaultParam(nh, "FrontierManager/good_observation_direction_score", 0.8);
  defaultParam(nh, "FrontierManager/good_observation_force_trust_length", 2.0);
  defaultParam(nh, "FrontierManager/good_observation_trust_length", 6.0);
  defaultParam(nh, "FrontierManager/update_length", 15.0);
  defaultParam(nh, "FrontierManager/cluster_max_size", 7.5);
  defaultParam(nh, "FrontierManager/cluster_min_size", 0.8);
  defaultParam(nh, "FrontierManager/cluster_min_radius", 1.0);
  defaultParam(nh, "FrontierManager/cluster_direction_radius", 0.15);
  defaultParam(nh, "FrontierManager/cluster_minmum_point_num", 10);
  defaultParam(nh, "ViewpointManager/sample_pillar_min_height", -1.2);
  defaultParam(nh, "ViewpointManager/sample_pillar_max_height", 1.8);
  defaultParam(nh, "ViewpointManager/sample_pillar_height_layer_num", 5);
  defaultParam(nh, "ViewpointManager/sample_pillar_min_radius", 0.8);
  defaultParam(nh, "ViewpointManager/sample_pillar_max_radius", 3.2);
  defaultParam(nh, "ViewpointManager/sample_pillar_radius_layer_num", 6);
  defaultParam(nh, "ViewpointManager/sample_pillar_circle_sample_num", 16);
  defaultParam(nh, "ViewpointManager/local_tsp_size", 20);
  defaultParam(nh, "bubble_astar/allocate_num", 1000000);
  defaultParam(nh, "bubble_astar/lambda_heu", 100.0);
  defaultParam(nh, "bubble_astar/debug", false);
  defaultParam(nh, "bubble_astar/resolution_astar", 0.2);
  defaultParam(nh, "viewpoint_param/global_viewpoint_num", 16);
  defaultParam(nh, "viewpoint_param/local_viewpoint_num", 8);
  defaultParam(nh, "ViewpointManager/consider_range", 30);
  defaultParam(nh, "ViewpointManager/global_recluster_size", 20);
  defaultParam(nh, "fsm/vaild_num_cube", 10);
  defaultParam(nh, "fsm/sample_traj_point", 0.1);
  defaultParam(nh, "fsm/hybrid_search_radius", 15.0);
  defaultParam(nh, "fsm/unknown_penalty_factor", 1.5);
  const std::vector<std::string> shared = {"box_num", "dead_area_num", "odometry_topic", "cloud_topic",
      "lidar_perception", "MaxVelMag", "MaxTiltAngle", "GravAcc", "yaw_max_vel"};
  for (const auto &key : shared) {
    XmlRpc::XmlRpcValue value;
    if (parent.getParam(key, value)) nh.setParam(key, value);
  }
  int boxes = 0, dead = 0;
  parent.getParam("box_num", boxes); parent.param("dead_area_num", dead, 0);
  if (boxes < 1) throw std::runtime_error("point-cloud exploration requires an exploration box");
  for (const auto &entry : std::vector<std::pair<std::string,int>>{{"box_",boxes},{"dead_",dead}}) {
    for (int i=0;i<entry.second;++i) {
      const auto key=entry.first+std::to_string(i);
      XmlRpc::XmlRpcValue value;
      if (!parent.getParam(key,value)) throw std::runtime_error("point-cloud exploration missing " + key);
      for (const auto *bound : {"down","up"}) {
        if (!value.hasMember(bound) || value[bound].getType()!=XmlRpc::XmlRpcValue::TypeArray || value[bound].size()!=3)
          throw std::runtime_error("point-cloud exploration invalid box " + key);
      }
      nh.setParam(key,value);
    }
  }
}
}
struct EpiconFrontend::Impl {
  epicon_native::fast_planner::FastPlannerManager::Ptr planner;
  epicon_native::fast_planner::FastExplorationManager::Ptr explorer;
  epicon_native::FrontierManager::Ptr frontier;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> new_path;
  Eigen::Vector3f pos{Eigen::Vector3f::Zero()}, vel{Eigen::Vector3f::Zero()};
  float yaw{0.0f};
  bool cloud_ready{false};
  bool selected{false}, audited{false};
  std::uint64_t cloud_revision{0}, topology_revision{0};
  ros::Time cloud_stamp;
  bool path_sampled{false};
  Eigen::Vector3f last_path_sample{Eigen::Vector3f::Zero()};
  using Points=std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>>;
  Points unknown, active;
  struct DeferredGoal { Eigen::Vector3f position; ros::WallTime until; double radius; };
  std::vector<DeferredGoal> deferred;
  void clearGoal() {
    planner->topo_graph_->removeNode(explorer->ed_->next_goal_node_);
    explorer->ed_->next_goal_node_=std::make_shared<epicon_native::TopoNode>();
    explorer->ed_->global_tour_.clear(); explorer->ed_->path_next_goal_.clear();
    explorer->last_goal_cost_=5e3; selected=false;
  }
  int valid_cubes{10};
  double sample_path{0.1}, hybrid_radius{15.0}, unknown_penalty{1.5};
  ros::Publisher status;
  void publish(const char *result, size_t unknown, size_t active) {
    std_msgs::String msg;
    std::ostringstream out;
    out << "{\"backend\":\"epicon_pointcloud\",\"result\":\"" << result
        << "\",\"clusters\":" << frontier->cluster_list_.size()
        << ",\"history_nodes\":" << planner->topo_graph_->history_odom_nodes_.size()
        << ",\"unknown_nodes\":" << unknown << ",\"active_nodes\":" << active
        << ",\"tour_size\":" << explorer->ed_->global_tour_.size()
        << ",\"cloud_revision\":" << cloud_revision
        << ",\"cloud_stamp\":" << std::fixed << cloud_stamp.toSec()
        << ",\"audited\":" << (audited ? "true" : "false")
        << ",\"selected\":" << (selected ? "true" : "false")
        << ",\"deferred_goals\":" << deferred.size()
        << ",\"geometry_rejected\":" << frontier->geometry_rejected_
        << ",\"connectivity_rejected\":" << frontier->connectivity_rejected_
        << ",\"unobservable_clusters\":" << frontier->unobservable_clusters_ << "}";
    msg.data=out.str(); status.publish(msg);
  }
};
EpiconFrontend::EpiconFrontend() : impl_(new Impl) {}
EpiconFrontend::~EpiconFrontend() = default;
void EpiconFrontend::init(ros::NodeHandle &runtime_nh) {
  ros::NodeHandle nh(runtime_nh,"epicon");
  configureNativeParameters(runtime_nh,nh);
  auto &i=*impl_;
  i.planner=std::make_shared<epicon_native::fast_planner::FastPlannerManager>();
  auto &p=*i.planner;
  p.lidar_map_interface_=std::make_shared<epicon_native::fast_planner::LIOInterface>();
  p.lidar_map_interface_->init(nh);
  p.parallel_path_finder_=std::make_shared<epicon_native::ParallelBubbleAstar>();
  p.parallel_path_finder_->init(nh,p.lidar_map_interface_);
  p.topo_graph_=std::make_shared<epicon_native::TopoGraph>();
  p.topo_graph_->init(nh,p.lidar_map_interface_,p.parallel_path_finder_);
  p.bubble_path_finder_=std::make_shared<epicon_native::fast_planner::BubbleAstar>();
  p.bubble_path_finder_->init(nh,p.lidar_map_interface_);
  p.fast_searcher_=std::make_shared<epicon_native::fast_planner::FastSearcher>();
  p.fast_searcher_->init(p.topo_graph_,p.bubble_path_finder_);
  p.graph_visualizer_=std::make_shared<epicon_native::GraphVisualizer>();
  p.graph_visualizer_->init(nh);
  nh.getParam("MaxVelMag",p.gcopter_config_->maxVelMag);
  nh.getParam("MaxTiltAngle",p.gcopter_config_->maxTiltAngle);
  nh.getParam("GravAcc",p.gcopter_config_->gravAcc);
  nh.getParam("yaw_max_vel",p.gcopter_config_->yaw_max_vel);
  i.frontier=std::make_shared<epicon_native::FrontierManager>();
  i.frontier->init(nh,p.lidar_map_interface_,p.topo_graph_);
  i.frontier->viewpoint_allowed_=[&i](const Eigen::Vector3f &point) {
    for (const auto &goal:i.deferred)
      if ((point-goal.position).norm()<goal.radius) return false;
    return true;
  };
  i.explorer=std::make_shared<epicon_native::fast_planner::FastExplorationManager>();
  i.explorer->initialize(nh,i.frontier,i.planner);
  nh.getParam("fsm/vaild_num_cube",i.valid_cubes);
  nh.getParam("fsm/sample_traj_point",i.sample_path);
  nh.getParam("fsm/hybrid_search_radius",i.hybrid_radius);
  nh.getParam("fsm/unknown_penalty_factor",i.unknown_penalty);
  i.status=runtime_nh.advertise<std_msgs::String>("/planning/exploration/epicon_status",1,true);
  runtime_nh.setParam("exploration/frontend",std::string("epicon_pointcloud"));
  ROS_INFO("[exploration] native point-cloud frontier/topology/tour enabled; backend=ExplorationCostManager+ExplorationTrajOpt");
}
void EpiconFrontend::ingestCloud(const sensor_msgs::PointCloud2ConstPtr &cloud, const nav_msgs::OdometryConstPtr &odom) {
  if (!cloud || !odom || cloud->width*cloud->height==0) return;
  auto &i=*impl_; auto &p=*i.planner;
  p.lidar_map_interface_->updateCloudMapOdometry(cloud,odom);
  if (p.lidar_map_interface_->ld_->lidar_cloud_.empty()) return;
  i.cloud_ready=true;
  // Duplicate/out-of-order frames must never multiply finish evidence.
  if (!cloud->header.stamp.isZero() && cloud->header.stamp>i.cloud_stamp) {
    i.cloud_stamp=cloud->header.stamp; ++i.cloud_revision;
  }
  const Eigen::Vector3f pose=p.lidar_map_interface_->ld_->lidar_pose_;
  if (p.lidar_map_interface_->IsInBox(pose) &&
      (!i.path_sampled || (i.last_path_sample-pose).norm()>=i.sample_path)) {
    i.new_path.push_back(pose); i.last_path_sample=pose; i.path_sampled=true;
  }
  std::vector<epicon_native::ClusterInfo::Ptr> added; std::vector<int> removed;
  i.frontier->updateFrontierClusters(added,removed);
  const int odom_id=std::max(0,static_cast<int>(p.topo_graph_->history_odom_nodes_.size())-1);
  for(auto &c:added) c->odom_id_=odom_id;
}
void EpiconFrontend::setOdometry(const Eigen::Vector3f &position,const Eigen::Vector3f &velocity,float yaw) {
  auto &i=*impl_; i.pos=position; i.vel=velocity; i.yaw=yaw;
  i.planner->local_data_.curr_yaw_=yaw;
  i.planner->topo_graph_->odom_node_->center_=position;
}
void EpiconFrontend::resetTask() {
  auto &i=*impl_;
  i.clearGoal(); i.deferred.clear(); i.audited=false;
}
void EpiconFrontend::deferCurrentGoal(double seconds, double radius) {
  auto &i=*impl_;
  if (i.selected && tour().size()>=2) {
    i.deferred.push_back({tour()[1],ros::WallTime::now()+ros::WallDuration(std::max(1.0,seconds)),
                          std::max(0.1,radius)});
    ROS_WARN_STREAM("[exploration] defer failed goal " << tour()[1].transpose()
                    << " for " << seconds << "s; select an alternative viewpoint");
  }
  i.clearGoal();
}
EpiconFrontend::Result EpiconFrontend::update(bool select_goal, bool full_audit) {
  auto &i=*impl_; auto &p=*i.planner;
  i.audited=false;
  const auto wall_now=ros::WallTime::now();
  i.deferred.erase(std::remove_if(i.deferred.begin(),i.deferred.end(),
      [&](const Impl::DeferredGoal &g){return g.until<=wall_now;}),i.deferred.end());
  if (!i.cloud_ready) return Result::WAITING_MAP;
  if (!i.pos.allFinite() || !p.lidar_map_interface_->IsInBox(i.pos)) {
    i.publish("OUTSIDE_BOX",0,0); return Result::DISCONNECTED;
  }
  const auto start=ros::WallTime::now();
  i.frontier->full_audit_=full_audit;
  if (full_audit) i.frontier->refreshAllClusters();
  Impl::Points upper,lower;
  for(const auto &c:i.frontier->cluster_list_) if(!c->is_dormant_ && c->is_reachable_) {
    upper.push_back(c->box_max_); lower.push_back(c->box_min_);
  }
  if (full_audit || i.topology_revision!=i.cloud_revision || !i.new_path.empty()) {
    p.topo_graph_->getRegionsToUpdate();
    i.unknown.clear(); i.active.clear();
    p.topo_graph_->updateSkeleton(i.new_path,upper,lower,i.unknown,i.active,i.valid_cubes,i.sample_path);
    i.topology_revision=i.cloud_revision;
    p.topo_graph_->updateHistoricalOdoms();
  }
  p.topo_graph_->updateOdomNode(i.pos,i.yaw);
  if (!ready()) { i.selected=false; i.publish("DISCONNECTED",i.unknown.size(),i.active.size()); return Result::DISCONNECTED; }
  if (!select_goal) return Result::SUCCEED;
  int result=i.explorer->planGlobalPath(i.pos.cast<double>(),i.vel.cast<double>(),i.unknown,i.active,i.hybrid_radius,i.unknown_penalty);
  if (result==epicon_native::fast_planner::NO_FRONTIER && !i.deferred.empty())
    result=epicon_native::fast_planner::BLOCKED;
  i.audited=full_audit;
  i.selected=result==epicon_native::fast_planner::SUCCEED;
  p.graph_visualizer_->vizBox(p.topo_graph_);
  if(i.explorer->ep_->view_graph_) p.graph_visualizer_->vizGraph(p.topo_graph_);
  i.frontier->visfrtcluster(); i.frontier->viz_pocc();
  const char *name=result==epicon_native::fast_planner::SUCCEED ? "SUCCEED" :
                   result==epicon_native::fast_planner::NO_FRONTIER ? "NO_FRONTIER" :
                   result==epicon_native::fast_planner::BLOCKED ? "BLOCKED" : "FAIL";
  i.publish(name,i.unknown.size(),i.active.size());
  ROS_INFO_STREAM("[exploration] global result=" << name << " clusters=" << clusterCount()
      << " unknown=" << i.unknown.size() << " active=" << i.active.size()
      << " tour=" << tour().size() << " speed=" << i.vel.norm() << " ms=" << (ros::WallTime::now()-start).toSec()*1000.0);
  if(result==epicon_native::fast_planner::NO_FRONTIER) return Result::NO_FRONTIER;
  if(result==epicon_native::fast_planner::BLOCKED) return Result::BLOCKED;
  return result==epicon_native::fast_planner::SUCCEED ? Result::SUCCEED : Result::FAIL;
}
EpiconFrontend::Result EpiconFrontend::pathToGoal(std::vector<Eigen::Vector3f> &path) {
  path.clear(); auto &i=*impl_; auto &p=*i.planner;
  if (!ready()) return Result::DISCONNECTED;
  if(!i.selected || tour().size()<2) return Result::NO_FRONTIER;
  if(p.lidar_map_interface_->getDisToOcc(i.explorer->ed_->next_goal_node_->center_)<p.topo_graph_->bubble_min_radius_) return Result::FAIL;
  const int result=p.fast_searcher_->search(p.topo_graph_->odom_node_,i.vel,i.explorer->ed_->next_goal_node_,0.2,path);
  if(result!=epicon_native::fast_planner::BubbleAstar::REACH_END || path.size()<2) return Result::FAIL;
  // Upstream's 1 m resampling. The General adapter alone owns the future
  // trajectory head; prepending it here would create a reverse path segment.
  std::vector<Eigen::Vector3f> sampled{path.front()};
  for(size_t n=1;n<path.size();++n) {
    while((path[n]-sampled.back()).norm()>1.0f) {
      const Eigen::Vector3f step=sampled.back()+(path[n]-sampled.back()).normalized(); sampled.push_back(step);
    }
    if((path[n]-sampled.back()).norm()>=.01f) sampled.push_back(path[n]);
  }
  path.swap(sampled); return Result::SUCCEED;
}
EpiconFrontend::Result EpiconFrontend::pathFrom(const Eigen::Vector3d &position,
    const Eigen::Vector3d &velocity, double yaw, std::vector<Eigen::Vector3f> &path) {
  auto &i=*impl_;
  if (!position.allFinite() || !velocity.allFinite() || !contains(position)) return Result::DISCONNECTED;
  // The graph and sensor history retain measured odometry. Only the search
  // origin is temporarily reconnected at the exact future trajectory head.
  auto &graph=*i.planner->topo_graph_;
  Eigen::Vector3f search_position=position.cast<float>();
  float search_yaw=static_cast<float>(yaw);
  graph.updateOdomNode(search_position,search_yaw);
  const Eigen::Vector3f old_velocity=i.vel;
  i.vel=velocity.cast<float>();
  const auto result=pathToGoal(path);
  i.vel=old_velocity;
  graph.updateOdomNode(i.pos,i.yaw);
  return result;
}
bool EpiconFrontend::corridor(const std::vector<Eigen::Vector3d> &path,
    double range, double clearance, EpiconCorridor &result) const {
  if (path.size()<2 || !hasCloud()) { result={}; result.failure="no point-cloud path"; return false; }
  Eigen::Vector3d lower=path.front(), upper=path.front();
  for (const auto &p:path) { lower=lower.cwiseMin(p); upper=upper.cwiseMax(p); }
  // Match EPICON's obstacle crop, especially its local vertical envelope.
  lower-=Eigen::Vector3d(3,3,1); upper+=Eigen::Vector3d(3,3,1);
  epicon_native::PointVector points;
  impl_->planner->lidar_map_interface_->boxSearch(lower.cast<float>(),upper.cast<float>(),points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
  input->points=points;
  pcl::PointCloud<pcl::PointXYZ> filtered;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input); filter.setLeafSize(.2f,.2f,.2f); filter.filter(filtered);
  std::vector<Eigen::Vector3d> surface; surface.reserve(filtered.size());
  for (const auto &p:filtered) surface.emplace_back(p.x,p.y,p.z);
  return buildEpiconCorridor(path,surface,lower,upper,range,clearance,result);
}
double EpiconFrontend::clearance(const Eigen::Vector3d &position) const {
  if (!hasCloud() || !position.allFinite()) return -std::numeric_limits<double>::infinity();
  return impl_->planner->lidar_map_interface_->getDisToOcc(position);
}
bool EpiconFrontend::contains(const Eigen::Vector3d &position) const {
  return position.allFinite() && impl_->planner->lidar_map_interface_->IsInBox(position.cast<float>().eval());
}
bool EpiconFrontend::recoveryPoint(const Eigen::Vector3d &start, Eigen::Vector3d &goal) const {
  if (!hasCloud() || !contains(start)) return false;
  epicon_native::PointVector points;
  const Eigen::Vector3f extent(2,2,1);
  impl_->planner->lidar_map_interface_->boxSearch(start.cast<float>()-extent,start.cast<float>()+extent,points);
  std::vector<Eigen::Vector3d> surface; surface.reserve(points.size());
  for (const auto &p:points) surface.emplace_back(p.x,p.y,p.z);
  if (!epiconRecoveryPoint(start,surface,goal)) return false;
  // Prefer the same-height interior when it restores clearance. Vertical
  // escape is reserved for geometry that actually requires it.
  Eigen::Vector3d level=goal; level.z()=start.z();
  const double required=std::max(.6,clearance(start)+.05);
  if (contains(level) && clearance(level)>=required && (level-start).norm()>.1) goal=level;
  return contains(goal) && clearance(goal)>=required;
}
const std::vector<Eigen::Vector3f> &EpiconFrontend::tour() const { return impl_->explorer->ed_->global_tour_; }
double EpiconFrontend::goalYaw() const { return impl_->planner->local_data_.end_yaw_; }
bool EpiconFrontend::ready() const { return impl_->cloud_ready && !impl_->planner->topo_graph_->odom_node_->neighbors_.empty(); }
bool EpiconFrontend::hasCloud() const { return impl_->cloud_ready; }
std::size_t EpiconFrontend::clusterCount() const { return impl_->frontier->cluster_list_.size(); }
std::size_t EpiconFrontend::historyCount() const { return impl_->planner->topo_graph_->history_odom_nodes_.size(); }
bool EpiconFrontend::hasSelectedGoal() const { return impl_->selected; }
bool EpiconFrontend::lastUpdateAudited() const { return impl_->audited; }
std::uint64_t EpiconFrontend::cloudRevision() const { return impl_->cloud_revision; }
ros::Time EpiconFrontend::cloudStamp() const { return impl_->cloud_stamp; }
}
