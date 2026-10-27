#include <general_core/exploration/highspeed/epicon_frontend.h>
#include <epicon_native/epicon_planner/fast_exploration_manager.h>
#include <epicon_native/epicon_planner/expl_data.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <pcl/filters/voxel_grid.h>

namespace fast_planner {
namespace {
template <typename T> void defaultParam(ros::NodeHandle &nh, const std::string &key, const T &value) {
  if (!nh.hasParam(key)) nh.setParam(key, value);
}
// Merge leaves so a native override does not hide unrelated scene settings.
void inheritParameters(ros::NodeHandle &parent, ros::NodeHandle &nh,
                       const std::string &key, XmlRpc::XmlRpcValue value) {
  if (value.getType()==XmlRpc::XmlRpcValue::TypeStruct) {
    for (auto it=value.begin();it!=value.end();++it)
      inheritParameters(parent,nh,key+"/"+it->first,it->second);
  } else if (!nh.hasParam(key)) nh.setParam(key,value);
}
void configureNativeParameters(ros::NodeHandle &parent, ros::NodeHandle &nh) {
  // Explicit epicon overrides > scene/launch values > native defaults.
  for (const auto &key : {"global_planning", "view_graph", "FrontierManager",
       "ViewpointManager", "viewpoint_param", "bubble_topo", "bubble_astar",
       "parallel_astar", "max_update_region_num", "lidar_perception"}) {
    XmlRpc::XmlRpcValue value;
    if (parent.getParam(key,value)) inheritParameters(parent,nh,key,value);
  }

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
  defaultParam(nh, "ViewpointManager/min_clearance", 0.9);
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
      "MaxVelMag", "MaxTiltAngle", "GravAcc", "yaw_max_vel"};
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
  Points safe_history;
  Points backtrack_attempts, backtrack_origins, progress_regions;
  bool backtracking{false};
  ros::WallTime backtrack_started;
  std::vector<Eigen::Vector3f> execution_path, selected_route;
  std::vector<std::pair<Eigen::Vector3d,ros::WallTime>> recovery_history;
  struct DeferredGoal { Eigen::Vector3f position; ros::WallTime until; double radius; };
  std::vector<DeferredGoal> deferred;
  void clearGoal() {
    planner->topo_graph_->removeNode(explorer->ed_->next_goal_node_);
    explorer->ed_->next_goal_node_=std::make_shared<epicon_native::TopoNode>();
    explorer->ed_->global_tour_.clear(); explorer->ed_->path_next_goal_.clear();
    explorer->last_goal_cost_=5e3; selected=false; execution_path.clear(); selected_route.clear(); backtracking=false;
  }
  ros::WallTime goal_selected_at;
  double goal_hold_seconds{1.2};
  int valid_cubes{10};
  double sample_path{0.1}, hybrid_radius{15.0}, unknown_penalty{1.5};
  double planning_clearance{0.6}, physical_floor{0.48}, execution_clearance{0.48};
  double connection_timeout{.002};
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
        << ",\"backtracking\":" << (backtracking ? "true" : "false")
        << ",\"backtrack_attempts\":" << backtrack_attempts.size()
        << ",\"deferred_goals\":" << deferred.size()
        << ",\"geometry_rejected\":" << frontier->geometry_rejected_
        << ",\"connectivity_rejected\":" << frontier->connectivity_rejected_
        << ",\"unobservable_clusters\":" << frontier->unobservable_clusters_
        << ",\"viewpoint_retry_reached\":" << frontier->viewpoint_retry_results_[1]
        << ",\"viewpoint_retry_no_path\":" << frontier->viewpoint_retry_results_[2]
        << ",\"viewpoint_retry_start_fail\":" << frontier->viewpoint_retry_results_[3]
        << ",\"viewpoint_retry_end_fail\":" << frontier->viewpoint_retry_results_[4]
        << ",\"viewpoint_retry_timeout\":" << frontier->viewpoint_retry_results_[5]
        << ",\"viewpoint_alternatives_reached\":" << frontier->viewpoint_alternatives_reached_
        << ",\"recovery_attempts\":" << recovery_history.size()
        << ",\"odom_clearance\":" << planner->lidar_map_interface_->getDisToOcc(pos)
        << ",\"planning_clearance\":" << planning_clearance
        << ",\"execution_clearance\":" << execution_clearance
        << ",\"connect_reused\":" << planner->topo_graph_->odom_reused_connections_
        << ",\"connect_retry\":" << planner->topo_graph_->odom_retry_count_
        << ",\"connect_ms\":" << planner->topo_graph_->odom_connection_ms_
        << ",\"odom_neighbors\":" << planner->topo_graph_->odom_node_->neighbors_.size()
        << ",\"connect_reached\":" << planner->topo_graph_->odom_connection_results_[1]
        << ",\"connect_no_path\":" << planner->topo_graph_->odom_connection_results_[2]
        << ",\"connect_start_fail\":" << planner->topo_graph_->odom_connection_results_[3]
        << ",\"connect_end_fail\":" << planner->topo_graph_->odom_connection_results_[4]
        << ",\"connect_timeout\":" << planner->topo_graph_->odom_connection_results_[5]
        << ",\"connection_cache_hits\":" << frontier->connection_cache_hits_
        << ",\"largest_retry_budget_ms\":" << frontier->largest_retry_budget_*1000.0
        << ",\"residuals\":[";
    bool first=true;
    if(audited) for(const auto &c:frontier->cluster_list_) {
      if(!first) out << ",";
      first=false;
      out << "{\"id\":" << c->id_ << ",\"center\":[" << c->center_.x() << "," << c->center_.y() << "," << c->center_.z()
          << "],\"cells\":" << c->cells_.size() << ",\"dormant\":" << (c->is_dormant_ ? "true":"false")
          << ",\"connection_state\":" << c->connection_state_ << "}";
    }
    out << "]}";
    msg.data=out.str(); status.publish(msg);
  }
};
EpiconFrontend::EpiconFrontend() : impl_(new Impl) {}
EpiconFrontend::~EpiconFrontend() = default;
void EpiconFrontend::init(ros::NodeHandle &runtime_nh) {
  ros::NodeHandle nh(runtime_nh,"epicon");
  configureNativeParameters(runtime_nh,nh);
  auto &i=*impl_;
  runtime_nh.param("GoalSwitchMinInterval",i.goal_hold_seconds,1.2);
  i.goal_hold_seconds=std::clamp(i.goal_hold_seconds,0.0,5.0);
  i.planner=std::make_shared<epicon_native::fast_planner::FastPlannerManager>();
  auto &p=*i.planner;
  p.lidar_map_interface_=std::make_shared<epicon_native::fast_planner::LIOInterface>();
  p.lidar_map_interface_->init(nh);
  p.parallel_path_finder_=std::make_shared<epicon_native::ParallelBubbleAstar>();
  p.parallel_path_finder_->init(nh,p.lidar_map_interface_);
  double hard=.5, soft=.6, commit=.5, tolerance=.02;
  runtime_nh.param("DilateRadiusHard",hard,hard);
  runtime_nh.param("DilateRadiusSoft",soft,soft);
  runtime_nh.param("CommitKnownFreeSafeDistance",commit,commit);
  runtime_nh.param("SafetyClearanceTolerance",tolerance,tolerance);
  if(!std::isfinite(hard) || hard<=0 || !std::isfinite(soft) || soft<hard ||
     !std::isfinite(commit) || commit<hard || !std::isfinite(tolerance) ||
     tolerance<0 || tolerance>.05 || tolerance>=hard ||
     !std::isfinite(p.parallel_path_finder_->safe_distance_) ||
     p.parallel_path_finder_->safe_distance_<=0 ||
     !std::isfinite(p.parallel_path_finder_->resolution_) || p.parallel_path_finder_->resolution_<=0)
    throw std::invalid_argument("invalid exploration clearance/search policy: require soft,commit>=hard>0 and tolerance in [0,0.05]");
  // Search must never accept a route below the common hard/commit contract.
  p.parallel_path_finder_->safe_distance_=std::max(
      p.parallel_path_finder_->safe_distance_,std::max(hard,commit));
  nh.setParam("bubble_astar/safe_distance",p.parallel_path_finder_->safe_distance_);
  i.physical_floor=std::max(.05,hard-tolerance);
  // Hard/commit clearance governs execution. Soft clearance shapes new
  // corridors; search endpoint slack must not silently raise the hard stop.
  i.execution_clearance=std::max(.05,std::max(hard,commit)-tolerance);
  i.planning_clearance=std::max({hard,soft,commit});
  // The direct connector needs >1 cm of free sphere beyond its search radius.
  double bubble_radius=.61;
  nh.param("bubble_topo/bubble_min_radius",bubble_radius,bubble_radius);
  const double endpoint_clearance=std::max(std::max(hard,commit),
      p.parallel_path_finder_->safe_distance_+.02);
  if(!std::isfinite(bubble_radius) || bubble_radius<=0)
    throw std::invalid_argument("invalid bubble_min_radius");
  nh.setParam("bubble_topo/bubble_min_radius",std::max(bubble_radius,endpoint_clearance));
  double viewpoint_clearance=.9;
  nh.getParam("ViewpointManager/min_clearance",viewpoint_clearance);
  if(!std::isfinite(viewpoint_clearance) || viewpoint_clearance<=0)
    throw std::invalid_argument("invalid viewpoint min_clearance");
  // A viewpoint must also be a valid graph endpoint when an indoor profile
  // lowers the sampling clearance below the native 0.9 m default.
  nh.setParam("ViewpointManager/min_clearance",
      std::max({viewpoint_clearance,bubble_radius,endpoint_clearance}));
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
    // Keep viewpoints away from task boundaries, so the backend has room to
    // build a bounded corridor through overlapping floor volumes.
    bool fits=false;
    const auto &bounds=*i.planner->lidar_map_interface_->lp_;
    for (size_t box=0;box<bounds.global_box_min_boundary_vec_.size();++box) {
      const auto lo=bounds.global_box_min_boundary_vec_[box].array()+i.planning_clearance;
      const auto hi=bounds.global_box_max_boundary_vec_[box].array()-i.planning_clearance;
      if ((point.array()>=lo).all() && (point.array()<=hi).all()) { fits=true; break; }
    }
    if (!fits) return false;
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
  nh.getParam("parallel_astar/update_connection_timeout",i.connection_timeout);
  i.status=runtime_nh.advertise<std_msgs::String>("/planning/exploration/epicon_status",1,true);
  runtime_nh.setParam("exploration/frontend",std::string("epicon_pointcloud"));
  const std::string effective="epicon/effective_parameters/";
  runtime_nh.setParam(effective+"corridor_backend",std::string("firi"));
  runtime_nh.setParam(effective+"geometry",std::string("obstacle_points"));
  runtime_nh.setParam(effective+"hard_clearance",hard);
  runtime_nh.setParam(effective+"commit_clearance",commit);
  runtime_nh.setParam(effective+"soft_clearance",i.planning_clearance);
  runtime_nh.setParam(effective+"execution_clearance",i.execution_clearance);
  runtime_nh.setParam(effective+"search_clearance",p.parallel_path_finder_->safe_distance_);
  runtime_nh.setParam(effective+"search_resolution",p.parallel_path_finder_->resolution_);
  runtime_nh.setParam(effective+"graph_endpoint_clearance",std::max(bubble_radius,endpoint_clearance));
  runtime_nh.setParam(effective+"viewpoint_clearance",std::max({viewpoint_clearance,bubble_radius,endpoint_clearance}));
  ROS_INFO_STREAM("[exploration policy] namespace=" << runtime_nh.getNamespace()
      << " corridor=firi geometry=obstacle_points hard=" << hard << " commit=" << commit
      << " soft=" << i.planning_clearance << " execution=" << i.execution_clearance
      << " search=" << p.parallel_path_finder_->safe_distance_
      << " resolution=" << p.parallel_path_finder_->resolution_
      << " endpoint=" << std::max(bubble_radius,endpoint_clearance)
      << " viewpoint=" << std::max({viewpoint_clearance,bubble_radius,endpoint_clearance}));
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
    if(p.lidar_map_interface_->getDisToOcc(pose)>=recoveryClearance()) {
      i.safe_history.push_back(pose);
      // Keep a sparse mission-scale history as well as recent recovery samples.
      if(i.safe_history.size()>2000) i.safe_history.erase(i.safe_history.begin());
    }
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
  i.clearGoal(); i.deferred.clear(); i.recovery_history.clear(); i.backtrack_attempts.clear(); i.backtrack_origins.clear();
  i.progress_regions={i.pos}; i.audited=false;
  i.frontier->connection_records_.clear(); i.frontier->viewpoint_retry_offset_=0;
}
void EpiconFrontend::deferCurrentGoal(double seconds, double radius) {
  auto &i=*impl_;
  // DISCONNECTED clears selected before the FSM handles the failed route.
  // The last tour still identifies the viewpoint that led into this pocket.
  if (tour().size()>=2) {
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
    i.clearGoal();
    i.publish("OUTSIDE_BOX",0,0); return Result::OUTSIDE_BOX;
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
  // A retreat is a regular, revalidated navigation goal, not a clearance
  // escape. Keep it across audits until arrival, invalidation or timeout.
  if(i.backtracking && tour().size()>=2) {
    std::vector<Eigen::Vector3f> certified;
    if((tour()[1]-i.pos).norm()>.4f &&
       (wall_now-i.backtrack_started).toSec()<45.0 &&
       i.frontier->viewpoint_allowed_(tour()[1]) &&
       pathToGoal(certified)==Result::SUCCEED &&
       p.parallel_path_finder_->collisionCheck_shortenPath(certified)) {
      i.explorer->ed_->global_tour_.front()=i.pos;
      i.audited=full_audit; i.publish("BACKTRACKING",i.unknown.size(),i.active.size());
      return Result::SUCCEED;
    }
    i.clearGoal();
  }
  // Retain a moving goal for a bounded interval, but always revalidate its
  // route. Audits, arrival, exclusions and disconnections bypass the hold.
  if (!full_audit && i.selected && tour().size()>=2 &&
      !i.goal_selected_at.isZero() &&
      (wall_now-i.goal_selected_at).toSec()<i.goal_hold_seconds &&
      (tour()[1]-i.pos).norm()>.5f && i.frontier->viewpoint_allowed_(tour()[1])) {
    std::vector<Eigen::Vector3f> certified;
    if (pathToGoal(certified)==Result::SUCCEED &&
        p.parallel_path_finder_->collisionCheck_shortenPath(certified)) {
      i.explorer->ed_->global_tour_.front()=i.pos;
      i.publish("GOAL_HELD",i.unknown.size(),i.active.size());
      return Result::SUCCEED;
    }
  }
  const auto previous_tour=tour();
  const bool had_selected=i.selected;
  int result=i.explorer->planGlobalPath(i.pos.cast<double>(),i.vel.cast<double>(),i.unknown,i.active,i.hybrid_radius,i.unknown_penalty);
  // Unknown-space fallback candidates are generated only by a full audit.
  // Their absence on the next incremental update must not cancel a still
  // reachable selected goal before the vehicle can make any progress.
  if((result==epicon_native::fast_planner::NO_FRONTIER ||
      result==epicon_native::fast_planner::BLOCKED) && !full_audit && had_selected &&
     previous_tour.size()>=2 && (previous_tour[1]-i.pos).norm()>.5f &&
     (i.explorer->ed_->next_goal_node_->center_-previous_tour[1]).norm()<1e-3f &&
     i.frontier->viewpoint_allowed_(previous_tour[1])) {
    const auto rejected_tour=tour();
    i.explorer->ed_->global_tour_=previous_tour;
    i.explorer->ed_->global_tour_.front()=i.pos;
    std::vector<Eigen::Vector3f> certified;
    if(pathToGoal(certified)==Result::SUCCEED &&
       p.parallel_path_finder_->collisionCheck_shortenPath(certified)) {
      result=epicon_native::fast_planner::SUCCEED;
      ROS_INFO_THROTTLE(1.0,"[exploration] retain certified goal between full audits");
    } else i.explorer->ed_->global_tour_=rejected_tour;
  }
  if(result==epicon_native::fast_planner::BLOCKED && full_audit &&
     i.backtrack_attempts.size()<3 && !i.frontier->cluster_list_.empty()) {
    // Revisit a previously measured safe pose that approaches unresolved
    // frontier regions. Never treat old odometry as present-day free space:
    // reconnect it through the current map and run the normal backend.
    std::vector<std::pair<double,Eigen::Vector3f>> candidates;
    auto relocation_points=i.safe_history;
    for(const auto &node:p.topo_graph_->history_odom_nodes_)
      relocation_points.push_back(node->center_);
    for(const auto &point:relocation_points) {
      const double distance=(point-i.pos).norm();
      if(distance<2.0 || !i.frontier->viewpoint_allowed_(point) ||
         clearance(point.cast<double>())<p.topo_graph_->bubble_min_radius_) continue;
      bool visited=false;
      for(const auto &old:i.backtrack_attempts) if((old-point).norm()<1.0f) { visited=true; break; }
      // Do not immediately return to the dead end that initiated a retreat.
      for(const auto &old:i.backtrack_origins) if((old-point).norm()<2.0f) { visited=true; break; }
      if(visited) continue;
      double frontier=std::numeric_limits<double>::infinity();
      for(const auto &cluster:i.frontier->cluster_list_) {
        const Eigen::Vector3f center=(cluster->box_min_+cluster->box_max_)*.5f;
        const double from_candidate=(point-center).norm();
        // A valid retreat may initially move away from the unresolved frontier.
        frontier=std::min(frontier,from_candidate);
      }
      if(!std::isfinite(frontier)) continue;
      candidates.emplace_back(frontier+.1*distance,point);
    }
    std::sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.first<b.first;});
    // Adjacent odometry samples must not spend the whole recovery budget on one place.
    std::vector<std::pair<double,Eigen::Vector3f>> distinct;
    for(const auto &candidate:candidates) {
      bool nearby=false;
      for(const auto &old:distinct) if((old.second-candidate.second).norm()<1.0f) { nearby=true; break; }
      if(!nearby) distinct.push_back(candidate);
    }
    candidates.swap(distinct);
    const auto deadline=ros::WallTime::now()+ros::WallDuration(.04);
    for(const auto &candidate:candidates) {
      const double remaining=(deadline-ros::WallTime::now()).toSec();
      if(remaining<=0) break;
      std::vector<Eigen::Vector3f> certified;
      // Prefer existing global connectivity over a new long voxel search.
      for(const auto &node:p.topo_graph_->history_odom_nodes_) {
        if((node->center_-candidate.second).norm()>.75f) continue;
        std::vector<epicon_native::TopoNode::Ptr> nodes;
        const double left=(deadline-ros::WallTime::now()).toSec();
        if(left<=0) break;
        if(!p.topo_graph_->graphSearch(p.topo_graph_->odom_node_,node,nodes,std::min(.003,left))) continue;
        certified={i.pos};
        bool complete=true;
        for(size_t n=1;n<nodes.size();++n) {
          const auto edge=nodes[n-1]->paths_.find(nodes[n]);
          if(edge==nodes[n-1]->paths_.end() || edge->second.empty()) { complete=false; break; }
          certified.insert(certified.end(),edge->second.begin(),edge->second.end());
        }
        if(complete) certified.push_back(candidate.second);
        if(!complete || certified.size()<2 || !p.parallel_path_finder_->collisionCheck_shortenPath(certified))
          certified.clear();
        if(!certified.empty()) break;
      }
      if(certified.empty()) {
        const double left=(deadline-ros::WallTime::now()).toSec();
        if(left<=0) break;
        if(p.parallel_path_finder_->search(i.pos,candidate.second,certified,std::min(.01,left),true)!=
            epicon_native::ParallelBubbleAstar::REACH_END ||
           !p.parallel_path_finder_->collisionCheck_shortenPath(certified)) continue;
      }
      i.explorer->ed_->global_tour_={i.pos,candidate.second};
      i.explorer->updateGoalNode();
      i.selected_route=certified;
      i.selected=true;
      if(pathToGoal(certified)!=Result::SUCCEED) { i.clearGoal(); continue; }
      i.backtracking=true; i.backtrack_started=ros::WallTime::now();
      i.backtrack_attempts.push_back(candidate.second); i.backtrack_origins.push_back(i.pos);
      result=epicon_native::fast_planner::SUCCEED;
      ROS_INFO_STREAM("[exploration] bounded backtrack goal=" << candidate.second.transpose()
                      << " attempt=" << i.backtrack_attempts.size() << "/3");
      break;
    }
  }
  i.audited=full_audit;
  i.selected=result==epicon_native::fast_planner::SUCCEED;
  if(i.selected && tour().size()>=2 && (!had_selected || previous_tour.size()<2 ||
      (tour()[1]-previous_tour[1]).norm()>.5f)) i.goal_selected_at=wall_now;
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
  if (!hasCloud()) return Result::WAITING_MAP;
  if(!i.selected || tour().size()<2) return Result::NO_FRONTIER;
  if(p.lidar_map_interface_->getDisToOcc(i.explorer->ed_->next_goal_node_->center_)<p.topo_graph_->bubble_min_radius_) return Result::FAIL;
  // Reanchoring odometry deliberately deletes transient goal edges. Keep the
  // geometric route, never its old certification: validate every segment in
  // the current map from the exact requested root before reusing it.
  bool reused=false;
  if(i.selected_route.size()>=2 &&
     (i.selected_route.back()-i.explorer->ed_->next_goal_node_->center_).norm()<1e-3f) {
    path=i.selected_route; path.front()=p.topo_graph_->odom_node_->center_;
    reused=p.parallel_path_finder_->collisionCheck_shortenPath(path);
  }
  if(!reused) {
    path.clear();
    const int result=p.fast_searcher_->search(p.topo_graph_->odom_node_,i.vel,i.explorer->ed_->next_goal_node_,0.2,path);
    if(result!=epicon_native::fast_planner::BubbleAstar::REACH_END || path.size()<2) {
      const auto root=p.topo_graph_->odom_node_->center_;
      const auto goal=i.explorer->ed_->next_goal_node_->center_;
      auto &records=i.frontier->connection_records_;
      auto record=std::find_if(records.begin(),records.end(),[&](const auto &r) {
        return (r.endpoint-goal).norm()<.3f && (r.root-root).norm()<2.0f;
      });
      if(record==records.end()) {
        if(records.size()>=256) records.erase(records.begin());
        records.emplace_back(); record=std::prev(records.end());
        record->root=root; record->endpoint=goal;
      }
      record->touched=ros::WallTime::now();
      const double budget=std::min(.06,.02*double(1u<<std::min(2u,record->timeouts)));
      const int fallback=p.parallel_path_finder_->search(root,goal,path,budget,true);
      if(fallback==epicon_native::ParallelBubbleAstar::TIME_OUT)
        record->timeouts=std::min(3u,record->timeouts+1);
      if(fallback!=epicon_native::ParallelBubbleAstar::REACH_END || path.size()<2 ||
         !p.parallel_path_finder_->collisionCheck_shortenPath(path)) {
        // Preserve normal graph/voxel guide selection. A saved global route is
        // the last fallback, and must be certified again from this exact head.
        for(const auto &record:i.frontier->connection_records_) {
          if(record.route.size()<2 || (record.endpoint-i.explorer->ed_->next_goal_node_->center_).norm()>.3f) continue;
          path=record.route; path.front()=p.topo_graph_->odom_node_->center_;
          path.back()=i.explorer->ed_->next_goal_node_->center_;
          if(p.parallel_path_finder_->collisionCheck_shortenPath(path)) { reused=true; break; }
        }
        if(!reused) { i.selected_route.clear(); return Result::FAIL; }
      }
    }
  }
  // Upstream's 1 m resampling. The General adapter alone owns the future
  // trajectory head; prepending it here would create a reverse path segment.
  std::vector<Eigen::Vector3f> sampled{path.front()};
  for(size_t n=1;n<path.size();++n) {
    while((path[n]-sampled.back()).norm()>1.0f) {
      const Eigen::Vector3f step=sampled.back()+(path[n]-sampled.back()).normalized(); sampled.push_back(step);
    }
    if((path[n]-sampled.back()).norm()>=.01f) sampled.push_back(path[n]);
  }
  path.swap(sampled); i.selected_route=path; return Result::SUCCEED;
}
EpiconFrontend::Result EpiconFrontend::pathFrom(const Eigen::Vector3d &position,
    const Eigen::Vector3d &velocity, double yaw, std::vector<Eigen::Vector3f> &path) {
  auto &i=*impl_;
  if (!position.allFinite() || !velocity.allFinite()) return Result::DISCONNECTED;
  if (!contains(position)) return Result::OUTSIDE_BOX;
  // The graph and sensor history retain measured odometry. Only the search
  // origin is temporarily reconnected at the exact future trajectory head.
  auto &graph=*i.planner->topo_graph_;
  Eigen::Vector3f search_position=position.cast<float>();
  float search_yaw=static_cast<float>(yaw);
  if(i.selected && i.explorer->ed_->next_goal_node_) {
    const auto edge=graph.odom_node_->paths_.find(i.explorer->ed_->next_goal_node_);
    if(edge!=graph.odom_node_->paths_.end() && edge->second.size()>=2 &&
       (edge->second.back()-i.explorer->ed_->next_goal_node_->center_).norm()<1e-3f)
      i.selected_route=edge->second;
  }
  graph.updateOdomNode(search_position,search_yaw);
  const Eigen::Vector3f old_velocity=i.vel;
  i.vel=velocity.cast<float>();
  const auto result=pathToGoal(path);
  if(result==Result::SUCCEED) i.execution_path=path;
  else i.execution_path.clear();
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
  std::vector<Eigen::Vector3d> surface;
  if (clearance < planningClearance()-1e-6) {
    // With only millimetres of spare clearance, voxel centroids can move a
    // wall enough to make the corridor disagree with final safety checks.
    surface.reserve(points.size());
    for (const auto &p:points) surface.emplace_back(p.x,p.y,p.z);
  } else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    input->points=points;
    pcl::PointCloud<pcl::PointXYZ> filtered;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(input); filter.setLeafSize(.2f,.2f,.2f); filter.filter(filtered);
    surface.reserve(filtered.size());
    for (const auto &p:filtered) surface.emplace_back(p.x,p.y,p.z);
  }
  const bool narrow=clearance<planningClearance()-1e-6;
  std::vector<Eigen::Vector3d> guide=path;
  if(narrow) {
    guide={path.front()};
    for(size_t n=1;n<path.size();++n) {
      const double local_clearance=std::min({this->clearance(path[n-1]),this->clearance(path[n]),
          this->clearance((path[n-1]+path[n])*.5)});
      const double step=local_clearance<planningClearance()+.05 ? .15 : 1.0;
      const int steps=std::max(1,int(std::ceil((path[n]-path[n-1]).norm()/step)));
      for(int k=1;k<=steps;++k) guide.push_back(path[n-1]+double(k)/steps*(path[n]-path[n-1]));
    }
  }
  // Local raw-point crops keep long indoor routes within the commit budget.
  // The crop still extends beyond the required obstacle clearance on all sides.
  const double local_range=narrow ? std::min(range,std::max(1.0,clearance+.3)) : range;
  const bool generated=buildEpiconCorridor(guide,surface,lower,upper,local_range,clearance,result,false);
  result.map_revision=impl_->cloud_revision;
  if(!generated) return false;
  // This is a numerical positive-overlap requirement, not obstacle clearance.
  // A hard-safe doorway may leave less than 1 cm of interior radius.
  const double min_overlap=narrow ? 1e-3 : 1e-2;
  result.min_overlap=min_overlap;
  const auto &bounds=*impl_->planner->lidar_map_interface_->lp_;
  std::vector<Eigen::Vector3d> box_lower,box_upper;
  for(size_t n=0;n<bounds.global_box_min_boundary_vec_.size();++n) {
    box_lower.push_back(bounds.global_box_min_boundary_vec_[n].cast<double>());
    box_upper.push_back(bounds.global_box_max_boundary_vec_[n].cast<double>());
  }
  if(!clipEpiconCorridorToBoxes(result.path.front(),result.path.back(),box_lower,box_upper,result.planes,min_overlap)) {
    // If only a prefix has positive corridor overlap, end at a certified
    // guide point in that prefix and replan after moving. Never use the
    // interior of an unbounded cell as an arbitrary upward relocation.
    bool prefix=false;
    for(size_t n=result.path.size()-1;n>0;--n) {
      if((result.path[n]-result.path.front()).norm()<.3) continue;
      if(clipEpiconCorridorToBoxes(result.path.front(),result.path[n],box_lower,box_upper,result.planes,min_overlap)) {
        result.path.resize(n+1); result.truncated=true; prefix=true; break;
      }
    }
    if(!prefix) { result.failure="corridor disconnected inside task-box union"; return false; }
  }
  return true;
}
double EpiconFrontend::clearance(const Eigen::Vector3d &position) const {
  if (!hasCloud() || !position.allFinite()) return -std::numeric_limits<double>::infinity();
  return impl_->planner->lidar_map_interface_->getDisToOcc(position);
}
double EpiconFrontend::planningClearance() const { return impl_->planning_clearance; }
double EpiconFrontend::executionClearance() const { return impl_->execution_clearance; }
double EpiconFrontend::recoveryClearance() const {
  return std::max(planningClearance(),impl_->planner->parallel_path_finder_->safe_distance_+.02);
}
bool EpiconFrontend::connectedAt(const Eigen::Vector3d &position) const {
  if (!hasCloud() || !contains(position)) return false;
  const auto &i=*impl_; const auto &graph=*i.planner->topo_graph_;
  std::vector<epicon_native::TopoNode::Ptr> nodes;
  for (const auto &region:graph.reg_map_idx2ptr_) for (const auto &node:region.second->topo_nodes_) {
    if (node==graph.odom_node_ || (node->center_.cast<double>()-position).norm()>4.0) continue;
    // Match ready(): connect to a persistent skeleton node. Requiring that
    // node to already have another edge prevents bootstrapping small rooms.
    if (clearance(node->center_.cast<double>())>=planningClearance()) nodes.push_back(node);
  }
  std::sort(nodes.begin(),nodes.end(),[&](const epicon_native::TopoNode::Ptr &a,
                                      const epicon_native::TopoNode::Ptr &b) {
    return (a->center_.cast<double>()-position).squaredNorm()<
           (b->center_.cast<double>()-position).squaredNorm();
  });
  const auto deadline=ros::WallTime::now()+ros::WallDuration(.01);
  for (size_t n=0;n<std::min<size_t>(nodes.size(),12);++n) {
    const double remaining=(deadline-ros::WallTime::now()).toSec();
    if(remaining<=0) break;
    std::vector<Eigen::Vector3f> path;
    if (i.planner->parallel_path_finder_->search(position.cast<float>(),nodes[n]->center_,path,
        std::min(i.connection_timeout,remaining),true)==epicon_native::ParallelBubbleAstar::REACH_END &&
        i.planner->parallel_path_finder_->collisionCheck_shortenPath(path)) return true;
  }
  return false;
}
bool EpiconFrontend::contains(const Eigen::Vector3d &position) const {
  return position.allFinite() && impl_->planner->lidar_map_interface_->IsInBox(position.cast<float>().eval());
}
bool EpiconFrontend::recoveryPoint(const Eigen::Vector3d &start, Eigen::Vector3d &goal) const {
  if (!hasCloud() || !contains(start) || clearance(start)<impl_->physical_floor) return false;
  // A recovery episode ends only with measured exploration progress or a new
  // task. Time passing cannot re-enable a previously failed relocation loop.
  if(impl_->recovery_history.size()>=8) {
    ROS_WARN_THROTTLE(2.0,"[exploration recovery] recovery budget exhausted; holding for a reachable goal");
    return false;
  }
  epicon_native::PointVector points;
  const Eigen::Vector3f extent(2,2,1);
  impl_->planner->lidar_map_interface_->boxSearch(start.cast<float>()-extent,start.cast<float>()+extent,points);
  std::vector<Eigen::Vector3d> surface; surface.reserve(points.size());
  for (const auto &p:points) surface.emplace_back(p.x,p.y,p.z);
  std::vector<Eigen::Vector3d> candidates;
  Eigen::Vector3d interior;
  if (epiconRecoveryPoint(start,surface,interior)) {
    candidates.push_back(interior);
    interior.z()=start.z(); candidates.push_back(interior);
  }
  for (const auto &node:impl_->planner->topo_graph_->history_odom_nodes_)
    if ((node->center_.cast<double>()-start).norm()<=2.5) candidates.push_back(node->center_.cast<double>());
  for(auto pose=impl_->safe_history.rbegin();pose!=impl_->safe_history.rend();++pose) {
    Eigen::Vector3d delta=pose->cast<double>()-start;
    if(delta.norm()>.2 && delta.norm()<3.0)
      candidates.push_back(start+delta*std::min(1.0,1.0/delta.norm()));
  }
  // Also handle a root that has clearance but is on the wrong side of a turn.
  // Prefer level moves; a single convex interior can oscillate vertically.
  for (double dz:{-.35,.35,-.7,.7,-1.05,1.05})
    candidates.push_back(start+Eigen::Vector3d(0,0,dz));
  for (double dz:{0.0,-.35,.35,-.7,.7}) for (double radius:{.4,.8,1.2,1.6})
    for (int k=0;k<12;++k) {
      const double angle=k*2.0*M_PI/12.0;
      candidates.push_back(start+Eigen::Vector3d(radius*std::cos(angle),radius*std::sin(angle),dz));
    }
  struct Candidate { Eigen::Vector3d point; double score; };
  std::vector<Candidate> safe_candidates;
  std::vector<Eigen::Vector3d> centers;
  for(const auto &region:impl_->planner->topo_graph_->reg_map_idx2ptr_)
    for(const auto &node:region.second->topo_nodes_)
      if(node!=impl_->planner->topo_graph_->odom_node_) centers.push_back(node->center_.cast<double>());
  auto distance_to_graph=[&](const Eigen::Vector3d &p) {
    double distance=4.0;
    for(const auto &center:centers) distance=std::min(distance,(center-p).norm());
    return distance;
  };
  const double required=recoveryClearance();
  const auto &recent=impl_->recovery_history;
  for (auto candidate:candidates) {
    Eigen::Vector3d delta=candidate-start;
    if(delta.norm()>1.0) candidate=start+delta.normalized();
    const double distance=(candidate-start).norm();
    if(distance<.15 || !contains(candidate) || clearance(candidate)<required) continue;
    bool repeated=false;
    for(const auto &entry:recent) if((candidate-entry.first).norm()<.25) repeated=true;
    if(repeated) continue;
    bool safe=true; double best=std::min(required,clearance(start));
    const int samples=std::max(1,static_cast<int>(std::ceil(distance/.04)));
    for(int k=1;k<=samples;++k) {
      const Eigen::Vector3d p=start+(candidate-start)*(static_cast<double>(k)/samples);
      const double d=clearance(p);
      if(!contains(p) || d<impl_->physical_floor || d+.02<best) { safe=false; break; }
      best=std::max(best,std::min(required,d));
    }
    if(!safe) continue;
    double history_distance=2.0;
    for(const auto &pose:impl_->safe_history)
      if((pose.cast<double>()-start).norm()>.25)
        history_distance=std::min(history_distance,(pose.cast<double>()-candidate).norm());
    const double score=distance_to_graph(candidate)+.5*history_distance+
        .75*std::abs(candidate.z()-start.z())+.1*distance-.2*std::min(2.0,clearance(candidate));
    safe_candidates.push_back({candidate,score});
  }
  std::sort(safe_candidates.begin(),safe_candidates.end(),[](const auto &a,const auto &b) {
    return a.score<b.score;
  });
  if(safe_candidates.empty()) {
    ROS_WARN_STREAM_THROTTLE(1.0,"[exploration recovery] no locally certified target; clearance="
        <<clearance(start)<<" required="<<required);
    return false;
  }
  // Graph connection is a preference, not a prerequisite for a bounded local
  // escape. The backend certifies the entire trajectory again before commit.
  goal=safe_candidates.front().point;
  bool connected=false;
  const auto deadline=ros::WallTime::now()+ros::WallDuration(.03);
  for(size_t n=0;n<std::min<size_t>(3,safe_candidates.size()) && ros::WallTime::now()<deadline;++n)
    if(connectedAt(safe_candidates[n].point)) { goal=safe_candidates[n].point; connected=true; break; }
  ROS_INFO_STREAM_THROTTLE(1.0,"[exploration recovery] certified local target="<<goal.transpose()
      <<" graph_connected="<<connected<<" clearance="<<clearance(goal));
  return true;
}
bool EpiconFrontend::noteExplorationProgress() {
  auto &i=*impl_;
  // Retreat navigation has its own finite budget/timeout. It can keep the
  // motion watchdog alive, but cannot replenish recovery or retreat attempts.
  if(i.backtracking) return true;
  for(const auto &old:i.progress_regions) if((old-i.pos).norm()<.75f) return false;
  i.progress_regions.push_back(i.pos);
  i.recovery_history.clear();
  i.backtrack_attempts.clear(); i.backtrack_origins.clear();
  return true;
}
const std::vector<Eigen::Vector3f> &EpiconFrontend::executionPath() const { return impl_->execution_path; }
void EpiconFrontend::noteRecoveryAttempt(const Eigen::Vector3d &goal) {
  impl_->recovery_history.push_back({goal,ros::WallTime::now()});
  if(impl_->recovery_history.size()>12) impl_->recovery_history.erase(impl_->recovery_history.begin());
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

namespace fast_planner {
std::string EpiconFrontend::unresolvedSummary() const {
  std::size_t dormant=0,timeout=0,unresolved=0,connected=0,no_viewpoint=0;
  for(const auto &c:impl_->frontier->cluster_list_) {
    dormant+=c->is_dormant_;
    timeout+=c->connection_state_==3;
    connected+=c->connection_state_==1;
    no_viewpoint+=c->connection_state_==4;
    unresolved+=c->connection_state_==0 || c->connection_state_==2;
  }
  std::ostringstream out;
  out << "residual_clusters=" << clusterCount() << " dormant=" << dormant
      << " connected=" << connected << " connection_timeout=" << timeout
      << " connection_unresolved=" << unresolved << " no_sampled_viewpoint=" << no_viewpoint
      << " deferred=" << impl_->deferred.size();
  return out.str();
}
}
