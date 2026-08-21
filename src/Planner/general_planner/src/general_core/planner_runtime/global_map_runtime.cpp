#include <general_core/planner_runtime/global_map_runtime.hpp>

#include <general_core/exploration/exploration_utils/lidar_map/lidar_map.h>
#include <general_planner/TopologyFrontierPoint.h>
#include <general_planner/TopologyFrontierPointArray.h>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace general_planner::planner_runtime {

GlobalMapRuntime::~GlobalMapRuntime() {
  sync_.reset();
  cloud_sub_.reset();
  odom_sync_sub_.reset();
  odom_sub_.shutdown();
  topology_frontier_timer_.stop();
  topology_maintainer_.reset();
}

void GlobalMapRuntime::init(ros::NodeHandle nh,
                            const std::string &map_config_path) {
  if (initialized_) {
    throw std::logic_error("GlobalMapRuntime::init called twice");
  }
  if (map_config_path.empty()) {
    throw std::invalid_argument("global map config path is empty");
  }

  nh_ = std::move(nh);
  nh_.param("global_map/max_odom_age", max_odom_age_, 0.20);
  nh_.param("global_map/max_cloud_age", max_cloud_age_, 0.50);
  max_odom_age_ = std::clamp(max_odom_age_, 0.05, 5.0);
  max_cloud_age_ = std::max(0.0, max_cloud_age_);

  context_ = std::make_shared<GlobalMapContext>();
  context_->rog_map = std::make_shared<rog_map::ROGMapROS>(nh_, map_config_path);
  if (context_->rog_map->getMapConfig().ros_callback_en) {
    throw std::invalid_argument(
        "GlobalMapRuntime requires rog_map/ros_callback/enable=false; "
        "otherwise ROGMapROS would fuse cloud frames a second time");
  }
  context_->map_manager = std::make_shared<MapManager>(context_->rog_map);
  context_->map_manager->setWorldEpoch(
      context_->world_epoch.load(std::memory_order_acquire));

  // No task-mode callback: topology remains active in exploration, navigation,
  // WAIT and stable hold for the complete world lifetime.
  topology_maintainer_ = std::make_unique<TopologyGraphROS1>(
      nh_, context_->map_manager, "global_topology");

  std::string topology_frontier_topic{
      "/planner/world/topology_frontier_points"};
  nh_.param("global_topology/frame_id", topology_frame_id_, topology_frame_id_);
  nh_.param("global_topology/frontier_topic", topology_frontier_topic,
            topology_frontier_topic);
  nh_.param("global_topology/frontier_publish_period",
            topology_frontier_publish_period_, topology_frontier_publish_period_);
  nh_.param("global_topology/frontier_probe_radius",
            topology_frontier_probe_radius_, topology_frontier_probe_radius_);
  nh_.param("global_topology/frontier_probe_step",
            topology_frontier_probe_step_, topology_frontier_probe_step_);
  int frontier_min_unknown_directions =
      static_cast<int>(topology_frontier_min_unknown_directions_);
  nh_.param("global_topology/frontier_min_unknown_directions",
            frontier_min_unknown_directions, frontier_min_unknown_directions);
  topology_frontier_publish_period_ = std::max(
      0.05, topology_frontier_publish_period_);
  topology_frontier_probe_radius_ = std::max(
      0.05, topology_frontier_probe_radius_);
  topology_frontier_probe_step_ = std::max(0.0, topology_frontier_probe_step_);
  topology_frontier_min_unknown_directions_ = static_cast<std::uint8_t>(
      std::clamp(frontier_min_unknown_directions, 1, 26));
  topology_frontier_pub_ =
      nh_.advertise<general_planner::TopologyFrontierPointArray>(
          topology_frontier_topic, 1, true);
  topology_frontier_timer_ = nh_.createWallTimer(
      ros::WallDuration(topology_frontier_publish_period_),
      &GlobalMapRuntime::topologyFrontierTimerCallback, this);

  std::string odom_topic{"/lidar_slam/odom"};
  std::string cloud_topic{"/cloud_registered"};
  int odom_queue = 50;
  int cloud_queue = 1;
  int sync_queue = 20;
  nh_.param<std::string>("odometry_topic", odom_topic, odom_topic);
  nh_.param<std::string>("cloud_topic", cloud_topic, cloud_topic);
  nh_.param("global_map/odom_queue", odom_queue, odom_queue);
  nh_.param("global_map/cloud_queue", cloud_queue, cloud_queue);
  nh_.param("global_map/sync_queue", sync_queue, sync_queue);
  odom_queue = std::max(1, odom_queue);
  cloud_queue = std::max(1, cloud_queue);
  sync_queue = std::max(1, sync_queue);

  odom_sub_ = nh_.subscribe(odom_topic, odom_queue,
                             &GlobalMapRuntime::odomCallback, this,
                             ros::TransportHints().tcpNoDelay());
  cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
      nh_, cloud_topic, cloud_queue);
  odom_sync_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(
      nh_, odom_topic, odom_queue);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue), *cloud_sub_, *odom_sync_sub_);
  sync_->registerCallback(
      boost::bind(&GlobalMapRuntime::cloudOdomCallback, this, _1, _2));

  initialized_ = true;
  ROS_INFO_STREAM("[global_map_runtime] unique ROG/MapManager ready: cloud="
                  << cloud_topic << " odom=" << odom_topic
                  << " topology="
                  << (topology_maintainer_->enabled() ? "enabled" : "disabled")
                  << " topology_frontier_topic="
                  << nh_.resolveName(topology_frontier_topic)
                  << " map_config=" << map_config_path);
}

void GlobalMapRuntime::attachLioMap(
    const std::shared_ptr<fast_planner::LIOInterface> &lio_map) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!context_) {
    throw std::logic_error("attachLioMap before GlobalMapRuntime::init");
  }
  context_->lio_map = lio_map;
}

void GlobalMapRuntime::addCloudConsumer(CloudConsumer consumer) {
  if (!consumer) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  cloud_consumers_.push_back(std::move(consumer));
}

void GlobalMapRuntime::addOdomConsumer(OdomConsumer consumer) {
  if (!consumer) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  odom_consumers_.push_back(std::move(consumer));
}

void GlobalMapRuntime::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  if (!msg || !context_ || !context_->rog_map) {
    return;
  }
  context_->rog_map->ingestOdometry(msg);

  std::vector<OdomConsumer> consumers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_odom_time_ = ros::Time::now();
    consumers = odom_consumers_;
  }
  for (const auto &consumer : consumers) {
    consumer(msg);
  }
}

void GlobalMapRuntime::cloudOdomCallback(
    const sensor_msgs::PointCloud2ConstPtr &cloud,
    const nav_msgs::OdometryConstPtr &odom) {
  if (!cloud || !odom || !context_ || !context_->map_manager) {
    return;
  }
  const ros::Time now = ros::Time::now();
  if (max_cloud_age_ > 0.0 && !now.isZero() && !cloud->header.stamp.isZero() &&
      (now - cloud->header.stamp).toSec() > max_cloud_age_) {
    ROS_WARN_STREAM_THROTTLE(1.0,
        "[global_map_runtime] drop stale cloud age="
        << (now - cloud->header.stamp).toSec()
        << "s max=" << max_cloud_age_);
    return;
  }

  rog_map::PointCloud rog_cloud;
  pcl::fromROSMsg(*cloud, rog_cloud);
  if (rog_cloud.empty()) {
    return;
  }
  general_utils::Pose pose;
  pose.first = rog_map::Vec3f(odom->pose.pose.position.x,
                               odom->pose.pose.position.y,
                               odom->pose.pose.position.z);
  pose.second = rog_map::Quatf(odom->pose.pose.orientation.w,
                                odom->pose.pose.orientation.x,
                                odom->pose.pose.orientation.y,
                                odom->pose.pose.orientation.z);

  std::shared_ptr<fast_planner::LIOInterface> lio_map;
  std::vector<CloudConsumer> consumers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lio_map = context_->lio_map;
    consumers = cloud_consumers_;
  }
  // LIO and ROG are each updated exactly once for an accepted sensor pair.
  if (lio_map) {
    lio_map->updateCloudMapOdometry(cloud, odom);
  }
  const MapManager::UpdateSnapshot update =
      context_->map_manager->updateMap(rog_cloud, pose);
  // Topology is a world-lifetime map product, not a navigation-mode timer
  // side effect.  Request maintenance only after MapManager has stored this
  // accepted fusion's revision and dirty bounds, so the worker can seed from
  // valid odometry and consume the new free-space evidence immediately.
  // The request is coalesced and the worker remains rate-limited.
  if (topology_maintainer_) {
    topology_maintainer_->updateAndPublish();
  }
  publishTopologyFrontierSnapshot();
  context_->sensor_revision.fetch_add(1, std::memory_order_acq_rel);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_map_time_ = now;
  }
  for (const auto &consumer : consumers) {
    consumer(cloud, odom, update);
  }
}

void GlobalMapRuntime::publishTopologyFrontierSnapshot() {
  std::lock_guard<std::mutex> lock(topology_frontier_mutex_);
  if (!context_ || !context_->map_manager || !topology_frontier_pub_) {
    return;
  }

  const auto snapshot = context_->map_manager->topologySnapshot();
  const ros::WallTime now = ros::WallTime::now();
  const bool revision_changed =
      !topology_frontier_published_ ||
      snapshot.revision != last_topology_frontier_revision_;
  if (!revision_changed && !last_topology_frontier_publish_time_.isZero() &&
      (now - last_topology_frontier_publish_time_).toSec() <
          topology_frontier_publish_period_) {
    return;
  }

  IncrementalTopologyGraph::FrontierQueryConfig frontier_config;
  frontier_config.sample_step = topology_frontier_probe_step_;
  frontier_config.probe_radius = topology_frontier_probe_radius_;
  frontier_config.min_unknown_directions =
      topology_frontier_min_unknown_directions_;
  const auto frontier_evidence =
      context_->map_manager->classifyTopologyFrontiers(
          snapshot, frontier_config);

  general_planner::TopologyFrontierPointArray message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = topology_frame_id_;
  message.world_epoch =
      context_->world_epoch.load(std::memory_order_acquire);
  message.map_revision = context_->map_manager->mapRevision();
  message.topology_revision = snapshot.revision;
  message.topology_node_count = static_cast<std::uint32_t>(snapshot.nodes.size());
  message.points.reserve(frontier_evidence.size());

  std::unordered_map<IncrementalTopologyGraph::NodeId, std::uint32_t> degree;
  degree.reserve(snapshot.nodes.size());
  for (const auto &edge : snapshot.edges) {
    ++degree[edge.from];
    ++degree[edge.to];
  }

  std::unordered_map<IncrementalTopologyGraph::NodeId,
                     const IncrementalTopologyGraph::Node *> nodes;
  nodes.reserve(snapshot.nodes.size());
  for (const auto &node : snapshot.nodes) {
    nodes.emplace(node.id, &node);
  }
  for (const auto &frontier : frontier_evidence) {
    const auto node_it = nodes.find(frontier.id);
    if (node_it == nodes.end()) {
      continue;
    }
    const auto &node = *node_it->second;
    const auto degree_it = degree.find(node.id);
    general_planner::TopologyFrontierPoint point;
    point.id = node.id;
    point.position.x = node.position.x();
    point.position.y = node.position.y();
    point.position.z = node.position.z();
    point.boundary_position.x = frontier.boundary_position.x();
    point.boundary_position.y = frontier.boundary_position.y();
    point.boundary_position.z = frontier.boundary_position.z();
    point.boundary_normal.x = frontier.boundary_normal.x();
    point.boundary_normal.y = frontier.boundary_normal.y();
    point.boundary_normal.z = frontier.boundary_normal.z();
    point.clearance = static_cast<float>(node.clearance);
    point.boundary_distance = static_cast<float>(frontier.boundary_distance);
    point.degree = degree_it == degree.end() ? 0U : degree_it->second;
    point.state = node.state == IncrementalTopologyGraph::NodeState::ACTIVE
        ? general_planner::TopologyFrontierPoint::STATE_ACTIVE
        : general_planner::TopologyFrontierPoint::STATE_HISTORICAL;
    point.unknown_direction_mask = frontier.unknown_direction_mask;
    point.unknown_direction_count = frontier.unknown_direction_count;
    point.node_revision = node.revision;
    point.last_observed_revision = node.last_observed_revision;
    message.points.push_back(std::move(point));
  }
  message.frontier_count = static_cast<std::uint32_t>(message.points.size());

  topology_frontier_pub_.publish(message);
  last_topology_frontier_publish_time_ = now;
  last_topology_frontier_revision_ = snapshot.revision;
  topology_frontier_published_ = true;
}

void GlobalMapRuntime::topologyFrontierTimerCallback(
    const ros::WallTimerEvent &) {
  publishTopologyFrontierSnapshot();
}

GlobalMapStatus GlobalMapRuntime::status() const {
  GlobalMapStatus result;
  if (!context_ || !context_->map_manager) {
    return result;
  }
  const ros::Time now = ros::Time::now();
  ros::Time last_odom;
  ros::Time last_map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_odom = last_odom_time_;
    last_map = last_map_time_;
  }
  result.world_epoch = context_->world_epoch.load(std::memory_order_acquire);
  result.sensor_revision =
      context_->sensor_revision.load(std::memory_order_acquire);
  result.map_revision = context_->map_manager->mapRevision();
  result.odom_valid = !last_odom.isZero() &&
      (now - last_odom).toSec() <= max_odom_age_;
  result.map_ready = !last_map.isZero() && result.map_revision > 0;
  const auto snapshot = context_->map_manager->topologySearchSnapshot();
  result.topo_revision = snapshot ? snapshot->revision : 0;
  result.topology_ready = result.map_ready &&
      context_->map_manager->topologyReady() && result.topo_revision > 0;
  return result;
}

}  // namespace general_planner::planner_runtime
