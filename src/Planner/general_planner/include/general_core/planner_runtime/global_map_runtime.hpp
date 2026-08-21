#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <map_manager/map_manager.hpp>
#include <map_manager/topology_graph_ros1.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace fast_planner {
class LIOInterface;
}

namespace general_planner::planner_runtime {

/**
 * Long-lived world-model handles.  This object deliberately contains no task
 * state: a task change may invalidate trajectories, but never replaces this
 * context.  Only GlobalMapRuntime may mutate the ROG map or topology setup.
 */
struct GlobalMapContext {
  using Ptr = std::shared_ptr<GlobalMapContext>;

  rog_map::ROGMapROS::Ptr rog_map;
  MapManager::Ptr map_manager;
  std::shared_ptr<fast_planner::LIOInterface> lio_map;

  std::atomic<std::uint64_t> world_epoch{1};
  std::atomic<std::uint64_t> sensor_revision{0};
};

struct GlobalMapStatus {
  std::uint64_t world_epoch{0};
  std::uint64_t sensor_revision{0};
  std::uint64_t map_revision{0};
  std::uint64_t topo_revision{0};
  bool odom_valid{false};
  bool map_ready{false};
  bool topology_ready{false};
};

/**
 * The only cloud/odom fusion owner in the composed M2 runtime.
 *
 * ROGMapROS must be configured with `rog_map/ros_callback/enable: false`.
 * The runtime receives raw odom plus one cloud/odom synchronized stream,
 * updates LIO and ROG exactly once, then notifies read-side task adapters.
 */
class GlobalMapRuntime {
 public:
  using Ptr = std::shared_ptr<GlobalMapRuntime>;
  using CloudConsumer = std::function<void(
      const sensor_msgs::PointCloud2ConstPtr &,
      const nav_msgs::OdometryConstPtr &,
      const MapManager::UpdateSnapshot &)>;
  using OdomConsumer = std::function<void(const nav_msgs::OdometryConstPtr &)>;

  GlobalMapRuntime() = default;
  ~GlobalMapRuntime();

  GlobalMapRuntime(const GlobalMapRuntime &) = delete;
  GlobalMapRuntime &operator=(const GlobalMapRuntime &) = delete;

  void init(ros::NodeHandle nh, const std::string &map_config_path);

  GlobalMapContext::Ptr context() const { return context_; }
  MapManager::Ptr mapManager() const {
    return context_ ? context_->map_manager : MapManager::Ptr{};
  }

  /** Attach the one exploration LIO map before the first cloud callback. */
  void attachLioMap(const std::shared_ptr<fast_planner::LIOInterface> &lio_map);
  void addCloudConsumer(CloudConsumer consumer);
  void addOdomConsumer(OdomConsumer consumer);

  GlobalMapStatus status() const;

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>;

  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                         const nav_msgs::OdometryConstPtr &odom);
  void topologyExpansionTimerCallback(const ros::WallTimerEvent &);
  /** Publish the latest immutable global-topology expansion snapshot. */
  void publishTopologyExpansionSnapshot();

  ros::NodeHandle nh_;
  GlobalMapContext::Ptr context_;
  std::unique_ptr<TopologyGraphROS1> topology_maintainer_;
  ros::Publisher topology_expansion_pub_;
  ros::WallTimer topology_expansion_timer_;

  ros::Subscriber odom_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
      cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sync_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  mutable std::mutex mutex_;
  std::mutex topology_expansion_mutex_;
  std::vector<CloudConsumer> cloud_consumers_;
  std::vector<OdomConsumer> odom_consumers_;
  ros::Time last_odom_time_;
  ros::Time last_map_time_;
  ros::WallTime last_topology_expansion_publish_time_;
  std::string topology_frame_id_{"world"};
  double topology_expansion_publish_period_{0.50};
  std::uint64_t last_topology_expansion_revision_{0};
  bool topology_expansion_published_{false};
  double max_odom_age_{0.20};
  double max_cloud_age_{0.50};
  bool initialized_{false};
};

}  // namespace general_planner::planner_runtime
