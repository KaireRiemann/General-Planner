#pragma once

#ifdef USE_ROS1

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <map_manager/map_manager.hpp>

namespace general_planner {

/**
 * Asynchronous topology-maintenance adapter.
 *
 * ROS timers and map-fusion callbacks may request work, but the worker owns
 * the actual update cadence.  This is important for the composed runtime:
 * a busy ROS callback queue must never stop the world-lifetime topology from
 * consuming already-fused map evidence.
 */
class TopologyGraphROS1 {
public:
    using Ptr = std::shared_ptr<TopologyGraphROS1>;

    TopologyGraphROS1(const ros::NodeHandle &node,
                      const MapManager::Ptr &map_manager,
                      const std::string &parameter_namespace = "topology",
                      std::function<bool()> mission_active = {});
    ~TopologyGraphROS1();

    bool enabled() const { return enabled_; }
    /** Thread-safe, coalesced request after a successful map fusion. */
    void updateAndPublish();

private:
    void timerCallback(const ros::TimerEvent &);
    void workerLoop();
    bool missionActive() const;
    visualization_msgs::MarkerArray makeMarkers(
        const IncrementalTopologyGraph::Snapshot &snapshot) const;

    ros::NodeHandle node_;
    std::weak_ptr<MapManager> map_manager_;
    ros::Publisher publisher_;
    ros::Timer timer_;
    std::function<bool()> mission_active_;
    std::mutex worker_mutex_;
    std::condition_variable worker_cv_;
    std::thread worker_;
    bool update_requested_{false};
    bool stopping_{false};
    std::string frame_id_{"world"};
    bool enabled_{false};
    double node_scale_{0.22};
    double edge_scale_{0.06};
    double update_period_{0.20};
    double publish_period_{0.50};
    ros::WallTime last_publish_time_;
    std::size_t max_regions_per_tick_{4};
};

} // namespace general_planner

#endif // USE_ROS1
