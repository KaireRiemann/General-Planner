#include <map_manager/topology_graph_ros1.hpp>

#ifdef USE_ROS1

#include <algorithm>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace general_planner {

TopologyGraphROS1::TopologyGraphROS1(
    const ros::NodeHandle &node,
    const MapManager::Ptr &map_manager,
    const std::string &parameter_namespace,
    std::function<bool()> mission_active)
    : node_(node), map_manager_(map_manager),
      mission_active_(std::move(mission_active)) {
    IncrementalTopologyGraph::Config config =
        map_manager && map_manager->topologyGraph()
            ? map_manager->topologyGraph()->config()
            : IncrementalTopologyGraph::Config{};
    const std::string prefix = parameter_namespace.empty()
        ? std::string{} : parameter_namespace + "/";
    node_.param(prefix + "enabled", config.enabled, config.enabled);
    std::string construction_mode =
        IncrementalTopologyGraph::constructionModeName(
            config.construction_mode);
    node_.param(prefix + "construction_mode", construction_mode,
                construction_mode);
    config.construction_mode =
        IncrementalTopologyGraph::constructionModeFromString(
            construction_mode);
    node_.param(prefix + "planar_mode", config.planar_mode, config.planar_mode);
    node_.param(prefix + "navigation_altitude", config.navigation_altitude,
                config.navigation_altitude);
    node_.param(prefix + "region_size", config.region_size, config.region_size);
    node_.param(prefix + "sample_spacing", config.sample_spacing, config.sample_spacing);
    node_.param(prefix + "min_clearance", config.min_clearance, config.min_clearance);
    node_.param(prefix + "max_clearance", config.max_clearance, config.max_clearance);
    node_.param(prefix + "candidate_separation", config.candidate_separation,
                config.candidate_separation);
    node_.param(prefix + "stable_match_distance", config.stable_match_distance,
                config.stable_match_distance);
    node_.param(prefix + "connection_radius", config.connection_radius,
                config.connection_radius);
    node_.param(prefix + "edge_sample_spacing", config.edge_sample_spacing,
                config.edge_sample_spacing);
    node_.param(prefix + "dirty_padding", config.dirty_padding, config.dirty_padding);
    node_.param(prefix + "bubble_overlap_margin", config.bubble_overlap_margin,
                config.bubble_overlap_margin);
    node_.param(prefix + "unknown_as_free", config.unknown_as_free,
                config.unknown_as_free);
    node_.param(prefix + "snapshot_every_update",
                config.snapshot_every_update,
                config.snapshot_every_update);

    int max_nodes = static_cast<int>(config.max_nodes_per_region);
    int max_bubbles = static_cast<int>(config.max_bubbles_per_region);
    int max_neighbors = static_cast<int>(config.max_neighbors);
    int max_regions = static_cast<int>(config.max_regions_per_update);
    node_.param(prefix + "max_nodes_per_region", max_nodes, max_nodes);
    node_.param(prefix + "max_bubbles_per_region", max_bubbles, max_bubbles);
    node_.param(prefix + "max_neighbors", max_neighbors, max_neighbors);
    node_.param(prefix + "max_regions_per_update", max_regions, max_regions);
    node_.param(prefix + "require_fresh_focus", config.require_fresh_focus,
                config.require_fresh_focus);
    node_.param(prefix + "focus_timeout", config.focus_timeout,
                config.focus_timeout);
    node_.param(prefix + "max_focus_distance", config.max_focus_distance,
                config.max_focus_distance);
    config.max_nodes_per_region = static_cast<std::size_t>(std::max(1, max_nodes));
    config.max_bubbles_per_region = static_cast<std::size_t>(std::max(1, max_bubbles));
    config.max_neighbors = static_cast<std::size_t>(std::max(1, max_neighbors));
    config.max_regions_per_update = static_cast<std::size_t>(std::max(1, max_regions));
    max_regions_per_tick_ = config.max_regions_per_update;

    double update_period = config.update_period;
    double publish_period = config.publish_period;
    std::string topic{"map_manager/topology"};
    node_.param(prefix + "frame_id", frame_id_, frame_id_);
    node_.param(prefix + "topic", topic, topic);
    node_.param(prefix + "update_period", update_period, update_period);
    node_.param(prefix + "publish_period", publish_period, publish_period);
    update_period_ = std::max(0.02, update_period);
    publish_period_ = std::max(0.05, publish_period);
    config.update_period = update_period_;
    config.publish_period = publish_period_;
    node_.param(prefix + "node_scale", node_scale_, node_scale_);
    node_.param(prefix + "edge_scale", edge_scale_, edge_scale_);

    enabled_ = config.enabled && static_cast<bool>(map_manager);
    if (!enabled_) {
        return;
    }
    map_manager->configureTopology(config);
    map_manager->setTopologyActive(missionActive());
    publisher_ = node_.advertise<visualization_msgs::MarkerArray>(topic, 1, true);
    worker_ = std::thread(&TopologyGraphROS1::workerLoop, this);
    timer_ = node_.createTimer(ros::Duration(update_period_),
                               &TopologyGraphROS1::timerCallback, this);
    if (missionActive()) {
        updateAndPublish();
    }
    ROS_INFO_STREAM("[map_manager] Incremental topology enabled, mode: "
                    << IncrementalTopologyGraph::constructionModeName(
                           config.construction_mode)
                    << ", topic: "
                    << node_.resolveName(topic)
                    << ", update=" << config.update_period
                    << "s, publish=" << publish_period_
                    << "s, asynchronous worker active");
}

TopologyGraphROS1::~TopologyGraphROS1() {
    timer_.stop();
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        stopping_ = true;
    }
    worker_cv_.notify_one();
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool TopologyGraphROS1::missionActive() const {
    return !mission_active_ || mission_active_();
}

void TopologyGraphROS1::timerCallback(const ros::TimerEvent &) {
    // The worker performs the mission gate and owns the periodic fallback.
    // Keep this ROS-timer path as a compatibility wake-up only: when the
    // shared queue is busy it may be delayed indefinitely, which must not
    // suppress maintenance of the global topology.
    updateAndPublish();
}

void TopologyGraphROS1::setMaintenanceEnabled(const bool enabled) {
    if (!enabled_) {
        return;
    }
    const bool previous = maintenance_enabled_.exchange(
        enabled, std::memory_order_acq_rel);
    if (previous == enabled) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        if (!enabled) {
            // Do not process a dirty-region backlog after a terminal HOLD.
            // Map evidence remains in MapManager and is considered only once
            // a subsequent active task explicitly resumes maintenance.
            update_requested_ = false;
        }
    }
    worker_cv_.notify_one();
    ROS_INFO_STREAM("[topology update] maintenance "
                    << (enabled ? "resumed" : "frozen"));
}

void TopologyGraphROS1::updateAndPublish() {
    if (!enabled_ || !maintenanceEnabled()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        update_requested_ = true;
    }
    worker_cv_.notify_one();
}

void TopologyGraphROS1::workerLoop() {
    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(update_period_));
    // Run once immediately for startup, then no faster than update_period_.
    // Requests received in between are intentionally coalesced.
    auto next_update = Clock::now();
    for (;;) {
        {
            std::unique_lock<std::mutex> lock(worker_mutex_);
            if (!maintenance_enabled_.load(std::memory_order_acquire)) {
                update_requested_ = false;
                worker_cv_.wait(lock, [this]() {
                    return stopping_ ||
                           maintenance_enabled_.load(std::memory_order_acquire);
                });
                if (stopping_) {
                    return;
                }
                // Resume from the current map state instead of trying to
                // catch up with timer periods that elapsed while frozen.
                next_update = Clock::now();
            }
            worker_cv_.wait_until(lock, next_update, [this]() {
                return stopping_ || update_requested_ ||
                       !maintenance_enabled_.load(std::memory_order_acquire);
            });
            if (stopping_) {
                return;
            }
            if (!maintenance_enabled_.load(std::memory_order_acquire)) {
                continue;
            }
            // A map-fusion request may arrive before the configured budget is
            // available.  Preserve the rate limit while retaining the request
            // as coalesced work for this tick.  Only shutdown may interrupt
            // this wait; otherwise a high-rate cloud topic could spin the
            // worker at sensor rate.
            if (Clock::now() < next_update) {
                worker_cv_.wait_until(lock, next_update, [this]() {
                    return stopping_ ||
                           !maintenance_enabled_.load(std::memory_order_acquire);
                });
                if (stopping_) {
                    return;
                }
                if (!maintenance_enabled_.load(std::memory_order_acquire)) {
                    continue;
                }
            }
            update_requested_ = false;
        }

        const auto manager = map_manager_.lock();
        if (enabled_ && maintenanceEnabled() && manager) {
            // This must live in the worker rather than the ROS timer.  The
            // runtime-wide graph keeps its data, but may suspend mutations
            // during a verified HOLD; standalone users still receive the
            // mission gate at the configured maintenance cadence.
            const bool active = missionActive();
            manager->setTopologyActive(active);
            if (active && manager->topologyReady()) {
                const ros::WallTime update_begin = ros::WallTime::now();
                const std::size_t processed =
                    manager->updateTopology(max_regions_per_tick_);
                const double update_ms =
                    (ros::WallTime::now() - update_begin).toSec() * 1000.0;
                const auto stats = manager->topologyGraph()->stats();
                ROS_INFO_STREAM_THROTTLE(
                    1.0, "[topology update] processed=" << processed
                         << " dirty=" << stats.dirty_region_count
                         << " raw_nodes=" << stats.node_count
                         << " public_nodes=" << stats.public_node_count
                         << " pending=" << stats.pending_node_count
                         << " isolated=" << stats.isolated_node_count
                         << " raw_edges=" << stats.edge_count
                         << " public_edges=" << stats.public_edge_count
                         << " cost=" << update_ms << "ms");
                if (update_ms > update_period_ * 1000.0) {
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[topology update] worker exceeds period: cost="
                             << update_ms << "ms period="
                             << update_period_ * 1000.0
                             << "ms; updates are being coalesced");
                }
                const ros::WallTime now = ros::WallTime::now();
                if (last_publish_time_.isZero() ||
                    (now - last_publish_time_).toSec() >= publish_period_) {
                    manager->topologyGraph()->refreshSnapshot();
                    publisher_.publish(makeMarkers(manager->topologySnapshot()));
                    last_publish_time_ = now;
                }
            }
        }
        // Do not attempt to catch up after a costly rebuild: a bounded rate
        // keeps topology maintenance from competing with map fusion.
        next_update = Clock::now() + period;
    }
}

visualization_msgs::MarkerArray TopologyGraphROS1::makeMarkers(
    const IncrementalTopologyGraph::Snapshot &snapshot) const {
    visualization_msgs::MarkerArray output;
    const ros::Time stamp = ros::Time::now();

    visualization_msgs::Marker nodes;
    nodes.header.frame_id = frame_id_;
    nodes.header.stamp = stamp;
    nodes.ns = "incremental_topology_nodes";
    nodes.id = 0;
    nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes.action = visualization_msgs::Marker::ADD;
    nodes.pose.orientation.w = 1.0;
    nodes.scale.x = node_scale_;
    nodes.scale.y = node_scale_;
    nodes.scale.z = node_scale_;
    nodes.color.r = 1.0F;
    nodes.color.g = 1.0F;
    nodes.color.b = 1.0F;
    nodes.color.a = 1.0F;

    std::unordered_map<IncrementalTopologyGraph::NodeId, rog_map::Vec3f> positions;
    positions.reserve(snapshot.nodes.size());
    std::unordered_set<IncrementalTopologyGraph::NodeId> edge_endpoints;
    edge_endpoints.reserve(snapshot.edges.size() * 2U);
    for (const auto &edge : snapshot.edges) {
        edge_endpoints.insert(edge.from);
        edge_endpoints.insert(edge.to);
    }
    std::size_t historical_nodes = 0;
    for (const auto &node : snapshot.nodes) {
        // Snapshot publication already enforces this invariant.  Keep the
        // visualization boundary defensive so a malformed external snapshot
        // can never recreate a dot without an incident edge in RViz.
        if (edge_endpoints.count(node.id) == 0U) {
            continue;
        }
        geometry_msgs::Point point;
        point.x = node.position.x();
        point.y = node.position.y();
        point.z = node.position.z();
        nodes.points.push_back(point);
        std_msgs::ColorRGBA color;
        if (node.state == IncrementalTopologyGraph::NodeState::HISTORICAL) {
            color.r = 0.95F;
            color.g = 0.65F;
            color.b = 0.15F;
            color.a = 0.80F;
            ++historical_nodes;
        } else {
            color.r = 0.10F;
            color.g = 0.85F;
            color.b = 0.95F;
            color.a = 0.95F;
        }
        nodes.colors.push_back(color);
        positions.emplace(node.id, node.position);
    }
    output.markers.push_back(nodes);

    visualization_msgs::Marker edges;
    edges.header.frame_id = frame_id_;
    edges.header.stamp = stamp;
    edges.ns = "incremental_topology_edges";
    edges.id = 0;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = edge_scale_;
    edges.color.r = 0.20F;
    edges.color.g = 0.55F;
    edges.color.b = 1.00F;
    edges.color.a = 0.75F;
    for (const auto &edge : snapshot.edges) {
        const auto from = positions.find(edge.from);
        const auto to = positions.find(edge.to);
        if (from == positions.end() || to == positions.end()) {
            continue;
        }
        geometry_msgs::Point from_point;
        from_point.x = from->second.x();
        from_point.y = from->second.y();
        from_point.z = from->second.z();
        geometry_msgs::Point to_point;
        to_point.x = to->second.x();
        to_point.y = to->second.y();
        to_point.z = to->second.z();
        edges.points.push_back(from_point);
        edges.points.push_back(to_point);
    }
    output.markers.push_back(edges);

    visualization_msgs::Marker status;
    status.header.frame_id = frame_id_;
    status.header.stamp = stamp;
    status.ns = "incremental_topology_status";
    status.id = 0;
    status.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::Marker::ADD;
    status.pose.orientation.w = 1.0;
    if (!snapshot.nodes.empty()) {
        // Snapshot nodes come from an unordered graph. Anchoring the text at
        // front() made a passive RViz label jump around the map and look like
        // a random topology expansion. The oldest surviving node is stable
        // across snapshot iteration order and newly appended regions.
        const auto anchor = std::min_element(
            snapshot.nodes.begin(), snapshot.nodes.end(),
            [](const IncrementalTopologyGraph::Node &lhs,
               const IncrementalTopologyGraph::Node &rhs) {
                return lhs.id < rhs.id;
            });
        status.pose.position.x = anchor->position.x();
        status.pose.position.y = anchor->position.y();
        status.pose.position.z = anchor->position.z() + 0.5;
    }
    status.scale.z = std::max(0.20, 1.5 * node_scale_);
    status.color.r = 1.0F;
    status.color.g = 1.0F;
    status.color.b = 1.0F;
    status.color.a = 0.9F;
    status.text = std::string{"topology mode="} +
                  IncrementalTopologyGraph::constructionModeName(
                      snapshot.construction_mode) +
                  " nodes=" + std::to_string(snapshot.nodes.size()) +
                  " raw=" + std::to_string(snapshot.raw_node_count) +
                  " pending=" + std::to_string(snapshot.pending_node_count) +
                  " isolated=" + std::to_string(snapshot.isolated_node_count) +
                  " components=" +
                  std::to_string(snapshot.connected_component_count) +
                  " active=" +
                  std::to_string(snapshot.nodes.size() - historical_nodes) +
                  " historical=" + std::to_string(historical_nodes) +
                  " edges=" + std::to_string(snapshot.edges.size()) +
                  " known_free=" +
                  std::to_string(snapshot.known_free_cell_count) +
                  " dirty=" + std::to_string(snapshot.dirty_region_count) +
                  " rev=" + std::to_string(snapshot.revision) +
                  " empty=" + std::to_string(snapshot.empty_region_count) +
                  " last[sampled=" +
                  std::to_string(snapshot.last_sampled_center_count) +
                  " free=" +
                  std::to_string(snapshot.last_traversable_center_count) +
                  " clearance_reject=" +
                  std::to_string(snapshot.last_clearance_rejected_count) + "]";
    output.markers.push_back(status);
    return output;
}

} // namespace general_planner

#endif // USE_ROS1
