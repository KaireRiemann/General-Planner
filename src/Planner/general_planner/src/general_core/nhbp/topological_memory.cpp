#include <general_core/nhbp/topological_memory.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace general_planner::nhbp {

TopologicalMemory::TopologicalMemory()
    : TopologicalMemory(Config{})
{
}

TopologicalMemory::TopologicalMemory(Config config)
    : config_(config)
{
}

void TopologicalMemory::reset()
{
    nodes_.clear();
    edges_.clear();
    next_node_id_ = 0;
    next_edge_id_ = 0;
}

int TopologicalMemory::observePose(const general_utils::Vec3f &position,
                                   const double stamp,
                                   const TopoNodeType node_type,
                                   const int floor_id)
{
    if (!config_.enable || !position.allFinite()) {
        return -1;
    }

    const double merge_radius = std::max(1.0e-3, config_.node_merge_radius);
    int node_id = nearestNode(position, merge_radius, stamp, true);
    if (node_id < 0) {
        node_id = next_node_id_++;
        TopoNode node;
        node.node_id = node_id;
        node.position = position;
        node.node_type = node_type;
        node.floor_id = floor_id;
        node.created_time = stamp;
        node.last_seen_time = stamp;
        node.visit_count = 1;
        nodes_[node_id] = node;
    } else {
        TopoNode &node = nodes_[node_id];
        const double visits = static_cast<double>(std::max(1, node.visit_count));
        node.position = (node.position * visits + position) / (visits + 1.0);
        node.last_seen_time = stamp;
        if (node_type != TopoNodeType::POSE) {
            node.node_type = node_type;
        }
        node.floor_id = floor_id;
        ++node.visit_count;
    }
    prune(stamp);
    return node_id;
}

int TopologicalMemory::observeVerticalConnector(const general_utils::Vec3f &position,
                                                const double stamp,
                                                const int floor_id)
{
    return observePose(position, stamp, TopoNodeType::VERTICAL_CONNECTOR, floor_id);
}

void TopologicalMemory::observeTransition(const general_utils::Vec3f &from,
                                          const general_utils::Vec3f &to,
                                          const double stamp)
{
    if (!config_.enable || !from.allFinite() || !to.allFinite()) {
        return;
    }

    const int from_node = observePose(from, stamp);
    const int to_node = observePose(to, stamp);
    if (from_node < 0 || to_node < 0 || from_node == to_node) {
        return;
    }

    TopoEdge &edge = edges_[edgeKey(from_node, to_node)];
    if (edge.edge_id < 0) {
        edge.edge_id = next_edge_id_++;
        edge.from_node = from_node;
        edge.to_node = to_node;
    }
    edge.length = (nodes_[from_node].position - nodes_[to_node].position).norm();
    edge.status = TopoEdgeStatus::VALID;
    edge.last_validated_time = stamp;
    if (edge.blacklist_until <= stamp) {
        edge.blacklist_until = 0.0;
    }
    prune(stamp);
}

void TopologicalMemory::recordFailureNear(const general_utils::Vec3f &position,
                                          const double stamp)
{
    if (!config_.enable || !position.allFinite()) {
        return;
    }

    const double lookup_radius = std::max(1.0e-3, config_.node_merge_radius * 2.0);
    const int node_id = nearestNode(position, lookup_radius, stamp, true);
    if (node_id < 0) {
        return;
    }

    TopoNode &node = nodes_[node_id];
    ++node.failure_count;
    node.blacklist_until = stamp + std::max(0.0, config_.node_blacklist_ttl);

    for (auto &entry : edges_) {
        TopoEdge &edge = entry.second;
        if (edge.from_node != node_id && edge.to_node != node_id) {
            continue;
        }
        ++edge.failure_count;
        edge.status = TopoEdgeStatus::BLOCKED;
        edge.blacklist_until = stamp + std::max(0.0, config_.edge_blacklist_ttl);
    }
}

bool TopologicalMemory::findRecoveryPosition(const general_utils::Vec3f &robot_pos,
                                              const double stamp,
                                              general_utils::Vec3f &position,
                                              const std::function<bool(const general_utils::Vec3f &)> &accept) const
{
    position = general_utils::Vec3f::Zero();
    if (!config_.enable || !robot_pos.allFinite()) {
        return false;
    }

    const double min_distance = std::max(0.0, config_.recovery_min_distance);
    const double max_distance = std::max(min_distance, config_.recovery_max_distance);
    const int current_node =
            nearestNode(robot_pos,
                        std::max(1.0e-3, config_.node_merge_radius * 2.0),
                        stamp,
                        false);

    const auto search_best = [&](const bool require_current_edge,
                                 general_utils::Vec3f &best_position) {
        double best_score = std::numeric_limits<double>::infinity();
        bool found = false;

        for (const auto &entry : nodes_) {
            const TopoNode &node = entry.second;
            if (!node.position.allFinite() || node.blacklist_until > stamp) {
                continue;
            }
            if (accept && !accept(node.position)) {
                continue;
            }
            const double distance = (node.position - robot_pos).norm();
            if (distance < min_distance || distance > max_distance) {
                continue;
            }

            double edge_penalty = 0.0;
            if (require_current_edge) {
                bool edge_found = false;
                for (const auto &edge_entry : edges_) {
                    const TopoEdge &edge = edge_entry.second;
                    const bool connected =
                            (edge.from_node == current_node && edge.to_node == node.node_id) ||
                            (edge.to_node == current_node && edge.from_node == node.node_id);
                    if (!connected || !edgeUsable(edge, stamp)) {
                        continue;
                    }
                    edge_found = true;
                    if (edge.status != TopoEdgeStatus::VALID) {
                        edge_penalty = std::max(edge_penalty, 1.0);
                    }
                    edge_penalty += 0.2 * static_cast<double>(edge.failure_count);
                }
                if (!edge_found) {
                    continue;
                }
            }

            const double recency = std::max(0.0, stamp - node.last_seen_time);
            const double score = distance +
                                 0.2 * recency +
                                 0.5 * static_cast<double>(node.failure_count) +
                                 edge_penalty;
            if (score < best_score) {
                best_score = score;
                best_position = node.position;
                found = true;
            }
        }
        return found;
    };

    if (current_node >= 0 && search_best(true, position)) {
        return true;
    }
    return search_best(false, position);
}

int TopologicalMemory::activeNodeCount(const double stamp) const
{
    int count = 0;
    for (const auto &entry : nodes_) {
        if (entry.second.blacklist_until <= stamp) {
            ++count;
        }
    }
    return count;
}

int TopologicalMemory::blockedNodeCount(const double stamp) const
{
    int count = 0;
    for (const auto &entry : nodes_) {
        if (entry.second.blacklist_until > stamp) {
            ++count;
        }
    }
    return count;
}

int TopologicalMemory::edgeCount() const
{
    return static_cast<int>(edges_.size());
}

int TopologicalMemory::nearestNode(const general_utils::Vec3f &position,
                                   const double max_distance,
                                   const double stamp,
                                   const bool allow_blocked) const
{
    const double max_distance_sq = max_distance * max_distance;
    double best_distance_sq = max_distance_sq;
    int best_node = -1;
    for (const auto &entry : nodes_) {
        const TopoNode &node = entry.second;
        if (!allow_blocked && node.blacklist_until > stamp) {
            continue;
        }
        const double distance_sq = (node.position - position).squaredNorm();
        if (distance_sq <= best_distance_sq) {
            best_distance_sq = distance_sq;
            best_node = node.node_id;
        }
    }
    return best_node;
}

std::string TopologicalMemory::edgeKey(const int from_node,
                                       const int to_node) const
{
    std::ostringstream oss;
    oss << from_node << ":" << to_node;
    return oss.str();
}

bool TopologicalMemory::edgeUsable(const TopoEdge &edge,
                                   const double stamp) const
{
    return edge.blacklist_until <= stamp &&
           edge.status != TopoEdgeStatus::BLOCKED &&
           edge.status != TopoEdgeStatus::STALE;
}

void TopologicalMemory::prune(const double stamp)
{
    if (!config_.enable) {
        reset();
        return;
    }

    for (auto &entry : edges_) {
        TopoEdge &edge = entry.second;
        if (edge.blacklist_until <= stamp && edge.status == TopoEdgeStatus::BLOCKED) {
            edge.status = TopoEdgeStatus::SUSPECT;
        }
    }

    const int max_edges = std::max(1, config_.max_edges);
    while (static_cast<int>(edges_.size()) > max_edges) {
        auto oldest = edges_.begin();
        for (auto it = edges_.begin(); it != edges_.end(); ++it) {
            if (it->second.last_validated_time < oldest->second.last_validated_time) {
                oldest = it;
            }
        }
        edges_.erase(oldest);
    }

    const int max_nodes = std::max(1, config_.max_nodes);
    while (static_cast<int>(nodes_.size()) > max_nodes) {
        auto oldest = nodes_.begin();
        for (auto it = nodes_.begin(); it != nodes_.end(); ++it) {
            if (it->second.last_seen_time < oldest->second.last_seen_time) {
                oldest = it;
            }
        }
        const int removed_node = oldest->first;
        nodes_.erase(oldest);
        for (auto it = edges_.begin(); it != edges_.end();) {
            if (it->second.from_node == removed_node || it->second.to_node == removed_node) {
                it = edges_.erase(it);
            } else {
                ++it;
            }
        }
    }
}

const char *toString(const TopoEdgeStatus status)
{
    switch (status) {
        case TopoEdgeStatus::UNKNOWN:
            return "UNKNOWN";
        case TopoEdgeStatus::VALID:
            return "VALID";
        case TopoEdgeStatus::SUSPECT:
            return "SUSPECT";
        case TopoEdgeStatus::BLOCKED:
            return "BLOCKED";
        case TopoEdgeStatus::STALE:
            return "STALE";
    }
    return "UNKNOWN";
}

const char *toString(const TopoNodeType type)
{
    switch (type) {
        case TopoNodeType::POSE:
            return "POSE";
        case TopoNodeType::BRANCH:
            return "BRANCH";
        case TopoNodeType::FRONTIER:
            return "FRONTIER";
        case TopoNodeType::FAILURE:
            return "FAILURE";
        case TopoNodeType::VERTICAL_CONNECTOR:
            return "VERTICAL_CONNECTOR";
    }
    return "UNKNOWN";
}

} // namespace general_planner::nhbp
