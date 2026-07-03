#include <general_core/nhbp/topological_memory.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <utility>

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

bool TopologicalMemory::findRecoveryPath(
        const general_utils::Vec3f &robot_pos,
        const double stamp,
        TopoPath &path,
        const std::function<bool(const general_utils::Vec3f &)> &accept) const
{
    path = TopoPath{};
    if (!config_.enable || !robot_pos.allFinite()) {
        path.reason = "disabled_or_invalid_robot";
        return false;
    }

    const int current_node =
            nearestNode(robot_pos,
                        std::max(1.0e-3, config_.node_merge_radius * 2.0),
                        stamp,
                        false);
    if (current_node < 0) {
        path.reason = "no_current_topology_node";
        return false;
    }

    const double min_distance = std::max(0.0, config_.recovery_min_distance);
    const double max_distance = std::max(min_distance, config_.recovery_max_distance);
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;
    std::unordered_map<int, int> prev_edge;
    using QueueEntry = std::pair<double, int>;
    std::priority_queue<QueueEntry,
                        std::vector<QueueEntry>,
                        std::greater<QueueEntry>>
            queue;

    dist[current_node] = 0.0;
    queue.push({0.0, current_node});

    while (!queue.empty()) {
        const auto [distance_so_far, node_id] = queue.top();
        queue.pop();
        const auto dist_it = dist.find(node_id);
        if (dist_it == dist.end() || distance_so_far > dist_it->second + 1.0e-6) {
            continue;
        }
        const auto node_it = nodes_.find(node_id);
        if (node_it == nodes_.end() || node_it->second.blacklist_until > stamp) {
            continue;
        }

        for (const auto &entry : edges_) {
            const TopoEdge &edge = entry.second;
            if (!edgeUsable(edge, stamp)) {
                continue;
            }
            int next_node = -1;
            if (edge.from_node == node_id) {
                next_node = edge.to_node;
            } else if (edge.to_node == node_id) {
                next_node = edge.from_node;
            } else {
                continue;
            }
            const auto next_it = nodes_.find(next_node);
            if (next_it == nodes_.end() || next_it->second.blacklist_until > stamp) {
                continue;
            }
            const double edge_length =
                    edge.length > 1.0e-3
                            ? edge.length
                            : (node_it->second.position - next_it->second.position).norm();
            const double next_distance = distance_so_far + edge_length;
            const auto old = dist.find(next_node);
            if (old != dist.end() && old->second <= next_distance) {
                continue;
            }
            dist[next_node] = next_distance;
            prev[next_node] = node_id;
            prev_edge[next_node] = edge.edge_id;
            queue.push({next_distance, next_node});
        }
    }

    int best_node = -1;
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto &entry : dist) {
        const int node_id = entry.first;
        if (node_id == current_node) {
            continue;
        }
        const auto node_it = nodes_.find(node_id);
        if (node_it == nodes_.end()) {
            continue;
        }
        const TopoNode &node = node_it->second;
        if (!node.position.allFinite() || node.blacklist_until > stamp) {
            continue;
        }
        if (accept && !accept(node.position)) {
            continue;
        }
        const double robot_distance = (node.position - robot_pos).norm();
        if (robot_distance < min_distance || robot_distance > max_distance) {
            continue;
        }
        const double recency = std::max(0.0, stamp - node.last_seen_time);
        const double score = entry.second +
                             0.2 * recency +
                             0.5 * static_cast<double>(node.failure_count);
        if (score < best_score) {
            best_score = score;
            best_node = node_id;
        }
    }

    if (best_node < 0) {
        path.reason = "no_connected_recovery_node";
        return false;
    }

    std::vector<int> reversed_nodes;
    std::vector<int> reversed_edges;
    for (int node = best_node; node >= 0;) {
        reversed_nodes.push_back(node);
        if (node == current_node) {
            break;
        }
        const auto prev_it = prev.find(node);
        if (prev_it == prev.end()) {
            path.reason = "path_reconstruction_failed";
            return false;
        }
        const auto edge_it = prev_edge.find(node);
        if (edge_it != prev_edge.end()) {
            reversed_edges.push_back(edge_it->second);
        }
        node = prev_it->second;
    }
    std::reverse(reversed_nodes.begin(), reversed_nodes.end());
    std::reverse(reversed_edges.begin(), reversed_edges.end());

    path.valid = true;
    path.node_ids = reversed_nodes;
    path.edge_ids = reversed_edges;
    path.positions.push_back(robot_pos);
    for (const int node_id : reversed_nodes) {
        const auto node_it = nodes_.find(node_id);
        if (node_it != nodes_.end()) {
            path.positions.push_back(node_it->second.position);
        }
    }
    path.length = dist[best_node];
    path.reason = "topology_recovery_path";
    return !path.positions.empty();
}

bool TopologicalMemory::searchPath(const general_utils::Vec3f &start,
                                   const general_utils::Vec3f &goal,
                                   const double stamp,
                                   TopoPath &path) const
{
    path = TopoPath{};
    if (!config_.enable || !start.allFinite() || !goal.allFinite()) {
        path.reason = "disabled_or_invalid_input";
        return false;
    }

    const double lookup_radius = std::max(1.0e-3, config_.node_merge_radius * 2.0);
    const int start_node = nearestNode(start, lookup_radius, stamp, false);
    const int goal_node = nearestNode(goal,
                                      std::max(lookup_radius, config_.recovery_max_distance),
                                      stamp,
                                      false);
    if (start_node < 0 || goal_node < 0 || start_node == goal_node) {
        path.reason = "missing_distinct_topology_endpoints";
        return false;
    }

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;
    std::unordered_map<int, int> prev_edge;
    using QueueEntry = std::pair<double, int>;
    std::priority_queue<QueueEntry,
                        std::vector<QueueEntry>,
                        std::greater<QueueEntry>>
            queue;

    dist[start_node] = 0.0;
    queue.push({0.0, start_node});

    while (!queue.empty()) {
        const auto [distance_so_far, node_id] = queue.top();
        queue.pop();
        const auto dist_it = dist.find(node_id);
        if (dist_it == dist.end() || distance_so_far > dist_it->second + 1.0e-6) {
            continue;
        }
        if (node_id == goal_node) {
            break;
        }
        const auto node_it = nodes_.find(node_id);
        if (node_it == nodes_.end() || node_it->second.blacklist_until > stamp) {
            continue;
        }

        for (const auto &entry : edges_) {
            const TopoEdge &edge = entry.second;
            if (!edgeUsable(edge, stamp)) {
                continue;
            }
            int next_node = -1;
            if (edge.from_node == node_id) {
                next_node = edge.to_node;
            } else if (edge.to_node == node_id) {
                next_node = edge.from_node;
            } else {
                continue;
            }
            const auto next_it = nodes_.find(next_node);
            if (next_it == nodes_.end() || next_it->second.blacklist_until > stamp) {
                continue;
            }
            const double edge_length =
                    edge.length > 1.0e-3
                            ? edge.length
                            : (node_it->second.position - next_it->second.position).norm();
            const double next_distance = distance_so_far + edge_length;
            const auto old = dist.find(next_node);
            if (old != dist.end() && old->second <= next_distance) {
                continue;
            }
            dist[next_node] = next_distance;
            prev[next_node] = node_id;
            prev_edge[next_node] = edge.edge_id;
            queue.push({next_distance, next_node});
        }
    }

    if (dist.find(goal_node) == dist.end()) {
        path.reason = "topology_goal_unreachable";
        return false;
    }

    std::vector<int> reversed_nodes;
    std::vector<int> reversed_edges;
    for (int node = goal_node; node >= 0;) {
        reversed_nodes.push_back(node);
        if (node == start_node) {
            break;
        }
        const auto prev_it = prev.find(node);
        if (prev_it == prev.end()) {
            path.reason = "path_reconstruction_failed";
            return false;
        }
        const auto edge_it = prev_edge.find(node);
        if (edge_it != prev_edge.end()) {
            reversed_edges.push_back(edge_it->second);
        }
        node = prev_it->second;
    }
    std::reverse(reversed_nodes.begin(), reversed_nodes.end());
    std::reverse(reversed_edges.begin(), reversed_edges.end());

    path.valid = true;
    path.node_ids = reversed_nodes;
    path.edge_ids = reversed_edges;
    path.positions.push_back(start);
    for (const int node_id : reversed_nodes) {
        const auto node_it = nodes_.find(node_id);
        if (node_it != nodes_.end()) {
            path.positions.push_back(node_it->second.position);
        }
    }
    path.positions.push_back(goal);
    path.length = dist[goal_node] + (nodes_.at(goal_node).position - goal).norm();
    path.reason = "topology_path";
    return path.positions.size() >= 2;
}

bool TopologicalMemory::selectLocalSubgoalFromPath(const TopoPath &path,
                                                   const general_utils::Vec3f &robot_pos,
                                                   const double local_radius,
                                                   general_utils::Vec3f &subgoal) const
{
    subgoal = general_utils::Vec3f::Zero();
    if (!path.valid || path.positions.empty() || !robot_pos.allFinite()) {
        return false;
    }
    const double radius = std::max(0.1, local_radius);
    general_utils::Vec3f best = path.positions.front();
    bool found = false;
    for (const general_utils::Vec3f &position : path.positions) {
        if (!position.allFinite()) {
            continue;
        }
        const double distance = (position - robot_pos).norm();
        if (distance <= radius) {
            best = position;
            found = true;
            continue;
        }
        if (found) {
            subgoal = position;
            return true;
        }
        subgoal = position;
        return true;
    }
    if (found) {
        subgoal = best;
        return true;
    }
    return false;
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
