#include "exploration/topo_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace general_planner {
namespace exploration {

TopoGraph::TopoGraph(Config cfg,
                     MapManager::Ptr map_manager,
                     path_search::Astar::Ptr astar,
                     std::shared_ptr<ObservationMap> observation_map)
        : cfg_(std::move(cfg)),
          map_manager_(std::move(map_manager)),
          astar_(std::move(astar)),
          observation_map_(std::move(observation_map)) {}

void TopoGraph::updateOdomNode(const super_utils::Vec3f &pos, const double yaw) {
    if (odom_node_id_ < 0) {
        odom_node_id_ = next_node_id_++;
        ExplorationTopoNode node;
        node.id = odom_node_id_;
        node.type = TopoNodeType::ODOM;
        nodes_[node.id] = node;
    }
    auto &node = nodes_[odom_node_id_];
    node.position = pos;
    node.yaw = yaw;
    node.active = true;
    rebuildEdges();
}

void TopoGraph::updateHistoryOdomNodes(const super_utils::Vec3f &pos, const double yaw) {
    if (!history_node_ids_.empty()) {
        const auto it = nodes_.find(history_node_ids_.back());
        if (it != nodes_.end() &&
            (it->second.position - pos).norm() < cfg_.history_node_min_distance) {
            return;
        }
    }
    ExplorationTopoNode node;
    node.id = next_node_id_++;
    node.type = TopoNodeType::HISTORY_ODOM;
    node.position = pos;
    node.yaw = yaw;
    nodes_[node.id] = node;
    history_node_ids_.push_back(node.id);
    while (static_cast<int>(history_node_ids_.size()) > cfg_.max_history_nodes) {
        const int old_id = history_node_ids_.front();
        history_node_ids_.erase(history_node_ids_.begin());
        nodes_.erase(old_id);
        adjacency_.erase(old_id);
    }
    rebuildEdges();
}

void TopoGraph::insertOrUpdateFrontierNodes(const std::vector<FrontierRecord> &frontiers) {
    std::unordered_set<int> active_ids;
    for (const auto &frontier : frontiers) {
        if (!frontier.has_reachable_viewpoint && !frontier.best_viewpoint.global_safe) {
            continue;
        }
        active_ids.insert(frontier.stable_id);
        int node_id = -1;
        const auto it = frontier_id_to_node_id_.find(frontier.stable_id);
        if (it == frontier_id_to_node_id_.end()) {
            node_id = next_node_id_++;
            frontier_id_to_node_id_[frontier.stable_id] = node_id;
        } else {
            node_id = it->second;
        }
        ExplorationTopoNode &node = nodes_[node_id];
        node.id = node_id;
        node.type = TopoNodeType::FRONTIER_VIEWPOINT;
        node.frontier_id = frontier.stable_id;
        node.region_id = frontier.region_id;
        node.position = frontier.best_viewpoint.position;
        node.yaw = frontier.best_viewpoint.yaw;
        node.active = true;
    }
    removeInactiveFrontierNodes(active_ids);
    rebuildEdges();
}

bool TopoGraph::graphSearchToFrontier(const int frontier_id, GlobalRoute &route) const {
    route = GlobalRoute{};
    if (odom_node_id_ < 0) {
        return false;
    }
    const auto it = frontier_id_to_node_id_.find(frontier_id);
    if (it == frontier_id_to_node_id_.end()) {
        return false;
    }
    double cost = 0.0;
    const std::vector<int> node_path = dijkstra(odom_node_id_, it->second, cost);
    if (node_path.empty()) {
        return false;
    }
    route.valid = true;
    route.target_frontier_id = frontier_id;
    route.node_ids = node_path;
    route.cost = cost;
    for (std::size_t i = 0; i + 1 < node_path.size(); ++i) {
        const int from = node_path[i];
        const int to = node_path[i + 1];
        const auto ait = adjacency_.find(from);
        if (ait == adjacency_.end()) {
            continue;
        }
        for (const auto &edge : ait->second) {
            if (edge.to != to) {
                continue;
            }
            if (route.path.empty()) {
                route.path.insert(route.path.end(), edge.path.begin(), edge.path.end());
            } else if (!edge.path.empty()) {
                route.path.insert(route.path.end(), edge.path.begin() + 1, edge.path.end());
            }
            break;
        }
    }
    if (route.path.empty()) {
        for (const int id : node_path) {
            route.path.push_back(nodes_.at(id).position);
        }
    }
    return true;
}

bool TopoGraph::graphSearchBetweenFrontiers(const int from_frontier_id,
                                            const int to_frontier_id,
                                            double &cost) const {
    cost = std::numeric_limits<double>::infinity();
    const auto fit = frontier_id_to_node_id_.find(from_frontier_id);
    const auto tit = frontier_id_to_node_id_.find(to_frontier_id);
    if (fit == frontier_id_to_node_id_.end() || tit == frontier_id_to_node_id_.end()) {
        return false;
    }
    const std::vector<int> path = dijkstra(fit->second, tit->second, cost);
    return !path.empty();
}

bool TopoGraph::getNode(const int node_id, ExplorationTopoNode &node) const {
    const auto it = nodes_.find(node_id);
    if (it == nodes_.end()) {
        return false;
    }
    node = it->second;
    return true;
}

void TopoGraph::markEdgeUnreachable(const int from, const int to) {
    unreachable_edges_.insert(canonicalEdge(from, to));
}

void TopoGraph::removeInactiveFrontierNodes(const std::unordered_set<int> &active_frontier_ids) {
    for (auto it = frontier_id_to_node_id_.begin(); it != frontier_id_to_node_id_.end();) {
        if (active_frontier_ids.find(it->first) != active_frontier_ids.end()) {
            ++it;
            continue;
        }
        nodes_.erase(it->second);
        adjacency_.erase(it->second);
        it = frontier_id_to_node_id_.erase(it);
    }
}

void TopoGraph::reset() {
    next_node_id_ = 0;
    odom_node_id_ = -1;
    nodes_.clear();
    adjacency_.clear();
    frontier_id_to_node_id_.clear();
    history_node_ids_.clear();
    unreachable_edges_.clear();
}

int TopoGraph::edgeCount() const {
    int count = 0;
    for (const auto &kv : adjacency_) {
        count += static_cast<int>(kv.second.size());
    }
    return count / 2;
}

bool TopoGraph::tryBuildEdge(const int from_id,
                             const int to_id,
                             ExplorationTopoEdge &edge) const {
    const auto fit = nodes_.find(from_id);
    const auto tit = nodes_.find(to_id);
    if (fit == nodes_.end() || tit == nodes_.end()) {
        return false;
    }
    const super_utils::Vec3f start = fit->second.position;
    const super_utils::Vec3f goal = tit->second.position;
    super_utils::vec_E<super_utils::Vec3f> path;
    double cost = 0.0;
    bool ok = false;
    if (cfg_.use_local_astar_for_edges &&
        map_manager_ != nullptr &&
        map_manager_->insideLocalMap(start) &&
        map_manager_->insideLocalMap(goal)) {
        ok = localAstarPath(start, goal, path, cost);
    }
    if (!ok && cfg_.use_global_line_free_for_edges) {
        ok = globalLineOrObservedFreePath(start, goal, path, cost);
    }
    if (!ok) {
        return false;
    }
    edge.from = from_id;
    edge.to = to_id;
    edge.cost = cost;
    edge.reachable = true;
    edge.path = path;
    return true;
}

bool TopoGraph::localAstarPath(const super_utils::Vec3f &start,
                               const super_utils::Vec3f &goal,
                               super_utils::vec_E<super_utils::Vec3f> &path,
                               double &cost) const {
    if (astar_ == nullptr) {
        return false;
    }
    path.clear();
    const int flag = path_search::ON_INF_MAP | path_search::UNKNOWN_AS_OCCUPIED;
    const auto ret = astar_->pointToPointPathSearch(start,
                                                    goal,
                                                    flag,
                                                    cfg_.global_edge_max_length,
                                                    path,
                                                    cfg_.local_edge_astar_timeout);
    if (ret != super_utils::SUCCESS || path.empty()) {
        return false;
    }
    cost = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        cost += (path[i] - path[i - 1]).norm();
    }
    return true;
}

bool TopoGraph::globalLineOrObservedFreePath(const super_utils::Vec3f &start,
                                             const super_utils::Vec3f &goal,
                                             super_utils::vec_E<super_utils::Vec3f> &path,
                                             double &cost) const {
    const double length = (goal - start).norm();
    if (length > cfg_.global_edge_max_length) {
        return false;
    }
    if (observation_map_ != nullptr &&
        !observation_map_->lineOfSightFree(start,
                                           goal,
                                           cfg_.global_line_safe_distance,
                                           cfg_.global_line_step)) {
        return false;
    }
    path.clear();
    const int samples = std::max(1, static_cast<int>(
            std::ceil(length / std::max(0.25, cfg_.global_line_step))));
    for (int i = 0; i <= samples; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(samples);
        path.push_back(start + ratio * (goal - start));
    }
    cost = length;
    return true;
}

std::vector<int> TopoGraph::dijkstra(const int start_id,
                                     const int goal_id,
                                     double &cost) const {
    cost = std::numeric_limits<double>::infinity();
    if (nodes_.find(start_id) == nodes_.end() || nodes_.find(goal_id) == nodes_.end()) {
        return {};
    }
    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parent;
    for (const auto &kv : nodes_) {
        dist[kv.first] = std::numeric_limits<double>::infinity();
    }
    dist[start_id] = 0.0;
    queue.emplace(0.0, start_id);
    while (!queue.empty()) {
        const auto [current_cost, current] = queue.top();
        queue.pop();
        if (current_cost > dist[current]) {
            continue;
        }
        if (current == goal_id) {
            break;
        }
        const auto ait = adjacency_.find(current);
        if (ait == adjacency_.end()) {
            continue;
        }
        for (const auto &edge : ait->second) {
            const double next_cost = current_cost + edge.cost;
            if (next_cost < dist[edge.to]) {
                dist[edge.to] = next_cost;
                parent[edge.to] = current;
                queue.emplace(next_cost, edge.to);
            }
        }
    }
    if (!std::isfinite(dist[goal_id])) {
        return {};
    }
    cost = dist[goal_id];
    std::vector<int> path;
    for (int current = goal_id;; current = parent[current]) {
        path.push_back(current);
        if (current == start_id) {
            break;
        }
        if (parent.find(current) == parent.end()) {
            return {};
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void TopoGraph::rebuildEdges() {
    adjacency_.clear();
    std::vector<int> ids;
    ids.reserve(nodes_.size());
    for (const auto &kv : nodes_) {
        if (kv.second.active) {
            ids.push_back(kv.first);
        }
    }

    const double radius_sq = cfg_.connect_radius * cfg_.connect_radius;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const int a = ids[i];
            const int b = ids[j];
            if (unreachable_edges_.find(canonicalEdge(a, b)) != unreachable_edges_.end()) {
                continue;
            }
            if ((nodes_[a].position - nodes_[b].position).squaredNorm() > radius_sq) {
                continue;
            }
            ExplorationTopoEdge edge;
            if (!tryBuildEdge(a, b, edge)) {
                unreachable_edges_.insert(canonicalEdge(a, b));
                continue;
            }
            adjacency_[a].push_back(edge);
            ExplorationTopoEdge reverse = edge;
            reverse.from = b;
            reverse.to = a;
            std::reverse(reverse.path.begin(), reverse.path.end());
            adjacency_[b].push_back(reverse);
        }
    }
}

std::pair<int, int> TopoGraph::canonicalEdge(const int a, const int b) {
    return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

}  // namespace exploration
}  // namespace general_planner
