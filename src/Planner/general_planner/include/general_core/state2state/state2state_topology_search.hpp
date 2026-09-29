#pragma once

#include <general_core/state2state/state2state_topology_route.hpp>
#include <queue>
#include <unordered_map>
#include <utility>

namespace general_planner::state2state_task {

// One graph search, without a forward-progress or local-distance filter: a
// useful known route can initially go backwards or leave the rolling window.
// The goal itself may be UNKNOWN. Only the attachment to the graph is checked
// here; the caller revalidates each executed prefix against its current map.
template <typename Snapshot, typename KnownFreeLine>
bool findClosestReachableTopologyRoute(const Snapshot &snapshot,
                                      const general_utils::Vec3f &start,
                                      const general_utils::Vec3f &goal,
                                      KnownFreeLine known_free_line,
                                      general_utils::vec_Vec3f &path) {
    path.clear();
    if (!start.allFinite() || !goal.allFinite() || snapshot.graph.empty()) return false;
    using Id = std::uint64_t;
    using Item = std::pair<double, Id>;
    std::vector<Item> attachments;
    for (const auto &entry : snapshot.graph) {
        const double distance = (entry.second.node.position - start).norm();
        if (distance <= snapshot.config.connection_radius)
            attachments.emplace_back(distance, entry.first);
    }
    std::sort(attachments.begin(), attachments.end());
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    std::unordered_map<Id, double> distances;
    std::unordered_map<Id, Id> parents;
    const std::size_t max_links = std::max<std::size_t>(1, snapshot.config.max_neighbors);
    const std::size_t max_checks = std::max<std::size_t>(8, 4 * max_links);
    for (std::size_t i = 0; i < attachments.size() && i < max_checks &&
                            distances.size() < max_links; ++i) {
        const auto &item = attachments[i];
        if (!known_free_line(start, snapshot.graph.at(item.second).node.position)) continue;
        distances[item.second] = item.first;
        parents[item.second] = 0;
        queue.push(item);
    }
    Id closest = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    double closest_cost = std::numeric_limits<double>::infinity();
    while (!queue.empty()) {
        const auto current = queue.top();
        queue.pop();
        if (current.first > distances.at(current.second) + 1.0e-9) continue;
        const auto &node = snapshot.graph.at(current.second);
        const double remaining = (node.node.position - goal).norm();
        if (remaining < closest_distance - 1.0e-9 ||
            (std::abs(remaining - closest_distance) <= 1.0e-9 &&
             (current.first < closest_cost - 1.0e-9 ||
              (std::abs(current.first - closest_cost) <= 1.0e-9 && current.second < closest)))) {
            closest = current.second;
            closest_distance = remaining;
            closest_cost = current.first;
        }
        for (const auto &edge : node.neighbors) {
            if (!snapshot.graph.count(edge.first) || !std::isfinite(edge.second) || edge.second <= 0.0) continue;
            const double cost = current.first + edge.second;
            const auto found = distances.find(edge.first);
            if (found != distances.end() && cost >= found->second - 1.0e-9) continue;
            distances[edge.first] = cost;
            parents[edge.first] = current.second;
            queue.emplace(cost, edge.first);
        }
    }
    if (closest == 0) return false;
    std::vector<Id> reverse;
    for (Id id = closest; id != 0; id = parents.at(id)) reverse.push_back(id);
    path.push_back(start);
    for (auto it = reverse.rbegin(); it != reverse.rend(); ++it) {
        const auto &position = snapshot.graph.at(*it).node.position;
        if ((position - path.back()).norm() > 1.0e-6) path.push_back(position);
    }
    return true;
}

} // namespace general_planner::state2state_task
