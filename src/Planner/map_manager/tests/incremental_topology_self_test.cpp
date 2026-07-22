#include <map_manager/incremental_topology_graph.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>

namespace {

bool expect(const bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "[incremental_topology_self_test] " << message << std::endl;
        return false;
    }
    return true;
}

} // namespace

int main() {
    using general_planner::IncrementalTopologyGraph;
    using rog_map::Vec3f;

    IncrementalTopologyGraph::Config config;
    config.enabled = true;
    config.region_size = 2.0;
    config.sample_spacing = 1.0;
    config.min_clearance = 0.1;
    config.max_clearance = 0.8;
    config.candidate_separation = 1.0;
    config.stable_match_distance = 0.25;
    config.connection_radius = 2.5;
    config.edge_sample_spacing = 0.1;
    config.dirty_padding = 0.0;
    config.max_nodes_per_region = 2;
    config.max_bubbles_per_region = 128;
    config.max_neighbors = 6;
    config.max_regions_per_update = 8;

    IncrementalTopologyGraph graph(config);
    bool middle_blocked = false;
    IncrementalTopologyGraph::Query query;
    query.traversable = [&](const Vec3f &point) {
        const bool inside = point.x() >= 0.0 && point.x() < 6.0 &&
                            point.y() >= 0.0 && point.y() < 2.0 &&
                            point.z() >= 0.0 && point.z() < 2.0;
        return inside && !(middle_blocked && point.x() >= 2.0 && point.x() < 4.0);
    };
    query.clearance = [&](const Vec3f &point, double &distance) {
        if (!query.traversable(point)) {
            return false;
        }
        distance = 0.8;
        return true;
    };

    bool ok = true;
    graph.observePlannedPath({Vec3f(0.5, 0.5, 0.5),
                              Vec3f(5.5, 0.5, 0.5)});
    ok &= expect(graph.update(query, 16) == 3,
                 "the initial update must rebuild exactly three regions");
    const auto initial = graph.snapshot();
    ok &= expect(initial.nodes.size() == 3,
                 "each connected free-space bubble union must retain one representative");
    ok &= expect(!initial.edges.empty(), "neighboring regions must be connected");

    std::unordered_map<std::uint64_t, Vec3f> stable_nodes;
    for (const auto &node : initial.nodes) {
        if (node.position.x() < 2.0 || node.position.x() >= 4.0) {
            stable_nodes.emplace(node.id, node.position);
        }
    }

    middle_blocked = true;
    graph.markDirty(Vec3f(3.0, 1.0, 1.0));
    ok &= expect(graph.update(query, 1) == 1,
                 "one dirty-region budget must rebuild one region");
    const auto blocked = graph.snapshot();
    for (const auto &node : blocked.nodes) {
        ok &= expect(node.position.x() < 2.0 || node.position.x() >= 4.0,
                     "blocked-region nodes must be removed");
    }
    for (const auto &stable : stable_nodes) {
        bool found = false;
        for (const auto &node : blocked.nodes) {
            found = found || (node.id == stable.first &&
                              (node.position - stable.second).norm() < 1.0e-9);
        }
        ok &= expect(found, "unaffected regions must preserve stable node IDs");
    }

    rog_map::vec_Vec3f path;
    ok &= expect(!graph.findPath(Vec3f(0.5, 0.5, 0.5),
                                 Vec3f(5.5, 0.5, 0.5), query, path),
                 "a disconnected graph must not fabricate a direct path");
    ok &= expect(path.empty(), "failed path output must be empty");

    const auto stats = graph.stats();
    ok &= expect(stats.rebuilt_region_count == 4 && stats.revision == 4,
                 "incremental revision statistics are inconsistent");

    // A dirty region may contain no node while an edge passes through it. The
    // edge still has to be invalidated when an obstacle appears in the middle.
    IncrementalTopologyGraph::Config crossing_config = config;
    crossing_config.connection_radius = 6.0;
    crossing_config.max_nodes_per_region = 1;
    IncrementalTopologyGraph crossing_graph(crossing_config);
    bool crossing_blocked = false;
    IncrementalTopologyGraph::Query crossing_query;
    crossing_query.traversable = [&](const Vec3f &point) {
        const bool inside = point.x() >= 0.0 && point.x() < 6.0 &&
                            point.y() >= 0.0 && point.y() < 2.0 &&
                            point.z() >= 0.0 && point.z() < 2.0;
        return inside && !(crossing_blocked && point.x() >= 2.0 && point.x() < 4.0);
    };
    crossing_query.clearance = [&](const Vec3f &point, double &distance) {
        if (!crossing_query.traversable(point)) {
            return false;
        }
        distance = 0.8;
        return true;
    };
    crossing_graph.markDirty(Vec3f(1.0, 1.0, 1.0));
    crossing_graph.markDirty(Vec3f(5.0, 1.0, 1.0));
    crossing_graph.update(crossing_query, 2);
    ok &= expect(crossing_graph.snapshot().edges.size() == 1,
                 "the endpoint regions must initially have one crossing edge");
    crossing_blocked = true;
    crossing_graph.markDirty(Vec3f(3.0, 1.0, 1.0));
    crossing_graph.update(crossing_query, 1);
    ok &= expect(crossing_graph.snapshot().edges.empty(),
                 "an edge crossing a node-free dirty region must be removed");

    if (!ok) {
        return EXIT_FAILURE;
    }
    std::cout << "incremental_topology_self_test: PASS" << std::endl;
    return EXIT_SUCCESS;
}
