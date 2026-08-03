#include <map_manager/incremental_topology_graph.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
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
    using rog_map::Vec3i;

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

    // A narrow inflated flight-height band is valid for quasi-2D
    // state2state planning, but a 3D minimum-clearance ray test necessarily
    // sees the floor/ceiling first. Planar topology must keep those bounds as
    // traversability gates while measuring bubble clearance in XY only.
    IncrementalTopologyGraph::Config narrow_config = config;
    narrow_config.min_clearance = 0.45;
    narrow_config.max_clearance = 0.8;
    narrow_config.planar_mode = false;
    IncrementalTopologyGraph::Query narrow_query;
    narrow_query.traversable = [](const Vec3f &point) {
        return point.x() >= 0.0 && point.x() < 4.0 &&
               point.y() >= 0.0 && point.y() < 2.0 &&
               point.z() > 0.9 && point.z() < 1.3;
    };
    IncrementalTopologyGraph spatial_narrow_graph(narrow_config);
    spatial_narrow_graph.observePlannedPath({Vec3f(0.5, 0.5, 1.1),
                                             Vec3f(3.5, 0.5, 1.1)});
    spatial_narrow_graph.update(narrow_query, 2);
    ok &= expect(spatial_narrow_graph.snapshot().nodes.empty(),
                 "3D clearance should expose the narrow-height regression fixture");

    narrow_config.planar_mode = true;
    narrow_config.navigation_altitude = 1.1;
    IncrementalTopologyGraph planar_narrow_graph(narrow_config);
    planar_narrow_graph.observePlannedPath({Vec3f(0.5, 0.5, 1.1),
                                            Vec3f(3.5, 0.5, 1.1)});
    ok &= expect(planar_narrow_graph.update(narrow_query, 2) == 2,
                 "planar topology must rebuild both XY regions");
    const auto planar_narrow = planar_narrow_graph.snapshot();
    ok &= expect(planar_narrow.nodes.size() == 2,
                 "planar topology must retain nodes in a narrow height band");
    ok &= expect(planar_narrow.edges.size() == 1,
                 "planar topology must connect adjacent free XY regions");
    for (const auto &node : planar_narrow.nodes) {
        ok &= expect(std::abs(node.position.z() - 1.1) < 1.0e-9,
                     "planar topology node altitude is inconsistent");
    }
    ok &= expect(planar_narrow.last_sampled_center_count > 0 &&
                     planar_narrow.last_traversable_center_count > 0,
                 "candidate diagnostics must expose successful planar sampling");

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

    // The state2state graph is a mission-scoped MapManager resource. Turning
    // it off for exploration must suppress dirty tracking, maintenance and
    // planner queries without destroying exploration's independent TopoGraph.
    const auto dirty_before_deactivation = crossing_graph.stats().dirty_region_count;
    crossing_graph.setActive(false);
    crossing_graph.markDirty(Vec3f(1.0, 1.0, 1.0));
    ok &= expect(!crossing_graph.active(),
                 "the runtime topology ownership gate must deactivate");
    ok &= expect(crossing_graph.stats().dirty_region_count ==
                     dirty_before_deactivation,
                 "inactive topology must not consume exploration map changes");
    path.clear();
    ok &= expect(!crossing_graph.findPath(Vec3f(0.5, 0.5, 0.5),
                                          Vec3f(1.5, 0.5, 0.5),
                                          crossing_query, path),
                 "inactive topology must not participate in planning");
    ok &= expect(crossing_graph.update(crossing_query, 1) == 0,
                 "inactive topology worker must not rebuild regions");
    crossing_graph.setActive(true);
    ok &= expect(crossing_graph.active(),
                 "configured topology must reactivate for state2state");

    // Dense known-free mode represents every safe point on one globally
    // aligned lattice and keeps earlier observed regions when the robot/map
    // focus moves forward. A planned path alone must not create evidence.
    IncrementalTopologyGraph::Config dense_config = config;
    dense_config.dense_known_free = true;
    dense_config.unknown_as_free = true; // sanitized to false by dense mode.
    dense_config.planar_mode = true;
    dense_config.navigation_altitude = 1.0;
    dense_config.region_size = 2.0;
    dense_config.sample_spacing = 1.0;
    dense_config.connection_radius = 1.5;
    dense_config.max_nodes_per_region = 8;
    dense_config.max_neighbors = 8;
    dense_config.snapshot_every_update = true;
    IncrementalTopologyGraph dense_graph(dense_config);
    ok &= expect(!dense_graph.config().unknown_as_free,
                 "dense persistent topology must force unknown_as_free off");
    IncrementalTopologyGraph::Query dense_query;
    dense_query.traversable = [](const Vec3f &) {
        return false;
    };
    dense_query.clearance = [](const Vec3f &, double &distance) {
        distance = 0.8;
        return true;
    };
    dense_graph.observePlannedPath(
        {Vec3f(0.5, 0.5, 1.0), Vec3f(5.5, 0.5, 1.0)});
    ok &= expect(dense_graph.stats().dirty_region_count == 0,
                 "planned paths must not seed dense known-free nodes");
    std::vector<IncrementalTopologyGraph::VoxelEvidenceDelta>
        first_dense_observation;
    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
            first_dense_observation.push_back(
                {Vec3i(x, y, 0), 1, 0});
        }
    }
    dense_graph.integrateDenseEvidence(
        first_dense_observation, 1.0);
    ok &= expect(dense_graph.update(dense_query, 1) == 1,
                 "the first observed dense region must rebuild");
    const auto dense_initial = dense_graph.snapshot();
    ok &= expect(dense_initial.nodes.size() == 4,
                 "a 2x2 m planar region at 1 m spacing must retain four nodes");
    std::unordered_map<std::uint64_t, Vec3f> dense_history;
    for (const auto &node : dense_initial.nodes) {
        dense_history.emplace(node.id, node.position);
    }

    std::vector<IncrementalTopologyGraph::VoxelEvidenceDelta>
        moved_dense_observation;
    for (int x = 4; x < 6; ++x) {
        for (int y = 0; y < 2; ++y) {
            moved_dense_observation.push_back(
                {Vec3i(x, y, 0), 1, 0});
        }
    }
    dense_graph.integrateDenseEvidence(
        moved_dense_observation, 1.0);
    dense_graph.update(dense_query, 1);
    const auto dense_moved = dense_graph.snapshot();
    ok &= expect(dense_moved.nodes.size() == 8,
                 "moving observation must append a dense region without erasing history");
    for (const auto &historic : dense_history) {
        bool found = false;
        for (const auto &node : dense_moved.nodes) {
            found = found || (node.id == historic.first &&
                              (node.position - historic.second).norm() < 1.0e-9);
        }
        ok &= expect(found,
                     "previously observed dense nodes must persist after movement");
    }
    dense_graph.integrateDenseEvidence(
        {{Vec3i(0, 0, 0), -1, 1}}, 1.0);
    dense_graph.update(dense_query, 1);
    ok &= expect(dense_graph.snapshot().nodes.size() == 7,
                 "new occupied evidence must invalidate its coarse free cell");

    // A graph rebuild may be expensive. A planner query must use the already
    // available graph instead of waiting for the maintenance update mutex.
    std::atomic<bool> slow_update_entered{false};
    std::mutex slow_mutex;
    std::condition_variable slow_cv;
    bool release_slow_update = false;
    IncrementalTopologyGraph::Query slow_query;
    slow_query.traversable = [&](const Vec3f &point) {
        if (!slow_update_entered.exchange(true)) {
            slow_cv.notify_all();
        }
        std::unique_lock<std::mutex> lock(slow_mutex);
        slow_cv.wait(lock, [&]() { return release_slow_update; });
        return crossing_query.traversable(point);
    };
    slow_query.clearance = crossing_query.clearance;
    crossing_graph.markDirty(Vec3f(1.0, 1.0, 1.0));
    std::thread slow_worker([&]() { crossing_graph.update(slow_query, 1); });
    {
        std::unique_lock<std::mutex> lock(slow_mutex);
        slow_cv.wait(lock, [&]() { return slow_update_entered.load(); });
    }
    std::thread delayed_release([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        {
            std::lock_guard<std::mutex> lock(slow_mutex);
            release_slow_update = true;
        }
        slow_cv.notify_all();
    });
    const auto query_start = std::chrono::steady_clock::now();
    path.clear();
    const bool concurrent_path = crossing_graph.findPath(
        Vec3f(0.5, 0.5, 0.5), Vec3f(1.5, 0.5, 0.5),
        crossing_query, path);
    const double concurrent_query_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - query_start).count();
    ok &= expect(concurrent_path && concurrent_query_ms < 100.0,
                 "planner query blocked on asynchronous topology maintenance");
    delayed_release.join();
    slow_worker.join();

    if (!ok) {
        return EXIT_FAILURE;
    }
    std::cout << "incremental_topology_self_test: PASS" << std::endl;
    return EXIT_SUCCESS;
}
