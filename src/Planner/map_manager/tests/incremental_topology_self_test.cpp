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
    ok &= expect(initial.nodes.size() >= 3 && initial.nodes.size() <= 6,
                 "each free region needs a sparse core plus bounded portal nodes");
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
    ok &= expect(planar_narrow.nodes.size() >= 2 &&
                     planar_narrow.nodes.size() <= 4,
                 "planar topology must retain a bounded skeleton in a narrow height band");
    ok &= expect(!planar_narrow.edges.empty(),
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

    // Dense known-free mode samples the complete map-view KNOWN_FREE volume.
    // Odom/planned paths do not provide free-space evidence.
    IncrementalTopologyGraph::Config dense_config = config;
    dense_config.construction_mode =
        IncrementalTopologyGraph::ConstructionMode::DENSE_KNOWN_FREE_DEBUG;
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
    int dense_phase = 0;
    bool first_cell_occupied = false;
    IncrementalTopologyGraph::Query dense_query;
    dense_query.traversable = [&](const Vec3f &point) {
        const int x = static_cast<int>(std::floor(point.x()));
        const int y = static_cast<int>(std::floor(point.y()));
        if (y < 0 || y >= 2) return false;
        if (x >= 0 && x < 2) {
            return !(first_cell_occupied && x == 0 && y == 0);
        }
        return dense_phase >= 1 && x >= 4 && x < 6;
    };
    dense_query.clearance = [](const Vec3f &, double &distance) {
        distance = 0.8;
        return true;
    };
    dense_graph.observePlannedPath(
        {Vec3f(0.5, 0.5, 1.0), Vec3f(5.5, 0.5, 1.0)});
    ok &= expect(dense_graph.stats().dirty_region_count == 0,
                 "planned paths must not seed dense known-free nodes");
    dense_graph.markDirty(Vec3f(0.5, 0.5, 1.0));
    ok &= expect(dense_graph.update(dense_query, 1) == 1,
                 "the current ROG free-space region must rebuild");
    const auto dense_initial = dense_graph.snapshot();
    ok &= expect(dense_initial.nodes.size() == 4,
                 "all four KNOWN_FREE samples in a 2x2 region must become nodes");
    ok &= expect(dense_initial.edges.size() >= 3,
                 "the complete KNOWN_FREE region must be connected");
    std::unordered_map<std::uint64_t, Vec3f> dense_history;
    for (const auto &node : dense_initial.nodes) {
        dense_history.emplace(node.id, node.position);
    }

    // Direct dense-cell indexing must preserve adjacency across region
    // boundaries; this is the common case as the odom-centred ROG window
    // advances into a newly observed region.
    IncrementalTopologyGraph dense_boundary_graph(dense_config);
    IncrementalTopologyGraph::Query dense_boundary_query;
    dense_boundary_query.traversable = [](const Vec3f &point) {
        const int x = static_cast<int>(std::floor(point.x()));
        const int y = static_cast<int>(std::floor(point.y()));
        return y == 0 && (x == 1 || x == 2);
    };
    dense_boundary_query.clearance = dense_query.clearance;
    dense_boundary_graph.markDirty(Vec3f(1.5, 0.5, 1.0));
    dense_boundary_graph.markDirty(Vec3f(2.5, 0.5, 1.0));
    while (dense_boundary_graph.stats().dirty_region_count > 0) {
        dense_boundary_graph.update(dense_boundary_query, 4);
    }
    const auto dense_boundary_snapshot = dense_boundary_graph.snapshot();
    ok &= expect(dense_boundary_snapshot.nodes.size() == 2 &&
                     dense_boundary_snapshot.edges.size() == 1,
                 "dense lattice nodes across adjacent regions must connect");

    dense_phase = 1;
    dense_graph.markDirty(Vec3f(4.5, 0.5, 1.0));
    while (dense_graph.stats().dirty_region_count > 0) {
        dense_graph.update(dense_query, 4);
    }
    const auto dense_moved = dense_graph.snapshot();
    ok &= expect(dense_moved.nodes.size() == 8,
                 "new ROG KNOWN_FREE regions must append without odom gating");
    for (const auto &historic : dense_history) {
        bool found = false;
        for (const auto &node : dense_moved.nodes) {
            found = found || (node.id == historic.first &&
                              (node.position - historic.second).norm() < 1.0e-9);
        }
        ok &= expect(found,
                     "previously observed dense nodes must persist after movement");
    }

    // A rolling window can lose current evidence without observing an
    // obstacle. UNKNOWN must retain committed nodes/edges, while a subsequent
    // confirmed OCCUPIED observation is allowed to remove them.
    IncrementalTopologyGraph persistent_graph(dense_config);
    int persistent_phase = 0; // 0=free, 1=unknown, 2=occupied
    IncrementalTopologyGraph::Query persistent_query;
    persistent_query.evidence = [&](const Vec3f &point) {
        const int x = static_cast<int>(std::floor(point.x()));
        const int y = static_cast<int>(std::floor(point.y()));
        if (x < 0 || x >= 2 || y < 0 || y >= 2) {
            return general_planner::TopologyMapView::EvidenceState::OCCUPIED;
        }
        if (persistent_phase == 0) {
            return general_planner::TopologyMapView::EvidenceState::KNOWN_FREE;
        }
        if (persistent_phase == 1) {
            return general_planner::TopologyMapView::EvidenceState::UNKNOWN;
        }
        return general_planner::TopologyMapView::EvidenceState::OCCUPIED;
    };
    persistent_query.clearance = dense_query.clearance;
    persistent_graph.markDirty(Vec3f(0.5, 0.5, 1.0));
    persistent_graph.update(persistent_query, 1);
    const auto committed_snapshot = persistent_graph.snapshot();
    ok &= expect(committed_snapshot.nodes.size() == 4,
                 "known-free evidence must commit the dense fixture");
    persistent_phase = 1;
    persistent_graph.markDirty(Vec3f(0.5, 0.5, 1.0));
    persistent_graph.update(persistent_query, 1);
    const auto unknown_snapshot = persistent_graph.snapshot();
    ok &= expect(unknown_snapshot.nodes.size() == committed_snapshot.nodes.size() &&
                     unknown_snapshot.edges.size() == committed_snapshot.edges.size(),
                 "UNKNOWN evidence must not erase committed topology");
    bool all_historical = !unknown_snapshot.nodes.empty();
    for (const auto &node : unknown_snapshot.nodes) {
        all_historical = all_historical &&
            node.state == IncrementalTopologyGraph::NodeState::HISTORICAL;
    }
    ok &= expect(all_historical,
                 "nodes retained through UNKNOWN must be marked historical");
    persistent_phase = 2;
    persistent_graph.markDirty(Vec3f(0.5, 0.5, 1.0));
    persistent_graph.update(persistent_query, 1);
    ok &= expect(persistent_graph.snapshot().nodes.empty(),
                 "confirmed OCCUPIED evidence must invalidate historic nodes");

    // Sparse ray-carved ROG evidence may not coincide with an octree center.
    // Exact state-transition voxels must seed the bubble builder without ever
    // promoting neighboring UNKNOWN cells.
    IncrementalTopologyGraph::Config seed_config = config;
    seed_config.min_clearance = 0.1;
    seed_config.max_nodes_per_region = 4;
    IncrementalTopologyGraph seed_graph(seed_config);
    const rog_map::Vec3i seed_id(0, 0, 5);
    const Vec3f seed_position =
        (seed_id.cast<double>() + Vec3f::Constant(0.5)) * 0.2;
    IncrementalTopologyGraph::Query seed_query;
    seed_query.evidence = [&](const Vec3f &point) {
        return (point - seed_position).norm() < 1.0e-9
            ? general_planner::TopologyMapView::EvidenceState::KNOWN_FREE
            : general_planner::TopologyMapView::EvidenceState::UNKNOWN;
    };
    seed_query.clearance = [&](const Vec3f &point, double &distance) {
        if ((point - seed_position).norm() >= 1.0e-9) {
            return false;
        }
        distance = 0.4;
        return true;
    };
    seed_graph.markDirtyVoxels({seed_id}, 0.2);
    seed_graph.update(seed_query, 1);
    const auto seed_snapshot = seed_graph.snapshot();
    ok &= expect(seed_snapshot.nodes.size() == 1 &&
                     (seed_snapshot.nodes.front().position - seed_position).norm() <
                         1.0e-9,
                 "ROG state-transition voxel must seed an evidence-aligned bubble");

    // Persistent bubble regions are append-only: a later observation in the
    // same region may fill an uncovered gap, but it must not move/replace a
    // previously committed node or insert another node inside global spacing.
    IncrementalTopologyGraph::Config append_config = seed_config;
    append_config.region_size = 4.0;
    append_config.candidate_separation = 1.0;
    append_config.max_nodes_per_region = 4;
    IncrementalTopologyGraph append_graph(append_config);
    const rog_map::Vec3i append_id(10, 0, 5);
    const Vec3f append_position =
        (append_id.cast<double>() + Vec3f::Constant(0.5)) * 0.2;
    const rog_map::Vec3i close_id(1, 0, 5);
    const Vec3f close_position =
        (close_id.cast<double>() + Vec3f::Constant(0.5)) * 0.2;
    int append_phase = 0;
    IncrementalTopologyGraph::Query append_query;
    append_query.evidence = [&](const Vec3f &point) {
        const bool first = (point - seed_position).norm() < 1.0e-9;
        const bool second = append_phase >= 1 &&
                            (point - append_position).norm() < 1.0e-9;
        const bool close = append_phase >= 2 &&
                           (point - close_position).norm() < 1.0e-9;
        return first || second || close
            ? general_planner::TopologyMapView::EvidenceState::KNOWN_FREE
            : general_planner::TopologyMapView::EvidenceState::UNKNOWN;
    };
    append_query.clearance = [&](const Vec3f &point, double &distance) {
        if (append_query.evidenceState(point) !=
            general_planner::TopologyMapView::EvidenceState::KNOWN_FREE) {
            return false;
        }
        distance = 0.4;
        return true;
    };
    append_graph.markDirtyVoxels({seed_id}, 0.2);
    append_graph.update(append_query, 1);
    const auto append_initial = append_graph.snapshot();
    ok &= expect(append_initial.nodes.size() == 1,
                 "the append-only fixture must commit its first node");
    const auto committed_id = append_initial.nodes.front().id;
    append_phase = 1;
    append_graph.markDirtyVoxels({append_id}, 0.2);
    append_graph.update(append_query, 1);
    const auto append_filled = append_graph.snapshot();
    bool original_unchanged = false;
    for (const auto &node : append_filled.nodes) {
        original_unchanged = original_unchanged ||
            (node.id == committed_id &&
             (node.position - seed_position).norm() < 1.0e-9);
    }
    ok &= expect(append_filled.nodes.size() == 2 && original_unchanged,
                 "new free evidence must append without replacing committed topology");
    append_phase = 2;
    append_graph.markDirtyVoxels({close_id}, 0.2);
    append_graph.update(append_query, 1);
    ok &= expect(append_graph.snapshot().nodes.size() == 2,
                 "global candidate separation must reject a local node cluster");
    append_graph.markDirty(seed_position);
    append_graph.update(append_query, 1);
    const auto append_revisit = append_graph.snapshot();
    ok &= expect(append_revisit.nodes.size() == 2 &&
                     append_revisit.last_sampled_center_count == 0,
                 "a committed region without new evidence must not be fully resampled");

    first_cell_occupied = true;
    dense_graph.markDirty(Vec3f(0.5, 0.5, 1.0));
    dense_graph.update(dense_query, 1);
    ok &= expect(dense_graph.snapshot().nodes.size() == 7,
                 "ROG occupied updates must remove a formerly free node");

    // Unknown gaps are not fabricated merely to force connectivity.
    IncrementalTopologyGraph::Config gap_config = dense_config;
    gap_config.connection_radius = 2.5;
    gap_config.max_nodes_per_region = 64;
    IncrementalTopologyGraph gap_graph(gap_config);
    IncrementalTopologyGraph::Query gap_query = dense_query;
    gap_query.traversable = [](const Vec3f &point) {
        const int x = static_cast<int>(std::floor(point.x()));
        const int y = static_cast<int>(std::floor(point.y()));
        return y == 0 && (x == 0 || x == 2);
    };
    gap_graph.markDirty(Vec3f(0.5, 0.5, 1.0));
    gap_graph.markDirty(Vec3f(2.5, 0.5, 1.0));
    while (gap_graph.stats().dirty_region_count > 0) {
        gap_graph.update(gap_query, 4);
    }
    const auto gap_snap = gap_graph.snapshot();
    ok &= expect(gap_snap.nodes.size() == 2,
                 "both ROG KNOWN_FREE endpoints must remain");
    ok &= expect(gap_snap.edges.empty(),
                 "an unknown middle cell must prevent a fabricated bridge");

    // Valid adjacent lattice edges are mandatory and cannot be removed by a
    // small max_neighbors budget, including in the vertical direction.
    IncrementalTopologyGraph::Config layer_config = dense_config;
    layer_config.planar_mode = false;
    layer_config.region_size = 4.0;
    layer_config.connection_radius = 1.1;
    layer_config.max_neighbors = 2;
    IncrementalTopologyGraph layer_graph(layer_config);
    IncrementalTopologyGraph::Query layer_query;
    layer_query.traversable = [](const Vec3f &point) {
        const int x = static_cast<int>(std::floor(point.x()));
        const int y = static_cast<int>(std::floor(point.y()));
        const int z = static_cast<int>(std::floor(point.z()));
        return (x == 0 && y == 0 && z >= 0 && z <= 2) ||
               (z == 1 && ((x == 1 && y == 0) ||
                            (x == 0 && y == 1)));
    };
    layer_query.clearance = dense_query.clearance;
    layer_graph.markDirty(Vec3f(0.5, 0.5, 1.5));
    while (layer_graph.stats().dirty_region_count > 0) {
        layer_graph.update(layer_query, 4);
    }
    const auto layer_snap = layer_graph.snapshot();
    std::uint64_t lower_id = 0;
    std::uint64_t middle_id = 0;
    std::uint64_t upper_id = 0;
    for (const auto &node : layer_snap.nodes) {
        if (std::abs(node.position.x() - 0.5) > 1.0e-9 ||
            std::abs(node.position.y() - 0.5) > 1.0e-9) {
            continue;
        }
        if (std::abs(node.position.z() - 0.5) < 1.0e-9) lower_id = node.id;
        if (std::abs(node.position.z() - 1.5) < 1.0e-9) middle_id = node.id;
        if (std::abs(node.position.z() - 2.5) < 1.0e-9) upper_id = node.id;
    }
    bool lower_connected = false;
    bool upper_connected = false;
    for (const auto &edge : layer_snap.edges) {
        lower_connected = lower_connected ||
            ((edge.from == lower_id && edge.to == middle_id) ||
             (edge.from == middle_id && edge.to == lower_id));
        upper_connected = upper_connected ||
            ((edge.from == upper_id && edge.to == middle_id) ||
             (edge.from == middle_id && edge.to == upper_id));
    }
    ok &= expect(lower_id != 0 && middle_id != 0 && upper_id != 0,
                 "ROG KNOWN_FREE samples must include lower/middle/upper layers");
    ok &= expect(lower_connected && upper_connected,
                 "dense 3D graph must retain valid portals to both height layers");

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
