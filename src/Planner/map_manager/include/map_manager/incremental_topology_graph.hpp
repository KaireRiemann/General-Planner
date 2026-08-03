#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rog_map/rog_map_core/common_lib.hpp>

namespace general_planner {

/**
 * Read-only map contract used by the topology core.
 *
 * The graph deliberately knows nothing about ROGMap, LIO or a planner FSM.
 * MapManager and exploration may provide different adapters with identical
 * traversability/clearance semantics.
 */
class TopologyMapView {
public:
    virtual ~TopologyMapView() = default;
    virtual bool isTraversable(const rog_map::Vec3f &position) const = 0;
    virtual bool getClearance(const rog_map::Vec3f &position,
                              double &distance) const = 0;
};

/** Incremental persistent free-space topology owned by MapManager. */
class IncrementalTopologyGraph {
public:
    using Ptr = std::shared_ptr<IncrementalTopologyGraph>;
    using NodeId = std::uint64_t;

    struct Config {
        bool enabled{false};
        /**
         * Build a deterministic globally aligned lattice over observed free
         * space. Unlike the legacy bubble-component representation, every
         * traversable lattice sample is retained, so revisited and
         * slide-out regions form one persistent dense roadmap.
         *
         * This mode always rejects unknown space, regardless of
         * unknown_as_free. Planned paths are scheduling hints only and never
         * become free-space evidence.
         */
        bool dense_known_free{false};
        /**
         * Build a 2.5D graph on navigation_altitude. This is intended for
         * state2state flight in a narrow inflated altitude band: virtual
         * ground/ceiling still gate traversability, but they are not treated
         * as horizontal bubble obstacles a second time.
         */
        bool planar_mode{false};
        double navigation_altitude{0.0};
        double region_size{4.0};
        double sample_spacing{1.0};
        /**
         * In planar dense mode, raw free/occupied voxel transitions within
         * this vertical distance of navigation_altitude are projected into
         * the persistent topology evidence grid.
         */
        double dense_evidence_vertical_tolerance{0.50};
        double min_clearance{0.45};
        double max_clearance{2.5};
        double candidate_separation{1.5};
        double stable_match_distance{1.0};
        double connection_radius{6.0};
        double edge_sample_spacing{0.20};
        double dirty_padding{2.5};
        double bubble_overlap_margin{0.10};
        bool unknown_as_free{false};
        std::size_t max_nodes_per_region{4};
        std::size_t max_bubbles_per_region{256};
        std::size_t max_neighbors{8};
        std::size_t max_regions_per_update{4};
        double update_period{0.20};
        double publish_period{0.50};
        /**
         * Real-time planning needs a fresh immutable graph after each batch.
         * Recording/visualization-only dense maps disable this and let their
         * ROS adapter refresh at a lower publication rate.
         */
        bool snapshot_every_update{true};
    };

    struct Query final : TopologyMapView {
        /** True only for free space satisfying the desired robot inflation. */
        std::function<bool(const rog_map::Vec3f &)> traversable;
        /** Optional ESDF/clearance query. Return false when unavailable. */
        std::function<bool(const rog_map::Vec3f &, double &)> clearance;

        bool isTraversable(const rog_map::Vec3f &position) const override {
            return traversable && traversable(position);
        }

        bool getClearance(const rog_map::Vec3f &position,
                          double &distance) const override {
            return clearance && clearance(position, distance);
        }
    };

    struct Node {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        NodeId id{0};
        rog_map::Vec3f position{rog_map::Vec3f::Zero()};
        double clearance{0.0};
        std::uint64_t revision{0};
    };

    struct Edge {
        NodeId from{0};
        NodeId to{0};
        double cost{0.0};
    };

    /**
     * Map-independent discrete observation update. MapManager translates ROG
     * state transitions into these signed deltas before the rolling map can
     * discard them.
     */
    struct VoxelEvidenceDelta {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        rog_map::Vec3i index{rog_map::Vec3i::Zero()};
        int free_delta{0};
        int occupied_delta{0};
    };

    struct Snapshot {
        std::vector<Node, Eigen::aligned_allocator<Node>> nodes;
        std::vector<Edge> edges;
        bool dense_known_free{false};
        std::uint64_t revision{0};
        std::size_t dense_evidence_cell_count{0};
        std::size_t dirty_region_count{0};
        std::size_t empty_region_count{0};
        std::size_t last_sampled_center_count{0};
        std::size_t last_traversable_center_count{0};
        std::size_t last_clearance_rejected_count{0};
    };

    /** Immutable graph revision shared by real-time planning queries. */
    struct SearchNode {
        Node node;
        std::unordered_map<NodeId, double> neighbors;
    };

    struct SearchSnapshot {
        Config config;
        std::unordered_map<NodeId, SearchNode> graph;
        std::uint64_t revision{0};
    };
    using SearchSnapshotPtr = std::shared_ptr<const SearchSnapshot>;

    struct Stats {
        std::size_t node_count{0};
        std::size_t edge_count{0};
        std::size_t region_count{0};
        std::size_t dense_evidence_cell_count{0};
        std::size_t dirty_region_count{0};
        std::size_t rebuilt_region_count{0};
        std::size_t empty_region_count{0};
        std::size_t last_sampled_center_count{0};
        std::size_t last_traversable_center_count{0};
        std::size_t last_clearance_rejected_count{0};
        std::uint64_t revision{0};
    };

    IncrementalTopologyGraph();
    explicit IncrementalTopologyGraph(const Config &config);

    Config config() const;
    void configure(const Config &config);
    void clear();

    /**
     * Runtime ownership gate. Configuration says whether this resource may be
     * used; active says whether the current mission owns it. Exploration keeps
     * this false so its independent TopoGraph remains the only graph builder.
     */
    bool active() const;
    void setActive(bool active);

    /** Lightweight priority hint consumed by the asynchronous map worker. */
    void requestUpdateFocus(const rog_map::Vec3f &focus);

    void markDirty(const rog_map::Vec3f &position);
    void markDirtyBox(const rog_map::Vec3f &box_min,
                      const rog_map::Vec3f &box_max);
    void markDirtyVoxels(const std::vector<rog_map::Vec3i> &indices,
                         double resolution);
    /**
     * Persist raw occupancy transitions in the globally aligned coarse
     * evidence grid and dirty only cells whose coarse traversability changed.
     */
    void integrateDenseEvidence(
        const std::vector<VoxelEvidenceDelta> &deltas, double resolution);

    /** Seed not-yet-observed regions along a successful navigation route. */
    void observePlannedPath(const rog_map::vec_Vec3f &path);

    /** Rebuild at most max_regions (or the configured budget when zero). */
    std::size_t update(const TopologyMapView &map_view,
                       std::size_t max_regions = 0,
                       const rog_map::Vec3f *focus = nullptr);

    Snapshot snapshot() const;
    Stats stats() const;
    SearchSnapshotPtr acquireSearchSnapshot() const;
    /** Publish the current mutable graph as a new immutable snapshot. */
    void refreshSnapshot();

    /** Weighted shortest path. Failure never fabricates a direct connection. */
    bool findPath(const rog_map::Vec3f &start,
                  const rog_map::Vec3f &goal,
                  const TopologyMapView &map_view,
                  rog_map::vec_Vec3f &path,
                  double attach_radius = 0.0) const;
    bool findPath(const SearchSnapshotPtr &snapshot,
                  const rog_map::Vec3f &start,
                  const rog_map::Vec3f &goal,
                  const TopologyMapView &map_view,
                  rog_map::vec_Vec3f &path,
                  double attach_radius = 0.0) const;

private:
    struct RegionKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const RegionKey &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct RegionKeyHash {
        std::size_t operator()(const RegionKey &key) const;
    };

    struct DenseEvidence {
        std::int64_t free_count{0};
        std::int64_t occupied_count{0};
    };

    struct NodeRecord {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Node node;
        RegionKey region;
        std::unordered_map<NodeId, double> neighbors;
    };

    using NodeMap = std::unordered_map<NodeId, NodeRecord>;
    using RegionMap = std::unordered_map<RegionKey,
                                         std::vector<NodeId>,
                                         RegionKeyHash>;

    struct CandidateDiagnostics {
        std::size_t sampled_center_count{0};
        std::size_t traversable_center_count{0};
        std::size_t clearance_rejected_count{0};
    };

    static Config sanitized(const Config &config);
    RegionKey regionOf(const rog_map::Vec3f &position) const;
    rog_map::Vec3f regionMin(const RegionKey &region) const;
    RegionKey denseCellOf(const rog_map::Vec3f &position) const;
    bool denseEvidenceTraversable(const rog_map::Vec3f &position) const;
    bool constructionTraversable(const rog_map::Vec3f &position,
                                 const TopologyMapView &map_view) const;
    bool lineTraversable(const rog_map::Vec3f &start,
                         const rog_map::Vec3f &goal,
                         const TopologyMapView &map_view,
                         double sample_spacing = 0.0,
                         bool use_dense_evidence = false) const;
    double estimateClearance(const rog_map::Vec3f &position,
                             const TopologyMapView &map_view) const;
    std::vector<Node, Eigen::aligned_allocator<Node>> generateCandidates(
        const RegionKey &region, const TopologyMapView &map_view,
        CandidateDiagnostics &diagnostics) const;
    bool popDirtyRegion(RegionKey &region, const rog_map::Vec3f *focus);
    void rebuildRegion(const RegionKey &region, const TopologyMapView &map_view);
    void rebuildIncidentEdges(const std::vector<NodeId> &source_ids,
                              const TopologyMapView &map_view);
    void publishSearchSnapshot();

    mutable std::shared_mutex graph_mutex_;
    mutable std::shared_mutex dense_evidence_mutex_;
    mutable std::mutex dirty_mutex_;
    mutable std::mutex update_mutex_;
    mutable std::mutex focus_mutex_;
    Config config_;
    std::atomic<bool> active_{false};
    rog_map::Vec3f update_focus_{rog_map::Vec3f::Zero()};
    bool has_update_focus_{false};
    SearchSnapshotPtr search_snapshot_;
    NodeMap nodes_;
    RegionMap regions_;
    std::unordered_map<RegionKey, DenseEvidence, RegionKeyHash>
        dense_evidence_;
    std::unordered_set<RegionKey, RegionKeyHash> dirty_regions_;
    std::unordered_set<RegionKey, RegionKeyHash> observed_route_regions_;
    NodeId next_node_id_{1};
    std::uint64_t revision_{0};
    std::size_t rebuilt_region_count_{0};
    std::size_t empty_region_count_{0};
    CandidateDiagnostics last_candidate_diagnostics_;
};

} // namespace general_planner
