#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
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
    enum class EvidenceState : std::uint8_t {
        UNKNOWN = 0,
        KNOWN_FREE = 1,
        OCCUPIED = 2
    };

    virtual ~TopologyMapView() = default;
    /**
     * Persistent topology must distinguish missing evidence from a confirmed
     * obstacle. UNKNOWN may neither create new graph geometry nor erase
     * geometry committed from an earlier KNOWN_FREE observation.
     */
    virtual EvidenceState evidenceState(
        const rog_map::Vec3f &position) const = 0;
    bool isTraversable(const rog_map::Vec3f &position) const {
        return evidenceState(position) == EvidenceState::KNOWN_FREE;
    }
    virtual bool getClearance(const rog_map::Vec3f &position,
                              double &distance) const = 0;
};

/** Incremental persistent free-space topology owned by MapManager. */
class IncrementalTopologyGraph {
public:
    using Ptr = std::shared_ptr<IncrementalTopologyGraph>;
    using NodeId = std::uint64_t;

    enum class ConstructionMode : std::uint8_t {
        PERSISTENT_BUBBLE_SKELETON = 0,
        DENSE_KNOWN_FREE_DEBUG = 1
    };

    enum class NodeState : std::uint8_t {
        ACTIVE = 0,
        HISTORICAL = 1
    };

    /**
     * Skeleton edges are inferred from sampled free space. Traversal edges
     * are different: they encode a route that the vehicle has actually flown
     * and therefore must not be evicted by ordinary region/degree rebuilding.
     */
    enum class EdgeRole : std::uint8_t {
        SKELETON = 0,
        TRAVERSAL_BACKBONE = 1,
        TRAVERSAL_ATTACHMENT = 2
    };

    /** Faces of a topology region reached by a selected bubble portal. */
    enum PortalFace : std::uint8_t {
        PORTAL_NEG_X = 1U << 0U,
        PORTAL_POS_X = 1U << 1U,
        PORTAL_NEG_Y = 1U << 2U,
        PORTAL_POS_Y = 1U << 3U,
        PORTAL_NEG_Z = 1U << 4U,
        PORTAL_POS_Z = 1U << 5U
    };

    static const char *constructionModeName(ConstructionMode mode);
    static ConstructionMode constructionModeFromString(
        const std::string &name);

    struct Config {
        bool enabled{false};
        /**
         * Build a deterministic globally aligned lattice over observed free
         * space. Unlike the legacy bubble-component representation, every
         * traversable lattice sample is retained, so revisited and
         * slide-out regions form one persistent dense roadmap.
         *
         * This mode always rejects unknown space, regardless of
         * unknown_as_free. Traversability comes directly from the map view
         * supplied by MapManager (current ROG Map with global-map fallback).
         */
        ConstructionMode construction_mode{
            ConstructionMode::PERSISTENT_BUBBLE_SKELETON};
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
        /** Do not consume an unordered historical dirty backlog without a
         * current robot focus. */
        bool require_fresh_focus{false};
        /** A non-positive value disables the focus-age gate. */
        double focus_timeout{0.0};
        /** A non-positive value allows all dirty regions around a focus. */
        double max_focus_distance{0.0};
        double update_period{0.20};
        double publish_period{0.50};
        /**
         * Real-time planning needs a fresh immutable graph after each batch.
         * Recording/visualization-only dense maps disable this and let their
         * ROS adapter refresh at a lower publication rate.
         */
        bool snapshot_every_update{true};
        /** Keep an evidence-validated topology chain along received odometry. */
        bool retain_traversal_backbone{true};
        /** Maximum chord length between consecutive recorded trajectory anchors. */
        double traversal_sample_spacing{0.50};
        /** Radius used to merge a confirmed anchor into the sampled skeleton. */
        double traversal_attach_radius{3.0};
        /** Bound per-worker validation work; pending segments are prioritized. */
        std::size_t max_traversal_edges_per_update{128};
        /** Bound asynchronous odometry buffering without silently joining a gap. */
        std::size_t max_pending_traversal_samples{4096};
        /** Avoid attaching an unbounded number of trajectory anchors to one core node. */
        std::size_t max_traversal_attachments_per_node{4};
    };

    struct Query final : TopologyMapView {
        /** Preferred tri-state global occupancy query. */
        std::function<EvidenceState(const rog_map::Vec3f &)> evidence;
        /** True only for free space satisfying the desired robot inflation. */
        std::function<bool(const rog_map::Vec3f &)> traversable;
        /** Optional ESDF/clearance query. Return false when unavailable. */
        std::function<bool(const rog_map::Vec3f &, double &)> clearance;

        EvidenceState evidenceState(
            const rog_map::Vec3f &position) const override {
            if (evidence) {
                return evidence(position);
            }
            // Compatibility for standalone users/tests which still provide a
            // binary map. A false binary result is conservative OCCUPIED.
            return traversable && traversable(position)
                ? EvidenceState::KNOWN_FREE : EvidenceState::OCCUPIED;
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
        std::uint64_t last_observed_revision{0};
        NodeState state{NodeState::ACTIVE};
        /** Region faces this bubble reached when it entered the topology. */
        std::uint8_t portal_mask{0};
        /** Portal faces not currently connected to another active region. */
        std::uint8_t expansion_mask{0};
        /** This node is an anchor sampled from the vehicle's actual motion. */
        bool traversal_anchor{false};
    };

    struct Edge {
        NodeId from{0};
        NodeId to{0};
        double cost{0.0};
        rog_map::vec_Vec3f polyline;
        std::uint64_t validated_revision{0};
        EdgeRole role{EdgeRole::SKELETON};
    };

    struct Snapshot {
        std::vector<Node, Eigen::aligned_allocator<Node>> nodes;
        std::vector<Edge> edges;
        ConstructionMode construction_mode{
            ConstructionMode::PERSISTENT_BUBBLE_SKELETON};
        std::uint64_t revision{0};
        std::size_t known_free_cell_count{0};
        std::size_t dirty_region_count{0};
        std::size_t empty_region_count{0};
        std::size_t last_sampled_center_count{0};
        std::size_t last_traversable_center_count{0};
        std::size_t last_clearance_rejected_count{0};
        /** Raw internal candidates retained for future reconnection. */
        std::size_t raw_node_count{0};
        /** Raw nodes deliberately omitted from the public/search graph. */
        std::size_t pending_node_count{0};
        /** Raw candidates with no collision-validated graph edge. */
        std::size_t isolated_node_count{0};
        /** Number of raw connected components with at least one edge. */
        std::size_t connected_component_count{0};
        std::size_t traversal_node_count{0};
        std::size_t traversal_edge_count{0};
        std::size_t pending_traversal_edge_count{0};
        std::size_t invalid_traversal_edge_count{0};
        std::size_t dropped_traversal_sample_count{0};
    };

    /** Immutable graph revision shared by real-time planning queries. */
    struct SearchNode {
        Node node;
        std::unordered_map<NodeId, double> neighbors;
        std::unordered_map<NodeId, EdgeRole> edge_roles;
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
        std::size_t known_free_cell_count{0};
        std::size_t dirty_region_count{0};
        std::size_t rebuilt_region_count{0};
        std::size_t empty_region_count{0};
        std::size_t last_sampled_center_count{0};
        std::size_t last_traversable_center_count{0};
        std::size_t last_clearance_rejected_count{0};
        /** node_count/edge_count describe raw retained candidates. */
        std::size_t public_node_count{0};
        std::size_t public_edge_count{0};
        std::size_t pending_node_count{0};
        std::size_t isolated_node_count{0};
        std::size_t connected_component_count{0};
        std::size_t traversal_node_count{0};
        std::size_t traversal_edge_count{0};
        std::size_t pending_traversal_edge_count{0};
        std::size_t invalid_traversal_edge_count{0};
        std::size_t dropped_traversal_sample_count{0};
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
    /** Seed not-yet-observed regions along a successful navigation route. */
    void observePlannedPath(const rog_map::vec_Vec3f &path);
    /**
     * Non-blocking odometry ingress.  This records vehicle motion only; the
     * topology worker later validates it against the global map before it can
     * become a routable edge.
     */
    void recordTraversalPose(const rog_map::Vec3f &position);

    /** Rebuild at most max_regions (or the configured budget when zero). */
    std::size_t update(const TopologyMapView &map_view,
                       std::size_t max_regions = 0,
                       const rog_map::Vec3f *focus = nullptr);

    Snapshot snapshot() const;
    Stats stats() const;
    SearchSnapshotPtr acquireSearchSnapshot() const;
    /** Publish the current mutable graph as a new immutable snapshot. */
    void refreshSnapshot();

    /** Euclidean-heuristic A* path. Failure never fabricates a direct connection. */
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

    struct NodeRecord {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Node node;
        RegionKey region;
        std::unordered_map<NodeId, double> neighbors;
        /**
         * Updated once per maintenance pass.  False means this node is a
         * retained candidate only: it must never reach RViz, the topology
         * expansion message or a planning SearchSnapshot.
         */
        bool public_node{false};
        bool traversal_anchor{false};
    };

    struct EdgeKey {
        NodeId low{0};
        NodeId high{0};

        EdgeKey() = default;
        EdgeKey(const NodeId first, const NodeId second)
            : low(std::min(first, second)), high(std::max(first, second)) {}

        bool operator==(const EdgeKey &other) const {
            return low == other.low && high == other.high;
        }
    };

    struct EdgeKeyHash {
        std::size_t operator()(const EdgeKey &key) const {
            std::size_t seed = std::hash<NodeId>{}(key.low);
            seed ^= std::hash<NodeId>{}(key.high) + 0x9e3779b9U +
                    (seed << 6U) + (seed >> 2U);
            return seed;
        }
    };

    enum class TraversalSegmentState : std::uint8_t {
        PENDING = 0,
        ACTIVE = 1,
        INVALID = 2
    };

    struct TraversalSegment {
        NodeId from{0};
        NodeId to{0};
        TraversalSegmentState state{TraversalSegmentState::PENDING};
        bool validation_queued{true};
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
    /** Globally aligned dense-lattice coordinate used for O(1) lookup. */
    RegionKey denseCellOf(const rog_map::Vec3f &position) const;
    bool constructionTraversable(const rog_map::Vec3f &position,
                                 const TopologyMapView &map_view) const;
    TopologyMapView::EvidenceState lineEvidence(
        const rog_map::Vec3f &start,
        const rog_map::Vec3f &goal,
        const TopologyMapView &map_view,
        double sample_spacing = 0.0) const;
    bool lineTraversable(const rog_map::Vec3f &start,
                         const rog_map::Vec3f &goal,
                         const TopologyMapView &map_view,
                         double sample_spacing = 0.0) const;
    double estimateClearance(const rog_map::Vec3f &position,
                             const TopologyMapView &map_view) const;
    std::vector<Node, Eigen::aligned_allocator<Node>> generateCandidates(
        const RegionKey &region, const TopologyMapView &map_view,
        CandidateDiagnostics &diagnostics,
        const rog_map::vec_Vec3f &evidence_seeds,
        bool evidence_only) const;
    bool popDirtyRegion(RegionKey &region,
                        std::vector<RegionKey> &changed_dense_cells,
                        rog_map::vec_Vec3f &evidence_seeds,
                        const rog_map::Vec3f *focus);
    void rebuildRegion(const RegionKey &region,
                       const std::vector<RegionKey> &changed_dense_cells,
                       const rog_map::vec_Vec3f &evidence_seeds,
                       const TopologyMapView &map_view);
    void rebuildIncidentEdges(const std::vector<NodeId> &source_ids,
                              const TopologyMapView &map_view);
    /** Consume odometry samples and reconcile their persistent map evidence. */
    bool updateTraversalBackbone(const TopologyMapView &map_view);
    /** Prioritize backbone spans geometrically affected by a rebuilt region. */
    void queueTraversalSegmentsForRegion(const RegionKey &region);
    /** Attach confirmed trajectory anchors to nearby sampled skeleton cores. */
    bool attachTraversalAnchors(const TopologyMapView &map_view);
    bool isTraversalEdgeLocked(NodeId from, NodeId to) const;
    EdgeRole edgeRoleLocked(NodeId from, NodeId to) const;
    std::size_t regularNeighborCountLocked(const NodeRecord &node) const;
    std::size_t traversalAttachmentCountLocked(NodeId node_id) const;
    void eraseTraversalEdgesForNodeLocked(NodeId node_id);
    /**
     * Keep every connected component public while filtering zero-degree
     * candidates at the publication boundary. Robot attachment belongs to a
     * path query, not to lifetime topology visibility.
     */
    bool refreshPublicTopology();
    void publishSearchSnapshot();

    mutable std::shared_mutex graph_mutex_;
    mutable std::mutex dirty_mutex_;
    mutable std::mutex update_mutex_;
    mutable std::mutex focus_mutex_;
    mutable std::mutex traversal_ingress_mutex_;
    Config config_;
    std::atomic<bool> active_{false};
    rog_map::Vec3f update_focus_{rog_map::Vec3f::Zero()};
    bool has_update_focus_{false};
    std::chrono::steady_clock::time_point update_focus_time_{};
    SearchSnapshotPtr search_snapshot_;
    NodeMap nodes_;
    RegionMap regions_;
    /** Regions which already completed their one-time full bubble sampling. */
    std::unordered_set<RegionKey, RegionKeyHash> initialized_regions_;
    /** Present only in dense mode; maps one lattice cell to its live node. */
    std::unordered_map<RegionKey, NodeId, RegionKeyHash> dense_node_index_;
    std::unordered_set<RegionKey, RegionKeyHash> dirty_regions_;
    /** Exact coarse cells touched by ROG transitions, grouped by dirty region. */
    std::unordered_map<
        RegionKey,
        std::unordered_set<RegionKey, RegionKeyHash>,
        RegionKeyHash> dirty_dense_cells_;
    /** Exact ROG transition voxel centers used as evidence-aligned bubble seeds. */
    std::unordered_map<RegionKey, rog_map::vec_Vec3f, RegionKeyHash>
        dirty_evidence_seeds_;
    std::unordered_set<RegionKey, RegionKeyHash> observed_route_regions_;
    /** Pose samples await map evidence; only the topology worker consumes them. */
    std::deque<rog_map::Vec3f, Eigen::aligned_allocator<rog_map::Vec3f>>
        pending_traversal_poses_;
    rog_map::Vec3f last_traversal_pose_{rog_map::Vec3f::Zero()};
    bool has_last_traversal_pose_{false};
    std::size_t dropped_traversal_sample_count_{0};
    std::vector<NodeId> traversal_anchor_ids_;
    std::vector<TraversalSegment> traversal_segments_;
    std::unordered_map<RegionKey, std::vector<std::size_t>, RegionKeyHash>
        traversal_segments_by_region_;
    std::unordered_map<EdgeKey, EdgeRole, EdgeKeyHash> traversal_edges_;
    std::size_t traversal_validation_cursor_{0};
    std::size_t traversal_attachment_cursor_{0};
    NodeId last_traversal_node_{0};
    NodeId next_node_id_{1};
    std::uint64_t revision_{0};
    std::size_t rebuilt_region_count_{0};
    std::size_t empty_region_count_{0};
    std::size_t pending_node_count_{0};
    std::size_t isolated_node_count_{0};
    std::size_t connected_component_count_{0};
    /** Cache key for avoiding an O(V + E) component scan on an idle tick. */
    std::uint64_t public_topology_revision_{0};
    bool public_topology_initialized_{false};
    CandidateDiagnostics last_candidate_diagnostics_;
};

} // namespace general_planner
