#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include <map_manager/boundary_map.hpp>
#include <map_manager/incremental_topology_graph.hpp>
#include <rog_map/rog_map.h>
#include <rog_map_ros/rog_map_ros1.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <general_utils/type_utils.hpp>

namespace general_planner
{
class MapManager
{
public:
    using Ptr = std::shared_ptr<MapManager>;

    struct UpdateSnapshot
    {
        std::uint64_t revision{0};
        bool changed_box_valid{false};
        rog_map::Vec3f changed_min{rog_map::Vec3f::Zero()};
        rog_map::Vec3f changed_max{rog_map::Vec3f::Zero()};
    };

    MapManager() = default;

    explicit MapManager(const rog_map::ROGMapROS::Ptr &map)
    {
        setMap(map);
    }

    void setMap(const rog_map::ROGMapROS::Ptr &map)
    {
        if (map_) {
            map_->setStateChangeCallback({});
            map_->setRobotStateCallback({});
        }
        map_ = map;
        map_revision_.store(0, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(update_snapshot_mutex_);
            latest_update_ = UpdateSnapshot{};
        }
        topology_seeded_.store(false, std::memory_order_release);
        boundary_map_.reset();
        if (!topology_graph_) {
            topology_graph_ = std::make_shared<IncrementalTopologyGraph>();
        } else {
            topology_graph_->clear();
        }
        if (map_) {
            // BoundaryMap consumes only sensor-driven discrete transitions.
            // Any pending stream from a previous owner is not part of this
            // manager's global history.
            map_->setStateChangeTrackingEnabled(false);
            map_->drainStateChanges();
            boundary_map_ = std::make_shared<BoundaryMap>(map_->getResolution());
            map_->setStateChangeTrackingEnabled(true);
            const std::weak_ptr<rog_map::ROGMapROS> weak_map = map_;
            const std::weak_ptr<BoundaryMap> weak_boundary = boundary_map_;
            const std::weak_ptr<IncrementalTopologyGraph> weak_topology = topology_graph_;
            map_->setStateChangeCallback([weak_map, weak_boundary, weak_topology]() {
                const auto map = weak_map.lock();
                const auto boundary = weak_boundary.lock();
                const auto topology = weak_topology.lock();
                if (map && boundary) {
                    syncBoundaryMapImpl(map, boundary, topology);
                }
            });

            // Odom only prioritizes asynchronous maintenance in dense
            // known-free mode. New graph content comes from post-integration
            // occupancy transitions, never from robot motion itself.
            struct OdomTopologyTrigger {
                std::mutex mutex;
                bool initialized{false};
                rog_map::Vec3f last_marked{rog_map::Vec3f::Zero()};
            };
            const auto trigger = std::make_shared<OdomTopologyTrigger>();
            map_->setRobotStateCallback(
                [weak_topology, trigger](const rog_map::RobotState &robot) {
                    const auto topology = weak_topology.lock();
                    if (!topology || !topology->active() || !robot.rcv ||
                        !robot.p.allFinite()) {
                        return;
                    }
                    topology->requestUpdateFocus(robot.p);
                    const auto config = topology->config();
                    const double trigger_distance =
                        std::max(0.5, 0.5 * config.region_size);
                    bool mark = false;
                    {
                        std::lock_guard<std::mutex> lock(trigger->mutex);
                        rog_map::Vec3f delta = robot.p - trigger->last_marked;
                        if (config.planar_mode) {
                            delta.z() = 0.0;
                        }
                        mark = !trigger->initialized ||
                               delta.norm() >= trigger_distance;
                        if (mark) {
                            trigger->initialized = true;
                            trigger->last_marked = robot.p;
                        }
                    }
                    if (mark && !config.dense_known_free) {
                        topology->markDirty(robot.p);
                    }
                });
        }
    }

    bool ready() const
    {
        return map_ != nullptr;
    }

    bool boundaryReady() const
    {
        return boundary_map_ != nullptr;
    }

    const rog_map::ROGMap *rawMap() const
    {
        return map_.get();
    }

    rog_map::ROGMapROS::Ptr rawRosMap() const
    {
        return map_;
    }

    rog_map::Config getMapConfig() const
    {
        return map_->getMapConfig();
    }

    UpdateSnapshot updateMap(const rog_map::PointCloud &cloud,
                             const general_utils::Pose &pose) const
    {
        if (!map_) {
            return latestUpdate();
        }
        map_->updateMap(cloud, pose);

        UpdateSnapshot snapshot;
        snapshot.revision =
            map_revision_.fetch_add(1, std::memory_order_acq_rel) + 1;
        snapshot.changed_box_valid =
            map_->getUpdatedBox(snapshot.changed_min, snapshot.changed_max);
        {
            std::lock_guard<std::mutex> lock(update_snapshot_mutex_);
            latest_update_ = snapshot;
        }
        return snapshot;
    }

    std::uint64_t mapRevision() const
    {
        return map_revision_.load(std::memory_order_acquire);
    }

    UpdateSnapshot latestUpdate() const
    {
        std::lock_guard<std::mutex> lock(update_snapshot_mutex_);
        return latest_update_;
    }

    /** Synchronize pending ROG discrete transitions into the sparse global map. */
    void syncBoundaryMap() const
    {
        syncBoundaryMapImpl(map_, boundary_map_, topology_graph_);
    }

    void configureTopology(const IncrementalTopologyGraph::Config &config)
    {
        if (!topology_graph_) {
            topology_graph_ = std::make_shared<IncrementalTopologyGraph>(config);
        } else {
            topology_graph_->configure(config);
        }
        topology_seeded_.store(false, std::memory_order_release);
        seedTopologyFromCurrentWindow();
    }

    bool topologyReady() const
    {
        return topology_graph_ != nullptr && topology_graph_->active();
    }

    /** Enable the state2state-owned graph only while that mission is active. */
    void setTopologyActive(const bool active)
    {
        if (!topology_graph_) {
            return;
        }
        const bool was_active = topology_graph_->active();
        topology_graph_->setActive(active);
        if (!was_active && topology_graph_->active()) {
            topology_seeded_.store(false, std::memory_order_release);
            seedTopologyFromCurrentWindow();
        }
    }

    IncrementalTopologyGraph::Ptr topologyGraph() const
    {
        return topology_graph_;
    }

    std::size_t updateTopology(std::size_t max_regions = 0) const
    {
        if (!map_ || !topologyReady()) {
            return 0;
        }
        if (!seedTopologyFromCurrentWindow()) {
            return 0;
        }
        syncBoundaryMap();
        return topology_graph_->update(makeTopologyQuery(), max_regions);
    }

    /**
     * Non-blocking planner hint. The asynchronous topology worker consumes the
     * latest focus; no map sampling or graph mutation occurs on this call.
     */
    void requestTopologyUpdateAround(const rog_map::Vec3f &focus) const
    {
        if (topologyReady()) {
            topology_graph_->requestUpdateFocus(focus);
        }
    }

    /** Synchronous maintenance API for tests/tools; planners must not call it. */
    std::size_t updateTopologyAround(const rog_map::Vec3f &focus,
                                     std::size_t max_regions = 0) const
    {
        if (!map_ || !topologyReady()) {
            return 0;
        }
        if (!seedTopologyFromCurrentWindow()) {
            return 0;
        }
        syncBoundaryMap();
        return topology_graph_->update(makeTopologyQuery(), max_regions, &focus);
    }

    void observePlannedTopologyPath(const rog_map::vec_Vec3f &path) const
    {
        if (topologyReady()) {
            topology_graph_->observePlannedPath(path);
        }
    }

    IncrementalTopologyGraph::Snapshot topologySnapshot() const
    {
        return topology_graph_ ? topology_graph_->snapshot()
                               : IncrementalTopologyGraph::Snapshot{};
    }

    IncrementalTopologyGraph::SearchSnapshotPtr topologySearchSnapshot() const
    {
        return topology_graph_ ? topology_graph_->acquireSearchSnapshot()
                               : IncrementalTopologyGraph::SearchSnapshotPtr{};
    }

    IncrementalTopologyGraph::Stats topologyStats() const
    {
        return topology_graph_ ? topology_graph_->stats()
                               : IncrementalTopologyGraph::Stats{};
    }

    bool findTopologyPath(const rog_map::Vec3f &start,
                          const rog_map::Vec3f &goal,
                          rog_map::vec_Vec3f &path,
                          const double attach_radius = 0.0) const
    {
        if (!map_ || !topologyReady()) {
            path.clear();
            return false;
        }
        return topology_graph_->findPath(start, goal, makeTopologyQuery(),
                                         path, attach_radius);
    }

    bool findTopologyPath(
        const IncrementalTopologyGraph::SearchSnapshotPtr &snapshot,
        const rog_map::Vec3f &start,
        const rog_map::Vec3f &goal,
        rog_map::vec_Vec3f &path,
        const double attach_radius = 0.0) const
    {
        if (!map_ || !topologyReady() || !snapshot) {
            path.clear();
            return false;
        }
        return topology_graph_->findPath(snapshot, start, goal,
                                         makeTopologyQuery(), path,
                                         attach_radius);
    }

    rog_map::RobotState getRobotState() const
    {
        return map_->getRobotState();
    }

    double getResolution() const
    {
        return map_->getResolution();
    }

    double getInfResolution() const
    {
        return map_->getInfResolution();
    }

    bool insideLocalMap(const rog_map::Vec3f &pos) const
    {
        return map_->insideLocalMap(pos);
    }

    bool insideLocalMap(const rog_map::Vec3i &id_g) const
    {
        return map_->insideLocalMap(id_g);
    }

    rog_map::GridType getGridType(const rog_map::Vec3f &pos) const
    {
        return map_->getGridType(pos);
    }

    /** Persistent BoundaryMap state, without consulting the local ROG window. */
    rog_map::GridType getBoundaryGridType(const rog_map::Vec3f &pos) const
    {
        syncBoundaryMap();
        if (!boundary_map_) {
            return rog_map::GridType::UNKNOWN;
        }
        return boundary_map_->getGridType(pos);
    }

    /**
     * Global raw occupancy query for long-range planning.
     *
     * Current local ROG evidence has priority.  If the local ring-buffer cell
     * is unknown after a slide, historical BoundaryMap evidence fills the gap.
     * Local safety and trajectory code must continue using getGridType() and
     * getInfGridType(), whose semantics remain strictly local.
     */
    rog_map::GridType getGlobalGridType(const rog_map::Vec3f &pos) const
    {
        if (!map_ || !pos.allFinite()) {
            return rog_map::GridType::OUT_OF_MAP;
        }
        const rog_map::Config config = map_->getMapConfig();
        if (pos.z() <= config.virtual_ground_height ||
            pos.z() >= config.virtual_ceil_height) {
            return rog_map::GridType::OCCUPIED;
        }
        if (map_->insideLocalMap(pos)) {
            const rog_map::GridType local = map_->getGridType(pos);
            if (local == rog_map::GridType::KNOWN_FREE ||
                local == rog_map::GridType::OCCUPIED) {
                return local;
            }
        }
        return getBoundaryGridType(pos);
    }

    bool isGloballyKnownFree(const rog_map::Vec3f &pos) const
    {
        return getGlobalGridType(pos) == rog_map::GridType::KNOWN_FREE;
    }

    bool isGloballyOccupied(const rog_map::Vec3f &pos) const
    {
        return getGlobalGridType(pos) == rog_map::GridType::OCCUPIED;
    }

    rog_map::vec_Vec3f getGlobalFrontiers(std::size_t max_count = 0) const
    {
        syncBoundaryMap();
        return boundary_map_ ? boundary_map_->frontierPositions(max_count)
                             : rog_map::vec_Vec3f{};
    }

    BoundaryMap::Stats getBoundaryMapStats() const
    {
        syncBoundaryMap();
        return boundary_map_ ? boundary_map_->stats() : BoundaryMap::Stats{};
    }

    BoundaryMap::Ptr rawBoundaryMap() const
    {
        syncBoundaryMap();
        return boundary_map_;
    }

    rog_map::GridType getInfGridType(const rog_map::Vec3f &pos) const
    {
        return map_->getInfGridType(pos);
    }

    bool isOccupiedInflate(const rog_map::Vec3f &pos) const
    {
        return map_->isOccupiedInflate(pos);
    }

    bool isLineFree(const rog_map::Vec3f &start_pt,
                    const rog_map::Vec3f &end_pt,
                    const double &max_dis,
                    const rog_map::vec_E<rog_map::Vec3i> &neighbor_list) const
    {
        return map_->isLineFree(start_pt, end_pt, max_dis, neighbor_list);
    }

    bool isLineFree(const rog_map::Vec3f &start_pt,
                    const rog_map::Vec3f &end_pt,
                    const bool &use_inf_map,
                    const bool &use_unk_as_occ) const
    {
        const bool safe_use_unk_as_occ =
                use_unk_as_occ &&
                (!use_inf_map || map_->getMapConfig().unk_inflation_en);
        return map_->isLineFree(start_pt, end_pt, use_inf_map, safe_use_unk_as_occ);
    }

    bool getNearestCellNot(const rog_map::GridType &target_type,
                           const rog_map::Vec3f &start_pos,
                           rog_map::Vec3f &nearest_pt,
                           const double &max_dis) const
    {
        return map_->getNearestCellNot(target_type, start_pos, nearest_pt, max_dis);
    }

    bool getNearestInfCellNot(const rog_map::GridType &target_type,
                              const rog_map::Vec3f &start_pos,
                              rog_map::Vec3f &nearest_pt,
                              const double &max_dis) const
    {
        return map_->getNearestInfCellNot(target_type, start_pos, nearest_pt, max_dis);
    }

    void probMapPosToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const
    {
        map_->probMapPosToGlobalIndex(pos, id_g);
    }

    void probMapGlobalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const
    {
        map_->probMapGlobalIndexToPos(id_g, pos);
    }

    void infMapPosToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const
    {
        map_->infMapPosToGlobalIndex(pos, id_g);
    }

    void infMapGlobalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const
    {
        map_->infMapGlobalIndexToPos(id_g, pos);
    }

    void boundBoxByLocalMap(rog_map::Vec3f &box_min, rog_map::Vec3f &box_max) const
    {
        map_->boundBoxByLocalMap(box_min, box_max);
    }

    bool getUpdatedBox(rog_map::Vec3f &box_min, rog_map::Vec3f &box_max) const
    {
        return map_ != nullptr && map_->getUpdatedBox(box_min, box_max);
    }

    void boxSearch(const rog_map::Vec3f &box_min,
                   const rog_map::Vec3f &box_max,
                   const rog_map::GridType &gt,
                   rog_map::vec_E<rog_map::Vec3f> &out_points) const
    {
        map_->boxSearch(box_min, box_max, gt, out_points);
    }

    void boxSearchInflate(const rog_map::Vec3f &box_min,
                          const rog_map::Vec3f &box_max,
                          const rog_map::GridType &gt,
                          rog_map::vec_E<rog_map::Vec3f> &out_points) const
    {
        map_->boxSearchInflate(box_min, box_max, gt, out_points);
    }

    bool hasESDF() const
    {
        return map_ != nullptr && map_->hasESDF();
    }

    bool evaluateESDF(const rog_map::Vec3f &pos,
                      double &dist,
                      rog_map::Vec3f &grad) const
    {
        if (map_ == nullptr) {
            dist = 0.0;
            grad.setZero();
            return false;
        }
        return map_->evaluateESDF(pos, dist, grad);
    }

    double getESDFDistance(const rog_map::Vec3f &pos) const
    {
        return map_ == nullptr ? 0.0 : map_->getESDFDistance(pos);
    }

    bool findNearestESDFSafe(const rog_map::Vec3f &start_pos,
                             const double min_distance,
                             rog_map::Vec3f &nearest_pt,
                             const double max_dis) const
    {
        if (map_ == nullptr || !hasESDF() || min_distance <= 0.0 || max_dis < 0.0) {
            return false;
        }

        auto isSafe = [&](const rog_map::Vec3f &pos, double *dist_out = nullptr) {
            if (!insideLocalMap(pos)) {
                return false;
            }
            const auto grid_type = getGridType(pos);
            const auto inf_grid_type = getInfGridType(pos);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP ||
                inf_grid_type == rog_map::GridType::OCCUPIED ||
                inf_grid_type == rog_map::GridType::OUT_OF_MAP) {
                return false;
            }

            double dist = 0.0;
            rog_map::Vec3f grad = rog_map::Vec3f::Zero();
            if (!evaluateESDF(pos, dist, grad)) {
                return false;
            }
            if (dist_out != nullptr) {
                *dist_out = dist;
            }
            return std::isfinite(dist) && dist >= min_distance;
        };

        if (isSafe(start_pos)) {
            nearest_pt = start_pos;
            return true;
        }

        const double res = std::max(0.05, getResolution());
        const int max_step = static_cast<int>(std::ceil(max_dis / res));
        const int max_z_step = std::max(1, static_cast<int>(std::ceil(0.6 / res)));
        double best_sq = std::numeric_limits<double>::infinity();
        bool found = false;

        for (int r = 1; r <= max_step; ++r) {
            const int z_bound = std::min(r, max_z_step);
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    for (int dz = -z_bound; dz <= z_bound; ++dz) {
                        if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
                            continue;
                        }
                        const rog_map::Vec3f candidate = start_pos + res * rog_map::Vec3f(dx, dy, dz);
                        const double sq = (candidate - start_pos).squaredNorm();
                        if (sq > max_dis * max_dis || sq >= best_sq) {
                            continue;
                        }
                        if (isSafe(candidate)) {
                            nearest_pt = candidate;
                            best_sq = sq;
                            found = true;
                        }
                    }
                }
            }
            if (found) {
                return true;
            }
        }
        return false;
    }

private:
    /**
     * Delay the initial dirty-window seed until the first real odometry has
     * initialized ROG's sliding-map origin. Dense mode then samples only
     * observed known-free cells; legacy unknown-as-free mode is likewise
     * prevented from creating nodes around the default origin.
     */
    bool seedTopologyFromCurrentWindow() const
    {
        if (!map_ || !topologyReady()) {
            return false;
        }
        const rog_map::RobotState robot = map_->getRobotState();
        if (!robot.rcv) {
            return false;
        }
        bool expected = false;
        if (topology_seeded_.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
            if (topology_graph_->config().dense_known_free) {
                // Dense mode persists map transitions while state2state owns
                // it. Seeding the full rolling window would enqueue hundreds
                // of mostly empty regions and starve moving updates.
                return true;
            }
            rog_map::Vec3f box_min(-1.0e6, -1.0e6, -1.0e6);
            rog_map::Vec3f box_max(1.0e6, 1.0e6, 1.0e6);
            map_->boundBoxByLocalMap(box_min, box_max);
            topology_graph_->markDirtyBox(box_min, box_max);
        }
        return true;
    }

    static void syncBoundaryMapImpl(const rog_map::ROGMapROS::Ptr &map,
                                    const BoundaryMap::Ptr &boundary_map,
                                    const IncrementalTopologyGraph::Ptr &topology_graph);

    IncrementalTopologyGraph::Query makeTopologyQuery() const
    {
        IncrementalTopologyGraph::Query query;
        const auto topology_config = topology_graph_
            ? topology_graph_->config()
            : IncrementalTopologyGraph::Config{};
        const bool unknown_as_free =
            topology_config.unknown_as_free &&
            !topology_config.dense_known_free;
        const bool dense_known_free = topology_config.dense_known_free;
        query.traversable = [this, unknown_as_free, dense_known_free](
                                const rog_map::Vec3f &position) {
            if (!map_ || !position.allFinite()) {
                return false;
            }
            const rog_map::Config config = map_->getMapConfig();
            if (position.z() <= config.virtual_ground_height ||
                position.z() >= config.virtual_ceil_height) {
                return false;
            }
            if (map_->insideLocalMap(position)) {
                const rog_map::GridType raw = map_->getGridType(position);
                const rog_map::GridType inflated =
                    map_->getInfGridType(position);
                const bool inflation_safe =
                    inflated != rog_map::GridType::OCCUPIED &&
                    inflated != rog_map::GridType::OUT_OF_MAP;
                if (raw == rog_map::GridType::KNOWN_FREE) {
                    // Dense topology records observed free-space evidence.
                    // The inflated layer is a planning/safety representation
                    // and may conservatively mark an entire narrow flight
                    // layer occupied. It must not erase valid raw-map memory.
                    if (dense_known_free) {
                        return true;
                    }
                    return inflation_safe &&
                           (unknown_as_free || inflated == rog_map::GridType::KNOWN_FREE);
                }
                if (raw == rog_map::GridType::OCCUPIED) {
                    return false;
                }
                if (dense_known_free) {
                    // Never promote UNKNOWN via lattice proximity. Every
                    // stored dense node must lie in a raw KNOWN_FREE voxel.
                    return false;
                }
                // This option is explicit because ordinary state2state may be
                // configured to plan through unknown cells. It is restricted
                // to the current local map; unseen global space is never
                // promoted to persistent free space.
                if (unknown_as_free) {
                    return inflation_safe;
                }
            }
            return boundary_map_ &&
                   boundary_map_->getGridType(position) ==
                   rog_map::GridType::KNOWN_FREE;
        };
        query.clearance = [this](const rog_map::Vec3f &position, double &distance) {
            if (!map_ || !map_->insideLocalMap(position) || !map_->hasESDF()) {
                return false;
            }
            rog_map::Vec3f gradient = rog_map::Vec3f::Zero();
            return map_->evaluateESDF(position, distance, gradient);
        };
        return query;
    }

    rog_map::ROGMapROS::Ptr map_;
    BoundaryMap::Ptr boundary_map_;
    IncrementalTopologyGraph::Ptr topology_graph_{
        std::make_shared<IncrementalTopologyGraph>()};
    mutable std::atomic<bool> topology_seeded_{false};
    mutable std::atomic<std::uint64_t> map_revision_{0};
    mutable std::mutex update_snapshot_mutex_;
    mutable UpdateSnapshot latest_update_;
};
} // namespace general_planner
