#include <map_manager/map_manager.hpp>

#include <algorithm>
#include <vector>

namespace general_planner {

void MapManager::syncBoundaryMapImpl(const rog_map::ROGMapROS::Ptr &map,
                                     const BoundaryMap::Ptr &boundary_map,
                                     const IncrementalTopologyGraph::Ptr &topology_graph) {
    if (!map || !boundary_map) {
        return;
    }
    const std::vector<rog_map::ProbMap::CellStateChange> changes =
        map->drainStateChanges();
    if (changes.empty()) {
        return;
    }

    if (topology_graph) {
        const auto topology_config = topology_graph->config();
        if (topology_graph->active() &&
            topology_config.enabled &&
            topology_config.dense_known_free) {
            std::vector<
                IncrementalTopologyGraph::VoxelEvidenceDelta> deltas;
            deltas.reserve(changes.size());
            for (const auto &change : changes) {
                IncrementalTopologyGraph::VoxelEvidenceDelta delta;
                delta.index = change.id_g;
                delta.free_delta =
                    (change.to_type == rog_map::GridType::KNOWN_FREE ? 1 : 0) -
                    (change.from_type == rog_map::GridType::KNOWN_FREE ? 1 : 0);
                delta.occupied_delta =
                    (change.to_type == rog_map::GridType::OCCUPIED ? 1 : 0) -
                    (change.from_type == rog_map::GridType::OCCUPIED ? 1 : 0);
                deltas.push_back(delta);
            }
            topology_graph->integrateDenseEvidence(
                deltas, map->getResolution());
        } else if (topology_graph->active()) {
            std::vector<rog_map::Vec3i> changed_indices;
            changed_indices.reserve(changes.size());
            for (const auto &change : changes) {
                changed_indices.push_back(change.id_g);
            }
            topology_graph->markDirtyVoxels(
                changed_indices, map->getResolution());
        }
    }

    static const rog_map::Vec3i kSixNeighbors[] = {
        rog_map::Vec3i(1, 0, 0),  rog_map::Vec3i(-1, 0, 0),
        rog_map::Vec3i(0, 1, 0),  rog_map::Vec3i(0, -1, 0),
        rog_map::Vec3i(0, 0, 1),  rog_map::Vec3i(0, 0, -1)};

    // Only a changed voxel and its six neighbors can change boundary status.
    std::vector<rog_map::Vec3i> affected;
    affected.reserve(changes.size() * 7U);
    for (const auto &change : changes) {
        affected.push_back(change.id_g);
        for (const rog_map::Vec3i &offset : kSixNeighbors) {
            affected.push_back(change.id_g + offset);
        }
    }

    const auto indexLess = [](const rog_map::Vec3i &lhs,
                              const rog_map::Vec3i &rhs) {
        if (lhs.x() != rhs.x()) return lhs.x() < rhs.x();
        if (lhs.y() != rhs.y()) return lhs.y() < rhs.y();
        return lhs.z() < rhs.z();
    };
    const auto indexEqual = [](const rog_map::Vec3i &lhs,
                               const rog_map::Vec3i &rhs) {
        return lhs.x() == rhs.x() && lhs.y() == rhs.y() &&
               lhs.z() == rhs.z();
    };
    std::sort(affected.begin(), affected.end(), indexLess);
    affected.erase(std::unique(affected.begin(), affected.end(), indexEqual),
                   affected.end());

    const auto stateAt = [&](const rog_map::Vec3i &index) {
        if (map->insideLocalMap(index)) {
            rog_map::Vec3i mutable_index = index;
            return map->getGridType(mutable_index);
        }
        return boundary_map->getGridType(index);
    };

    std::vector<BoundaryMap::BoundaryVoxel> updates;
    updates.reserve(affected.size());
    for (const rog_map::Vec3i &index : affected) {
        const rog_map::GridType state = stateAt(index);
        BoundaryMap::BoundaryType type = BoundaryMap::BoundaryType::NONE;
        if (state == rog_map::GridType::OCCUPIED) {
            type = BoundaryMap::BoundaryType::OCCUPIED;
        } else {
            bool neighbor_free = false;
            bool neighbor_non_free = false;
            for (const rog_map::Vec3i &offset : kSixNeighbors) {
                const rog_map::GridType neighbor = stateAt(index + offset);
                neighbor_free = neighbor_free ||
                                neighbor == rog_map::GridType::KNOWN_FREE;
                neighbor_non_free = neighbor_non_free ||
                                    neighbor != rog_map::GridType::KNOWN_FREE;
            }
            if (state == rog_map::GridType::KNOWN_FREE && neighbor_non_free) {
                type = BoundaryMap::BoundaryType::INTERIOR_FREE;
            } else if (state == rog_map::GridType::UNKNOWN && neighbor_free) {
                type = BoundaryMap::BoundaryType::EXTERIOR_UNKNOWN;
            }
        }
        updates.push_back({index, type});
    }
    boundary_map->apply(updates);
}

} // namespace general_planner
