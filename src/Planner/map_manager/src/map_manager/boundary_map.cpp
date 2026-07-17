#include <map_manager/boundary_map.hpp>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <unordered_map>

namespace general_planner {
namespace {

std::size_t hashCombine(std::size_t seed, const int value) {
    seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
}

struct IndexKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const IndexKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct IndexKeyHash {
    std::size_t operator()(const IndexKey &key) const {
        return hashCombine(hashCombine(hashCombine(0U, key.x), key.y), key.z);
    }
};

IndexKey toKey(const rog_map::Vec3i &index) {
    return {index.x(), index.y(), index.z()};
}

} // namespace

BoundaryMap::BoundaryMap(const double resolution)
    : resolution_(resolution), resolution_inv_(1.0 / resolution) {
    if (!std::isfinite(resolution) || resolution <= 0.0) {
        throw std::invalid_argument("BoundaryMap resolution must be positive");
    }
}

std::size_t BoundaryMap::ColumnKeyHash::operator()(const ColumnKey &key) const {
    return hashCombine(hashCombine(0U, key.x), key.y);
}

BoundaryMap::ColumnKey BoundaryMap::columnKey(const rog_map::Vec3i &index) {
    return {index.x(), index.y()};
}

bool BoundaryMap::sameIndex(const rog_map::Vec3i &lhs, const rog_map::Vec3i &rhs) {
    return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
}

general_utils::GridType BoundaryMap::stateOf(const BoundaryType type) {
    switch (type) {
    case BoundaryType::INTERIOR_FREE:
        return general_utils::GridType::KNOWN_FREE;
    case BoundaryType::OCCUPIED:
        return general_utils::GridType::OCCUPIED;
    case BoundaryType::EXTERIOR_UNKNOWN:
    case BoundaryType::NONE:
    default:
        return general_utils::GridType::UNKNOWN;
    }
}

void BoundaryMap::clear() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    columns_.clear();
    update_count_ = 0;
}

void BoundaryMap::apply(const BoundaryVoxel &update) {
    apply(std::vector<BoundaryVoxel>{update});
}

void BoundaryMap::apply(const std::vector<BoundaryVoxel> &updates) {
    if (updates.empty()) {
        return;
    }

    // A ray batch may affect the same voxel several times. The status derived
    // from the final local-map state is authoritative.
    std::unordered_map<IndexKey, BoundaryVoxel, IndexKeyHash> final_updates;
    final_updates.reserve(updates.size());
    for (const BoundaryVoxel &update : updates) {
        final_updates[toKey(update.index)] = update;
    }

    std::unique_lock<std::shared_mutex> lock(mutex_);
    for (const auto &entry : final_updates) {
        const BoundaryVoxel &update = entry.second;
        const ColumnKey key = columnKey(update.index);
        auto column_it = columns_.find(key);

        if (column_it == columns_.end()) {
            if (update.type != BoundaryType::NONE) {
                columns_.emplace(key, Column{update});
                ++update_count_;
            }
            continue;
        }

        Column &column = column_it->second;
        auto voxel_it = std::lower_bound(
            column.begin(), column.end(), update.index.z(),
            [](const BoundaryVoxel &voxel, const int z) {
                return voxel.index.z() < z;
            });
        const bool exists = voxel_it != column.end() &&
                            sameIndex(voxel_it->index, update.index);

        if (update.type == BoundaryType::NONE) {
            if (exists) {
                column.erase(voxel_it);
                ++update_count_;
            }
        } else if (exists) {
            if (voxel_it->type != update.type) {
                voxel_it->type = update.type;
                ++update_count_;
            }
        } else {
            column.insert(voxel_it, update);
            ++update_count_;
        }

        if (column.empty()) {
            columns_.erase(column_it);
        }
    }
}

general_utils::GridType BoundaryMap::getGridType(const rog_map::Vec3i &index) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    const auto column_it = columns_.find(columnKey(index));
    if (column_it == columns_.end()) {
        return general_utils::GridType::UNKNOWN;
    }

    const Column &column = column_it->second;
    const auto nearest = std::lower_bound(
        column.begin(), column.end(), index.z(),
        [](const BoundaryVoxel &voxel, const int z) {
            return voxel.index.z() < z;
        });
    if (nearest == column.end()) {
        return general_utils::GridType::UNKNOWN;
    }
    if (sameIndex(nearest->index, index)) {
        return stateOf(nearest->type);
    }
    return nearest->type == BoundaryType::INTERIOR_FREE
               ? general_utils::GridType::KNOWN_FREE
               : general_utils::GridType::UNKNOWN;
}

general_utils::GridType BoundaryMap::getGridType(const rog_map::Vec3f &position) const {
    return getGridType(positionToIndex(position));
}

BoundaryMap::BoundaryType BoundaryMap::getBoundaryType(
    const rog_map::Vec3i &index) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    const auto column_it = columns_.find(columnKey(index));
    if (column_it == columns_.end()) {
        return BoundaryType::NONE;
    }
    const Column &column = column_it->second;
    const auto found = std::lower_bound(
        column.begin(), column.end(), index.z(),
        [](const BoundaryVoxel &voxel, const int z) {
            return voxel.index.z() < z;
        });
    return found != column.end() && sameIndex(found->index, index)
               ? found->type
               : BoundaryType::NONE;
}

bool BoundaryMap::isKnownFree(const rog_map::Vec3i &index) const {
    return getGridType(index) == general_utils::GridType::KNOWN_FREE;
}

bool BoundaryMap::isOccupied(const rog_map::Vec3i &index) const {
    return getGridType(index) == general_utils::GridType::OCCUPIED;
}

bool BoundaryMap::isFrontier(const rog_map::Vec3i &index) const {
    return getBoundaryType(index) == BoundaryType::EXTERIOR_UNKNOWN;
}

rog_map::Vec3i BoundaryMap::positionToIndex(const rog_map::Vec3f &position) const {
    return (position.array() * resolution_inv_).floor().cast<int>();
}

rog_map::Vec3f BoundaryMap::indexToPosition(const rog_map::Vec3i &index) const {
    return (index.cast<double>() + rog_map::Vec3f::Constant(0.5)) * resolution_;
}

std::vector<BoundaryMap::BoundaryVoxel> BoundaryMap::boundaryVoxels(
    const BoundaryType type, const std::size_t max_count) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    std::vector<BoundaryVoxel> output;
    for (const auto &entry : columns_) {
        for (const BoundaryVoxel &voxel : entry.second) {
            if (voxel.type != type) {
                continue;
            }
            output.push_back(voxel);
            if (max_count > 0 && output.size() >= max_count) {
                return output;
            }
        }
    }
    return output;
}

rog_map::vec_Vec3f BoundaryMap::frontierPositions(const std::size_t max_count) const {
    const std::vector<BoundaryVoxel> frontiers =
        boundaryVoxels(BoundaryType::EXTERIOR_UNKNOWN, max_count);
    rog_map::vec_Vec3f positions;
    positions.reserve(frontiers.size());
    for (const BoundaryVoxel &frontier : frontiers) {
        positions.push_back(indexToPosition(frontier.index));
    }
    return positions;
}

BoundaryMap::Stats BoundaryMap::stats() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    Stats output;
    output.column_count = columns_.size();
    output.update_count = update_count_;
    for (const auto &entry : columns_) {
        output.boundary_count += entry.second.size();
        for (const BoundaryVoxel &voxel : entry.second) {
            if (voxel.type == BoundaryType::INTERIOR_FREE) {
                ++output.interior_boundary_count;
            } else if (voxel.type == BoundaryType::EXTERIOR_UNKNOWN) {
                ++output.frontier_count;
            } else if (voxel.type == BoundaryType::OCCUPIED) {
                ++output.occupied_count;
            }
        }
    }
    return output;
}

} // namespace general_planner
