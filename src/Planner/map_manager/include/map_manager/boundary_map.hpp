#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include <rog_map/rog_map_core/common_lib.hpp>
#include <general_utils/type_utils.hpp>

namespace general_planner {

/**
 * Persistent global occupancy represented only by boundary voxels.
 *
 * The projection axis is z. Each occupied (x,y) hash cell owns a z-sorted
 * vector of boundary voxels. Free volume is implicit: a non-boundary query is
 * free exactly when the first boundary voxel in +z is INTERIOR_FREE; otherwise
 * it is unknown. Occupied state is returned only for an exact OCCUPIED match.
 * This follows the BDM 2D hash-grid plus sorted-column query construction and
 * never stores the dense free volume.
 */
class BoundaryMap {
public:
    using Ptr = std::shared_ptr<BoundaryMap>;

    enum class BoundaryType : std::uint8_t {
        NONE = 0,
        INTERIOR_FREE = 1,
        EXTERIOR_UNKNOWN = 2,
        OCCUPIED = 3
    };

    struct BoundaryVoxel {
        rog_map::Vec3i index{rog_map::Vec3i::Zero()};
        BoundaryType type{BoundaryType::NONE};
    };

    struct Stats {
        std::size_t column_count{0};
        std::size_t boundary_count{0};
        std::size_t interior_boundary_count{0};
        std::size_t frontier_count{0};
        std::size_t occupied_count{0};
        std::size_t update_count{0};
    };

    explicit BoundaryMap(double resolution);

    double resolution() const { return resolution_; }

    void clear();
    void apply(const BoundaryVoxel &update);
    void apply(const std::vector<BoundaryVoxel> &updates);

    general_utils::GridType getGridType(const rog_map::Vec3i &index) const;
    general_utils::GridType getGridType(const rog_map::Vec3f &position) const;
    BoundaryType getBoundaryType(const rog_map::Vec3i &index) const;

    bool isKnownFree(const rog_map::Vec3i &index) const;
    bool isOccupied(const rog_map::Vec3i &index) const;
    bool isFrontier(const rog_map::Vec3i &index) const;

    rog_map::Vec3i positionToIndex(const rog_map::Vec3f &position) const;
    rog_map::Vec3f indexToPosition(const rog_map::Vec3i &index) const;

    std::vector<BoundaryVoxel> boundaryVoxels(BoundaryType type,
                                               std::size_t max_count = 0) const;
    rog_map::vec_Vec3f frontierPositions(std::size_t max_count = 0) const;
    Stats stats() const;

private:
    struct ColumnKey {
        int x{0};
        int y{0};

        bool operator==(const ColumnKey &other) const {
            return x == other.x && y == other.y;
        }
    };

    struct ColumnKeyHash {
        std::size_t operator()(const ColumnKey &key) const;
    };

    using Column = std::vector<BoundaryVoxel>;

    static ColumnKey columnKey(const rog_map::Vec3i &index);
    static bool sameIndex(const rog_map::Vec3i &lhs, const rog_map::Vec3i &rhs);
    static general_utils::GridType stateOf(BoundaryType type);

    double resolution_{0.0};
    double resolution_inv_{0.0};
    mutable std::shared_mutex mutex_;
    std::unordered_map<ColumnKey, Column, ColumnKeyHash> columns_;
    std::size_t update_count_{0};
};

} // namespace general_planner
