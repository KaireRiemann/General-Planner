#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <general_core/exploration_types_complete.hpp>
#include <map_manager/map_manager.hpp>

namespace general_planner {

class ExplorationMemoryGrid {
public:
    using Ptr = std::shared_ptr<ExplorationMemoryGrid>;

    struct Config {
        double resolution{0.2};
        double memory_timeout{-1.0};
        bool use_global_memory{true};
        bool unknown_outside_bounds{false};
        super_utils::Vec3f exploration_min{super_utils::Vec3f(-50.0, -50.0, -2.0)};
        super_utils::Vec3f exploration_max{super_utils::Vec3f(50.0, 50.0, 5.0)};
    };

    ExplorationMemoryGrid();
    explicit ExplorationMemoryGrid(const Config &cfg);

    void reset();

    void updateFromMap(const MapManager::Ptr &map_manager,
                       const super_utils::Vec3f &robot_pos,
                       double update_radius,
                       double stamp);

    ExplorationVoxelState getState(const super_utils::Vec3i &index) const;
    ExplorationVoxelState getState(const super_utils::Vec3f &pos) const;

    bool insideExplorationBounds(const super_utils::Vec3f &pos) const;
    bool isKnownFree(const super_utils::Vec3f &pos) const;
    bool isUnknown(const super_utils::Vec3f &pos) const;
    bool isOccupied(const super_utils::Vec3f &pos) const;

    void getCandidateFrontierCells(super_utils::vec_E<CompleteFrontierCell> &out) const;
    void getKnownFreeCells(super_utils::vec_E<CompleteFrontierCell> &out) const;

    int knownFreeCount() const;
    int occupiedCount() const;
    int explicitUnknownCount() const;
    int unknownQueryCount() const;

    super_utils::Vec3i positionToIndex(const super_utils::Vec3f &pos) const;
    super_utils::Vec3f indexToPosition(const super_utils::Vec3i &index) const;
    double resolution() const;

private:
    struct GridKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const GridKey &other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct GridKeyHasher {
        std::size_t operator()(const GridKey &key) const;
    };

    struct CellRecord {
        ExplorationVoxelState state{ExplorationVoxelState::UNKNOWN};
        double stamp{0.0};
    };

    static GridKey makeKey(const super_utils::Vec3i &index);
    bool neighborUnknown(const super_utils::Vec3i &index) const;
    ExplorationVoxelState mapGridTypeToState(rog_map::GridType grid_type,
                                             rog_map::GridType inf_grid_type) const;

    Config cfg_;
    std::unordered_map<GridKey, CellRecord, GridKeyHasher> cells_;
    int known_free_count_{0};
    int occupied_count_{0};
    int explicit_unknown_count_{0};
    mutable int unknown_query_count_{0};
};

}  // namespace general_planner
