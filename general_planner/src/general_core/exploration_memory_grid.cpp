#include <general_core/exploration_memory_grid.hpp>

#include <algorithm>
#include <cmath>

namespace general_planner {
namespace {
std::size_t mixHash(std::size_t seed, const int value) {
    const std::size_t h = std::hash<int>{}(value);
    return seed ^ (h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
}
}  // namespace

std::size_t ExplorationMemoryGrid::GridKeyHasher::operator()(const GridKey &key) const {
    std::size_t seed = 0;
    seed = mixHash(seed, key.x);
    seed = mixHash(seed, key.y);
    seed = mixHash(seed, key.z);
    return seed;
}

ExplorationMemoryGrid::ExplorationMemoryGrid()
        : ExplorationMemoryGrid(Config{}) {
}

ExplorationMemoryGrid::ExplorationMemoryGrid(const Config &cfg)
        : cfg_(cfg) {
    cfg_.resolution = std::max(0.05, cfg_.resolution);
}

void ExplorationMemoryGrid::reset() {
    cells_.clear();
    known_free_count_ = 0;
    occupied_count_ = 0;
    explicit_unknown_count_ = 0;
    unknown_query_count_ = 0;
}

ExplorationMemoryGrid::GridKey ExplorationMemoryGrid::makeKey(const super_utils::Vec3i &index) {
    return GridKey{index.x(), index.y(), index.z()};
}

super_utils::Vec3i ExplorationMemoryGrid::positionToIndex(const super_utils::Vec3f &pos) const {
    return super_utils::Vec3i(static_cast<int>(std::floor(pos.x() / cfg_.resolution)),
                              static_cast<int>(std::floor(pos.y() / cfg_.resolution)),
                              static_cast<int>(std::floor(pos.z() / cfg_.resolution)));
}

super_utils::Vec3f ExplorationMemoryGrid::indexToPosition(const super_utils::Vec3i &index) const {
    return (index.cast<double>() + super_utils::Vec3f::Constant(0.5)) * cfg_.resolution;
}

double ExplorationMemoryGrid::resolution() const {
    return cfg_.resolution;
}

bool ExplorationMemoryGrid::insideExplorationBounds(const super_utils::Vec3f &pos) const {
    return pos.allFinite() &&
           (pos.array() >= cfg_.exploration_min.array()).all() &&
           (pos.array() <= cfg_.exploration_max.array()).all();
}

ExplorationVoxelState ExplorationMemoryGrid::mapGridTypeToState(const rog_map::GridType grid_type,
                                                                const rog_map::GridType inf_grid_type) const {
    if (grid_type == rog_map::GridType::OUT_OF_MAP ||
        inf_grid_type == rog_map::GridType::OUT_OF_MAP) {
        return ExplorationVoxelState::OUT_OF_BOUNDS;
    }
    if (grid_type == rog_map::GridType::OCCUPIED ||
        inf_grid_type == rog_map::GridType::OCCUPIED) {
        return ExplorationVoxelState::OCCUPIED;
    }
    if (grid_type == rog_map::GridType::KNOWN_FREE) {
        return ExplorationVoxelState::KNOWN_FREE;
    }
    return ExplorationVoxelState::UNKNOWN;
}

void ExplorationMemoryGrid::updateFromMap(const MapManager::Ptr &map_manager,
                                          const super_utils::Vec3f &robot_pos,
                                          const double update_radius,
                                          const double stamp) {
    if (!cfg_.use_global_memory || map_manager == nullptr || !map_manager->ready() ||
        !robot_pos.allFinite() || update_radius <= 0.0) {
        return;
    }

    super_utils::Vec3f box_min = robot_pos - super_utils::Vec3f::Constant(update_radius);
    super_utils::Vec3f box_max = robot_pos + super_utils::Vec3f::Constant(update_radius);
    box_min = box_min.cwiseMax(cfg_.exploration_min);
    box_max = box_max.cwiseMin(cfg_.exploration_max);
    if ((box_max - box_min).minCoeff() <= 0.0) {
        return;
    }

    const super_utils::Vec3i min_id = positionToIndex(box_min);
    const super_utils::Vec3i max_id = positionToIndex(box_max);
    const double radius_sq = update_radius * update_radius;

    for (int ix = min_id.x(); ix <= max_id.x(); ++ix) {
        for (int iy = min_id.y(); iy <= max_id.y(); ++iy) {
            for (int iz = min_id.z(); iz <= max_id.z(); ++iz) {
                const super_utils::Vec3i id(ix, iy, iz);
                const super_utils::Vec3f pos = indexToPosition(id);
                if ((pos - robot_pos).squaredNorm() > radius_sq ||
                    !insideExplorationBounds(pos) ||
                    !map_manager->insideLocalMap(pos)) {
                    continue;
                }

                const ExplorationVoxelState observed =
                        mapGridTypeToState(map_manager->getGridType(pos),
                                           map_manager->getInfGridType(pos));
                if (observed == ExplorationVoxelState::OUT_OF_BOUNDS) {
                    continue;
                }

                const GridKey key = makeKey(id);
                auto it = cells_.find(key);
                const ExplorationVoxelState old_state =
                        it == cells_.end() ? ExplorationVoxelState::UNKNOWN : it->second.state;
                if (observed == ExplorationVoxelState::UNKNOWN &&
                    (old_state == ExplorationVoxelState::KNOWN_FREE ||
                     old_state == ExplorationVoxelState::OCCUPIED)) {
                    it->second.stamp = stamp;
                    continue;
                }
                if (old_state == observed) {
                    if (it != cells_.end()) {
                        it->second.stamp = stamp;
                    } else {
                        cells_[key] = CellRecord{observed, stamp};
                        ++explicit_unknown_count_;
                    }
                    continue;
                }
                if (old_state == ExplorationVoxelState::KNOWN_FREE) {
                    --known_free_count_;
                } else if (old_state == ExplorationVoxelState::OCCUPIED) {
                    --occupied_count_;
                } else if (it != cells_.end() &&
                           old_state == ExplorationVoxelState::UNKNOWN) {
                    --explicit_unknown_count_;
                }
                if (observed == ExplorationVoxelState::KNOWN_FREE) {
                    ++known_free_count_;
                } else if (observed == ExplorationVoxelState::OCCUPIED) {
                    ++occupied_count_;
                } else if (observed == ExplorationVoxelState::UNKNOWN) {
                    ++explicit_unknown_count_;
                }
                cells_[key] = CellRecord{observed, stamp};
            }
        }
    }
}

ExplorationVoxelState ExplorationMemoryGrid::getState(const super_utils::Vec3i &index) const {
    const super_utils::Vec3f pos = indexToPosition(index);
    if (!insideExplorationBounds(pos)) {
        return ExplorationVoxelState::OUT_OF_BOUNDS;
    }
    const auto it = cells_.find(makeKey(index));
    if (it == cells_.end()) {
        ++unknown_query_count_;
        return ExplorationVoxelState::UNKNOWN;
    }
    return it->second.state;
}

ExplorationVoxelState ExplorationMemoryGrid::getState(const super_utils::Vec3f &pos) const {
    if (!insideExplorationBounds(pos)) {
        return ExplorationVoxelState::OUT_OF_BOUNDS;
    }
    return getState(positionToIndex(pos));
}

bool ExplorationMemoryGrid::isKnownFree(const super_utils::Vec3f &pos) const {
    return getState(pos) == ExplorationVoxelState::KNOWN_FREE;
}

bool ExplorationMemoryGrid::isUnknown(const super_utils::Vec3f &pos) const {
    return getState(pos) == ExplorationVoxelState::UNKNOWN;
}

bool ExplorationMemoryGrid::isOccupied(const super_utils::Vec3f &pos) const {
    return getState(pos) == ExplorationVoxelState::OCCUPIED;
}

bool ExplorationMemoryGrid::neighborUnknown(const super_utils::Vec3i &index) const {
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const super_utils::Vec3i nid = index + super_utils::Vec3i(dx, dy, dz);
                if (getState(nid) == ExplorationVoxelState::UNKNOWN) {
                    return true;
                }
            }
        }
    }
    return false;
}

void ExplorationMemoryGrid::getCandidateFrontierCells(super_utils::vec_E<CompleteFrontierCell> &out) const {
    out.clear();
    out.reserve(cells_.size() / 8U + 1U);
    for (const auto &kv : cells_) {
        if (kv.second.state != ExplorationVoxelState::KNOWN_FREE) {
            continue;
        }
        const super_utils::Vec3i index(kv.first.x, kv.first.y, kv.first.z);
        if (!neighborUnknown(index)) {
            continue;
        }
        CompleteFrontierCell cell;
        cell.index = index;
        cell.position = indexToPosition(index);
        out.push_back(cell);
    }
    std::sort(out.begin(), out.end(), [](const CompleteFrontierCell &lhs,
                                         const CompleteFrontierCell &rhs) {
        if (lhs.index.x() != rhs.index.x()) return lhs.index.x() < rhs.index.x();
        if (lhs.index.y() != rhs.index.y()) return lhs.index.y() < rhs.index.y();
        return lhs.index.z() < rhs.index.z();
    });
}

void ExplorationMemoryGrid::getKnownFreeCells(super_utils::vec_E<CompleteFrontierCell> &out) const {
    out.clear();
    out.reserve(static_cast<std::size_t>(std::max(0, known_free_count_)));
    for (const auto &kv : cells_) {
        if (kv.second.state != ExplorationVoxelState::KNOWN_FREE) {
            continue;
        }
        CompleteFrontierCell cell;
        cell.index = super_utils::Vec3i(kv.first.x, kv.first.y, kv.first.z);
        cell.position = indexToPosition(cell.index);
        out.push_back(cell);
    }
    std::sort(out.begin(), out.end(), [](const CompleteFrontierCell &lhs,
                                         const CompleteFrontierCell &rhs) {
        if (lhs.index.x() != rhs.index.x()) return lhs.index.x() < rhs.index.x();
        if (lhs.index.y() != rhs.index.y()) return lhs.index.y() < rhs.index.y();
        return lhs.index.z() < rhs.index.z();
    });
}

int ExplorationMemoryGrid::knownFreeCount() const {
    return known_free_count_;
}

int ExplorationMemoryGrid::occupiedCount() const {
    return occupied_count_;
}

int ExplorationMemoryGrid::explicitUnknownCount() const {
    return explicit_unknown_count_;
}

int ExplorationMemoryGrid::unknownQueryCount() const {
    return unknown_query_count_;
}

}  // namespace general_planner
