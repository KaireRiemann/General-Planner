#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <unordered_set>

#include <rog_map/rog_map.h>
#include <rog_map_ros/rog_map_ros1.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <super_utils/type_utils.hpp>

namespace general_planner
{
struct FrontierVoxel
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    super_utils::Vec3f position{super_utils::Vec3f::Zero()};
    super_utils::Vec3i index{super_utils::Vec3i::Zero()};
};

struct FrontierSearchStats
{
    int searched_cells{0};
    int known_free_cells{0};
    int unknown_cells{0};
    int occupied_cells{0};
    int frontier_cells{0};
    bool rog_frontier_extractor_enabled{false};
    bool used_rog_frontier_extractor{false};
};

class MapManager
{
public:
    using Ptr = std::shared_ptr<MapManager>;

    MapManager() = default;

    explicit MapManager(const rog_map::ROGMapROS::Ptr &map)
        : map_(map)
    {
    }

    void setMap(const rog_map::ROGMapROS::Ptr &map)
    {
        map_ = map;
    }

    bool ready() const
    {
        return map_ != nullptr;
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

    void updateMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) const
    {
        map_->updateMap(cloud, pose);
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
        if (map_ == nullptr) {
            return false;
        }
        if (use_inf_map && use_unk_as_occ && !map_->getMapConfig().unk_inflation_en) {
            return map_->isLineFree(start_pt, end_pt, true, false) &&
                   map_->isLineFree(start_pt, end_pt, false, true);
        }
        return map_->isLineFree(start_pt, end_pt, use_inf_map, use_unk_as_occ);
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

    bool frontierExtractorEnabled() const
    {
        return map_ != nullptr && map_->getMapConfig().frontier_extraction_en;
    }

    bool extractFrontierVoxels(const rog_map::Vec3f &center,
                               const double radius,
                               const double sample_resolution,
                               rog_map::vec_E<FrontierVoxel> &frontier_voxels,
                               FrontierSearchStats *stats = nullptr) const
    {
        frontier_voxels.clear();
        if (stats != nullptr) {
            *stats = FrontierSearchStats{};
        }
        if (map_ == nullptr || !center.allFinite() || radius <= 0.0) {
            return false;
        }

        rog_map::Vec3f box_min = center - rog_map::Vec3f::Constant(radius);
        rog_map::Vec3f box_max = center + rog_map::Vec3f::Constant(radius);
        map_->boundBoxByLocalMap(box_min, box_max);
        if ((box_max - box_min).minCoeff() <= 0.0) {
            return false;
        }

        if (stats != nullptr) {
            fillObservationStats(center, radius, sample_resolution, *stats);
            stats->rog_frontier_extractor_enabled = frontierExtractorEnabled();
        }

        const double radius_sq = radius * radius;
        std::unordered_set<GridKey, GridKeyHasher> inserted;
        if (frontierExtractorEnabled()) {
            rog_map::vec_E<rog_map::Vec3f> raw_frontiers;
            map_->boxSearch(box_min, box_max, rog_map::GridType::FRONTIER, raw_frontiers);
            inserted.reserve(raw_frontiers.size());
            for (const auto &pos : raw_frontiers) {
                if (!pos.allFinite() ||
                    (pos - center).squaredNorm() > radius_sq ||
                    !map_->insideLocalMap(pos) ||
                    !isUnknownLike(map_->getGridType(pos))) {
                    continue;
                }
                FrontierVoxel voxel;
                voxel.position = pos;
                map_->probMapPosToGlobalIndex(pos, voxel.index);
                if (!inserted.insert(makeKey(voxel.index)).second) {
                    continue;
                }
                frontier_voxels.push_back(voxel);
            }
            if (stats != nullptr) {
                stats->used_rog_frontier_extractor = true;
                stats->frontier_cells = static_cast<int>(frontier_voxels.size());
            }
            return true;
        }

        manualExtractFrontierVoxels(center, radius, sample_resolution, frontier_voxels);
        if (stats != nullptr) {
            stats->frontier_cells = static_cast<int>(frontier_voxels.size());
        }
        return true;
    }

    bool isKnownFreeForViewpoint(const rog_map::Vec3f &pos,
                                 const double min_esdf_distance) const
    {
        if (map_ == nullptr || !pos.allFinite() || !map_->insideLocalMap(pos)) {
            return false;
        }
        if (map_->getGridType(pos) != rog_map::GridType::KNOWN_FREE) {
            return false;
        }
        const rog_map::GridType inf_type = map_->getInfGridType(pos);
        if (inf_type == rog_map::GridType::OCCUPIED ||
            inf_type == rog_map::GridType::OUT_OF_MAP) {
            return false;
        }
        if (hasESDF() && min_esdf_distance > 0.0) {
            double dist = 0.0;
            rog_map::Vec3f grad = rog_map::Vec3f::Zero();
            if (!evaluateESDF(pos, dist, grad) ||
                !std::isfinite(dist) ||
                dist < min_esdf_distance) {
                return false;
            }
        }
        return true;
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
    struct GridKey
    {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const GridKey &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct GridKeyHasher
    {
        std::size_t operator()(const GridKey &key) const
        {
            std::size_t seed = 0;
            const auto mix = [&seed](const int value) {
                const std::size_t h = std::hash<int>{}(value);
                seed ^= h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
            };
            mix(key.x);
            mix(key.y);
            mix(key.z);
            return seed;
        }
    };

    static GridKey makeKey(const rog_map::Vec3i &id)
    {
        return GridKey{id.x(), id.y(), id.z()};
    }

    static bool isUnknownLike(const rog_map::GridType type)
    {
        return type == rog_map::GridType::UNKNOWN ||
               type == rog_map::GridType::UNDEFINED ||
               type == rog_map::GridType::FRONTIER;
    }

    static bool isFreeLike(const rog_map::GridType type)
    {
        return type == rog_map::GridType::KNOWN_FREE;
    }

    void fillObservationStats(const rog_map::Vec3f &center,
                              const double radius,
                              const double sample_resolution,
                              FrontierSearchStats &stats) const
    {
        if (map_ == nullptr) {
            return;
        }
        const double map_res = std::max(1.0e-3, map_->getResolution());
        const double sample_res = std::max(map_res, sample_resolution);
        const int index_step = std::max(1, static_cast<int>(std::round(sample_res / map_res)));

        rog_map::Vec3f box_min = center - rog_map::Vec3f::Constant(radius);
        rog_map::Vec3f box_max = center + rog_map::Vec3f::Constant(radius);
        map_->boundBoxByLocalMap(box_min, box_max);
        if ((box_max - box_min).minCoeff() <= 0.0) {
            return;
        }

        rog_map::Vec3i min_id;
        rog_map::Vec3i max_id;
        map_->probMapPosToGlobalIndex(box_min, min_id);
        map_->probMapPosToGlobalIndex(box_max, max_id);
        for (int axis = 0; axis < 3; ++axis) {
            if (min_id(axis) > max_id(axis)) {
                std::swap(min_id(axis), max_id(axis));
            }
        }

        const double radius_sq = radius * radius;
        for (int ix = min_id.x(); ix <= max_id.x(); ix += index_step) {
            for (int iy = min_id.y(); iy <= max_id.y(); iy += index_step) {
                for (int iz = min_id.z(); iz <= max_id.z(); iz += index_step) {
                    const rog_map::Vec3i id(ix, iy, iz);
                    rog_map::Vec3f pos;
                    map_->probMapGlobalIndexToPos(id, pos);
                    if ((pos - center).squaredNorm() > radius_sq ||
                        !map_->insideLocalMap(pos)) {
                        continue;
                    }
                    ++stats.searched_cells;
                    const auto grid_type = map_->getGridType(pos);
                    if (isFreeLike(grid_type)) {
                        ++stats.known_free_cells;
                    } else if (isUnknownLike(grid_type)) {
                        ++stats.unknown_cells;
                    } else if (grid_type == rog_map::GridType::OCCUPIED) {
                        ++stats.occupied_cells;
                    }
                }
            }
        }
    }

    bool hasKnownFreeNeighbor(const rog_map::Vec3f &pos) const
    {
        const double map_res = std::max(1.0e-3, map_->getResolution());
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const rog_map::Vec3f neighbor_pos =
                            pos + map_res * rog_map::Vec3f(dx, dy, dz);
                    if (map_->insideLocalMap(neighbor_pos) &&
                        isFreeLike(map_->getGridType(neighbor_pos))) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    void manualExtractFrontierVoxels(const rog_map::Vec3f &center,
                                     const double radius,
                                     const double sample_resolution,
                                     rog_map::vec_E<FrontierVoxel> &frontier_voxels) const
    {
        const double map_res = std::max(1.0e-3, map_->getResolution());
        const double sample_res = std::max(map_res, sample_resolution);
        const int index_step = std::max(1, static_cast<int>(std::round(sample_res / map_res)));

        rog_map::Vec3f box_min = center - rog_map::Vec3f::Constant(radius);
        rog_map::Vec3f box_max = center + rog_map::Vec3f::Constant(radius);
        map_->boundBoxByLocalMap(box_min, box_max);
        if ((box_max - box_min).minCoeff() <= 0.0) {
            return;
        }

        rog_map::Vec3i min_id;
        rog_map::Vec3i max_id;
        map_->probMapPosToGlobalIndex(box_min, min_id);
        map_->probMapPosToGlobalIndex(box_max, max_id);
        for (int axis = 0; axis < 3; ++axis) {
            if (min_id(axis) > max_id(axis)) {
                std::swap(min_id(axis), max_id(axis));
            }
        }

        const double radius_sq = radius * radius;
        std::unordered_set<GridKey, GridKeyHasher> inserted;
        for (int ix = min_id.x(); ix <= max_id.x(); ix += index_step) {
            for (int iy = min_id.y(); iy <= max_id.y(); iy += index_step) {
                for (int iz = min_id.z(); iz <= max_id.z(); iz += index_step) {
                    FrontierVoxel voxel;
                    voxel.index = rog_map::Vec3i(ix, iy, iz);
                    map_->probMapGlobalIndexToPos(voxel.index, voxel.position);
                    if ((voxel.position - center).squaredNorm() > radius_sq ||
                        !map_->insideLocalMap(voxel.position) ||
                        !isUnknownLike(map_->getGridType(voxel.position)) ||
                        !hasKnownFreeNeighbor(voxel.position) ||
                        !inserted.insert(makeKey(voxel.index)).second) {
                        continue;
                    }
                    frontier_voxels.push_back(voxel);
                }
            }
        }
    }

    rog_map::ROGMapROS::Ptr map_;
};
} // namespace general_planner
