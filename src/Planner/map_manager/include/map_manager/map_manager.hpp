#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <map_manager/boundary_map.hpp>
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

    MapManager() = default;

    explicit MapManager(const rog_map::ROGMapROS::Ptr &map)
    {
        setMap(map);
    }

    void setMap(const rog_map::ROGMapROS::Ptr &map)
    {
        if (map_) {
            map_->setStateChangeCallback({});
        }
        map_ = map;
        boundary_map_.reset();
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
            map_->setStateChangeCallback([weak_map, weak_boundary]() {
                const auto map = weak_map.lock();
                const auto boundary = weak_boundary.lock();
                if (map && boundary) {
                    syncBoundaryMapImpl(map, boundary);
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

    void updateMap(const rog_map::PointCloud &cloud, const general_utils::Pose &pose) const
    {
        map_->updateMap(cloud, pose);
    }

    /** Synchronize pending ROG discrete transitions into the sparse global map. */
    void syncBoundaryMap() const
    {
        syncBoundaryMapImpl(map_, boundary_map_);
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
    static void syncBoundaryMapImpl(const rog_map::ROGMapROS::Ptr &map,
                                    const BoundaryMap::Ptr &boundary_map);

    rog_map::ROGMapROS::Ptr map_;
    BoundaryMap::Ptr boundary_map_;
};
} // namespace general_planner
