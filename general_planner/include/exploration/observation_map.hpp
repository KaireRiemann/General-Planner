#pragma once

#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include <map_manager/map_manager.hpp>

#include "exploration/exploration_types.hpp"

namespace general_planner {
namespace exploration {

struct SurfaceVoxel {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SurfaceVoxelState state{SurfaceVoxelState::POORLY_OBSERVED};
    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f normal{super_utils::Vec3f::UnitX()};
    double quality{0.0};
    double last_seen_time{0.0};
    double first_seen_time{0.0};
    double generated_travel_distance{0.0};
    super_utils::Vec3f generated_position{super_utils::Vec3f::Zero()};
};

class ObservationMap {
public:
    using Ptr = std::shared_ptr<ObservationMap>;

    struct Config {
        bool enable{true};
        double resolution{0.25};
        double min_observation_distance{0.2};
        double well_observed_distance{4.0};
        double max_observation_distance{12.0};
        int cloud_downsample_step{1};
        int max_points_per_update{12000};
        double normal_ema_alpha{0.35};
        double frontier_cluster_radius{0.65};
        double frontier_normal_similarity{0.35};
        int min_frontier_cluster_size{8};
        double bbox_min_x{-50.0};
        double bbox_min_y{-50.0};
        double bbox_min_z{-2.0};
        double bbox_max_x{50.0};
        double bbox_max_y{50.0};
        double bbox_max_z{10.0};
    };

    explicit ObservationMap(Config cfg);

    void update(const rog_map::PointCloud &cloud,
                const super_utils::Pose &pose,
                CloudFrame frame,
                const super_utils::Vec3f &sensor_position,
                double travel_distance,
                double stamp);

    void getFrontierClusters(std::vector<SurfaceFrontierCluster> &clusters) const;

    bool getVoxel(const super_utils::Vec3f &position, SurfaceVoxel &voxel) const;

    double nearestSurfaceDistance(const super_utils::Vec3f &position,
                                  double max_search_radius) const;

    bool lineOfSightFree(const super_utils::Vec3f &start,
                         const super_utils::Vec3f &end,
                         double safe_distance,
                         double step) const;

    int observedVoxelCount() const;
    int frontierVoxelCount() const;
    double resolution() const { return cfg_.resolution; }

    void reset();

private:
    using VoxelKey = Eigen::Vector3i;

    VoxelKey posToKey(const super_utils::Vec3f &position) const;
    super_utils::Vec3f keyToPos(const VoxelKey &key) const;
    super_utils::Vec3f transformPointToWorld(const rog_map::PclPoint &point,
                                             const super_utils::Pose &pose,
                                             CloudFrame frame) const;

    bool insideBounds(const super_utils::Vec3f &position) const;
    bool computeIsFrontier(const VoxelKey &key) const;
    void updateFrontierAround(const VoxelKey &key);

private:
    Config cfg_;
    mutable std::mutex mutex_;
    std::unordered_map<VoxelKey, SurfaceVoxel, VoxelKeyHash, VoxelKeyEqual> voxels_;
    std::unordered_set<VoxelKey, VoxelKeyHash, VoxelKeyEqual> frontier_keys_;
};

}  // namespace exploration
}  // namespace general_planner

