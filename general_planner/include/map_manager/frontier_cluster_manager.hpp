#pragma once

#include <memory>

#include <map_manager/map_manager.hpp>

namespace general_planner
{
enum class FrontierClusterStatus
{
    NEW,
    ACTIVE,
    STALE
};

struct FrontierCluster
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int id{-1};
    FrontierClusterStatus status{FrontierClusterStatus::NEW};
    rog_map::vec_E<FrontierVoxel> cells;
    super_utils::Vec3f center{super_utils::Vec3f::Zero()};
    super_utils::Vec3f unknown_direction{super_utils::Vec3f::UnitX()};
    super_utils::Vec3f bbox_min{super_utils::Vec3f::Zero()};
    super_utils::Vec3f bbox_max{super_utils::Vec3f::Zero()};
    int size{0};
    int age{0};
    int missed_count{0};
    int first_seen_seq{0};
    int last_seen_seq{0};

    bool observedThisFrame() const
    {
        return missed_count == 0;
    }
};

class FrontierClusterManager
{
public:
    using Ptr = std::shared_ptr<FrontierClusterManager>;

    struct Config
    {
        double cluster_radius{0.8};
        int min_cluster_size{5};
        double lifecycle_match_distance{1.2};
        int lifecycle_min_observations{1};
        int lifecycle_max_missing_frames{3};
    };

    FrontierClusterManager();

    explicit FrontierClusterManager(Config cfg);

    void reset();

    void update(const rog_map::vec_E<FrontierVoxel> &frontier_voxels,
                const MapManager &map_manager,
                rog_map::vec_E<FrontierCluster> &active_clusters);

    const rog_map::vec_E<FrontierCluster> &trackedClusters() const;

private:
    void clusterObservedFrontiers(const rog_map::vec_E<FrontierVoxel> &frontier_voxels,
                                  const MapManager &map_manager,
                                  rog_map::vec_E<FrontierCluster> &observed_clusters) const;

    void finalizeClusterGeometry(FrontierCluster &cluster,
                                 const MapManager &map_manager) const;

    Config cfg_;
    rog_map::vec_E<FrontierCluster> clusters_;
    int next_id_{1};
    int sequence_{0};
};
} // namespace general_planner
