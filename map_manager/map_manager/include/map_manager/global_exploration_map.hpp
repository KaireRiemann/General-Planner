#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Eigen>
#include <map_manager/exploration_types.hpp>
#include <rog_map/rog_map.h>
#include <super_utils/type_utils.hpp>

namespace general_planner {

enum class CloudFrame {
    LIDAR,
    BODY,
    WORLD
};

struct VoxelKeyHash {
    std::size_t operator()(const Eigen::Vector3i &key) const;
};

struct VoxelKeyEqual {
    bool operator()(const Eigen::Vector3i &lhs, const Eigen::Vector3i &rhs) const;
};

enum class GlobalVoxelState : uint8_t {
    UNKNOWN = 0,
    OBSERVED_FREE = 1,
    OBSERVED_OCCUPIED = 2
};

struct GlobalVoxel {
    GlobalVoxelState state{GlobalVoxelState::UNKNOWN};
    uint16_t hit_count{0};
    uint16_t miss_count{0};
    double first_observed_time{-1.0};
    double last_observed_time{-1.0};
    bool frontier{false};
    int region_id{-1};
    int floor_id{-1};
};

struct GlobalExplorationMapConfig {
    bool enable{true};
    double resolution{0.20};
    double raycast_step{0.10};
    double max_range{8.0};
    double min_range{0.20};

    Eigen::Vector3d bbox_min{-50.0, -50.0, -2.0};
    Eigen::Vector3d bbox_max{50.0, 50.0, 10.0};

    int occupied_hit_threshold{2};
    int free_miss_threshold{1};

    bool maintain_frontiers{true};
    bool use_26_neighbor_frontier{true};

    double update_min_interval{0.2};
    int cloud_downsample_step{1};
    int max_points_per_update{8000};
};

class GlobalExplorationMap {
public:
    using Ptr = std::shared_ptr<GlobalExplorationMap>;

    explicit GlobalExplorationMap(const GlobalExplorationMapConfig &cfg);

    void updateObservation(const rog_map::PointCloud &cloud,
                           const super_utils::Pose &pose,
                           CloudFrame frame,
                           double stamp);

    GlobalVoxelState getVoxelState(const Eigen::Vector3d &p) const;

    bool isObserved(const Eigen::Vector3d &p) const;
    bool isUnknown(const Eigen::Vector3d &p) const;
    bool isFrontier(const Eigen::Vector3d &p) const;

    void getFrontierPoints(std::vector<Eigen::Vector3d> &points) const;
    void getObservedFreePoints(std::vector<Eigen::Vector3d> &points) const;
    void getOccupiedPoints(std::vector<Eigen::Vector3d> &points) const;

    void getFrontierClusters(double cluster_radius,
                             int min_cluster_size,
                             std::vector<FrontierCluster> &clusters) const;

    double exploredVolume() const;
    double occupiedVolume() const;
    int frontierCount() const;
    int observedVoxelCount() const;
    double resolution() const;

    void reset();

private:
    using VoxelKey = Eigen::Vector3i;

    VoxelKey posToKey(const Eigen::Vector3d &p) const;
    Eigen::Vector3d keyToPos(const VoxelKey &k) const;

    bool insideBBox(const Eigen::Vector3d &p) const;

    Eigen::Vector3d transformPointToWorld(const rog_map::PclPoint &p,
                                          const super_utils::Pose &pose,
                                          CloudFrame frame) const;

    void raycastAndUpdate(const Eigen::Vector3d &origin,
                          const Eigen::Vector3d &hit,
                          double stamp);

    void markFree(const VoxelKey &key, double stamp);
    void markOccupied(const VoxelKey &key, double stamp);

    void updateFrontierAround(const VoxelKey &key);
    bool computeIsFrontier(const VoxelKey &key) const;

private:
    GlobalExplorationMapConfig cfg_;
    mutable std::mutex mutex_;

    std::unordered_map<VoxelKey, GlobalVoxel, VoxelKeyHash, VoxelKeyEqual> voxels_;
    std::unordered_set<VoxelKey, VoxelKeyHash, VoxelKeyEqual> frontier_keys_;
};

}  // namespace general_planner
