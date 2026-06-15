#include <map_manager/frontier_cluster_manager.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace general_planner
{
namespace
{
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

GridKey makeBucketKey(const super_utils::Vec3f &pos, const double bucket_size)
{
    return GridKey{
            static_cast<int>(std::floor(pos.x() / bucket_size)),
            static_cast<int>(std::floor(pos.y() / bucket_size)),
            static_cast<int>(std::floor(pos.z() / bucket_size))};
}

bool isFreeLike(const rog_map::GridType type)
{
    return type == rog_map::GridType::KNOWN_FREE;
}
} // namespace

FrontierClusterManager::FrontierClusterManager()
    : FrontierClusterManager(Config{})
{
}

FrontierClusterManager::FrontierClusterManager(Config cfg)
    : cfg_(cfg)
{
}

void FrontierClusterManager::reset()
{
    clusters_.clear();
    next_id_ = 1;
    sequence_ = 0;
}

void FrontierClusterManager::update(const rog_map::vec_E<FrontierVoxel> &frontier_voxels,
                                    const MapManager &map_manager,
                                    rog_map::vec_E<FrontierCluster> &active_clusters)
{
    active_clusters.clear();
    ++sequence_;

    rog_map::vec_E<FrontierCluster> observed_clusters;
    clusterObservedFrontiers(frontier_voxels, map_manager, observed_clusters);

    std::vector<int> observed_to_tracked(observed_clusters.size(), -1);
    std::vector<char> tracked_used(clusters_.size(), 0);
    const double match_distance =
            std::max({cfg_.lifecycle_match_distance,
                      cfg_.cluster_radius,
                      map_manager.getResolution()});
    const double match_distance_sq = match_distance * match_distance;

    for (std::size_t obs_idx = 0; obs_idx < observed_clusters.size(); ++obs_idx) {
        double best_sq = std::numeric_limits<double>::infinity();
        int best_tracked = -1;
        for (std::size_t tracked_idx = 0; tracked_idx < clusters_.size(); ++tracked_idx) {
            if (tracked_used[tracked_idx] != 0) {
                continue;
            }
            const FrontierCluster &tracked = clusters_[tracked_idx];
            if (tracked.missed_count > cfg_.lifecycle_max_missing_frames) {
                continue;
            }
            const double dist_sq =
                    (observed_clusters[obs_idx].center - tracked.center).squaredNorm();
            if (dist_sq < best_sq && dist_sq <= match_distance_sq) {
                best_sq = dist_sq;
                best_tracked = static_cast<int>(tracked_idx);
            }
        }
        if (best_tracked >= 0) {
            observed_to_tracked[obs_idx] = best_tracked;
            tracked_used[static_cast<std::size_t>(best_tracked)] = 1;
        }
    }

    rog_map::vec_E<FrontierCluster> updated_clusters;
    updated_clusters.reserve(observed_clusters.size() + clusters_.size());

    for (std::size_t obs_idx = 0; obs_idx < observed_clusters.size(); ++obs_idx) {
        FrontierCluster cluster = observed_clusters[obs_idx];
        const int tracked_idx = observed_to_tracked[obs_idx];
        if (tracked_idx >= 0) {
            const FrontierCluster &tracked = clusters_[static_cast<std::size_t>(tracked_idx)];
            cluster.id = tracked.id;
            cluster.first_seen_seq = tracked.first_seen_seq;
            cluster.age = tracked.age + 1;
        } else {
            cluster.id = next_id_++;
            cluster.first_seen_seq = sequence_;
            cluster.age = 1;
        }
        cluster.last_seen_seq = sequence_;
        cluster.missed_count = 0;
        cluster.status = cluster.age >= std::max(1, cfg_.lifecycle_min_observations)
                                 ? FrontierClusterStatus::ACTIVE
                                 : FrontierClusterStatus::NEW;
        updated_clusters.push_back(cluster);
    }

    for (std::size_t tracked_idx = 0; tracked_idx < clusters_.size(); ++tracked_idx) {
        if (tracked_used[tracked_idx] != 0) {
            continue;
        }
        FrontierCluster stale_cluster = clusters_[tracked_idx];
        ++stale_cluster.missed_count;
        if (stale_cluster.missed_count > std::max(0, cfg_.lifecycle_max_missing_frames)) {
            continue;
        }
        stale_cluster.status = FrontierClusterStatus::STALE;
        updated_clusters.push_back(stale_cluster);
    }

    clusters_ = std::move(updated_clusters);
    for (const FrontierCluster &cluster : clusters_) {
        if (cluster.status == FrontierClusterStatus::ACTIVE && cluster.observedThisFrame()) {
            active_clusters.push_back(cluster);
        }
    }
}

const rog_map::vec_E<FrontierCluster> &FrontierClusterManager::trackedClusters() const
{
    return clusters_;
}

void FrontierClusterManager::clusterObservedFrontiers(
        const rog_map::vec_E<FrontierVoxel> &frontier_voxels,
        const MapManager &map_manager,
        rog_map::vec_E<FrontierCluster> &observed_clusters) const
{
    observed_clusters.clear();
    if (frontier_voxels.empty()) {
        return;
    }

    const double radius = std::max(map_manager.getResolution(), cfg_.cluster_radius);
    const double radius_sq = radius * radius;
    std::unordered_map<GridKey, std::vector<int>, GridKeyHasher> buckets;
    buckets.reserve(frontier_voxels.size());
    for (int i = 0; i < static_cast<int>(frontier_voxels.size()); ++i) {
        buckets[makeBucketKey(frontier_voxels[static_cast<std::size_t>(i)].position, radius)].push_back(i);
    }

    std::vector<char> visited(frontier_voxels.size(), 0);
    std::queue<int> q;
    for (int seed = 0; seed < static_cast<int>(frontier_voxels.size()); ++seed) {
        if (visited[static_cast<std::size_t>(seed)] != 0) {
            continue;
        }

        FrontierCluster cluster;
        visited[static_cast<std::size_t>(seed)] = 1;
        q.push(seed);
        while (!q.empty()) {
            const int current = q.front();
            q.pop();
            const FrontierVoxel &current_voxel = frontier_voxels[static_cast<std::size_t>(current)];
            cluster.cells.push_back(current_voxel);

            const GridKey bucket = makeBucketKey(current_voxel.position, radius);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const GridKey neighbor_bucket{bucket.x + dx, bucket.y + dy, bucket.z + dz};
                        const auto it = buckets.find(neighbor_bucket);
                        if (it == buckets.end()) {
                            continue;
                        }
                        for (const int neighbor_id : it->second) {
                            if (visited[static_cast<std::size_t>(neighbor_id)] != 0) {
                                continue;
                            }
                            const FrontierVoxel &neighbor =
                                    frontier_voxels[static_cast<std::size_t>(neighbor_id)];
                            if ((neighbor.position - current_voxel.position).squaredNorm() > radius_sq) {
                                continue;
                            }
                            visited[static_cast<std::size_t>(neighbor_id)] = 1;
                            q.push(neighbor_id);
                        }
                    }
                }
            }
        }

        if (static_cast<int>(cluster.cells.size()) < std::max(1, cfg_.min_cluster_size)) {
            continue;
        }
        finalizeClusterGeometry(cluster, map_manager);
        observed_clusters.push_back(cluster);
    }
}

void FrontierClusterManager::finalizeClusterGeometry(FrontierCluster &cluster,
                                                     const MapManager &map_manager) const
{
    cluster.size = static_cast<int>(cluster.cells.size());
    cluster.center.setZero();
    cluster.bbox_min = cluster.cells.front().position;
    cluster.bbox_max = cluster.cells.front().position;
    super_utils::Vec3f unknown_dir = super_utils::Vec3f::Zero();

    const double map_res = std::max(1.0e-3, map_manager.getResolution());
    for (const FrontierVoxel &voxel : cluster.cells) {
        cluster.center += voxel.position;
        cluster.bbox_min = cluster.bbox_min.cwiseMin(voxel.position);
        cluster.bbox_max = cluster.bbox_max.cwiseMax(voxel.position);

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const super_utils::Vec3f neighbor_pos =
                            voxel.position + map_res * super_utils::Vec3f(dx, dy, dz);
                    if (map_manager.insideLocalMap(neighbor_pos) &&
                        isFreeLike(map_manager.getGridType(neighbor_pos))) {
                        unknown_dir += voxel.position - neighbor_pos;
                    }
                }
            }
        }
    }

    cluster.center /= static_cast<double>(std::max(1, cluster.size));
    if (unknown_dir.norm() > 1.0e-6) {
        cluster.unknown_direction = unknown_dir.normalized();
    } else {
        cluster.unknown_direction = super_utils::Vec3f::UnitX();
    }
}
} // namespace general_planner
