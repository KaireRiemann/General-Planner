#include <map_manager/global_exploration_map.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace general_planner {
namespace {
constexpr double kMinResolution = 1.0e-3;

struct BucketKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const BucketKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct BucketKeyHash {
    std::size_t operator()(const BucketKey &key) const {
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

BucketKey makeBucketKey(const super_utils::Vec3f &p, const double size) {
    return BucketKey{static_cast<int>(std::floor(p.x() / size)),
                     static_cast<int>(std::floor(p.y() / size)),
                     static_cast<int>(std::floor(p.z() / size))};
}

}  // namespace

std::size_t VoxelKeyHash::operator()(const Eigen::Vector3i &key) const {
    std::size_t seed = 0;
    const auto mix = [&seed](const int value) {
        const std::size_t h = std::hash<int>{}(value);
        seed ^= h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
    };
    mix(key.x());
    mix(key.y());
    mix(key.z());
    return seed;
}

bool VoxelKeyEqual::operator()(const Eigen::Vector3i &lhs, const Eigen::Vector3i &rhs) const {
    return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
}

GlobalExplorationMap::GlobalExplorationMap(const GlobalExplorationMapConfig &cfg)
        : cfg_(cfg) {
    cfg_.resolution = std::max(kMinResolution, cfg_.resolution);
    cfg_.raycast_step = std::max(kMinResolution, cfg_.raycast_step);
    cfg_.max_range = std::max(cfg_.min_range, cfg_.max_range);
    cfg_.occupied_hit_threshold = std::max(1, cfg_.occupied_hit_threshold);
    cfg_.free_miss_threshold = std::max(1, cfg_.free_miss_threshold);
}

void GlobalExplorationMap::updateObservation(const rog_map::PointCloud &cloud,
                                             const super_utils::Pose &pose,
                                             const CloudFrame frame,
                                             const double stamp) {
    if (!cfg_.enable) {
        return;
    }

    const Eigen::Vector3d origin = pose.first;
    if (!origin.allFinite() || !insideBBox(origin)) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    markFree(posToKey(origin), stamp);

    if (cloud.empty()) {
        return;
    }

    for (const auto &p : cloud) {
        Eigen::Vector3d hit = transformPointToWorld(p, pose, frame);
        if (!hit.allFinite() || !insideBBox(hit)) {
            continue;
        }
        const Eigen::Vector3d delta = hit - origin;
        const double dist = delta.norm();
        if (dist < cfg_.min_range || dist > cfg_.max_range) {
            continue;
        }
        raycastAndUpdate(origin, hit, stamp);
    }
}

GlobalVoxelState GlobalExplorationMap::getVoxelState(const Eigen::Vector3d &p) const {
    if (!insideBBox(p)) {
        return GlobalVoxelState::UNKNOWN;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = voxels_.find(posToKey(p));
    return it == voxels_.end() ? GlobalVoxelState::UNKNOWN : it->second.state;
}

bool GlobalExplorationMap::isObserved(const Eigen::Vector3d &p) const {
    return getVoxelState(p) != GlobalVoxelState::UNKNOWN;
}

bool GlobalExplorationMap::isUnknown(const Eigen::Vector3d &p) const {
    return getVoxelState(p) == GlobalVoxelState::UNKNOWN;
}

bool GlobalExplorationMap::isFrontier(const Eigen::Vector3d &p) const {
    if (!insideBBox(p)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = voxels_.find(posToKey(p));
    return it != voxels_.end() && it->second.frontier;
}

void GlobalExplorationMap::getFrontierPoints(std::vector<Eigen::Vector3d> &points) const {
    points.clear();
    std::lock_guard<std::mutex> lock(mutex_);
    points.reserve(frontier_keys_.size());
    for (const auto &key : frontier_keys_) {
        points.push_back(keyToPos(key));
    }
}

void GlobalExplorationMap::getObservedFreePoints(std::vector<Eigen::Vector3d> &points) const {
    points.clear();
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &kv : voxels_) {
        if (kv.second.state == GlobalVoxelState::OBSERVED_FREE) {
            points.push_back(keyToPos(kv.first));
        }
    }
}

void GlobalExplorationMap::getOccupiedPoints(std::vector<Eigen::Vector3d> &points) const {
    points.clear();
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &kv : voxels_) {
        if (kv.second.state == GlobalVoxelState::OBSERVED_OCCUPIED) {
            points.push_back(keyToPos(kv.first));
        }
    }
}

void GlobalExplorationMap::getFrontierClusters(const double cluster_radius,
                                               const int min_cluster_size,
                                               std::vector<FrontierCluster> &clusters) const {
    clusters.clear();
    std::vector<super_utils::Vec3f> points;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        points.reserve(frontier_keys_.size());
        for (const auto &key : frontier_keys_) {
            points.push_back(keyToPos(key));
        }
    }
    if (points.empty()) {
        return;
    }

    const double radius = std::max(cfg_.resolution, cluster_radius);
    const double radius_sq = radius * radius;
    std::unordered_map<BucketKey, std::vector<int>, BucketKeyHash> buckets;
    buckets.reserve(points.size());
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        buckets[makeBucketKey(points[static_cast<size_t>(i)], radius)].push_back(i);
    }

    std::vector<char> visited(points.size(), 0);
    std::queue<int> q;
    int next_id = 0;
    for (int seed = 0; seed < static_cast<int>(points.size()); ++seed) {
        if (visited[static_cast<size_t>(seed)] != 0) {
            continue;
        }
        FrontierCluster cluster;
        cluster.id = next_id++;
        visited[static_cast<size_t>(seed)] = 1;
        q.push(seed);
        while (!q.empty()) {
            const int current = q.front();
            q.pop();
            const super_utils::Vec3f current_pos = points[static_cast<size_t>(current)];
            cluster.cells.push_back(current_pos);

            const BucketKey bucket = makeBucketKey(current_pos, radius);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const BucketKey neighbor_bucket{bucket.x + dx, bucket.y + dy, bucket.z + dz};
                        const auto it = buckets.find(neighbor_bucket);
                        if (it == buckets.end()) {
                            continue;
                        }
                        for (const int neighbor : it->second) {
                            if (visited[static_cast<size_t>(neighbor)] != 0) {
                                continue;
                            }
                            if ((points[static_cast<size_t>(neighbor)] - current_pos).squaredNorm() > radius_sq) {
                                continue;
                            }
                            visited[static_cast<size_t>(neighbor)] = 1;
                            q.push(neighbor);
                        }
                    }
                }
            }
        }

        cluster.raw_size = static_cast<int>(cluster.cells.size());
        if (cluster.raw_size < std::max(1, min_cluster_size)) {
            continue;
        }
        cluster.filtered_cells = cluster.cells;
        cluster.center.setZero();
        cluster.bbox_min = cluster.cells.front();
        cluster.bbox_max = cluster.cells.front();
        for (const auto &cell : cluster.cells) {
            cluster.center += cell;
            cluster.bbox_min = cluster.bbox_min.cwiseMin(cell);
            cluster.bbox_max = cluster.bbox_max.cwiseMax(cell);
        }
        cluster.center /= static_cast<double>(cluster.raw_size);
        cluster.valid = true;
        clusters.push_back(cluster);
    }
}

double GlobalExplorationMap::exploredVolume() const {
    std::lock_guard<std::mutex> lock(mutex_);
    int count = 0;
    for (const auto &kv : voxels_) {
        if (kv.second.state != GlobalVoxelState::UNKNOWN) {
            ++count;
        }
    }
    return static_cast<double>(count) * cfg_.resolution * cfg_.resolution * cfg_.resolution;
}

double GlobalExplorationMap::occupiedVolume() const {
    std::lock_guard<std::mutex> lock(mutex_);
    int count = 0;
    for (const auto &kv : voxels_) {
        if (kv.second.state == GlobalVoxelState::OBSERVED_OCCUPIED) {
            ++count;
        }
    }
    return static_cast<double>(count) * cfg_.resolution * cfg_.resolution * cfg_.resolution;
}

int GlobalExplorationMap::frontierCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(frontier_keys_.size());
}

int GlobalExplorationMap::observedVoxelCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    int count = 0;
    for (const auto &kv : voxels_) {
        if (kv.second.state != GlobalVoxelState::UNKNOWN) {
            ++count;
        }
    }
    return count;
}

double GlobalExplorationMap::resolution() const {
    return cfg_.resolution;
}

void GlobalExplorationMap::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    voxels_.clear();
    frontier_keys_.clear();
}

GlobalExplorationMap::VoxelKey GlobalExplorationMap::posToKey(const Eigen::Vector3d &p) const {
    return VoxelKey(static_cast<int>(std::floor(p.x() / cfg_.resolution)),
                    static_cast<int>(std::floor(p.y() / cfg_.resolution)),
                    static_cast<int>(std::floor(p.z() / cfg_.resolution)));
}

Eigen::Vector3d GlobalExplorationMap::keyToPos(const VoxelKey &k) const {
    return (k.cast<double>() + Eigen::Vector3d::Constant(0.5)) * cfg_.resolution;
}

bool GlobalExplorationMap::insideBBox(const Eigen::Vector3d &p) const {
    return (p - cfg_.bbox_min).minCoeff() >= -1.0e-9 &&
           (cfg_.bbox_max - p).minCoeff() >= -1.0e-9;
}

Eigen::Vector3d GlobalExplorationMap::transformPointToWorld(const rog_map::PclPoint &p,
                                                            const super_utils::Pose &pose,
                                                            const CloudFrame frame) const {
    const Eigen::Vector3d local(p.x, p.y, p.z);
    if (frame == CloudFrame::WORLD) {
        return local;
    }
    return pose.first + pose.second * local;
}

void GlobalExplorationMap::raycastAndUpdate(const Eigen::Vector3d &origin,
                                            const Eigen::Vector3d &hit,
                                            const double stamp) {
    const Eigen::Vector3d delta = hit - origin;
    const double dist = delta.norm();
    if (dist < cfg_.min_range) {
        return;
    }

    const Eigen::Vector3d dir = delta / dist;
    const int steps = std::max(1, static_cast<int>(std::floor(dist / cfg_.raycast_step)));
    VoxelKey last_key = posToKey(origin);
    markFree(last_key, stamp);
    for (int i = 1; i < steps; ++i) {
        const Eigen::Vector3d p = origin + dir * (static_cast<double>(i) * cfg_.raycast_step);
        if (!insideBBox(p)) {
            break;
        }
        const VoxelKey key = posToKey(p);
        if (!VoxelKeyEqual{}(key, last_key)) {
            markFree(key, stamp);
            last_key = key;
        }
    }
    markOccupied(posToKey(hit), stamp);
}

void GlobalExplorationMap::markFree(const VoxelKey &key, const double stamp) {
    GlobalVoxel &voxel = voxels_[key];
    if (voxel.first_observed_time < 0.0) {
        voxel.first_observed_time = stamp;
    }
    voxel.last_observed_time = stamp;
    if (voxel.miss_count < std::numeric_limits<uint16_t>::max()) {
        ++voxel.miss_count;
    }
    if (voxel.state != GlobalVoxelState::OBSERVED_OCCUPIED &&
        voxel.miss_count >= static_cast<uint16_t>(cfg_.free_miss_threshold)) {
        voxel.state = GlobalVoxelState::OBSERVED_FREE;
    }
    updateFrontierAround(key);
}

void GlobalExplorationMap::markOccupied(const VoxelKey &key, const double stamp) {
    GlobalVoxel &voxel = voxels_[key];
    if (voxel.first_observed_time < 0.0) {
        voxel.first_observed_time = stamp;
    }
    voxel.last_observed_time = stamp;
    if (voxel.hit_count < std::numeric_limits<uint16_t>::max()) {
        ++voxel.hit_count;
    }
    if (voxel.hit_count >= static_cast<uint16_t>(cfg_.occupied_hit_threshold)) {
        voxel.state = GlobalVoxelState::OBSERVED_OCCUPIED;
        voxel.frontier = false;
        frontier_keys_.erase(key);
    }
    updateFrontierAround(key);
}

void GlobalExplorationMap::updateFrontierAround(const VoxelKey &key) {
    if (!cfg_.maintain_frontiers) {
        return;
    }
    const int bound = cfg_.use_26_neighbor_frontier ? 1 : 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                if (bound == 0 && std::abs(dx) + std::abs(dy) + std::abs(dz) != 1) {
                    continue;
                }
                const VoxelKey nkey = key + VoxelKey(dx, dy, dz);
                const auto it = voxels_.find(nkey);
                if (it == voxels_.end()) {
                    continue;
                }
                const bool is_frontier = computeIsFrontier(nkey);
                it->second.frontier = is_frontier;
                if (is_frontier) {
                    frontier_keys_.insert(nkey);
                } else {
                    frontier_keys_.erase(nkey);
                }
            }
        }
    }

    const auto it = voxels_.find(key);
    if (it == voxels_.end()) {
        return;
    }
    const bool is_frontier = computeIsFrontier(key);
    it->second.frontier = is_frontier;
    if (is_frontier) {
        frontier_keys_.insert(key);
    } else {
        frontier_keys_.erase(key);
    }
}

bool GlobalExplorationMap::computeIsFrontier(const VoxelKey &key) const {
    const auto it = voxels_.find(key);
    if (it == voxels_.end() || it->second.state != GlobalVoxelState::OBSERVED_FREE) {
        return false;
    }

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                if (!cfg_.use_26_neighbor_frontier &&
                    std::abs(dx) + std::abs(dy) + std::abs(dz) != 1) {
                    continue;
                }
                const VoxelKey nkey = key + VoxelKey(dx, dy, dz);
                const auto nit = voxels_.find(nkey);
                if (nit == voxels_.end() || nit->second.state == GlobalVoxelState::UNKNOWN) {
                    return true;
                }
            }
        }
    }
    return false;
}

}  // namespace general_planner
