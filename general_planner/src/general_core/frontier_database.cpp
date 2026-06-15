#include <general_core/frontier_database.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace general_planner {
namespace {
std::size_t mixHash(std::size_t seed, const int value) {
    const std::size_t h = std::hash<int>{}(value);
    return seed ^ (h + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
}

bool bboxOverlap(const CompleteFrontierCluster &lhs, const CompleteFrontierCluster &rhs) {
    return (lhs.bbox_min.array() <= rhs.bbox_max.array()).all() &&
           (rhs.bbox_min.array() <= lhs.bbox_max.array()).all();
}
}  // namespace

std::size_t FrontierDatabase::ClusterBucketHasher::operator()(const ClusterBucketKey &key) const {
    std::size_t seed = 0;
    seed = mixHash(seed, key.x);
    seed = mixHash(seed, key.y);
    seed = mixHash(seed, key.z);
    return seed;
}

FrontierDatabase::FrontierDatabase()
        : FrontierDatabase(Config{}) {
}

FrontierDatabase::FrontierDatabase(const Config &cfg)
        : cfg_(cfg) {
    cfg_.cluster_radius = std::max(0.1, cfg_.cluster_radius);
    cfg_.min_cluster_size = std::max(1, cfg_.min_cluster_size);
}

void FrontierDatabase::reset() {
    frontiers_.clear();
    next_id_ = 1;
}

FrontierDatabase::ClusterBucketKey FrontierDatabase::makeBucketKey(const super_utils::Vec3f &pos) const {
    return ClusterBucketKey{static_cast<int>(std::floor(pos.x() / cfg_.cluster_radius)),
                            static_cast<int>(std::floor(pos.y() / cfg_.cluster_radius)),
                            static_cast<int>(std::floor(pos.z() / cfg_.cluster_radius))};
}

void FrontierDatabase::clusterObservedFrontiers(const super_utils::vec_E<CompleteFrontierCell> &cells,
                                                super_utils::vec_E<CompleteFrontierCluster> &clusters) const {
    clusters.clear();
    if (cells.empty()) {
        return;
    }

    std::unordered_map<ClusterBucketKey, std::vector<int>, ClusterBucketHasher> buckets;
    buckets.reserve(cells.size());
    for (int i = 0; i < static_cast<int>(cells.size()); ++i) {
        buckets[makeBucketKey(cells[static_cast<std::size_t>(i)].position)].push_back(i);
    }

    std::vector<char> visited(cells.size(), 0);
    const double radius_sq = cfg_.cluster_radius * cfg_.cluster_radius;
    for (int seed = 0; seed < static_cast<int>(cells.size()); ++seed) {
        if (visited[static_cast<std::size_t>(seed)] != 0) {
            continue;
        }
        CompleteFrontierCluster cluster;
        std::queue<int> q;
        q.push(seed);
        visited[static_cast<std::size_t>(seed)] = 1;
        while (!q.empty()) {
            const int current = q.front();
            q.pop();
            cluster.cells.push_back(cells[static_cast<std::size_t>(current)]);
            const ClusterBucketKey key = makeBucketKey(cells[static_cast<std::size_t>(current)].position);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const ClusterBucketKey nk{key.x + dx, key.y + dy, key.z + dz};
                        const auto bit = buckets.find(nk);
                        if (bit == buckets.end()) {
                            continue;
                        }
                        for (const int candidate : bit->second) {
                            if (visited[static_cast<std::size_t>(candidate)] != 0) {
                                continue;
                            }
                            if ((cells[static_cast<std::size_t>(candidate)].position -
                                 cells[static_cast<std::size_t>(current)].position).squaredNorm() <= radius_sq) {
                                visited[static_cast<std::size_t>(candidate)] = 1;
                                q.push(candidate);
                            }
                        }
                    }
                }
            }
        }
        if (static_cast<int>(cluster.cells.size()) >= cfg_.min_cluster_size) {
            clusters.push_back(cluster);
        }
    }
}

void FrontierDatabase::finalizeCluster(CompleteFrontierCluster &cluster,
                                       const ExplorationMemoryGrid &memory,
                                       const double stamp) const {
    cluster.size = static_cast<int>(cluster.cells.size());
    cluster.center.setZero();
    cluster.bbox_min = super_utils::Vec3f::Constant(std::numeric_limits<double>::infinity());
    cluster.bbox_max = super_utils::Vec3f::Constant(-std::numeric_limits<double>::infinity());
    super_utils::Vec3f unknown_dir = super_utils::Vec3f::Zero();
    int unknown_neighbors = 0;

    for (const auto &cell : cluster.cells) {
        cluster.center += cell.position;
        cluster.bbox_min = cluster.bbox_min.cwiseMin(cell.position);
        cluster.bbox_max = cluster.bbox_max.cwiseMax(cell.position);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const super_utils::Vec3i nid = cell.index + super_utils::Vec3i(dx, dy, dz);
                    if (memory.getState(nid) == ExplorationVoxelState::UNKNOWN) {
                        unknown_dir += memory.indexToPosition(nid) - cell.position;
                        ++unknown_neighbors;
                    }
                }
            }
        }
    }
    if (!cluster.cells.empty()) {
        cluster.center /= static_cast<double>(cluster.cells.size());
    }
    if (unknown_dir.norm() > 1.0e-3) {
        cluster.unknown_direction = unknown_dir.normalized();
    } else {
        cluster.unknown_direction = super_utils::Vec3f::UnitX();
    }
    cluster.estimated_gain = static_cast<double>(cluster.size) +
                             0.25 * static_cast<double>(unknown_neighbors);
    cluster.coverage_priority = cluster.estimated_gain;
    cluster.last_seen_time = stamp;
}

int FrontierDatabase::matchExistingCluster(const CompleteFrontierCluster &cluster,
                                           const std::unordered_map<int, bool> &used_ids) const {
    double best_score = std::numeric_limits<double>::infinity();
    int best_id = -1;
    std::vector<int> ids;
    ids.reserve(frontiers_.size());
    for (const auto &kv : frontiers_) {
        ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());
    for (const int id : ids) {
        if (used_ids.find(id) != used_ids.end()) {
            continue;
        }
        const auto it = frontiers_.find(id);
        if (it == frontiers_.end() ||
            it->second.status == FrontierStatus::BLACKLISTED) {
            continue;
        }
        const double center_distance = (cluster.center - it->second.center).norm();
        if (center_distance > cfg_.merge_center_distance &&
            !bboxOverlap(cluster, it->second)) {
            continue;
        }
        if (center_distance < best_score) {
            best_score = center_distance;
            best_id = id;
        }
    }
    return best_id;
}

bool FrontierDatabase::clusterStillFrontier(const CompleteFrontierCluster &cluster,
                                            const ExplorationMemoryGrid &memory) const {
    for (const auto &cell : cluster.cells) {
        if (!memory.isKnownFree(cell.position)) {
            continue;
        }
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const super_utils::Vec3i neighbor_index =
                            (cell.index + super_utils::Vec3i(dx, dy, dz)).eval();
                    if (memory.getState(neighbor_index) ==
                        ExplorationVoxelState::UNKNOWN) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void FrontierDatabase::update(const super_utils::vec_E<CompleteFrontierCell> &frontier_cells,
                              const ExplorationMemoryGrid &memory,
                              const double stamp) {
    super_utils::vec_E<CompleteFrontierCluster> observed_clusters;
    clusterObservedFrontiers(frontier_cells, observed_clusters);
    for (auto &cluster : observed_clusters) {
        finalizeCluster(cluster, memory, stamp);
    }
    std::sort(observed_clusters.begin(), observed_clusters.end(),
              [](const CompleteFrontierCluster &lhs, const CompleteFrontierCluster &rhs) {
                  if (lhs.center.x() != rhs.center.x()) return lhs.center.x() < rhs.center.x();
                  if (lhs.center.y() != rhs.center.y()) return lhs.center.y() < rhs.center.y();
                  return lhs.center.z() < rhs.center.z();
              });

    std::unordered_map<int, bool> used_ids;
    for (auto &cluster : observed_clusters) {
        const int matched_id = matchExistingCluster(cluster, used_ids);
        if (matched_id > 0) {
            CompleteFrontierCluster previous = frontiers_[matched_id];
            cluster.id = matched_id;
            cluster.first_seen_time = previous.first_seen_time;
            cluster.last_selected_time = previous.last_selected_time;
            cluster.fail_count = previous.fail_count;
            cluster.best_travel_cost = previous.best_travel_cost;
            cluster.status = previous.status == FrontierStatus::UNREACHABLE
                                     ? FrontierStatus::UNREACHABLE
                                     : FrontierStatus::ACTIVE;
            frontiers_[matched_id] = cluster;
            used_ids[matched_id] = true;
        } else {
            cluster.id = next_id_++;
            cluster.first_seen_time = stamp;
            cluster.status = FrontierStatus::ACTIVE;
            frontiers_[cluster.id] = cluster;
            used_ids[cluster.id] = true;
        }
    }

    for (auto &kv : frontiers_) {
        if (used_ids.find(kv.first) != used_ids.end()) {
            continue;
        }
        auto &frontier = kv.second;
        if (frontier.status == FrontierStatus::BLACKLISTED ||
            frontier.status == FrontierStatus::COVERED) {
            continue;
        }
        const bool still_frontier = clusterStillFrontier(frontier, memory);
        if (!still_frontier) {
            frontier.status = FrontierStatus::COVERED;
            frontier.last_seen_time = stamp;
        } else if (frontier.status == FrontierStatus::ACTIVE &&
                   stamp - frontier.last_seen_time > cfg_.stale_timeout) {
            frontier.status = FrontierStatus::DORMANT;
        } else if (frontier.status == FrontierStatus::DORMANT &&
                   stamp - frontier.last_seen_time > cfg_.stale_timeout + cfg_.dormant_timeout) {
            frontier.status = FrontierStatus::UNREACHABLE;
            frontier.fail_count = std::max(frontier.fail_count, cfg_.max_fail_count);
        }
    }
}

std::vector<int> FrontierDatabase::activeFrontierIds() const {
    std::vector<int> ids;
    for (const auto &kv : frontiers_) {
        if (kv.second.status == FrontierStatus::ACTIVE) {
            ids.push_back(kv.first);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<int> FrontierDatabase::reachableFrontierIds() const {
    std::vector<int> ids;
    for (const auto &kv : frontiers_) {
        if (kv.second.status == FrontierStatus::ACTIVE &&
            kv.second.fail_count < cfg_.max_fail_count) {
            ids.push_back(kv.first);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<CompleteFrontierCluster> FrontierDatabase::getActiveFrontiers() const {
    std::vector<CompleteFrontierCluster> clusters;
    for (const auto &kv : frontiers_) {
        if (kv.second.status == FrontierStatus::ACTIVE &&
            kv.second.fail_count < cfg_.max_fail_count) {
            clusters.push_back(kv.second);
        }
    }
    std::sort(clusters.begin(), clusters.end(),
              [](const CompleteFrontierCluster &lhs, const CompleteFrontierCluster &rhs) {
                  return lhs.id < rhs.id;
              });
    return clusters;
}

std::vector<CompleteFrontierCluster> FrontierDatabase::getAllFrontiers() const {
    std::vector<CompleteFrontierCluster> clusters;
    clusters.reserve(frontiers_.size());
    for (const auto &kv : frontiers_) {
        clusters.push_back(kv.second);
    }
    std::sort(clusters.begin(), clusters.end(),
              [](const CompleteFrontierCluster &lhs, const CompleteFrontierCluster &rhs) {
                  return lhs.id < rhs.id;
              });
    return clusters;
}

bool FrontierDatabase::getFrontier(const int id, CompleteFrontierCluster &out) const {
    const auto it = frontiers_.find(id);
    if (it == frontiers_.end()) {
        return false;
    }
    out = it->second;
    return true;
}

void FrontierDatabase::markSelected(const int id, const double stamp) {
    const auto it = frontiers_.find(id);
    if (it != frontiers_.end()) {
        it->second.last_selected_time = stamp;
    }
}

void FrontierDatabase::markFailed(const int id, const std::string &reason, const double stamp) {
    (void)reason;
    const auto it = frontiers_.find(id);
    if (it == frontiers_.end()) {
        return;
    }
    ++it->second.fail_count;
    it->second.last_seen_time = stamp;
    if (it->second.fail_count >= cfg_.max_fail_count) {
        it->second.status = FrontierStatus::UNREACHABLE;
    }
}

void FrontierDatabase::setStatus(const int id, const FrontierStatus status, const double stamp) {
    const auto it = frontiers_.find(id);
    if (it != frontiers_.end()) {
        it->second.status = status;
        it->second.last_seen_time = stamp;
    }
}

void FrontierDatabase::markCovered(const int id, const double stamp) {
    setStatus(id, FrontierStatus::COVERED, stamp);
}

void FrontierDatabase::markDormant(const int id, const double stamp) {
    setStatus(id, FrontierStatus::DORMANT, stamp);
}

void FrontierDatabase::markUnreachable(const int id, const double stamp) {
    setStatus(id, FrontierStatus::UNREACHABLE, stamp);
}

void FrontierDatabase::markBlacklisted(const int id, const double stamp) {
    setStatus(id, FrontierStatus::BLACKLISTED, stamp);
}

void FrontierDatabase::reviveUnreachableNear(const super_utils::Vec3f &pos,
                                             const double radius,
                                             const double stamp) {
    const double radius_sq = radius * radius;
    const double revive_timeout = std::max(0.0, cfg_.dormant_timeout);
    for (auto &kv : frontiers_) {
        auto &frontier = kv.second;
        if (frontier.status != FrontierStatus::UNREACHABLE) {
            continue;
        }
        if (revive_timeout > 0.0 && stamp - frontier.last_seen_time < revive_timeout) {
            continue;
        }
        if ((frontier.center - pos).squaredNorm() <= radius_sq) {
            frontier.status = FrontierStatus::ACTIVE;
            frontier.fail_count = std::max(0, cfg_.max_fail_count - 1);
            frontier.last_seen_time = stamp;
        }
    }
}

int FrontierDatabase::countStatus(const FrontierStatus status) const {
    int count = 0;
    for (const auto &kv : frontiers_) {
        if (kv.second.status == status) {
            ++count;
        }
    }
    return count;
}

int FrontierDatabase::activeCount() const {
    return countStatus(FrontierStatus::ACTIVE);
}

int FrontierDatabase::reachableCount() const {
    return static_cast<int>(reachableFrontierIds().size());
}

int FrontierDatabase::coveredCount() const {
    return countStatus(FrontierStatus::COVERED);
}

int FrontierDatabase::unreachableCount() const {
    return countStatus(FrontierStatus::UNREACHABLE);
}

int FrontierDatabase::dormantCount() const {
    return countStatus(FrontierStatus::DORMANT);
}

int FrontierDatabase::blacklistedCount() const {
    return countStatus(FrontierStatus::BLACKLISTED);
}

}  // namespace general_planner
