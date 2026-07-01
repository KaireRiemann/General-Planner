#include <general_core/exploration/exploration_frontend.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Eigenvalues>

#include <general_core/exploration/exploration_manager.hpp>
#include <general_core/utils/string_utils.hpp>

using namespace general_utils;

namespace general_planner {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kInfCost = 1.0e9;

struct GridKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const GridKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHasher {
    std::size_t operator()(const GridKey &key) const {
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

GridKey makeKey(const Vec3i &id) {
    return GridKey{id.x(), id.y(), id.z()};
}

GridKey makeBucketKey(const Vec3f &pos, const double bucket_size) {
    return GridKey{
            static_cast<int>(std::floor(pos.x() / bucket_size)),
            static_cast<int>(std::floor(pos.y() / bucket_size)),
            static_cast<int>(std::floor(pos.z() / bucket_size))};
}

double angleBetween(Vec3f lhs, Vec3f rhs) {
    lhs.z() = 0.0;
    rhs.z() = 0.0;
    const double lhs_norm = lhs.norm();
    const double rhs_norm = rhs.norm();
    if (lhs_norm < 1.0e-6 || rhs_norm < 1.0e-6) {
        return 0.0;
    }
    const double c = std::clamp(lhs.dot(rhs) / (lhs_norm * rhs_norm), -1.0, 1.0);
    return std::acos(c);
}

double pathLength(const vec_E<Vec3f> &path) {
    if (path.size() < 2) {
        return 0.0;
    }
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += (path[i] - path[i - 1]).norm();
    }
    return length;
}

bool sourceAllowsRogMap(const std::string &source) {
    return source == "auto" ||
           source == "rog_map" ||
           source == "rog_map_frontier" ||
           source == "frontier";
}

bool sourceAllowsFallback(const std::string &source) {
    return source == "auto" ||
           source == "fallback" ||
           source == "fallback_scan" ||
           source == "local_scan";
}

GridKey quantizedKey(const Vec3f &pos, const double bucket_size) {
    const double bucket = std::max(1.0e-3, bucket_size);
    return GridKey{
            static_cast<int>(std::floor(pos.x() / bucket)),
            static_cast<int>(std::floor(pos.y() / bucket)),
            static_cast<int>(std::floor(pos.z() / bucket))};
}

int stableIdFromKey(const GridKey &key) {
    const std::size_t hashed = GridKeyHasher{}(key);
    return static_cast<int>(hashed & 0x7fffffffU);
}

std::string memoryKeyFromKey(const GridKey &key) {
    std::ostringstream oss;
    oss << key.x << ":" << key.y << ":" << key.z;
    return oss.str();
}

Vec3f horizontalNormalized(Vec3f direction, Vec3f fallback) {
    direction.z() = 0.0;
    fallback.z() = 0.0;
    if (direction.norm() < 1.0e-6) {
        direction = fallback;
    }
    if (direction.norm() < 1.0e-6) {
        direction = Vec3f::UnitX();
    }
    return direction.normalized();
}

int boundedStride(const size_t count, const int max_samples) {
    if (count == 0U || max_samples <= 0) {
        return 1;
    }
    return std::max(1, static_cast<int>(std::ceil(static_cast<double>(count) /
                                                  static_cast<double>(max_samples))));
}

int sampledCount(const size_t count, const int max_samples) {
    if (count == 0U) {
        return 0;
    }
    const int stride = boundedStride(count, max_samples);
    return static_cast<int>((count + static_cast<size_t>(stride) - 1U) /
                            static_cast<size_t>(stride));
}
}  // namespace

class ExplorationFrontend::FrontierObjectManager {
public:
    FrontierObjectManager(const Config &cfg, MapManager::Ptr map_manager)
            : cfg_(cfg),
              map_manager_(std::move(map_manager)) {
    }

    void reset() {
        records_.clear();
        next_collision_salt_ = 1;
    }

    void update(const vec_E<FrontierCluster> &observed_clusters,
                const Vec3f &robot_pos,
                const double stamp,
                vec_E<FrontierCluster> &active_clusters,
                FrontierObjectStats &stats) {
        active_clusters.clear();
        stats = FrontierObjectStats{};
        stats.observed = static_cast<int>(observed_clusters.size());

        if (!enabled()) {
            active_clusters = observed_clusters;
            stats.active = static_cast<int>(active_clusters.size());
            stats.records = stats.active;
            return;
        }

        prune(stamp);
        std::unordered_set<int> matched_records;
        matched_records.reserve(observed_clusters.size());

        for (const FrontierCluster &cluster : observed_clusters) {
            int record_id = findMatchingRecord(cluster, matched_records);
            if (record_id < 0) {
                record_id = allocateRecordId(cluster);
                FrontierRecord record;
                record.id = record_id;
                record.first_seen_stamp = stamp;
                records_.emplace(record_id, std::move(record));
            }

            FrontierRecord &record = records_.at(record_id);
            matched_records.insert(record_id);
            record.cluster = cluster;
            record.reference = cluster.center;
            record.last_seen_stamp = stamp;
            record.last_state_stamp = stamp;
            ++record.seen_count;
            if (record.state == FrontierState::COVERED ||
                record.state == FrontierState::STALE) {
                record.state = FrontierState::ACTIVE;
            }

            if (record.dormant_until > stamp) {
                record.state = FrontierState::DORMANT;
                continue;
            }

            if (!frontierStillUseful(record.cluster)) {
                record.state = FrontierState::COVERED;
                record.blocked_until = stamp + std::max(cfg_.frontier_manager_stale_time,
                                                        cfg_.frontier_manager_dormant_time);
                continue;
            }

            record.state = FrontierState::ACTIVE;
            FrontierCluster annotated = record.cluster;
            annotateCluster(record, stamp, annotated);
            active_clusters.push_back(std::move(annotated));
        }

        for (auto &entry : records_) {
            FrontierRecord &record = entry.second;
            if (matched_records.find(record.id) != matched_records.end()) {
                continue;
            }
            if (record.state == FrontierState::COVERED ||
                record.state == FrontierState::STALE) {
                continue;
            }
            if (!frontierBoxInsideLocalMap(record.cluster)) {
                continue;
            }
            if (!frontierStillUseful(record.cluster)) {
                record.state = FrontierState::COVERED;
                record.last_state_stamp = stamp;
                record.blocked_until = stamp + std::max(cfg_.frontier_manager_stale_time,
                                                        cfg_.frontier_manager_dormant_time);
                continue;
            }
            if (cfg_.frontier_manager_stale_time > 0.0 &&
                stamp - record.last_seen_stamp > cfg_.frontier_manager_stale_time) {
                record.state = FrontierState::STALE;
                record.last_state_stamp = stamp;
            }
        }

        recomputeStats(active_clusters, stats, stamp);
    }

    void recordNoView(const int object_id, const double stamp) {
        if (!enabled() || object_id < 0) {
            return;
        }
        FrontierRecord *record = findRecord(object_id);
        if (record == nullptr) {
            return;
        }
        ++record->no_view_count;
        if (cfg_.frontier_manager_no_view_threshold > 0 &&
            record->no_view_count >= cfg_.frontier_manager_no_view_threshold) {
            record->state = FrontierState::DORMANT;
            record->dormant_until = stamp + std::max(0.0, cfg_.frontier_manager_dormant_time);
            record->last_state_stamp = stamp;
        }
    }

    void recordCommitted(const ExplorationGoal &goal,
                         const double stamp,
                         const bool goal_switched) {
        if (!enabled() || !goal.valid || goal.frontier_id < 0 || !goal_switched) {
            return;
        }
        FrontierRecord *record = findRecord(goal.frontier_id);
        if (record == nullptr) {
            return;
        }
        ++record->selection_count;
        record->last_selected_stamp = stamp;
        record->no_view_count = 0;
        record->state = FrontierState::ACTIVE;
        record->last_state_stamp = stamp;
    }

    void recordFailed(const ExplorationGoal &goal, const double stamp) {
        if (!enabled() || !goal.valid || goal.frontier_id < 0) {
            return;
        }
        FrontierRecord *record = findRecord(goal.frontier_id);
        if (record == nullptr) {
            return;
        }
        ++record->failure_count;
        record->state = FrontierState::DORMANT;
        record->dormant_until = stamp + std::max(0.0, cfg_.frontier_manager_dormant_time);
        record->last_state_stamp = stamp;
    }

private:
    enum class FrontierState {
        ACTIVE,
        DORMANT,
        COVERED,
        STALE
    };

    struct FrontierRecord {
        FrontierCluster cluster;
        Vec3f reference{Vec3f::Zero()};
        FrontierState state{FrontierState::ACTIVE};
        int id{-1};
        int seen_count{0};
        int selection_count{0};
        int failure_count{0};
        int no_view_count{0};
        double first_seen_stamp{0.0};
        double last_seen_stamp{0.0};
        double last_state_stamp{0.0};
        double last_selected_stamp{0.0};
        double dormant_until{0.0};
        double blocked_until{0.0};
    };

    bool enabled() const {
        return cfg_.frontier_manager_enable &&
               map_manager_ != nullptr &&
               map_manager_->ready();
    }

    FrontierRecord *findRecord(const int object_id) {
        const auto it = records_.find(object_id);
        return it == records_.end() ? nullptr : &it->second;
    }

    int allocateRecordId(const FrontierCluster &cluster) {
        const double id_resolution =
                std::max(0.5, 0.5 * cfg_.frontier_manager_match_radius);
        GridKey key = quantizedKey(cluster.center, id_resolution);
        int id = stableIdFromKey(key);
        while (records_.find(id) != records_.end()) {
            id = stableIdFromKey(GridKey{key.x, key.y, key.z + next_collision_salt_});
            ++next_collision_salt_;
        }
        return id;
    }

    int findMatchingRecord(const FrontierCluster &cluster,
                           const std::unordered_set<int> &already_matched) const {
        const double match_radius =
                std::max(map_manager_->getResolution(), cfg_.frontier_manager_match_radius);
        const double match_radius_sq = match_radius * match_radius;
        int best_id = -1;
        double best_score = std::numeric_limits<double>::infinity();
        for (const auto &entry : records_) {
            const FrontierRecord &record = entry.second;
            if (already_matched.find(record.id) != already_matched.end() ||
                record.state == FrontierState::COVERED ||
                record.state == FrontierState::STALE) {
                continue;
            }
            const double center_distance_sq = (record.reference - cluster.center).squaredNorm();
            const bool overlaps = boxesOverlap(record.cluster, cluster, 0.25 * match_radius);
            if (!overlaps && center_distance_sq > match_radius_sq) {
                continue;
            }
            const double score = std::sqrt(std::max(0.0, center_distance_sq)) -
                                 (overlaps ? 0.5 * match_radius : 0.0);
            if (score < best_score) {
                best_score = score;
                best_id = record.id;
            }
        }
        return best_id;
    }

    static bool boxesOverlap(const FrontierCluster &lhs,
                             const FrontierCluster &rhs,
                             const double inflation) {
        const Vec3f lhs_min = lhs.bbox_min - Vec3f::Constant(inflation);
        const Vec3f lhs_max = lhs.bbox_max + Vec3f::Constant(inflation);
        const Vec3f rhs_min = rhs.bbox_min - Vec3f::Constant(inflation);
        const Vec3f rhs_max = rhs.bbox_max + Vec3f::Constant(inflation);
        return (lhs_min.array() <= rhs_max.array()).all() &&
               (rhs_min.array() <= lhs_max.array()).all();
    }

    bool frontierBoxInsideLocalMap(const FrontierCluster &cluster) const {
        return map_manager_ != nullptr &&
               map_manager_->ready() &&
               map_manager_->insideLocalMap(cluster.bbox_min) &&
               map_manager_->insideLocalMap(cluster.bbox_max);
    }

    bool isUnknownLike(const rog_map::GridType type) const {
        return type == rog_map::GridType::UNKNOWN ||
               type == rog_map::GridType::UNDEFINED ||
               type == rog_map::GridType::FRONTIER;
    }

    bool isFreeLike(const rog_map::GridType type) const {
        return type == rog_map::GridType::KNOWN_FREE;
    }

    bool hasUnknownNeighbor(const Vec3f &position) const {
        const double map_res = std::max(1.0e-3, map_manager_->getResolution());
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const Vec3f neighbor_pos = position + map_res * Vec3f(dx, dy, dz);
                    if (map_manager_->insideLocalMap(neighbor_pos) &&
                        isUnknownLike(map_manager_->getGridType(neighbor_pos))) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    int countUnknownNear(const Vec3f &position,
                         const double radius,
                         const double resolution) const {
        if (!position.allFinite() ||
            map_manager_ == nullptr ||
            !map_manager_->ready() ||
            !map_manager_->insideLocalMap(position)) {
            return 0;
        }
        const double res = std::max(map_manager_->getResolution(), resolution);
        const int steps = std::max(1, static_cast<int>(std::ceil(radius / res)));
        const double radius_sq = radius * radius;
        int count = 0;
        for (int dx = -steps; dx <= steps; ++dx) {
            for (int dy = -steps; dy <= steps; ++dy) {
                for (int dz = -steps; dz <= steps; ++dz) {
                    const Vec3f offset = res * Vec3f(dx, dy, dz);
                    if (offset.squaredNorm() > radius_sq) {
                        continue;
                    }
                    const Vec3f sample = position + offset;
                    if (map_manager_->insideLocalMap(sample) &&
                        isUnknownLike(map_manager_->getGridType(sample))) {
                        ++count;
                    }
                }
            }
        }
        return count;
    }

    bool frontierStillUseful(const FrontierCluster &cluster) const {
        if (cluster.cells.empty()) {
            return false;
        }
        if (!frontierBoxInsideLocalMap(cluster)) {
            return true;
        }

        const int stride = boundedStride(cluster.cells.size(), 96);
        int checked = 0;
        int still_frontier = 0;
        for (size_t i = 0; i < cluster.cells.size(); i += static_cast<size_t>(stride)) {
            const Vec3f &position = cluster.cells[i].position;
            if (!map_manager_->insideLocalMap(position)) {
                continue;
            }
            ++checked;
            if (isFreeLike(map_manager_->getGridType(position)) &&
                hasUnknownNeighbor(position)) {
                ++still_frontier;
            }
        }

        if (checked == 0) {
            return true;
        }
        const double changed_fraction =
                std::clamp(cfg_.frontier_manager_min_changed_fraction, 0.0, 1.0);
        const int min_still_frontier =
                std::max(1, static_cast<int>(std::ceil(changed_fraction *
                                                       static_cast<double>(checked))));
        if (still_frontier >= min_still_frontier) {
            return true;
        }

        const double unknown_radius =
                std::max(map_manager_->getResolution(),
                         cfg_.frontier_manager_covered_unknown_radius);
        return countUnknownNear(cluster.center,
                                unknown_radius,
                                std::max(map_manager_->getResolution(), cfg_.map_resolution)) > 0;
    }

    void annotateCluster(const FrontierRecord &record,
                         const double stamp,
                         FrontierCluster &cluster) const {
        cluster.object_id = record.id;
        cluster.object_seen_count = record.seen_count;
        cluster.object_selection_count = record.selection_count;
        cluster.object_failure_count = record.failure_count;
        cluster.object_last_seen_stamp = record.last_seen_stamp;

        double score_delta =
                cfg_.frontier_manager_selection_penalty *
                static_cast<double>(std::max(0, record.selection_count));
        score_delta += 0.5 * cfg_.frontier_manager_selection_penalty *
                       static_cast<double>(std::max(0, record.failure_count));

        if (record.last_selected_stamp > 0.0 &&
            cfg_.frontier_manager_recent_selection_window > 1.0e-6) {
            const double elapsed = std::max(0.0, stamp - record.last_selected_stamp);
            if (elapsed < cfg_.frontier_manager_recent_selection_window) {
                const double ratio =
                        1.0 - elapsed / cfg_.frontier_manager_recent_selection_window;
                score_delta += cfg_.frontier_manager_recent_selection_penalty * ratio;
            }
        }
        cluster.object_score_delta = score_delta;
    }

    void recomputeStats(const vec_E<FrontierCluster> &active_clusters,
                        FrontierObjectStats &stats,
                        const double stamp) const {
        stats.active = static_cast<int>(active_clusters.size());
        stats.records = static_cast<int>(records_.size());
        stats.dormant = 0;
        stats.covered = 0;
        stats.stale = 0;
        for (const auto &entry : records_) {
            const FrontierRecord &record = entry.second;
            if (record.state == FrontierState::DORMANT && record.dormant_until > stamp) {
                ++stats.dormant;
            } else if (record.state == FrontierState::COVERED) {
                ++stats.covered;
            } else if (record.state == FrontierState::STALE) {
                ++stats.stale;
            }
        }
    }

    void prune(const double stamp) {
        const double keep_time =
                std::max(4.0 * cfg_.frontier_manager_stale_time,
                         2.0 * cfg_.frontier_manager_dormant_time);
        for (auto it = records_.begin(); it != records_.end();) {
            const FrontierRecord &record = it->second;
            const bool terminal =
                    record.state == FrontierState::COVERED ||
                    record.state == FrontierState::STALE;
            if (terminal && keep_time > 0.0 && stamp - record.last_state_stamp > keep_time) {
                it = records_.erase(it);
            } else {
                ++it;
            }
        }

        const int max_records = std::max(0, cfg_.frontier_manager_max_records);
        while (max_records > 0 && static_cast<int>(records_.size()) > max_records) {
            auto oldest = records_.begin();
            for (auto it = records_.begin(); it != records_.end(); ++it) {
                if (it->second.last_seen_stamp < oldest->second.last_seen_stamp) {
                    oldest = it;
                }
            }
            records_.erase(oldest);
        }
    }

    Config cfg_;
    MapManager::Ptr map_manager_;
    std::unordered_map<int, FrontierRecord> records_;
    int next_collision_salt_{1};
};

ExplorationFrontend::ExplorationFrontend(const Config &cfg,
                                         const MapManager::Ptr &map_manager,
                                         const path_search::Astar::Ptr &astar)
        : cfg_(cfg),
          map_manager_(map_manager),
          astar_(astar),
          frontier_object_manager_(
                  std::make_unique<FrontierObjectManager>(cfg_, map_manager_)) {
}

ExplorationFrontend::~ExplorationFrontend() = default;

bool ExplorationFrontend::planNextGoal(const StatePVAJ &robot_state,
                                       const double current_yaw,
                                       ExplorationGoal &goal,
                                       const double stamp) {
    goal = ExplorationGoal{};
    exploration_finished_ = false;

    if (!cfg_.enable) {
        goal.reason = "exploration disabled";
        return false;
    }
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        goal.reason = "map manager not ready";
        return false;
    }

    const Vec3f robot_pos = robot_state.col(0);
    if (!robot_pos.allFinite()) {
        goal.reason = "robot state is not finite";
        return false;
    }
    if (!map_manager_->insideLocalMap(robot_pos)) {
        goal.reason = "robot is outside local map";
        return false;
    }

    vec_E<FrontierCell> frontier_cells;
    FrontierSearchStats search_stats;
    if (!collectFrontierCells(robot_pos, frontier_cells, search_stats)) {
        goal.reason = "frontier search failed";
        return false;
    }
    if (frontier_cells.empty()) {
        if (!mapObservationReady(search_stats)) {
            if (mission_manager_ != nullptr) {
                mission_manager_->updateNoFrontier(robot_pos, stamp, false);
            }
            exploration_finished_ = false;
            std::ostringstream oss;
            oss << "map observation not ready"
                << " source=" << search_stats.source
                << " searched=" << search_stats.searched_cells
                << " free=" << search_stats.known_free_cells
                << " unknown=" << search_stats.unknown_cells
                << " occupied=" << search_stats.occupied_cells;
            goal.reason = oss.str();
            if (cfg_.print_log) {
                std::cout << " -- [ExplorationFrontend] Waiting for usable map: "
                          << goal.reason << "." << std::endl;
            }
            return false;
        }
        if (mission_manager_ != nullptr) {
            mission_manager_->updateNoFrontier(robot_pos, stamp, true);
            exploration_finished_ = mission_manager_->isFinished();
            goal.reason = exploration_finished_
                                  ? mission_manager_->finishReason()
                                  : "no local frontier inside mission box";
        } else {
            exploration_finished_ = true;
            goal.reason = "no frontier";
        }
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] "
                      << (exploration_finished_ ? "Exploration finished" : "No local frontier")
                      << ": " << goal.reason << ". "
                      << "source=" << search_stats.source
                      << ", raw_frontiers=" << search_stats.raw_frontier_cells
                      << ", fallback=" << static_cast<int>(search_stats.fallback_used)
                      << ", "
                      << "searched=" << search_stats.searched_cells
                      << ", free=" << search_stats.known_free_cells
                      << ", unknown=" << search_stats.unknown_cells
                      << ", occupied=" << search_stats.occupied_cells
                      << "." << std::endl;
        }
        return false;
    }

    vec_E<FrontierCluster> clusters;
    clusterFrontiers(frontier_cells, clusters);
    if (clusters.empty()) {
        exploration_finished_ = false;
        goal.reason = "no frontier cluster";
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No frontier cluster yet. "
                      << "source=" << search_stats.source
                      << ", raw_frontiers=" << search_stats.raw_frontier_cells
                      << ", fallback=" << static_cast<int>(search_stats.fallback_used)
                      << ", "
                      << "frontiers=" << frontier_cells.size()
                      << ", min_cluster_size=" << cfg_.min_frontier_cluster_size
                      << "." << std::endl;
        }
        return false;
    }

    const size_t raw_cluster_count = clusters.size();
    vec_E<FrontierCluster> split_clusters;
    splitLargeFrontierClusters(clusters, robot_pos, split_clusters);
    if (!split_clusters.empty()) {
        clusters.swap(split_clusters);
    }

    FrontierObjectStats object_stats;
    if (frontier_object_manager_ != nullptr) {
        vec_E<FrontierCluster> managed_clusters;
        frontier_object_manager_->update(clusters,
                                         robot_pos,
                                         stamp,
                                         managed_clusters,
                                         object_stats);
        clusters.swap(managed_clusters);
    } else {
        object_stats.observed = static_cast<int>(clusters.size());
        object_stats.active = static_cast<int>(clusters.size());
        object_stats.records = static_cast<int>(clusters.size());
    }

    std::sort(clusters.begin(), clusters.end(),
              [this, &robot_pos](const FrontierCluster &lhs, const FrontierCluster &rhs) {
                  const double lhs_score = clusterPriorityScore(lhs, robot_pos);
                  const double rhs_score = clusterPriorityScore(rhs, robot_pos);
                  if (std::abs(lhs_score - rhs_score) > 1.0e-6) {
                      return lhs_score < rhs_score;
                  }
                  return lhs.size > rhs.size;
              });
    const int max_cluster_count = cfg_.max_frontier_clusters > 0
                                          ? cfg_.max_frontier_clusters
                                          : std::numeric_limits<int>::max();
    if (clusters.size() > static_cast<size_t>(max_cluster_count)) {
        clusters.resize(static_cast<size_t>(max_cluster_count));
    }

    vec_E<ExplorationGoal> candidates;
    const int max_candidate_num = std::max(1, cfg_.max_candidate_num);
    const int per_cluster_keep =
            cfg_.max_candidates_per_frontier_cluster > 0
                    ? std::max(1, cfg_.max_candidates_per_frontier_cluster)
                    : std::max(1, max_candidate_num /
                                      std::max(1, static_cast<int>(clusters.size())));
    const int per_cluster_sample_budget =
            std::min(max_candidate_num, std::max(per_cluster_keep, per_cluster_keep * 4));
    for (const auto &cluster : clusters) {
        vec_E<ExplorationGoal> cluster_candidates;
        sampleViewpointsForCluster(cluster,
                                   robot_state,
                                   current_yaw,
                                   cluster_candidates,
                                   per_cluster_sample_budget);
        if (cluster_candidates.empty() && frontier_object_manager_ != nullptr) {
            frontier_object_manager_->recordNoView(cluster.object_id, stamp);
        }
        std::sort(cluster_candidates.begin(), cluster_candidates.end(),
                  [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                      return lhs.score < rhs.score;
                  });
        int kept_from_cluster = 0;
        for (const ExplorationGoal &candidate : cluster_candidates) {
            if (static_cast<int>(candidates.size()) >= max_candidate_num ||
                kept_from_cluster >= per_cluster_keep) {
                break;
            }
            candidates.push_back(candidate);
            ++kept_from_cluster;
        }
        if (static_cast<int>(candidates.size()) >= max_candidate_num) {
            break;
        }
    }

    if (mission_manager_ != nullptr) {
        mission_manager_->filterAndScoreCandidates(robot_pos, stamp, candidates);
    }
    diversifyCandidates(candidates);

    if (candidates.empty()) {
        exploration_finished_ =
                mission_manager_ != nullptr && mission_manager_->isFinished();
        goal.reason = exploration_finished_
                              ? mission_manager_->finishReason()
                              : "no valid viewpoint";
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No valid viewpoint from "
                      << clusters.size() << " clusters"
                      << ", finished=" << static_cast<int>(exploration_finished_)
                      << " (raw_clusters=" << raw_cluster_count
                      << ", objects=" << object_stats.records
                      << ", dormant=" << object_stats.dormant
                      << ", covered=" << object_stats.covered
                      << ", stale=" << object_stats.stale << ")." << std::endl;
        }
        return false;
    }

    vec_E<ExplorationGoal> candidate_pool = candidates;
    bool using_distance_filtered_pool = false;
    if (cfg_.min_goal_distance > 0.0) {
        vec_E<ExplorationGoal> distant_candidates;
        distant_candidates.reserve(candidates.size());
        for (const ExplorationGoal &candidate : candidates) {
            if (candidate.distance_to_robot >= cfg_.min_goal_distance) {
                distant_candidates.push_back(candidate);
            }
        }
        if (!distant_candidates.empty()) {
            candidate_pool.swap(distant_candidates);
            using_distance_filtered_pool = true;
        }
    }

    vec_E<ExplorationGoal> reachable_candidates;
    const int max_candidate_checks = std::max(1, cfg_.max_candidate_num);
    const int configured_max_astar_checks =
            cfg_.max_astar_checks > 0 ? cfg_.max_astar_checks : cfg_.max_candidate_num;
    const int max_astar_checks = std::max(0, configured_max_astar_checks);
    const int max_reachable_candidates =
            cfg_.max_reachable_candidate_num > 0
                    ? std::max(1, cfg_.max_reachable_candidate_num)
                    : std::max(1, cfg_.max_candidate_num);
    const int min_direct_reachable_before_astar =
            std::max(0, cfg_.min_direct_reachable_before_astar);
    const int max_astar_checks_per_frontier =
            cfg_.max_astar_checks_per_frontier > 0
                    ? cfg_.max_astar_checks_per_frontier
                    : max_astar_checks;
    int checked = 0;
    int astar_checked = 0;

    const auto prune_time = std::chrono::steady_clock::now();
    for (auto it = astar_failure_cache_.begin(); it != astar_failure_cache_.end();) {
        if (it->second.expires_at <= prune_time) {
            it = astar_failure_cache_.erase(it);
        } else {
            ++it;
        }
    }

    const auto astar_candidate_cache_key = [](const ExplorationGoal &candidate) {
        return candidate.memory_key.empty()
                       ? std::to_string(candidate.candidate_id)
                       : candidate.memory_key;
    };
    const auto astar_frontier_cache_key = [](const ExplorationGoal &candidate) {
        return std::string("frontier:") + std::to_string(candidate.frontier_id);
    };
    const auto astar_key_blocked = [&](const std::string &key) {
        const auto it = astar_failure_cache_.find(key);
        return it != astar_failure_cache_.end() &&
               it->second.expires_at > std::chrono::steady_clock::now();
    };
    const auto astar_temporarily_blocked = [&](const ExplorationGoal &candidate) {
        if (cfg_.astar_failure_cache_ttl <= 0.0) {
            return false;
        }
        return astar_key_blocked(astar_candidate_cache_key(candidate)) ||
               astar_key_blocked(astar_frontier_cache_key(candidate));
    };
    const auto record_astar_failure = [&](const ExplorationGoal &candidate) {
        if (cfg_.astar_failure_cache_ttl <= 0.0) {
            return;
        }
        const auto record_key = [&](const std::string &key) {
            auto &entry = astar_failure_cache_[key];
            ++entry.failure_count;
            entry.expires_at =
                    std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(cfg_.astar_failure_cache_ttl));
        };
        record_key(astar_candidate_cache_key(candidate));
        record_key(astar_frontier_cache_key(candidate));
    };

    const auto build_reachable_candidates = [&](vec_E<ExplorationGoal> pool,
                                                int &checked_count,
                                                int &astar_checked_count,
                                                vec_E<ExplorationGoal> &reachable) {
        reachable.clear();
        checked_count = 0;
        astar_checked_count = 0;
        const auto astar_budget_start = std::chrono::steady_clock::now();
        const auto astar_elapsed_ms = [&]() {
            return std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - astar_budget_start)
                    .count();
        };
        const auto astar_budget_exhausted = [&]() {
            return cfg_.astar_total_time_budget_ms > 0.0 &&
                   astar_elapsed_ms() >= cfg_.astar_total_time_budget_ms;
        };
        const auto astar_budget_has_room = [&]() {
            if (cfg_.astar_total_time_budget_ms <= 0.0) {
                return true;
            }
            if (astar_checked_count == 0) {
                return true;
            }
            const double expected_ms = 1000.0 * std::max(0.005, cfg_.astar_time_out);
            return astar_elapsed_ms() + expected_ms <= cfg_.astar_total_time_budget_ms;
        };
        std::sort(pool.begin(), pool.end(),
                  [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                      return lhs.score < rhs.score;
                  });

        std::vector<int> astar_candidate_indices;
        std::unordered_map<int, int> astar_checks_by_frontier;
        const auto add_reachable_candidate = [&](ExplorationGoal candidate,
                                                 const double travel_cost,
                                                 const vec_E<Vec3f> &guide_path) {
            candidate.travel_cost = travel_cost;
            candidate.distance_to_robot = (candidate.position - robot_pos).norm();
            candidate.guide_path = guide_path;
            const double unknown_risk = estimateUnknownRisk(candidate.position);
            candidate.score = scoreCandidate(candidate, unknown_risk) +
                              candidate.history_score_delta;
            candidate.reason = "selected reachable frontier viewpoint";
            reachable.push_back(candidate);
        };

        const int direct_candidate_num =
                std::min(static_cast<int>(pool.size()), max_candidate_checks);
        for (int i = 0; i < direct_candidate_num; ++i) {
            if (static_cast<int>(reachable.size()) >= max_reachable_candidates) {
                break;
            }
            ++checked_count;
            ExplorationGoal candidate = pool[static_cast<size_t>(i)];
            if (candidate.information_gain < cfg_.min_information_gain) {
                continue;
            }

            vec_E<Vec3f> guide_path;
            const double travel_cost = estimateTravelCost(robot_pos,
                                                          candidate.position,
                                                          guide_path,
                                                          false);
            if (std::isfinite(travel_cost) && travel_cost < kInfCost) {
                add_reachable_candidate(candidate, travel_cost, guide_path);
            } else if (cfg_.use_astar_cost && astar_ != nullptr) {
                if (astar_temporarily_blocked(candidate)) {
                    continue;
                }
                astar_candidate_indices.push_back(i);
            }
        }

        if (!cfg_.use_astar_cost ||
            astar_ == nullptr ||
            (min_direct_reachable_before_astar > 0 &&
             static_cast<int>(reachable.size()) >= min_direct_reachable_before_astar)) {
            return;
        }

        for (const int candidate_index : astar_candidate_indices) {
            if (astar_checked_count >= max_astar_checks ||
                static_cast<int>(reachable.size()) >= max_reachable_candidates ||
                astar_budget_exhausted() ||
                !astar_budget_has_room()) {
                break;
            }
            ExplorationGoal candidate = pool[static_cast<size_t>(candidate_index)];
            if (candidate.information_gain < cfg_.min_information_gain) {
                continue;
            }
            if (astar_temporarily_blocked(candidate)) {
                continue;
            }
            if (max_astar_checks_per_frontier > 0 &&
                astar_checks_by_frontier[candidate.frontier_id] >=
                        max_astar_checks_per_frontier) {
                continue;
            }

            vec_E<Vec3f> guide_path;
            bool astar_used = false;
            const double travel_cost = estimateTravelCost(robot_pos,
                                                          candidate.position,
                                                          guide_path,
                                                          true,
                                                          &astar_used);
            if (astar_used) {
                ++astar_checked_count;
                ++astar_checks_by_frontier[candidate.frontier_id];
            }
            if (!std::isfinite(travel_cost) || travel_cost >= kInfCost) {
                if (astar_used) {
                    record_astar_failure(candidate);
                }
                continue;
            }
            astar_failure_cache_.erase(astar_candidate_cache_key(candidate));
            astar_failure_cache_.erase(astar_frontier_cache_key(candidate));
            add_reachable_candidate(candidate, travel_cost, guide_path);
        }
    };

    build_reachable_candidates(candidate_pool, checked, astar_checked, reachable_candidates);
    if (reachable_candidates.empty() && using_distance_filtered_pool) {
        build_reachable_candidates(candidates, checked, astar_checked, reachable_candidates);
    }

    if (reachable_candidates.empty()) {
        exploration_finished_ = false;
        goal.reason = "no reachable frontier";
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No reachable frontier yet. "
                      << "source=" << search_stats.source
                      << ", raw_frontiers=" << search_stats.raw_frontier_cells
                      << ", fallback=" << static_cast<int>(search_stats.fallback_used)
                      << ", "
                      << "frontiers=" << frontier_cells.size()
                      << ", clusters=" << clusters.size()
                  << ", raw_clusters=" << raw_cluster_count
                  << ", candidates=" << candidates.size() << std::endl;
        }
        return false;
    }

    ExplorationGoal best_goal;
    if (cfg_.use_atsp && reachable_candidates.size() > 1) {
        best_goal = selectGoalWithAtsp(robot_pos, current_yaw, reachable_candidates);
    } else {
        best_goal = *std::min_element(reachable_candidates.begin(), reachable_candidates.end(),
                                      [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                                          return lhs.score < rhs.score;
                                      });
    }

    best_goal.valid = true;
    best_goal.checked_candidate_count = checked;
    best_goal.astar_check_count = astar_checked;
    best_goal.reachable_candidate_count = static_cast<int>(reachable_candidates.size());
    best_goal.cluster_count = static_cast<int>(clusters.size());
    best_goal.raw_cluster_count = static_cast<int>(raw_cluster_count);
    best_goal.frontier_cell_count = static_cast<int>(frontier_cells.size());
    best_goal.raw_frontier_cell_count = search_stats.raw_frontier_cells;
    goal = best_goal;
    exploration_finished_ = false;

    if (cfg_.print_log) {
        std::cout << " -- [ExplorationFrontend] Goal selected: p=["
                  << goal.position.transpose() << "], yaw=" << goal.yaw
                  << ", candidate_id=" << goal.candidate_id
                  << ", frontier_id=" << goal.frontier_id
                  << ", key=" << goal.memory_key
                  << ", score=" << goal.score
                  << ", info=" << goal.information_gain
                  << ", area=" << goal.frontier_area
                  << ", visible=" << goal.visible_frontier_cell_count
                  << ", visible_ratio=" << goal.visible_frontier_ratio
                  << ", travel=" << goal.travel_cost
                  << ", yaw_cost=" << goal.yaw_cost
                  << ", curvature=" << goal.curvature_cost
                  << ", checked=" << checked
                  << ", astar_checks=" << astar_checked
                  << ", reachable=" << reachable_candidates.size()
                  << ", clusters=" << clusters.size()
                  << ", raw_clusters=" << raw_cluster_count
                  << ", objects=" << object_stats.records
                  << ", dormant=" << object_stats.dormant
                  << ", covered=" << object_stats.covered
                  << ", stale=" << object_stats.stale
                  << ", frontiers=" << frontier_cells.size()
                  << ", source=" << search_stats.source
                  << ", raw_frontiers=" << search_stats.raw_frontier_cells
                  << ", fallback=" << static_cast<int>(search_stats.fallback_used)
                  << std::endl;
    }
    return true;
}

bool ExplorationFrontend::isExplorationFinished() const {
    return exploration_finished_;
}

void ExplorationFrontend::reset() {
    exploration_finished_ = false;
    astar_failure_cache_.clear();
    if (frontier_object_manager_ != nullptr) {
        frontier_object_manager_->reset();
    }
}

void ExplorationFrontend::setMissionManager(ExplorationManager *manager) {
    mission_manager_ = manager;
}

void ExplorationFrontend::recordGoalCommitted(const ExplorationGoal &goal,
                                              const double stamp,
                                              const bool goal_switched) {
    if (frontier_object_manager_ != nullptr) {
        frontier_object_manager_->recordCommitted(goal, stamp, goal_switched);
    }
}

void ExplorationFrontend::recordGoalFailed(const ExplorationGoal &goal,
                                           const double stamp) {
    if (frontier_object_manager_ != nullptr) {
        frontier_object_manager_->recordFailed(goal, stamp);
    }
}

bool ExplorationFrontend::collectFrontierCells(const Vec3f &robot_pos,
                                               vec_E<FrontierCell> &frontier_cells,
                                               FrontierSearchStats &stats) const {
    frontier_cells.clear();
    stats = FrontierSearchStats{};
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }

    std::string source = core_utils::normalizeToken(cfg_.frontier_source);
    if (source.empty()) {
        source = "fallback_scan";
    }
    FrontierSearchStats rog_stats;
    const bool allow_rog = sourceAllowsRogMap(source);
    const bool allow_fallback = sourceAllowsFallback(source);

    if (allow_rog && map_manager_->getMapConfig().frontier_extraction_en) {
        if (!collectRogMapFrontierCells(robot_pos, frontier_cells, rog_stats)) {
            return false;
        }
        if (!frontier_cells.empty() || !allow_fallback) {
            stats = rog_stats;
            return true;
        }
    }

    if (allow_fallback || !allow_rog) {
        const bool fallback_ok = collectFallbackFrontierCells(robot_pos, frontier_cells, stats);
        stats.fallback_used = allow_rog;
        if (allow_rog) {
            stats.raw_frontier_cells = rog_stats.raw_frontier_cells;
            if (rog_stats.raw_frontier_cells > 0) {
                stats.source = "rog_map_frontier+fallback_scan";
            }
        }
        return fallback_ok;
    }

    return collectFallbackFrontierCells(robot_pos, frontier_cells, stats);
}

bool ExplorationFrontend::collectFallbackFrontierCells(const Vec3f &robot_pos,
                                                       vec_E<FrontierCell> &frontier_cells,
                                                       FrontierSearchStats &stats) const {
    frontier_cells.clear();
    stats = FrontierSearchStats{};
    stats.source = "fallback_scan";
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }

    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    const double sample_res = std::max(map_res, cfg_.map_resolution);
    const int index_step = std::max(1, static_cast<int>(std::round(sample_res / map_res)));

    Vec3f box_min = robot_pos - Vec3f::Constant(cfg_.frontier_search_radius);
    Vec3f box_max = robot_pos + Vec3f::Constant(cfg_.frontier_search_radius);
    map_manager_->boundBoxByLocalMap(box_min, box_max);
    if (!clipTaskSearchBox(box_min, box_max)) {
        return true;
    }
    if ((box_max - box_min).minCoeff() <= 0.0) {
        return false;
    }

    Vec3i min_id;
    Vec3i max_id;
    map_manager_->probMapPosToGlobalIndex(box_min, min_id);
    map_manager_->probMapPosToGlobalIndex(box_max, max_id);
    for (int axis = 0; axis < 3; ++axis) {
        if (min_id(axis) > max_id(axis)) {
            std::swap(min_id(axis), max_id(axis));
        }
    }

    const double radius_sq = cfg_.frontier_search_radius * cfg_.frontier_search_radius;
    std::unordered_set<GridKey, GridKeyHasher> inserted;
    for (int ix = min_id.x(); ix <= max_id.x(); ix += index_step) {
        for (int iy = min_id.y(); iy <= max_id.y(); iy += index_step) {
            for (int iz = min_id.z(); iz <= max_id.z(); iz += index_step) {
                FrontierCell cell;
                cell.index = Vec3i(ix, iy, iz);
                map_manager_->probMapGlobalIndexToPos(cell.index, cell.position);
                if ((cell.position - robot_pos).squaredNorm() > radius_sq ||
                    !map_manager_->insideLocalMap(cell.position) ||
                    !insideTaskRegion(cell.position)) {
                    continue;
                }
                ++stats.searched_cells;
                if (inserted.find(makeKey(cell.index)) != inserted.end()) {
                    continue;
                }
                const rog_map::GridType grid_type = map_manager_->getGridType(cell.position);
                if (isFreeLike(grid_type)) {
                    ++stats.known_free_cells;
                } else if (isUnknownLike(grid_type)) {
                    ++stats.unknown_cells;
                } else if (grid_type == rog_map::GridType::OCCUPIED) {
                    ++stats.occupied_cells;
                }

                if (isFrontierCell(cell, grid_type)) {
                    inserted.insert(makeKey(cell.index));
                    ++stats.frontier_cells;
                    frontier_cells.push_back(cell);
                }
            }
        }
    }
    return true;
}

bool ExplorationFrontend::collectRogMapFrontierCells(const Vec3f &robot_pos,
                                                     vec_E<FrontierCell> &frontier_cells,
                                                     FrontierSearchStats &stats) const {
    frontier_cells.clear();
    stats = FrontierSearchStats{};
    stats.source = "rog_map_frontier";
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }
    if (!map_manager_->getMapConfig().frontier_extraction_en) {
        return true;
    }

    Vec3f box_min = robot_pos - Vec3f::Constant(cfg_.frontier_search_radius);
    Vec3f box_max = robot_pos + Vec3f::Constant(cfg_.frontier_search_radius);
    map_manager_->boundBoxByLocalMap(box_min, box_max);
    if (!clipTaskSearchBox(box_min, box_max)) {
        return true;
    }
    if ((box_max - box_min).minCoeff() <= 0.0) {
        return false;
    }

    vec_E<Vec3f> rog_frontiers;
    map_manager_->boxSearch(box_min, box_max, rog_map::GridType::FRONTIER, rog_frontiers);
    stats.raw_frontier_cells = static_cast<int>(rog_frontiers.size());
    stats.searched_cells = stats.raw_frontier_cells;

    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    const double sample_res = std::max(map_res, cfg_.frontier_sample_resolution);
    const double radius_sq = cfg_.frontier_search_radius * cfg_.frontier_search_radius;
    const int max_raw_points = cfg_.max_raw_frontier_points > 0
                                       ? cfg_.max_raw_frontier_points
                                       : std::numeric_limits<int>::max();
    const int max_frontier_cells = cfg_.max_frontier_cells > 0
                                           ? cfg_.max_frontier_cells
                                           : std::numeric_limits<int>::max();

    struct SampledPoint {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vec3f position{Vec3f::Zero()};
        double distance_sq{std::numeric_limits<double>::infinity()};
    };
    struct SampledCell {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        FrontierCell cell;
        double distance_sq{std::numeric_limits<double>::infinity()};
    };

    std::unordered_map<GridKey, SampledPoint, GridKeyHasher> sampled_rog_frontiers;
    sampled_rog_frontiers.reserve(std::min(rog_frontiers.size(),
                                           static_cast<size_t>(std::max(1, max_raw_points))));

    for (const Vec3f &frontier_pos : rog_frontiers) {
        if (!frontier_pos.allFinite() ||
            (frontier_pos - robot_pos).squaredNorm() > radius_sq ||
            !map_manager_->insideLocalMap(frontier_pos) ||
            !insideTaskRegion(frontier_pos)) {
            continue;
        }
        ++stats.unknown_cells;
        const double distance_sq = (frontier_pos - robot_pos).squaredNorm();
        const GridKey sample_key = makeBucketKey(frontier_pos, sample_res);
        const auto it = sampled_rog_frontiers.find(sample_key);
        if (it == sampled_rog_frontiers.end()) {
            SampledPoint sampled;
            sampled.position = frontier_pos;
            sampled.distance_sq = distance_sq;
            sampled_rog_frontiers.emplace(sample_key, sampled);
        } else if (distance_sq < it->second.distance_sq) {
            it->second.position = frontier_pos;
            it->second.distance_sq = distance_sq;
        }
    }

    std::vector<SampledPoint> sampled_points;
    sampled_points.reserve(sampled_rog_frontiers.size());
    for (const auto &entry : sampled_rog_frontiers) {
        sampled_points.push_back(entry.second);
    }
    std::sort(sampled_points.begin(), sampled_points.end(),
              [](const SampledPoint &lhs, const SampledPoint &rhs) {
                  return lhs.distance_sq > rhs.distance_sq;
              });
    if (sampled_points.size() > static_cast<size_t>(max_raw_points)) {
        sampled_points.resize(static_cast<size_t>(max_raw_points));
    }

    std::unordered_map<GridKey, SampledCell, GridKeyHasher> sampled_cells;
    sampled_cells.reserve(std::min(static_cast<size_t>(std::max(1, max_frontier_cells)),
                                   sampled_points.size() * 4U + 1U));

    for (const SampledPoint &sampled_frontier : sampled_points) {
        const Vec3f &frontier_pos = sampled_frontier.position;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }

                    const Vec3f free_side_pos =
                            frontier_pos + map_res * Vec3f(dx, dy, dz);
                    if (!map_manager_->insideLocalMap(free_side_pos) ||
                        (free_side_pos - robot_pos).squaredNorm() > radius_sq ||
                        !insideTaskRegion(free_side_pos)) {
                        continue;
                    }

                    FrontierCell cell;
                    map_manager_->probMapPosToGlobalIndex(free_side_pos, cell.index);
                    map_manager_->probMapGlobalIndexToPos(cell.index, cell.position);
                    if (!map_manager_->insideLocalMap(cell.position) ||
                        !insideTaskRegion(cell.position)) {
                        continue;
                    }

                    const rog_map::GridType grid_type = map_manager_->getGridType(cell.position);
                    if (isFreeLike(grid_type)) {
                        ++stats.known_free_cells;
                    } else if (isUnknownLike(grid_type)) {
                        ++stats.unknown_cells;
                    } else if (grid_type == rog_map::GridType::OCCUPIED) {
                        ++stats.occupied_cells;
                    }

                    if (!isFrontierCell(cell, grid_type)) {
                        continue;
                    }

                    const double distance_sq = (cell.position - robot_pos).squaredNorm();
                    const GridKey cell_key = makeBucketKey(cell.position, sample_res);
                    const auto it = sampled_cells.find(cell_key);
                    if (it == sampled_cells.end()) {
                        SampledCell sampled_cell;
                        sampled_cell.cell = cell;
                        sampled_cell.distance_sq = distance_sq;
                        sampled_cells.emplace(cell_key, sampled_cell);
                    } else if (distance_sq < it->second.distance_sq) {
                        it->second.cell = cell;
                        it->second.distance_sq = distance_sq;
                    }
                }
            }
        }
    }

    frontier_cells.reserve(sampled_cells.size());
    for (const auto &entry : sampled_cells) {
        frontier_cells.push_back(entry.second.cell);
    }
    std::sort(frontier_cells.begin(), frontier_cells.end(),
              [&robot_pos](const FrontierCell &lhs, const FrontierCell &rhs) {
                  return (lhs.position - robot_pos).squaredNorm() >
                         (rhs.position - robot_pos).squaredNorm();
              });
    if (frontier_cells.size() > static_cast<size_t>(max_frontier_cells)) {
        frontier_cells.resize(static_cast<size_t>(max_frontier_cells));
    }
    stats.frontier_cells = static_cast<int>(frontier_cells.size());
    return true;
}

bool ExplorationFrontend::isFrontierCell(const FrontierCell &cell,
                                         const rog_map::GridType grid_type) const {
    if (!isFreeLike(grid_type)) {
        return false;
    }
    const rog_map::GridType inf_type = map_manager_->getInfGridType(cell.position);
    if (inf_type == rog_map::GridType::OCCUPIED ||
        inf_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }

    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const Vec3f neighbor_pos =
                        cell.position + map_res * Vec3f(dx, dy, dz);
                if (!map_manager_->insideLocalMap(neighbor_pos) ||
                    !insideTaskRegion(neighbor_pos)) {
                    continue;
                }
                if (isUnknownLike(map_manager_->getGridType(neighbor_pos))) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ExplorationFrontend::mapObservationReady(const FrontierSearchStats &stats) const {
    const int min_known_free_cells = std::max(10, cfg_.min_frontier_cluster_size);
    return stats.known_free_cells >= min_known_free_cells;
}

void ExplorationFrontend::clusterFrontiers(const vec_E<FrontierCell> &frontier_cells,
                                           vec_E<FrontierCluster> &clusters) const {
    clusters.clear();
    if (frontier_cells.empty()) {
        return;
    }

    const double radius = std::max(map_manager_->getResolution(), cfg_.frontier_cluster_radius);
    const double radius_sq = radius * radius;
    std::unordered_map<GridKey, std::vector<int>, GridKeyHasher> buckets;
    buckets.reserve(frontier_cells.size());
    for (int i = 0; i < static_cast<int>(frontier_cells.size()); ++i) {
        buckets[makeBucketKey(frontier_cells[static_cast<size_t>(i)].position, radius)].push_back(i);
    }

    std::vector<char> visited(frontier_cells.size(), 0);
    std::queue<int> q;
    for (int seed = 0; seed < static_cast<int>(frontier_cells.size()); ++seed) {
        if (visited[static_cast<size_t>(seed)] != 0) {
            continue;
        }

        FrontierCluster cluster;
        visited[static_cast<size_t>(seed)] = 1;
        q.push(seed);
        while (!q.empty()) {
            const int current = q.front();
            q.pop();
            const FrontierCell &current_cell = frontier_cells[static_cast<size_t>(current)];
            cluster.cells.push_back(current_cell);

            const GridKey bucket = makeBucketKey(current_cell.position, radius);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const GridKey neighbor_bucket{bucket.x + dx, bucket.y + dy, bucket.z + dz};
                        const auto it = buckets.find(neighbor_bucket);
                        if (it == buckets.end()) {
                            continue;
                        }
                        for (const int neighbor_id : it->second) {
                            if (visited[static_cast<size_t>(neighbor_id)] != 0) {
                                continue;
                            }
                            const FrontierCell &neighbor =
                                    frontier_cells[static_cast<size_t>(neighbor_id)];
                            if ((neighbor.position - current_cell.position).squaredNorm() > radius_sq) {
                                continue;
                            }
                            visited[static_cast<size_t>(neighbor_id)] = 1;
                            q.push(neighbor_id);
                        }
                    }
                }
            }
        }

        if (static_cast<int>(cluster.cells.size()) < cfg_.min_frontier_cluster_size) {
            continue;
        }

        if (!finalizeFrontierCluster(cluster)) {
            continue;
        }
        clusters.push_back(cluster);
    }
}

bool ExplorationFrontend::finalizeFrontierCluster(FrontierCluster &cluster) const {
    if (cluster.cells.empty()) {
        return false;
    }
    if (static_cast<int>(cluster.cells.size()) < cfg_.min_frontier_cluster_size) {
        return false;
    }

    cluster.size = static_cast<int>(cluster.cells.size());
    cluster.center.setZero();
    cluster.bbox_min = cluster.cells.front().position;
    cluster.bbox_max = cluster.cells.front().position;
    cluster.extent.setZero();
    cluster.area = 0.0;
    cluster.unknown_neighbor_count = 0;
    cluster.unknown_direction = Vec3f::UnitX();
    cluster.normal = Vec3f::UnitX();

    Vec3f unknown_dir = Vec3f::Zero();
    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    for (const FrontierCell &cell : cluster.cells) {
        cluster.center += cell.position;
        cluster.bbox_min = cluster.bbox_min.cwiseMin(cell.position);
        cluster.bbox_max = cluster.bbox_max.cwiseMax(cell.position);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const Vec3f neighbor_pos = cell.position + map_res * Vec3f(dx, dy, dz);
                    if (map_manager_->insideLocalMap(neighbor_pos) &&
                        insideTaskRegion(neighbor_pos) &&
                        isUnknownLike(map_manager_->getGridType(neighbor_pos))) {
                        unknown_dir += neighbor_pos - cell.position;
                        ++cluster.unknown_neighbor_count;
                    }
                }
            }
        }
    }

    cluster.center /= static_cast<double>(cluster.size);
    cluster.extent = cluster.bbox_max - cluster.bbox_min;
    cluster.area = map_res * map_res * static_cast<double>(cluster.size);
    if (unknown_dir.norm() > 1.0e-6) {
        cluster.unknown_direction = unknown_dir.normalized();
    }

    if (cluster.size >= 3) {
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const FrontierCell &cell : cluster.cells) {
            const Eigen::Vector3d delta = (cell.position - cluster.center).cast<double>();
            covariance += delta * delta.transpose();
        }
        covariance /= static_cast<double>(cluster.size);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() == Eigen::Success) {
            Vec3f normal = solver.eigenvectors().col(0);
            if (normal.dot(cluster.unknown_direction) < 0.0) {
                normal = -normal;
            }
            if (normal.norm() > 1.0e-6) {
                cluster.normal = normal.normalized();
            }
        }
    }

    cluster.free_direction = horizontalNormalized(-cluster.unknown_direction,
                                                  -cluster.normal);
    return frontierClusterValid(cluster);
}

void ExplorationFrontend::splitLargeFrontierClusters(
        const vec_E<FrontierCluster> &clusters,
        const Vec3f &robot_pos,
        vec_E<FrontierCluster> &split_clusters) const {
    split_clusters.clear();
    const double split_size = cfg_.frontier_subcluster_size;
    const int max_subclusters = cfg_.max_subclusters_per_cluster > 0
                                        ? cfg_.max_subclusters_per_cluster
                                        : std::numeric_limits<int>::max();
    if (split_size <= std::max(0.5, map_manager_->getResolution())) {
        split_clusters = clusters;
        return;
    }

    for (const FrontierCluster &cluster : clusters) {
        const bool large_extent = cluster.extent.x() > split_size ||
                                  cluster.extent.y() > split_size ||
                                  cluster.extent.z() > split_size;
        const bool large_cell_count =
                cluster.size >= std::max(2 * cfg_.min_frontier_cluster_size,
                                         cfg_.min_frontier_cluster_size + 12);
        if (!large_extent || !large_cell_count) {
            split_clusters.push_back(cluster);
            continue;
        }

        std::unordered_map<GridKey, FrontierCluster, GridKeyHasher> buckets;
        buckets.reserve(cluster.cells.size());
        for (const FrontierCell &cell : cluster.cells) {
            buckets[makeBucketKey(cell.position, split_size)].cells.push_back(cell);
        }

        vec_E<FrontierCluster> patches;
        patches.reserve(buckets.size());
        for (auto &entry : buckets) {
            FrontierCluster patch = std::move(entry.second);
            if (finalizeFrontierCluster(patch)) {
                patches.push_back(std::move(patch));
            }
        }

        if (patches.empty()) {
            split_clusters.push_back(cluster);
            continue;
        }
        std::sort(patches.begin(), patches.end(),
                  [this, &robot_pos](const FrontierCluster &lhs, const FrontierCluster &rhs) {
                      const double lhs_score = clusterPriorityScore(lhs, robot_pos);
                      const double rhs_score = clusterPriorityScore(rhs, robot_pos);
                      if (std::abs(lhs_score - rhs_score) > 1.0e-6) {
                          return lhs_score < rhs_score;
                      }
                      return lhs.size > rhs.size;
                  });
        if (patches.size() > static_cast<size_t>(max_subclusters)) {
            patches.resize(static_cast<size_t>(max_subclusters));
        }
        split_clusters.insert(split_clusters.end(), patches.begin(), patches.end());
    }
}

double ExplorationFrontend::clusterPriorityScore(const FrontierCluster &cluster,
                                                 const Vec3f &robot_pos) const {
    const double distance = std::max(0.0, (cluster.center - robot_pos).norm());
    const double preferred_distance = std::max(cfg_.min_goal_distance,
                                               cfg_.preferred_goal_distance);
    const double distance_reward = preferred_distance > 1.0e-3
                                           ? std::min(distance, preferred_distance)
                                           : distance;
    const double near_shortfall = std::max(0.0, cfg_.min_goal_distance - distance);
    const double area_reward = std::sqrt(std::max(0.0, cluster.area));
    const double unknown_reward =
            std::min(600.0, static_cast<double>(std::max(0, cluster.unknown_neighbor_count)));
    return 3.0 * near_shortfall * near_shortfall -
           1.4 * distance_reward -
           0.6 * area_reward -
           0.01 * unknown_reward;
}

bool ExplorationFrontend::frontierClusterValid(const FrontierCluster &cluster) const {
    if (cluster.size < cfg_.min_frontier_cluster_size) {
        return false;
    }
    if (cfg_.min_frontier_area > 0.0 && cluster.area < cfg_.min_frontier_area) {
        return false;
    }
    if (cfg_.min_frontier_extent > 0.0 &&
        cluster.extent.maxCoeff() < cfg_.min_frontier_extent) {
        return false;
    }
    if (cfg_.min_unknown_neighbor_count > 0 &&
        cluster.unknown_neighbor_count < cfg_.min_unknown_neighbor_count) {
        return false;
    }
    return true;
}

void ExplorationFrontend::sampleViewpointsForCluster(const FrontierCluster &cluster,
                                                     const StatePVAJ &robot_state,
                                                     const double current_yaw,
                                                     vec_E<ExplorationGoal> &candidates,
                                                     const int candidate_budget) const {
    const int max_candidate_num = std::max(1, candidate_budget);
    const int yaw_num = std::max(1, cfg_.viewpoint_yaw_sample_num);
    const int radius_num = std::max(1, cfg_.viewpoint_radius_sample_num);
    const int z_num = std::max(1, cfg_.viewpoint_z_sample_num);
    const double min_radius = std::max(0.0, cfg_.viewpoint_min_distance);
    const double max_radius = std::max(min_radius, cfg_.viewpoint_max_distance);
    const Vec3f robot_pos = robot_state.col(0);
    const Vec3f base_direction = horizontalNormalized(cluster.free_direction,
                                                      robot_pos - cluster.center);
    const double base_angle = std::atan2(base_direction.y(), base_direction.x());
    const double normal_angle = std::clamp(cfg_.viewpoint_normal_angle, 0.0, kPi);
    const int visible_threshold = visibleFrontierThreshold(cluster);

    for (int ri = 0; ri < radius_num; ++ri) {
        const double alpha = radius_num == 1
                                     ? 0.0
                                     : static_cast<double>(ri) / static_cast<double>(radius_num - 1);
        const double radius = min_radius + alpha * (max_radius - min_radius);
        for (int zi = 0; zi < z_num; ++zi) {
            const double z_alpha = z_num == 1
                                           ? 0.5
                                           : static_cast<double>(zi) / static_cast<double>(z_num - 1);
            const double z_offset = cfg_.viewpoint_z_min +
                                    z_alpha * (cfg_.viewpoint_z_max - cfg_.viewpoint_z_min);
            for (int yi = 0; yi < yaw_num; ++yi) {
                if (static_cast<int>(candidates.size()) >= max_candidate_num) {
                    return;
                }
                double angle = 0.0;
                if (cfg_.viewpoint_use_normal_sampling) {
                    const double yaw_alpha = yaw_num == 1
                                                     ? 0.5
                                                     : static_cast<double>(yi) /
                                                               static_cast<double>(yaw_num - 1);
                    angle = base_angle - normal_angle + 2.0 * normal_angle * yaw_alpha;
                } else {
                    angle = 2.0 * kPi * static_cast<double>(yi) /
                            static_cast<double>(yaw_num);
                }

                Vec3f viewpoint =
                        cluster.center + radius * Vec3f(std::cos(angle), std::sin(angle), 0.0);
                viewpoint.z() = cluster.center.z() + cfg_.viewpoint_height_offset + z_offset;
                if (!insideTaskRegion(viewpoint)) {
                    continue;
                }

                const double distance_to_robot = (viewpoint - robot_pos).norm();
                if (distance_to_robot <= std::max(0.2, cfg_.goal_reached_distance)) {
                    continue;
                }
                if (!viewpointSafe(viewpoint)) {
                    continue;
                }
                if (cfg_.require_line_free_to_frontier &&
                    !map_manager_->isLineFree(viewpoint, cluster.center, true, false)) {
                    continue;
                }

                const int visible_count =
                        countVisibleFrontierCells(viewpoint, cluster, cfg_.max_gain_rays);
                if (visible_count < visible_threshold) {
                    continue;
                }

                const double information_gain = estimateInformationGain(viewpoint, cluster);
                if (information_gain < cfg_.min_information_gain) {
                    continue;
                }

                ExplorationGoal candidate;
                candidate.valid = true;
                candidate.position = viewpoint;
                candidate.yaw = resolveCandidateYaw(current_yaw,
                                                    robot_pos,
                                                    viewpoint,
                                                    cluster);
                const GridKey frontier_key =
                        quantizedKey(cluster.center, std::max(0.25, cfg_.frontier_cluster_radius));
                const GridKey candidate_key =
                        quantizedKey(viewpoint, std::max(0.25, cfg_.map_resolution));
                candidate.frontier_id =
                        cluster.object_id >= 0 ? cluster.object_id : stableIdFromKey(frontier_key);
                candidate.candidate_id = stableIdFromKey(candidate_key);
                candidate.memory_key = memoryKeyFromKey(candidate_key);
                candidate.frontier_center_valid = true;
                candidate.frontier_center = cluster.center;
                candidate.frontier_bbox_min = cluster.bbox_min;
                candidate.frontier_bbox_max = cluster.bbox_max;
                candidate.information_gain = information_gain;
                candidate.frontier_area = cluster.area;
                candidate.visible_frontier_cell_count = visible_count;
                const int visibility_denominator =
                        sampledCount(cluster.cells.size(), std::max(1, cfg_.max_gain_rays));
                candidate.visible_frontier_ratio =
                        visibility_denominator <= 0
                                ? 0.0
                                : static_cast<double>(visible_count) /
                                          static_cast<double>(visibility_denominator);
                candidate.distance_to_robot = distance_to_robot;
                candidate.travel_cost = distance_to_robot;
                candidate.yaw_cost = estimateYawCost(current_yaw, candidate.yaw);
                candidate.curvature_cost = estimateCurvatureCost(robot_state, viewpoint, cluster);
                candidate.history_score_delta = cluster.object_score_delta;
                const double unknown_risk = estimateUnknownRisk(viewpoint);
                candidate.score = scoreCandidate(candidate, unknown_risk) +
                                  candidate.history_score_delta;
                candidate.reason = "sampled semantic frontier viewpoint";
                if (cluster.object_id >= 0) {
                    candidate.reason += " frontier_object_seen=" +
                                        std::to_string(cluster.object_seen_count) +
                                        " frontier_object_selected=" +
                                        std::to_string(cluster.object_selection_count) +
                                        " frontier_object_failed=" +
                                        std::to_string(cluster.object_failure_count);
                }
                candidates.push_back(candidate);
            }
        }
    }
}

void ExplorationFrontend::diversifyCandidates(vec_E<ExplorationGoal> &candidates) const {
    if (candidates.size() <= 1U) {
        return;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                  return lhs.score < rhs.score;
              });

    const int max_candidate_num = std::max(1, cfg_.max_candidate_num);
    const int max_per_frontier =
            cfg_.max_candidates_per_frontier_cluster > 0
                    ? std::max(1, cfg_.max_candidates_per_frontier_cluster)
                    : max_candidate_num;
    const double separation = std::max(0.0, cfg_.candidate_separation_distance);
    const double separation_sq = separation * separation;

    std::unordered_map<int, int> kept_by_frontier;
    vec_E<ExplorationGoal> diversified;
    diversified.reserve(std::min(candidates.size(),
                                 static_cast<size_t>(max_candidate_num)));

    const auto separated_from_kept = [&](const ExplorationGoal &candidate) {
        if (separation <= 1.0e-6) {
            return true;
        }
        for (const ExplorationGoal &kept : diversified) {
            if ((candidate.position - kept.position).squaredNorm() < separation_sq &&
                candidate.frontier_id == kept.frontier_id) {
                return false;
            }
        }
        return true;
    };

    for (const ExplorationGoal &candidate : candidates) {
        if (static_cast<int>(diversified.size()) >= max_candidate_num) {
            break;
        }
        if (kept_by_frontier[candidate.frontier_id] >= max_per_frontier) {
            continue;
        }
        if (!separated_from_kept(candidate)) {
            continue;
        }
        diversified.push_back(candidate);
        ++kept_by_frontier[candidate.frontier_id];
    }

    if (diversified.empty()) {
        diversified.push_back(candidates.front());
    }
    candidates.swap(diversified);
}

bool ExplorationFrontend::viewpointSafe(const Vec3f &viewpoint) const {
    if (!viewpoint.allFinite() ||
        map_manager_ == nullptr ||
        !map_manager_->ready() ||
        !map_manager_->insideLocalMap(viewpoint) ||
        !insideTaskRegion(viewpoint)) {
        return false;
    }

    const rog_map::GridType grid_type = map_manager_->getGridType(viewpoint);
    if (grid_type == rog_map::GridType::OCCUPIED ||
        grid_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }
    if (cfg_.unknown_as_occupied_for_motion && !isFreeLike(grid_type)) {
        return false;
    }

    const rog_map::GridType inf_type = map_manager_->getInfGridType(viewpoint);
    if (inf_type == rog_map::GridType::OCCUPIED ||
        inf_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }

    if (map_manager_->hasESDF() && cfg_.viewpoint_safe_distance > 0.0) {
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        if (!map_manager_->evaluateESDF(viewpoint, dist, grad) ||
            !std::isfinite(dist) ||
            dist < cfg_.viewpoint_safe_distance) {
            return false;
        }
    }
    return true;
}

bool ExplorationFrontend::viewpointVisible(const Vec3f &viewpoint,
                                           const FrontierCluster &cluster) const {
    if (!insideTaskRegion(viewpoint) || !insideTaskRegion(cluster.center)) {
        return false;
    }
    if (cfg_.require_line_free_to_frontier &&
        !map_manager_->isLineFree(viewpoint, cluster.center, true, false)) {
        return false;
    }
    return countVisibleFrontierCells(viewpoint, cluster, cfg_.max_gain_rays) >=
           visibleFrontierThreshold(cluster);
}

int ExplorationFrontend::visibleFrontierThreshold(const FrontierCluster &cluster) const {
    const int cell_count = static_cast<int>(cluster.cells.size());
    if (cell_count <= 0) {
        return 1;
    }
    const int checked_count = sampledCount(cluster.cells.size(), std::max(1, cfg_.max_gain_rays));
    const double ratio = std::clamp(cfg_.min_visible_frontier_ratio, 0.0, 1.0);
    const int ratio_threshold =
            static_cast<int>(std::ceil(ratio * static_cast<double>(checked_count)));
    return std::min(checked_count,
                    std::max(std::max(1, cfg_.min_visible_frontier_cells),
                             ratio_threshold));
}

int ExplorationFrontend::countVisibleFrontierCells(const Vec3f &viewpoint,
                                                   const FrontierCluster &cluster,
                                                   const int max_checks) const {
    if (!viewpoint.allFinite() ||
        map_manager_ == nullptr ||
        !map_manager_->ready() ||
        cluster.cells.empty()) {
        return 0;
    }

    const int check_limit = max_checks > 0 ? max_checks : cfg_.max_gain_rays;
    const int stride = boundedStride(cluster.cells.size(), std::max(1, check_limit));
    int visible_count = 0;
    for (size_t i = 0; i < cluster.cells.size(); i += static_cast<size_t>(stride)) {
        const Vec3f &frontier_pos = cluster.cells[i].position;
        if (!insideTaskRegion(frontier_pos)) {
            continue;
        }
        if (map_manager_->isLineFree(viewpoint, frontier_pos, true, false)) {
            ++visible_count;
        }
    }
    return visible_count;
}

double ExplorationFrontend::estimateInformationGain(const Vec3f &viewpoint,
                                                    const FrontierCluster &cluster) const {
    if (cluster.cells.empty()) {
        return 0.0;
    }

    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    const int max_rays = std::max(1, cfg_.max_gain_rays);
    const int stride = boundedStride(cluster.cells.size(), max_rays);
    const double ray_step = std::max(map_res, cfg_.gain_ray_step);
    const double ray_length = std::max(ray_step, cfg_.gain_ray_length);
    Vec3f ray_direction = cluster.unknown_direction;
    if (ray_direction.norm() > 1.0e-6) {
        ray_direction.normalize();
    } else {
        ray_direction = horizontalNormalized(-cluster.free_direction, cluster.normal);
    }

    std::unordered_set<GridKey, GridKeyHasher> unknown_voxels;
    unknown_voxels.reserve(static_cast<size_t>(max_rays * 8));
    int visible_count = 0;
    int neighbor_unknown_count = 0;

    const auto insert_unknown = [&](const Vec3f &pos) {
        if (!map_manager_->insideLocalMap(pos) ||
            !insideTaskRegion(pos)) {
            return false;
        }
        const rog_map::GridType type = map_manager_->getGridType(pos);
        if (!isUnknownLike(type)) {
            return false;
        }
        unknown_voxels.insert(makeBucketKey(pos, map_res));
        return true;
    };

    for (size_t i = 0; i < cluster.cells.size(); i += static_cast<size_t>(stride)) {
        const Vec3f &frontier_pos = cluster.cells[i].position;
        if (!insideTaskRegion(frontier_pos)) {
            continue;
        }
        if (!map_manager_->isLineFree(viewpoint, frontier_pos, true, false)) {
            continue;
        }
        ++visible_count;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const Vec3f neighbor_pos = frontier_pos + map_res * Vec3f(dx, dy, dz);
                    if (insert_unknown(neighbor_pos)) {
                        ++neighbor_unknown_count;
                    }
                }
            }
        }

        for (double distance = ray_step; distance <= ray_length; distance += ray_step) {
            const Vec3f sample_pos = frontier_pos + distance * ray_direction;
            if (!map_manager_->insideLocalMap(sample_pos) ||
                !insideTaskRegion(sample_pos)) {
                break;
            }
            const rog_map::GridType type = map_manager_->getGridType(sample_pos);
            if (type == rog_map::GridType::OCCUPIED ||
                type == rog_map::GridType::OUT_OF_MAP) {
                break;
            }
            if (isUnknownLike(type)) {
                unknown_voxels.insert(makeBucketKey(sample_pos, map_res));
            }
        }
    }

    const double fallback_gain = static_cast<double>(cluster.size);
    const double local_unknown_bonus =
            std::min(static_cast<double>(neighbor_unknown_count),
                     static_cast<double>(unknown_voxels.size()));
    double raw_gain = static_cast<double>(unknown_voxels.size()) +
                      0.1 * local_unknown_bonus +
                      0.5 * static_cast<double>(visible_count);
    if (raw_gain <= 0.0) {
        raw_gain = visible_count > 0 ? static_cast<double>(visible_count) : fallback_gain;
    }
    const double distance = std::max(0.0, (viewpoint - cluster.center).norm());
    return raw_gain / (1.0 + 0.1 * distance);
}

double ExplorationFrontend::estimateTravelCost(const Vec3f &robot_pos,
                                               const Vec3f &viewpoint,
                                               vec_E<Vec3f> &guide_path,
                                               const bool allow_astar,
                                               bool *astar_used) const {
    guide_path.clear();
    if (astar_used != nullptr) {
        *astar_used = false;
    }
    if (!robot_pos.allFinite() || !viewpoint.allFinite()) {
        return kInfCost;
    }

    const bool inflated_line_free = map_manager_->isLineFree(robot_pos, viewpoint, true, false);
    const bool known_line_free =
            !cfg_.unknown_as_occupied_for_motion ||
            map_manager_->isLineFree(robot_pos, viewpoint, false, true);
    if (inflated_line_free && known_line_free) {
        guide_path.push_back(robot_pos);
        guide_path.push_back(viewpoint);
        return (viewpoint - robot_pos).norm();
    }

    if (!allow_astar || !cfg_.use_astar_cost || astar_ == nullptr) {
        return kInfCost;
    }
    if (astar_used != nullptr) {
        *astar_used = true;
    }

    vec_E<Vec3f> path;
    const int astar_flag =
            cfg_.unknown_as_occupied_for_motion
                    ? (path_search::ON_PROB_MAP |
                       path_search::UNKNOWN_AS_OCCUPIED |
                       path_search::DONT_USE_INF_NEIGHBOR)
                    : (path_search::ON_INF_MAP |
                       path_search::UNKNOWN_AS_FREE);
    const double distance = (viewpoint - robot_pos).norm();
    const double search_horizon = std::max(cfg_.frontier_search_radius * 1.5,
                                           distance * 1.8 + 2.0);
    const RET_CODE ret = astar_->pointToPointPathSearch(robot_pos,
                                                        viewpoint,
                                                        astar_flag,
                                                        search_horizon,
                                                        path,
                                                        std::max(0.005, cfg_.astar_time_out));
    if (ret != REACH_GOAL || path.empty()) {
        return kInfCost;
    }

    if ((path.front() - robot_pos).norm() > map_manager_->getResolution()) {
        path.insert(path.begin(), robot_pos);
    }
    if ((path.back() - viewpoint).norm() > map_manager_->getResolution()) {
        path.push_back(viewpoint);
    }

    for (size_t i = 1; i < path.size(); ++i) {
        const bool segment_inflated_free = map_manager_->isLineFree(path[i - 1], path[i], true, false);
        const bool segment_known_free =
                !cfg_.unknown_as_occupied_for_motion ||
                map_manager_->isLineFree(path[i - 1], path[i], false, true);
        if (!segment_inflated_free || !segment_known_free) {
            return kInfCost;
        }
    }

    guide_path = path;
    return pathLength(guide_path);
}

double ExplorationFrontend::estimateYawCost(const double current_yaw,
                                            const double candidate_yaw) const {
    if (!std::isfinite(current_yaw) || !std::isfinite(candidate_yaw)) {
        return 0.0;
    }
    return std::abs(wrapAngleDiff(candidate_yaw, current_yaw));
}

double ExplorationFrontend::resolveCandidateYaw(const double current_yaw,
                                                const Vec3f &robot_pos,
                                                const Vec3f &viewpoint,
                                                const FrontierCluster &cluster) const {
    const std::string policy = core_utils::normalizeToken(cfg_.yaw_policy);
    if (policy == "velocity" ||
        policy == "vel" ||
        policy == "free" ||
        policy == "yaw_to_vel") {
        return std::numeric_limits<double>::quiet_NaN();
    }

    Vec3f motion_dir = viewpoint - robot_pos;
    motion_dir.z() = 0.0;
    Vec3f frontier_dir = cluster.center - viewpoint;
    frontier_dir.z() = 0.0;

    if (motion_dir.norm() < 1.0e-3) {
        return current_yaw;
    }
    if (policy == "frontier" || policy == "frontier_center" || policy == "observe") {
        if (frontier_dir.norm() > 1.0e-3) {
            return std::atan2(frontier_dir.y(), frontier_dir.x());
        }
        return current_yaw;
    }
    if (policy == "blend" && frontier_dir.norm() > 1.0e-3) {
        const Vec3f blended = motion_dir.normalized() + 0.35 * frontier_dir.normalized();
        if (blended.head<2>().norm() > 1.0e-3) {
            return std::atan2(blended.y(), blended.x());
        }
    }
    return std::atan2(motion_dir.y(), motion_dir.x());
}

double ExplorationFrontend::estimateCurvatureCost(const StatePVAJ &robot_state,
                                                  const Vec3f &viewpoint,
                                                  const FrontierCluster &cluster) const {
    const Vec3f robot_pos = robot_state.col(0);
    const Vec3f robot_vel = robot_state.col(1);
    const Vec3f to_goal = viewpoint - robot_pos;
    const Vec3f observe_dir = cluster.center - viewpoint;

    double cost = angleBetween(to_goal, observe_dir);
    Vec3f vel_dir = robot_vel;
    vel_dir.z() = 0.0;
    if (vel_dir.norm() > 0.25) {
        cost += angleBetween(vel_dir, to_goal);
    }
    return cost;
}

double ExplorationFrontend::estimateUnknownRisk(const Vec3f &viewpoint) const {
    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    int unknown_count = 0;
    int checked_count = 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const Vec3f neighbor_pos = viewpoint + map_res * Vec3f(dx, dy, dz);
                if (!map_manager_->insideLocalMap(neighbor_pos) ||
                    !insideTaskRegion(neighbor_pos)) {
                    continue;
                }
                ++checked_count;
                if (isUnknownLike(map_manager_->getGridType(neighbor_pos))) {
                    ++unknown_count;
                }
            }
        }
    }
    if (checked_count == 0) {
        return 1.0;
    }
    return static_cast<double>(unknown_count) / static_cast<double>(checked_count);
}

double ExplorationFrontend::scoreCandidate(const ExplorationGoal &candidate,
                                           const double unknown_risk) const {
    const double distance = std::max(0.0, candidate.distance_to_robot);
    const double preferred_distance = std::max(0.0, cfg_.preferred_goal_distance);
    const double progress_reward = preferred_distance > 0.0
                                           ? std::min(distance, preferred_distance)
                                           : distance;
    const double shortfall = std::max(0.0, cfg_.min_goal_distance - distance);
    const double preferred_shortfall =
            preferred_distance > 0.0 ? std::max(0.0, preferred_distance - distance) : 0.0;
    const double preferred_shortfall_scale =
            preferred_distance > 1.0e-3 ? preferred_shortfall / preferred_distance : 0.0;
    return cfg_.weight_travel * candidate.travel_cost +
           cfg_.weight_yaw * candidate.yaw_cost +
           cfg_.weight_curvature * candidate.curvature_cost +
           cfg_.weight_info_gain * effectiveInformationGain(candidate.information_gain) +
           cfg_.weight_unknown_risk * unknown_risk +
           cfg_.weight_progress * progress_reward +
           cfg_.weight_short_goal * shortfall * shortfall +
           0.5 * cfg_.weight_short_goal * preferred_shortfall * preferred_shortfall_scale;
}

double ExplorationFrontend::effectiveInformationGain(const double information_gain) const {
    if (!std::isfinite(information_gain) || information_gain <= 0.0) {
        return 0.0;
    }
    const double saturation = std::max(0.0, cfg_.information_gain_saturation);
    if (saturation <= 0.0) {
        return information_gain;
    }
    return std::min(information_gain, saturation);
}

ExplorationGoal ExplorationFrontend::selectGoalWithAtsp(
        const Vec3f &robot_pos,
        const double current_yaw,
        const vec_E<ExplorationGoal> &reachable_candidates) const {
    if (reachable_candidates.empty()) {
        return ExplorationGoal{};
    }

    const int candidate_num = std::min(static_cast<int>(reachable_candidates.size()),
                                       std::max(1, cfg_.atsp.max_candidate_num));
    exploration::ATSPProblem problem;
    problem.depot_index = 0;
    problem.time_budget_ms = cfg_.atsp.time_budget_ms;
    problem.candidates.reserve(static_cast<size_t>(candidate_num));
    problem.node_reward.reserve(static_cast<size_t>(candidate_num));
    problem.directed_cost_matrix =
            Eigen::MatrixXd::Zero(candidate_num + 1, candidate_num + 1);

    for (int i = 0; i < candidate_num; ++i) {
        problem.candidates.push_back(exploration::ATSPCandidate{i});
        problem.node_reward.push_back(effectiveInformationGain(reachable_candidates[i].information_gain));
    }

    for (int i = 0; i < candidate_num; ++i) {
        const ExplorationGoal &candidate = reachable_candidates[i];
        const double yaw_cost = estimateYawCost(current_yaw, candidate.yaw);
        problem.directed_cost_matrix(0, i + 1) =
                std::max(0.0, (candidate.position - robot_pos).norm()) +
                cfg_.weight_yaw * yaw_cost +
                cfg_.weight_curvature * candidate.curvature_cost +
                cfg_.weight_info_gain * effectiveInformationGain(candidate.information_gain);
        problem.directed_cost_matrix(i + 1, 0) = 0.0;
    }

    for (int i = 0; i < candidate_num; ++i) {
        for (int j = 0; j < candidate_num; ++j) {
            problem.directed_cost_matrix(i + 1, j + 1) =
                    i == j ? 0.0 : pairwiseCandidateCost(reachable_candidates[i],
                                                          reachable_candidates[j]);
        }
    }

    exploration::ATSPTourPlanner planner(cfg_.atsp);
    const exploration::ATSPSolution solution = planner.solve(problem);
    if (!solution.ordered_candidate_ids.empty()) {
        const int selected_id = solution.ordered_candidate_ids.front();
        if (selected_id >= 0 && selected_id < candidate_num) {
            ExplorationGoal selected = reachable_candidates[selected_id];
            selected.reason = "selected by atsp " + solution.solver_status;
            return selected;
        }
    }

    return *std::min_element(reachable_candidates.begin(),
                             reachable_candidates.begin() + candidate_num,
                             [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                                 return lhs.score < rhs.score;
                             });
}

double ExplorationFrontend::pairwiseCandidateCost(const ExplorationGoal &from,
                                                  const ExplorationGoal &to) const {
    const double travel = (to.position - from.position).norm();
    const double yaw_cost = estimateYawCost(from.yaw, to.yaw);
    const double node_cost =
            cfg_.weight_info_gain * to.information_gain +
            cfg_.weight_yaw * yaw_cost +
            cfg_.weight_curvature * to.curvature_cost;
    return std::max(0.0, travel + node_cost);
}

bool ExplorationFrontend::isUnknownLike(const rog_map::GridType type) const {
    return type == rog_map::GridType::UNKNOWN ||
           type == rog_map::GridType::UNDEFINED ||
           type == rog_map::GridType::FRONTIER;
}

bool ExplorationFrontend::isFreeLike(const rog_map::GridType type) const {
    return type == rog_map::GridType::KNOWN_FREE;
}

bool ExplorationFrontend::insideTaskRegion(const Vec3f &position) const {
    return mission_manager_ == nullptr ||
           mission_manager_->insideTaskBox(position);
}

bool ExplorationFrontend::clipTaskSearchBox(Vec3f &box_min, Vec3f &box_max) const {
    return mission_manager_ == nullptr ||
           mission_manager_->clipSearchBox(box_min, box_max);
}

double ExplorationFrontend::wrapAngleDiff(const double lhs, const double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

}  // namespace general_planner
