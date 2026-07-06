#include <general_core/exploration/exploration_frontend.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <Eigen/Eigenvalues>
#include <boost/filesystem.hpp>

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
    return source == "rog_map" ||
           source == "rog_map_frontier";
}

bool sourceAllowsFallback(const std::string &source) {
    return source == "auto" ||
           source == "frontier" ||
           source == "fallback" ||
           source == "fallback_scan" ||
           source == "planner_scan" ||
           source == "map_query" ||
           source == "frontier_scan" ||
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

bool carriesRecoveryIntent(const ExplorationGoal &goal) {
    return goal.identity.recovery_intent ||
           goal.reason.find("recovery") != std::string::npos ||
           goal.reason.find("bootstrap") != std::string::npos ||
           goal.memory_key.find("recovery") != std::string::npos ||
           goal.memory_key.find("bootstrap") != std::string::npos;
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

std::string csvEscape(const std::string &value) {
    bool needs_quotes = false;
    std::string escaped;
    escaped.reserve(value.size());
    for (const char c : value) {
        if (c == '"' || c == ',' || c == '\n' || c == '\r') {
            needs_quotes = true;
        }
        if (c == '"') {
            escaped.push_back('"');
        }
        escaped.push_back(c);
    }
    return needs_quotes ? "\"" + escaped + "\"" : escaped;
}

std::mutex &explorationFrontendLogMutex() {
    static std::mutex mutex;
    return mutex;
}

std::ofstream &explorationFrontendLogStream() {
    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        const std::string path =
                std::string(ROOT_DIR) + "log/diagnostic_events/exploration_frontend.csv";
        const boost::filesystem::path log_path(path);
        const boost::filesystem::path parent = log_path.parent_path();
        boost::system::error_code ec;
        if (!parent.empty()) {
            boost::filesystem::create_directories(parent, ec);
        }
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream.is_open()) {
            stream << "stamp,event,reason,robot_x,robot_y,robot_z,"
                   << "goal_valid,goal_x,goal_y,goal_z,goal_yaw,"
                   << "candidate_id,frontier_id,score,information_gain,travel_cost,"
                   << "yaw_cost,curvature_cost,visible_cells,visible_ratio,frontier_area,"
                   << "candidate_count,checked,astar_checks,reachable,clusters,raw_clusters,"
                   << "frontiers,raw_frontiers,source,fallback,object_records,object_active,"
                   << "object_dormant,object_covered,object_stale,dormant_bootstrap,"
                   << "bootstrap_used,bootstrap_attempts,bootstrap_reject_bounds,"
                   << "bootstrap_reject_occupied,bootstrap_reject_unknown,"
                   << "bootstrap_reject_inflated,bootstrap_reject_esdf,"
                   << "bootstrap_reject_near,bootstrap_reject_unreachable,"
                   << "expansion_attempts,expansion_added,expansion_memory_added,"
                   << "sample_budget_hit,plan_ms"
                   << std::endl;
        }
    }
    return stream;
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
            const int unknown_near_count = countUnknownNear(
                    record.reference,
                    std::max(map_manager_->getResolution(),
                             cfg_.frontier_manager_covered_unknown_radius),
                    std::max(map_manager_->getResolution(), cfg_.map_resolution));
            const bool low_unknown_evidence =
                    unknown_near_count <=
                    std::max(1, cfg_.min_unknown_neighbor_count / 2);
            if (low_unknown_evidence &&
                (record.selection_count > 0 || record.seen_count > 2)) {
                ++record.low_unknown_count;
            } else {
                record.low_unknown_count = 0;
            }
            if (record.state == FrontierState::COVERED ||
                record.state == FrontierState::STALE) {
                record.state = FrontierState::ACTIVE;
            }

            if (record.dormant_until > stamp) {
                record.state = FrontierState::DORMANT;
                continue;
            }

            if (record.low_unknown_count >= 2 ||
                !frontierStillUseful(record.cluster)) {
                record.state = FrontierState::COVERED;
                record.blocked_until = stamp + std::max(cfg_.frontier_manager_stale_time,
                                                        cfg_.frontier_manager_dormant_time);
                record.last_state_stamp = stamp;
                continue;
            }

            record.state = FrontierState::ACTIVE;
            record.no_view_count = 0;
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
            const int stale_threshold =
                    std::max(2, 2 * cfg_.frontier_manager_no_view_threshold);
            if (record->no_view_count >= stale_threshold) {
                record->state = FrontierState::STALE;
                record->dormant_until = 0.0;
            } else {
                record->state = FrontierState::DORMANT;
                record->dormant_until =
                        stamp + std::max(0.0, cfg_.frontier_manager_dormant_time);
            }
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
        const int stale_threshold =
                std::max(2, cfg_.frontier_manager_no_view_threshold + 1);
        if (record->failure_count >= stale_threshold) {
            record->state = FrontierState::STALE;
            record->dormant_until = 0.0;
        } else {
            record->state = FrontierState::DORMANT;
            const double failure_scale =
                    std::clamp(0.5 * static_cast<double>(record->failure_count),
                               1.0,
                               4.0);
            record->dormant_until =
                    stamp + failure_scale *
                                    std::max(0.0, cfg_.frontier_manager_dormant_time);
        }
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
        int low_unknown_count{0};
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

bool ExplorationFrontend::generateCandidates(const StatePVAJ &robot_state,
                                             const double current_yaw,
                                             const double stamp,
                                             ExplorationCandidateSet &out) {
    out = ExplorationCandidateSet{};
    out.source = core_utils::normalizeToken(cfg_.frontier_source);
    if (out.source.empty()) {
        out.source = "fallback_scan";
    }
    ExplorationGoal goal;
    last_candidate_set_ = ExplorationCandidateSet{};
    exploration_finished_ = false;

    const auto finish_failure = [&](const std::string &reason) {
        out.valid = false;
        out.exploration_finished = exploration_finished_;
        out.reason = reason;
        last_candidate_set_ = out;
        return false;
    };

    if (!cfg_.enable) {
        goal.reason = "exploration disabled";
        return finish_failure(goal.reason);
    }
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        goal.reason = "map manager not ready";
        return finish_failure(goal.reason);
    }

    const Vec3f robot_pos = robot_state.col(0);
    if (!robot_pos.allFinite()) {
        goal.reason = "robot state is not finite";
        return finish_failure(goal.reason);
    }
    if (!map_manager_->insideLocalMap(robot_pos)) {
        goal.reason = "robot is outside local map";
        return finish_failure(goal.reason);
    }
    const auto plan_start = std::chrono::steady_clock::now();
    const auto plan_elapsed_ms = [&]() {
        return std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - plan_start)
                .count();
    };
    const double candidate_sampling_budget_ms =
            std::max(90.0,
                     60.0 + std::max(0.0, cfg_.astar_total_time_budget_ms) +
                             static_cast<double>(std::max(0, cfg_.atsp.time_budget_ms)));
    bool candidate_sampling_budget_hit = false;

    vec_E<FrontierCell> frontier_cells;
    vec_E<FrontierCluster> clusters;
    FrontierSearchStats search_stats;
    FrontierObjectStats object_stats;
    bool clusters_from_incremental_cache = false;
    size_t raw_cluster_count = 0U;
    bool using_dormant_bootstrap_clusters = false;
    bool used_bootstrap_candidates = false;
    int bootstrap_attempt_count = 0;
    int bootstrap_reject_bounds = 0;
    int bootstrap_reject_occupied = 0;
    int bootstrap_reject_unknown = 0;
    int bootstrap_reject_inflated = 0;
    int bootstrap_reject_esdf = 0;
    int bootstrap_reject_near = 0;
    int bootstrap_reject_unreachable = 0;
    int expansion_attempt_count = 0;
    int expansion_added_count = 0;
    int expansion_memory_added_count = 0;

    std::string source = out.source;
    const bool allow_rog = sourceAllowsRogMap(source);
    const bool allow_fallback = sourceAllowsFallback(source);
    FrontierSearchStats rog_stats;
    if (allow_rog && map_manager_->getMapConfig().frontier_extraction_en) {
        if (!collectRogMapFrontierClusters(robot_pos, clusters, rog_stats)) {
            goal.reason = "frontier search failed";
            return finish_failure(goal.reason);
        }
        if (!clusters.empty() || !allow_fallback) {
            search_stats = rog_stats;
            clusters_from_incremental_cache = true;
        }
    }

    if (!clusters_from_incremental_cache) {
        const bool fallback_ok =
                (allow_fallback || !allow_rog ||
                 !map_manager_->getMapConfig().frontier_extraction_en)
                        ? collectFallbackFrontierCells(robot_pos, frontier_cells, search_stats)
                        : collectFrontierCells(robot_pos, frontier_cells, search_stats);
        if (!fallback_ok) {
            goal.reason = "frontier search failed";
            return finish_failure(goal.reason);
        }
        if (allow_rog) {
            search_stats.fallback_used = true;
            search_stats.raw_frontier_cells = rog_stats.raw_frontier_cells;
            if (rog_stats.raw_frontier_cells > 0) {
                search_stats.source = rog_stats.source + "+fallback_scan";
            }
        }
    }
    if (!search_stats.source.empty()) {
        out.source = search_stats.source;
    }

    const int frontier_output_count =
            clusters_from_incremental_cache
                    ? search_stats.frontier_cells
                    : static_cast<int>(frontier_cells.size());
    const auto log_frontend_decision =
            [&](const std::string &event,
                const std::string &reason,
                const ExplorationGoal *selected_goal,
                const int candidate_count,
                const int checked_count,
                const int astar_checked_count,
                const int reachable_count) {
                std::lock_guard<std::mutex> lock(explorationFrontendLogMutex());
                std::ofstream &stream = explorationFrontendLogStream();
                if (!stream.is_open()) {
                    return;
                }
                const bool goal_valid =
                        selected_goal != nullptr && selected_goal->valid;
                const Vec3f goal_pos =
                        goal_valid ? selected_goal->position : Vec3f::Zero();
                stream << std::fixed << std::setprecision(9)
                       << stamp << ","
                       << csvEscape(event) << ","
                       << csvEscape(reason) << ","
                       << robot_pos.x() << ","
                       << robot_pos.y() << ","
                       << robot_pos.z() << ","
                       << static_cast<int>(goal_valid) << ","
                       << goal_pos.x() << ","
                       << goal_pos.y() << ","
                       << goal_pos.z() << ","
                       << (goal_valid ? selected_goal->yaw : 0.0) << ","
                       << (goal_valid ? selected_goal->candidate_id : -1) << ","
                       << (goal_valid ? selected_goal->frontier_id : -1) << ","
                       << (goal_valid ? selected_goal->score : 0.0) << ","
                       << (goal_valid ? selected_goal->information_gain : 0.0) << ","
                       << (goal_valid ? selected_goal->travel_cost : 0.0) << ","
                       << (goal_valid ? selected_goal->yaw_cost : 0.0) << ","
                       << (goal_valid ? selected_goal->curvature_cost : 0.0) << ","
                       << (goal_valid ? selected_goal->visible_frontier_cell_count : 0) << ","
                       << (goal_valid ? selected_goal->visible_frontier_ratio : 0.0) << ","
                       << (goal_valid ? selected_goal->frontier_area : 0.0) << ","
                       << candidate_count << ","
                       << checked_count << ","
                       << astar_checked_count << ","
                       << reachable_count << ","
                       << clusters.size() << ","
                       << raw_cluster_count << ","
                       << frontier_output_count << ","
                       << search_stats.raw_frontier_cells << ","
                       << csvEscape(search_stats.source) << ","
                       << static_cast<int>(search_stats.fallback_used) << ","
                       << object_stats.records << ","
                       << object_stats.active << ","
                       << object_stats.dormant << ","
                       << object_stats.covered << ","
                       << object_stats.stale << ","
                       << static_cast<int>(using_dormant_bootstrap_clusters) << ","
                       << static_cast<int>(used_bootstrap_candidates) << ","
                       << bootstrap_attempt_count << ","
                       << bootstrap_reject_bounds << ","
                       << bootstrap_reject_occupied << ","
                       << bootstrap_reject_unknown << ","
                       << bootstrap_reject_inflated << ","
                       << bootstrap_reject_esdf << ","
                       << bootstrap_reject_near << ","
                       << bootstrap_reject_unreachable << ","
                       << expansion_attempt_count << ","
                       << expansion_added_count << ","
                       << expansion_memory_added_count << ","
                       << static_cast<int>(candidate_sampling_budget_hit) << ","
                       << plan_elapsed_ms()
                       << std::endl;
            };
    if ((clusters_from_incremental_cache && clusters.empty()) ||
        (!clusters_from_incremental_cache && frontier_cells.empty())) {
        const bool observation_ready = mapObservationReady(search_stats);
        if (!observation_ready) {
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
            log_frontend_decision("wait_map", goal.reason, nullptr, 0, 0, 0, 0);
            return finish_failure(goal.reason);
        }

        if (!cfg_.expansion_fallback_enable) {
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
            log_frontend_decision(exploration_finished_ ? "finished_no_frontier"
                                                        : "no_frontier",
                                  goal.reason,
                                  nullptr,
                                  0,
                                  0,
                                  0,
                                  0);
            return finish_failure(goal.reason);
        }

        exploration_finished_ = false;
        if (mission_manager_ != nullptr) {
            mission_manager_->updateNoFrontier(robot_pos, stamp, true);
        }
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No local frontier; using expansion fallback. "
                      << "source=" << search_stats.source
                      << ", raw_frontiers=" << search_stats.raw_frontier_cells
                      << ", fallback=" << static_cast<int>(search_stats.fallback_used)
                      << ", searched=" << search_stats.searched_cells
                      << ", free=" << search_stats.known_free_cells
                      << ", unknown=" << search_stats.unknown_cells
                      << ", occupied=" << search_stats.occupied_cells
                      << "." << std::endl;
        }
    }

    if (!clusters_from_incremental_cache) {
        clusterFrontiers(frontier_cells, clusters);
    }
    if (clusters.empty()) {
        if (!cfg_.expansion_fallback_enable) {
            exploration_finished_ = false;
            goal.reason = "no frontier cluster";
            if (cfg_.print_log) {
                std::cout << " -- [ExplorationFrontend] No frontier cluster yet. "
                          << "source=" << search_stats.source
                          << ", raw_frontiers=" << search_stats.raw_frontier_cells
                          << ", fallback=" << static_cast<int>(search_stats.fallback_used)
                          << ", "
                          << "frontiers=" << frontier_output_count
                          << ", min_cluster_size=" << cfg_.min_frontier_cluster_size
                          << "." << std::endl;
            }
            log_frontend_decision("no_frontier_cluster",
                                  goal.reason,
                                  nullptr,
                                  0,
                                  0,
                                  0,
                                  0);
            return finish_failure(goal.reason);
        }
        if (cfg_.print_log) {
            std::cout << " -- [ExplorationFrontend] No frontier cluster; using expansion fallback. "
                      << "source=" << search_stats.source
                      << ", raw_frontiers=" << search_stats.raw_frontier_cells
                      << ", fallback=" << static_cast<int>(search_stats.fallback_used)
                      << ", frontiers=" << frontier_output_count
                      << ", min_cluster_size=" << cfg_.min_frontier_cluster_size
                      << "." << std::endl;
        }
    }

    raw_cluster_count = clusters.size();
    vec_E<FrontierCluster> split_clusters;
    splitLargeFrontierClusters(clusters, robot_pos, split_clusters);
    if (!split_clusters.empty()) {
        clusters.swap(split_clusters);
    }

    const vec_E<FrontierCluster> unmanaged_clusters = clusters;
    if (frontier_object_manager_ != nullptr) {
        vec_E<FrontierCluster> managed_clusters;
        frontier_object_manager_->update(clusters,
                                         robot_pos,
                                         stamp,
                                         managed_clusters,
                                         object_stats);
        clusters.swap(managed_clusters);
        if (clusters.empty() &&
            !unmanaged_clusters.empty() &&
            object_stats.dormant > 0 &&
            object_stats.active == 0) {
            clusters = unmanaged_clusters;
            using_dormant_bootstrap_clusters = true;
        }
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
        if (!candidates.empty() && plan_elapsed_ms() >= candidate_sampling_budget_ms) {
            candidate_sampling_budget_hit = true;
            break;
        }
        vec_E<ExplorationGoal> cluster_candidates;
        sampleViewpointsForCluster(cluster,
                                   robot_state,
                                   current_yaw,
                                   cluster_candidates,
                                   per_cluster_sample_budget);
        if (cluster_candidates.empty() &&
            frontier_object_manager_ != nullptr &&
            !using_dormant_bootstrap_clusters) {
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

    if (candidates.empty()) {
        const int bootstrap_limit = std::min(max_candidate_num,
                                             std::max(4,
                                                      per_cluster_keep *
                                                              std::max(1, static_cast<int>(clusters.size()))));
        const auto bootstrap_viewpoint_safe = [&](const Vec3f &viewpoint) {
            if (!viewpoint.allFinite() ||
                map_manager_ == nullptr ||
                !map_manager_->ready() ||
                !map_manager_->insideLocalMap(viewpoint) ||
                !insideTaskRegion(viewpoint)) {
                ++bootstrap_reject_bounds;
                return false;
            }

            const rog_map::GridType grid_type = map_manager_->getGridType(viewpoint);
            if (grid_type == rog_map::GridType::OCCUPIED ||
                grid_type == rog_map::GridType::OUT_OF_MAP) {
                ++bootstrap_reject_occupied;
                return false;
            }
            if (cfg_.unknown_as_occupied_for_motion && !isFreeLike(grid_type)) {
                ++bootstrap_reject_unknown;
                return false;
            }

            const rog_map::GridType inf_type = map_manager_->getInfGridType(viewpoint);
            if (inf_type == rog_map::GridType::OCCUPIED ||
                inf_type == rog_map::GridType::OUT_OF_MAP) {
                ++bootstrap_reject_inflated;
                return false;
            }

            if (map_manager_->hasESDF() && cfg_.viewpoint_safe_distance > 0.0) {
                double dist = 0.0;
                Vec3f grad = Vec3f::Zero();
                if (!map_manager_->evaluateESDF(viewpoint, dist, grad) ||
                    !std::isfinite(dist) ||
                    dist < cfg_.viewpoint_safe_distance) {
                    ++bootstrap_reject_esdf;
                    return false;
                }
            }
            return true;
        };
        const auto add_bootstrap_candidate = [&](const FrontierCluster &cluster,
                                                 Vec3f viewpoint) {
            if (static_cast<int>(candidates.size()) >= bootstrap_limit) {
                return;
            }
            ++bootstrap_attempt_count;
            if (!viewpoint.allFinite()) {
                ++bootstrap_reject_bounds;
                return;
            }
            viewpoint.z() = robot_pos.z() + cfg_.viewpoint_height_offset;
            clampViewpointToTaskRegion(viewpoint);
            const double distance_to_robot = (viewpoint - robot_pos).norm();
            if (distance_to_robot <= std::max(0.2, 0.5 * cfg_.goal_reached_distance)) {
                ++bootstrap_reject_near;
                return;
            }
            if (!bootstrap_viewpoint_safe(viewpoint)) {
                return;
            }

            vec_E<Vec3f> guide_path;
            const double travel_cost = estimateTravelCost(robot_pos,
                                                          viewpoint,
                                                          guide_path,
                                                          true);
            if (!std::isfinite(travel_cost) || travel_cost >= kInfCost) {
                ++bootstrap_reject_unreachable;
                return;
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
            candidate.identity.intent_mode = "recovery";
            candidate.identity.recovery_intent = true;
            candidate.identity.candidate_id = candidate.candidate_id;
            candidate.identity.frontier_id = candidate.frontier_id;
            candidate.identity.candidate_key = "candidate:" + candidate.memory_key;
            candidate.identity.frontier_key =
                    cluster.object_id >= 0
                            ? std::string("frontier_object:") + std::to_string(cluster.object_id)
                            : "frontier:" + memoryKeyFromKey(frontier_key);
            candidate.identity.goal_key = candidate.identity.candidate_key;
            candidate.identity.guide_path_key =
                    nhbp::makeGuidePathKey(guide_path, cfg_.map_resolution);
            candidate.frontier_center_valid = true;
            candidate.frontier_center = cluster.center;
            candidate.frontier_bbox_min = cluster.bbox_min;
            candidate.frontier_bbox_max = cluster.bbox_max;
            candidate.frontier_area = cluster.area;
            candidate.visible_frontier_cell_count =
                    countVisibleFrontierCells(viewpoint, cluster, cfg_.max_gain_rays);
            const int visibility_denominator =
                    sampledCount(cluster.cells.size(), std::max(1, cfg_.max_gain_rays));
            candidate.visible_frontier_ratio =
                    visibility_denominator <= 0
                            ? 0.0
                            : static_cast<double>(candidate.visible_frontier_cell_count) /
                                      static_cast<double>(visibility_denominator);
            candidate.information_gain =
                    std::max(cfg_.min_information_gain,
                             estimateInformationGain(viewpoint, cluster));
            candidate.distance_to_robot = distance_to_robot;
            candidate.travel_cost = travel_cost;
            candidate.guide_path = guide_path;
            candidate.yaw_cost = estimateYawCost(current_yaw, candidate.yaw);
            candidate.curvature_cost = estimateCurvatureCost(robot_state, viewpoint, cluster);
            candidate.history_score_delta = cluster.object_score_delta;
            const double unknown_risk = estimateUnknownRisk(viewpoint);
            candidate.score = scoreCandidate(candidate, unknown_risk) +
                              candidate.history_score_delta;
            candidate.reason = using_dormant_bootstrap_clusters
                                       ? "bootstrap dormant frontier motion"
                                       : "bootstrap frontier motion";
            candidates.push_back(candidate);
            used_bootstrap_candidates = true;
        };

        for (const FrontierCluster &cluster : clusters) {
            if (static_cast<int>(candidates.size()) >= bootstrap_limit) {
                break;
            }
            const Vec3f free_dir = horizontalNormalized(robot_pos - cluster.center,
                                                        cluster.free_direction);
            const Vec3f to_frontier = horizontalNormalized(cluster.center - robot_pos,
                                                           -free_dir);
            const double offsets[] = {0.6, 1.0, 1.5, 2.2, 3.0};
            for (const double offset : offsets) {
                add_bootstrap_candidate(cluster, cluster.center + offset * free_dir);
            }

            const double center_distance =
                    std::max(0.0, (cluster.center - robot_pos).norm());
            const double steps[] = {
                    0.35,
                    0.7,
                    1.2,
                    std::min(2.0, std::max(0.0, center_distance - 0.5)),
                    std::min(3.5, std::max(0.0, center_distance - 0.8)),
                    std::min(5.0, std::max(0.0, center_distance - 1.0))};
            for (const double step : steps) {
                if (step > 0.0) {
                    add_bootstrap_candidate(cluster, robot_pos + step * to_frontier);
                }
            }
        }

        if (candidates.empty() && !clusters.empty()) {
            const FrontierCluster &cluster = clusters.front();
            ExplorationGoal candidate;
            candidate.valid = true;
            candidate.position = robot_pos;
            Vec3f yaw_direction = cluster.center - robot_pos;
            yaw_direction.z() = 0.0;
            candidate.yaw = yaw_direction.norm() > 1.0e-3
                                    ? std::atan2(yaw_direction.y(), yaw_direction.x())
                                    : (std::isfinite(current_yaw) ? current_yaw : 0.0);
            const GridKey frontier_key =
                    quantizedKey(cluster.center, std::max(0.25, cfg_.frontier_cluster_radius));
            const GridKey candidate_key =
                    quantizedKey(robot_pos, std::max(0.25, cfg_.map_resolution));
            candidate.frontier_id =
                    cluster.object_id >= 0 ? cluster.object_id : stableIdFromKey(frontier_key);
            candidate.candidate_id = stableIdFromKey(candidate_key);
            candidate.memory_key = memoryKeyFromKey(candidate_key);
            candidate.identity.intent_mode = "recovery";
            candidate.identity.recovery_intent = true;
            candidate.identity.candidate_id = candidate.candidate_id;
            candidate.identity.frontier_id = candidate.frontier_id;
            candidate.identity.candidate_key = "candidate:" + candidate.memory_key;
            candidate.identity.frontier_key =
                    cluster.object_id >= 0
                            ? std::string("frontier_object:") + std::to_string(cluster.object_id)
                            : "frontier:" + memoryKeyFromKey(frontier_key);
            candidate.identity.goal_key = candidate.identity.candidate_key;
            candidate.identity.guide_path_key =
                    nhbp::makeGuidePathKey(vec_E<Vec3f>{robot_pos, robot_pos}, cfg_.map_resolution);
            candidate.frontier_center_valid = true;
            candidate.frontier_center = cluster.center;
            candidate.frontier_bbox_min = cluster.bbox_min;
            candidate.frontier_bbox_max = cluster.bbox_max;
            candidate.frontier_area = cluster.area;
            candidate.visible_frontier_cell_count =
                    countVisibleFrontierCells(robot_pos, cluster, cfg_.max_gain_rays);
            const int visibility_denominator =
                    sampledCount(cluster.cells.size(), std::max(1, cfg_.max_gain_rays));
            candidate.visible_frontier_ratio =
                    visibility_denominator <= 0
                            ? 0.0
                            : static_cast<double>(candidate.visible_frontier_cell_count) /
                                      static_cast<double>(visibility_denominator);
            candidate.information_gain =
                    std::max(cfg_.min_information_gain,
                             estimateInformationGain(robot_pos, cluster));
            candidate.distance_to_robot = 0.0;
            candidate.travel_cost = 0.0;
            candidate.guide_path = vec_E<Vec3f>{robot_pos, robot_pos};
            candidate.yaw_cost = estimateYawCost(current_yaw, candidate.yaw);
            candidate.curvature_cost = 0.0;
            candidate.history_score_delta = cluster.object_score_delta;
            const double unknown_risk = estimateUnknownRisk(robot_pos);
            candidate.score = scoreCandidate(candidate, unknown_risk) +
                              candidate.history_score_delta;
            candidate.reason = "bootstrap frontier yaw scan";
            candidates.push_back(candidate);
            used_bootstrap_candidates = true;
        }
    }

    const int expansion_trigger_min_candidates =
            std::clamp(cfg_.expansion_trigger_min_candidates, 0, max_candidate_num);
    const int expansion_trigger_max_clusters =
            std::max(0, cfg_.expansion_trigger_max_clusters);
    const bool expansion_sparse_candidates =
            candidates.empty() ||
            static_cast<int>(candidates.size()) < expansion_trigger_min_candidates;
    const bool expansion_weak_frontier_structure =
            clusters.empty() ||
            static_cast<int>(clusters.size()) <= expansion_trigger_max_clusters;
    const bool expansion_needed =
            cfg_.expansion_fallback_enable &&
            (candidates.empty() ||
             (expansion_sparse_candidates && expansion_weak_frontier_structure) ||
             (candidate_sampling_budget_hit && expansion_sparse_candidates));
    if (expansion_needed) {
        appendExpansionFallbackCandidates(robot_state,
                                          current_yaw,
                                          stamp,
                                          clusters,
                                          candidates,
                                          expansion_attempt_count,
                                          expansion_added_count);
    }
    if (cfg_.expansion_fallback_enable &&
        cfg_.expansion_memory_enable &&
        expansion_needed &&
        static_cast<int>(candidates.size()) < max_candidate_num) {
        appendRememberedExpansionCandidates(robot_state,
                                            current_yaw,
                                            stamp,
                                            clusters,
                                            candidates,
                                            expansion_memory_added_count);
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
                      << ", stale=" << object_stats.stale
                      << ", dormant_bootstrap="
                      << static_cast<int>(using_dormant_bootstrap_clusters)
                      << ", bootstrap_candidates="
                      << static_cast<int>(used_bootstrap_candidates)
                      << ", bootstrap_attempts=" << bootstrap_attempt_count
                      << ", bootstrap_reject_bounds=" << bootstrap_reject_bounds
                      << ", bootstrap_reject_occupied=" << bootstrap_reject_occupied
                      << ", bootstrap_reject_unknown=" << bootstrap_reject_unknown
                      << ", bootstrap_reject_inflated=" << bootstrap_reject_inflated
                      << ", bootstrap_reject_esdf=" << bootstrap_reject_esdf
                      << ", bootstrap_reject_near=" << bootstrap_reject_near
                      << ", bootstrap_reject_unreachable=" << bootstrap_reject_unreachable
                      << ", expansion_attempts=" << expansion_attempt_count
                      << ", expansion_added=" << expansion_added_count
                      << ", expansion_memory_added=" << expansion_memory_added_count
                      << ")." << std::endl;
        }
        log_frontend_decision(exploration_finished_ ? "finished_no_valid_viewpoint"
                                                    : "no_valid_viewpoint",
                              goal.reason,
                              nullptr,
                              0,
                              0,
                              0,
                              0);
        return finish_failure(goal.reason);
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
        if (frontier_object_manager_ != nullptr) {
            frontier_object_manager_->recordFailed(candidate, stamp);
        }
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
            const bool recovery_candidate = carriesRecoveryIntent(candidate);
            const std::string previous_reason = candidate.reason;
            candidate.travel_cost = travel_cost;
            candidate.distance_to_robot = (candidate.position - robot_pos).norm();
            candidate.guide_path = guide_path;
            candidate.identity.guide_path_key =
                    nhbp::makeGuidePathKey(guide_path, cfg_.map_resolution);
            const double unknown_risk = estimateUnknownRisk(candidate.position);
            candidate.score = scoreCandidate(candidate, unknown_risk) +
                              candidate.history_score_delta;
            if (recovery_candidate) {
                candidate.identity.intent_mode = "recovery";
                candidate.identity.recovery_intent = true;
                candidate.reason = previous_reason.empty()
                                           ? "selected reachable recovery viewpoint"
                                           : previous_reason + " selected_reachable";
            } else {
                candidate.reason = "selected reachable frontier viewpoint";
            }
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
                      << "frontiers=" << frontier_output_count
                      << ", clusters=" << clusters.size()
                  << ", raw_clusters=" << raw_cluster_count
                  << ", candidates=" << candidates.size() << std::endl;
        }
        log_frontend_decision("no_reachable_frontier",
                              goal.reason,
                              nullptr,
                              static_cast<int>(candidates.size()),
                              checked,
                              astar_checked,
                              0);
        return finish_failure(goal.reason);
    }

    for (ExplorationGoal &candidate : reachable_candidates) {
        candidate.valid = true;
        candidate.checked_candidate_count = checked;
        candidate.astar_check_count = astar_checked;
        candidate.reachable_candidate_count =
                static_cast<int>(reachable_candidates.size());
        candidate.cluster_count = static_cast<int>(clusters.size());
        candidate.raw_cluster_count = static_cast<int>(raw_cluster_count);
        candidate.frontier_cell_count = frontier_output_count;
        candidate.raw_frontier_cell_count = search_stats.raw_frontier_cells;
    }

    exploration_finished_ = false;
    out.valid = true;
    out.exploration_finished = false;
    out.reason = "reachable_frontier_candidates";
    out.suggested_goal = ExplorationGoal{};
    out.candidates = reachable_candidates;
    out.checked_candidate_count = checked;
    out.astar_check_count = astar_checked;
    out.reachable_candidate_count =
            static_cast<int>(reachable_candidates.size());
    out.cluster_count = static_cast<int>(clusters.size());
    out.raw_cluster_count = static_cast<int>(raw_cluster_count);
    out.frontier_cell_count = frontier_output_count;
    out.raw_frontier_cell_count = search_stats.raw_frontier_cells;
    out.source = search_stats.source;
    last_candidate_set_ = out;

    if (cfg_.print_log) {
        std::cout << " -- [ExplorationFrontend] Candidate set generated: count="
                  << out.candidates.size()
                  << ", source=" << out.source
                  << ", checked=" << checked
                  << ", astar_checks=" << astar_checked
                  << ", reachable=" << reachable_candidates.size()
                  << ", clusters=" << clusters.size()
                  << ", raw_clusters=" << raw_cluster_count
                  << ", plan_ms=" << plan_elapsed_ms()
                  << ", sample_budget_hit="
                  << static_cast<int>(candidate_sampling_budget_hit)
                  << ", objects=" << object_stats.records
                  << ", dormant=" << object_stats.dormant
                  << ", covered=" << object_stats.covered
                  << ", stale=" << object_stats.stale
                  << ", dormant_bootstrap="
                  << static_cast<int>(using_dormant_bootstrap_clusters)
                  << ", bootstrap_candidates="
                  << static_cast<int>(used_bootstrap_candidates)
                  << ", bootstrap_attempts=" << bootstrap_attempt_count
                  << ", expansion_attempts=" << expansion_attempt_count
                  << ", expansion_added=" << expansion_added_count
                  << ", expansion_memory_added=" << expansion_memory_added_count
                  << ", frontiers=" << frontier_output_count
                  << ", raw_frontiers=" << search_stats.raw_frontier_cells
                  << ", fallback=" << static_cast<int>(search_stats.fallback_used)
                  << std::endl;
    }
    log_frontend_decision("candidates_generated",
                          out.reason,
                          nullptr,
                          static_cast<int>(candidates.size()),
                          checked,
                          astar_checked,
                          static_cast<int>(reachable_candidates.size()));
    return true;
}

bool ExplorationFrontend::planNextGoal(const StatePVAJ &robot_state,
                                       const double current_yaw,
                                       ExplorationGoal &goal,
                                       const double stamp) {
    goal = ExplorationGoal{};

    ExplorationCandidateSet candidate_set;
    if (!generateCandidates(robot_state, current_yaw, stamp, candidate_set)) {
        goal.reason = candidate_set.reason.empty()
                              ? "no exploration candidates"
                              : candidate_set.reason;
        return false;
    }
    if (candidate_set.candidates.empty()) {
        goal.reason = candidate_set.reason.empty()
                              ? "no exploration candidates"
                              : candidate_set.reason;
        return false;
    }

    const Vec3f robot_pos = robot_state.col(0);
    ExplorationGoal best_goal;
    if (cfg_.use_atsp && candidate_set.candidates.size() > 1) {
        best_goal = selectGoalWithAtsp(robot_pos,
                                       current_yaw,
                                       candidate_set.candidates);
    } else {
        best_goal =
                *std::min_element(candidate_set.candidates.begin(),
                                  candidate_set.candidates.end(),
                                  [](const ExplorationGoal &lhs,
                                     const ExplorationGoal &rhs) {
                                      return lhs.score < rhs.score;
                                  });
    }

    best_goal.valid = true;
    best_goal.checked_candidate_count = candidate_set.checked_candidate_count;
    best_goal.astar_check_count = candidate_set.astar_check_count;
    best_goal.reachable_candidate_count =
            candidate_set.reachable_candidate_count;
    best_goal.cluster_count = candidate_set.cluster_count;
    best_goal.raw_cluster_count = candidate_set.raw_cluster_count;
    best_goal.frontier_cell_count = candidate_set.frontier_cell_count;
    best_goal.raw_frontier_cell_count =
            candidate_set.raw_frontier_cell_count;
    goal = best_goal;

    candidate_set.suggested_goal = best_goal;
    last_candidate_set_ = candidate_set;
    exploration_finished_ = false;

    if (cfg_.print_log) {
        std::cout << " -- [ExplorationFrontend] Goal selected: p=["
                  << goal.position.transpose() << "], yaw=" << goal.yaw
                  << ", candidate_id=" << goal.candidate_id
                  << ", frontier_id=" << goal.frontier_id
                  << ", key=" << goal.memory_key
                  << ", reason=" << goal.reason
                  << ", score=" << goal.score
                  << ", info=" << goal.information_gain
                  << ", area=" << goal.frontier_area
                  << ", visible=" << goal.visible_frontier_cell_count
                  << ", visible_ratio=" << goal.visible_frontier_ratio
                  << ", travel=" << goal.travel_cost
                  << ", yaw_cost=" << goal.yaw_cost
                  << ", curvature=" << goal.curvature_cost
                  << ", checked=" << goal.checked_candidate_count
                  << ", astar_checks=" << goal.astar_check_count
                  << ", reachable=" << goal.reachable_candidate_count
                  << ", clusters=" << goal.cluster_count
                  << ", raw_clusters=" << goal.raw_cluster_count
                  << ", frontiers=" << goal.frontier_cell_count
                  << ", source=" << candidate_set.source
                  << ", raw_frontiers=" << goal.raw_frontier_cell_count
                  << std::endl;
    }
    {
        std::lock_guard<std::mutex> lock(explorationFrontendLogMutex());
        std::ofstream &stream = explorationFrontendLogStream();
        if (stream.is_open()) {
            stream << std::fixed << std::setprecision(9)
                   << stamp << ","
                   << "goal_selected" << ","
                   << csvEscape(goal.reason) << ","
                   << robot_pos.x() << ","
                   << robot_pos.y() << ","
                   << robot_pos.z() << ","
                   << 1 << ","
                   << goal.position.x() << ","
                   << goal.position.y() << ","
                   << goal.position.z() << ","
                   << goal.yaw << ","
                   << goal.candidate_id << ","
                   << goal.frontier_id << ","
                   << goal.score << ","
                   << goal.information_gain << ","
                   << goal.travel_cost << ","
                   << goal.yaw_cost << ","
                   << goal.curvature_cost << ","
                   << goal.visible_frontier_cell_count << ","
                   << goal.visible_frontier_ratio << ","
                   << goal.frontier_area << ","
                   << candidate_set.candidates.size() << ","
                   << candidate_set.checked_candidate_count << ","
                   << candidate_set.astar_check_count << ","
                   << candidate_set.reachable_candidate_count << ","
                   << candidate_set.cluster_count << ","
                   << candidate_set.raw_cluster_count << ","
                   << candidate_set.frontier_cell_count << ","
                   << candidate_set.raw_frontier_cell_count << ","
                   << csvEscape(candidate_set.source) << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0 << ","
                   << 0.0
                   << std::endl;
        }
    }
    return true;
}

bool ExplorationFrontend::isExplorationFinished() const {
    return exploration_finished_;
}

bool ExplorationFrontend::getLastCandidateSet(ExplorationCandidateSet &candidate_set) const {
    candidate_set = last_candidate_set_;
    return candidate_set.valid && !candidate_set.candidates.empty();
}

void ExplorationFrontend::reset() {
    exploration_finished_ = false;
    last_candidate_set_ = ExplorationCandidateSet{};
    astar_failure_cache_.clear();
    cached_frontier_clusters_.clear();
    expansion_viewpoint_records_.clear();
    expansion_visit_records_.clear();
    frontier_cache_initialized_ = false;
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
    if (goal.reason.find("expansion") != std::string::npos ||
        goal.identity.intent_mode == "exploration_expansion") {
        rememberExpansionVisit(goal, stamp);
    }
    if (frontier_object_manager_ != nullptr) {
        frontier_object_manager_->recordCommitted(goal, stamp, goal_switched);
    }
}

void ExplorationFrontend::recordGoalFailed(const ExplorationGoal &goal,
                                           const double stamp) {
    if (goal.reason.find("expansion") != std::string::npos ||
        goal.identity.intent_mode == "exploration_expansion") {
        rememberExpansionVisit(goal, stamp);
        const double match_radius = std::max(0.1, cfg_.expansion_memory_match_radius);
        const double match_radius_sq = match_radius * match_radius;
        for (auto it = expansion_viewpoint_records_.begin();
             it != expansion_viewpoint_records_.end();) {
            if ((it->goal.position - goal.position).squaredNorm() <= match_radius_sq) {
                it = expansion_viewpoint_records_.erase(it);
            } else {
                ++it;
            }
        }
    }
    if (frontier_object_manager_ != nullptr) {
        frontier_object_manager_->recordFailed(goal, stamp);
    }
}

bool ExplorationFrontend::collectFrontierCells(const Vec3f &robot_pos,
                                               vec_E<FrontierCell> &frontier_cells,
                                               FrontierSearchStats &stats) {
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
    stats.source = "planner_scan";
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
                    ++stats.raw_frontier_cells;
                    frontier_cells.push_back(cell);
                }
            }
        }
    }

    const int max_frontier_cells =
            cfg_.max_frontier_cells > 0
                    ? cfg_.max_frontier_cells
                    : std::numeric_limits<int>::max();
    if (static_cast<int>(frontier_cells.size()) > max_frontier_cells) {
        const double preferred_distance =
                std::max(cfg_.min_goal_distance, cfg_.preferred_goal_distance);
        std::sort(frontier_cells.begin(),
                  frontier_cells.end(),
                  [&robot_pos, preferred_distance](const FrontierCell &lhs,
                                                   const FrontierCell &rhs) {
                      const double lhs_distance =
                              (lhs.position - robot_pos).norm();
                      const double rhs_distance =
                              (rhs.position - robot_pos).norm();
                      const double lhs_score =
                              std::abs(lhs_distance - preferred_distance) +
                              0.01 * lhs_distance;
                      const double rhs_score =
                              std::abs(rhs_distance - preferred_distance) +
                              0.01 * rhs_distance;
                      if (std::abs(lhs_score - rhs_score) > 1.0e-6) {
                          return lhs_score < rhs_score;
                      }
                      return lhs_distance > rhs_distance;
                  });
        frontier_cells.resize(static_cast<size_t>(max_frontier_cells));
    }
    stats.frontier_cells = static_cast<int>(frontier_cells.size());
    return true;
}

bool ExplorationFrontend::collectRogMapFrontierCells(const Vec3f &robot_pos,
                                                     vec_E<FrontierCell> &frontier_cells,
                                                     FrontierSearchStats &stats) {
    frontier_cells.clear();
    stats = FrontierSearchStats{};
    stats.source = "rog_map_frontier";
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }
    if (!map_manager_->getMapConfig().frontier_extraction_en) {
        return true;
    }

    if (!updateIncrementalRogFrontiers(robot_pos, stats)) {
        return false;
    }
    appendCachedFrontierCells(robot_pos, frontier_cells, stats);
    return true;
}

bool ExplorationFrontend::collectRogMapFrontierClusters(const Vec3f &robot_pos,
                                                        vec_E<FrontierCluster> &clusters,
                                                        FrontierSearchStats &stats) {
    clusters.clear();
    stats = FrontierSearchStats{};
    stats.source = "rog_map_frontier_incremental";
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }
    if (!map_manager_->getMapConfig().frontier_extraction_en) {
        return true;
    }

    if (!updateIncrementalRogFrontiers(robot_pos, stats)) {
        return false;
    }
    appendCachedFrontierClusters(robot_pos, clusters, stats);
    return true;
}

bool ExplorationFrontend::updateIncrementalRogFrontiers(const Vec3f &robot_pos,
                                                        FrontierSearchStats &stats) {
    if (map_manager_ == nullptr || !map_manager_->ready()) {
        return false;
    }

    stats.source = "rog_map_frontier_incremental";
    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    Vec3f update_min = Vec3f::Zero();
    Vec3f update_max = Vec3f::Zero();
    bool full_rebuild = !frontier_cache_initialized_;
    bool has_update_box =
            !full_rebuild && map_manager_->getUpdatedBox(update_min, update_max);

    if (!has_update_box) {
        if (frontier_cache_initialized_) {
            return true;
        }
        update_min = robot_pos - Vec3f::Constant(cfg_.frontier_search_radius);
        update_max = robot_pos + Vec3f::Constant(cfg_.frontier_search_radius);
        full_rebuild = true;
        has_update_box = true;
    }

    const double update_margin =
            full_rebuild ? 0.0 : std::max(map_res, cfg_.frontier_cluster_radius);
    update_min -= Vec3f::Constant(update_margin);
    update_max += Vec3f::Constant(update_margin);
    map_manager_->boundBoxByLocalMap(update_min, update_max);
    if (!clipTaskSearchBox(update_min, update_max)) {
        frontier_cache_initialized_ = true;
        return true;
    }
    if ((update_max - update_min).minCoeff() <= 0.0) {
        frontier_cache_initialized_ = true;
        return true;
    }

    if (full_rebuild) {
        cached_frontier_clusters_.clear();
    } else {
        vec_E<FrontierCluster> kept_clusters;
        kept_clusters.reserve(cached_frontier_clusters_.size());
        const double keep_radius =
                std::max(0.0, cfg_.frontier_search_radius) +
                2.0 * std::max(map_res, cfg_.frontier_cluster_radius);
        const double keep_radius_sq = keep_radius * keep_radius;
        for (const FrontierCluster &cluster : cached_frontier_clusters_) {
            if (!cluster.center.allFinite() ||
                !map_manager_->insideLocalMap(cluster.center) ||
                !insideTaskRegion(cluster.center) ||
                (cluster.center - robot_pos).squaredNorm() > keep_radius_sq) {
                continue;
            }
            if (frontierClusterOverlapsBox(cluster, update_min, update_max) &&
                frontierClusterChanged(cluster)) {
                continue;
            }
            kept_clusters.push_back(cluster);
        }
        cached_frontier_clusters_.swap(kept_clusters);
    }

    size_t retained_raw_cells = 0U;
    for (const FrontierCluster &cluster : cached_frontier_clusters_) {
        const vec_E<FrontierCell> &raw_cells =
                cluster.raw_cells.empty() ? cluster.cells : cluster.raw_cells;
        retained_raw_cells += raw_cells.size();
    }
    std::unordered_set<GridKey, GridKeyHasher> frontier_flags;
    frontier_flags.reserve(retained_raw_cells + 1024U);
    for (const FrontierCluster &cluster : cached_frontier_clusters_) {
        const vec_E<FrontierCell> &raw_cells =
                cluster.raw_cells.empty() ? cluster.cells : cluster.raw_cells;
        for (const FrontierCell &cell : raw_cells) {
            frontier_flags.insert(makeKey(cell.index));
        }
    }

    Vec3i min_id;
    Vec3i max_id;
    map_manager_->probMapPosToGlobalIndex(update_min, min_id);
    map_manager_->probMapPosToGlobalIndex(update_max, max_id);
    for (int axis = 0; axis < 3; ++axis) {
        if (min_id(axis) > max_id(axis)) {
            std::swap(min_id(axis), max_id(axis));
        }
    }

    const int min_cluster_size = std::max(1, cfg_.min_frontier_cluster_size);
    const double patch_radius =
            cfg_.frontier_subcluster_size > map_res
                    ? cfg_.frontier_subcluster_size
                    : std::numeric_limits<double>::infinity();
    const int max_cluster_count = cfg_.max_frontier_clusters > 0
                                          ? cfg_.max_frontier_clusters
                                          : 64;
    const int raw_cluster_cap =
            cfg_.max_raw_frontier_points > 0
                    ? std::max(min_cluster_size,
                               std::max(256, cfg_.max_raw_frontier_points /
                                                     std::max(1, max_cluster_count)))
                    : std::numeric_limits<int>::max();
    const int raw_frontier_budget =
            cfg_.max_raw_frontier_points > 0
                    ? cfg_.max_raw_frontier_points
                    : std::numeric_limits<int>::max();
    bool raw_budget_exhausted = false;

    const auto collect_frontier_cluster = [&](const Vec3i &seed,
                                              FrontierCluster &cluster) {
        cluster = FrontierCluster{};
        std::queue<Vec3i> queue;
        Vec3f seed_position = Vec3f::Zero();
        map_manager_->probMapGlobalIndexToPos(seed, seed_position);
        const auto try_push = [&](const Vec3i &idx, std::queue<Vec3i> &target_queue) {
            if (static_cast<int>(cluster.raw_cells.size()) >= raw_cluster_cap) {
                return;
            }
            const GridKey key = makeKey(idx);
            if (frontier_flags.find(key) != frontier_flags.end() ||
                !map_manager_->insideLocalMap(idx)) {
                return;
            }
            FrontierCell cell;
            cell.index = idx;
            map_manager_->probMapGlobalIndexToPos(cell.index, cell.position);
            if (!cell.position.allFinite() ||
                !insideTaskRegion(cell.position)) {
                return;
            }
            if (std::isfinite(patch_radius)) {
                const Vec3f delta = cell.position - seed_position;
                if (std::abs(delta.x()) > patch_radius ||
                    std::abs(delta.y()) > patch_radius ||
                    std::abs(delta.z()) > patch_radius) {
                    return;
                }
            }
            const rog_map::GridType grid_type = map_manager_->getGridType(cell.position);
            if (!isFrontierCell(cell, grid_type)) {
                return;
            }
            frontier_flags.insert(key);
            cluster.raw_cells.push_back(cell);
            target_queue.push(idx);
        };

        try_push(seed, queue);
        while (!queue.empty()) {
            const Vec3i current = queue.front();
            queue.pop();
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        try_push(current + Vec3i(dx, dy, dz), queue);
                    }
                }
            }
        }
        cluster.cells = cluster.raw_cells;
        return !cluster.raw_cells.empty();
    };

    for (int ix = min_id.x(); ix <= max_id.x() && !raw_budget_exhausted; ++ix) {
        for (int iy = min_id.y(); iy <= max_id.y() && !raw_budget_exhausted; ++iy) {
            for (int iz = min_id.z(); iz <= max_id.z() && !raw_budget_exhausted; ++iz) {
                FrontierCell seed;
                seed.index = Vec3i(ix, iy, iz);
                if (frontier_flags.find(makeKey(seed.index)) != frontier_flags.end() ||
                    !map_manager_->insideLocalMap(seed.index)) {
                    continue;
                }
                map_manager_->probMapGlobalIndexToPos(seed.index, seed.position);
                if (!seed.position.allFinite() ||
                    !insideTaskRegion(seed.position)) {
                    continue;
                }

                ++stats.searched_cells;
                const rog_map::GridType grid_type = map_manager_->getGridType(seed.position);
                if (isFreeLike(grid_type)) {
                    ++stats.known_free_cells;
                } else if (isUnknownLike(grid_type)) {
                    ++stats.unknown_cells;
                } else if (grid_type == rog_map::GridType::OCCUPIED) {
                    ++stats.occupied_cells;
                }
                if (!isFrontierCell(seed, grid_type)) {
                    continue;
                }

                FrontierCluster cluster;
                if (!collect_frontier_cluster(seed.index, cluster)) {
                    continue;
                }
                stats.raw_frontier_cells += static_cast<int>(cluster.raw_cells.size());
                if (stats.raw_frontier_cells >= raw_frontier_budget) {
                    raw_budget_exhausted = true;
                }
                if (static_cast<int>(cluster.raw_cells.size()) < min_cluster_size) {
                    continue;
                }
                downsampleFrontierCluster(cluster);
                if (!finalizeFrontierCluster(cluster)) {
                    continue;
                }
                cached_frontier_clusters_.push_back(std::move(cluster));
            }
        }
    }

    frontier_cache_initialized_ = true;
    return true;
}

bool ExplorationFrontend::frontierClusterChanged(const FrontierCluster &cluster) const {
    const vec_E<FrontierCell> &raw_cells =
            cluster.raw_cells.empty() ? cluster.cells : cluster.raw_cells;
    if (raw_cells.empty()) {
        return true;
    }
    const int stride = boundedStride(raw_cells.size(), 192);
    int checked = 0;
    int changed = 0;
    for (size_t i = 0; i < raw_cells.size(); i += static_cast<size_t>(stride)) {
        const FrontierCell &cell = raw_cells[i];
        ++checked;
        if (!cell.position.allFinite() ||
            !map_manager_->insideLocalMap(cell.position) ||
            !insideTaskRegion(cell.position)) {
            ++changed;
            continue;
        }
        if (!isFrontierCell(cell, map_manager_->getGridType(cell.position))) {
            ++changed;
        }
    }
    if (checked == 0) {
        return true;
    }
    const double min_changed_fraction =
            std::clamp(cfg_.frontier_manager_min_changed_fraction, 0.05, 1.0);
    return static_cast<double>(changed) / static_cast<double>(checked) >=
           min_changed_fraction;
}

bool ExplorationFrontend::frontierClusterOverlapsBox(const FrontierCluster &cluster,
                                                     const Vec3f &box_min,
                                                     const Vec3f &box_max) const {
    Vec3f bmin = cluster.bbox_min;
    Vec3f bmax = cluster.bbox_max;
    if (cluster.raw_cells.empty() && cluster.cells.empty()) {
        return false;
    }
    const double margin = std::max(1.0e-3, map_manager_->getResolution());
    bmin -= Vec3f::Constant(margin);
    bmax += Vec3f::Constant(margin);
    for (int axis = 0; axis < 3; ++axis) {
        if (std::max(bmin(axis), box_min(axis)) >
            std::min(bmax(axis), box_max(axis)) + 1.0e-3) {
            return false;
        }
    }
    return true;
}

void ExplorationFrontend::downsampleFrontierCluster(FrontierCluster &cluster) const {
    if (cluster.raw_cells.empty()) {
        cluster.raw_cells = cluster.cells;
    }
    if (cluster.raw_cells.empty()) {
        return;
    }
    const double leaf_size =
            std::max(std::max(1.0e-3, map_manager_->getResolution()),
                     cfg_.frontier_sample_resolution);
    std::unordered_map<GridKey, FrontierCell, GridKeyHasher> buckets;
    buckets.reserve(cluster.raw_cells.size());
    for (const FrontierCell &cell : cluster.raw_cells) {
        const GridKey key = makeBucketKey(cell.position, leaf_size);
        if (buckets.find(key) == buckets.end()) {
            buckets.emplace(key, cell);
        }
    }
    cluster.cells.clear();
    cluster.cells.reserve(buckets.size());
    for (const auto &entry : buckets) {
        cluster.cells.push_back(entry.second);
    }
}

void ExplorationFrontend::appendCachedFrontierCells(const Vec3f &robot_pos,
                                                    vec_E<FrontierCell> &frontier_cells,
                                                    FrontierSearchStats &stats) const {
    frontier_cells.clear();
    if (cached_frontier_clusters_.empty()) {
        stats.frontier_cells = 0;
        return;
    }

    std::vector<const FrontierCluster *> active_clusters;
    active_clusters.reserve(cached_frontier_clusters_.size());
    const double radius_sq = cfg_.frontier_search_radius * cfg_.frontier_search_radius;
    for (const FrontierCluster &cluster : cached_frontier_clusters_) {
        if (cluster.cells.empty() ||
            !cluster.center.allFinite() ||
            !map_manager_->insideLocalMap(cluster.center) ||
            !insideTaskRegion(cluster.center) ||
            (cluster.center - robot_pos).squaredNorm() > radius_sq) {
            continue;
        }
        active_clusters.push_back(&cluster);
    }
    std::sort(active_clusters.begin(), active_clusters.end(),
              [this, &robot_pos](const FrontierCluster *lhs, const FrontierCluster *rhs) {
                  const double lhs_score = clusterPriorityScore(*lhs, robot_pos);
                  const double rhs_score = clusterPriorityScore(*rhs, robot_pos);
                  if (std::abs(lhs_score - rhs_score) > 1.0e-6) {
                      return lhs_score < rhs_score;
                  }
                  return lhs->size > rhs->size;
              });

    const int max_cluster_count = cfg_.max_frontier_clusters > 0
                                          ? cfg_.max_frontier_clusters
                                          : std::numeric_limits<int>::max();
    const int max_frontier_cells = cfg_.max_frontier_cells > 0
                                           ? cfg_.max_frontier_cells
                                           : std::numeric_limits<int>::max();
    int selected_clusters = 0;
    for (const FrontierCluster *cluster : active_clusters) {
        if (selected_clusters >= max_cluster_count ||
            static_cast<int>(frontier_cells.size()) >= max_frontier_cells) {
            break;
        }
        ++selected_clusters;
        for (const FrontierCell &cell : cluster->cells) {
            if (static_cast<int>(frontier_cells.size()) >= max_frontier_cells) {
                break;
            }
            frontier_cells.push_back(cell);
        }
    }
    stats.frontier_cells = static_cast<int>(frontier_cells.size());
}

void ExplorationFrontend::appendCachedFrontierClusters(const Vec3f &robot_pos,
                                                       vec_E<FrontierCluster> &clusters,
                                                       FrontierSearchStats &stats) const {
    clusters.clear();
    stats.frontier_cells = 0;
    if (cached_frontier_clusters_.empty()) {
        return;
    }

    std::vector<const FrontierCluster *> active_clusters;
    active_clusters.reserve(cached_frontier_clusters_.size());
    const double radius_sq = cfg_.frontier_search_radius * cfg_.frontier_search_radius;
    for (const FrontierCluster &cluster : cached_frontier_clusters_) {
        if (cluster.cells.empty() ||
            !cluster.center.allFinite() ||
            !map_manager_->insideLocalMap(cluster.center) ||
            !insideTaskRegion(cluster.center) ||
            (cluster.center - robot_pos).squaredNorm() > radius_sq) {
            continue;
        }
        active_clusters.push_back(&cluster);
    }
    std::sort(active_clusters.begin(), active_clusters.end(),
              [this, &robot_pos](const FrontierCluster *lhs, const FrontierCluster *rhs) {
                  const double lhs_score = clusterPriorityScore(*lhs, robot_pos);
                  const double rhs_score = clusterPriorityScore(*rhs, robot_pos);
                  if (std::abs(lhs_score - rhs_score) > 1.0e-6) {
                      return lhs_score < rhs_score;
                  }
                  return lhs->size > rhs->size;
              });

    const int max_cluster_count = cfg_.max_frontier_clusters > 0
                                          ? cfg_.max_frontier_clusters
                                          : std::numeric_limits<int>::max();
    const int max_frontier_cells = cfg_.max_frontier_cells > 0
                                           ? cfg_.max_frontier_cells
                                           : std::numeric_limits<int>::max();
    clusters.reserve(std::min(active_clusters.size(),
                              static_cast<size_t>(std::max(0, max_cluster_count))));
    for (const FrontierCluster *cluster : active_clusters) {
        if (static_cast<int>(clusters.size()) >= max_cluster_count ||
            stats.frontier_cells >= max_frontier_cells) {
            break;
        }
        if (!clusters.empty() &&
            stats.frontier_cells + static_cast<int>(cluster->cells.size()) >
                    max_frontier_cells) {
            break;
        }
        clusters.push_back(*cluster);
        stats.frontier_cells += static_cast<int>(cluster->cells.size());
    }
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
    return frontier_cache_initialized_ || stats.known_free_cells >= min_known_free_cells;
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
                viewpoint.z() = robot_pos.z() + cfg_.viewpoint_height_offset + z_offset;
                clampViewpointToTaskRegion(viewpoint);
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
                candidate.identity.intent_mode = "exploration";
                candidate.identity.candidate_id = candidate.candidate_id;
                candidate.identity.frontier_id = candidate.frontier_id;
                candidate.identity.candidate_key = "candidate:" + candidate.memory_key;
                candidate.identity.frontier_key =
                        cluster.object_id >= 0
                                ? std::string("frontier_object:") +
                                          std::to_string(cluster.object_id)
                                : "frontier:" + memoryKeyFromKey(frontier_key);
                candidate.identity.goal_key = candidate.identity.candidate_key;
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

void ExplorationFrontend::appendExpansionFallbackCandidates(
        const StatePVAJ &robot_state,
        const double current_yaw,
        const double stamp,
        const vec_E<FrontierCluster> &clusters,
        vec_E<ExplorationGoal> &candidates,
        int &attempt_count,
        int &added_count) {
    attempt_count = 0;
    added_count = 0;
    if (!cfg_.expansion_fallback_enable ||
        map_manager_ == nullptr ||
        !map_manager_->ready() ||
        static_cast<int>(candidates.size()) >= std::max(1, cfg_.max_candidate_num)) {
        return;
    }

    pruneExpansionViewpointMemory(stamp);
    pruneExpansionVisitMemory(stamp);

    const Vec3f robot_pos = robot_state.col(0);
    const int max_total_candidates = std::max(1, cfg_.max_candidate_num);
    const int max_add = std::min(std::max(1, cfg_.expansion_max_candidate_num),
                                 max_total_candidates - static_cast<int>(candidates.size()));
    if (max_add <= 0) {
        return;
    }

    const int yaw_num = std::max(4, cfg_.expansion_yaw_sample_num);
    const int radius_num = std::max(1, cfg_.expansion_radius_sample_num);
    const int z_num = std::max(1, cfg_.expansion_z_sample_num);
    const double min_radius = std::max(0.2, cfg_.expansion_min_radius);
    const double max_radius = std::max(min_radius, cfg_.expansion_max_radius);
    const double separation = std::max(0.0, cfg_.candidate_separation_distance);

    vec_E<ExplorationGoal> expansion_candidates;
    expansion_candidates.reserve(static_cast<size_t>(max_add));

    for (int ri = 0; ri < radius_num; ++ri) {
        const double radius_alpha =
                radius_num == 1
                        ? 0.0
                        : static_cast<double>(ri) / static_cast<double>(radius_num - 1);
        const double radius = min_radius + radius_alpha * (max_radius - min_radius);
        const double ring_phase = (ri % 2 == 0) ? 0.0 : (kPi / static_cast<double>(yaw_num));
        for (int zi = 0; zi < z_num; ++zi) {
            const double z_alpha =
                    z_num == 1
                            ? 0.5
                            : static_cast<double>(zi) / static_cast<double>(z_num - 1);
            const double z_offset =
                    cfg_.expansion_z_min +
                    z_alpha * (cfg_.expansion_z_max - cfg_.expansion_z_min);
            for (int yi = 0; yi < yaw_num; ++yi) {
                ++attempt_count;
                const double angle =
                        ring_phase +
                        2.0 * kPi * static_cast<double>(yi) / static_cast<double>(yaw_num);
                Vec3f viewpoint =
                        robot_pos + radius * Vec3f(std::cos(angle), std::sin(angle), 0.0);
                viewpoint.z() = robot_pos.z() + cfg_.viewpoint_height_offset + z_offset;
                clampViewpointToTaskRegion(viewpoint);

                ExplorationGoal candidate;
                if (!makeExpansionCandidate(robot_state,
                                            current_yaw,
                                            stamp,
                                            viewpoint,
                                            clusters,
                                            candidate)) {
                    continue;
                }
                if (!candidateSeparatedFromPool(candidate, candidates, separation) ||
                    !candidateSeparatedFromPool(candidate, expansion_candidates, separation)) {
                    continue;
                }
                expansion_candidates.push_back(candidate);
            }
        }
    }

    std::sort(expansion_candidates.begin(), expansion_candidates.end(),
              [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                  return lhs.score < rhs.score;
              });

    for (const ExplorationGoal &candidate : expansion_candidates) {
        if (added_count >= max_add ||
            static_cast<int>(candidates.size()) >= max_total_candidates) {
            break;
        }
        candidates.push_back(candidate);
        rememberExpansionCandidate(candidate, stamp);
        ++added_count;
    }
}

void ExplorationFrontend::appendRememberedExpansionCandidates(
        const StatePVAJ &robot_state,
        const double current_yaw,
        const double stamp,
        const vec_E<FrontierCluster> &clusters,
        vec_E<ExplorationGoal> &candidates,
        int &added_count) {
    added_count = 0;
    if (!cfg_.expansion_memory_enable ||
        expansion_viewpoint_records_.empty() ||
        map_manager_ == nullptr ||
        !map_manager_->ready()) {
        return;
    }

    pruneExpansionViewpointMemory(stamp);
    pruneExpansionVisitMemory(stamp);
    const int max_total_candidates = std::max(1, cfg_.max_candidate_num);
    if (static_cast<int>(candidates.size()) >= max_total_candidates) {
        return;
    }

    const int max_add = std::min(std::max(1, cfg_.expansion_max_candidate_num),
                                 max_total_candidates - static_cast<int>(candidates.size()));
    const double separation = std::max(0.0, cfg_.candidate_separation_distance);
    vec_E<ExplorationGoal> remembered_candidates;
    remembered_candidates.reserve(static_cast<size_t>(max_add));

    for (const ExpansionViewpointRecord &record : expansion_viewpoint_records_) {
        ExplorationGoal candidate;
        if (!makeExpansionCandidate(robot_state,
                                    current_yaw,
                                    stamp,
                                    record.goal.position,
                                    clusters,
                                    candidate)) {
            continue;
        }
        candidate.reason += " remembered_expansion_seen=" +
                            std::to_string(std::max(1, record.seen_count));
        if (!candidateSeparatedFromPool(candidate, candidates, separation) ||
            !candidateSeparatedFromPool(candidate, remembered_candidates, separation)) {
            continue;
        }
        remembered_candidates.push_back(candidate);
    }

    std::sort(remembered_candidates.begin(), remembered_candidates.end(),
              [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                  return lhs.score < rhs.score;
              });

    for (const ExplorationGoal &candidate : remembered_candidates) {
        if (added_count >= max_add ||
            static_cast<int>(candidates.size()) >= max_total_candidates) {
            break;
        }
        candidates.push_back(candidate);
        ++added_count;
    }
}

bool ExplorationFrontend::makeExpansionCandidate(
        const StatePVAJ &robot_state,
        const double current_yaw,
        const double stamp,
        const Vec3f &viewpoint,
        const vec_E<FrontierCluster> &clusters,
        ExplorationGoal &candidate) const {
    candidate = ExplorationGoal{};
    if (!viewpoint.allFinite() ||
        !expansionViewpointSafe(viewpoint, cfg_.expansion_allow_unknown_viewpoint)) {
        return false;
    }

    const Vec3f robot_pos = robot_state.col(0);
    const double distance_to_robot = (viewpoint - robot_pos).norm();
    if (distance_to_robot <= std::max(0.2, cfg_.goal_reached_distance)) {
        return false;
    }
    if (expansionBlockedByVisitMemory(viewpoint, stamp)) {
        return false;
    }

    const double local_unknown_gain =
            estimateLocalUnknownGain(viewpoint, cfg_.max_gain_rays);
    const int cluster_index = nearestClusterIndex(viewpoint, clusters);

    FrontierCluster reference_cluster;
    bool has_reference_frontier = false;
    int visible_count = 0;
    double visible_ratio = 0.0;
    double frontier_gain = 0.0;
    if (cluster_index >= 0) {
        const FrontierCluster &cluster = clusters[static_cast<size_t>(cluster_index)];
        visible_count = countVisibleFrontierCells(viewpoint, cluster, cfg_.max_gain_rays);
        const int visibility_denominator =
                sampledCount(cluster.cells.size(), std::max(1, cfg_.max_gain_rays));
        visible_ratio = visibility_denominator <= 0
                                ? 0.0
                                : static_cast<double>(visible_count) /
                                          static_cast<double>(visibility_denominator);
        if (visible_count > 0) {
            frontier_gain = estimateInformationGain(viewpoint, cluster);
            reference_cluster = cluster;
            has_reference_frontier = true;
        }
    }

    if (!has_reference_frontier) {
        const Vec3f motion_dir = horizontalNormalized(viewpoint - robot_pos,
                                                      Vec3f::UnitX());
        reference_cluster.center =
                viewpoint + std::max(1.0, 0.5 * cfg_.gain_ray_length) * motion_dir;
        reference_cluster.unknown_direction = motion_dir;
        reference_cluster.normal = motion_dir;
        reference_cluster.free_direction = -motion_dir;
        reference_cluster.bbox_min = reference_cluster.center - Vec3f::Constant(0.5);
        reference_cluster.bbox_max = reference_cluster.center + Vec3f::Constant(0.5);
        reference_cluster.extent = Vec3f::Constant(1.0);
        reference_cluster.area = 0.0;
        reference_cluster.size = 0;
    }

    const double synthetic_gain_ratio =
            std::clamp(cfg_.expansion_synthetic_gain_ratio, 0.05, 1.0);
    const double synthetic_gain_cap =
            std::max(cfg_.expansion_min_information_gain,
                     std::max(1.0, cfg_.information_gain_saturation) *
                             synthetic_gain_ratio);
    const double expansion_unknown_gain =
            has_reference_frontier ? local_unknown_gain
                                   : std::min(local_unknown_gain, synthetic_gain_cap);
    const double information_gain =
            expansion_unknown_gain + 0.5 * frontier_gain +
            0.25 * static_cast<double>(visible_count);
    const double min_expansion_gain =
            std::max(0.0, cfg_.expansion_min_information_gain);
    if (information_gain < min_expansion_gain ||
        information_gain < std::max(0.0, cfg_.min_information_gain)) {
        return false;
    }

    const GridKey candidate_key =
            quantizedKey(viewpoint, std::max(0.25, cfg_.map_resolution));
    const GridKey frontier_key =
            has_reference_frontier
                    ? quantizedKey(reference_cluster.center,
                                   std::max(0.25, cfg_.frontier_cluster_radius))
                    : quantizedKey(reference_cluster.center,
                                   std::max(1.0, cfg_.expansion_memory_match_radius));

    candidate.valid = true;
    candidate.position = viewpoint;
    candidate.yaw = resolveCandidateYaw(current_yaw,
                                        robot_pos,
                                        viewpoint,
                                        reference_cluster);
    candidate.candidate_id = stableIdFromKey(candidate_key);
    candidate.frontier_id =
            has_reference_frontier && reference_cluster.object_id >= 0
                    ? reference_cluster.object_id
                    : stableIdFromKey(frontier_key);
    candidate.memory_key = memoryKeyFromKey(candidate_key);
    candidate.identity.intent_mode = "exploration_expansion";
    candidate.identity.candidate_id = candidate.candidate_id;
    candidate.identity.frontier_id = candidate.frontier_id;
    candidate.identity.candidate_key = "candidate:" + candidate.memory_key;
    candidate.identity.frontier_key =
            has_reference_frontier && reference_cluster.object_id >= 0
                    ? std::string("frontier_object:") +
                              std::to_string(reference_cluster.object_id)
                    : "expansion:" + memoryKeyFromKey(frontier_key);
    candidate.identity.goal_key = candidate.identity.candidate_key;
    candidate.identity.guide_path_key =
            nhbp::makeGuidePathKey(vec_E<Vec3f>{robot_pos, viewpoint}, cfg_.map_resolution);
    candidate.frontier_center_valid = has_reference_frontier;
    candidate.frontier_center = has_reference_frontier ? reference_cluster.center
                                                       : viewpoint;
    candidate.frontier_bbox_min = has_reference_frontier
                                          ? reference_cluster.bbox_min
                                          : viewpoint - Vec3f::Constant(0.5);
    candidate.frontier_bbox_max = has_reference_frontier
                                          ? reference_cluster.bbox_max
                                          : viewpoint + Vec3f::Constant(0.5);
    candidate.frontier_area = has_reference_frontier ? reference_cluster.area : 0.0;
    candidate.visible_frontier_cell_count =
            std::max(visible_count,
                     static_cast<int>(std::round(std::min(1000.0,
                                                          expansion_unknown_gain))));
    candidate.visible_frontier_ratio =
            has_reference_frontier
                    ? visible_ratio
                    : std::min(1.0,
                               expansion_unknown_gain /
                                       std::max(1.0, static_cast<double>(cfg_.max_gain_rays)));
    candidate.information_gain = information_gain;
    candidate.distance_to_robot = distance_to_robot;
    candidate.travel_cost = distance_to_robot;
    candidate.guide_path = vec_E<Vec3f>{robot_pos, viewpoint};
    candidate.yaw_cost = estimateYawCost(current_yaw, candidate.yaw);
    candidate.curvature_cost = estimateCurvatureCost(robot_state,
                                                     viewpoint,
                                                     reference_cluster);
    candidate.history_score_delta =
            has_reference_frontier ? reference_cluster.object_score_delta : 0.0;
    const double unknown_risk = estimateUnknownRisk(viewpoint);
    candidate.score = scoreCandidate(candidate, unknown_risk) +
                      candidate.history_score_delta;
    candidate.reason =
            "expansion fallback viewpoint local_unknown_gain=" +
            std::to_string(local_unknown_gain) +
            " capped_unknown_gain=" + std::to_string(expansion_unknown_gain) +
            " frontier_gain=" + std::to_string(frontier_gain);
    return true;
}

bool ExplorationFrontend::expansionViewpointSafe(const Vec3f &viewpoint,
                                                 const bool allow_unknown) const {
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
    if (!allow_unknown && !isFreeLike(grid_type)) {
        return false;
    }

    const rog_map::GridType inf_type = map_manager_->getInfGridType(viewpoint);
    if (inf_type == rog_map::GridType::OCCUPIED ||
        inf_type == rog_map::GridType::OUT_OF_MAP) {
        return false;
    }
    if (!allow_unknown && !isFreeLike(inf_type)) {
        return false;
    }

    if (map_manager_->hasESDF() && cfg_.viewpoint_safe_distance > 0.0) {
        double dist = 0.0;
        Vec3f grad = Vec3f::Zero();
        const bool esdf_ok = map_manager_->evaluateESDF(viewpoint, dist, grad);
        if (!esdf_ok) {
            return allow_unknown;
        }
        if (!std::isfinite(dist) || dist < cfg_.viewpoint_safe_distance) {
            return false;
        }
    }
    return true;
}

bool ExplorationFrontend::candidateSeparatedFromPool(
        const ExplorationGoal &candidate,
        const vec_E<ExplorationGoal> &pool,
        const double separation) const {
    const double separation_sq = std::max(0.0, separation) * std::max(0.0, separation);
    for (const ExplorationGoal &other : pool) {
        if (candidate.candidate_id >= 0 &&
            other.candidate_id == candidate.candidate_id) {
            return false;
        }
        if (!candidate.memory_key.empty() &&
            candidate.memory_key == other.memory_key) {
            return false;
        }
        if (separation > 1.0e-6 &&
            (candidate.position - other.position).squaredNorm() < separation_sq) {
            return false;
        }
    }
    return true;
}

int ExplorationFrontend::nearestClusterIndex(const Vec3f &viewpoint,
                                             const vec_E<FrontierCluster> &clusters) const {
    int best_index = -1;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < clusters.size(); ++i) {
        const FrontierCluster &cluster = clusters[i];
        if (!cluster.center.allFinite()) {
            continue;
        }
        const double distance_sq = (cluster.center - viewpoint).squaredNorm();
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}

void ExplorationFrontend::rememberExpansionCandidate(const ExplorationGoal &candidate,
                                                     const double stamp) {
    if (!cfg_.expansion_memory_enable ||
        !candidate.valid ||
        !candidate.position.allFinite()) {
        return;
    }

    const double match_radius = std::max(0.1, cfg_.expansion_memory_match_radius);
    const double match_radius_sq = match_radius * match_radius;
    for (ExpansionViewpointRecord &record : expansion_viewpoint_records_) {
        if ((record.goal.position - candidate.position).squaredNorm() <= match_radius_sq) {
            record.goal = candidate;
            record.last_seen_stamp = stamp;
            ++record.seen_count;
            return;
        }
    }

    ExpansionViewpointRecord record;
    record.goal = candidate;
    record.last_seen_stamp = stamp;
    record.seen_count = 1;
    expansion_viewpoint_records_.push_back(record);
    pruneExpansionViewpointMemory(stamp);
}

void ExplorationFrontend::pruneExpansionViewpointMemory(const double stamp) {
    if (!cfg_.expansion_memory_enable) {
        expansion_viewpoint_records_.clear();
        return;
    }

    const double ttl = std::max(0.0, cfg_.expansion_memory_ttl);
    if (ttl > 0.0 && stamp > 0.0) {
        vec_E<ExpansionViewpointRecord> kept_records;
        kept_records.reserve(expansion_viewpoint_records_.size());
        for (const ExpansionViewpointRecord &record : expansion_viewpoint_records_) {
            if (!record.goal.valid || !record.goal.position.allFinite()) {
                continue;
            }
            if (record.last_seen_stamp > 0.0 &&
                stamp - record.last_seen_stamp > ttl) {
                continue;
            }
            kept_records.push_back(record);
        }
        expansion_viewpoint_records_.swap(kept_records);
    }

    const int max_records = std::max(0, cfg_.expansion_memory_max_records);
    if (max_records == 0) {
        expansion_viewpoint_records_.clear();
        return;
    }
    if (static_cast<int>(expansion_viewpoint_records_.size()) <= max_records) {
        return;
    }

    std::sort(expansion_viewpoint_records_.begin(), expansion_viewpoint_records_.end(),
              [](const ExpansionViewpointRecord &lhs,
                 const ExpansionViewpointRecord &rhs) {
                  if (lhs.last_seen_stamp != rhs.last_seen_stamp) {
                      return lhs.last_seen_stamp > rhs.last_seen_stamp;
                  }
                  return lhs.goal.information_gain > rhs.goal.information_gain;
    });
    expansion_viewpoint_records_.resize(static_cast<size_t>(max_records));
}

bool ExplorationFrontend::expansionBlockedByVisitMemory(
        const Vec3f &viewpoint,
        const double stamp) const {
    if (!cfg_.expansion_anti_revisit_enable ||
        !viewpoint.allFinite() ||
        expansion_visit_records_.empty()) {
        return false;
    }

    const int retire_after_commits =
            std::max(1, cfg_.expansion_retire_after_commits);
    const double block_radius =
            std::max(0.1, cfg_.expansion_commit_block_radius);
    const double block_radius_sq = block_radius * block_radius;
    const double ttl = std::max(0.0, cfg_.expansion_commit_block_ttl);

    for (const ExpansionVisitRecord &record : expansion_visit_records_) {
        if (!record.position.allFinite() ||
            record.commit_count < retire_after_commits) {
            continue;
        }
        if (ttl > 0.0 &&
            stamp > 0.0 &&
            record.last_commit_stamp > 0.0 &&
            stamp - record.last_commit_stamp > ttl) {
            continue;
        }
        if ((record.position - viewpoint).squaredNorm() <= block_radius_sq) {
            return true;
        }
    }
    return false;
}

void ExplorationFrontend::rememberExpansionVisit(const ExplorationGoal &goal,
                                                 const double stamp) {
    if (!cfg_.expansion_anti_revisit_enable ||
        !goal.valid ||
        !goal.position.allFinite()) {
        return;
    }

    pruneExpansionVisitMemory(stamp);
    const double match_radius =
            std::max(0.1, cfg_.expansion_commit_block_radius);
    const double match_radius_sq = match_radius * match_radius;
    for (ExpansionVisitRecord &record : expansion_visit_records_) {
        if (!record.position.allFinite()) {
            continue;
        }
        if ((record.position - goal.position).squaredNorm() <= match_radius_sq) {
            const double count = static_cast<double>(std::max(1, record.commit_count));
            record.position =
                    (count * record.position + goal.position) / (count + 1.0);
            record.last_commit_stamp = stamp;
            ++record.commit_count;
            return;
        }
    }

    ExpansionVisitRecord record;
    record.position = goal.position;
    record.last_commit_stamp = stamp;
    record.commit_count = 1;
    expansion_visit_records_.push_back(record);
    pruneExpansionVisitMemory(stamp);
}

void ExplorationFrontend::pruneExpansionVisitMemory(const double stamp) {
    if (!cfg_.expansion_anti_revisit_enable) {
        expansion_visit_records_.clear();
        return;
    }

    const double ttl = std::max(0.0, cfg_.expansion_commit_block_ttl);
    if (ttl > 0.0 && stamp > 0.0) {
        vec_E<ExpansionVisitRecord> kept_records;
        kept_records.reserve(expansion_visit_records_.size());
        for (const ExpansionVisitRecord &record : expansion_visit_records_) {
            if (!record.position.allFinite()) {
                continue;
            }
            if (record.last_commit_stamp > 0.0 &&
                stamp - record.last_commit_stamp > ttl) {
                continue;
            }
            kept_records.push_back(record);
        }
        expansion_visit_records_.swap(kept_records);
    }

    const int max_records = std::max(1, cfg_.expansion_memory_max_records);
    if (static_cast<int>(expansion_visit_records_.size()) <= max_records) {
        return;
    }

    std::sort(expansion_visit_records_.begin(), expansion_visit_records_.end(),
              [](const ExpansionVisitRecord &lhs,
                 const ExpansionVisitRecord &rhs) {
                  if (lhs.last_commit_stamp != rhs.last_commit_stamp) {
                      return lhs.last_commit_stamp > rhs.last_commit_stamp;
                  }
                  return lhs.commit_count > rhs.commit_count;
              });
    expansion_visit_records_.resize(static_cast<size_t>(max_records));
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

double ExplorationFrontend::estimateLocalUnknownGain(const Vec3f &viewpoint,
                                                     const int max_rays) const {
    if (!viewpoint.allFinite() ||
        map_manager_ == nullptr ||
        !map_manager_->ready() ||
        !map_manager_->insideLocalMap(viewpoint) ||
        !insideTaskRegion(viewpoint)) {
        return 0.0;
    }

    const double map_res = std::max(1.0e-3, map_manager_->getResolution());
    const double ray_step = std::max(map_res, cfg_.gain_ray_step);
    const double ray_length = std::max(ray_step, cfg_.gain_ray_length);
    const int requested_rays = max_rays > 0 ? max_rays : cfg_.max_gain_rays;
    const int yaw_samples = std::clamp(requested_rays / 3, 8, 32);
    constexpr std::array<double, 3> pitch_samples = {-0.35, 0.0, 0.35};

    std::unordered_set<GridKey, GridKeyHasher> unknown_voxels;
    unknown_voxels.reserve(static_cast<size_t>(yaw_samples * 12));
    int unknown_ray_count = 0;
    int free_ray_prefix_count = 0;

    for (int yi = 0; yi < yaw_samples; ++yi) {
        const double yaw =
                2.0 * kPi * static_cast<double>(yi) / static_cast<double>(yaw_samples);
        for (const double pitch : pitch_samples) {
            const double cp = std::cos(pitch);
            const Vec3f direction(cp * std::cos(yaw),
                                  cp * std::sin(yaw),
                                  std::sin(pitch));
            bool ray_has_unknown = false;
            bool ray_has_free_prefix = false;
            for (double distance = ray_step; distance <= ray_length; distance += ray_step) {
                const Vec3f sample_pos = viewpoint + distance * direction;
                if (!map_manager_->insideLocalMap(sample_pos) ||
                    !insideTaskRegion(sample_pos)) {
                    break;
                }

                const rog_map::GridType grid_type = map_manager_->getGridType(sample_pos);
                const rog_map::GridType inf_type = map_manager_->getInfGridType(sample_pos);
                if (grid_type == rog_map::GridType::OCCUPIED ||
                    grid_type == rog_map::GridType::OUT_OF_MAP ||
                    inf_type == rog_map::GridType::OCCUPIED ||
                    inf_type == rog_map::GridType::OUT_OF_MAP) {
                    break;
                }

                if (isUnknownLike(grid_type)) {
                    unknown_voxels.insert(makeBucketKey(sample_pos, map_res));
                    ray_has_unknown = true;
                } else if (isFreeLike(grid_type)) {
                    ray_has_free_prefix = true;
                }
            }
            if (ray_has_unknown) {
                ++unknown_ray_count;
            }
            if (ray_has_free_prefix) {
                ++free_ray_prefix_count;
            }
        }
    }

    return static_cast<double>(unknown_voxels.size()) +
           0.5 * static_cast<double>(unknown_ray_count) +
           0.05 * static_cast<double>(free_ray_prefix_count);
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

    const double direct_distance = (viewpoint - robot_pos).norm();
    if (direct_distance <= std::max(1.0e-3, map_manager_->getResolution())) {
        guide_path.push_back(robot_pos);
        guide_path.push_back(viewpoint);
        return direct_distance;
    }

    const bool inflated_line_free = map_manager_->isLineFree(robot_pos, viewpoint, true, false);
    const bool known_line_free =
            !cfg_.unknown_as_occupied_for_motion ||
            map_manager_->isLineFree(robot_pos, viewpoint, false, true);
    if (inflated_line_free && known_line_free) {
        guide_path.push_back(robot_pos);
        guide_path.push_back(viewpoint);
        return direct_distance;
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
    const double search_horizon = std::max(cfg_.frontier_search_radius * 1.5,
                                           direct_distance * 1.8 + 2.0);
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
    if (policy == "free" ||
        policy == "free_yaw" ||
        policy == "nan") {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto finite_yaw_near_current = [current_yaw](const double yaw) {
        if (!std::isfinite(yaw)) {
            return std::isfinite(current_yaw) ? current_yaw : 0.0;
        }
        if (!std::isfinite(current_yaw)) {
            return std::atan2(std::sin(yaw), std::cos(yaw));
        }
        return current_yaw + ExplorationFrontend::wrapAngleDiff(yaw, current_yaw);
    };

    Vec3f motion_dir = viewpoint - robot_pos;
    motion_dir.z() = 0.0;
    Vec3f frontier_dir = cluster.center - viewpoint;
    frontier_dir.z() = 0.0;

    if (motion_dir.norm() < 1.0e-3) {
        return finite_yaw_near_current(current_yaw);
    }
    if (policy == "frontier" || policy == "frontier_center" || policy == "observe") {
        if (frontier_dir.norm() > 1.0e-3) {
            return finite_yaw_near_current(
                    std::atan2(frontier_dir.y(), frontier_dir.x()));
        }
        return finite_yaw_near_current(current_yaw);
    }
    if (policy == "blend" && frontier_dir.norm() > 1.0e-3) {
        const Vec3f blended = motion_dir.normalized() + 0.35 * frontier_dir.normalized();
        if (blended.head<2>().norm() > 1.0e-3) {
            return finite_yaw_near_current(std::atan2(blended.y(), blended.x()));
        }
    }
    return finite_yaw_near_current(std::atan2(motion_dir.y(), motion_dir.x()));
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
        std::ostringstream tour_key;
        tour_key << "atsp:" << solution.solver_status;
        for (const int ordered_id : solution.ordered_candidate_ids) {
            if (ordered_id >= 0 && ordered_id < candidate_num) {
                const std::string key =
                        reachable_candidates[ordered_id].identity.canonicalKey();
                tour_key << "|"
                         << (key.empty()
                                     ? std::to_string(reachable_candidates[ordered_id].candidate_id)
                                     : key);
            }
        }
        const int selected_id = solution.ordered_candidate_ids.front();
        if (selected_id >= 0 && selected_id < candidate_num) {
            ExplorationGoal selected = reachable_candidates[selected_id];
            selected.identity.tour_key = tour_key.str();
            selected.identity.tour_rank = 0;
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
    if (!position.allFinite()) {
        return false;
    }
    if (mission_manager_ != nullptr) {
        return mission_manager_->insideTaskBox(position);
    }
    if (!cfg_.global_box_enable) {
        return true;
    }
    return (position.array() >= cfg_.global_box_min.array()).all() &&
           (position.array() <= cfg_.global_box_max.array()).all();
}

bool ExplorationFrontend::clipTaskSearchBox(Vec3f &box_min, Vec3f &box_max) const {
    if (mission_manager_ != nullptr) {
        return mission_manager_->clipSearchBox(box_min, box_max);
    }
    if (!cfg_.global_box_enable) {
        return true;
    }
    box_min = box_min.cwiseMax(cfg_.global_box_min);
    box_max = box_max.cwiseMin(cfg_.global_box_max);
    return (box_max.array() >= box_min.array()).all();
}

void ExplorationFrontend::clampViewpointToTaskRegion(Vec3f &viewpoint) const {
    if (!viewpoint.allFinite() || !cfg_.global_box_enable) {
        return;
    }
    const double map_res =
            map_manager_ != nullptr && map_manager_->ready()
                    ? std::max(1.0e-3, map_manager_->getResolution())
                    : std::max(1.0e-3, cfg_.map_resolution);
    const double min_z = std::min(cfg_.global_box_min.z(), cfg_.global_box_max.z());
    const double max_z = std::max(cfg_.global_box_min.z(), cfg_.global_box_max.z());
    const double margin = 0.5 * map_res;
    if (max_z - min_z > 2.0 * margin) {
        viewpoint.z() = std::clamp(viewpoint.z(), min_z + margin, max_z - margin);
    } else {
        viewpoint.z() = 0.5 * (min_z + max_z);
    }
}

double ExplorationFrontend::wrapAngleDiff(const double lhs, const double rhs) {
    return std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs));
}

}  // namespace general_planner
