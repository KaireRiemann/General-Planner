#include "exploration/frontier_database.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace general_planner {
namespace exploration {

FrontierDatabase::FrontierDatabase(Config cfg) : cfg_(std::move(cfg)) {}

void FrontierDatabase::update(const std::vector<SurfaceFrontierCluster> &clusters,
                              const double stamp) {
    std::unordered_set<int> matched;
    for (const auto &cluster : clusters) {
        const int id = associateOrCreateId(cluster, stamp);
        matched.insert(id);
        auto &record = records_[id];
        record.stable_id = id;
        record.center = cluster.center;
        record.normal = cluster.normal;
        record.bbox_min = cluster.bbox_min;
        record.bbox_max = cluster.bbox_max;
        record.cells = cluster.cells;
        record.normals = cluster.normals;
        record.cell_states = cluster.cell_states;
        record.cell_count = cluster.raw_size;
        record.dominant_state = cluster.dominant_state;
        record.last_observed_time = stamp;
        if (record.first_observed_time < 0.0) {
            record.first_observed_time = stamp;
            record.generated_position = cluster.generated_position;
            record.generated_travel_distance = cluster.generated_travel_distance;
        }
        if ((record.state == FrontierState::DORMANT &&
             record.last_selected_time > 0.0 &&
             stamp - record.last_selected_time > cfg_.dormant_time) ||
            expiredBlacklist(record, cfg_, stamp)) {
            record.state = FrontierState::ACTIVE;
            record.failed_count = 0;
        }
    }

    for (auto &kv : records_) {
        FrontierRecord &record = kv.second;
        if (matched.find(kv.first) != matched.end()) {
            continue;
        }
        if (!frontierStateSelectable(record.state)) {
            continue;
        }
        if (record.last_observed_time > 0.0 &&
            stamp - record.last_observed_time > cfg_.missing_frontier_timeout) {
            record.state = FrontierState::COVERED;
        }
    }
}

std::vector<FrontierRecord> FrontierDatabase::getActiveFrontiers() const {
    std::vector<FrontierRecord> out;
    for (const auto &kv : records_) {
        const FrontierRecord &record = kv.second;
        if (frontierStateSelectable(record.state)) {
            out.push_back(record);
        }
    }
    return out;
}

std::vector<FrontierRecord> FrontierDatabase::getReachableFrontiers() const {
    std::vector<FrontierRecord> out;
    for (const auto &kv : records_) {
        const FrontierRecord &record = kv.second;
        if (frontierStateSelectable(record.state) && record.has_reachable_viewpoint) {
            out.push_back(record);
        }
    }
    return out;
}

bool FrontierDatabase::getFrontier(const int stable_id, FrontierRecord &out) const {
    const auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return false;
    }
    out = it->second;
    return true;
}

bool FrontierDatabase::isFrontierActive(const int stable_id) const {
    const auto it = records_.find(stable_id);
    return it != records_.end() && frontierStateSelectable(it->second.state);
}

void FrontierDatabase::setViewpoints(const int stable_id,
                                     const std::vector<ExplorationViewpoint> &viewpoints,
                                     const ExplorationViewpoint &best_viewpoint,
                                     const double stamp) {
    auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return;
    }
    FrontierRecord &record = it->second;
    record.viewpoints = viewpoints;
    record.best_viewpoint = best_viewpoint;
    record.has_reachable_viewpoint = best_viewpoint.reachable || best_viewpoint.global_safe;
    record.last_gain = best_viewpoint.gain_raw;
    record.best_gain = std::max(record.best_gain, best_viewpoint.gain_raw);
    if (!record.has_reachable_viewpoint) {
        record.failed_count++;
        if (record.failed_count >= cfg_.max_failed_count) {
            record.state = FrontierState::UNREACHABLE;
        }
    } else if (record.state == FrontierState::UNREACHABLE) {
        record.state = FrontierState::ACTIVE;
        record.failed_count = 0;
    }
    (void)stamp;
}

void FrontierDatabase::markSelected(const int stable_id, const double stamp) {
    auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return;
    }
    FrontierRecord &record = it->second;
    record.state = FrontierState::SELECTED;
    record.selected_count++;
    record.last_selected_time = stamp;
    if (record.selected_count >= cfg_.max_selected_count_without_gain &&
        record.last_gain <= cfg_.covered_gain_threshold) {
        record.state = FrontierState::DORMANT;
    }
}

void FrontierDatabase::markCovered(const int stable_id, const double stamp) {
    auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return;
    }
    it->second.state = FrontierState::COVERED;
    it->second.last_observed_time = stamp;
}

void FrontierDatabase::markDormant(const int stable_id, const double stamp) {
    auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return;
    }
    it->second.state = FrontierState::DORMANT;
    it->second.last_selected_time = stamp;
}

void FrontierDatabase::markUnreachable(const int stable_id, const double stamp) {
    auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return;
    }
    it->second.state = FrontierState::UNREACHABLE;
    it->second.failed_count++;
    it->second.last_selected_time = stamp;
}

void FrontierDatabase::markBlacklisted(const int stable_id, const double stamp) {
    auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return;
    }
    it->second.state = FrontierState::BLACKLISTED;
    it->second.last_selected_time = stamp;
}

void FrontierDatabase::markViewpointVisited(const int frontier_id,
                                            const int viewpoint_id,
                                            const double stamp) {
    auto it = records_.find(frontier_id);
    if (it == records_.end()) {
        return;
    }
    for (auto &viewpoint : it->second.viewpoints) {
        if (viewpoint.viewpoint_id == viewpoint_id) {
            viewpoint.visited = true;
            viewpoint.last_checked_time = stamp;
        }
    }
}

void FrontierDatabase::onLowGain(const int stable_id,
                                 const double actual_gain,
                                 const double stamp) {
    auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return;
    }
    it->second.last_gain = actual_gain;
    if (actual_gain < cfg_.covered_gain_threshold) {
        markCovered(stable_id, stamp);
    }
}

void FrontierDatabase::onFailed(const int stable_id, const double stamp) {
    auto it = records_.find(stable_id);
    if (it == records_.end()) {
        return;
    }
    it->second.failed_count++;
    it->second.last_selected_time = stamp;
    if (it->second.failed_count >= cfg_.max_failed_count) {
        it->second.state = FrontierState::UNREACHABLE;
    }
}

void FrontierDatabase::reset() {
    records_.clear();
    next_id_ = 0;
}

int FrontierDatabase::activeCount() const {
    int count = 0;
    for (const auto &kv : records_) {
        if (frontierStateSelectable(kv.second.state)) {
            ++count;
        }
    }
    return count;
}

int FrontierDatabase::associateOrCreateId(const SurfaceFrontierCluster &cluster,
                                          const double stamp) {
    int best_id = -1;
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto &kv : records_) {
        const FrontierRecord &record = kv.second;
        if (record.state == FrontierState::BLACKLISTED &&
            !expiredBlacklist(record, cfg_, stamp)) {
            continue;
        }
        const double dist = (record.center - cluster.center).norm();
        const double overlap = bboxOverlapRatio(record, cluster);
        if (dist > cfg_.association_distance && overlap < cfg_.bbox_overlap_min_ratio) {
            continue;
        }
        const double score = dist - overlap;
        if (score < best_score) {
            best_score = score;
            best_id = kv.first;
        }
    }
    if (best_id >= 0) {
        return best_id;
    }
    return next_id_++;
}

double FrontierDatabase::bboxOverlapRatio(const FrontierRecord &record,
                                          const SurfaceFrontierCluster &cluster) {
    const super_utils::Vec3f overlap_min = record.bbox_min.cwiseMax(cluster.bbox_min);
    const super_utils::Vec3f overlap_max = record.bbox_max.cwiseMin(cluster.bbox_max);
    const super_utils::Vec3f overlap = (overlap_max - overlap_min).cwiseMax(super_utils::Vec3f::Zero());
    const double overlap_volume = overlap.x() * overlap.y() * overlap.z();
    const super_utils::Vec3f a = (record.bbox_max - record.bbox_min).cwiseMax(super_utils::Vec3f::Constant(1.0e-3));
    const super_utils::Vec3f b = (cluster.bbox_max - cluster.bbox_min).cwiseMax(super_utils::Vec3f::Constant(1.0e-3));
    const double volume_a = a.x() * a.y() * a.z();
    const double volume_b = b.x() * b.y() * b.z();
    return overlap_volume / std::max(1.0e-6, std::min(volume_a, volume_b));
}

bool FrontierDatabase::expiredBlacklist(const FrontierRecord &record,
                                        const Config &cfg,
                                        const double stamp) {
    return record.state == FrontierState::BLACKLISTED &&
           record.last_selected_time > 0.0 &&
           stamp - record.last_selected_time > cfg.blacklist_time;
}

}  // namespace exploration
}  // namespace general_planner
