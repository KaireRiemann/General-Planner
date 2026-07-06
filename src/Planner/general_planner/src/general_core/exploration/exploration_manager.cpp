#include <general_core/exploration/exploration_manager.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace general_utils;

namespace general_planner {
namespace {
constexpr double kEps = 1.0e-6;

Vec3f vectorToVec3f(const std::vector<double> &values, const Vec3f &fallback) {
    if (values.size() < 3U) {
        return fallback;
    }
    return Vec3f(values[0], values[1], values[2]);
}

bool isUnknownLike(const rog_map::GridType type) {
    return type == rog_map::GridType::UNKNOWN ||
           type == rog_map::GridType::UNDEFINED ||
           type == rog_map::GridType::FRONTIER;
}

std::string stateName(const ExplorationManager::Diagnostics &diag) {
    std::ostringstream oss;
    oss << "active=" << diag.active_frontiers
        << ",covered=" << diag.covered_frontiers
        << ",failed=" << diag.failed_frontiers
        << ",coverage_cells=" << diag.coverage_cells
        << ",unknown=" << diag.unknown_voxels
        << ",coverage_ratio=" << diag.coverage_ratio
        << ",stable_finish=" << diag.stable_finish_cycles
        << ",candidates=" << diag.last_input_candidates
        << "->" << diag.last_output_candidates
        << ",reject_box=" << diag.last_reject_outside_box
        << ",reject_blocked=" << diag.last_reject_blocked
        << ",reject_covered=" << diag.last_reject_covered
        << ",reject_low_unknown=" << diag.last_reject_low_unknown
        << ",reject_duplicate=" << diag.last_reject_duplicate;
    return oss.str();
}
} // namespace

ExplorationManager::Config ExplorationManager::makeConfig(
        const general_planner::Config &planner_cfg) {
    Config cfg;
    cfg.enable = planner_cfg.exploration_manager_enable;
    cfg.print_log = planner_cfg.exploration_print_log;
    cfg.global_box_enable = planner_cfg.exploration_global_box_enable;
    cfg.global_box_min = vectorToVec3f(planner_cfg.exploration_global_box_min,
                                       Vec3f(-50.0, -50.0, 0.0));
    cfg.global_box_max = vectorToVec3f(planner_cfg.exploration_global_box_max,
                                       Vec3f(50.0, 50.0, 3.0));
    cfg.frontier_merge_radius =
            std::max(0.25, planner_cfg.exploration_manager_frontier_merge_radius);
    cfg.frontier_record_ttl =
            std::max(1.0, planner_cfg.exploration_manager_frontier_record_ttl);
    cfg.frontier_failure_ttl =
            std::max(0.0, planner_cfg.exploration_manager_frontier_failure_ttl);
    cfg.frontier_covered_radius =
            std::max(0.1, planner_cfg.exploration_manager_frontier_covered_radius);
    cfg.max_frontier_records =
            std::max(1, planner_cfg.exploration_manager_max_frontier_records);
    cfg.coverage_resolution =
            std::max(0.2, planner_cfg.exploration_manager_coverage_resolution);
    cfg.coverage_radius =
            std::max(0.5, planner_cfg.exploration_manager_coverage_radius);
    cfg.coverage_z_half =
            std::max(0.1, planner_cfg.exploration_manager_coverage_z_half);
    cfg.max_coverage_cells =
            std::max(1, planner_cfg.exploration_manager_max_coverage_cells);
    cfg.unknown_gain_radius =
            std::max(0.5, planner_cfg.exploration_manager_unknown_gain_radius);
    cfg.unknown_gain_resolution =
            std::max(0.2, planner_cfg.exploration_manager_unknown_gain_resolution);
    cfg.min_unknown_gain =
            std::max(0, planner_cfg.exploration_manager_min_unknown_gain);
    cfg.unknown_gain_score_weight =
            planner_cfg.exploration_manager_unknown_gain_score_weight;
    cfg.max_viewpoints_per_frontier =
            std::max(1, planner_cfg.exploration_manager_max_viewpoints_per_frontier);
    cfg.revisit_score_weight =
            planner_cfg.exploration_manager_revisit_score_weight;
    cfg.covered_score_weight =
            planner_cfg.exploration_manager_covered_score_weight;
    cfg.stale_frontier_score_weight =
            planner_cfg.exploration_manager_stale_frontier_score_weight;
    cfg.committed_frontier_score_weight =
            planner_cfg.exploration_manager_committed_frontier_score_weight;
    cfg.recent_commit_score_weight =
            planner_cfg.exploration_manager_recent_commit_score_weight;
    cfg.recent_commit_time_window =
            std::max(0.0, planner_cfg.exploration_manager_recent_commit_time_window);
    cfg.max_commits_before_cooldown =
            std::max(0, planner_cfg.exploration_manager_max_commits_before_cooldown);
    cfg.overcommit_cooldown =
            std::max(0.0, planner_cfg.exploration_manager_overcommit_cooldown);
    cfg.completion_scan_resolution =
            std::max(0.2, planner_cfg.exploration_manager_completion_scan_resolution);
    cfg.completion_max_unknown_voxels =
            std::max(0, planner_cfg.exploration_manager_completion_max_unknown_voxels);
    cfg.completion_min_coverage_ratio =
            std::clamp(planner_cfg.exploration_manager_completion_min_coverage_ratio,
                       0.0,
                       1.0);
    cfg.completion_stable_cycles =
            std::max(1, planner_cfg.exploration_manager_completion_stable_cycles);
    cfg.completion_max_active_frontiers =
            std::max(0, planner_cfg.exploration_manager_completion_max_active_frontiers);

    for (int axis = 0; axis < 3; ++axis) {
        if (cfg.global_box_min(axis) > cfg.global_box_max(axis)) {
            std::swap(cfg.global_box_min(axis), cfg.global_box_max(axis));
        }
    }
    return cfg;
}

ExplorationManager::ExplorationManager(const Config &cfg, MapManager::Ptr map_manager)
        : cfg_(cfg),
          map_manager_(std::move(map_manager)) {
}

void ExplorationManager::reset() {
    frontier_records_.clear();
    coverage_cells_.clear();
    finished_ = false;
    finish_reason_.clear();
    stable_finish_cycles_ = 0;
    last_unknown_voxels_ = 0;
    last_coverage_ratio_ = 0.0;
    last_input_candidates_ = 0;
    last_output_candidates_ = 0;
    last_reject_outside_box_ = 0;
    last_reject_blocked_ = 0;
    last_reject_covered_ = 0;
    last_reject_low_unknown_ = 0;
    last_reject_duplicate_ = 0;
}

bool ExplorationManager::enabled() const {
    return cfg_.enable;
}

bool ExplorationManager::globalBoxEnabled() const {
    return cfg_.enable && cfg_.global_box_enable;
}

bool ExplorationManager::insideTaskBox(const Vec3f &position) const {
    if (!cfg_.enable || !cfg_.global_box_enable) {
        return true;
    }
    if (!position.allFinite()) {
        return false;
    }
    return (position.array() >= cfg_.global_box_min.array()).all() &&
           (position.array() <= cfg_.global_box_max.array()).all();
}

bool ExplorationManager::clipSearchBox(Vec3f &box_min, Vec3f &box_max) const {
    if (!cfg_.enable || !cfg_.global_box_enable) {
        return true;
    }
    box_min = box_min.cwiseMax(cfg_.global_box_min);
    box_max = box_max.cwiseMin(cfg_.global_box_max);
    return (box_max - box_min).minCoeff() > 0.0;
}

void ExplorationManager::beginCycle(const Vec3f &robot_pos, const double stamp) {
    if (!cfg_.enable || map_manager_ == nullptr || !map_manager_->ready()) {
        return;
    }
    prune(stamp);
    observeCoverageAround(robot_pos, stamp);
    updateCoveredRecords(robot_pos, stamp);
}

void ExplorationManager::filterAndScoreCandidates(const Vec3f &robot_pos,
                                                  const double stamp,
                                                  vec_E<ExplorationGoal> &candidates) {
    if (!cfg_.enable || map_manager_ == nullptr || !map_manager_->ready()) {
        return;
    }
    beginCycle(robot_pos, stamp);
    last_input_candidates_ = static_cast<int>(candidates.size());
    last_output_candidates_ = 0;
    last_reject_outside_box_ = 0;
    last_reject_blocked_ = 0;
    last_reject_covered_ = 0;
    last_reject_low_unknown_ = 0;
    last_reject_duplicate_ = 0;

    vec_E<ExplorationGoal> scored;
    scored.reserve(candidates.size());

    for (ExplorationGoal candidate : candidates) {
        if (!candidate.valid || !candidate.position.allFinite()) {
            ++last_reject_blocked_;
            continue;
        }
        const Vec3f reference = frontierReference(candidate);
        if (!insideTaskBox(candidate.position) || !insideTaskBox(reference)) {
            ++last_reject_outside_box_;
            continue;
        }

        const FrontierRecord *existing = findRecord(candidate);
        if (existing != nullptr) {
            if (existing->state == FrontierState::FAILED &&
                existing->blocked_until > stamp) {
                ++last_reject_blocked_;
                continue;
            }
            if (existing->state == FrontierState::COVERED &&
                !recordExpired(*existing, stamp)) {
                ++last_reject_covered_;
                continue;
            }
            if (cfg_.max_commits_before_cooldown > 0 &&
                existing->commit_count >= cfg_.max_commits_before_cooldown &&
                existing->blocked_until > stamp) {
                ++last_reject_blocked_;
                continue;
            }
        }

        const bool bootstrap_candidate =
                candidate.reason.find("bootstrap") != std::string::npos;
        const bool expansion_candidate =
                candidate.identity.intent_mode == "exploration_expansion" ||
                candidate.reason.find("expansion") != std::string::npos;
        if (!bootstrap_candidate &&
            !expansion_candidate &&
            isFrontierCovered(reference, robot_pos)) {
            FrontierRecord &covered = upsertFrontier(candidate, stamp, reference);
            covered.state = FrontierState::COVERED;
            covered.last_state_stamp = stamp;
            covered.blocked_until = stamp + cfg_.frontier_record_ttl;
            ++last_reject_covered_;
            continue;
        }

        FrontierRecord &record = upsertFrontier(candidate, stamp, reference);
        const int unknown_gain = countUnknownNear(reference,
                                                 cfg_.unknown_gain_radius,
                                                 cfg_.unknown_gain_resolution);
        if (!bootstrap_candidate &&
            !expansion_candidate &&
            unknown_gain < cfg_.min_unknown_gain) {
            if (unknown_gain <= 0) {
                record.state = FrontierState::COVERED;
                record.last_state_stamp = stamp;
                record.blocked_until = stamp + cfg_.frontier_record_ttl;
            }
            ++last_reject_low_unknown_;
            continue;
        }
        const int revisit_count = countCoverageNear(reference, cfg_.frontier_covered_radius);
        const double repeat_penalty =
                cfg_.stale_frontier_score_weight *
                static_cast<double>(std::max(0, record.seen_count - 1));
        double commit_penalty =
                cfg_.committed_frontier_score_weight *
                static_cast<double>(std::max(0, record.commit_count));
        if (record.last_commit_stamp > 0.0 && cfg_.recent_commit_time_window > 1.0e-6) {
            const double elapsed = std::max(0.0, stamp - record.last_commit_stamp);
            if (elapsed < cfg_.recent_commit_time_window) {
                const double ratio = 1.0 - elapsed / cfg_.recent_commit_time_window;
                commit_penalty += cfg_.recent_commit_score_weight * ratio;
            }
        }
        const double manager_delta =
                cfg_.unknown_gain_score_weight * static_cast<double>(unknown_gain) +
                cfg_.revisit_score_weight * static_cast<double>(revisit_count) +
                repeat_penalty +
                commit_penalty;

        candidate.score += manager_delta;
        candidate.history_score_delta += manager_delta;
        candidate.information_gain += 0.1 * static_cast<double>(unknown_gain);
        candidate.reason += " mission_unknown=" + std::to_string(unknown_gain) +
                            " mission_revisit=" + std::to_string(revisit_count) +
                            " mission_commits=" + std::to_string(record.commit_count);
        record.goal = candidate;
        scored.push_back(candidate);
    }

    std::unordered_map<std::string, vec_E<ExplorationGoal>> grouped_candidates;
    grouped_candidates.reserve(scored.size());
    for (const ExplorationGoal &candidate : scored) {
        const Vec3f reference = frontierReference(candidate);
        std::ostringstream oss;
        if (candidate.frontier_id >= 0) {
            oss << "frontier:" << candidate.frontier_id;
        } else {
            const Key key = keyForPosition(reference, cfg_.frontier_merge_radius);
            oss << "spatial:" << key.x << ":" << key.y << ":" << key.z;
        }
        grouped_candidates[oss.str()].push_back(candidate);
    }

    vec_E<ExplorationGoal> filtered;
    filtered.reserve(scored.size());
    const int max_per_frontier = std::max(1, cfg_.max_viewpoints_per_frontier);
    for (auto &entry : grouped_candidates) {
        vec_E<ExplorationGoal> &group = entry.second;
        std::sort(group.begin(),
                  group.end(),
                  [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                      if (lhs.score != rhs.score) {
                          return lhs.score < rhs.score;
                      }
                      if (lhs.information_gain != rhs.information_gain) {
                          return lhs.information_gain > rhs.information_gain;
                      }
                      return lhs.travel_cost < rhs.travel_cost;
                  });
        const int keep_count =
                std::min(max_per_frontier, static_cast<int>(group.size()));
        for (int i = 0; i < keep_count; ++i) {
            group[static_cast<size_t>(i)].reason +=
                    " mission_group_rank=" + std::to_string(i) +
                    " mission_group_size=" + std::to_string(group.size());
            filtered.push_back(group[static_cast<size_t>(i)]);
        }
        last_reject_duplicate_ += static_cast<int>(group.size()) - keep_count;
    }

    std::sort(filtered.begin(),
              filtered.end(),
              [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                  if (lhs.score != rhs.score) {
                      return lhs.score < rhs.score;
                  }
                  return lhs.information_gain > rhs.information_gain;
              });
    candidates.swap(filtered);
    last_output_candidates_ = static_cast<int>(candidates.size());
    updateFinishState(robot_pos, stamp, !candidates.empty(), true);
}

void ExplorationManager::updateNoFrontier(const Vec3f &robot_pos,
                                          const double stamp,
                                          const bool map_observation_ready) {
    if (!cfg_.enable) {
        return;
    }
    last_input_candidates_ = 0;
    last_output_candidates_ = 0;
    last_reject_outside_box_ = 0;
    last_reject_blocked_ = 0;
    last_reject_covered_ = 0;
    last_reject_low_unknown_ = 0;
    last_reject_duplicate_ = 0;
    beginCycle(robot_pos, stamp);
    updateFinishState(robot_pos, stamp, false, map_observation_ready);
}

bool ExplorationManager::recoverGoal(const Vec3f &robot_pos,
                                     const double current_yaw,
                                     const double stamp,
                                     ExplorationGoal &goal) {
    goal = ExplorationGoal{};
    if (!cfg_.enable || !robot_pos.allFinite()) {
        return false;
    }
    beginCycle(robot_pos, stamp);

    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;
    for (auto &entry : frontier_records_) {
        FrontierRecord &record = entry.second;
        if (recordExpired(record, stamp) ||
            record.state == FrontierState::FAILED ||
            record.state == FrontierState::COVERED ||
            record.blocked_until > stamp ||
            !record.goal.valid ||
            !insideTaskBox(record.goal.position) ||
            !insideTaskBox(record.reference)) {
            continue;
        }
        if (isFrontierCovered(record.reference, robot_pos)) {
            record.state = FrontierState::COVERED;
            record.last_state_stamp = stamp;
            record.blocked_until = stamp + cfg_.frontier_record_ttl;
            continue;
        }

        const double distance = (record.goal.position - robot_pos).norm();
        const int unknown_gain = countUnknownNear(record.reference,
                                                 cfg_.unknown_gain_radius,
                                                 cfg_.unknown_gain_resolution);
        const double score = record.goal.score + 0.15 * distance -
                             0.05 * static_cast<double>(unknown_gain);
        if (score < best_score) {
            best_score = score;
            goal = record.goal;
            found = true;
        }
    }

    if (!found) {
        return false;
    }
    if (!std::isfinite(goal.yaw)) {
        const Vec3f diff = goal.position - robot_pos;
        goal.yaw = std::atan2(diff.y(), diff.x());
    } else if (std::abs(goal.yaw) < kEps && std::isfinite(current_yaw)) {
        goal.yaw = current_yaw;
    }
    goal.reason = "exploration_manager_recovery " + goal.reason;
    return true;
}

void ExplorationManager::recordCommitted(const ExplorationGoal &goal,
                                         const Vec3f &robot_pos,
                                         const double stamp) {
    if (!cfg_.enable || !goal.valid) {
        return;
    }
    beginCycle(robot_pos, stamp);
    const Vec3f reference = frontierReference(goal);
    FrontierRecord &record = upsertFrontier(goal, stamp, reference);
    record.state = FrontierState::COMMITTED;
    record.last_state_stamp = stamp;
    record.last_commit_stamp = stamp;
    ++record.commit_count;
    if (cfg_.max_commits_before_cooldown > 0 &&
        cfg_.overcommit_cooldown > 0.0 &&
        record.commit_count >= cfg_.max_commits_before_cooldown) {
        record.blocked_until = std::max(record.blocked_until,
                                        stamp + cfg_.overcommit_cooldown);
    }
}

void ExplorationManager::recordFailure(const ExplorationGoal &goal, const double stamp) {
    if (!cfg_.enable || !goal.valid) {
        return;
    }
    const Vec3f reference = frontierReference(goal);
    FrontierRecord &record = upsertFrontier(goal, stamp, reference);
    record.state = FrontierState::FAILED;
    record.last_state_stamp = stamp;
    record.blocked_until = stamp + cfg_.frontier_failure_ttl;
    ++record.failure_count;
}

bool ExplorationManager::isFinished() const {
    return cfg_.enable && finished_;
}

std::string ExplorationManager::finishReason() const {
    return finish_reason_;
}

ExplorationManager::Diagnostics ExplorationManager::diagnostics(const double stamp) const {
    Diagnostics diag;
    diag.active_frontiers = activeFrontierCount(stamp);
    diag.covered_frontiers = stateCount(FrontierState::COVERED, stamp);
    diag.failed_frontiers = stateCount(FrontierState::FAILED, stamp);
    diag.coverage_cells = static_cast<int>(coverage_cells_.size());
    diag.unknown_voxels = last_unknown_voxels_;
    diag.coverage_ratio = last_coverage_ratio_;
    diag.stable_finish_cycles = stable_finish_cycles_;
    diag.last_input_candidates = last_input_candidates_;
    diag.last_output_candidates = last_output_candidates_;
    diag.last_reject_outside_box = last_reject_outside_box_;
    diag.last_reject_blocked = last_reject_blocked_;
    diag.last_reject_covered = last_reject_covered_;
    diag.last_reject_low_unknown = last_reject_low_unknown_;
    diag.last_reject_duplicate = last_reject_duplicate_;
    return diag;
}

std::string ExplorationManager::diagnosticSummary(const double stamp) const {
    return stateName(diagnostics(stamp));
}

std::size_t ExplorationManager::KeyHasher::operator()(const Key &key) const {
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

ExplorationManager::Key ExplorationManager::keyForPosition(const Vec3f &position,
                                                           const double resolution) const {
    const double res = std::max(0.05, resolution);
    return Key{
            static_cast<int>(std::floor(position.x() / res)),
            static_cast<int>(std::floor(position.y() / res)),
            static_cast<int>(std::floor(position.z() / res))};
}

std::string ExplorationManager::keyForFrontier(const ExplorationGoal &goal) const {
    const Vec3f reference = frontierReference(goal);
    const Key key = keyForPosition(reference, cfg_.frontier_merge_radius);
    std::ostringstream oss;
    oss << "f:" << key.x << ":" << key.y << ":" << key.z;
    if (goal.frontier_id >= 0) {
        oss << ":" << goal.frontier_id;
    }
    return oss.str();
}

Vec3f ExplorationManager::frontierReference(const ExplorationGoal &goal) const {
    if (goal.identity.intent_mode == "exploration_expansion" ||
        goal.reason.find("expansion") != std::string::npos) {
        return goal.position;
    }
    if (goal.frontier_center_valid && goal.frontier_center.allFinite()) {
        return goal.frontier_center;
    }
    return goal.position;
}

ExplorationManager::FrontierRecord &ExplorationManager::upsertFrontier(
        const ExplorationGoal &goal,
        const double stamp,
        const Vec3f &reference) {
    std::string key = keyForFrontier(goal);
    auto it = frontier_records_.find(key);
    if (it == frontier_records_.end()) {
        const double merge_radius_sq = cfg_.frontier_merge_radius * cfg_.frontier_merge_radius;
        for (auto record_it = frontier_records_.begin();
             record_it != frontier_records_.end();
             ++record_it) {
            const FrontierRecord &record = record_it->second;
            if (record.state == FrontierState::FAILED ||
                record.state == FrontierState::COVERED) {
                continue;
            }
            if ((record.reference - reference).squaredNorm() <= merge_radius_sq) {
                key = record_it->first;
                it = record_it;
                break;
            }
        }
    }

    if (it == frontier_records_.end()) {
        FrontierRecord record;
        record.goal = goal;
        record.reference = reference;
        record.first_seen_stamp = stamp;
        record.last_seen_stamp = stamp;
        record.last_state_stamp = stamp;
        record.seen_count = 1;
        it = frontier_records_.emplace(key, record).first;
    } else {
        FrontierRecord &record = it->second;
        record.goal = goal;
        record.reference = 0.7 * record.reference + 0.3 * reference;
        record.last_seen_stamp = stamp;
        if (record.state == FrontierState::COVERED &&
            record.blocked_until <= stamp) {
            record.state = FrontierState::ACTIVE;
            record.last_state_stamp = stamp;
        } else if (record.state != FrontierState::COMMITTED &&
                   record.state != FrontierState::FAILED) {
            record.state = FrontierState::ACTIVE;
        }
        ++record.seen_count;
    }
    prune(stamp);
    return it->second;
}

const ExplorationManager::FrontierRecord *ExplorationManager::findRecord(
        const ExplorationGoal &goal) const {
    const std::string key = keyForFrontier(goal);
    const auto it = frontier_records_.find(key);
    if (it != frontier_records_.end()) {
        return &it->second;
    }

    const Vec3f reference = frontierReference(goal);
    const double merge_radius_sq = cfg_.frontier_merge_radius * cfg_.frontier_merge_radius;
    for (const auto &entry : frontier_records_) {
        if ((entry.second.reference - reference).squaredNorm() <= merge_radius_sq) {
            return &entry.second;
        }
    }
    return nullptr;
}

ExplorationManager::FrontierRecord *ExplorationManager::findRecordMutable(
        const ExplorationGoal &goal) {
    return const_cast<FrontierRecord *>(
            static_cast<const ExplorationManager *>(this)->findRecord(goal));
}

void ExplorationManager::prune(const double stamp) {
    for (auto it = frontier_records_.begin(); it != frontier_records_.end();) {
        if (recordExpired(it->second, stamp)) {
            it = frontier_records_.erase(it);
        } else {
            ++it;
        }
    }

    while (static_cast<int>(frontier_records_.size()) > cfg_.max_frontier_records) {
        auto oldest = frontier_records_.begin();
        for (auto it = frontier_records_.begin(); it != frontier_records_.end(); ++it) {
            if (it->second.last_seen_stamp < oldest->second.last_seen_stamp) {
                oldest = it;
            }
        }
        frontier_records_.erase(oldest);
    }

    while (static_cast<int>(coverage_cells_.size()) > cfg_.max_coverage_cells) {
        auto oldest = coverage_cells_.begin();
        for (auto it = coverage_cells_.begin(); it != coverage_cells_.end(); ++it) {
            if (it->second.last_seen_stamp < oldest->second.last_seen_stamp) {
                oldest = it;
            }
        }
        coverage_cells_.erase(oldest);
    }
}

void ExplorationManager::observeCoverageAround(const Vec3f &robot_pos,
                                               const double stamp) {
    if (!robot_pos.allFinite() || !insideTaskBox(robot_pos)) {
        return;
    }

    const double res = std::max(0.2, cfg_.coverage_resolution);
    const int xy_steps = std::max(1, static_cast<int>(std::ceil(cfg_.coverage_radius / res)));
    const int z_steps = std::max(1, static_cast<int>(std::ceil(cfg_.coverage_z_half / res)));
    const double radius_sq = cfg_.coverage_radius * cfg_.coverage_radius;

    for (int dx = -xy_steps; dx <= xy_steps; ++dx) {
        for (int dy = -xy_steps; dy <= xy_steps; ++dy) {
            const double xy_sq = res * res * static_cast<double>(dx * dx + dy * dy);
            if (xy_sq > radius_sq) {
                continue;
            }
            for (int dz = -z_steps; dz <= z_steps; ++dz) {
                const Vec3f pos = robot_pos + res * Vec3f(dx, dy, dz);
                if (!insideTaskBox(pos)) {
                    continue;
                }
                if (map_manager_ != nullptr &&
                    map_manager_->ready() &&
                    !map_manager_->insideLocalMap(pos)) {
                    continue;
                }
                CoverageRecord &record =
                        coverage_cells_[keyForPosition(pos, cfg_.coverage_resolution)];
                if (record.visits == 0) {
                    record.first_seen_stamp = stamp;
                }
                record.last_seen_stamp = stamp;
                ++record.visits;
            }
        }
    }
}

int ExplorationManager::countCoverageNear(const Vec3f &position,
                                          const double radius) const {
    if (!position.allFinite()) {
        return 0;
    }
    const double res = std::max(0.2, cfg_.coverage_resolution);
    const int steps = std::max(0, static_cast<int>(std::ceil(radius / res)));
    const Key center = keyForPosition(position, res);
    int count = 0;
    for (int dx = -steps; dx <= steps; ++dx) {
        for (int dy = -steps; dy <= steps; ++dy) {
            for (int dz = -steps; dz <= steps; ++dz) {
                const Key key{center.x + dx, center.y + dy, center.z + dz};
                if (coverage_cells_.find(key) != coverage_cells_.end()) {
                    ++count;
                }
            }
        }
    }
    return count;
}

int ExplorationManager::countUnknownNear(const Vec3f &position,
                                         const double radius,
                                         const double resolution) const {
    if (map_manager_ == nullptr ||
        !map_manager_->ready() ||
        !position.allFinite() ||
        !insideTaskBox(position) ||
        !map_manager_->insideLocalMap(position)) {
        return 0;
    }

    const double res = std::max(0.2, resolution);
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
                const Vec3f pos = position + offset;
                if (!insideTaskBox(pos) || !map_manager_->insideLocalMap(pos)) {
                    continue;
                }
                if (isUnknownLike(map_manager_->getGridType(pos))) {
                    ++count;
                }
            }
        }
    }
    return count;
}

int ExplorationManager::countUnknownInTaskBox() const {
    if (!cfg_.global_box_enable ||
        map_manager_ == nullptr ||
        !map_manager_->ready()) {
        return 0;
    }

    const double res = std::max(0.2, cfg_.completion_scan_resolution);
    const Vec3f extent = cfg_.global_box_max - cfg_.global_box_min;
    const int nx = std::max(1, static_cast<int>(std::ceil(extent.x() / res)));
    const int ny = std::max(1, static_cast<int>(std::ceil(extent.y() / res)));
    const int nz = std::max(1, static_cast<int>(std::ceil(extent.z() / res)));
    const int early_stop = cfg_.completion_max_unknown_voxels + 1;
    int count = 0;

    for (int ix = 0; ix <= nx; ++ix) {
        for (int iy = 0; iy <= ny; ++iy) {
            for (int iz = 0; iz <= nz; ++iz) {
                const Vec3f pos = cfg_.global_box_min + res * Vec3f(ix, iy, iz);
                if (!insideTaskBox(pos) || !map_manager_->insideLocalMap(pos)) {
                    continue;
                }
                if (isUnknownLike(map_manager_->getGridType(pos))) {
                    ++count;
                    if (count >= early_stop) {
                        return count;
                    }
                }
            }
        }
    }
    return count;
}

double ExplorationManager::estimateCoverageRatio() const {
    if (!cfg_.global_box_enable) {
        return 0.0;
    }
    const double res = std::max(0.2, cfg_.coverage_resolution);
    const Vec3f extent = cfg_.global_box_max - cfg_.global_box_min;
    const int nx = std::max(1, static_cast<int>(std::ceil(extent.x() / res)));
    const int ny = std::max(1, static_cast<int>(std::ceil(extent.y() / res)));
    const int nz = std::max(1, static_cast<int>(std::ceil(extent.z() / res)));
    const double total = static_cast<double>((nx + 1) * (ny + 1) * (nz + 1));
    if (total <= 0.0) {
        return 0.0;
    }

    int covered_inside = 0;
    for (const auto &entry : coverage_cells_) {
        const Vec3f pos = res * Vec3f(entry.first.x, entry.first.y, entry.first.z);
        if (insideTaskBox(pos)) {
            ++covered_inside;
        }
    }
    return std::clamp(static_cast<double>(covered_inside) / total, 0.0, 1.0);
}

bool ExplorationManager::frontierStillUseful(const Vec3f &reference) const {
    if (map_manager_ == nullptr ||
        !map_manager_->ready() ||
        !reference.allFinite() ||
        !insideTaskBox(reference) ||
        !map_manager_->insideLocalMap(reference)) {
        return true;
    }
    return countUnknownNear(reference,
                            cfg_.frontier_covered_radius,
                            std::max(map_manager_->getResolution(),
                                     cfg_.unknown_gain_resolution)) > 0;
}

bool ExplorationManager::isFrontierCovered(const Vec3f &reference,
                                           const Vec3f &robot_pos) const {
    (void) robot_pos;
    if (!reference.allFinite() || !insideTaskBox(reference)) {
        return true;
    }
    if (!frontierStillUseful(reference)) {
        return true;
    }
    return false;
}

void ExplorationManager::updateCoveredRecords(const Vec3f &robot_pos,
                                              const double stamp) {
    for (auto &entry : frontier_records_) {
        FrontierRecord &record = entry.second;
        if (record.state == FrontierState::FAILED ||
            record.state == FrontierState::COVERED) {
            continue;
        }
        if (isFrontierCovered(record.reference, robot_pos)) {
            record.state = FrontierState::COVERED;
            record.last_state_stamp = stamp;
            record.blocked_until = stamp + cfg_.frontier_record_ttl;
        }
    }
}

void ExplorationManager::updateFinishState(const Vec3f &robot_pos,
                                           const double stamp,
                                           const bool local_frontier_available,
                                           const bool map_observation_ready) {
    (void) robot_pos;
    prune(stamp);
    last_unknown_voxels_ = cfg_.global_box_enable ? countUnknownInTaskBox() : 0;
    last_coverage_ratio_ = cfg_.global_box_enable ? estimateCoverageRatio() : 0.0;

    const int active_count = activeFrontierCount(stamp);
    bool finish_candidate = false;
    if (cfg_.global_box_enable) {
        finish_candidate =
                map_observation_ready &&
                !local_frontier_available &&
                active_count <= cfg_.completion_max_active_frontiers &&
                last_unknown_voxels_ <= cfg_.completion_max_unknown_voxels &&
                last_coverage_ratio_ >= cfg_.completion_min_coverage_ratio;
    } else {
        finish_candidate =
                map_observation_ready &&
                !local_frontier_available &&
                active_count <= cfg_.completion_max_active_frontiers;
    }

    if (finish_candidate) {
        ++stable_finish_cycles_;
    } else {
        stable_finish_cycles_ = 0;
        finished_ = false;
        finish_reason_.clear();
    }

    if (stable_finish_cycles_ >= cfg_.completion_stable_cycles) {
        finished_ = true;
        std::ostringstream oss;
        oss << "mission complete: active=" << active_count
            << ", unknown=" << last_unknown_voxels_
            << ", coverage=" << last_coverage_ratio_
            << ", stable_cycles=" << stable_finish_cycles_;
        finish_reason_ = oss.str();
    }
}

int ExplorationManager::activeFrontierCount(const double stamp) const {
    int count = 0;
    for (const auto &entry : frontier_records_) {
        const FrontierRecord &record = entry.second;
        if (!recordExpired(record, stamp) &&
            (record.state == FrontierState::ACTIVE ||
             record.state == FrontierState::COMMITTED)) {
            ++count;
        }
    }
    return count;
}

int ExplorationManager::stateCount(const FrontierState state,
                                   const double stamp) const {
    int count = 0;
    for (const auto &entry : frontier_records_) {
        const FrontierRecord &record = entry.second;
        if (!recordExpired(record, stamp) && record.state == state) {
            ++count;
        }
    }
    return count;
}

bool ExplorationManager::recordExpired(const FrontierRecord &record,
                                       const double stamp) const {
    if (record.state == FrontierState::FAILED &&
        record.blocked_until > stamp) {
        return false;
    }
    if (record.state == FrontierState::COVERED &&
        record.blocked_until > stamp) {
        return false;
    }
    return stamp - record.last_seen_stamp > cfg_.frontier_record_ttl;
}

} // namespace general_planner
