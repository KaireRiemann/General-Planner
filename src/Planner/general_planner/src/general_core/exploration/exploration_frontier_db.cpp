#include <general_core/exploration/exploration_frontier_db.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include <general_core/nhbp/nav_identity.hpp>

namespace general_planner {
namespace {

double finiteOr(const double value, const double fallback = 0.0)
{
    return std::isfinite(value) ? value : fallback;
}

double clampPositive(const double value)
{
    return std::max(0.0, finiteOr(value));
}

} // namespace

ExplorationFrontierDB::ExplorationFrontierDB(const Config &cfg)
        : cfg_(cfg)
{
}

void ExplorationFrontierDB::reset()
{
    records_.clear();
}

void ExplorationFrontierDB::observeCandidates(
        const ExplorationCandidateSet &candidate_set,
        const ObservationContext &context)
{
    if (!enabled() || !candidate_set.valid) {
        return;
    }
    prune(context.stamp);
    for (const ExplorationGoal &candidate : candidate_set.candidates) {
        observeCandidate(candidate, context);
    }
}

general_utils::vec_E<ExplorationFrontierDB::ObjectSnapshot>
ExplorationFrontierDB::activeObjects(const double stamp) const
{
    general_utils::vec_E<ObjectSnapshot> objects;
    if (!enabled()) {
        return objects;
    }
    objects.reserve(records_.size());
    for (const auto &entry : records_) {
        const Record &record = entry.second;
        if (!recordActive(record, stamp) || record.viewpoints.empty()) {
            continue;
        }
        objects.push_back(makeSnapshot(record, stamp));
    }
    std::sort(objects.begin(),
              objects.end(),
              [](const ObjectSnapshot &lhs, const ObjectSnapshot &rhs) {
                  if (lhs.best_score != rhs.best_score) {
                      return lhs.best_score < rhs.best_score;
                  }
                  return lhs.total_gain > rhs.total_gain;
              });
    return objects;
}

void ExplorationFrontierDB::markCommitted(const ExplorationGoal &goal,
                                          const double stamp)
{
    if (!enabled() || !goal.valid) {
        return;
    }
    Record *record = findRecordForGoal(goal);
    if (record == nullptr) {
        return;
    }
    ++record->selection_count;
    record->last_committed_stamp = stamp;
    record->last_state_stamp = stamp;
    record->state = State::COMMITTED;
}

void ExplorationFrontierDB::markCompleted(const ExplorationGoal &goal,
                                          const double stamp)
{
    if (!enabled() || !goal.valid) {
        return;
    }
    Record *record = findRecordForGoal(goal);
    if (record == nullptr) {
        return;
    }
    ++record->completed_count;
    record->last_completed_stamp = stamp;
    record->last_state_stamp = stamp;
    record->state = State::COVERED;
}

void ExplorationFrontierDB::markFailed(const ExplorationGoal &goal,
                                       const double stamp)
{
    if (!enabled() || !goal.valid) {
        return;
    }
    Record *record = findRecordForGoal(goal);
    if (record == nullptr) {
        return;
    }
    ++record->failure_count;
    record->last_state_stamp = stamp;
    record->state = State::BLOCKED;
    const double failure_ttl =
            std::max({cfg_.exploration_frontier_memory_failure_ttl,
                      cfg_.exploration_local_trap_cooldown,
                      cfg_.exploration_frontier_manager_dormant_time,
                      1.0});
    record->block_until = stamp + failure_ttl;
}

void ExplorationFrontierDB::markCoveredNear(
        const general_utils::Vec3f &position,
        const double stamp,
        const double radius)
{
    if (!enabled() || !position.allFinite() || radius <= 0.0) {
        return;
    }
    const double radius_sq = radius * radius;
    for (auto &entry : records_) {
        Record &record = entry.second;
        if (!record.reference.allFinite() ||
            (record.reference - position).squaredNorm() > radius_sq) {
            continue;
        }
        if (record.state == State::BLOCKED && stamp < record.block_until) {
            continue;
        }
        record.state = State::COVERED;
        record.last_completed_stamp = std::max(record.last_completed_stamp, stamp);
        record.last_state_stamp = stamp;
    }
}

bool ExplorationFrontierDB::goalActive(const ExplorationGoal &goal,
                                       const double stamp) const
{
    if (!enabled() || !goal.valid) {
        return false;
    }
    const Record *record = findRecordForGoal(goal);
    if (record == nullptr) {
        return true;
    }
    return recordActive(*record, stamp);
}

std::string ExplorationFrontierDB::objectKeyForGoal(
        const ExplorationGoal &goal,
        const std::string &sector_key) const
{
    const bool expansion = expansionCandidate(goal);
    if (expansion && !sector_key.empty()) {
        return "expansion_object:" + sector_key;
    }
    const std::string identity_frontier = goal.identity.frontierIdentityKey();
    if (!identity_frontier.empty()) {
        return identity_frontier;
    }
    if (goal.frontier_id >= 0) {
        return "frontier:" + std::to_string(goal.frontier_id);
    }
    if (goal.frontier_center_valid && goal.frontier_center.allFinite()) {
        return nhbp::quantizedPositionKey(
                goal.frontier_center,
                std::max({0.25,
                          cfg_.exploration_frontier_cluster_radius,
                          0.5 * cfg_.exploration_frontier_manager_match_radius}),
                "frontier_object");
    }
    if (!sector_key.empty()) {
        return "sector_object:" + sector_key;
    }
    if (goal.position.allFinite()) {
        return nhbp::quantizedPositionKey(
                goal.position,
                std::max(0.5, 0.5 * cfg_.exploration_active_sector_size),
                "frontier_object");
    }
    return {};
}

bool ExplorationFrontierDB::enabled() const
{
    return cfg_.exploration_task_planner_enable ||
           cfg_.exploration_frontier_manager_enable;
}

void ExplorationFrontierDB::prune(const double stamp)
{
    const double stale_time =
            std::max({cfg_.exploration_frontier_manager_stale_time,
                      cfg_.exploration_frontier_memory_ttl,
                      1.0});
    const double terminal_keep_time =
            std::max({4.0 * stale_time,
                      2.0 * cfg_.exploration_frontier_manager_dormant_time,
                      10.0});
    for (auto it = records_.begin(); it != records_.end();) {
        Record &record = it->second;
        if (record.state == State::BLOCKED && stamp >= record.block_until) {
            record.state = State::ACTIVE;
            record.block_until = 0.0;
        }
        if (record.state != State::COVERED &&
            record.state != State::BLOCKED &&
            stamp - record.last_seen_stamp > stale_time) {
            record.state = State::STALE;
            record.last_state_stamp = std::max(record.last_state_stamp,
                                               record.last_seen_stamp);
        }
        const bool terminal =
                record.state == State::COVERED || record.state == State::STALE;
        if (terminal &&
            record.last_state_stamp > 0.0 &&
            stamp - record.last_state_stamp > terminal_keep_time) {
            it = records_.erase(it);
        } else {
            ++it;
        }
    }
    const int max_records =
            std::max(1, cfg_.exploration_frontier_manager_max_records);
    if (static_cast<int>(records_.size()) <= max_records) {
        return;
    }

    std::vector<std::pair<std::string, double>> records_by_age;
    records_by_age.reserve(records_.size());
    for (const auto &entry : records_) {
        records_by_age.emplace_back(entry.first, entry.second.last_seen_stamp);
    }
    std::sort(records_by_age.begin(),
              records_by_age.end(),
              [](const auto &lhs, const auto &rhs) {
                  return lhs.second < rhs.second;
              });
    const int remove_count =
            static_cast<int>(records_.size()) - max_records;
    for (int i = 0; i < remove_count; ++i) {
        records_.erase(records_by_age[static_cast<size_t>(i)].first);
    }
}

void ExplorationFrontierDB::observeCandidate(
        const ExplorationGoal &candidate,
        const ObservationContext &context)
{
    if (!candidate.valid || !candidate.position.allFinite()) {
        return;
    }
    const std::string sector_key =
            context.sector_key ? context.sector_key(candidate) : std::string{};
    const std::string key = objectKeyForGoal(candidate, sector_key);
    if (key.empty()) {
        return;
    }

    Record &record = records_[key];
    if (record.key.empty()) {
        record.key = key;
        record.first_seen_stamp = context.stamp;
        record.last_state_stamp = context.stamp;
        record.state = State::ACTIVE;
    }

    const bool first_observation_this_stamp =
            std::abs(record.last_seen_stamp - context.stamp) > 1.0e-6;
    if (first_observation_this_stamp) {
        record.viewpoints.clear();
        record.candidate_count = 0;
        record.expansion_count = 0;
        record.total_gain = 0.0;
        record.max_gain = 0.0;
        record.coverage_intent = 0.0;
        record.best_score = std::numeric_limits<double>::infinity();
    }

    record.sector_key = sector_key;
    record.reference =
            context.sector_reference
                    ? context.sector_reference(candidate)
                    : (candidate.frontier_center_valid ? candidate.frontier_center
                                                       : candidate.position);
    if (candidate.frontier_center_valid) {
        record.bbox_min = candidate.frontier_bbox_min;
        record.bbox_max = candidate.frontier_bbox_max;
    } else {
        record.bbox_min = candidate.position;
        record.bbox_max = candidate.position;
    }
    record.last_seen_stamp = context.stamp;
    ++record.total_seen_count;
    ++record.candidate_count;
    if (expansionCandidate(candidate)) {
        ++record.expansion_count;
    }

    double adjusted_score = finiteOr(candidate.score);
    if (context.node_penalty) {
        adjusted_score += clampPositive(context.node_penalty(candidate));
    }

    ExplorationGoal adjusted = candidate;
    adjusted.score = adjusted_score;
    adjusted.identity.frontier_key =
            adjusted.identity.frontier_key.empty() ? key : adjusted.identity.frontier_key;
    adjusted.reason += " frontier_db_object=" + key;
    insertViewpoint(record, adjusted);

    record.total_gain += clampPositive(candidate.information_gain);
    record.max_gain = std::max(record.max_gain, clampPositive(candidate.information_gain));
    if (context.coverage_intent_reward) {
        record.coverage_intent =
                std::max(record.coverage_intent,
                         clampPositive(context.coverage_intent_reward(candidate)));
    }
    record.best_score = std::min(record.best_score, adjusted_score);

    if (record.state == State::BLOCKED && context.stamp < record.block_until) {
        return;
    }
    if (record.state == State::BLOCKED && context.stamp >= record.block_until) {
        record.block_until = 0.0;
    }
    record.state = State::ACTIVE;
    record.last_state_stamp = context.stamp;
}

void ExplorationFrontierDB::insertViewpoint(Record &record,
                                            const ExplorationGoal &candidate) const
{
    record.viewpoints.push_back(candidate);
    std::sort(record.viewpoints.begin(),
              record.viewpoints.end(),
              [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                  if (lhs.score != rhs.score) {
                      return lhs.score < rhs.score;
                  }
                  return lhs.information_gain > rhs.information_gain;
              });
    const int keep =
            std::max(1, cfg_.exploration_task_planner_viewpoints_per_frontier);
    if (static_cast<int>(record.viewpoints.size()) > keep) {
        record.viewpoints.resize(static_cast<size_t>(keep));
    }
}

bool ExplorationFrontierDB::recordActive(const Record &record,
                                         const double stamp) const
{
    const double stale_time =
            std::max({cfg_.exploration_frontier_manager_stale_time,
                      cfg_.exploration_frontier_memory_ttl,
                      1.0});
    if (record.state == State::BLOCKED) {
        return stamp >= record.block_until;
    }
    if (record.state == State::COVERED) {
        return false;
    }
    if (record.state == State::STALE) {
        return false;
    }
    if (record.last_seen_stamp > 0.0 &&
        stamp - record.last_seen_stamp > stale_time) {
        return false;
    }
    return true;
}

double ExplorationFrontierDB::recordPenalty(const Record &record,
                                            const double stamp) const
{
    double penalty = 0.0;
    penalty += cfg_.exploration_frontier_manager_selection_penalty *
               static_cast<double>(std::max(0, record.selection_count));
    penalty += 0.5 * cfg_.exploration_frontier_manager_selection_penalty *
               static_cast<double>(std::max(0, record.failure_count));
    penalty += cfg_.exploration_sector_completed_penalty *
               static_cast<double>(std::max(0, record.completed_count));

    if (record.state == State::BLOCKED && stamp < record.block_until) {
        penalty += cfg_.exploration_sector_blocked_penalty;
    }
    if (record.last_committed_stamp > 0.0 &&
        cfg_.exploration_frontier_manager_recent_selection_window > 1.0e-6) {
        const double elapsed = std::max(0.0, stamp - record.last_committed_stamp);
        if (elapsed < cfg_.exploration_frontier_manager_recent_selection_window) {
            const double ratio =
                    1.0 - elapsed /
                                  cfg_.exploration_frontier_manager_recent_selection_window;
            penalty += cfg_.exploration_frontier_manager_recent_selection_penalty *
                       ratio;
        }
    }
    return penalty;
}

ExplorationFrontierDB::ObjectSnapshot ExplorationFrontierDB::makeSnapshot(
        const Record &record,
        const double stamp) const
{
    ObjectSnapshot snapshot;
    snapshot.key = record.key;
    snapshot.sector_key = record.sector_key;
    snapshot.reference = record.reference;
    snapshot.bbox_min = record.bbox_min;
    snapshot.bbox_max = record.bbox_max;
    snapshot.viewpoints = record.viewpoints;
    snapshot.state = record.state;
    snapshot.best_score =
            finiteOr(record.best_score, std::numeric_limits<double>::infinity()) +
            recordPenalty(record, stamp);
    snapshot.total_gain = record.total_gain;
    snapshot.max_gain = record.max_gain;
    snapshot.coverage_intent = record.coverage_intent;
    snapshot.first_seen_stamp = record.first_seen_stamp;
    snapshot.last_seen_stamp = record.last_seen_stamp;
    snapshot.last_committed_stamp = record.last_committed_stamp;
    snapshot.last_completed_stamp = record.last_completed_stamp;
    snapshot.block_until = record.block_until;
    snapshot.candidate_count = record.candidate_count;
    snapshot.total_seen_count = record.total_seen_count;
    snapshot.selection_count = record.selection_count;
    snapshot.failure_count = record.failure_count;
    snapshot.completed_count = record.completed_count;
    snapshot.expansion_count = record.expansion_count;
    snapshot.expansion_only =
            record.candidate_count > 0 &&
            record.expansion_count == record.candidate_count;
    for (ExplorationGoal &viewpoint : snapshot.viewpoints) {
        viewpoint.score += recordPenalty(record, stamp);
    }
    return snapshot;
}

ExplorationFrontierDB::Record *ExplorationFrontierDB::findRecordForGoal(
        const ExplorationGoal &goal)
{
    const std::string key = objectKeyForGoal(goal, std::string{});
    auto it = records_.find(key);
    if (it != records_.end()) {
        return &it->second;
    }
    if (goal.frontier_id >= 0) {
        const std::string frontier_key = "frontier:" + std::to_string(goal.frontier_id);
        it = records_.find(frontier_key);
        if (it != records_.end()) {
            return &it->second;
        }
    }
    const std::string identity_frontier = goal.identity.frontierIdentityKey();
    if (!identity_frontier.empty()) {
        it = records_.find(identity_frontier);
        if (it != records_.end()) {
            return &it->second;
        }
    }
    if (!goal.position.allFinite()) {
        return nullptr;
    }
    const double match_radius =
            std::max({cfg_.exploration_active_tour_match_radius,
                      cfg_.exploration_frontier_manager_match_radius,
                      cfg_.exploration_goal_reached_distance,
                      0.75});
    const double match_radius_sq = match_radius * match_radius;
    Record *best = nullptr;
    double best_dist_sq = std::numeric_limits<double>::infinity();
    for (auto &entry : records_) {
        Record &record = entry.second;
        if (!record.reference.allFinite()) {
            continue;
        }
        const double dist_sq = (record.reference - goal.position).squaredNorm();
        if (dist_sq <= match_radius_sq && dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best = &record;
        }
    }
    return best;
}

const ExplorationFrontierDB::Record *ExplorationFrontierDB::findRecordForGoal(
        const ExplorationGoal &goal) const
{
    const std::string key = objectKeyForGoal(goal, std::string{});
    auto it = records_.find(key);
    if (it != records_.end()) {
        return &it->second;
    }
    if (goal.frontier_id >= 0) {
        const std::string frontier_key = "frontier:" + std::to_string(goal.frontier_id);
        it = records_.find(frontier_key);
        if (it != records_.end()) {
            return &it->second;
        }
    }
    const std::string identity_frontier = goal.identity.frontierIdentityKey();
    if (!identity_frontier.empty()) {
        it = records_.find(identity_frontier);
        if (it != records_.end()) {
            return &it->second;
        }
    }
    if (!goal.position.allFinite()) {
        return nullptr;
    }
    const double match_radius =
            std::max({cfg_.exploration_active_tour_match_radius,
                      cfg_.exploration_frontier_manager_match_radius,
                      cfg_.exploration_goal_reached_distance,
                      0.75});
    const double match_radius_sq = match_radius * match_radius;
    const Record *best = nullptr;
    double best_dist_sq = std::numeric_limits<double>::infinity();
    for (const auto &entry : records_) {
        const Record &record = entry.second;
        if (!record.reference.allFinite()) {
            continue;
        }
        const double dist_sq = (record.reference - goal.position).squaredNorm();
        if (dist_sq <= match_radius_sq && dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best = &record;
        }
    }
    return best;
}

bool ExplorationFrontierDB::expansionCandidate(const ExplorationGoal &goal)
{
    return goal.identity.intent_mode == "exploration_expansion" ||
           goal.reason.find("expansion") != std::string::npos;
}

} // namespace general_planner
