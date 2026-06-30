#include <general_core/exploration/frontier_memory.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace general_planner {

FrontierMemory::FrontierMemory()
    : FrontierMemory(Config{})
{
}

FrontierMemory::FrontierMemory(Config config)
    : config_(config)
{
}

void FrontierMemory::reset()
{
    records_.clear();
}

void FrontierMemory::observe(const ExplorationGoal &goal, const double stamp)
{
    if (!config_.enable || !goal.valid) {
        return;
    }
    FrontierMemoryRecord &record = upsert(goal, stamp);
    record.goal = goal;
    record.last_seen_stamp = stamp;
    ++record.observe_count;
    if (record.state == FrontierMemoryState::COVERED &&
        record.blocked_until <= stamp) {
        record.state = FrontierMemoryState::ACTIVE;
        record.last_state_stamp = stamp;
    }
    prune(stamp);
}

void FrontierMemory::markCommitted(const ExplorationGoal &goal, const double stamp)
{
    if (!config_.enable || !goal.valid) {
        return;
    }
    FrontierMemoryRecord &record = upsert(goal, stamp);
    record.goal = goal;
    record.state = FrontierMemoryState::COMMITTED;
    record.last_state_stamp = stamp;
    record.last_seen_stamp = stamp;
    prune(stamp);
}

void FrontierMemory::markFailed(const ExplorationGoal &goal, const double stamp)
{
    if (!config_.enable || !goal.valid) {
        return;
    }
    FrontierMemoryRecord &record = upsert(goal, stamp);
    record.goal = goal;
    record.state = FrontierMemoryState::FAILED;
    record.last_state_stamp = stamp;
    record.last_seen_stamp = stamp;
    record.blocked_until = stamp + std::max(0.0, config_.failure_ttl);
    ++record.failure_count;
    prune(stamp);
}

void FrontierMemory::markCoveredNear(const general_utils::Vec3f &position,
                                     const double stamp)
{
    if (!config_.enable || !position.allFinite()) {
        return;
    }
    const double radius = std::max(0.0, config_.covered_radius);
    const double radius_sq = radius * radius;
    for (auto &entry : records_) {
        FrontierMemoryRecord &record = entry.second;
        if (!record.goal.valid ||
            !record.goal.position.allFinite() ||
            record.state == FrontierMemoryState::COVERED) {
            continue;
        }
        if ((record.goal.position - position).squaredNorm() <= radius_sq) {
            record.state = FrontierMemoryState::COVERED;
            record.last_state_stamp = stamp;
            record.blocked_until = stamp + std::max(config_.record_ttl, config_.failure_ttl);
        }
    }
    prune(stamp);
}

void FrontierMemory::prune(const double stamp)
{
    if (!config_.enable) {
        records_.clear();
        return;
    }

    for (auto it = records_.begin(); it != records_.end();) {
        if (expired(it->second, stamp)) {
            it = records_.erase(it);
        } else {
            ++it;
        }
    }

    const int max_records = std::max(1, config_.max_records);
    while (static_cast<int>(records_.size()) > max_records) {
        auto oldest = records_.begin();
        for (auto it = records_.begin(); it != records_.end(); ++it) {
            if (it->second.last_seen_stamp < oldest->second.last_seen_stamp) {
                oldest = it;
            }
        }
        records_.erase(oldest);
    }
}

bool FrontierMemory::blocked(const ExplorationGoal &goal, const double stamp) const
{
    if (!config_.enable || !goal.valid) {
        return false;
    }
    const auto it = records_.find(keyForGoal(goal));
    if (it != records_.end()) {
        const FrontierMemoryRecord &record = it->second;
        if ((record.state == FrontierMemoryState::FAILED ||
             record.state == FrontierMemoryState::COVERED) &&
            record.blocked_until > stamp) {
            return true;
        }
    }
    return failedRegionBlocked(goal, stamp);
}

bool FrontierMemory::hasRecoverableGoal(const general_utils::Vec3f &robot_pos,
                                        const double stamp,
                                        ExplorationGoal &goal,
                                        const std::function<bool(const ExplorationGoal &)> &accept) const
{
    goal = ExplorationGoal{};
    if (!config_.enable || !robot_pos.allFinite()) {
        return false;
    }

    double best_score = std::numeric_limits<double>::infinity();
    bool found = false;
    const double min_distance = std::max(0.0, config_.recovery_min_distance);
    for (const auto &entry : records_) {
        const FrontierMemoryRecord &record = entry.second;
        if (!record.goal.valid ||
            !record.goal.position.allFinite() ||
            expired(record, stamp) ||
            record.state == FrontierMemoryState::FAILED ||
            record.state == FrontierMemoryState::COVERED ||
            record.blocked_until > stamp) {
            continue;
        }
        if (accept && !accept(record.goal)) {
            continue;
        }
        if (failedRegionBlocked(record.goal, stamp)) {
            continue;
        }
        const double distance = (record.goal.position - robot_pos).norm();
        if (distance < min_distance) {
            continue;
        }
        const double score = record.goal.score + 0.2 * distance;
        if (score < best_score) {
            best_score = score;
            goal = record.goal;
            goal.reason = "frontier_memory_recovery";
            found = true;
        }
    }
    return found;
}

int FrontierMemory::activeCount(const double stamp) const
{
    int count = 0;
    for (const auto &entry : records_) {
        const FrontierMemoryRecord &record = entry.second;
        if (!expired(record, stamp) &&
            (record.state == FrontierMemoryState::ACTIVE ||
             record.state == FrontierMemoryState::COMMITTED)) {
            ++count;
        }
    }
    return count;
}

int FrontierMemory::failedCount(const double stamp) const
{
    int count = 0;
    for (const auto &entry : records_) {
        const FrontierMemoryRecord &record = entry.second;
        if (!expired(record, stamp) &&
            record.state == FrontierMemoryState::FAILED &&
            record.blocked_until > stamp) {
            ++count;
        }
    }
    return count;
}

int FrontierMemory::coveredCount() const
{
    int count = 0;
    for (const auto &entry : records_) {
        if (entry.second.state == FrontierMemoryState::COVERED) {
            ++count;
        }
    }
    return count;
}

std::string FrontierMemory::keyForGoal(const ExplorationGoal &goal) const
{
    if (!goal.memory_key.empty()) {
        return goal.memory_key;
    }
    if (goal.candidate_id >= 0) {
        return std::to_string(goal.candidate_id);
    }
    std::ostringstream oss;
    oss << static_cast<int>(std::floor(goal.position.x() * 10.0)) << ":"
        << static_cast<int>(std::floor(goal.position.y() * 10.0)) << ":"
        << static_cast<int>(std::floor(goal.position.z() * 10.0));
    return oss.str();
}

FrontierMemoryRecord &FrontierMemory::upsert(const ExplorationGoal &goal,
                                             const double stamp)
{
    FrontierMemoryRecord &record = records_[keyForGoal(goal)];
    if (record.observe_count == 0 &&
        record.failure_count == 0 &&
        record.first_seen_stamp <= 0.0) {
        record.first_seen_stamp = stamp;
        record.last_state_stamp = stamp;
    }
    return record;
}

bool FrontierMemory::expired(const FrontierMemoryRecord &record,
                             const double stamp) const
{
    if (record.blocked_until > stamp) {
        return false;
    }
    const double ttl = std::max(0.0, config_.record_ttl);
    return ttl > 0.0 && stamp - record.last_seen_stamp > ttl;
}

bool FrontierMemory::failedRegionBlocked(const ExplorationGoal &goal,
                                         const double stamp) const
{
    if (!goal.valid || !goal.position.allFinite()) {
        return false;
    }
    const double radius = std::max(0.0, config_.failure_block_radius);
    if (radius <= 0.0) {
        return false;
    }
    const double radius_sq = radius * radius;
    const std::string goal_key = keyForGoal(goal);
    for (const auto &entry : records_) {
        if (entry.first == goal_key) {
            continue;
        }
        const FrontierMemoryRecord &record = entry.second;
        if (record.state != FrontierMemoryState::FAILED ||
            record.blocked_until <= stamp ||
            expired(record, stamp) ||
            !record.goal.valid ||
            !record.goal.position.allFinite()) {
            continue;
        }
        if ((record.goal.position - goal.position).squaredNorm() <= radius_sq) {
            return true;
        }
    }
    return false;
}

const char *toString(const FrontierMemoryState state)
{
    switch (state) {
        case FrontierMemoryState::ACTIVE:
            return "ACTIVE";
        case FrontierMemoryState::COMMITTED:
            return "COMMITTED";
        case FrontierMemoryState::FAILED:
            return "FAILED";
        case FrontierMemoryState::COVERED:
            return "COVERED";
    }
    return "UNKNOWN";
}

} // namespace general_planner
