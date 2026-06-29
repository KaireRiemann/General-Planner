#include <general_core/nhbp/navigation_memory.hpp>

#include <algorithm>

namespace general_planner::nhbp {

NavigationMemory::NavigationMemory()
    : NavigationMemory(Config{})
{
}

NavigationMemory::NavigationMemory(Config config)
    : config_(config)
{
}

void NavigationMemory::reset()
{
    decisions_.clear();
    failures_.clear();
}

void NavigationMemory::recordDecision(const DecisionRecord &decision)
{
    decisions_.push_back(decision);
    const int max_history = std::max(1, config_.max_decision_history);
    if (static_cast<int>(decisions_.size()) > max_history) {
        decisions_.erase(decisions_.begin(),
                         decisions_.begin() +
                                 (static_cast<int>(decisions_.size()) - max_history));
    }
}

void NavigationMemory::recordFailure(const std::string &key,
                                     const FailureReason reason,
                                     const double stamp,
                                     const double ttl)
{
    FailureRecord &record = failures_[key];
    if (record.count == 0) {
        record.key = key;
        record.first_time = stamp;
    }
    record.reason = reason;
    record.last_time = stamp;
    record.blacklist_until = stamp + std::max(0.0, ttl > 0.0 ? ttl : config_.default_blacklist_ttl);
    ++record.count;
}

bool NavigationMemory::isBlacklisted(const std::string &key, const double stamp) const
{
    const auto it = failures_.find(key);
    return it != failures_.end() && it->second.blacklist_until > stamp;
}

NdoDiagnosis NavigationMemory::diagnose(const double stamp) const
{
    NdoDiagnosis diagnosis;
    if (repeatedSwitching()) {
        diagnosis.state = NdoState::SUSPECT;
        diagnosis.reason = "candidate_switch_cycle";
        for (const DecisionRecord &decision : decisions_) {
            diagnosis.implicated_candidate_ids.push_back(decision.candidate_id);
        }
    }
    if (noProgress(stamp)) {
        diagnosis.state = diagnosis.state == NdoState::SUSPECT
                                  ? NdoState::DEADLOCKED
                                  : NdoState::SUSPECT;
        diagnosis.reason = diagnosis.state == NdoState::DEADLOCKED
                                   ? "switch_cycle_and_no_progress"
                                   : "no_progress";
    }
    return diagnosis;
}

const std::vector<DecisionRecord> &NavigationMemory::decisionHistory() const
{
    return decisions_;
}

const std::unordered_map<std::string, FailureRecord> &NavigationMemory::failures() const
{
    return failures_;
}

bool NavigationMemory::repeatedSwitching() const
{
    if (decisions_.size() < 4) {
        return false;
    }

    int switch_count = 0;
    int first_id = -1;
    int second_id = -1;
    int previous_id = decisions_.front().candidate_id;
    if (previous_id >= 0) {
        first_id = previous_id;
    }

    for (size_t i = 1; i < decisions_.size(); ++i) {
        const int id = decisions_[i].candidate_id;
        if (id < 0) {
            continue;
        }
        if (first_id < 0) {
            first_id = id;
        } else if (id != first_id && second_id < 0) {
            second_id = id;
        } else if (id != first_id && id != second_id) {
            return false;
        }
        if (previous_id >= 0 && id != previous_id) {
            ++switch_count;
        }
        previous_id = id;
    }

    return first_id >= 0 &&
           second_id >= 0 &&
           switch_count >= std::max(2, config_.max_switches);
}

bool NavigationMemory::noProgress(const double stamp) const
{
    if (decisions_.size() < 2) {
        return false;
    }
    const DecisionRecord &first = decisions_.front();
    const DecisionRecord &last = decisions_.back();
    if (stamp - first.stamp < config_.no_progress_time) {
        return false;
    }
    return (last.position - first.position).norm() < config_.no_progress_distance;
}

const char *toString(const FailureReason reason)
{
    switch (reason) {
        case FailureReason::ASTAR_FAIL:
            return "ASTAR_FAIL";
        case FailureReason::ASTAR_TOO_LONG:
            return "ASTAR_TOO_LONG";
        case FailureReason::OPTIMIZATION_FAIL:
            return "OPTIMIZATION_FAIL";
        case FailureReason::SAFETY_COLLISION:
            return "SAFETY_COLLISION";
        case FailureReason::NO_PROGRESS:
            return "NO_PROGRESS";
        case FailureReason::NDO_OSCILLATION:
            return "NDO_OSCILLATION";
        case FailureReason::BACKUP_TRIGGERED:
            return "BACKUP_TRIGGERED";
        case FailureReason::VIEWPOINT_UNSAFE:
            return "VIEWPOINT_UNSAFE";
        case FailureReason::FRONTIER_COVERED:
            return "FRONTIER_COVERED";
        case FailureReason::MAP_STALE:
            return "MAP_STALE";
    }
    return "UNKNOWN";
}

const char *toString(const NdoState state)
{
    switch (state) {
        case NdoState::STABLE:
            return "STABLE";
        case NdoState::SUSPECT:
            return "SUSPECT";
        case NdoState::DEADLOCKED:
            return "DEADLOCKED";
    }
    return "UNKNOWN";
}

} // namespace general_planner::nhbp
