#include <general_core/nhbp/navigation_memory.hpp>

#include <algorithm>
#include <functional>
#include <unordered_set>

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
    traces_.clear();
    failures_.clear();
}

void NavigationMemory::recordDecision(const DecisionRecord &decision)
{
    DecisionRecord stored = decision;
    if (!stored.robot_position.allFinite() ||
        stored.robot_position.squaredNorm() <= 1.0e-12) {
        stored.robot_position = stored.position;
    }
    if (!stored.target_position.allFinite()) {
        stored.target_position = general_utils::Vec3f::Zero();
    }
    if (!stored.selected_goal.allFinite() ||
        stored.selected_goal.squaredNorm() <= 1.0e-12) {
        stored.selected_goal = stored.target_position;
    }
    if (!stored.committed_goal.allFinite()) {
        stored.committed_goal = general_utils::Vec3f::Zero();
    }
    if (!stored.committed_end.allFinite()) {
        stored.committed_end = stored.target_position;
    }
    if (!stored.guide_first_direction.allFinite()) {
        stored.guide_first_direction = general_utils::Vec3f::Zero();
    }
    if (stored.raw_score == 0.0) {
        stored.raw_score = stored.score;
    }
    if (stored.final_score == 0.0) {
        stored.final_score = stored.score;
    }
    if (stored.goal_distance <= 0.0 &&
        stored.robot_position.allFinite() &&
        stored.target_position.allFinite() &&
        stored.target_position.squaredNorm() > 1.0e-12) {
        stored.goal_distance = (stored.target_position - stored.robot_position).norm();
    }
    if (!decisions_.empty() && stored.robot_position.allFinite()) {
        const DecisionRecord &previous = decisions_.back();
        if (previous.robot_position.allFinite()) {
            stored.travel_since_last =
                    (stored.robot_position - previous.robot_position).norm();
        }
        if (stored.goal_progress == 0.0 &&
            sameGoalIntent(stored.identity, previous.identity) &&
            previous.goal_distance > 0.0 &&
            stored.goal_distance > 0.0) {
            stored.goal_progress = previous.goal_distance - stored.goal_distance;
        }
    }
    decisions_.push_back(stored);
    const int max_history = std::max(1, config_.max_decision_history);
    if (static_cast<int>(decisions_.size()) > max_history) {
        decisions_.erase(decisions_.begin(),
                         decisions_.begin() +
                                 (static_cast<int>(decisions_.size()) - max_history));
    }
}

void NavigationMemory::recordTrace(const DecisionTrace &trace)
{
    DecisionTrace stored = trace;
    if (stored.raw_score == 0.0) {
        stored.raw_score = stored.score;
    }
    if (stored.final_score == 0.0) {
        stored.final_score = stored.score;
    }
    stored.keep_current =
            stored.keep_current || stored.action == DecisionTraceAction::KEEP_CURRENT;
    stored.recovery =
            stored.recovery ||
            stored.action == DecisionTraceAction::RECOVERY_REQUESTED ||
            stored.action == DecisionTraceAction::RECOVERY_SELECTED ||
            stored.identity.recovery_intent;
    stored.committed =
            stored.committed || stored.action == DecisionTraceAction::COMMIT;
    traces_.push_back(stored);
    const int max_history = std::max(4, config_.max_decision_history * 4);
    if (static_cast<int>(traces_.size()) > max_history) {
        traces_.erase(traces_.begin(),
                      traces_.begin() +
                              (static_cast<int>(traces_.size()) - max_history));
    }
}

void NavigationMemory::recordFailure(const std::string &key,
                                     const FailureReason reason,
                                     const double stamp,
                                     const double ttl)
{
    if (key.empty()) {
        return;
    }
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
    diagnosis.metrics = computeMetrics(stamp);
    diagnosis.goal_switch_count = diagnosis.metrics.candidate_switch_count;
    diagnosis.frontier_switch_count = diagnosis.metrics.frontier_switch_count;
    diagnosis.guide_switch_count = diagnosis.metrics.guide_direction_switch_count;
    diagnosis.recovery_request_count = diagnosis.metrics.recovery_request_count;
    diagnosis.robot_progress = diagnosis.metrics.net_displacement;

    if (repeatedSwitching(diagnosis)) {
        addReason(diagnosis, NdoState::SUSPECT, diagnosis.reason);
    }
    if (noProgress(stamp, diagnosis.metrics)) {
        addReason(diagnosis,
                  diagnosis.state == NdoState::SUSPECT
                          ? NdoState::DEADLOCKED
                          : NdoState::SUSPECT,
                  "no_progress");
    }
    if (revisitLoop(diagnosis.metrics)) {
        addReason(diagnosis,
                  diagnosis.state == NdoState::SUSPECT
                          ? NdoState::DEADLOCKED
                          : NdoState::SUSPECT,
                  "revisit_loop");
    }
    if (commitChurn(diagnosis.metrics)) {
        addReason(diagnosis,
                  diagnosis.state == NdoState::SUSPECT
                          ? NdoState::DEADLOCKED
                          : NdoState::SUSPECT,
                  "commit_churn");
    }
    if (failureLoop(stamp, diagnosis)) {
        addReason(diagnosis, NdoState::DEADLOCKED, "failure_loop");
    }
    if (recoveryLoop(stamp, diagnosis)) {
        addReason(diagnosis, NdoState::DEADLOCKED, "recovery_request_loop");
    }
    return diagnosis;
}

NdoMetrics NavigationMemory::latestMetrics(const double stamp) const
{
    return computeMetrics(stamp);
}

const std::vector<DecisionRecord> &NavigationMemory::decisionHistory() const
{
    return decisions_;
}

const std::vector<DecisionTrace> &NavigationMemory::traceHistory() const
{
    return traces_;
}

const std::unordered_map<std::string, FailureRecord> &NavigationMemory::failures() const
{
    return failures_;
}

bool NavigationMemory::repeatedSwitching(NdoDiagnosis &diagnosis) const
{
    if (decisions_.size() < 4) {
        return false;
    }

    const auto count_switches =
            [this](const std::function<std::string(const DecisionRecord &)> &key_fn,
                   std::vector<std::string> &keys) {
                keys.clear();
                std::string previous;
                int switch_count = 0;
                for (const DecisionRecord &decision : decisions_) {
                    const std::string key = key_fn(decision);
                    if (key.empty()) {
                        continue;
                    }
                    if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
                        keys.push_back(key);
                    }
                    if (!previous.empty() && key != previous) {
                        ++switch_count;
                    }
                    previous = key;
                }
                return switch_count;
            };

    std::vector<std::string> goal_keys;
    std::vector<std::string> frontier_keys;
    std::vector<std::string> guide_keys;
    std::vector<std::string> branch_keys;
    const int goal_switches = count_switches(
            [this](const DecisionRecord &decision) { return decisionGoalKey(decision); },
            goal_keys);
    const int frontier_switches = count_switches(
            [this](const DecisionRecord &decision) { return decisionFrontierKey(decision); },
            frontier_keys);
    const int guide_switches = count_switches(
            [this](const DecisionRecord &decision) { return decisionGuideKey(decision); },
            guide_keys);
    const int branch_switches = count_switches(
            [this](const DecisionRecord &decision) { return decisionBranchKey(decision); },
            branch_keys);

    diagnosis.goal_switch_count = goal_switches;
    diagnosis.frontier_switch_count = frontier_switches;
    diagnosis.guide_switch_count = guide_switches;
    diagnosis.metrics.branch_switch_count = branch_switches;

    const int switch_threshold = std::max(2, config_.max_switches);
    const bool goal_churn =
            goal_keys.size() >= 2 && goal_switches >= switch_threshold;
    const bool frontier_churn =
            frontier_keys.size() >= 2 && frontier_switches >= switch_threshold;
    const bool guide_churn =
            guide_keys.size() >= 2 && guide_switches >= switch_threshold;
    const bool branch_churn =
            branch_keys.size() >= 2 && branch_switches >= switch_threshold;
    if (!goal_churn && !frontier_churn && !guide_churn && !branch_churn) {
        return false;
    }

    if (goal_churn) {
        diagnosis.reason = "goal_identity_switch_cycle";
        diagnosis.implicated_identity_keys = goal_keys;
    } else if (branch_churn) {
        diagnosis.reason = "branch_switch_cycle";
        diagnosis.implicated_identity_keys = branch_keys;
        diagnosis.implicated_branch_keys = branch_keys;
    } else if (frontier_churn) {
        diagnosis.reason = "frontier_switch_cycle";
        diagnosis.implicated_identity_keys = frontier_keys;
    } else {
        diagnosis.reason = "guide_path_churn";
        diagnosis.implicated_identity_keys = guide_keys;
    }
    for (const DecisionRecord &decision : decisions_) {
        if (decision.candidate_id >= 0 &&
            std::find(diagnosis.implicated_candidate_ids.begin(),
                      diagnosis.implicated_candidate_ids.end(),
                      decision.candidate_id) == diagnosis.implicated_candidate_ids.end()) {
            diagnosis.implicated_candidate_ids.push_back(decision.candidate_id);
        }
        if (decision.frontier_id >= 0 &&
            std::find(diagnosis.implicated_frontier_ids.begin(),
                      diagnosis.implicated_frontier_ids.end(),
                      decision.frontier_id) == diagnosis.implicated_frontier_ids.end()) {
            diagnosis.implicated_frontier_ids.push_back(decision.frontier_id);
        }
        if (decision.identity.topo_edge_id >= 0 &&
            std::find(diagnosis.implicated_topo_edges.begin(),
                      diagnosis.implicated_topo_edges.end(),
                      decision.identity.topo_edge_id) == diagnosis.implicated_topo_edges.end()) {
            diagnosis.implicated_topo_edges.push_back(decision.identity.topo_edge_id);
        }
    }
    return true;
}

bool NavigationMemory::noProgress(const double stamp,
                                  const NdoMetrics &metrics) const
{
    if (decisions_.size() < 2) {
        return false;
    }
    const DecisionRecord &first = decisions_.front();
    const DecisionRecord &last = decisions_.back();
    if (stamp - first.stamp < config_.no_progress_time) {
        return false;
    }
    const general_utils::Vec3f first_pos =
            first.robot_position.allFinite() ? first.robot_position : first.position;
    const general_utils::Vec3f last_pos =
            last.robot_position.allFinite() ? last.robot_position : last.position;
    if (!first_pos.allFinite() || !last_pos.allFinite()) {
        return false;
    }
    const double min_distance = std::max(0.0, config_.no_progress_distance);
    const double min_travel = std::max(min_distance * 2.0, 0.5);
    return metrics.travel_distance >= min_travel &&
           metrics.net_displacement < min_distance &&
           metrics.goal_progress < min_distance &&
           metrics.progress_ratio < 0.35;
}

bool NavigationMemory::revisitLoop(const NdoMetrics &metrics) const
{
    return metrics.revisit_count >= 2 &&
           metrics.travel_distance >= std::max(0.5, config_.no_progress_distance * 2.0) &&
           metrics.progress_ratio < 0.5;
}

bool NavigationMemory::failureLoop(const double stamp,
                                   NdoDiagnosis &diagnosis) const
{
    bool loop = false;
    for (const auto &entry : failures_) {
        const FailureRecord &record = entry.second;
        if (record.count < 2 ||
            record.blacklist_until <= stamp) {
            continue;
        }
        loop = true;
        if (std::find(diagnosis.implicated_identity_keys.begin(),
                      diagnosis.implicated_identity_keys.end(),
                      record.key) == diagnosis.implicated_identity_keys.end()) {
            diagnosis.implicated_identity_keys.push_back(record.key);
        }
    }
    const double min_net_progress =
            std::max(0.5, config_.no_progress_distance * 2.0);
    const bool poor_progress =
            diagnosis.metrics.progress_ratio < 0.65 ||
            diagnosis.metrics.net_displacement < min_net_progress;
    return loop && poor_progress;
}

bool NavigationMemory::commitChurn(const NdoMetrics &metrics) const
{
    const int threshold = std::max(3, config_.max_switches);
    return metrics.commit_count >= threshold &&
           metrics.candidate_switch_count >= std::max(2, threshold - 1) &&
           metrics.progress_ratio < 0.6;
}

bool NavigationMemory::recoveryLoop(const double stamp, NdoDiagnosis &diagnosis) const
{
    if (traces_.empty()) {
        return false;
    }
    int recovery_requests = 0;
    int recovery_failures = 0;
    std::unordered_set<std::string> recovery_keys;
    const double window = std::max(config_.no_progress_time, 1.0);
    for (auto it = traces_.rbegin(); it != traces_.rend(); ++it) {
        if (stamp - it->stamp > window) {
            break;
        }
        if (it->action == DecisionTraceAction::RECOVERY_REQUESTED) {
            ++recovery_requests;
            const std::string key = it->identity.canonicalKey();
            if (!key.empty()) {
                recovery_keys.insert(key);
            }
        } else if (it->action == DecisionTraceAction::FAILURE &&
                   it->identity.recovery_intent) {
            ++recovery_failures;
            const std::string key = it->identity.canonicalKey();
            if (!key.empty()) {
                recovery_keys.insert(key);
            }
        }
    }
    diagnosis.recovery_request_count = recovery_requests;
    const double min_net_progress =
            std::max(0.5, config_.no_progress_distance * 2.0);
    const bool poor_progress =
            diagnosis.metrics.progress_ratio < 0.65 ||
            diagnosis.metrics.net_displacement < min_net_progress;
    const bool repeated_recovery_identity =
            !recovery_keys.empty() && recovery_keys.size() <= 1 &&
            recovery_requests >= 3;
    return poor_progress &&
           ((recovery_requests >= 2 && recovery_failures >= 1) ||
            repeated_recovery_identity);
}

NdoMetrics NavigationMemory::computeMetrics(const double stamp) const
{
    NdoMetrics metrics;
    if (decisions_.empty()) {
        return metrics;
    }

    const double window = std::max(config_.no_progress_time, 1.0);
    std::vector<const DecisionRecord *> window_decisions;
    window_decisions.reserve(decisions_.size());
    for (const DecisionRecord &decision : decisions_) {
        if (stamp - decision.stamp <= window || window_decisions.empty()) {
            window_decisions.push_back(&decision);
        }
    }
    if (window_decisions.empty()) {
        window_decisions.push_back(&decisions_.back());
    }
    metrics.replan_count = static_cast<int>(window_decisions.size());

    const auto count_switches =
            [](const std::vector<std::string> &keys) {
                int switches = 0;
                std::string previous;
                for (const std::string &key : keys) {
                    if (key.empty()) {
                        continue;
                    }
                    if (!previous.empty() && key != previous) {
                        ++switches;
                    }
                    previous = key;
                }
                return switches;
            };

    std::vector<std::string> goal_keys;
    std::vector<std::string> branch_keys;
    std::vector<std::string> frontier_keys;
    std::vector<std::string> guide_keys;
    goal_keys.reserve(window_decisions.size());
    branch_keys.reserve(window_decisions.size());
    frontier_keys.reserve(window_decisions.size());
    guide_keys.reserve(window_decisions.size());

    for (std::size_t i = 0; i < window_decisions.size(); ++i) {
        const DecisionRecord &decision = *window_decisions[i];
        goal_keys.push_back(decisionGoalKey(decision));
        branch_keys.push_back(decisionBranchKey(decision));
        frontier_keys.push_back(decisionFrontierKey(decision));
        guide_keys.push_back(decisionGuideKey(decision));
        if (i == 0) {
            continue;
        }
        const DecisionRecord &previous = *window_decisions[i - 1];
        if (decision.robot_position.allFinite() && previous.robot_position.allFinite()) {
            metrics.travel_distance +=
                    (decision.robot_position - previous.robot_position).norm();
        }
        metrics.goal_progress += std::max(0.0, decision.goal_progress);
    }

    const DecisionRecord &first = *window_decisions.front();
    const DecisionRecord &last = *window_decisions.back();
    if (first.robot_position.allFinite() && last.robot_position.allFinite()) {
        metrics.net_displacement = (last.robot_position - first.robot_position).norm();
    }
    if (metrics.goal_progress <= 0.0 &&
        sameGoalIntent(first.identity, last.identity) &&
        first.goal_distance > 0.0 &&
        last.goal_distance > 0.0) {
        metrics.goal_progress = std::max(0.0, first.goal_distance - last.goal_distance);
    }
    const double progress = std::max(metrics.net_displacement, metrics.goal_progress);
    metrics.progress_ratio =
            metrics.travel_distance > 1.0e-6
                    ? progress / std::max(1.0e-6, metrics.travel_distance)
                    : 1.0;

    metrics.candidate_switch_count = count_switches(goal_keys);
    metrics.branch_switch_count = count_switches(branch_keys);
    metrics.frontier_switch_count = count_switches(frontier_keys);
    metrics.guide_direction_switch_count = count_switches(guide_keys);

    const double revisit_radius = std::max(0.1, config_.no_progress_distance);
    const double revisit_radius_sq = revisit_radius * revisit_radius;
    for (std::size_t i = 2; i < window_decisions.size(); ++i) {
        const general_utils::Vec3f &pos = window_decisions[i]->robot_position;
        if (!pos.allFinite()) {
            continue;
        }
        for (std::size_t j = 0; j + 1 < i; ++j) {
            const general_utils::Vec3f &old_pos = window_decisions[j]->robot_position;
            if (!old_pos.allFinite()) {
                continue;
            }
            if ((pos - old_pos).squaredNorm() <= revisit_radius_sq) {
                ++metrics.revisit_count;
                break;
            }
        }
    }

    for (auto it = traces_.rbegin(); it != traces_.rend(); ++it) {
        if (stamp - it->stamp > window) {
            break;
        }
        if (it->action == DecisionTraceAction::COMMIT) {
            ++metrics.commit_count;
        } else if (it->action == DecisionTraceAction::FAILURE) {
            ++metrics.failure_count;
        } else if (it->action == DecisionTraceAction::KEEP_CURRENT) {
            ++metrics.keep_current_count;
        } else if (it->action == DecisionTraceAction::RECOVERY_REQUESTED) {
            ++metrics.recovery_request_count;
        }
    }

    return metrics;
}

std::string NavigationMemory::decisionGoalKey(const DecisionRecord &decision) const
{
    const std::string key = decision.identity.canonicalKey();
    if (!key.empty()) {
        return key;
    }
    if (!decision.reason.empty()) {
        return decision.reason;
    }
    if (decision.candidate_id >= 0) {
        return std::string("candidate:") + std::to_string(decision.candidate_id);
    }
    return {};
}

std::string NavigationMemory::decisionFrontierKey(const DecisionRecord &decision) const
{
    const std::string key = decision.identity.frontierIdentityKey();
    if (!key.empty()) {
        return key;
    }
    if (decision.frontier_id >= 0) {
        return std::string("frontier:") + std::to_string(decision.frontier_id);
    }
    return {};
}

std::string NavigationMemory::decisionGuideKey(const DecisionRecord &decision) const
{
    return decision.identity.guideIdentityKey();
}

std::string NavigationMemory::decisionBranchKey(const DecisionRecord &decision) const
{
    if (!decision.identity.branch_key.empty()) {
        return decision.identity.branch_key;
    }
    if (!decision.identity.tour_key.empty()) {
        return decision.identity.tour_key;
    }
    return {};
}

void NavigationMemory::addReason(NdoDiagnosis &diagnosis,
                                 const NdoState state,
                                 const std::string &reason) const
{
    if (state > diagnosis.state) {
        diagnosis.state = state;
    }
    if (reason.empty() || reason == "stable") {
        return;
    }
    if (diagnosis.reason == "stable") {
        diagnosis.reason = reason;
        return;
    }
    if (diagnosis.reason.find(reason) == std::string::npos) {
        diagnosis.reason += "_and_" + reason;
    }
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

const char *toString(const DecisionTraceAction action)
{
    switch (action) {
        case DecisionTraceAction::CANDIDATE_EVALUATED:
            return "CANDIDATE_EVALUATED";
        case DecisionTraceAction::ACCEPT_CANDIDATE:
            return "ACCEPT_CANDIDATE";
        case DecisionTraceAction::KEEP_CURRENT:
            return "KEEP_CURRENT";
        case DecisionTraceAction::REJECT_CANDIDATE:
            return "REJECT_CANDIDATE";
        case DecisionTraceAction::RECOVERY_REQUESTED:
            return "RECOVERY_REQUESTED";
        case DecisionTraceAction::RECOVERY_SELECTED:
            return "RECOVERY_SELECTED";
        case DecisionTraceAction::COMMIT:
            return "COMMIT";
        case DecisionTraceAction::FAILURE:
            return "FAILURE";
    }
    return "UNKNOWN";
}

} // namespace general_planner::nhbp
