#include "general_core/exploration/exploration_runtime_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

namespace general_planner {

ExplorationRuntimeManager::ExplorationRuntimeManager(const Config &cfg)
        : cfg_(cfg),
          navigation_memory_(nhbp::NavigationMemory::Config{
                  cfg.exploration_nhbp_decision_history,
                  cfg.exploration_nhbp_blacklist_ttl,
                  cfg.exploration_nhbp_min_progress_distance,
                  cfg.exploration_nhbp_no_progress_time,
                  cfg.exploration_nhbp_max_switches}),
          decision_stabilizer_(nhbp::DecisionStabilizer::Config{
                  cfg.exploration_nhbp_enable,
                  cfg.exploration_nhbp_min_commit_time,
                  cfg.exploration_nhbp_switch_margin,
                  cfg.exploration_nhbp_recovery_enable}),
          frontier_memory_(FrontierMemory::Config{
                  cfg.exploration_use_frontier_memory,
                  cfg.exploration_frontier_memory_max_records,
                  cfg.exploration_frontier_memory_ttl,
                  cfg.exploration_frontier_memory_failure_ttl,
                  cfg.exploration_frontier_memory_failure_block_radius,
                  cfg.exploration_frontier_memory_covered_radius,
                  cfg.exploration_frontier_memory_recovery_min_distance}),
          coverage_grid_(CoverageGrid::Config{
                  cfg.exploration_use_coverage_grid,
                  cfg.exploration_coverage_grid_resolution,
                  cfg.exploration_coverage_revisit_radius,
                  cfg.exploration_coverage_revisit_time_window,
                  cfg.exploration_coverage_grid_max_cells}),
          topological_memory_(nhbp::TopologicalMemory::Config{
                  cfg.exploration_use_topological_memory,
                  cfg.exploration_topology_max_nodes,
                  cfg.exploration_topology_max_edges,
                  cfg.exploration_topology_node_merge_radius,
                  cfg.exploration_topology_node_blacklist_ttl,
                  cfg.exploration_topology_edge_blacklist_ttl,
                  cfg.exploration_topology_recovery_min_distance,
                  cfg.exploration_topology_recovery_max_distance})
{
}

void ExplorationRuntimeManager::reset()
{
    latest_goal_ = ExplorationGoal{};
    committed_goal_ = ExplorationGoal{};
    status_ = Status::IDLE;
    phase_ = Phase::IDLE;
    consecutive_temporary_failures_ = 0;
    has_latest_goal_ = false;
    has_committed_goal_ = false;
    has_last_trap_candidate_ = false;
    last_trap_candidate_position_.setZero();
    last_trap_robot_position_.setZero();
    last_trap_frontier_id_ = -1;
    repeated_local_region_count_ = 0;
    local_trap_recovery_request_count_ = 0;
    local_trap_cooldown_until_ = 0.0;
    has_recent_trap_region_ = false;
    recent_trap_position_.setZero();
    recent_trap_frontier_id_ = -1;
    recent_trap_block_until_ = 0.0;
    recovery_query_count_ = 0;
    frontier_recovery_selected_count_ = 0;
    topology_recovery_selected_count_ = 0;
    recovery_unavailable_count_ = 0;
    recovery_blocked_by_recent_trap_count_ = 0;
    navigation_memory_.reset();
    frontier_memory_.reset();
    coverage_grid_.reset();
    topological_memory_.reset();
}

void ExplorationRuntimeManager::onSelectingGoal()
{
    status_ = Status::SELECTING_GOAL;
    phase_ = Phase::SELECT_LOCAL_GOAL;
}

void ExplorationRuntimeManager::onGoalSelected(const ExplorationGoal &goal)
{
    latest_goal_ = goal;
    has_latest_goal_ = goal.valid;
    consecutive_temporary_failures_ = 0;
    status_ = Status::GOAL_SELECTED;
    phase_ = goal.reason.find("recovery") != std::string::npos
                     ? Phase::RECOVERY_ESCAPE
                     : Phase::PLAN_LOCAL_TRAJECTORY;
}

void ExplorationRuntimeManager::onCommitted(const ExplorationGoal &goal)
{
    latest_goal_ = goal;
    committed_goal_ = goal;
    has_latest_goal_ = goal.valid;
    has_committed_goal_ = goal.valid;
    consecutive_temporary_failures_ = 0;
    status_ = Status::ACTIVE_COMMITTED;
    phase_ = Phase::EXECUTE_COMMITTED;
}

void ExplorationRuntimeManager::onKeepCurrentGoal()
{
    status_ = Status::KEEP_CURRENT_GOAL;
    phase_ = Phase::EXECUTE_COMMITTED;
}

void ExplorationRuntimeManager::onTemporaryFailure(const ExplorationGoal &goal)
{
    if (goal.valid) {
        latest_goal_ = goal;
        has_latest_goal_ = true;
    }
    ++consecutive_temporary_failures_;
    status_ = Status::TEMPORARY_FAILURE;
    phase_ = Phase::FAILED;
}

void ExplorationRuntimeManager::onFinished(const ExplorationGoal &goal)
{
    latest_goal_ = goal;
    committed_goal_ = ExplorationGoal{};
    has_latest_goal_ = goal.valid;
    has_committed_goal_ = false;
    status_ = Status::FINISHED;
    phase_ = Phase::FINISHED;
}

bool ExplorationRuntimeManager::latestGoalReusable(const general_utils::Vec3f &robot_pos,
                                                   const double committed_remaining,
                                                   const bool new_task) const
{
    if (new_task || !has_latest_goal_ || !latest_goal_.valid ||
        !robot_pos.allFinite()) {
        return false;
    }

    const ExplorationGoal &active_goal =
            has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_;
    if (!active_goal.position.allFinite()) {
        return false;
    }

    const double reached_distance = std::max(0.0, cfg_.exploration_goal_reached_distance);
    if ((robot_pos - active_goal.position).norm() <= reached_distance) {
        return false;
    }

    const double min_remaining = std::max(0.25, cfg_.replan_forward_dt);
    return committed_remaining > min_remaining;
}

bool ExplorationRuntimeManager::shouldKeepCurrentGoal(const ExplorationGoal &candidate,
                                                      const general_utils::Vec3f &robot_pos,
                                                      const double committed_remaining,
                                                      const bool new_task) const
{
    if (!candidate.valid ||
        !latestGoalReusable(robot_pos, committed_remaining, new_task)) {
        return false;
    }

    const double switch_margin =
            std::max(0.0, cfg_.exploration_goal_switch_min_score_improvement);
    const ExplorationGoal &active_goal =
            has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_;
    return candidate.score >= active_goal.score - switch_margin;
}

bool ExplorationRuntimeManager::shouldReuseLatestGoal(const general_utils::Vec3f &robot_pos,
                                                      const double committed_remaining,
                                                      const bool new_task) const
{
    return latestGoalReusable(robot_pos, committed_remaining, new_task);
}

ExplorationRuntimeManager::SelectionDecision ExplorationRuntimeManager::stabilizeCandidate(
        const ExplorationGoal &candidate,
        const general_utils::Vec3f &robot_pos,
        const double committed_remaining,
        const double stamp,
        const bool new_task)
{
    SelectionDecision out;
    out.ndo = navigation_memory_.diagnose(stamp);
    if (!candidate.valid) {
        out.reject = true;
        out.reason = "candidate_invalid";
        return out;
    }
    frontier_memory_.observe(candidate, stamp);

    ExplorationGoal adjusted_candidate = candidate;
    const double revisit_penalty = coverage_grid_.revisitPenalty(candidate.position, stamp);
    if (revisit_penalty > 0.0) {
        adjusted_candidate.score += std::max(0.0, cfg_.exploration_coverage_revisit_penalty_weight) *
                                    revisit_penalty;
        adjusted_candidate.reason += " coverage_revisit_penalty=" + std::to_string(revisit_penalty);
    }

    std::string trap_reason;
    if (localTrapDetected(adjusted_candidate, robot_pos, stamp, revisit_penalty, trap_reason)) {
        frontier_memory_.markFailed(adjusted_candidate, stamp);
        topological_memory_.recordFailureNear(adjusted_candidate.position, stamp);
        if (nhbpEnabled() && !adjusted_candidate.memory_key.empty()) {
            navigation_memory_.recordFailure(adjusted_candidate.memory_key,
                                             nhbp::FailureReason::NDO_OSCILLATION,
                                             stamp,
                                             cfg_.exploration_nhbp_blacklist_ttl);
        }
        ++local_trap_recovery_request_count_;
        local_trap_cooldown_until_ =
                stamp + std::max(0.0, cfg_.exploration_local_trap_cooldown);
        has_recent_trap_region_ = true;
        recent_trap_position_ = adjusted_candidate.position;
        recent_trap_frontier_id_ = adjusted_candidate.frontier_id;
        recent_trap_block_until_ = local_trap_cooldown_until_;
        out.ready = false;
        out.reject = true;
        out.recovery_requested = true;
        out.allow_candidate_fallback = true;
        out.reason = trap_reason;
        return out;
    }

    const bool current_reusable = latestGoalReusable(robot_pos,
                                                     committed_remaining,
                                                     new_task);
    if (revisit_penalty > 0.0 &&
        current_reusable &&
        (has_committed_goal_ || has_latest_goal_)) {
        out.ready = true;
        out.keep_current = true;
        out.reason = "coverage_recent_revisit_keep_current";
        out.goal = has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_;
        return out;
    }

    if (frontier_memory_.blocked(candidate, stamp)) {
        if (current_reusable && (has_committed_goal_ || has_latest_goal_)) {
            out.ready = true;
            out.keep_current = true;
            out.reason = "frontier_memory_blocked_keep_current";
            out.goal = has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_;
            return out;
        }
        if (!has_committed_goal_) {
            out.ready = true;
            out.keep_current = false;
            out.reason = "frontier_memory_blocked_without_committed_goal_accept";
            out.goal = adjusted_candidate;
            out.goal.reason = adjusted_candidate.reason + " nhbp=" + out.reason;
            return out;
        }
        out.reject = true;
        out.reason = "frontier_memory_blocked";
        return out;
    }

    nhbp::DecisionContext context;
    context.new_task = new_task;
    context.current_reusable = current_reusable;
    context.committed_remaining = committed_remaining;
    context.stamp = stamp;

    const nhbp::StabilizerDecision decision =
            decision_stabilizer_.stabilize(toDecisionCandidate(adjusted_candidate),
                                           toDecisionCandidate(has_committed_goal_ && committed_goal_.valid
                                                               ? committed_goal_
                                                               : latest_goal_),
                                           context,
                                           navigation_memory_);
    out.ndo = decision.ndo;
    out.reason = decision.reason;

    if (decision.action == nhbp::StabilizerAction::KEEP_CURRENT &&
        current_reusable &&
        (has_committed_goal_ && committed_goal_.valid ? committed_goal_.valid : latest_goal_.valid)) {
        out.ready = true;
        out.keep_current = true;
        out.goal = has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_;
        out.goal.reason = "nhbp keep current: " + decision.reason;
        return out;
    }

    if (decision.action == nhbp::StabilizerAction::REJECT_CANDIDATE) {
        out.reject = true;
        return out;
    }

    out.ready = true;
    out.keep_current = false;
    out.goal = adjusted_candidate;
    out.goal.reason = adjusted_candidate.reason + " nhbp=" + decision.reason;
    return out;
}

void ExplorationRuntimeManager::recordDecision(const ExplorationGoal &goal,
                                               const general_utils::Vec3f &robot_pos,
                                               const double stamp)
{
    if (!goal.valid) {
        return;
    }
    coverage_grid_.observePose(robot_pos, stamp);
    frontier_memory_.markCommitted(goal, stamp);
    frontier_memory_.markCoveredNear(robot_pos, stamp);
    topological_memory_.observeTransition(robot_pos, goal.position, stamp);
    if (!nhbpEnabled()) {
        return;
    }
    nhbp::DecisionRecord record;
    record.candidate_id = goal.candidate_id;
    record.frontier_id = goal.frontier_id;
    record.position = robot_pos;
    record.stamp = stamp;
    record.score = goal.score;
    navigation_memory_.recordDecision(record);
}

void ExplorationRuntimeManager::recordFailure(const ExplorationGoal &goal,
                                              const nhbp::FailureReason reason,
                                              const double stamp)
{
    if (!goal.valid) {
        return;
    }
    frontier_memory_.markFailed(goal, stamp);
    topological_memory_.recordFailureNear(goal.position, stamp);
    if (!nhbpEnabled() || goal.memory_key.empty()) {
        return;
    }
    navigation_memory_.recordFailure(goal.memory_key,
                                     reason,
                                     stamp,
                                     cfg_.exploration_nhbp_blacklist_ttl);
}

nhbp::NdoDiagnosis ExplorationRuntimeManager::diagnose(const double stamp) const
{
    return navigation_memory_.diagnose(stamp);
}

bool ExplorationRuntimeManager::hasRecoveryGoal(const general_utils::Vec3f &robot_pos,
                                                const double stamp,
                                                ExplorationGoal &goal)
{
    ++recovery_query_count_;
    if (!cfg_.exploration_nhbp_recovery_enable) {
        goal = ExplorationGoal{};
        ++recovery_unavailable_count_;
        return false;
    }
    if (frontier_memory_.hasRecoverableGoal(
            robot_pos,
            stamp,
            goal,
            [this, stamp](const ExplorationGoal &candidate) {
                if (!candidate.memory_key.empty() &&
                    navigation_memory_.isBlacklisted(candidate.memory_key, stamp)) {
                    return false;
                }
                const bool blocked_by_trap = recoveryBlockedByRecentTrap(candidate, stamp);
                if (blocked_by_trap) {
                    ++recovery_blocked_by_recent_trap_count_;
                }
                return !blocked_by_trap;
            })) {
        ++frontier_recovery_selected_count_;
        return true;
    }

    general_utils::Vec3f recovery_position = general_utils::Vec3f::Zero();
    if (!topological_memory_.findRecoveryPosition(
            robot_pos,
            stamp,
            recovery_position,
            [this, stamp](const general_utils::Vec3f &candidate_position) {
                const bool blocked_by_trap =
                        recoveryPositionBlockedByRecentTrap(candidate_position, -1, stamp);
                if (blocked_by_trap) {
                    ++recovery_blocked_by_recent_trap_count_;
                }
                return !blocked_by_trap;
            })) {
        ++recovery_unavailable_count_;
        return false;
    }

    goal = ExplorationGoal{};
    goal.valid = true;
    goal.position = recovery_position;
    const general_utils::Vec3f diff = recovery_position - robot_pos;
    goal.yaw = std::atan2(diff.y(), diff.x());
    goal.score = diff.norm();
    goal.travel_cost = diff.norm();
    goal.distance_to_robot = diff.norm();
    goal.candidate_id = -1;
    goal.frontier_id = -1;
    goal.memory_key = "topology_recovery";
    goal.reason = "topological_memory_recovery";
    if (recoveryBlockedByRecentTrap(goal, stamp)) {
        ++recovery_blocked_by_recent_trap_count_;
        ++recovery_unavailable_count_;
        goal = ExplorationGoal{};
        return false;
    }
    ++topology_recovery_selected_count_;
    return true;
}

bool ExplorationRuntimeManager::shouldDelayFinish(const double stamp) const
{
    return cfg_.exploration_use_frontier_memory &&
           frontier_memory_.activeCount(stamp) > 0;
}

bool ExplorationRuntimeManager::getLatestGoal(ExplorationGoal &goal) const
{
    if (has_committed_goal_ && committed_goal_.valid) {
        goal = committed_goal_;
        return true;
    }
    goal = latest_goal_;
    return has_latest_goal_ && latest_goal_.valid;
}

std::string ExplorationRuntimeManager::diagnosticSummary(const double stamp) const
{
    const nhbp::NdoDiagnosis ndo = navigation_memory_.diagnose(stamp);
    std::ostringstream oss;
    oss << "status=" << toString(status_)
        << ";phase=" << toString(phase_)
        << ";temporary_failures=" << consecutive_temporary_failures_
        << ";has_latest=" << static_cast<int>(has_latest_goal_)
        << ";has_committed=" << static_cast<int>(has_committed_goal_)
        << ";frontier_active=" << frontier_memory_.activeCount(stamp)
        << ";frontier_failed=" << frontier_memory_.failedCount(stamp)
        << ";frontier_covered=" << frontier_memory_.coveredCount()
        << ";coverage_cells=" << coverage_grid_.visitedCellCount()
        << ";coverage_visits=" << coverage_grid_.totalVisitCount()
        << ";topology_nodes=" << topological_memory_.activeNodeCount(stamp)
        << ";topology_blocked=" << topological_memory_.blockedNodeCount(stamp)
        << ";topology_edges=" << topological_memory_.edgeCount()
        << ";local_trap_recovery_requests=" << local_trap_recovery_request_count_
        << ";recent_trap_active="
        << static_cast<int>(has_recent_trap_region_ && recent_trap_block_until_ > stamp)
        << ";recovery_queries=" << recovery_query_count_
        << ";frontier_recovery_selected=" << frontier_recovery_selected_count_
        << ";topology_recovery_selected=" << topology_recovery_selected_count_
        << ";recovery_unavailable=" << recovery_unavailable_count_
        << ";recovery_blocked_by_recent_trap=" << recovery_blocked_by_recent_trap_count_
        << ";ndo=" << nhbp::toString(ndo.state)
        << ";ndo_reason=" << ndo.reason;
    return oss.str();
}

ExplorationRuntimeManager::Status ExplorationRuntimeManager::status() const
{
    return status_;
}

ExplorationRuntimeManager::Phase ExplorationRuntimeManager::phase() const
{
    return phase_;
}

int ExplorationRuntimeManager::consecutiveTemporaryFailures() const
{
    return consecutive_temporary_failures_;
}

bool ExplorationRuntimeManager::hasCommittedGoal() const
{
    return has_committed_goal_;
}

bool ExplorationRuntimeManager::localTrapDetected(const ExplorationGoal &candidate,
                                                  const general_utils::Vec3f &robot_pos,
                                                  const double stamp,
                                                  const double revisit_penalty,
                                                  std::string &reason)
{
    reason.clear();
    if (!cfg_.exploration_local_trap_detection_enable ||
        !candidate.valid ||
        !candidate.position.allFinite() ||
        !robot_pos.allFinite()) {
        return false;
    }

    const double same_region_radius =
            std::max(0.1, cfg_.exploration_local_trap_same_region_radius);
    const bool same_region =
            has_last_trap_candidate_ &&
            (candidate.position - last_trap_candidate_position_).norm() <= same_region_radius;
    const bool same_frontier =
            candidate.frontier_id >= 0 &&
            candidate.frontier_id == last_trap_frontier_id_;

    if (same_region || same_frontier) {
        ++repeated_local_region_count_;
    } else {
        repeated_local_region_count_ = 1;
    }

    const double robot_motion =
            has_last_trap_candidate_
                    ? (robot_pos - last_trap_robot_position_).norm()
                    : std::numeric_limits<double>::infinity();
    last_trap_candidate_position_ = candidate.position;
    last_trap_robot_position_ = robot_pos;
    last_trap_frontier_id_ = candidate.frontier_id;
    has_last_trap_candidate_ = true;

    if (stamp < local_trap_cooldown_until_) {
        return false;
    }

    const int repeat_threshold =
            std::max(2, cfg_.exploration_local_trap_repeat_threshold);
    const int cluster_threshold =
            std::max(1, cfg_.exploration_local_trap_cluster_threshold);
    const int astar_threshold =
            std::max(1, cfg_.exploration_local_trap_astar_threshold);
    const double min_information_gain =
            std::max(0.0, cfg_.exploration_local_trap_min_information_gain);
    const bool repeated = repeated_local_region_count_ >= repeat_threshold;
    const int cluster_diversity =
            candidate.raw_cluster_count > 0 ? candidate.raw_cluster_count : candidate.cluster_count;
    const bool low_frontier_diversity =
            cluster_diversity > 0 &&
            cluster_diversity <= cluster_threshold;
    const bool high_astar_pressure =
            candidate.astar_check_count >= astar_threshold;
    const bool high_local_information_gain =
            min_information_gain <= 0.0 ||
            candidate.information_gain >= min_information_gain;
    const bool revisiting =
            revisit_penalty > 0.0 ||
            robot_motion <= std::max(0.1, cfg_.exploration_nhbp_min_progress_distance);

    if (!(repeated && low_frontier_diversity && high_astar_pressure &&
          high_local_information_gain && revisiting)) {
        return false;
    }

    std::ostringstream oss;
    oss << "local_trap_escape_requested"
        << ":repeat=" << repeated_local_region_count_
        << ",clusters=" << cluster_diversity
        << ",astar_checks=" << candidate.astar_check_count
        << ",reachable=" << candidate.reachable_candidate_count
        << ",frontier=" << candidate.frontier_id
        << ",info=" << candidate.information_gain
        << ",revisit=" << revisit_penalty
        << ",robot_motion=" << robot_motion;
    reason = oss.str();
    return true;
}

bool ExplorationRuntimeManager::recoveryBlockedByRecentTrap(const ExplorationGoal &goal,
                                                            const double stamp) const
{
    if (!goal.valid || !goal.position.allFinite()) {
        return false;
    }
    return recoveryPositionBlockedByRecentTrap(goal.position, goal.frontier_id, stamp);
}

bool ExplorationRuntimeManager::recoveryPositionBlockedByRecentTrap(
        const general_utils::Vec3f &position,
        const int frontier_id,
        const double stamp) const
{
    if (!has_recent_trap_region_ ||
        recent_trap_block_until_ <= stamp ||
        !position.allFinite() ||
        !recent_trap_position_.allFinite()) {
        return false;
    }

    const double same_region_radius =
            std::max(0.1, cfg_.exploration_local_trap_same_region_radius);
    const bool same_region =
            (position - recent_trap_position_).norm() <= same_region_radius;
    const bool same_frontier =
            frontier_id >= 0 &&
            recent_trap_frontier_id_ >= 0 &&
            frontier_id == recent_trap_frontier_id_;
    return same_region || same_frontier;
}

const char *ExplorationRuntimeManager::toString(const Status status)
{
    switch (status) {
        case Status::IDLE:
            return "IDLE";
        case Status::SELECTING_GOAL:
            return "SELECTING_GOAL";
        case Status::GOAL_SELECTED:
            return "GOAL_SELECTED";
        case Status::ACTIVE_COMMITTED:
            return "ACTIVE_COMMITTED";
        case Status::KEEP_CURRENT_GOAL:
            return "KEEP_CURRENT_GOAL";
        case Status::TEMPORARY_FAILURE:
            return "TEMPORARY_FAILURE";
        case Status::FINISHED:
            return "FINISHED";
    }
    return "UNKNOWN";
}

const char *ExplorationRuntimeManager::toString(const Phase phase)
{
    switch (phase) {
        case Phase::IDLE:
            return "IDLE";
        case Phase::WAIT_MAP:
            return "WAIT_MAP";
        case Phase::UPDATE_BELIEF:
            return "UPDATE_BELIEF";
        case Phase::SELECT_TOUR:
            return "SELECT_TOUR";
        case Phase::SELECT_LOCAL_GOAL:
            return "SELECT_LOCAL_GOAL";
        case Phase::PLAN_LOCAL_TRAJECTORY:
            return "PLAN_LOCAL_TRAJECTORY";
        case Phase::EXECUTE_COMMITTED:
            return "EXECUTE_COMMITTED";
        case Phase::RECOVERY_ESCAPE:
            return "RECOVERY_ESCAPE";
        case Phase::FINISHED:
            return "FINISHED";
        case Phase::FAILED:
            return "FAILED";
    }
    return "UNKNOWN";
}

nhbp::DecisionCandidate ExplorationRuntimeManager::toDecisionCandidate(
        const ExplorationGoal &goal) const
{
    nhbp::DecisionCandidate candidate;
    candidate.valid = goal.valid;
    candidate.candidate_id = goal.candidate_id;
    candidate.frontier_id = goal.frontier_id;
    candidate.key = goal.memory_key;
    candidate.position = goal.position;
    candidate.score = goal.score;
    return candidate;
}

bool ExplorationRuntimeManager::nhbpEnabled() const
{
    return cfg_.exploration_nhbp_enable;
}

} // namespace general_planner
