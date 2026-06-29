#include "general_core/exploration/exploration_runtime_manager.hpp"

#include <algorithm>
#include <cmath>
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
                                                ExplorationGoal &goal) const
{
    if (!cfg_.exploration_nhbp_recovery_enable) {
        goal = ExplorationGoal{};
        return false;
    }
    if (frontier_memory_.hasRecoverableGoal(
            robot_pos,
            stamp,
            goal,
            [this, stamp](const ExplorationGoal &candidate) {
                return candidate.memory_key.empty() ||
                       !navigation_memory_.isBlacklisted(candidate.memory_key, stamp);
            })) {
        return true;
    }

    general_utils::Vec3f recovery_position = general_utils::Vec3f::Zero();
    if (!topological_memory_.findRecoveryPosition(robot_pos, stamp, recovery_position)) {
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
