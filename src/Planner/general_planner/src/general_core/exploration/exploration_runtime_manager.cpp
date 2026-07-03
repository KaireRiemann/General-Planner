#include "general_core/exploration/exploration_runtime_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

namespace general_planner {
namespace {
general_utils::Vec3f firstGuideDirection(const ExplorationGoal &goal)
{
    if (goal.guide_path.size() < 2) {
        return general_utils::Vec3f::Zero();
    }
    general_utils::Vec3f direction = goal.guide_path[1] - goal.guide_path[0];
    direction.z() = 0.0;
    const double norm = direction.norm();
    if (norm <= 1.0e-6) {
        return general_utils::Vec3f::Zero();
    }
    direction /= norm;
    return direction;
}

bool goalHasRecoveryIntentText(const ExplorationGoal &goal)
{
    return goal.reason.find("recovery") != std::string::npos ||
           goal.reason.find("bootstrap") != std::string::npos ||
           goal.memory_key.find("recovery") != std::string::npos ||
           goal.memory_key.find("bootstrap") != std::string::npos;
}

std::string stripLockedRecoveryReusePrefix(std::string reason)
{
    const std::string prefix = "locked_recovery_goal_reuse:";
    while (reason.rfind(prefix, 0) == 0) {
        reason.erase(0, prefix.size());
    }
    return reason;
}
} // namespace

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
                  cfg.exploration_coverage_grid_max_cells,
                  cfg.exploration_coverage_information_gain_alpha,
                  cfg.exploration_coverage_covered_visit_threshold,
                  cfg.exploration_coverage_stale_time}),
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
    locked_recovery_goal_reused_count_ = 0;
    recovery_unavailable_count_ = 0;
    recovery_blocked_by_recent_trap_count_ = 0;
    recovery_lock_active_ = false;
    recovery_state_ = RecoveryState{};
    recovery_lock_trigger_goal_ = ExplorationGoal{};
    active_recovery_goal_ = ExplorationGoal{};
    has_active_recovery_goal_ = false;
    recovery_lock_reason_.clear();
    recovery_lock_started_stamp_ = 0.0;
    recovery_lock_until_ = 0.0;
    recovery_lock_request_count_ = 0;
    recovery_lock_release_count_ = 0;
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
    phase_ = isRecoveryGoal(goal) ? Phase::RECOVERY_ESCAPE
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
    phase_ = isRecoveryGoal(goal) ? Phase::RECOVERY_ESCAPE
                                  : Phase::EXECUTE_COMMITTED;
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

    const double min_remaining =
            std::max({0.25, cfg_.replan_forward_dt, cfg_.exploration_keep_old_min_remaining});
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
    const ExplorationGoal current_goal =
            has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_;
    if (!candidate.valid) {
        out.reject = true;
        out.reason = "candidate_invalid";
        recordTrace(nhbp::DecisionTraceAction::REJECT_CANDIDATE,
                    candidate,
                    current_goal,
                    robot_pos,
                    stamp,
                    committed_remaining,
                    out.ndo,
                    out.reason);
        return out;
    }
    updateRecoveryLock(robot_pos, stamp);
    recordTrace(nhbp::DecisionTraceAction::CANDIDATE_EVALUATED,
                candidate,
                current_goal,
                robot_pos,
                stamp,
                committed_remaining,
                out.ndo,
                "candidate_evaluated");
    if (recoveryLockActive(stamp) && !isRecoveryGoal(candidate)) {
        out.reject = true;
        out.recovery_requested = true;
        out.allow_candidate_fallback = false;
        out.reason = "recovery_lock_active:" + recovery_lock_reason_;
        recordTrace(nhbp::DecisionTraceAction::RECOVERY_REQUESTED,
                    candidate,
                    current_goal,
                    robot_pos,
                    stamp,
                    committed_remaining,
                    out.ndo,
                    out.reason);
        return out;
    }
    if (out.ndo.state == nhbp::NdoState::DEADLOCKED &&
        cfg_.exploration_nhbp_recovery_enable &&
        !isRecoveryGoal(candidate)) {
        out.reject = true;
        out.recovery_requested = true;
        out.allow_candidate_fallback = false;
        out.reason = "ndo_deadlocked_recovery_required:" + out.ndo.reason;
        enterRecoveryLock(candidate, robot_pos, stamp, out.reason);
        recordTrace(nhbp::DecisionTraceAction::RECOVERY_REQUESTED,
                    candidate,
                    current_goal,
                    robot_pos,
                    stamp,
                    committed_remaining,
                    out.ndo,
                    out.reason);
        return out;
    }
    frontier_memory_.observe(candidate, stamp);
    coverage_grid_.observeFrontierEvidence(candidate.position,
                                           stamp,
                                           candidate.visible_frontier_cell_count,
                                           candidate.visible_frontier_cell_count,
                                           candidate.information_gain);

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
        coverage_grid_.markNoProgressBasin(
                adjusted_candidate.position,
                stamp,
                std::max(cfg_.exploration_local_trap_same_region_radius,
                         cfg_.exploration_coverage_revisit_radius));
        const std::string failure_key = normalizedIdentity(adjusted_candidate).blacklistKey();
        if (nhbpEnabled() && !failure_key.empty()) {
            navigation_memory_.recordFailure(failure_key,
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
        enterRecoveryLock(adjusted_candidate, robot_pos, stamp, trap_reason);
        out.ready = false;
        out.reject = true;
        out.recovery_requested = true;
        out.allow_candidate_fallback = false;
        out.reason = trap_reason;
        recordTrace(nhbp::DecisionTraceAction::RECOVERY_REQUESTED,
                    adjusted_candidate,
                    current_goal,
                    robot_pos,
                    stamp,
                    committed_remaining,
                    out.ndo,
                    out.reason);
        return out;
    }

    const bool current_reusable = latestGoalReusable(robot_pos,
                                                     committed_remaining,
                                                     new_task);
    const double revisit_escape_threshold =
            std::max(6.0, 3.0 / std::max(0.1, cfg_.exploration_coverage_revisit_penalty_weight));
    if (revisit_penalty >= revisit_escape_threshold &&
        current_reusable &&
        (has_committed_goal_ || has_latest_goal_)) {
        frontier_memory_.markFailed(adjusted_candidate, stamp);
        topological_memory_.recordFailureNear(adjusted_candidate.position, stamp);
        coverage_grid_.markNoProgressBasin(
                adjusted_candidate.position,
                stamp,
                std::max(cfg_.exploration_local_trap_same_region_radius,
                         cfg_.exploration_coverage_revisit_radius));
        const std::string failure_key = normalizedIdentity(adjusted_candidate).blacklistKey();
        if (nhbpEnabled() && !failure_key.empty()) {
            navigation_memory_.recordFailure(failure_key,
                                             nhbp::FailureReason::NDO_OSCILLATION,
                                             stamp,
                                             cfg_.exploration_nhbp_blacklist_ttl);
        }
        has_recent_trap_region_ = true;
        recent_trap_position_ = adjusted_candidate.position;
        recent_trap_frontier_id_ = adjusted_candidate.frontier_id;
        recent_trap_block_until_ =
                stamp + std::max(cfg_.exploration_local_trap_cooldown,
                                 cfg_.exploration_coverage_revisit_time_window);
        if (isRecoveryGoal(current_goal)) {
            out.ready = true;
            out.keep_current = true;
            out.goal = current_goal;
            out.goal.reason = "coverage_revisit_escape_keep_current_recovery";
            out.reason = "coverage_revisit_escape_keep_current_recovery:penalty=" +
                         std::to_string(revisit_penalty);
            recordTrace(nhbp::DecisionTraceAction::KEEP_CURRENT,
                        adjusted_candidate,
                        out.goal,
                        robot_pos,
                        stamp,
                        committed_remaining,
                        out.ndo,
                        out.reason);
            return out;
        }
        out.ready = false;
        out.reject = true;
        out.recovery_requested = true;
        out.allow_candidate_fallback = true;
        out.reason = "coverage_revisit_escape_requested:penalty=" +
                     std::to_string(revisit_penalty);
        recordTrace(nhbp::DecisionTraceAction::RECOVERY_REQUESTED,
                    adjusted_candidate,
                    current_goal,
                    robot_pos,
                    stamp,
                    committed_remaining,
                    out.ndo,
                    out.reason);
        return out;
    }

    if (frontier_memory_.blocked(candidate, stamp)) {
        if (current_reusable && (has_committed_goal_ || has_latest_goal_)) {
            out.ready = true;
            out.keep_current = true;
            out.reason = "frontier_memory_blocked_keep_current";
            out.goal = has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_;
            recordTrace(nhbp::DecisionTraceAction::KEEP_CURRENT,
                        adjusted_candidate,
                        out.goal,
                        robot_pos,
                        stamp,
                        committed_remaining,
                        out.ndo,
                        out.reason);
            return out;
        }
        if (!has_committed_goal_) {
            out.ready = true;
            out.keep_current = false;
            out.reason = "frontier_memory_blocked_without_committed_goal_accept";
            out.goal = adjusted_candidate;
            out.goal.reason = adjusted_candidate.reason + " nhbp=" + out.reason;
            recordTrace(nhbp::DecisionTraceAction::ACCEPT_CANDIDATE,
                        out.goal,
                        current_goal,
                        robot_pos,
                        stamp,
                        committed_remaining,
                        out.ndo,
                        out.reason);
            return out;
        }
        out.reject = true;
        out.reason = "frontier_memory_blocked";
        recordTrace(nhbp::DecisionTraceAction::REJECT_CANDIDATE,
                    adjusted_candidate,
                    current_goal,
                    robot_pos,
                    stamp,
                    committed_remaining,
                    out.ndo,
                    out.reason);
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
        recordTrace(nhbp::DecisionTraceAction::KEEP_CURRENT,
                    adjusted_candidate,
                    out.goal,
                    robot_pos,
                    stamp,
                    committed_remaining,
                    out.ndo,
                    out.reason);
        return out;
    }

    if (decision.action == nhbp::StabilizerAction::REJECT_CANDIDATE) {
        out.reject = true;
        recordTrace(nhbp::DecisionTraceAction::REJECT_CANDIDATE,
                    adjusted_candidate,
                    current_goal,
                    robot_pos,
                    stamp,
                    committed_remaining,
                    out.ndo,
                    out.reason);
        return out;
    }

    out.ready = true;
    out.keep_current = false;
    out.goal = adjusted_candidate;
    out.goal.reason = adjusted_candidate.reason + " nhbp=" + decision.reason;
    recordTrace(nhbp::DecisionTraceAction::ACCEPT_CANDIDATE,
                out.goal,
                current_goal,
                robot_pos,
                stamp,
                committed_remaining,
                out.ndo,
                out.reason);
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
    coverage_grid_.observeFrontierEvidence(goal.position,
                                           stamp,
                                           goal.visible_frontier_cell_count,
                                           goal.visible_frontier_cell_count,
                                           goal.information_gain);
    coverage_grid_.markCoveredNear(robot_pos,
                                   stamp,
                                   std::max(cfg_.exploration_coverage_revisit_radius,
                                            cfg_.exploration_frontier_memory_covered_radius));
    frontier_memory_.markCommitted(goal, stamp);
    frontier_memory_.markCoveredNear(robot_pos, stamp);
    topological_memory_.observeTransition(robot_pos, goal.position, stamp);
    if (!nhbpEnabled()) {
        return;
    }
    nhbp::DecisionRecord record;
    record.candidate_id = goal.candidate_id;
    record.frontier_id = goal.frontier_id;
    record.identity = normalizedIdentity(goal);
    record.position = robot_pos;
    record.robot_position = robot_pos;
    record.robot_velocity = general_utils::Vec3f::Zero();
    record.selected_goal = goal.position;
    record.committed_goal =
            has_committed_goal_ && committed_goal_.valid ? committed_goal_.position : goal.position;
    record.committed_end = goal.position;
    record.guide_first_direction = firstGuideDirection(goal);
    record.target_position = goal.position;
    record.stamp = stamp;
    record.score = goal.score;
    record.raw_score = goal.score - goal.history_score_delta;
    record.final_score = goal.score;
    record.information_gain = goal.information_gain;
    record.travel_cost = goal.travel_cost;
    record.goal_distance = (goal.position - robot_pos).norm();
    record.committed = true;
    record.recovery = isRecoveryGoal(goal);
    record.reason = goal.reason;
    navigation_memory_.recordDecision(record);
    if (record.recovery) {
        bindRecoveryGoal(goal, robot_pos, stamp);
    }
    recordTrace(nhbp::DecisionTraceAction::COMMIT,
                goal,
                committed_goal_,
                robot_pos,
                stamp,
                0.0,
                navigation_memory_.diagnose(stamp),
                goal.reason);
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
    const nhbp::NavIdentity identity = normalizedIdentity(goal);
    const std::string key =
            identity.blacklistKey().empty() ? goal.memory_key : identity.blacklistKey();
    const auto sameFailedGoal = [this, &goal, &key](const ExplorationGoal &stored_goal) {
        if (!stored_goal.valid) {
            return false;
        }
        const nhbp::NavIdentity stored_identity = normalizedIdentity(stored_goal);
        const std::string stored_key = stored_identity.blacklistKey().empty()
                                               ? stored_goal.memory_key
                                               : stored_identity.blacklistKey();
        if (!key.empty() && !stored_key.empty() && key == stored_key) {
            return true;
        }
        return goal.position.allFinite() &&
               stored_goal.position.allFinite() &&
               (goal.position - stored_goal.position).norm() <=
                       std::max({cfg_.exploration_goal_reached_distance,
                                 cfg_.exploration_coverage_revisit_radius,
                                 0.5});
    };
    if (sameFailedGoal(latest_goal_)) {
        latest_goal_ = ExplorationGoal{};
        has_latest_goal_ = false;
    }
    if (sameFailedGoal(committed_goal_)) {
        committed_goal_ = ExplorationGoal{};
        has_committed_goal_ = false;
    }
    const nhbp::NavIdentity active_identity =
            has_active_recovery_goal_ ? normalizedIdentity(active_recovery_goal_)
                                      : nhbp::NavIdentity{};
    const std::string active_key = active_identity.blacklistKey();
    const bool same_active_recovery_key =
            has_active_recovery_goal_ && !key.empty() && !active_key.empty() &&
            key == active_key;
    const bool same_active_recovery_region =
            has_active_recovery_goal_ &&
            goal.position.allFinite() &&
            active_recovery_goal_.position.allFinite() &&
            (goal.position - active_recovery_goal_.position).norm() <=
                    std::max({cfg_.exploration_goal_reached_distance,
                              cfg_.exploration_coverage_revisit_radius,
                              0.5});
    if (same_active_recovery_key || same_active_recovery_region) {
        releaseRecoveryLock(stamp, "released_by_recovery_goal_failure", false);
    }
    if (!nhbpEnabled() || key.empty()) {
        return;
    }
    navigation_memory_.recordFailure(key,
                                     reason,
                                     stamp,
                                     cfg_.exploration_nhbp_blacklist_ttl);
    recordTrace(nhbp::DecisionTraceAction::FAILURE,
                goal,
                committed_goal_,
                goal.position,
                stamp,
                0.0,
                navigation_memory_.diagnose(stamp),
                nhbp::toString(reason));
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
    updateRecoveryLock(robot_pos, stamp);
    if (!cfg_.exploration_nhbp_recovery_enable) {
        goal = ExplorationGoal{};
        ++recovery_unavailable_count_;
        return false;
    }
    if (lockedRecoveryGoalReusable(robot_pos, stamp)) {
        goal = active_recovery_goal_;
        goal.identity = normalizedIdentity(goal);
        goal.identity.recovery_intent = true;
        goal.identity.intent_mode = "recovery";
        goal.reason = "locked_recovery_goal_reuse:" +
                      stripLockedRecoveryReusePrefix(recovery_state_.reason);
        recordTrace(nhbp::DecisionTraceAction::RECOVERY_SELECTED,
                    goal,
                    has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_,
                    robot_pos,
                    stamp,
                    0.0,
                    navigation_memory_.diagnose(stamp),
                    goal.reason);
        ++locked_recovery_goal_reused_count_;
        return true;
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
                const double revisit_recovery_block_threshold =
                        std::max(6.0,
                                 3.0 / std::max(0.1,
                                                cfg_.exploration_coverage_revisit_penalty_weight));
                if (coverage_grid_.revisitPenalty(candidate.position, stamp) >=
                    revisit_recovery_block_threshold) {
                    return false;
                }
                const bool blocked_by_trap = recoveryBlockedByRecentTrap(candidate, stamp);
                if (blocked_by_trap) {
                    ++recovery_blocked_by_recent_trap_count_;
                }
                return !blocked_by_trap;
            })) {
        goal.identity = normalizedIdentity(goal);
        goal.identity.recovery_intent = true;
        goal.identity.intent_mode = "recovery";
        if (goal.identity.goal_key.empty()) {
            goal.identity.goal_key = goal.identity.blacklistKey();
        }
        goal.reason = "frontier_memory_recovery";
        recordTrace(nhbp::DecisionTraceAction::RECOVERY_SELECTED,
                    goal,
                    has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_,
                    robot_pos,
                    stamp,
                    0.0,
                    navigation_memory_.diagnose(stamp),
                    goal.reason);
        bindRecoveryGoal(goal, robot_pos, stamp);
        ++frontier_recovery_selected_count_;
        return true;
    }

    nhbp::TopoPath recovery_path;
    general_utils::Vec3f recovery_position = general_utils::Vec3f::Zero();
    const auto recovery_accept = [this, stamp](const general_utils::Vec3f &candidate_position) {
        const double revisit_recovery_block_threshold =
                std::max(6.0,
                         3.0 / std::max(0.1,
                                        cfg_.exploration_coverage_revisit_penalty_weight));
        if (coverage_grid_.revisitPenalty(candidate_position, stamp) >=
            revisit_recovery_block_threshold) {
            return false;
        }
        const bool blocked_by_trap =
                recoveryPositionBlockedByRecentTrap(candidate_position, -1, stamp);
        if (blocked_by_trap) {
            ++recovery_blocked_by_recent_trap_count_;
        }
        return !blocked_by_trap;
    };
    const bool has_topology_path =
            topological_memory_.findRecoveryPath(robot_pos,
                                                 stamp,
                                                 recovery_path,
                                                 recovery_accept);
    if (has_topology_path && recovery_path.valid && !recovery_path.positions.empty()) {
        recovery_position = recovery_path.positions.back();
    } else if (!topological_memory_.findRecoveryPosition(
            robot_pos,
            stamp,
            recovery_position,
            recovery_accept)) {
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
    if (has_topology_path && recovery_path.valid) {
        goal.guide_path = recovery_path.positions;
    }
    goal.candidate_id = -1;
    goal.frontier_id = -1;
    goal.memory_key = "topology_recovery";
    goal.identity.intent_mode = "recovery";
    goal.identity.recovery_intent = true;
    goal.identity.candidate_key =
            nhbp::quantizedPositionKey(recovery_position,
                                       std::max(0.25, cfg_.exploration_coverage_grid_resolution),
                                       "topology_recovery");
    goal.identity.goal_key = goal.identity.candidate_key;
    goal.identity.guide_path_key =
            nhbp::makeGuidePathKey(goal.guide_path, cfg_.exploration_coverage_grid_resolution);
    goal.reason = "topological_memory_recovery";
    if (has_topology_path && recovery_path.valid) {
        goal.reason = recovery_path.reason;
    }
    if (!goal.identity.blacklistKey().empty() &&
        navigation_memory_.isBlacklisted(goal.identity.blacklistKey(), stamp)) {
        ++recovery_unavailable_count_;
        goal = ExplorationGoal{};
        return false;
    }
    if (recoveryBlockedByRecentTrap(goal, stamp)) {
        ++recovery_blocked_by_recent_trap_count_;
        ++recovery_unavailable_count_;
        goal = ExplorationGoal{};
        return false;
    }
    recordTrace(nhbp::DecisionTraceAction::RECOVERY_SELECTED,
                goal,
                has_committed_goal_ && committed_goal_.valid ? committed_goal_ : latest_goal_,
                robot_pos,
                stamp,
                0.0,
                navigation_memory_.diagnose(stamp),
                goal.reason);
    bindRecoveryGoal(goal, robot_pos, stamp);
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
    const nhbp::NavIdentity latest_identity =
            has_latest_goal_ ? normalizedIdentity(latest_goal_) : nhbp::NavIdentity{};
    const nhbp::NavIdentity committed_identity =
            has_committed_goal_ ? normalizedIdentity(committed_goal_) : nhbp::NavIdentity{};
    std::ostringstream oss;
    oss << "status=" << toString(status_)
        << ";phase=" << toString(phase_)
        << ";temporary_failures=" << consecutive_temporary_failures_
        << ";has_latest=" << static_cast<int>(has_latest_goal_)
        << ";has_committed=" << static_cast<int>(has_committed_goal_)
        << ";latest_key=" << latest_identity.canonicalKey()
        << ";committed_key=" << committed_identity.canonicalKey()
        << ";frontier_active=" << frontier_memory_.activeCount(stamp)
        << ";frontier_failed=" << frontier_memory_.failedCount(stamp)
        << ";frontier_covered=" << frontier_memory_.coveredCount()
        << ";coverage_cells=" << coverage_grid_.visitedCellCount()
        << ";coverage_visits=" << coverage_grid_.totalVisitCount()
        << ";coverage_covered=" << coverage_grid_.coveredCellCount()
        << ";coverage_no_progress_basins=" << coverage_grid_.noProgressBasinCount(stamp)
        << ";coverage_recent_gain=" << coverage_grid_.recentInformationGain(stamp)
        << ";topology_nodes=" << topological_memory_.activeNodeCount(stamp)
        << ";topology_blocked=" << topological_memory_.blockedNodeCount(stamp)
        << ";topology_edges=" << topological_memory_.edgeCount()
        << ";local_trap_recovery_requests=" << local_trap_recovery_request_count_
        << ";recent_trap_active="
        << static_cast<int>(has_recent_trap_region_ && recent_trap_block_until_ > stamp)
        << ";recovery_queries=" << recovery_query_count_
        << ";frontier_recovery_selected=" << frontier_recovery_selected_count_
        << ";topology_recovery_selected=" << topology_recovery_selected_count_
        << ";locked_recovery_goal_reused=" << locked_recovery_goal_reused_count_
        << ";recovery_unavailable=" << recovery_unavailable_count_
        << ";recovery_blocked_by_recent_trap=" << recovery_blocked_by_recent_trap_count_
        << ";recovery_lock_active=" << static_cast<int>(recoveryLockActive(stamp))
        << ";recovery_lock_reason=" << recovery_lock_reason_
        << ";recovery_state_active=" << static_cast<int>(recovery_state_.active)
        << ";recovery_state_reason=" << recovery_state_.reason
        << ";recovery_state_exit=" << recovery_state_.exit_reason
        << ";recovery_state_goal_key=" << recovery_state_.recovery_id.canonicalKey()
        << ";recovery_state_elapsed="
        << (recovery_state_.active ? std::max(0.0, stamp - recovery_state_.start_stamp) : 0.0)
        << ";recovery_state_distance="
        << (recovery_state_.active && recovery_state_.start_pos.allFinite()
            ? (recovery_state_.goal_pos - recovery_state_.start_pos).norm()
            : 0.0)
        << ";recovery_lock_requests=" << recovery_lock_request_count_
        << ";recovery_lock_releases=" << recovery_lock_release_count_
        << ";decision_traces=" << navigation_memory_.traceHistory().size()
        << ";ndo=" << nhbp::toString(ndo.state)
        << ";ndo_reason=" << ndo.reason
        << ";ndo_goal_switches=" << ndo.goal_switch_count
        << ";ndo_frontier_switches=" << ndo.frontier_switch_count
        << ";ndo_guide_switches=" << ndo.guide_switch_count
        << ";ndo_recovery_requests=" << ndo.recovery_request_count
        << ";ndo_travel=" << ndo.metrics.travel_distance
        << ";ndo_net=" << ndo.metrics.net_displacement
        << ";ndo_goal_progress=" << ndo.metrics.goal_progress
        << ";ndo_progress_ratio=" << ndo.metrics.progress_ratio
        << ";ndo_branch_switches=" << ndo.metrics.branch_switch_count
        << ";ndo_revisits=" << ndo.metrics.revisit_count
        << ";ndo_commits=" << ndo.metrics.commit_count
        << ";ndo_failures=" << ndo.metrics.failure_count
        << ";ndo_keeps=" << ndo.metrics.keep_current_count;
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
    candidate.identity = normalizedIdentity(goal);
    candidate.key = candidate.identity.blacklistKey().empty()
                            ? goal.memory_key
                            : candidate.identity.blacklistKey();
    candidate.position = goal.position;
    candidate.score = goal.score;
    return candidate;
}

bool ExplorationRuntimeManager::isRecoveryGoal(const ExplorationGoal &goal) const
{
    return goal.valid &&
           (goal.identity.recovery_intent ||
            goalHasRecoveryIntentText(goal));
}

nhbp::NavIdentity ExplorationRuntimeManager::normalizedIdentity(const ExplorationGoal &goal) const
{
    nhbp::NavIdentity identity = goal.identity;
    if (!goal.valid) {
        return identity;
    }
    if (identity.candidate_id < 0) {
        identity.candidate_id = goal.candidate_id;
    }
    if (identity.frontier_id < 0) {
        identity.frontier_id = goal.frontier_id;
    }
    if (identity.candidate_key.empty() && !goal.memory_key.empty()) {
        identity.candidate_key = "candidate:" + goal.memory_key;
    }
    if (identity.frontier_key.empty() && goal.frontier_id >= 0) {
        identity.frontier_key = "frontier:" + std::to_string(goal.frontier_id);
    }
    if (identity.goal_key.empty()) {
        identity.goal_key = identity.candidate_key;
    }
    if (identity.goal_key.empty() && goal.position.allFinite()) {
        identity.goal_key =
                nhbp::quantizedPositionKey(goal.position,
                                           std::max(0.25, cfg_.exploration_coverage_grid_resolution),
                                           "goal");
    }
    if (identity.guide_path_key.empty() && !goal.guide_path.empty()) {
        identity.guide_path_key =
                nhbp::makeGuidePathKey(goal.guide_path,
                                       std::max(0.25, cfg_.exploration_coverage_grid_resolution));
    }
    if (identity.recovery_intent || goalHasRecoveryIntentText(goal)) {
        identity.recovery_intent = true;
        identity.intent_mode = "recovery";
    }
    return identity;
}

void ExplorationRuntimeManager::recordTrace(const nhbp::DecisionTraceAction action,
                                            const ExplorationGoal &goal,
                                            const ExplorationGoal &previous_goal,
                                            const general_utils::Vec3f &robot_pos,
                                            const double stamp,
                                            const double committed_remaining,
                                            const nhbp::NdoDiagnosis &ndo,
                                            const std::string &reason)
{
    if (!nhbpEnabled()) {
        return;
    }
    nhbp::DecisionTrace trace;
    trace.action = action;
    trace.identity = normalizedIdentity(goal);
    trace.previous_identity = normalizedIdentity(previous_goal);
    trace.task = "exploration";
    trace.phase = toString(phase_);
    trace.source = reason;
    trace.robot_position = robot_pos;
    trace.target_position = goal.position;
    trace.stamp = stamp;
    trace.score = goal.score;
    trace.raw_score = goal.score - goal.history_score_delta;
    trace.final_score = goal.score;
    trace.memory_delta = goal.history_score_delta;
    trace.stability_margin = cfg_.exploration_nhbp_switch_margin;
    trace.information_gain = goal.information_gain;
    trace.travel_cost = goal.travel_cost;
    trace.committed_remaining = committed_remaining;
    trace.ndo_state = ndo.state;
    trace.keep_current = action == nhbp::DecisionTraceAction::KEEP_CURRENT;
    trace.recovery = isRecoveryGoal(goal) ||
                     action == nhbp::DecisionTraceAction::RECOVERY_REQUESTED ||
                     action == nhbp::DecisionTraceAction::RECOVERY_SELECTED;
    trace.committed = action == nhbp::DecisionTraceAction::COMMIT;
    trace.reason = reason;
    navigation_memory_.recordTrace(trace);
}

void ExplorationRuntimeManager::enterRecoveryLock(const ExplorationGoal &trigger_goal,
                                                  const general_utils::Vec3f &robot_pos,
                                                  const double stamp,
                                                  const std::string &reason)
{
    if (!cfg_.exploration_nhbp_recovery_enable) {
        return;
    }
    recovery_lock_active_ = true;
    recovery_lock_trigger_goal_ = trigger_goal;
    active_recovery_goal_ = ExplorationGoal{};
    has_active_recovery_goal_ = false;
    recovery_lock_reason_ = reason;
    recovery_lock_started_stamp_ = stamp;
    const double lock_duration =
            std::max({3.0,
                      cfg_.exploration_recovery_min_duration,
                      cfg_.exploration_local_trap_cooldown,
                      cfg_.exploration_nhbp_no_progress_time});
    recovery_lock_until_ = stamp + lock_duration;
    ++recovery_lock_request_count_;
    recovery_state_.active = true;
    recovery_state_.recovery_id = normalizedIdentity(trigger_goal);
    recovery_state_.start_pos = robot_pos.allFinite() ? robot_pos : trigger_goal.position;
    recovery_state_.goal_pos =
            trigger_goal.position.allFinite() ? trigger_goal.position : recovery_state_.start_pos;
    recovery_state_.start_stamp = stamp;
    recovery_state_.min_duration = std::max(0.0, cfg_.exploration_recovery_min_duration);
    recovery_state_.min_distance = std::max(
            {cfg_.exploration_recovery_min_distance,
             cfg_.exploration_topology_recovery_min_distance,
             cfg_.exploration_nhbp_min_progress_distance * 2.0});
    recovery_state_.lock_until = recovery_lock_until_;
    recovery_state_.exit_success = false;
    recovery_state_.reason = reason;
    recovery_state_.exit_reason.clear();
    phase_ = Phase::RECOVERY_ESCAPE;
    if (robot_pos.allFinite()) {
        has_recent_trap_region_ = true;
        recent_trap_position_ = trigger_goal.position.allFinite()
                                        ? trigger_goal.position
                                        : robot_pos;
        recent_trap_frontier_id_ = trigger_goal.frontier_id;
        recent_trap_block_until_ = recovery_lock_until_;
    }
}

void ExplorationRuntimeManager::bindRecoveryGoal(const ExplorationGoal &goal,
                                                 const general_utils::Vec3f &robot_pos,
                                                 const double stamp)
{
    if (!goal.valid || !isRecoveryGoal(goal)) {
        return;
    }
    const bool locked_reuse =
            goal.reason.rfind("locked_recovery_goal_reuse:", 0) == 0;
    recovery_state_.active = true;
    recovery_state_.recovery_id = normalizedIdentity(goal);
    if (!recovery_state_.start_pos.allFinite() ||
        recovery_state_.start_pos.squaredNorm() <= 1.0e-12) {
        recovery_state_.start_pos = robot_pos.allFinite() ? robot_pos : goal.position;
    }
    recovery_state_.goal_pos = goal.position;
    recovery_state_.min_duration = std::max(0.0, cfg_.exploration_recovery_min_duration);
    recovery_state_.min_distance = std::max(
            {cfg_.exploration_recovery_min_distance,
             cfg_.exploration_topology_recovery_min_distance,
             cfg_.exploration_nhbp_min_progress_distance * 2.0});
    if (recovery_state_.start_stamp <= 0.0) {
        recovery_state_.start_stamp = stamp;
    }
    if (!locked_reuse) {
        recovery_state_.lock_until =
                std::max(recovery_state_.lock_until,
                         recovery_state_.start_stamp +
                                 std::max({recovery_state_.min_duration,
                                           cfg_.exploration_local_trap_cooldown,
                                           cfg_.exploration_nhbp_no_progress_time}));
        recovery_lock_until_ = std::max(recovery_lock_until_, recovery_state_.lock_until);
    }
    recovery_state_.reason = stripLockedRecoveryReusePrefix(goal.reason);
    recovery_state_.exit_reason.clear();
    recovery_lock_reason_ = recovery_state_.reason;
    active_recovery_goal_ = goal;
    active_recovery_goal_.identity = normalizedIdentity(goal);
    active_recovery_goal_.identity.recovery_intent = true;
    active_recovery_goal_.identity.intent_mode = "recovery";
    has_active_recovery_goal_ = true;
    phase_ = Phase::RECOVERY_ESCAPE;
}

void ExplorationRuntimeManager::releaseRecoveryLock(const double stamp,
                                                    const std::string &reason,
                                                    const bool success)
{
    const bool was_active = recovery_lock_active_ ||
                            recovery_state_.active ||
                            has_active_recovery_goal_;
    recovery_lock_active_ = false;
    recovery_state_.active = false;
    recovery_state_.exit_success = success;
    recovery_state_.exit_reason = reason;
    recovery_state_.lock_until = stamp;
    recovery_lock_until_ = stamp;
    recovery_lock_reason_ = reason;
    active_recovery_goal_ = ExplorationGoal{};
    has_active_recovery_goal_ = false;
    if (was_active) {
        ++recovery_lock_release_count_;
    }
}

void ExplorationRuntimeManager::updateRecoveryLock(const general_utils::Vec3f &robot_pos,
                                                   const double stamp)
{
    if (!recovery_lock_active_ && !recovery_state_.active) {
        return;
    }
    const double max_lock_until =
            recovery_state_.start_stamp +
            std::max({recovery_state_.min_duration * 3.0,
                      cfg_.exploration_local_trap_cooldown * 2.0,
                      cfg_.exploration_nhbp_no_progress_time * 2.0,
                      3.0});
    const bool timed_out = stamp >= std::max(recovery_lock_until_, max_lock_until);
    const bool min_elapsed =
            stamp - recovery_state_.start_stamp >= recovery_state_.min_duration;
    const bool escaped =
            robot_pos.allFinite() &&
            recovery_state_.start_pos.allFinite() &&
            (robot_pos - recovery_state_.start_pos).norm() >= recovery_state_.min_distance;
    const bool reached_goal_region =
            robot_pos.allFinite() &&
            recovery_state_.goal_pos.allFinite() &&
            (robot_pos - recovery_state_.goal_pos).norm() <=
                    std::max(cfg_.exploration_goal_reached_distance,
                             cfg_.exploration_coverage_revisit_radius);
    bool active_goal_invalid = false;
    if (has_active_recovery_goal_ && active_recovery_goal_.valid) {
        const nhbp::NavIdentity active_identity = normalizedIdentity(active_recovery_goal_);
        const std::string active_key = active_identity.blacklistKey();
        active_goal_invalid =
                frontier_memory_.blocked(active_recovery_goal_, stamp) ||
                (!active_key.empty() &&
                 navigation_memory_.isBlacklisted(active_key, stamp));
    }
    if (!active_goal_invalid &&
        !timed_out &&
        !(min_elapsed && (escaped || reached_goal_region))) {
        return;
    }
    const std::string release_reason =
            active_goal_invalid ? "released_by_invalid_recovery_goal"
                                : (timed_out ? "released_by_timeout"
                                             : (reached_goal_region
                                                        ? "released_by_goal_region"
                                                        : "released_by_escape_distance"));
    releaseRecoveryLock(stamp, release_reason, !timed_out && !active_goal_invalid);
}

bool ExplorationRuntimeManager::recoveryLockActive(const double stamp) const
{
    return (recovery_lock_active_ && stamp < recovery_lock_until_) ||
           (recovery_state_.active && stamp < recovery_state_.lock_until);
}

bool ExplorationRuntimeManager::lockedRecoveryGoalReusable(
        const general_utils::Vec3f &robot_pos,
        const double stamp) const
{
    if (!recoveryLockActive(stamp) ||
        !has_active_recovery_goal_ ||
        !active_recovery_goal_.valid ||
        !active_recovery_goal_.position.allFinite()) {
        return false;
    }
    const nhbp::NavIdentity identity = normalizedIdentity(active_recovery_goal_);
    const std::string key = identity.blacklistKey();
    if (!key.empty() && navigation_memory_.isBlacklisted(key, stamp)) {
        return false;
    }
    if (frontier_memory_.blocked(active_recovery_goal_, stamp)) {
        return false;
    }
    if (recoveryBlockedByRecentTrap(active_recovery_goal_, stamp)) {
        return false;
    }
    if (!robot_pos.allFinite()) {
        return true;
    }
    const double max_reuse_distance =
            std::max({cfg_.exploration_goal_reached_distance,
                      cfg_.exploration_coverage_revisit_radius,
                      0.3});
    const double distance_to_goal =
            (active_recovery_goal_.position - robot_pos).norm();
    if (distance_to_goal <= max_reuse_distance) {
        return stamp < std::max(recovery_lock_until_, recovery_state_.lock_until);
    }
    return true;
}

bool ExplorationRuntimeManager::nhbpEnabled() const
{
    return cfg_.exploration_nhbp_enable;
}

} // namespace general_planner
