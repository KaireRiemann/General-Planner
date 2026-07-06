#include "general_core/exploration/exploration_runtime_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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

double yawDistance(const double from_yaw, const double to_yaw)
{
    if (!std::isfinite(from_yaw) || !std::isfinite(to_yaw)) {
        return 0.0;
    }
    return std::abs(std::atan2(std::sin(to_yaw - from_yaw),
                               std::cos(to_yaw - from_yaw)));
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
                  cfg.exploration_coverage_intent_radius,
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
                  cfg.exploration_topology_recovery_max_distance}),
          frontier_db_(cfg),
          task_planner_(cfg)
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
    active_sector_ = ActiveSector{};
    active_tour_ = ActiveTour{};
    sector_memory_.clear();
    active_sector_reuse_count_ = 0;
    active_sector_switch_count_ = 0;
    active_sector_invalid_count_ = 0;
    active_sector_filter_count_ = 0;
    sector_completed_count_ = 0;
    sector_blocked_count_ = 0;
    sector_reactivated_count_ = 0;
    active_tour_reuse_count_ = 0;
    active_tour_rebuild_count_ = 0;
    active_tour_repair_count_ = 0;
    active_tour_advance_count_ = 0;
    active_tour_invalid_count_ = 0;
    active_tour_node_completed_count_ = 0;
    active_tour_node_failed_count_ = 0;
    active_tour_node_skipped_count_ = 0;
    navigation_memory_.reset();
    frontier_memory_.reset();
    coverage_grid_.reset();
    topological_memory_.reset();
    frontier_db_.reset();
    task_planner_.reset();
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

ExplorationRuntimeManager::SelectionDecision
ExplorationRuntimeManager::selectGoalFromCandidates(
        const ExplorationCandidateSet &candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double current_yaw,
        const double committed_remaining,
        const double stamp,
        const bool new_task)
{
    SelectionDecision out;
    if (!candidate_set.valid || candidate_set.candidates.empty()) {
        out.reject = true;
        out.reason = candidate_set.reason.empty()
                             ? "candidate_set_empty"
                             : candidate_set.reason;
        out.ndo = navigation_memory_.diagnose(stamp);
        return out;
    }

    const auto best_fallback_goal =
            [](const ExplorationCandidateSet &set) {
                if (set.suggested_goal.valid) {
                    return set.suggested_goal;
                }
                return *std::min_element(set.candidates.begin(),
                                         set.candidates.end(),
                                         [](const ExplorationGoal &lhs,
                                            const ExplorationGoal &rhs) {
                                             return lhs.score < rhs.score;
                                         });
            };

    if (!activeTourEnabled()) {
        const ExplorationGoal fallback = best_fallback_goal(candidate_set);
        return stabilizeCandidate(fallback,
                                  robot_pos,
                                  committed_remaining,
                                  stamp,
                                  new_task);
    }

    phase_ = Phase::SELECT_TOUR;
    updateRecoveryLock(robot_pos, stamp);
    updateSectorMemoryFromCandidates(candidate_set, stamp);
    ExplorationFrontierDB::ObservationContext frontier_db_context;
    frontier_db_context.robot_pos = robot_pos;
    frontier_db_context.stamp = stamp;
    frontier_db_context.node_penalty =
            [this, stamp](const ExplorationGoal &goal) {
                const std::string sector_key = sectorKeyForGoal(goal);
                return std::max(0.0, cfg_.exploration_tour_coverage_penalty_weight) *
                               coverage_grid_.revisitPenalty(sectorReference(goal), stamp) +
                       sectorMemoryPenalty(sector_key, stamp);
            };
    frontier_db_context.coverage_intent_reward =
            [this, stamp](const ExplorationGoal &goal) {
                if (!cfg_.exploration_coverage_intent_enable) {
                    return 0.0;
                }
                return coverage_grid_.intentReward(sectorReference(goal), stamp);
            };
    frontier_db_context.sector_key =
            [this](const ExplorationGoal &goal) {
                return sectorKeyForGoal(goal);
            };
    frontier_db_context.sector_reference =
            [this](const ExplorationGoal &goal) {
                return sectorReference(goal);
            };
    frontier_db_.observeCandidates(candidate_set, frontier_db_context);
    advanceCompletedTourNodes(robot_pos, stamp);

    std::string sector_reason;
    const ExplorationCandidateSet sector_candidate_set =
            selectSectorCandidates(candidate_set,
                                   robot_pos,
                                   stamp,
                                   new_task,
                                   sector_reason);
    const ExplorationCandidateSet &decision_candidates =
            sector_candidate_set.valid && !sector_candidate_set.candidates.empty()
                    ? sector_candidate_set
                    : candidate_set;

    ExplorationGoal prefix_goal;
    std::string prefix_reason;
    if (findActiveTourCandidate(decision_candidates,
                                robot_pos,
                                stamp,
                                prefix_goal,
                                prefix_reason)) {
        prefix_goal.reason += " active_sector=" + sector_reason +
                              " active_tour_prefix_reuse=" + prefix_reason;
        SelectionDecision prefix_decision =
                stabilizeCandidate(prefix_goal,
                                   robot_pos,
                                   committed_remaining,
                                   stamp,
                                   new_task);
        if (prefix_decision.ready) {
            markTourNodeExecuting(prefix_decision.goal, stamp);
            ++active_tour_reuse_count_;
            return prefix_decision;
        }
        if (prefix_decision.recovery_requested) {
            return prefix_decision;
        }
        ++active_tour_repair_count_;
    }

    std::string repair_reason;
    if (repairActiveTourFromCandidates(decision_candidates,
                                       robot_pos,
                                       stamp,
                                       repair_reason) &&
        findActiveTourCandidate(decision_candidates,
                                robot_pos,
                                stamp,
                                prefix_goal,
                                prefix_reason)) {
        prefix_goal.reason += " active_sector=" + sector_reason +
                              " active_tour_repair=" + repair_reason +
                              " active_tour_prefix_reuse=" + prefix_reason;
        SelectionDecision repaired_decision =
                stabilizeCandidate(prefix_goal,
                                   robot_pos,
                                   committed_remaining,
                                   stamp,
                                   new_task);
        if (repaired_decision.ready) {
            markTourNodeExecuting(repaired_decision.goal, stamp);
            ++active_tour_reuse_count_;
            return repaired_decision;
        }
        if (repaired_decision.recovery_requested) {
            return repaired_decision;
        }
    }

    std::string rebuild_reason;
    const bool rebuild_throttled =
            active_tour_.valid &&
            cfg_.exploration_active_tour_rebuild_min_interval > 0.0 &&
            stamp - active_tour_.last_rebuild_stamp <
                    cfg_.exploration_active_tour_rebuild_min_interval;
    if (rebuild_throttled && !decision_candidates.candidates.empty()) {
        ExplorationGoal throttled_goal = best_fallback_goal(decision_candidates);
        throttled_goal.reason += " active_sector=" + sector_reason +
                                 " active_tour_rebuild_throttled";
        return stabilizeCandidate(throttled_goal,
                                  robot_pos,
                                  committed_remaining,
                                  stamp,
                                  new_task);
    }

    if (!rebuildActiveTour(decision_candidates,
                           robot_pos,
                           current_yaw,
                           stamp,
                           rebuild_reason)) {
        const ExplorationGoal fallback = best_fallback_goal(decision_candidates);
        return stabilizeCandidate(fallback,
                                  robot_pos,
                                  committed_remaining,
                                  stamp,
                                  new_task);
    }

    for (int i = std::max(0, active_tour_.cursor);
         active_tour_.valid &&
         i < static_cast<int>(active_tour_.goals.size());
         ++i) {
        ExplorationGoal tour_goal = active_tour_.goals[static_cast<size_t>(i)];
        tour_goal.identity.tour_key = active_tour_.tour_key;
        tour_goal.identity.tour_rank = i;
        tour_goal.reason += " active_sector=" + sector_reason +
                            " active_tour=" + rebuild_reason +
                            " rank=" + std::to_string(i);
        SelectionDecision decision =
                stabilizeCandidate(tour_goal,
                                   robot_pos,
                                   committed_remaining,
                                   stamp,
                                   new_task);
        if (decision.ready) {
            active_tour_.cursor = i;
            markTourNodeExecuting(decision.goal, stamp);
            return decision;
        }
        if (decision.recovery_requested) {
            return decision;
        }
        ++active_tour_repair_count_;
    }

    invalidateActiveTour("all_tour_nodes_rejected");
    if (activeSectorEnabled()) {
        ++active_sector_.failure_count;
        if (active_sector_.failure_count >= 3) {
            invalidateActiveSector("tour_nodes_rejected");
        }
    }
    const ExplorationGoal fallback = best_fallback_goal(decision_candidates);
    return stabilizeCandidate(fallback,
                              robot_pos,
                              committed_remaining,
                              stamp,
                              new_task);
}

ExplorationRuntimeManager::SelectionDecision
ExplorationRuntimeManager::selectGoalFromActiveTour(
        const general_utils::Vec3f &robot_pos,
        const double committed_remaining,
        const double stamp,
        const bool new_task)
{
    SelectionDecision out;
    if (!activeTourEnabled() || !active_tour_.valid || active_tour_.goals.empty()) {
        out.reject = true;
        out.reason = "active_tour_unavailable";
        out.ndo = navigation_memory_.diagnose(stamp);
        return out;
    }

    phase_ = Phase::SELECT_TOUR;
    updateRecoveryLock(robot_pos, stamp);
    advanceCompletedTourNodes(robot_pos, stamp);

    ExplorationCandidateSet empty_candidate_set;
    empty_candidate_set.valid = true;
    ExplorationGoal prefix_goal;
    std::string prefix_reason;
    if (!findActiveTourCandidate(empty_candidate_set,
                                 robot_pos,
                                 stamp,
                                 prefix_goal,
                                 prefix_reason)) {
        out.reject = true;
        out.reason = "active_tour_prefix_unavailable:" + prefix_reason;
        out.ndo = navigation_memory_.diagnose(stamp);
        return out;
    }

    prefix_goal.reason += " active_tour_no_new_frontend_candidate=" + prefix_reason;
    SelectionDecision decision =
            stabilizeCandidate(prefix_goal,
                               robot_pos,
                               committed_remaining,
                               stamp,
                               new_task);
    if (decision.ready) {
        markTourNodeExecuting(decision.goal, stamp);
        ++active_tour_reuse_count_;
    }
    return decision;
}

ExplorationRuntimeManager::SelectionDecision
ExplorationRuntimeManager::selectGoalForExecution(
        const ExplorationCandidateSet &candidate_set,
        const bool has_candidate_set,
        const ExplorationGoal &frontend_goal,
        const general_utils::Vec3f &robot_pos,
        const double current_yaw,
        const double committed_remaining,
        const double stamp,
        const bool new_task)
{
    phase_ = Phase::UPDATE_BELIEF;
    updateRecoveryLock(robot_pos, stamp);

    if (has_candidate_set && candidate_set.valid && !candidate_set.candidates.empty()) {
        SelectionDecision decision =
                selectGoalFromCandidates(candidate_set,
                                         robot_pos,
                                         current_yaw,
                                         committed_remaining,
                                         stamp,
                                         new_task);
        if (decision.ready || decision.recovery_requested || decision.reject) {
            return decision;
        }
    }

    SelectionDecision tour_decision =
            selectGoalFromActiveTour(robot_pos,
                                     committed_remaining,
                                     stamp,
                                     new_task);
    if (tour_decision.ready || tour_decision.recovery_requested) {
        tour_decision.reason =
                "manager_active_tour_fallback:" + tour_decision.reason;
        return tour_decision;
    }

    if (frontend_goal.valid) {
        SelectionDecision fallback =
                stabilizeCandidate(frontend_goal,
                                   robot_pos,
                                   committed_remaining,
                                   stamp,
                                   new_task);
        if (fallback.ready) {
            fallback.reason =
                    "manager_frontend_single_goal_fallback:" + fallback.reason;
        }
        return fallback;
    }

    SelectionDecision out;
    out.reject = true;
    out.ndo = navigation_memory_.diagnose(stamp);
    out.reason =
            has_candidate_set && !candidate_set.reason.empty()
                    ? "manager_no_executable_goal:" + candidate_set.reason
                    : "manager_no_executable_goal:" + tour_decision.reason;
    return out;
}

bool ExplorationRuntimeManager::refreshGlobalTaskGraph(
        const ExplorationCandidateSet &candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double current_yaw,
        const double stamp,
        const bool new_task,
        std::string &reason)
{
    reason.clear();
    (void)new_task;
    phase_ = Phase::UPDATE_BELIEF;
    updateRecoveryLock(robot_pos, stamp);

    if (!activeTourEnabled()) {
        reason = "task_graph_refresh_disabled";
        return false;
    }
    if (!candidate_set.valid || candidate_set.candidates.empty()) {
        reason = candidate_set.reason.empty()
                         ? "task_graph_refresh_empty_candidate_set"
                         : "task_graph_refresh_empty_candidate_set:" +
                                   candidate_set.reason;
        return false;
    }

    updateSectorMemoryFromCandidates(candidate_set, stamp);
    ExplorationFrontierDB::ObservationContext frontier_db_context;
    frontier_db_context.robot_pos = robot_pos;
    frontier_db_context.stamp = stamp;
    frontier_db_context.node_penalty =
            [this, stamp](const ExplorationGoal &goal) {
                const std::string sector_key = sectorKeyForGoal(goal);
                return std::max(0.0, cfg_.exploration_tour_coverage_penalty_weight) *
                               coverage_grid_.revisitPenalty(sectorReference(goal), stamp) +
                       sectorMemoryPenalty(sector_key, stamp);
            };
    frontier_db_context.coverage_intent_reward =
            [this, stamp](const ExplorationGoal &goal) {
                if (!cfg_.exploration_coverage_intent_enable) {
                    return 0.0;
                }
                return coverage_grid_.intentReward(sectorReference(goal), stamp);
            };
    frontier_db_context.sector_key =
            [this](const ExplorationGoal &goal) {
                return sectorKeyForGoal(goal);
            };
    frontier_db_context.sector_reference =
            [this](const ExplorationGoal &goal) {
                return sectorReference(goal);
            };
    frontier_db_.observeCandidates(candidate_set, frontier_db_context);
    const bool advanced = advanceCompletedTourNodes(robot_pos, stamp);

    std::string repair_reason;
    const bool repaired =
            repairActiveTourFromCandidates(candidate_set,
                                           robot_pos,
                                           stamp,
                                           repair_reason);
    bool rebuilt = false;
    std::string rebuild_reason;
    const bool needs_rebuild =
            !active_tour_.valid || pendingTourNodeCount() <= 0;
    const bool rebuild_allowed =
            !active_tour_.valid ||
            cfg_.exploration_active_tour_rebuild_min_interval <= 0.0 ||
            stamp - active_tour_.last_rebuild_stamp >=
                    cfg_.exploration_active_tour_rebuild_min_interval;
    if (!repaired && needs_rebuild && rebuild_allowed) {
        rebuilt = rebuildActiveTour(candidate_set,
                                    robot_pos,
                                    current_yaw,
                                    stamp,
                                    rebuild_reason);
    }

    std::ostringstream oss;
    oss << "task_graph_refresh"
        << ":raw=" << candidate_set.candidates.size()
        << ",tour_valid=" << static_cast<int>(active_tour_.valid)
        << ",pending=" << pendingTourNodeCount()
        << ",executing=" << executingTourNodeCount()
        << ",advanced=" << static_cast<int>(advanced)
        << ",repaired=" << static_cast<int>(repaired)
        << ",rebuilt=" << static_cast<int>(rebuilt);
    if (!repair_reason.empty()) {
        oss << ",repair=" << repair_reason;
    }
    if (!rebuild_reason.empty()) {
        oss << ",rebuild=" << rebuild_reason;
    }
    reason = oss.str();
    return advanced || repaired || rebuilt;
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
    frontier_db_.markCommitted(goal, stamp);
    frontier_db_.markCoveredNear(robot_pos,
                                 stamp,
                                 std::max(cfg_.exploration_coverage_revisit_radius,
                                          cfg_.exploration_frontier_memory_covered_radius));
    topological_memory_.observePose(goal.position,
                                    stamp,
                                    nhbp::TopoNodeType::FRONTIER);
    topological_memory_.observeTransition(robot_pos, goal.position, stamp);
    markTourNodeExecuting(goal, stamp);
    markSectorProgress(goal, stamp);
    if (activeSectorEnabled() && active_sector_.valid) {
        const std::string goal_sector = sectorKeyForGoal(goal);
        if (goal_sector == active_sector_.key) {
            active_sector_.last_progress_stamp = stamp;
            active_sector_.failure_count = 0;
        }
    }
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
    if (activeTourEnabled() && active_tour_.valid) {
        markTourNodeFailed(goal, stamp);
    }
    markSectorFailure(goal, stamp);
    if (activeSectorEnabled() && active_sector_.valid) {
        const std::string failed_sector = sectorKeyForGoal(goal);
        if (!failed_sector.empty() && failed_sector == active_sector_.key) {
            ++active_sector_.failure_count;
            if (active_sector_.failure_count >= 3) {
                invalidateActiveSector("sector_goal_failures");
            }
        }
    }
    frontier_memory_.markFailed(goal, stamp);
    frontier_db_.markFailed(goal, stamp);
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
        << ";active_sector_valid=" << static_cast<int>(active_sector_.valid)
        << ";active_sector_generation=" << active_sector_.generation
        << ";active_sector_key=" << active_sector_.key
        << ";active_sector_candidates=" << active_sector_.candidate_count
        << ";active_sector_score=" << active_sector_.score
        << ";active_sector_failures=" << active_sector_.failure_count
        << ";active_sector_invalid_reason=" << active_sector_.invalid_reason
        << ";active_sector_reuse=" << active_sector_reuse_count_
        << ";active_sector_switch=" << active_sector_switch_count_
        << ";active_sector_invalid=" << active_sector_invalid_count_
        << ";active_sector_filter=" << active_sector_filter_count_
        << ";sector_memory_size=" << sector_memory_.size()
        << ";sector_completed_events=" << sector_completed_count_
        << ";sector_blocked_events=" << sector_blocked_count_
        << ";sector_reactivated_events=" << sector_reactivated_count_
        << ";active_tour_valid=" << static_cast<int>(active_tour_.valid)
        << ";active_tour_generation=" << active_tour_.generation
        << ";active_tour_size=" << active_tour_.goals.size()
        << ";active_tour_cursor=" << active_tour_.cursor
        << ";active_tour_executing_rank=" << active_tour_.executing_rank
        << ";active_tour_pending_nodes=" << pendingTourNodeCount()
        << ";active_tour_executing_nodes=" << executingTourNodeCount()
        << ";active_tour_completed_nodes=" << completedTourNodeCount()
        << ";active_tour_failed_nodes=" << failedTourNodeCount()
        << ";active_tour_key=" << active_tour_.tour_key
        << ";active_tour_sector=" << active_tour_.sector_key
        << ";active_tour_invalid_reason=" << active_tour_.invalid_reason
        << ";active_tour_reuse=" << active_tour_reuse_count_
        << ";active_tour_rebuild=" << active_tour_rebuild_count_
        << ";active_tour_repair=" << active_tour_repair_count_
        << ";active_tour_advance=" << active_tour_advance_count_
        << ";active_tour_invalid=" << active_tour_invalid_count_
        << ";active_tour_node_completed_events=" << active_tour_node_completed_count_
        << ";active_tour_node_failed_events=" << active_tour_node_failed_count_
        << ";active_tour_node_skipped_events=" << active_tour_node_skipped_count_
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
    const std::string recovery_reason = stripLockedRecoveryReusePrefix(goal.reason);
    const bool frontier_memory_recovery =
            recovery_reason.find("frontier_memory_recovery") != std::string::npos;
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
    if (frontier_memory_recovery) {
        const double short_lock_duration =
                std::max(0.2, cfg_.exploration_frontier_memory_recovery_lock_duration);
        const double short_lock_distance =
                std::max({0.3,
                          cfg_.exploration_frontier_memory_recovery_lock_distance,
                          cfg_.exploration_frontier_memory_recovery_min_distance,
                          cfg_.exploration_nhbp_min_progress_distance});
        recovery_state_.min_duration =
                std::min(recovery_state_.min_duration, short_lock_duration);
        recovery_state_.min_distance =
                std::min(recovery_state_.min_distance, short_lock_distance);
    }
    if (recovery_state_.start_stamp <= 0.0) {
        recovery_state_.start_stamp = stamp;
    }
    if (!locked_reuse) {
        const double lock_duration =
                frontier_memory_recovery
                        ? recovery_state_.min_duration
                        : std::max({recovery_state_.min_duration,
                                    cfg_.exploration_local_trap_cooldown,
                                    cfg_.exploration_nhbp_no_progress_time});
        const double next_lock_until =
                recovery_state_.start_stamp + std::max(0.2, lock_duration);
        if (frontier_memory_recovery) {
            recovery_state_.lock_until = next_lock_until;
            recovery_lock_until_ = next_lock_until;
        } else {
            recovery_state_.lock_until =
                    std::max(recovery_state_.lock_until, next_lock_until);
            recovery_lock_until_ =
                    std::max(recovery_lock_until_, recovery_state_.lock_until);
        }
    }
    recovery_state_.reason = recovery_reason;
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

bool ExplorationRuntimeManager::activeTourEnabled() const
{
    return cfg_.exploration_active_tour_enable && cfg_.exploration_use_atsp;
}

bool ExplorationRuntimeManager::activeSectorEnabled() const
{
    return cfg_.exploration_active_sector_enable && activeTourEnabled();
}

double ExplorationRuntimeManager::activeSectorResolution() const
{
    return std::max({cfg_.exploration_active_sector_size,
                     2.0 * cfg_.exploration_frontier_cluster_radius,
                     2.0 * cfg_.exploration_coverage_grid_resolution,
                     1.0});
}

general_utils::Vec3f ExplorationRuntimeManager::sectorReference(
        const ExplorationGoal &goal) const
{
    const bool expansion_candidate =
            goal.identity.intent_mode == "exploration_expansion" ||
            goal.reason.find("expansion") != std::string::npos;
    if (expansion_candidate) {
        return goal.position;
    }
    if (goal.frontier_center_valid && goal.frontier_center.allFinite()) {
        return goal.frontier_center;
    }
    return goal.position;
}

std::string ExplorationRuntimeManager::sectorKeyForGoal(
        const ExplorationGoal &goal) const
{
    const general_utils::Vec3f reference = sectorReference(goal);
    return nhbp::quantizedPositionKey(reference,
                                     activeSectorResolution(),
                                     "sector");
}

void ExplorationRuntimeManager::updateSectorMemoryFromCandidates(
        const ExplorationCandidateSet &candidate_set,
        const double stamp)
{
    if (!activeSectorEnabled() ||
        !candidate_set.valid ||
        candidate_set.candidates.empty()) {
        return;
    }

    struct Accumulator {
        general_utils::Vec3f center{general_utils::Vec3f::Zero()};
        double score{std::numeric_limits<double>::infinity()};
        int count{0};
    };

    std::unordered_map<std::string, Accumulator> observed;
    observed.reserve(candidate_set.candidates.size());
    for (const ExplorationGoal &candidate : candidate_set.candidates) {
        if (!candidate.valid || !candidate.position.allFinite()) {
            continue;
        }
        const std::string key = sectorKeyForGoal(candidate);
        if (key.empty()) {
            continue;
        }
        Accumulator &acc = observed[key];
        acc.center += sectorReference(candidate);
        ++acc.count;
        acc.score = std::min(acc.score, candidate.score);
    }

    for (const auto &entry : observed) {
        const std::string &key = entry.first;
        const Accumulator &acc = entry.second;
        if (acc.count <= 0) {
            continue;
        }
        SectorMemoryEntry &memory = sector_memory_[key];
        const bool new_entry = memory.key.empty();
        const SectorStatus previous_status = memory.status;
        memory.key = key;
        memory.center = acc.center / static_cast<double>(acc.count);
        memory.score = acc.score;
        memory.candidate_count = acc.count;
        memory.total_seen_count += acc.count;
        if (new_entry || memory.first_seen_stamp <= 0.0) {
            memory.first_seen_stamp = stamp;
        }
        memory.last_seen_stamp = stamp;
        if (memory.status == SectorStatus::UNKNOWN) {
            memory.status = SectorStatus::STALE;
        }
        const double completed_age =
                memory.completed_stamp > 0.0
                        ? std::max(0.0, stamp - memory.completed_stamp)
                        : std::numeric_limits<double>::infinity();
        if (memory.status == SectorStatus::COMPLETED &&
            completed_age >= std::max(0.0, cfg_.exploration_active_sector_min_duration) &&
            acc.count > std::max(1, cfg_.exploration_sector_completion_max_candidates)) {
            memory.status = SectorStatus::STALE;
            ++sector_reactivated_count_;
        }
        if (memory.status == SectorStatus::BLOCKED && stamp >= memory.block_until) {
            memory.status = SectorStatus::STALE;
        }
        if (previous_status == SectorStatus::COMPLETED &&
            memory.status != SectorStatus::COMPLETED) {
            memory.completed_stamp = 0.0;
        }
    }

    const int max_records = std::max(16, cfg_.exploration_sector_memory_max_records);
    while (static_cast<int>(sector_memory_.size()) > max_records) {
        auto oldest = sector_memory_.end();
        for (auto it = sector_memory_.begin(); it != sector_memory_.end(); ++it) {
            if (it->second.status == SectorStatus::ACTIVE) {
                continue;
            }
            if (oldest == sector_memory_.end() ||
                it->second.last_seen_stamp < oldest->second.last_seen_stamp) {
                oldest = it;
            }
        }
        if (oldest == sector_memory_.end()) {
            break;
        }
        sector_memory_.erase(oldest);
    }
}

void ExplorationRuntimeManager::markSectorActive(const std::string &sector_key,
                                                 const general_utils::Vec3f &center,
                                                 const int candidate_count,
                                                 const double score,
                                                 const double stamp)
{
    if (sector_key.empty()) {
        return;
    }
    SectorMemoryEntry &memory = sector_memory_[sector_key];
    const bool was_completed = memory.status == SectorStatus::COMPLETED;
    const bool was_blocked = memory.status == SectorStatus::BLOCKED &&
                             stamp < memory.block_until;
    memory.key = sector_key;
    if (center.allFinite()) {
        memory.center = center;
    }
    memory.candidate_count = candidate_count;
    memory.score = score;
    memory.last_selected_stamp = stamp;
    memory.last_seen_stamp = std::max(memory.last_seen_stamp, stamp);
    if (memory.first_seen_stamp <= 0.0) {
        memory.first_seen_stamp = stamp;
    }
    if (!was_blocked) {
        memory.status = SectorStatus::ACTIVE;
        if (was_completed) {
            ++sector_reactivated_count_;
            memory.completed_stamp = 0.0;
        }
    }
    ++memory.selection_count;
}

void ExplorationRuntimeManager::markSectorProgress(const ExplorationGoal &goal,
                                                   const double stamp)
{
    const std::string key = sectorKeyForGoal(goal);
    if (key.empty()) {
        return;
    }
    SectorMemoryEntry &memory = sector_memory_[key];
    memory.key = key;
    memory.center = sectorReference(goal);
    memory.last_progress_stamp = stamp;
    memory.last_seen_stamp = std::max(memory.last_seen_stamp, stamp);
    if (memory.first_seen_stamp <= 0.0) {
        memory.first_seen_stamp = stamp;
    }
    ++memory.progress_count;
    memory.failure_count = 0;
    if (memory.status == SectorStatus::BLOCKED && stamp >= memory.block_until) {
        memory.status = SectorStatus::ACTIVE;
    }
    const bool enough_progress =
            memory.progress_count >=
            std::max(1, cfg_.exploration_sector_completion_min_commits);
    const bool candidate_count_low =
            memory.candidate_count <=
            std::max(0, cfg_.exploration_sector_completion_max_candidates);
    if (enough_progress && candidate_count_low) {
        if (memory.status != SectorStatus::COMPLETED) {
            ++sector_completed_count_;
        }
        memory.status = SectorStatus::COMPLETED;
        memory.completed_stamp = stamp;
        if (active_sector_.valid && active_sector_.key == key) {
            active_sector_.last_progress_stamp = stamp;
        }
    } else if (memory.status != SectorStatus::COMPLETED) {
        memory.status = SectorStatus::ACTIVE;
    }
}

void ExplorationRuntimeManager::markSectorFailure(const ExplorationGoal &goal,
                                                  const double stamp)
{
    const std::string key = sectorKeyForGoal(goal);
    if (key.empty()) {
        return;
    }
    SectorMemoryEntry &memory = sector_memory_[key];
    memory.key = key;
    memory.center = sectorReference(goal);
    memory.last_seen_stamp = std::max(memory.last_seen_stamp, stamp);
    if (memory.first_seen_stamp <= 0.0) {
        memory.first_seen_stamp = stamp;
    }
    ++memory.failure_count;
    if (memory.failure_count >=
        std::max(1, cfg_.exploration_sector_block_failure_threshold)) {
        if (memory.status != SectorStatus::BLOCKED) {
            ++sector_blocked_count_;
        }
        memory.status = SectorStatus::BLOCKED;
        memory.block_until =
                stamp + std::max(cfg_.exploration_local_trap_cooldown,
                                 cfg_.exploration_sector_memory_stale_time * 0.25);
    }
}

double ExplorationRuntimeManager::sectorMemoryPenalty(const std::string &sector_key,
                                                      const double stamp) const
{
    const auto it = sector_memory_.find(sector_key);
    if (it == sector_memory_.end()) {
        return 0.0;
    }
    const SectorMemoryEntry &memory = it->second;
    if (memory.status == SectorStatus::BLOCKED && stamp < memory.block_until) {
        return std::max(0.0, cfg_.exploration_sector_blocked_penalty);
    }
    if (memory.status == SectorStatus::COMPLETED) {
        return std::max(0.0, cfg_.exploration_sector_completed_penalty);
    }
    if (memory.status == SectorStatus::STALE &&
        stamp - memory.last_seen_stamp >
                std::max(1.0, cfg_.exploration_sector_memory_stale_time)) {
        return -0.25 * std::max(0.0, cfg_.exploration_sector_completed_penalty);
    }
    return 0.0;
}

ExplorationCandidateSet ExplorationRuntimeManager::selectSectorCandidates(
        const ExplorationCandidateSet &candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double stamp,
        const bool new_task,
        std::string &reason)
{
    reason = "sector_disabled";
    if (!activeSectorEnabled() ||
        !candidate_set.valid ||
        candidate_set.candidates.empty()) {
        return candidate_set;
    }

    struct SectorAccumulator {
        std::string key;
        double sum_x{0.0};
        double sum_y{0.0};
        double sum_z{0.0};
        double best_score{std::numeric_limits<double>::infinity()};
        double total_gain{0.0};
        int best_index{-1};
        int count{0};
    };

    std::unordered_map<std::string, SectorAccumulator> sectors;
    sectors.reserve(candidate_set.candidates.size());
    for (int i = 0; i < static_cast<int>(candidate_set.candidates.size()); ++i) {
        const ExplorationGoal &candidate =
                candidate_set.candidates[static_cast<size_t>(i)];
        if (!candidate.valid || !candidate.position.allFinite()) {
            continue;
        }
        const std::string key = sectorKeyForGoal(candidate);
        if (key.empty()) {
            continue;
        }
        const general_utils::Vec3f reference = sectorReference(candidate);
        SectorAccumulator &sector = sectors[key];
        sector.key = key;
        sector.sum_x += reference.x();
        sector.sum_y += reference.y();
        sector.sum_z += reference.z();
        ++sector.count;
        sector.total_gain += std::max(0.0, candidate.information_gain);
        const double coverage_penalty =
                std::max(0.0, cfg_.exploration_tour_coverage_penalty_weight) *
                coverage_grid_.revisitPenalty(reference, stamp);
        const double memory_penalty = sectorMemoryPenalty(key, stamp);
        const double coverage_reward =
                cfg_.exploration_coverage_intent_enable
                        ? std::max(0.0, cfg_.exploration_coverage_intent_weight) *
                                  coverage_grid_.intentReward(reference, stamp)
                        : 0.0;
        const double distance_bias =
                robot_pos.allFinite()
                        ? 0.05 * std::max(0.0, (reference - robot_pos).norm())
                        : 0.0;
        const double score =
                candidate.score + coverage_penalty + memory_penalty +
                distance_bias - coverage_reward;
        if (score < sector.best_score) {
            sector.best_score = score;
            sector.best_index = i;
        }
    }

    if (sectors.empty()) {
        reason = "sector_no_valid_candidates";
        return candidate_set;
    }

    auto sectorAdjustedScore = [this](const SectorAccumulator &sector) {
        const double gain_bonus =
                0.03 * std::min(sector.total_gain,
                                std::max(1.0, cfg_.exploration_information_gain_saturation));
        const double breadth_bonus = 0.25 * std::log1p(static_cast<double>(sector.count));
        return sector.best_score - gain_bonus - breadth_bonus;
    };

    const SectorAccumulator *best_sector = nullptr;
    const SectorAccumulator *active_sector_acc = nullptr;
    double best_sector_score = std::numeric_limits<double>::infinity();
    double active_sector_score = std::numeric_limits<double>::infinity();
    for (const auto &entry : sectors) {
        const SectorAccumulator &sector = entry.second;
        const double score = sectorAdjustedScore(sector);
        if (score < best_sector_score) {
            best_sector_score = score;
            best_sector = &sector;
        }
        if (active_sector_.valid && sector.key == active_sector_.key) {
            active_sector_acc = &sector;
            active_sector_score = score;
        }
    }

    if (best_sector == nullptr || best_sector->best_index < 0) {
        reason = "sector_no_best_candidate";
        return candidate_set;
    }

    bool keep_active_sector =
            active_sector_.valid &&
            active_sector_acc != nullptr &&
            !new_task;
    if (keep_active_sector) {
        const auto active_memory = sector_memory_.find(active_sector_.key);
        if (active_memory != sector_memory_.end() &&
            active_memory->second.status == SectorStatus::BLOCKED &&
            stamp < active_memory->second.block_until) {
            keep_active_sector = false;
        }
    }
    if (keep_active_sector) {
        const double active_age = std::max(0.0, stamp - active_sector_.created_stamp);
        const double switch_margin =
                std::max(0.0, cfg_.exploration_active_sector_switch_margin);
        const bool min_duration_elapsed =
                active_age >= std::max(0.0,
                                       cfg_.exploration_active_sector_min_duration);
        const bool outsider_clearly_better =
                best_sector->key != active_sector_.key &&
                best_sector_score + switch_margin < active_sector_score;
        if (min_duration_elapsed && outsider_clearly_better) {
            keep_active_sector = false;
        }
    }

    const SectorAccumulator *selected_sector =
            keep_active_sector ? active_sector_acc : best_sector;
    if (selected_sector == nullptr) {
        selected_sector = best_sector;
    }

    if (!active_sector_.valid || active_sector_.key != selected_sector->key || new_task) {
        if (active_sector_.valid && active_sector_.key != selected_sector->key) {
            ++active_sector_switch_count_;
        }
        active_sector_.valid = true;
        active_sector_.key = selected_sector->key;
        active_sector_.generation += 1;
        active_sector_.created_stamp = stamp;
        active_sector_.failure_count = 0;
    } else {
        ++active_sector_reuse_count_;
    }
    active_sector_.candidate_count = selected_sector->count;
    active_sector_.score = sectorAdjustedScore(*selected_sector);
    active_sector_.center =
            general_utils::Vec3f(selected_sector->sum_x / selected_sector->count,
                                 selected_sector->sum_y / selected_sector->count,
                                 selected_sector->sum_z / selected_sector->count);
    active_sector_.last_update_stamp = stamp;
    active_sector_.invalid_reason.clear();
    markSectorActive(active_sector_.key,
                     active_sector_.center,
                     active_sector_.candidate_count,
                     active_sector_.score,
                     stamp);

    ExplorationCandidateSet filtered = candidate_set;
    filtered.candidates.clear();
    filtered.reason = candidate_set.reason + " active_sector=" + active_sector_.key;
    filtered.suggested_goal = ExplorationGoal{};
    filtered.valid = true;
    for (const ExplorationGoal &candidate : candidate_set.candidates) {
        if (sectorKeyForGoal(candidate) != active_sector_.key) {
            continue;
        }
        filtered.candidates.push_back(candidate);
    }
    const int min_tour_candidates =
            std::min({static_cast<int>(candidate_set.candidates.size()),
                      std::max(1, cfg_.exploration_atsp_max_candidate_num),
                      std::max(2, std::min(6, cfg_.exploration_atsp_max_candidate_num))});
    int prefix_extra_count = 0;
    if (static_cast<int>(filtered.candidates.size()) < min_tour_candidates) {
        auto same_candidate = [](const ExplorationGoal &lhs,
                                 const ExplorationGoal &rhs) {
            if (!lhs.identity.candidate_key.empty() &&
                lhs.identity.candidate_key == rhs.identity.candidate_key) {
                return true;
            }
            if (!lhs.memory_key.empty() && lhs.memory_key == rhs.memory_key) {
                return true;
            }
            if (lhs.candidate_id != 0 && lhs.candidate_id == rhs.candidate_id) {
                return true;
            }
            return (lhs.position - rhs.position).squaredNorm() <= 0.25;
        };
        auto already_kept = [&filtered, &same_candidate](const ExplorationGoal &candidate) {
            for (const ExplorationGoal &kept : filtered.candidates) {
                if (same_candidate(candidate, kept)) {
                    return true;
                }
            }
            return false;
        };

        struct ExtraCandidate {
            int index{-1};
            double score{std::numeric_limits<double>::infinity()};
        };
        std::vector<ExtraCandidate> extras;
        extras.reserve(candidate_set.candidates.size());
        for (int i = 0; i < static_cast<int>(candidate_set.candidates.size()); ++i) {
            const ExplorationGoal &candidate =
                    candidate_set.candidates[static_cast<size_t>(i)];
            if (!candidate.valid ||
                !candidate.position.allFinite() ||
                already_kept(candidate)) {
                continue;
            }
            const general_utils::Vec3f reference = sectorReference(candidate);
            const double sector_distance_bias =
                    active_sector_.center.allFinite()
                            ? 0.12 * std::max(0.0,
                                              (reference - active_sector_.center).norm())
                            : 0.0;
            const double robot_distance_bias =
                    robot_pos.allFinite()
                            ? 0.02 * std::max(0.0, (reference - robot_pos).norm())
                            : 0.0;
            const double memory_penalty =
                    sectorMemoryPenalty(sectorKeyForGoal(candidate), stamp);
            const double gain_bonus =
                    0.02 * std::min(std::max(0.0, candidate.information_gain),
                                    std::max(1.0,
                                             cfg_.exploration_information_gain_saturation));
            extras.push_back(ExtraCandidate{
                    i,
                    candidate.score + sector_distance_bias + robot_distance_bias +
                            memory_penalty - gain_bonus});
        }
        std::sort(extras.begin(),
                  extras.end(),
                  [](const ExtraCandidate &lhs, const ExtraCandidate &rhs) {
                      return lhs.score < rhs.score;
                  });
        for (const ExtraCandidate &extra : extras) {
            if (static_cast<int>(filtered.candidates.size()) >= min_tour_candidates) {
                break;
            }
            if (extra.index < 0 ||
                extra.index >= static_cast<int>(candidate_set.candidates.size())) {
                continue;
            }
            filtered.candidates.push_back(
                    candidate_set.candidates[static_cast<size_t>(extra.index)]);
            ++prefix_extra_count;
        }
    }
    filtered.reachable_candidate_count =
            static_cast<int>(filtered.candidates.size());
    if (!filtered.candidates.empty()) {
        const int selected_best_index = selected_sector->best_index;
        if (selected_best_index >= 0 &&
            selected_best_index < static_cast<int>(candidate_set.candidates.size()) &&
            sectorKeyForGoal(candidate_set.candidates[static_cast<size_t>(selected_best_index)]) ==
                    active_sector_.key) {
            filtered.suggested_goal =
                    candidate_set.candidates[static_cast<size_t>(selected_best_index)];
        } else {
            filtered.suggested_goal =
                    *std::min_element(filtered.candidates.begin(),
                                      filtered.candidates.end(),
                                      [](const ExplorationGoal &lhs,
                                         const ExplorationGoal &rhs) {
                                          return lhs.score < rhs.score;
                                      });
        }
        ++active_sector_filter_count_;
    }

    std::ostringstream oss;
    oss << active_sector_.key
        << ",generation=" << active_sector_.generation
        << ",candidates=" << filtered.candidates.size()
        << ",tour_prefix_extras=" << prefix_extra_count
        << ",score=" << active_sector_.score
        << ",memory_penalty=" << sectorMemoryPenalty(active_sector_.key, stamp)
        << (keep_active_sector ? ",kept=1" : ",kept=0");
    reason = oss.str();
    return filtered.candidates.empty() ? candidate_set : filtered;
}

void ExplorationRuntimeManager::invalidateActiveTour(const std::string &reason)
{
    if (active_tour_.valid) {
        ++active_tour_invalid_count_;
    }
    const int generation = active_tour_.generation;
    active_tour_ = ActiveTour{};
    active_tour_.generation = generation;
    active_tour_.invalid_reason = reason;
}

void ExplorationRuntimeManager::invalidateActiveSector(const std::string &reason)
{
    if (active_sector_.valid) {
        ++active_sector_invalid_count_;
    }
    const int generation = active_sector_.generation;
    active_sector_ = ActiveSector{};
    active_sector_.generation = generation;
    active_sector_.invalid_reason = reason;
    invalidateActiveTour("sector_invalidated:" + reason);
}

void ExplorationRuntimeManager::ensureActiveTourState()
{
    const size_t size = active_tour_.goals.size();
    active_tour_.node_status.resize(size, ActiveTour::NodeStatus::PENDING);
    active_tour_.node_failures.resize(size, 0);
    active_tour_.node_enter_stamp.resize(size, 0.0);
    active_tour_.node_exit_stamp.resize(size, 0.0);
    if (active_tour_.cursor < 0) {
        active_tour_.cursor = 0;
    }
    if (active_tour_.cursor > static_cast<int>(size)) {
        active_tour_.cursor = static_cast<int>(size);
    }
    if (active_tour_.executing_rank >= static_cast<int>(size)) {
        active_tour_.executing_rank = -1;
    }
}

void ExplorationRuntimeManager::markTourNodeCompleted(const int rank,
                                                      const double stamp)
{
    if (!active_tour_.valid ||
        rank < 0 ||
        rank >= static_cast<int>(active_tour_.goals.size())) {
        return;
    }
    ensureActiveTourState();
    ActiveTour::NodeStatus &status =
            active_tour_.node_status[static_cast<size_t>(rank)];
    if (status == ActiveTour::NodeStatus::COMPLETED ||
        status == ActiveTour::NodeStatus::FAILED ||
        status == ActiveTour::NodeStatus::SKIPPED) {
        return;
    }
    status = ActiveTour::NodeStatus::COMPLETED;
    active_tour_.node_exit_stamp[static_cast<size_t>(rank)] = stamp;
    frontier_db_.markCompleted(active_tour_.goals[static_cast<size_t>(rank)], stamp);
    if (active_tour_.executing_rank == rank) {
        active_tour_.executing_rank = -1;
    }
    ++active_tour_node_completed_count_;
}

void ExplorationRuntimeManager::markTourNodeSkipped(const int rank,
                                                    const double stamp)
{
    if (!active_tour_.valid ||
        rank < 0 ||
        rank >= static_cast<int>(active_tour_.goals.size())) {
        return;
    }
    ensureActiveTourState();
    ActiveTour::NodeStatus &status =
            active_tour_.node_status[static_cast<size_t>(rank)];
    if (status == ActiveTour::NodeStatus::COMPLETED ||
        status == ActiveTour::NodeStatus::FAILED ||
        status == ActiveTour::NodeStatus::SKIPPED) {
        return;
    }
    status = ActiveTour::NodeStatus::SKIPPED;
    active_tour_.node_exit_stamp[static_cast<size_t>(rank)] = stamp;
    if (active_tour_.executing_rank == rank) {
        active_tour_.executing_rank = -1;
    }
    if (rank <= active_tour_.cursor) {
        active_tour_.cursor = rank + 1;
        ++active_tour_advance_count_;
    }
    ++active_tour_node_skipped_count_;
    if (pendingTourNodeCount() + executingTourNodeCount() <= 0) {
        invalidateActiveTour("all_tour_nodes_terminal");
    }
}

bool ExplorationRuntimeManager::advanceCompletedTourNodes(
        const general_utils::Vec3f &robot_pos,
        const double stamp)
{
    if (!active_tour_.valid || active_tour_.goals.empty()) {
        return false;
    }
    ensureActiveTourState();
    bool advanced = false;
    const double reached_distance =
            std::max({cfg_.exploration_goal_reached_distance,
                      0.5 * cfg_.exploration_coverage_revisit_radius,
                      0.35});
    while (active_tour_.cursor < static_cast<int>(active_tour_.goals.size())) {
        const size_t rank = static_cast<size_t>(active_tour_.cursor);
        ActiveTour::NodeStatus status = active_tour_.node_status[rank];
        if (status == ActiveTour::NodeStatus::COMPLETED ||
            status == ActiveTour::NodeStatus::SKIPPED ||
            status == ActiveTour::NodeStatus::FAILED) {
            ++active_tour_.cursor;
            ++active_tour_advance_count_;
            advanced = true;
            continue;
        }
        if (robot_pos.allFinite() &&
            active_tour_.goals[rank].position.allFinite() &&
            (active_tour_.goals[rank].position - robot_pos).norm() <=
                    reached_distance) {
            markTourNodeCompleted(active_tour_.cursor, stamp);
            ++active_tour_.cursor;
            ++active_tour_advance_count_;
            advanced = true;
            continue;
        }
        break;
    }
    if (active_tour_.cursor >= static_cast<int>(active_tour_.goals.size())) {
        invalidateActiveTour("tour_prefix_exhausted");
    }
    return advanced;
}

void ExplorationRuntimeManager::markTourNodeExecuting(const ExplorationGoal &goal,
                                                      const double stamp)
{
    if (!active_tour_.valid || active_tour_.goals.empty() || !goal.valid) {
        return;
    }
    ensureActiveTourState();
    int rank = -1;
    if (goal.identity.tour_key == active_tour_.tour_key &&
        goal.identity.tour_rank >= 0 &&
        goal.identity.tour_rank < static_cast<int>(active_tour_.goals.size())) {
        rank = goal.identity.tour_rank;
    }
    const std::string goal_key = tourGoalKey(goal);
    const double match_radius =
            std::max({cfg_.exploration_active_tour_match_radius,
                      cfg_.exploration_goal_reached_distance,
                      cfg_.exploration_coverage_revisit_radius,
                      0.75});
    if (rank < 0) {
        for (int i = std::max(0, active_tour_.cursor);
             i < static_cast<int>(active_tour_.goals.size());
             ++i) {
            const ExplorationGoal &tour_goal =
                    active_tour_.goals[static_cast<size_t>(i)];
            const std::string tour_key = tourGoalKey(tour_goal);
            const bool same_key =
                    !goal_key.empty() && !tour_key.empty() && goal_key == tour_key;
            const bool same_frontier =
                    goal.frontier_id >= 0 &&
                    tour_goal.frontier_id >= 0 &&
                    goal.frontier_id == tour_goal.frontier_id;
            const bool same_region =
                    goal.position.allFinite() &&
                    tour_goal.position.allFinite() &&
                    (goal.position - tour_goal.position).norm() <= match_radius;
            if (same_key || same_frontier || same_region) {
                rank = i;
                break;
            }
        }
    }
    if (rank < 0) {
        return;
    }
    ActiveTour::NodeStatus &status =
            active_tour_.node_status[static_cast<size_t>(rank)];
    if (status == ActiveTour::NodeStatus::COMPLETED ||
        status == ActiveTour::NodeStatus::FAILED ||
        status == ActiveTour::NodeStatus::SKIPPED) {
        return;
    }
    status = ActiveTour::NodeStatus::EXECUTING;
    active_tour_.executing_rank = rank;
    if (active_tour_.node_enter_stamp[static_cast<size_t>(rank)] <= 0.0) {
        active_tour_.node_enter_stamp[static_cast<size_t>(rank)] = stamp;
    }
    active_tour_.cursor = std::min(active_tour_.cursor, rank);
}

void ExplorationRuntimeManager::markTourNodeFailed(const ExplorationGoal &goal,
                                                   const double stamp)
{
    if (!active_tour_.valid || active_tour_.goals.empty() || !goal.valid) {
        return;
    }
    ensureActiveTourState();
    const std::string failed_key = tourGoalKey(goal);
    const double match_radius =
            std::max({cfg_.exploration_goal_reached_distance,
                      cfg_.exploration_coverage_revisit_radius,
                      cfg_.exploration_active_tour_match_radius,
                      0.75});
    int rank = -1;
    for (int i = 0; i < static_cast<int>(active_tour_.goals.size()); ++i) {
        const ExplorationGoal &tour_goal = active_tour_.goals[static_cast<size_t>(i)];
        const std::string tour_key = tourGoalKey(tour_goal);
        const bool same_key =
                !failed_key.empty() && !tour_key.empty() && failed_key == tour_key;
        const bool same_frontier =
                goal.frontier_id >= 0 &&
                tour_goal.frontier_id >= 0 &&
                goal.frontier_id == tour_goal.frontier_id;
        const bool same_region =
                goal.position.allFinite() &&
                tour_goal.position.allFinite() &&
                (goal.position - tour_goal.position).norm() <= match_radius;
        if (same_key || same_frontier || same_region) {
            rank = i;
            break;
        }
    }
    if (rank < 0) {
        return;
    }
    const size_t index = static_cast<size_t>(rank);
    ++active_tour_.node_failures[index];
    const int max_failures = std::max(1, cfg_.exploration_tour_max_node_failures);
    if (active_tour_.node_failures[index] < max_failures) {
        return;
    }
    ActiveTour::NodeStatus &status = active_tour_.node_status[index];
    if (status != ActiveTour::NodeStatus::FAILED) {
        status = ActiveTour::NodeStatus::FAILED;
        active_tour_.node_exit_stamp[index] = stamp;
        ++active_tour_node_failed_count_;
    }
    if (active_tour_.executing_rank == rank) {
        active_tour_.executing_rank = -1;
    }
    if (rank <= active_tour_.cursor) {
        active_tour_.cursor = rank + 1;
        ++active_tour_advance_count_;
    }
    if (pendingTourNodeCount() + executingTourNodeCount() <= 0) {
        invalidateActiveTour("all_tour_nodes_terminal");
    }
}

int ExplorationRuntimeManager::pendingTourNodeCount() const
{
    int count = 0;
    for (const ActiveTour::NodeStatus status : active_tour_.node_status) {
        if (status == ActiveTour::NodeStatus::PENDING) {
            ++count;
        }
    }
    return count;
}

int ExplorationRuntimeManager::executingTourNodeCount() const
{
    int count = 0;
    for (const ActiveTour::NodeStatus status : active_tour_.node_status) {
        if (status == ActiveTour::NodeStatus::EXECUTING) {
            ++count;
        }
    }
    return count;
}

int ExplorationRuntimeManager::completedTourNodeCount() const
{
    int count = 0;
    for (const ActiveTour::NodeStatus status : active_tour_.node_status) {
        if (status == ActiveTour::NodeStatus::COMPLETED) {
            ++count;
        }
    }
    return count;
}

int ExplorationRuntimeManager::failedTourNodeCount() const
{
    int count = 0;
    for (const ActiveTour::NodeStatus status : active_tour_.node_status) {
        if (status == ActiveTour::NodeStatus::FAILED) {
            ++count;
        }
    }
    return count;
}

std::string ExplorationRuntimeManager::tourGoalKey(const ExplorationGoal &goal) const
{
    nhbp::NavIdentity identity = normalizedIdentity(goal);
    const std::string canonical = identity.canonicalKey();
    if (!canonical.empty()) {
        return canonical;
    }
    const std::string frontier = identity.frontierIdentityKey();
    if (!frontier.empty()) {
        return frontier;
    }
    if (!goal.memory_key.empty()) {
        return "candidate:" + goal.memory_key;
    }
    if (goal.position.allFinite()) {
        return nhbp::quantizedPositionKey(
                goal.position,
                std::max(0.25, cfg_.exploration_coverage_grid_resolution),
                "goal");
    }
    return {};
}

bool ExplorationRuntimeManager::candidateMatchesTourGoal(
        const ExplorationGoal &candidate,
        const ExplorationGoal &tour_goal,
        const double match_radius) const
{
    if (!candidate.valid) {
        return false;
    }
    const std::string target_key = tourGoalKey(tour_goal);
    const std::string candidate_key = tourGoalKey(candidate);
    const bool same_key =
            !target_key.empty() &&
            !candidate_key.empty() &&
            target_key == candidate_key;
    const bool same_frontier =
            tour_goal.frontier_id >= 0 &&
            candidate.frontier_id >= 0 &&
            tour_goal.frontier_id == candidate.frontier_id;
    const double distance =
            tour_goal.position.allFinite() && candidate.position.allFinite()
                    ? (tour_goal.position - candidate.position).norm()
                    : std::numeric_limits<double>::infinity();
    const bool same_region =
            distance <= std::max(0.0, match_radius) &&
            (same_frontier || target_key.empty() || candidate_key.empty());
    return same_key || same_frontier || same_region;
}

bool ExplorationRuntimeManager::findActiveTourCandidate(
        const ExplorationCandidateSet &candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double stamp,
        ExplorationGoal &goal,
        std::string &reason)
{
    goal = ExplorationGoal{};
    reason.clear();
    if (!active_tour_.valid || active_tour_.goals.empty()) {
        return false;
    }
    ensureActiveTourState();
    advanceCompletedTourNodes(robot_pos, stamp);
    if (active_tour_.cursor >= static_cast<int>(active_tour_.goals.size())) {
        invalidateActiveTour("tour_prefix_exhausted");
        return false;
    }

    const ExplorationGoal &tour_goal =
            active_tour_.goals[static_cast<size_t>(active_tour_.cursor)];
    const ActiveTour::NodeStatus target_status =
            active_tour_.node_status[static_cast<size_t>(active_tour_.cursor)];
    if (target_status == ActiveTour::NodeStatus::COMPLETED ||
        target_status == ActiveTour::NodeStatus::FAILED ||
        target_status == ActiveTour::NodeStatus::SKIPPED) {
        ++active_tour_.cursor;
        ++active_tour_advance_count_;
        return findActiveTourCandidate(candidate_set, robot_pos, stamp, goal, reason);
    }
    const double match_radius =
            std::max({cfg_.exploration_goal_reached_distance,
                      cfg_.exploration_frontier_cluster_radius,
                      cfg_.exploration_coverage_revisit_radius,
                      cfg_.exploration_active_tour_match_radius,
                      0.75});
    int best_index = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(candidate_set.candidates.size()); ++i) {
        const ExplorationGoal &candidate =
                candidate_set.candidates[static_cast<size_t>(i)];
        if (!candidateMatchesTourGoal(candidate, tour_goal, match_radius)) {
            continue;
        }
        const double distance =
                tour_goal.position.allFinite() && candidate.position.allFinite()
                        ? (tour_goal.position - candidate.position).norm()
                        : std::numeric_limits<double>::infinity();
        if (distance < best_distance) {
            best_distance = distance;
            best_index = i;
        }
    }
    if (best_index < 0) {
        const bool prefix_commit_active =
                target_status == ActiveTour::NodeStatus::EXECUTING &&
                active_tour_.node_enter_stamp[static_cast<size_t>(active_tour_.cursor)] > 0.0 &&
                stamp - active_tour_.node_enter_stamp[static_cast<size_t>(active_tour_.cursor)] <=
                        std::max(0.0, cfg_.exploration_tour_prefix_commit_duration);
        const nhbp::NavIdentity target_identity = normalizedIdentity(tour_goal);
        const std::string target_blacklist_key = target_identity.blacklistKey();
        const bool frontier_object_active = frontier_db_.goalActive(tour_goal, stamp);
        const bool prefix_blocked =
                (!target_blacklist_key.empty() &&
                 navigation_memory_.isBlacklisted(target_blacklist_key, stamp)) ||
                frontier_memory_.blocked(tour_goal, stamp) ||
                recoveryBlockedByRecentTrap(tour_goal, stamp) ||
                !frontier_object_active;
        if (prefix_commit_active && !prefix_blocked) {
            goal = tour_goal;
            goal.identity.tour_key = active_tour_.tour_key;
            goal.identity.tour_rank = active_tour_.cursor;
            reason = "cursor=" + std::to_string(active_tour_.cursor) +
                     ",prefix_commit_hold=1,age=" +
                     std::to_string(
                             std::max(0.0, stamp - active_tour_.created_stamp));
            return true;
        }
        markTourNodeSkipped(active_tour_.cursor, stamp);
        reason = prefix_blocked ? "prefix_blocked_or_inactive"
                                : "prefix_candidate_missing_revalidate_failed";
        return false;
    }

    goal = candidate_set.candidates[static_cast<size_t>(best_index)];
    goal.identity.tour_key = active_tour_.tour_key;
    goal.identity.tour_rank = active_tour_.cursor;
    reason = "cursor=" + std::to_string(active_tour_.cursor) +
             ",match_distance=" + std::to_string(best_distance) +
             ",age=" + std::to_string(std::max(0.0, stamp - active_tour_.created_stamp));
    return true;
}

bool ExplorationRuntimeManager::repairActiveTourFromCandidates(
        const ExplorationCandidateSet &candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double stamp,
        std::string &reason)
{
    reason.clear();
    if (!active_tour_.valid ||
        active_tour_.goals.empty() ||
        candidate_set.candidates.empty()) {
        return false;
    }
    ensureActiveTourState();

    const double match_radius =
            std::max({cfg_.exploration_active_tour_match_radius,
                      cfg_.exploration_frontier_cluster_radius,
                      cfg_.exploration_coverage_revisit_radius,
                      0.75});
    const std::string active_sector_key =
            activeSectorEnabled() && active_sector_.valid
                    ? active_sector_.key
                    : active_tour_.sector_key;

    std::vector<bool> used(candidate_set.candidates.size(), false);
    general_utils::vec_E<ExplorationGoal> repaired_goals;
    std::vector<ActiveTour::NodeStatus> repaired_status;
    std::vector<int> repaired_failures;
    std::vector<double> repaired_enter_stamp;
    std::vector<double> repaired_exit_stamp;
    repaired_goals.reserve(active_tour_.goals.size());
    repaired_status.reserve(active_tour_.goals.size());
    repaired_failures.reserve(active_tour_.goals.size());
    repaired_enter_stamp.reserve(active_tour_.goals.size());
    repaired_exit_stamp.reserve(active_tour_.goals.size());

    const int start_rank =
            std::clamp(active_tour_.cursor,
                       0,
                       static_cast<int>(active_tour_.goals.size()));
    for (int rank = start_rank;
         rank < static_cast<int>(active_tour_.goals.size());
         ++rank) {
        const ActiveTour::NodeStatus old_status =
                active_tour_.node_status[static_cast<size_t>(rank)];
        if (old_status == ActiveTour::NodeStatus::COMPLETED ||
            old_status == ActiveTour::NodeStatus::FAILED ||
            old_status == ActiveTour::NodeStatus::SKIPPED) {
            continue;
        }
        const ExplorationGoal &old_goal =
                active_tour_.goals[static_cast<size_t>(rank)];
        const std::string old_key = tourGoalKey(old_goal);
        const std::string old_sector = sectorKeyForGoal(old_goal);
        int best_index = -1;
        int best_priority = 100;
        double best_distance = std::numeric_limits<double>::infinity();

        for (int i = 0; i < static_cast<int>(candidate_set.candidates.size()); ++i) {
            if (used[static_cast<size_t>(i)]) {
                continue;
            }
            const ExplorationGoal &candidate =
                    candidate_set.candidates[static_cast<size_t>(i)];
            if (!candidate.valid || !candidate.position.allFinite()) {
                continue;
            }
            const std::string candidate_sector = sectorKeyForGoal(candidate);
            if (!active_sector_key.empty() && candidate_sector != active_sector_key) {
                continue;
            }
            const std::string candidate_key = tourGoalKey(candidate);
            const bool same_key =
                    !old_key.empty() &&
                    !candidate_key.empty() &&
                    old_key == candidate_key;
            const bool same_frontier =
                    old_goal.frontier_id >= 0 &&
                    candidate.frontier_id >= 0 &&
                    old_goal.frontier_id == candidate.frontier_id;
            const double distance =
                    old_goal.position.allFinite()
                            ? (old_goal.position - candidate.position).norm()
                            : std::numeric_limits<double>::infinity();
            const bool same_sector =
                    !old_sector.empty() && old_sector == candidate_sector;
            int priority = 100;
            if (same_key) {
                priority = 0;
            } else if (same_frontier) {
                priority = 1;
            } else if (same_sector && distance <= match_radius) {
                priority = 2;
            }
            if (priority >= 100) {
                continue;
            }
            if (priority < best_priority ||
                (priority == best_priority && distance < best_distance)) {
                best_priority = priority;
                best_distance = distance;
                best_index = i;
            }
        }

        if (best_index < 0) {
            continue;
        }
        used[static_cast<size_t>(best_index)] = true;
        ExplorationGoal repaired =
                candidate_set.candidates[static_cast<size_t>(best_index)];
        repaired.identity.tour_key = active_tour_.tour_key;
        repaired.identity.tour_rank = static_cast<int>(repaired_goals.size());
        repaired.reason += " tour_repaired_from_rank=" + std::to_string(rank);
        repaired_goals.push_back(repaired);
        repaired_status.push_back(old_status);
        repaired_failures.push_back(active_tour_.node_failures[static_cast<size_t>(rank)]);
        repaired_enter_stamp.push_back(
                active_tour_.node_enter_stamp[static_cast<size_t>(rank)]);
        repaired_exit_stamp.push_back(
                active_tour_.node_exit_stamp[static_cast<size_t>(rank)]);
    }

    if (repaired_goals.empty()) {
        reason = "no_repairable_tour_nodes";
        return false;
    }

    for (int rank = 0; rank < static_cast<int>(repaired_goals.size()); ++rank) {
        repaired_goals[static_cast<size_t>(rank)].identity.tour_rank = rank;
    }

    active_tour_.goals = repaired_goals;
    active_tour_.node_status = repaired_status;
    active_tour_.node_failures = repaired_failures;
    active_tour_.node_enter_stamp = repaired_enter_stamp;
    active_tour_.node_exit_stamp = repaired_exit_stamp;
    active_tour_.cursor = 0;
    active_tour_.executing_rank = -1;
    for (int rank = 0; rank < static_cast<int>(active_tour_.node_status.size()); ++rank) {
        if (active_tour_.node_status[static_cast<size_t>(rank)] ==
            ActiveTour::NodeStatus::EXECUTING) {
            active_tour_.executing_rank = rank;
            break;
        }
    }
    active_tour_.last_rebuild_stamp = stamp;
    active_tour_.valid = true;
    if (activeSectorEnabled() && active_sector_.valid) {
        active_tour_.sector_key = active_sector_.key;
    }
    ++active_tour_repair_count_;
    reason = "repaired_nodes=" + std::to_string(active_tour_.goals.size()) +
             ",sector=" + active_tour_.sector_key;
    return true;
}

double ExplorationRuntimeManager::coverageIntentReward(
        const ExplorationGoal &goal,
        const double stamp) const
{
    if (!cfg_.exploration_coverage_intent_enable) {
        return 0.0;
    }
    return std::max(0.0, cfg_.exploration_coverage_intent_weight) *
           coverage_grid_.intentReward(sectorReference(goal), stamp);
}

general_utils::vec_E<ExplorationFrontierDB::ObjectSnapshot>
ExplorationRuntimeManager::selectLiveFrontierObjectsForTour(
        const ExplorationCandidateSet &candidate_set,
        const double stamp) const
{
    general_utils::vec_E<ExplorationFrontierDB::ObjectSnapshot> live_objects;
    if (!candidate_set.valid || candidate_set.candidates.empty()) {
        return live_objects;
    }

    const general_utils::vec_E<ExplorationFrontierDB::ObjectSnapshot> db_objects =
            frontier_db_.activeObjects(stamp);
    if (db_objects.empty()) {
        return live_objects;
    }

    const double match_radius =
            std::max({cfg_.exploration_active_tour_match_radius,
                      cfg_.exploration_frontier_manager_match_radius,
                      cfg_.exploration_frontier_cluster_radius,
                      cfg_.exploration_coverage_revisit_radius,
                      0.75});

    for (const ExplorationFrontierDB::ObjectSnapshot &db_object : db_objects) {
        if (db_object.key.empty()) {
            continue;
        }

        ExplorationFrontierDB::ObjectSnapshot object = db_object;
        object.viewpoints.clear();
        object.best_score = std::numeric_limits<double>::infinity();
        object.total_gain = 0.0;
        object.max_gain = 0.0;
        object.candidate_count = 0;
        object.expansion_count = 0;

        std::unordered_set<std::string> emitted;
        for (const ExplorationGoal &candidate : candidate_set.candidates) {
            if (!candidate.valid || !candidate.position.allFinite()) {
                continue;
            }
            const std::string candidate_sector = sectorKeyForGoal(candidate);
            const std::string candidate_object_key =
                    frontier_db_.objectKeyForGoal(candidate, candidate_sector);
            const bool same_key =
                    !candidate_object_key.empty() &&
                    candidate_object_key == db_object.key;
            const bool same_frontier =
                    candidate.frontier_id >= 0 &&
                    db_object.key == "frontier:" + std::to_string(candidate.frontier_id);
            const double reference_distance =
                    db_object.reference.allFinite()
                            ? (sectorReference(candidate) - db_object.reference).norm()
                            : std::numeric_limits<double>::infinity();
            const bool same_region =
                    reference_distance <= match_radius &&
                    (candidate.frontier_center_valid || candidate.frontier_id < 0);
            if (!same_key && !same_frontier && !same_region) {
                continue;
            }

            ExplorationGoal live_viewpoint = candidate;
            if (live_viewpoint.identity.frontier_key.empty()) {
                live_viewpoint.identity.frontier_key = db_object.key;
            }
            live_viewpoint.reason += " live_frontier_object=" + db_object.key;
            const std::string viewpoint_key = tourGoalKey(live_viewpoint);
            const std::string dedup_key =
                    !viewpoint_key.empty()
                            ? viewpoint_key
                            : nhbp::quantizedPositionKey(
                                      live_viewpoint.position,
                                      std::max(0.25,
                                               cfg_.exploration_coverage_grid_resolution),
                                      "live_viewpoint");
            if (!dedup_key.empty() && emitted.find(dedup_key) != emitted.end()) {
                continue;
            }
            if (!dedup_key.empty()) {
                emitted.insert(dedup_key);
            }

            object.viewpoints.push_back(live_viewpoint);
            object.best_score = std::min(object.best_score, live_viewpoint.score);
            object.total_gain += std::max(0.0, live_viewpoint.information_gain);
            object.max_gain =
                    std::max(object.max_gain,
                             std::max(0.0, live_viewpoint.information_gain));
            ++object.candidate_count;
            if (live_viewpoint.identity.intent_mode == "exploration_expansion" ||
                live_viewpoint.reason.find("expansion") != std::string::npos) {
                ++object.expansion_count;
            }
        }

        if (object.viewpoints.empty()) {
            continue;
        }
        if (!std::isfinite(object.best_score)) {
            object.best_score = db_object.best_score;
        }
        object.expansion_only =
                object.candidate_count > 0 &&
                object.expansion_count == object.candidate_count;
        std::sort(object.viewpoints.begin(),
                  object.viewpoints.end(),
                  [](const ExplorationGoal &lhs, const ExplorationGoal &rhs) {
                      if (lhs.score != rhs.score) {
                          return lhs.score < rhs.score;
                      }
                      return lhs.information_gain > rhs.information_gain;
                  });
        const int keep =
                std::max(1, cfg_.exploration_task_planner_viewpoints_per_frontier);
        if (static_cast<int>(object.viewpoints.size()) > keep) {
            object.viewpoints.resize(static_cast<size_t>(keep));
        }
        live_objects.push_back(std::move(object));
    }

    std::sort(live_objects.begin(),
              live_objects.end(),
              [](const ExplorationFrontierDB::ObjectSnapshot &lhs,
                 const ExplorationFrontierDB::ObjectSnapshot &rhs) {
                  if (lhs.best_score != rhs.best_score) {
                      return lhs.best_score < rhs.best_score;
                  }
                  return lhs.total_gain > rhs.total_gain;
              });
    return live_objects;
}

bool ExplorationRuntimeManager::rebuildActiveTour(
        const ExplorationCandidateSet &candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double current_yaw,
        const double stamp,
        std::string &reason)
{
    reason.clear();
    if (candidate_set.candidates.empty()) {
        reason = "empty_candidate_set";
        return false;
    }

    const general_utils::vec_E<ExplorationFrontierDB::ObjectSnapshot> frontier_objects =
            selectLiveFrontierObjectsForTour(candidate_set, stamp);
    ExplorationTaskPlanner::Request task_request;
    task_request.candidate_set = &candidate_set;
    task_request.frontier_objects = &frontier_objects;
    task_request.robot_pos = robot_pos;
    task_request.current_yaw = current_yaw;
    task_request.stamp = stamp;
    task_request.pairwise_cost =
            [this, stamp](const ExplorationGoal &from, const ExplorationGoal &to) {
                return tourPairwiseCandidateCost(from, to, stamp);
            };
    task_request.start_cost =
            [this, &robot_pos, current_yaw, stamp](const ExplorationGoal &goal) {
                const double travel_cost =
                        std::isfinite(goal.travel_cost)
                                ? goal.travel_cost
                                : (goal.position - robot_pos).norm();
                const double yaw_cost = yawDistance(current_yaw, goal.yaw);
                const std::string sector_key = sectorKeyForGoal(goal);
                const double coverage_penalty =
                        std::max(0.0, cfg_.exploration_tour_coverage_penalty_weight) *
                        coverage_grid_.revisitPenalty(sectorReference(goal), stamp);
                const double memory_penalty = sectorMemoryPenalty(sector_key, stamp);
                const double coverage_reward = coverageIntentReward(goal, stamp);
                return std::max(0.0,
                       travel_cost +
                       cfg_.exploration_weight_yaw * yaw_cost +
                       cfg_.exploration_weight_curvature * goal.curvature_cost +
                       coverage_penalty +
                       memory_penalty -
                       coverage_reward);
            };
    task_request.node_penalty =
            [this, stamp](const ExplorationGoal &goal) {
                const std::string sector_key = sectorKeyForGoal(goal);
                return std::max(0.0, cfg_.exploration_tour_coverage_penalty_weight) *
                               coverage_grid_.revisitPenalty(sectorReference(goal), stamp) +
                       sectorMemoryPenalty(sector_key, stamp);
            };
    task_request.goal_key =
            [this](const ExplorationGoal &goal) {
                return tourGoalKey(goal);
            };
    task_request.sector_key =
            [this](const ExplorationGoal &goal) {
                return sectorKeyForGoal(goal);
            };
    task_request.sector_reference =
            [this](const ExplorationGoal &goal) {
                return sectorReference(goal);
            };

    ExplorationTaskPlanner::Plan task_plan;
    general_utils::vec_E<ExplorationGoal> ordered_goals;
    if (task_planner_.plan(task_request, task_plan)) {
        ordered_goals = task_plan.ordered_goals;
        reason = task_plan.reason +
                 ",live_objects=" + std::to_string(frontier_objects.size());
    } else {
        const int candidate_num =
                std::min(static_cast<int>(candidate_set.candidates.size()),
                         std::max(1, cfg_.exploration_atsp_max_candidate_num));
        std::vector<int> ordered_indices(static_cast<size_t>(candidate_num));
        for (int i = 0; i < candidate_num; ++i) {
            ordered_indices[static_cast<size_t>(i)] = i;
        }
        std::sort(ordered_indices.begin(),
                  ordered_indices.end(),
                  [&candidate_set](const int lhs, const int rhs) {
                      return candidate_set.candidates[static_cast<size_t>(lhs)].score <
                             candidate_set.candidates[static_cast<size_t>(rhs)].score;
                  });
        ordered_goals.reserve(ordered_indices.size());
        for (const int index : ordered_indices) {
            ordered_goals.push_back(candidate_set.candidates[static_cast<size_t>(index)]);
        }
        reason = task_plan.reason.empty()
                         ? "task_planner_fallback_score_order"
                         : task_plan.reason + "_fallback_score_order";
        reason += ",live_objects=" + std::to_string(frontier_objects.size());
    }
    if (ordered_goals.empty()) {
        reason = reason.empty() ? "empty_task_tour" : reason + "_empty_task_tour";
        return false;
    }

    ActiveTour rebuilt;
    rebuilt.cursor = 0;
    rebuilt.generation = active_tour_.generation + 1;
    rebuilt.created_stamp = stamp;
    rebuilt.last_rebuild_stamp = stamp;
    rebuilt.valid = true;
    rebuilt.goals.reserve(ordered_goals.size());
    rebuilt.node_status.reserve(ordered_goals.size());
    rebuilt.node_failures.reserve(ordered_goals.size());
    rebuilt.node_enter_stamp.reserve(ordered_goals.size());
    rebuilt.node_exit_stamp.reserve(ordered_goals.size());

    std::ostringstream key;
    key << "tour:" << rebuilt.generation << ":" << reason;
    std::unordered_set<std::string> used_goal_keys;
    int duplicate_goal_count = 0;
    for (int rank = 0; rank < static_cast<int>(ordered_goals.size()); ++rank) {
        ExplorationGoal goal = ordered_goals[static_cast<size_t>(rank)];
        const std::string goal_key = tourGoalKey(goal);
        const std::string stable_goal_key =
                goal_key.empty()
                        ? nhbp::quantizedPositionKey(
                                  goal.position,
                                  std::max(0.25, cfg_.exploration_coverage_grid_resolution),
                                  "goal")
                        : goal_key;
        if (!stable_goal_key.empty() &&
            used_goal_keys.find(stable_goal_key) != used_goal_keys.end()) {
            ++duplicate_goal_count;
            continue;
        }
        if (!stable_goal_key.empty()) {
            used_goal_keys.insert(stable_goal_key);
        }
        goal.identity.tour_rank = static_cast<int>(rebuilt.goals.size());
        key << "|" << (goal_key.empty() ? std::to_string(goal.candidate_id) : goal_key);
        rebuilt.goals.push_back(goal);
        rebuilt.node_status.push_back(ActiveTour::NodeStatus::PENDING);
        rebuilt.node_failures.push_back(0);
        rebuilt.node_enter_stamp.push_back(0.0);
        rebuilt.node_exit_stamp.push_back(0.0);
    }
    rebuilt.tour_key = key.str();
    if (!rebuilt.goals.empty()) {
        const general_utils::Vec3f sector_position =
                rebuilt.goals.front().frontier_center_valid
                        ? rebuilt.goals.front().frontier_center
                        : rebuilt.goals.front().position;
        rebuilt.sector_key = activeSectorEnabled() && active_sector_.valid
                                     ? active_sector_.key
                                     : nhbp::quantizedPositionKey(
                                               sector_position,
                                               activeSectorResolution(),
                                               "sector");
        for (int rank = 0; rank < static_cast<int>(rebuilt.goals.size()); ++rank) {
            rebuilt.goals[static_cast<size_t>(rank)].identity.tour_key =
                    rebuilt.tour_key;
            rebuilt.goals[static_cast<size_t>(rank)].identity.tour_rank = rank;
        }
    }
    if (duplicate_goal_count > 0) {
        reason += ",dedup=" + std::to_string(duplicate_goal_count);
    }

    active_tour_ = rebuilt;
    ++active_tour_rebuild_count_;
    reason += ":generation=" + std::to_string(active_tour_.generation) +
              ",size=" + std::to_string(active_tour_.goals.size());
    return active_tour_.valid && !active_tour_.goals.empty();
}

double ExplorationRuntimeManager::tourPairwiseCandidateCost(
        const ExplorationGoal &from,
        const ExplorationGoal &to,
        const double stamp) const
{
    if (!from.position.allFinite() || !to.position.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    const double travel = topologyAwareTravelCost(from, to, stamp);
    const double yaw_cost = yawDistance(from.yaw, to.yaw);
    const double info_saturation =
            std::max(1.0e-6, cfg_.exploration_information_gain_saturation);
    const double effective_gain =
            std::min(std::max(0.0, to.information_gain), info_saturation);
    const double node_cost =
            cfg_.exploration_weight_yaw * yaw_cost +
            cfg_.exploration_weight_curvature * to.curvature_cost +
            cfg_.exploration_weight_info_gain * effective_gain;
    const double same_frontier_penalty =
            from.frontier_id >= 0 &&
            from.frontier_id == to.frontier_id
                    ? cfg_.exploration_frontier_cluster_radius
                    : 0.0;
    const std::string from_sector = sectorKeyForGoal(from);
    const std::string to_sector = sectorKeyForGoal(to);
    const double cross_sector_penalty =
            !from_sector.empty() &&
            !to_sector.empty() &&
            from_sector != to_sector
                    ? std::max(0.0, cfg_.exploration_tour_cross_sector_penalty)
                    : 0.0;
    const double coverage_penalty =
            std::max(0.0, cfg_.exploration_tour_coverage_penalty_weight) *
            coverage_grid_.revisitPenalty(sectorReference(to), stamp);
    const double coverage_reward = coverageIntentReward(to, stamp);
    const double sector_memory_penalty = sectorMemoryPenalty(to_sector, stamp);
    return std::max(0.0,
                    travel +
                            node_cost +
                            same_frontier_penalty +
                            cross_sector_penalty +
                            coverage_penalty +
                            sector_memory_penalty -
                            coverage_reward);
}

double ExplorationRuntimeManager::topologyAwareTravelCost(
        const ExplorationGoal &from,
        const ExplorationGoal &to,
        const double stamp) const
{
    if (!from.position.allFinite() || !to.position.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    const double euclidean = std::max(0.0, (to.position - from.position).norm());
    const double topology_weight =
            std::clamp(cfg_.exploration_tour_topology_weight, 0.0, 1.0);
    if (!cfg_.exploration_use_topological_memory || topology_weight <= 1.0e-6) {
        return euclidean;
    }
    nhbp::TopoPath topo_path;
    if (topological_memory_.searchPath(from.position, to.position, stamp, topo_path) &&
        topo_path.valid &&
        topo_path.length > 1.0e-3) {
        const double topo_length = std::max(euclidean, topo_path.length);
        return (1.0 - topology_weight) * euclidean + topology_weight * topo_length;
    }
    const bool topology_has_context =
            topological_memory_.activeNodeCount(stamp) >= 2 &&
            topological_memory_.edgeCount() > 0;
    if (!topology_has_context) {
        return euclidean;
    }
    return euclidean * (1.0 + 0.25 * topology_weight);
}

} // namespace general_planner
