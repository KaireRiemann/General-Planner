#include <general_core/exploration/exploration_global_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <general_core/nhbp/nav_identity.hpp>

namespace general_planner {
namespace {

double finiteOr(const double value, const double fallback = 0.0)
{
    return std::isfinite(value) ? value : fallback;
}

double positiveFinite(const double value, const double fallback = 0.0)
{
    return std::max(0.0, finiteOr(value, fallback));
}

double yawDistance(const double lhs, const double rhs)
{
    if (!std::isfinite(lhs) || !std::isfinite(rhs)) {
        return 0.0;
    }
    return std::abs(std::atan2(std::sin(rhs - lhs), std::cos(rhs - lhs)));
}

} // namespace

ExplorationGlobalPlanner::ExplorationGlobalPlanner(const Config &cfg)
        : cfg_(cfg),
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
          frontier_db_(cfg),
          task_planner_(cfg)
{
}

void ExplorationGlobalPlanner::reset()
{
    coverage_grid_.reset();
    frontier_db_.reset();
    task_planner_.reset();
    active_tour_ = ActiveTour{};
    observe_count_ = 0;
    rebuild_count_ = 0;
    repair_count_ = 0;
    reuse_count_ = 0;
    committed_count_ = 0;
    failed_count_ = 0;
    invalid_count_ = 0;
    last_raw_candidate_count_ = 0;
    last_active_object_count_ = 0;
    last_rebuilt_tour_size_ = 0;
    last_refresh_reason_.clear();
    last_select_reason_.clear();
    last_rebuild_reason_.clear();
}

ExplorationGlobalPlanner::RefreshResult ExplorationGlobalPlanner::refresh(
        const ExplorationCandidateSet *candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double current_yaw,
        const double stamp,
        const bool force_rebuild)
{
    RefreshResult result;
    if (!enabled()) {
        result.reason = "global_planner_disabled";
        last_refresh_reason_ = result.reason;
        return result;
    }

    coverage_grid_.observePose(robot_pos, stamp);
    result.observed_candidates =
            candidate_set != nullptr &&
            candidate_set->valid &&
            !candidate_set->candidates.empty() &&
            observeCandidates(*candidate_set, robot_pos, stamp);
    if (candidate_set != nullptr && candidate_set->valid) {
        result.raw_candidate_count =
                static_cast<int>(candidate_set->candidates.size());
        last_raw_candidate_count_ = result.raw_candidate_count;
    }

    advanceCompletedTourNodes(robot_pos, stamp);

    std::string repair_reason;
    if (candidate_set != nullptr &&
        candidate_set->valid &&
        !candidate_set->candidates.empty()) {
        result.repaired_tour =
                repairTourFromCandidates(*candidate_set, stamp, repair_reason);
    }

    const auto active_objects = frontier_db_.activeObjects(stamp);
    result.active_object_count = static_cast<int>(active_objects.size());
    last_active_object_count_ = result.active_object_count;

    const bool tour_empty =
            !active_tour_.valid ||
            pendingTourCount() + executingTourCount() <= 0;
    const bool should_rebuild =
            (force_rebuild || tour_empty) &&
            tourRebuildAllowed(stamp);

    std::string rebuild_reason;
    if (should_rebuild) {
        result.rebuilt_tour =
                rebuildTour(candidate_set,
                            robot_pos,
                            current_yaw,
                            stamp,
                            rebuild_reason);
    }

    result.pending_tour_count = pendingTourCount();
    result.executing_tour_count = executingTourCount();
    result.rebuilt_tour_size = last_rebuilt_tour_size_;
    result.updated = result.observed_candidates ||
                     result.repaired_tour ||
                     result.rebuilt_tour ||
                     active_tour_.valid;

    std::ostringstream oss;
    oss << "global_manager_refresh"
        << ":observed=" << static_cast<int>(result.observed_candidates)
        << ",raw=" << result.raw_candidate_count
        << ",objects=" << result.active_object_count
        << ",tour_valid=" << static_cast<int>(active_tour_.valid)
        << ",pending=" << result.pending_tour_count
        << ",executing=" << result.executing_tour_count
        << ",repaired=" << static_cast<int>(result.repaired_tour)
        << ",rebuilt=" << static_cast<int>(result.rebuilt_tour);
    if (!repair_reason.empty()) {
        oss << ",repair=" << repair_reason;
    }
    if (!rebuild_reason.empty()) {
        oss << ",rebuild=" << rebuild_reason;
    }
    result.reason = oss.str();
    last_refresh_reason_ = result.reason;
    return result;
}

ExplorationGlobalPlanner::Decision ExplorationGlobalPlanner::select(
        const ExplorationCandidateSet *candidate_set,
        const ExplorationGoal &frontend_goal,
        const general_utils::Vec3f &robot_pos,
        const double current_yaw,
        const double stamp,
        const bool allow_frontend_fallback)
{
    Decision decision;
    if (!enabled()) {
        decision.reason = "global_planner_disabled";
        last_select_reason_ = decision.reason;
        return decision;
    }

    RefreshResult refresh_result =
            refresh(candidate_set, robot_pos, current_yaw, stamp, false);

    ExplorationGoal tour_goal;
    std::string tour_reason;
    if (selectTourGoal(robot_pos, stamp, tour_goal, tour_reason)) {
        decision.ready = true;
        decision.from_active_tour = true;
        decision.goal = tour_goal;
        decision.reason = "global_manager_active_tour:" + tour_reason +
                          " refresh=" + refresh_result.reason;
        last_select_reason_ = decision.reason;
        ++reuse_count_;
        return decision;
    }

    if (tourRebuildAllowed(stamp)) {
        std::string rebuild_reason;
        if (rebuildTour(candidate_set,
                        robot_pos,
                        current_yaw,
                        stamp,
                        rebuild_reason) &&
            selectTourGoal(robot_pos, stamp, tour_goal, tour_reason)) {
            decision.ready = true;
            decision.from_active_tour = true;
            decision.rebuilt_tour = true;
            decision.goal = tour_goal;
            decision.reason = "global_manager_rebuilt_tour:" + rebuild_reason +
                              " select=" + tour_reason;
            last_select_reason_ = decision.reason;
            return decision;
        }
    }

    if (allow_frontend_fallback) {
        ExplorationGoal fallback =
                bestFallbackCandidate(candidate_set, frontend_goal);
        if (fallback.valid) {
            fallback.reason += " global_manager_fallback";
            decision.ready = true;
            decision.goal = fallback;
            decision.reason = "global_manager_frontend_fallback:" +
                              refresh_result.reason;
            last_select_reason_ = decision.reason;
            return decision;
        }
    }

    decision.reason = "global_manager_no_goal:" + refresh_result.reason;
    last_select_reason_ = decision.reason;
    return decision;
}

void ExplorationGlobalPlanner::recordCommitted(
        const ExplorationGoal &goal,
        const general_utils::Vec3f &robot_pos,
        const double stamp)
{
    if (!enabled() || !goal.valid) {
        return;
    }
    ++committed_count_;
    coverage_grid_.observePose(robot_pos, stamp);
    coverage_grid_.observeFrontierEvidence(sectorReference(goal),
                                           stamp,
                                           goal.visible_frontier_cell_count,
                                           goal.visible_frontier_cell_count,
                                           goal.information_gain);
    coverage_grid_.markCoveredNear(robot_pos,
                                   stamp,
                                   std::max(cfg_.exploration_coverage_revisit_radius,
                                            cfg_.exploration_frontier_memory_covered_radius));
    frontier_db_.markCommitted(goal, stamp);
    frontier_db_.markCoveredNear(robot_pos,
                                 stamp,
                                 std::max(cfg_.exploration_coverage_revisit_radius,
                                          cfg_.exploration_frontier_memory_covered_radius));
    markTourNodeExecuting(goal, stamp);

    if (goal.position.allFinite() &&
        robot_pos.allFinite() &&
        (goal.position - robot_pos).norm() <= completionRadius()) {
        frontier_db_.markCompleted(goal, stamp);
        frontier_db_.markCoveredNear(goal.position, stamp, completionRadius());
        coverage_grid_.markCoveredNear(goal.position, stamp, completionRadius());
    }
}

void ExplorationGlobalPlanner::recordFailed(const ExplorationGoal &goal,
                                            const double stamp)
{
    if (!enabled() || !goal.valid) {
        return;
    }
    ++failed_count_;
    frontier_db_.markFailed(goal, stamp);
    coverage_grid_.markNoProgressBasin(goal.position,
                                       stamp,
                                       std::max(cfg_.exploration_coverage_revisit_radius,
                                                cfg_.exploration_frontier_memory_failure_block_radius));
    markTourNodeFailed(goal, stamp);
}

bool ExplorationGlobalPlanner::hasPendingTour() const
{
    return enabled() &&
           active_tour_.valid &&
           pendingTourCount() + executingTourCount() > 0;
}

bool ExplorationGlobalPlanner::goalActive(const ExplorationGoal &goal,
                                          const double stamp) const
{
    return frontier_db_.goalActive(goal, stamp);
}

std::string ExplorationGlobalPlanner::diagnosticSummary(const double stamp) const
{
    std::ostringstream oss;
    oss << "global_frontier_records=" << frontier_db_.recordCount()
        << ";global_frontier_active=" << frontier_db_.activeObjectCount(stamp)
        << ";global_frontier_blocked=" << frontier_db_.blockedObjectCount(stamp)
        << ";global_frontier_covered=" << frontier_db_.coveredObjectCount()
        << ";global_frontier_stale=" << frontier_db_.staleObjectCount(stamp)
        << ";global_coverage_cells=" << coverage_grid_.visitedCellCount()
        << ";global_coverage_visits=" << coverage_grid_.totalVisitCount()
        << ";global_coverage_covered=" << coverage_grid_.coveredCellCount()
        << ";global_coverage_known_ratio=" << coverage_grid_.knownCoverageRatio(stamp)
        << ";global_coverage_recent_gain=" << coverage_grid_.recentInformationGain(stamp)
        << ";global_tour_valid=" << static_cast<int>(active_tour_.valid)
        << ";global_tour_generation=" << active_tour_.generation
        << ";global_tour_size=" << active_tour_.goals.size()
        << ";global_tour_cursor=" << active_tour_.cursor
        << ";global_tour_pending=" << pendingTourCount()
        << ";global_tour_executing=" << executingTourCount()
        << ";global_tour_completed=" << completedTourCount()
        << ";global_tour_failed=" << failedTourCount()
        << ";global_observe_count=" << observe_count_
        << ";global_rebuild_count=" << rebuild_count_
        << ";global_repair_count=" << repair_count_
        << ";global_reuse_count=" << reuse_count_
        << ";global_committed_count=" << committed_count_
        << ";global_failed_count=" << failed_count_
        << ";global_invalid_count=" << invalid_count_
        << ";global_last_raw_candidates=" << last_raw_candidate_count_
        << ";global_last_active_objects=" << last_active_object_count_
        << ";global_last_rebuilt_tour_size=" << last_rebuilt_tour_size_
        << ";global_last_refresh_reason=" << last_refresh_reason_
        << ";global_last_select_reason=" << last_select_reason_
        << ";global_last_rebuild_reason=" << last_rebuild_reason_;
    return oss.str();
}

bool ExplorationGlobalPlanner::enabled() const
{
    return cfg_.exploration_enable &&
           cfg_.exploration_task_planner_enable &&
           cfg_.exploration_active_tour_enable;
}

bool ExplorationGlobalPlanner::tourRebuildAllowed(const double stamp) const
{
    return !active_tour_.valid ||
           cfg_.exploration_active_tour_rebuild_min_interval <= 0.0 ||
           stamp - active_tour_.last_rebuild_stamp >=
                   cfg_.exploration_active_tour_rebuild_min_interval;
}

bool ExplorationGlobalPlanner::observeCandidates(
        const ExplorationCandidateSet &candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double stamp)
{
    if (!candidate_set.valid || candidate_set.candidates.empty()) {
        return false;
    }
    ++observe_count_;
    coverage_grid_.observePose(robot_pos, stamp);
    for (const ExplorationGoal &candidate : candidate_set.candidates) {
        if (!candidate.valid) {
            continue;
        }
        coverage_grid_.observeFrontierEvidence(
                sectorReference(candidate),
                stamp,
                candidate.visible_frontier_cell_count,
                candidate.visible_frontier_cell_count,
                candidate.information_gain);
    }

    ExplorationFrontierDB::ObservationContext context;
    context.robot_pos = robot_pos;
    context.stamp = stamp;
    context.node_penalty =
            [this, stamp](const ExplorationGoal &goal) {
                return nodePenalty(goal, stamp);
            };
    context.coverage_intent_reward =
            [this, stamp](const ExplorationGoal &goal) {
                return coverageIntentReward(goal, stamp);
            };
    context.sector_key =
            [this](const ExplorationGoal &goal) {
                return sectorKeyForGoal(goal);
            };
    context.sector_reference =
            [this](const ExplorationGoal &goal) {
                return sectorReference(goal);
            };
    frontier_db_.observeCandidates(candidate_set, context);
    return true;
}

bool ExplorationGlobalPlanner::rebuildTour(
        const ExplorationCandidateSet *candidate_set,
        const general_utils::Vec3f &robot_pos,
        const double current_yaw,
        const double stamp,
        std::string &reason)
{
    reason.clear();
    const general_utils::vec_E<ExplorationFrontierDB::ObjectSnapshot> objects =
            frontier_db_.activeObjects(stamp);
    last_active_object_count_ = static_cast<int>(objects.size());
    if (objects.empty()) {
        reason = "no_active_global_frontier_objects";
        return false;
    }

    ExplorationTaskPlanner::Request request;
    request.candidate_set = candidate_set;
    request.frontier_objects = &objects;
    request.robot_pos = robot_pos;
    request.current_yaw = current_yaw;
    request.stamp = stamp;
    request.start_cost =
            [this, &robot_pos, stamp](const ExplorationGoal &goal) {
                return startCost(goal, robot_pos, stamp);
            };
    request.pairwise_cost =
            [this, stamp](const ExplorationGoal &from,
                          const ExplorationGoal &to) {
                return pairwiseCost(from, to, stamp);
            };
    request.node_penalty =
            [this, stamp](const ExplorationGoal &goal) {
                return nodePenalty(goal, stamp);
            };
    request.goal_key =
            [this](const ExplorationGoal &goal) {
                return goalKeyForGoal(goal);
            };
    request.sector_key =
            [this](const ExplorationGoal &goal) {
                return sectorKeyForGoal(goal);
            };
    request.sector_reference =
            [this](const ExplorationGoal &goal) {
                return sectorReference(goal);
            };

    ExplorationTaskPlanner::Plan plan;
    if (!task_planner_.plan(request, plan) || plan.ordered_goals.empty()) {
        reason = plan.reason.empty() ? "task_planner_failed" : plan.reason;
        last_rebuild_reason_ = reason;
        return false;
    }

    ActiveTour next_tour;
    next_tour.goals = plan.ordered_goals;
    next_tour.status.assign(next_tour.goals.size(), NodeStatus::PENDING);
    next_tour.failures.assign(next_tour.goals.size(), 0);
    next_tour.enter_stamp.assign(next_tour.goals.size(), 0.0);
    next_tour.exit_stamp.assign(next_tour.goals.size(), 0.0);
    next_tour.cursor = 0;
    next_tour.executing_rank = -1;
    next_tour.generation = active_tour_.generation + 1;
    next_tour.created_stamp = stamp;
    next_tour.last_rebuild_stamp = stamp;
    next_tour.valid = true;
    next_tour.key = "global_tour:" + std::to_string(next_tour.generation);
    for (int i = 0; i < static_cast<int>(next_tour.goals.size()); ++i) {
        ExplorationGoal &goal = next_tour.goals[static_cast<size_t>(i)];
        goal.identity.tour_key = next_tour.key;
        goal.identity.tour_rank = i;
        goal.reason += " global_manager_tour_rank=" + std::to_string(i);
    }

    active_tour_ = std::move(next_tour);
    ++rebuild_count_;
    last_rebuilt_tour_size_ = static_cast<int>(active_tour_.goals.size());
    std::ostringstream oss;
    oss << plan.reason
        << ",tour_key=" << active_tour_.key
        << ",size=" << active_tour_.goals.size();
    reason = oss.str();
    last_rebuild_reason_ = reason;
    return true;
}

bool ExplorationGlobalPlanner::repairTourFromCandidates(
        const ExplorationCandidateSet &candidate_set,
        const double stamp,
        std::string &reason)
{
    reason.clear();
    if (!active_tour_.valid || candidate_set.candidates.empty()) {
        reason = "tour_unavailable_or_empty_candidates";
        return false;
    }

    int repaired = 0;
    for (int i = std::max(0, active_tour_.cursor);
         i < static_cast<int>(active_tour_.goals.size());
         ++i) {
        NodeStatus &status = active_tour_.status[static_cast<size_t>(i)];
        if (status != NodeStatus::PENDING &&
            status != NodeStatus::EXECUTING) {
            continue;
        }

        const ExplorationGoal original =
                active_tour_.goals[static_cast<size_t>(i)];
        const ExplorationGoal *best = nullptr;
        for (const ExplorationGoal &candidate : candidate_set.candidates) {
            if (!candidate.valid || !candidateMatchesGoal(candidate, original)) {
                continue;
            }
            if (best == nullptr || candidate.score < best->score) {
                best = &candidate;
            }
        }
        if (best == nullptr) {
            continue;
        }

        ExplorationGoal repaired_goal = *best;
        repaired_goal.identity.tour_key = active_tour_.key;
        repaired_goal.identity.tour_rank = i;
        repaired_goal.reason += " global_manager_tour_repair";
        active_tour_.goals[static_cast<size_t>(i)] = repaired_goal;
        ++repaired;
    }

    if (repaired <= 0) {
        reason = "no_matching_tour_nodes";
        return false;
    }
    ++repair_count_;
    reason = "repaired_nodes=" + std::to_string(repaired) +
             ",stamp=" + std::to_string(stamp);
    return true;
}

bool ExplorationGlobalPlanner::selectTourGoal(
        const general_utils::Vec3f &robot_pos,
        const double stamp,
        ExplorationGoal &goal,
        std::string &reason)
{
    goal = ExplorationGoal{};
    reason.clear();
    if (!active_tour_.valid || active_tour_.goals.empty()) {
        reason = "tour_unavailable";
        return false;
    }
    advanceCompletedTourNodes(robot_pos, stamp);

    for (int i = std::max(0, active_tour_.cursor);
         i < static_cast<int>(active_tour_.goals.size());
         ++i) {
        NodeStatus &status = active_tour_.status[static_cast<size_t>(i)];
        if (status != NodeStatus::PENDING &&
            status != NodeStatus::EXECUTING) {
            continue;
        }
        ExplorationGoal candidate = active_tour_.goals[static_cast<size_t>(i)];
        if (!candidate.valid || !candidate.position.allFinite()) {
            status = NodeStatus::SKIPPED;
            active_tour_.exit_stamp[static_cast<size_t>(i)] = stamp;
            continue;
        }
        if (!frontier_db_.goalActive(candidate, stamp)) {
            status = NodeStatus::COMPLETED;
            active_tour_.exit_stamp[static_cast<size_t>(i)] = stamp;
            continue;
        }
        active_tour_.cursor = i;
        active_tour_.executing_rank = i;
        status = NodeStatus::EXECUTING;
        active_tour_.enter_stamp[static_cast<size_t>(i)] =
                active_tour_.enter_stamp[static_cast<size_t>(i)] > 0.0
                        ? active_tour_.enter_stamp[static_cast<size_t>(i)]
                        : stamp;
        candidate.identity.tour_key = active_tour_.key;
        candidate.identity.tour_rank = i;
        candidate.reason += " global_manager_active_tour_select rank=" +
                            std::to_string(i);
        goal = candidate;
        reason = "tour_key=" + active_tour_.key +
                 ",rank=" + std::to_string(i) +
                 ",pending=" + std::to_string(pendingTourCount());
        return true;
    }

    invalidateTour("no_pending_tour_goal");
    reason = "no_pending_tour_goal";
    return false;
}

void ExplorationGlobalPlanner::invalidateTour(const std::string &reason)
{
    if (active_tour_.valid) {
        ++invalid_count_;
    }
    active_tour_.valid = false;
    active_tour_.invalid_reason = reason;
}

bool ExplorationGlobalPlanner::advanceCompletedTourNodes(
        const general_utils::Vec3f &robot_pos,
        const double stamp)
{
    if (!active_tour_.valid || !robot_pos.allFinite()) {
        return false;
    }
    bool advanced = false;
    const double radius = completionRadius();
    for (int i = std::max(0, active_tour_.cursor);
         i < static_cast<int>(active_tour_.goals.size());
         ++i) {
        NodeStatus &status = active_tour_.status[static_cast<size_t>(i)];
        if (status != NodeStatus::PENDING &&
            status != NodeStatus::EXECUTING) {
            if (i == active_tour_.cursor) {
                active_tour_.cursor = i + 1;
                advanced = true;
            }
            continue;
        }
        const ExplorationGoal &candidate =
                active_tour_.goals[static_cast<size_t>(i)];
        const bool reached =
                candidate.position.allFinite() &&
                (candidate.position - robot_pos).norm() <= radius;
        const bool inactive = !frontier_db_.goalActive(candidate, stamp);
        if (!reached && !inactive) {
            break;
        }
        status = NodeStatus::COMPLETED;
        active_tour_.exit_stamp[static_cast<size_t>(i)] = stamp;
        frontier_db_.markCompleted(candidate, stamp);
        frontier_db_.markCoveredNear(candidate.position, stamp, radius);
        coverage_grid_.markCoveredNear(candidate.position, stamp, radius);
        if (i == active_tour_.cursor) {
            active_tour_.cursor = i + 1;
        }
        advanced = true;
    }
    if (pendingTourCount() + executingTourCount() <= 0) {
        invalidateTour("tour_exhausted");
    }
    return advanced;
}

void ExplorationGlobalPlanner::markTourNodeExecuting(
        const ExplorationGoal &goal,
        const double stamp)
{
    if (!active_tour_.valid || !goal.valid) {
        return;
    }
    int best_rank = goal.identity.tour_rank;
    if (best_rank < 0 ||
        best_rank >= static_cast<int>(active_tour_.goals.size()) ||
        !candidateMatchesGoal(goal,
                              active_tour_.goals[static_cast<size_t>(best_rank)])) {
        best_rank = -1;
        for (int i = 0; i < static_cast<int>(active_tour_.goals.size()); ++i) {
            if (candidateMatchesGoal(goal,
                                     active_tour_.goals[static_cast<size_t>(i)])) {
                best_rank = i;
                break;
            }
        }
    }
    if (best_rank < 0 ||
        best_rank >= static_cast<int>(active_tour_.goals.size())) {
        return;
    }
    active_tour_.cursor = best_rank;
    active_tour_.executing_rank = best_rank;
    active_tour_.status[static_cast<size_t>(best_rank)] = NodeStatus::EXECUTING;
    active_tour_.enter_stamp[static_cast<size_t>(best_rank)] =
            active_tour_.enter_stamp[static_cast<size_t>(best_rank)] > 0.0
                    ? active_tour_.enter_stamp[static_cast<size_t>(best_rank)]
                    : stamp;
}

void ExplorationGlobalPlanner::markTourNodeFailed(
        const ExplorationGoal &goal,
        const double stamp)
{
    if (!active_tour_.valid || !goal.valid) {
        return;
    }
    for (int i = 0; i < static_cast<int>(active_tour_.goals.size()); ++i) {
        if (!candidateMatchesGoal(goal,
                                  active_tour_.goals[static_cast<size_t>(i)])) {
            continue;
        }
        ++active_tour_.failures[static_cast<size_t>(i)];
        active_tour_.exit_stamp[static_cast<size_t>(i)] = stamp;
        active_tour_.status[static_cast<size_t>(i)] = NodeStatus::FAILED;
        if (i == active_tour_.executing_rank) {
            active_tour_.executing_rank = -1;
        }
        if (i == active_tour_.cursor) {
            active_tour_.cursor = i + 1;
        }
        if (active_tour_.failures[static_cast<size_t>(i)] >=
            std::max(1, cfg_.exploration_tour_max_node_failures)) {
            frontier_db_.markFailed(active_tour_.goals[static_cast<size_t>(i)],
                                    stamp);
        }
        return;
    }
}

int ExplorationGlobalPlanner::pendingTourCount() const
{
    int count = 0;
    if (!active_tour_.valid) {
        return count;
    }
    for (const NodeStatus status : active_tour_.status) {
        if (status == NodeStatus::PENDING) {
            ++count;
        }
    }
    return count;
}

int ExplorationGlobalPlanner::executingTourCount() const
{
    int count = 0;
    if (!active_tour_.valid) {
        return count;
    }
    for (const NodeStatus status : active_tour_.status) {
        if (status == NodeStatus::EXECUTING) {
            ++count;
        }
    }
    return count;
}

int ExplorationGlobalPlanner::completedTourCount() const
{
    int count = 0;
    for (const NodeStatus status : active_tour_.status) {
        if (status == NodeStatus::COMPLETED ||
            status == NodeStatus::SKIPPED) {
            ++count;
        }
    }
    return count;
}

int ExplorationGlobalPlanner::failedTourCount() const
{
    int count = 0;
    for (const NodeStatus status : active_tour_.status) {
        if (status == NodeStatus::FAILED) {
            ++count;
        }
    }
    return count;
}

bool ExplorationGlobalPlanner::candidateMatchesGoal(
        const ExplorationGoal &candidate,
        const ExplorationGoal &goal) const
{
    if (!candidate.valid || !goal.valid) {
        return false;
    }
    const std::string candidate_goal_key = goalKeyForGoal(candidate);
    const std::string goal_key = goalKeyForGoal(goal);
    if (!candidate_goal_key.empty() &&
        !goal_key.empty() &&
        candidate_goal_key == goal_key) {
        return true;
    }
    const std::string candidate_frontier =
            candidate.identity.frontierIdentityKey();
    const std::string goal_frontier = goal.identity.frontierIdentityKey();
    if (!candidate_frontier.empty() &&
        !goal_frontier.empty() &&
        candidate_frontier == goal_frontier) {
        return true;
    }
    if (candidate.candidate_id >= 0 &&
        candidate.candidate_id == goal.candidate_id) {
        return true;
    }
    const double match_radius =
            std::max({cfg_.exploration_active_tour_match_radius,
                      cfg_.exploration_frontier_manager_match_radius,
                      cfg_.exploration_goal_reached_distance,
                      0.75});
    if (candidate.position.allFinite() &&
        goal.position.allFinite() &&
        (candidate.position - goal.position).norm() <= match_radius) {
        return true;
    }
    if (candidate.frontier_center_valid &&
        goal.frontier_center_valid &&
        candidate.frontier_center.allFinite() &&
        goal.frontier_center.allFinite() &&
        (candidate.frontier_center - goal.frontier_center).norm() <= match_radius) {
        return true;
    }
    return false;
}

ExplorationGoal ExplorationGlobalPlanner::bestFallbackCandidate(
        const ExplorationCandidateSet *candidate_set,
        const ExplorationGoal &frontend_goal) const
{
    if (frontend_goal.valid) {
        return frontend_goal;
    }
    if (candidate_set == nullptr ||
        !candidate_set->valid ||
        candidate_set->candidates.empty()) {
        return ExplorationGoal{};
    }
    return *std::min_element(candidate_set->candidates.begin(),
                             candidate_set->candidates.end(),
                             [](const ExplorationGoal &lhs,
                                const ExplorationGoal &rhs) {
                                 if (lhs.score != rhs.score) {
                                     return lhs.score < rhs.score;
                                 }
                                 return lhs.information_gain > rhs.information_gain;
                             });
}

double ExplorationGlobalPlanner::startCost(
        const ExplorationGoal &goal,
        const general_utils::Vec3f &robot_pos,
        const double stamp) const
{
    if (!goal.position.allFinite() || !robot_pos.allFinite()) {
        return positiveFinite(goal.travel_cost, 1000.0);
    }
    return (goal.position - robot_pos).norm() + nodePenalty(goal, stamp);
}

double ExplorationGlobalPlanner::pairwiseCost(
        const ExplorationGoal &from,
        const ExplorationGoal &to,
        const double stamp) const
{
    double cost = 0.0;
    if (from.position.allFinite() && to.position.allFinite()) {
        cost += (to.position - from.position).norm();
    } else {
        cost += 1000.0;
    }
    cost += nodePenalty(to, stamp);
    cost += 0.5 * yawDistance(from.yaw, to.yaw);
    const std::string from_sector = sectorKeyForGoal(from);
    const std::string to_sector = sectorKeyForGoal(to);
    if (!from_sector.empty() &&
        !to_sector.empty() &&
        from_sector != to_sector) {
        cost += std::max(0.0, cfg_.exploration_tour_cross_sector_penalty);
    }
    return std::max(0.0, cost);
}

double ExplorationGlobalPlanner::nodePenalty(
        const ExplorationGoal &goal,
        const double stamp) const
{
    return std::max(0.0, cfg_.exploration_tour_coverage_penalty_weight) *
           coverage_grid_.revisitPenalty(sectorReference(goal), stamp);
}

double ExplorationGlobalPlanner::coverageIntentReward(
        const ExplorationGoal &goal,
        const double stamp) const
{
    if (!cfg_.exploration_coverage_intent_enable) {
        return 0.0;
    }
    return std::max(0.0, cfg_.exploration_coverage_intent_weight) *
           coverage_grid_.intentReward(sectorReference(goal), stamp);
}

std::string ExplorationGlobalPlanner::sectorKeyForGoal(
        const ExplorationGoal &goal) const
{
    if (!cfg_.exploration_active_sector_enable) {
        return {};
    }
    const double resolution =
            std::max(0.5, cfg_.exploration_active_sector_size);
    return nhbp::quantizedPositionKey(sectorReference(goal),
                                      resolution,
                                      "global_sector");
}

general_utils::Vec3f ExplorationGlobalPlanner::sectorReference(
        const ExplorationGoal &goal) const
{
    if (goal.frontier_center_valid && goal.frontier_center.allFinite()) {
        return goal.frontier_center;
    }
    return goal.position;
}

std::string ExplorationGlobalPlanner::goalKeyForGoal(
        const ExplorationGoal &goal) const
{
    const std::string canonical = goal.identity.canonicalKey();
    if (!canonical.empty()) {
        return canonical;
    }
    const std::string frontier = goal.identity.frontierIdentityKey();
    if (!frontier.empty()) {
        return frontier;
    }
    if (goal.candidate_id >= 0) {
        return "candidate:" + std::to_string(goal.candidate_id);
    }
    if (goal.position.allFinite()) {
        return nhbp::quantizedPositionKey(
                goal.position,
                std::max(0.25, cfg_.exploration_coverage_grid_resolution),
                "goal");
    }
    return {};
}

double ExplorationGlobalPlanner::completionRadius() const
{
    return std::max({cfg_.exploration_goal_reached_distance,
                     cfg_.exploration_frontier_memory_covered_radius,
                     0.5 * cfg_.exploration_active_tour_match_radius,
                     0.8});
}

} // namespace general_planner
