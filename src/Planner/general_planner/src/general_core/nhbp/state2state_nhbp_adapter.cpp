#include <general_core/nhbp/state2state_nhbp_adapter.hpp>

#include <general_core/config.hpp>

#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <limits>

namespace general_planner::nhbp {
namespace {
double clampTrajectoryTime(const geometry_utils::Trajectory &traj, const double stamp)
{
    if (traj.empty()) {
        return 0.0;
    }
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0.0 || !std::isfinite(traj.start_WT)) {
        return 0.0;
    }
    return std::clamp(stamp - traj.start_WT, 0.0, duration);
}

double trajectoryRemaining(const geometry_utils::Trajectory &traj, const double stamp)
{
    if (traj.empty()) {
        return 0.0;
    }
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0.0) {
        return 0.0;
    }
    return std::max(0.0, duration - clampTrajectoryTime(traj, stamp));
}

double xyCross(const general_utils::Vec3f &a, const general_utils::Vec3f &b)
{
    return a.x() * b.y() - a.y() * b.x();
}
} // namespace

State2StateNHBPAdapter::State2StateNHBPAdapter()
    : State2StateNHBPAdapter(Config{})
{
}

State2StateNHBPAdapter::State2StateNHBPAdapter(Config config)
    : config_(config),
      memory_(NavigationMemory::Config{
              config.decision_history,
              config.blacklist_ttl,
              config.min_progress_distance,
              config.no_progress_time,
              config.max_switches})
{
}

void State2StateNHBPAdapter::configure(Config config)
{
    config_ = config;
    memory_ = NavigationMemory(NavigationMemory::Config{
            config.decision_history,
            config.blacklist_ttl,
            config.min_progress_distance,
            config.no_progress_time,
            config.max_switches});
    active_goal_key_.clear();
    clearCommitHistory();
}

void State2StateNHBPAdapter::reset()
{
    memory_.reset();
    active_goal_key_.clear();
    clearCommitHistory();
}

State2StateNHBPDecision State2StateNHBPAdapter::evaluateReplan(
        const geometry_utils::Trajectory &candidate_traj,
        const geometry_utils::Trajectory &current_traj,
        const double current_backup_start_t,
        const general_utils::RobotState &robot_state,
        const general_utils::Vec3f &goal,
        const double stamp,
        const bool new_goal,
        const RuntimeTrajectorySafetyServices &safety_services,
        const ::general_planner::Config &planner_cfg)
{
    State2StateNHBPDecision decision;
    decision.ndo = memory_.diagnose(stamp);

    if (!config_.enable) {
        decision.reason = "state2state_nhbp_disabled";
        return decision;
    }

    const std::string goal_key = makeGoalKey(goal);
    if (new_goal || !sameLongRangeIntent(goal_key)) {
        memory_.reset();
        active_goal_key_ = goal_key;
        clearCommitHistory();
        decision.reason = new_goal ? "new_goal_accept" : "goal_key_changed_accept";
        return decision;
    }

    decision.candidate = makeSignature(candidate_traj, goal, stamp);
    decision.current = makeSignature(current_traj, goal, stamp);
    decision.candidate_score = scoreSignature(decision.candidate);
    decision.current_score = scoreSignature(decision.current);
    decision.score_improvement = decision.current_score - decision.candidate_score;
    decision.same_branch = decision.candidate.branch_key == decision.current.branch_key;
    decision.endpoint_delta = decision.candidate.valid && decision.current.valid
                                      ? (decision.candidate.end - decision.current.end).norm()
                                      : std::numeric_limits<double>::infinity();
    decision.lateral_delta = decision.candidate.valid && decision.current.valid
                                     ? std::abs(decision.candidate.lateral_peak -
                                                decision.current.lateral_peak)
                                     : std::numeric_limits<double>::infinity();
    decision.time_since_last_commit = has_last_new_commit_
                                              ? std::max(0.0, stamp - last_new_commit_stamp_)
                                              : std::numeric_limits<double>::infinity();
    decision.commits_in_window = commitsInWindow(stamp);
    decision.commit_churn =
            config_.commit_churn_window > 0.0 &&
            config_.max_commits_in_window > 0 &&
            decision.commits_in_window >= config_.max_commits_in_window;

    if (!decision.candidate.valid || !decision.current.valid) {
        decision.reason = !decision.candidate.valid ? "candidate_signature_invalid"
                                                    : "current_signature_invalid";
        return decision;
    }

    const double switch_margin = std::max(0.0, config_.switch_margin);
    const double same_branch_margin = std::max(0.0, config_.same_branch_margin);
    const bool current_reusable = currentTrajectoryReusable(current_traj,
                                                            current_backup_start_t,
                                                            stamp,
                                                            safety_services,
                                                            planner_cfg,
                                                            decision);
    if (!current_reusable) {
        decision.reason = "current_not_reusable_accept";
        return decision;
    }

    if (memory_.isBlacklisted(decision.candidate.branch_key, stamp)) {
        decision.action = State2StateGateAction::KEEP_CURRENT;
        decision.reason = "candidate_branch_blacklisted_keep_current";
        return decision;
    }

    if (decision.same_branch) {
        if (decision.commit_churn && decision.score_improvement <= switch_margin) {
            decision.action = State2StateGateAction::KEEP_CURRENT;
            decision.reason = "same_branch_commit_churn_keep_current";
            return decision;
        }

        if (decision.time_since_last_commit <
                    std::max(0.0, config_.min_commit_interval) &&
            decision.score_improvement <= same_branch_margin) {
            decision.action = State2StateGateAction::KEEP_CURRENT;
            decision.reason = "same_branch_min_commit_interval_keep_current";
            return decision;
        }

        const bool small_endpoint_change =
                decision.endpoint_delta <= std::max(0.0, config_.endpoint_change_threshold);
        const bool small_lateral_change =
                decision.lateral_delta <= std::max(0.0, config_.lateral_oscillation_threshold);
        if (small_endpoint_change &&
            small_lateral_change &&
            decision.score_improvement <= same_branch_margin) {
            decision.action = State2StateGateAction::KEEP_CURRENT;
            decision.reason = "same_branch_hysteresis_keep_current";
            return decision;
        }

        if (decision.ndo.state != NdoState::STABLE &&
            decision.score_improvement <= switch_margin) {
            decision.action = State2StateGateAction::KEEP_CURRENT;
            decision.reason = decision.ndo.state == NdoState::DEADLOCKED
                                      ? "same_branch_ndo_deadlocked_keep_current"
                                      : "same_branch_ndo_suspect_keep_current";
            return decision;
        }

        decision.reason = decision.score_improvement > same_branch_margin
                                  ? "same_branch_candidate_improves"
                                  : "same_branch_endpoint_changed_accept";
        return decision;
    }

    if (decision.commit_churn && decision.score_improvement <= switch_margin) {
        decision.action = State2StateGateAction::KEEP_CURRENT;
        decision.reason = "branch_commit_churn_keep_current";
        return decision;
    }

    if (decision.score_improvement > switch_margin) {
        decision.reason = "candidate_improves_over_margin";
        return decision;
    }

    const bool ndo_implicated =
            decision.ndo.state != NdoState::STABLE &&
            candidateId(decision.candidate) >= 0 &&
            std::find(decision.ndo.implicated_candidate_ids.begin(),
                      decision.ndo.implicated_candidate_ids.end(),
                      candidateId(decision.candidate)) !=
                    decision.ndo.implicated_candidate_ids.end();
    if (ndo_implicated) {
        decision.action = State2StateGateAction::KEEP_CURRENT;
        decision.reason = decision.ndo.state == NdoState::DEADLOCKED
                                  ? "branch_ndo_deadlocked_keep_current"
                                  : "branch_ndo_suspect_keep_current";
        return decision;
    }

    decision.action = State2StateGateAction::KEEP_CURRENT;
    decision.reason = "branch_commit_hysteresis_keep_current";
    return decision;
}

void State2StateNHBPAdapter::recordCommitted(const geometry_utils::Trajectory &traj,
                                             const general_utils::RobotState &robot_state,
                                             const general_utils::Vec3f &goal,
                                             const double stamp,
                                             const std::string &source)
{
    if (!config_.enable) {
        return;
    }
    const TrajectoryBranchSignature signature = makeSignature(traj, goal, stamp);
    if (!signature.valid) {
        return;
    }
    if (!sameLongRangeIntent(signature.goal_key)) {
        memory_.reset();
        active_goal_key_ = signature.goal_key;
        clearCommitHistory();
    }

    DecisionRecord record;
    record.candidate_id = candidateId(signature);
    record.frontier_id = -1;
    record.identity.intent_mode = "state2state";
    record.identity.candidate_id = record.candidate_id;
    record.identity.candidate_key = signature.branch_key;
    record.identity.goal_key = signature.goal_key;
    record.identity.branch_key = signature.branch_key;
    record.position = robot_state.p;
    record.robot_position = robot_state.p;
    record.target_position = goal;
    record.stamp = stamp;
    record.score = scoreSignature(signature);
    record.travel_cost = signature.length;
    record.goal_distance = signature.goal_distance;
    record.reason = source;
    memory_.recordDecision(record);

    DecisionTrace trace;
    trace.action = DecisionTraceAction::COMMIT;
    trace.identity = record.identity;
    trace.robot_position = robot_state.p;
    trace.target_position = goal;
    trace.stamp = stamp;
    trace.score = record.score;
    trace.travel_cost = record.travel_cost;
    trace.reason = source;
    memory_.recordTrace(trace);

    if (source != "keep_current") {
        has_last_new_commit_ = true;
        last_new_commit_stamp_ = stamp;
        last_new_commit_signature_ = signature;
        last_new_commit_robot_position_ = robot_state.p;
        recent_new_commit_stamps_.push_back(stamp);
        pruneRecentCommitHistory(stamp);
    }
}

void State2StateNHBPAdapter::recordFailure(
        const geometry_utils::Trajectory *candidate_traj,
        const general_utils::RobotState &robot_state,
        const general_utils::Vec3f &goal,
        const double stamp,
        const FailureReason reason)
{
    if (!config_.enable) {
        return;
    }
    std::string key = makeGoalKey(goal);
    if (candidate_traj != nullptr && !candidate_traj->empty()) {
        const TrajectoryBranchSignature signature = makeSignature(*candidate_traj, goal, stamp);
        if (signature.valid) {
            key = signature.branch_key;
        }
    }
    memory_.recordFailure(key, reason, stamp, config_.blacklist_ttl);

    DecisionRecord record;
    record.candidate_id = -1;
    record.frontier_id = -1;
    record.identity.intent_mode = "state2state";
    record.identity.candidate_key = key;
    record.identity.goal_key = makeGoalKey(goal);
    record.identity.branch_key = key;
    record.position = robot_state.p;
    record.robot_position = robot_state.p;
    record.target_position = goal;
    record.stamp = stamp;
    record.score = 0.0;
    record.reason = toString(reason);
    memory_.recordDecision(record);

    DecisionTrace trace;
    trace.action = DecisionTraceAction::FAILURE;
    trace.identity = record.identity;
    trace.robot_position = robot_state.p;
    trace.target_position = goal;
    trace.stamp = stamp;
    trace.reason = toString(reason);
    memory_.recordTrace(trace);
}

NdoDiagnosis State2StateNHBPAdapter::diagnose(const double stamp) const
{
    return memory_.diagnose(stamp);
}

std::string State2StateNHBPAdapter::diagnosticSummary(const double stamp) const
{
    const NdoDiagnosis diagnosis = memory_.diagnose(stamp);
    return fmt::format("state2state_nhbp={{enabled={},goal={},decisions={},failures={},traces={},ndo={},reason={},commits_window={}}}",
                       static_cast<int>(config_.enable),
                       active_goal_key_.empty() ? "none" : active_goal_key_,
                       memory_.decisionHistory().size(),
                       memory_.failures().size(),
                       memory_.traceHistory().size(),
                       toString(diagnosis.state),
                       diagnosis.reason,
                       commitsInWindow(stamp));
}

std::string State2StateNHBPAdapter::formatDecisionDiagnostic(
        const State2StateNHBPDecision &decision) const
{
    return fmt::format(";nhbp_action={};nhbp_reason={};nhbp_ndo={};nhbp_same_branch={};"
                       "nhbp_candidate_branch={};nhbp_current_branch={};"
                       "nhbp_candidate_score={:.3f};nhbp_current_score={:.3f};"
                       "nhbp_score_improvement={:.3f};nhbp_current_safe={};"
                       "nhbp_current_safety_reason={};nhbp_current_safety_ttc={:.3f};"
                       "nhbp_current_safety_collision_t={:.3f};"
                       "nhbp_current_safety_horizon={:.3f};"
                       "nhbp_current_safety_hits={};nhbp_current_safety_grid={};"
                       "nhbp_current_safety_pos=({:.3f},{:.3f},{:.3f});"
                       "nhbp_current_remaining={:.3f};nhbp_time_since_commit={:.3f};"
                       "nhbp_endpoint_delta={:.3f};nhbp_lateral_delta={:.3f};"
                       "nhbp_commits_window={};nhbp_commit_churn={}",
                       toString(decision.action),
                       decision.reason,
                       toString(decision.ndo.state),
                       static_cast<int>(decision.same_branch),
                       decision.candidate.branch_id,
                       decision.current.branch_id,
                       decision.candidate_score,
                       decision.current_score,
                       decision.score_improvement,
                       static_cast<int>(decision.current_safe),
                       decision.current_safety_reason,
                       decision.current_safety_ttc,
                       decision.current_safety_collision_t,
                       decision.current_safety_check_horizon,
                       decision.current_safety_hit_count,
                       decision.current_safety_grid_type,
                       decision.current_safety_collision_pos.x(),
                       decision.current_safety_collision_pos.y(),
                       decision.current_safety_collision_pos.z(),
                       decision.current_remaining,
                       decision.time_since_last_commit,
                       decision.endpoint_delta,
                       decision.lateral_delta,
                       decision.commits_in_window,
                       static_cast<int>(decision.commit_churn));
}

TrajectoryBranchSignature State2StateNHBPAdapter::makeSignature(
        const geometry_utils::Trajectory &traj,
        const general_utils::Vec3f &goal,
        const double stamp) const
{
    TrajectoryBranchSignature signature;
    signature.goal_key = makeGoalKey(goal);
    signature.branch_key = makeBranchKey(signature.goal_key, signature.branch_id);

    if (traj.empty() || !goal.allFinite()) {
        return signature;
    }
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 1.0e-4) {
        return signature;
    }

    const double start_t = clampTrajectoryTime(traj, stamp);
    const double end_t = std::min(duration, start_t + std::max(0.1, config_.signature_horizon));
    const general_utils::Vec3f start = traj.getPos(start_t);
    const general_utils::Vec3f end = traj.getPos(end_t);
    if (!start.allFinite() || !end.allFinite()) {
        return signature;
    }

    const general_utils::Vec3f goal_vec = goal - start;
    const double goal_xy_norm = goal_vec.head<2>().norm();
    general_utils::Vec3f goal_dir = general_utils::Vec3f::Zero();
    if (goal_xy_norm > 1.0e-4) {
        goal_dir.x() = goal_vec.x() / goal_xy_norm;
        goal_dir.y() = goal_vec.y() / goal_xy_norm;
    }

    double max_abs_lateral = 0.0;
    double signed_lateral_at_peak = 0.0;
    double length = 0.0;
    general_utils::Vec3f last = start;
    const double sample_dt = 0.1;
    for (double t = start_t; t <= end_t + 1.0e-6; t += sample_dt) {
        const general_utils::Vec3f pos = traj.getPos(std::min(t, duration));
        if (!pos.allFinite()) {
            continue;
        }
        length += (pos - last).norm();
        last = pos;
        if (goal_xy_norm > 1.0e-4) {
            const double lateral = xyCross(goal_dir, pos - start);
            if (std::abs(lateral) > max_abs_lateral) {
                max_abs_lateral = std::abs(lateral);
                signed_lateral_at_peak = lateral;
            }
        }
    }

    signature.valid = true;
    signature.lateral_peak = signed_lateral_at_peak;
    signature.length = length;
    signature.duration = std::max(0.0, end_t - start_t);
    signature.goal_distance = (end - goal).norm();
    signature.start = start;
    signature.end = end;
    if (max_abs_lateral >= std::max(0.0, config_.branch_lateral_threshold)) {
        signature.lateral_sign = signed_lateral_at_peak < 0.0 ? -1 : 1;
        signature.branch_id = signature.lateral_sign < 0 ? 0 : 2;
    } else {
        signature.lateral_sign = 0;
        signature.branch_id = 1;
    }
    signature.branch_key = makeBranchKey(signature.goal_key, signature.branch_id);
    return signature;
}

double State2StateNHBPAdapter::scoreSignature(const TrajectoryBranchSignature &signature) const
{
    if (!signature.valid) {
        return std::numeric_limits<double>::infinity();
    }
    return signature.length +
           std::max(0.0, config_.goal_progress_weight) * signature.goal_distance +
           0.1 * signature.duration;
}

int State2StateNHBPAdapter::candidateId(const TrajectoryBranchSignature &signature) const
{
    return signature.valid ? signature.branch_id : -1;
}

std::string State2StateNHBPAdapter::makeGoalKey(const general_utils::Vec3f &goal) const
{
    const double resolution = std::max(1.0e-3, config_.goal_key_resolution);
    const auto q = [resolution](const double value) {
        return static_cast<long long>(std::llround(value / resolution));
    };
    return fmt::format("g:{}:{}:{}", q(goal.x()), q(goal.y()), q(goal.z()));
}

std::string State2StateNHBPAdapter::makeBranchKey(const std::string &goal_key,
                                                  const int branch_id) const
{
    return fmt::format("{}:branch:{}", goal_key, branch_id);
}

bool State2StateNHBPAdapter::sameLongRangeIntent(const std::string &goal_key) const
{
    return active_goal_key_.empty() || active_goal_key_ == goal_key;
}

bool State2StateNHBPAdapter::currentTrajectoryReusable(
        const geometry_utils::Trajectory &current_traj,
        const double current_backup_start_t,
        const double stamp,
        const RuntimeTrajectorySafetyServices &safety_services,
        const ::general_planner::Config &planner_cfg,
        State2StateNHBPDecision &decision) const
{
    if (current_traj.empty()) {
        decision.current_safe = false;
        decision.current_safety_reason = "empty_current_trajectory";
        return false;
    }
    const double current_t = clampTrajectoryTime(current_traj, stamp);
    decision.current_remaining = trajectoryRemaining(current_traj, stamp);
    if (decision.current_remaining <= std::max(0.0, config_.min_commit_time)) {
        decision.current_safe = false;
        decision.current_safety_reason = "remaining_below_min_commit_time";
        return false;
    }

    double reuse_horizon = std::max(0.0, config_.reuse_safety_check_horizon);
    if (reuse_horizon <= 1.0e-6) {
        reuse_horizon = std::max(0.0, config_.safety_check_horizon);
    } else if (config_.safety_check_horizon > 1.0e-6) {
        reuse_horizon = std::min(reuse_horizon, config_.safety_check_horizon);
    }
    double horizon = std::min(reuse_horizon, decision.current_remaining);
    if (std::isfinite(current_backup_start_t) &&
        current_backup_start_t > 0.0 &&
        current_backup_start_t < current_traj.getTotalDuration()) {
        if (current_t >= current_backup_start_t - 1.0e-3) {
            decision.current_safe = false;
            decision.current_safety_reason = "backup_start_reached";
            decision.current_safety_check_horizon = 0.0;
            return false;
        }
        horizon = std::min(horizon, std::max(0.0, current_backup_start_t - current_t));
    }
    decision.current_safety_check_horizon = horizon;
    if (horizon <= 1.0e-3) {
        decision.current_safe = false;
        decision.current_safety_reason = "reuse_horizon_empty";
        return false;
    }

    CommittedTrajectorySafetyReport report;
    const int consecutive_hits = std::max(1, config_.reuse_safety_consecutive_hits);
    decision.current_safe = checkPositionTrajectorySafety(safety_services,
                                                          current_traj,
                                                          stamp,
                                                          horizon,
                                                          std::max(0.02, planner_cfg.sample_traj_dt),
                                                          consecutive_hits,
                                                          planner_cfg.state2state_nhbp_safety_unknown_as_occupied,
                                                          &report);
    decision.current_safety_reason =
            report.reason.empty()
                    ? (decision.current_safe ? std::string("safe") : std::string("unknown"))
                    : report.reason;
    decision.current_safety_ttc = report.time_to_collision;
    decision.current_safety_collision_t = report.collision_t;
    decision.current_safety_collision_pos = report.collision_pos;
    decision.current_safety_grid_type = report.grid_type;
    decision.current_safety_hit_count = report.hit_count;
    decision.current_safety_check_horizon = report.valid ? report.check_horizon : horizon;
    return decision.current_safe;
}

int State2StateNHBPAdapter::commitsInWindow(const double stamp) const
{
    if (config_.commit_churn_window <= 0.0) {
        return static_cast<int>(recent_new_commit_stamps_.size());
    }
    const double window_start = stamp - config_.commit_churn_window;
    int count = 0;
    for (const double commit_stamp : recent_new_commit_stamps_) {
        if (commit_stamp >= window_start && commit_stamp <= stamp + 1.0e-6) {
            ++count;
        }
    }
    return count;
}

void State2StateNHBPAdapter::pruneRecentCommitHistory(const double stamp)
{
    if (config_.commit_churn_window <= 0.0) {
        return;
    }
    const double window_start = stamp - config_.commit_churn_window;
    recent_new_commit_stamps_.erase(
            std::remove_if(recent_new_commit_stamps_.begin(),
                           recent_new_commit_stamps_.end(),
                           [window_start](const double commit_stamp) {
                               return commit_stamp < window_start;
                           }),
            recent_new_commit_stamps_.end());
}

void State2StateNHBPAdapter::clearCommitHistory()
{
    has_last_new_commit_ = false;
    last_new_commit_stamp_ = 0.0;
    last_new_commit_signature_ = TrajectoryBranchSignature{};
    last_new_commit_robot_position_.setZero();
    recent_new_commit_stamps_.clear();
}

const char *toString(const State2StateGateAction action)
{
    switch (action) {
        case State2StateGateAction::ACCEPT_CANDIDATE:
            return "ACCEPT_CANDIDATE";
        case State2StateGateAction::KEEP_CURRENT:
            return "KEEP_CURRENT";
    }
    return "UNKNOWN";
}

} // namespace general_planner::nhbp
