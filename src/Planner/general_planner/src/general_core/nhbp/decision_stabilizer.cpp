#include <general_core/nhbp/decision_stabilizer.hpp>

#include <algorithm>

namespace general_planner::nhbp {

DecisionStabilizer::DecisionStabilizer()
    : DecisionStabilizer(Config{})
{
}

DecisionStabilizer::DecisionStabilizer(Config config)
    : config_(config)
{
}

StabilizerDecision DecisionStabilizer::stabilize(
        const DecisionCandidate &candidate,
        const DecisionCandidate &current,
        const DecisionContext &context,
        const NavigationMemory &memory) const
{
    StabilizerDecision decision;
    decision.ndo = memory.diagnose(context.stamp);

    if (!config_.enable) {
        decision.action = StabilizerAction::ACCEPT_CANDIDATE;
        decision.reason = "nhbp_disabled";
        return decision;
    }

    if (!candidate.valid) {
        decision.action = context.current_reusable
                                  ? StabilizerAction::KEEP_CURRENT
                                  : StabilizerAction::REJECT_CANDIDATE;
        decision.reason = context.current_reusable
                                  ? "candidate_invalid_keep_current"
                                  : "candidate_invalid";
        return decision;
    }

    if (!candidate.key.empty() && memory.isBlacklisted(candidate.key, context.stamp)) {
        decision.action = context.current_reusable
                                  ? StabilizerAction::KEEP_CURRENT
                                  : StabilizerAction::REJECT_CANDIDATE;
        decision.reason = context.current_reusable
                                  ? "candidate_blacklisted_keep_current"
                                  : "candidate_blacklisted";
        return decision;
    }

    if (context.new_task || !context.current_reusable || !current.valid) {
        decision.action = StabilizerAction::ACCEPT_CANDIDATE;
        decision.reason = context.new_task ? "new_task_accept" : "no_reusable_current";
        return decision;
    }

    if (sameCandidate(candidate, current)) {
        decision.action = StabilizerAction::KEEP_CURRENT;
        decision.reason = "same_current_goal";
        return decision;
    }

    if (decision.ndo.state != NdoState::STABLE && implicatedByNdo(candidate, decision.ndo)) {
        decision.action = StabilizerAction::KEEP_CURRENT;
        decision.reason = decision.ndo.state == NdoState::DEADLOCKED
                                  ? "ndo_deadlocked_keep_current"
                                  : "ndo_suspect_keep_current";
        return decision;
    }

    const double min_commit_time = std::max(0.0, config_.min_commit_time);
    const double switch_margin = std::max(0.0, config_.switch_margin);
    if (context.committed_remaining > min_commit_time &&
        candidate.score >= current.score - switch_margin) {
        decision.action = StabilizerAction::KEEP_CURRENT;
        decision.reason = "commit_hysteresis_keep_current";
        return decision;
    }

    if (candidate.score < current.score - switch_margin) {
        decision.action = StabilizerAction::ACCEPT_CANDIDATE;
        decision.reason = "candidate_improves_over_margin";
        return decision;
    }

    decision.action = StabilizerAction::KEEP_CURRENT;
    decision.reason = "switch_margin_keep_current";
    return decision;
}

bool DecisionStabilizer::sameCandidate(const DecisionCandidate &lhs,
                                       const DecisionCandidate &rhs) const
{
    if (!lhs.valid || !rhs.valid) {
        return false;
    }
    if (!lhs.key.empty() && !rhs.key.empty()) {
        return lhs.key == rhs.key;
    }
    return lhs.candidate_id >= 0 &&
           rhs.candidate_id >= 0 &&
           lhs.candidate_id == rhs.candidate_id;
}

bool DecisionStabilizer::implicatedByNdo(const DecisionCandidate &candidate,
                                         const NdoDiagnosis &diagnosis) const
{
    if (!candidate.valid || candidate.candidate_id < 0) {
        return false;
    }
    return std::find(diagnosis.implicated_candidate_ids.begin(),
                     diagnosis.implicated_candidate_ids.end(),
                     candidate.candidate_id) != diagnosis.implicated_candidate_ids.end();
}

const char *toString(const StabilizerAction action)
{
    switch (action) {
        case StabilizerAction::ACCEPT_CANDIDATE:
            return "ACCEPT_CANDIDATE";
        case StabilizerAction::KEEP_CURRENT:
            return "KEEP_CURRENT";
        case StabilizerAction::REJECT_CANDIDATE:
            return "REJECT_CANDIDATE";
    }
    return "UNKNOWN";
}

} // namespace general_planner::nhbp
