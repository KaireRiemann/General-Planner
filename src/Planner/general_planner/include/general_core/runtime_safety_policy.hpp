#pragma once

#include <algorithm>
#include <string>

#include <general_core/planning_semantics.hpp>

namespace general_planner::architecture {

struct RuntimeSafetyDecision {
    SafetyAction action{SafetyAction::ACCEPT};
    SafetySeverity severity{SafetySeverity::INFO};
    bool emergency{false};
    bool request_replan{false};
    std::string reason;
};

class RuntimeSafetyPolicy {
public:
    static RuntimeSafetyDecision decideCommittedTrajectoryCollision(
            const bool report_valid,
            const bool trajectory_safe,
            const double time_to_collision,
            const double emergency_horizon,
            const bool interval_ok) {
        RuntimeSafetyDecision decision;
        if (trajectory_safe) {
            decision.reason = "trajectory_safe";
            return decision;
        }
        if (!report_valid) {
            decision.reason = "invalid_safety_report";
            decision.action = SafetyAction::KEEP_CURRENT;
            decision.severity = SafetySeverity::WARN;
            return decision;
        }

        decision.emergency = time_to_collision <= std::max(0.0, emergency_horizon);
        decision.action = decision.emergency ? SafetyAction::EMERGENCY_STOP : SafetyAction::REPLAN;
        decision.severity = decision.emergency ? SafetySeverity::ERROR : SafetySeverity::WARN;
        decision.request_replan = interval_ok || decision.emergency;
        decision.reason = decision.emergency ? "collision_emergency" : "collision_replan";
        return decision;
    }
};

} // namespace general_planner::architecture
