#pragma once

#include <string>

#include <general_core/planning_semantics.hpp>
#include <general_core/runtime_safety_policy.hpp>

namespace general_planner::architecture {

struct SafetyMonitorContext {
    TaskIdentity identity;
    ExecutionPhase phase{ExecutionPhase::WAITING_INPUT};
    std::string source;
};

struct SafetyMonitorReport {
    SafetyAction action{SafetyAction::ACCEPT};
    SafetySeverity severity{SafetySeverity::INFO};
    bool request_replan{false};
    bool emergency{false};
    std::string reason;
};

class SafetyMonitor {
public:
    SafetyMonitorReport evaluateCommittedCollision(
            const SafetyMonitorContext &context,
            bool report_valid,
            bool trajectory_safe,
            double time_to_collision,
            double emergency_horizon,
            bool interval_ok) const {
        const auto decision = RuntimeSafetyPolicy::decideCommittedTrajectoryCollision(
                report_valid,
                trajectory_safe,
                time_to_collision,
                emergency_horizon,
                interval_ok);
        SafetyMonitorReport report;
        report.action = decision.action;
        report.severity = decision.severity;
        report.request_replan = decision.request_replan;
        report.emergency = decision.emergency;
        report.reason = context.source.empty() ? decision.reason : context.source + ":" + decision.reason;
        return report;
    }
};

} // namespace general_planner::architecture
