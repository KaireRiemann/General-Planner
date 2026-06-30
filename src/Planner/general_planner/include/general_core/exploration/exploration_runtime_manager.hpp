#pragma once

#include <memory>
#include <string>

#include "general_core/config.hpp"
#include "general_core/exploration/coverage_grid.hpp"
#include "general_core/exploration/exploration_frontend.hpp"
#include "general_core/exploration/frontier_memory.hpp"
#include "general_core/nhbp/decision_stabilizer.hpp"
#include "general_core/nhbp/navigation_memory.hpp"
#include "general_core/nhbp/topological_memory.hpp"

namespace general_planner {

class ExplorationRuntimeManager {
public:
    enum class Status {
        IDLE,
        SELECTING_GOAL,
        GOAL_SELECTED,
        ACTIVE_COMMITTED,
        KEEP_CURRENT_GOAL,
        TEMPORARY_FAILURE,
        FINISHED
    };

    enum class Phase {
        IDLE,
        WAIT_MAP,
        UPDATE_BELIEF,
        SELECT_TOUR,
        SELECT_LOCAL_GOAL,
        PLAN_LOCAL_TRAJECTORY,
        EXECUTE_COMMITTED,
        RECOVERY_ESCAPE,
        FINISHED,
        FAILED
    };

    struct SelectionDecision {
        bool ready{false};
        bool keep_current{false};
        bool reject{false};
        bool recovery_requested{false};
        bool allow_candidate_fallback{false};
        ExplorationGoal goal;
        std::string reason;
        nhbp::NdoDiagnosis ndo;
    };

    explicit ExplorationRuntimeManager(const Config &cfg);

    void reset();

    void onSelectingGoal();
    void onGoalSelected(const ExplorationGoal &goal);
    void onCommitted(const ExplorationGoal &goal);
    void onKeepCurrentGoal();
    void onTemporaryFailure(const ExplorationGoal &goal);
    void onFinished(const ExplorationGoal &goal);

    bool shouldKeepCurrentGoal(const ExplorationGoal &candidate,
                               const general_utils::Vec3f &robot_pos,
                               double committed_remaining,
                               bool new_task) const;

    bool shouldReuseLatestGoal(const general_utils::Vec3f &robot_pos,
                               double committed_remaining,
                               bool new_task) const;

    SelectionDecision stabilizeCandidate(const ExplorationGoal &candidate,
                                         const general_utils::Vec3f &robot_pos,
                                         double committed_remaining,
                                         double stamp,
                                         bool new_task);

    void recordDecision(const ExplorationGoal &goal,
                        const general_utils::Vec3f &robot_pos,
                        double stamp);

    void recordFailure(const ExplorationGoal &goal,
                       nhbp::FailureReason reason,
                       double stamp);

    nhbp::NdoDiagnosis diagnose(double stamp) const;

    bool hasRecoveryGoal(const general_utils::Vec3f &robot_pos,
                         double stamp,
                         ExplorationGoal &goal);

    bool shouldDelayFinish(double stamp) const;

    bool getLatestGoal(ExplorationGoal &goal) const;
    std::string diagnosticSummary(double stamp) const;

    Status status() const;
    Phase phase() const;
    int consecutiveTemporaryFailures() const;
    bool hasCommittedGoal() const;

    static const char *toString(Status status);
    static const char *toString(Phase phase);

private:
    bool latestGoalReusable(const general_utils::Vec3f &robot_pos,
                            double committed_remaining,
                            bool new_task) const;

    nhbp::DecisionCandidate toDecisionCandidate(const ExplorationGoal &goal) const;

    bool localTrapDetected(const ExplorationGoal &candidate,
                           const general_utils::Vec3f &robot_pos,
                           double stamp,
                           double revisit_penalty,
                           std::string &reason);
    bool recoveryBlockedByRecentTrap(const ExplorationGoal &goal, double stamp) const;
    bool recoveryPositionBlockedByRecentTrap(const general_utils::Vec3f &position,
                                             int frontier_id,
                                             double stamp) const;

    bool nhbpEnabled() const;

    const Config &cfg_;
    nhbp::NavigationMemory navigation_memory_;
    nhbp::DecisionStabilizer decision_stabilizer_;
    FrontierMemory frontier_memory_;
    CoverageGrid coverage_grid_;
    nhbp::TopologicalMemory topological_memory_;
    ExplorationGoal latest_goal_;
    ExplorationGoal committed_goal_;
    Status status_{Status::IDLE};
    Phase phase_{Phase::IDLE};
    int consecutive_temporary_failures_{0};
    bool has_latest_goal_{false};
    bool has_committed_goal_{false};
    bool has_last_trap_candidate_{false};
    general_utils::Vec3f last_trap_candidate_position_{general_utils::Vec3f::Zero()};
    general_utils::Vec3f last_trap_robot_position_{general_utils::Vec3f::Zero()};
    int last_trap_frontier_id_{-1};
    int repeated_local_region_count_{0};
    int local_trap_recovery_request_count_{0};
    double local_trap_cooldown_until_{0.0};
    bool has_recent_trap_region_{false};
    general_utils::Vec3f recent_trap_position_{general_utils::Vec3f::Zero()};
    int recent_trap_frontier_id_{-1};
    double recent_trap_block_until_{0.0};
    int recovery_query_count_{0};
    int frontier_recovery_selected_count_{0};
    int topology_recovery_selected_count_{0};
    int recovery_unavailable_count_{0};
    int recovery_blocked_by_recent_trap_count_{0};
};

} // namespace general_planner
