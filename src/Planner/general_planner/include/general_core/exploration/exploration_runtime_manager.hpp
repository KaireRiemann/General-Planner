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
                         ExplorationGoal &goal) const;

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
};

} // namespace general_planner
