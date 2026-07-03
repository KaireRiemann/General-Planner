#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "general_core/config.hpp"
#include "general_core/exploration/coverage_grid.hpp"
#include "general_core/exploration/exploration_frontend.hpp"
#include "general_core/exploration/exploration_frontier_db.hpp"
#include "general_core/exploration/exploration_task_planner.hpp"
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

    struct RecoveryState {
        bool active{false};
        nhbp::NavIdentity recovery_id;
        general_utils::Vec3f start_pos{general_utils::Vec3f::Zero()};
        general_utils::Vec3f goal_pos{general_utils::Vec3f::Zero()};
        double start_stamp{0.0};
        double min_duration{0.0};
        double min_distance{0.0};
        double lock_until{0.0};
        bool exit_success{false};
        std::string reason;
        std::string exit_reason;
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

    SelectionDecision selectGoalFromCandidates(
            const ExplorationCandidateSet &candidate_set,
            const general_utils::Vec3f &robot_pos,
            double current_yaw,
            double committed_remaining,
            double stamp,
            bool new_task);

    SelectionDecision selectGoalFromActiveTour(
            const general_utils::Vec3f &robot_pos,
            double committed_remaining,
            double stamp,
            bool new_task);

    SelectionDecision selectGoalForExecution(
            const ExplorationCandidateSet &candidate_set,
            bool has_candidate_set,
            const ExplorationGoal &frontend_goal,
            const general_utils::Vec3f &robot_pos,
            double current_yaw,
            double committed_remaining,
            double stamp,
            bool new_task);

    bool refreshGlobalTaskGraph(
            const ExplorationCandidateSet &candidate_set,
            const general_utils::Vec3f &robot_pos,
            double current_yaw,
            double stamp,
            bool new_task,
            std::string &reason);

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
    bool isRecoveryGoal(const ExplorationGoal &goal) const;
    nhbp::NavIdentity normalizedIdentity(const ExplorationGoal &goal) const;
    void recordTrace(nhbp::DecisionTraceAction action,
                     const ExplorationGoal &goal,
                     const ExplorationGoal &previous_goal,
                     const general_utils::Vec3f &robot_pos,
                     double stamp,
                     double committed_remaining,
                     const nhbp::NdoDiagnosis &ndo,
                     const std::string &reason);
    void enterRecoveryLock(const ExplorationGoal &trigger_goal,
                           const general_utils::Vec3f &robot_pos,
                           double stamp,
                           const std::string &reason);
    void updateRecoveryLock(const general_utils::Vec3f &robot_pos, double stamp);
    bool recoveryLockActive(double stamp) const;
    void releaseRecoveryLock(double stamp, const std::string &reason, bool success);
    void bindRecoveryGoal(const ExplorationGoal &goal,
                          const general_utils::Vec3f &robot_pos,
                          double stamp);
    bool lockedRecoveryGoalReusable(const general_utils::Vec3f &robot_pos,
                                    double stamp) const;

    bool nhbpEnabled() const;
    bool activeTourEnabled() const;
    bool activeSectorEnabled() const;
    double activeSectorResolution() const;
    general_utils::Vec3f sectorReference(const ExplorationGoal &goal) const;
    std::string sectorKeyForGoal(const ExplorationGoal &goal) const;
    void updateSectorMemoryFromCandidates(const ExplorationCandidateSet &candidate_set,
                                          double stamp);
    void markSectorActive(const std::string &sector_key,
                          const general_utils::Vec3f &center,
                          int candidate_count,
                          double score,
                          double stamp);
    void markSectorProgress(const ExplorationGoal &goal, double stamp);
    void markSectorFailure(const ExplorationGoal &goal, double stamp);
    double sectorMemoryPenalty(const std::string &sector_key, double stamp) const;
    ExplorationCandidateSet selectSectorCandidates(
            const ExplorationCandidateSet &candidate_set,
            const general_utils::Vec3f &robot_pos,
            double stamp,
            bool new_task,
            std::string &reason);
    void invalidateActiveTour(const std::string &reason);
    void invalidateActiveSector(const std::string &reason);
    void ensureActiveTourState();
    bool advanceCompletedTourNodes(const general_utils::Vec3f &robot_pos,
                                   double stamp);
    void markTourNodeExecuting(const ExplorationGoal &goal, double stamp);
    void markTourNodeCompleted(int rank, double stamp);
    void markTourNodeSkipped(int rank, double stamp);
    void markTourNodeFailed(const ExplorationGoal &goal, double stamp);
    int pendingTourNodeCount() const;
    int executingTourNodeCount() const;
    int completedTourNodeCount() const;
    int failedTourNodeCount() const;
    bool findActiveTourCandidate(const ExplorationCandidateSet &candidate_set,
                                 const general_utils::Vec3f &robot_pos,
                                 double stamp,
                                 ExplorationGoal &goal,
                                 std::string &reason);
    bool repairActiveTourFromCandidates(const ExplorationCandidateSet &candidate_set,
                                        const general_utils::Vec3f &robot_pos,
                                        double stamp,
                                        std::string &reason);
    bool rebuildActiveTour(const ExplorationCandidateSet &candidate_set,
                           const general_utils::Vec3f &robot_pos,
                           double current_yaw,
                           double stamp,
                           std::string &reason);
    general_utils::vec_E<ExplorationFrontierDB::ObjectSnapshot>
    selectLiveFrontierObjectsForTour(const ExplorationCandidateSet &candidate_set,
                                     double stamp) const;
    bool candidateMatchesTourGoal(const ExplorationGoal &candidate,
                                  const ExplorationGoal &tour_goal,
                                  double match_radius) const;
    double coverageIntentReward(const ExplorationGoal &goal, double stamp) const;
    double tourPairwiseCandidateCost(const ExplorationGoal &from,
                                     const ExplorationGoal &to,
                                     double stamp) const;
    double topologyAwareTravelCost(const ExplorationGoal &from,
                                   const ExplorationGoal &to,
                                   double stamp) const;
    std::string tourGoalKey(const ExplorationGoal &goal) const;

    const Config &cfg_;
    nhbp::NavigationMemory navigation_memory_;
    nhbp::DecisionStabilizer decision_stabilizer_;
    FrontierMemory frontier_memory_;
    CoverageGrid coverage_grid_;
    nhbp::TopologicalMemory topological_memory_;
    ExplorationFrontierDB frontier_db_;
    ExplorationTaskPlanner task_planner_;
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
    int locked_recovery_goal_reused_count_{0};
    int recovery_unavailable_count_{0};
    int recovery_blocked_by_recent_trap_count_{0};
    bool recovery_lock_active_{false};
    RecoveryState recovery_state_;
    ExplorationGoal recovery_lock_trigger_goal_;
    ExplorationGoal active_recovery_goal_;
    bool has_active_recovery_goal_{false};
    std::string recovery_lock_reason_;
    double recovery_lock_started_stamp_{0.0};
    double recovery_lock_until_{0.0};
    int recovery_lock_request_count_{0};
    int recovery_lock_release_count_{0};

    struct ActiveSector {
        bool valid{false};
        std::string key;
        general_utils::Vec3f center{general_utils::Vec3f::Zero()};
        double score{0.0};
        int candidate_count{0};
        int generation{0};
        int failure_count{0};
        double created_stamp{0.0};
        double last_update_stamp{0.0};
        double last_progress_stamp{0.0};
        std::string invalid_reason;
    };

    struct ActiveTour {
        enum class NodeStatus {
            PENDING,
            EXECUTING,
            COMPLETED,
            SKIPPED,
            FAILED
        };

        general_utils::vec_E<ExplorationGoal> goals;
        std::vector<NodeStatus> node_status;
        std::vector<int> node_failures;
        std::vector<double> node_enter_stamp;
        std::vector<double> node_exit_stamp;
        std::string tour_key;
        std::string sector_key;
        int cursor{0};
        int executing_rank{-1};
        int generation{0};
        double created_stamp{0.0};
        double last_rebuild_stamp{0.0};
        bool valid{false};
        std::string invalid_reason;
    };

    enum class SectorStatus {
        UNKNOWN,
        ACTIVE,
        COMPLETED,
        BLOCKED,
        STALE
    };

    struct SectorMemoryEntry {
        std::string key;
        SectorStatus status{SectorStatus::UNKNOWN};
        general_utils::Vec3f center{general_utils::Vec3f::Zero()};
        double score{0.0};
        int candidate_count{0};
        int total_seen_count{0};
        int selection_count{0};
        int progress_count{0};
        int failure_count{0};
        double first_seen_stamp{0.0};
        double last_seen_stamp{0.0};
        double last_selected_stamp{0.0};
        double last_progress_stamp{0.0};
        double block_until{0.0};
        double completed_stamp{0.0};
    };

    ActiveSector active_sector_;
    ActiveTour active_tour_;
    std::unordered_map<std::string, SectorMemoryEntry> sector_memory_;
    int active_sector_reuse_count_{0};
    int active_sector_switch_count_{0};
    int active_sector_invalid_count_{0};
    int active_sector_filter_count_{0};
    int sector_completed_count_{0};
    int sector_blocked_count_{0};
    int sector_reactivated_count_{0};
    int active_tour_reuse_count_{0};
    int active_tour_rebuild_count_{0};
    int active_tour_repair_count_{0};
    int active_tour_advance_count_{0};
    int active_tour_invalid_count_{0};
    int active_tour_node_completed_count_{0};
    int active_tour_node_failed_count_{0};
    int active_tour_node_skipped_count_{0};
};

} // namespace general_planner
