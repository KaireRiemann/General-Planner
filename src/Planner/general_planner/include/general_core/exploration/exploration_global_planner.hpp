#pragma once

#include <string>
#include <vector>

#include <general_core/config.hpp>
#include <general_core/exploration/coverage_grid.hpp>
#include <general_core/exploration/exploration_frontier_db.hpp>
#include <general_core/exploration/exploration_task_planner.hpp>

namespace general_planner {

class ExplorationGlobalPlanner {
public:
    struct RefreshResult {
        bool updated{false};
        bool observed_candidates{false};
        bool repaired_tour{false};
        bool rebuilt_tour{false};
        int raw_candidate_count{0};
        int active_object_count{0};
        int pending_tour_count{0};
        int executing_tour_count{0};
        int rebuilt_tour_size{0};
        std::string reason;
    };

    struct Decision {
        bool ready{false};
        bool from_active_tour{false};
        bool rebuilt_tour{false};
        ExplorationGoal goal;
        std::string reason;
    };

    explicit ExplorationGlobalPlanner(const Config &cfg);

    void reset();

    RefreshResult refresh(const ExplorationCandidateSet *candidate_set,
                          const general_utils::Vec3f &robot_pos,
                          double current_yaw,
                          double stamp,
                          bool force_rebuild);

    Decision select(const ExplorationCandidateSet *candidate_set,
                    const ExplorationGoal &frontend_goal,
                    const general_utils::Vec3f &robot_pos,
                    double current_yaw,
                    double stamp,
                    bool allow_frontend_fallback);

    void recordCommitted(const ExplorationGoal &goal,
                         const general_utils::Vec3f &robot_pos,
                         double stamp);

    void recordFailed(const ExplorationGoal &goal, double stamp);

    bool hasPendingTour() const;
    bool goalActive(const ExplorationGoal &goal, double stamp) const;

    std::string diagnosticSummary(double stamp) const;

private:
    enum class NodeStatus {
        PENDING,
        EXECUTING,
        COMPLETED,
        SKIPPED,
        FAILED
    };

    struct ActiveTour {
        general_utils::vec_E<ExplorationGoal> goals;
        std::vector<NodeStatus> status;
        std::vector<int> failures;
        std::vector<double> enter_stamp;
        std::vector<double> exit_stamp;
        std::string key;
        int cursor{0};
        int executing_rank{-1};
        int generation{0};
        double created_stamp{0.0};
        double last_rebuild_stamp{0.0};
        bool valid{false};
        std::string invalid_reason;
    };

    bool enabled() const;
    bool tourRebuildAllowed(double stamp) const;
    bool observeCandidates(const ExplorationCandidateSet &candidate_set,
                           const general_utils::Vec3f &robot_pos,
                           double stamp);
    bool rebuildTour(const ExplorationCandidateSet *candidate_set,
                     const general_utils::Vec3f &robot_pos,
                     double current_yaw,
                     double stamp,
                     std::string &reason);
    bool repairTourFromCandidates(const ExplorationCandidateSet &candidate_set,
                                  double stamp,
                                  std::string &reason);
    bool selectTourGoal(const general_utils::Vec3f &robot_pos,
                        double stamp,
                        ExplorationGoal &goal,
                        std::string &reason);
    void invalidateTour(const std::string &reason);
    bool advanceCompletedTourNodes(const general_utils::Vec3f &robot_pos,
                                   double stamp);
    void markTourNodeExecuting(const ExplorationGoal &goal, double stamp);
    void markTourNodeFailed(const ExplorationGoal &goal, double stamp);
    int pendingTourCount() const;
    int executingTourCount() const;
    int completedTourCount() const;
    int failedTourCount() const;

    bool candidateMatchesGoal(const ExplorationGoal &candidate,
                              const ExplorationGoal &goal) const;
    ExplorationGoal bestFallbackCandidate(const ExplorationCandidateSet *candidate_set,
                                          const ExplorationGoal &frontend_goal) const;
    double startCost(const ExplorationGoal &goal,
                     const general_utils::Vec3f &robot_pos,
                     double stamp) const;
    double pairwiseCost(const ExplorationGoal &from,
                        const ExplorationGoal &to,
                        double stamp) const;
    double nodePenalty(const ExplorationGoal &goal, double stamp) const;
    double coverageIntentReward(const ExplorationGoal &goal, double stamp) const;
    std::string sectorKeyForGoal(const ExplorationGoal &goal) const;
    general_utils::Vec3f sectorReference(const ExplorationGoal &goal) const;
    std::string goalKeyForGoal(const ExplorationGoal &goal) const;
    double completionRadius() const;

    const Config &cfg_;
    CoverageGrid coverage_grid_;
    ExplorationFrontierDB frontier_db_;
    ExplorationTaskPlanner task_planner_;
    ActiveTour active_tour_;
    int observe_count_{0};
    int rebuild_count_{0};
    int repair_count_{0};
    int reuse_count_{0};
    int committed_count_{0};
    int failed_count_{0};
    int invalid_count_{0};
    int last_raw_candidate_count_{0};
    int last_active_object_count_{0};
    int last_rebuilt_tour_size_{0};
    std::string last_refresh_reason_;
    std::string last_select_reason_;
    std::string last_rebuild_reason_;
};

} // namespace general_planner
