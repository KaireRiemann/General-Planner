#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include <general_core/config.hpp>
#include <general_core/exploration/exploration_frontend.hpp>

namespace general_planner {

class ExplorationFrontierDB {
public:
    using GoalCostFn = std::function<double(const ExplorationGoal &)>;
    using GoalKeyFn = std::function<std::string(const ExplorationGoal &)>;
    using GoalRefFn = std::function<general_utils::Vec3f(const ExplorationGoal &)>;

    enum class State {
        ACTIVE,
        COMMITTED,
        COVERED,
        BLOCKED,
        STALE
    };

    struct ObservationContext {
        general_utils::Vec3f robot_pos{general_utils::Vec3f::Zero()};
        double stamp{0.0};
        GoalCostFn node_penalty;
        GoalCostFn coverage_intent_reward;
        GoalKeyFn sector_key;
        GoalRefFn sector_reference;
    };

    struct ObjectSnapshot {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::string key;
        std::string sector_key;
        general_utils::Vec3f reference{general_utils::Vec3f::Zero()};
        general_utils::Vec3f bbox_min{general_utils::Vec3f::Zero()};
        general_utils::Vec3f bbox_max{general_utils::Vec3f::Zero()};
        general_utils::vec_E<ExplorationGoal> viewpoints;
        State state{State::ACTIVE};
        double best_score{0.0};
        double total_gain{0.0};
        double max_gain{0.0};
        double coverage_intent{0.0};
        double first_seen_stamp{0.0};
        double last_seen_stamp{0.0};
        double last_committed_stamp{0.0};
        double last_completed_stamp{0.0};
        double block_until{0.0};
        int candidate_count{0};
        int total_seen_count{0};
        int selection_count{0};
        int failure_count{0};
        int completed_count{0};
        int expansion_count{0};
        bool expansion_only{false};
    };

    explicit ExplorationFrontierDB(const Config &cfg);

    void reset();

    void observeCandidates(const ExplorationCandidateSet &candidate_set,
                           const ObservationContext &context);

    general_utils::vec_E<ObjectSnapshot> activeObjects(double stamp) const;

    void markCommitted(const ExplorationGoal &goal, double stamp);

    void markCompleted(const ExplorationGoal &goal, double stamp);

    void markFailed(const ExplorationGoal &goal, double stamp);

    void markCoveredNear(const general_utils::Vec3f &position,
                         double stamp,
                         double radius);

    bool goalActive(const ExplorationGoal &goal, double stamp) const;

    int recordCount() const;
    int activeObjectCount(double stamp) const;
    int blockedObjectCount(double stamp) const;
    int coveredObjectCount() const;
    int staleObjectCount(double stamp) const;

    std::string objectKeyForGoal(const ExplorationGoal &goal,
                                 const std::string &sector_key) const;

private:
    struct Record {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::string key;
        std::string sector_key;
        general_utils::Vec3f reference{general_utils::Vec3f::Zero()};
        general_utils::Vec3f bbox_min{general_utils::Vec3f::Zero()};
        general_utils::Vec3f bbox_max{general_utils::Vec3f::Zero()};
        general_utils::vec_E<ExplorationGoal> viewpoints;
        State state{State::ACTIVE};
        double best_score{0.0};
        double total_gain{0.0};
        double max_gain{0.0};
        double coverage_intent{0.0};
        double first_seen_stamp{0.0};
        double last_seen_stamp{0.0};
        double last_state_stamp{0.0};
        double last_committed_stamp{0.0};
        double last_completed_stamp{0.0};
        double block_until{0.0};
        int candidate_count{0};
        int total_seen_count{0};
        int selection_count{0};
        int failure_count{0};
        int completed_count{0};
        int expansion_count{0};
    };

    bool enabled() const;

    void prune(double stamp);

    void observeCandidate(const ExplorationGoal &candidate,
                          const ObservationContext &context);

    void insertViewpoint(Record &record, const ExplorationGoal &candidate) const;

    bool recordActive(const Record &record, double stamp) const;

    double recordPenalty(const Record &record, double stamp) const;

    ObjectSnapshot makeSnapshot(const Record &record, double stamp) const;

    Record *findRecordForGoal(const ExplorationGoal &goal);
    const Record *findRecordForGoal(const ExplorationGoal &goal) const;

    static bool expansionCandidate(const ExplorationGoal &goal);

    const Config &cfg_;
    std::unordered_map<std::string, Record> records_;
};

} // namespace general_planner
