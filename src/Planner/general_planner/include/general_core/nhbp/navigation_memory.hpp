#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <general_core/nhbp/nav_identity.hpp>
#include <general_utils/type_utils.hpp>

namespace general_planner::nhbp {

enum class FailureReason {
    ASTAR_FAIL,
    ASTAR_TOO_LONG,
    OPTIMIZATION_FAIL,
    SAFETY_COLLISION,
    NO_PROGRESS,
    NDO_OSCILLATION,
    BACKUP_TRIGGERED,
    VIEWPOINT_UNSAFE,
    FRONTIER_COVERED,
    MAP_STALE
};

enum class NdoState {
    STABLE,
    SUSPECT,
    DEADLOCKED
};

struct FailureRecord {
    std::string key;
    FailureReason reason{FailureReason::ASTAR_FAIL};
    int count{0};
    double first_time{0.0};
    double last_time{0.0};
    double blacklist_until{0.0};
};

struct DecisionRecord {
    int candidate_id{-1};
    int frontier_id{-1};
    NavIdentity identity;
    general_utils::Vec3f position{general_utils::Vec3f::Zero()};
    general_utils::Vec3f robot_position{general_utils::Vec3f::Zero()};
    general_utils::Vec3f robot_velocity{general_utils::Vec3f::Zero()};
    general_utils::Vec3f selected_goal{general_utils::Vec3f::Zero()};
    general_utils::Vec3f committed_goal{general_utils::Vec3f::Zero()};
    general_utils::Vec3f committed_end{general_utils::Vec3f::Zero()};
    general_utils::Vec3f guide_first_direction{general_utils::Vec3f::Zero()};
    general_utils::Vec3f target_position{general_utils::Vec3f::Zero()};
    double stamp{0.0};
    double score{0.0};
    double raw_score{0.0};
    double final_score{0.0};
    double information_gain{0.0};
    double travel_cost{0.0};
    double goal_distance{0.0};
    double goal_progress{0.0};
    double travel_since_last{0.0};
    double committed_remaining{0.0};
    bool committed{false};
    bool keep_current{false};
    bool recovery{false};
    bool safety_event{false};
    int ret_code{-1};
    std::string reason;
};

enum class DecisionTraceAction {
    CANDIDATE_EVALUATED,
    ACCEPT_CANDIDATE,
    KEEP_CURRENT,
    REJECT_CANDIDATE,
    RECOVERY_REQUESTED,
    RECOVERY_SELECTED,
    COMMIT,
    FAILURE
};

struct DecisionTrace {
    DecisionTraceAction action{DecisionTraceAction::CANDIDATE_EVALUATED};
    NavIdentity identity;
    NavIdentity previous_identity;
    std::string task;
    std::string phase;
    std::string source;
    general_utils::Vec3f robot_position{general_utils::Vec3f::Zero()};
    general_utils::Vec3f target_position{general_utils::Vec3f::Zero()};
    double stamp{0.0};
    double score{0.0};
    double raw_score{0.0};
    double final_score{0.0};
    double memory_delta{0.0};
    double stability_margin{0.0};
    double information_gain{0.0};
    double travel_cost{0.0};
    double committed_remaining{0.0};
    NdoState ndo_state{NdoState::STABLE};
    bool keep_current{false};
    bool recovery{false};
    bool committed{false};
    bool safety_override{false};
    int ret_code{-1};
    std::string reason;
};

struct NdoMetrics {
    double travel_distance{0.0};
    double net_displacement{0.0};
    double goal_progress{0.0};
    double progress_ratio{1.0};
    int candidate_switch_count{0};
    int branch_switch_count{0};
    int frontier_switch_count{0};
    int guide_direction_switch_count{0};
    int revisit_count{0};
    int replan_count{0};
    int commit_count{0};
    int failure_count{0};
    int keep_current_count{0};
    int recovery_request_count{0};
};

struct NdoDiagnosis {
    NdoState state{NdoState::STABLE};
    std::string reason{"stable"};
    NdoMetrics metrics;
    std::vector<int> implicated_candidate_ids;
    std::vector<int> implicated_frontier_ids;
    std::vector<int> implicated_topo_edges;
    std::vector<std::string> implicated_identity_keys;
    std::vector<std::string> implicated_branch_keys;
    int goal_switch_count{0};
    int frontier_switch_count{0};
    int guide_switch_count{0};
    int recovery_request_count{0};
    double robot_progress{0.0};
};

class NavigationMemory {
public:
    struct Config {
        int max_decision_history{16};
        double default_blacklist_ttl{12.0};
        double no_progress_distance{0.3};
        double no_progress_time{3.0};
        int max_switches{4};
    };

    NavigationMemory();
    explicit NavigationMemory(Config config);

    void reset();

    void recordDecision(const DecisionRecord &decision);
    void recordTrace(const DecisionTrace &trace);

    void recordFailure(const std::string &key,
                       FailureReason reason,
                       double stamp,
                       double ttl);

    bool isBlacklisted(const std::string &key, double stamp) const;

    NdoDiagnosis diagnose(double stamp) const;
    NdoMetrics latestMetrics(double stamp) const;

    const std::vector<DecisionRecord> &decisionHistory() const;
    const std::vector<DecisionTrace> &traceHistory() const;
    const std::unordered_map<std::string, FailureRecord> &failures() const;

private:
    bool repeatedSwitching(NdoDiagnosis &diagnosis) const;
    bool noProgress(double stamp, const NdoMetrics &metrics) const;
    bool revisitLoop(const NdoMetrics &metrics) const;
    bool failureLoop(double stamp, NdoDiagnosis &diagnosis) const;
    bool commitChurn(const NdoMetrics &metrics) const;
    bool recoveryLoop(double stamp, NdoDiagnosis &diagnosis) const;
    NdoMetrics computeMetrics(double stamp) const;
    std::string decisionGoalKey(const DecisionRecord &decision) const;
    std::string decisionFrontierKey(const DecisionRecord &decision) const;
    std::string decisionGuideKey(const DecisionRecord &decision) const;
    std::string decisionBranchKey(const DecisionRecord &decision) const;
    void addReason(NdoDiagnosis &diagnosis,
                   NdoState state,
                   const std::string &reason) const;

    Config config_;
    std::vector<DecisionRecord> decisions_;
    std::vector<DecisionTrace> traces_;
    std::unordered_map<std::string, FailureRecord> failures_;
};

const char *toString(FailureReason reason);
const char *toString(NdoState state);
const char *toString(DecisionTraceAction action);

} // namespace general_planner::nhbp
