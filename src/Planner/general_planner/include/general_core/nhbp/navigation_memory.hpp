#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

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
    general_utils::Vec3f position{general_utils::Vec3f::Zero()};
    double stamp{0.0};
    double score{0.0};
};

struct NdoDiagnosis {
    NdoState state{NdoState::STABLE};
    std::string reason{"stable"};
    std::vector<int> implicated_candidate_ids;
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

    void recordFailure(const std::string &key,
                       FailureReason reason,
                       double stamp,
                       double ttl);

    bool isBlacklisted(const std::string &key, double stamp) const;

    NdoDiagnosis diagnose(double stamp) const;

    const std::vector<DecisionRecord> &decisionHistory() const;
    const std::unordered_map<std::string, FailureRecord> &failures() const;

private:
    bool repeatedSwitching() const;
    bool noProgress(double stamp) const;

    Config config_;
    std::vector<DecisionRecord> decisions_;
    std::unordered_map<std::string, FailureRecord> failures_;
};

const char *toString(FailureReason reason);
const char *toString(NdoState state);

} // namespace general_planner::nhbp
