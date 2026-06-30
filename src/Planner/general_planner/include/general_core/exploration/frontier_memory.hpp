#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <general_core/exploration/exploration_frontend.hpp>

namespace general_planner {

enum class FrontierMemoryState {
    ACTIVE,
    COMMITTED,
    FAILED,
    COVERED
};

struct FrontierMemoryRecord {
    ExplorationGoal goal;
    FrontierMemoryState state{FrontierMemoryState::ACTIVE};
    int observe_count{0};
    int failure_count{0};
    double first_seen_stamp{0.0};
    double last_seen_stamp{0.0};
    double last_state_stamp{0.0};
    double blocked_until{0.0};
};

class FrontierMemory {
public:
    struct Config {
        bool enable{false};
        int max_records{256};
        double record_ttl{45.0};
        double failure_ttl{12.0};
        double failure_block_radius{1.8};
        double covered_radius{0.8};
        double recovery_min_distance{0.8};
    };

    FrontierMemory();
    explicit FrontierMemory(Config config);

    void reset();

    void observe(const ExplorationGoal &goal, double stamp);
    void markCommitted(const ExplorationGoal &goal, double stamp);
    void markFailed(const ExplorationGoal &goal, double stamp);
    void markCoveredNear(const general_utils::Vec3f &position, double stamp);
    void prune(double stamp);

    bool blocked(const ExplorationGoal &goal, double stamp) const;
    bool hasRecoverableGoal(const general_utils::Vec3f &robot_pos,
                            double stamp,
                            ExplorationGoal &goal,
                            const std::function<bool(const ExplorationGoal &)> &accept =
                                    std::function<bool(const ExplorationGoal &)>{}) const;

    int activeCount(double stamp) const;
    int failedCount(double stamp) const;
    int coveredCount() const;

private:
    std::string keyForGoal(const ExplorationGoal &goal) const;
    FrontierMemoryRecord &upsert(const ExplorationGoal &goal, double stamp);
    bool expired(const FrontierMemoryRecord &record, double stamp) const;
    bool failedRegionBlocked(const ExplorationGoal &goal, double stamp) const;

    Config config_;
    std::unordered_map<std::string, FrontierMemoryRecord> records_;
};

const char *toString(FrontierMemoryState state);

} // namespace general_planner
