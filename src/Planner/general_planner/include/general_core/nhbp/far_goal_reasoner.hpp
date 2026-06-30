#pragma once

#include <string>

#include <general_core/nhbp/sparse_global_map.hpp>
#include <general_core/nhbp/topological_memory.hpp>

namespace general_planner::nhbp {

enum class FarGoalDecisionType {
    DIRECT_GOAL,
    FRONTIER_TOWARD_GOAL,
    TOPOLOGY_RECOVERY,
    UNAVAILABLE
};

struct FarGoalDecision {
    FarGoalDecisionType type{FarGoalDecisionType::UNAVAILABLE};
    bool ready{false};
    general_utils::Vec3f local_goal{general_utils::Vec3f::Zero()};
    std::string reason{"unavailable"};
    double score{0.0};
};

class FarGoalReasoner {
public:
    struct Config {
        bool enable{false};
        double direct_goal_radius{8.0};
        double frontier_search_radius{40.0};
        int max_frontier_candidates{128};
        double travel_weight{1.0};
        double goal_weight{1.5};
    };

    FarGoalReasoner();
    explicit FarGoalReasoner(Config config);

    void configure(Config config);

    FarGoalDecision selectSubgoal(const general_utils::Vec3f &robot_pos,
                                  const general_utils::Vec3f &far_goal,
                                  double stamp,
                                  const SparseGlobalMap *sparse_map,
                                  const TopologicalMemory *topology) const;

private:
    Config config_;
};

const char *toString(FarGoalDecisionType type);

} // namespace general_planner::nhbp
