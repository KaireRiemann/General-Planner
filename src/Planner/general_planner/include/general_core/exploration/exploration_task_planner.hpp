#pragma once

#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <general_core/config.hpp>
#include <general_core/exploration/atsp/atsp_tour_planner.hpp>
#include <general_core/exploration/exploration_frontend.hpp>
#include <general_core/exploration/exploration_frontier_db.hpp>
#include <general_core/nhbp/nav_identity.hpp>

namespace general_planner {

class ExplorationTaskPlanner {
public:
    using PairwiseCostFn =
            std::function<double(const ExplorationGoal &, const ExplorationGoal &)>;
    using GoalCostFn = std::function<double(const ExplorationGoal &)>;
    using GoalKeyFn = std::function<std::string(const ExplorationGoal &)>;
    using GoalRefFn = std::function<general_utils::Vec3f(const ExplorationGoal &)>;

    struct Request {
        const ExplorationCandidateSet *candidate_set{nullptr};
        const general_utils::vec_E<ExplorationFrontierDB::ObjectSnapshot> *frontier_objects{
                nullptr};
        general_utils::Vec3f robot_pos{general_utils::Vec3f::Zero()};
        double current_yaw{0.0};
        double stamp{0.0};
        PairwiseCostFn pairwise_cost;
        GoalCostFn start_cost;
        GoalCostFn node_penalty;
        GoalKeyFn goal_key;
        GoalKeyFn sector_key;
        GoalRefFn sector_reference;
    };

    struct Plan {
        bool valid{false};
        general_utils::vec_E<ExplorationGoal> ordered_goals;
        std::string reason;
        int frontier_object_count{0};
        int sector_count{0};
        int raw_candidate_count{0};
        int representative_count{0};
    };

    explicit ExplorationTaskPlanner(const Config &cfg);

    void reset();

    bool plan(const Request &request, Plan &plan_out) const;

private:
    using FrontierObject = ExplorationFrontierDB::ObjectSnapshot;

    double objectScore(const FrontierObject &object) const;

    ExplorationGoal representativeGoal(const FrontierObject &object) const;

    ExplorationGoal refineViewpointForObject(const FrontierObject &object,
                                             const ExplorationGoal *previous_goal,
                                             const Request &request,
                                             int tour_rank) const;

    void solveObjectTour(const Request &request,
                         const std::vector<FrontierObject> &objects,
                         std::vector<int> &ordered_indices,
                         std::string &reason) const;

    const Config &cfg_;
};

} // namespace general_planner
