#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <general_core/exploration_region_graph.hpp>

namespace general_planner {

class CoveragePathPlanner {
public:
    using Ptr = std::shared_ptr<CoveragePathPlanner>;

    struct Config {
        bool enable{true};
        int max_region_num{64};
        double weight_region_priority{-1.0};
        double weight_graph_distance{1.0};
        double weight_revisit{2.0};
        bool use_two_opt{true};
    };

    CoveragePathPlanner();
    explicit CoveragePathPlanner(const Config &cfg);

    bool planRegionRoute(const ExplorationRegionGraph &graph,
                         int start_region,
                         std::vector<int> &region_route);

    double orderCost(int region_id) const;

    void reset();

private:
    double routeCost(const ExplorationRegionGraph &graph,
                     const std::vector<int> &route) const;

    Config cfg_;
    std::unordered_map<int, double> order_cost_;
};

}  // namespace general_planner
