#pragma once

#include <string>

#include "exploration/topo_graph.hpp"
#include "exploration/tsp_solver.hpp"

namespace general_planner {
namespace exploration {

struct GlobalGuidanceResult {
    bool valid{false};
    std::vector<int> ordered_frontier_ids;
    GlobalRoute route;
    std::string reason;
};

class GlobalGuidancePlanner {
public:
    using Ptr = std::shared_ptr<GlobalGuidancePlanner>;

    struct Config {
        bool enable{true};
        int max_frontiers_in_tour{16};
        double weight_path_cost{1.0};
        double weight_gain{2.0};
        double weight_revisit{0.5};
        bool use_two_opt{true};
        bool keep_current_target{true};
        bool use_lkh{false};
        bool lkh_fallback_to_two_opt{true};
        std::string tsp_dir{"/tmp/general_planner_tsp"};
        std::string tsp_problem_name{"general_planner_global"};
        std::string lkh_executable;
        int lkh_cost_scale{100};
    };

    explicit GlobalGuidancePlanner(Config cfg);

    bool buildGuidance(const super_utils::Vec3f &robot_pos,
                       double current_travel_distance,
                       const std::vector<FrontierRecord> &frontiers,
                       TopoGraph &graph,
                       int current_target_frontier_id,
                       const GlobalRoute &current_route,
                       GlobalGuidanceResult &result);

    void reset();

private:
    double frontierPriorityCost(const FrontierRecord &frontier,
                                double current_travel_distance,
                                double path_cost) const;

private:
    Config cfg_;
    TspSolver tsp_solver_;
    std::vector<int> last_ordered_frontier_ids_;
};

}  // namespace exploration
}  // namespace general_planner
