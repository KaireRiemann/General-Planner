#pragma once

#include <string>
#include <vector>

namespace general_planner {
namespace exploration {

class TspSolver {
public:
    struct Config {
        bool use_two_opt{true};
        int two_opt_max_iterations{40};
        bool use_lkh{false};
        bool lkh_fallback_to_two_opt{true};
        std::string tsp_dir{"/tmp/general_planner_tsp"};
        std::string problem_name{"general_planner_global"};
        std::string lkh_executable;
        int lkh_cost_scale{100};
    };

    explicit TspSolver(Config cfg);

    bool solveOpenTour(const std::vector<std::vector<double>> &cost_matrix,
                       std::vector<int> &order) const;

private:
    bool solveATSPWithLKH(const std::vector<std::vector<double>> &cost_matrix,
                          std::vector<int> &order) const;
    bool solveGreedyTwoOpt(const std::vector<std::vector<double>> &cost_matrix,
                           std::vector<int> &order) const;

    void twoOpt(const std::vector<std::vector<double>> &cost_matrix,
                std::vector<int> &order) const;

    static double tourCost(const std::vector<std::vector<double>> &cost_matrix,
                           const std::vector<int> &order);

private:
    Config cfg_;
};

}  // namespace exploration
}  // namespace general_planner
