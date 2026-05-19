#pragma once

#include <vector>

namespace general_planner {
namespace exploration {

class TspSolver {
public:
    struct Config {
        bool use_two_opt{true};
        int two_opt_max_iterations{40};
    };

    explicit TspSolver(Config cfg);

    bool solveOpenTour(const std::vector<std::vector<double>> &cost_matrix,
                       std::vector<int> &order) const;

private:
    void twoOpt(const std::vector<std::vector<double>> &cost_matrix,
                std::vector<int> &order) const;

    static double tourCost(const std::vector<std::vector<double>> &cost_matrix,
                           const std::vector<int> &order);

private:
    Config cfg_;
};

}  // namespace exploration
}  // namespace general_planner

