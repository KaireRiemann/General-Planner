#include "exploration/tsp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

namespace general_planner {
namespace exploration {

TspSolver::TspSolver(Config cfg) : cfg_(std::move(cfg)) {}

bool TspSolver::solveOpenTour(const std::vector<std::vector<double>> &cost_matrix,
                              std::vector<int> &order) const {
    order.clear();
    const int n = static_cast<int>(cost_matrix.size());
    if (n <= 1) {
        return false;
    }
    std::vector<char> used(static_cast<std::size_t>(n), 0);
    int current = 0;
    order.push_back(0);
    used[0] = 1;
    for (int step = 1; step < n; ++step) {
        int best = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for (int candidate = 1; candidate < n; ++candidate) {
            if (used[static_cast<std::size_t>(candidate)] != 0) {
                continue;
            }
            const double cost = cost_matrix[static_cast<std::size_t>(current)]
                                          [static_cast<std::size_t>(candidate)];
            if (std::isfinite(cost) && cost < best_cost) {
                best_cost = cost;
                best = candidate;
            }
        }
        if (best < 0) {
            return order.size() > 1U;
        }
        order.push_back(best);
        used[static_cast<std::size_t>(best)] = 1;
        current = best;
    }
    if (cfg_.use_two_opt && order.size() > 3U) {
        twoOpt(cost_matrix, order);
    }
    return order.size() > 1U;
}

void TspSolver::twoOpt(const std::vector<std::vector<double>> &cost_matrix,
                       std::vector<int> &order) const {
    bool improved = true;
    int iterations = 0;
    while (improved && iterations++ < cfg_.two_opt_max_iterations) {
        improved = false;
        for (std::size_t i = 1; i + 2 < order.size(); ++i) {
            for (std::size_t j = i + 1; j + 1 < order.size(); ++j) {
                std::vector<int> candidate = order;
                std::reverse(candidate.begin() + static_cast<std::ptrdiff_t>(i),
                             candidate.begin() + static_cast<std::ptrdiff_t>(j + 1));
                if (tourCost(cost_matrix, candidate) + 1.0e-6 <
                    tourCost(cost_matrix, order)) {
                    order.swap(candidate);
                    improved = true;
                }
            }
        }
    }
}

double TspSolver::tourCost(const std::vector<std::vector<double>> &cost_matrix,
                           const std::vector<int> &order) {
    double cost = 0.0;
    for (std::size_t i = 1; i < order.size(); ++i) {
        const double edge = cost_matrix[static_cast<std::size_t>(order[i - 1])]
                                      [static_cast<std::size_t>(order[i])];
        if (!std::isfinite(edge)) {
            return std::numeric_limits<double>::infinity();
        }
        cost += edge;
    }
    return cost;
}

}  // namespace exploration
}  // namespace general_planner
