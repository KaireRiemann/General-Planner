#include "exploration/tsp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <boost/filesystem.hpp>

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
    if (cfg_.use_lkh && solveATSPWithLKH(cost_matrix, order)) {
        return order.size() > 1U;
    }
    if (cfg_.use_lkh && !cfg_.lkh_fallback_to_two_opt) {
        return false;
    }
    return solveGreedyTwoOpt(cost_matrix, order);
}

bool TspSolver::solveATSPWithLKH(const std::vector<std::vector<double>> &cost_matrix,
                                 std::vector<int> &order) const {
    order.clear();
    const int dimension = static_cast<int>(cost_matrix.size());
    if (dimension < 3 || cfg_.lkh_executable.empty()) {
        return false;
    }
    for (const auto &row : cost_matrix) {
        if (static_cast<int>(row.size()) != dimension) {
            return false;
        }
    }

    boost::system::error_code ec;
    boost::filesystem::create_directories(cfg_.tsp_dir, ec);
    if (ec) {
        return false;
    }

    const std::string stem = cfg_.tsp_dir + "/" + cfg_.problem_name;
    const std::string tsp_path = stem + ".tsp";
    const std::string par_path = stem + ".par";
    const std::string out_path = stem + ".txt";
    {
        std::ofstream prob_file(tsp_path);
        if (!prob_file.is_open()) {
            return false;
        }
        prob_file << "NAME : " << cfg_.problem_name << "\n"
                  << "TYPE : ATSP\n"
                  << "DIMENSION : " << dimension << "\n"
                  << "EDGE_WEIGHT_TYPE : EXPLICIT\n"
                  << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
                  << "EDGE_WEIGHT_SECTION\n";
        const int scale = std::max(1, cfg_.lkh_cost_scale);
        for (int i = 0; i < dimension; ++i) {
            for (int j = 0; j < dimension; ++j) {
                const double raw_cost = cost_matrix[static_cast<std::size_t>(i)]
                                                   [static_cast<std::size_t>(j)];
                const double bounded = std::isfinite(raw_cost) ? std::max(0.0, raw_cost) : 1.0e6;
                const long long int_cost = static_cast<long long>(std::llround(bounded * scale));
                prob_file << int_cost << " ";
            }
            prob_file << "\n";
        }
        prob_file << "EOF\n";
    }
    {
        std::ofstream par_file(par_path);
        if (!par_file.is_open()) {
            return false;
        }
        par_file << "PROBLEM_FILE = " << tsp_path << "\n"
                 << "OUTPUT_TOUR_FILE = " << out_path << "\n"
                 << "RUNS = 1\n"
                 << "TRACE_LEVEL = 0\n";
    }

    const std::string command = cfg_.lkh_executable + " " + par_path + " > /dev/null 2>&1";
    if (std::system(command.c_str()) != 0) {
        return false;
    }

    std::ifstream res_file(out_path);
    if (!res_file.is_open()) {
        return false;
    }
    std::string line;
    while (std::getline(res_file, line)) {
        if (line == "TOUR_SECTION") {
            break;
        }
    }
    if (!res_file.good()) {
        return false;
    }

    std::vector<int> tour;
    while (std::getline(res_file, line)) {
        std::stringstream ss(line);
        int id = -1;
        ss >> id;
        if (!ss || id == -1) {
            break;
        }
        const int zero_based = id - 1;
        if (zero_based >= 0 && zero_based < dimension) {
            tour.push_back(zero_based);
        }
    }
    if (tour.size() < 2U) {
        return false;
    }

    const auto start_it = std::find(tour.begin(), tour.end(), 0);
    if (start_it != tour.end()) {
        std::rotate(tour.begin(), start_it, tour.end());
    } else {
        tour.insert(tour.begin(), 0);
    }

    std::unordered_set<int> used;
    order.clear();
    order.reserve(tour.size());
    for (const int idx : tour) {
        if (idx < 0 || idx >= dimension || used.find(idx) != used.end()) {
            continue;
        }
        used.insert(idx);
        order.push_back(idx);
    }
    return order.size() > 1U && order.front() == 0;
}

bool TspSolver::solveGreedyTwoOpt(const std::vector<std::vector<double>> &cost_matrix,
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
