#include <general_core/exploration/atsp/atsp_tour_planner.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

#include <boost/filesystem.hpp>

#include <general_core/utils/string_utils.hpp>
#include <lkh_tsp_solver/lkh_interface.h>

namespace general_planner::exploration {
namespace {

double elapsedMs(const std::chrono::steady_clock::time_point &start)
{
    const auto elapsed = std::chrono::steady_clock::now() - start;
    return std::chrono::duration<double, std::milli>(elapsed).count();
}

bool validCostMatrix(const ATSPProblem &problem)
{
    const int expected_size = static_cast<int>(problem.candidates.size()) + 1;
    return expected_size >= 1 &&
           problem.depot_index == 0 &&
           problem.directed_cost_matrix.rows() == expected_size &&
           problem.directed_cost_matrix.cols() == expected_size;
}

} // namespace

ATSPTourPlanner::ATSPTourPlanner()
    : ATSPTourPlanner(Config{})
{
}

ATSPTourPlanner::ATSPTourPlanner(Config config)
    : config_(std::move(config))
{
}

ATSPSolution ATSPTourPlanner::solve(const ATSPProblem &problem) const
{
    const auto start = std::chrono::steady_clock::now();
    ATSPSolution solution;
    if (!config_.enable) {
        solution = solveGreedy(problem, "disabled_greedy", true);
        solution.solve_time_ms = elapsedMs(start);
        return solution;
    }
    if (!validCostMatrix(problem)) {
        solution.solver_status = "invalid_problem";
        solution.fallback_used = true;
        solution.solve_time_ms = elapsedMs(start);
        return solution;
    }
    if (problem.candidates.empty()) {
        solution.solver_status = "empty_problem";
        solution.solve_time_ms = elapsedMs(start);
        return solution;
    }

    const std::string solver = core_utils::normalizeToken(config_.solver);
    const bool wants_lkh = solver == "lkh" ||
                           solver == "fy_node_lkh" ||
                           solver == "linked_lkh";
    if (wants_lkh && solveLinkedLkh(problem, solution)) {
        solution.solve_time_ms = elapsedMs(start);
        return solution;
    }
    if ((wants_lkh || solver == "external_lkh") && !config_.external_command.empty()) {
        if (solveExternalLkh(problem, solution)) {
            solution.solve_time_ms = elapsedMs(start);
            return solution;
        }
    }

    solution = solveGreedy(problem,
                           solver.empty() ? "greedy" : solver + "_fallback_greedy",
                           solver != "greedy");
    solution.solve_time_ms = elapsedMs(start);
    return solution;
}

ATSPSolution ATSPTourPlanner::solveGreedy(const ATSPProblem &problem,
                                          const std::string &status,
                                          const bool fallback_used) const
{
    ATSPSolution solution;
    solution.solver_status = status;
    solution.fallback_used = fallback_used;
    if (!validCostMatrix(problem) || problem.candidates.empty()) {
        return solution;
    }

    std::vector<int> remaining;
    remaining.reserve(problem.candidates.size());
    for (int i = 0; i < static_cast<int>(problem.candidates.size()); ++i) {
        remaining.push_back(i);
    }

    int current = problem.depot_index;
    while (!remaining.empty()) {
        auto best_it = remaining.begin();
        double best_cost = std::numeric_limits<double>::infinity();
        for (auto it = remaining.begin(); it != remaining.end(); ++it) {
            const int node = *it + 1;
            double cost = problem.directed_cost_matrix(current, node);
            if (*it < static_cast<int>(problem.node_reward.size())) {
                cost -= problem.node_reward[static_cast<size_t>(*it)];
            }
            if (!std::isfinite(cost)) {
                continue;
            }
            if (cost < best_cost ||
                (std::abs(cost - best_cost) < 1.0e-9 && *it < *best_it)) {
                best_cost = cost;
                best_it = it;
            }
        }

        const int next_candidate_index = *best_it;
        const int next_node = next_candidate_index + 1;
        solution.total_cost += problem.directed_cost_matrix(current, next_node);
        solution.ordered_candidate_ids.push_back(
                problem.candidates[static_cast<size_t>(next_candidate_index)].id);
        current = next_node;
        remaining.erase(best_it);
    }
    return solution;
}

bool ATSPTourPlanner::solveExternalLkh(const ATSPProblem &problem,
                                       ATSPSolution &solution) const
{
    boost::system::error_code ec;
    boost::filesystem::create_directories(config_.work_dir, ec);
    if (ec) {
        return false;
    }

    const std::string problem_file = config_.work_dir + "/single.tsp";
    const std::string parameter_file = config_.work_dir + "/single.par";
    const std::string tour_file = config_.work_dir + "/single.txt";

    if (!writeFyNodeTsplibProblem(problem.directed_cost_matrix, problem_file) ||
        !writeFyNodeLkhParameterFile(problem_file, tour_file, parameter_file)) {
        return false;
    }

    std::ostringstream cmd;
    cmd << config_.external_command << " " << parameter_file;
    const int ret = std::system(cmd.str().c_str());
    if (ret != 0) {
        return false;
    }
    if (!readFyNodeTourResult(tour_file, problem, solution)) {
        return false;
    }
    solution.solver_status = "external_lkh";
    solution.fallback_used = false;
    return true;
}

bool ATSPTourPlanner::solveLinkedLkh(const ATSPProblem &problem,
                                     ATSPSolution &solution) const
{
    boost::system::error_code ec;
    boost::filesystem::create_directories(config_.work_dir, ec);
    if (ec) {
        return false;
    }

    const std::string problem_file = config_.work_dir + "/single.tsp";
    const std::string parameter_file = config_.work_dir + "/single.par";
    const std::string tour_file = config_.work_dir + "/single.txt";

    if (!writeFyNodeTsplibProblem(problem.directed_cost_matrix, problem_file) ||
        !writeFyNodeLkhParameterFile(problem_file, tour_file, parameter_file)) {
        return false;
    }

    const int ret = solveTSPLKH(parameter_file.c_str());
    if (ret != 0) {
        return false;
    }
    if (!readFyNodeTourResult(tour_file, problem, solution)) {
        return false;
    }
    solution.solver_status = "linked_lkh";
    solution.fallback_used = false;
    return true;
}

bool ATSPTourPlanner::writeFyNodeTsplibProblem(const Eigen::MatrixXd &cost_matrix,
                                               const std::string &problem_file) const
{
    std::ofstream out(problem_file);
    if (!out.is_open()) {
        return false;
    }

    const int dimension = static_cast<int>(cost_matrix.rows());
    out << "NAME : single\n"
        << "TYPE : ATSP\n"
        << "DIMENSION : " << dimension << "\n"
        << "EDGE_WEIGHT_TYPE : EXPLICIT\n"
        << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
        << "EDGE_WEIGHT_SECTION\n";

    const int scale = std::max(1, config_.cost_scale);
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
            const double raw_cost = cost_matrix(i, j);
            const int int_cost = std::isfinite(raw_cost)
                                         ? static_cast<int>(std::round(raw_cost * scale))
                                         : std::numeric_limits<int>::max() / 4;
            out << int_cost << " ";
        }
        out << "\n";
    }
    out << "EOF";
    return true;
}

bool ATSPTourPlanner::writeFyNodeLkhParameterFile(const std::string &problem_file,
                                                  const std::string &tour_file,
                                                  const std::string &parameter_file) const
{
    std::ofstream out(parameter_file);
    if (!out.is_open()) {
        return false;
    }
    out << "PROBLEM_FILE = " << problem_file << "\n";
    out << "GAIN23 = NO\n";
    out << "OUTPUT_TOUR_FILE = " << tour_file << "\n";
    out << "RUNS = 1\n";
    if (config_.time_budget_ms > 0) {
        const double seconds = std::max(1.0, static_cast<double>(config_.time_budget_ms) / 1000.0);
        out << "TIME_LIMIT = " << seconds << "\n";
    }
    return true;
}

bool ATSPTourPlanner::readFyNodeTourResult(const std::string &tour_file,
                                           const ATSPProblem &problem,
                                           ATSPSolution &solution) const
{
    std::ifstream in(tour_file);
    if (!in.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(in, line)) {
        if (line == "TOUR_SECTION") {
            break;
        }
    }
    if (!in.good()) {
        return false;
    }

    solution.ordered_candidate_ids.clear();
    int previous_node = problem.depot_index;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        const int lkh_id = std::stoi(line);
        if (lkh_id == -1) {
            break;
        }
        if (lkh_id == 1) {
            previous_node = 0;
            continue;
        }
        const int candidate_index = lkh_id - 2;
        if (candidate_index < 0 ||
            candidate_index >= static_cast<int>(problem.candidates.size())) {
            continue;
        }
        const int node = candidate_index + 1;
        solution.total_cost += problem.directed_cost_matrix(previous_node, node);
        solution.ordered_candidate_ids.push_back(
                problem.candidates[static_cast<size_t>(candidate_index)].id);
        previous_node = node;
    }
    return !solution.ordered_candidate_ids.empty();
}

} // namespace general_planner::exploration
