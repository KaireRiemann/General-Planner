#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>

namespace general_planner::exploration {

struct ATSPCandidate {
    int id{-1};
};

struct ATSPProblem {
    std::vector<ATSPCandidate> candidates;
    Eigen::MatrixXd directed_cost_matrix;
    std::vector<double> node_reward;
    int depot_index{0};
    int time_budget_ms{30};
};

struct ATSPSolution {
    std::vector<int> ordered_candidate_ids;
    double total_cost{0.0};
    std::string solver_status{"not_started"};
    double solve_time_ms{0.0};
    bool fallback_used{false};
};

class ATSPTourPlanner {
public:
    struct Config {
        bool enable{false};
        std::string solver{"greedy"};
        std::string work_dir{"/tmp/general_planner_atsp"};
        std::string external_command;
        int cost_scale{100};
        int time_budget_ms{30};
        int max_candidate_num{96};
    };

    ATSPTourPlanner();
    explicit ATSPTourPlanner(Config config);

    ATSPSolution solve(const ATSPProblem &problem) const;

private:
    ATSPSolution solveGreedy(const ATSPProblem &problem,
                             const std::string &status,
                             bool fallback_used) const;

    bool solveExternalLkh(const ATSPProblem &problem,
                          ATSPSolution &solution) const;

    bool solveLinkedLkh(const ATSPProblem &problem,
                        ATSPSolution &solution) const;

    bool writeFyNodeTsplibProblem(const Eigen::MatrixXd &cost_matrix,
                                  const std::string &problem_file) const;

    bool writeFyNodeLkhParameterFile(const std::string &problem_file,
                                     const std::string &tour_file,
                                     const std::string &parameter_file) const;

    bool readFyNodeTourResult(const std::string &tour_file,
                              const ATSPProblem &problem,
                              ATSPSolution &solution) const;

    Config config_;
};

} // namespace general_planner::exploration
