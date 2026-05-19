#include "exploration/global_guidance_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace general_planner {
namespace exploration {

GlobalGuidancePlanner::GlobalGuidancePlanner(Config cfg)
        : cfg_(std::move(cfg)),
          tsp_solver_(TspSolver::Config{cfg_.use_two_opt, 40}) {}

bool GlobalGuidancePlanner::buildGuidance(const super_utils::Vec3f &robot_pos,
                                          const double current_travel_distance,
                                          const std::vector<FrontierRecord> &frontiers,
                                          TopoGraph &graph,
                                          const int current_target_frontier_id,
                                          const GlobalRoute &current_route,
                                          GlobalGuidanceResult &result) {
    result = GlobalGuidanceResult{};
    if (!cfg_.enable) {
        result.reason = "global guidance disabled";
        return false;
    }
    if (cfg_.keep_current_target &&
        current_target_frontier_id >= 0 &&
        current_route.valid) {
        const auto it = std::find_if(frontiers.begin(), frontiers.end(),
                                     [current_target_frontier_id](const FrontierRecord &frontier) {
                                         return frontier.stable_id == current_target_frontier_id &&
                                                frontierStateSelectable(frontier.state);
                                     });
        if (it != frontiers.end()) {
            result.valid = true;
            result.ordered_frontier_ids.push_back(current_target_frontier_id);
            result.route = current_route;
            result.reason = "keep current target";
            return true;
        }
    }

    struct Candidate {
        FrontierRecord frontier;
        GlobalRoute route;
        double priority{0.0};
    };
    std::vector<Candidate> candidates;
    candidates.reserve(frontiers.size());
    for (const auto &frontier : frontiers) {
        if (!frontierStateSelectable(frontier.state) ||
            !frontier.has_reachable_viewpoint) {
            continue;
        }
        GlobalRoute route;
        if (!graph.graphSearchToFrontier(frontier.stable_id, route)) {
            continue;
        }
        Candidate candidate;
        candidate.frontier = frontier;
        candidate.route = route;
        candidate.priority = frontierPriorityCost(frontier,
                                                  current_travel_distance,
                                                  route.cost);
        candidates.push_back(candidate);
    }

    if (candidates.empty()) {
        result.reason = "no reachable frontier for global guidance";
        return false;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &a, const Candidate &b) {
                  return a.priority < b.priority;
              });
    if (static_cast<int>(candidates.size()) > cfg_.max_frontiers_in_tour) {
        candidates.resize(static_cast<std::size_t>(cfg_.max_frontiers_in_tour));
    }

    const int n = static_cast<int>(candidates.size());
    std::vector<std::vector<double>> cost_matrix(static_cast<std::size_t>(n + 1),
                                                 std::vector<double>(static_cast<std::size_t>(n + 1),
                                                                     std::numeric_limits<double>::infinity()));
    for (int i = 0; i <= n; ++i) {
        cost_matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = 0.0;
    }
    for (int i = 0; i < n; ++i) {
        cost_matrix[0][static_cast<std::size_t>(i + 1)] = candidates[static_cast<std::size_t>(i)].route.cost;
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) {
                continue;
            }
            double graph_cost = std::numeric_limits<double>::infinity();
            if (!graph.graphSearchBetweenFrontiers(candidates[static_cast<std::size_t>(i)].frontier.stable_id,
                                                   candidates[static_cast<std::size_t>(j)].frontier.stable_id,
                                                   graph_cost)) {
                graph_cost = (candidates[static_cast<std::size_t>(i)].frontier.best_viewpoint.position -
                              candidates[static_cast<std::size_t>(j)].frontier.best_viewpoint.position).norm() * 1.5;
            }
            const double gain_bias = cfg_.weight_gain *
                                     candidates[static_cast<std::size_t>(j)].frontier.best_viewpoint.gain_norm;
            cost_matrix[static_cast<std::size_t>(i + 1)][static_cast<std::size_t>(j + 1)] =
                    cfg_.weight_path_cost * graph_cost -
                    gain_bias +
                    cfg_.weight_revisit *
                    static_cast<double>(candidates[static_cast<std::size_t>(j)].frontier.selected_count);
        }
    }

    std::vector<int> order;
    if (!tsp_solver_.solveOpenTour(cost_matrix, order)) {
        result.reason = "tsp solver failed";
        return false;
    }

    for (const int idx : order) {
        if (idx == 0) {
            continue;
        }
        result.ordered_frontier_ids.push_back(candidates[static_cast<std::size_t>(idx - 1)].frontier.stable_id);
    }
    if (result.ordered_frontier_ids.empty()) {
        result.reason = "empty tour";
        return false;
    }

    const int first_id = result.ordered_frontier_ids.front();
    const auto route_it = std::find_if(candidates.begin(), candidates.end(),
                                       [first_id](const Candidate &candidate) {
                                           return candidate.frontier.stable_id == first_id;
                                       });
    if (route_it == candidates.end()) {
        result.reason = "missing first route";
        return false;
    }
    result.valid = true;
    result.route = route_it->route;
    result.reason = "global guidance ready";
    last_ordered_frontier_ids_ = result.ordered_frontier_ids;
    (void)robot_pos;
    return true;
}

void GlobalGuidancePlanner::reset() {
    last_ordered_frontier_ids_.clear();
}

double GlobalGuidancePlanner::frontierPriorityCost(const FrontierRecord &frontier,
                                                   const double current_travel_distance,
                                                   const double path_cost) const {
    const double backtrack = std::max(0.0, current_travel_distance - frontier.generated_travel_distance);
    const double generated_to_center = (frontier.center - frontier.generated_position).norm();
    const double revisit_penalty = cfg_.weight_revisit * static_cast<double>(frontier.selected_count);
    const double gain_reward = cfg_.weight_gain * frontier.best_viewpoint.gain_norm;
    return 0.5 * (backtrack + generated_to_center) +
           cfg_.weight_path_cost * path_cost +
           revisit_penalty -
           gain_reward;
}

}  // namespace exploration
}  // namespace general_planner
