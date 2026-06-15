#include <general_core/exploration_coverage_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace general_planner {

CoveragePathPlanner::CoveragePathPlanner()
        : CoveragePathPlanner(Config{}) {
}

CoveragePathPlanner::CoveragePathPlanner(const Config &cfg)
        : cfg_(cfg) {
}

void CoveragePathPlanner::reset() {
    order_cost_.clear();
}

double CoveragePathPlanner::routeCost(const ExplorationRegionGraph &graph,
                                      const std::vector<int> &route) const {
    if (route.empty()) {
        return 0.0;
    }
    double cost = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i) {
        cost += graph.graphDistanceCost(route[i - 1], route[i]);
    }
    for (std::size_t i = 0; i < route.size(); ++i) {
        cost += static_cast<double>(i) * 0.05 -
                0.01 * graph.regionCoveragePriority(route[i]);
    }
    return cost;
}

bool CoveragePathPlanner::planRegionRoute(const ExplorationRegionGraph &graph,
                                          const int start_region,
                                          std::vector<int> &region_route) {
    region_route.clear();
    order_cost_.clear();

    std::vector<int> active = graph.activeRegionIds();
    if (active.empty()) {
        return false;
    }
    std::sort(active.begin(), active.end(), [&](const int lhs, const int rhs) {
        const double lp = graph.regionCoveragePriority(lhs);
        const double rp = graph.regionCoveragePriority(rhs);
        if (std::abs(lp - rp) > 1.0e-9) return lp > rp;
        return lhs < rhs;
    });
    if (cfg_.max_region_num > 0 &&
        static_cast<int>(active.size()) > cfg_.max_region_num) {
        active.resize(static_cast<std::size_t>(cfg_.max_region_num));
    }

    if (!cfg_.enable) {
        region_route = active;
    } else {
        std::unordered_set<int> remaining(active.begin(), active.end());
        int current = start_region > 0 ? start_region : active.front();
        if (remaining.find(current) != remaining.end()) {
            region_route.push_back(current);
            remaining.erase(current);
        }
        while (!remaining.empty()) {
            double best_score = std::numeric_limits<double>::infinity();
            int best_id = -1;
            std::vector<int> candidates(remaining.begin(), remaining.end());
            std::sort(candidates.begin(), candidates.end());
            for (const int candidate : candidates) {
                const double score =
                        cfg_.weight_graph_distance * graph.graphDistanceCost(current, candidate) +
                        cfg_.weight_region_priority * graph.regionCoveragePriority(candidate);
                if (score < best_score ||
                    (std::abs(score - best_score) < 1.0e-9 && candidate < best_id)) {
                    best_score = score;
                    best_id = candidate;
                }
            }
            if (best_id < 0) {
                break;
            }
            region_route.push_back(best_id);
            remaining.erase(best_id);
            current = best_id;
        }

        if (cfg_.use_two_opt && region_route.size() > 3) {
            bool improved = true;
            while (improved) {
                improved = false;
                for (std::size_t i = 1; i + 2 < region_route.size(); ++i) {
                    for (std::size_t j = i + 1; j + 1 < region_route.size(); ++j) {
                        std::vector<int> candidate = region_route;
                        std::reverse(candidate.begin() + static_cast<long>(i),
                                     candidate.begin() + static_cast<long>(j + 1));
                        if (routeCost(graph, candidate) + 1.0e-6 < routeCost(graph, region_route)) {
                            region_route.swap(candidate);
                            improved = true;
                        }
                    }
                }
            }
        }
    }

    for (std::size_t i = 0; i < region_route.size(); ++i) {
        order_cost_[region_route[i]] = static_cast<double>(i);
    }
    return !region_route.empty();
}

double CoveragePathPlanner::orderCost(const int region_id) const {
    const auto it = order_cost_.find(region_id);
    return it == order_cost_.end() ? 1000.0 : it->second;
}

}  // namespace general_planner
