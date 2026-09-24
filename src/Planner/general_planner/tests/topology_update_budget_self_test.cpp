#include <general_core/exploration/exploration_utils/pointcloud_topo/topology_update_budget.h>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

using Budget = fast_planner::TopologyUpdateBudget<int>;
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  try {
    Budget budget;
    std::vector<Budget::Candidate> duplicate{{1, 1}, {1, 1}, {2, 2}, {2, 2}, {3, 3}, {4, 4}};
    auto selected = budget.select(duplicate, 4);
    std::set<int> unique;
    for (auto index : selected) unique.insert(duplicate[index].key);
    require(unique.size() == 4, "duplicates consumed region budget");

    // Continuous nearby churn must not starve distant doorway/stair links.
    std::set<int> served;
    for (int round = 0; round < 12; ++round) {
      std::vector<Budget::Candidate> work;
      for (int k = 0; k < 8; ++k) work.push_back({100 + round * 8 + k, 0.1});
      for (int k = 0; k < 6; ++k) work.push_back({10 + k, 100.0 + k});
      selected = budget.select(work, 4);
      require(selected.size() == 4, "budget was exceeded or left unused");
      for (auto index : selected)
        if (work[index].key < 100) served.insert(work[index].key);
    }
    require(served.size() == 6, "old distant work was starved by nearby arrivals");

    Budget first, second;
    std::vector<Budget::Candidate> ordered{{1, 1}, {2, 1}, {3, 1}, {4, 1}};
    auto shuffled = ordered;
    std::reverse(shuffled.begin(), shuffled.end());
    auto a = first.select(ordered, 2), b = second.select(shuffled, 2);
    require(a.size() == b.size(), "candidate order changed budget size");
    for (std::size_t i = 0; i < a.size(); ++i)
      require(ordered[a[i]].key == shuffled[b[i]].key, "hash order changed selected work");
    require(budget.select({}, 4).empty(), "removed work was retried");
    require(budget.select(ordered, 0).empty(), "zero budget selected work");
    require(budget.select({{1, std::numeric_limits<double>::quiet_NaN()}}, 4).empty(),
            "non-finite work entered scheduling");
    std::cout << "topology update budget PASS: deduplication, fairness, deterministic order, removal\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
