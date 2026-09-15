#pragma once
#include <algorithm>
#include <vector>
namespace fast_planner {
// Keep a near-field prefix and reserve the remaining capacity for coverage
// intent / overdue tasks. Inputs are already deterministically distance-sorted.
inline std::vector<int> coverageCandidateWindow(const std::vector<int> &order,
    const std::vector<bool> &preferred,int capacity) {
  capacity=std::max(0,capacity);
  std::vector<int> result;
  auto append=[&](int index) {
    if (result.size()<static_cast<std::size_t>(capacity) &&
        std::find(result.begin(),result.end(),index)==result.end()) result.push_back(index);
  };
  const int near_count=(capacity+1)/2;
  for (int i=0;i<std::min(near_count,static_cast<int>(order.size()));++i) append(order[i]);
  for (int index:order) if (index>=0 && index<static_cast<int>(preferred.size()) && preferred[index]) append(index);
  for (int index:order) append(index);
  return result;
}
}  // namespace fast_planner
