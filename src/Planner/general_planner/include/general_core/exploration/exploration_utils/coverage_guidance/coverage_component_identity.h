#pragma once
#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

namespace fast_planner {
// Worker-owned identity tracking using actual voxel overlap. On split the
// largest overlap retains the ID; on merge one predecessor survives. Old IDs
// are never reassigned to an unrelated component or to another voxel state.
class CoverageComponentIdentity {
 public:
  std::vector<std::uint64_t> update(const std::vector<int> &labels,
                                    const std::vector<int> &states) {
    struct Match {int count, zone; std::uint64_t id;};
    std::map<std::pair<int,std::uint64_t>,int> overlaps;
    for (std::size_t i=0; i<labels.size() && i<previous_.size(); ++i) {
      const int z=labels[i];
      if (z>=0 && z<static_cast<int>(states.size()) && previous_[i] &&
          previous_states_[i]==states[z]) ++overlaps[{z,previous_[i]}];
    }
    std::vector<Match> matches;
    for (const auto &m:overlaps) matches.push_back({m.second,m.first.first,m.first.second});
    std::sort(matches.begin(),matches.end(),[](const Match &a,const Match &b){
      if (a.count!=b.count) return a.count>b.count;
      if (a.id!=b.id) return a.id<b.id;
      return a.zone<b.zone;
    });
    std::vector<std::uint64_t> ids(states.size(),0); std::set<std::uint64_t> used;
    for (const auto &m:matches) if (!ids[m.zone] && !used.count(m.id)) {
      ids[m.zone]=m.id; used.insert(m.id);
    }
    for (auto &id:ids) if (!id) id=next_++;
    previous_.assign(labels.size(),0); previous_states_.assign(labels.size(),-1);
    for (std::size_t i=0;i<labels.size();++i) if (labels[i]>=0 && labels[i]<static_cast<int>(ids.size())) {
      previous_[i]=ids[labels[i]]; previous_states_[i]=states[labels[i]];
    }
    return ids;
  }
 private:
  std::uint64_t next_{1};
  std::vector<std::uint64_t> previous_;
  std::vector<int> previous_states_;
};
}  // namespace fast_planner
