#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

namespace fast_planner {

// Reserve part of each maintenance pass for work that has waited. Keys are
// geometric identities, so regenerated temporary nodes retain their place.
// Only work offered in the current pass is retained; stale nodes are not owned.
template <class Key> class TopologyUpdateBudget {
public:
  struct Candidate {
    Key key;
    double distance;
  };

  std::vector<std::size_t> select(const std::vector<Candidate> &candidates,
                                std::size_t limit) {
    ++epoch_;
    struct Entry {
      Key key;
      double distance;
      std::size_t index;
      std::uint64_t since;
    };
    std::map<Key, Entry> unique;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const auto &candidate = candidates[i];
      if (!std::isfinite(candidate.distance)) continue;
      const auto pending = pending_.find(candidate.key);
      const auto since = pending == pending_.end() ? epoch_ : pending->second;
      auto inserted = unique.emplace(candidate.key, Entry{
          candidate.key, candidate.distance, i, since});
      if (!inserted.second && candidate.distance < inserted.first->second.distance)
        inserted.first->second = {candidate.key, candidate.distance, i, since};
    }
    pending_.clear();
    std::vector<Entry> entries;
    for (const auto &item : unique) {
      entries.push_back(item.second);
      pending_.emplace(item.first, item.second.since);
    }
    const auto by_distance = [](const Entry &a, const Entry &b) {
      if (a.distance != b.distance) return a.distance < b.distance;
      return a.key < b.key;
    };
    std::sort(entries.begin(), entries.end(), by_distance);
    limit = std::min(limit, entries.size());
    const std::size_t nearest_count = limit - (limit + 3) / 4;
    // The oldest quarter prevents a persistent distant connection from being
    // starved by new, short connections. Distance breaks equal-age ties.
    std::stable_sort(entries.begin() + nearest_count, entries.end(),
        [&](const Entry &a, const Entry &b) {
          if (a.since != b.since) return a.since < b.since;
          return by_distance(a, b);
        });
    std::vector<std::size_t> selected;
    selected.reserve(limit);
    for (std::size_t i = 0; i < limit; ++i) {
      selected.push_back(entries[i].index);
      pending_.erase(entries[i].key);
    }
    return selected;
  }

private:
  std::uint64_t epoch_{0};
  std::map<Key, std::uint64_t> pending_;
};

}  // namespace fast_planner
