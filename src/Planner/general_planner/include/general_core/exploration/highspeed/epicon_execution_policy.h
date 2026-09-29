#pragma once
#include <algorithm>
#include <cstdint>

namespace fast_planner {
// A native NO_FRONTIER is provisional. Mission completion needs independent
// sensor observations acquired after the last executable trajectory settled.
class EpiconFinishVerifier {
public:
  void reset() { settled_since_ = first_empty_ = -1.0; revision_ = 0; count_ = 0; }
  bool observe(double now, bool settled, bool fresh, bool audited_empty,
               std::uint64_t revision, double acquisition_time,
               int min_count, double min_duration) {
    if (!settled || !fresh) { reset(); return false; }
    if (settled_since_ < 0.0) settled_since_ = now;
    if (!audited_empty) {
      first_empty_ = -1.0; count_ = 0; revision_ = revision;
      return false;
    }
    if (acquisition_time <= settled_since_ || revision <= revision_) return false;
    revision_ = revision;
    if (count_++ == 0) first_empty_ = now;
    return count_ >= std::max(2, min_count) &&
           now - first_empty_ >= std::max(0.0, min_duration);
  }
  int count() const { return count_; }
private:
  double settled_since_{-1.0}, first_empty_{-1.0};
  std::uint64_t revision_{0};
  int count_{0};
};
}
