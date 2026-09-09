#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace general_planner {
inline double planningWallSeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline int planningFailureLimit(int configured) {
  // Legacy zero meant unlimited retries. It must not disable this safeguard.
  return configured > 0 ? std::min(configured, 100) : 5;
}
inline double planningRetryBackoff(double configured) {
  return std::isfinite(configured) ? std::clamp(configured, 0.05, 2.0) : 0.25;
}
class PlanningDeadline {
 public:
  bool expired(bool planning, std::uint64_t epoch, std::uint64_t sequence,
               double now, double timeout) {
    if (!planning) { active_ = false; return false; }
    if (!active_ || epoch != epoch_ || sequence != sequence_) {
      active_ = true; epoch_ = epoch; sequence_ = sequence; start_ = now;
    }
    const double bound = std::isfinite(timeout) ? std::clamp(timeout, 0.5, 60.0) : 5.0;
    return now - start_ >= bound;
  }
 private:
  bool active_{false};
  std::uint64_t epoch_{0}, sequence_{0};
  double start_{0};
};
}
