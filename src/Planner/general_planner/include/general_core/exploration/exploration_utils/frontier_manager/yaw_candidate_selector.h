#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace viewpoint_yaw_selector {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kYawTieEpsilon = 1.0e-6f;

inline float normalizeYaw(const float yaw) {
  if (!std::isfinite(yaw)) {
    return 0.0f;
  }
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

// Keep the legacy eight sector centers, but also sample the current heading.
// The latter is essential for a level 360-degree lidar: all sector centers
// have identical vertical visibility, so keeping the current heading is the
// physically meaningful zero-turn solution.
inline std::vector<float> buildCandidates(const float current_yaw) {
  std::vector<float> candidates;
  candidates.reserve(9);
  for (int sector = -4; sector < 4; ++sector) {
    candidates.push_back(normalizeYaw(
        (45.0f * static_cast<float>(sector) + 22.5f) * kPi / 180.0f));
  }

  const float reference_yaw = normalizeYaw(current_yaw);
  const bool already_sampled =
      std::any_of(candidates.begin(), candidates.end(),
                  [&](const float candidate_yaw) {
                    return std::fabs(
                               normalizeYaw(candidate_yaw - reference_yaw)) <
                           1.0e-4f;
                  });
  if (!already_sampled) {
    candidates.push_back(reference_yaw);
  }
  return candidates;
}

struct Selection {
  int index = -1;
  int visible_gain = 0;
  float yaw_delta = std::numeric_limits<float>::infinity();

  bool valid() const { return index >= 0; }
};

// Rank executable yaw candidates by:
//   1. yaw hard-gate feasibility (when a finite limit is supplied),
//   2. maximum visible gain,
//   3. minimum wrapped rotation from the current heading,
//   4. stable input order.
inline Selection select(const std::vector<int> &visible_gains,
                        const std::vector<float> &candidate_yaws,
                        const float current_yaw,
                        const float max_abs_yaw_delta =
                            std::numeric_limits<float>::infinity()) {
  Selection best;
  if (visible_gains.size() != candidate_yaws.size()) {
    return best;
  }

  const float reference_yaw = normalizeYaw(current_yaw);
  const float yaw_limit =
      std::isfinite(max_abs_yaw_delta)
          ? std::max(0.0f, max_abs_yaw_delta)
          : std::numeric_limits<float>::infinity();

  for (int i = 0; i < static_cast<int>(candidate_yaws.size()); ++i) {
    if (visible_gains[i] <= 0 || !std::isfinite(candidate_yaws[i])) {
      continue;
    }
    const float yaw_delta =
        std::fabs(normalizeYaw(candidate_yaws[i] - reference_yaw));
    if (yaw_delta > yaw_limit + kYawTieEpsilon) {
      continue;
    }
    if (!best.valid() || visible_gains[i] > best.visible_gain ||
        (visible_gains[i] == best.visible_gain &&
         yaw_delta + kYawTieEpsilon < best.yaw_delta)) {
      best.index = i;
      best.visible_gain = visible_gains[i];
      best.yaw_delta = yaw_delta;
    }
  }
  return best;
}

}  // namespace viewpoint_yaw_selector
