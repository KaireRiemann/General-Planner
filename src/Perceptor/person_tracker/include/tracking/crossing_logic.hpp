#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace person_tracker::crossing_logic
{

inline double nearestHorizontalSeedDistance(
  const Eigen::Vector3d & point, const std::vector<Eigen::Vector3d> & seeds)
{
  double best_squared = std::numeric_limits<double>::infinity();
  for (const auto & seed : seeds) {
    const double dx = point.x() - seed.x();
    const double dy = point.y() - seed.y();
    best_squared = std::min(best_squared, dx * dx + dy * dy);
  }
  return std::sqrt(best_squared);
}

// Returns 0 for the selected target, 1 for the distractor and -1 for an
// intentionally rejected point close to the Voronoi bisector.
inline int assignSeedFamily(
  const Eigen::Vector3d & point,
  const std::vector<Eigen::Vector3d> & target_seeds,
  const std::vector<Eigen::Vector3d> & distractor_seeds,
  double ambiguity_margin)
{
  const double target_distance = nearestHorizontalSeedDistance(point, target_seeds);
  const double distractor_distance = nearestHorizontalSeedDistance(point, distractor_seeds);
  if (std::abs(target_distance - distractor_distance) < std::max(0.0, ambiguity_margin)) {
    return -1;
  }
  return target_distance <= distractor_distance ? 0 : 1;
}

inline bool shouldResolveIdentity(
  double best_average_score, double second_average_score, int frames,
  int confirmation_frames, int maximum_frames, double required_margin)
{
  const bool enough_frames = frames >= std::max(2, confirmation_frames);
  const bool decisive = second_average_score - best_average_score >=
    std::max(0.0, required_margin);
  return (enough_frames && decisive) || frames >= std::max(confirmation_frames, maximum_frames);
}

}  // namespace person_tracker::crossing_logic
