#pragma once

#include <Eigen/Core>

namespace fast_planner {
// This is an exclusion preflight, NOT a capacity or occupancy test: a remote
// destination may be unknown. Keep obstacle checks in the local planner.
template <typename Allowed>
const char *targetWorkspaceFailure(const Eigen::Vector3f &start,
                                   Eigen::Vector3f goal,
                                   bool use_message_z, Allowed allowed) {
  if (!start.allFinite()) return "START_NONFINITE";
  if (!goal.allFinite()) return "TARGET_NONFINITE";
  if (!use_message_z) goal.z() = start.z();
  if (!allowed(start)) return "START_IN_EXCLUSION";
  if (!allowed(goal)) return "TARGET_IN_EXCLUSION";
  return nullptr;
}
}  // namespace fast_planner
