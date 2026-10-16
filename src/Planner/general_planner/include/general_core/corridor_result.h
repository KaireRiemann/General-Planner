#pragma once
#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>
#include <utility>

namespace general_planner {
// Both generators report geometry in the same contract. This does not imply
// that obstacle points and occupancy voxels certify the same observed space.
struct CorridorResult {
  std::vector<Eigen::Vector3d> path;
  std::vector<Eigen::MatrixX4d> planes;
  bool truncated{false};
  double min_overlap{1e-2};
  std::string failure;
  std::string backend;
  std::string map_representation;
  double clearance{0.0};
  std::uint64_t map_revision{0}; // Zero means the provider has no revision API.
};

// Prefer a full soft corridor. A soft prefix must not suppress a full hard-safe
// route. Failed retries must not overwrite a usable soft prefix.
template<class Build>
bool selectCorridor(double soft, double hard, bool allow_retry, Build build,
                    CorridorResult &result, bool &used_hard) {
  used_hard=false;
  const bool soft_ok=build(soft,result);
  if (soft_ok && !result.truncated) return true;
  if (allow_retry && hard<soft) {
    CorridorResult candidate;
    const bool hard_ok=build(hard,candidate);
    // For two prefixes keep the soft one; endpoint distance alone cannot tell
    // which prefix progresses farther along a turning guide.
    if (hard_ok && (!soft_ok || !candidate.truncated)) {
      result=std::move(candidate); used_hard=true; return true;
    }
    if (!soft_ok) result=std::move(candidate);
  }
  return soft_ok;
}
}
