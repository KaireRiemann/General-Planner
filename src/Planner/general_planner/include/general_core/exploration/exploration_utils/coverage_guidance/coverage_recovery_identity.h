#pragma once

#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_types.h>

#include <algorithm>
#include <array>
#include <cstdint>

namespace fast_planner {

// Persistent recovery state belongs to an executable observation action, not
// only to the raw coverage component.  Coverage targets may be regenerated
// with different stable ids and may choose a different entry from their
// approach_candidates list on every planning pass.  Keep both identities so a
// terminal result cannot be bypassed by id churn before FINISH.
constexpr std::size_t kCoverageRecoveryAliasCapacity = 8U;

struct CoverageRecoveryIdentity {
  std::uint64_t primary_stable_id{0};
  // IDs are only an acceleration hint; canonical approach matching remains
  // authoritative. Keep a small fixed alias cache so pathological frontend id
  // churn cannot turn the terminal registry into an unbounded allocation.
  std::array<std::uint64_t, kCoverageRecoveryAliasCapacity> stable_id_aliases{};
  std::size_t stable_id_alias_count{0U};
  Eigen::Vector3d approach{Eigen::Vector3d::Zero()};
  bool has_approach{false};
};

inline bool coverageStableIdMatches(const CoverageRecoveryIdentity &identity,
                                    const std::uint64_t stable_id) {
  if (stable_id == 0) {
    return false;
  }
  return identity.primary_stable_id == stable_id ||
         std::find(identity.stable_id_aliases.begin(),
                   identity.stable_id_aliases.begin() +
                       identity.stable_id_alias_count,
                   stable_id) != identity.stable_id_aliases.begin() +
                                    identity.stable_id_alias_count;
}

inline void rememberCoverageStableId(CoverageRecoveryIdentity &identity,
                                     const std::uint64_t stable_id) {
  if (stable_id == 0) {
    return;
  }
  if (identity.primary_stable_id == 0) {
    identity.primary_stable_id = stable_id;
    return;
  }
  if (identity.primary_stable_id != stable_id) {
    const auto alias_end = identity.stable_id_aliases.begin() +
                           identity.stable_id_alias_count;
    if (std::find(identity.stable_id_aliases.begin(), alias_end, stable_id) ==
            alias_end &&
        identity.stable_id_alias_count <
            identity.stable_id_aliases.size()) {
      identity.stable_id_aliases[identity.stable_id_alias_count++] = stable_id;
    }
  }
}

inline bool coverageApproachMatches(const CoverageRecoveryIdentity &identity,
                                    const CoverageTarget &target,
                                    const double match_radius) {
  return identity.has_approach && target.has_approach &&
         identity.approach.allFinite() &&
         target.approach_position.allFinite() &&
         (identity.approach - target.approach_position).norm() <=
             std::max(0.0, match_radius);
}

inline bool coverageRecoveryIdentityMatches(
    const CoverageRecoveryIdentity &identity, const CoverageTarget &target,
    const double match_radius) {
  return coverageStableIdMatches(identity, target.stable_id) ||
         coverageApproachMatches(identity, target, match_radius);
}

inline bool sameCoverageExecutionTarget(const CoverageTarget &first,
                                        const CoverageTarget &second,
                                        const double match_radius) {
  const bool same_stable_id =
      first.stable_id != 0 && first.stable_id == second.stable_id;
  const bool same_approach =
      first.has_approach && second.has_approach &&
      first.approach_position.allFinite() &&
      second.approach_position.allFinite() &&
      (first.approach_position - second.approach_position).norm() <=
          std::max(0.0, match_radius);
  return same_stable_id || same_approach;
}

}  // namespace fast_planner
