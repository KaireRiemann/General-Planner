#include <general_core/exploration/exploration_utils/coverage_guidance/coverage_recovery_identity.h>

#include <iostream>

namespace {

bool require(const bool condition, const char *message) {
  if (!condition) {
    std::cerr << "coverage_recovery_identity_self_test: " << message
              << std::endl;
  }
  return condition;
}

}  // namespace

int main() {
  using fast_planner::CoverageRecoveryIdentity;
  using fast_planner::CoverageTarget;

  constexpr std::uint64_t first_id = 7742730272344778555ULL;
  constexpr std::uint64_t second_id = 8203284889734444770ULL;
  const Eigen::Vector3d canonical_approach(7.02727, -8.40909, 4.79091);

  CoverageRecoveryIdentity exhausted;
  fast_planner::rememberCoverageStableId(exhausted, first_id);
  exhausted.approach = canonical_approach;
  exhausted.has_approach = true;

  // Reproduce the log failure: before approach selection the regenerated
  // component does not match the terminal action recorded by the planner.
  CoverageTarget regenerated;
  regenerated.stable_id = second_id;
  regenerated.has_approach = true;
  regenerated.approach_position = Eigen::Vector3d(-2.0, 3.0, 4.8);
  if (!require(!fast_planner::coverageRecoveryIdentityMatches(
                   exhausted, regenerated, 1.8),
               "raw regenerated target unexpectedly matched")) {
    return 1;
  }

  // Once the safe approach is canonicalized, both ids represent one
  // executable action and must share its exhausted state.
  regenerated.approach_position = canonical_approach;
  if (!require(fast_planner::coverageRecoveryIdentityMatches(
                   exhausted, regenerated, 1.8),
               "canonical approach did not match exhausted action")) {
    return 1;
  }
  fast_planner::rememberCoverageStableId(exhausted, regenerated.stable_id);
  if (!require(exhausted.primary_stable_id == first_id,
               "primary stable id was overwritten by an alias") ||
      !require(fast_planner::coverageStableIdMatches(exhausted, second_id),
               "regenerated id was not retained as an alias")) {
    return 1;
  }

  for (std::uint64_t id = 100; id < 132; ++id) {
    fast_planner::rememberCoverageStableId(exhausted, id);
  }
  if (!require(exhausted.stable_id_alias_count ==
                   fast_planner::kCoverageRecoveryAliasCapacity,
               "stable-id alias cache is not bounded")) {
    return 1;
  }

  // FINISH later evaluates a fresh raw target. Alias retention must match it
  // even before another approach candidate is selected.
  CoverageTarget next_plan_raw = regenerated;
  next_plan_raw.approach_position = Eigen::Vector3d(1.0, 1.0, 4.8);
  if (!require(fast_planner::coverageRecoveryIdentityMatches(
                   exhausted, next_plan_raw, 1.8),
               "alias did not stabilize identity across plan revisions")) {
    return 1;
  }

  CoverageTarget duplicate = regenerated;
  duplicate.stable_id = 42;
  if (!require(fast_planner::sameCoverageExecutionTarget(
                   regenerated, duplicate, 1.8),
               "same canonical approach was not deduplicated")) {
    return 1;
  }

  CoverageTarget distinct = regenerated;
  distinct.stable_id = 43;
  distinct.approach_position += Eigen::Vector3d(3.0, 0.0, 0.0);
  if (!require(!fast_planner::sameCoverageExecutionTarget(
                   regenerated, distinct, 1.8),
               "distinct execution approaches were merged")) {
    return 1;
  }

  std::cout << "coverage_recovery_identity_self_test: PASS" << std::endl;
  return 0;
}
