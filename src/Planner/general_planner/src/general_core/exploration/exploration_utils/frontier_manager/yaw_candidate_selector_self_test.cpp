#include <general_core/exploration/exploration_utils/frontier_manager/yaw_candidate_selector.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

namespace {

constexpr float deg2rad(const float degree) {
  return degree * viewpoint_yaw_selector::kPi / 180.0f;
}

void expectNear(const float actual, const float expected,
                const float tolerance = 1.0e-5f) {
  if (std::fabs(actual - expected) > tolerance) {
    std::cerr << "expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

void require(const bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main() {
  using viewpoint_yaw_selector::buildCandidates;
  using viewpoint_yaw_selector::normalizeYaw;
  using viewpoint_yaw_selector::select;

  const std::vector<float> legacy_yaws = {
      deg2rad(-157.5f), deg2rad(-112.5f), deg2rad(-67.5f), deg2rad(-22.5f),
      deg2rad(22.5f),   deg2rad(67.5f),   deg2rad(112.5f), deg2rad(157.5f)};

  // Regression: the old max_element implementation selected -157.5 degrees
  // when a level 360-degree lidar made every yaw equally informative.
  const auto level_lidar_yaws = buildCandidates(0.0f);
  require(level_lidar_yaws.size() == 9,
          "current yaw was not added to legacy candidates");
  const std::vector<int> level_lidar_gains(level_lidar_yaws.size(), 21);
  const auto level_lidar = select(level_lidar_gains, level_lidar_yaws, 0.0f,
                                  deg2rad(91.7f));
  require(level_lidar.valid(), "level-lidar selection is invalid");
  expectNear(level_lidar_yaws[level_lidar.index], 0.0f);
  expectNear(level_lidar.yaw_delta, 0.0f);

  // The current heading is de-duplicated when it is already a sector center.
  require(buildCandidates(deg2rad(22.5f)).size() == 8,
          "duplicate current yaw was added");

  // A unique information-gain maximum retains priority when no hard yaw
  // constraint is active, preserving the Marsim behavior.
  std::vector<int> unique_gains(legacy_yaws.size(), 4);
  unique_gains[6] = 9;
  const auto unique = select(unique_gains, legacy_yaws, 0.0f);
  require(unique.valid(), "unique-gain selection is invalid");
  require(unique.index == 6, "unique maximum gain lost priority");

  // Under the hard gate, a lower-gain feasible yaw is preferable to rejecting
  // the whole viewpoint because its unconstrained maximum turns too far.
  std::vector<int> constrained_gains(legacy_yaws.size(), 0);
  constrained_gains[0] = 12;
  constrained_gains[4] = 10;
  const auto constrained =
      select(constrained_gains, legacy_yaws, 0.0f, deg2rad(90.0f));
  require(constrained.valid(), "hard-gate feasible selection is invalid");
  require(constrained.index == 4,
          "yaw selector did not choose the feasible positive-gain yaw");

  // Wrapped distance around +/-pi is small rather than almost 2*pi.
  const std::vector<float> wrap_yaws = {deg2rad(-179.0f), deg2rad(135.0f)};
  const auto wrapped = select({7, 7}, wrap_yaws, deg2rad(179.0f));
  require(wrapped.valid(), "wrapped-yaw selection is invalid");
  require(wrapped.index == 0, "wrapped-yaw nearest candidate is wrong");
  expectNear(wrapped.yaw_delta, deg2rad(2.0f), 1.0e-4f);
  expectNear(std::fabs(normalizeYaw(deg2rad(-358.0f))), deg2rad(2.0f),
             1.0e-4f);

  // Zero-gain, malformed and non-finite candidates are not executable.
  require(!select({0, 0}, {0.0f, 1.0f}, 0.0f).valid(),
          "zero-gain yaw became executable");
  require(!select({1}, {0.0f, 1.0f}, 0.0f).valid(),
          "malformed candidate input became executable");
  require(!select({1}, {std::numeric_limits<float>::infinity()}, 0.0f)
               .valid(),
          "non-finite yaw became executable");

  std::cout << "yaw_candidate_selector_self_test: PASS\n";
  return 0;
}
