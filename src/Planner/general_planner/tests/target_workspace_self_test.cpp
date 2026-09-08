#include <general_core/exploration/highspeed/target_workspace_validation.h>
#include <cassert>
#include <cstring>
#include <limits>
#include <iostream>

int main() {
  using Eigen::Vector3f;
  using fast_planner::targetWorkspaceFailure;
  const Vector3f start(21.5f, 0.f, 1.5f), goal(-42.f, 0.f, 0.f);
  auto legacy = [](const Vector3f &p) {
    return p.x() >= -20 && p.x() <= 20 && p.y() >= -20 &&
           p.y() <= 20.2f && p.z() >= -.2f && p.z() <= 3.8f;
  };
  auto capacity = [start](const Vector3f &p) {
    return std::abs(p.x() - start.x()) <= 120 && std::abs(p.y()) <= 120 &&
           p.z() >= -.5f && p.z() <= 6.5f;
  };
  assert(std::strcmp(targetWorkspaceFailure(start, goal, false, legacy),
                     "START_OUTSIDE_CAPACITY_OR_IN_EXCLUSION") == 0);
  assert(!targetWorkspaceFailure(start, goal, false, capacity));
  assert(!targetWorkspaceFailure(start, Vector3f(-42, 0, -10), false, capacity));
  assert(targetWorkspaceFailure(start, Vector3f(-42, 0, -10), true, capacity));
  assert(targetWorkspaceFailure(start, Vector3f(200, 0, 1.5), true, capacity));
  auto exclusion = [capacity](const Vector3f &p) {
    return capacity(p) && !(p.x() < -40 && p.x() > -45);
  };
  assert(targetWorkspaceFailure(start, goal, false, exclusion));
  assert(targetWorkspaceFailure(start, Vector3f(std::numeric_limits<float>::quiet_NaN(), 0, 0), false, capacity));
  assert(!targetWorkspaceFailure(Vector3f(0,0,1), Vector3f(10,0,1), true, legacy));
  std::cout << "target workspace regression passed\n";
}
