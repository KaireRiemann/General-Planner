#pragma once
#include <algorithm>
#include <cmath>

namespace fast_planner {
// Native EPICON checks the remaining trajectory at 0.3 s intervals and uses
// a 1 m/s allowance. Candidate and executing trajectories share this rule.
template <typename Trajectory>
bool epiconVelocityAccepts(const Trajectory &trajectory, double start,
                           double duration, double max_velocity) {
  for(double t=std::max(0.0,start);t<duration;t+=.3) {
    const double speed=trajectory.getVel(t).norm();
    if(!std::isfinite(speed) || speed>max_velocity+1.0) return false;
  }
  return true;
}
}
