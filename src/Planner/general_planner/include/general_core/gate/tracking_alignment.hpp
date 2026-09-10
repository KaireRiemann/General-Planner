#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace general_planner::gate {
// Match only a bounded interval preceding feedback receipt. The interval moves
// with time: a stopped vehicle cannot stay matched to an arbitrarily old point.
// This estimates progress; it does not recover the sensor acquisition timestamp.
template<class PositionAt>
double matchFeedbackTime(const Eigen::Vector3d &measured, double elapsed,
                         double receipt_age, double max_delay, double duration,
                         PositionAt position_at) {
  const double receipt_t=elapsed-std::max(0.0,receipt_age);
  const double lo=std::clamp(receipt_t-max_delay,0.0,duration);
  const double hi=std::clamp(receipt_t,0.0,duration);
  const int count=std::max(1,static_cast<int>(std::ceil((hi-lo)/.002)));
  double best=hi, distance=(position_at(hi)-measured).squaredNorm();
  for(int i=0;i<=count;++i) {
    const double t=lo+(hi-lo)*i/count;
    const double candidate=(position_at(t)-measured).squaredNorm();
    if(candidate<distance) {distance=candidate;best=t;}
  }
  return best;
}
}  // namespace general_planner::gate
