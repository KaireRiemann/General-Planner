#pragma once
#include <algorithm>
#include <cstdint>
#include <Eigen/Core>
#include <vector>
#include <limits>

namespace fast_planner {
// Distance along a fixed certified route, including lateral tracking error.
// Keeping the reference fixed across replans prevents goal/path reselection
// from manufacturing progress. Around a corner Euclidean goal distance may grow.
inline double epiconRemainingRoute(const Eigen::Vector3f &position,
                                  const std::vector<Eigen::Vector3f> &path) {
  if(path.size()<2) return std::numeric_limits<double>::infinity();
  double remaining=0, best_offset=std::numeric_limits<double>::infinity(), result=remaining;
  for(size_t n=path.size()-1;n>0;--n) {
    const Eigen::Vector3f delta=path[n]-path[n-1];
    const double length=delta.norm();
    if(length<1e-6) continue;
    const double t=std::clamp(double((position-path[n-1]).dot(delta))/(length*length),0.0,1.0);
    const double offset=(position-(path[n-1]+t*delta)).norm();
    if(offset<best_offset) { best_offset=offset; result=offset+remaining+(1-t)*length; }
    remaining+=length;
  }
  return result;
}
// Only movement toward a normal exploration goal is productive. Goal reselection,
// successful optimization and spatial recovery alone do not replenish the budget.
class EpiconProgressWatchdog {
public:
  void reset(double now) { last_progress_=now; }
  void observe(double now, bool advances_goal) {
    if (advances_goal || now<last_progress_) last_progress_=now;
  }
  double idleSeconds(double now) const { return std::max(0.0,now-last_progress_); }
private:
  double last_progress_{0.0};
};
// Per-goal execution window. Reselection never resets the mission watchdog.
class EpiconGoalWindow {
public:
  void reset() { valid_=false; }
  void select(const Eigen::Vector3f &goal, double now) {
    if(!valid_ || (goal-goal_).norm()>.3f) {
      valid_=true; goal_=goal; since_=now;
    }
  }
  void progress(double now) { if(valid_) since_=now; }
  bool expired(double now, double interval) const {
    return valid_ && now-since_>=interval;
  }
private:
  bool valid_{false}; double since_{0};
  Eigen::Vector3f goal_{Eigen::Vector3f::Zero()};
};
// A native NO_FRONTIER is provisional. Mission completion needs independent
// sensor observations acquired after the last executable trajectory settled.
class EpiconFinishVerifier {
public:
  void reset() { settled_since_ = first_empty_ = -1.0; revision_ = 0; count_ = 0; }
  bool observe(double now, bool settled, bool fresh, bool audited_empty,
               std::uint64_t revision, double acquisition_time,
               int min_count, double min_duration) {
    if (!settled || !fresh) { reset(); return false; }
    if (settled_since_ < 0.0) settled_since_ = now;
    if (!audited_empty) {
      first_empty_ = -1.0; count_ = 0; revision_ = revision;
      return false;
    }
    if (acquisition_time <= settled_since_ || revision <= revision_) return false;
    revision_ = revision;
    if (count_++ == 0) first_empty_ = now;
    return count_ >= std::max(2, min_count) &&
           now - first_empty_ >= std::max(0.0, min_duration);
  }
  int count() const { return count_; }
private:
  double settled_since_{-1.0}, first_empty_{-1.0};
  std::uint64_t revision_{0};
  int count_{0};
};
}
