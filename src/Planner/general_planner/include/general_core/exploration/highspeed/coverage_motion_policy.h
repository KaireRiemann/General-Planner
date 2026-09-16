#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace fast_planner {

// Opt-in, single-robot coverage policy. No target/navigation solver settings
// are changed by enabling this policy.
struct CoverageMotionConfig {
  bool enabled{false};
  bool continuous_observation{true};
  double extension_min{2.0};
  double extension_max{6.0};
  double extension_max_turn{0.60};
  double path_match_radius{0.75};
  double path_switch_improvement{1.5};
  double path_switch_ratio{0.20};
  double shortcut_distance{8.0};
  double region_radius{6.0};
  double region_height{1.5};
  double region_duration{10.0};
  double region_switch_cost{2.0};
  double caution_timeout{20.0};
};

// A transit observation is still required to pass through the original
// observation ball. It is not permission to replace that task by a farther
// endpoint, or to publish a nonzero terminal state without a braking backup.
struct CoverageObservationGate {
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  double radius{0.35}, yaw{0.0}, yaw_tolerance{M_PI};
  std::uint64_t identity{0};
  int cluster{-1};
};
struct CoverageObservationContext {
  bool enabled{false};
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
  double radius{0.35};
  std::vector<CoverageObservationGate> gates;
  std::vector<CoverageObservationGate> orderedGates() const {
    if (!enabled) return {};
    if (!gates.empty()) return gates;
    CoverageObservationGate gate;gate.goal=goal;gate.radius=radius;
    return {gate};
  }
};

// Passed per invocation, so shared trajectory code cannot retain coverage
// behavior when the next task is target navigation or clearance recovery.
struct CoverageExecutionContext {
  bool enabled{false};
  bool spatial_repair{false};
  Eigen::Vector3d repair_position{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
};

enum class CoverageFailureKind { NONE, PATH, SPATIAL, DYNAMICS, HEAD, BUDGET };
struct CoveragePlanningFailure {
  CoverageFailureKind kind{CoverageFailureKind::NONE};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

namespace coverage_motion {
using Path = std::vector<Eigen::Vector3d>;
using SegmentFree = std::function<bool(const Eigen::Vector3d &, const Eigen::Vector3d &)>;

inline double length(const Path &path) {
  double result = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i)
    result += (path[i] - path[i - 1]).norm();
  return result;
}

inline double angle(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  if (a.norm() < 1e-6 || b.norm() < 1e-6) return 0.0;
  return std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0));
}

inline double pointSegmentDistance(const Eigen::Vector3d &p,
                                   const Eigen::Vector3d &a,
                                   const Eigen::Vector3d &b) {
  const Eigen::Vector3d d = b - a;
  const double u = d.squaredNorm() > 1e-10
      ? std::clamp((p - a).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.0;
  return (p - (a + u * d)).norm();
}

// Sampling density must not determine the number of MINCO pieces. Remove a
// vertex only when the replacement chord stays close to the original route
// AND has current free-space evidence. Keep genuine doorway corners.
inline Path compact(const Path &path, const SegmentFree &free,
                    double deviation = 0.20, double max_length = 6.0) {
  if (path.size() < 3) return path;
  Path out{path.front()};
  for (std::size_t i = 0; i + 1 < path.size();) {
    std::size_t next = i + 1;
    for (std::size_t j = i + 2; j < path.size(); ++j) {
      if ((path[j] - path[i]).norm() > max_length) break;
      bool close = true;
      for (std::size_t k = i + 1; k < j && close; ++k)
        close = pointSegmentDistance(path[k], path[i], path[j]) <= deviation;
      if (!close) break;
      if (free(path[i], path[j])) next = j;
    }
    if ((path[next] - out.back()).norm() > 1e-5) out.push_back(path[next]);
    i = next;
  }
  return out;
}

// Acceleration-limited estimate with the actual initial forward velocity.
// Sideways/backward momentum must first be removed. Keep a stopped endpoint
// as the conservative assumption until a continuation has been validated.
inline double executionTime(double distance, double speed, double heading,
                            double vmax, double acceleration, double braking) {
  if (!std::isfinite(distance) || distance < 0.0 || !std::isfinite(speed) ||
      !std::isfinite(heading) || !std::isfinite(vmax) || vmax <= 0.0)
    return std::numeric_limits<double>::infinity();
  const double a = std::max(0.1, acceleration), b = std::max(0.1, braking);
  const double v = std::max(0.0, speed);
  const double c = std::cos(std::clamp(heading, 0.0, std::acos(-1.0)));
  const double forward = std::max(0.0, v * c);
  const double v0 = std::min(vmax, forward);
  const double turn_time = v * std::sqrt(std::max(0.0, 1.0 - std::max(0.0, c) *
                                         std::max(0.0, c))) / b;
  const double overspeed_time = std::max(0.0, forward - vmax) / b;
  if (distance < v0 * v0 / (2.0 * b))
    return turn_time + overspeed_time + v0 / b +
           2.0 * std::sqrt((v0 * v0 / (2.0 * b) - distance) / a);
  const double peak = std::sqrt((2.0 * distance + v0 * v0 / a) / (1.0 / a + 1.0 / b));
  if (peak <= vmax)
    return turn_time + overspeed_time + (peak - v0) / a + peak / b;
  const double cruise = distance - (vmax * vmax - v0 * v0) / (2.0 * a) -
                        vmax * vmax / (2.0 * b);
  return turn_time + overspeed_time + (vmax - v0) / a + vmax / b + cruise / vmax;
}

inline bool sameRegion(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                       const CoverageMotionConfig &cfg) {
  return (a - b).head<2>().norm() <= cfg.region_radius &&
         std::abs(a.z() - b.z()) <= cfg.region_height;
}

inline bool pathFree(const Path &path, const SegmentFree &free) {
  if (path.size() < 2) return false;
  for (std::size_t i = 1; i < path.size(); ++i)
    if (!path[i].allFinite() || !path[i - 1].allFinite() ||
        !free(path[i - 1], path[i])) return false;
  return true;
}

inline Path prefix(const Path &path, double distance) {
  if (path.empty() || !std::isfinite(distance) || distance <= 0.0) return {};
  Path out{path.front()};
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double d = (path[i] - out.back()).norm();
    if (!std::isfinite(d)) return {};
    if (d < 1e-6) continue;
    if (d > distance) { out.push_back(out.back() + (path[i] - out.back()) * (distance / d)); break; }
    out.push_back(path[i]); distance -= d;
    if (distance <= 1e-6) break;
  }
  return out;
}

// Stop before the first unobserved/unsafe segment, with a reserve at the
// boundary. This does not certify dynamics; the normal MINCO/commit checks do.
inline Path observedPrefix(const Path &path, const SegmentFree &free,
                           double sample_step = 0.10, double reserve = 0.30) {
  if (path.size() < 2 || !free(path.front(), path.front())) return {};
  Path sampled{path.front()};
  double distance = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!path[i].allFinite()) return {};
    const Eigen::Vector3d from = path[i - 1], delta = path[i] - from;
    const int count = std::max(1, static_cast<int>(std::ceil(delta.norm() / std::max(0.02, sample_step))));
    for (int j = 1; j <= count; ++j) {
      const Eigen::Vector3d next = from + delta * (static_cast<double>(j) / count);
      if (!free(sampled.back(), next)) return prefix(path, distance - std::max(0.0, reserve));
      distance += (next - sampled.back()).norm(); sampled.push_back(next);
    }
  }
  return path;
}

// Reconnect only near the old path; never jump across a wall to a distant
// branch. Every retained segment is revalidated against current observations.
inline Path reconnect(const Path &path, const Eigen::Vector3d &position,
                      double match_radius, const SegmentFree &free) {
  double best = std::max(0.0, match_radius);
  std::size_t index = path.size();
  Eigen::Vector3d projected = position;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d d = path[i] - path[i - 1];
    const double u = d.squaredNorm() > 1e-10
        ? std::clamp((position - path[i - 1]).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.0;
    const Eigen::Vector3d p = path[i - 1] + u * d;
    if ((p - position).norm() < best) {
      best = (p - position).norm(); index = i; projected = p;
    }
  }
  if (index == path.size()) return {};
  Path result{position};
  // A perpendicular projection produces [robot, lateral foot, forward]
  // and an artificial 90-degree turn even for centimetres of tracking error.
  // Join the next retained vertex directly when the observed map allows it.
  if ((projected - position).norm() > 0.02 && !free(position, path[index]))
    result.push_back(projected);
  for (std::size_t i = index; i < path.size(); ++i)
    if ((path[i] - result.back()).norm() > 0.02) result.push_back(path[i]);
  return pathFree(result, free) ? result : Path{};
}

inline Path shortcut(const Path &path, double max_distance, const SegmentFree &free) {
  if (path.size() < 3) return path;
  Path result{path.front()};
  int budget = 64;
  for (std::size_t i = 0; i + 1 < path.size();) {
    std::size_t next = i + 1;
    // Bound both collision-query work and the spatial shortcut horizon.
    for (std::size_t j = std::min(path.size() - 1, i + 16); j > i + 1 && budget > 0; --j) {
      if ((path[j] - path[i]).norm() > max_distance) continue;
      --budget;
      if (free(path[i], path[j])) { next = j; break; }
    }
    result.push_back(path[next]); i = next;
  }
  return result;
}

inline bool preferRetained(const Path &retained, const Path &candidate,
                           const CoverageMotionConfig &cfg) {
  if (retained.size() < 2) return false;
  if (candidate.size() < 2) return true;
  const double old_length = length(retained), new_length = length(candidate);
  return old_length - new_length <= std::max(cfg.path_switch_improvement,
                                             cfg.path_switch_ratio * old_length);
}

inline bool appendContinuation(Path &path, const Eigen::Vector3d &next,
                               double horizon, const CoverageMotionConfig &cfg,
                               const SegmentFree &free) {
  if (path.size() < 2 || !next.allFinite()) return false;
  const Eigen::Vector3d goal = path.back(), outgoing = next - goal;
  const double available = std::min({cfg.extension_max, outgoing.norm(), horizon - length(path)});
  if (available < cfg.extension_min ||
      angle(goal - path[path.size() - 2], outgoing) > cfg.extension_max_turn) return false;
  const Eigen::Vector3d endpoint = goal + outgoing.normalized() * available;
  if (!free(goal, endpoint)) return false;
  // Do not hand a six-metre seed edge to a corridor configured for two.
  const int pieces = std::max(1, static_cast<int>(std::ceil(available)));
  for (int i = 1; i <= pieces; ++i)
    path.push_back(goal + (endpoint - goal) * (static_cast<double>(i) / pieces));
  return true;
}

// Follow the actual topology polyline through a doorway. Only its initial
// tangent controls passage speed; later corners stay in the path for MINCO.
inline bool appendPathContinuation(Path &path, const Path &continuation,
                                   double horizon, const CoverageMotionConfig &cfg,
                                   const SegmentFree &free) {
  if (path.size()<2 || continuation.size()<2 ||
      (path.back()-continuation.front()).norm()>.1) return false;
  const double available=std::min({cfg.extension_max, length(continuation), horizon-length(path)});
  if (available<cfg.extension_min) return false;
  Path extension=prefix(continuation,available);
  if (extension.empty()) return false;
  extension.front()=path.back();
  extension=observedPrefix(extension,free);
  if (length(extension)<cfg.extension_min || extension.size()<2 || angle(path.back()-path[path.size()-2],
      extension[1]-extension[0])>cfg.extension_max_turn || !pathFree(extension,free)) return false;
  Path merged=path;
  for (std::size_t i=1;i<extension.size();++i) {
    const Eigen::Vector3d start=merged.back();
    const int pieces=std::max(1,static_cast<int>(std::ceil((extension[i]-start).norm())));
    for (int j=1;j<=pieces;++j) merged.push_back(start+(extension[i]-start)*(static_cast<double>(j)/pieces));
  }
  path.swap(merged);return true;
}

// Measured computation cost determines when the next planning attempt starts.
// This is a scheduling budget, never permission to extend an expired command.
struct PlanningBudget {
  double local{.2}, global{.1};
  void observe(double seconds, bool global_work=false) {
    if (!std::isfinite(seconds) || seconds<0) return;
    double &estimate=global_work ? global : local;
    estimate=std::clamp(std::max(seconds, .9*estimate+.1*seconds), .02, 2.0);
  }
  double lead(double minimum, double control_latency) const {
    return std::max(minimum, local+global+std::max(.05,control_latency)+.1);
  }
  double retry(double configured, double remaining, double control_latency) const {
    return std::clamp(remaining-lead(0.0,control_latency), .02, std::max(.02,configured));
  }
};

struct LivenessMonitor {
  double since{0.0};int observed{-1};Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  bool stalled(double now,const Eigen::Vector3d &measured,int evidence,double timeout) {
    if (!std::isfinite(now) || !measured.allFinite() || evidence<0 || timeout<=0) return false;
    if (since==0 || now<since || (measured-position).norm()>=.5 || evidence>=observed+8) {
      since=now;position=measured;observed=evidence;
    }
    return now-since>=timeout;
  }
};

// Called with measured odometry, never with a planned trajectory state.
struct PassageMonitor {
  bool active{false}, passed{false}, have_previous{false};
  Eigen::Vector3d goal{Eigen::Vector3d::Zero()}, previous{Eigen::Vector3d::Zero()};
  double previous_time{0.0}, radius{0.35};
  void observe(const Eigen::Vector3d &position, double time, double max_speed) {
    if (!active || !position.allFinite() || !std::isfinite(time)) return;
    passed = passed || (position - goal).norm() <= radius;
    const double dt = time - previous_time;
    if (have_previous && dt > 0.0 && dt <= 0.25 &&
        (position - previous).norm() <= std::max(0.5, 2.0 * radius) &&
        (position - previous).norm() <= 1.5 * max_speed * dt + 0.1)
      passed = passed || pointSegmentDistance(goal, previous, position) <= radius;
    previous = position; previous_time = time; have_previous = true;
  }
};
}  // namespace coverage_motion
}  // namespace fast_planner
