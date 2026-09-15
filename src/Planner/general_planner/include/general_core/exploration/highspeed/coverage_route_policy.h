#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <general_core/exploration/highspeed/coverage_motion_policy.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

namespace fast_planner {
namespace coverage_route {

struct Config {
  bool enabled{false};
  int max_tasks{8}, alternatives{2}, anchors{1}, beam_width{32};
  int mandatory_after{6};
  double solve_ms{5.0}, edge_budget_ms{35.0};
  double intention_hold{6.0}, max_prefix_time{10.0};
  double switch_margin{0.25}, stall_timeout{60.0};
};

// Only OBSERVATION nodes can become commands. Anchors are ordered hypotheses
// about the suffix, including unknown space, and never certify an executable edge.
struct Node {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  int group{-1}, anchor{-1}, source{-1};
  bool stop{false};  // recovery observations end the executable prefix
  std::uint64_t identity{0};
  std::uint64_t region{0};
  std::vector<std::uint64_t> visible;
};
struct Edge {
  bool valid{false}, executable{false};
  double length{0.0}, bend{0.0}, speed_limit{std::numeric_limits<double>::infinity()};
  Eigen::Vector3d first{Eigen::Vector3d::Zero()}, last{Eigen::Vector3d::Zero()};
};
struct Problem {
  std::vector<Node> nodes;  // node zero is the future switching state
  std::vector<std::vector<Edge>> edges;
  int groups{0}, anchors{0}, committed_first{-1}, preferred_first{-1};
  std::uint32_t mandatory{0}, entry_groups{0};
  double speed{0.0}, max_speed{3.0}, acceleration{2.0}, yaw_rate{1.0};
  Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
};
struct Result {
  bool valid{false}, complete{false}, budget_exhausted{false};
  std::vector<int> route, prefix, deferred;
  double score{std::numeric_limits<double>::infinity()}, compute_ms{0.0};
};
struct ProgressMonitor {
  double progress_time{0}, selected_time{0}, fraction{0};
  double best_distance{std::numeric_limits<double>::infinity()};
  bool observe(double now,double distance,double evidence,double timeout) {
    if (!std::isfinite(now) || !std::isfinite(distance) || !std::isfinite(evidence)) return true;
    timeout=std::max(.1,timeout);
    if (progress_time==0 || now<selected_time || now-selected_time>timeout) {
      progress_time=now;best_distance=distance;fraction=evidence;
    } else if (distance+.25<best_distance || evidence>fraction+.05) {
      progress_time=now;best_distance=std::min(best_distance,distance);fraction=std::max(fraction,evidence);
    }
    selected_time=now;
    return now-progress_time>timeout;
  }
};
inline double angle(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return a.norm() < 1e-6 || b.norm() < 1e-6 ? 0.0 :
      std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0));
}
inline double yawDistance(double a, double b) {
  return std::abs(std::remainder(a - b, 2.0 * M_PI));
}

// Jointly order admitted observation groups and the next CP entry.
// The complete route and its executable prefix are different objects: a time
// limit or a recovery stop defers execution, never deletes the remaining work.
inline Result solve(const Problem &p, const Config &cfg) {
  Result out, preferred;
  const auto started = std::chrono::steady_clock::now();
  auto elapsed = [&]() { return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count(); };
  if (p.nodes.size() < 2 || p.groups < 1 || p.groups > 16 || p.anchors < 0 ||
      p.edges.size() != p.nodes.size()) return out;
  for (const auto &row : p.edges) if (row.size() != p.nodes.size()) return out;
  const std::uint32_t all = (1U << p.groups) - 1U;
  struct Label {
    std::vector<int> route, prefix;
    std::uint32_t visited{0}, executed{0};
    int last{0}, anchor{0};
    bool closed{false};
    double score{0.0}, prefix_time{0.0};
  };
  auto travel = [&](int a, int b) {
    const auto &edge = p.edges[a][b];
    if (!edge.valid || !std::isfinite(edge.length) || edge.length < 0.0 ||
        std::isnan(edge.speed_limit) || edge.speed_limit <= 0.0)
      return std::numeric_limits<double>::infinity();
    // One lightweight travel-time surrogate. Ordinary observations are
    // transit tasks, so do not charge a full stop at every frontier/CP node.
    // Actual passage speed and braking remain backend feasibility decisions.
    const double vmax=std::min(std::max(.2,p.max_speed),edge.speed_limit);
    const double acc=std::max(.2,p.acceleration);
    const double speed=a==0 ? std::max(0.0,p.speed) : (p.nodes[a].stop ? 0.0 : vmax);
    const double heading=a==0 ? angle(p.direction,edge.first) : 0.0;
    const double projected=std::max(0.0,speed*std::cos(heading));
    const double entry=std::min(vmax,projected);
    const double accel_distance=(vmax*vmax-entry*entry)/(2*acc);
    double time=edge.length<=accel_distance ?
        (std::sqrt(entry*entry+2*acc*edge.length)-entry)/acc :
        (vmax-entry)/acc+(edge.length-accel_distance)/vmax;
    time+=speed*std::sin(std::min(M_PI/2,heading))/acc+std::max(0.0,projected-vmax)/acc;
    if (p.nodes[b].stop) time=coverage_motion::executionTime(edge.length,speed,heading,vmax,acc,acc);
    return p.nodes[b].anchor >= 0 ? time : std::max(time,
        yawDistance(p.nodes[a].yaw, p.nodes[b].yaw) / std::max(.1, p.yaw_rate));
  };
  auto extend = [&](const Label &label, int i, Label &child) {
    const auto &node = p.nodes[i];
    const bool anchor=node.anchor>=0;
    if (anchor) {
      if (node.anchor!=label.anchor || label.prefix.empty() ||
          (label.visited&p.mandatory)!=p.mandatory) return false;
    } else if (node.group<0 || node.group>=p.groups ||
               (label.visited&(1U<<node.group))) return false;
    const auto &edge = p.edges[label.last][i];
    if (label.route.empty() && !anchor && p.entry_groups &&
        !(p.entry_groups&(1U<<node.group))) return false;
    if (label.route.empty() && (!edge.executable ||
        (p.committed_first>=0 && i!=p.committed_first))) return false;
    const double time = travel(label.last, i);
    if (!std::isfinite(time)) return false;
    child=label;child.route.push_back(i);child.last=i;child.score+=time;
    if (anchor) {
      ++child.anchor;child.closed=true;
    } else {
      child.visited|=1U<<node.group;
      if (!child.closed && edge.executable &&
          (child.prefix.empty() || child.prefix_time+time<=cfg.max_prefix_time)) {
        child.prefix.push_back(i);child.executed|=1U<<node.group;child.prefix_time+=time;
      } else child.closed=true;
      if (node.stop) child.closed=true;
    }
    return true;
  };
  auto accept = [&](const Label &label) {
    if (label.prefix.empty() || label.anchor!=p.anchors ||
        (label.visited&p.mandatory)!=p.mandatory) return;
    Result candidate;
    candidate.valid = true; candidate.complete = label.visited == all;
    candidate.score = label.score; candidate.route = label.route; candidate.prefix = label.prefix;
    for (int g = 0; g < p.groups; ++g)
      if (!(label.executed & (1U << g))) candidate.deferred.push_back(g);
    auto better = [&](const Result &old) {
      if (!old.valid) return true;
      // Missing edges/budget are explicit partial results. Never prefer a
      // cheap singleton over a feasible route covering all local groups.
      const int visits = static_cast<int>(candidate.route.size()) - p.anchors;
      const int old_visits = static_cast<int>(old.route.size()) - p.anchors;
      return visits != old_visits ? visits > old_visits : candidate.score < old.score;
    };
    if (better(out)) out = candidate;
    if (candidate.prefix.front() == p.preferred_first && better(preferred)) preferred = candidate;
  };
  // Deterministic feasible incumbents exist before the timed improvement
  // search. A tiny budget therefore cannot force a hover or lose the exit.
  for (int first = 1; first < static_cast<int>(p.nodes.size()); ++first) {
    Label label;
    if (!extend(Label{}, first, label)) continue;
    accept(label);
    for (int depth = 1; depth < p.groups+p.anchors; ++depth) {
      Label best; bool found = false;
      for (int i = 1; i < static_cast<int>(p.nodes.size()); ++i) {
        Label child;
        if (extend(label, i, child) && (!found || child.score < best.score)) {
          best = std::move(child); found = true;
        }
      }
      if (!found) break;
      label = std::move(best); accept(label);
    }
  }
  std::vector<Label> beam(1);
  bool exhausted = false;
  for (int depth = 0; depth < p.groups+p.anchors && !beam.empty(); ++depth) {
    std::vector<Label> next;
    for (const auto &label : beam) {
      if (elapsed() >= std::max(.01, cfg.solve_ms)) { exhausted = true; break; }
      for (int i = 1; i < static_cast<int>(p.nodes.size()); ++i) {
        Label child;
        if (!extend(label, i, child)) continue;
        accept(child); next.push_back(std::move(child));
      }
    }
    if (exhausted) break;
    std::stable_sort(next.begin(), next.end(), [](const Label &a, const Label &b) {
      return a.score < b.score;
    });
    if (next.size() > static_cast<std::size_t>(std::max(1, cfg.beam_width)))
      next.resize(std::max(1, cfg.beam_width));
    beam = std::move(next);
  }
  // Route-level hysteresis: tolerate small cost noise, but never lock an
  // inferior first goal for several seconds independently of its suffix.
  if (preferred.valid && preferred.route.size() == out.route.size() &&
      preferred.score <= out.score + std::max(0.0, cfg.switch_margin)) out = preferred;
  out.budget_exhausted = exhausted; out.compute_ms = elapsed();
  return out;
}

}  // namespace coverage_route
}  // namespace fast_planner
