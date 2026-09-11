#include <general_core/exploration/highspeed/target_route_runtime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace fast_planner {
namespace {
using V = Eigen::Vector3d;
using Id = TargetRouteRuntime::Graph::NodeId;

double clampFinite(double value, double fallback, double lo, double hi) {
  return std::isfinite(value) ? std::clamp(value, lo, hi) : fallback;
}

// Endpoints are always checked, including zero-length/sub-step connectors.
bool lineFree(const V &a, const V &b, double step,
              const std::function<bool(const V &)> &free) {
  if (!a.allFinite() || !b.allFinite() || !free) return false;
  const double length = (b - a).norm();
  if (!std::isfinite(length) || length / step > 20000.0) return false;
  const int n = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int i = 0; i <= n; ++i)
    if (!free(a + (b - a) * (static_cast<double>(i) / n))) return false;
  return true;
}

V pointAt(const TargetRoute &route, double arc) {
  const auto it = std::upper_bound(route.arc.begin(), route.arc.end(), arc);
  if (it == route.arc.begin()) return route.points.front();
  if (it == route.arc.end()) return route.points.back();
  const std::size_t i = std::distance(route.arc.begin(), it);
  const double len = route.arc[i] - route.arc[i - 1];
  return route.points[i - 1] + (route.points[i] - route.points[i - 1]) *
      ((arc - route.arc[i - 1]) / std::max(1e-9, len));
}
}  // namespace

bool targetRouteTrajectoryKnownFree(double duration, double dt, double step,
    const std::function<V(double)> &position,
    const std::function<bool(const V &)> &local_free) {
  if (!std::isfinite(duration) || duration <= 0.0 || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(step) || step <= 0.0 || !position || !local_free || duration / dt > 100000.0)
    return false;
  const int samples = std::max(1, static_cast<int>(std::ceil(duration / dt)));
  V previous = position(0.0);
  if (!previous.allFinite() || !local_free(previous)) return false;
  int checks = 0;
  const auto bounded_free = [&](const V &p) { return ++checks <= 100000 && local_free(p); };
  for (int i = 1; i <= samples; ++i) {
    const V point = position(duration * static_cast<double>(i) / samples);
    if (!lineFree(previous, point, step, bounded_free)) return false;
    previous = point;
  }
  return true;
}

void TargetRouteRuntime::configure(TargetRouteConfig c) {
  if (c.mode != "legacy" && c.mode != "shadow" && c.mode != "prefer_known")
    c.mode = "legacy";  // invalid configuration cannot enable new behavior
  c.query_interval = clampFinite(c.query_interval, 1.0, 0.2, 10.0);
  c.query_budget_ms = clampFinite(c.query_budget_ms, 12.0, 1.0, 50.0);
  c.max_nodes = std::clamp(c.max_nodes, 32, 100000);
  c.max_expansions = std::clamp(c.max_expansions, 16, 20000);
  c.max_map_checks = std::clamp(c.max_map_checks, 64, 100000);
  c.sample_step = clampFinite(c.sample_step, 0.10, 0.02, 0.20);
  c.prefix_length = clampFinite(c.prefix_length, 12.0, 1.0, 30.0);
  c.min_prefix_length = clampFinite(c.min_prefix_length, 0.75, 0.2, 2.0);
  c.stop_margin = clampFinite(c.stop_margin, 0.4, 0.1, 2.0);
  c.projection_radius = clampFinite(c.projection_radius, 1.5, 0.2, 3.0);
  c.progress_timeout = clampFinite(c.progress_timeout, 12.0, 2.0, 60.0);
  c.cooldown = clampFinite(c.cooldown, 5.0, 1.0, 60.0);
  c.anchor_switch_margin = clampFinite(c.anchor_switch_margin, 1.0, 0.0, 10.0);
  config_ = c;
  reset();
}

void TargetRouteRuntime::reset() {
  ++task_;
  route_ = TargetRoute{};
  cooldowns_.clear();
  goal_.setConstant(std::numeric_limits<double>::quiet_NaN());
  last_query_ = -std::numeric_limits<double>::infinity();
  local_goal_retry_after_ = 0.0;
  have_odom_ = false;
  status_ = "RESET";
}

bool TargetRouteRuntime::cooling(Id a, Id b, double now) const {
  if (a > b) std::swap(a, b);
  for (const auto &entry : cooldowns_)
    if (entry.a == a && entry.b == b && entry.until > now) return true;
  return false;
}

void TargetRouteRuntime::cool(Id a, Id b, double now) {
  if (!a && !b) return;
  if (a > b) std::swap(a, b);
  for (auto &entry : cooldowns_)
    if (entry.a == a && entry.b == b) {
      entry.until = now + config_.cooldown;
      return;
    }
  if (cooldowns_.size() >= 64) cooldowns_.erase(cooldowns_.begin());
  cooldowns_.push_back({a, b, now + config_.cooldown});
}

void TargetRouteRuntime::fail(double now, const std::string &reason) {
  if (route_.valid()) {
    if (route_.source == TargetRouteSource::LOCAL_GOAL)
      local_goal_retry_after_ = now + config_.cooldown;
    auto it = std::upper_bound(route_.arc.begin(), route_.arc.end(), route_.progress + 0.2);
    const std::size_t i = std::min<std::size_t>(
        std::max<std::size_t>(1, std::distance(route_.arc.begin(), it)), route_.nodes.size() - 1);
    if (route_.nodes[i - 1] && route_.nodes[i])
      cool(route_.nodes[i - 1], route_.nodes[i], now);
    // Anchor failures must survive a fresh route id/query. Connector failures
    // are associated with the attached node, never frontier cluster ids.
    else cool(0, route_.nodes[i] ? route_.nodes[i] : route_.nodes[i - 1], now);
    if (route_.source == TargetRouteSource::KNOWN_ANCHOR)
      cool(0, route_.nodes.back(), now);
  }
  route_ = TargetRoute{};
  last_query_ = now;  // leave at least one query interval to local exploration
  status_ = reason;
}

TargetRoute TargetRouteRuntime::query(const Graph::SearchSnapshotPtr &snapshot,
                                      const V &start, const V &goal, double now,
                                      const TargetRouteMapView &map) {
  TargetRoute result;
  status_ = "NO_TOPOLOGY";
  if (!map.local_free) return result;
  const auto began = std::chrono::steady_clock::now();
  int checks = 0, expansions = 0, scans = 0;
  struct Audit {
    double &elapsed; int &saved_checks, &saved_expansions;
    const int &checks, &expansions;
    std::chrono::steady_clock::time_point began;
    ~Audit() {
      elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
      saved_checks = checks; saved_expansions = expansions;
    }
  } audit{last_query_ms_, last_map_checks_, last_expansions_, checks, expansions, began};
  query_stage_ = "local_goal";
  bool exhausted = false;
  auto budget = [&]() {
    exhausted = exhausted || checks >= config_.max_map_checks ||
        expansions >= config_.max_expansions || scans > config_.max_nodes ||
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count() >=
            config_.query_budget_ms;
    return !exhausted;
  };
  auto global_free = [&](const V &p) {
    if (!budget()) return false;
    ++checks;
    return map.global(p) == TargetRouteEvidence::FREE;
  };
  auto local_free = [&](const V &p) {
    if (!budget()) return false;
    ++checks;
    return map.local_free(p);
  };
  // A near final goal does not need a graph if its ENTIRE straight connector
  // has current raw-known-free evidence. This is local terminal handoff, not
  // a claim of global-topology reuse, nor an UNKNOWN/LIO-only fallback.
  const double direct_length = (goal - start).norm();
  if (now >= local_goal_retry_after_ && direct_length <= config_.prefix_length &&
      direct_length > 1e-4 && lineFree(start, goal, config_.sample_step, local_free)) {
    result.source = TargetRouteSource::LOCAL_GOAL;
    result.points = {start, goal}; result.nodes = {0, 0}; result.arc = {0.0, direct_length};
    result.goal = goal;
    status_ = "LOCAL_GOAL";
    return result;
  }
  if (!snapshot || snapshot->graph.empty() || !map.global) return result;
  const double radius = clampFinite(snapshot->config.connection_radius, 6.0, 0.5, 12.0);
  std::vector<std::pair<double, Id>> starts, ends;
  query_stage_ = "nearest_nodes";
  for (const auto &entry : snapshot->graph) {
    ++scans;
    if (!budget()) break;
    const auto &node = entry.second.node;
    if (!node.position.allFinite() || cooling(0, entry.first, now)) continue;
    const double ds = (node.position - start).norm();
    const double dg = (node.position - goal).norm();
    if (ds <= radius) starts.emplace_back(ds, entry.first);
    if (dg <= radius) ends.emplace_back(dg, entry.first);
  }
  if (exhausted) { status_ = "QUERY_BUDGET"; return result; }
  std::sort(starts.begin(), starts.end());
  std::sort(ends.begin(), ends.end());
  if (starts.size() > 16) starts.resize(16);
  if (ends.size() > 16) ends.resize(16);
  std::unordered_map<Id, double> start_cost, end_cost;
  query_stage_ = "start_attachments";
  for (const auto &s : starts)
    if (lineFree(start, snapshot->graph.at(s.second).node.position, config_.sample_step, local_free)) {
      start_cost[s.second] = s.first;
      if (start_cost.size() >= 2) break;
    }
  query_stage_ = "goal_attachments";
  for (const auto &g : ends)
    if (lineFree(snapshot->graph.at(g.second).node.position, goal, config_.sample_step, global_free)) {
      end_cost[g.second] = g.first;
      if (end_cost.size() >= 2) break;
    }
  status_ = exhausted ? "QUERY_BUDGET" : "NO_ATTACHMENT";
  if (exhausted || start_cost.empty()) return result;

  // At most three searches: invalidate a stale edge/connector and try another
  // route within the SAME wall/expansion budget. No unbounded per-anchor A*.
  for (int attempt = 0; attempt < 3 && budget(); ++attempt) {
    query_stage_ = "graph_search";
    using QueueItem = std::pair<double, Id>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    std::unordered_map<Id, double> distance;
    std::unordered_map<Id, Id> parent;
    for (const auto &s : start_cost) {
      if (cooling(0, s.first, now)) continue;
      distance[s.first] = s.second;
      parent[s.first] = 0;
      open.emplace(s.second, s.first);
    }
    Id goal_node = 0;
    double goal_cost = std::numeric_limits<double>::infinity();
    while (!open.empty() && budget()) {
      const auto item = open.top(); open.pop();
      if (item.first > distance[item.second] + 1e-6) continue;
      if (item.first > goal_cost) break;
      ++expansions;
      auto endpoint = end_cost.find(item.second);
      if (endpoint != end_cost.end() && item.first + endpoint->second < goal_cost) {
        goal_node = item.second;
        goal_cost = item.first + endpoint->second;
      }
      const auto &entry = snapshot->graph.at(item.second);
      for (const auto &edge : entry.neighbors) {
        if (!budget()) break;
        const auto next = snapshot->graph.find(edge.first);
        if (next == snapshot->graph.end() || !std::isfinite(edge.second) || edge.second < 0.0 ||
            cooling(item.second, edge.first, now) || cooling(0, edge.first, now)) continue;
        const double cost = item.first + std::max(edge.second,
            (entry.node.position - next->second.node.position).norm());
        const auto old = distance.find(edge.first);
        if (old == distance.end() || cost < old->second - 1e-6) {
          distance[edge.first] = cost;
          parent[edge.first] = item.second;
          open.emplace(cost, edge.first);
        }
      }
    }
    if (!budget()) break;
    bool reaches_goal = goal_node != 0;
    if (!reaches_goal) {
      query_stage_ = "anchor_evidence";
      // Expansion masks are portals, not frontiers: require reachable FREE
      // approach plus actual UNKNOWN beyond the corresponding portal face.
      std::vector<QueueItem> anchors;
      for (const auto &d : distance) {
        if (!budget()) break;
        const auto &node = snapshot->graph.at(d.first).node;
        if (!node.expansion_mask || d.second < 1.0 || cooling(0, d.first, now)) continue;
        anchors.emplace_back(d.second + (node.position - goal).norm(), d.first);
      }
      std::sort(anchors.begin(), anchors.end());
      if (anchors.size() > 16) anchors.resize(16);
      for (const auto &a : anchors) {
        const auto &node = snapshot->graph.at(a.second).node;
        if (!global_free(node.position)) continue;
        bool frontier = false;
        for (int face = 0; face < 6 && !frontier && budget(); ++face) {
          if (!(node.expansion_mask & (1U << face))) continue;
          V dir = V::Zero(); dir[face / 2] = face % 2 ? 1.0 : -1.0;
          for (double l = config_.sample_step; l <= 2.0 && budget(); l += config_.sample_step) {
            ++checks;
            const auto e = map.global(node.position + l * dir);
            if (e == TargetRouteEvidence::UNKNOWN) { frontier = true; break; }
            if (e != TargetRouteEvidence::FREE) break;
          }
        }
        if (frontier) { goal_node = a.second; break; }
      }
    }
    if (!goal_node || !budget()) break;
    std::vector<Id> chain;
    for (Id id = goal_node; id && chain.size() <= parent.size(); id = parent.at(id)) chain.push_back(id);
    std::reverse(chain.begin(), chain.end());
    result.points = {start}; result.nodes = {0};
    query_stage_ = "route_validation";
    bool valid = true;
    for (Id id : chain) {
      const V &p = snapshot->graph.at(id).node.position;
      if (!lineFree(result.points.back(), p, config_.sample_step, global_free)) {
        if (!exhausted) cool(result.nodes.back(), id, now);
        valid = false; break;
      }
      // Keep coincident graph nodes too: attachment/edge identities must not
      // disappear merely because the vehicle is exactly on a graph vertex.
      result.points.push_back(p); result.nodes.push_back(id);
    }
    if (!valid) { result = TargetRoute{}; continue; }
    if (!budget()) break;
    if (reaches_goal) { result.points.push_back(goal); result.nodes.push_back(0); }
    result.arc = {0.0};
    for (std::size_t i = 1; i < result.points.size(); ++i)
      result.arc.push_back(result.arc.back() + (result.points[i] - result.points[i - 1]).norm());
    result.source = reaches_goal ? TargetRouteSource::KNOWN_GOAL : TargetRouteSource::KNOWN_ANCHOR;
    result.topology_revision = snapshot->revision;
    result.goal = goal;
    status_ = reaches_goal ? "KNOWN_GOAL" : "KNOWN_ANCHOR";
    return result;
  }
  status_ = exhausted ? "QUERY_BUDGET" : "NO_ROUTE";
  return TargetRoute{};
}

TargetRoutePrefix TargetRouteRuntime::prefix(const V &position, double now,
                                             const TargetRouteMapView &map) {
  TargetRoutePrefix out;
  if (!route_.valid()) { out.reason = "NO_ACTIVE_ROUTE"; return out; }
  if (!map.local_free || !map.local_free(position)) {
    out.reason = "LOCAL_START_UNOBSERVED"; return out;
  }
  // Projection cannot jump to a far-away branch of a folded/U-shaped route.
  // Only observed displacement expands the forward arc search window.
  const double motion = have_odom_ ? (position - last_odom_).norm() : 0.0;
  const double dt = have_odom_ ? std::max(0.0, now - last_sample_time_) : 0.0;
  if (have_odom_ && (!std::isfinite(motion) || motion > std::max(3.0, 20.0 * dt))) {
    fail(now, "ODOMETRY_JUMP"); return out;
  }
  const double low = std::max(0.0, route_.progress - 0.5);
  const double high = std::min(route_.arc.back(), route_.progress + std::max(1.0, 1.5 * motion + 0.5));
  double best_arc = route_.progress, best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < route_.points.size(); ++i) {
    if (route_.arc[i] < low || route_.arc[i - 1] > high) continue;
    const V delta = route_.points[i] - route_.points[i - 1];
    const double len = delta.norm();
    if (len < 1e-6) continue;
    const double s = std::clamp(route_.arc[i - 1] +
        (position - route_.points[i - 1]).dot(delta) / len,
        std::max(low, route_.arc[i - 1]), std::min(high, route_.arc[i]));
    const double d = (pointAt(route_, s) - position).norm();
    if (d < best_distance) { best_distance = d; best_arc = s; }
  }
  last_odom_ = position; have_odom_ = true; last_sample_time_ = now;
  if (best_distance > config_.projection_radius) { fail(now, "OFF_ROUTE"); return out; }
  // Bounded local repair: try only nearby forward joins, each with a strict
  // current-map connector. Never globally project or chord across a wall.
  double join = best_arc;
  bool attached = lineFree(position, pointAt(route_, join), config_.sample_step, map.local_free);
  for (int i = 1; !attached && i <= 3; ++i) {
    join = std::min(high, best_arc + 0.25 * i);
    attached = lineFree(position, pointAt(route_, join), config_.sample_step, map.local_free);
  }
  if (!attached) { fail(now, "LOCAL_REJOIN_BLOCKED"); return out; }
  // The planned join/prefix endpoint is NOT progress. Only odometry's bounded
  // projection can credit arc movement, including necessary Euclidean retreat.
  route_.progress = std::max(route_.progress, best_arc);
  if (route_.progress >= route_.progress_checkpoint + 0.20) {
    route_.progress_checkpoint = route_.progress;
    route_.last_progress_time = now;
  }
  if (now - route_.last_progress_time > config_.progress_timeout) {
    fail(now, "ROUTE_NO_PROGRESS"); return out;
  }
  if (route_.source == TargetRouteSource::KNOWN_ANCHOR &&
      route_.arc.back() - route_.progress <= 0.5) {
    fail(now, "ANCHOR_REACHED_LOCAL_HANDOFF"); return out;
  }
  const double limit = std::min(route_.arc.back(), join + config_.prefix_length);
  double end = join;
  bool clipped = false;
  while (end < limit - 1e-6) {
    const double next = std::min(limit, end + config_.sample_step);
    if (!map.local_free(pointAt(route_, next))) { clipped = true; break; }
    end = next;
  }
  if (clipped) end = std::max(join, end - config_.stop_margin);
  if (end - join < config_.min_prefix_length &&
      (clipped || (route_.source != TargetRouteSource::KNOWN_GOAL &&
                   route_.source != TargetRouteSource::LOCAL_GOAL) || limit < route_.arc.back() - 1e-6)) {
    fail(now, "LOCAL_PREFIX_SHORT"); return out;
  }
  out.path.push_back(position.cast<float>());
  auto append = [&](const V &p) {
    if ((p - out.path.back().cast<double>()).norm() > 1e-4) out.path.push_back(p.cast<float>());
  };
  append(pointAt(route_, join));
  for (std::size_t i = 0; i < route_.points.size(); ++i)
    if (route_.arc[i] > join && route_.arc[i] < end) append(route_.points[i]);
  append(pointAt(route_, end));
  // Recheck every complete segment: sampling across a vertex must not omit
  // either side of a short corner, including the repaired start connector.
  for (std::size_t i = 1; i < out.path.size(); ++i)
    if (!lineFree(out.path[i - 1].cast<double>(), out.path[i].cast<double>(),
                  config_.sample_step, map.local_free)) {
      fail(now, "PREFIX_SEGMENT_BLOCKED"); return TargetRoutePrefix{};
    }
  out.context.source = route_.source;
  out.context.route_id = route_.id; out.context.task = route_.task; out.context.world = route_.world;
  // Conservative V2 execution: every local prefix ends at rest. Replanning
  // can extend it while moving; unknown edges never carry terminal velocity.
  out.context.stop_at_boundary = true;
  out.remaining = route_.arc.back() - route_.progress;
  out.reason = clipped ? "OBSERVED_PREFIX_STOP" : "KNOWN_PREFIX_STOP";
  return out;
}

TargetRoutePrefix TargetRouteRuntime::prepare(const Graph::SearchSnapshotPtr &snapshot,
    std::uint64_t world, const V &position, const V &goal, double now,
    const TargetRouteMapView &map, double local_objective) {
  if (config_.mode == "legacy" || !position.allFinite() || !goal.allFinite() || !std::isfinite(now))
    return {};
  if (world != world_ || !goal_.allFinite() || (goal - goal_).norm() > 1e-4 ||
      (have_odom_ && now < last_sample_time_)) {
    reset(); world_ = world; goal_ = goal;
  }
  cooldowns_.erase(std::remove_if(cooldowns_.begin(), cooldowns_.end(),
      [now](const Cooldown &c) { return c.until <= now; }), cooldowns_.end());
  if (!route_.valid() && now - last_query_ >= config_.query_interval) {
    last_query_ = now;
    auto candidate = query(snapshot, position, goal, now, map);
    if (candidate.valid()) {
      const double objective = candidate.arc.back() + (candidate.points.back() - goal).norm();
      if (candidate.source == TargetRouteSource::KNOWN_ANCHOR &&
          objective + config_.anchor_switch_margin >= local_objective) {
        status_ = "LOCAL_ROUTE_PREFERRED";
      } else {
        candidate.id = ++next_id_; candidate.task = task_; candidate.world = world;
        candidate.last_progress_time = now;
        route_ = std::move(candidate);
        last_odom_ = position; have_odom_ = true; last_sample_time_ = now;
      }
    }
  }
  auto out = prefix(position, now, map);
  if (config_.mode == "shadow") return {};
  return out;
}
}  // namespace fast_planner
