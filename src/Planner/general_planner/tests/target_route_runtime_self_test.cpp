#include <general_core/exploration/highspeed/target_route_runtime.h>
#include <cassert>
#include <chrono>
#include <iostream>

using namespace fast_planner;
using V = Eigen::Vector3d;
using Graph = TargetRouteRuntime::Graph;

Graph::SearchSnapshotPtr makeGraph(const std::vector<V> &points, bool portal = false) {
  auto graph = std::make_shared<Graph::SearchSnapshot>();
  graph->config.connection_radius = 0.6;
  graph->revision = 42;
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto &entry = graph->graph[i + 1];
    entry.node.id = i + 1;
    entry.node.position = points[i];
    if (i) {
      const double cost = (points[i] - points[i - 1]).norm();
      entry.neighbors[i] = cost;
      graph->graph[i].neighbors[i + 1] = cost;
    }
  }
  if (portal) graph->graph[points.size()].node.expansion_mask = Graph::PORTAL_POS_X;
  return graph;
}

int main() {
  const V start(0,0,1), goal(4,0,1);
  const std::vector<V> u = {start, V(0,4,1), V(4,4,1), goal};
  const auto graph = makeGraph(u);
  auto free = [](const V &p) {
    return p.allFinite() && std::abs(p.z()-1) < 0.2 &&
        ((std::abs(p.x()) < 0.2 && p.y() >= -0.2 && p.y() <= 4.2) ||
         (std::abs(p.y()-4) < 0.2 && p.x() >= -0.2 && p.x() <= 4.2) ||
         (std::abs(p.x()-4) < 0.2 && p.y() >= -0.2 && p.y() <= 4.2));
  };
  TargetRouteMapView map;
  map.local_free = free;
  map.global = [free](const V &p) { return free(p) ? TargetRouteEvidence::FREE : TargetRouteEvidence::OCCUPIED; };
  TargetRouteConfig config;
  config.query_budget_ms = 50.0;  // deterministic under loaded CI
  TargetRouteRuntime runtime;
  runtime.configure(config);
  assert(!runtime.prepare({}, 1, start, goal, 0, map).ready());
  auto prefix = runtime.prepare(graph, 1, start, goal, 2, map);
  assert(prefix.ready() && prefix.context.source == TargetRouteSource::KNOWN_GOAL);
  assert(prefix.path.size() == 4);  // both U corners preserved, not [start, goal]
  assert(prefix.context.stop_at_boundary && std::abs(runtime.route().arc.back()-12.0) < 1e-6);
  const auto route_id = runtime.route().id;
  prefix = runtime.prepare(graph, 1, V(0,.5,1), goal, 2.5, map);
  assert(prefix.ready() && runtime.route().id == route_id);
  assert(runtime.route().progress > .49 && (V(0,.5,1)-goal).norm() > (start-goal).norm());
  // Replanning a long prefix while stationary must never earn planned progress.
  runtime.prepare(graph, 1, V(0,.5,1), goal, 3.5, map);
  assert(runtime.route().progress < .51);
  assert(!runtime.prepare(graph, 1, V(0,.5,1), goal, 16, map).ready());
  assert(runtime.status() == "ROUTE_NO_PROGRESS" && runtime.cooldownSize() > 0);
  assert(!runtime.prepare(graph, 1, V(0,.5,1), goal, 17.1, map).ready());
  assert(runtime.cooldownSize() > 0);  // new query ids do not clear edge cooldown

  runtime.reset();
  auto local_window = map;
  local_window.local_free = [free](const V &p) { return free(p) && p.y() < 2.0; };
  prefix = runtime.prepare(graph, 1, start, goal, 20, local_window);
  assert(prefix.ready() && prefix.path.back().y() < 1.61 && prefix.path.back().y() > 1.3);
  assert(prefix.reason == "OBSERVED_PREFIX_STOP");
  const auto task = prefix.context.task;
  prefix = runtime.prepare(graph, 2, start, goal, 21, map);
  assert(prefix.ready() && prefix.context.world == 2 && prefix.context.task != task);
  assert(runtime.cooldownSize() == 0 && runtime.route().progress == 0.0);
  const auto task2 = prefix.context.task;
  runtime.prepare(graph, 2, start, V(4,.1,1), 22, map);
  assert(runtime.route().task != task2);  // exact goal replacement lifecycle

  runtime.reset();
  runtime.prepare(graph, 1, start, goal, 30, map);
  assert(!runtime.prepare(graph, 1, V(4,0,1), goal, 30.01, map).ready());
  assert(runtime.status() == "ODOMETRY_JUMP");
  // Nearby crossing/folded branch cannot advance 12m without measured motion.
  runtime.reset();
  runtime.prepare(graph, 1, start, goal, 40, map);
  assert(!runtime.prepare(graph, 1, V(4,0,1), goal, 45, map).ready());
  assert(runtime.status() == "OFF_ROUTE");

  auto blocked = map;
  blocked.global = [free](const V &p) {
    return free(p) && !(p.y() > 1.9 && p.y() < 2.1 && p.x() < .1)
        ? TargetRouteEvidence::FREE : TargetRouteEvidence::OCCUPIED;
  };
  runtime.reset();
  assert(!runtime.prepare(graph, 1, start, goal, 50, blocked).ready());
  assert(runtime.cooldownSize() > 0);  // stale historical edge rejected
  auto unknown_local = map;
  unknown_local.local_free = [](const V &) { return false; };
  runtime.reset();
  assert(!runtime.prepare(graph, 1, start, goal, 51, unknown_local).ready());

  // Reachable anchors require actual UNKNOWN evidence, not an expansion bit.
  const auto anchor_graph = makeGraph({start, V(3,0,1)}, true);
  auto anchor_map = map;
  anchor_map.local_free = [](const V &) { return true; };
  anchor_map.global = [](const V &p) {
    return p.x() <= 3.1 ? TargetRouteEvidence::FREE : TargetRouteEvidence::UNKNOWN;
  };
  runtime.reset();
  prefix = runtime.prepare(anchor_graph, 1, start, V(100,0,1), 60, anchor_map);
  assert(prefix.ready() && prefix.context.source == TargetRouteSource::KNOWN_ANCHOR);
  runtime.reset();
  assert(!runtime.prepare(anchor_graph, 1, start, V(100,0,1), 61, anchor_map, 99).ready());
  assert(runtime.status() == "LOCAL_ROUTE_PREFERRED");
  anchor_map.global = [](const V &) { return TargetRouteEvidence::FREE; };
  runtime.reset();
  assert(!runtime.prepare(anchor_graph, 1, start, V(100,0,1), 62, anchor_map).ready());

  config.mode = "shadow"; runtime.configure(config);
  assert(!runtime.prepare(graph, 1, start, goal, 70, map).ready() && runtime.route().valid());
  config.mode = "legacy"; runtime.configure(config);
  assert(!runtime.prepare(graph, 1, start, goal, 71, map).ready() && !runtime.route().valid());
  config.mode = "prefer_known"; config.max_map_checks = 64; runtime.configure(config);
  assert(!runtime.prepare(graph, 1, start, goal, 72, map).ready());
  assert(runtime.status() == "QUERY_BUDGET");
  config.max_nodes = 32; config.max_map_checks = 16000; runtime.configure(config);
  std::vector<V> many;
  for (int i=0; i<100; ++i) many.emplace_back(i,0,1);
  const auto began = std::chrono::steady_clock::now();
  assert(!runtime.prepare(makeGraph(many), 1, start, goal, 73, map).ready());
  assert(runtime.status() == "QUERY_BUDGET");
  assert(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count() < 1.0);

  // Actual command sampler: first/final/sub-dt points, intermediate chord and
  // backup suffix all go through the same strict callback, with/without backup.
  const auto position = [](double t) { return V(t,0,1); };
  const auto all_free = [](const V &p) { return p.allFinite(); };
  assert(targetRouteTrajectoryKnownFree(.005, .01, .1, position, all_free));
  assert(!targetRouteTrajectoryKnownFree(.005, .01, .1, position,
      [](const V &p) { return p.x() < .0049; }));
  assert(!targetRouteTrajectoryKnownFree(.105, .01, .1, position,
      [](const V &p) { return p.x() < .1049; }));
  assert(!targetRouteTrajectoryKnownFree(1, .1, .1, position,
      [](const V &p) { return p.x() > 0; }));
  assert(!targetRouteTrajectoryKnownFree(1, 1, .01, position,
      [](const V &p) { return p.x() < .49 || p.x() > .51; }));
  assert(!targetRouteTrajectoryKnownFree(2, .01, .1, position,
      [](const V &p) { return p.x() < 1.5; }));
  assert(!targetRouteTrajectoryKnownFree(0, .01, .1, position, all_free));
  config = TargetRouteConfig{}; config.query_budget_ms = 50.0;
  runtime.configure(config);
  TargetRouteMapView local_only;
  local_only.local_free = all_free;
  prefix = runtime.prepare({}, 1, start, goal, 80, local_only);
  assert(prefix.ready() && prefix.context.source == TargetRouteSource::LOCAL_GOAL);
  runtime.fail(80.1, "LOCAL_PLAN_FAILED");
  assert(!runtime.prepare({}, 1, start, goal, 81.2, local_only).ready());
  assert(runtime.prepare({}, 1, start, goal, 86, local_only).ready());
  runtime.reset();
  assert(!runtime.prepare({}, 1, start, V(100,0,1), 90, local_only).ready());
  std::cout << "target route full polyline / U-detour / prefix / lifecycle / cooldown / budget / commit endpoints: PASS\n";
}
