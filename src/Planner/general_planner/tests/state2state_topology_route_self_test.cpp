#include <general_core/state2state/state2state_topology_route.hpp>
#include <general_core/state2state/state2state_topology_search.hpp>
#include <map_manager/incremental_topology_graph.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void expect(const bool condition, const char *message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(const double lhs, const double rhs, const double tolerance = 1.0e-6) {
    return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
    using general_planner::state2state_task::State2StateTopologyRouteRuntime;
    using general_planner::state2state_task::buildRouteArcLength;
    using general_planner::state2state_task::projectRouteMonotonically;
    using general_planner::state2state_task::sliceRouteByArcLength;
    using general_utils::Vec3f;
    using general_utils::vec_Vec3f;

    vec_Vec3f route{
        Vec3f(0.0, 0.0, 0.0),
        Vec3f(2.0, 0.0, 0.0),
        Vec3f(2.0, 3.0, 0.0)};
    std::vector<double> arc_length;
    buildRouteArcLength(route, arc_length);
    expect(arc_length.size() == 3, "arc length size");
    expect(near(arc_length.back(), 5.0), "arc length value");

    double projected_s = -1.0;
    Vec3f projected_point = Vec3f::Zero();
    expect(projectRouteMonotonically(route, arc_length, Vec3f(1.2, 0.4, 0.0),
                                     0.0, projected_s, projected_point),
           "initial projection");
    expect(near(projected_s, 1.2), "initial projection arc");
    expect(near(projected_point.x(), 1.2), "initial projection point");

    expect(projectRouteMonotonically(route, arc_length, Vec3f(0.1, 0.0, 0.0),
                                     2.5, projected_s, projected_point),
           "monotonic projection");
    expect(near(projected_s, 2.5), "monotonic lower bound");
    expect(near(projected_point.x(), 2.0) && near(projected_point.y(), 0.5),
           "monotonic projection point");

    vec_Vec3f prefix;
    expect(sliceRouteByArcLength(route, arc_length, 0.5, 3.5, prefix),
           "route slice");
    expect(prefix.size() == 3, "route slice points");
    expect(near(prefix.front().x(), 0.5), "route slice start");
    expect(near(prefix.back().x(), 2.0) && near(prefix.back().y(), 1.5),
           "route slice end");

    State2StateTopologyRouteRuntime runtime;
    expect(!runtime.policy_enabled.load(), "local-only default");
    runtime.setPolicy(true);
    expect(runtime.policy_enabled.load() && runtime.policy_generation.load() == 1,
           "enable topology policy");
    runtime.setPolicy(true);
    expect(runtime.policy_generation.load() == 1, "idempotent enable");
    runtime.setPolicy(false);
    expect(!runtime.policy_enabled.load() && runtime.policy_generation.load() == 2,
           "disable topology policy");

    using namespace general_planner::state2state_task;
    using Graph = general_planner::IncrementalTopologyGraph;
    Graph::SearchSnapshot graph;
    graph.config.connection_radius = 0.4;
    graph.config.max_neighbors = 8;
    // The useful relay is only 3m from the goal, but reaching it requires
    // initially moving backwards and travelling outside the 8m local horizon.
    const vec_Vec3f points{{0,0,1}, {-4,0,1}, {-4,30,1}, {6,30,1}, {6,0,1}};
    for (std::size_t i = 0; i < points.size(); ++i) {
        auto &node = graph.graph[i + 1];
        node.node.id = i + 1;
        node.node.position = points[i];
        if (i) {
            const double cost = (points[i] - points[i - 1]).norm();
            node.neighbors[i] = cost;
            graph.graph[i].neighbors[i + 1] = cost;
        }
    }
    const Vec3f outside_goal(9,0,1);
    // More than three closer nodes exist in an unreachable component.
    for (std::uint64_t id = 20; id < 25; ++id) {
        auto &node = graph.graph[id];
        node.node.id = id;
        node.node.position = Vec3f(8.0 + .1 * (id - 20), 0, 1);
        if (id > 20) {
            node.neighbors[id - 1] = .1;
            graph.graph[id - 1].neighbors[id] = .1;
        }
    }
    vec_Vec3f relay_route;
    auto free_attachment = [](const Vec3f &, const Vec3f &) { return true; };
    expect(findClosestReachableTopologyRoute(graph, points.front(), outside_goal,
                                             free_attachment, relay_route), "off-graph relay query");
    expect(relay_route.size() == points.size(), "must retain the long graph detour");
    for (std::size_t i = 0; i < points.size(); ++i)
        expect((relay_route[i] - points[i]).norm() < 1e-6, "graph corners must be retained");
    expect((relay_route.back() - points.back()).norm() < 1e-6,
           "nearest unreachable component must not win relay selection");
    expect(findClosestReachableTopologyRoute(graph, points.front(), Vec3f(6,40,1),
                                             free_attachment, relay_route) &&
           (relay_route.back() - points[3]).norm() < 1e-6,
           "relay outside three local planning horizons must remain eligible");
    expect(!findClosestReachableTopologyRoute(graph, points.front(), outside_goal,
        [](const Vec3f &, const Vec3f &) { return false; }, relay_route) && relay_route.empty(),
        "failed known-free attachment must never fabricate a direct route");

    runtime.setPolicy(true);
    runtime.consumed_policy_generation = runtime.policy_generation.load();
    runtime.route.valid = true;
    runtime.route.phase = TopologyRoutePhase::FOLLOW_GRAPH;
    runtime.route.goal = outside_goal;
    runtime.route.raw_topology_route = points;
    buildRouteArcLength(points, runtime.route.arc_length);
    // Even if predictive planning has consumed the entire route, actual
    // odometry is still at its beginning: local flight must remain locked.
    runtime.route.committed_route_s = runtime.route.arc_length.back();
    expect(!finishTopologyLeg(runtime.route, points.front(), .3) &&
           runtime.requiresTopologyLeg(outside_goal), "predictive progress released the relay early");
    expect(finishTopologyLeg(runtime.route, points.back(), .3) &&
           !runtime.requiresTopologyLeg(outside_goal), "measured relay arrival must release local flight");
    expect(runtime.requiresTopologyLeg(Vec3f(10,0,1)), "new goal must reacquire a graph relay");
    runtime.setTaskEpoch(2);
    expect(runtime.requiresTopologyLeg(outside_goal), "new task must not inherit local-flight phase");
    runtime.consumed_task_generation = runtime.task_generation.load();
    runtime.setPolicy(false);
    runtime.setPolicy(true);
    expect(runtime.requiresTopologyLeg(outside_goal), "policy re-enable must reacquire relay");
    resetGlobalTopologyRoute(runtime.route, "BLOCKED");
    expect(runtime.route.phase == TopologyRoutePhase::QUERY &&
           runtime.requiresTopologyLeg(outside_goal), "blocked route must not release local flight");

    std::cout << "state2state_topology_route_self_test passed\n";
    return 0;
}
