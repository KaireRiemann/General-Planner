#include <general_core/state2state/state2state_topology_route.hpp>

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

    std::cout << "state2state_topology_route_self_test passed\n";
    return 0;
}
