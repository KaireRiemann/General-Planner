#include <rog_map/rog_map_core/raycast_geometry.hpp>

#include <cstdlib>
#include <iostream>

namespace {

void expect(const bool condition, const char* message) {
    if (!condition) {
        std::cerr << "raycast_geometry_self_test: " << message << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

}  // namespace

int main() {
    const rog_map::Vec3f sensor(1.0, 2.0, 1.0);
    rog_map::Vec3f endpoint(11.0, 2.0, 5.0);
    expect(rog_map::raycast_geometry::clipSegmentToHeightPlane(
               endpoint, sensor, 3.0),
           "a crossing ray must clip to the plane");
    expect((endpoint - rog_map::Vec3f(6.0, 2.0, 3.0)).norm() < 1.0e-9,
           "clip must preserve the segment ratio rather than normalising it");

    endpoint = rog_map::Vec3f(11.0, 2.0, 5.0);
    expect(!rog_map::raycast_geometry::clipSegmentToHeightPlane(
               endpoint, sensor, 0.0),
           "a plane outside the finite segment must be rejected");
    expect((endpoint - rog_map::Vec3f(11.0, 2.0, 5.0)).norm() < 1.0e-9,
           "a rejected clip must not mutate the endpoint");

    endpoint = rog_map::Vec3f(11.0, 2.0, 1.0);
    expect(!rog_map::raycast_geometry::clipSegmentToHeightPlane(
               endpoint, sensor, 3.0),
           "a segment parallel to the plane must be rejected");

    std::cout << "raycast_geometry_self_test: PASS" << std::endl;
    return EXIT_SUCCESS;
}
