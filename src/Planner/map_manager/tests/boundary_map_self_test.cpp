#include <map_manager/boundary_map.hpp>
#include <rog_map/prob_map.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

class TestProbMap final : public rog_map::ProbMap {
public:
    void emitChange(const rog_map::Vec3i &index) {
        recordStateChange(index,
                          rog_map::GridType::UNKNOWN,
                          rog_map::GridType::KNOWN_FREE);
        notifyStateChangeCallback();
    }
};

bool expect(const bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "[boundary_map_self_test] " << message << std::endl;
        return false;
    }
    return true;
}

} // namespace

int main() {
    using general_planner::BoundaryMap;
    using BoundaryType = BoundaryMap::BoundaryType;
    using general_utils::GridType;
    using rog_map::Vec3f;
    using rog_map::Vec3i;

    bool ok = true;
    BoundaryMap map(0.5);

    // A free vertical interval [1,4] is represented by its two interior
    // boundary voxels and the adjacent exterior-unknown voxels only.
    map.apply({{Vec3i(0, 0, 0), BoundaryType::EXTERIOR_UNKNOWN},
               {Vec3i(0, 0, 1), BoundaryType::INTERIOR_FREE},
               {Vec3i(0, 0, 4), BoundaryType::INTERIOR_FREE},
               {Vec3i(0, 0, 5), BoundaryType::EXTERIOR_UNKNOWN}});
    ok &= expect(map.getGridType(Vec3i(0, 0, 0)) == GridType::UNKNOWN,
                 "exterior boundary must query as unknown");
    ok &= expect(map.getGridType(Vec3i(0, 0, 2)) == GridType::KNOWN_FREE,
                 "implicit free volume must be recovered from +z boundary");
    ok &= expect(map.getGridType(Vec3i(0, 0, 5)) == GridType::UNKNOWN,
                 "upper exterior boundary must close the free interval");
    ok &= expect(map.isFrontier(Vec3i(0, 0, 5)),
                 "exterior-unknown boundary must be directly retrievable");

    BoundaryMap::Stats stats = map.stats();
    ok &= expect(stats.column_count == 1 && stats.boundary_count == 4,
                 "free volume must store boundaries rather than dense voxels");
    ok &= expect(stats.interior_boundary_count == 2 && stats.frontier_count == 2,
                 "boundary type counters are inconsistent");

    map.apply({Vec3i(2, 0, 3), BoundaryType::OCCUPIED});
    ok &= expect(map.isOccupied(Vec3i(2, 0, 3)),
                 "exact occupied boundary query was lost");
    ok &= expect(map.getGridType(Vec3i(2, 0, 2)) == GridType::UNKNOWN,
                 "occupied boundary must not imply occupied free-space volume");

    map.apply({{Vec3i(4, 5, 6), BoundaryType::EXTERIOR_UNKNOWN},
               {Vec3i(4, 5, 6), BoundaryType::OCCUPIED}});
    ok &= expect(map.getGridType(Vec3i(4, 5, 6)) == GridType::OCCUPIED,
                 "last boundary status in a batch must win");

    map.apply({Vec3i(4, 5, 6), BoundaryType::NONE});
    ok &= expect(map.getBoundaryType(Vec3i(4, 5, 6)) == BoundaryType::NONE,
                 "NONE update must remove an outdated boundary voxel");

    const Vec3i negative = map.positionToIndex(Vec3f(-0.01, -0.51, 0.99));
    ok &= expect(negative == Vec3i(-1, -2, 1),
                 "negative coordinates must use floor discretization");
    const Vec3f center = map.indexToPosition(negative);
    ok &= expect((center - Vec3f(-0.25, -0.75, 0.75)).norm() < 1.0e-9,
                 "index-to-position must return voxel center");

    const auto frontiers = map.frontierPositions();
    ok &= expect(frontiers.size() == 2,
                 "frontier snapshot must expose exterior-unknown boundaries");

    TestProbMap delta_source;
    std::vector<rog_map::ProbMap::CellStateChange> observed_changes;
    delta_source.setStateChangeTrackingEnabled(true);
    delta_source.setStateChangeCallback([&]() {
        observed_changes = delta_source.drainStateChanges();
    });
    delta_source.emitChange(Vec3i(7, 8, 9));
    ok &= expect(observed_changes.size() == 1 &&
                     observed_changes.front().id_g == Vec3i(7, 8, 9),
                 "end-of-update observer must deliver the ROG delta stream");

    map.clear();
    stats = map.stats();
    ok &= expect(stats.column_count == 0 && stats.boundary_count == 0,
                 "clear must reset all sparse boundary columns");

    if (!ok) {
        return EXIT_FAILURE;
    }
    std::cout << "boundary_map_self_test: PASS" << std::endl;
    return EXIT_SUCCESS;
}
