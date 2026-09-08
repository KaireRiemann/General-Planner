#include <rog_map/rog_map_core/raycaster.h>
#include <iostream>
#include <random>
#include <stdexcept>

int main() {
    using Eigen::Vector3d;
    const auto expect = [](bool ok, const char *message) {
        if (!ok) throw std::runtime_error(message);
    };
    std::mt19937 rng(20260908);
    std::uniform_int_distribution<int> grid(-200, 200);
    for (const double resolution : {0.1, 0.15, 0.2, 0.3}) {
        rog_map::raycaster::RayCaster caster(resolution);
        for (int trial = 0; trial < 15000; ++trial) {
            Vector3d start, end, sample;
            for (int axis = 0; axis < 3; ++axis) {
                start[axis] = (grid(rng) + (trial % 2 ? 0.5 : 0.0)) * resolution;
                end[axis] = (grid(rng) + (trial % 3 ? 0.0 : 0.5)) * resolution;
            }
            if (trial % 5 == 0) end.z() = start.z();
            if (trial % 7 == 0) end = start;
            Eigen::Vector3i first, last;
            for (int axis = 0; axis < 3; ++axis) {
                caster.posToIndex(start[axis], first[axis]);
                caster.posToIndex(end[axis], last[axis]);
            }
            const int max_steps = (first - last).cwiseAbs().sum();
            caster.setInput(start, end);
            int count = 0;
            while (caster.step(sample)) {
                if (++count > max_steps) {
                    std::cerr << "resolution=" << resolution << " start=" << start.transpose()
                              << " end=" << end.transpose() << std::endl;
                    throw std::runtime_error("ray overshot target / failed to terminate");
                }
                for (int axis = 0; axis < 3; ++axis) {
                    int index;
                    caster.posToIndex(sample[axis], index);
                    expect(index >= std::min(first[axis], last[axis]) &&
                           index <= std::max(first[axis], last[axis]), "axis overshot its terminal voxel");
                }
            }
            expect(count == max_steps, "ray skipped an intermediate voxel");
            expect(!caster.step(sample), "finished ray should stay finished");
        }
        Vector3d sample;
        caster.setInput(Vector3d::Zero(), Vector3d::Ones());
        expect(!caster.setInput(Vector3d::Constant(std::numeric_limits<double>::infinity()),
                                Vector3d::Zero()), "infinite input accepted");
        expect(!caster.step(sample), "invalid input reused a previous ray");
    }
    std::cout << "raycaster_self_test: PASS (60000 boundary/negative/degenerate rays)\n";
}
