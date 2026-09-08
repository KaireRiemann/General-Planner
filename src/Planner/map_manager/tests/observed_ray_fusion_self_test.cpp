#include <rog_map/prob_map.h>
#include <iostream>
#include <stdexcept>

class ObservedMap : public rog_map::ProbMap {
public:
    explicit ObservedMap(const std::string &config) {
        cfg_ = rog_map::Config(config);
        cfg_.map_size_d = rog_map::Vec3f(12, 12, 6);
        cfg_.resetMapSize();
        initProbMap();
    }
};
void expect(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
int main(int argc, char **argv) {
    if (argc != 2) return 2;
    ObservedMap map(argv[1]);
    rog_map::Pose pose{rog_map::Vec3f(0.075, 0.075, 1.575), Eigen::Quaterniond::Identity()};
    rog_map::PclPoint near;
    near.x = 1.575f; near.y = 0.075f; near.z = 1.575f; near.intensity = 1;
    rog_map::PclPoint far = near; far.x = 3.075f;
    rog_map::PointCloud raw, confirmed;
    raw.push_back(near);
    for (int i = 0; i < 20; ++i) map.updateProbMap(raw, pose, &confirmed);
    expect(map.isKnownFree(rog_map::Vec3f(0.975, 0.075, 1.575)),
           "unconfirmed scans must bootstrap observed free space at rest");
    expect(!map.isKnownFree(rog_map::Vec3f(1.575, 0.075, 1.575)),
           "an unconfirmed endpoint is not free space");
    expect(map.isUnknown(rog_map::Vec3f(2.175, 0.075, 1.575)),
           "must not carve through an unconfirmed obstacle");
    confirmed = raw;
    for (int i = 0; i < 8; ++i) map.updateProbMap(raw, pose, &confirmed);
    expect(map.isOccupied(rog_map::Vec3f(1.575, 0.075, 1.575)),
           "confirmed static endpoint must be occupied");
    raw.clear(); raw.push_back(far); confirmed.clear();
    for (int i = 0; i < 12; ++i) map.updateProbMap(raw, pose, &confirmed);
    expect(map.isKnownFree(rog_map::Vec3f(1.575, 0.075, 1.575)),
           "later raw miss rays must clear old occupancy even with zero confirmed hits");
    expect(!map.isOccupied(rog_map::Vec3f(3.075, 0.075, 1.575)),
           "unconfirmed geometry must not become persistent occupancy");
    // Production path: raw fusion, no temporal gate. Timestamps/vehicle
    // baseline do not enter this API, including at 2 Hz or after a pause.
    for (int i = 0; i < 4; ++i) map.updateProbMap(raw, pose);
    expect(map.isOccupied(rog_map::Vec3f(3.075, 0.075, 1.575)),
           "raw return must build a local obstacle without temporal confirmation");
    rog_map::PclPoint farther = far; farther.x = 4.575f;
    raw.clear(); raw.push_back(farther);
    for (int i = 0; i < 12; ++i) map.updateProbMap(raw, pose);
    expect(map.isKnownFree(rog_map::Vec3f(3.075, 0.075, 1.575)),
           "raw local fusion must clear a departed obstacle");
    expect(map.isOccupied(rog_map::Vec3f(4.575, 0.075, 1.575)),
           "clearing old occupancy must not discard the new obstacle");
    std::cout << "observed_ray_fusion_self_test: PASS\n";
}
