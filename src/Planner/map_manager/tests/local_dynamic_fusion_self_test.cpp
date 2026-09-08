#include <rog_map/prob_map.h>
#include <iostream>
#include <stdexcept>

class TestMap : public rog_map::ProbMap {
public:
    explicit TestMap(const std::string &path) {
        cfg_ = rog_map::Config(path);
        cfg_.map_size_d = rog_map::Vec3f(12, 12, 6);
        cfg_.resetMapSize();
        initProbMap();
    }
};
void expect(bool ok, const char *what) {
    if (!ok) throw std::runtime_error(what);
}
int main(int argc, char **argv) {
    if (argc != 2) return 2;
    using rog_map::Vec3f;
    TestMap map(argv[1]);
    rog_map::Pose pose{Vec3f(.075, .075, 1.575), Eigen::Quaterniond::Identity()};
    rog_map::PointCloud cloud;
    map.updateProbMap(cloud, pose);
    expect(map.isUnknown(pose.first), "no scan must not clear a bootstrap sphere");
    rog_map::PclPoint near, far;
    near.x = 1.575f; near.y = .075f; near.z = 1.575f; near.intensity = 1;
    far = near; far.x = 3.075f;
    for (int i = 0; i < 1000; ++i) cloud.push_back(near);
    map.updateProbMap(cloud, pose);
    expect(map.isOccupied(Vec3f(near.x, near.y, near.z)), "raw hit builds obstacle at rest");
    expect(map.isOccupiedInflate(Vec3f(near.x, near.y, near.z)),
           "raw hit updates the inflated collision map");
    expect(map.getMapValue(pose.first) < .1,
           "first valid scan supplies origin free evidence without motion");
    cloud.clear(); cloud.push_back(far);
    map.updateProbMap(cloud, pose);
    expect(map.isKnownFree(pose.first), "two valid scans establish origin free space");
    for (int i = 0; i < 5; ++i) map.updateProbMap(cloud, pose);
    expect(map.isKnownFree(Vec3f(near.x, near.y, near.z)),
           "one dense frame must not count as 1000 hits; six observed misses clear it");
    expect(map.isOccupied(Vec3f(far.x, far.y, far.z)), "new obstacle stays occupied");
    cloud.push_back(near);
    for (int i = 0; i < 8; ++i) map.updateProbMap(cloud, pose);
    expect(map.isOccupied(Vec3f(near.x, near.y, near.z)),
           "current endpoint wins over a crossing ray in the same frame");
    cloud.clear();
    for (int i = 0; i < 10; ++i) map.updateProbMap(cloud, pose);
    expect(map.isOccupied(Vec3f(near.x, near.y, near.z)),
           "absence of a scan is not evidence of free space");
    cloud.push_back(far);
    for (int i = 0; i < 8; ++i) map.updateProbMap(cloud, pose);
    expect(map.isKnownFree(Vec3f(near.x, near.y, near.z)),
           "observed misses clear a saturated departed obstacle");
    expect(!map.isOccupiedInflate(Vec3f(near.x, near.y, near.z)),
           "departed obstacle must also disappear from the inflated collision map");
    pose.first = Vec3f(-62.625, 26.475, 1.575);
    far.x = -60.625f; far.y = 26.475f;
    cloud.clear(); cloud.push_back(far);
    for (int i = 0; i < 8; ++i) map.updateProbMap(cloud, pose);
    expect(map.isKnownFree(pose.first), "fusion resumes after a distant map slide");
    expect(map.isUnknown(Vec3f(pose.first + Vec3f(0, -.3, 0))),
           "unobserved neighbor is not sphere-cleared");
    std::cout << "local_dynamic_fusion_self_test: PASS\n";
}
