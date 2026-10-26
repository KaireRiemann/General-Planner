#include <rog_map/prob_map.h>
#include <random>
#include <thread>
#include <atomic>
#include <iostream>
#include <stdexcept>

class TestMap : public rog_map::ProbMap {
public:
    explicit TestMap(const std::string &path) {
        cfg_ = rog_map::Config(path);
        cfg_.corridor_obstacle_backend = "bdm";
        cfg_.bdm_search_enable = true;
        cfg_.map_size_d = rog_map::Vec3f(12, 12, 6);
        cfg_.map_sliding_en = true;
        cfg_.resetMapSize();
        initProbMap();
        slideAllMap(rog_map::Vec3f(0, 0, 1.5));
    }
    void slide(const rog_map::Vec3f &p) { slideAllMap(p); }
    void clear() { resetLocalMap(); }
    void verify() { cfg_.corridor_obstacle_backend = "verify"; }
    void margin(int n) { sc_.safe_margin_i = n; }
};

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    using namespace rog_map;
    try {
        TestMap map(argv[1]);
        std::mt19937 rng(19);
        std::uniform_real_distribution<double> xy(-7, 7), z(-1, 4);
        std::size_t comparisons = 0;
        auto compare = [&](const Vec3f &origin) {
            for (int n = 0; n < 150; ++n) {
                Vec3f a(xy(rng), xy(rng), z(rng)), b(xy(rng), xy(rng), z(rng));
                Vec3f lo = a.cwiseMin(b) + origin, hi = a.cwiseMax(b) + origin;
                vec_E<Vec3f> dense, sparse;
                map.boxSearch(lo, hi, general_utils::OCCUPIED, dense);
                map.corridorOccupiedSearch(lo, hi, sparse);
                if (dense != sparse) throw std::runtime_error("random query differs from dense scan");
                const auto inf = map.inflationMap(false);
                Vec3i id; inf->infMapPosToGlobalIndex((lo+hi)*0.5,id);
                std::vector<std::pair<int,int>> runs;
                if (!inf->bdmIntervals(id.x(),id.y(),-30,40,false,runs))
                    throw std::runtime_error("inflated BDM unavailable");
                for (int k=-30;k<=40;++k) {
                    const Vec3i index(id.x(),id.y(),k);
                    bool represented=false;
                    for (const auto &run:runs) represented |= k>=run.first && k<=run.second;
                    const auto state=inf->getGridType(index);
                    // Query conservatively excludes the outer sliding boundary;
                    // every represented cell must be traversable, and inner
                    // traversable cells must be represented.
                    if (represented && (state==general_utils::OCCUPIED || state==general_utils::OUT_OF_MAP))
                        throw std::runtime_error("inflated BDM false free");
                    Vec3f p; inf->infMapGlobalIndexToPos(index,p);
                    Vec3f inner_lo=p-Vec3f::Constant(.3),inner_hi=p+Vec3f::Constant(.3);
                    map.boundBoxByLocalMap(inner_lo,inner_hi,true);
                    if (state!=general_utils::OCCUPIED && state!=general_utils::OUT_OF_MAP &&
                        (inner_lo.array()<p.array()).all() && (inner_hi.array()>p.array()).all() && !represented)
                        throw std::runtime_error("inflated BDM missing free");
                }
                ++comparisons;
            }
        };
        compare(Vec3f::Zero()); // Empty map.
        // Occupancy-only initialization before MapManager tracking is enabled.
        PointCloud cloud;
        for (int n = 0; n < 900; ++n) {
            PclPoint p; p.x = xy(rng); p.y = xy(rng); p.z = z(rng); p.intensity = 1;
            cloud.push_back(p);
        }
        map.updateOccPointCloud(cloud);
        compare(Vec3f::Zero());
        map.margin(3); compare(Vec3f::Zero()); map.margin(0);
        // Partial slides in both directions, distant clear, and revisiting old cells.
        for (const Vec3f &shift : {Vec3f(2, -2, 1.5), Vec3f(-2, 2, 1.5),
                                 Vec3f(60, -60, 1.5), Vec3f(0, 0, 1.5)}) {
            map.slide(shift);
            compare(Vec3f(shift.x(), shift.y(), 0));
        }
        Pose pose{Vec3f(.075, .075, 1.575), Eigen::Quaterniond::Identity()};
        PclPoint near; near.x = 1.575; near.y = .075; near.z = 1.575; near.intensity = 1;
        PclPoint far = near; far.x = 3.075;
        cloud.clear(); cloud.push_back(near);
        for (int i=0; i<8; ++i) map.updateProbMap(cloud, pose);
        compare(Vec3f::Zero());
        cloud.clear(); cloud.push_back(far);
        for (int i=0; i<12; ++i) map.updateProbMap(cloud, pose);
        compare(Vec3f::Zero());
        map.clear(); compare(Vec3f::Zero());
        map.verify();
        std::atomic<bool> failed{false};
        std::thread reader([&]() {
            try {
                for (int n=0; n<500; ++n) {
                    vec_E<Vec3f> out;
                    map.corridorOccupiedSearch(Vec3f(-5,-5,0), Vec3f(5,5,2.5), out);
                }
            } catch (...) { failed = true; }
        });
        for (int n=0; n<60; ++n) {
            cloud.clear(); cloud.push_back(n%2 ? near : far);
            pose.first.y() = n%2 ? .075 : .675;
            map.updateProbMap(cloud, pose);
        }
        reader.join();
        if (failed) throw std::runtime_error("concurrent snapshot mismatch");
        std::cout << "bdm_corridor_self_test PASS comparisons=" << comparisons << std::endl;
    } catch (const std::exception &e) { std::cerr << e.what() << std::endl; return 1; }
}
