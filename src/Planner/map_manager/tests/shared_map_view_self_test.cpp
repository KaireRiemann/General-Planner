#include <map_manager/map_manager.hpp>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "shared_map_view_self_test");
    if (argc != 2) return 2;
    try {
        ros::NodeHandle nh("~");
        using namespace rog_map;
        auto grid = std::make_shared<ROGMapROS>(nh, argv[1]);
        auto owner = std::make_shared<general_planner::MapManager>(grid);
        auto tracking = general_planner::MapManager::trackingView(owner);
        check(grid->inflationMap(true)->insideLocalMap(Vec3f(.075, .075, 1.575)),
              "tracking layer must be positioned before the first scan or PCD load");
        check(owner->rawMap() == tracking->rawMap(), "occupancy must be shared");
        check(owner->boundaryReady() && !tracking->boundaryReady(), "only owner maintains history");
        check(!tracking->topologyReady(), "tracking must not construct another topology");
        check(owner->getMapConfig().inflation_step == 3 &&
              tracking->getMapConfig().inflation_step == 1, "inflation margins must remain distinct");
        double floor, ceiling, track_floor, track_ceiling;
        owner->getInflatedVirtualHeightBounds(floor, ceiling);
        tracking->getInflatedVirtualHeightBounds(track_floor, track_ceiling);
        check(std::abs(floor - track_floor - .30) < 1e-8 &&
              std::abs(track_ceiling - ceiling - .30) < 1e-8, "per-view virtual bounds");
        owner->setWorldEpoch(7);
        owner->enableIndependentOdometry();
        RobotState state{};
        state.p = Vec3f(.075, .075, 1.575); state.rcv = true;
        owner->updateOdometrySnapshot(state);
        check(tracking->worldEpoch() == 7 && tracking->getRobotState().p == state.p,
              "view must share epoch and independent odometry");
        Pose pose{state.p, Quatf::Identity()};
        PointCloud cloud;
        PclPoint near;
        near.x = 1.575; near.y = .075; near.z = 1.575; near.intensity = 1;
        cloud.push_back(near);
        owner->updateMap(cloud, pose);
        const Vec3f offset(1.575, .375, 1.575);
        check(owner->isOccupiedInflate(offset) && !tracking->isOccupiedInflate(offset),
              "same hit must give thick navigation and thin tracking inflation");
        check(owner->mapRevision() == tracking->mapRevision() &&
              tracking->latestUpdate().revision == 1, "one fusion revision for both views");
        bool rejected = false;
        try { tracking->updateMap(cloud, pose); } catch (const std::logic_error &) { rejected = true; }
        check(rejected, "query view must reject duplicate fusion");
        const Vec3f a(1.275, .375, 1.575), b(1.875, .375, 1.575);
        check(!owner->isLineFree(a, b, true, false) && tracking->isLineFree(a, b, true, false),
              "line queries must select their inflation view");
        vec_E<Vec3f> hits;
        tracking->boxSearchInflate(offset-Vec3f::Constant(.01), offset+Vec3f::Constant(.01), OCCUPIED, hits);
        check(hits.empty(), "tracking box query must not leak navigation inflation");
        Vec3f nearest;
        check(tracking->getNearestInfCellNot(OCCUPIED, offset, nearest, .2), "tracking nearest-cell query");
        pose.first.y() += .6;
        owner->updateMap(cloud, pose);
        check(owner->isOccupiedInflate(offset) && !tracking->isOccupiedInflate(offset),
              "partial map slide preserves both inflation layers");
        pose.first = state.p;

        // Raw sensor evidence near the floor must survive the larger navigation margin.
        PclPoint low = near; low.x = .075; low.y = 2.025; low.z = .225;
        cloud.clear(); cloud.push_back(low); owner->updateMap(cloud, pose);
        check(grid->isOccupied(Vec3f(low.x, low.y, low.z)), "fusion must use physical height bounds");
        check(tracking->isOccupiedInflate(Vec3f(low.x, low.y, low.z)), "thin view receives near-floor hit");

        PclPoint far = near; far.x = 3.075;
        cloud.clear(); cloud.push_back(far);
        for (int i = 0; i < 8; ++i) owner->updateMap(cloud, pose);
        const Vec3f old_hit(near.x, near.y, near.z);
        check(!owner->isOccupiedInflate(old_hit) && !tracking->isOccupiedInflate(old_hit),
              "observed misses must remove inflation from both views");
        cloud.push_back(near);
        for (int i = 0; i < 8; ++i) owner->updateMap(cloud, pose);
        check(tracking->isOccupiedInflate(old_hit), "hit wins over crossing miss ray");
        tracking.reset(); // Must not detach owner's state-change callback.
        cloud.clear(); far.y = 1.575; cloud.push_back(far); owner->updateMap(cloud, pose);
        check(owner->getBoundaryGridType(Vec3f(far.x, far.y, far.z)) == OCCUPIED,
              "destroying a view must not detach history maintenance");
        tracking = general_planner::MapManager::trackingView(owner);
        // updateMap intentionally ignores empty scans, so drive sliding with a valid scan.
        pose.first.x() = 150.; cloud[0].x = 153.075; owner->updateMap(cloud, pose);
        pose.first.x() = .075; cloud[0].x = 3.075; owner->updateMap(cloud, pose);
        check(!owner->isOccupiedInflate(old_hit) && !tracking->isOccupiedInflate(old_hit),
              "ring-buffer reset must clear both inflation layers");
        std::cout << "shared_map_view_self_test PASS\n";
    } catch (const std::exception &e) {
        std::cerr << "shared_map_view_self_test FAIL: " << e.what() << '\n';
        return 1;
    }
}
