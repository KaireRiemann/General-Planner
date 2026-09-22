// Offline replay of the 2026-09-21 tracking stall (bag
// tracking_20260921_125218_0.bag). The detector lost the car during its pass
// at t~68 s, the target filter removed points at the wrong place, and the
// car roof (z<=1.40 m) leaked into the occupancy map at x~-22.3 as a
// persistent ghost. With the shared world map (inflation_step=3, 0.45 m) the
// ghost inflates to z=1.875 m - exactly flight level - and the CIRI seed
// line passes 0.103 m from it, hard-failing every PlanFromRest. This test
// replays the extracted occupancy snapshot and asserts:
//   1. the thick world map reproduces the failure (seed blocked, strict
//      CIRI fails);
//   2. the tracking-dedicated thin map (inflation_step=1) frees the same
//      seed line and the tracking corridor generator (tight-seed enabled)
//      succeeds;
//   3. the full tracking frontend + corridor chain succeeds on the thin map
//      for the frozen target estimate from the bag.
#include <general_core/tracking/tracking_frontend.hpp>
#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_map_query.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

rog_map::PointCloud loadSnapshot(const std::string &path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) throw std::runtime_error("cannot open " + path);
    const std::streamsize bytes = in.tellg();
    in.seekg(0);
    std::vector<float> data(static_cast<std::size_t>(bytes) / sizeof(float));
    in.read(reinterpret_cast<char *>(data.data()), bytes);
    rog_map::PointCloud cloud;
    cloud.reserve(data.size() / 3);
    for (std::size_t i = 0; i + 2 < data.size(); i += 3) {
        rog_map::PointType point;
        point.x = data[i]; point.y = data[i + 1]; point.z = data[i + 2];
        point.intensity = 1.f;
        cloud.push_back(point);
    }
    return cloud;
}

double minDistanceToLine(const general_utils::vec_Vec3f &points,
                         const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    const Eigen::Vector3d ab = b - a;
    double best = std::numeric_limits<double>::max();
    for (const auto &p : points) {
        const Eigen::Vector3d ap = p - a;
        const double t = std::max(0., std::min(1., ap.dot(ab) / ab.squaredNorm()));
        best = std::min(best, (a + t * ab - p).norm());
    }
    return best;
}
}  // namespace

int main(int argc, char **argv) {
    const char *master = std::getenv("ROS_MASTER_URI");
    if (!master || std::string(master) != "http://127.0.0.1:11319") {
        std::cerr << "requires isolated ROS_MASTER_URI=http://127.0.0.1:11319\n";
        return 2;
    }
    ros::init(argc, argv, "tracking_wall_replay_self_test");
    try {
        ros::NodeHandle nh("~");
        const auto cloud = loadSnapshot(std::string(ROOT_DIR) +
                                        "tests/data/tracking_wall_20260921.bin");
        std::cout << "snapshot points: " << cloud.size() << '\n';

        auto grid_main = std::make_shared<rog_map::ROGMapROS>(
            nh, std::string(ROOT_DIR) + "config/exploration_rog_map.yaml");
        auto grid_track = std::make_shared<rog_map::ROGMapROS>(
            nh, std::string(ROOT_DIR) + "config/tracking_rog_map.yaml");
        grid_main->updateOccPointCloud(cloud);
        grid_track->updateOccPointCloud(cloud);
        auto map_main = std::make_shared<general_planner::MapManager>(grid_main);
        auto map_track = std::make_shared<general_planner::MapManager>(grid_track);
        require(std::abs(map_main->getInfResolution() - 0.15) < 1e-9 &&
                std::abs(map_track->getInfResolution() - 0.15) < 1e-9,
                "both maps share the 0.15 m grid; only inflation differs");

        // Runtime masks the tracked body. The bag's frozen (false) target
        // estimate was (-32.47, 2.52, -0.05); the exclusion cylinder is
        // lifted to tracking height exactly like the optimizer does.
        general_planner::setTrackingOccupancyExclusion(
            Eigen::Vector3d(-32.47, 2.52, -0.05 + 1.0), .8, .8);

        // The exact CIRI seed line from the live failure log.
        const Eigen::Vector3d seed_a(-22.025, 2.229, 1.863);
        const Eigen::Vector3d seed_b(-23.925, 2.175, 1.875);

        general_utils::vec_Vec3f occ_main, occ_track;
        general_planner::trackingOccupiedVoxels(
            map_main, seed_a - Eigen::Vector3d::Ones(), seed_a + Eigen::Vector3d::Ones(),
            occ_main);
        general_planner::trackingOccupiedVoxels(
            map_track, seed_a - Eigen::Vector3d::Ones(), seed_a + Eigen::Vector3d::Ones(),
            occ_track);
        const double d_main = minDistanceToLine(occ_main, seed_a, seed_b);
        const double d_track = minDistanceToLine(occ_track, seed_a, seed_b);
        std::cout << "seed-line clearance: world map=" << d_main
                  << " m, tracking map=" << d_track << " m\n";
        require(d_main < 0.12,
                "world map (0.45 m inflation) must reproduce the 0.103 m bag clearance");
        require(d_track > 0.20,
                "tracking map (0.15 m inflation) must clear the car-roof ghost");

        require(!general_planner::trackingSeedLineFree(map_main, seed_a, seed_b),
                "world map must reject the bag seed line");
        require(general_planner::trackingSeedLineFree(map_track, seed_a, seed_b),
                "tracking map must accept the bag seed line");

        auto ros_if = std::make_shared<ros_interface::Ros1Interface>(nh);
        const double voxel_radius =
            0.5 * std::sqrt(3.0) * map_track->getInfResolution() + 1.e-3;
        const auto ellipsoid_cfg =
            optimization_utils::EllipsoidOptimizer::makeConfig("classic", false);

        // Strict CIRI on the thick world map: the live failure loop.
        general_planner::CorridorGenerator cg_main(ros_if, map_main, 2., 2., .15,
                                                   -0.10, 6.40, voxel_radius, 2, 2,
                                                   ellipsoid_cfg, true);
        general_utils::Line line{{seed_a.x(), seed_a.y(), seed_a.z()},
                                 {seed_b.x(), seed_b.y(), seed_b.z()}};
        geometry_utils::Polytope poly;
        require(!cg_main.GeneratePolytopeFromLine(line, poly),
                "strict CIRI on the world map must hard-fail (bag regression)");

        // Tracking corridor on the thin map: tight-seed enabled like the
        // runtime construction in general_planner.cpp.
        general_planner::CorridorGenerator cg_track(ros_if, map_track, 2., 2., .15,
                                                    -0.10, 6.40, voxel_radius, 2, 2,
                                                    ellipsoid_cfg, true);
        cg_track.SetAllowTightSeed(true);
        require(cg_track.GeneratePolytopeFromLine(line, poly) &&
                poly.PointIsInside(seed_a, 1.e-6) &&
                poly.PointIsInside(seed_b, 1.e-6),
                "tracking corridor must contain the bag seed line");

        // Full chain on the thin map: frontend ring search for the frozen
        // target, then a corridor along the whole guide path.
        general_planner::TrackingFrontend::Config cfg;
        cfg.tracking_distance = 2.5;
        cfg.distance_tolerance = .8;
        cfg.height_offset = 1.0;
        cfg.height_tolerance = .6;
        cfg.search_budget_seconds = 2.0;
        cfg.unknown_as_occupied = false;
        general_utils::StatePVAJ head = general_utils::StatePVAJ::Zero();
        head.col(0) = seed_a;
        traj_opt::DynamicTargetStates prediction(21);
        for (int i = 0; i < 21; ++i) {
            prediction[i].t = .2 * i;
            prediction[i].position << -32.47, 2.52, -0.05;
            prediction[i].velocity.setZero();
        }
        traj_opt::TrackingProblem problem;
        require(general_planner::TrackingFrontend(cfg, map_track)
                    .buildProblem(head, prediction, problem),
                "frontend must find a guide path over the ghost on the thin map");
        geometry_utils::PolytopeVec sfcs;
        general_utils::Vec3f shifted;
        require(cg_track.SearchPolytopeOnPath(problem.guide_path, sfcs, shifted,
                                              false, true),
                "corridor along the full guide path must succeed on the thin map");
        std::cout << "guide path: " << problem.guide_path.size()
                  << " pts, corridor polytopes: " << sfcs.size() << '\n';

        general_planner::trackingOccupancyExclusion().clear();
        std::cout << "tracking_wall_replay_self_test PASS: world map reproduces the "
                     "0.103 m CIRI hard-fail; thin tracking map frees the seed and the "
                     "full frontend+corridor chain succeeds\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
