#include <general_core/general_planner.h>
#include <general_core/state2state/state2state_frontend_services.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

// Deterministic map evidence for testing the real frontend; no sensor timing
// or live ROS topics are involved. Unknown inflation stays disabled, as in Unity.
class FixtureMap : public rog_map::ROGMapROS {
public:
    using ROGMapROS::ROGMapROS;
    void knownFree() {
        std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), cfg_.l_min);
        map_empty_ = false;
    }
    void unknown() {
        std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(),
                  (cfg_.l_free + cfg_.l_occ) / 2.0);
    }
};
}

int main(int argc, char **argv) {
    const char *master = std::getenv("ROS_MASTER_URI");
    if (!master || std::string(master) != "http://127.0.0.1:11386") {
        std::cerr << "requires isolated ROS_MASTER_URI=http://127.0.0.1:11386\n";
        return 2;
    }
    ros::init(argc, argv, "state2state_topology_frontend_test");
    using namespace general_planner;
    using namespace general_planner::state2state_task;
    try {
        ros::NodeHandle nh("~");
        auto grid = std::make_shared<FixtureMap>(nh, std::string(ROOT_DIR) + "tests/gate_unknown_map.yaml");
        grid->knownFree();
        auto map = std::make_shared<MapManager>(grid);
        map->enableIndependentOdometry();
        auto ros = std::make_shared<ros_interface::Ros1Interface>(nh);
        const std::string astar_config = "/tmp/state2state_topology_frontend_astar.yaml";
        {
            std::ofstream out(astar_config);
            out << "astar:\n  map_voxel_num: [61,61,31]\n  allow_diag: true\n  heu_type: 2\n";
        }
        auto astar = std::make_shared<path_search::Astar>(astar_config, ros, map);
        general_planner::Config cfg;
        cfg.resolution = .15;
        cfg.frontend_in_known_free = false;
        cfg.unknown_goal_reveal_en = false;
        cfg.print_log = false;
        cfg.esdf_traj_en = cfg.plain_traj_en = false;
        cfg.corridor_line_max_length = 2.;
        cfg.state2state_topology_enable = true;
        cfg.state2state_topology_query_capability_enable = true;
        cfg.state2state_topology_min_query_distance = 8.;
        cfg.state2state_topology_local_prefix_length = 8.;
        cfg.state2state_topology_local_boundary_margin = .2;
        cfg.state2state_over_goal_guard_enable = true;
        cfg.over_wall_search_en = true;
        const Vec3f start(-2,0,1.5), relay(2,0,1.5), goal(3,0,1.5);
        Vec3f local_start = start;
        bool goal_valid = true;
        State2StateTopologyRouteRuntime runtime;
        runtime.setPolicy(true);
        const auto seed = [&] {
            resetGlobalTopologyRoute(runtime.route, "TEST_ROUTE");
            runtime.consumed_policy_generation = runtime.policy_generation.load();
            runtime.consumed_task_generation = runtime.task_generation.load();
            auto &route = runtime.route;
            route.valid = true;
            route.phase = TopologyRoutePhase::FOLLOW_GRAPH;
            route.goal = goal;
            route.world_epoch = map->worldEpoch();
            route.raw_topology_route = {start, Vec3f(-2,2,1.5), Vec3f(2,2,1.5), relay};
            buildRouteArcLength(route.raw_topology_route, route.arc_length);
        };
        rog_map::RobotState measured;
        measured.rcv = true;
        measured.p = start;
        map->updateOdometrySnapshot(measured);
        StateToStateFrontendServices services{cfg, map, ros, astar, nullptr,
                                              &runtime, local_start, goal, goal_valid};
        seed();
        vec_Vec3f path;
        require(pathSearch(services, start, goal, 8., path), "graph prefix failed");
        require((path.back() - relay).norm() < 1e-6, "local candidate replaced the relay");
        bool passed_corner = false;
        for (const auto &point : path) {
            passed_corner |= point.y() > 1.9;
            // Escape A* snaps its first point to a voxel centre. That small
            // attachment offset is distinct from the >=0.3m over-wall climb.
            require(std::abs(point.z() - 1.5) <= cfg.resolution,
                    "over-wall candidate replaced topology");
        }
        require(passed_corner, "line-of-sight shortcut discarded graph detour");
        require(!pathSearch(services, relay, goal, 8., path) &&
                runtime.route.last_result == "TOPO_WAIT_ANCHOR_ARRIVAL",
                "future planning head released graph before measured arrival");
        measured.p = relay;
        map->updateOdometrySnapshot(measured);
        require(pathSearch(services, relay, goal, 8., path) &&
                runtime.route.phase == TopologyRoutePhase::LOCAL_GOAL &&
                (path.back() - goal).norm() < 1e-6, "measured arrival did not enable final segment");
        require(pathSearch(services, Vec3f(2.5,0,1.5), goal, 8., path) &&
                runtime.route.phase == TopologyRoutePhase::LOCAL_GOAL,
                "final segment reattached to the old relay");

        seed();
        measured.p = start;
        map->updateOdometrySnapshot(measured);
        grid->unknown();
        require(!pathSearch(services, start, goal, 8., path),
                "invalid graph prefix silently fell back to unknown-space direct flight");
        runtime.setPolicy(false);
        require(pathSearch(services, start, goal, 8., path) &&
                (path.back() - goal).norm() < 1e-6, "local-only navigation regressed");
        std::cout << "state2state_topology_frontend_test passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
