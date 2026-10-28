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
        require(!runtime.policy_enabled.load(), "fresh runtime must default to local-only");
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
        vec_Vec3f path;
        // Capability and graph availability must not opt a fresh task into
        // topology navigation. A disabled policy must also discard old routes.
        require(pathSearch(services, start, goal, 8., path) &&
                (path.back() - goal).norm() < 1e-6 &&
                runtime.route.route_id == 0 && !runtime.route.valid,
                "default local navigation attempted a topology query");
        runtime.setPolicy(false);
        seed();
        require(pathSearch(services, start, goal, 8., path) &&
                (path.back() - goal).norm() < 1e-6,
                "disabled policy followed a cached topology relay");
        for (const auto &point : path)
            require(std::abs(point.y()) < cfg.resolution,
                    "disabled policy retained the topology dogleg");
        runtime.setPolicy(true);
        seed();
        runtime.setPolicy(false);
        require(pathSearch(services, start, goal, 8., path) &&
                !runtime.route.valid && runtime.route.raw_topology_route.empty() &&
                runtime.consumed_policy_generation == runtime.policy_generation.load() &&
                (path.back() - goal).norm() < 1e-6,
                "disable did not invalidate and consume the previous route");
        runtime.setPolicy(true);
        seed();
        require(pathSearch(services, start, goal, 8., path), "graph prefix failed");
        require((path.front() - start).norm() < 1.e-9,
                "strict topology changed the continuous start position");
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
                runtime.route.last_result == "TOPO_WAIT_ANCHOR_ARRIVAL" && services.waiting_for_topology,
                "future planning head released graph before measured arrival");
        runtime.wait_started -= cfg.state2state_topology_wait_timeout + 1.0;
        require(!pathSearch(services, relay, goal, 8., path) && !services.waiting_for_topology,
                "topology wait did not become a bounded failure");
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
        require(path.empty() && runtime.frontendFailure() == "TOPO_START_UNKNOWN" &&
                !services.waiting_for_topology,
                "unknown start must report a bounded failure without stale output");
        grid->knownFree();
        seed();
        auto inflation = grid->inflationMap();
        inflation->updateGridCounter(start, rog_map::UNKNOWN, rog_map::OCCUPIED);
        require(map->getGridType(start) == rog_map::KNOWN_FREE &&
                map->getInfGridType(start) == rog_map::OCCUPIED,
                "inflated-only start fixture is invalid");
        require(!pathSearch(services, start, goal, 8., path) && path.empty() &&
                runtime.frontendFailure() == "TOPO_START_INFLATED_OCCUPIED",
                "inflated start was projected or misreported as an attachment failure");
        inflation->updateGridCounter(start, rog_map::OCCUPIED, rog_map::UNKNOWN);
        seed();
        require(pathSearch(services, start, goal, 8., path) && runtime.frontendFailure().empty(),
                "valid planning after obstacle removal retained a failure");
        require(!pathSearch(services, Vec3f(100,0,1.5), goal, 8., path) &&
                runtime.frontendFailure() == "TOPO_START_OUT_OF_MAP",
                "out-of-map start did not fail before graph search");
        // Capability switches must also gate graph search even if a caller
        // has retained an enabled runtime selection.
        measured.p = start;
        map->updateOdometrySnapshot(measured);
        seed();
        cfg.state2state_topology_query_capability_enable = false;
        require(pathSearch(services, start, goal, 8., path) &&
                (path.back() - goal).norm() < 1e-6,
                "disabled query capability followed a topology relay");
        cfg.state2state_topology_query_capability_enable = true;
        cfg.state2state_topology_enable = false;
        require(pathSearch(services, start, goal, 8., path) &&
                (path.back() - goal).norm() < 1e-6,
                "disabled topology capability followed a topology relay");
        cfg.state2state_topology_enable = true;
        grid->unknown();
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
