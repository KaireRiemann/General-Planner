#include <general_core/tracking/tracking_frontend.hpp>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
}

int main(int argc, char **argv) {
    const char *master = std::getenv("ROS_MASTER_URI");
    if (!master || std::string(master) != "http://127.0.0.1:11319") {
        std::cerr << "requires isolated ROS_MASTER_URI=http://127.0.0.1:11319\n";
        return 2;
    }
    ros::init(argc, argv, "tracking_frontend_map_self_test");
    try {
        ros::NodeHandle nh("~");
        auto grid = std::make_shared<rog_map::ROGMapROS>(nh, std::string(ROOT_DIR) + "tests/gate_unknown_map.yaml");
        auto map = std::make_shared<general_planner::MapManager>(grid);
        general_planner::TrackingFrontend::Config cfg;
        cfg.tracking_distance = 1.5;
        cfg.distance_lower_tolerance = cfg.distance_upper_tolerance = .3;
        cfg.height_offset = .7; cfg.height_tolerance = .2;
        cfg.search_budget_seconds = .5;
        general_utils::StatePVAJ head = general_utils::StatePVAJ::Zero();
        head.col(0) << -2., 0., 1.2;
        traj_opt::DynamicTargetStates prediction(3);
        for (int i = 0; i < 3; ++i) {
            prediction[i].t = .5 * i;
            prediction[i].position << .25 * i, 0., .5;
            prediction[i].velocity << .5, 0., 0.;
        }
        traj_opt::TrackingProblem problem;
        require(!general_planner::TrackingFrontend(cfg, map).buildProblem(head, prediction, problem),
                "unknown space must be rejected when configured occupied");
        cfg.unknown_as_occupied = false;
        rog_map::PointCloud target_body;
        rog_map::PointType target_point;
        target_point.x = -.3; target_point.y = 0.; target_point.z = .6; target_point.intensity = 1.;
        target_body.push_back(target_point);
        grid->updateOccPointCloud(target_body);
        auto stationary = prediction;
        for (auto &sample : stationary) {
            sample.position << 0., 0., .5; sample.velocity.setZero();
        }
        require(general_planner::TrackingFrontend(cfg, map).buildProblem(head, stationary, problem),
                "the target's own occupied body must not invalidate every observation point");
        rog_map::PointCloud wall;
        for (double y = -.6; y <= .6; y += .1)
            for (double z = 0.; z <= 3.; z += .1) {
                rog_map::PointType point;
                point.x = -1.; point.y = y; point.z = z; point.intensity = 1.;
                wall.push_back(point);
            }
        grid->updateOccPointCloud(wall);
        require(!map->isLineFree(head.col(0), prediction.front().position, false, false),
                "test wall must occlude the target");
        require(general_planner::TrackingFrontend(cfg, map).buildProblem(head, prediction, problem),
                "region search must find a visible path around the wall");
        double lateral = 0.;
        for (std::size_t i = 1; i < problem.guide_path.size(); ++i) {
            lateral = std::max(lateral, std::abs(problem.guide_path[i].y()));
            require(map->isLineFree(problem.guide_path[i - 1], problem.guide_path[i], true, false),
                    "obstacle route must respect inflated collision cells");
        }
        require(lateral > .6, "occlusion requires a lateral detour");
        head.col(0) << -1., 0., 1.2;
        require(!general_planner::TrackingFrontend(cfg, map).buildProblem(head, prediction, problem),
                "occupied initial state must fail without a fabricated path");
        std::cout << "tracking_frontend_map_self_test PASS: unknown, occlusion detour, collision segments, blocked head\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
