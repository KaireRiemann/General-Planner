#include <general_core/tracking/tracking_frontend.hpp>
#include <general_core/general_planner.h>
#include <general_core/tracking/tracking_map_query.hpp>
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
        // Empty space near the virtual floor/ceiling must have one occupancy
        // meaning for corridor, segment and brake queries (bag regression).
        for (double z : {.225,.375,.525,.825,1.125,2.325,2.475,2.625}) {
            const Eigen::Vector3d point(2.,2.,z);
            require(map->isOccupiedInflate(point) ==
                    (map->getInfGridType(point) == rog_map::OCCUPIED),
                    "inflated virtual bounds differ between map query APIs");
        }
        require(map->getInfGridType(Eigen::Vector3d(2.,2.,.825)) != rog_map::OCCUPIED,
                "virtual floor must not be inflated twice");
        general_planner::trackingOccupancyExclusion().clear();
        general_planner::TrackingFrontend::Config cfg;
        cfg.tracking_distance = 1.5;
        cfg.distance_tolerance = .3;
        cfg.unknown_as_occupied = true;
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
        rog_map::PointCloud target_high;
        target_point.x = 0.; target_point.y = 0.; target_point.z = 1.2; target_point.intensity = 1.;
        target_high.push_back(target_point);
        grid->updateOccPointCloud(target_high);
        general_planner::setTrackingOccupancyExclusion(Eigen::Vector3d(0., 0., 1.2), .8, .8);
        require(!general_planner::trackingInflatedOccupied(map, Eigen::Vector3d(0., 0., 1.2)),
                "tracking occupancy query must ignore the target body");
        require(general_planner::trackingSeedLineFree(map, Eigen::Vector3d(-1.5, 0., 1.2),
                                                      Eigen::Vector3d(.5, 0., 1.2)),
                "tracking CIRI seed through the target body must be free");
        general_utils::vec_Vec3f occupied_body;
        general_planner::trackingOccupiedVoxels(map, Eigen::Vector3d(-1., -1., .6),
                                               Eigen::Vector3d(1., 1., 1.8), occupied_body);
        for (const auto &point : occupied_body) {
            require(!general_planner::trackingOccupancyExclusion().contains(point),
                    "CIRI obstacle cloud still contains the tracked body");
        }
        general_planner::CorridorGenerator body_corridor(std::make_shared<ros_interface::Ros1Interface>(nh),
            map, 1., 2., .02, 0., 3.,
            .5 * std::sqrt(3.) * map->getInfResolution() + 1.e-3, 1, 2,
            optimization_utils::EllipsoidOptimizerConfig(), true);
        geometry_utils::Polytope body_poly;
        general_utils::Line body_line{{-1.5, 0., 1.2}, {.5, 0., 1.2}};
        require(body_corridor.GeneratePolytopeFromLine(body_line, body_poly) &&
                body_poly.PointIsInside(Eigen::Vector3d(0., 0., 1.2), 1.e-3),
                "tracking CIRI must still grow a corridor through the excluded body");
        general_planner::trackingOccupancyExclusion().clear();
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
        // Runtime tracking always masks the target body. Without that mask the
        // ring center itself is occupied and Elastic-style visibility search
        // correctly reports no observation ray.
        general_planner::setTrackingOccupancyExclusion(prediction.front().position, .8, .8);
        require(general_planner::TrackingFrontend(cfg, map).buildProblem(head, prediction, problem),
                "region search must find a visible path around the wall");
        double lateral = 0.;
        for (std::size_t i = 1; i < problem.guide_path.size(); ++i) {
            lateral = std::max(lateral, std::abs(problem.guide_path[i].y()));
            require(general_planner::trackingSeedLineFree(map, problem.guide_path[i - 1],
                                                          problem.guide_path[i]),
                    "obstacle route must respect inflated collision cells");
        }
        require(lateral > .6, "occlusion requires a lateral detour");
        // A direct chord excludes the apex of this curved guide. Tracking
        // must retain every guide point while simplifying collision-free seeds.
        auto corridor_ros = std::make_shared<ros_interface::Ros1Interface>(nh);
        general_planner::CorridorGenerator corridor(corridor_ros,map,.25,5.,.02,0.,3.,.05,1,1);
        geometry_utils::PolytopeVec sfcs;
        general_utils::Vec3f shifted;
        general_utils::vec_E<general_utils::Vec3f> curve{{-2.,2.,1.5},{0.,3.,1.5},{2.,2.,1.5}};
        require(corridor.SearchPolytopeOnPath(curve,sfcs,shifted,false,true),"curved guide corridor");
        for (const auto &point : curve) {
            bool covered=false;
            for (const auto &poly : sfcs) covered=covered || poly.PointIsInside(point,1.e-4);
            require(covered,"shortcut corridor dropped a guide point");
        }
        // The tracking corridor encloses free configuration space, including
        // occupied voxel extents, instead of raw obstacles plus a smaller radius.
        const double voxel_radius=.5*std::sqrt(3.)*map->getInfResolution()+.001;
        general_planner::CorridorGenerator inflated(corridor_ros,map,1.,2.,.02,0.,3.,
            voxel_radius,1,2,optimization_utils::EllipsoidOptimizerConfig(),true);
        geometry_utils::Polytope corridor_poly;
        general_utils::Line line{{-2.5,1.5,1.2},{-1.5,1.5,1.2}};
        require(inflated.GeneratePolytopeFromLine(line,corridor_poly),"inflated tracking corridor construction");
        for (double x=-3.;x<0.;x+=.075) for(double y=.5;y<2.;y+=.075)
            for(double z=.6;z<1.8;z+=.15) {
                const Eigen::Vector3d point(x,y,z);
                require(!corridor_poly.PointIsInside(point,-1.e-4) ||
                    map->getInfGridType(point)!=rog_map::OCCUPIED,
                    "optimized corridor permits a cell rejected by final safety checking");
            }
        // Latest bag stopped at z=0.427844: free in the inflated map,
        // below the old raw-floor + voxel-radius corridor boundary (0.430904).
        // Both point and climb-line construction must accept that safe head.
        double floor, ceiling;
        map->getInflatedVirtualHeightBounds(floor, ceiling);
        general_utils::Line low_line{{-2.5,2.5,floor+.052844},{-1.5,2.5,1.2}};
        require(map->getInfGridType(low_line.first)!=rog_map::OCCUPIED,
                "low-altitude regression head must be free");
        require(general_planner::trackingSeedLineFree(map,low_line.first,low_line.second),
                "virtual floor voxels must not exclude a safe climbing seed");
        require(inflated.GeneratePolytopeFromLine(low_line,corridor_poly) &&
                corridor_poly.PointIsInside(low_line.first,1.e-6) &&
                corridor_poly.PointIsInside(low_line.second,1.e-6),
                "safe low-altitude head excluded from climb corridor");
        require(!corridor_poly.PointIsInside(Eigen::Vector3d(-2.5,2.5,floor-.001),0.),
                "tracking corridor crosses virtual floor");
        require(inflated.GeneratePolytopeFromPoint(low_line.first,corridor_poly) &&
                corridor_poly.PointIsInside(low_line.first,1.e-6),
                "safe low-altitude head excluded from point corridor");
        general_utils::Line high_line{{-2.5,2.5,ceiling-.02},{-1.5,2.5,1.2}};
        require(inflated.GeneratePolytopeFromLine(high_line,corridor_poly) &&
                corridor_poly.PointIsInside(high_line.first,1.e-6) &&
                !corridor_poly.PointIsInside(Eigen::Vector3d(-2.5,2.5,ceiling+.001),0.),
                "inflated ceiling must also be shared by corridor and safety checks");

        // Tight-seed regression (live CIRI loop, consecutive_failures>2000):
        // the ring search accepts zero-clearance free cells, so a free seed
        // line can pass within the voxel-corner radius of an inflated voxel.
        // Strict CIRI hard-fails there; tracking degrades to a zero-radius
        // tangent plane and still returns a valid polytope containing the seed.
        {
            Eigen::Matrix<double,6,4> bd=Eigen::Matrix<double,6,4>::Zero();
            bd(0,0)=1.; bd(1,0)=-1.; bd(2,1)=1.; bd(3,1)=-1.; bd(4,2)=1.; bd(5,2)=-1.;
            bd(0,3)=-2.; bd(1,3)=-2.; bd(2,3)=-2.; bd(3,3)=-2.; bd(4,3)=-2.; bd(5,3)=-2.;
            Eigen::Matrix<double,3,1> tight_pc;
            tight_pc.col(0)=Eigen::Vector3d(0.,0.103,0.5);
            const Eigen::Vector3d seed_a(-1.,0.,0.5), seed_b(1.,0.,0.5);
            general_planner::CIRI strict_ciri(corridor_ros);
            strict_ciri.setupParams(voxel_radius,2);
            require(strict_ciri.comvexDecomposition(bd,tight_pc,seed_a,seed_b)!=general_utils::SUCCESS,
                    "test setup: strict CIRI must reject the tight seed");
            general_planner::CIRI tight_ciri(corridor_ros);
            tight_ciri.setupParams(voxel_radius,2);
            tight_ciri.setAllowTightSeed(true);
            require(tight_ciri.comvexDecomposition(bd,tight_pc,seed_a,seed_b)==general_utils::SUCCESS,
                    "tight-seed tracking CIRI must not hard-fail");
            geometry_utils::Polytope tight_poly;
            tight_ciri.getPolytope(tight_poly);
            require(tight_poly.PointIsInside(seed_a,1.e-6) &&
                    tight_poly.PointIsInside(seed_b,1.e-6),
                    "tight-seed corridor must still contain the seed line");
        }

        head.col(0) << -1., 0., 1.2;
        require(!general_planner::TrackingFrontend(cfg, map).buildProblem(head, prediction, problem),
                "occupied initial state must fail without a fabricated path");

        // Exercise the real constructor -> task services -> optimizer -> commit
        // governor -> HOLD chain. Testing only the frontend/runtime manager did
        // not catch incompatible GeneralPlanner layouts in stale object files.
        auto ros_interface = std::make_shared<ros_interface::Ros1Interface>(nh);
        general_planner::GeneralPlanner planner(
            std::string(ROOT_DIR) + "config/task_planner_runtime_state2state.yaml",
            ros_interface, map,
            std::string(ROOT_DIR) + "config/task_planner_runtime_tracking.yaml");
        auto services = planner.makeTrackingTaskServices();
        services.robot_state.rcv = true;
        services.robot_state.p = head.col(0);
        services.robot_state.v.setZero();
        services.robot_state.a.setZero();
        services.robot_state.yaw = 0.0;
        for (auto &sample : prediction) sample.reference_time = ros::Time::now().toSec();
        require(planner.PlanTrackingFromRest(prediction, true) == general_utils::FAILED,
                "an occupied initial state must fail without crashing");
        require(planner.getCommittedPositionTrajectory().empty(),
                "an occupied initial state must not create an unsafe HOLD");

        // Successful end-to-end planning uses this map and the real CIRI
        // generator. Replan replaces the previous polynomial (Elastic publish).
        services.robot_state.p << -3., 2., 1.5;
        traj_opt::DynamicTargetStates moving(21);
        auto update_target = [&](double elapsed) {
            const double epoch = ros::Time::now().toSec();
            for (int i=0;i<21;++i) {
                moving[i].t=.2*i; moving[i].reference_time=epoch;
                moving[i].position << .25*(elapsed+moving[i].t),2.,.5;
                moving[i].velocity << .25,0.,0.;
            }
        };
        update_target(0.);
        const auto initial_ret=planner.PlanTrackingFromRest(moving,true);
        if(initial_ret!=general_utils::SUCCESS) {
            const auto d=planner.getLatestTrackingDiagnosticSnapshot();
            throw std::runtime_error("CIRI tracking commit failed: "+d.phase+" "+d.reason);
        }
        const auto initial=planner.getCommittedPositionTrajectory();
        require(!initial.empty(),"successful solve did not publish a command");
        ros::WallDuration(.4).sleep();
        const double elapsed=ros::Time::now().toSec()-initial.start_WT;
        services.robot_state.p=initial.getPos(std::max(0.,elapsed));
        services.robot_state.v=initial.getVel(std::max(0.,elapsed));
        services.robot_state.a=initial.getAcc(std::max(0.,elapsed));
        update_target(.4);
        require(planner.ReplanTrackingOnce(moving,false)==general_utils::SUCCESS,
                "CIRI tracking replan did not commit");
        const auto replanned=planner.getCommittedPositionTrajectory();
        require(!replanned.empty(),"replan removed issued command");
        require(replanned.start_WT > initial.start_WT - 1.e-6,
                "replan must stamp a new Elastic-style command rather than splice");

        // Elastic-style yaw: the planner commits a constant scalar heading
        // setpoint; the command sampler (traj_server role) slews toward it.
        const auto committed_yaw=planner.getCommittedYawTrajectory();
        require(committed_yaw.getPieceNum()==1 &&
                std::abs(committed_yaw.getMaxVelRate())<1.0e-6,
                "tracking yaw must be a constant scalar setpoint, not a shaped polynomial");
        require(planner.trackingYawServoActive(),
                "tracking commit must arm the command-side yaw servo");
        // Swing the target sideways so the new setpoint differs from the
        // settled heading; the command yaw must walk there at the configured
        // 2 rad/s instead of jumping.
        traj_opt::DynamicTargetStates side=moving;
        for(auto &s:side){s.position.y()=3.0;s.reference_time=ros::Time::now().toSec();}
        const auto side_ret=planner.ReplanTrackingOnce(side,false);
        if(side_ret!=general_utils::SUCCESS) {
            const auto d=planner.getLatestTrackingDiagnosticSnapshot();
            throw std::runtime_error("sideways-target replan did not commit: "+d.phase+" "+d.reason);
        }
        general_utils::StatePVAJ servo_pvaj;
        double yaw_a,yawd_a; bool servo_backup,servo_finish;
        planner.getOneCommandFromTraj(servo_pvaj,yaw_a,yawd_a,servo_backup,servo_finish);
        require(std::abs(yaw_a)<0.05,
                "yaw servo must start from the measured heading, not the setpoint");
        ros::WallDuration(0.05).sleep();
        double yaw_b,yawd_b;
        planner.getOneCommandFromTraj(servo_pvaj,yaw_b,yawd_b,servo_backup,servo_finish);
        require(yaw_b>yaw_a+0.02 && yaw_b<yaw_a+0.15,
                "yaw servo must slew toward the target bearing at the configured rate");
        require(std::abs(yawd_b)<=2.0+1.0e-6,
                "yaw servo rate exceeds the configured limit");
        require(std::abs(planner.getCommittedYawTrajectory().getPos(0.).x()-
                         std::atan2(3.0-servo_pvaj(1,0),side.front().position.x()-servo_pvaj(0,0)))<0.2,
                "committed yaw setpoint must face the target");

        services.robot_state.p << -2.5, 2.0, 1.2;
        const double epoch = ros::Time::now().toSec();
        for (std::size_t i = 0; i < prediction.size(); ++i) {
            prediction[i].t = 0.1 * i;
            prediction[i].reference_time = epoch;
        }
        require(planner.PlanTrackingFromRest(prediction, true) == general_utils::FAILED,
                "insufficient prediction at a from-rest start must fail without fabricating a hold");
        require(planner.getCommittedPositionTrajectory().empty(),
                "failed first plan must not publish a hover or brake command");
        services.robot_state.p << -2.5,2.5,1.2;
        for (auto &sample : prediction) sample.reference_time = ros::Time::now().toSec();
        require(planner.PlanTrackingFromRest(prediction,true) == general_utils::FAILED,
                "reacquisition without a usable prediction must fail without a hold");
        require(planner.getCommittedPositionTrajectory().empty(),
                "from-rest failure must leave the command queue empty");
        planner.invalidateTrackingCommand("test_emergency_stop");
        // Match ROS publication's outer lock across the position/yaw snapshot.
        planner.lockCommittedTraj();
        const auto retired_position = planner.getCommittedPositionTrajectory();
        const auto retired_yaw = planner.getCommittedYawTrajectory();
        planner.unlockCommittedTraj();
        require(retired_position.empty() && retired_yaw.empty() &&
                !planner.getLatestTrackingDiagnosticSnapshot().has_committed_tracking,
                "emergency stop did not retire the executable trajectory");
        std::cout << "tracking_frontend_map_self_test PASS: unknown, occlusion detour, collision segments, CIRI commit/replan, constant yaw setpoint + command-side slew servo, blocked head, from-rest fail without hold\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
