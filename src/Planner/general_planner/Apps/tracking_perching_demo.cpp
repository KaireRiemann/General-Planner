#include <cmath>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rog_map/rog_map.h"
#include "rog_map_ros/rog_map_ros1.hpp"
#include "path_search/astar.h"
#include "ros_interface/ros1/ros1_interface.hpp"
#include "general_core/tracking/tracking_perching_frontend.hpp"
#include "traj_opt/config.hpp"
#include "traj_opt/traj_manager.h"
#include "traj_opt/tracking_perching_traj_opt.hpp"
#include "utils/header/color_msg_utils.hpp"

#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/" + name))

namespace
{

using geometry_utils::Trajectory;
using general_utils::StatePVAJ;
using general_utils::Vec3f;

traj_opt::DynamicTargetStates makeTrackingTargetPrediction()
{
    traj_opt::DynamicTargetStates targets;
    constexpr int sample_num = 16;
    constexpr double dt = 0.25;
    for (int i = 0; i < sample_num; ++i)
    {
        const double t = static_cast<double>(i) * dt;
        traj_opt::DynamicTargetState target;
        target.t = t;
        target.position = Vec3f(2.0 + 0.45 * t,
                                1.0 + 0.8 * std::sin(0.7 * t),
                                0.8);
        target.velocity = Vec3f(0.45,
                                0.56 * std::cos(0.7 * t),
                                0.0);
        target.yaw = std::atan2(target.velocity.y(), target.velocity.x());
        targets.emplace_back(target);
    }
    return targets;
}

std::vector<double> allocatePathTime(const general_utils::vec_E<Vec3f> &path, double speed)
{
    std::vector<double> times(path.size(), 0.0);
    speed = std::max(0.2, speed);
    for (int i = 1; i < static_cast<int>(path.size()); ++i)
    {
        times[static_cast<std::size_t>(i)] =
            times[static_cast<std::size_t>(i - 1)] +
            std::max(0.12, (path[i] - path[i - 1]).norm() / speed);
    }
    return times;
}

Vec3f interpolatePath(const general_utils::vec_E<Vec3f> &path,
                      const std::vector<double> &times,
                      double t,
                      Vec3f &velocity)
{
    velocity.setZero();
    if (path.empty())
    {
        return Vec3f::Zero();
    }
    if (path.size() == 1 || t <= times.front())
    {
        return path.front();
    }
    if (t >= times.back())
    {
        return path.back();
    }
    const auto upper = std::lower_bound(times.begin(), times.end(), t);
    const int idx = static_cast<int>(std::distance(times.begin(), upper));
    const double t0 = times[static_cast<std::size_t>(idx - 1)];
    const double t1 = times[static_cast<std::size_t>(idx)];
    const double alpha = (t - t0) / std::max(1.0e-6, t1 - t0);
    velocity = (path[static_cast<std::size_t>(idx)] - path[static_cast<std::size_t>(idx - 1)]) /
               std::max(1.0e-6, t1 - t0);
    return path[static_cast<std::size_t>(idx - 1)] +
           alpha * (path[static_cast<std::size_t>(idx)] - path[static_cast<std::size_t>(idx - 1)]);
}

traj_opt::DynamicTargetStates sampleTargetFromPath(const general_utils::vec_E<Vec3f> &path,
                                                   double speed,
                                                   double dt)
{
    traj_opt::DynamicTargetStates targets;
    if (path.size() < 2)
    {
        return targets;
    }
    const auto times = allocatePathTime(path, speed);
    const double total_t = times.back();
    const int sample_num = std::max(2, static_cast<int>(std::ceil(total_t / dt)) + 1);
    for (int i = 0; i < sample_num; ++i)
    {
        const double t = std::min(total_t, static_cast<double>(i) * dt);
        traj_opt::DynamicTargetState target;
        target.t = t;
        target.position = interpolatePath(path, times, t, target.velocity);
        target.yaw = std::atan2(target.velocity.y(), target.velocity.x());
        targets.emplace_back(target);
    }
    return targets;
}

traj_opt::DynamicTargetStates buildStateToStateTargetPrediction(
    const traj_opt::Config &cfg,
    const ros_interface::RosInterface::Ptr &ros_ptr,
    const general_planner::MapManager::Ptr &map_manager,
    const path_search::Astar::Ptr &astar,
    const std::string &target_planner,
    Trajectory &target_traj,
    general_utils::vec_E<Vec3f> &target_guide)
{
    const Vec3f target_start(-2.2, -2.3, 1.25);
    const Vec3f target_goal(4.2, 2.15, 1.25);
    target_guide.clear();
    target_guide.emplace_back(target_start);

    if (astar != nullptr && map_manager != nullptr && map_manager->ready())
    {
        auto tryAstar = [&](const int flag, const std::string &label) {
            general_utils::vec_E<Vec3f> astar_path;
            const auto ret =
                astar->pointToPointPathSearch(target_start, target_goal, flag, 9.0, astar_path, 0.2);
            const auto ret_str = (ret >= 0 && static_cast<std::size_t>(ret) < general_utils::RET_CODE_STR.size())
                                     ? general_utils::RET_CODE_STR[static_cast<std::size_t>(ret)]
                                     : std::to_string(ret);
            if ((ret == general_utils::SUCCESS || ret == general_utils::REACH_GOAL) && astar_path.size() >= 2) {
                target_guide = astar_path;
                ROS_INFO("Target state-to-state %s A* guide: %zu waypoints, ret=%s.",
                         label.c_str(),
                         target_guide.size(),
                         ret_str.c_str());
                return true;
            }
            ROS_WARN("Target state-to-state %s A* failed with ret=%s.", label.c_str(), ret_str.c_str());
            return false;
        };

        const int prob_flag = path_search::ON_PROB_MAP |
                              path_search::UNKNOWN_AS_FREE |
                              path_search::DONT_USE_INF_NEIGHBOR;
        const int inf_flag = path_search::ON_INF_MAP |
                             path_search::UNKNOWN_AS_FREE |
                             path_search::USE_INF_NEIGHBOR;
        if (target_planner == "inflated_astar") {
            tryAstar(inf_flag, "inflated-map") || tryAstar(prob_flag, "prob-map");
        }
        else {
            tryAstar(prob_flag, "prob-map") || tryAstar(inf_flag, "inflated-map");
        }
    }

    if (target_guide.size() < 2)
    {
        target_guide.clear();
        target_guide.emplace_back(target_start);
        target_guide.emplace_back(Vec3f(-1.6, 0.25, 1.25));
        target_guide.emplace_back(Vec3f(0.55, 1.95, 1.25));
        target_guide.emplace_back(Vec3f(2.4, 0.25, 1.25));
        target_guide.emplace_back(target_goal);
        ROS_WARN("Target state-to-state planner unavailable, use deterministic waypoint guide.");
    }

    if (target_planner == "esdf" && map_manager != nullptr && map_manager->hasESDF())
    {
        StatePVAJ head = StatePVAJ::Zero();
        StatePVAJ tail = StatePVAJ::Zero();
        head.col(0) = target_guide.front();
        tail.col(0) = target_guide.back();
        traj_opt::ESDFTrajOpt target_opt(cfg, ros_ptr);
        target_opt.setMapManager(map_manager);
        target_opt.setSafeDistance(0.25);
        target_opt.setWeight(std::max(10.0, cfg.penna_pos));
        const auto guide_t = allocatePathTime(target_guide, 1.4);
        if (target_opt.optimize(head, tail, target_guide, guide_t, target_traj))
        {
            traj_opt::DynamicTargetStates targets;
            const double total_t = target_traj.getTotalDuration();
            const double dt = 0.25;
            const int sample_num = std::max(2, static_cast<int>(std::ceil(total_t / dt)) + 1);
            for (int i = 0; i < sample_num; ++i)
            {
                const double t = std::min(total_t, static_cast<double>(i) * dt);
                traj_opt::DynamicTargetState target;
                target.t = t;
                target.position = target_traj.getPos(t);
                target.velocity = target_traj.getVel(t);
                target.acceleration = target_traj.getAcc(t);
                target.yaw = std::atan2(target.velocity.y(), target.velocity.x());
                targets.emplace_back(target);
            }
            return targets;
        }
        ROS_WARN("Target ESDF state-to-state optimization failed, use A*/path timed target prediction.");
    }

    return sampleTargetFromPath(target_guide, 1.3, 0.25);
}

traj_opt::PerchingSurfaceState makePerchingSurface()
{
    traj_opt::PerchingSurfaceState surface;
    surface.t = 2.5;
    surface.position = Vec3f(3.6, 0.4, 0.95);
    surface.velocity = Vec3f(0.15, 0.0, 0.0);
    surface.surface_x = Vec3f::UnitX();
    surface.surface_y = Vec3f::UnitY();
    surface.surface_z = Vec3f::UnitZ();
    surface.yaw = 0.0;
    return surface;
}

StatePVAJ makeHeadState(const Vec3f &p, const Vec3f &v = Vec3f::Zero())
{
    StatePVAJ state = StatePVAJ::Zero();
    state.col(0) = p;
    state.col(1) = v;
    return state;
}

void visualizeRepeated(const ros_interface::RosInterface::Ptr &ros_ptr,
                       const Trajectory &traj,
                       const general_utils::vec_E<Vec3f> &guide,
                       const std::string &ns)
{
    ros::Rate rate(2.0);
    for (int i = 0; ros::ok() && i < 8; ++i)
    {
        ros_ptr->vizFrontendPath(guide);
        ros_ptr->vizExpTraj(traj, ns);
        rate.sleep();
    }
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_perching_demo");
    ros::NodeHandle nh("~");

    std::string cfg_path;
    std::string cfg_name;
    std::string mode;
    nh.param<std::string>("config_name", cfg_name, "click_esdf_ros1.yaml");
    nh.param<std::string>("config_path", cfg_path, CONFIG_FILE_DIR(cfg_name));
    nh.param<std::string>("mode", mode, "tracking");

    auto ros_ptr = std::make_shared<ros_interface::Ros1Interface>(nh);
    ros_ptr->setVisualizationEn(true);
    ros_ptr->setResolution(0.1);

    traj_opt::Config cfg(cfg_path, "esdf_traj");
    cfg.penna_omg = 0.0;
    cfg.penna_thr = 0.0;
    cfg.penna_t = std::max(1.0, cfg.penna_t * 1.0e-3);
    cfg.penna_vel = std::max(1.0, cfg.penna_vel * 1.0e-3);
    cfg.penna_acc = std::max(1.0, cfg.penna_acc * 1.0e-3);
    cfg.penna_jerk = std::max(1.0, std::abs(cfg.penna_jerk) * 1.0e-3);
    cfg.penna_pos = std::max(50.0, cfg.penna_pos * 1.0e-4);
    cfg.integral_reso = std::max(8, cfg.integral_reso);

    bool use_map = true;
    bool tracker_frontend_astar = false;
    std::string target_planner;
    nh.param<bool>("use_map", use_map, true);
    nh.param<bool>("tracker_frontend_astar", tracker_frontend_astar, false);
    nh.param<std::string>("target_planner", target_planner, "esdf");

    general_planner::MapManager::Ptr map_manager;
    path_search::Astar::Ptr astar;
    rog_map::ROGMapROS::Ptr rog_map;
    if (use_map)
    {
        rog_map = std::make_shared<rog_map::ROGMapROS>(nh, cfg_path);
        map_manager = std::make_shared<general_planner::MapManager>(rog_map);
        astar = std::make_shared<path_search::Astar>(cfg_path, ros_ptr, map_manager);
    }

    bool ok = true;
    if (mode == "tracking" || mode == "both")
    {
        general_planner::TrackingFrontend::Config frontend_cfg;
        frontend_cfg.tracking_distance = 2.2;
        frontend_cfg.height_offset = 0.7;
        frontend_cfg.safe_distance = use_map ? 0.35 : 0.0;
        frontend_cfg.visibility_safe_distance = use_map ? 0.25 : 0.0;
        frontend_cfg.unknown_as_occupied = false;
        frontend_cfg.use_astar = tracker_frontend_astar;
        general_planner::TrackingFrontend frontend(frontend_cfg, map_manager, astar);

        traj_opt::TrackingProblem problem;
        const StatePVAJ head = makeHeadState(Vec3f(0.0, -1.8, 1.5));
        Trajectory target_traj;
        general_utils::vec_E<Vec3f> target_guide;
        const auto target_prediction =
            use_map ? buildStateToStateTargetPrediction(cfg,
                                                        ros_ptr,
                                                        map_manager,
                                                        astar,
                                                        target_planner,
                                                        target_traj,
                                                        target_guide)
                    : makeTrackingTargetPrediction();

        if (!target_traj.empty())
        {
            ros_ptr->vizExpTraj(target_traj, "target_state_to_state");
        }
        if (!frontend.buildProblem(head, target_prediction, problem))
        {
            ROS_ERROR("Tracking frontend failed.");
            ok = false;
        }
        else
        {
            traj_opt::TrackingJerkTrajOpt optimizer(cfg, ros_ptr);
            optimizer.setMapManager(map_manager);
            optimizer.setSafeDistance(frontend_cfg.safe_distance);
            Trajectory traj;
            if (!optimizer.optimize(problem, traj))
            {
                ROS_ERROR("Tracking jerk optimization failed.");
                ok = false;
            }
            else
            {
                ROS_INFO("Tracking demo trajectory: pieces=%d duration=%.3f",
                         traj.getPieceNum(),
                         traj.getTotalDuration());
                visualizeRepeated(ros_ptr, traj, problem.guide_path, "tracking_jerk_demo");
            }
        }
    }

    if (mode == "perching" || mode == "both")
    {
        general_planner::PerchingFrontend::Config frontend_cfg;
        frontend_cfg.robot_l = 0.28;
        frontend_cfg.v_plus = 0.8;
        frontend_cfg.pre_contact_distance = 0.55;
        frontend_cfg.safe_distance = 0.0;
        general_planner::PerchingFrontend frontend(frontend_cfg, map_manager, astar);

        traj_opt::PerchingProblem problem;
        const StatePVAJ head = makeHeadState(Vec3f(0.0, -2.6, 1.4), Vec3f(0.4, 0.0, 0.0));
        if (!frontend.buildProblem(head, makePerchingSurface(), problem))
        {
            ROS_ERROR("Perching frontend failed.");
            ok = false;
        }
        else
        {
            traj_opt::PerchingSnapTrajOpt optimizer(cfg, ros_ptr);
            optimizer.setMapManager(map_manager);
            optimizer.setSafeDistance(0.0);
            Trajectory traj;
            if (!optimizer.optimize(problem, traj))
            {
                ROS_ERROR("Perching snap optimization failed.");
                ok = false;
            }
            else
            {
                ROS_INFO("Perching demo trajectory: pieces=%d duration=%.3f",
                         traj.getPieceNum(),
                         traj.getTotalDuration());
                visualizeRepeated(ros_ptr, traj, problem.guide_path, "perching_snap_demo");
            }
        }
    }

    return ok ? 0 : 1;
}
