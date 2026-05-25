#include "exploration/epic_exploration_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <boost/filesystem.hpp>
#include <frontier_manager/frontier_manager.h>
#include <lidar_map/lidar_map.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <nav_msgs/Odometry.h>
#include <path_searching/bubble_astar.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_topo/graph.h>
#include <pointcloud_topo/graph_visualizer.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <unistd.h>

namespace general_planner {
namespace exploration {

namespace {

template <typename T>
void setParamDefault(ros::NodeHandle &nh,
                     const std::string &key,
                     const T &value) {
    if (!nh.hasParam(key)) {
        nh.setParam(key, value);
    }
}

double yawFromQuaternion(const Eigen::Quaterniond &q_in) {
    const Eigen::Quaterniond q = q_in.normalized();
    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
}

template <typename Vec>
std::vector<double> vecToStd(const Vec &v) {
    return {static_cast<double>(v.x()),
            static_cast<double>(v.y()),
            static_cast<double>(v.z())};
}

bool isLocalGuideStartUnsafe(const std::string &reason) {
    return reason.find("local guide start is unsafe") != std::string::npos;
}

}  // namespace

EpicExplorationManager::EpicExplorationManager(Config cfg,
                                               MapManager::Ptr map_manager,
                                               path_search::Astar::Ptr astar,
                                               ros_interface::RosInterface::Ptr ros_ptr)
        : cfg_(std::move(cfg)),
          map_manager_(std::move(map_manager)),
          astar_(std::move(astar)),
          ros_ptr_(std::move(ros_ptr)),
          nh_("~") {
    seedEpicParams();
    initializeNativeModules();
}

void EpicExplorationManager::seedEpicParams() {
    const auto &obs = cfg_.observation_map;
    const auto &lidar = cfg_.lidar_perception;
    const auto &vp = cfg_.viewpoint_manager;
    const auto &topo = cfg_.topo_graph;

    super_utils::Vec3f bbox_min(obs.bbox_min_x, obs.bbox_min_y, obs.bbox_min_z);
    super_utils::Vec3f bbox_max(obs.bbox_max_x, obs.bbox_max_y, obs.bbox_max_z);
    if (!(bbox_min.x() < bbox_max.x() &&
          bbox_min.y() < bbox_max.y() &&
          bbox_min.z() < bbox_max.z())) {
        bbox_min = super_utils::Vec3f(-50.0, -50.0, -2.0);
        bbox_max = super_utils::Vec3f(50.0, 50.0, 10.0);
    }

    boost::system::error_code ec;
    boost::filesystem::create_directories(cfg_.global_guidance.tsp_dir, ec);
    if (ec && ros_ptr_) {
        ros_ptr_->warn(" -- [EPIC Native] Failed to create tsp_dir '{}': {}.",
                       cfg_.global_guidance.tsp_dir,
                       ec.message());
    }
    const boost::filesystem::path tsp_dir(cfg_.global_guidance.tsp_dir);
    const boost::filesystem::path write_probe = tsp_dir / ".general_planner_write_probe";
    {
        std::ofstream probe(write_probe.string(), std::ios::out | std::ios::trunc);
        if (!probe.good() && ros_ptr_) {
            ros_ptr_->warn(" -- [EPIC Native] tsp_dir '{}' is not writable.",
                           cfg_.global_guidance.tsp_dir);
        }
    }
    boost::filesystem::remove(write_probe, ec);

    setParamDefault(nh_, "odometry_topic", std::string("/quad_0/lidar_slam/odom"));
    setParamDefault(nh_, "cloud_topic", std::string("/cloud_registered"));
    nh_.setParam("box_num", 1);
    nh_.setParam("box_0/down", vecToStd(bbox_min));
    nh_.setParam("box_0/up", vecToStd(bbox_max));
    nh_.setParam("dead_area_num", 0);

    nh_.setParam("lidar_perception/max_ray_length",
                 std::max(1.0, lidar.max_ray_length));
    nh_.setParam("lidar_perception/fov_up", lidar.fov_up_deg);
    nh_.setParam("lidar_perception/fov_down", lidar.fov_down_deg);
    nh_.setParam("lidar_perception/fov_viewpoint_up",
                 lidar.viewpoint_fov_up_deg);
    nh_.setParam("lidar_perception/fov_viewpoint_down",
                 lidar.viewpoint_fov_down_deg);
    nh_.setParam("lidar_perception/lidar_pitch", lidar.lidar_pitch_deg);

    const double min_region_xy = std::max(0.05, topo.min_subregion_size_xy);
    const double min_region_z = std::max(0.05, topo.min_subregion_size_z);
    nh_.setParam("bubble_topo/min_x", min_region_xy);
    nh_.setParam("bubble_topo/min_y", min_region_xy);
    nh_.setParam("bubble_topo/min_z", min_region_z);
    nh_.setParam("bubble_topo/init_region_size_x",
                 std::max(0.5, topo.region_size_xy));
    nh_.setParam("bubble_topo/init_region_size_y",
                 std::max(0.5, topo.region_size_xy));
    nh_.setParam("bubble_topo/init_region_size_z",
                 std::max(0.5, topo.region_size_z));
    nh_.setParam("bubble_topo/bubble_min_radius",
                 std::max(0.1, topo.bubble_min_radius));
    nh_.setParam("bubble_topo/frontier_bubble_min_radius",
                 std::max(0.1, topo.frontier_bubble_min_radius));
    nh_.setParam("bubble_topo/cube_discrete_size",
                 std::max(0.05, topo.cube_discrete_size));
    nh_.setParam("parallel_astar/update_connection_timeout",
                 std::max(1.0e-4, topo.local_edge_astar_timeout));
    nh_.setParam("parallel_astar/insert_node_timeout",
                 std::max(1.0e-4, topo.local_edge_astar_timeout));
    nh_.setParam("max_update_region_num",
                 std::max(1, topo.max_update_region_num));

    setParamDefault(nh_, "bubble_astar/resolution_astar",
                    std::max(0.05, topo.bubble_astar_resolution));
    setParamDefault(nh_, "bubble_astar/lambda_heu", 1.0);
    setParamDefault(nh_, "bubble_astar/allocate_num",
                    std::max(1000, topo.bubble_astar_max_nodes));
    setParamDefault(nh_, "bubble_astar/safe_distance",
                    std::max(0.05, topo.bubble_astar_safe_distance));
    setParamDefault(nh_, "bubble_astar/debug", false);

    nh_.setParam("FrontierManager/cell_size", std::max(0.05, obs.resolution));
    nh_.setParam("FrontierManager/noise_cell_range", 1);
    nh_.setParam("FrontierManager/good_observation_direction_score",
                 obs.good_observation_direction_score);
    nh_.setParam("FrontierManager/good_observation_trust_length",
                 obs.good_observation_trust_length);
    nh_.setParam("FrontierManager/good_observation_force_trust_length",
                 obs.good_observation_force_trust_length);
    nh_.setParam("FrontierManager/update_length",
                 std::max(1.0, obs.max_observation_distance));
    nh_.setParam("FrontierManager/view_frt", true);
    nh_.setParam("FrontierManager/view_cluster", true);
    nh_.setParam("FrontierManager/cluster_min_radius",
                 std::max(0.1, obs.frontier_cluster_min_radius));
    nh_.setParam("FrontierManager/cluster_min_size",
                 std::max(0.1, obs.frontier_cluster_min_size));
    nh_.setParam("FrontierManager/cluster_max_size",
                 std::max(obs.frontier_cluster_min_size,
                          obs.frontier_cluster_max_size));
    nh_.setParam("FrontierManager/cluster_direction_radius",
                 obs.frontier_cluster_direction_radius);
    nh_.setParam("FrontierManager/cluster_minmum_point_num",
                 std::max(1, obs.frontier_cluster_minimum_point_num));

    nh_.setParam("ViewpointManager/sample_pillar_height_layer_num",
                 std::max(1, vp.sample_pillar_height_layer_num));
    nh_.setParam("ViewpointManager/sample_pillar_radius_layer_num",
                 std::max(1, vp.sample_pillar_radius_layer_num));
    nh_.setParam("ViewpointManager/sample_pillar_circle_sample_num",
                 std::max(1, vp.sample_pillar_circle_sample_num));
    nh_.setParam("ViewpointManager/sample_pillar_max_height",
                 vp.sample_pillar_max_height);
    nh_.setParam("ViewpointManager/sample_pillar_min_height",
                 vp.sample_pillar_min_height);
    nh_.setParam("ViewpointManager/sample_pillar_min_radius",
                 std::max(0.1, vp.sample_pillar_min_radius));
    nh_.setParam("ViewpointManager/sample_pillar_max_radius",
                 std::max(vp.sample_pillar_min_radius + 0.1,
                          vp.sample_pillar_max_radius));
    nh_.setParam("ViewpointManager/consider_range",
                 std::max(1, vp.max_cells_per_gain_eval));
    nh_.setParam("ViewpointManager/global_recluster_size",
                 std::max(1, vp.max_viewpoint_clusters));
    nh_.setParam("ViewpointManager/local_tsp_size",
                 std::max(1, std::max(vp.local_tsp_size,
                                      cfg_.global_guidance.max_frontiers_in_tour)));
    nh_.setParam("ViewpointManager/view_direction_range", 120.0);
    nh_.setParam("ViewpointManager/min_clearance",
                 std::max(0.05, vp.safe_distance));
    nh_.setParam("ViewpointManager/topo_reachability_timeout",
                 std::max(1.0e-3, vp.topo_reachability_timeout));
    nh_.setParam("ViewpointManager/reachability_only_raycast", false);
    nh_.setParam("lidar_perception/voxel_leaf_size",
                 std::max(0.01, obs.resolution * 0.4));
    nh_.setParam("epic_lio/publish_map", cfg_.publish_lio_map);
    nh_.setParam("epic_lio/publish_map_period",
                 std::max(0.05, cfg_.publish_lio_map_period));
    nh_.setParam("epic_lio/self_filter_radius",
                 std::max(0.0, cfg_.lio_self_filter_radius));

    nh_.setParam("max_traj_len", std::max(2.0, cfg_.local_guide.planning_horizon));
    nh_.setParam("DilateRadiusSoft", 0.6);
    nh_.setParam("DilateRadiusHard", 0.45);
    nh_.setParam("MaxCorridorSize", 4.0);
    nh_.setParam("MaxVelMag", 4.0);
    nh_.setParam("maxBdrMag", 10000.0);
    nh_.setParam("MaxTiltAngle", 1.0);
    nh_.setParam("MinThrust", 15.0);
    nh_.setParam("MaxThrust", 22.0);
    nh_.setParam("VehicleMass", 1.9);
    nh_.setParam("GravAcc", 9.8);
    nh_.setParam("HorizDrag", 0.0);
    nh_.setParam("VertDrag", 0.0);
    nh_.setParam("ParasDrag", 0.0);
    nh_.setParam("SpeedEps", 0.0001);
    nh_.setParam("WeightT", 700.0);
    nh_.setParam("WeightSafeT", 200.0);
    nh_.setParam("ChiVec", std::vector<double>{1.1e+5, 1.5e+4, 1.1e+4, 1.1e+5, 1.1e+6});
    nh_.setParam("SmoothingEps", 0.015);
    nh_.setParam("IntegralIntervs", 32);
    nh_.setParam("RelCostTol", 1.1e-6);
    nh_.setParam("yaw_rho_vis", 10000.0);
    nh_.setParam("yaw_max_vel", 4.0);
    nh_.setParam("yaw_time_fwd", 2.5);

    setParamDefault(nh_, "exploration/tsp_dir", cfg_.global_guidance.tsp_dir);
    setParamDefault(nh_, "viewpoint_param/global_viewpoint_num",
                    std::max(1, cfg_.global_guidance.max_frontiers_in_tour));
    setParamDefault(nh_, "viewpoint_param/local_viewpoint_num",
                    std::max(1, vp.max_viewpoint_clusters));
    setParamDefault(nh_, "view_graph", true);
    setParamDefault(nh_, "global_planning/w_vdir", 0.0);
    setParamDefault(nh_, "global_planning/w_yawdir", 0.0);
}

void EpicExplorationManager::initializeNativeModules() {
    initialized_ = false;

    if (map_manager_ == nullptr) {
        throw std::runtime_error("EPIC Native exploration requires MapManager");
    }
    map_manager_->initEpicLioMap(nh_);
    lio_interface_ = map_manager_->epicLio();
    if (lio_interface_ == nullptr) {
        throw std::runtime_error("EPIC Native exploration failed to initialize LIOInterface");
    }
    parallel_path_finder_ = std::make_shared<ParallelBubbleAstar>();
    bubble_path_finder_ = std::make_shared<fast_planner::BubbleAstar>();
    fast_searcher_ = std::make_shared<fast_planner::FastSearcher>();
    frontier_manager_ = std::make_shared<FrontierManager>();
    graph_ = std::make_shared<TopoGraph>();
    graph_visualizer_ = std::make_shared<GraphVisualizer>();
    next_goal_node_ = std::make_shared<TopoNode>();
    next_goal_node_->is_viewpoint_ = true;
    next_goal_node_->center_ = Eigen::Vector3f::Zero();
    next_goal_node_->yaw_ = 0.0F;
    local_guide_builder_ = std::make_unique<LocalGuideBuilder>(
            cfg_.local_guide, map_manager_, astar_);

    graph_->init(nh_, lio_interface_, parallel_path_finder_);
    parallel_path_finder_->init(nh_, lio_interface_);
    bubble_path_finder_->init(nh_, lio_interface_);
    fast_searcher_->init(graph_, bubble_path_finder_);
    graph_visualizer_->init(nh_);
    native_tour_order_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>(
                    "visualization/exploration/tour_order", 5);
    exploration_box_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>(
                    "visualization/exploration/box", 1, true);
    frontier_manager_->init(nh_, lio_interface_, graph_);

    initialized_ = true;
    resetRuntimeState();
    publishExplorationBox();
    if (ros_ptr_) {
        ros_ptr_->info(" -- [EPIC Native] Initialized native EPIC exploration frontend inside general_planner.");
    }
}

void EpicExplorationManager::updateObservation(const rog_map::PointCloud &cloud,
                                               const super_utils::Pose &pose,
                                               const CloudFrame frame,
                                               const super_utils::Vec3f &sensor_position,
                                               const double stamp) {
    rog_map::RobotState robot;
    robot.rcv = true;
    robot.p = sensor_position;
    robot.q = pose.second.normalized();
    robot.v.setZero();
    robot.a.setZero();
    robot.j.setZero();
    robot.rcv_time = stamp;
    if (map_manager_ != nullptr) {
        map_manager_->updateEpicLioMap(cloud, pose, frame, robot);
    }
    onCloudOdom(cloud, pose, frame, robot, stamp);
}

void EpicExplorationManager::onCloudOdom(const rog_map::PointCloud &cloud,
                                         const super_utils::Pose &pose,
                                         const CloudFrame frame,
                                         const rog_map::RobotState &robot,
                                         const double stamp) {
    (void)frame;
    if (!cfg_.enable) {
        return;
    }
    if (cloud.empty()) {
        const double now = ros_ptr_ ? ros_ptr_->getSimTime() : stamp;
        if (ros_ptr_ && (last_empty_cloud_log_stamp_ < 0.0 ||
                         now - last_empty_cloud_log_stamp_ > 1.0)) {
            ros_ptr_->warn(" -- [EPIC Native] Drop empty exploration cloud before native observation update.");
            last_empty_cloud_log_stamp_ = now;
        }
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return;
    }

    const super_utils::Vec3f sensor_position = robot.rcv ? robot.p : pose.first;
    if (has_last_sensor_position_ && last_observation_stamp_ > 0.0 &&
        stamp > last_observation_stamp_) {
        travel_distance_ += (sensor_position - last_sensor_position_).norm();
    }
    last_sensor_position_ = sensor_position;
    last_observation_stamp_ = stamp;
    has_last_sensor_position_ = true;

    if (lio_interface_ == nullptr ||
        lio_interface_->ld_ == nullptr ||
        lio_interface_->ld_->lidar_cloud_.empty()) {
        return;
    }

    const bool first_observation = !has_observation_;
    has_observation_ = true;
    ++observation_update_count_;
    last_observation_cloud_size_ = lio_interface_->ld_->lidar_cloud_.points.size();
    if (ros_ptr_ && first_observation) {
        ros_ptr_->info(" -- [EPIC Native] First cloud observation accepted: raw_points={}, used_points={}, stamp={:.3f}, sensor=[{:.3f} {:.3f} {:.3f}].",
                       cloud.points.size(),
                       last_observation_cloud_size_,
                       stamp,
                       sensor_position.x(),
                       sensor_position.y(),
                       sensor_position.z());
    }

    const Eigen::Vector3f pos_f = sensor_position.cast<float>();
    const Eigen::Quaterniond q = (robot.rcv ? robot.q : pose.second).normalized();
    const float yaw_f = static_cast<float>(yawFromQuaternion(q));
    if (graph_ && graph_->odom_node_) {
        graph_->odom_node_->center_ = pos_f;
        graph_->odom_node_->yaw_ = yaw_f;
    }
    updateNativeFrontiersFromLatestCloud(stamp);
}

int EpicExplorationManager::updateFrontiers(const double stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || !has_observation_) {
        return 0;
    }
    updateNativeFrontiersFromLatestCloud(stamp);
    return frontier_manager_ ? static_cast<int>(frontier_manager_->cluster_list_.size()) : 0;
}

void EpicExplorationManager::updateTopoGraph(const rog_map::RobotState &robot,
                                             const double current_yaw,
                                             const double stamp) {
    (void)stamp;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || !robot.rcv || !has_observation_) {
        return;
    }
    Eigen::Vector3f pos = robot.p.cast<float>();
    float yaw = static_cast<float>(current_yaw);
    graph_->odom_node_->center_ = pos;
    graph_->updateOdomNode(pos, yaw);
    graph_->updateHistoricalOdoms();
}

bool EpicExplorationManager::planNextGoal(const rog_map::RobotState &robot,
                                          const double current_yaw,
                                          ExplorationGoal &goal) {
    ExplorationPlan plan;
    const bool ok = planOnce(robot, current_yaw, plan);
    goal = plan.goal;
    return ok;
}

bool EpicExplorationManager::planOnce(const rog_map::RobotState &robot,
                                      const double current_yaw,
                                      ExplorationPlan &plan) {
    return planOnce(robot,
                    current_yaw,
                    plan,
                    ExplorationPlanningMode::REFRESH_GLOBAL_IF_NEEDED);
}

bool EpicExplorationManager::planOnce(const rog_map::RobotState &robot,
                                      const double current_yaw,
                                      ExplorationPlan &plan,
                                      const ExplorationPlanningMode mode) {
    plan = ExplorationPlan{};
    std::lock_guard<std::mutex> lock(mutex_);

    if (!cfg_.enable) {
        plan.status = ExplorationPlanStatus::FRONTEND_NOT_READY;
        plan.reason = "exploration disabled";
        plan.goal.reason = plan.reason;
        return false;
    }
    if (!initialized_) {
        plan.status = ExplorationPlanStatus::FRONTEND_NOT_READY;
        plan.reason = "native EPIC modules are not ready";
        plan.goal.reason = plan.reason;
        return false;
    }
    if (!robot.rcv) {
        plan.status = ExplorationPlanStatus::WAIT_FOR_OBSERVATION;
        plan.reason = "no odom";
        plan.goal.reason = plan.reason;
        return false;
    }
    if (!has_observation_) {
        plan.status = ExplorationPlanStatus::WAIT_FOR_OBSERVATION;
        plan.reason = "native EPIC has no cloud observation";
        plan.goal.reason = plan.reason;
        return false;
    }

    if (!updateNativeTopoState(robot, current_yaw, plan)) {
        last_plan_ = plan;
        return false;
    }

    bool goal_ready = false;
    if (mode != ExplorationPlanningMode::FORCE_REFRESH_GLOBAL) {
        goal_ready = buildPlanFromActiveNativeGoal(robot, current_yaw, plan);
    }
    if (!goal_ready) {
        if (mode == ExplorationPlanningMode::FOLLOW_ACTIVE_TARGET) {
            plan.status = ExplorationPlanStatus::VIEWPOINT_EXISTS_NO_TOPO_ROUTE;
            plan.reason = "native EPIC active tour target is unavailable";
            plan.goal.reason = plan.reason;
            last_plan_ = plan;
            visualizeNativeState();
            return false;
        }
        goal_ready = updateNativeGlobalPlan(robot, current_yaw, plan);
    }
    if (!goal_ready) {
        last_plan_ = plan;
        return false;
    }
    if (!buildNativeGuide(robot, current_yaw, plan)) {
        last_plan_ = plan;
        return false;
    }

    plan.valid = true;
    plan.no_frontier = false;
    plan.status = ExplorationPlanStatus::OK;
    plan.reason = "native EPIC guide selected";
    plan.goal.reason = plan.reason;
    active_frontier_id_ = plan.target_frontier_id;
    active_viewpoint_id_ = plan.target_viewpoint_id;
    last_plan_ = plan;

    if (ros_ptr_ && cfg_.print_log) {
        ros_ptr_->info(" -- [EPIC Native] Guide selected frontier_id={}, viewpoint_id={}, native_clusters={}, topo_nodes={}, path_size={}, next=[{:.3f} {:.3f} {:.3f}], final=[{:.3f} {:.3f} {:.3f}], final_reached={}.",
                       plan.target_frontier_id,
                       plan.target_viewpoint_id,
                       frontier_manager_ ? frontier_manager_->cluster_list_.size() : 0,
                       graph_ ? graph_->history_odom_nodes_.size() : 0,
                       plan.guide_path.size(),
                       plan.next_goal.x(),
                       plan.next_goal.y(),
                       plan.next_goal.z(),
                       plan.final_goal.x(),
                       plan.final_goal.y(),
                       plan.final_goal.z(),
                       plan.local_goal_is_final);
    }
    visualizeNativeState();
    return true;
}

bool EpicExplorationManager::updateNativeTopoState(const rog_map::RobotState &robot,
                                                   const double current_yaw,
                                                   ExplorationPlan &plan) {
    if (!frontier_manager_ || !graph_ || !parallel_path_finder_) {
        plan.status = ExplorationPlanStatus::FRONTEND_NOT_READY;
        plan.reason = "native EPIC modules missing";
        plan.goal.reason = plan.reason;
        return false;
    }

    Eigen::Vector3f pos = robot.p.cast<float>();
    float yaw = static_cast<float>(current_yaw);
    graph_->odom_node_->center_ = pos;
    graph_->odom_node_->yaw_ = yaw;

    graph_->getRegionsToUpdate();
    graph_->updateSkeleton();
    graph_->updateOdomNode(pos, yaw);
    graph_->updateHistoricalOdoms();

    if (graph_->odom_node_->neighbors_.empty()) {
        plan.reason = "native EPIC topo odom node has no neighbors";
        plan.status = ExplorationPlanStatus::VIEWPOINT_EXISTS_NO_TOPO_ROUTE;
        plan.goal.reason = plan.reason;
        visualizeNativeState();
        return false;
    }
    return true;
}

bool EpicExplorationManager::buildPlanFromActiveNativeGoal(
        const rog_map::RobotState &robot,
        const double current_yaw,
        ExplorationPlan &plan) {
    (void)current_yaw;
    const bool has_persistent_goal = next_goal_node_ && next_goal_node_valid_;
    const bool has_tour_target =
            !native_tour_targets_.empty() &&
            native_tour_cursor_ < native_tour_targets_.size();
    if (!has_persistent_goal && !has_tour_target) {
        return false;
    }

    Eigen::Vector3f active_goal =
            has_persistent_goal
            ? next_goal_node_->center_
            : native_tour_targets_[native_tour_cursor_].position;
    float active_yaw =
            has_persistent_goal
            ? next_goal_node_->yaw_
            : native_tour_targets_[native_tour_cursor_].yaw;
    int active_frontier_id = active_frontier_id_;
    int active_viewpoint_id = active_viewpoint_id_;
    if (has_tour_target) {
        const auto &target = native_tour_targets_[native_tour_cursor_];
        if (active_frontier_id < 0) {
            active_frontier_id = target.frontier_id;
        }
        if (active_viewpoint_id < 0) {
            active_viewpoint_id = target.viewpoint_id;
        }
    }

    const double goal_distance = (robot.p - toVec3d(active_goal)).norm();
    if (goal_distance <= std::max(0.3, cfg_.final_goal_radius)) {
        rememberVisitedNativeViewpoint(active_goal);
        if (active_frontier_id >= 0) {
            setNativeFrontierDormant(active_frontier_id, false);
            local_fail_count_by_frontier_.erase(active_frontier_id);
            topo_route_fail_count_by_frontier_.erase(active_frontier_id);
        }
        if (advanceNativeTourTarget(robot.p.cast<float>())) {
            active_goal = native_tour_targets_[native_tour_cursor_].position;
            active_yaw = native_tour_targets_[native_tour_cursor_].yaw;
            active_frontier_id = active_frontier_id_;
            active_viewpoint_id = active_viewpoint_id_;
            if (ros_ptr_ && cfg_.print_log) {
                ros_ptr_->info(" -- [EPIC Native] Active viewpoint reached; continue native tour cursor={}/{}.",
                               native_tour_cursor_ + 1U,
                               native_tour_targets_.size());
            }
        } else {
            clearNativeTourState();
            has_last_goal_cost_frame_value_ = false;
            last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
            if (ros_ptr_ && cfg_.print_log) {
                ros_ptr_->info(" -- [EPIC Native] Active viewpoint reached; refresh frontier viewpoints and global tour.");
            }
            return false;
        }
    }

    if (!frontierSelectable(active_frontier_id)) {
        clearNativeTourState();
        return false;
    }

    if (lio_interface_) {
        const double clearance = lio_interface_->getDisToOcc(active_goal);
        if (!std::isfinite(clearance) ||
            clearance <= parallel_path_finder_->safe_distance_ + 0.1) {
            return false;
        }
    }

    GlobalRoute route;
    const double graph_timeout = std::max(1.0e-3, cfg_.topo_graph.local_edge_astar_timeout);
    const bool goal_node_ready = updateNativeGoalNode(active_goal, active_yaw);
    if (goal_node_ready) {
        active_goal = next_goal_node_->center_;
        active_yaw = next_goal_node_->yaw_;
        rebuildRouteToNextGoal(robot, graph_timeout, route);
    }
    if (!route.valid) {
        super_utils::vec_E<super_utils::Vec3f> direct_path;
        double direct_cost = std::numeric_limits<double>::infinity();
        if (buildDirectNativePath(robot.p.cast<float>(),
                                  active_goal,
                                  std::max(0.2, graph_timeout),
                                  direct_path,
                                  direct_cost)) {
            route.valid = true;
            route.path = std::move(direct_path);
            route.cost = direct_cost;
            route.bubble_astar_edge_count =
                    static_cast<int>(std::max<std::size_t>(1U, route.path.size() - 1U));
        }
    }
    if (!route.valid) {
        return false;
    }
    route.target_frontier_id = active_frontier_id;

    plan.goal.valid = true;
    plan.goal.type = ExplorationGoalType::FRONTIER_VIEWPOINT;
    plan.goal.position = toVec3d(active_goal);
    plan.goal.yaw = active_yaw;
    plan.goal.frontier_id = active_frontier_id;
    plan.goal.route_node_id = active_viewpoint_id;
    plan.goal.travel_cost = route.cost;
    plan.goal.gain = 0.0;
    plan.goal.route = route;
    plan.final_goal = plan.goal.position;
    plan.final_yaw = plan.goal.yaw;
    plan.target_frontier_id = plan.goal.frontier_id;
    plan.target_viewpoint_id = plan.goal.route_node_id;
    plan.raw_route_path_size = route.path.size();
    last_native_result_ = 1;

    if (ros_ptr_ && cfg_.print_log) {
        ros_ptr_->info(" -- [EPIC Native] Continue active tour target frontier_id={}, viewpoint_id={}, cursor={}/{}, route_cost={:.3f}, goal=[{:.3f} {:.3f} {:.3f}].",
                       active_frontier_id,
                       active_viewpoint_id,
                       native_tour_targets_.empty() ? 0U : native_tour_cursor_ + 1U,
                       native_tour_targets_.size(),
                       route.cost,
                       plan.final_goal.x(),
                       plan.final_goal.y(),
                       plan.final_goal.z());
    }
    return true;
}

bool EpicExplorationManager::updateNativeGlobalPlan(const rog_map::RobotState &robot,
                                                    const double current_yaw,
                                                    ExplorationPlan &plan) {
    plan.no_frontier = false;
    plan.reason.clear();
    exploration_finished_ = false;

    Eigen::Vector3f pos = robot.p.cast<float>();
    (void)current_yaw;

    std::vector<TopoNode::Ptr> viewpoints;
    Eigen::Vector3f center_pose = pos;
    frontier_manager_->generateTSPViewpoints(center_pose, viewpoints);
    if (viewpoints.empty()) {
        const NativeFrontierStats stats = getNativeFrontierStats();
        const bool no_selectable_frontier_in_box =
                stats.in_search_box == 0 || stats.selectable == 0;
        exploration_finished_ = no_selectable_frontier_in_box;
        plan.no_frontier = no_selectable_frontier_in_box;
        plan.status = no_selectable_frontier_in_box
                      ? ExplorationPlanStatus::TRUE_FINISHED
                      : ExplorationPlanStatus::FRONTIER_EXISTS_NO_VIEWPOINT;
        plan.reason = no_selectable_frontier_in_box
                      ? "native EPIC no selectable frontier in search box"
                      : "native EPIC has frontier clusters but no reachable/selected viewpoint";
        plan.goal.reason = plan.reason;
        if (ros_ptr_) {
            ros_ptr_->warn(" -- [EPIC Native] No viewpoint selected: status={}, finished={}, total_clusters={}, in_box={}, selectable={}, reachable={}, dormant={}, unreachable={}.",
                           explorationPlanStatusName(plan.status),
                           exploration_finished_,
                           stats.total,
                           stats.in_search_box,
                           stats.selectable,
                           stats.reachable,
                           stats.dormant,
                           stats.unreachable);
        }
        visualizeNativeState();
        return false;
    }

    graph_->insertNodes(viewpoints, false);
    if (next_goal_node_valid_) {
        updateNativeGoalNode(next_goal_node_->center_, next_goal_node_->yaw_);
    }

    constexpr double kUnreachableCost = 2.0e3;
    const double graph_timeout = std::max(1.0e-3, cfg_.topo_graph.local_edge_astar_timeout);

    double dis2last_goal = std::numeric_limits<double>::infinity();
    super_utils::vec_E<super_utils::Vec3f> last_goal_path;
    bool last_goal_reachable = false;
    const bool allow_keep_current_target =
            cfg_.keep_active_target || cfg_.global_guidance.keep_current_target;
    if (allow_keep_current_target &&
        next_goal_node_valid_ &&
        next_goal_node_ &&
        lio_interface_) {
        const double clearance = lio_interface_->getDisToOcc(next_goal_node_->center_);
        if (std::isfinite(clearance) &&
            clearance > parallel_path_finder_->safe_distance_ + 0.1 &&
            routeBetweenNativeNodes(graph_->odom_node_,
                                    next_goal_node_,
                                    graph_timeout,
                                    last_goal_path,
                                    dis2last_goal) &&
            std::isfinite(dis2last_goal) &&
            dis2last_goal < kUnreachableCost) {
            if (!has_last_goal_cost_frame_value_) {
                last_goal_cost_frame_value_ = dis2last_goal;
                has_last_goal_cost_frame_value_ = true;
            }
            if (dis2last_goal < 1.5 * last_goal_cost_frame_value_) {
                last_goal_reachable = true;
                last_goal_cost_frame_value_ = dis2last_goal;
            }
        }
    }

    struct Candidate {
        TopoNode::Ptr node;
        super_utils::vec_E<super_utils::Vec3f> odom_path;
        super_utils::vec_E<super_utils::Vec3f> base_path;
        double odom_cost{std::numeric_limits<double>::infinity()};
        double base_cost{std::numeric_limits<double>::infinity()};
        double visited_penalty{0.0};
        int frontier_id{-1};
    };
    std::vector<Candidate> candidates;
    candidates.reserve(viewpoints.size());
    const auto base_node = last_goal_reachable ? next_goal_node_ : graph_->odom_node_;
    int rejected_no_neighbor = 0;
    int rejected_odom_route = 0;
    int rejected_bad_cost = 0;
    int repaired_direct_route = 0;
    std::unordered_set<int> rejected_frontier_ids;
    for (const auto &viewpoint : viewpoints) {
        Candidate candidate;
        candidate.node = viewpoint;
        candidate.frontier_id = viewpoint != nullptr
                                ? frontierIdForViewpoint(toVec3d(viewpoint->center_))
                                : -1;
        if (!viewpoint) {
            ++rejected_no_neighbor;
            continue;
        }
        if (viewpoint->neighbors_.empty()) {
            const double direct_timeout = std::max(0.2, graph_timeout);
            if (!buildDirectNativePath(graph_->odom_node_->center_,
                                       viewpoint->center_,
                                       direct_timeout,
                                       candidate.odom_path,
                                       candidate.odom_cost)) {
                ++rejected_no_neighbor;
                if (candidate.frontier_id >= 0) {
                    rejected_frontier_ids.insert(candidate.frontier_id);
                }
                continue;
            }
            ++repaired_direct_route;
        }
        if (candidate.odom_path.empty() &&
            !routeBetweenNativeNodes(graph_->odom_node_,
                                     viewpoint,
                                     graph_timeout,
                                     candidate.odom_path,
                                     candidate.odom_cost) &&
            !buildDirectNativePath(graph_->odom_node_->center_,
                                   viewpoint->center_,
                                   std::max(0.2, graph_timeout),
                                   candidate.odom_path,
                                   candidate.odom_cost)) {
            ++rejected_odom_route;
            if (candidate.frontier_id >= 0) {
                rejected_frontier_ids.insert(candidate.frontier_id);
            }
            continue;
        }
        if (candidate.odom_path.size() < 2U ||
            !std::isfinite(candidate.odom_cost) ||
            candidate.odom_cost > kUnreachableCost) {
            ++rejected_bad_cost;
            if (candidate.frontier_id >= 0) {
                rejected_frontier_ids.insert(candidate.frontier_id);
            }
            continue;
        }
        if (base_node == graph_->odom_node_) {
            candidate.base_path = candidate.odom_path;
            candidate.base_cost = candidate.odom_cost;
        } else if ((!routeBetweenNativeNodes(base_node,
                                             viewpoint,
                                             graph_timeout,
                                             candidate.base_path,
                                             candidate.base_cost) &&
                    !buildDirectNativePath(base_node->center_,
                                           viewpoint->center_,
                                           std::max(0.2, graph_timeout),
                                           candidate.base_path,
                                           candidate.base_cost)) ||
                   candidate.base_path.size() < 2U ||
                   !std::isfinite(candidate.base_cost)) {
            candidate.base_cost =
                    kUnreachableCost +
                    static_cast<double>((base_node->center_ - viewpoint->center_).norm());
        }
        candidate.visited_penalty = visitedNativeViewpointPenalty(viewpoint->center_);
        candidates.push_back(std::move(candidate));
    }

    if (candidates.empty()) {
        std::vector<int> newly_dormant;
        for (const int frontier_id : rejected_frontier_ids) {
            const int fail_count = ++topo_route_fail_count_by_frontier_[frontier_id];
            if (fail_count >= std::max(1, cfg_.max_local_segment_fail_count)) {
                setNativeFrontierDormant(frontier_id, true);
                newly_dormant.push_back(frontier_id);
                topo_route_fail_count_by_frontier_.erase(frontier_id);
                local_fail_count_by_frontier_.erase(frontier_id);
            }
        }
        graph_->removeNodes(viewpoints);
        plan.status = ExplorationPlanStatus::VIEWPOINT_EXISTS_NO_TOPO_ROUTE;
        plan.reason = "native EPIC topo graph could not route to any frontier viewpoint";
        plan.goal.reason = plan.reason;
        if (ros_ptr_) {
            ros_ptr_->warn(" -- [EPIC Native] Viewpoint route filtering rejected all candidates: viewpoints={}, no_neighbor={}, odom_route_failed={}, bad_path_or_cost={}, direct_repaired={}, affected_frontiers={}, newly_dormant={}.",
                           viewpoints.size(),
                           rejected_no_neighbor,
                           rejected_odom_route,
                           rejected_bad_cost,
                           repaired_direct_route,
                           rejected_frontier_ids.size(),
                           newly_dormant.size());
        }
        visualizeNativeState();
        return false;
    }

    for (const auto &candidate : candidates) {
        if (candidate.frontier_id >= 0) {
            topo_route_fail_count_by_frontier_.erase(candidate.frontier_id);
        }
    }

    const auto failPenalty = [&](const Candidate &candidate) {
        if (candidate.frontier_id < 0) {
            return 0.0;
        }
        const auto it = local_fail_count_by_frontier_.find(candidate.frontier_id);
        if (it == local_fail_count_by_frontier_.end()) {
            return 0.0;
        }
        const int capped_count =
                std::min(it->second, std::max(1, cfg_.max_local_segment_fail_count));
        return 20.0 * static_cast<double>(capped_count);
    };
    const auto entryCost = [&](const Candidate &candidate) {
        const double path_cost =
                cfg_.global_guidance.weight_path_cost * candidate.base_cost;
        const double revisit_cost =
                cfg_.global_guidance.weight_revisit * failPenalty(candidate) +
                candidate.visited_penalty;
        const double outward_reward =
                cfg_.global_guidance.weight_gain * 0.2 *
                std::min(candidate.odom_cost, kUnreachableCost);
        return std::max(1.0, path_cost + revisit_cost - outward_reward);
    };
    const int max_candidates =
            std::max(1, cfg_.global_guidance.max_frontiers_in_tour);
    if (static_cast<int>(candidates.size()) > max_candidates) {
        std::stable_sort(candidates.begin(), candidates.end(),
                         [&](const Candidate &a, const Candidate &b) {
                             return entryCost(a) < entryCost(b);
                         });
        candidates.resize(static_cast<std::size_t>(max_candidates));
    }

    const int dim = static_cast<int>(candidates.size()) + 1;
    Eigen::MatrixXd cost_matrix(dim, dim);
    cost_matrix.setZero();
    for (int i = 1; i < dim; ++i) {
        const auto &candidate = candidates[static_cast<std::size_t>(i - 1)];
        cost_matrix(0, i) = entryCost(candidate);
    }

    for (int i = 1; i < dim; ++i) {
        for (int j = 1; j < dim; ++j) {
            if (i == j) {
                cost_matrix(i, j) = 0.0;
                continue;
            }
            const auto &from = candidates[static_cast<std::size_t>(i - 1)].node;
            const auto &to = candidates[static_cast<std::size_t>(j - 1)].node;
            super_utils::vec_E<super_utils::Vec3f> pair_path;
            double pair_cost = std::numeric_limits<double>::infinity();
            if (!routeBetweenNativeNodes(from, to, graph_timeout, pair_path, pair_cost) ||
                !std::isfinite(pair_cost) ||
                pair_path.size() < 2U) {
                pair_cost = kUnreachableCost +
                            static_cast<double>((from->center_ - to->center_).norm());
            }
            cost_matrix(i, j) =
                    std::max(1.0,
                             cfg_.global_guidance.weight_path_cost * pair_cost +
                             candidates[static_cast<std::size_t>(j - 1)].visited_penalty);
        }
    }

    // EPIC uses an ATSP tour and makes the return-to-start edge expensive; the
    // small distance bias encourages real outward exploration rather than a
    // closed loop around the robot.
    for (int i = 1; i < dim; ++i) {
        cost_matrix(i, 0) =
                std::max(1.0,
                         kUnreachableCost -
                         cfg_.global_guidance.weight_gain *
                         candidates[static_cast<std::size_t>(i - 1)].odom_cost * 0.2);
    }
    for (int i = 0; i < dim; ++i) {
        for (int j = 1; j < dim; ++j) {
            for (int k = 1; k < dim; ++k) {
                if (cost_matrix(i, j) > cost_matrix(i, k) + cost_matrix(k, j)) {
                    cost_matrix(i, j) = cost_matrix(i, k) + cost_matrix(k, j) + 1.0e-2;
                }
            }
        }
    }

    std::vector<int> tour_order;
    if (dim == 2) {
        tour_order = {0, 1};
    } else if (!solveNativeAtspTour(cost_matrix, tour_order)) {
        graph_->removeNodes(viewpoints);
        plan.status = ExplorationPlanStatus::VIEWPOINT_EXISTS_NO_TOPO_ROUTE;
        plan.reason = "native EPIC global tour solver failed";
        plan.goal.reason = plan.reason;
        visualizeNativeState();
        return false;
    }
    normalizeTourOrder(tour_order, dim);

    std::vector<int> ordered_candidate_indices;
    ordered_candidate_indices.reserve(static_cast<std::size_t>(std::max(0, dim - 1)));
    for (const int node_idx : tour_order) {
        if (node_idx > 0 && node_idx < dim) {
            ordered_candidate_indices.push_back(node_idx - 1);
        }
    }
    if (ordered_candidate_indices.empty()) {
        ordered_candidate_indices.push_back(static_cast<int>(
                std::min_element(candidates.begin(), candidates.end(),
                                 [](const Candidate &a, const Candidate &b) {
                                     return a.odom_cost < b.odom_cost;
                                 }) - candidates.begin()));
    }

    const int selected_candidate_idx = ordered_candidate_indices.front();
    Candidate selected = candidates[static_cast<std::size_t>(selected_candidate_idx)];
    native_tour_targets_.clear();
    native_tour_targets_.reserve(ordered_candidate_indices.size());
    for (std::size_t order_id = 0U; order_id < ordered_candidate_indices.size(); ++order_id) {
        const auto &candidate = candidates[static_cast<std::size_t>(
                ordered_candidate_indices[order_id])];
        if (!candidate.node) {
            continue;
        }
        NativeTourTarget target;
        target.position = candidate.node->center_;
        target.yaw = candidate.node->yaw_;
        target.frontier_id = candidate.frontier_id >= 0
                             ? candidate.frontier_id
                             : plan_seq_ + 1 + static_cast<int>(order_id);
        target.viewpoint_id = plan_seq_ + 1 + static_cast<int>(order_id);
        native_tour_targets_.push_back(target);
    }
    native_tour_cursor_ = 0U;
    if (native_tour_targets_.empty()) {
        graph_->removeNodes(viewpoints);
        plan.status = ExplorationPlanStatus::VIEWPOINT_EXISTS_NO_TOPO_ROUTE;
        plan.reason = "native EPIC global tour has no valid target";
        plan.goal.reason = plan.reason;
        visualizeNativeState();
        return false;
    }
    selected.frontier_id = native_tour_targets_.front().frontier_id;
    const int viewpoint_id = native_tour_targets_.front().viewpoint_id;

    rebuildGlobalTourFromCursor(pos);

    if (!last_goal_reachable && std::isfinite(selected.base_cost)) {
        last_goal_cost_frame_value_ = selected.base_cost;
        has_last_goal_cost_frame_value_ = true;
    }

    GlobalRoute route;
    route.valid = true;
    route.target_frontier_id = selected.frontier_id;

    graph_->removeNodes(viewpoints);
    const bool goal_node_ready = setNativeGoalFromTourTarget(0U);
    if (!goal_node_ready ||
        !rebuildRouteToNextGoal(robot, graph_timeout, route)) {
        if (ros_ptr_) {
            ros_ptr_->warn(" -- [EPIC Native] Persistent next_goal_node route failed; use selected temporary viewpoint path for this cycle.");
        }
        route.path = std::move(selected.odom_path);
        route.cost = selected.odom_cost;
        route.valid = route.path.size() >= 2U && std::isfinite(route.cost);
    }
    if (!route.valid) {
        plan.status = ExplorationPlanStatus::VIEWPOINT_EXISTS_NO_TOPO_ROUTE;
        plan.reason = "native EPIC failed to rebuild route to selected next goal";
        plan.goal.reason = plan.reason;
        visualizeNativeState();
        return false;
    }
    route.target_frontier_id = selected.frontier_id;
    route.bubble_astar_edge_count =
            static_cast<int>(std::max<std::size_t>(1U, route.path.size() - 1U));

    plan.goal.valid = true;
    plan.goal.type = ExplorationGoalType::FRONTIER_VIEWPOINT;
    plan.goal.position = toVec3d(selected.node->center_);
    plan.goal.yaw = selected.node->yaw_;
    plan.goal.frontier_id = route.target_frontier_id;
    plan.goal.route_node_id = viewpoint_id;
    plan.goal.travel_cost = route.cost;
    plan.goal.gain = 0.0;
    plan.goal.route = route;
    plan.final_goal = plan.goal.position;
    plan.final_yaw = plan.goal.yaw;
    plan.target_frontier_id = plan.goal.frontier_id;
    plan.target_viewpoint_id = plan.goal.route_node_id;
    plan.raw_route_path_size = route.path.size();
    last_native_result_ = 1;

    if (ros_ptr_ && cfg_.print_log) {
        ros_ptr_->info(" -- [EPIC Native] Global planning target frontier_id={}, candidates={}, last_goal_reachable={}, dis2last_goal={:.3f}, route_cost={:.3f}, visited_penalty={:.1f}, visited_cache={}, global_tour_size={}.",
                       selected.frontier_id,
                       candidates.size(),
                       last_goal_reachable,
                       std::isfinite(dis2last_goal) ? dis2last_goal : -1.0,
                       route.cost,
                       selected.visited_penalty,
                       visited_native_viewpoints_.size(),
                       global_tour_.size());
    }
    return true;
}

bool EpicExplorationManager::buildNativeGuide(const rog_map::RobotState &robot,
                                              const double current_yaw,
                                              ExplorationPlan &plan) {
    if (!parallel_path_finder_ || !local_guide_builder_) {
        plan.status = ExplorationPlanStatus::FRONTEND_NOT_READY;
        plan.reason = "native EPIC guide builder is not ready";
        plan.goal.reason = plan.reason;
        return false;
    }

    GlobalRoute local_route = plan.goal.route;
    if (local_route.path.size() < 2U) {
        std::vector<Eigen::Vector3f> native_path;
        const Eigen::Vector3f start = robot.p.cast<float>();
        const Eigen::Vector3f goal = plan.final_goal.cast<float>();
        const int search_ret = parallel_path_finder_->search(start, goal, native_path, 1.0, true);
        if (search_ret != ParallelBubbleAstar::REACH_END || native_path.size() < 2U) {
            plan.status = ExplorationPlanStatus::VIEWPOINT_EXISTS_NO_TOPO_ROUTE;
            plan.reason = "native EPIC failed to connect odom to next viewpoint";
            plan.goal.reason = plan.reason;
            return false;
        }
        local_route.path.clear();
        appendUnique(local_route.path, robot.p);
        for (const auto &p_f : native_path) {
            appendUnique(local_route.path, toVec3d(p_f));
        }
        appendUnique(local_route.path, plan.final_goal);
        local_route.valid = local_route.path.size() >= 2U;
        local_route.cost = pathLength(local_route.path);
        local_route.bubble_astar_edge_count =
                static_cast<int>(std::max<std::size_t>(1U, local_route.path.size() - 1U));
    }
    plan.goal.route = local_route;

    LocalGuideBuilder::Request request;
    request.route = local_route;
    request.robot_pos = robot.p;
    request.current_yaw = current_yaw;
    request.final_goal = plan.final_goal;
    request.final_yaw = plan.final_yaw;
    request.target_frontier_id = plan.target_frontier_id;
    request.target_viewpoint_id = plan.target_viewpoint_id;
    LocalGuideBuilder::Result result;
    if (local_guide_builder_ == nullptr || !local_guide_builder_->build(request, result)) {
        plan.status = ExplorationPlanStatus::LOCAL_GUIDE_FAILED;
        plan.reason = result.reason.empty() ? "native EPIC local guide refinement failed"
                                            : result.reason;
        plan.goal.reason = plan.reason;
        return false;
    }

    ++plan_seq_;
    plan.next_goal = result.next_goal;
    plan.next_yaw = result.next_yaw;
    plan.local_goal_is_final = result.local_goal_is_final;
    plan.guide_path = std::move(result.guide_path);
    plan.refined_guide_path_size = plan.guide_path.size();
    plan.route_progress_length = pathLength(plan.guide_path);
    return true;
}

void EpicExplorationManager::clearNativeGoalState() {
    if (!next_goal_node_) {
        next_goal_node_inserted_ = false;
        next_goal_node_valid_ = false;
        return;
    }

    std::vector<TopoNode::Ptr> old_neighbors(next_goal_node_->neighbors_.begin(),
                                             next_goal_node_->neighbors_.end());
    for (auto &nbr : old_neighbors) {
        if (!nbr) {
            continue;
        }
        nbr->neighbors_.erase(next_goal_node_);
        nbr->paths_.erase(next_goal_node_);
        nbr->weight_.erase(next_goal_node_);
        nbr->unreachable_nbrs_.erase(next_goal_node_);
    }

    if (next_goal_node_inserted_ && graph_) {
        Eigen::Vector3i idx;
        graph_->getIndex(next_goal_node_->center_, idx);
        auto region = graph_->getRegionNode(idx);
        if (region) {
            region->topo_nodes_.erase(next_goal_node_);
        }
    }

    next_goal_node_->neighbors_.clear();
    next_goal_node_->paths_.clear();
    next_goal_node_->weight_.clear();
    next_goal_node_->unreachable_nbrs_.clear();
    next_goal_node_inserted_ = false;
    next_goal_node_valid_ = false;
}

void EpicExplorationManager::clearNativeTourState() {
    clearNativeGoalState();
    global_tour_.clear();
    native_tour_targets_.clear();
    native_tour_cursor_ = 0U;
    active_frontier_id_ = -1;
    active_viewpoint_id_ = -1;
    publishNativeTourOrder();
}

void EpicExplorationManager::rebuildGlobalTourFromCursor(const Eigen::Vector3f &start) {
    global_tour_.clear();
    if (!start.allFinite() ||
        native_tour_targets_.empty() ||
        native_tour_cursor_ >= native_tour_targets_.size()) {
        publishNativeTourOrder();
        return;
    }

    global_tour_.push_back(start);
    for (std::size_t i = native_tour_cursor_; i < native_tour_targets_.size(); ++i) {
        if (native_tour_targets_[i].position.allFinite()) {
            global_tour_.push_back(native_tour_targets_[i].position);
        }
    }
    if (graph_visualizer_) {
        graph_visualizer_->vizTour(global_tour_, VizColor::RED, "global");
    }
    publishNativeTourOrder();
}

bool EpicExplorationManager::advanceNativeTourTarget(const Eigen::Vector3f &start) {
    if (native_tour_targets_.empty() ||
        native_tour_cursor_ + 1U >= native_tour_targets_.size()) {
        return false;
    }

    const std::size_t next_cursor = native_tour_cursor_ + 1U;
    if (!setNativeGoalFromTourTarget(next_cursor)) {
        clearNativeTourState();
        return false;
    }

    has_last_goal_cost_frame_value_ = false;
    last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
    rebuildGlobalTourFromCursor(start);
    return true;
}

bool EpicExplorationManager::setNativeGoalFromTourTarget(const std::size_t target_index) {
    if (target_index >= native_tour_targets_.size()) {
        return false;
    }
    const auto target = native_tour_targets_[target_index];
    if (!target.position.allFinite()) {
        return false;
    }
    native_tour_cursor_ = target_index;
    active_frontier_id_ = target.frontier_id;
    active_viewpoint_id_ = target.viewpoint_id;
    if (!updateNativeGoalNode(target.position, target.yaw)) {
        return false;
    }
    return true;
}

bool EpicExplorationManager::frontierSelectable(const int frontier_id) const {
    if (frontier_id < 0 || !frontier_manager_) {
        return true;
    }
    for (const auto &cluster : frontier_manager_->cluster_list_) {
        if (!cluster || cluster->id_ != frontier_id) {
            continue;
        }
        return !cluster->is_dormant_ && cluster->is_reachable_;
    }
    return true;
}

void EpicExplorationManager::rememberVisitedNativeViewpoint(const Eigen::Vector3f &position) {
    if (!position.allFinite()) {
        return;
    }
    const double radius =
            std::max(0.05, cfg_.frontier_database.visited_viewpoint_radius);
    for (auto &visited : visited_native_viewpoints_) {
        if ((visited - position).norm() <= radius * 0.5) {
            visited = 0.7F * visited + 0.3F * position;
            return;
        }
    }

    visited_native_viewpoints_.push_back(position);
    const int max_count =
            std::max(0, cfg_.frontier_database.max_visited_viewpoints);
    if (max_count <= 0) {
        visited_native_viewpoints_.clear();
        return;
    }
    while (static_cast<int>(visited_native_viewpoints_.size()) > max_count) {
        visited_native_viewpoints_.erase(visited_native_viewpoints_.begin());
    }
}

double EpicExplorationManager::visitedNativeViewpointPenalty(
        const Eigen::Vector3f &position) const {
    const double radius =
            std::max(0.05, cfg_.frontier_database.visited_viewpoint_radius);
    const double max_penalty =
            std::max(0.0, cfg_.frontier_database.visited_viewpoint_penalty);
    if (!position.allFinite() ||
        visited_native_viewpoints_.empty() ||
        max_penalty <= 0.0) {
        return 0.0;
    }

    double nearest = std::numeric_limits<double>::infinity();
    for (const auto &visited : visited_native_viewpoints_) {
        if (!visited.allFinite()) {
            continue;
        }
        nearest = std::min(nearest,
                           static_cast<double>((visited - position).norm()));
    }
    if (!std::isfinite(nearest) || nearest > 2.0 * radius) {
        return 0.0;
    }
    if (nearest <= radius) {
        return max_penalty * (1.0 + (radius - nearest) / radius);
    }
    return max_penalty * (2.0 * radius - nearest) / radius;
}

bool EpicExplorationManager::updateNativeGoalNode(const Eigen::Vector3f &goal,
                                                  const float yaw) {
    if (!graph_ || !parallel_path_finder_ || !next_goal_node_ || !goal.allFinite()) {
        return false;
    }

    clearNativeGoalState();
    next_goal_node_->center_ = goal;
    next_goal_node_->yaw_ = yaw;
    next_goal_node_->is_viewpoint_ = true;

    Eigen::Vector3i idx;
    graph_->getIndex(goal, idx);
    if (!graph_->getRegionNode(idx)) {
        return false;
    }

    std::vector<TopoNode::Ptr> neighbor_nodes;
    std::unordered_set<TopoNode::Ptr> seen;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                Eigen::Vector3i nbr_idx = idx;
                nbr_idx.x() += dx;
                nbr_idx.y() += dy;
                nbr_idx.z() += dz;
                auto region = graph_->getRegionNode(nbr_idx);
                if (!region) {
                    continue;
                }
                for (const auto &node : region->topo_nodes_) {
                    if (!node || node == next_goal_node_ || seen.count(node) > 0U) {
                        continue;
                    }
                    if ((node->center_ - goal).norm() < 1.0e-3F) {
                        continue;
                    }
                    neighbor_nodes.emplace_back(node);
                    seen.insert(node);
                }
            }
        }
    }

    std::vector<TopoNode::Ptr> connected_neighbors;
    std::vector<std::vector<Eigen::Vector3f>> connected_paths;
    connected_neighbors.reserve(neighbor_nodes.size());
    connected_paths.reserve(neighbor_nodes.size());
    const double timeout = std::max(1.0e-3, cfg_.topo_graph.local_edge_astar_timeout);
    for (const auto &nbr : neighbor_nodes) {
        std::vector<Eigen::Vector3f> path;
        const int res = parallel_path_finder_->search(goal,
                                                      nbr->center_,
                                                      path,
                                                      timeout,
                                                      false);
        if (res == ParallelBubbleAstar::REACH_END &&
            path.size() >= 2U &&
            parallel_path_finder_->collisionCheck_shortenPath(path)) {
            connected_neighbors.emplace_back(nbr);
            connected_paths.emplace_back(std::move(path));
        }
    }
    if (connected_neighbors.empty() && graph_->odom_node_) {
        std::vector<Eigen::Vector3f> path;
        const int res = parallel_path_finder_->search(goal,
                                                      graph_->odom_node_->center_,
                                                      path,
                                                      std::max(0.2, timeout),
                                                      true);
        if (res == ParallelBubbleAstar::REACH_END &&
            path.size() >= 2U &&
            parallel_path_finder_->collisionCheck_shortenPath(path)) {
            connected_neighbors.emplace_back(graph_->odom_node_);
            connected_paths.emplace_back(std::move(path));
        }
    }

    if (connected_neighbors.empty()) {
        return false;
    }

    graph_->insertNode(next_goal_node_, connected_neighbors, connected_paths);
    next_goal_node_inserted_ = true;
    next_goal_node_valid_ = true;
    return true;
}

bool EpicExplorationManager::rebuildRouteToNextGoal(const rog_map::RobotState &robot,
                                                    const double timeout,
                                                    GlobalRoute &route) const {
    route = GlobalRoute{};
    if (!robot.rcv || !graph_ || !graph_->odom_node_ ||
        !next_goal_node_ || !next_goal_node_valid_) {
        return false;
    }

    double cost = std::numeric_limits<double>::infinity();
    super_utils::vec_E<super_utils::Vec3f> path;
    if (fast_searcher_) {
        std::vector<Eigen::Vector3f> native_path;
        const int ret = fast_searcher_->search(graph_->odom_node_,
                                               robot.v.cast<float>(),
                                               next_goal_node_,
                                               std::max(0.2, timeout),
                                               native_path);
        if (ret == fast_planner::BubbleAstar::REACH_END && native_path.size() >= 2U) {
            appendUnique(path, toVec3d(graph_->odom_node_->center_));
            for (const auto &p : native_path) {
                appendUnique(path, toVec3d(p));
            }
            appendUnique(path, toVec3d(next_goal_node_->center_));
            std::vector<Eigen::Vector3f> complete_native_path;
            complete_native_path.reserve(path.size());
            for (const auto &p : path) {
                complete_native_path.emplace_back(p.cast<float>());
            }
            cost = nativeRouteCost(complete_native_path);
        }
    }
    if (path.size() < 2U || !std::isfinite(cost)) {
        if (!routeBetweenNativeNodes(graph_->odom_node_,
                                     next_goal_node_,
                                     timeout,
                                     path,
                                     cost) ||
            path.size() < 2U ||
            !std::isfinite(cost)) {
            return false;
        }
    }

    route.valid = true;
    route.path = std::move(path);
    route.cost = cost;
    route.bubble_astar_edge_count =
            static_cast<int>(std::max<std::size_t>(1U, route.path.size() - 1U));
    return true;
}

bool EpicExplorationManager::routeBetweenNativeNodes(
        const std::shared_ptr<TopoNode> &start,
        const std::shared_ptr<TopoNode> &goal,
        const double timeout,
        super_utils::vec_E<super_utils::Vec3f> &path,
        double &cost) const {
    path.clear();
    cost = std::numeric_limits<double>::infinity();
    if (!graph_ || !start || !goal) {
        return false;
    }

    if (fast_searcher_) {
        std::vector<Eigen::Vector3f> native_path;
        const int ret = fast_searcher_->topoSearch(start, goal, timeout, native_path);
        if (ret == fast_planner::BubbleAstar::REACH_END && native_path.size() >= 2U) {
            appendUnique(path, toVec3d(start->center_));
            for (const auto &p : native_path) {
                appendUnique(path, toVec3d(p));
            }
            appendUnique(path, toVec3d(goal->center_));
            std::vector<Eigen::Vector3f> complete_native_path;
            complete_native_path.reserve(path.size());
            for (const auto &p : path) {
                complete_native_path.emplace_back(p.cast<float>());
            }
            cost = nativeRouteCost(complete_native_path);
            return path.size() >= 2U && std::isfinite(cost);
        }
        path.clear();
    }

    std::vector<TopoNode::Ptr> topo_path;
    if (!graph_->graphSearch(start, goal, topo_path, timeout)) {
        return false;
    }
    if (topo_path.empty()) {
        return false;
    }

    appendUnique(path, toVec3d(topo_path.front()->center_));
    for (std::size_t i = 0; i + 1U < topo_path.size(); ++i) {
        const auto &from = topo_path[i];
        const auto &to = topo_path[i + 1U];
        const auto path_it = from->paths_.find(to);
        if (path_it != from->paths_.end()) {
            for (const auto &p : path_it->second) {
                appendUnique(path, toVec3d(p));
            }
        } else {
            appendUnique(path, toVec3d(to->center_));
        }
    }
    appendUnique(path, toVec3d(goal->center_));
    std::vector<Eigen::Vector3f> native_path;
    native_path.reserve(path.size());
    for (const auto &p : path) {
        native_path.emplace_back(p.cast<float>());
    }
    cost = nativeRouteCost(native_path);
    return path.size() >= 2U && std::isfinite(cost);
}

bool EpicExplorationManager::buildDirectNativePath(
        const Eigen::Vector3f &start,
        const Eigen::Vector3f &goal,
        const double timeout,
        super_utils::vec_E<super_utils::Vec3f> &path,
        double &cost) const {
    path.clear();
    cost = std::numeric_limits<double>::infinity();
    if (!parallel_path_finder_ || !start.allFinite() || !goal.allFinite()) {
        return false;
    }
    if ((goal - start).norm() < 1.0e-3F) {
        appendUnique(path, toVec3d(start));
        appendUnique(path, toVec3d(goal));
        cost = pathLength(path);
        return true;
    }

    std::vector<Eigen::Vector3f> native_path;
    const int res = parallel_path_finder_->search(start,
                                                  goal,
                                                  native_path,
                                                  std::max(1.0e-3, timeout),
                                                  true);
    if (res != ParallelBubbleAstar::REACH_END || native_path.size() < 2U) {
        return false;
    }
    if (!parallel_path_finder_->collisionCheck_shortenPath(native_path)) {
        return false;
    }
    appendUnique(path, toVec3d(start));
    for (const auto &point : native_path) {
        appendUnique(path, toVec3d(point));
    }
    appendUnique(path, toVec3d(goal));
    cost = nativeRouteCost(native_path);
    return path.size() >= 2U && std::isfinite(cost);
}

bool EpicExplorationManager::solveNativeAtspTour(const Eigen::MatrixXd &cost_matrix,
                                                 std::vector<int> &order) const {
    order.clear();
    if (cost_matrix.rows() != cost_matrix.cols() || cost_matrix.rows() < 2) {
        return false;
    }
    if (!cfg_.global_guidance.use_lkh) {
        if (ros_ptr_) {
            ros_ptr_->warn(" -- [EPIC Native] EPIC LKH global tour is disabled.");
        }
        return false;
    }
    const bool ok = solveNativeAtspTourLkh(cost_matrix, order);
    if (!ok && ros_ptr_) {
        ros_ptr_->warn(" -- [EPIC Native] LKH global tour failed.");
    }
    if (ok) {
        normalizeTourOrder(order, static_cast<int>(cost_matrix.rows()));
    }
    return ok && order.size() >= 2U;
}

bool EpicExplorationManager::solveNativeAtspTourLkh(const Eigen::MatrixXd &cost_matrix,
                                                    std::vector<int> &order) const {
    order.clear();
    const int dimension = static_cast<int>(cost_matrix.rows());
    if (dimension < 3) {
        return false;
    }

    boost::system::error_code ec;
    boost::filesystem::create_directories(cfg_.global_guidance.tsp_dir, ec);
    if (ec) {
        return false;
    }

    std::ostringstream base_name;
    base_name << cfg_.global_guidance.tsp_problem_name
              << "_" << static_cast<long>(::getpid())
              << "_" << plan_seq_;
    const boost::filesystem::path tsp_dir(cfg_.global_guidance.tsp_dir);
    const boost::filesystem::path tsp_path = tsp_dir / (base_name.str() + ".tsp");
    const boost::filesystem::path par_path = tsp_dir / (base_name.str() + ".par");
    const boost::filesystem::path tour_path = tsp_dir / (base_name.str() + ".txt");

    {
        std::ofstream par_file(par_path.string(), std::ios::out | std::ios::trunc);
        if (!par_file.good()) {
            return false;
        }
        par_file << "PROBLEM_FILE = " << tsp_path.string() << "\n";
        par_file << "GAIN23 = NO\n";
        par_file << "MOVE_TYPE = 2\n";
        par_file << "OUTPUT_TOUR_FILE = " << tour_path.string() << "\n";
        par_file << "RUNS = 10\n";
    }

    {
        std::ofstream prob_file(tsp_path.string(), std::ios::out | std::ios::trunc);
        if (!prob_file.good()) {
            return false;
        }
        prob_file << "NAME : " << base_name.str() << "\n";
        prob_file << "TYPE : ATSP\n";
        prob_file << "DIMENSION : " << dimension << "\n";
        prob_file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
        prob_file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
        prob_file << "EDGE_WEIGHT_SECTION\n";
        const int scale = std::max(1, cfg_.global_guidance.lkh_cost_scale);
        for (int i = 0; i < dimension; ++i) {
            for (int j = 0; j < dimension; ++j) {
                const double cost = std::isfinite(cost_matrix(i, j))
                                    ? std::max(0.0, cost_matrix(i, j))
                                    : 1.0e6;
                prob_file << static_cast<int>(std::lround(cost * scale)) << " ";
            }
            prob_file << "\n";
        }
        prob_file << "EOF\n";
    }

    if (solveTSPLKH(par_path.string().c_str()) != EXIT_SUCCESS) {
        return false;
    }

    std::ifstream result_file(tour_path.string());
    if (!result_file.good()) {
        return false;
    }
    std::string line;
    while (std::getline(result_file, line)) {
        if (line == "TOUR_SECTION") {
            break;
        }
    }
    while (std::getline(result_file, line)) {
        try {
            const int id = std::stoi(line);
            if (id == -1) {
                break;
            }
            order.push_back(id - 1);
        } catch (const std::exception &) {
            continue;
        }
    }
    boost::filesystem::remove(tsp_path, ec);
    boost::filesystem::remove(par_path, ec);
    boost::filesystem::remove(tour_path, ec);
    normalizeTourOrder(order, dimension);
    return order.size() >= 2U;
}

void EpicExplorationManager::normalizeTourOrder(std::vector<int> &order,
                                                const int dimension) {
    if (dimension <= 0) {
        order.clear();
        return;
    }
    std::vector<int> clean;
    clean.reserve(static_cast<std::size_t>(dimension));
    std::vector<bool> seen(static_cast<std::size_t>(dimension), false);
    for (const int id : order) {
        if (id < 0 || id >= dimension || seen[static_cast<std::size_t>(id)]) {
            continue;
        }
        clean.push_back(id);
        seen[static_cast<std::size_t>(id)] = true;
    }
    if (clean.empty()) {
        clean.push_back(0);
        seen[0] = true;
    }
    auto zero_it = std::find(clean.begin(), clean.end(), 0);
    if (zero_it == clean.end()) {
        clean.insert(clean.begin(), 0);
        seen[0] = true;
    } else {
        std::rotate(clean.begin(), zero_it, clean.end());
    }
    for (int id = 0; id < dimension; ++id) {
        if (!seen[static_cast<std::size_t>(id)]) {
            clean.push_back(id);
        }
    }
    order.swap(clean);
}

int EpicExplorationManager::frontierIdForViewpoint(const super_utils::Vec3f &viewpoint) const {
    if (!frontier_manager_) {
        return -1;
    }
    int best_id = -1;
    double best_dis = std::numeric_limits<double>::infinity();
    for (const auto &cluster : frontier_manager_->cluster_list_) {
        if (!cluster) {
            continue;
        }
        const double dis = (toVec3d(cluster->best_vp_) - viewpoint).norm();
        if (dis < best_dis) {
            best_dis = dis;
            best_id = cluster->id_;
        }
    }
    return best_dis < 0.5 ? best_id : -1;
}

void EpicExplorationManager::setNativeFrontierDormant(const int frontier_id,
                                                      const bool unreachable) {
    if (!frontier_manager_ || frontier_id < 0) {
        return;
    }
    for (auto &cluster : frontier_manager_->cluster_list_) {
        if (!cluster || cluster->id_ != frontier_id) {
            continue;
        }
        cluster->is_dormant_ = true;
        if (unreachable) {
            cluster->is_reachable_ = false;
        }
        break;
    }
}

void EpicExplorationManager::updateNativeFrontiersFromLatestCloud(const double stamp) {
    (void)stamp;
    if (!frontier_manager_ || !graph_ || !lio_interface_ || !lio_interface_->ld_) {
        return;
    }
    std::vector<ClusterInfo::Ptr> new_clusters;
    std::vector<int> cluster_removed;
    frontier_manager_->updateFrontierClusters(new_clusters, cluster_removed);
    const int odom_id = graph_->history_odom_nodes_.empty()
                        ? -1
                        : static_cast<int>(graph_->history_odom_nodes_.size()) - 1;
    for (auto &cluster : new_clusters) {
        if (cluster) {
            cluster->odom_id_ = odom_id;
        }
    }
}

void EpicExplorationManager::visualizeNativeState() const {
    if (!frontier_manager_) {
        return;
    }
    frontier_manager_->viz_pocc();
    frontier_manager_->visfrtcluster();
    if (graph_visualizer_ && graph_) {
        graph_visualizer_->vizBox(graph_);
        graph_visualizer_->vizGraph(graph_);
    }
    publishExplorationBox();
    publishNativeTourOrder();
}

void EpicExplorationManager::publishNativeTourOrder() const {
    if (!native_tour_order_pub_) {
        return;
    }

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const ros::Time stamp = ros::Time::now();
    if (global_tour_.size() < 2U) {
        native_tour_order_pub_.publish(markers);
        return;
    }

    auto init_marker = [&](visualization_msgs::Marker &marker,
                           const std::string &ns,
                           const int id) {
        marker.header.frame_id = "world";
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
    };

    visualization_msgs::Marker path_marker;
    init_marker(path_marker, "tour_order_path", 0);
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.scale.x = 0.18;
    path_marker.color.r = 0.02F;
    path_marker.color.g = 0.02F;
    path_marker.color.b = 0.02F;
    path_marker.color.a = 0.95F;
    path_marker.points.reserve(global_tour_.size());
    for (const auto &p : global_tour_) {
        geometry_msgs::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z() + 0.05F;
        path_marker.points.push_back(point);
    }
    markers.markers.push_back(path_marker);

    visualization_msgs::Marker target_marker;
    init_marker(target_marker, "tour_order_targets", 0);
    target_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    target_marker.scale.x = 0.42;
    target_marker.scale.y = 0.42;
    target_marker.scale.z = 0.42;
    target_marker.color.r = 1.0F;
    target_marker.color.g = 0.72F;
    target_marker.color.b = 0.05F;
    target_marker.color.a = 0.95F;
    target_marker.points.reserve(global_tour_.size() - 1U);
    for (std::size_t i = 1U; i < global_tour_.size(); ++i) {
        geometry_msgs::Point point;
        point.x = global_tour_[i].x();
        point.y = global_tour_[i].y();
        point.z = global_tour_[i].z();
        target_marker.points.push_back(point);
    }
    markers.markers.push_back(target_marker);

    visualization_msgs::Marker start_label;
    init_marker(start_label, "tour_order_labels", 0);
    start_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    start_label.scale.z = 0.65;
    start_label.color.r = 0.02F;
    start_label.color.g = 0.02F;
    start_label.color.b = 0.02F;
    start_label.color.a = 1.0F;
    start_label.pose.position.x = global_tour_.front().x();
    start_label.pose.position.y = global_tour_.front().y();
    start_label.pose.position.z = global_tour_.front().z() + 0.55F;
    start_label.text = "S";
    markers.markers.push_back(start_label);

    for (std::size_t i = 1U; i < global_tour_.size(); ++i) {
        visualization_msgs::Marker label;
        init_marker(label, "tour_order_labels", static_cast<int>(i));
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.scale.z = 0.75;
        label.color.r = 0.02F;
        label.color.g = 0.02F;
        label.color.b = 0.02F;
        label.color.a = 1.0F;
        label.pose.position.x = global_tour_[i].x();
        label.pose.position.y = global_tour_[i].y();
        label.pose.position.z = global_tour_[i].z() + 0.65F;
        label.text = std::to_string(i);
        markers.markers.push_back(label);
    }

    native_tour_order_pub_.publish(markers);
}

void EpicExplorationManager::publishExplorationBox() const {
    if (!exploration_box_pub_) {
        return;
    }

    Eigen::Vector3f box_min(static_cast<float>(cfg_.observation_map.bbox_min_x),
                            static_cast<float>(cfg_.observation_map.bbox_min_y),
                            static_cast<float>(cfg_.observation_map.bbox_min_z));
    Eigen::Vector3f box_max(static_cast<float>(cfg_.observation_map.bbox_max_x),
                            static_cast<float>(cfg_.observation_map.bbox_max_y),
                            static_cast<float>(cfg_.observation_map.bbox_max_z));
    if (lio_interface_ && lio_interface_->lp_) {
        box_min = lio_interface_->lp_->global_map_min_boundary_;
        box_max = lio_interface_->lp_->global_map_max_boundary_;
    }
    if (!(box_min.allFinite() && box_max.allFinite()) ||
        !(box_min.x() < box_max.x() &&
          box_min.y() < box_max.y() &&
          box_min.z() < box_max.z())) {
        return;
    }

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const ros::Time stamp = ros::Time::now();
    auto make_point = [](const Eigen::Vector3f &p) {
        geometry_msgs::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        return point;
    };
    auto corner = [&](const int ix, const int iy, const int iz) {
        return make_point(Eigen::Vector3f(ix == 0 ? box_min.x() : box_max.x(),
                                          iy == 0 ? box_min.y() : box_max.y(),
                                          iz == 0 ? box_min.z() : box_max.z()));
    };
    auto add_edge = [&](visualization_msgs::Marker &marker,
                        const int ax,
                        const int ay,
                        const int az,
                        const int bx,
                        const int by,
                        const int bz) {
        marker.points.push_back(corner(ax, ay, az));
        marker.points.push_back(corner(bx, by, bz));
    };

    visualization_msgs::Marker box_marker;
    box_marker.header.frame_id = "world";
    box_marker.header.stamp = stamp;
    box_marker.ns = "exploration_box";
    box_marker.id = 0;
    box_marker.action = visualization_msgs::Marker::ADD;
    box_marker.type = visualization_msgs::Marker::LINE_LIST;
    box_marker.pose.orientation.w = 1.0;
    box_marker.scale.x = 0.12;
    box_marker.color.r = 0.05F;
    box_marker.color.g = 0.05F;
    box_marker.color.b = 0.05F;
    box_marker.color.a = 0.55F;
    add_edge(box_marker, 0, 0, 0, 1, 0, 0);
    add_edge(box_marker, 1, 0, 0, 1, 1, 0);
    add_edge(box_marker, 1, 1, 0, 0, 1, 0);
    add_edge(box_marker, 0, 1, 0, 0, 0, 0);
    add_edge(box_marker, 0, 0, 1, 1, 0, 1);
    add_edge(box_marker, 1, 0, 1, 1, 1, 1);
    add_edge(box_marker, 1, 1, 1, 0, 1, 1);
    add_edge(box_marker, 0, 1, 1, 0, 0, 1);
    add_edge(box_marker, 0, 0, 0, 0, 0, 1);
    add_edge(box_marker, 1, 0, 0, 1, 0, 1);
    add_edge(box_marker, 1, 1, 0, 1, 1, 1);
    add_edge(box_marker, 0, 1, 0, 0, 1, 1);
    markers.markers.push_back(box_marker);

    visualization_msgs::Marker label;
    label.header.frame_id = "world";
    label.header.stamp = stamp;
    label.ns = "exploration_box_label";
    label.id = 0;
    label.action = visualization_msgs::Marker::ADD;
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.pose.orientation.w = 1.0;
    label.pose.position.x = box_min.x();
    label.pose.position.y = box_min.y();
    label.pose.position.z = box_max.z() + 0.5F;
    label.scale.z = 0.8;
    label.color.r = 0.05F;
    label.color.g = 0.05F;
    label.color.b = 0.05F;
    label.color.a = 0.8F;
    label.text = "exploration box";
    markers.markers.push_back(label);

    exploration_box_pub_.publish(markers);
}

void EpicExplorationManager::onGoalReached(const ExplorationGoal &goal,
                                           const double stamp) {
    (void)stamp;
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal.frontier_id == active_frontier_id_) {
        rememberVisitedNativeViewpoint(goal.position.cast<float>());
        setNativeFrontierDormant(goal.frontier_id, false);
        local_fail_count_by_frontier_.erase(goal.frontier_id);
        topo_route_fail_count_by_frontier_.erase(goal.frontier_id);
        has_last_goal_cost_frame_value_ = false;
        last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
        if (advanceNativeTourTarget(goal.position.cast<float>())) {
            if (ros_ptr_ && cfg_.print_log) {
                ros_ptr_->info(" -- [EPIC Native] Reached current viewpoint; advance native tour cursor={}/{}.",
                               native_tour_cursor_ + 1U,
                               native_tour_targets_.size());
            }
            return;
        }
        clearNativeTourState();
        if (ros_ptr_ && cfg_.print_log) {
            ros_ptr_->info(" -- [EPIC Native] Reached current viewpoint; clear active target and refresh global planning next tick.");
        }
    }
}

void EpicExplorationManager::onGoalFailed(const ExplorationGoal &goal,
                                          const std::string &reason,
                                          const double stamp) {
    (void)stamp;
    std::lock_guard<std::mutex> lock(mutex_);
    if (ros_ptr_) {
        ros_ptr_->warn(" -- [EPIC Native] Goal failed frontier_id={}, reason={}.",
                       goal.frontier_id,
                       reason);
    }
    if (goal.frontier_id == active_frontier_id_) {
        setNativeFrontierDormant(goal.frontier_id, true);
        local_fail_count_by_frontier_.erase(goal.frontier_id);
        topo_route_fail_count_by_frontier_.erase(goal.frontier_id);
        clearNativeTourState();
        has_last_goal_cost_frame_value_ = false;
        last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
    }
}

void EpicExplorationManager::onLocalSegmentCommitted(const ExplorationPlan &plan,
                                                     const double stamp) {
    (void)stamp;
    std::lock_guard<std::mutex> lock(mutex_);
    last_plan_ = plan;
    if (plan.target_frontier_id >= 0) {
        local_fail_count_by_frontier_[plan.target_frontier_id] = 0;
        topo_route_fail_count_by_frontier_.erase(plan.target_frontier_id);
    }
}

void EpicExplorationManager::onLocalSegmentFailed(const ExplorationPlan &plan,
                                                  const std::string &reason,
                                                  const double stamp) {
    (void)stamp;
    std::lock_guard<std::mutex> lock(mutex_);
    if (ros_ptr_) {
        ros_ptr_->warn(" -- [EPIC Native] Local segment failed frontier_id={}, viewpoint_id={}, reason={}.",
                       plan.target_frontier_id,
                       plan.target_viewpoint_id,
                       reason);
    }
    if (isLocalGuideStartUnsafe(reason)) {
        if (plan.target_frontier_id >= 0) {
            local_fail_count_by_frontier_.erase(plan.target_frontier_id);
        }
        if (plan.target_frontier_id == active_frontier_id_) {
            clearNativeTourState();
            has_last_goal_cost_frame_value_ = false;
            last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
        }
        if (ros_ptr_) {
            ros_ptr_->warn(" -- [EPIC Native] Local guide start is unsafe; refresh native tour without marking frontier unreachable.");
        }
        return;
    }
    if (plan.target_frontier_id < 0) {
        clearNativeTourState();
        has_last_goal_cost_frame_value_ = false;
        last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
        return;
    }
    int &fail_count = local_fail_count_by_frontier_[plan.target_frontier_id];
    fail_count = std::min(fail_count + 1,
                          std::max(1, cfg_.max_local_segment_fail_count));
    if (fail_count >= std::max(1, cfg_.max_local_segment_fail_count)) {
        if (ros_ptr_) {
            ros_ptr_->warn(" -- [EPIC Native] Frontier {} deferred after {} local segment failures; keep reachable for future global refresh.",
                           plan.target_frontier_id,
                           fail_count);
        }
        if (plan.target_frontier_id == active_frontier_id_) {
            clearNativeTourState();
            has_last_goal_cost_frame_value_ = false;
            last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
        }
    }
}

void EpicExplorationManager::onLowGain(const ExplorationGoal &goal,
                                       const double actual_gain,
                                       const double stamp) {
    (void)stamp;
    if (ros_ptr_) {
        ros_ptr_->warn(" -- [EPIC Native] Low gain frontier_id={}, gain={:.3f}.",
                       goal.frontier_id,
                       actual_gain);
    }
}

bool EpicExplorationManager::isExplorationFinished() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return exploration_finished_;
}

bool EpicExplorationManager::hasObservation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_ && has_observation_;
}

double EpicExplorationManager::lastObservationStamp() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_observation_stamp_;
}

void EpicExplorationManager::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    clearNativeGoalState();
    exploration_finished_ = false;
    last_native_result_ = -1;
    active_frontier_id_ = -1;
    active_viewpoint_id_ = -1;
    plan_seq_ = 0;
    has_last_goal_cost_frame_value_ = false;
    last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
    global_tour_.clear();
    native_tour_targets_.clear();
    visited_native_viewpoints_.clear();
    native_tour_cursor_ = 0U;
    local_fail_count_by_frontier_.clear();
    topo_route_fail_count_by_frontier_.clear();
    last_plan_ = ExplorationPlan{};
}

void EpicExplorationManager::resetRuntimeState() {
    clearNativeGoalState();
    exploration_finished_ = false;
    last_native_result_ = -1;
    active_frontier_id_ = -1;
    active_viewpoint_id_ = -1;
    plan_seq_ = 0;
    has_last_goal_cost_frame_value_ = false;
    last_goal_cost_frame_value_ = std::numeric_limits<double>::infinity();
    global_tour_.clear();
    native_tour_targets_.clear();
    visited_native_viewpoints_.clear();
    native_tour_cursor_ = 0U;
    local_fail_count_by_frontier_.clear();
    topo_route_fail_count_by_frontier_.clear();
    has_observation_ = false;
    has_last_sensor_position_ = false;
    observation_update_count_ = 0;
    last_observation_cloud_size_ = 0;
    last_empty_cloud_log_stamp_ = -1.0;
    travel_distance_ = 0.0;
    last_observation_stamp_ = -1.0;
    last_plan_ = ExplorationPlan{};
}

super_utils::Vec3f EpicExplorationManager::toVec3d(const Eigen::Vector3f &p) {
    return p.cast<double>();
}

double EpicExplorationManager::pathLength(const super_utils::vec_E<super_utils::Vec3f> &path) {
    double length = 0.0;
    for (std::size_t i = 1U; i < path.size(); ++i) {
        length += (path[i] - path[i - 1U]).norm();
    }
    return length;
}

double EpicExplorationManager::nativeRouteCost(
        const std::vector<Eigen::Vector3f> &path) const {
    if (path.size() < 2U) {
        return std::numeric_limits<double>::infinity();
    }
    double weighted_length = 0.0;
    for (std::size_t i = 1U; i < path.size(); ++i) {
        const Eigen::Vector3f delta = path[i] - path[i - 1U];
        weighted_length += static_cast<double>(delta.norm() +
                                               0.5F * std::fabs(delta.z()));
    }
    return weighted_length / 2.0;
}

void EpicExplorationManager::appendUnique(super_utils::vec_E<super_utils::Vec3f> &path,
                                          const super_utils::Vec3f &point) {
    if (!point.allFinite()) {
        return;
    }
    if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
        path.emplace_back(point);
    }
}

EpicExplorationManager::NativeFrontierStats
EpicExplorationManager::getNativeFrontierStats() const {
    NativeFrontierStats stats;
    if (!frontier_manager_) {
        return stats;
    }

    for (const auto &cluster : frontier_manager_->cluster_list_) {
        if (!cluster) {
            continue;
        }
        ++stats.total;

        const bool in_box =
                (lio_interface_ == nullptr) ||
                lio_interface_->IsInBox(cluster->center_) ||
                lio_interface_->IsInBox(cluster->box_min_) ||
                lio_interface_->IsInBox(cluster->box_max_) ||
                lio_interface_->IsInBox(0.5F * (cluster->box_min_ + cluster->box_max_));
        if (!in_box) {
            continue;
        }
        ++stats.in_search_box;

        if (cluster->is_dormant_) {
            ++stats.dormant;
            continue;
        }
        ++stats.selectable;

        if (cluster->is_reachable_) {
            ++stats.reachable;
        } else {
            ++stats.unreachable;
        }
    }
    return stats;
}

}  // namespace exploration
}  // namespace general_planner
