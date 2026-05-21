#include "exploration/epic_exploration_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <boost/filesystem.hpp>
#include <frontier_manager/frontier_manager.h>
#include <lidar_map/lidar_map.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pointcloud_topo/graph.h>
#include <pointcloud_topo/graph_visualizer.hpp>
#include <sensor_msgs/PointCloud2.h>
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
    setParamDefault(nh_, "box_num", 1);
    setParamDefault(nh_, "box_0/down", vecToStd(bbox_min));
    setParamDefault(nh_, "box_0/up", vecToStd(bbox_max));
    setParamDefault(nh_, "dead_area_num", 0);

    const double sensor_range = std::max(1.0, vp.sensor_range);
    const double vertical_half_fov = std::max(5.0, 0.5 * vp.vertical_fov_deg);
    setParamDefault(nh_, "lidar_perception/max_ray_length", sensor_range);
    setParamDefault(nh_, "lidar_perception/fov_up", vertical_half_fov);
    setParamDefault(nh_, "lidar_perception/fov_down", -vertical_half_fov);
    setParamDefault(nh_, "lidar_perception/fov_viewpoint_up", vertical_half_fov);
    setParamDefault(nh_, "lidar_perception/fov_viewpoint_down", -vertical_half_fov);
    setParamDefault(nh_, "lidar_perception/lidar_pitch", 0.0);

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
                 std::max(0.01, topo.local_edge_astar_timeout));
    nh_.setParam("parallel_astar/insert_node_timeout",
                 std::max(0.01, topo.local_edge_astar_timeout));
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

    setParamDefault(nh_, "FrontierManager/cell_size",
                    std::max(0.05, obs.resolution));
    setParamDefault(nh_, "FrontierManager/noise_cell_range", 1);
    setParamDefault(nh_, "FrontierManager/good_observation_direction_score",
                    obs.good_observation_direction_score);
    setParamDefault(nh_, "FrontierManager/good_observation_trust_length",
                    obs.good_observation_trust_length);
    setParamDefault(nh_, "FrontierManager/good_observation_force_trust_length",
                    obs.good_observation_force_trust_length);
    setParamDefault(nh_, "FrontierManager/update_length",
                    std::max(1.0, obs.max_observation_distance));
    setParamDefault(nh_, "FrontierManager/view_frt", true);
    setParamDefault(nh_, "FrontierManager/view_cluster", true);
    setParamDefault(nh_, "FrontierManager/cluster_min_radius",
                    std::max(0.1, obs.frontier_cluster_radius * 0.5));
    setParamDefault(nh_, "FrontierManager/cluster_min_size",
                    std::max(0.1, obs.frontier_cluster_radius));
    setParamDefault(nh_, "FrontierManager/cluster_max_size",
                    std::max(0.2, obs.frontier_cluster_radius * 2.0));
    setParamDefault(nh_, "FrontierManager/cluster_direction_radius",
                    std::max(0.1, obs.frontier_cluster_radius));
    setParamDefault(nh_, "FrontierManager/cluster_minmum_point_num",
                    std::max(1, obs.min_frontier_cluster_size));

    setParamDefault(nh_, "ViewpointManager/sample_pillar_height_layer_num",
                    std::max(1, vp.height_samples));
    setParamDefault(nh_, "ViewpointManager/sample_pillar_radius_layer_num",
                    std::max(1, vp.radius_samples));
    setParamDefault(nh_, "ViewpointManager/sample_pillar_circle_sample_num",
                    std::max(4, vp.yaw_samples));
    const double height_span = std::max(0.1, vp.height_step *
                                             std::max(1, vp.height_samples - 1));
    setParamDefault(nh_, "ViewpointManager/sample_pillar_max_height",
                    0.5 * height_span);
    setParamDefault(nh_, "ViewpointManager/sample_pillar_min_height",
                    -0.5 * height_span);
    setParamDefault(nh_, "ViewpointManager/sample_pillar_min_radius",
                    std::max(0.1, vp.min_distance));
    setParamDefault(nh_, "ViewpointManager/sample_pillar_max_radius",
                    std::max(vp.min_distance + 0.1, vp.max_distance));
    setParamDefault(nh_, "ViewpointManager/consider_range",
                    std::max(1, vp.max_cells_per_gain_eval));
    setParamDefault(nh_, "ViewpointManager/global_recluster_size",
                    std::max(1, vp.max_viewpoint_clusters));
    setParamDefault(nh_, "ViewpointManager/local_tsp_size",
                    std::max(1, cfg_.global_guidance.max_frontiers_in_tour));
    setParamDefault(nh_, "ViewpointManager/view_direction_range",
                    120.0);
    nh_.setParam("ViewpointManager/min_clearance",
                 std::max(0.05, vp.safe_distance));
    nh_.setParam("ViewpointManager/topo_reachability_timeout",
                 std::max(1.0e-3, vp.topo_reachability_timeout));
    nh_.setParam("ViewpointManager/reachability_only_raycast", false);
    nh_.setParam("lidar_perception/voxel_leaf_size",
                 std::max(0.01, obs.resolution * 0.4));

    setParamDefault(nh_, "max_traj_len",
                    std::max(2.0, cfg_.local_guide.planning_horizon));
    setParamDefault(nh_, "DilateRadiusSoft", std::max(0.05, cfg_.local_guide.safe_distance));
    setParamDefault(nh_, "DilateRadiusHard", std::max(0.05, cfg_.local_goal_safe_distance));
    setParamDefault(nh_, "MaxVelMag", 3.0);
    setParamDefault(nh_, "maxBdrMag", 3.0);
    setParamDefault(nh_, "MaxTiltAngle", 0.8);
    setParamDefault(nh_, "MinThrust", 0.0);
    setParamDefault(nh_, "MaxThrust", 20.0);
    setParamDefault(nh_, "VehicleMass", 1.0);
    setParamDefault(nh_, "GravAcc", 9.81);
    setParamDefault(nh_, "HorizDrag", 0.0);
    setParamDefault(nh_, "VertDrag", 0.0);
    setParamDefault(nh_, "ParasDrag", 0.0);
    setParamDefault(nh_, "SpeedEps", 0.01);
    setParamDefault(nh_, "WeightT", 1.0);
    setParamDefault(nh_, "WeightSafeT", 1.0);
    setParamDefault(nh_, "ChiVec", std::vector<double>{1.0, 1.0, 1.0});
    setParamDefault(nh_, "SmoothingEps", 0.01);
    setParamDefault(nh_, "IntegralIntervs", 8);
    setParamDefault(nh_, "RelCostTol", 1.0e-5);
    setParamDefault(nh_, "MaxCorridorSize", 7.0);
    setParamDefault(nh_, "yaw_rho_vis", 1.0);
    setParamDefault(nh_, "yaw_max_vel", 1.0);
    setParamDefault(nh_, "yaw_time_fwd", 1.0);

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
    frontier_manager_ = std::make_shared<FrontierManager>();
    graph_ = std::make_shared<TopoGraph>();
    graph_visualizer_ = std::make_shared<GraphVisualizer>();
    local_guide_builder_ = std::make_unique<LocalGuideBuilder>(
            cfg_.local_guide, map_manager_, astar_);

    graph_->init(nh_, lio_interface_, parallel_path_finder_);
    parallel_path_finder_->init(nh_, lio_interface_);
    graph_visualizer_->init(nh_);
    frontier_manager_->init(nh_, lio_interface_, graph_);

    initialized_ = true;
    resetRuntimeState();
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
    plan = ExplorationPlan{};
    std::lock_guard<std::mutex> lock(mutex_);

    if (!cfg_.enable) {
        plan.reason = "exploration disabled";
        plan.goal.reason = plan.reason;
        return false;
    }
    if (!initialized_) {
        plan.reason = "native EPIC modules are not ready";
        plan.goal.reason = plan.reason;
        return false;
    }
    if (!robot.rcv) {
        plan.reason = "no odom";
        plan.goal.reason = plan.reason;
        return false;
    }
    if (!has_observation_) {
        plan.reason = "native EPIC has no cloud observation";
        plan.goal.reason = plan.reason;
        return false;
    }

    if (!updateNativeGlobalPlan(robot, current_yaw, plan)) {
        last_plan_ = plan;
        return false;
    }
    if (!buildNativeGuide(robot, current_yaw, plan)) {
        last_plan_ = plan;
        return false;
    }

    plan.valid = true;
    plan.no_frontier = false;
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

bool EpicExplorationManager::updateNativeGlobalPlan(const rog_map::RobotState &robot,
                                                    const double current_yaw,
                                                    ExplorationPlan &plan) {
    plan.no_frontier = false;
    plan.reason.clear();
    exploration_finished_ = false;

    if (!frontier_manager_ || !graph_ || !parallel_path_finder_) {
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
        plan.goal.reason = plan.reason;
        visualizeNativeState();
        return false;
    }

    std::vector<TopoNode::Ptr> viewpoints;
    Eigen::Vector3f center_pose = pos;
    frontier_manager_->generateTSPViewpoints(center_pose, viewpoints);
    if (viewpoints.empty()) {
        const bool has_frontier = !frontier_manager_->cluster_list_.empty();
        exploration_finished_ = !has_frontier;
        plan.no_frontier = !has_frontier;
        plan.reason = has_frontier
                      ? "native EPIC has no reachable frontier viewpoint"
                      : "native EPIC no frontier";
        plan.goal.reason = plan.reason;
        visualizeNativeState();
        return false;
    }

    graph_->insertNodes(viewpoints, false);

    struct Candidate {
        TopoNode::Ptr node;
        super_utils::vec_E<super_utils::Vec3f> odom_path;
        double odom_cost{std::numeric_limits<double>::infinity()};
        int frontier_id{-1};
    };
    std::vector<Candidate> candidates;
    candidates.reserve(viewpoints.size());
    const double graph_timeout = std::max(1.0e-3, cfg_.topo_graph.local_edge_astar_timeout);
    for (const auto &viewpoint : viewpoints) {
        if (!viewpoint || viewpoint->neighbors_.empty()) {
            continue;
        }
        Candidate candidate;
        candidate.node = viewpoint;
        candidate.frontier_id = frontierIdForViewpoint(toVec3d(viewpoint->center_));
        if (!routeBetweenNativeNodes(graph_->odom_node_,
                                     viewpoint,
                                     graph_timeout,
                                     candidate.odom_path,
                                     candidate.odom_cost)) {
            continue;
        }
        if (candidate.odom_path.size() < 2U ||
            !std::isfinite(candidate.odom_cost) ||
            candidate.odom_cost > 1.0e3) {
            continue;
        }
        candidates.push_back(std::move(candidate));
    }

    if (candidates.empty()) {
        graph_->removeNodes(viewpoints);
        plan.reason = "native EPIC topo graph could not route to any frontier viewpoint";
        plan.goal.reason = plan.reason;
        visualizeNativeState();
        return false;
    }

    const int dim = static_cast<int>(candidates.size()) + 1;
    Eigen::MatrixXd cost_matrix(dim, dim);
    cost_matrix.setZero();
    for (int i = 1; i < dim; ++i) {
        const auto &candidate = candidates[static_cast<std::size_t>(i - 1)];
        cost_matrix(0, i) =
                candidate.odom_cost +
                cfg_.global_guidance.weight_revisit *
                static_cast<double>(local_fail_count_by_frontier_[candidate.frontier_id]);
    }

    constexpr double kUnreachableCost = 2.0e3;
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
            cost_matrix(i, j) = pair_cost;
        }
    }

    // EPIC uses an ATSP tour and makes the return-to-start edge expensive; the
    // small distance bias encourages real outward exploration rather than a
    // closed loop around the robot.
    for (int i = 1; i < dim; ++i) {
        cost_matrix(i, 0) =
                kUnreachableCost -
                candidates[static_cast<std::size_t>(i - 1)].odom_cost * 0.2;
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
        plan.reason = "native EPIC global tour solver failed";
        plan.goal.reason = plan.reason;
        visualizeNativeState();
        return false;
    }
    normalizeTourOrder(tour_order, dim);

    int selected_candidate_idx = -1;
    for (const int node_idx : tour_order) {
        if (node_idx > 0 && node_idx < dim) {
            selected_candidate_idx = node_idx - 1;
            break;
        }
    }
    if (selected_candidate_idx < 0) {
        selected_candidate_idx = static_cast<int>(
                std::min_element(candidates.begin(), candidates.end(),
                                 [](const Candidate &a, const Candidate &b) {
                                     return a.odom_cost < b.odom_cost;
                                 }) - candidates.begin());
    }

    Candidate selected = candidates[static_cast<std::size_t>(selected_candidate_idx)];
    if (selected.frontier_id < 0) {
        selected.frontier_id = plan_seq_ + 1;
    }
    const int viewpoint_id = plan_seq_ + 1;

    std::vector<Eigen::Vector3f> global_tour;
    global_tour.reserve(tour_order.size() + 1U);
    if (tour_order.empty() || tour_order.front() != 0) {
        global_tour.push_back(pos);
    }
    for (const int node_idx : tour_order) {
        if (node_idx == 0) {
            global_tour.push_back(pos);
        } else if (node_idx > 0 && node_idx < dim) {
            const auto &candidate = candidates[static_cast<std::size_t>(node_idx - 1)];
            if (candidate.node) {
                global_tour.push_back(candidate.node->center_);
            }
        }
    }
    if (graph_visualizer_) {
        graph_visualizer_->vizTour(global_tour, VizColor::RED, "global");
    }

    GlobalRoute route;
    route.valid = true;
    route.target_frontier_id = selected.frontier_id;
    route.path = std::move(selected.odom_path);
    route.cost = selected.odom_cost;
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

    graph_->removeNodes(viewpoints);
    return true;
}

bool EpicExplorationManager::buildNativeGuide(const rog_map::RobotState &robot,
                                              const double current_yaw,
                                              ExplorationPlan &plan) {
    if (!parallel_path_finder_ || !local_guide_builder_) {
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
    cost = pathLength(path);
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
    if (!cfg_.print_log || !frontier_manager_) {
        return;
    }
    frontier_manager_->viz_pocc();
    frontier_manager_->visfrtcluster();
    if (graph_visualizer_ && graph_) {
        graph_visualizer_->vizBox(graph_);
        graph_visualizer_->vizGraph(graph_);
    }
}

void EpicExplorationManager::onGoalReached(const ExplorationGoal &goal,
                                           const double stamp) {
    (void)stamp;
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal.frontier_id == active_frontier_id_) {
        setNativeFrontierDormant(goal.frontier_id, false);
        local_fail_count_by_frontier_.erase(goal.frontier_id);
        active_frontier_id_ = -1;
        active_viewpoint_id_ = -1;
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
        active_frontier_id_ = -1;
        active_viewpoint_id_ = -1;
    }
}

void EpicExplorationManager::onLocalSegmentCommitted(const ExplorationPlan &plan,
                                                     const double stamp) {
    (void)stamp;
    std::lock_guard<std::mutex> lock(mutex_);
    last_plan_ = plan;
    if (plan.target_frontier_id >= 0) {
        local_fail_count_by_frontier_[plan.target_frontier_id] = 0;
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
    const int fail_count = ++local_fail_count_by_frontier_[plan.target_frontier_id];
    if (fail_count >= std::max(1, cfg_.max_local_segment_fail_count)) {
        setNativeFrontierDormant(plan.target_frontier_id, true);
        if (ros_ptr_) {
            ros_ptr_->warn(" -- [EPIC Native] Frontier {} marked unreachable after {} local segment failures.",
                           plan.target_frontier_id,
                           fail_count);
        }
        if (plan.target_frontier_id == active_frontier_id_) {
            active_frontier_id_ = -1;
            active_viewpoint_id_ = -1;
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
    exploration_finished_ = false;
    last_native_result_ = -1;
    active_frontier_id_ = -1;
    active_viewpoint_id_ = -1;
    plan_seq_ = 0;
    local_fail_count_by_frontier_.clear();
    last_plan_ = ExplorationPlan{};
}

void EpicExplorationManager::resetRuntimeState() {
    exploration_finished_ = false;
    last_native_result_ = -1;
    active_frontier_id_ = -1;
    active_viewpoint_id_ = -1;
    plan_seq_ = 0;
    local_fail_count_by_frontier_.clear();
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

void EpicExplorationManager::appendUnique(super_utils::vec_E<super_utils::Vec3f> &path,
                                          const super_utils::Vec3f &point) {
    if (!point.allFinite()) {
        return;
    }
    if (path.empty() || (path.back() - point).norm() > 1.0e-4) {
        path.emplace_back(point);
    }
}

}  // namespace exploration
}  // namespace general_planner
