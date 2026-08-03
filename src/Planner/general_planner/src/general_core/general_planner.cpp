/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <general_core/general_planner.h>
#include <checker/state2state_checker.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <fmt/format.h>

using namespace general_utils;

namespace general_planner {
    namespace {
        void logCheckResult(const ros_interface::RosInterface::Ptr &ros_ptr,
                            const std::string &context,
                            const checker::CheckResult &result) {
            if (result.severity == checker::Severity::OK || ros_ptr == nullptr) {
                return;
            }
            const std::string msg = fmt::format(" -- [Checker] {} [{}]: {}",
                                                context,
                                                result.code,
                                                result.message);
            if (result.severity == checker::Severity::WARN) {
                ros_ptr->warn(msg);
            } else {
                ros_ptr->error(msg);
            }
        }

        Eigen::Vector3d vectorToVec3d(const std::vector<double> &values,
                                      const Eigen::Vector3d &fallback) {
            if (values.size() < 3) {
                return fallback;
            }
            Eigen::Vector3d out(values[0], values[1], values[2]);
            return out.allFinite() ? out : fallback;
        }

        traj_opt::SwarmPenaltyConfig makeSwarmPenaltyConfig(const Config &cfg) {
            traj_opt::SwarmPenaltyConfig swarm_config;
            swarm_config.enable = cfg.swarm_enable;
            swarm_config.self_id = cfg.swarm_drone_id;
            swarm_config.weight = cfg.swarm_weight;
            swarm_config.clearance = cfg.swarm_clearance;
            swarm_config.des_clearance = cfg.swarm_des_clearance;
            swarm_config.horizontal_scale = cfg.swarm_horizontal_scale;
            swarm_config.vertical_scale = cfg.swarm_vertical_scale;
            swarm_config.activation_scale = cfg.swarm_activation_scale;
            swarm_config.time_horizon = cfg.swarm_time_horizon;
            swarm_config.stale_timeout = cfg.swarm_stale_timeout;
            swarm_config.formation_enable = cfg.swarm_formation_enable;
            swarm_config.formation_weight = cfg.swarm_formation_weight;
            swarm_config.formation_num = cfg.swarm_formation_num;
            swarm_config.formation_offsets = cfg.swarm_formation_offsets;
            swarm_config.formation_start =
                    vectorToVec3d(cfg.swarm_formation_start, Eigen::Vector3d::Zero());
            swarm_config.formation_end =
                    vectorToVec3d(cfg.swarm_formation_end, Eigen::Vector3d::UnitX());
            swarm_config.formation_time_horizon = cfg.swarm_formation_time_horizon;
            swarm_config.formation_stale_timeout = cfg.swarm_formation_stale_timeout;
            return swarm_config;
        }

    }

    class GeneralPlanner::StateToStateBackendContextAdapter final
            : public state2state_task::StateToStateBackendContext {
    public:
        explicit StateToStateBackendContextAdapter(GeneralPlanner &planner)
            : planner_(planner) {}

        void setGoalInfo(const Vec3f &goal,
                         const double goal_yaw,
                         const bool new_goal,
                         const bool goal_valid) override {
            planner_.setGoalInfo(goal, goal_yaw, new_goal, goal_valid);
        }

        void markGoalConsumed() override {
            planner_.markGoalConsumed();
        }

    private:
        GeneralPlanner &planner_;
    };

    class GeneralPlanner::StateToStateSE3BackendRuntimeAdapter final
            : public state2state_task::StateToStateSE3BackendRuntime {
    public:
        explicit StateToStateSE3BackendRuntimeAdapter(GeneralPlanner &planner)
            : planner_(planner) {}

        StatePVAJ makeTaskHeadState(const bool from_rest) override {
            return planner_.makeTaskHeadState(from_rest);
        }

        bool commitSE3AggressiveTrajectory(const Trajectory &pos_traj,
                                           const std::string &traj_ns) override {
            return planner_.commitSE3AggressiveTrajectory(pos_traj, traj_ns);
        }

    private:
        GeneralPlanner &planner_;
    };

    class GeneralPlanner::TrackingBackendRuntimeAdapter final
            : public tracking_task::TrackingBackendRuntime {
    public:
        explicit TrackingBackendRuntimeAdapter(GeneralPlanner &planner)
            : planner_(planner) {}

        RET_CODE optimizeTrackingTask(const traj_opt::DynamicTargetStates &target_prediction,
                                      const bool from_rest) override {
            return planner_.optimizeTrackingTask(target_prediction, from_rest);
        }

    private:
        GeneralPlanner &planner_;
    };

    GeneralPlanner::~GeneralPlanner() = default;

    void GeneralPlanner::setGoalInfo(const Vec3f &goal,
                                     const double goal_yaw,
                                     const bool new_goal,
                                     const bool goal_valid) {
        gi_.goal_p = goal;
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = goal_valid;
    }

    void GeneralPlanner::markGoalConsumed() {
        gi_.new_goal = false;
    }

    GeneralPlanner::GeneralPlanner
            (const std::string &cfg_path,
             const ros_interface::RosInterface::Ptr &ros_ptr,
             const rog_map::ROGMapROS::Ptr &map_ptr
            ) : cfg_(Config(cfg_path)),
                map_manager_(std::make_shared<MapManager>(map_ptr)),
                ros_ptr_(ros_ptr) {

        state2state_backend_context_ =
                std::make_unique<StateToStateBackendContextAdapter>(*this);
        state2state_se3_runtime_ =
                std::make_unique<StateToStateSE3BackendRuntimeAdapter>(*this);
        tracking_backend_runtime_ =
                std::make_unique<TrackingBackendRuntimeAdapter>(*this);

        const auto config_check = checker::checkState2StateConfig(cfg_);
        logCheckResult(ros_ptr_, "state2state config", config_check);
        if (config_check.rejected()) {
            throw std::invalid_argument("state2state config invalid: " + config_check.code +
                                        " " + config_check.message);
        }
        IncrementalTopologyGraph::Config topology_config;
        topology_config.enabled = cfg_.state2state_topology_enable;
        topology_config.dense_known_free =
            cfg_.state2state_topology_construction_mode == "dense_known_free";
        topology_config.unknown_as_free = cfg_.state2state_topology_unknown_as_free;
        topology_config.snapshot_every_update =
            cfg_.state2state_topology_query_enable;
        topology_config.planar_mode = cfg_.state2state_topology_planar_mode;
        const auto topology_map_config = map_manager_->getMapConfig();
        const bool explicit_topology_altitude =
            std::isfinite(cfg_.state2state_topology_navigation_altitude) &&
            cfg_.state2state_topology_navigation_altitude >
                topology_map_config.virtual_ground_height &&
            cfg_.state2state_topology_navigation_altitude <
                topology_map_config.virtual_ceil_height;
        topology_config.navigation_altitude = explicit_topology_altitude
            ? cfg_.state2state_topology_navigation_altitude
            : 0.5 * (topology_map_config.virtual_ground_height +
                     topology_map_config.virtual_ceil_height);
        topology_config.region_size = cfg_.state2state_topology_region_size;
        topology_config.sample_spacing = cfg_.state2state_topology_sample_spacing;
        topology_config.dense_evidence_vertical_tolerance =
            cfg_.state2state_topology_evidence_vertical_tolerance;
        topology_config.min_clearance = std::max(
            cfg_.state2state_topology_min_clearance, cfg_.robot_r);
        topology_config.max_clearance = cfg_.state2state_topology_max_clearance;
        topology_config.connection_radius =
            cfg_.state2state_topology_connection_radius;
        topology_config.dirty_padding = cfg_.state2state_topology_dirty_padding;
        topology_config.bubble_overlap_margin =
            cfg_.state2state_topology_bubble_overlap_margin;
        topology_config.max_nodes_per_region = static_cast<std::size_t>(
            std::max(1, cfg_.state2state_topology_max_nodes_per_region));
        topology_config.max_bubbles_per_region = static_cast<std::size_t>(
            std::max(1, cfg_.state2state_topology_max_bubbles_per_region));
        topology_config.max_neighbors = static_cast<std::size_t>(
            std::max(1, cfg_.state2state_topology_max_neighbors));
        topology_config.max_regions_per_update = static_cast<std::size_t>(
            std::max(1, cfg_.state2state_topology_update_budget));
        topology_config.update_period =
            cfg_.state2state_topology_update_period;
        topology_config.publish_period =
            cfg_.state2state_topology_publish_period;
        map_manager_->configureTopology(topology_config);
        if (topology_config.enabled) {
            ros_ptr_->info(
                " -- [GeneralPlanner] Incremental topology enabled: mode={}, region={:.2f}m, cell={:.2f}m, clearance={:.2f}m, unknown_as_free={}, planar={}, navigation_z={:.3f}m, planning_query={}.",
                topology_config.dense_known_free
                    ? "dense_known_free" : "bubble_topology",
                topology_config.region_size,
                topology_config.sample_spacing,
                topology_config.min_clearance,
                topology_config.unknown_as_free,
                topology_config.planar_mode,
                topology_config.navigation_altitude,
                cfg_.state2state_topology_query_enable);
        }
        ros_ptr_->setResolution(cfg_.resolution);
        ros_ptr_->setVisualizationEn(cfg_.visualization_en);
        tracking_runtime_manager_ = std::make_unique<TrackingRuntimeManager>(cfg_, map_manager_);
        perching_runtime_manager_ = std::make_unique<PerchingRuntimeManager>(cfg_, map_manager_);
        takeoff_runtime_manager_ = std::make_unique<TakeoffRuntimeManager>(cfg_, map_manager_);
        tracking_perching_manager_ = std::make_unique<TrackingPerchingTransitionManager>();
        tracking_to_perching_initializer_ = std::make_unique<TrackingToPerchingInitializer>();
        traj_manager_ = std::make_shared<traj_opt::TrajManager>(cfg_.exp_traj_cfg,
                                                                cfg_.esdf_traj_cfg,
                                                                cfg_.plain_traj_cfg,
                                                                cfg_.back_traj_cfg,
                                                                cfg_.yaw_dot_max,
                                                                cfg_.esdf_safe_distance,
                                                                ros_ptr_,
                                                                map_manager_);
        dynamic_obstacle_layer_ = std::make_unique<DynamicObstacleLayer>();
        DynamicObstacleLayer::Config dynamic_obstacle_cfg;
        dynamic_obstacle_cfg.enable = cfg_.dynamic_obstacle_layer_enable;
        dynamic_obstacle_cfg.ttl = cfg_.dynamic_obstacle_layer_ttl;
        dynamic_obstacle_cfg.voxel_size = cfg_.dynamic_obstacle_layer_voxel_size;
        dynamic_obstacle_cfg.inflation_radius = cfg_.dynamic_obstacle_layer_inflation_radius;
        dynamic_obstacle_cfg.line_check_step = cfg_.dynamic_obstacle_layer_line_check_step;
        dynamic_obstacle_cfg.max_points_per_frame = cfg_.dynamic_obstacle_layer_max_points_per_frame;
        dynamic_obstacle_cfg.local_half_size = cfg_.dynamic_obstacle_layer_local_half_size;
        dynamic_obstacle_layer_->configure(dynamic_obstacle_cfg);
        if (dynamic_obstacle_cfg.enable) {
            ros_ptr_->info(" -- [GeneralPlanner] Dynamic obstacle layer enabled: ttl={:.3f}s, voxel={:.3f}m, inflation={:.3f}m, local_half=({:.1f},{:.1f},{:.1f}).",
                           dynamic_obstacle_cfg.ttl,
                           dynamic_obstacle_cfg.voxel_size,
                           dynamic_obstacle_cfg.inflation_radius,
                           dynamic_obstacle_cfg.local_half_size.x(),
                           dynamic_obstacle_cfg.local_half_size.y(),
                           dynamic_obstacle_cfg.local_half_size.z());
        }
        swarm_trajs_ = std::make_shared<traj_opt::SwarmTrajectories>();
        traj_manager_->setSwarmConfig(makeSwarmPenaltyConfig(cfg_));
        traj_manager_->setSwarmTrajectories(swarm_trajs_);

        const auto rog_map_cfg = map_manager_->getMapConfig();
        astar_ptr_ = std::make_shared<path_search::Astar>(cfg_path, ros_ptr_, map_manager_);
        if (traj_manager_->plain()) {
            traj_manager_->plain()->setLocalAstar(astar_ptr_);
        }
        takeoff_frontend_ = std::make_unique<TakeoffFrontend>(
                makeTakeoffFrontendConfig(), map_manager_, astar_ptr_);
        takeoff_optimizer_ =
                std::make_unique<traj_opt::DynamicTakeoffSnapTrajOpt>(cfg_.esdf_traj_cfg, ros_ptr_);
        takeoff_optimizer_->setMapManager(map_manager_);
        takeoff_optimizer_->setSafeDistance(cfg_.esdf_safe_distance);
        const auto ellipsoid_optimizer_config =
                optimization_utils::EllipsoidOptimizer::makeConfig(cfg_.ellipsoid_optimizer,
                                                                   cfg_.ellipsoid_optimizer_fallback);
        cg_ptr_ = std::make_shared<CorridorGenerator>(ros_ptr_, map_manager_, cfg_.corridor_bound_dis,
                                                      cfg_.corridor_line_max_length,
                                                      cfg_.resolution, rog_map_cfg.virtual_ground_height,
                                                      rog_map_cfg.virtual_ceil_height,
                                                      cfg_.robot_r,
                                                      cfg_.obs_skip_num,
                                                      cfg_.iris_iter_num,
                                                      ellipsoid_optimizer_config);
        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);
        se3_aggressive_manager_ =
                std::make_unique<SE3AggressiveManager>(cfg_, ros_ptr_, map_manager_, astar_ptr_, cg_ptr_);


        time_consuming_.resize(8);

        robot_state_.rcv = false;
        planner_process_start_WT_ = ros_ptr_->getSimTime();
        fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI,
                                                    -1.0,
                                                    -35.0,
                                                    35.0);

        const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);
        astar_ptr_->setFineInfNeighbors(neighbor_step);
    }

}
