/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
*/

#include <general_core/general_planner.h>

namespace general_planner {
    namespace {
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

    void GeneralPlanner::setSwarmTrajectories(const traj_opt::SwarmTrajectories &trajectories) {
        auto snapshot = std::make_shared<traj_opt::SwarmTrajectories>(trajectories);
        std::lock_guard<std::mutex> lock(swarm_traj_mutex_);
        swarm_trajs_ = snapshot;
        if (traj_manager_) {
            traj_manager_->setSwarmTrajectories(swarm_trajs_);
        }
    }

    void GeneralPlanner::setSwarmDroneId(const int drone_id) {
        cfg_.swarm_drone_id = drone_id;
        if (!traj_manager_) {
            return;
        }

        traj_manager_->setSwarmConfig(makeSwarmPenaltyConfig(cfg_));
    }

    void GeneralPlanner::setSwarmFormationReference(const Vec3f &start, const Vec3f &end) {
        if (!start.allFinite() || !end.allFinite()) {
            return;
        }
        cfg_.swarm_formation_start = {start.x(), start.y(), start.z()};
        cfg_.swarm_formation_end = {end.x(), end.y(), end.z()};
        if (traj_manager_) {
            traj_manager_->setSwarmConfig(makeSwarmPenaltyConfig(cfg_));
        }
    }

}
