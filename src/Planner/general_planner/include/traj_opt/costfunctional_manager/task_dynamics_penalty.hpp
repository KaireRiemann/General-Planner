#pragma once

#include <cmath>

#include <map_manager/map_manager.hpp>
#include "traj_opt/config.hpp"
#include "traj_opt/costfunctional/spatialcosts/acceleration_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/angular_rate_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/esdf_distance_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/flatness_state.hpp"
#include "traj_opt/costfunctional/spatialcosts/jerk_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/thrust_band_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
#include "utils/geometry/quadrotor_flatness.hpp"

namespace cost_functional_manager
{
namespace detail
{

struct DynamicsPenaltyConfig
{
    const general_planner::MapManager *map{nullptr};
    flatness::FlatnessMap *flatness{nullptr};
    double safe_distance{0.45};
    double smooth_eps{0.01};
    double weight_esdf{0.0};
    double weight_vel{0.0};
    double weight_acc{0.0};
    double weight_jerk{0.0};
    double weight_omega{0.0};
    double weight_thrust{0.0};
    double max_vel{0.0};
    double max_acc{0.0};
    double max_jerk{0.0};
    double max_omega{0.0};
    double min_thrust{0.0};
    double max_thrust{0.0};
};

inline double clampPositive(double value, double fallback)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        return fallback;
    }
    return value;
}

inline double accumulateDynamicsPenalty(const DynamicsPenaltyConfig &config,
                                        const Eigen::Vector3d &position,
                                        const Eigen::Vector3d &velocity,
                                        const Eigen::Vector3d &acceleration,
                                        const Eigen::Vector3d &jerk,
                                        Eigen::Vector3d &grad_position,
                                        Eigen::Vector3d &grad_velocity,
                                        Eigen::Vector3d &grad_acceleration,
                                        Eigen::Vector3d &grad_jerk)
{
    double cost = 0.0;
    cost += cost_functional::accumulateESDFDistancePenalty(config.map,
                                                           position,
                                                           config.safe_distance,
                                                           config.smooth_eps,
                                                           config.weight_esdf,
                                                           grad_position);
    cost += cost_functional::accumulateVelocityBoundPenalty(velocity,
                                                            config.max_vel * config.max_vel,
                                                            config.smooth_eps,
                                                            config.weight_vel,
                                                            grad_velocity);
    cost += cost_functional::accumulateAccelerationBoundPenalty(acceleration,
                                                                config.max_acc * config.max_acc,
                                                                config.smooth_eps,
                                                                config.weight_acc,
                                                                grad_acceleration);
    cost += cost_functional::accumulateJerkBoundPenalty(jerk,
                                                        config.max_jerk * config.max_jerk,
                                                        config.smooth_eps,
                                                        config.weight_jerk,
                                                        grad_jerk);

    if (config.flatness != nullptr && (config.weight_omega > 0.0 || config.weight_thrust > 0.0))
    {
        const auto flatness_state =
            cost_functional::evaluateFlatnessPenaltyState(config.flatness, velocity, acceleration, jerk);
        Eigen::Vector3d grad_omega = Eigen::Vector3d::Zero();
        double grad_thrust = 0.0;

        cost += cost_functional::accumulateAngularRateBoundPenalty(flatness_state.angular_rate,
                                                                   config.max_omega * config.max_omega,
                                                                   config.smooth_eps,
                                                                   config.weight_omega,
                                                                   grad_omega);
        cost += cost_functional::accumulateThrustBandPenalty(flatness_state.thrust,
                                                             config.min_thrust,
                                                             config.max_thrust,
                                                             config.smooth_eps,
                                                             config.weight_thrust,
                                                             grad_thrust);

        Eigen::Vector3d total_gp = grad_position;
        Eigen::Vector3d total_gv = grad_velocity;
        Eigen::Vector3d total_ga = grad_acceleration;
        Eigen::Vector3d total_gj = grad_jerk;
        double grad_psi = 0.0;
        double grad_psi_d = 0.0;
        config.flatness->backward(grad_position,
                                  grad_velocity,
                                  grad_acceleration,
                                  grad_jerk,
                                  grad_thrust,
                                  general_utils::Vec4f::Zero(),
                                  grad_omega,
                                  total_gp,
                                  total_gv,
                                  total_ga,
                                  total_gj,
                                  grad_psi,
                                  grad_psi_d);
        grad_position = total_gp;
        grad_velocity = total_gv;
        grad_acceleration = total_ga;
        grad_jerk = total_gj;
    }

    return cost;
}

inline double accumulateDynamicsPenalty(const DynamicsPenaltyConfig &config,
                                        const Eigen::Vector3d &position,
                                        const Eigen::Vector3d &velocity,
                                        const Eigen::Vector3d &acceleration,
                                        const Eigen::Vector3d &jerk,
                                        const double yaw,
                                        const double yaw_dot,
                                        Eigen::Vector3d &grad_position,
                                        Eigen::Vector3d &grad_velocity,
                                        Eigen::Vector3d &grad_acceleration,
                                        Eigen::Vector3d &grad_jerk,
                                        double &grad_yaw,
                                        double &grad_yaw_dot)
{
    double cost = 0.0;
    cost += cost_functional::accumulateESDFDistancePenalty(config.map,
                                                           position,
                                                           config.safe_distance,
                                                           config.smooth_eps,
                                                           config.weight_esdf,
                                                           grad_position);
    cost += cost_functional::accumulateVelocityBoundPenalty(velocity,
                                                            config.max_vel * config.max_vel,
                                                            config.smooth_eps,
                                                            config.weight_vel,
                                                            grad_velocity);
    cost += cost_functional::accumulateAccelerationBoundPenalty(acceleration,
                                                                config.max_acc * config.max_acc,
                                                                config.smooth_eps,
                                                                config.weight_acc,
                                                                grad_acceleration);
    cost += cost_functional::accumulateJerkBoundPenalty(jerk,
                                                        config.max_jerk * config.max_jerk,
                                                        config.smooth_eps,
                                                        config.weight_jerk,
                                                        grad_jerk);

    if (config.flatness != nullptr && (config.weight_omega > 0.0 || config.weight_thrust > 0.0))
    {
        const auto flatness_state =
            cost_functional::evaluateFlatnessPenaltyState(config.flatness,
                                                          velocity,
                                                          acceleration,
                                                          jerk,
                                                          yaw,
                                                          yaw_dot);
        Eigen::Vector3d grad_omega = Eigen::Vector3d::Zero();
        double grad_thrust = 0.0;

        cost += cost_functional::accumulateAngularRateBoundPenalty(flatness_state.angular_rate,
                                                                   config.max_omega * config.max_omega,
                                                                   config.smooth_eps,
                                                                   config.weight_omega,
                                                                   grad_omega);
        cost += cost_functional::accumulateThrustBandPenalty(flatness_state.thrust,
                                                             config.min_thrust,
                                                             config.max_thrust,
                                                             config.smooth_eps,
                                                             config.weight_thrust,
                                                             grad_thrust);

        Eigen::Vector3d total_gp = grad_position;
        Eigen::Vector3d total_gv = grad_velocity;
        Eigen::Vector3d total_ga = grad_acceleration;
        Eigen::Vector3d total_gj = grad_jerk;
        double grad_psi = 0.0;
        double grad_psi_d = 0.0;
        config.flatness->backward(grad_position,
                                  grad_velocity,
                                  grad_acceleration,
                                  grad_jerk,
                                  grad_thrust,
                                  general_utils::Vec4f::Zero(),
                                  grad_omega,
                                  total_gp,
                                  total_gv,
                                  total_ga,
                                  total_gj,
                                  grad_psi,
                                  grad_psi_d);
        grad_position = total_gp;
        grad_velocity = total_gv;
        grad_acceleration = total_ga;
        grad_jerk = total_gj;
        grad_yaw += grad_psi;
        grad_yaw_dot += grad_psi_d;
    }

    return cost;
}

inline DynamicsPenaltyConfig makeDynamicsPenaltyConfig(const traj_opt::Config &cfg,
                                                       const general_planner::MapManager *map,
                                                       const double safe_distance,
                                                       flatness::FlatnessMap *flatness)
{
    DynamicsPenaltyConfig dynamics;
    dynamics.map = map;
    dynamics.flatness = flatness;
    dynamics.safe_distance = safe_distance;
    dynamics.smooth_eps = cfg.smooth_eps;
    dynamics.weight_esdf = cfg.penna_pos;
    dynamics.weight_vel = cfg.penna_vel;
    dynamics.weight_acc = cfg.penna_acc;
    dynamics.weight_jerk = cfg.penna_jerk;
    dynamics.weight_omega = cfg.penna_omg;
    dynamics.weight_thrust = cfg.penna_thr;
    dynamics.max_vel = clampPositive(cfg.max_vel, 2.0);
    dynamics.max_acc = clampPositive(cfg.max_acc, 2.0);
    dynamics.max_jerk = clampPositive(cfg.max_jerk, 6.0);
    dynamics.max_omega = clampPositive(cfg.max_omg, 4.0);
    dynamics.min_thrust = cfg.min_acc_thr > 0.0 ? cfg.min_acc_thr * cfg.mass : 0.0;
    dynamics.max_thrust = cfg.max_acc_thr > 0.0 ? cfg.max_acc_thr * cfg.mass : 100.0;
    return dynamics;
}

} // namespace detail
} // namespace cost_functional_manager
