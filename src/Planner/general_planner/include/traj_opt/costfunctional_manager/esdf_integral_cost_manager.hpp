#pragma once

#include "general_core/map_manager.hpp"
#include "traj_opt/costfunctional/spatialcosts/acceleration_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/angular_rate_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/esdf_distance_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/flatness_state.hpp"
#include "traj_opt/costfunctional/spatialcosts/jerk_bound_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/swarm_collision_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/thrust_band_penalty.hpp"
#include "traj_opt/costfunctional/spatialcosts/velocity_bound_penalty.hpp"
#include "utils/geometry/quadrotor_flatness.hpp"
#include "utils/header/type_utils.hpp"

namespace cost_functional_manager
{
class ESDFIntegralCostManager
{
public:
    const general_planner::MapManager *map = nullptr;
    double safe_distance = 0.0;
    double smooth_eps = 0.0;
    double weight = 0.0;
    super_utils::VecDf magnitude_bounds;
    super_utils::VecDf penalty_weights;
    flatness::FlatnessMap *quadrotor_flatness = nullptr;
    traj_opt::SwarmPenaltyConfig swarm_config;
    traj_opt::SwarmTrajectoriesConstPtr swarm_trajs;
    double swarm_current_wall_time{0.0};

    void reset(const general_planner::MapManager *map_in,
               const double safe_distance_in,
               const double smooth_eps_in,
               const double weight_in,
               const super_utils::VecDf &magnitude_bounds_in,
               const super_utils::VecDf &penalty_weights_in,
               flatness::FlatnessMap *quadrotor_flatness_in,
               const traj_opt::SwarmPenaltyConfig &swarm_config_in,
               const traj_opt::SwarmTrajectoriesConstPtr &swarm_trajs_in,
               double swarm_current_wall_time_in)
    {
        map = map_in;
        safe_distance = safe_distance_in;
        smooth_eps = smooth_eps_in;
        weight = weight_in;
        magnitude_bounds = magnitude_bounds_in;
        penalty_weights = penalty_weights_in;
        quadrotor_flatness = quadrotor_flatness_in;
        swarm_config = swarm_config_in;
        swarm_trajs = swarm_trajs_in;
        swarm_current_wall_time = swarm_current_wall_time_in;
        max_violation_.resize(9);
        max_violation_.setZero();
    }

    double operator()(double /*t*/, double t_global, int /*seg_idx*/, int /*step_in_seg*/,
                      const Eigen::Vector3d &position,
                      const Eigen::Vector3d &velocity,
                      const Eigen::Vector3d &acceleration,
                      const Eigen::Vector3d &jerk,
                      const Eigen::Vector3d &/*snap*/,
                      Eigen::Vector3d &grad_position,
                      Eigen::Vector3d &grad_velocity,
                      Eigen::Vector3d &grad_acceleration,
                      Eigen::Vector3d &grad_jerk,
                      Eigen::Vector3d &/*grad_snap*/,
                      double &grad_time) const
    {
        if (map == nullptr || !quadrotor_flatness || magnitude_bounds.size() < 6 || penalty_weights.size() < 7)
        {
            return 0.0;
        }

        const double weight_vel = penalty_weights(1);
        const double weight_acc = penalty_weights(2);
        const double weight_jer = penalty_weights(3);
        const double weight_omg = penalty_weights(5);
        const double weight_acc_thr = penalty_weights(6);

        double local_cost = 0.0;
        Eigen::Vector3d grad_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_jer = Eigen::Vector3d::Zero();

        local_cost += cost_functional::accumulateESDFDistancePenalty(map,
                                                                     position,
                                                                     safe_distance,
                                                                     smooth_eps,
                                                                     weight,
                                                                     grad_pos,
                                                                     &max_violation_(1));
        local_cost += cost_functional::accumulateVelocityBoundPenalty(velocity,
                                                                      magnitude_bounds(0) * magnitude_bounds(0),
                                                                      smooth_eps,
                                                                      weight_vel,
                                                                      grad_vel,
                                                                      &max_violation_(2));
        local_cost += cost_functional::accumulateAccelerationBoundPenalty(acceleration,
                                                                          magnitude_bounds(1) * magnitude_bounds(1),
                                                                          smooth_eps,
                                                                          weight_acc,
                                                                          grad_acc,
                                                                          &max_violation_(3));
        local_cost += cost_functional::accumulateJerkBoundPenalty(jerk,
                                                                  magnitude_bounds(2) * magnitude_bounds(2),
                                                                  smooth_eps,
                                                                  weight_jer,
                                                                  grad_jer,
                                                                  &max_violation_(4));

        Eigen::Vector3d total_grad_pos = grad_pos;
        Eigen::Vector3d total_grad_vel = grad_vel;
        Eigen::Vector3d total_grad_acc = grad_acc;
        Eigen::Vector3d total_grad_jer = grad_jer;

        local_cost += cost_functional::accumulateSwarmCollisionPenalty(swarm_config,
                                                                       swarm_trajs.get(),
                                                                       swarm_current_wall_time,
                                                                       t_global,
                                                                       position,
                                                                       velocity,
                                                                       grad_pos,
                                                                       grad_time,
                                                                       &max_violation_(8));
        local_cost += cost_functional::accumulateSwarmFormationPenalty(swarm_config,
                                                                       swarm_trajs.get(),
                                                                       swarm_current_wall_time,
                                                                       t_global,
                                                                       position,
                                                                       velocity,
                                                                       grad_pos,
                                                                       grad_time);
        total_grad_pos = grad_pos;

        if (weight_omg > 0.0 || weight_acc_thr > 0.0)
        {
            const auto flatness = cost_functional::evaluateFlatnessPenaltyState(quadrotor_flatness,
                                                                                velocity,
                                                                                acceleration,
                                                                                jerk);
            Eigen::Vector3d grad_omg = Eigen::Vector3d::Zero();
            double grad_thr = 0.0;

            local_cost += cost_functional::accumulateAngularRateBoundPenalty(flatness.angular_rate,
                                                                             magnitude_bounds(3) * magnitude_bounds(3),
                                                                             smooth_eps,
                                                                             weight_omg,
                                                                             grad_omg,
                                                                             &max_violation_(6));
            local_cost += cost_functional::accumulateThrustBandPenalty(flatness.thrust,
                                                                       magnitude_bounds(4),
                                                                       magnitude_bounds(5),
                                                                       smooth_eps,
                                                                       weight_acc_thr,
                                                                       grad_thr,
                                                                       &max_violation_(7));

            double total_grad_psi = 0.0;
            double total_grad_psi_d = 0.0;
            quadrotor_flatness->backward(grad_pos,
                                         grad_vel,
                                         grad_acc,
                                         grad_jer,
                                         grad_thr,
                                         super_utils::Vec4f::Zero(),
                                         grad_omg,
                                         total_grad_pos,
                                         total_grad_vel,
                                         total_grad_acc,
                                         total_grad_jer,
                                         total_grad_psi,
                                         total_grad_psi_d);
        }

        grad_position += total_grad_pos;
        grad_velocity += total_grad_vel;
        grad_acceleration += total_grad_acc;
        grad_jerk += total_grad_jer;
        return local_cost;
    }

    double getMaxViolation() const { return max_violation_.size() > 1 ? max_violation_(1) : 0.0; }
    const super_utils::VecDf &getPenaltyLog() const { return max_violation_; }

private:
    mutable super_utils::VecDf max_violation_{super_utils::VecDf::Zero(9)};
};
} // namespace cost_functional_manager
