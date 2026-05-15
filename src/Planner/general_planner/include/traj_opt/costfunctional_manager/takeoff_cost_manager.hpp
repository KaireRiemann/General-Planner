#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "traj_opt/costfunctional/spatialcosts/perching_platform_penalty.hpp"
#include "traj_opt/costfunctional_manager/task_dynamics_penalty.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace cost_functional_manager
{

class TakeoffCostManager
{
public:
    void reset(const traj_opt::Config &cfg,
               const general_planner::MapManager::Ptr &map_manager,
               traj_opt::DynamicTakeoffProblem problem,
               flatness::FlatnessMap *flatness)
    {
        cfg_ = &cfg;
        map_manager_ = map_manager;
        problem_ = std::move(problem);
        dynamics_ = detail::makeDynamicsPenaltyConfig(cfg,
                                                       map_manager_.get(),
                                                       problem_.safe_distance,
                                                       flatness);
    }

    double evaluateIntegral(int,
                            double,
                            double t_global,
                            int,
                            int,
                            const Eigen::Vector3d &position,
                            const Eigen::Vector3d &velocity,
                            const Eigen::Vector3d &acceleration,
                            const Eigen::Vector3d &jerk,
                            Eigen::Vector3d &grad_position,
                            Eigen::Vector3d &grad_velocity,
                            Eigen::Vector3d &grad_acceleration,
                            Eigen::Vector3d &grad_jerk,
                            double &grad_time) const
    {
        double cost = detail::accumulateDynamicsPenalty(dynamics_,
                                                        position,
                                                        velocity,
                                                        acceleration,
                                                        jerk,
                                                        grad_position,
                                                        grad_velocity,
                                                        grad_acceleration,
                                                        grad_jerk);

        const auto frame = frameAt(t_global);
        const Eigen::Vector3d frame_velocity = surfaceVelocityAt(t_global);
        if (t_global > std::max(0.0, problem_.release_contact_time))
        {
            const Eigen::Vector3d platform_grad_before = grad_position;
            cost += cost_functional::accumulatePerchingSE3DiskClearancePenalty(
                position,
                acceleration,
                frame,
                problem_.robot_l,
                problem_.robot_radius,
                problem_.platform_clearance,
                problem_.platform_collision_activation_distance,
                cfg_->grav,
                cfg_->smooth_eps,
                problem_.weight_platform_collision,
                grad_position,
                grad_acceleration);
            grad_time += -(grad_position - platform_grad_before).dot(frame_velocity);

            const Eigen::Vector3d rel_height_grad_before = grad_position;
            cost += accumulateRelativeHeightPenalty(frame, position, grad_position);
            grad_time += -(grad_position - rel_height_grad_before).dot(frame_velocity);
        }
        return cost;
    }

    template <typename SampleBuffer>
    double evaluateSample(const SampleBuffer &,
                          Eigen::Matrix<double, 3, Eigen::Dynamic> &,
                          Eigen::VectorXd &) const
    {
        return 0.0;
    }

private:
    cost_functional::PerchingPlatformFrame frameAt(const double t_global) const
    {
        cost_functional::PerchingPlatformFrame frame;
        const double dt = t_global - problem_.surface.t;
        frame.origin = problem_.surface.position +
                       problem_.surface.velocity * dt +
                       0.5 * problem_.surface.acceleration * dt * dt;
        frame.x = problem_.surface.surface_x;
        frame.y = problem_.surface.surface_y;
        frame.z = problem_.surface.surface_z;
        if (problem_.boundary.rotate_surface_with_yaw_rate &&
            std::abs(problem_.surface.yaw_rate) > 1.0e-9)
        {
            const double theta = problem_.surface.yaw_rate * dt;
            const double c = std::cos(theta);
            const double s = std::sin(theta);
            const Eigen::Matrix3d R =
                (Eigen::Matrix3d() << c, -s, 0.0,
                                      s, c, 0.0,
                                      0.0, 0.0, 1.0)
                    .finished();
            frame.x = R * frame.x;
            frame.y = R * frame.y;
            frame.z = R * frame.z;
        }
        frame.normalize();
        return frame;
    }

    Eigen::Vector3d surfaceVelocityAt(const double t_global) const
    {
        const double dt = t_global - problem_.surface.t;
        return problem_.surface.velocity + problem_.surface.acceleration * dt;
    }

    double accumulateRelativeHeightPenalty(const cost_functional::PerchingPlatformFrame &frame,
                                           const Eigen::Vector3d &position,
                                           Eigen::Vector3d &grad_position) const
    {
        const double weight = problem_.weight_relative_height;
        if (weight <= 0.0)
        {
            return 0.0;
        }

        const Eigen::Vector3d rel = position - frame.origin;
        const double normal_dist = rel.dot(frame.z);
        double penalty = 0.0;
        double penalty_grad = 0.0;
        double cost = 0.0;
        if (cost_functional::smoothedL1(problem_.relative_z_min - normal_dist,
                                        cfg_->smooth_eps,
                                        penalty,
                                        penalty_grad))
        {
            cost += weight * penalty;
            grad_position += -weight * penalty_grad * frame.z;
        }
        if (cost_functional::smoothedL1(normal_dist - problem_.relative_z_max,
                                        cfg_->smooth_eps,
                                        penalty,
                                        penalty_grad))
        {
            cost += weight * penalty;
            grad_position += weight * penalty_grad * frame.z;
        }
        return cost;
    }

private:
    const traj_opt::Config *cfg_{nullptr};
    general_planner::MapManager::Ptr map_manager_;
    traj_opt::DynamicTakeoffProblem problem_;
    detail::DynamicsPenaltyConfig dynamics_;
};

} // namespace cost_functional_manager
