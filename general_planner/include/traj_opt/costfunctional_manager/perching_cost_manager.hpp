#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include "traj_opt/costfunctional/spatialcosts/perching_platform_penalty.hpp"
#include "traj_opt/costfunctional_manager/task_dynamics_penalty.hpp"
#include "traj_opt/tracking_perching_traj_opt.hpp"

namespace cost_functional_manager
{

class PerchingCostManager
{
public:
    void reset(const traj_opt::Config &cfg,
               const general_planner::MapManager::Ptr &map_manager,
               traj_opt::PerchingProblem problem,
               flatness::FlatnessMap *flatness)
    {
        cfg_ = &cfg;
        map_manager_ = map_manager;
        problem_ = std::move(problem);
        frame_.origin = problem_.surface.position;
        frame_.x = problem_.surface.surface_x;
        frame_.y = problem_.surface.surface_y;
        frame_.z = problem_.surface.surface_z;
        frame_.normalize();
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
        const auto frame = frameAt(t_global);
        const Eigen::Vector3d frame_velocity = surfaceVelocityAt(t_global);
        double cost = detail::accumulateDynamicsPenalty(dynamics_,
                                                        position,
                                                        velocity,
                                                        acceleration,
                                                        jerk,
                                                        grad_position,
                                                        grad_velocity,
                                                        grad_acceleration,
                                                        grad_jerk);
        const Eigen::Vector3d platform_grad_before = grad_position;
        cost += cost_functional::accumulatePerchingSE3DiskClearancePenalty(position,
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

        const Eigen::Vector3d relative_height_grad_before = grad_position;
        cost += accumulateRelativeHeightPenalty(frame,
                                                position,
                                                grad_position);
        grad_time += -(grad_position - relative_height_grad_before).dot(frame_velocity);

        Eigen::Vector3d visual_grad_origin = Eigen::Vector3d::Zero();
        cost += accumulatePixelVisualPenalty(frame,
                                             frame_velocity,
                                             position,
                                             velocity,
                                             acceleration,
                                             jerk,
                                             grad_position,
                                             grad_velocity,
                                             grad_acceleration,
                                             grad_jerk,
                                             visual_grad_origin);
        grad_time += visual_grad_origin.dot(frame_velocity);
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
    cost_functional::PerchingPlatformFrame frameAt(double t_global) const
    {
        cost_functional::PerchingPlatformFrame frame;
        const double dt = t_global - problem_.surface.t;
        frame.origin = problem_.surface.position +
                       problem_.surface.velocity * dt +
                       0.5 * problem_.surface.acceleration * dt * dt;
        frame.x = problem_.surface.surface_x;
        frame.y = problem_.surface.surface_y;
        frame.z = problem_.surface.surface_z;
        if (problem_.terminal.rotate_surface_with_yaw_rate &&
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

    Eigen::Vector3d surfaceVelocityAt(double t_global) const
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

        const Eigen::Vector3d world_e3 = Eigen::Vector3d::UnitZ();
        const double relative_z = (position - frame.origin).dot(world_e3);
        double penalty = 0.0;
        double penalty_grad = 0.0;
        double cost = 0.0;
        if (cost_functional::smoothedL1(problem_.relative_z_min - relative_z,
                                        cfg_->smooth_eps,
                                        penalty,
                                        penalty_grad))
        {
            cost += weight * penalty;
            grad_position += -weight * penalty_grad * world_e3;
        }
        if (cost_functional::smoothedL1(relative_z - problem_.relative_z_max,
                                        cfg_->smooth_eps,
                                        penalty,
                                        penalty_grad))
        {
            cost += weight * penalty;
            grad_position += weight * penalty_grad * world_e3;
        }
        return cost;
    }

    double projectionCostOnly(const cost_functional::PerchingPlatformFrame &frame,
                              const Eigen::Vector3d &position,
                              const Eigen::Vector3d &velocity,
                              const Eigen::Vector3d &acceleration,
                              const Eigen::Vector3d &jerk) const
    {
        Eigen::Vector3d unused_p = Eigen::Vector3d::Zero();
        Eigen::Vector3d unused_v = Eigen::Vector3d::Zero();
        Eigen::Vector3d unused_a = Eigen::Vector3d::Zero();
        Eigen::Vector3d unused_j = Eigen::Vector3d::Zero();
        Eigen::Vector3d unused_o = Eigen::Vector3d::Zero();
        return accumulatePixelVisualPenalty(frame,
                                            Eigen::Vector3d::Zero(),
                                            position,
                                            velocity,
                                            acceleration,
                                            jerk,
                                            unused_p,
                                            unused_v,
                                            unused_a,
                                            unused_j,
                                            unused_o,
                                            false);
    }

    double accumulatePixelVisualPenalty(const cost_functional::PerchingPlatformFrame &frame,
                                        const Eigen::Vector3d &,
                                        const Eigen::Vector3d &position,
                                        const Eigen::Vector3d &velocity,
                                        const Eigen::Vector3d &acceleration,
                                        const Eigen::Vector3d &jerk,
                                        Eigen::Vector3d &grad_position,
                                        Eigen::Vector3d &grad_velocity,
                                        Eigen::Vector3d &grad_acceleration,
                                        Eigen::Vector3d &,
                                        Eigen::Vector3d &grad_origin,
                                        bool accumulate_gradient = true) const
    {
        const double weight = problem_.weight_visual_alignment;
        const double max_dist = problem_.visual_activation_distance;
        const double min_dist = std::max(0.0, problem_.visual_min_distance);
        if (weight <= 0.0 || max_dist <= min_dist)
        {
            return 0.0;
        }

        const Eigen::Vector3d rel_world = frame.origin - position;
        const double distance_sq = rel_world.squaredNorm();
        const auto min_gate =
            cost_functional::smoothLogisticActivation(distance_sq - min_dist * min_dist,
                                                       cfg_->smooth_eps);
        const auto max_gate =
            cost_functional::smoothLogisticActivation(max_dist * max_dist - distance_sq,
                                                       cfg_->smooth_eps);
        const double range_gate = min_gate.value * max_gate.value;
        if (range_gate <= 0.0)
        {
            return 0.0;
        }

        if (dynamics_.flatness == nullptr)
        {
            return cost_functional::accumulatePerchingVisualAxisPenalty(position,
                                                                        frame,
                                                                        max_dist,
                                                                        weight,
                                                                        grad_position);
        }

        double thrust = 0.0;
        Eigen::Vector4d quat = Eigen::Vector4d::Zero();
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        dynamics_.flatness->forward(velocity,
                                    acceleration,
                                    jerk,
                                    0.0,
                                    0.0,
                                    thrust,
                                    quat,
                                    omega);
        if (!quat.allFinite() || quat.norm() < 1.0e-6)
        {
            return 0.0;
        }
        quat.normalize();
        const Eigen::Quaterniond q_wb(quat(0), quat(1), quat(2), quat(3));
        const Eigen::Matrix3d R_wb = q_wb.toRotationMatrix();
        const Eigen::Matrix3d R_cb =
            (Eigen::Matrix3d() << 1.0, 0.0, 0.0,
                                  0.0, -1.0, 0.0,
                                  0.0, 0.0, -1.0)
                .finished();
        const Eigen::Vector3d rel_body = R_wb.transpose() * rel_world;
        const Eigen::Vector3d rel_cam = R_cb * rel_body;
        if (rel_cam.z() <= 1.0e-3)
        {
            return 0.0;
        }

        const double fx = std::max(1.0e-6, problem_.visual_fx);
        const double fy = std::max(1.0e-6, problem_.visual_fy);
        const double inv_z = 1.0 / rel_cam.z();
        const double u = fx * rel_cam.x() * inv_z;
        const double v = fy * rel_cam.y() * inv_z;
        const double pixel_error = u * u + v * v;
        const double cost = weight * range_gate * pixel_error;

        if (!accumulate_gradient)
        {
            return cost;
        }

        Eigen::Vector3d grad_cam = Eigen::Vector3d::Zero();
        grad_cam.x() = 2.0 * weight * range_gate * u * fx * inv_z;
        grad_cam.y() = 2.0 * weight * range_gate * v * fy * inv_z;
        grad_cam.z() = -2.0 * weight * range_gate * pixel_error * inv_z;
        const Eigen::Vector3d grad_rel_world = R_wb * R_cb.transpose() * grad_cam;
        grad_position -= grad_rel_world;
        grad_origin += grad_rel_world;

        const double d_gate_d_dist_sq =
            min_gate.derivative * max_gate.value -
            min_gate.value * max_gate.derivative;
        const Eigen::Vector3d grad_range_rel =
            2.0 * weight * pixel_error * d_gate_d_dist_sq * rel_world;
        grad_position -= grad_range_rel;
        grad_origin += grad_range_rel;

        constexpr double kEps = 1.0e-4;
        for (int axis = 0; axis < 3; ++axis)
        {
            Eigen::Vector3d dv = Eigen::Vector3d::Zero();
            dv(axis) = kEps;
            grad_velocity(axis) +=
                (projectionCostOnly(frame, position, velocity + dv, acceleration, jerk) -
                 projectionCostOnly(frame, position, velocity - dv, acceleration, jerk)) /
                (2.0 * kEps);

            Eigen::Vector3d da = Eigen::Vector3d::Zero();
            da(axis) = kEps;
            grad_acceleration(axis) +=
                (projectionCostOnly(frame, position, velocity, acceleration + da, jerk) -
                 projectionCostOnly(frame, position, velocity, acceleration - da, jerk)) /
                (2.0 * kEps);
        }
        return cost;
    }

    const traj_opt::Config *cfg_{nullptr};
    general_planner::MapManager::Ptr map_manager_;
    traj_opt::PerchingProblem problem_;
    cost_functional::PerchingPlatformFrame frame_;
    detail::DynamicsPenaltyConfig dynamics_;
};

} // namespace cost_functional_manager
