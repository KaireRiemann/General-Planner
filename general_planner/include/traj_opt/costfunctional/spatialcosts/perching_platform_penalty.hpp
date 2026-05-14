#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

#include "traj_opt/costfunctional/penalty_utils.hpp"

namespace cost_functional
{

inline Eigen::Vector3d normalizedOr(const Eigen::Vector3d &v,
                                    const Eigen::Vector3d &fallback)
{
    if (!v.allFinite() || v.norm() < 1.0e-6)
    {
        return fallback;
    }
    return v.normalized();
}

struct PerchingPlatformFrame
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    Eigen::Vector3d x{Eigen::Vector3d::UnitX()};
    Eigen::Vector3d y{Eigen::Vector3d::UnitY()};
    Eigen::Vector3d z{Eigen::Vector3d::UnitZ()};

    void normalize()
    {
        z = normalizedOr(z, Eigen::Vector3d::UnitZ());
        x = normalizedOr(x, Eigen::Vector3d::UnitX());
        y = normalizedOr(z.cross(x), Eigen::Vector3d::UnitY());
        x = normalizedOr(y.cross(z), Eigen::Vector3d::UnitX());
    }
};

struct SmoothActivation
{
    double value{0.0};
    double derivative{0.0};
};

inline SmoothActivation smoothLogisticActivation(const double x,
                                                 const double epsilon)
{
    const double eps = std::max(1.0e-9, epsilon);
    if (x <= -eps)
    {
        return {};
    }
    if (x >= eps)
    {
        return {1.0, 0.0};
    }

    const double inv_eps4 = 1.0 / (eps * eps * eps * eps);
    if (x <= 0.0)
    {
        const double xp = x + eps;
        return {0.5 * xp * xp * xp * (eps - x) * inv_eps4,
                xp * xp * (eps - 2.0 * x) * inv_eps4};
    }

    const double xm = x - eps;
    return {0.5 * xm * xm * xm * (eps + x) * inv_eps4 + 1.0,
            xm * xm * (eps + 2.0 * x) * inv_eps4};
}

inline SmoothActivation perchingCollisionActivation(const Eigen::Vector3d &rel,
                                                    const double activation_distance,
                                                    const double smooth_eps)
{
    if (activation_distance <= 0.0 || !rel.allFinite())
    {
        return {};
    }
    const double distance_gate =
        activation_distance * activation_distance - rel.squaredNorm();
    return smoothLogisticActivation(distance_gate, smooth_eps);
}

inline double accumulatePerchingPlatformClearancePenalty(const Eigen::Vector3d &position,
                                                         const PerchingPlatformFrame &frame,
                                                         const double robot_radius,
                                                         const double platform_clearance,
                                                         const double activation_distance,
                                                         const double smooth_eps,
                                                         const double weight,
                                                         Eigen::Vector3d &grad_position)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector3d rel = position - frame.origin;
    const SmoothActivation gate =
        perchingCollisionActivation(rel, activation_distance, smooth_eps);
    if (gate.value <= 0.0)
    {
        return 0.0;
    }

    const double normal_dist = rel.dot(frame.z);
    const double required_clearance = std::max(0.0, robot_radius + platform_clearance);
    double penalty = 0.0;
    double penalty_grad = 0.0;
    if (!smoothedL1(required_clearance - normal_dist, smooth_eps, penalty, penalty_grad))
    {
        return 0.0;
    }

    grad_position += -weight * gate.value * penalty_grad * frame.z;
    grad_position += -2.0 * weight * penalty * gate.derivative * rel;
    return weight * gate.value * penalty;
}

inline double accumulatePerchingSE3DiskClearancePenalty(const Eigen::Vector3d &position,
                                                        const Eigen::Vector3d &acceleration,
                                                        const PerchingPlatformFrame &frame,
                                                        const double robot_l,
                                                        const double robot_radius,
                                                        const double platform_clearance,
                                                        const double activation_distance,
                                                        const double gravity,
                                                        const double smooth_eps,
                                                        const double weight,
                                                        Eigen::Vector3d &grad_position,
                                                        Eigen::Vector3d &grad_acceleration)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector3d rel = position - frame.origin;
    const SmoothActivation gate =
        perchingCollisionActivation(rel, activation_distance, smooth_eps);
    if (gate.value <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector3d thrust_vec =
        acceleration + Eigen::Vector3d(0.0, 0.0, std::max(0.0, gravity));
    const double thrust_norm = thrust_vec.norm();
    if (thrust_norm < 1.0e-6)
    {
        return accumulatePerchingPlatformClearancePenalty(position,
                                                          frame,
                                                          robot_radius,
                                                          platform_clearance,
                                                          activation_distance,
                                                          smooth_eps,
                                                          weight,
                                                          grad_position);
    }

    const Eigen::Vector3d body_z = thrust_vec / thrust_norm;
    const double normal_align = std::clamp(frame.z.dot(body_z), -1.0, 1.0);
    const double tangent_component =
        std::sqrt(std::max(1.0e-8, 1.0 - normal_align * normal_align));
    const double min_disk_clearance =
        rel.dot(frame.z) -
        std::max(0.0, robot_l) * normal_align -
        std::max(0.0, robot_radius) * tangent_component;
    const double violation = std::max(0.0, platform_clearance) - min_disk_clearance;

    double penalty = 0.0;
    double penalty_grad = 0.0;
    if (!smoothedL1(violation, smooth_eps, penalty, penalty_grad))
    {
        return 0.0;
    }

    const double weighted_grad = weight * gate.value * penalty_grad;
    grad_position += weighted_grad * (-frame.z);
    grad_position += -2.0 * weight * penalty * gate.derivative * rel;

    const double d_tangent_d_align = -normal_align / tangent_component;
    const double d_violation_d_align =
        std::max(0.0, robot_l) +
        std::max(0.0, robot_radius) * d_tangent_d_align;
    const Eigen::Vector3d grad_body_z = weighted_grad * d_violation_d_align * frame.z;
    const Eigen::Matrix3d d_body_z_d_acc =
        (Eigen::Matrix3d::Identity() - body_z * body_z.transpose()) / thrust_norm;
    grad_acceleration += d_body_z_d_acc.transpose() * grad_body_z;

    return weight * gate.value * penalty;
}

inline double accumulatePerchingVisualAxisPenalty(const Eigen::Vector3d &position,
                                                  const PerchingPlatformFrame &frame,
                                                  const double activation_distance,
                                                  const double weight,
                                                  Eigen::Vector3d &grad_position)
{
    if (weight <= 0.0 || activation_distance <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector3d rel = position - frame.origin;
    const double normal_dist = rel.dot(frame.z);
    const Eigen::Vector3d axis_point = frame.origin + normal_dist * frame.z;
    const Eigen::Vector3d tangent_error = position - axis_point;
    const double tangent_dist = tangent_error.norm();
    if (tangent_dist > activation_distance)
    {
        return 0.0;
    }

    const double gate = 1.0 - std::clamp(tangent_dist / activation_distance, 0.0, 1.0);
    grad_position += 2.0 * weight * gate * tangent_error;
    return weight * gate * tangent_error.squaredNorm();
}

} // namespace cost_functional
