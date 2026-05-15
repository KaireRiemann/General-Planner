#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "traj_opt/costfunctional/penalty_utils.hpp"

namespace cost_functional
{

struct TrackingObservationDistanceConfig
{
    double horizontal_lower{0.0};
    double horizontal_upper{0.0};
    double vertical_lower{0.0};
    double vertical_upper{0.0};
    double weight_near{0.0};
    double weight_far{0.0};
    double weight_vertical{0.0};
    double smooth_eps{0.01};
};

struct TrackingObservationAngleConfig
{
    double weight{0.0};
};

struct TrackingVelocityConfig
{
    double weight_relative{0.0};
    double weight_tangent{0.0};
};

struct TrackingCameraFovConfig
{
    double weight{0.0};
    double horizontal_fov{1.5707963267948966};
    double vertical_fov{1.0471975511965976};
    double range{4.0};
    double angle_clearance{0.0};
    double min_forward{0.05};
    double smooth_eps{0.01};
};

struct TrackingTargetForwardConfig
{
    double weight{0.0};
    double margin{0.15};
    double speed_threshold{0.25};
    double smooth_eps{0.01};
};

struct TrackingPointAttractorConfig
{
    double weight{0.0};
};

struct TrackingVisibleRegionConfig
{
    double theta{0.0};
    double confidence{0.0};
    double angle_clearance{0.0};
    double weight{0.0};
    double smooth_eps{0.01};
};

inline double accumulateTrackingObservationDistancePenalty(const Eigen::Vector3d &position,
                                                           const Eigen::Vector3d &target_position,
                                                           const TrackingObservationDistanceConfig &config,
                                                           Eigen::Vector3d &grad_position,
                                                           Eigen::Vector3d *grad_target_position = nullptr)
{
    double cost = 0.0;
    Eigen::Vector3d local_grad = Eigen::Vector3d::Zero();
    const Eigen::Vector3d rel = position - target_position;
    const double horizontal = rel.head<2>().norm();
    double f = 0.0;
    double df = 0.0;

    if (positivePartCubic(config.horizontal_lower - horizontal, f, df))
    {
        cost += config.weight_near * f;
        if (horizontal > 1.0e-6)
        {
            local_grad.head<2>() -= config.weight_near * df * rel.head<2>() / horizontal;
        }
    }
    if (smoothedL1(horizontal - config.horizontal_upper, config.smooth_eps, f, df))
    {
        cost += config.weight_far * f;
        if (horizontal > 1.0e-6)
        {
            local_grad.head<2>() += config.weight_far * df * rel.head<2>() / horizontal;
        }
    }

    const double vertical = rel.z();
    if (positivePartCubic(config.vertical_lower - vertical, f, df))
    {
        cost += config.weight_vertical * f;
        local_grad.z() -= config.weight_vertical * df;
    }
    if (positivePartCubic(vertical - config.vertical_upper, f, df))
    {
        cost += config.weight_vertical * f;
        local_grad.z() += config.weight_vertical * df;
    }

    grad_position += local_grad;
    if (grad_target_position != nullptr)
    {
        *grad_target_position -= local_grad;
    }
    return cost;
}

inline double accumulateTrackingObservationAnglePenalty(const Eigen::Vector3d &position,
                                                        const double yaw,
                                                        const Eigen::Vector3d &target_position,
                                                        const TrackingObservationAngleConfig &config,
                                                        Eigen::Vector3d &grad_position,
                                                        double &grad_yaw,
                                                        Eigen::Vector3d *grad_target_position = nullptr)
{
    if (config.weight <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector3d dir = target_position - position;
    const double r2 = dir.head<2>().squaredNorm();
    if (r2 < 1.0e-8)
    {
        return 0.0;
    }

    const double desired = std::atan2(dir.y(), dir.x());
    const double err = std::atan2(std::sin(yaw - desired), std::cos(yaw - desired));
    const double grad_err = 2.0 * config.weight * err;
    grad_yaw += grad_err;

    Eigen::Vector3d local_grad = Eigen::Vector3d::Zero();
    local_grad.x() += -grad_err * dir.y() / r2;
    local_grad.y() += grad_err * dir.x() / r2;
    grad_position += local_grad;
    if (grad_target_position != nullptr)
    {
        *grad_target_position -= local_grad;
    }
    return config.weight * err * err;
}

inline double accumulateTrackingCameraFovPenalty(const Eigen::Vector3d &position,
                                                 const double yaw,
                                                 const Eigen::Vector3d &target_position,
                                                 const TrackingCameraFovConfig &config,
                                                 Eigen::Vector3d &grad_position,
                                                 double &grad_yaw,
                                                 Eigen::Vector3d *grad_target_position = nullptr)
{
    if (config.weight <= 0.0)
    {
        return 0.0;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double clearance = std::max(0.0, config.angle_clearance);
    const double half_h = std::clamp(0.5 * config.horizontal_fov - clearance,
                                     kPi / 180.0,
                                     0.5 * kPi - 1.0e-3);
    const double half_v = std::clamp(0.5 * config.vertical_fov - clearance,
                                     kPi / 180.0,
                                     0.5 * kPi - 1.0e-3);
    const double tan_h = std::tan(half_h);
    const double tan_v = std::tan(half_v);
    const double smooth_eps = std::max(1.0e-6, config.smooth_eps);

    const Eigen::Vector3d rel = target_position - position;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const Eigen::Vector3d q(c * rel.x() + s * rel.y(),
                            -s * rel.x() + c * rel.y(),
                            rel.z());

    double cost = 0.0;
    Eigen::Vector3d grad_q = Eigen::Vector3d::Zero();
    auto addViolation = [&](const double violation,
                            const Eigen::Vector3d &grad_violation) {
        double f = 0.0;
        double df = 0.0;
        if (!smoothedL1(violation, smooth_eps, f, df))
        {
            return;
        }
        cost += config.weight * f;
        grad_q += config.weight * df * grad_violation;
    };

    const double min_forward = std::max(1.0e-3, config.min_forward);
    addViolation(min_forward - q.x(), Eigen::Vector3d(-1.0, 0.0, 0.0));

    constexpr double kAbsEps = 1.0e-4;
    const double abs_y = std::sqrt(q.y() * q.y() + kAbsEps * kAbsEps);
    addViolation(abs_y - tan_h * q.x(),
                 Eigen::Vector3d(-tan_h, q.y() / abs_y, 0.0));

    const double abs_z = std::sqrt(q.z() * q.z() + kAbsEps * kAbsEps);
    addViolation(abs_z - tan_v * q.x(),
                 Eigen::Vector3d(-tan_v, 0.0, q.z() / abs_z));

    if (config.range > 0.0)
    {
        const double dist = q.norm();
        if (dist > 1.0e-6)
        {
            addViolation(dist - config.range, q / dist);
        }
    }

    if (grad_q.squaredNorm() <= 0.0)
    {
        return cost;
    }

    const Eigen::Vector3d grad_rel(c * grad_q.x() - s * grad_q.y(),
                                   s * grad_q.x() + c * grad_q.y(),
                                   grad_q.z());
    grad_position -= grad_rel;
    if (grad_target_position != nullptr)
    {
        *grad_target_position += grad_rel;
    }

    grad_yaw += grad_q.x() * q.y() - grad_q.y() * q.x();
    return cost;
}

inline double accumulateTrackingTargetForwardPenalty(const Eigen::Vector3d &position,
                                                     const Eigen::Vector3d &target_position,
                                                     const Eigen::Vector3d &target_velocity,
                                                     const TrackingTargetForwardConfig &config,
                                                     Eigen::Vector3d &grad_position,
                                                     Eigen::Vector3d *grad_target_position = nullptr)
{
    if (config.weight <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector2d target_vel_xy = target_velocity.head<2>();
    const double speed = target_vel_xy.norm();
    if (speed < std::max(0.0, config.speed_threshold))
    {
        return 0.0;
    }

    const Eigen::Vector2d target_dir = target_vel_xy / speed;
    const Eigen::Vector2d rel_xy = (position - target_position).head<2>();
    const double ahead = rel_xy.dot(target_dir);
    const double violation = ahead - std::max(0.0, config.margin);

    double f = 0.0;
    double df = 0.0;
    if (!smoothedL1(violation, std::max(1.0e-6, config.smooth_eps), f, df))
    {
        return 0.0;
    }

    Eigen::Vector3d local_grad = Eigen::Vector3d::Zero();
    local_grad.head<2>() = config.weight * df * target_dir;
    grad_position += local_grad;
    if (grad_target_position != nullptr)
    {
        *grad_target_position -= local_grad;
    }
    return config.weight * f;
}

inline double accumulateTrackingVelocityPenalty(const Eigen::Vector3d &position,
                                                const Eigen::Vector3d &velocity,
                                                const Eigen::Vector3d &target_position,
                                                const Eigen::Vector3d &target_velocity,
                                                const Eigen::Vector3d &target_acceleration,
                                                const TrackingVelocityConfig &config,
                                                Eigen::Vector3d &grad_velocity,
                                                double &grad_time)
{
    double cost = 0.0;
    const Eigen::Vector3d rel_v = velocity - target_velocity;
    if (config.weight_relative > 0.0)
    {
        cost += 0.5 * config.weight_relative * rel_v.squaredNorm();
        grad_velocity += config.weight_relative * rel_v;
        grad_time -= config.weight_relative * rel_v.dot(target_acceleration);
    }

    if (config.weight_tangent > 0.0)
    {
        const Eigen::Vector3d rel = position - target_position;
        const double horizontal = rel.head<2>().norm();
        if (horizontal > 1.0e-6)
        {
            const Eigen::Vector3d tangent(-rel.y() / horizontal, rel.x() / horizontal, 0.0);
            const double tangent_velocity = rel_v.dot(tangent);
            cost += 0.5 * config.weight_tangent * tangent_velocity * tangent_velocity;
            grad_velocity += config.weight_tangent * tangent_velocity * tangent;
        }
    }
    return cost;
}

inline double accumulateTrackingPointAttractorPenalty(const Eigen::Vector3d &position,
                                                      const Eigen::Vector3d &reference_position,
                                                      const Eigen::Vector3d &reference_velocity,
                                                      const TrackingPointAttractorConfig &config,
                                                      Eigen::Vector3d &grad_position,
                                                      double &grad_time)
{
    if (config.weight <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector3d diff = position - reference_position;
    grad_position += config.weight * diff;
    grad_time -= config.weight * diff.dot(reference_velocity);
    return 0.5 * config.weight * diff.squaredNorm();
}

inline double accumulateTrackingVisibleRegionPenalty(const Eigen::Vector3d &position,
                                                     const Eigen::Vector3d &target_position,
                                                     const Eigen::Vector3d &target_velocity,
                                                     const Eigen::Vector3d &visible_reference,
                                                     const Eigen::Vector3d &visible_reference_velocity,
                                                     const TrackingVisibleRegionConfig &config,
                                                     Eigen::Vector3d &grad_position,
                                                     double &grad_time)
{
    if (config.weight <= 0.0 || config.confidence <= 0.0)
    {
        return 0.0;
    }

    const Eigen::Vector3d a = position - target_position;
    const Eigen::Vector3d b = visible_reference - target_position;
    const double norm_a = a.norm();
    const double norm_b = b.norm();
    if (norm_a < 1.0e-6 || norm_b < 1.0e-6)
    {
        return 0.0;
    }

    const double theta_limit = std::max(0.0, config.theta - std::max(0.0, config.angle_clearance));
    const double cos_limit = std::cos(theta_limit);
    const double inner = a.dot(b);
    const double cos_ab = std::clamp(inner / (norm_a * norm_b), -1.0, 1.0);
    const double violation = cos_limit - cos_ab;

    double f = 0.0;
    double df = 0.0;
    if (!smoothedL1(violation, config.smooth_eps, f, df))
    {
        return 0.0;
    }

    const Eigen::Vector3d dcos_da = b / (norm_a * norm_b) -
                                    inner * a / (norm_a * norm_a * norm_a * norm_b);
    const Eigen::Vector3d dcos_db = a / (norm_a * norm_b) -
                                    inner * b / (norm_a * norm_b * norm_b * norm_b);
    const double soft_weight = config.weight * std::clamp(config.confidence, 0.0, 1.0);
    const Eigen::Vector3d local_grad_position = -soft_weight * df * dcos_da;
    const Eigen::Vector3d local_grad_target = soft_weight * df * (dcos_da + dcos_db);
    const Eigen::Vector3d local_grad_visible = -soft_weight * df * dcos_db;

    grad_position += local_grad_position;
    grad_time += local_grad_target.dot(target_velocity) +
                 local_grad_visible.dot(visible_reference_velocity);
    return soft_weight * f;
}

} // namespace cost_functional
