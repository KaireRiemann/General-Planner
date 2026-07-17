#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>

#include <map_manager/map_manager.hpp>
#include "traj_opt/costfunctional/spatialcosts/esdf_distance_penalty.hpp"
#include "traj_opt/costfunctional/penalty_utils.hpp"

namespace cost_functional
{

struct TrackingLineOfSightOcclusionStatus
{
    bool evaluated{false};
    double max_violation{0.0};
    double min_clearance{std::numeric_limits<double>::infinity()};
    double blocked_ratio{0.0};
    double first_block_lambda{1.0};
    double activation{0.0};
};

inline double accumulateTrackingDistancePenalty(const Eigen::Vector3d &position,
                                                const Eigen::Vector3d &target_position,
                                                const double tracking_distance,
                                                const double distance_tolerance,
                                                const double height_offset,
                                                const double height_tolerance,
                                                const double smooth_eps,
                                                const double weight,
                                                Eigen::Vector3d &grad_position,
                                                Eigen::Vector3d *grad_target_position = nullptr)
{
    if (weight <= 0.0)
    {
        return 0.0;
    }

    Eigen::Vector3d local_grad_position = Eigen::Vector3d::Zero();
    const Eigen::Vector3d rel = position - target_position;
    const Eigen::Vector2d rel_xy = rel.head<2>();
    const double horizontal_dist = rel_xy.norm();
    const double min_dist = std::max(0.05, tracking_distance - distance_tolerance);
    const double max_dist = std::max(min_dist + 0.05, tracking_distance + distance_tolerance);

    auto addBand = [&](const double value,
                       const double lower,
                       const double upper,
                       double &grad_value) {
        double cost = 0.0;
        grad_value = 0.0;
        double f = 0.0;
        double df = 0.0;
        if (smoothedL1(lower - value, smooth_eps, f, df))
        {
            cost += weight * f;
            grad_value -= weight * df;
        }
        if (smoothedL1(value - upper, smooth_eps, f, df))
        {
            cost += weight * f;
            grad_value += weight * df;
        }
        return cost;
    };

    double cost = 0.0;
    double grad_h = 0.0;
    cost += addBand(horizontal_dist, min_dist, max_dist, grad_h);
    if (horizontal_dist > 1.0e-6)
    {
        local_grad_position.head<2>() += grad_h * rel_xy / horizontal_dist;
    }

    const double desired_z = target_position.z() + height_offset;
    double grad_z = 0.0;
    cost += addBand(position.z(),
                    desired_z - height_tolerance,
                    desired_z + height_tolerance,
                    grad_z);
    local_grad_position.z() += grad_z;
    grad_position += local_grad_position;
    if (grad_target_position != nullptr)
    {
        *grad_target_position -= local_grad_position;
    }
    return cost;
}

template <typename MapT>
inline double accumulateLineOfSightESDFPenalty(const MapT *map,
                                               const Eigen::Vector3d &position,
                                               const Eigen::Vector3d &target_position,
                                               const double safe_distance,
                                               const double smooth_eps,
                                               const double weight,
                                               const int sample_num,
                                               Eigen::Vector3d &grad_position,
                                               Eigen::Vector3d *grad_target_position = nullptr)
{
    if (map == nullptr || weight <= 0.0 || safe_distance <= 0.0 || sample_num <= 0)
    {
        return 0.0;
    }

    double cost = 0.0;
    const int los_samples = std::max(1, sample_num);
    for (int k = 1; k <= los_samples; ++k)
    {
        const double alpha = static_cast<double>(k) / static_cast<double>(los_samples + 1);
        const Eigen::Vector3d center = position + alpha * (target_position - position);
        Eigen::Vector3d grad_center = Eigen::Vector3d::Zero();
        const double local_cost =
            accumulateESDFDistancePenalty(map,
                                          center,
                                          safe_distance,
                                          smooth_eps,
                                          weight,
                                          grad_center);
        cost += local_cost;
        grad_position += (1.0 - alpha) * grad_center;
        if (grad_target_position != nullptr)
        {
            *grad_target_position += alpha * grad_center;
        }
    }
    return cost;
}

template <typename MapT>
inline double accumulateConicalLineOfSightESDFPenalty(const MapT *map,
                                                      const Eigen::Vector3d &position,
                                                      const Eigen::Vector3d &target_position,
                                                      const double base_clearance,
                                                      const double cone_ratio,
                                                      const double smooth_eps,
                                                      const double weight,
                                                      const int sample_num,
                                                      Eigen::Vector3d &grad_position,
                                                      Eigen::Vector3d *grad_target_position = nullptr)
{
    if (map == nullptr || weight <= 0.0 || sample_num <= 0)
    {
        return 0.0;
    }

    const Eigen::Vector3d rel_pt = position - target_position;
    const double view_dist = rel_pt.norm();
    if (view_dist < 1.0e-6)
    {
        return 0.0;
    }

    double cost = 0.0;
    const int los_samples = std::max(1, sample_num);
    const double clamped_cone_ratio = std::max(0.0, cone_ratio);
    for (int k = 1; k <= los_samples; ++k)
    {
        const double alpha = static_cast<double>(k) / static_cast<double>(los_samples + 1);
        const Eigen::Vector3d center = position + alpha * (target_position - position);
        double dist = 0.0;
        Eigen::Vector3d grad_dist = Eigen::Vector3d::Zero();
        if (!map->evaluateESDF(center, dist, grad_dist))
        {
            continue;
        }

        const double radius = std::max(0.0, base_clearance) +
                              clamped_cone_ratio * (1.0 - alpha) * view_dist;
        const double violation = radius - dist;

        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!smoothedL1(violation, smooth_eps, penalty, penalty_grad))
        {
            continue;
        }

        const Eigen::Vector3d grad_dist_to_pos = rel_pt / view_dist;
        const Eigen::Vector3d grad_radius_pos =
            clamped_cone_ratio * (1.0 - alpha) * grad_dist_to_pos;
        const Eigen::Vector3d grad_radius_target = -grad_radius_pos;
        const Eigen::Vector3d grad_center_pos = (1.0 - alpha) * grad_dist;
        const Eigen::Vector3d grad_center_target = alpha * grad_dist;

        grad_position += weight * penalty_grad *
                         (grad_radius_pos - grad_center_pos);
        if (grad_target_position != nullptr)
        {
            *grad_target_position += weight * penalty_grad *
                                     (grad_radius_target - grad_center_target);
        }
        cost += weight * penalty;
    }
    return cost;
}

template <typename MapT>
inline double accumulateBallLineOfSightESDFPenalty(const MapT *map,
                                                   const Eigen::Vector3d &position,
                                                   const Eigen::Vector3d &target_position,
                                                   const double base_clearance,
                                                   const double fov_radius_ratio,
                                                   const double smooth_eps,
                                                   const double weight,
                                                   const int sample_num,
                                                   Eigen::Vector3d &grad_position,
                                                   Eigen::Vector3d *grad_target_position = nullptr)
{
    if (map == nullptr || weight <= 0.0 || sample_num <= 0)
    {
        return 0.0;
    }

    const Eigen::Vector3d rel_pt = position - target_position;
    const double view_dist = rel_pt.norm();
    if (view_dist < 1.0e-6)
    {
        return 0.0;
    }

    double cost = 0.0;
    const int los_samples = std::max(1, sample_num);
    const double ratio = std::max(0.0, fov_radius_ratio);
    for (int k = 1; k <= los_samples; ++k)
    {
        const double lambda = static_cast<double>(k) / static_cast<double>(los_samples + 1);
        const Eigen::Vector3d center = position + lambda * (target_position - position);
        double dist = 0.0;
        Eigen::Vector3d grad_dist = Eigen::Vector3d::Zero();
        if (!map->evaluateESDF(center, dist, grad_dist))
        {
            continue;
        }

        const double radius = std::max(0.0, base_clearance) + ratio * lambda * view_dist;
        const double violation = radius - dist;

        double penalty = 0.0;
        double penalty_grad = 0.0;
        (void)smooth_eps;
        if (!positivePartCubic(violation, penalty, penalty_grad))
        {
            continue;
        }

        const Eigen::Vector3d grad_radius_pos = ratio * lambda * rel_pt / view_dist;
        const Eigen::Vector3d grad_radius_target = -grad_radius_pos;
        const Eigen::Vector3d grad_center_pos = (1.0 - lambda) * grad_dist;
        const Eigen::Vector3d grad_center_target = lambda * grad_dist;

        grad_position += weight * penalty_grad *
                         (grad_radius_pos - grad_center_pos);
        if (grad_target_position != nullptr)
        {
            *grad_target_position += weight * penalty_grad *
                                     (grad_radius_target - grad_center_target);
        }
        cost += weight * penalty;
    }
    return cost;
}

template <typename MapT>
inline TrackingLineOfSightOcclusionStatus evaluateBallLineOfSightOcclusionStatus(
    const MapT *map,
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &target_position,
    const double base_clearance,
    const double fov_radius_ratio,
    const double activation_distance,
    const int sample_num)
{
    TrackingLineOfSightOcclusionStatus status;
    if (map == nullptr || sample_num <= 0)
    {
        return status;
    }

    const Eigen::Vector3d rel_pt = position - target_position;
    const double view_dist = rel_pt.norm();
    if (view_dist < 1.0e-6)
    {
        return status;
    }

    const int los_samples = std::max(1, sample_num);
    const double ratio = std::max(0.0, fov_radius_ratio);
    const double active_dist = std::max(1.0e-3, activation_distance);
    int evaluated_count = 0;
    int blocked_count = 0;
    for (int k = 1; k <= los_samples; ++k)
    {
        const double lambda = static_cast<double>(k) / static_cast<double>(los_samples + 1);
        const Eigen::Vector3d center = position + lambda * (target_position - position);
        double dist = 0.0;
        Eigen::Vector3d grad_dist = Eigen::Vector3d::Zero();
        if (!map->evaluateESDF(center, dist, grad_dist))
        {
            continue;
        }

        ++evaluated_count;
        const double radius = std::max(0.0, base_clearance) + ratio * lambda * view_dist;
        const double clearance = dist - radius;
        const double violation = -clearance;
        status.min_clearance = std::min(status.min_clearance, clearance);
        status.max_violation = std::max(status.max_violation, violation);
        if (violation > 0.0)
        {
            ++blocked_count;
            status.first_block_lambda = std::min(status.first_block_lambda, lambda);
        }
    }

    if (evaluated_count <= 0)
    {
        return status;
    }

    status.evaluated = true;
    status.blocked_ratio =
        static_cast<double>(blocked_count) / static_cast<double>(evaluated_count);
    status.activation =
        std::clamp(std::max(status.max_violation, 0.0) / active_dist +
                       0.5 * status.blocked_ratio,
                   0.0,
                   1.0);
    return status;
}

inline double yawReferenceToTarget(const Eigen::Vector3d &position,
                                   const Eigen::Vector3d &target_position)
{
    const Eigen::Vector3d rel = target_position - position;
    return std::atan2(rel.y(), rel.x());
}

} // namespace cost_functional
