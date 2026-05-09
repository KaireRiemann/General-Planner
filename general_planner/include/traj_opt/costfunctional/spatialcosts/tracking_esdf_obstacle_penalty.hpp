#pragma once

#include <algorithm>

#include <Eigen/Core>

#include "traj_opt/costfunctional/penalty_utils.hpp"

namespace cost_functional
{

template <typename MapT, typename Vec3T>
inline double accumulateTrackingESDFObstaclePenalty(const MapT *map,
                                                    const Vec3T &position,
                                                    const double safe_distance,
                                                    const double smooth_eps,
                                                    const double weight,
                                                    Vec3T &grad_position,
                                                    double *max_violation = nullptr)
{
    if (map == nullptr || weight <= 0.0 || safe_distance <= 0.0)
    {
        return 0.0;
    }

    double dist = 0.0;
    Vec3T grad_dist = Vec3T::Zero();
    if (!map->evaluateESDF(position, dist, grad_dist))
    {
        return 0.0;
    }

    const double violation = safe_distance * safe_distance - dist * dist;
    if (max_violation != nullptr)
    {
        *max_violation = std::max(*max_violation, violation);
    }

    double penalty = 0.0;
    double penalty_grad = 0.0;
    if (!positivePartCubic(violation, penalty, penalty_grad))
    {
        return 0.0;
    }

    grad_position += -2.0 * weight * penalty_grad * dist * grad_dist;
    (void)smooth_eps;
    return weight * penalty;
}

} // namespace cost_functional
