#pragma once

#include "traj_opt/costfunctional/penalty_utils.hpp"
#include <algorithm>
#include <cmath>

namespace cost_functional
{
    template <typename PositionT, typename GradT>
    inline double accumulatePVPairCollisionPenalty(const PositionT &position,
                                                   const PositionT &base_point,
                                                   const PositionT &direction,
                                                   const double clearance,
                                                   const double smooth_eps,
                                                   const double weight,
                                                   GradT &grad_position,
                                                   double *max_violation = nullptr)
    {
        if (weight <= 0.0 || clearance <= 0.0)
        {
            return 0.0;
        }

        const double dir_norm = direction.norm();
        if (!std::isfinite(dir_norm) || dir_norm < 1.0e-6)
        {
            return 0.0;
        }

        const PositionT normal = direction / dir_norm;
        const double signed_distance = (position - base_point).dot(normal);
        const double violation = clearance - signed_distance;
        if (max_violation != nullptr)
        {
            *max_violation = std::max(*max_violation, violation);
        }

        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!smoothedL1(violation, smooth_eps, penalty, penalty_grad))
        {
            return 0.0;
        }

        grad_position += -weight * penalty_grad * normal;
        return weight * penalty;
    }
} // namespace cost_functional
