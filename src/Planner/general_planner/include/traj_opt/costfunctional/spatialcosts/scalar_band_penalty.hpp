#pragma once

#include "traj_opt/costfunctional/penalty_utils.hpp"

namespace cost_functional
{
    inline double accumulateScalarBandPenalty(const double value,
                                            const double center,
                                            const double squared_radius,
                                            const double smooth_eps,
                                            const double weight,
                                            double &grad_value,
                                            double *max_violation = nullptr)
    {
        if (weight <= 0.0)
        {
            return 0.0;
        }

        const double delta = value - center;
        const double violation = delta * delta - squared_radius;
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

        grad_value += weight * penalty_grad * 2.0 * delta;
        return weight * penalty;
    }

}// namespace cost_functional