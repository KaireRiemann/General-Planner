#pragma once

#include "traj_opt/costfunctional/spatialcosts/squared_norm_bound_penalty.hpp"

namespace cost_functional
{
    template <typename VecT>
    inline double accumulateAngularRateBoundPenalty(const VecT &angular_rate,
                                                    const double max_angular_rate_squared,
                                                    const double smooth_eps,
                                                    const double weight,
                                                    VecT &grad_angular_rate,
                                                    double *max_violation = nullptr)
    {
        return accumulateSquaredNormBoundPenalty(angular_rate, max_angular_rate_squared, smooth_eps, weight,
                                                 grad_angular_rate, max_violation);
    }
}// namespace cost_functional