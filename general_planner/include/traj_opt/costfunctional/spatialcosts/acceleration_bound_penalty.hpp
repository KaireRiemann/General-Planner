#pragma once

#include "traj_opt/costfunctional/spatialcosts/squared_norm_bound_penalty.hpp"

namespace cost_functional
{
    template <typename VecT>
    inline double accumulateAccelerationBoundPenalty(const VecT &acceleration,
                                                     const double max_acceleration_squared,
                                                     const double smooth_eps,
                                                     const double weight,
                                                     VecT &grad_acceleration,
                                                     double *max_violation = nullptr)
    {
        return accumulateSquaredNormBoundPenalty(acceleration, max_acceleration_squared, smooth_eps, weight,
                                                 grad_acceleration, max_violation);
    }
}// namespace cost_functional