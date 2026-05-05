#pragma once    

#include <traj_opt/costfunctional/penalty_utils.hpp>
#include <traj_opt/costfunctional/spatialcosts/squared_norm_bound_penalty.hpp>

namespace cost_functional
{
    template <typename VecT>
    inline double accumulateVelocityBoundPenalty(const VecT &velocity,
                                                 const double max_velocity_squared,
                                                 const double smooth_eps,
                                                 const double weight,
                                                 VecT &grad_velocity,
                                                 double *max_violation = nullptr)
    {
        return accumulateSquaredNormBoundPenalty(velocity, max_velocity_squared, smooth_eps, weight, grad_velocity,
                                                 max_violation);
    }
}// namespace cost_functional