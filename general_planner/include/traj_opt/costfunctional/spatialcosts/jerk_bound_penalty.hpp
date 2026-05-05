#pragma once    

#include "traj_opt/costfunctional/spatialcosts/squared_norm_bound_penalty.hpp"

namespace cost_functional
{
    template <typename VecT>
    inline double accumulateJerkBoundPenalty(const VecT &jerk,
                                             const double max_jerk_squared,
                                             const double smooth_eps,
                                             const double weight,
                                             VecT &grad_jerk,
                                             double *max_violation = nullptr)
    {
        return accumulateSquaredNormBoundPenalty(jerk, max_jerk_squared, smooth_eps, weight, grad_jerk, max_violation);
    }
}// namespace cost_functional