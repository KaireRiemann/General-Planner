#pragma once

#include "traj_opt/costfunctional/penalty_utils.hpp"
#include <algorithm>

namespace cost_functional
{
    template <typename PolyhedronT, typename Vec3T>
    inline double accumulatePolytopePositionPenalty(const PolyhedronT &poly,
                                                    const Vec3T &position,
                                                    const double smooth_eps,
                                                    const double weight,
                                                    Vec3T &grad_position,
                                                    double *max_violation = nullptr,
                                                    double position_scale = 1.0)
    {
        if (weight <= 0.0)
        {
            return 0.0;
        }

        const double inv_scale = 1.0 / std::max(1.0e-6, position_scale);
        double cost = 0.0;
        for (int k = 0; k < poly.rows(); ++k)
        {
            const Vec3T outer_normal = poly.template block<1, 3>(k, 0).transpose();
            const double raw_violation = outer_normal.dot(position) + poly(k, 3);
            if (max_violation != nullptr)
            {
                *max_violation = std::max(*max_violation, raw_violation);
            }

            const double violation = raw_violation * inv_scale;
            double penalty = 0.0;
            double penalty_grad = 0.0;
            if (smoothedL1(violation, smooth_eps, penalty, penalty_grad))
            {
                grad_position += weight * penalty_grad * inv_scale * outer_normal;
                cost += weight * penalty;
            }
        }
        return cost;
    }

}// namespace cost_functional
