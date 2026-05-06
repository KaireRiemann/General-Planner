#pragma once

#include "traj_opt/costfunctional/penalty_utils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace cost_functional
{
    template <typename PairRangeT, typename PositionT, typename GradT>
    inline bool evaluatePVPairDistance(const PairRangeT &pairs,
                                       const PositionT &position,
                                       double &distance,
                                       GradT &grad_distance)
    {
        distance = std::numeric_limits<double>::infinity();
        grad_distance.setZero();

        bool valid = false;
        for (const auto &pair : pairs)
        {
            const double dir_norm = pair.direction.norm();
            if (!std::isfinite(dir_norm) || dir_norm < 1.0e-6)
            {
                continue;
            }

            const PositionT normal = pair.direction / dir_norm;
            const double signed_distance = (position - pair.base_point).dot(normal);
            if (!std::isfinite(signed_distance))
            {
                continue;
            }

            if (signed_distance < distance)
            {
                distance = signed_distance;
                grad_distance = normal;
                valid = true;
            }
        }

        return valid;
    }

    template <typename PairRangeT, typename PositionT, typename GradT>
    inline double accumulatePVPairDistancePenalty(const PairRangeT &pairs,
                                                  const PositionT &position,
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

        double distance = 0.0;
        GradT grad_distance = GradT::Zero();
        if (!evaluatePVPairDistance(pairs, position, distance, grad_distance))
        {
            return 0.0;
        }

        const double violation = clearance - distance;
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

        grad_position += -weight * penalty_grad * grad_distance;
        return weight * penalty;
    }

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
