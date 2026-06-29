#pragma once

#include "traj_opt/costfunctional/penalty_utils.hpp"
#include <algorithm>
#include <general_utils/type_utils.hpp>

namespace cost_functional
{
    template <typename MapT, typename Vec3T>
    inline double accumulateESDFDistancePenalty(const MapT *map,
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

        double occupied_penalty = 0.0;
        const auto inf_grid_type = map->getInfGridType(position);
        if (inf_grid_type == general_utils::GridType::OCCUPIED)
        {
            Vec3T nearest_free = position;
            const double search_radius = std::max(1.0, safe_distance + 0.5);
            if (map->getNearestInfCellNot(general_utils::GridType::OCCUPIED,
                                          position,
                                          nearest_free,
                                          search_radius))
            {
                const Vec3T diff = position - nearest_free;
                const double diff_norm = diff.norm();
                if (diff_norm > 1.0e-6)
                {
                    constexpr double occupied_weight = 300.0;
                    grad_position += weight * occupied_weight * diff;
                    occupied_penalty = 0.5 * weight * occupied_weight * diff.squaredNorm();
                    if (max_violation != nullptr)
                    {
                        *max_violation = std::max(*max_violation, safe_distance);
                    }
                }
            }
        }

        const double violation = safe_distance - dist;
        if (max_violation != nullptr)
        {
            *max_violation = std::max(*max_violation, violation);
        }

        double penalty = 0.0;
        double penalty_grad = 0.0;
        if (!smoothedL1(violation, smooth_eps, penalty, penalty_grad))
        {
            return occupied_penalty;
        }

        grad_position += -weight * penalty_grad * grad_dist;
        return weight * penalty + occupied_penalty;
    }
} // namespace cost_functional
