#pragma once

#include <algorithm>
#include <cmath>

#include "traj_opt/swarm_traj.hpp"

namespace cost_functional
{
inline double accumulateSwarmCollisionPenalty(const traj_opt::SwarmPenaltyConfig &config,
                                              const traj_opt::SwarmTrajectories *swarm_trajs,
                                              double current_wall_time,
                                              double local_time,
                                              const Eigen::Vector3d &position,
                                              const Eigen::Vector3d &velocity,
                                              Eigen::Vector3d &grad_position,
                                              double &grad_time,
                                              double *max_violation = nullptr)
{
  if (!config.enable || config.weight <= 0.0 || swarm_trajs == nullptr)
  {
    return 0.0;
  }
  if (config.time_horizon > 1.0e-6 && local_time > config.time_horizon)
  {
    return 0.0;
  }

  const double h_scale = std::max(1.0e-3, config.horizontal_scale);
  const double v_scale = std::max(1.0e-3, config.vertical_scale);
  const double inv_h2 = 1.0 / (h_scale * h_scale);
  const double inv_v2 = 1.0 / (v_scale * v_scale);
  const double activation_scale = std::max(1.0, config.activation_scale);

  double cost = 0.0;
  for (const auto &other : *swarm_trajs)
  {
    if (!other.valid() || other.drone_id == config.self_id)
    {
      continue;
    }
    if (config.stale_timeout > 0.0 &&
        current_wall_time - other.start_wall_time > other.duration + config.stale_timeout)
    {
      continue;
    }

    Eigen::Vector3d other_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d other_velocity = Eigen::Vector3d::Zero();
    if (!other.sample(current_wall_time, local_time, other_position, other_velocity))
    {
      continue;
    }

    const Eigen::Vector3d diff = position - other_position;
    const double ellip_dist2 = diff.x() * diff.x() * inv_h2 +
                               diff.y() * diff.y() * inv_h2 +
                               diff.z() * diff.z() * inv_v2;
    const double clearance = activation_scale *
                             (std::max(0.0, config.clearance) +
                              std::max(other.clearance, config.des_clearance));
    const double err = clearance * clearance - ellip_dist2;
    if (err <= 0.0)
    {
      continue;
    }

    const double err2 = err * err;
    const double err3 = err2 * err;
    cost += config.weight * err3;

    Eigen::Vector3d grad = Eigen::Vector3d(-2.0 * diff.x() * inv_h2,
                                           -2.0 * diff.y() * inv_h2,
                                           -2.0 * diff.z() * inv_v2);
    grad *= config.weight * 3.0 * err2;
    grad_position += grad;
    (void)velocity;
    grad_time += grad.dot(-other_velocity);

    if (max_violation != nullptr)
    {
      const double scaled_dist = std::sqrt(std::max(0.0, ellip_dist2));
      *max_violation = std::max(*max_violation, clearance - scaled_dist);
    }
  }

  return cost;
}
} // namespace cost_functional
