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

inline bool getSwarmFormationOffset(const traj_opt::SwarmPenaltyConfig &config,
                                    int drone_id,
                                    Eigen::Vector3d &offset)
{
  if (drone_id < 0)
  {
    return false;
  }
  const size_t base = static_cast<size_t>(drone_id) * 3;
  if (config.formation_offsets.size() < base + 3)
  {
    return false;
  }
  offset = Eigen::Vector3d(config.formation_offsets[base],
                           config.formation_offsets[base + 1],
                           config.formation_offsets[base + 2]);
  return offset.allFinite();
}

inline double accumulateSwarmFormationPenalty(const traj_opt::SwarmPenaltyConfig &config,
                                              const traj_opt::SwarmTrajectories *swarm_trajs,
                                              double current_wall_time,
                                              double local_time,
                                              const Eigen::Vector3d &position,
                                              const Eigen::Vector3d &velocity,
                                              Eigen::Vector3d &grad_position,
                                              double &grad_time)
{
  if (!config.enable || !config.formation_enable || config.formation_weight <= 0.0 ||
      swarm_trajs == nullptr || config.self_id < 0)
  {
    return 0.0;
  }
  if (config.formation_time_horizon > 1.0e-6 && local_time > config.formation_time_horizon)
  {
    return 0.0;
  }

  const int configured_num = config.formation_num > 0
                                 ? config.formation_num
                                 : static_cast<int>(config.formation_offsets.size() / 3);
  if (configured_num <= 1 || config.self_id >= configured_num ||
      static_cast<int>(config.formation_offsets.size()) < 3 * configured_num)
  {
    return 0.0;
  }

  Eigen::Vector3d self_offset = Eigen::Vector3d::Zero();
  if (!getSwarmFormationOffset(config, config.self_id, self_offset))
  {
    return 0.0;
  }

  const Eigen::Vector3d direction = config.formation_end - config.formation_start;
  if (!direction.allFinite() || direction.squaredNorm() < 1.0e-12)
  {
    return 0.0;
  }
  const Eigen::Vector3d axis = direction.normalized();
  Eigen::Vector3d horizontal_axis(axis.x(), axis.y(), 0.0);
  if (horizontal_axis.squaredNorm() < 1.0e-12)
  {
    horizontal_axis = Eigen::Vector3d::UnitX();
  }
  else
  {
    horizontal_axis.normalize();
  }
  const Eigen::Vector3d lateral_axis(-horizontal_axis.y(), horizontal_axis.x(), 0.0);
  const Eigen::Vector3d target_progress_axis(horizontal_axis.x(), horizontal_axis.y(), axis.z());

  double cost = 0.0;

  const double self_line_progress = (position - config.formation_start).dot(horizontal_axis);
  Eigen::Vector3d line_target = config.formation_start;
  line_target.x() += horizontal_axis.x() * self_line_progress +
                     lateral_axis.x() * self_offset.y();
  line_target.y() += horizontal_axis.y() * self_line_progress +
                     lateral_axis.y() * self_offset.y();
  line_target.z() += self_offset.z();

  const Eigen::Vector3d line_diff = position - line_target;
  const double line_weight = config.formation_weight * 2.0;
  cost += line_weight * line_diff.squaredNorm();
  grad_position += line_weight * 2.0 * line_diff;
  grad_time += line_weight * 2.0 * line_diff.dot(velocity);

  double progress_sum = 0.0;
  double progress_rate_sum = 0.0;
  int valid_neighbors = 0;
  for (const auto &other : *swarm_trajs)
  {
    if (!other.valid() || other.drone_id == config.self_id ||
        other.drone_id < 0 || other.drone_id >= configured_num)
    {
      continue;
    }
    if (config.formation_stale_timeout > 0.0 &&
        current_wall_time - other.start_wall_time > other.duration + config.formation_stale_timeout)
    {
      continue;
    }

    Eigen::Vector3d other_offset = Eigen::Vector3d::Zero();
    if (!getSwarmFormationOffset(config, other.drone_id, other_offset))
    {
      continue;
    }

    Eigen::Vector3d other_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d other_velocity = Eigen::Vector3d::Zero();
    if (!other.sample(current_wall_time, local_time, other_position, other_velocity))
    {
      continue;
    }

    progress_sum += (other_position - config.formation_start).dot(axis) - other_offset.x();
    progress_rate_sum += axis.dot(other_velocity);
    ++valid_neighbors;
  }

  if (valid_neighbors <= 0)
  {
    return cost;
  }

  const double progress = progress_sum / static_cast<double>(valid_neighbors);
  const double progress_rate = progress_rate_sum / static_cast<double>(valid_neighbors);

  Eigen::Vector3d target = config.formation_start;
  target.x() += horizontal_axis.x() * (self_offset.x() + progress) +
                lateral_axis.x() * self_offset.y();
  target.y() += horizontal_axis.y() * (self_offset.x() + progress) +
                lateral_axis.y() * self_offset.y();
  target.z() += axis.z() * progress + self_offset.z();

  const Eigen::Vector3d diff2 = 2.0 * (position - target);
  cost += config.formation_weight * 0.25 * diff2.squaredNorm();
  grad_position += config.formation_weight * diff2;
  grad_time += config.formation_weight * diff2.dot(velocity - target_progress_axis * progress_rate);
  return cost;
}
} // namespace cost_functional
