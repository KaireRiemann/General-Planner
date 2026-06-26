#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <data_structure/base/trajectory.h>

namespace traj_opt
{
struct SwarmPenaltyConfig
{
  bool enable{false};
  int self_id{-1};
  double weight{0.0};
  double clearance{0.75};
  double des_clearance{0.75};
  double horizontal_scale{1.0};
  double vertical_scale{2.0};
  double activation_scale{1.5};
  double time_horizon{5.0};
  double stale_timeout{1.0};
  bool formation_enable{false};
  double formation_weight{0.0};
  int formation_num{0};
  std::vector<double> formation_offsets;
  Eigen::Vector3d formation_start{Eigen::Vector3d::Zero()};
  Eigen::Vector3d formation_end{Eigen::Vector3d::UnitX()};
  double formation_time_horizon{5.0};
  double formation_stale_timeout{1.0};
};

struct SwarmTrajectory
{
  int drone_id{-1};
  unsigned int traj_id{0};
  double start_wall_time{0.0};
  double duration{0.0};
  double clearance{0.0};
  geometry_utils::Trajectory traj;

  bool valid() const
  {
    return drone_id >= 0 && duration > 1.0e-6 && !traj.empty();
  }

  bool sample(double current_wall_time,
              double local_time,
              Eigen::Vector3d &position,
              Eigen::Vector3d &velocity) const
  {
    if (!valid())
    {
      return false;
    }

    const double query_time = current_wall_time + local_time - start_wall_time;
    if (query_time < 0.0)
    {
      return false;
    }

    if (query_time <= duration)
    {
      position = traj.getPos(query_time);
      velocity = traj.getVel(query_time);
    }
    else
    {
      const Eigen::Vector3d terminal_position = traj.getPos(duration);
      const Eigen::Vector3d terminal_velocity = traj.getVel(duration);
      position = terminal_position + (query_time - duration) * terminal_velocity;
      velocity = terminal_velocity;
    }

    return position.allFinite() && velocity.allFinite();
  }
};

using SwarmTrajectories = std::vector<SwarmTrajectory>;
using SwarmTrajectoriesConstPtr = std::shared_ptr<const SwarmTrajectories>;
} // namespace traj_opt
