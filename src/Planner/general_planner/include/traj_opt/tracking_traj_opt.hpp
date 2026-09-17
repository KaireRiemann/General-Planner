#pragma once

#include <memory>
#include <string>
#include <map_manager/map_manager.hpp>
#include "data_structure/base/trajectory.h"
#include "traj_opt/config.hpp"
#include "traj_opt/tracking_problem.hpp"

namespace ros_interface { class RosInterface; }
namespace traj_opt {

class TrackingJerkTrajOpt
{
public:
  using Ptr = std::shared_ptr<TrackingJerkTrajOpt>;

  TrackingJerkTrajOpt(const traj_opt::Config &cfg,
                      const std::shared_ptr<ros_interface::RosInterface> &ros_ptr);

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setSafeDistance(double safe_distance);

  bool optimize(const TrackingProblem &problem,
                geometry_utils::Trajectory &out_traj,
                geometry_utils::Trajectory *out_yaw_traj = nullptr,
                std::string *failure_reason = nullptr);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

class TrackingSnapTrajOpt
{
public:
  using Ptr = std::shared_ptr<TrackingSnapTrajOpt>;

  TrackingSnapTrajOpt(const traj_opt::Config &cfg,
                      const std::shared_ptr<ros_interface::RosInterface> &ros_ptr);

  void setMapManager(const general_planner::MapManager::Ptr &map_manager);
  void setSafeDistance(double safe_distance);

  bool optimize(const TrackingProblem &problem,
                geometry_utils::Trajectory &out_traj,
                geometry_utils::Trajectory *out_yaw_traj = nullptr,
                std::string *failure_reason = nullptr);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};


} // namespace traj_opt
