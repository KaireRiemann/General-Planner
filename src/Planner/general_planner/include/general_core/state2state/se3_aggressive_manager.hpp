#pragma once

#include <memory>
#include <limits>
#include <string>

#include "data_structure/base/trajectory.h"
#include "general_core/config.hpp"
#include "general_core/state2state/se3_aggressive_frontend.hpp"
#include "traj_opt/flatness/se3_flatness_map.hpp"
#include "traj_opt/se3_aggressive_traj_opt.hpp"

namespace general_planner {

class SE3AggressiveManager {
public:
  using Ptr = std::shared_ptr<SE3AggressiveManager>;

  SE3AggressiveManager(const Config &cfg,
                       const ros_interface::RosInterface::Ptr &ros_ptr,
                       const MapManager::Ptr &map_manager,
                       const path_search::Astar::Ptr &astar,
                       const CorridorGenerator::Ptr &corridor_generator);

  bool plan(const general_utils::StatePVAJ &head,
            const general_utils::StatePVAJ &tail,
            geometry_utils::Trajectory &traj,
            std::string *reason,
            double yaw = std::numeric_limits<double>::quiet_NaN(),
            double yaw_rate = 0.0);

  bool checkTrajectory(const geometry_utils::Trajectory &traj,
                       const traj_opt::SE3AggressiveProblem &problem,
                       std::string *reason);

private:
  bool checkSample(double t,
                   int piece_id,
                   const geometry_utils::Trajectory &traj,
                   const traj_opt::SE3AggressiveProblem &problem,
                   std::string *reason,
                   double &max_vel,
                   double &max_thrust,
                   double &max_body_rate,
                   double &max_corridor_violation) const;

  Config cfg_;
  ros_interface::RosInterface::Ptr ros_ptr_;
  MapManager::Ptr map_manager_;
  std::unique_ptr<SE3AggressiveFrontend> frontend_;
  std::unique_ptr<traj_opt::SE3AggressiveTrajOpt> optimizer_;
  traj_opt::SE3AggressiveProblem active_problem_;
};

} // namespace general_planner
