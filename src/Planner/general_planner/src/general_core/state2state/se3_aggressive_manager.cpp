#include "general_core/state2state/se3_aggressive_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "traj_opt/costfunctional/spatialcosts/se3_shape_corridor_penalty.hpp"

namespace general_planner {

SE3AggressiveManager::SE3AggressiveManager(
    const Config &cfg,
    const ros_interface::RosInterface::Ptr &ros_ptr,
    const MapManager::Ptr &map_manager,
    const path_search::Astar::Ptr &astar,
    const CorridorGenerator::Ptr &corridor_generator)
    : cfg_(cfg),
      ros_ptr_(ros_ptr),
      map_manager_(map_manager) {
  frontend_ =
      std::make_unique<SE3AggressiveFrontend>(cfg_, map_manager_, astar, corridor_generator);
  optimizer_ = std::make_unique<traj_opt::SE3AggressiveTrajOpt>(cfg_.esdf_traj_cfg, ros_ptr_);
  optimizer_->setMapManager(map_manager_);
}

bool SE3AggressiveManager::plan(const general_utils::StatePVAJ &head,
                                const general_utils::StatePVAJ &tail,
                                geometry_utils::Trajectory &traj,
                                std::string *reason,
                                double yaw,
                                double yaw_rate) {
  traj.clear();
  if (!cfg_.se3_aggressive_enable) {
    if (reason != nullptr) {
      *reason = "SE3_AGGRESSIVE_DISABLED";
    }
    return false;
  }
  if (!frontend_ || !optimizer_) {
    if (reason != nullptr) {
      *reason = "SE3_MANAGER_NOT_INITIALIZED";
    }
    return false;
  }

  traj_opt::SE3AggressiveProblem problem;
  if (!frontend_->buildProblem(head, tail, problem)) {
    if (reason != nullptr) {
      *reason = "FRONTEND_FAILED";
    }
    return false;
  }
  problem.yaw = std::isfinite(yaw) ? yaw : 0.0;
  problem.yaw_rate = std::isfinite(yaw_rate) ? yaw_rate : 0.0;

  if (!optimizer_->optimize(problem, traj)) {
    if (reason != nullptr) {
      *reason = "OPTIMIZER_FAILED";
    }
    return false;
  }

  active_problem_ = problem;
  if (problem.runtime_check_enable && !checkTrajectory(traj, problem, reason)) {
    return false;
  }
  if (reason != nullptr) {
    *reason = "OK";
  }
  return true;
}

bool SE3AggressiveManager::checkSample(double t,
                                       int piece_id,
                                       const geometry_utils::Trajectory &traj,
                                       const traj_opt::SE3AggressiveProblem &problem,
                                       std::string *reason,
                                       double &max_vel,
                                       double &max_thrust,
                                       double &max_body_rate,
                                       double &max_corridor_violation) const {
  const Eigen::Vector3d p = traj.getPos(t);
  const Eigen::Vector3d v = traj.getVel(t);
  const Eigen::Vector3d a = traj.getAcc(t);
  const Eigen::Vector3d j = traj.getJer(t);
  const Eigen::Vector3d s = traj.getSnap(t);
  if (!p.allFinite() || !v.allFinite() || !a.allFinite() || !j.allFinite() || !s.allFinite()) {
    if (reason != nullptr) {
      *reason = "NON_FINITE_SAMPLE";
    }
    return false;
  }

  const double vel = v.norm();
  const double thrust = (a + cfg_.esdf_traj_cfg.grav * Eigen::Vector3d::UnitZ()).norm();
  max_vel = std::max(max_vel, vel);
  max_thrust = std::max(max_thrust, thrust);
  if (vel > problem.max_vel + 1.0e-3) {
    if (reason != nullptr) {
      *reason = "MAX_VEL";
    }
    return false;
  }
  if (thrust < problem.thrust_acc_min - 1.0e-3 ||
      thrust > problem.thrust_acc_max + 1.0e-3) {
    if (reason != nullptr) {
      *reason = "THRUST";
    }
    return false;
  }

  const double sample_yaw =
      problem.use_yaw ? problem.yaw : (problem.yaw_heading_to_velocity ? NAN : 0.0);
  const double sample_yaw_rate = problem.use_yaw ? problem.yaw_rate : 0.0;
  traj_opt::SE3FlatnessMap flatness;
  flatness.setYawMode(problem.use_yaw, problem.yaw_heading_to_velocity);
  traj_opt::SE3FlatnessOutput flat;
  if (!flatness.forward(v,
                        a,
                        j,
                        s,
                        sample_yaw,
                        sample_yaw_rate,
                        cfg_.esdf_traj_cfg.grav,
                        flat)) {
    if (reason != nullptr) {
      *reason = "FLATNESS_INVALID";
    }
    return false;
  }
  const double body_rate = flat.omega.norm();
  max_body_rate = std::max(max_body_rate, body_rate);
  if (body_rate > problem.body_rate_max + 1.0e-3) {
    if (reason != nullptr) {
      *reason = "BODY_RATE";
    }
    return false;
  }

  if (problem.use_corridor && !problem.hpolys.empty()) {
    int corridor_id = piece_id;
    if (piece_id >= 0 && piece_id < static_cast<int>(problem.piece_to_corridor.size())) {
      corridor_id = problem.piece_to_corridor[static_cast<std::size_t>(piece_id)];
    }
    cost_functional::SE3ShapeConfig shape_config;
    shape_config.ellipsoid =
        Eigen::Vector3d(problem.horiz_half_len, problem.horiz_half_len, problem.vert_half_len);
    shape_config.safe_margin = problem.safe_margin;
    shape_config.weight = problem.weight_corridor;
    shape_config.smooth_eps = cfg_.esdf_traj_cfg.smooth_eps;
    const double corridor_violation =
        cost_functional::maxSE3ShapeCorridorViolation(p,
                                                      v,
                                                      a,
                                                      j,
                                                      s,
                                                      sample_yaw,
                                                      sample_yaw_rate,
                                                      cfg_.esdf_traj_cfg.grav,
                                                      problem.hpolys,
                                                      corridor_id,
                                                      shape_config);
    max_corridor_violation = std::max(max_corridor_violation, corridor_violation);
    if (corridor_violation > 1.0e-3) {
      if (reason != nullptr) {
        *reason = "SE3_CORRIDOR";
      }
      return false;
    }
  }

  return true;
}

bool SE3AggressiveManager::checkTrajectory(
    const geometry_utils::Trajectory &traj,
    const traj_opt::SE3AggressiveProblem &problem,
    std::string *reason) {
  if (traj.empty()) {
    if (reason != nullptr) {
      *reason = "EMPTY_TRAJECTORY";
    }
    return false;
  }

  double max_vel = 0.0;
  double max_thrust = 0.0;
  double max_body_rate = 0.0;
  double max_corridor_violation = -std::numeric_limits<double>::infinity();
  const double total = traj.getTotalDuration();
  const double dt = std::clamp(total / 120.0, 0.03, 0.08);
  for (double t = 0.0; t <= total + 1.0e-6; t += dt) {
    double local_t = std::min(t, total);
    const int piece_id = traj.locatePieceIdx(local_t);
    std::string sample_reason;
    if (!checkSample(std::min(t, total),
                     piece_id,
                     traj,
                     problem,
                     &sample_reason,
                     max_vel,
                     max_thrust,
                     max_body_rate,
                     max_corridor_violation)) {
      if (reason != nullptr) {
        *reason = sample_reason;
      }
      std::cout << " -- [SE3AggressiveManager] SE3_RUNTIME_REJECT t=" << std::min(t, total)
                << ", reason=" << sample_reason
                << ", vel=" << max_vel
                << ", thrust=" << max_thrust
                << ", body_rate=" << max_body_rate
                << ", corridor_violation=" << max_corridor_violation
                << std::endl;
      return false;
    }
  }

  std::cout << " -- [SE3AggressiveManager] SE3_RUNTIME_ACCEPTED duration=" << total
            << ", max_vel=" << max_vel
            << ", max_thrust=" << max_thrust
            << ", max_body_rate=" << max_body_rate
            << ", corridor_violation="
            << (std::isfinite(max_corridor_violation) ? max_corridor_violation : 0.0)
            << std::endl;
  if (reason != nullptr) {
    *reason = "OK";
  }
  return true;
}

} // namespace general_planner
