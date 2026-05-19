#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "general_core/map_manager.hpp"
#include "traj_opt/costfunctional/penalty_utils.hpp"
#include "traj_opt/costfunctional/spatialcosts/se3_shape_corridor_penalty.hpp"
#include "traj_opt/flatness/se3_flatness_map.hpp"
#include "traj_opt/se3_aggressive_traj_opt.hpp"

namespace cost_functional_manager {

class SE3AggressiveCostManager {
public:
  void reset(const traj_opt::Config &cfg,
             const traj_opt::SE3AggressiveProblem &problem,
             const general_planner::MapManager::Ptr &map_manager) {
    cfg_ = &cfg;
    problem_ = &problem;
    map_manager_ = map_manager;
    flatness_.setYawMode(problem.use_yaw, problem.yaw_heading_to_velocity);
    shape_config_.ellipsoid =
        Eigen::Vector3d(problem.horiz_half_len, problem.horiz_half_len, problem.vert_half_len);
    shape_config_.safe_margin = problem.safe_margin;
    shape_config_.weight = problem.weight_corridor;
    shape_config_.smooth_eps = cfg.smooth_eps;
    shape_config_.use_numeric_shape_gradient = problem.use_numeric_shape_gradient;
    max_violation_.setZero();
  }

  const Eigen::Matrix<double, 4, 1> &getPenaltyLog() const { return max_violation_; }
  double getMaxCorridorViolation() const { return max_violation_(3); }

  double evaluateIntegral(int /*sample_id*/,
                          double /*local_t*/,
                          double /*t_global*/,
                          int piece_id,
                          int /*sample_in_piece*/,
                          const Eigen::Vector3d &position,
                          const Eigen::Vector3d &velocity,
                          const Eigen::Vector3d &acceleration,
                          const Eigen::Vector3d &jerk,
                          const Eigen::Vector3d &snap,
                          double yaw,
                          double yaw_rate,
                          Eigen::Vector3d &grad_position,
                          Eigen::Vector3d &grad_velocity,
                          Eigen::Vector3d &grad_acceleration,
                          Eigen::Vector3d &grad_jerk,
                          Eigen::Vector3d &grad_snap,
                          double &grad_yaw,
                          double &grad_yaw_rate,
                          double &grad_time) const {
    (void)map_manager_;
    (void)grad_time;
    if (cfg_ == nullptr || problem_ == nullptr) {
      return 0.0;
    }

    double cost = 0.0;
    cost += addVelocityCost(velocity, grad_velocity);
    cost += addThrustCost(acceleration, grad_acceleration);
    cost += addBodyRateCost(velocity,
                            acceleration,
                            jerk,
                            snap,
                            yaw,
                            yaw_rate,
                            grad_velocity,
                            grad_acceleration,
                            grad_jerk,
                            grad_yaw,
                            grad_yaw_rate);

    if (problem_->use_corridor && !problem_->hpolys.empty()) {
      int corridor_id = piece_id;
      if (piece_id >= 0 && piece_id < static_cast<int>(problem_->piece_to_corridor.size())) {
        corridor_id = problem_->piece_to_corridor[static_cast<std::size_t>(piece_id)];
      }
      cost += cost_functional::accumulateSE3ShapeCorridorPenalty(position,
                                                                 velocity,
                                                                 acceleration,
                                                                 jerk,
                                                                 snap,
                                                                 yaw,
                                                                 yaw_rate,
                                                                 cfg_->grav,
                                                                 problem_->hpolys,
                                                                 corridor_id,
                                                                 shape_config_,
                                                                 grad_position,
                                                                 grad_velocity,
                                                                 grad_acceleration,
                                                                 grad_jerk,
                                                                 grad_snap,
                                                                 grad_yaw,
                                                                 grad_yaw_rate);
      max_violation_(3) = std::max(max_violation_(3),
                                   cost_functional::maxSE3ShapeCorridorViolation(position,
                                                                                 velocity,
                                                                                 acceleration,
                                                                                 jerk,
                                                                                 snap,
                                                                                 yaw,
                                                                                 yaw_rate,
                                                                                 cfg_->grav,
                                                                                 problem_->hpolys,
                                                                                 corridor_id,
                                                                                 shape_config_));
    }

    return cost;
  }

private:
  double addVelocityCost(const Eigen::Vector3d &velocity,
                         Eigen::Vector3d &grad_velocity) const {
    const double weight = std::max(0.0, problem_->weight_vel);
    if (weight <= 0.0 || problem_->max_vel <= 0.0) {
      return 0.0;
    }
    const double violation = velocity.squaredNorm() - problem_->max_vel * problem_->max_vel;
    max_violation_(0) = std::max(max_violation_(0), violation);
    double penalty = 0.0;
    double d_penalty = 0.0;
    if (!cost_functional::smoothedL1(violation, cfg_->smooth_eps, penalty, d_penalty)) {
      return 0.0;
    }
    grad_velocity += weight * d_penalty * 2.0 * velocity;
    return weight * penalty;
  }

  double addThrustCost(const Eigen::Vector3d &acceleration,
                       Eigen::Vector3d &grad_acceleration) const {
    const double weight = std::max(0.0, problem_->weight_thrust);
    if (weight <= 0.0) {
      return 0.0;
    }

    const Eigen::Vector3d u = acceleration + cfg_->grav * Eigen::Vector3d::UnitZ();
    const double thrust_sq = u.squaredNorm();
    const double min_sq = problem_->thrust_acc_min * problem_->thrust_acc_min;
    const double max_sq = problem_->thrust_acc_max * problem_->thrust_acc_max;

    double cost = 0.0;
    double penalty = 0.0;
    double d_penalty = 0.0;

    const double lower_violation = min_sq - thrust_sq;
    max_violation_(1) = std::max(max_violation_(1), lower_violation);
    if (cost_functional::smoothedL1(lower_violation, cfg_->smooth_eps, penalty, d_penalty)) {
      cost += weight * penalty;
      grad_acceleration -= weight * d_penalty * 2.0 * u;
    }

    const double upper_violation = thrust_sq - max_sq;
    max_violation_(1) = std::max(max_violation_(1), upper_violation);
    if (cost_functional::smoothedL1(upper_violation, cfg_->smooth_eps, penalty, d_penalty)) {
      cost += weight * penalty;
      grad_acceleration += weight * d_penalty * 2.0 * u;
    }
    return cost;
  }

  double addBodyRateCost(const Eigen::Vector3d &velocity,
                         const Eigen::Vector3d &acceleration,
                         const Eigen::Vector3d &jerk,
                         const Eigen::Vector3d &snap,
                         double yaw,
                         double yaw_rate,
                         Eigen::Vector3d &grad_velocity,
                         Eigen::Vector3d &grad_acceleration,
                         Eigen::Vector3d &grad_jerk,
                         double &grad_yaw,
                         double &grad_yaw_rate) const {
    (void)grad_velocity;
    (void)grad_acceleration;
    (void)grad_yaw;
    const double weight = std::max(0.0, problem_->weight_body_rate);
    if (weight <= 0.0 || problem_->body_rate_max <= 0.0) {
      return 0.0;
    }

    traj_opt::SE3FlatnessOutput flat;
    if (!flatness_.forward(velocity, acceleration, jerk, snap, yaw, yaw_rate, cfg_->grav, flat)) {
      return 0.0;
    }

    const Eigen::Vector2d omega_xy(flat.omega.x(), flat.omega.y());
    const double violation =
        omega_xy.squaredNorm() - problem_->body_rate_max * problem_->body_rate_max;
    max_violation_(2) = std::max(max_violation_(2), violation);
    double penalty = 0.0;
    double d_penalty = 0.0;
    if (!cost_functional::smoothedL1(violation, cfg_->smooth_eps, penalty, d_penalty)) {
      return 0.0;
    }

    const Eigen::Vector2d grad_omega_xy = weight * d_penalty * 2.0 * omega_xy;
    Eigen::Vector3d grad_b = Eigen::Vector3d::Zero();
    grad_b.x() = grad_omega_xy.y();
    grad_b.y() = -grad_omega_xy.x();
    grad_jerk += flat.R * grad_b / std::max(1.0e-8, flat.thrust);

    if (problem_->use_yaw) {
      const double yaw_violation =
          yaw_rate * yaw_rate - problem_->yaw_rate_max * problem_->yaw_rate_max;
      double yaw_penalty = 0.0;
      double d_yaw_penalty = 0.0;
      if (cost_functional::smoothedL1(yaw_violation, cfg_->smooth_eps, yaw_penalty, d_yaw_penalty)) {
        grad_yaw_rate += weight * d_yaw_penalty * 2.0 * yaw_rate;
        return weight * penalty + weight * yaw_penalty;
      }
    }
    return weight * penalty;
  }

  const traj_opt::Config *cfg_{nullptr};
  const traj_opt::SE3AggressiveProblem *problem_{nullptr};
  general_planner::MapManager::Ptr map_manager_;
  mutable Eigen::Matrix<double, 4, 1> max_violation_{Eigen::Matrix<double, 4, 1>::Zero()};
  mutable traj_opt::SE3FlatnessMap flatness_;
  cost_functional::SE3ShapeConfig shape_config_;
};

} // namespace cost_functional_manager
