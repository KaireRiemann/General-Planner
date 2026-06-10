#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "traj_opt/costfunctional/penalty_utils.hpp"
#include "traj_opt/flatness/se3_flatness_map.hpp"

namespace cost_functional {

struct SE3ShapeConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d ellipsoid{0.35, 0.35, 0.12};
  double safe_margin{0.05};
  double weight{1.0e4};
  double smooth_eps{0.01};
  bool use_numeric_shape_gradient{true};
};

namespace se3_shape_detail {

inline double supportViolation(const Eigen::Vector3d &position,
                               const Eigen::Vector3d &velocity,
                               const Eigen::Vector3d &acceleration,
                               const Eigen::Vector3d &jerk,
                               const Eigen::Vector3d &snap,
                               double yaw,
                               double yaw_rate,
                               double gravity,
                               const Eigen::Vector3d &normal,
                               const Eigen::Vector3d &point,
                               const SE3ShapeConfig &config) {
  traj_opt::SE3FlatnessMap flatness;
  flatness.setYawMode(std::isfinite(yaw), true);
  traj_opt::SE3FlatnessOutput flat;
  if (!flatness.forward(velocity, acceleration, jerk, snap, yaw, yaw_rate, gravity, flat)) {
    return std::numeric_limits<double>::infinity();
  }

  const double normal_norm = normal.norm();
  if (normal_norm < 1.0e-9) {
    return -std::numeric_limits<double>::infinity();
  }
  const Eigen::Vector3d n = normal / normal_norm;
  const Eigen::Vector3d q = config.ellipsoid.asDiagonal() * (flat.R.transpose() * n);
  const double support = q.norm();
  return n.dot(position - point) + support + config.safe_margin;
}

inline double weightedPositivePenalty(double violation,
                                      const SE3ShapeConfig &config,
                                      double *d_penalty_d_violation) {
  double penalty = 0.0;
  double grad = 0.0;
  if (!smoothedL1(violation, config.smooth_eps, penalty, grad)) {
    if (d_penalty_d_violation != nullptr) {
      *d_penalty_d_violation = 0.0;
    }
    return 0.0;
  }
  if (d_penalty_d_violation != nullptr) {
    *d_penalty_d_violation = config.weight * grad;
  }
  return config.weight * penalty;
}

inline double corridorPenaltyValue(const Eigen::Vector3d &position,
                                   const Eigen::Vector3d &velocity,
                                   const Eigen::Vector3d &acceleration,
                                   const Eigen::Vector3d &jerk,
                                   const Eigen::Vector3d &snap,
                                   double yaw,
                                   double yaw_rate,
                                   double gravity,
                                   const Eigen::Matrix<double, 6, Eigen::Dynamic> &hpoly,
                                   const SE3ShapeConfig &config) {
  double cost = 0.0;
  for (int col = 0; col < hpoly.cols(); ++col) {
    const double violation = supportViolation(position,
                                              velocity,
                                              acceleration,
                                              jerk,
                                              snap,
                                              yaw,
                                              yaw_rate,
                                              gravity,
                                              hpoly.col(col).head<3>(),
                                              hpoly.col(col).tail<3>(),
                                              config);
    if (std::isfinite(violation)) {
      cost += weightedPositivePenalty(violation, config, nullptr);
    }
  }
  return cost;
}

} // namespace se3_shape_detail

inline double maxSE3ShapeCorridorViolation(
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &velocity,
    const Eigen::Vector3d &acceleration,
    const Eigen::Vector3d &jerk,
    const Eigen::Vector3d &snap,
    double yaw,
    double yaw_rate,
    double gravity,
    const std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> &hpolys,
    int corridor_id,
    const SE3ShapeConfig &config) {
  if (corridor_id < 0 || corridor_id >= static_cast<int>(hpolys.size())) {
    return 0.0;
  }

  double max_violation = -std::numeric_limits<double>::infinity();
  const auto &hpoly = hpolys[static_cast<std::size_t>(corridor_id)];
  for (int col = 0; col < hpoly.cols(); ++col) {
    const double violation = se3_shape_detail::supportViolation(position,
                                                                velocity,
                                                                acceleration,
                                                                jerk,
                                                                snap,
                                                                yaw,
                                                                yaw_rate,
                                                                gravity,
                                                                hpoly.col(col).head<3>(),
                                                                hpoly.col(col).tail<3>(),
                                                                config);
    if (std::isfinite(violation)) {
      max_violation = std::max(max_violation, violation);
    }
  }
  return std::isfinite(max_violation) ? max_violation : 0.0;
}

inline double accumulateSE3ShapeCorridorPenalty(
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &velocity,
    const Eigen::Vector3d &acceleration,
    const Eigen::Vector3d &jerk,
    const Eigen::Vector3d &snap,
    double yaw,
    double yaw_rate,
    double gravity,
    const std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> &hpolys,
    int corridor_id,
    const SE3ShapeConfig &config,
    Eigen::Vector3d &grad_position,
    Eigen::Vector3d &grad_velocity,
    Eigen::Vector3d &grad_acceleration,
    Eigen::Vector3d &grad_jerk,
    Eigen::Vector3d &grad_snap,
    double &grad_yaw,
    double &grad_yaw_rate) {
  if (config.weight <= 0.0 ||
      corridor_id < 0 ||
      corridor_id >= static_cast<int>(hpolys.size())) {
    return 0.0;
  }

  const auto &hpoly = hpolys[static_cast<std::size_t>(corridor_id)];
  double cost = 0.0;
  for (int col = 0; col < hpoly.cols(); ++col) {
    const Eigen::Vector3d raw_n = hpoly.col(col).head<3>();
    const double normal_norm = raw_n.norm();
    if (normal_norm < 1.0e-9) {
      continue;
    }
    const Eigen::Vector3d n = raw_n / normal_norm;
    const Eigen::Vector3d q = hpoly.col(col).tail<3>();
    const double violation = se3_shape_detail::supportViolation(position,
                                                                velocity,
                                                                acceleration,
                                                                jerk,
                                                                snap,
                                                                yaw,
                                                                yaw_rate,
                                                                gravity,
                                                                raw_n,
                                                                q,
                                                                config);
    if (!std::isfinite(violation)) {
      continue;
    }

    double d_penalty = 0.0;
    const double sample_cost =
        se3_shape_detail::weightedPositivePenalty(violation, config, &d_penalty);
    if (d_penalty <= 0.0) {
      continue;
    }
    cost += sample_cost;
    grad_position += d_penalty * n;
  }

  if (config.use_numeric_shape_gradient && cost > 0.0) {
    // First implementation hook: support-orientation gradients are finite
    // differenced behind an opt-in flag. The analytic chain d support / dR
    // and dR / d(acc, jerk, yaw) should replace this for high-rate racing.
    constexpr double eps = 1.0e-4;
    auto valueAt = [&](const Eigen::Vector3d &p,
                       const Eigen::Vector3d &v,
                       const Eigen::Vector3d &a,
                       const Eigen::Vector3d &j,
                       const Eigen::Vector3d &s,
                       double y,
                       double yd) {
      return se3_shape_detail::corridorPenaltyValue(p, v, a, j, s, y, yd, gravity, hpoly, config);
    };

    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d d = Eigen::Vector3d::Zero();
      d(axis) = eps;
      grad_velocity(axis) += (valueAt(position, velocity + d, acceleration, jerk, snap, yaw, yaw_rate) -
                              valueAt(position, velocity - d, acceleration, jerk, snap, yaw, yaw_rate)) /
                             (2.0 * eps);
      grad_acceleration(axis) += (valueAt(position, velocity, acceleration + d, jerk, snap, yaw, yaw_rate) -
                                  valueAt(position, velocity, acceleration - d, jerk, snap, yaw, yaw_rate)) /
                                 (2.0 * eps);
      grad_jerk(axis) += (valueAt(position, velocity, acceleration, jerk + d, snap, yaw, yaw_rate) -
                          valueAt(position, velocity, acceleration, jerk - d, snap, yaw, yaw_rate)) /
                         (2.0 * eps);
      grad_snap(axis) += (valueAt(position, velocity, acceleration, jerk, snap + d, yaw, yaw_rate) -
                          valueAt(position, velocity, acceleration, jerk, snap - d, yaw, yaw_rate)) /
                         (2.0 * eps);
    }

    if (std::isfinite(yaw)) {
      grad_yaw += (valueAt(position, velocity, acceleration, jerk, snap, yaw + eps, yaw_rate) -
                   valueAt(position, velocity, acceleration, jerk, snap, yaw - eps, yaw_rate)) /
                  (2.0 * eps);
    }
    grad_yaw_rate += (valueAt(position, velocity, acceleration, jerk, snap, yaw, yaw_rate + eps) -
                      valueAt(position, velocity, acceleration, jerk, snap, yaw, yaw_rate - eps)) /
                     (2.0 * eps);
  }

  return cost;
}

} // namespace cost_functional
