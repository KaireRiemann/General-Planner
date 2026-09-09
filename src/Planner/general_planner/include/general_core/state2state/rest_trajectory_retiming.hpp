#pragma once

#include <data_structure/base/trajectory.h>
#include <algorithm>
#include <cmath>

namespace general_planner {
inline double subdividedHullBound(const Eigen::MatrixXd &control, int depth) {
  if (depth == 0 || control.cols() == 1)
    return control.colwise().norm().maxCoeff();
  Eigen::MatrixXd work = control, left = control, right = control;
  const int n = control.cols() - 1;
  for (int level = 1; level <= n; ++level) {
    for (int i = 0; i <= n - level; ++i)
      work.col(i) = (0.5 * (work.col(i) + work.col(i + 1))).eval();
    left.col(level) = work.col(0);
    right.col(n - level) = work.col(n - level);
  }
  return std::max(subdividedHullBound(left, depth - 1),
                  subdividedHullBound(right, depth - 1));
}
// The convex hull of Bernstein coefficients bounds the entire derivative,
// including peaks between checker samples. Coefficients in Piece are descending.
inline double derivativeHullBound(const geometry_utils::Piece &piece, int order) {
  const auto &c = piece.getCoeffMat();
  const int degree = c.cols() - 1;
  const int n = degree - order;
  const double duration = piece.getDuration();
  if (!c.allFinite() || !std::isfinite(duration) || duration <= 0)
    return INFINITY;
  if (n < 0) return 0;
  Eigen::MatrixXd control(c.rows(), n + 1);
  for (int i = 0; i <= n; ++i) {
    Eigen::VectorXd b = Eigen::VectorXd::Zero(c.rows());
    double ratio = 1;
    for (int k = 0; k <= i; ++k) {
      double derivative = 1;
      for (int j = 1; j <= order; ++j) derivative *= k + j;
      b += c.col(degree - k - order) *
           (derivative * std::pow(duration, k) * ratio);
      if (k < i) ratio *= double(i - k) / double(n - k);
    }
    if (!b.allFinite()) return INFINITY;
    control.col(i) = b;
  }
  // Subdivision tightens the bound without turning it into a sampled check.
  return subdividedHullBound(control, 3);
}

// Only rest-to-rest trajectories can be uniformly slowed without changing
// nonzero boundary derivatives. Caller excludes rolling replans and swarm
// trajectories: their timing is part of the boundary/collision contract.
inline bool retimeRestTrajectory(geometry_utils::Trajectory &traj,
                                 double max_vel, double max_acc, double max_jerk) {
  if (traj.empty()) return false;
  const double end = traj.getTotalDuration();
  if (!std::isfinite(end) || end <= 0) return false;
  for (double t : {0.0, end}) {
    if (!traj.getVel(t).allFinite() || !traj.getAcc(t).allFinite() ||
        !traj.getJer(t).allFinite() || traj.getVel(t).norm() > 1e-6 ||
        traj.getAcc(t).norm() > 1e-6 || traj.getJer(t).norm() > 1e-6)
      return false;
  }
  const double limits[] = {max_vel, max_acc, max_jerk};
  double scale = 1;
  for (const auto &piece : traj) {
    for (int order = 1; order <= 3; ++order) {
      const double bound = derivativeHullBound(piece, order);
      if (!std::isfinite(bound)) return false;
      if (std::isfinite(limits[order - 1]) && limits[order - 1] > 0)
        scale = std::max(scale, std::pow(bound / limits[order - 1], 1.0 / order));
    }
  }
  if (scale <= 1) return true;
  scale *= 1.01;
  // A pathological optimizer output must still fail finitely, not create a
  // practically endless command trajectory.
  if (!std::isfinite(scale) || scale > 20 || end * scale > 120) return false;
  geometry_utils::Trajectory slowed;
  slowed.start_WT = traj.start_WT;
  for (const auto &piece : traj) {
    Eigen::MatrixXd c = piece.getCoeffMat();
    for (int col = 0; col < c.cols(); ++col)
      c.col(col) /= std::pow(scale, c.cols() - 1 - col);
    slowed.emplace_back(piece.getDuration() * scale, c);
  }
  traj = slowed;
  return true;
}
} // namespace general_planner
