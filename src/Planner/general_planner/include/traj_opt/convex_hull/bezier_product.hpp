#pragma once

#include "traj_opt/convex_hull/scalar_bernstein.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

namespace traj_opt
{
namespace convex_hull
{

inline double binomialCoefficient(int n, int k)
{
  if (k < 0 || k > n)
  {
    return 0.0;
  }
  k = std::min(k, n - k);
  double value = 1.0;
  for (int i = 1; i <= k; ++i)
  {
    value *= static_cast<double>(n - k + i);
    value /= static_cast<double>(i);
  }
  return value;
}

/**
 * Squared-norm Bernstein coefficients of a vector Bezier curve
 *
 *   v(u) = sum_{i=0}^d B_i^d(u) V_i
 *   ||v(u)||^2 = sum_{k=0}^{2d} B_k^{2d}(u) s_k
 *
 * with
 *
 *   s_k = sum_{i+j=k} C(d,i)C(d,j)/C(2d,k) * V_i·V_j
 */
inline Eigen::VectorXd squaredNormBernstein(
    const Eigen::Ref<const Eigen::MatrixXd> &controls)
{
  const int degree = static_cast<int>(controls.rows()) - 1;
  Eigen::VectorXd coeffs = Eigen::VectorXd::Zero(2 * degree + 1);
  if (degree < 0)
  {
    return coeffs;
  }

  std::vector<double> binom_d(static_cast<std::size_t>(degree + 1));
  for (int i = 0; i <= degree; ++i)
  {
    binom_d[static_cast<std::size_t>(i)] =
        binomialCoefficient(degree, i);
  }
  for (int k = 0; k <= 2 * degree; ++k)
  {
    const double denom = binomialCoefficient(2 * degree, k);
    if (denom <= 0.0)
    {
      continue;
    }
    double sum = 0.0;
    const int i_min = std::max(0, k - degree);
    const int i_max = std::min(degree, k);
    for (int i = i_min; i <= i_max; ++i)
    {
      const int j = k - i;
      sum += binom_d[static_cast<std::size_t>(i)] *
             binom_d[static_cast<std::size_t>(j)] *
             controls.row(i).dot(controls.row(j));
    }
    coeffs(k) = sum / denom;
  }
  return coeffs;
}

/**
 * Bernstein residuals for ||v(u)||^2 - bound^2.
 * SAFE when all residuals <= -safe_margin (continuous ||v|| <= bound).
 */
inline Eigen::VectorXd squaredNormBoundResiduals(
    const Eigen::Ref<const Eigen::MatrixXd> &controls,
    double bound)
{
  const Eigen::VectorXd coeffs = squaredNormBernstein(controls);
  const double bound_sq = bound * bound;
  return coeffs.array() - bound_sq;
}

inline ScalarBound squaredNormBoundBounds(
    const Eigen::Ref<const Eigen::MatrixXd> &controls,
    double bound)
{
  return scalarBounds(squaredNormBoundResiduals(controls, bound));
}

/**
 * Fast Level-A vector hull bound: max_i (||V_i||^2 - bound^2).
 * Sufficient but more conservative than squared-norm Bernstein.
 */
inline ScalarBound vectorHullNormBound(
    const Eigen::Ref<const Eigen::MatrixXd> &controls,
    double bound)
{
  const double bound_sq = bound * bound;
  Eigen::VectorXd residuals(controls.rows());
  for (Eigen::Index i = 0; i < controls.rows(); ++i)
  {
    residuals(i) = controls.row(i).squaredNorm() - bound_sq;
  }
  return scalarBounds(residuals);
}

} // namespace convex_hull
} // namespace traj_opt
