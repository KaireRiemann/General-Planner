#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace traj_opt
{
namespace convex_hull
{

enum class LeafStatus
{
  SAFE = 0,
  VIOLATED = 1,
  UNCERTAIN = 2,
  MAX_DEPTH_REACHED = 3
};

struct ScalarBound
{
  double lower{0.0};
  double upper{0.0};
  int argmin{0};
  int argmax{0};
};

/**
 * Scalar Bernstein residual for a half-space a^T p(u) + b evaluated on
 * Bezier controls Q (rows = control points).
 *
 *   r_i = a^T Q_i + b
 *   min r_i <= h(u) <= max r_i  for all u in the leaf.
 */
inline Eigen::VectorXd halfSpaceResiduals(
    const Eigen::Ref<const Eigen::MatrixXd> &controls,
    const Eigen::Vector3d &normal,
    double offset)
{
  Eigen::VectorXd residuals(controls.rows());
  for (Eigen::Index i = 0; i < controls.rows(); ++i)
  {
    residuals(i) = normal.dot(controls.row(i).transpose()) + offset;
  }
  return residuals;
}

inline ScalarBound scalarBounds(const Eigen::Ref<const Eigen::VectorXd> &values)
{
  ScalarBound bound;
  if (values.size() <= 0)
  {
    bound.lower = 0.0;
    bound.upper = 0.0;
    return bound;
  }
  bound.lower = values.minCoeff(&bound.argmin);
  bound.upper = values.maxCoeff(&bound.argmax);
  return bound;
}

inline ScalarBound halfSpaceBounds(
    const Eigen::Ref<const Eigen::MatrixXd> &controls,
    const Eigen::Vector3d &normal,
    double offset,
    double track_inflate = 0.0)
{
  const Eigen::VectorXd residuals =
      halfSpaceResiduals(controls, normal, offset);
  ScalarBound bound = scalarBounds(residuals);
  // Robust / tracking inflation: treat the half-space as if the surface were
  // pulled inward by track_inflate * ||a||.
  const double inflate = std::max(0.0, track_inflate) * normal.norm();
  bound.lower += inflate;
  bound.upper += inflate;
  return bound;
}

/**
 * Classify a scalar Bernstein leaf.
 *
 *   upper <= -safe_margin  → SAFE
 *   lower > 0              → VIOLATED
 *   otherwise              → UNCERTAIN
 */
inline LeafStatus classifyScalarLeaf(const ScalarBound &bound,
                                     double safe_margin,
                                     double bound_gap_tol = 0.0)
{
  const double margin = std::max(0.0, safe_margin);
  if (bound.upper <= -margin)
  {
    return LeafStatus::SAFE;
  }
  if (bound.lower > 0.0)
  {
    return LeafStatus::VIOLATED;
  }
  if (bound_gap_tol > 0.0 &&
      (bound.upper - bound.lower) <= bound_gap_tol)
  {
    // Gap too small to refine further usefully; treat as unresolved boundary.
    return LeafStatus::MAX_DEPTH_REACHED;
  }
  return LeafStatus::UNCERTAIN;
}

/**
 * Midpoint de Casteljau split of a Bezier control polygon.
 * Returns (left_controls, right_controls), each with the same degree.
 */
inline std::pair<Eigen::MatrixXd, Eigen::MatrixXd> deCasteljauSplit(
    const Eigen::Ref<const Eigen::MatrixXd> &controls,
    double split_u = 0.5)
{
  const int n = static_cast<int>(controls.rows()) - 1;
  const int dim = static_cast<int>(controls.cols());
  Eigen::MatrixXd left(n + 1, dim);
  Eigen::MatrixXd right(n + 1, dim);
  if (n < 0)
  {
    return {left, right};
  }

  const double t = std::clamp(split_u, 0.0, 1.0);
  const double one_minus_t = 1.0 - t;
  std::vector<Eigen::MatrixXd> pyramid(static_cast<std::size_t>(n + 1));
  pyramid[0] = controls;
  for (int r = 1; r <= n; ++r)
  {
    pyramid[static_cast<std::size_t>(r)].resize(n - r + 1, dim);
    for (int i = 0; i <= n - r; ++i)
    {
      pyramid[static_cast<std::size_t>(r)].row(i) =
          one_minus_t *
              pyramid[static_cast<std::size_t>(r - 1)].row(i) +
          t * pyramid[static_cast<std::size_t>(r - 1)].row(i + 1);
    }
  }

  for (int i = 0; i <= n; ++i)
  {
    left.row(i) = pyramid[static_cast<std::size_t>(i)].row(0);
    right.row(i) = pyramid[static_cast<std::size_t>(n - i)].row(i);
  }
  return {left, right};
}

/**
 * Evaluate a Bezier curve (or scalar Bernstein polynomial) at normalized u.
 */
inline Eigen::VectorXd evaluateBezier(
    const Eigen::Ref<const Eigen::MatrixXd> &controls,
    double u)
{
  const auto split = deCasteljauSplit(controls, std::clamp(u, 0.0, 1.0));
  // Right of a left-split at u, the shared point is the curve value.
  // Using left's last row equals right's first row.
  return split.first.row(split.first.rows() - 1).transpose();
}

inline double evaluateBernstein(
    const Eigen::Ref<const Eigen::VectorXd> &coeffs,
    double u)
{
  Eigen::MatrixXd as_matrix(coeffs.size(), 1);
  as_matrix.col(0) = coeffs;
  return evaluateBezier(as_matrix, u)(0);
}

} // namespace convex_hull
} // namespace traj_opt
