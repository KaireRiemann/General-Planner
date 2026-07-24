#pragma once

#include "traj_opt/convex_hull/bezier_product.hpp"
#include "traj_opt/convex_hull/constraint_pack.hpp"
#include "traj_opt/convex_hull/scalar_bernstein.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace traj_opt
{
namespace convex_hull
{

/**
 * Matrix S such that leaf_controls = S * source_controls for a de Casteljau
 * leaf identified by (depth, binary_index), midpoint splits.
 */
inline Eigen::MatrixXd leafSelectionMatrix(int degree,
                                           int depth,
                                           int binary_index)
{
  const int rows = degree + 1;
  Eigen::MatrixXd selection = Eigen::MatrixXd::Identity(rows, rows);
  depth = std::max(0, depth);
  for (int level = 0; level < depth; ++level)
  {
    const bool right =
        ((binary_index >> (depth - 1 - level)) & 1) != 0;
    const auto children = deCasteljauSplit(selection, 0.5);
    selection = right ? children.second : children.first;
  }
  return selection;
}

inline void buildHodographControls(
    const Eigen::MatrixXd &position_controls,
    const Eigen::VectorXd &durations,
    int controls_per_piece,
    std::array<Eigen::MatrixXd, 4> &order_controls)
{
  const int segments = static_cast<int>(durations.size());
  order_controls[0] = position_controls;
  for (int order = 1; order <= 3; ++order)
  {
    const int previous_cp = controls_per_piece - (order - 1);
    const int current_cp = previous_cp - 1;
    order_controls[static_cast<std::size_t>(order)].resize(
        segments * current_cp, 3);
    const double degree_factor = static_cast<double>(previous_cp - 1);
    for (int segment = 0; segment < segments; ++segment)
    {
      const double scale = degree_factor / std::max(durations(segment), 1.0e-9);
      const int previous_row = segment * previous_cp;
      const int current_row = segment * current_cp;
      for (int control = 0; control < current_cp; ++control)
      {
        order_controls[static_cast<std::size_t>(order)].row(
            current_row + control) =
            scale *
            (order_controls[static_cast<std::size_t>(order - 1)].row(
                 previous_row + control + 1) -
             order_controls[static_cast<std::size_t>(order - 1)].row(
                 previous_row + control));
      }
    }
  }
}

/**
 * Gradient of one squared-norm Bernstein coefficient s_k w.r.t. controls.
 */
inline Eigen::MatrixXd squaredNormBernsteinGradient(
    const Eigen::Ref<const Eigen::MatrixXd> &controls,
    int coeff_index)
{
  const int degree = static_cast<int>(controls.rows()) - 1;
  Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(controls.rows(), 3);
  if (degree < 0 || coeff_index < 0 || coeff_index > 2 * degree)
  {
    return gradient;
  }

  std::vector<double> binom_d(static_cast<std::size_t>(degree + 1));
  for (int i = 0; i <= degree; ++i)
  {
    binom_d[static_cast<std::size_t>(i)] = binomialCoefficient(degree, i);
  }
  const double denom = binomialCoefficient(2 * degree, coeff_index);
  if (denom <= 0.0)
  {
    return gradient;
  }

  const int i_min = std::max(0, coeff_index - degree);
  const int i_max = std::min(degree, coeff_index);
  for (int i = i_min; i <= i_max; ++i)
  {
    const int j = coeff_index - i;
    const double weight =
        binom_d[static_cast<std::size_t>(i)] *
        binom_d[static_cast<std::size_t>(j)] / denom;
    // s_k includes V_i·V_j; derivative w.r.t V_a accumulates both roles.
    gradient.row(i) += weight * controls.row(j);
    gradient.row(j) += weight * controls.row(i);
  }
  return gradient;
}

struct PackedResidualResult
{
  Eigen::VectorXd values;
  double phr_cost{0.0};
  std::size_t scalar_checks{0};
};

/**
 * Evaluate packed constraints with PHR merit and scatter gradients into
 * order-wise control workspaces (then caller runs reverse hodograph).
 *
 * Position: g = (a·Q_i + b) / position_scale
 * Derivative: g = (s_k - bound^2) / bound^2  (Bernstein), falling back to
 *             vector control residual when the index is in range of controls.
 */
template <typename Polyhedra>
inline PackedResidualResult evaluatePackedResiduals(
    const PackedConstraintSet &packed,
    const std::array<Eigen::MatrixXd, 4> &order_controls,
    const Eigen::VectorXd &durations,
    int controls_per_piece,
    const Polyhedra &h_polys,
    const Eigen::VectorXi &h_poly_idx,
    const Eigen::VectorXd &magnitude_bounds,
    double position_scale,
    const Eigen::VectorXd &multipliers,
    double penalty,
    std::array<Eigen::MatrixXd, 4> &order_gradients)
{
  PackedResidualResult result;
  result.values = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(packed.constraints.size()));
  position_scale = std::max(position_scale, 1.0e-6);
  penalty = std::max(penalty, 1.0e-12);

  for (int order = 0; order <= 3; ++order)
  {
    order_gradients[static_cast<std::size_t>(order)].setZero(
        order_controls[static_cast<std::size_t>(order)].rows(), 3);
  }

  for (std::size_t i = 0; i < packed.constraints.size(); ++i)
  {
    const auto &constraint = packed.constraints[i];
    const int order = constraint.derivative_order;
    if (order < 0 || order > 3 ||
        constraint.source_segment < 0 ||
        constraint.source_segment >= durations.size())
    {
      continue;
    }
    const int cp = controls_per_piece - order;
    if (cp <= 0)
    {
      continue;
    }
    const Eigen::MatrixXd source =
        order_controls[static_cast<std::size_t>(order)].middleRows(
            constraint.source_segment * cp, cp);
    const Eigen::MatrixXd selection =
        leafSelectionMatrix(cp - 1, constraint.depth, constraint.binary_index);
    const Eigen::MatrixXd leaf = selection * source;
    result.scalar_checks += static_cast<std::size_t>(leaf.rows());

    double normalized = 0.0;
    Eigen::MatrixXd leaf_gradient = Eigen::MatrixXd::Zero(leaf.rows(), 3);

    if (order == 0)
    {
      if (constraint.source_segment >= h_poly_idx.size())
      {
        continue;
      }
      const int poly_id = h_poly_idx(constraint.source_segment);
      if (poly_id < 0 || poly_id >= static_cast<int>(h_polys.size()) ||
          constraint.plane_id < 0 ||
          constraint.plane_id >= h_polys[static_cast<std::size_t>(poly_id)].rows())
      {
        continue;
      }
      const auto &poly = h_polys[static_cast<std::size_t>(poly_id)];
      const Eigen::Vector3d normal =
          poly.template block<1, 3>(constraint.plane_id, 0).transpose();
      const int index = std::clamp(constraint.control_or_bernstein_index,
                                   0,
                                   static_cast<int>(leaf.rows()) - 1);
      const double raw =
          normal.dot(leaf.row(index).transpose()) + poly(constraint.plane_id, 3);
      normalized = raw / position_scale;
      leaf_gradient.row(index) = (normal / position_scale).transpose();
    }
    else
    {
      if (order - 1 >= magnitude_bounds.size())
      {
        continue;
      }
      const double bound = magnitude_bounds(order - 1);
      if (!(bound > 0.0))
      {
        continue;
      }
      const double bound_sq = bound * bound;
      const Eigen::VectorXd residuals =
          squaredNormBoundResiduals(leaf, bound);
      int index = constraint.control_or_bernstein_index;
      if (index < 0 || index >= residuals.size())
      {
        index = 0;
        for (int k = 1; k < residuals.size(); ++k)
        {
          if (residuals(k) > residuals(index))
          {
            index = k;
          }
        }
      }
      normalized = residuals(index) / bound_sq;
      leaf_gradient =
          squaredNormBernsteinGradient(leaf, index) / bound_sq;
    }

    result.values(static_cast<Eigen::Index>(i)) = normalized;
    const double multiplier =
        (multipliers.size() ==
         static_cast<Eigen::Index>(packed.constraints.size()))
            ? multipliers(static_cast<Eigen::Index>(i))
            : 0.0;
    const double shifted = multiplier + penalty * normalized;
    result.phr_cost -= 0.5 * multiplier * multiplier / penalty;
    if (shifted > 0.0)
    {
      result.phr_cost += 0.5 * shifted * shifted / penalty;
      const Eigen::MatrixXd source_gradient =
          selection.transpose() * (shifted * leaf_gradient);
      order_gradients[static_cast<std::size_t>(order)].middleRows(
          constraint.source_segment * cp, cp) += source_gradient;
    }
  }

  return result;
}

/**
 * Reverse physical hodograph gradients into position controls / durations.
 * Mirrors ExpConvexAlmCostManager::backwardHodographs for orders 1..3.
 */
inline void reverseHodographGradients(
    std::array<Eigen::MatrixXd, 4> &order_gradients,
    const std::array<Eigen::MatrixXd, 4> &order_controls,
    const Eigen::VectorXd &durations,
    int controls_per_piece,
    Eigen::VectorXd &grad_durations)
{
  const int segments = static_cast<int>(durations.size());
  for (int order = 3; order >= 1; --order)
  {
    const int previous_cp = controls_per_piece - (order - 1);
    const int current_cp = previous_cp - 1;
    const double degree_factor = static_cast<double>(previous_cp - 1);
    for (int segment = 0; segment < segments; ++segment)
    {
      const double duration = std::max(durations(segment), 1.0e-9);
      const double scale = degree_factor / duration;
      const int previous_row = segment * previous_cp;
      const int current_row = segment * current_cp;
      for (int control = 0; control < current_cp; ++control)
      {
        const Eigen::RowVector3d grad =
            order_gradients[static_cast<std::size_t>(order)].row(
                current_row + control);
        order_gradients[static_cast<std::size_t>(order - 1)].row(
            previous_row + control) -= scale * grad;
        order_gradients[static_cast<std::size_t>(order - 1)].row(
            previous_row + control + 1) += scale * grad;

        const Eigen::RowVector3d delta =
            order_controls[static_cast<std::size_t>(order - 1)].row(
                previous_row + control + 1) -
            order_controls[static_cast<std::size_t>(order - 1)].row(
                previous_row + control);
        grad_durations(segment) -=
            (degree_factor / (duration * duration)) * grad.dot(delta);
      }
    }
  }
}

} // namespace convex_hull
} // namespace traj_opt
