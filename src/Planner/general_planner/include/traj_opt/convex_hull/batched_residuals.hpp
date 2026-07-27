#pragma once

#include "traj_opt/convex_hull/bezier_product.hpp"
#include "traj_opt/convex_hull/constraint_pack.hpp"
#include "traj_opt/convex_hull/flatness_convex_hull_cost.hpp"
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
  bool trust_region_feasible{true};
};

inline Eigen::MatrixXd packedDegreeElevationMatrix(int low_degree,
                                                   int high_degree)
{
  Eigen::MatrixXd elevation =
      Eigen::MatrixXd::Zero(high_degree + 1, low_degree + 1);
  const int extra = high_degree - low_degree;
  for (int i = 0; i <= high_degree; ++i)
  {
    const int begin = std::max(0, i - extra);
    const int end = std::min(low_degree, i);
    const double denominator = binomialCoefficient(high_degree, i);
    for (int j = begin; j <= end; ++j)
    {
      elevation(i, j) =
          binomialCoefficient(low_degree, j) *
          binomialCoefficient(extra, i - j) / denominator;
    }
  }
  return elevation;
}

inline Eigen::Vector3d packedUnitGradient(const Eigen::Vector3d &value,
                                          double epsilon)
{
  const double norm = value.norm();
  if (norm > epsilon)
  {
    return value / norm;
  }
  return Eigen::Vector3d::Zero();
}

struct PackedFlatnessEvaluation
{
  double normalized{0.0};
  std::array<Eigen::MatrixXd, 4> leaf_gradients;
};

inline PackedFlatnessEvaluation evaluatePackedFlatnessConstraint(
    const PackedConstraint &constraint,
    const std::array<Eigen::MatrixXd, 4> &leaves,
    const FlatnessConvexHullCost::Config &config)
{
  PackedFlatnessEvaluation out;
  for (int order = 0; order <= 3; ++order)
  {
    out.leaf_gradients[static_cast<std::size_t>(order)] =
        Eigen::MatrixXd::Zero(leaves[static_cast<std::size_t>(order)].rows(),
                              3);
  }

  const int vel_cp = static_cast<int>(leaves[1].rows());
  if (vel_cp <= 0)
  {
    return out;
  }
  const Eigen::MatrixXd acc_elevation =
      packedDegreeElevationMatrix(static_cast<int>(leaves[2].rows()) - 1,
                                  vel_cp - 1);
  const Eigen::MatrixXd jerk_elevation =
      packedDegreeElevationMatrix(static_cast<int>(leaves[3].rows()) - 1,
                                  vel_cp - 1);
  const Eigen::MatrixXd common_acceleration = acc_elevation * leaves[2];
  const Eigen::MatrixXd common_jerk = jerk_elevation * leaves[3];
  const int index = std::clamp(constraint.control_or_bernstein_index,
                               0,
                               vel_cp - 1);
  const Eigen::Vector3d velocity = leaves[1].row(index).transpose();
  const Eigen::Vector3d acceleration =
      common_acceleration.row(index).transpose();
  const Eigen::Vector3d jerk = common_jerk.row(index).transpose();
  const auto &reference = constraint.flatness;

  const double alpha = config.horizontal_drag / config.mass;
  const double abs_drag_difference =
      std::abs(config.vertical_drag - config.horizontal_drag);
  const double jacobian_lipschitz =
      4.0 * std::abs(config.parasitic_drag);
  const double angular_remainder_gain =
      std::abs(alpha) * jacobian_lipschitz * reference.trust_radius;
  const double angular_map_gain =
      1.0 / std::max(std::cos(0.5 * config.max_tilt_angle),
                     config.norm_epsilon);
  const double angular_budget =
      std::max(0.0,
               config.max_angular_rate -
                   std::abs(config.absolute_yaw_rate_bound));

  const Eigen::Vector3d affine_drag =
      reference.drag_jacobian * velocity + reference.drag_bias;
  const Eigen::Vector3d force =
      acceleration + alpha * affine_drag +
      config.gravity * Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d force_rate =
      jerk + alpha * reference.drag_jacobian * acceleration;
  const double force_norm = force.norm();
  const double force_rate_norm = force_rate.norm();
  const double acceleration_norm = acceleration.norm();
  const double drag_norm = affine_drag.norm();
  const double projected =
      reference.force_direction.dot(force) - reference.force_remainder;

  Eigen::Vector3d velocity_gradient = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_gradient = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk_gradient = Eigen::Vector3d::Zero();
  Eigen::Vector3d drag_gradient = Eigen::Vector3d::Zero();
  Eigen::Vector3d force_gradient = Eigen::Vector3d::Zero();
  Eigen::Vector3d force_rate_gradient = Eigen::Vector3d::Zero();
  double raw = 0.0;
  double scale = 1.0;

  switch (constraint.kind)
  {
    case ConstraintKind::FlatnessVelocityTrust:
    {
      const Eigen::Vector3d delta =
          velocity - reference.reference_velocity;
      raw = delta.norm() - reference.trust_radius;
      velocity_gradient = packedUnitGradient(delta, config.norm_epsilon);
      scale = std::max(1.0e-6, reference.trust_radius);
      break;
    }
    case ConstraintKind::FlatnessTilt:
    {
      const Eigen::Vector3d force_xy(force.x(), force.y(), 0.0);
      const double xy_norm = force_xy.norm();
      const double tangent = std::tan(config.max_tilt_angle);
      raw = xy_norm + reference.force_remainder -
            tangent * (force.z() - reference.force_remainder);
      if (xy_norm > config.norm_epsilon)
      {
        force_gradient.x() = force.x() / xy_norm;
        force_gradient.y() = force.y() / xy_norm;
      }
      force_gradient.z() = -tangent;
      scale = std::max(1.0, config.gravity);
      break;
    }
    case ConstraintKind::FlatnessThrustUpper:
      raw = config.mass * (force_norm + reference.force_remainder) +
            abs_drag_difference *
                (drag_norm + reference.drag_remainder) -
            config.max_thrust;
      force_gradient =
          config.mass * packedUnitGradient(force, config.norm_epsilon);
      drag_gradient =
          abs_drag_difference *
          packedUnitGradient(affine_drag, config.norm_epsilon);
      scale = std::max(1.0, std::abs(config.max_thrust));
      break;
    case ConstraintKind::FlatnessThrustLower:
      raw = config.min_thrust -
            config.mass * projected +
            abs_drag_difference *
                (drag_norm + reference.drag_remainder);
      force_gradient = -config.mass * reference.force_direction;
      drag_gradient =
          abs_drag_difference *
          packedUnitGradient(affine_drag, config.norm_epsilon);
      scale = std::max(1.0, std::abs(config.min_thrust));
      break;
    case ConstraintKind::FlatnessForceProjection:
      raw = config.min_force_projection - projected;
      force_gradient = -reference.force_direction;
      scale = std::max(1.0, config.gravity);
      break;
    case ConstraintKind::FlatnessAngularRate:
    {
      const Eigen::Vector3d cross = force.cross(force_rate);
      const double cross_norm = cross.norm();
      const Eigen::Vector3d u =
          packedUnitGradient(cross, config.norm_epsilon);
      const double force_error = reference.force_remainder;
      const double force_rate_error =
          angular_remainder_gain * acceleration_norm;
      const double remainder =
          force_error * force_rate_norm +
          force_rate_error * force_norm +
          force_error * force_rate_error;
      raw = angular_map_gain * (cross_norm + remainder) -
            angular_budget * projected * projected;
      force_gradient =
          angular_map_gain *
              (force_rate.cross(u) +
               force_rate_error *
                   packedUnitGradient(force, config.norm_epsilon)) -
          2.0 * angular_budget * projected *
              reference.force_direction;
      force_rate_gradient =
          angular_map_gain *
          (u.cross(force) +
           force_error *
               packedUnitGradient(force_rate, config.norm_epsilon));
      acceleration_gradient +=
          angular_map_gain * angular_remainder_gain *
          (force_norm + force_error) *
          packedUnitGradient(acceleration, config.norm_epsilon);
      scale = std::max(1.0,
                       angular_budget * config.gravity * config.gravity);
      break;
    }
    default:
      return out;
  }

  if (constraint.kind != ConstraintKind::FlatnessVelocityTrust)
  {
    drag_gradient += alpha * force_gradient;
    velocity_gradient +=
        reference.drag_jacobian.transpose() * drag_gradient;
    acceleration_gradient +=
        force_gradient +
        alpha * reference.drag_jacobian.transpose() *
            force_rate_gradient;
    jerk_gradient += force_rate_gradient;
  }

  out.normalized = raw / scale;
  out.leaf_gradients[1].row(index) =
      (velocity_gradient / scale).transpose();
  Eigen::MatrixXd common_acc_gradient =
      Eigen::MatrixXd::Zero(vel_cp, 3);
  Eigen::MatrixXd common_jerk_gradient =
      Eigen::MatrixXd::Zero(vel_cp, 3);
  common_acc_gradient.row(index) =
      (acceleration_gradient / scale).transpose();
  common_jerk_gradient.row(index) =
      (jerk_gradient / scale).transpose();
  out.leaf_gradients[2].noalias() =
      acc_elevation.transpose() * common_acc_gradient;
  out.leaf_gradients[3].noalias() =
      jerk_elevation.transpose() * common_jerk_gradient;
  return out;
}

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
    const FlatnessConvexHullCost::Config *flatness_config,
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
    if (constraint.source_segment < 0 ||
        constraint.source_segment >= durations.size())
    {
      continue;
    }

    const bool flatness_constraint =
        constraint.kind >= ConstraintKind::FlatnessVelocityTrust &&
        constraint.kind <= ConstraintKind::FlatnessAngularRate;
    if (flatness_constraint)
    {
      if (flatness_config == nullptr)
      {
        continue;
      }
      std::array<Eigen::MatrixXd, 4> leaves;
      std::array<Eigen::MatrixXd, 4> selections;
      for (int derivative = 0; derivative <= 3; ++derivative)
      {
        const int derivative_cp = controls_per_piece - derivative;
        selections[static_cast<std::size_t>(derivative)] =
            leafSelectionMatrix(derivative_cp - 1,
                                constraint.depth,
                                constraint.binary_index);
        const Eigen::MatrixXd source =
            order_controls[static_cast<std::size_t>(derivative)].middleRows(
                constraint.source_segment * derivative_cp,
                derivative_cp);
        leaves[static_cast<std::size_t>(derivative)] =
            selections[static_cast<std::size_t>(derivative)] * source;
      }
      result.scalar_checks +=
          static_cast<std::size_t>(leaves[1].rows());
      const auto flatness = evaluatePackedFlatnessConstraint(
          constraint, leaves, *flatness_config);
      const double normalized = flatness.normalized;
      if (constraint.kind == ConstraintKind::FlatnessVelocityTrust &&
          normalized > 1.0e-10)
      {
        result.trust_region_feasible = false;
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
        for (int derivative = 1; derivative <= 3; ++derivative)
        {
          const int derivative_cp = controls_per_piece - derivative;
          order_gradients[static_cast<std::size_t>(derivative)].middleRows(
              constraint.source_segment * derivative_cp,
              derivative_cp) +=
              selections[static_cast<std::size_t>(derivative)].transpose() *
              (shifted *
               flatness.leaf_gradients[
                   static_cast<std::size_t>(derivative)]);
        }
      }
      continue;
    }

    if (order < 0 || order > 3)
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
 * Reverse the physical hodograph chain for derivative orders 1..3.
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
