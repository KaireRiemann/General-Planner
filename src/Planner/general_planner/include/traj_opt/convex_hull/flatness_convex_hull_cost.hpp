#pragma once

/*
 * Experimental GCOPTER-style flatness envelope (tilt / thrust /
 * angular-rate) on General-Planner's Bezier hodograph layout.
 *
 * Tilt and thrust use convex sufficient bounds. The current angular row
 * evaluates a non-convex cross-product expression at matching joint controls,
 * so the combined result is advisory rather than a production certificate.
 *
 * Call refreshReference() only between L-BFGS major iterations. Inner
 * evaluations must keep the frozen affine drag model fixed.
 */

#include <Eigen/Dense>

#include "traj_opt/solver_quality_report.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace traj_opt
{
namespace convex_hull
{

class FlatnessConvexHullCost
{
public:
  using Vector = Eigen::Vector3d;
  using Matrix3 = Eigen::Matrix3d;

  struct Weights
  {
    double velocity_trust_region{1.0e5};
    double tilt{1.0e4};
    double thrust{1.0e5};
    double angular_rate{1.0e4};
    double force_projection{1.0e5};
  };

  struct Config
  {
    double mass{1.0};
    double gravity{9.81};
    double horizontal_drag{0.0};
    double vertical_drag{0.0};
    double parasitic_drag{0.0};
    double speed_smoothing{1.0e-4};

    double max_angular_rate{std::numeric_limits<double>::infinity()};
    double max_tilt_angle{1.05};
    double min_thrust{0.0};
    double max_thrust{std::numeric_limits<double>::infinity()};
    double absolute_yaw_rate_bound{0.0};
    double min_force_projection{1.0e-3};

    double trust_radius_scale{1.20};
    double trust_radius_padding{0.10};
    double minimum_trust_radius{0.20};
    double smooth_epsilon{1.0e-2};
    double norm_epsilon{1.0e-12};
    double certificate_tolerance{1.0e-5};
    Weights weights{};
  };

  struct Diagnostics
  {
    double objective{0.0};
    double max_velocity_trust_violation{0.0};
    double max_tilt_violation{0.0};
    double max_thrust_upper_violation{0.0};
    double max_thrust_lower_violation{0.0};
    double max_angular_rate_violation{0.0};
    double max_force_projection_violation{0.0};
    int num_pieces{0};
    int num_joint_controls{0};
    bool reference_ready{false};
    bool certified{false};

    double maxViolation() const
    {
      return std::max({max_velocity_trust_violation,
                       max_tilt_violation,
                       max_thrust_upper_violation,
                       max_thrust_lower_violation,
                       max_angular_rate_violation,
                       max_force_projection_violation});
    }
  };

  void configure(const Config &config) { config_ = config; }
  const Config &config() const { return config_; }
  const Diagnostics &diagnostics() const { return diagnostics_; }
  const std::vector<ConstraintCandidate> &constraintCandidates() const
  {
    return candidates_;
  }
  double normalizedMaxViolation() const
  {
    double maximum = 0.0;
    for (const auto &candidate : candidates_)
    {
      maximum = std::max(maximum, candidate.value);
    }
    return maximum;
  }
  bool referenceReady() const { return reference_ready_; }

  void clearReference()
  {
    reference_ready_ = false;
    references_.clear();
  }

  /**
   * Freeze per-leaf affine drag / force-direction model from current trajectory
   * and velocity Bezier controls (hodograph layout: pieces * vel_cp rows).
   */
  template <typename Trajectory>
  void refreshReference(const Trajectory &trajectory,
                        int num_pieces,
                        int leaves_per_segment,
                        const Eigen::MatrixXd &velocity_controls,
                        int vel_controls_per_piece,
                        const Eigen::VectorXd &durations)
  {
    references_.assign(static_cast<std::size_t>(num_pieces), PieceReference{});
    leaves_per_segment_ = std::max(1, leaves_per_segment);
    const double jacobian_lipschitz = dragJacobianLipschitz();
    const double alpha = config_.horizontal_drag / config_.mass;

    double absolute_time = 0.0;
    int prev_segment = -1;
    for (int piece = 0; piece < num_pieces; ++piece)
    {
      const int segment = piece / std::max(1, leaves_per_segment);
      if (segment != prev_segment)
      {
        if (prev_segment >= 0 && prev_segment < durations.size())
        {
          absolute_time += durations(prev_segment);
        }
        prev_segment = segment;
      }
      const double leaf_duration =
          durations(segment) / static_cast<double>(std::max(1, leaves_per_segment));
      const int leaf_in_segment = piece % std::max(1, leaves_per_segment);
      const double midpoint =
          absolute_time + (leaf_in_segment + 0.5) * leaf_duration;

      PieceReference &reference = references_[static_cast<std::size_t>(piece)];
      reference.velocity = trajectory.getVel(midpoint);
      const auto velocity_block = velocity_controls.middleRows(
          piece * vel_controls_per_piece, vel_controls_per_piece);
      double maximum_distance = 0.0;
      for (int local = 0; local < vel_controls_per_piece; ++local)
      {
        maximum_distance = std::max(
            maximum_distance,
            (velocity_block.row(local).transpose() - reference.velocity)
                .norm());
      }
      reference.trust_radius = std::max(
          config_.minimum_trust_radius,
          std::max(config_.trust_radius_scale * maximum_distance,
                   maximum_distance) +
              config_.trust_radius_padding);
      reference.anchor_radius = maximum_distance;

      Vector drag_at_reference;
      dragModel(reference.velocity, drag_at_reference, reference.drag_jacobian);
      reference.drag_bias =
          drag_at_reference - reference.drag_jacobian * reference.velocity;

      const Vector acceleration = trajectory.getAcc(midpoint);
      const Vector actual_force =
          acceleration + alpha * drag_at_reference +
          config_.gravity * Vector::UnitZ();
      const double actual_force_norm = actual_force.norm();
      if (actual_force_norm > config_.norm_epsilon)
      {
        reference.force_direction = actual_force / actual_force_norm;
      }
      else
      {
        reference.force_direction = Vector::UnitZ();
      }
      reference.drag_remainder =
          0.5 * jacobian_lipschitz * reference.trust_radius *
          reference.trust_radius;
      reference.force_remainder =
          std::abs(alpha) * reference.drag_remainder;
    }
    reference_ready_ = !references_.empty();
    ++reference_revision_;
  }

  /**
   * Evaluate flatness penalties on hodograph controls and accumulate gradients
   * into the same layout. Acc/jerk are degree-elevated to the velocity degree.
   * Accepts any Eigen dense matrix with 3 columns (e.g. HullMatrix).
   */
  template <typename VelMat, typename AccMat, typename JerkMat,
            typename VelGrad, typename AccGrad, typename JerkGrad>
  double accumulate(const Eigen::MatrixBase<VelMat> &velocity_controls,
                    const Eigen::MatrixBase<AccMat> &acceleration_controls,
                    const Eigen::MatrixBase<JerkMat> &jerk_controls,
                    int num_pieces,
                    int vel_cp,
                    int acc_cp,
                    int jerk_cp,
                    Eigen::MatrixBase<VelGrad> &velocity_gradients,
                    Eigen::MatrixBase<AccGrad> &acceleration_gradients,
                    Eigen::MatrixBase<JerkGrad> &jerk_gradients) const
  {
    diagnostics_ = Diagnostics{};
    candidates_.clear();
    diagnostics_.num_pieces = num_pieces;
    diagnostics_.num_joint_controls = num_pieces * vel_cp;
    diagnostics_.reference_ready = reference_ready_;
    if (!reference_ready_ || num_pieces <= 0 || vel_cp <= 0 ||
        static_cast<int>(references_.size()) != num_pieces ||
        velocity_controls.rows() != num_pieces * vel_cp ||
        acceleration_controls.rows() != num_pieces * acc_cp ||
        jerk_controls.rows() != num_pieces * jerk_cp)
    {
      return 0.0;
    }

    ensureElevation(acc_cp - 1, jerk_cp - 1, vel_cp - 1);

    common_velocity_.resize(num_pieces * vel_cp, 3);
    common_acceleration_.resize(num_pieces * vel_cp, 3);
    common_jerk_.resize(num_pieces * vel_cp, 3);
    common_velocity_gradient_.setZero(num_pieces * vel_cp, 3);
    common_acceleration_gradient_.setZero(num_pieces * vel_cp, 3);
    common_jerk_gradient_.setZero(num_pieces * vel_cp, 3);

    common_velocity_ = velocity_controls;
    for (int piece = 0; piece < num_pieces; ++piece)
    {
      const int common_row = piece * vel_cp;
      common_acceleration_.middleRows(common_row, vel_cp).noalias() =
          acceleration_elevation_ *
          acceleration_controls.middleRows(piece * acc_cp, acc_cp);
      common_jerk_.middleRows(common_row, vel_cp).noalias() =
          jerk_elevation_ * jerk_controls.middleRows(piece * jerk_cp, jerk_cp);
    }

    const double cost = evaluateJointControlPenalties();
    // Reverse elevation into native hodograph layouts.
    for (int piece = 0; piece < num_pieces; ++piece)
    {
      const int common_row = piece * vel_cp;
      velocity_gradients.derived().middleRows(piece * vel_cp, vel_cp) +=
          common_velocity_gradient_.middleRows(common_row, vel_cp);
      acceleration_gradients.derived()
          .middleRows(piece * acc_cp, acc_cp)
          .noalias() +=
          acceleration_elevation_.transpose() *
          common_acceleration_gradient_.middleRows(common_row, vel_cp);
      jerk_gradients.derived()
          .middleRows(piece * jerk_cp, jerk_cp)
          .noalias() +=
          jerk_elevation_.transpose() *
          common_jerk_gradient_.middleRows(common_row, vel_cp);
    }

    diagnostics_.objective = cost;
    diagnostics_.certified =
        diagnostics_.maxViolation() <= config_.certificate_tolerance;
    return cost;
  }

private:
  struct PieceReference
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector velocity = Vector::Zero();
    Vector force_direction = Vector::UnitZ();
    Matrix3 drag_jacobian = Matrix3::Identity();
    Vector drag_bias = Vector::Zero();
    double trust_radius{0.0};
    double anchor_radius{0.0};
    double drag_remainder{0.0};
    double force_remainder{0.0};
  };

  Config config_{};
  mutable Diagnostics diagnostics_{};
  mutable std::vector<PieceReference> references_;
  mutable std::vector<ConstraintCandidate> candidates_;
  mutable int leaves_per_segment_{1};
  mutable bool reference_ready_{false};
  mutable std::uint64_t reference_revision_{0};

  mutable Eigen::MatrixXd acceleration_elevation_;
  mutable Eigen::MatrixXd jerk_elevation_;
  mutable int elev_acc_degree_{-1};
  mutable int elev_jerk_degree_{-1};
  mutable int elev_common_degree_{-1};

  mutable Eigen::MatrixXd common_velocity_;
  mutable Eigen::MatrixXd common_acceleration_;
  mutable Eigen::MatrixXd common_jerk_;
  mutable Eigen::MatrixXd common_velocity_gradient_;
  mutable Eigen::MatrixXd common_acceleration_gradient_;
  mutable Eigen::MatrixXd common_jerk_gradient_;

  static double binomial(int n, int k)
  {
    if (k < 0 || k > n)
    {
      return 0.0;
    }
    k = std::min(k, n - k);
    double value = 1.0;
    for (int i = 1; i <= k; ++i)
    {
      value *= static_cast<double>(n - k + i) / static_cast<double>(i);
    }
    return value;
  }

  static Eigen::MatrixXd degreeElevationMatrix(int low_degree, int high_degree)
  {
    Eigen::MatrixXd elevation =
        Eigen::MatrixXd::Zero(high_degree + 1, low_degree + 1);
    const int extra = high_degree - low_degree;
    for (int i = 0; i <= high_degree; ++i)
    {
      const int j_begin = std::max(0, i - extra);
      const int j_end = std::min(low_degree, i);
      const double denominator = binomial(high_degree, i);
      for (int j = j_begin; j <= j_end; ++j)
      {
        elevation(i, j) =
            binomial(low_degree, j) * binomial(extra, i - j) / denominator;
      }
    }
    return elevation;
  }

  void ensureElevation(int acc_degree, int jerk_degree, int common_degree) const
  {
    if (elev_acc_degree_ == acc_degree && elev_jerk_degree_ == jerk_degree &&
        elev_common_degree_ == common_degree)
    {
      return;
    }
    acceleration_elevation_ =
        degreeElevationMatrix(acc_degree, common_degree);
    jerk_elevation_ = degreeElevationMatrix(jerk_degree, common_degree);
    elev_acc_degree_ = acc_degree;
    elev_jerk_degree_ = jerk_degree;
    elev_common_degree_ = common_degree;
  }

  void dragModel(const Vector &velocity,
                 Vector &drag,
                 Matrix3 &jacobian) const
  {
    const double speed = std::sqrt(std::max(
        velocity.squaredNorm() + config_.speed_smoothing,
        config_.norm_epsilon * config_.norm_epsilon));
    const double scale = 1.0 + config_.parasitic_drag * speed;
    drag = scale * velocity;
    jacobian = scale * Matrix3::Identity();
    if (speed > config_.norm_epsilon)
    {
      jacobian.noalias() += (config_.parasitic_drag / speed) *
                            (velocity * velocity.transpose());
    }
  }

  double dragJacobianLipschitz() const
  {
    return 4.0 * std::abs(config_.parasitic_drag);
  }

  static Vector unitGradient(const Vector &value, double norm, double epsilon)
  {
    if (norm <= epsilon)
    {
      return Vector::Zero();
    }
    return value / norm;
  }

  static void smoothPositivePart(double violation,
                                 double epsilon,
                                 double &value,
                                 double &derivative)
  {
    if (!(violation > 0.0))
    {
      value = 0.0;
      derivative = 0.0;
      return;
    }
    if (!(epsilon > 0.0) || violation >= epsilon)
    {
      value = violation - (epsilon > 0.0 ? 0.5 * epsilon : 0.0);
      derivative = 1.0;
      return;
    }
    const double ratio = violation / epsilon;
    const double ratio2 = ratio * ratio;
    value = epsilon * ratio2 * ratio * (1.0 - 0.5 * ratio);
    derivative = ratio2 * (3.0 - 2.0 * ratio);
  }

  double addPenalty(double violation,
                    double weight,
                    double &gradient_multiplier,
                    double &maximum_violation) const
  {
    maximum_violation = std::max(maximum_violation, violation);
    if (!(weight > 0.0))
    {
      gradient_multiplier = 0.0;
      return 0.0;
    }
    double hinge_value = 0.0;
    double hinge_derivative = 0.0;
    smoothPositivePart(violation, config_.smooth_epsilon, hinge_value,
                       hinge_derivative);
    gradient_multiplier = weight * hinge_derivative;
    return weight * hinge_value;
  }

  double evaluateJointControlPenalties() const
  {
    const double alpha = config_.horizontal_drag / config_.mass;
    const double abs_alpha = std::abs(alpha);
    const double abs_drag_difference =
        std::abs(config_.vertical_drag - config_.horizontal_drag);
    const double jacobian_lipschitz = dragJacobianLipschitz();
    const double tangent_tilt = std::tan(config_.max_tilt_angle);
    const double angular_map_gain =
        1.0 / std::max(std::cos(0.5 * config_.max_tilt_angle),
                       config_.norm_epsilon);
    const double angular_rate_budget = std::max(
        0.0, config_.max_angular_rate -
                 std::abs(config_.absolute_yaw_rate_bound));
    const int vel_cp = elev_common_degree_ + 1;
    double total_cost = 0.0;

    for (int piece = 0; piece < static_cast<int>(references_.size()); ++piece)
    {
      const PieceReference &reference =
          references_[static_cast<std::size_t>(piece)];
      const int row0 = piece * vel_cp;
      const double angular_remainder_gain =
          abs_alpha * jacobian_lipschitz * reference.trust_radius;

      for (int local = 0; local < vel_cp; ++local)
      {
        const int row = row0 + local;
        const Vector velocity = common_velocity_.row(row).transpose();
        const Vector acceleration =
            common_acceleration_.row(row).transpose();
        const Vector jerk = common_jerk_.row(row).transpose();

        const Vector affine_drag =
            reference.drag_jacobian * velocity + reference.drag_bias;
        const Vector force =
            acceleration + alpha * affine_drag +
            config_.gravity * Vector::UnitZ();
        const Vector force_rate =
            jerk + alpha * reference.drag_jacobian * acceleration;

        Vector velocity_gradient = Vector::Zero();
        Vector acceleration_gradient = Vector::Zero();
        Vector jerk_gradient = Vector::Zero();
        Vector drag_gradient = Vector::Zero();
        Vector force_gradient = Vector::Zero();
        Vector force_rate_gradient = Vector::Zero();
        double multiplier = 0.0;

        if (jacobian_lipschitz > 0.0)
        {
          const Vector trust_delta = velocity - reference.velocity;
          const double trust_norm = trust_delta.norm();
          const double trust_violation =
              trust_norm - reference.trust_radius;
          total_cost += addPenalty(
              trust_violation, config_.weights.velocity_trust_region,
              multiplier, diagnostics_.max_velocity_trust_violation);
          velocity_gradient +=
              multiplier * unitGradient(trust_delta, trust_norm,
                                        config_.norm_epsilon);
        }

        const bool angular_enabled =
            std::isfinite(config_.max_angular_rate) &&
            config_.max_angular_rate > 0.0;
        const bool thrust_upper_enabled =
            std::isfinite(config_.max_thrust) &&
            config_.max_thrust > 0.0;
        const bool thrust_lower_enabled =
            std::isfinite(config_.min_thrust) &&
            config_.min_thrust >= 0.0;
        const bool attitude_enabled =
            angular_enabled || thrust_upper_enabled || thrust_lower_enabled;

        const Vector force_xy(force.x(), force.y(), 0.0);
        const double force_xy_norm = force_xy.norm();
        const double tilt_violation =
            force_xy_norm + reference.force_remainder -
            tangent_tilt * (force.z() - reference.force_remainder);
        if (attitude_enabled)
        {
          total_cost += addPenalty(
              tilt_violation, config_.weights.tilt,
              multiplier, diagnostics_.max_tilt_violation);
          addCandidate(ConstraintKind::FlatnessTilt,
                       piece,
                       local,
                       tilt_violation,
                       reference,
                       std::max(1.0, config_.gravity));
          if (force_xy_norm > config_.norm_epsilon)
          {
            force_gradient.x() += multiplier * force.x() / force_xy_norm;
            force_gradient.y() += multiplier * force.y() / force_xy_norm;
          }
          force_gradient.z() -= multiplier * tangent_tilt;
        }

        const double force_norm = force.norm();
        const double drag_norm = affine_drag.norm();
        const Vector force_unit =
            unitGradient(force, force_norm, config_.norm_epsilon);
        const Vector drag_unit =
            unitGradient(affine_drag, drag_norm, config_.norm_epsilon);

        if (thrust_upper_enabled)
        {
          const double thrust_upper_violation =
              config_.mass * (force_norm + reference.force_remainder) +
              abs_drag_difference *
                  (drag_norm + reference.drag_remainder) -
              config_.max_thrust;
          total_cost += addPenalty(
              thrust_upper_violation, config_.weights.thrust, multiplier,
              diagnostics_.max_thrust_upper_violation);
          addCandidate(ConstraintKind::FlatnessThrustUpper,
                       piece,
                       local,
                       thrust_upper_violation,
                       reference,
                       std::max(1.0, std::abs(config_.max_thrust)));
          force_gradient += multiplier * config_.mass * force_unit;
          drag_gradient += multiplier * abs_drag_difference * drag_unit;
        }

        if (thrust_lower_enabled)
        {
          const double thrust_lower_certificate =
              config_.mass * (reference.force_direction.dot(force) -
                              reference.force_remainder) -
              abs_drag_difference *
                  (drag_norm + reference.drag_remainder);
          const double thrust_lower_violation =
              config_.min_thrust - thrust_lower_certificate;
          total_cost += addPenalty(
              thrust_lower_violation, config_.weights.thrust, multiplier,
              diagnostics_.max_thrust_lower_violation);
          addCandidate(ConstraintKind::FlatnessThrustLower,
                       piece,
                       local,
                       thrust_lower_violation,
                       reference,
                       std::max(1.0, std::abs(config_.min_thrust)));
          force_gradient -=
              multiplier * config_.mass * reference.force_direction;
          drag_gradient += multiplier * abs_drag_difference * drag_unit;
        }

        const double projected_force =
            reference.force_direction.dot(force) - reference.force_remainder;
        if (attitude_enabled)
        {
          const double projection_violation =
              config_.min_force_projection - projected_force;
          total_cost += addPenalty(
              projection_violation, config_.weights.force_projection,
              multiplier, diagnostics_.max_force_projection_violation);
          addCandidate(ConstraintKind::FlatnessForceProjection,
                       piece,
                       local,
                       projection_violation,
                       reference,
                       std::max(1.0, config_.gravity));
          force_gradient -= multiplier * reference.force_direction;
        }

        if (angular_enabled)
        {
          const double force_rate_norm = force_rate.norm();
          const double acceleration_norm = acceleration.norm();
          const Vector cross = force.cross(force_rate);
          const double cross_norm = cross.norm();
          const Vector cross_unit =
              unitGradient(cross, cross_norm, config_.norm_epsilon);
          // True force and force-rate are enclosed by the affine Taylor
          // model. Bound the missing cross-product terms explicitly:
          //   |(f+ef)x(fd+ed)| <= |fxfd| + ef|fd| + ed|f| + ef*ed.
          const double force_error = reference.force_remainder;
          const double force_rate_error =
              angular_remainder_gain * acceleration_norm;
          const double cross_remainder =
              force_error * force_rate_norm +
              force_rate_error * force_norm +
              force_error * force_rate_error;
          const double angular_violation =
              angular_map_gain *
                  (cross_norm + cross_remainder) -
              angular_rate_budget * projected_force * projected_force;
          total_cost += addPenalty(
              angular_violation, config_.weights.angular_rate, multiplier,
              diagnostics_.max_angular_rate_violation);
          addCandidate(
              ConstraintKind::FlatnessAngularRate,
              piece,
              local,
              angular_violation,
              reference,
              std::max(1.0,
                       angular_rate_budget * config_.gravity *
                           config_.gravity));

          // d|f x fd|/df = fd x u, d|f x fd|/dfd = u x f.
          force_gradient +=
              multiplier * angular_map_gain *
              (force_rate.cross(cross_unit) +
               force_rate_error *
                   unitGradient(force, force_norm, config_.norm_epsilon));
          force_rate_gradient +=
              multiplier * angular_map_gain *
              (cross_unit.cross(force) +
               force_error *
                   unitGradient(force_rate,
                                force_rate_norm,
                                config_.norm_epsilon));
          acceleration_gradient +=
              multiplier * angular_map_gain * angular_remainder_gain *
              (force_norm + force_error) *
              unitGradient(acceleration, acceleration_norm,
                           config_.norm_epsilon);
          force_gradient -=
              multiplier * 2.0 * angular_rate_budget * projected_force *
              reference.force_direction;
        }

        drag_gradient += alpha * force_gradient;
        velocity_gradient +=
            reference.drag_jacobian.transpose() * drag_gradient;
        acceleration_gradient +=
            force_gradient +
            alpha * reference.drag_jacobian.transpose() * force_rate_gradient;
        jerk_gradient += force_rate_gradient;

        common_velocity_gradient_.row(row) += velocity_gradient.transpose();
        common_acceleration_gradient_.row(row) +=
            acceleration_gradient.transpose();
        common_jerk_gradient_.row(row) += jerk_gradient.transpose();
      }
    }
    return total_cost;
  }

  void addCandidate(ConstraintKind kind,
                    int piece,
                    int local,
                    double raw_value,
                    const PieceReference &reference,
                    double scale) const
  {
    if (!(raw_value > config_.certificate_tolerance))
    {
      return;
    }
    ConstraintCandidate candidate;
    candidate.kind = kind;
    candidate.source_segment = piece / std::max(1, leaves_per_segment_);
    candidate.derivative_order = -1;
    candidate.depth = 0;
    int leaves = std::max(1, leaves_per_segment_);
    while ((1 << candidate.depth) < leaves)
    {
      ++candidate.depth;
    }
    candidate.binary_index = piece % leaves;
    candidate.control_or_bernstein_index = local;
    candidate.value = raw_value / std::max(1.0e-9, scale);
    candidate.margin = -candidate.value;
    candidate.flatness.reference_velocity = reference.velocity;
    candidate.flatness.force_direction = reference.force_direction;
    candidate.flatness.drag_jacobian = reference.drag_jacobian;
    candidate.flatness.drag_bias = reference.drag_bias;
    candidate.flatness.trust_radius = reference.trust_radius;
    candidate.flatness.anchor_radius = reference.anchor_radius;
    candidate.flatness.drag_remainder = reference.drag_remainder;
    candidate.flatness.force_remainder = reference.force_remainder;
    candidates_.push_back(candidate);
  }
};

} // namespace convex_hull
} // namespace traj_opt
