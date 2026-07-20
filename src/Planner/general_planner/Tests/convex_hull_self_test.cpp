#include "traj_opt/convex_hull/convex_hull.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

namespace
{

using Hull = traj_opt::convex_hull::Representation<3>;
using Matrix = Hull::Matrix;

void require(bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

double fallingFactorial(int n, int r)
{
  double value = 1.0;
  for (int i = 0; i < r; ++i)
  {
    value *= static_cast<double>(n - i);
  }
  return value;
}

Eigen::Vector3d evaluateDerivative(
    const Matrix &coefficients,
    int source_num_coeffs,
    int segment,
    int derivative,
    double local_time)
{
  Eigen::Vector3d value = Eigen::Vector3d::Zero();
  double time_power = 1.0;
  for (int k = derivative; k < source_num_coeffs; ++k)
  {
    value += fallingFactorial(k, derivative) *
             time_power *
             coefficients.row(
                 segment * source_num_coeffs + k).transpose();
    time_power *= local_time;
  }
  return value;
}

void checkValues(traj_opt::convex_hull::Basis basis,
                 int derivative,
                 int subdivision_depth)
{
  constexpr int segments = 3;
  constexpr int coeff_num = 8;
  Eigen::VectorXd durations(segments);
  durations << 0.73, 1.21, 0.91;

  std::mt19937 generator(42 + derivative + subdivision_depth);
  std::normal_distribution<double> normal(0.0, 0.25);
  Matrix coefficients(segments * coeff_num, 3);
  for (Eigen::Index row = 0; row < coefficients.rows(); ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      coefficients(row, col) = normal(generator);
    }
  }

  Hull hull;
  hull.resetTopology(
      segments, coeff_num, basis, derivative, subdivision_depth);
  hull.update(coefficients, durations, 0.37);

  const Eigen::MatrixXd control_to_power =
      Hull::powerToControlMatrix(basis, hull.degree()).inverse();
  for (int piece = 0; piece < hull.numPieces(); ++piece)
  {
    const Matrix local_power =
        control_to_power * hull.pieceControls(piece);
    const auto &info = hull.pieceInfo(piece);
    for (int sample = 0; sample <= 12; ++sample)
    {
      const double u = static_cast<double>(sample) / 12.0;
      Eigen::RowVectorXd powers(hull.degree() + 1);
      powers(0) = 1.0;
      for (int k = 1; k <= hull.degree(); ++k)
      {
        powers(k) = powers(k - 1) * u;
      }
      const Eigen::Vector3d converted =
          (powers * local_power).transpose();
      const double local_time =
          (info.source_fraction_begin +
           u * (info.source_fraction_end -
                info.source_fraction_begin)) *
          durations(info.source_segment);
      const Eigen::Vector3d expected =
          evaluateDerivative(coefficients,
                             coeff_num,
                             info.source_segment,
                             derivative,
                             local_time);
      require((converted - expected).norm() < 3.0e-8,
              "Convex-hull controls do not reproduce the source derivative.");
    }
  }
}

double linearHullObjective(const Matrix &coefficients,
                           const Eigen::VectorXd &durations,
                           traj_opt::convex_hull::Basis basis,
                           int derivative,
                           int depth,
                           const Matrix &weights)
{
  Hull hull;
  hull.resetTopology(
      static_cast<int>(durations.size()),
      static_cast<int>(coefficients.rows() / durations.size()),
      basis,
      derivative,
      depth);
  hull.update(coefficients, durations);
  return (hull.controls().array() * weights.array()).sum();
}

void checkBackward(traj_opt::convex_hull::Basis basis,
                   int derivative,
                   int depth)
{
  constexpr int segments = 2;
  constexpr int coeff_num = 6;
  Eigen::VectorXd durations(segments);
  durations << 0.82, 1.13;

  std::mt19937 generator(97 + derivative + depth);
  std::normal_distribution<double> normal(0.0, 0.3);
  Matrix coefficients(segments * coeff_num, 3);
  for (Eigen::Index row = 0; row < coefficients.rows(); ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      coefficients(row, col) = normal(generator);
    }
  }

  Hull hull;
  hull.resetTopology(
      segments, coeff_num, basis, derivative, depth);
  hull.update(coefficients, durations);
  Matrix weights(hull.controls().rows(), 3);
  for (Eigen::Index row = 0; row < weights.rows(); ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      weights(row, col) = normal(generator);
    }
  }
  const auto analytic = hull.backward(weights);

  Matrix reference_coefficients =
      Matrix::Zero(segments * coeff_num, 3);
  Eigen::VectorXd reference_durations =
      Eigen::VectorXd::Zero(segments);
  const int controls_per_segment =
      hull.piecesPerSegment() * hull.controlsPerPiece();
  for (int segment = 0; segment < segments; ++segment)
  {
    const Matrix normalized_gradient =
        hull.kernel()->stacked_control_to_power_adjoint *
        weights.middleRows(
            segment * controls_per_segment,
            controls_per_segment);
    double duration_power = 1.0;
    for (int power = 0; power <= hull.degree(); ++power)
    {
      const int source_power = power + derivative;
      const double scale =
          fallingFactorial(source_power, derivative) *
          duration_power;
      reference_coefficients.row(
          segment * coeff_num + source_power) +=
          scale * normalized_gradient.row(power);
      if (power > 0)
      {
        reference_durations(segment) +=
            (static_cast<double>(power) * scale /
             durations(segment)) *
            normalized_gradient.row(power).dot(
                coefficients.row(
                    segment * coeff_num + source_power));
      }
      duration_power *= durations(segment);
    }
  }
  require(
      (analytic.coefficients - reference_coefficients)
              .cwiseAbs()
              .maxCoeff() < 2.0e-12 &&
          (analytic.durations - reference_durations)
                  .cwiseAbs()
                  .maxCoeff() < 2.0e-12,
      "The complete-operator adjoint differs from SplineTrajectory's adjoint.");

  constexpr double epsilon = 2.0e-7;
  double max_coefficient_error = 0.0;
  for (Eigen::Index row = 0; row < coefficients.rows(); ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      Matrix plus = coefficients;
      Matrix minus = coefficients;
      plus(row, col) += epsilon;
      minus(row, col) -= epsilon;
      const double numerical =
          (linearHullObjective(plus,
                               durations,
                               basis,
                               derivative,
                               depth,
                               weights) -
           linearHullObjective(minus,
                               durations,
                               basis,
                               derivative,
                               depth,
                               weights)) /
          (2.0 * epsilon);
      max_coefficient_error = std::max(
          max_coefficient_error,
          std::abs(numerical -
                   analytic.coefficients(row, col)));
    }
  }

  double max_duration_error = 0.0;
  for (int segment = 0; segment < segments; ++segment)
  {
    Eigen::VectorXd plus = durations;
    Eigen::VectorXd minus = durations;
    plus(segment) += epsilon;
    minus(segment) -= epsilon;
    const double numerical =
        (linearHullObjective(coefficients,
                             plus,
                             basis,
                             derivative,
                             depth,
                             weights) -
         linearHullObjective(coefficients,
                             minus,
                             basis,
                             derivative,
                             depth,
                             weights)) /
        (2.0 * epsilon);
    max_duration_error = std::max(
        max_duration_error,
        std::abs(numerical - analytic.durations(segment)));
  }

  require(max_coefficient_error < 2.0e-7,
          "Control-to-coefficient adjoint failed finite differences.");
  require(max_duration_error < 8.0e-7,
          "Control-to-duration adjoint failed finite differences.");
}

void checkSubdivisionTightening()
{
  Matrix coefficients = Matrix::Zero(3, 3);
  // y(t) = 4t - 4t^2 has root Bezier ordinates [0, 2, 0]
  // and reaches only 1. Two half-interval hulls attain the exact upper bound.
  coefficients(1, 1) = 4.0;
  coefficients(2, 1) = -4.0;
  Eigen::VectorXd durations(1);
  durations << 1.0;

  Hull root;
  root.resetTopology(
      1, 3, traj_opt::convex_hull::Basis::Bezier, 0, 0);
  root.update(coefficients, durations);
  Hull split;
  split.resetTopology(
      1, 3, traj_opt::convex_hull::Basis::Bezier, 0, 1);
  split.update(coefficients, durations);

  require(std::abs(root.controls().col(1).maxCoeff() - 2.0) <
              1.0e-12,
          "Unexpected root Bezier hull.");
  require(std::abs(split.controls().col(1).maxCoeff() - 1.0) <
              1.0e-12,
          "Bezier subdivision did not tighten the hull.");
}

void checkCompleteLinearOperator()
{
  constexpr int segments = 2;
  constexpr int coeff_num = 7;
  constexpr int derivative = 2;
  constexpr int depth = 2;
  Eigen::VectorXd durations(segments);
  durations << 0.76, 1.18;

  std::mt19937 generator(151);
  std::normal_distribution<double> normal(0.0, 0.4);
  Matrix coefficients(segments * coeff_num, 3);
  for (Eigen::Index row = 0; row < coefficients.rows(); ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      coefficients(row, col) = normal(generator);
    }
  }

  Hull hull;
  hull.resetTopology(segments,
                     coeff_num,
                     traj_opt::convex_hull::Basis::MINVO,
                     derivative,
                     depth);
  hull.update(coefficients, durations);

  const int rows_per_segment =
      hull.piecesPerSegment() * hull.controlsPerPiece();
  for (int segment = 0; segment < segments; ++segment)
  {
    const Matrix converted =
        hull.sourceToControlOperator(segment) *
        coefficients.middleRows(segment * coeff_num, coeff_num);
    require(
        (converted -
         hull.controls().middleRows(
             segment * rows_per_segment,
             rows_per_segment))
                .norm() < 1.0e-12,
        "The complete source-to-control operator does not match update().");

    constexpr double epsilon = 1.0e-5;
    Eigen::VectorXd plus_times = durations;
    Eigen::VectorXd minus_times = durations;
    plus_times(segment) += epsilon;
    minus_times(segment) -= epsilon;
    Hull plus;
    Hull minus;
    plus.resetTopology(segments,
                       coeff_num,
                       traj_opt::convex_hull::Basis::MINVO,
                       derivative,
                       depth);
    minus.resetTopology(segments,
                        coeff_num,
                        traj_opt::convex_hull::Basis::MINVO,
                        derivative,
                        depth);
    plus.update(coefficients, plus_times);
    minus.update(coefficients, minus_times);
    const Eigen::MatrixXd numerical =
        (plus.sourceToControlOperator(segment) -
         minus.sourceToControlOperator(segment)) /
        (2.0 * epsilon);
    const double operator_derivative_error =
        (numerical - hull.durationDerivativeOperator(segment))
            .cwiseAbs()
            .maxCoeff();
    require(
        operator_derivative_error < 5.0e-8,
        "The duration derivative of the complete operator is incorrect.");
  }
}

double pieceTimeObjective(const Eigen::VectorXd &durations,
                          double start_time,
                          const Eigen::VectorXd &start_weights,
                          const Eigen::VectorXd &duration_weights)
{
  Matrix zero_coefficients =
      Matrix::Zero(durations.size() * 4, 3);
  Hull hull;
  hull.resetTopology(
      static_cast<int>(durations.size()),
      4,
      traj_opt::convex_hull::Basis::Bezier,
      1,
      3);
  hull.update(zero_coefficients, durations, start_time);

  double value = 0.0;
  for (int piece = 0; piece < hull.numPieces(); ++piece)
  {
    value += start_weights(piece) *
                 hull.pieceInfo(piece).start_time +
             duration_weights(piece) *
                 hull.pieceInfo(piece).duration;
  }
  return value;
}

void checkPieceTimeBackward()
{
  Eigen::VectorXd durations(3);
  durations << 0.61, 1.07, 0.83;
  constexpr double start_time = 0.42;

  Matrix zero_coefficients = Matrix::Zero(3 * 4, 3);
  Hull hull;
  hull.resetTopology(
      3, 4, traj_opt::convex_hull::Basis::Bezier, 1, 3);
  hull.update(zero_coefficients, durations, start_time);

  std::mt19937 generator(211);
  std::normal_distribution<double> normal(0.0, 0.5);
  Eigen::VectorXd start_weights(hull.numPieces());
  Eigen::VectorXd duration_weights(hull.numPieces());
  for (int piece = 0; piece < hull.numPieces(); ++piece)
  {
    start_weights(piece) = normal(generator);
    duration_weights(piece) = normal(generator);
  }

  Eigen::VectorXd analytic_durations =
      Eigen::VectorXd::Zero(durations.size());
  double analytic_start = 0.0;
  hull.backwardPieceTimesAdd(start_weights,
                             duration_weights,
                             analytic_durations,
                             analytic_start);

  constexpr double epsilon = 1.0e-6;
  for (Eigen::Index segment = 0;
       segment < durations.size();
       ++segment)
  {
    Eigen::VectorXd plus = durations;
    Eigen::VectorXd minus = durations;
    plus(segment) += epsilon;
    minus(segment) -= epsilon;
    const double numerical =
        (pieceTimeObjective(plus,
                            start_time,
                            start_weights,
                            duration_weights) -
         pieceTimeObjective(minus,
                            start_time,
                            start_weights,
                            duration_weights)) /
        (2.0 * epsilon);
    require(std::abs(numerical -
                     analytic_durations(segment)) < 2.0e-9,
            "Subdivided piece-time duration adjoint is incorrect.");
  }
  const double numerical_start =
      (pieceTimeObjective(durations,
                          start_time + epsilon,
                          start_weights,
                          duration_weights) -
       pieceTimeObjective(durations,
                          start_time - epsilon,
                          start_weights,
                          duration_weights)) /
      (2.0 * epsilon);
  require(std::abs(numerical_start - analytic_start) < 2.0e-9,
          "Subdivided piece start-time adjoint is incorrect.");
}

struct ExpTimeMap
{
  double toTime(double tau) const { return std::exp(tau); }
  double toTau(double time) const { return std::log(time); }
  double backward(double, double time, double grad_time) const
  {
    return time * grad_time;
  }
};

struct IdentitySpatialMap
{
  int getUnconstrainedDim(int) const { return 3; }

  Eigen::Vector3d toPhysical(const Eigen::VectorXd &x, int) const
  {
    return x.head<3>();
  }

  Eigen::VectorXd toUnconstrained(const Eigen::Vector3d &p, int) const
  {
    return p;
  }

  Eigen::VectorXd backwardGrad(const Eigen::VectorXd &,
                               const Eigen::Vector3d &gradient,
                               int) const
  {
    return gradient;
  }

  void addNormPenalty(const Eigen::VectorXd &,
                      double &,
                      Eigen::VectorXd &) const
  {
  }
};

struct ZeroTimeCost
{
  double operator()(const std::vector<double> &times,
                    Eigen::VectorXd &gradient) const
  {
    gradient.setZero(static_cast<Eigen::Index>(times.size()));
    return 0.0;
  }
};

template <typename Trajectory>
struct QuadraticVelocityHullCost
{
  using Coefficients = typename Trajectory::CoeffMat;
  mutable Hull velocity;
  mutable Matrix control_gradients;

  double evaluateIntegral(int,
                          double,
                          double,
                          int,
                          int,
                          const Eigen::Vector3d &,
                          const Eigen::Vector3d &,
                          const Eigen::Vector3d &,
                          const Eigen::Vector3d &,
                          Eigen::Vector3d &,
                          Eigen::Vector3d &,
                          Eigen::Vector3d &,
                          Eigen::Vector3d &,
                          double &) const
  {
    return 0.0;
  }

  template <typename SampleBuffer>
  double evaluateSample(
      const SampleBuffer &,
      Eigen::Matrix<double, 3, Eigen::Dynamic> &,
      Eigen::VectorXd &) const
  {
    return 0.0;
  }

  double evaluateCoefficient(
      const Trajectory &trajectory,
      Coefficients &grad_coefficients,
      Eigen::VectorXd &grad_durations) const
  {
    if (!velocity.kernel())
    {
      velocity.resetTopology(
          trajectory.getPieceNum(),
          Trajectory::COEFF_NUM,
          traj_opt::convex_hull::Basis::Bezier,
          1,
          2);
      control_gradients.resize(velocity.controls().rows(), 3);
    }
    trajectory.updateConvexHull(velocity);
    control_gradients = velocity.controls();
    velocity.backwardAdd(control_gradients,
                         grad_coefficients,
                         grad_durations);
    return 0.5 * velocity.controls().squaredNorm();
  }
};

void checkOptimizerGradient()
{
  using Optimizer =
      minco::MINCOOptimizer<3, 3, ExpTimeMap, IdentitySpatialMap>;
  using Trajectory = typename Optimizer::TrajType;

  Optimizer optimizer;
  optimizer.setEnergyWeight(0.0);
  optimizer.setSamplesPerPiece(1);

  std::vector<double> times{0.9, 1.1, 0.8};
  Optimizer::WaypointsType waypoints(4, 3);
  waypoints << 0.0, 0.0, 0.0,
      0.8, 0.4, -0.1,
      1.7, -0.2, 0.3,
      2.5, 0.1, 0.0;
  Optimizer::BoundaryState head =
      Optimizer::BoundaryState::Zero();
  Optimizer::BoundaryState tail =
      Optimizer::BoundaryState::Zero();
  head.col(0) = waypoints.row(0).transpose();
  tail.col(0) = waypoints.row(3).transpose();
  require(optimizer.setInitState(
              times, waypoints, head, tail),
          "Failed to initialize MINCO optimizer.");

  Eigen::VectorXd x = optimizer.generateInitialGuess();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
  ZeroTimeCost time_cost;
  QuadraticVelocityHullCost<Trajectory> cost;
  optimizer.evaluate(x, gradient, time_cost, cost);

  constexpr double epsilon = 1.0e-6;
  double max_error = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i)
  {
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += epsilon;
    minus(i) -= epsilon;
    Eigen::VectorXd scratch_plus =
        Eigen::VectorXd::Zero(x.size());
    Eigen::VectorXd scratch_minus =
        Eigen::VectorXd::Zero(x.size());
    const double value_plus =
        optimizer.evaluate(plus,
                           scratch_plus,
                           time_cost,
                           cost);
    const double value_minus =
        optimizer.evaluate(minus,
                           scratch_minus,
                           time_cost,
                           cost);
    const double numerical =
        (value_plus - value_minus) / (2.0 * epsilon);
    max_error = std::max(
        max_error,
        std::abs(numerical - gradient(i)) /
            std::max({1.0,
                      std::abs(numerical),
                      std::abs(gradient(i))}));
  }
  require(max_error < 2.0e-5,
          "Optimizer-level convex-hull gradient failed finite differences.");
  require(std::abs(optimizer.lastCoefficientCost()) > 1.0e-12,
          "Coefficient cost was not recorded by MINCOOptimizer.");
}

} // namespace

int main()
{
  try
  {
    for (int derivative = 0; derivative <= 3; ++derivative)
    {
      checkValues(
          traj_opt::convex_hull::Basis::Bezier, derivative, 2);
      checkValues(
          traj_opt::convex_hull::Basis::MINVO, derivative, 2);
    }
    checkBackward(
        traj_opt::convex_hull::Basis::Bezier, 0, 2);
    checkBackward(
        traj_opt::convex_hull::Basis::Bezier, 2, 2);
    checkBackward(
        traj_opt::convex_hull::Basis::MINVO, 1, 2);
    checkSubdivisionTightening();
    checkCompleteLinearOperator();
    checkPieceTimeBackward();
    checkOptimizerGradient();
    std::cout << "convex_hull_self_test passed" << std::endl;
  }
  catch (const std::exception &error)
  {
    std::cerr << "convex_hull_self_test failed: "
              << error.what() << std::endl;
    return 1;
  }
  return 0;
}
