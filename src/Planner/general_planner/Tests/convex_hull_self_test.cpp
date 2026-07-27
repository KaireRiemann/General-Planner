#include "traj_opt/convex_hull/convex_hull.hpp"
#include "traj_opt/costfunctional_manager/exp_convex_cost_manager.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"
#include "utils/optimization/phr_alm.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
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

void checkExpConvexCostManagerGradientAndTiming()
{
  using Optimizer =
      minco::MINCOOptimizer<3, 4, ExpTimeMap, IdentitySpatialMap>;

  Optimizer optimizer;
  optimizer.setEnergyWeight(0.0);
  optimizer.setSamplesPerPiece(10);
  optimizer.setTimingEnabled(true);

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
  require(optimizer.setInitState(times, waypoints, head, tail),
          "Failed to initialize ExpConvexCostManager test optimizer.");

  general_utils::PolyhedraH corridors(1);
  corridors[0].resize(6, 4);
  corridors[0] <<
      1.0, 0.0, 0.0, -0.5,
      -1.0, 0.0, 0.0, -10.0,
      0.0, 1.0, 0.0, -10.0,
      0.0, -1.0, 0.0, -10.0,
      0.0, 0.0, 1.0, -10.0,
      0.0, 0.0, -1.0, -10.0;
  Eigen::VectorXi corridor_indices(3);
  corridor_indices.setZero();

  general_utils::VecDf bounds(6);
  bounds << 0.7, 0.8, 1.2, 20.0, 0.1, 100.0;
  general_utils::VecDf weights(7);
  weights << 1.0, 0.2, 0.15, 0.1, 0.0, 0.0, 0.0;
  flatness::FlatnessMap flatness_map;
  flatness_map.reset(1.0, 9.81, 0.0, 0.0, 0.0, 1.0e-4);
  traj_opt::SwarmPenaltyConfig swarm_config;
  traj_opt::SwarmTrajectoriesConstPtr swarm_trajectories;

  cost_functional_manager::ExpIntegralCostManager dense_manager;
  dense_manager.reset(&corridors,
                      &corridor_indices,
                      nullptr,
                      nullptr,
                      1.0e-2,
                      bounds,
                      weights,
                      &flatness_map,
                      swarm_config,
                      swarm_trajectories,
                      0.0);

  Eigen::VectorXd x = optimizer.generateInitialGuess();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
  ZeroTimeCost time_cost;
  optimizer.resetTimingStatistics();
  for (int repetition = 0; repetition < 200; ++repetition)
  {
    optimizer.evaluate(x, gradient, time_cost, dense_manager);
  }
  const auto dense_timing = optimizer.cumulativeTimingStatistics();
  require(optimizer.lastIntegralCost() > 1.0e-12,
          "ExpIntegralCostManager dense cost was not active.");
  require(std::abs(optimizer.lastCoefficientCost()) < 1.0e-12,
          "Dense ExpIntegralCostManager unexpectedly produced coefficient cost.");

  cost_functional_manager::ExpConvexCostManager convex_manager;
  // Exercise the production depth-two path. Bezier omits the algebraically
  // duplicated first control of every leaf after the first one.
  convex_manager.configure(traj_opt::convex_hull::Basis::Bezier,
                           2);
  convex_manager.reset(&corridors,
                       &corridor_indices,
                       nullptr,
                       nullptr,
                       1.0e-2,
                       bounds,
                       weights,
                       &flatness_map,
                       swarm_config,
                       swarm_trajectories,
                       0.0);

  optimizer.resetTimingStatistics();
  double convex_cost = 0.0;
  for (int repetition = 0; repetition < 200; ++repetition)
  {
    convex_cost = optimizer.evaluate(x, gradient, time_cost, convex_manager);
  }
  require(convex_cost > 1.0e-12,
          "ExpConvexCostManager did not produce a convex-hull cost.");
  require(std::abs(optimizer.lastIntegralCost()) < 1.0e-12,
          "Polynomial dense costs remained active in ExpConvexCostManager.");
  require(optimizer.lastCoefficientCost() > 1.0e-12,
          "ExpConvexCostManager coefficient cost was not recorded.");
  require(!convex_manager.usesDenseSampling(),
          "Pure polynomial hull costs should bypass dense sampling.");
  // Production ExpTrajOpt uses seventh-degree MINCO_S4. The stable Bezier
  // functional keeps only the unique leaf-chain controls:
  // ((4*7+1)+(4*6+1)+(4*5+1)+(4*4+1))*3 = 276 checks.
  const std::size_t expected_checks = 276;
  require(convex_manager.activeControlPointChecksPerEvaluation() ==
              expected_checks,
          "Unexpected depth-two p/v/a/j control-point count.");
  const auto timing = optimizer.cumulativeTimingStatistics();
  require(timing.evaluations == 200 && timing.evaluation_seconds > 0.0 &&
              timing.dense_integral_seconds >= 0.0,
          "MINCO timing statistics were not recorded.");
  std::cout << "timing_microbenchmark stable_bezier"
            << " dense_integral_share="
            << dense_timing.denseIntegralShareOfEvaluation() * 100.0
            << "% convex_residual_integral_share="
            << timing.denseIntegralShareOfEvaluation() * 100.0
            << "% dense_eval_us="
            << dense_timing.evaluation_seconds * 1.0e6 /
                   static_cast<double>(dense_timing.evaluations)
            << " convex_eval_us="
            << timing.evaluation_seconds * 1.0e6 /
                   static_cast<double>(timing.evaluations)
            << std::endl;

  constexpr double epsilon = 1.0e-6;
  double max_error = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i)
  {
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += epsilon;
    minus(i) -= epsilon;
    Eigen::VectorXd scratch_plus = Eigen::VectorXd::Zero(x.size());
    Eigen::VectorXd scratch_minus = Eigen::VectorXd::Zero(x.size());
    const double value_plus =
        optimizer.evaluate(plus, scratch_plus, time_cost, convex_manager);
    const double value_minus =
        optimizer.evaluate(minus, scratch_minus, time_cost, convex_manager);
    const double numerical =
        (value_plus - value_minus) / (2.0 * epsilon);
    max_error = std::max(
        max_error,
        std::abs(numerical - gradient(i)) /
            std::max({1.0,
                      std::abs(numerical),
                      std::abs(gradient(i))}));
  }
  require(max_error < 8.0e-5,
          "ExpConvexCostManager gradient failed finite differences.");
}

#if 0
// Historical standalone full-ALM experiment. The production route now uses
// only certificate-triggered packed polish; keep this block excluded while
// old benchmark data are being archived.
void checkAdaptiveAlmGradientAndCertificate()
{
  using Optimizer =
      minco::MINCOOptimizer<3, 4, ExpTimeMap, IdentitySpatialMap>;

  Optimizer optimizer;
  optimizer.setEnergyWeight(0.0);
  optimizer.setSamplesPerPiece(2);
  std::vector<double> times{0.9, 1.1, 0.8};
  Optimizer::WaypointsType waypoints(4, 3);
  waypoints << 0.0, 0.0, 0.0,
      0.8, 0.1, 0.0,
      1.7, -0.1, 0.0,
      2.5, 0.0, 0.0;
  Optimizer::BoundaryState head = Optimizer::BoundaryState::Zero();
  Optimizer::BoundaryState tail = Optimizer::BoundaryState::Zero();
  head.col(0) = waypoints.row(0).transpose();
  tail.col(0) = waypoints.row(3).transpose();
  require(optimizer.setInitState(times, waypoints, head, tail),
          "Failed to initialize adaptive ALM optimizer.");

  auto make_box = [](double x_min, double x_max) {
    general_utils::MatDf box(6, 4);
    box << 1.0, 0.0, 0.0, -x_max,
        -1.0, 0.0, 0.0, x_min,
        0.0, 1.0, 0.0, -10.0,
        0.0, -1.0, 0.0, -10.0,
        0.0, 0.0, 1.0, -10.0,
        0.0, 0.0, -1.0, -10.0;
    return box;
  };
  general_utils::PolyhedraH corridors(3);
  corridors[0] = make_box(-10.0, 10.0);
  corridors[1] = make_box(-10.0, 1.2);
  corridors[2] = make_box(-10.0, 10.0);
  Eigen::VectorXi corridor_indices(3);
  corridor_indices << 0, 1, 2;

  general_utils::VecDf bounds(6);
  bounds << 20.0, 30.0, 100.0, 20.0, 0.1, 100.0;
  general_utils::VecDf weights = general_utils::VecDf::Zero(7);
  weights(0) = 1.0;
  flatness::FlatnessMap flatness_map;
  flatness_map.reset(1.0, 9.81, 0.0, 0.0, 0.0, 1.0e-4);
  traj_opt::SwarmPenaltyConfig swarm_config;
  traj_opt::SwarmTrajectoriesConstPtr swarm_trajectories;

  // Removed standalone full-trajectory ALM manager.
  RemovedStandaloneAlmManager manager;
  RemovedStandaloneAlmManager::Options options;
  options.adaptive = true;
  options.active_set = false;
  options.max_depth = 2;
  options.position_refine_margin = 0.05;
  options.derivative_refine_margin = 0.05;
  options.position_scale = 0.25;
  manager.configure(options);
  manager.reset(&corridors,
                &corridor_indices,
                nullptr,
                nullptr,
                1.0e-2,
                bounds,
                weights,
                &flatness_map,
                swarm_config,
                swarm_trajectories,
                0.0);

  Eigen::VectorXd x = optimizer.generateInitialGuess();
  require(optimizer.updateTrajectoryFromDecisionVector(x),
          "Failed to reconstruct adaptive ALM trajectory.");
  require(manager.initializeAlm(optimizer.getTrajectory()),
          "Failed to initialize adaptive ALM constraints.");
  manager.setPhrState(
      Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(manager.constraintCount())),
      100.0);
  require(manager.coarseSegmentCount() == 2 &&
              manager.fineSegmentCount() == 1,
          "Adaptive ALM did not keep free segments coarse and refine the constrained segment.");
  require(manager.activeControlPointChecksPerEvaluation() == 45,
          "Adaptive ALM selected an unexpected number of position controls.");
  require(manager.constraintCount() == 270,
          "Adaptive ALM selected an unexpected number of half-space constraints.");

  ZeroTimeCost time_cost;
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
  const double cost = optimizer.evaluate(x, gradient, time_cost, manager);
  require(cost > 1.0e-12 && gradient.allFinite(),
          "Adaptive ALM cost or gradient is inactive.");
  require(!manager.usesDenseSampling(),
          "Pure adaptive ALM constraints should bypass dense sampling.");

  constexpr double epsilon = 1.0e-6;
  double max_error = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i)
  {
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += epsilon;
    minus(i) -= epsilon;
    Eigen::VectorXd scratch_plus = Eigen::VectorXd::Zero(x.size());
    Eigen::VectorXd scratch_minus = Eigen::VectorXd::Zero(x.size());
    const double value_plus =
        optimizer.evaluate(plus, scratch_plus, time_cost, manager);
    const double value_minus =
        optimizer.evaluate(minus, scratch_minus, time_cost, manager);
    const double numerical =
        (value_plus - value_minus) / (2.0 * epsilon);
    max_error = std::max(
        max_error,
        std::abs(numerical - gradient(i)) /
            std::max({1.0, std::abs(numerical), std::abs(gradient(i))}));
  }
  require(max_error < 1.0e-4,
          "Adaptive ALM gradient failed finite differences.");

  require(optimizer.updateTrajectoryFromDecisionVector(x),
          "Failed to reconstruct adaptive ALM certificate trajectory.");
  const auto report = manager.updateAlmState(optimizer.getTrajectory());
  require(report.max_normalized_violation > 0.0 && !report.certified &&
              !report.topology_changed,
          "Adaptive ALM violation certificate is incorrect.");

  options.active_set = true;
  options.active_set_margin = 0.0;
  manager.configure(options);
  manager.reset(&corridors,
                &corridor_indices,
                nullptr,
                nullptr,
                1.0e-2,
                bounds,
                weights,
                &flatness_map,
                swarm_config,
                swarm_trajectories,
                0.0);
  require(manager.initializeAlm(optimizer.getTrajectory()) &&
              manager.constraintCount() > 0 &&
              manager.constraintCount() < manager.fullConstraintCount(),
          "Adaptive ALM active set did not reduce the full certificate layout.");

  corridors[1] = make_box(-10.0, 10.0);
  manager.reset(&corridors,
                &corridor_indices,
                nullptr,
                nullptr,
                1.0e-2,
                bounds,
                weights,
                &flatness_map,
                swarm_config,
                swarm_trajectories,
                0.0);
  require(manager.initializeAlm(optimizer.getTrajectory()),
          "Failed to initialize feasible adaptive ALM certificate.");
  const auto feasible_report =
      manager.updateAlmState(optimizer.getTrajectory());
  require(feasible_report.certified &&
              manager.coarseSegmentCount() == 3 &&
              manager.fineSegmentCount() == 0,
          "Adaptive ALM failed to certify an empty-corridor trajectory at depth zero.");
}
#endif

void checkGenericPhrAlmSolver()
{
  Eigen::VectorXd x(1);
  x(0) = 2.0;
  Eigen::VectorXd active_lambda;
  double active_penalty = 1.0;
  double minimum = 0.0;
  optimization::phr_alm::Parameters parameters;
  parameters.max_outer_iterations = 12;
  parameters.initial_penalty = 1.0;
  parameters.penalty_growth = 10.0;
  parameters.progress_ratio = 0.4;
  parameters.constraint_tolerance = 1.0e-7;
  optimization::phr_alm::Report report;
  const auto status = optimization::phr_alm::solve(
      x,
      minimum,
      parameters,
      [](const Eigen::VectorXd &decision, Eigen::VectorXd &constraints) {
        constraints = decision;
        return true;
      },
      [&active_lambda, &active_penalty](const Eigen::VectorXd &multipliers,
                                        double penalty) {
        active_lambda = multipliers;
        active_penalty = penalty;
      },
      [&active_lambda, &active_penalty](Eigen::VectorXd &decision,
                                        double &value) {
        // Exact minimizer of 0.5*(x-2)^2 plus the active PHR branch for x<=0.
        const double lambda = active_lambda(0);
        const double active_x =
            (2.0 - lambda) / (1.0 + active_penalty);
        decision(0) =
            lambda + active_penalty * active_x > 0.0 ? active_x : 2.0;
        const double shifted =
            std::max(0.0, lambda + active_penalty * decision(0));
        value = 0.5 * (decision(0) - 2.0) * (decision(0) - 2.0) +
                0.5 * (shifted * shifted - lambda * lambda) /
                    active_penalty;
        return 0;
      },
      [](const Eigen::VectorXd &decision,
         Eigen::VectorXd &constraints,
         optimization::phr_alm::TopologyUpdate &topology_update) {
        constraints = decision;
        topology_update = optimization::phr_alm::TopologyUpdate::UNCHANGED;
        return true;
      },
      report);
  require(status == optimization::phr_alm::Status::CONVERGED &&
              report.converged() && x(0) <= parameters.constraint_tolerance,
          "Generic PHR-ALM solver failed a scalar inequality problem.");

  bool replace_topology_once = true;
  int topology_inner_solves = 0;
  parameters.max_outer_iterations = 1;
  x(0) = -1.0;
  const auto topology_status = optimization::phr_alm::solve(
      x,
      minimum,
      parameters,
      [](const Eigen::VectorXd &decision, Eigen::VectorXd &constraints) {
        constraints = decision;
        return true;
      },
      [](const Eigen::VectorXd &, double) {},
      [&topology_inner_solves](Eigen::VectorXd &, double &value) {
        ++topology_inner_solves;
        value = 0.0;
        return 0;
      },
      [&replace_topology_once](const Eigen::VectorXd &decision,
                               Eigen::VectorXd &constraints,
                               optimization::phr_alm::TopologyUpdate &topology_update) {
        constraints = decision;
        topology_update =
            replace_topology_once
                ? optimization::phr_alm::TopologyUpdate::REPLACE
                : optimization::phr_alm::TopologyUpdate::UNCHANGED;
        replace_topology_once = false;
        return true;
      },
      report);
  require(topology_status == optimization::phr_alm::Status::CONVERGED &&
              report.outer_iterations == 1 && report.inner_solves == 2 &&
              report.topology_changes == 1 && topology_inner_solves == 2,
          "A topology replacement incorrectly consumed a PHR outer iteration.");

  parameters.accept_initial_feasible = true;
  int skipped_inner_solves = 0;
  x(0) = -1.0;
  const auto initially_feasible_status = optimization::phr_alm::solve(
      x,
      minimum,
      parameters,
      [](const Eigen::VectorXd &decision, Eigen::VectorXd &constraints) {
        constraints = decision;
        return true;
      },
      [](const Eigen::VectorXd &, double) {},
      [&skipped_inner_solves](Eigen::VectorXd &, double &) {
        ++skipped_inner_solves;
        return 0;
      },
      [](const Eigen::VectorXd &decision,
         Eigen::VectorXd &constraints,
         optimization::phr_alm::TopologyUpdate &topology_update) {
        constraints = decision;
        topology_update = optimization::phr_alm::TopologyUpdate::UNCHANGED;
        return true;
      },
      report);
  require(initially_feasible_status ==
              optimization::phr_alm::Status::CONVERGED &&
              skipped_inner_solves == 0 && report.inner_solves == 0 &&
              report.outer_iterations == 0,
          "PHR-ALM did not accept an explicitly allowed feasible warm start.");
}

#if 0
// Historical adaptive hot-loop experiment. Continuous refinement now belongs
// only to the post-solve certificate, never to the L-BFGS objective topology.
void checkTwoStageAdaptivePenaltyPath()
{
  using Optimizer =
      minco::MINCOOptimizer<3, 4, ExpTimeMap, IdentitySpatialMap>;

  Optimizer optimizer;
  optimizer.setEnergyWeight(0.0);
  optimizer.setSamplesPerPiece(8);
  optimizer.setTimingEnabled(false);

  std::vector<double> times{0.9, 1.1, 0.8};
  Optimizer::WaypointsType waypoints(4, 3);
  waypoints << 0.0, 0.0, 0.0,
      0.8, 0.4, -0.1,
      1.7, -0.2, 0.3,
      2.5, 0.1, 0.0;
  Optimizer::BoundaryState head = Optimizer::BoundaryState::Zero();
  Optimizer::BoundaryState tail = Optimizer::BoundaryState::Zero();
  head.col(0) = waypoints.row(0).transpose();
  tail.col(0) = waypoints.row(3).transpose();
  require(optimizer.setInitState(times, waypoints, head, tail),
          "Failed to initialize two-stage adaptive test optimizer.");

  // Narrow corridor so depth-0 certificates fail and Stage B must refine.
  general_utils::PolyhedraH corridors(1);
  corridors[0].resize(6, 4);
  corridors[0] <<
      1.0, 0.0, 0.0, -0.2,
      -1.0, 0.0, 0.0, -3.0,
      0.0, 1.0, 0.0, -0.15,
      0.0, -1.0, 0.0, -0.15,
      0.0, 0.0, 1.0, -0.2,
      0.0, 0.0, -1.0, -0.2;
  Eigen::VectorXi corridor_indices(3);
  corridor_indices.setZero();

  general_utils::VecDf bounds(6);
  bounds << 2.0, 4.0, 8.0, 20.0, 0.1, 100.0;
  general_utils::VecDf weights(7);
  weights << 1.0, 0.2, 0.15, 0.1, 0.0, 0.0, 0.0;
  flatness::FlatnessMap flatness_map;
  flatness_map.reset(1.0, 9.81, 0.0, 0.0, 0.0, 1.0e-4);
  traj_opt::SwarmPenaltyConfig swarm_config;
  traj_opt::SwarmTrajectoriesConstPtr swarm_trajectories;

  cost_functional_manager::ExpConvexCostManager::AdaptiveOptions options;
  options.enabled = true;
  options.position_refine_margin = 0.05;
  options.refine_derivative_constraints = false;

  cost_functional_manager::ExpConvexCostManager manager;
  manager.configure(traj_opt::convex_hull::Basis::Bezier, 2);
  manager.reset(&corridors,
                &corridor_indices,
                nullptr,
                nullptr,
                1.0e-2,
                bounds,
                weights,
                &flatness_map,
                swarm_config,
                swarm_trajectories,
                0.0);
  require(manager.adaptiveEnabled(),
          "Two-stage adaptive hull path was not enabled.");

  manager.freezeAllDepths(3, 0);
  require(manager.coarseSegmentCount() == 3 &&
              manager.fineSegmentCount() == 0,
          "Stage A must freeze every segment at depth 0.");

  Eigen::VectorXd x = optimizer.generateInitialGuess();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
  ZeroTimeCost time_cost;
  const double stage_a_cost =
      optimizer.evaluate(x, gradient, time_cost, manager);
  require(std::isfinite(stage_a_cost) && gradient.allFinite(),
          "Stage A adaptive evaluation produced non-finite values.");
  const std::size_t stage_a_checks =
      manager.activeControlPointChecksPerEvaluation();
  require(stage_a_checks > 0 && stage_a_checks < 276,
          "Stage A should evaluate fewer controls than fixed depth-2 V2.");

  require(optimizer.updateTrajectoryFromDecisionVector(x),
          "Failed to materialize trajectory before Stage B refine.");
  const bool refined =
      manager.refineFrozenDepths(optimizer.getTrajectory());
  require(refined && manager.fineSegmentCount() > 0,
          "Stage B refine did not promote any segment to depth 2.");
  require(manager.coarseSegmentCount() + manager.fineSegmentCount() == 3,
          "Frozen depth layout lost a source segment.");

  // Topology must stay fixed across evaluations of the same solve.
  const std::vector<int> frozen = manager.frozenSegmentDepths();
  const double stage_b_cost =
      optimizer.evaluate(x, gradient, time_cost, manager);
  require(std::isfinite(stage_b_cost) && gradient.allFinite(),
          "Stage B adaptive evaluation produced non-finite values.");
  require(manager.frozenSegmentDepths() == frozen,
          "Adaptive depth topology changed inside an evaluation/line-search.");
  require(manager.activeControlPointChecksPerEvaluation() > stage_a_checks,
          "Stage B should evaluate more controls after refining segments.");

  const auto certificate =
      manager.computeContinuousCertificate(
          optimizer.getTrajectory());
  require(certificate.scalar_constraint_checks > 0,
          "Post-solve adaptive certificate produced no scalar checks.");
  require(std::isfinite(certificate.max_normalized_violation),
          "Post-solve adaptive certificate max violation is non-finite.");
}
#endif

void checkDiscreteAttractorDoesNotForceDense()
{
  using Optimizer =
      minco::MINCOOptimizer<3, 4, ExpTimeMap, IdentitySpatialMap>;

  Optimizer optimizer;
  optimizer.setEnergyWeight(0.0);
  optimizer.setSamplesPerPiece(4);
  std::vector<double> times{0.8, 0.9, 1.0};
  Optimizer::WaypointsType waypoints(4, 3);
  waypoints << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      2.0, 0.0, 0.0,
      3.0, 0.0, 0.0;
  Optimizer::BoundaryState head = Optimizer::BoundaryState::Zero();
  Optimizer::BoundaryState tail = Optimizer::BoundaryState::Zero();
  head.col(0) = waypoints.row(0).transpose();
  tail.col(0) = waypoints.row(3).transpose();
  require(optimizer.setInitState(times, waypoints, head, tail),
          "Failed to initialize discrete-attractor optimizer.");

  general_utils::PolyhedraH corridors(1);
  corridors[0].resize(6, 4);
  corridors[0] <<
      1.0, 0.0, 0.0, -10.0,
      -1.0, 0.0, 0.0, -10.0,
      0.0, 1.0, 0.0, -10.0,
      0.0, -1.0, 0.0, -10.0,
      0.0, 0.0, 1.0, -10.0,
      0.0, 0.0, -1.0, -10.0;
  Eigen::VectorXi corridor_indices(3);
  corridor_indices.setZero();

  general_utils::Mat3Df attractors(3, 3);
  attractors << 1.0, 2.0, 2.5,
      0.2, -0.2, 0.1,
      0.0, 0.0, 0.0;
  general_utils::VecDf dead(3);
  dead << 0.05, 0.05, 0.05;

  general_utils::VecDf bounds(6);
  bounds << 20.0, 40.0, 80.0, 20.0, 0.1, 100.0;
  general_utils::VecDf weights = general_utils::VecDf::Zero(7);
  weights(4) = 10.0; // attractor only
  flatness::FlatnessMap flatness_map;
  flatness_map.reset(1.0, 9.81, 0.0, 0.0, 0.0, 1.0e-4);
  traj_opt::SwarmPenaltyConfig swarm_config;
  traj_opt::SwarmTrajectoriesConstPtr swarm_trajectories;

  cost_functional_manager::ExpConvexCostManager manager;
  manager.configure(traj_opt::convex_hull::Basis::Bezier, 0, 2);
  manager.reset(&corridors,
                &corridor_indices,
                &attractors,
                &dead,
                1.0e-2,
                bounds,
                weights,
                &flatness_map,
                swarm_config,
                swarm_trajectories,
                0.0);
  require(!manager.usesDenseSampling(),
          "Attractor-only hull cost must not force dense sampling.");

  Eigen::VectorXd x = optimizer.generateInitialGuess();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
  ZeroTimeCost time_cost;
  const double cost =
      optimizer.evaluate(x, gradient, time_cost, manager);
  require(cost > 1.0e-12 && gradient.allFinite(),
          "Discrete attractor cost/gradient inactive.");
  require(std::abs(optimizer.lastIntegralCost()) < 1.0e-12,
          "Discrete attractor unexpectedly used dense integral.");
  require(optimizer.lastCoefficientCost() > 1.0e-12,
          "Discrete attractor was not recorded as coefficient cost.");

  constexpr double epsilon = 1.0e-6;
  double max_error = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i)
  {
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += epsilon;
    minus(i) -= epsilon;
    Eigen::VectorXd scratch_plus = Eigen::VectorXd::Zero(x.size());
    Eigen::VectorXd scratch_minus = Eigen::VectorXd::Zero(x.size());
    const double value_plus =
        optimizer.evaluate(plus, scratch_plus, time_cost, manager);
    const double value_minus =
        optimizer.evaluate(minus, scratch_minus, time_cost, manager);
    const double numerical =
        (value_plus - value_minus) / (2.0 * epsilon);
    max_error = std::max(
        max_error,
        std::abs(numerical - gradient(i)) /
            std::max({1.0,
                      std::abs(numerical),
                      std::abs(gradient(i))}));
  }
  require(max_error < 8.0e-5,
          "Discrete attractor gradient failed finite differences.");
}

void checkPositionScaleNondimensionalization()
{
  using Optimizer =
      minco::MINCOOptimizer<3, 4, ExpTimeMap, IdentitySpatialMap>;

  Optimizer optimizer;
  optimizer.setEnergyWeight(0.0);
  optimizer.setSamplesPerPiece(2);
  std::vector<double> times{1.0};
  Optimizer::WaypointsType waypoints(2, 3);
  waypoints << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0;
  Optimizer::BoundaryState head = Optimizer::BoundaryState::Zero();
  Optimizer::BoundaryState tail = Optimizer::BoundaryState::Zero();
  head.col(0) = waypoints.row(0).transpose();
  tail.col(0) = waypoints.row(1).transpose();
  require(optimizer.setInitState(times, waypoints, head, tail),
          "Failed to initialize position-scale optimizer.");

  general_utils::PolyhedraH corridors(1);
  corridors[0].resize(6, 4);
  // Force a positive position violation on +x.
  corridors[0] <<
      1.0, 0.0, 0.0, -0.2,
      -1.0, 0.0, 0.0, 1.0,
      0.0, 1.0, 0.0, -1.0,
      0.0, -1.0, 0.0, -1.0,
      0.0, 0.0, 1.0, -1.0,
      0.0, 0.0, -1.0, -1.0;
  Eigen::VectorXi corridor_indices(1);
  corridor_indices.setZero();
  general_utils::VecDf bounds(6);
  bounds << 20.0, 40.0, 80.0, 20.0, 0.1, 100.0;
  general_utils::VecDf weights = general_utils::VecDf::Zero(7);
  weights(0) = 1.0;
  flatness::FlatnessMap flatness_map;
  flatness_map.reset(1.0, 9.81, 0.0, 0.0, 0.0, 1.0e-4);
  traj_opt::SwarmPenaltyConfig swarm_config;
  traj_opt::SwarmTrajectoriesConstPtr swarm_trajectories;

  cost_functional_manager::ExpConvexCostManager manager_a;
  cost_functional_manager::ExpConvexCostManager manager_b;
  manager_a.configure(
      traj_opt::convex_hull::Basis::Bezier, 0, 0.25, 0.05);
  manager_b.configure(
      traj_opt::convex_hull::Basis::Bezier, 0, 0.50, 0.05);
  manager_a.reset(&corridors,
                  &corridor_indices,
                  nullptr,
                  nullptr,
                  1.0e-2,
                  bounds,
                  weights,
                  &flatness_map,
                  swarm_config,
                  swarm_trajectories,
                  0.0);
  manager_b.reset(&corridors,
                  &corridor_indices,
                  nullptr,
                  nullptr,
                  1.0e-2,
                  bounds,
                  weights,
                  &flatness_map,
                  swarm_config,
                  swarm_trajectories,
                  0.0);

  Eigen::VectorXd x = optimizer.generateInitialGuess();
  Eigen::VectorXd ga = Eigen::VectorXd::Zero(x.size());
  Eigen::VectorXd gb = Eigen::VectorXd::Zero(x.size());
  ZeroTimeCost time_cost;
  const double cost_a = optimizer.evaluate(x, ga, time_cost, manager_a);
  const double cost_b = optimizer.evaluate(x, gb, time_cost, manager_b);
  require(cost_a > cost_b && cost_b > 0.0,
          "Larger position_scale must reduce nondimensional position penalty.");
  require(manager_a.lastScalarConstraintChecks() > 0 &&
              manager_b.lastScalarConstraintChecks() > 0,
          "Position-scale path must count scalar constraint checks.");
}

void checkPhrKktResidualReporting()
{
  optimization::phr_alm::Parameters parameters;
  parameters.max_outer_iterations = 3;
  parameters.initial_penalty = 1.0;
  parameters.penalty_growth = 2.0;
  parameters.progress_ratio = 0.5;
  parameters.constraint_tolerance = 1.0e-8;

  Eigen::VectorXd x(1);
  x(0) = 2.0;
  double minimum = 0.0;
  optimization::phr_alm::Report report;
  const auto status = optimization::phr_alm::solve(
      x,
      minimum,
      parameters,
      [](Eigen::VectorXd &decision, Eigen::VectorXd &constraints) {
        constraints.resize(1);
        constraints(0) = decision(0); // g(x)=x <= 0
        return true;
      },
      [](const Eigen::VectorXd &, double) {},
      [](Eigen::VectorXd &decision, double &objective) {
        // One projected gradient step onto the PHR surrogate.
        const double g = decision(0);
        objective = 0.5 * decision(0) * decision(0) +
                    0.5 * std::max(0.0, g) * std::max(0.0, g);
        decision(0) = std::min(0.0, decision(0) - 0.5);
        return 0;
      },
      [](const Eigen::VectorXd &decision,
         Eigen::VectorXd &constraints,
         optimization::phr_alm::TopologyUpdate &topology) {
        topology = optimization::phr_alm::TopologyUpdate::UNCHANGED;
        constraints.resize(1);
        constraints(0) = decision(0);
        return true;
      },
      report);
  require(status == optimization::phr_alm::Status::CONVERGED ||
              status == optimization::phr_alm::Status::MAX_OUTER_ITERATIONS,
          "PHR solve failed unexpectedly in KKT residual test.");
  require(std::isfinite(report.primal_residual) &&
              std::isfinite(report.dual_residual) &&
              std::isfinite(report.complementarity_residual),
          "PHR report did not populate KKT residual fields.");
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
    checkExpConvexCostManagerGradientAndTiming();
    checkDiscreteAttractorDoesNotForceDense();
    checkPositionScaleNondimensionalization();
    checkPhrKktResidualReporting();
    checkGenericPhrAlmSolver();
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
