#include "traj_opt/costfunctional/spatialmap/polytope_spatial_map.hpp"
#include "traj_opt/minco/minco_metric.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

namespace
{

using Trajectory = minco::MINCO_S4<3>;
using InnerPoints = Trajectory::InnerPointsMat;
using BoundaryState = Trajectory::BoundaryState;
using Coefficients = Trajectory::CoeffMat;

void require(const bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

Trajectory makeTrajectory(const InnerPoints &points,
                          const Eigen::VectorXd &times)
{
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  head.col(0) << 0.0, -0.2, 0.1;
  head.col(1) << 0.45, 0.1, 0.0;
  tail.col(0) << 4.2, 1.1, 0.8;
  tail.col(1) << 0.1, -0.15, 0.0;
  Trajectory trajectory;
  require(trajectory.generate(points, head, tail, times),
          "MINCO trajectory generation failed");
  return trajectory;
}

void testJvpVjpAndFiniteDifference()
{
  Eigen::VectorXd times(4);
  times << 0.85, 1.10, 0.72, 1.25;
  InnerPoints points(3, 3);
  points << 0.8, 1.9, 3.1,
            -0.1, 0.6, 0.9,
            0.3, 0.5, 0.7;
  const Trajectory trajectory = makeTrajectory(points, times);

  std::mt19937 generator(17);
  std::normal_distribution<double> normal(0.0, 1.0);
  InnerPoints delta_points(3, 3);
  for (Eigen::Index i = 0; i < delta_points.size(); ++i)
  {
    delta_points(i) = normal(generator);
  }
  Eigen::VectorXd delta_times(4);
  for (Eigen::Index i = 0; i < delta_times.size(); ++i)
  {
    delta_times(i) = 0.12 * normal(generator);
  }

  Coefficients delta_coefficients;
  require(trajectory.propagateTangent(delta_points,
                                      delta_times,
                                      delta_coefficients),
          "MINCO JVP failed");

  Coefficients coefficient_covector = Coefficients::Zero(
      Trajectory::COEFF_NUM * trajectory.getPieceNum(), 3);
  for (Eigen::Index i = 0; i < coefficient_covector.size(); ++i)
  {
    coefficient_covector(i) = normal(generator);
  }
  const auto vjp = trajectory.propagateGradFull(
      coefficient_covector,
      Eigen::VectorXd::Zero(trajectory.getPieceNum()));
  const double lhs = (delta_coefficients.array() *
                      coefficient_covector.array()).sum();
  const double rhs = (delta_points.array() * vjp.grad_by_points.array()).sum() +
                     delta_times.dot(vjp.grad_by_times);
  require(std::abs(lhs - rhs) <=
              2.0e-9 * std::max({1.0, std::abs(lhs), std::abs(rhs)}),
          "MINCO JVP/VJP adjoint identity failed");

  constexpr double epsilon = 2.0e-7;
  const Trajectory perturbed = makeTrajectory(
      points + epsilon * delta_points,
      times + epsilon * delta_times);
  const Coefficients finite_difference =
      (perturbed.getCoefficients() - trajectory.getCoefficients()) / epsilon;
  const double relative_error =
      (finite_difference - delta_coefficients).norm() /
      std::max(1.0, finite_difference.norm());
  require(relative_error < 3.0e-5,
          "MINCO JVP does not match coefficient finite difference");
}

void testFixedTimeMetricAndLinearCoordinateInvariance()
{
  Eigen::VectorXd times(3);
  times << 0.9, 1.1, 0.8;
  InnerPoints points(3, 2);
  points << 0.9, 2.6,
            0.1, 0.7,
            0.4, 0.75;
  const Trajectory trajectory = makeTrajectory(points, times);

  minco::MincoMetricOptions options;
  options.mode = minco::MincoMetricMode::kFrozenWaypoint;
  options.regularization = 1.0e-12;
  minco::MincoMetric<3, 4> metric;
  metric.setOptions(options);
  require(metric.update(trajectory), "fixed-time waypoint metric failed");
  const Eigen::MatrixXd &G = metric.waypointMetric();
  require(G.rows() == 6 && (G - G.transpose()).norm() < 1.0e-10,
          "waypoint metric is not symmetric");

  Eigen::VectorXd direction(6);
  direction << 0.2, -0.4, 0.1, 0.35, 0.2, -0.15;
  Eigen::VectorXd G_direction;
  require(metric.applyWaypointMetric(direction, G_direction),
          "metric apply failed");
  require(direction.dot(G_direction) > 0.0,
          "waypoint metric is not positive definite");

  const double h = 1.0e-4;
  InnerPoints delta_points = InnerPoints::Zero(3, 2);
  for (int i = 0; i < direction.size(); ++i)
  {
    delta_points(i % 3, i / 3) = direction(i);
  }
  const double energy_0 = trajectory.getEnergy();
  const double energy_plus = makeTrajectory(points + h * delta_points, times).getEnergy();
  const double energy_minus = makeTrajectory(points - h * delta_points, times).getEnergy();
  const double finite_hessian = (energy_plus - 2.0 * energy_0 + energy_minus) / (h * h);
  const double expected_hessian = 2.0 * direction.dot(G_direction);
  require(std::abs(finite_hessian - expected_hessian) <=
              3.0e-5 * std::max({1.0, std::abs(finite_hessian),
                                  std::abs(expected_hessian)}),
          "G_P is not one half of the fixed-time energy Hessian");

  Eigen::MatrixXd transform = Eigen::MatrixXd::Identity(6, 6);
  transform(0, 3) = 0.2;
  transform(2, 4) = -0.15;
  transform(5, 1) = 0.1;
  const Eigen::VectorXd gradient =
      Eigen::VectorXd::LinSpaced(6, -0.7, 0.8);
  const Eigen::VectorXd natural_x = G.ldlt().solve(gradient);
  const Eigen::MatrixXd G_y = transform.transpose() * G * transform;
  const Eigen::VectorXd gradient_y = transform.transpose() * gradient;
  const Eigen::VectorXd natural_y = G_y.ldlt().solve(gradient_y);
  require((transform * natural_y - natural_x).norm() < 1.0e-9,
          "natural direction is not invariant under linear coordinate change");
}

void testFullSpaceTimeMetricAndPolytopeLift()
{
  Eigen::VectorXd times(3);
  times << 0.85, 1.0, 1.2;
  InnerPoints points(3, 2);
  points << 0.8, 2.3,
            -0.05, 0.65,
            0.25, 0.7;
  const Trajectory trajectory = makeTrajectory(points, times);

  minco::MincoMetricOptions options;
  options.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
  options.regularization = 1.0e-10;
  options.time_metric_weight = 0.3;
  minco::MincoMetric<3, 4> metric;
  metric.setOptions(options);
  require(metric.update(trajectory), "full space-time GN metric failed");
  require(metric.isSpaceTimeMetric() && metric.spaceTimeDim() == 9,
          "unexpected full metric dimension");
  Eigen::VectorXd q = Eigen::VectorXd::LinSpaced(9, -0.4, 0.6);
  Eigen::VectorXd d;
  require(metric.solve(q, d) && q.dot(d) > 0.0,
          "space-time metric solve failed");

  spatial_map::PolyhedronV poly(3, 4);
  poly << 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0;
  spatial_map::PolytopeSpatialMap map;
  map.reset(&poly, 1, false);
  Eigen::VectorXd xi(4);
  xi << 0.35, 0.42, 0.31, 0.78;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd pinv;
  Eigen::MatrixXd gauge;
  require(map.physicalJacobian(xi, 1, jacobian) &&
              map.pseudoInverse(xi, 1, pinv, &gauge),
          "polytope metric lift Jacobian failed");
  require((jacobian * pinv - Eigen::Matrix3d::Identity()).norm() < 1.0e-8,
          "polytope pseudoinverse is not a physical right inverse");
  require((gauge * gauge - gauge).norm() < 1.0e-8 &&
              (gauge - gauge.transpose()).norm() < 1.0e-8,
          "polytope gauge projection is invalid");
}

} // namespace

int main()
{
  try
  {
    testJvpVjpAndFiniteDifference();
    testFixedTimeMetricAndLinearCoordinateInvariance();
    testFullSpaceTimeMetricAndPolytopeLift();
    std::cout << "[minco_metric_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &exception)
  {
    std::cerr << "[minco_metric_self_test] FAIL: " << exception.what() << '\n';
    return 1;
  }
}
