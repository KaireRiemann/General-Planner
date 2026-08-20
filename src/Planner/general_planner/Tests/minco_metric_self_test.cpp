#include "traj_opt/costfunctional/spatialmap/polytope_spatial_map.hpp"
#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/minco/minco_metric.hpp"
#include "traj_opt/minco/minco_whitening.hpp"
#include "utils/optimization/lbfgs.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using Trajectory = minco::MINCO_S4<3>;
using InnerPoints = Trajectory::InnerPointsMat;
using BoundaryState = Trajectory::BoundaryState;
using Coefficients = Trajectory::CoeffMat;

void require(const bool condition, const std::string &message)
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
  const double expected_hessian = direction.dot(G_direction);
  require(std::abs(finite_hessian - expected_hessian) <=
              3.0e-5 * std::max({1.0, std::abs(finite_hessian),
                                  std::abs(expected_hessian)}),
          "G_MCE is not the fixed-time getEnergy() Hessian");

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

std::string sci(double v, int d = 4)
{
  std::ostringstream oss;
  oss << std::scientific << std::setprecision(d) << v;
  return oss.str();
}

double conditionNumber(const Eigen::MatrixXd &G)
{
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (G + G.transpose()));
  require(es.info() == Eigen::Success, "eigendecomposition failed");
  const double lmin = es.eigenvalues().minCoeff();
  const double lmax = es.eigenvalues().maxCoeff();
  require(lmin > 0.0, "metric is not SPD");
  return lmax / lmin;
}

Eigen::MatrixXd whiteHessian(const Eigen::MatrixXd &G, const Eigen::MatrixXd &H)
{
  Eigen::LLT<Eigen::MatrixXd> llt(G);
  require(llt.info() == Eigen::Success, "Cholesky failed");
  const Eigen::MatrixXd L = llt.matrixL();
  const Eigen::MatrixXd Y = L.triangularView<Eigen::Lower>().solve(H);
  const Eigen::MatrixXd W =
      L.triangularView<Eigen::Lower>().solve(Y.transpose()).transpose();
  return 0.5 * (W + W.transpose());
}

Eigen::VectorXd flattenPoints(const InnerPoints &points)
{
  return Eigen::Map<const Eigen::VectorXd>(points.data(), points.size());
}

InnerPoints unflattenPoints(const Eigen::VectorXd &v, int cols)
{
  InnerPoints points(3, cols);
  Eigen::Map<Eigen::VectorXd>(points.data(), v.size()) = v;
  return points;
}

minco::MincoMetric<3, 4> makeMetric(minco::MincoMetricMode mode,
                                    const Trajectory &trajectory,
                                    double regularization = 1.0e-12,
                                    double time_weight = 1.0)
{
  minco::MincoMetricOptions options;
  options.mode = mode;
  options.regularization = regularization;
  options.time_metric_weight = time_weight;
  minco::MincoMetric<3, 4> metric;
  metric.setOptions(options);
  require(metric.update(trajectory), "metric update failed");
  return metric;
}

void testWaypointMetricIndependentOfPAndMatchesHalfEnergyHessian()
{
  Eigen::VectorXd times(5);
  times << 0.7, 1.0, 1.3, 0.9, 1.1;
  InnerPoints a(3, 4);
  a << 0.8, 1.7, 2.6, 3.4,
      0.12, 0.05, -0.08, 0.2,
      0.4, 0.5, 0.45, 0.55;
  InnerPoints b = a;
  b.row(1) *= -1.3;
  b.row(2).array() += 0.2;
  const Trajectory traj_a = makeTrajectory(a, times);
  const Trajectory traj_b = makeTrajectory(b, times);
  auto metric_a =
      makeMetric(minco::MincoMetricMode::kFrozenWaypoint, traj_a, 0.0);
  auto metric_b =
      makeMetric(minco::MincoMetricMode::kFrozenWaypoint, traj_b, 0.0);
  const double relative =
      (metric_a.waypointMetric() - metric_b.waypointMetric()).norm() /
      std::max(1.0, metric_a.waypointMetric().norm());
  require(relative < 1.0e-12,
          "fixed-time G_P must be independent of inner waypoints");

  const Eigen::MatrixXd &G = metric_a.waypointMetric();
  const Eigen::VectorXd P = flattenPoints(a);
  const int n = static_cast<int>(P.size());
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
  const double eta = 1.0e-6;
  for (int j = 0; j < n; ++j)
  {
    const double h = eta * std::max(1.0, std::abs(P(j)));
    Eigen::VectorXd ej = Eigen::VectorXd::Zero(n);
    ej(j) = 1.0;
    const Trajectory plus = makeTrajectory(unflattenPoints(P + h * ej, 4), times);
    const Trajectory minus =
        makeTrajectory(unflattenPoints(P - h * ej, 4), times);
    Trajectory::CoeffMat gdC_plus, gdC_minus;
    double e_plus = 0.0;
    double e_minus = 0.0;
    plus.getEnergyPartialGradByCoeffs(e_plus, gdC_plus);
    minus.getEnergyPartialGradByCoeffs(e_minus, gdC_minus);
    InnerPoints gP_plus, gP_minus;
    Eigen::VectorXd gT_plus, gT_minus;
    plus.propagateGrad(gdC_plus, Eigen::VectorXd::Zero(5), gP_plus, gT_plus);
    minus.propagateGrad(gdC_minus, Eigen::VectorXd::Zero(5), gP_minus, gT_minus);
    H.col(j) = (flattenPoints(gP_plus) - flattenPoints(gP_minus)) / (2.0 * h);
  }
  H = 0.5 * (H + H.transpose());
  const double e_H = (H - G).norm() / std::max(1.0, H.norm());
  require(e_H < 1.0e-6, "production G_MCE is not getEnergy() Hessian");
  const double kappa_E = conditionNumber(H);
  const double kappa_G = conditionNumber(whiteHessian(G, H));
  require(std::abs(kappa_G - 1.0) < 1.0e-6,
          "MCE whitening of the energy Hessian is not identity-scaled");
  std::cout << "  frozen G_P  n=" << n
            << "  e_H=" << sci(e_H)
            << "  kappa_E=" << sci(kappa_E)
            << "  kappa(G^{-1/2} H G^{-1/2})=" << sci(kappa_G)
            << "\n";
}

void testPieceAndTimeRatioConditioning()
{
  std::cout << "  M  pattern     kappa(G_P)\n";
  const int Ms[4] = {3, 5, 10, 20};
  for (int M : Ms)
  {
    Eigen::VectorXd times = Eigen::VectorXd::Ones(M);
    InnerPoints points(3, M - 1);
    for (int i = 0; i < M - 1; ++i)
    {
      const double u = static_cast<double>(i + 1) / static_cast<double>(M);
      points.col(i) << u * static_cast<double>(M),
          0.1 * std::sin(2.0 * M_PI * u),
          0.4 + 0.05 * std::cos(2.0 * M_PI * u);
    }
    const Trajectory trajectory = makeTrajectory(points, times);
    auto metric =
        makeMetric(minco::MincoMetricMode::kFrozenWaypoint, trajectory, 0.0);
    const double kappa = conditionNumber(metric.waypointMetric());
    std::cout << "  " << std::setw(2) << M << "  uniform    " << sci(kappa)
              << "\n";
    require(std::isfinite(kappa) && kappa > 1.0,
            "uniform-time G_P condition number invalid");
  }

  const std::vector<std::pair<const char *, std::vector<double>>> ratios = {
      {"uniform", {1, 1, 1, 1, 1}},
      {"mild", {0.7, 1.0, 1.3, 0.9, 1.1}},
      {"strong", {0.3, 0.6, 1.2, 1.8, 1.1}},
      {"extreme", {0.2, 0.4, 1.5, 2.0, 0.9}},
  };
  InnerPoints points(3, 4);
  for (int i = 0; i < 4; ++i)
  {
    const double u = static_cast<double>(i + 1) / 5.0;
    points.col(i) << u * 5.0, 0.1 * std::sin(2.0 * M_PI * u), 0.5;
  }
  for (const auto &ratio : ratios)
  {
    Eigen::VectorXd times(5);
    for (int i = 0; i < 5; ++i)
    {
      times(i) = ratio.second[static_cast<std::size_t>(i)];
    }
    const Trajectory trajectory = makeTrajectory(points, times);
    auto metric =
        makeMetric(minco::MincoMetricMode::kFrozenWaypoint, trajectory, 0.0);
    const double kappa = conditionNumber(metric.waypointMetric());
    std::cout << "   5  " << std::setw(8) << ratio.first << "  " << sci(kappa)
              << "\n";
    require(std::isfinite(kappa), "time-ratio G_P condition number invalid");
  }
}

void testBlockSpaceTimeIsDiagonalTimePlusWaypoint()
{
  Eigen::VectorXd times(3);
  times << 0.8, 1.2, 0.9;
  InnerPoints points(3, 2);
  points << 1.0, 2.4, 0.1, 0.4, 0.3, 0.6;
  const Trajectory trajectory = makeTrajectory(points, times);
  auto waypoint =
      makeMetric(minco::MincoMetricMode::kFrozenWaypoint, trajectory, 0.0);
  auto block = makeMetric(minco::MincoMetricMode::kBlockSpaceTime, trajectory,
                          0.0, 0.75);
  require(block.isSpaceTimeMetric() && block.spaceTimeDim() == 9,
          "block space-time dimension is wrong");
  const Eigen::MatrixXd &G = block.spaceTimeMetric();
  require(G.bottomRightCorner(6, 6).isApprox(waypoint.waypointMetric(), 1.0e-12),
          "block metric waypoint block is not G_P");
  require(G.topRightCorner(3, 6).norm() < 1.0e-14,
          "block metric has unexpected T-P coupling");
  for (int i = 0; i < 3; ++i)
  {
    const double expected = 0.75 / (times(i) * times(i));
    require(std::abs(G(i, i) - expected) < 1.0e-12,
            "block time metric is not w / T_i^2");
  }
}

void testQuadInvTimeMapPullback()
{
  temporal_map::QuadInvTimeMap map;
  const double T = 1.7;
  const double tau = map.toTau(T);
  require(std::abs(map.toTime(tau) - T) < 1.0e-12,
          "QuadInvTimeMap is not invertible");
  const double gT = -0.42;
  const double gTau = map.backward(tau, T, gT);
  const double dT_dTau = map.backward(tau, T, 1.0);
  require(std::abs(gTau - dT_dTau * gT) < 1.0e-12,
          "time covector pullback is not g_tau = (dT/dtau) g_T");
  const double w = 1.0;
  const double G_T = w / (T * T);
  const double physical_q = gTau / dT_dTau;
  const double d_T = physical_q / G_T;
  const double r_tau = d_T / dT_dTau;
  require(std::abs(r_tau - gTau / (dT_dTau * dT_dTau * G_T)) < 1.0e-12,
          "time H0 pullback formula is inconsistent");
}

Eigen::VectorXd applyPlannerH0(const Trajectory &trajectory,
                               const Eigen::VectorXd &x,
                               const Eigen::VectorXd &q,
                               bool space_time,
                               double gauge_weight)
{
  auto metric =
      makeMetric(space_time ? minco::MincoMetricMode::kBlockSpaceTime
                            : minco::MincoMetricMode::kFrozenWaypoint,
                 trajectory, 1.0e-12, 1.0);
  const int pieces = trajectory.getPieceNum();
  const int waypoint_dim = 3 * (pieces - 1);
  const int time_dim = pieces;
  require(q.size() == time_dim + waypoint_dim,
          "H0 test uses identity spatial map");

  spatial_map::PolytopeSpatialMap map;
  map.reset(nullptr, nullptr, pieces - 1, true);

  Eigen::VectorXd physical_q_points = Eigen::VectorXd::Zero(waypoint_dim);
  std::vector<Eigen::MatrixXd> pinvs;
  std::vector<Eigen::MatrixXd> gauges;
  int offset = time_dim;
  for (int i = 1; i < pieces; ++i)
  {
    Eigen::MatrixXd pinv;
    Eigen::MatrixXd gauge;
    require(map.pseudoInverse(x.segment(offset, 3), i, pinv, &gauge),
            "identity spatial pseudoinverse failed");
    physical_q_points.segment(3 * (i - 1), 3) =
        pinv.transpose() * q.segment(offset, 3);
    pinvs.push_back(pinv);
    gauges.push_back(gauge);
    offset += 3;
  }

  Eigen::VectorXd r = Eigen::VectorXd::Zero(q.size());
  Eigen::VectorXd physical_direction_points;
  if (space_time)
  {
    temporal_map::QuadInvTimeMap time_map;
    Eigen::VectorXd physical_q(pieces + waypoint_dim);
    const auto &times = trajectory.getDurations();
    for (int i = 0; i < pieces; ++i)
    {
      const double dT_dTau = time_map.backward(x(i), times(i), 1.0);
      physical_q(i) = q(i) / dT_dTau;
    }
    physical_q.tail(waypoint_dim) = physical_q_points;
    Eigen::VectorXd physical_direction;
    require(metric.solve(physical_q, physical_direction),
            "space-time solve failed");
    for (int i = 0; i < pieces; ++i)
    {
      const double dT_dTau = time_map.backward(x(i), times(i), 1.0);
      r(i) = physical_direction(i) / dT_dTau;
    }
    physical_direction_points = physical_direction.tail(waypoint_dim);
  }
  else
  {
    require(metric.solve(physical_q_points, physical_direction_points),
            "waypoint solve failed");
    r.head(time_dim) = q.head(time_dim);
  }

  offset = time_dim;
  for (int i = 1; i < pieces; ++i)
  {
    r.segment(offset, 3) =
        pinvs[static_cast<std::size_t>(i - 1)] *
            physical_direction_points.segment(3 * (i - 1), 3) +
        gauge_weight * gauges[static_cast<std::size_t>(i - 1)] *
            q.segment(offset, 3);
    offset += 3;
  }
  return r;
}

void testIdentitySpatialH0IsMetricInverse()
{
  Eigen::VectorXd times(3);
  times << 0.9, 1.1, 0.8;
  InnerPoints points(3, 2);
  points << 0.9, 2.6, 0.1, 0.7, 0.4, 0.75;
  const Trajectory trajectory = makeTrajectory(points, times);
  temporal_map::QuadInvTimeMap time_map;
  Eigen::VectorXd x(9);
  for (int i = 0; i < 3; ++i)
  {
    x(i) = time_map.toTau(times(i));
  }
  x.tail(6) = flattenPoints(points);
  Eigen::VectorXd q = Eigen::VectorXd::LinSpaced(9, -0.5, 0.7);
  const Eigen::VectorXd r = applyPlannerH0(trajectory, x, q, false, 1.0e-6);
  auto metric =
      makeMetric(minco::MincoMetricMode::kFrozenWaypoint, trajectory, 1.0e-12);
  Eigen::VectorXd expected_points;
  require(metric.solve(q.tail(6), expected_points),
          "direct waypoint solve failed");
  require((r.head(3) - q.head(3)).norm() < 1.0e-14,
          "frozen waypoint H0 must leave the time block Euclidean");
  require((r.tail(6) - expected_points).norm() < 1.0e-10,
          "identity spatial H0 is not G_P^{-1} on the waypoint block");
  require(q.dot(r) > 0.0, "H0 operator is not positive definite");
}

void testPolytopeLiftMatchesPinvFormula()
{
  spatial_map::PolyhedronV poly(3, 4);
  poly << 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0;
  spatial_map::PolytopeSpatialMap map;
  map.reset(&poly, 1, false);
  Eigen::VectorXd xi(4);
  xi << 0.35, 0.42, 0.31, 0.78;
  Eigen::MatrixXd J;
  Eigen::MatrixXd pinv;
  Eigen::MatrixXd gauge;
  require(map.physicalJacobian(xi, 1, J) &&
              map.pseudoInverse(xi, 1, pinv, &gauge),
          "polytope Jacobian failed");
  require((J * pinv - Eigen::Matrix3d::Identity()).norm() < 1.0e-9,
          "J J^dagger != I");
  require((pinv * J * pinv - pinv).norm() < 1.0e-9,
          "J^dagger is not a reflexive inverse");

  Eigen::VectorXd g_xi = Eigen::VectorXd::LinSpaced(4, -0.2, 0.5);
  Eigen::MatrixXd G_P = Eigen::Matrix3d::Identity();
  G_P(0, 0) = 4.0;
  G_P(1, 1) = 0.5;
  G_P(2, 2) = 2.0;
  const double lambda = 1.0e-6;
  const Eigen::VectorXd d_xi =
      pinv * G_P.ldlt().solve(pinv.transpose() * g_xi) + lambda * gauge * g_xi;
  const Eigen::VectorXd expected =
      pinv * G_P.ldlt().solve(pinv.transpose() * g_xi) + lambda * gauge * g_xi;
  require((d_xi - expected).norm() < 1.0e-12,
          "polytope H0 lift does not match J^dagger G^{-1} J^{dagger T}");
}

struct LbfgsH0Problem
{
  Eigen::VectorXd times;
  Eigen::MatrixXd G;
  int iters{0};
};

double lbfgsEnergy(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
  auto *problem = static_cast<LbfgsH0Problem *>(ptr);
  const int cols = static_cast<int>(problem->times.size()) - 1;
  const Trajectory traj = makeTrajectory(unflattenPoints(x, cols), problem->times);
  Trajectory::CoeffMat gdC;
  double energy = 0.0;
  traj.getEnergyPartialGradByCoeffs(energy, gdC);
  InnerPoints gP;
  Eigen::VectorXd gT;
  traj.propagateGrad(gdC, Eigen::VectorXd::Zero(traj.getPieceNum()), gP, gT);
  g = flattenPoints(gP);
  return energy;
}

bool lbfgsH0(void *ptr, const Eigen::VectorXd &, const Eigen::VectorXd &q,
             Eigen::VectorXd &r)
{
  auto *problem = static_cast<LbfgsH0Problem *>(ptr);
  r = problem->G.ldlt().solve(q);
  return r.allFinite();
}

int lbfgsCount(void *ptr, const Eigen::VectorXd &, const Eigen::VectorXd &,
               const double, const double, const int, const int)
{
  ++static_cast<LbfgsH0Problem *>(ptr)->iters;
  return 0;
}

void testFrozenH0GivesNaturalDescentOnPureEnergy()
{
  Eigen::VectorXd times(5);
  times.setConstant(1.0);
  InnerPoints points(3, 4);
  for (int i = 0; i < 4; ++i)
  {
    const double u = static_cast<double>(i + 1) / 5.0;
    points.col(i) << u * 4.2, 0.18 * std::sin(2.0 * M_PI * u),
        0.4 + 0.04 * std::cos(2.0 * M_PI * u);
  }
  const Trajectory trajectory = makeTrajectory(points, times);
  auto metric =
      makeMetric(minco::MincoMetricMode::kFrozenWaypoint, trajectory, 1.0e-12);
  LbfgsH0Problem problem;
  problem.times = times;
  problem.G = metric.waypointMetric();

  Eigen::VectorXd x = flattenPoints(points);
  Eigen::VectorXd g;
  const double j0 = lbfgsEnergy(&problem, x, g);
  Eigen::VectorXd natural;
  require(lbfgsH0(&problem, x, Eigen::VectorXd(-g), natural),
          "H0 solve failed");
  require(g.dot(natural) < 0.0, "H0 direction is not a descent direction");
  const Eigen::VectorXd expected = -problem.G.ldlt().solve(g);
  require((natural - expected).norm() < 1.0e-10,
          "H0(q=-g) is not the natural gradient -G^{-1}g");

  const Trajectory after =
      makeTrajectory(unflattenPoints(x + 0.5 * natural, 4), times);
  const double j1 = after.getEnergy();
  require(j1 < j0,
          "half natural step did not reduce snap energy");
  std::cout << "  J0=" << sci(j0, 6)
            << "  J_half_newton=" << sci(j1, 6)
            << "  drop=" << sci(j0 - j1, 3)
            << "  cos=" << sci(-g.dot(natural) / (g.norm() * natural.norm()), 3)
            << "\n";
}

void testFrozenWhiteningIsExactNewtonChart()
{
  Eigen::VectorXd times(5);
  times.setConstant(1.0);
  InnerPoints points(3, 4);
  for (int i = 0; i < 4; ++i)
  {
    const double u = static_cast<double>(i + 1) / 5.0;
    points.col(i) << u * 4.2, 0.18 * std::sin(2.0 * M_PI * u),
        0.4 + 0.04 * std::cos(2.0 * M_PI * u);
  }
  const Trajectory trajectory = makeTrajectory(points, times);
  auto metric =
      makeMetric(minco::MincoMetricMode::kFrozenWaypoint, trajectory, 0.0);
  require(metric.hasKroneckerStructure(),
          "fixed-time G_MCE must be G_scalar ⊗ I_3");

  const Eigen::VectorXd y0 = flattenPoints(points);
  minco::FrozenMceWhitening whitening;
  require(whitening.configureKronecker(0, 3, y0, metric.scalarWaypointMetric()),
          "Kronecker whitening configure failed");

  Eigen::VectorXd z;
  require(whitening.toWhitened(y0, z) && z.norm() < 1.0e-14,
          "seed must encode to z=0");
  Eigen::VectorXd y_back;
  require(whitening.toChart(z, y_back) && (y_back - y0).norm() < 1.0e-12,
          "whitening decode does not recover the seed");

  Trajectory::CoeffMat gdC;
  double energy = 0.0;
  trajectory.getEnergyPartialGradByCoeffs(energy, gdC);
  InnerPoints gP;
  Eigen::VectorXd gT;
  trajectory.propagateGrad(gdC, Eigen::VectorXd::Zero(5), gP, gT);
  const Eigen::VectorXd g = flattenPoints(gP);

  Eigen::VectorXd g_z;
  require(whitening.transformCovector(g, g_z), "g_z = L^{-1} g_P failed");
  Eigen::VectorXd d_p;
  require(whitening.transformDirectionToChart(-g_z, d_p),
          "Newton direction lift failed");
  const Eigen::VectorXd newton = -metric.waypointMetric().ldlt().solve(g);
  require((d_p - newton).norm() < 1.0e-8 * std::max(1.0, newton.norm()),
          "whitened Euclidean step is not the MCE Newton step");

  const Trajectory after =
      makeTrajectory(unflattenPoints(y0 + d_p, 4), times);
  double energy_after = 0.0;
  Trajectory::CoeffMat gdC_after;
  after.getEnergyPartialGradByCoeffs(energy_after, gdC_after);
  InnerPoints gP_after;
  Eigen::VectorXd gT_after;
  after.propagateGrad(gdC_after, Eigen::VectorXd::Zero(5), gP_after, gT_after);
  const Eigen::VectorXd g_after = flattenPoints(gP_after);
  require(energy_after < energy,
          "one frozen-whitening Newton step did not reduce snap energy");
  require(g_after.norm() < 1.0e-6 * std::max(1.0, g.norm()),
          "one frozen-whitening Newton step did not reach the quadratic minimizer");
  std::cout << "  J0=" << sci(energy, 6)
            << "  J_newton=" << sci(energy_after, 6)
            << "  ||g||=" << sci(g.norm(), 3)
            << "  ||g_after||=" << sci(g_after.norm(), 3)
            << "\n";
}

} // namespace

int main()
{
  try
  {
    std::cout << std::setprecision(6);
    std::cout << "Production MincoMetric / pullback / H0 validation\n";

    std::cout << "\n== JVP / VJP / finite difference ==\n";
    testJvpVjpAndFiniteDifference();
    std::cout << "  OK\n";

    std::cout << "\n== Fixed-time G_P, Hessian match, linear invariance ==\n";
    testFixedTimeMetricAndLinearCoordinateInvariance();
    testWaypointMetricIndependentOfPAndMatchesHalfEnergyHessian();
    std::cout << "  OK\n";

    std::cout << "\n== Conditioning vs piece count and time ratio ==\n";
    testPieceAndTimeRatioConditioning();
    std::cout << "  OK\n";

    std::cout << "\n== Block space-time metric ==\n";
    testBlockSpaceTimeIsDiagonalTimePlusWaypoint();
    std::cout << "  OK\n";

    std::cout << "\n== TimeMap and polytope pullback ==\n";
    testQuadInvTimeMapPullback();
    testIdentitySpatialH0IsMetricInverse();
    testPolytopeLiftMatchesPinvFormula();
    testFullSpaceTimeMetricAndPolytopeLift();
    std::cout << "  OK\n";

    std::cout << "\n== Frozen H0 natural direction on pure energy ==\n";
    testFrozenH0GivesNaturalDescentOnPureEnergy();
    std::cout << "  OK\n";

    std::cout << "\n== Frozen MCE whitening is the exact Newton chart ==\n";
    testFrozenWhiteningIsExactNewtonChart();
    std::cout << "  OK\n";

    std::cout << "\n[minco_metric_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &exception)
  {
    std::cerr << "[minco_metric_self_test] FAIL: " << exception.what() << '\n';
    return 1;
  }
}
