/**
 * Level A: full space-time control residual metric.
 *
 * A1 coefficient JVP vs finite difference
 * A2 control-residual JVP vs finite difference
 * A3 G = G^T
 * A4 SPD after G_T^{rel} + εI
 * A5 G_PP^{full} ≈ G_PP^{MCE}  (Gate 1)
 */

#include "traj_opt/minco/minco_metric.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

namespace
{

using Trajectory = minco::MINCO_S4<3>;
using InnerPoints = Trajectory::InnerPointsMat;
using BoundaryState = Trajectory::BoundaryState;
using CoeffMat = Trajectory::CoeffMat;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr std::array<double, 4> kNodes{
    0.06943184420297371, 0.33000947820757187, 0.66999052179242813,
    0.93056815579702623};
constexpr std::array<double, 4> kWeights{
    0.17392742256872693, 0.32607257743127307, 0.32607257743127307,
    0.17392742256872693};

void require(bool ok, const std::string &message)
{
  if (!ok)
  {
    throw std::runtime_error(message);
  }
}

Trajectory makeTrajectory(const InnerPoints &points, const VectorXd &times)
{
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  head.col(0) << 0.0, -0.2, 0.1;
  head.col(1) << 0.45, 0.1, 0.0;
  tail.col(0) << 4.2, 1.1, 0.8;
  tail.col(1) << 0.1, -0.15, 0.0;
  Trajectory traj;
  require(traj.generate(points, head, tail, times), "MINCO generate failed");
  return traj;
}

VectorXd flatten(const InnerPoints &points)
{
  return Eigen::Map<const VectorXd>(points.data(), points.size());
}

InnerPoints unflatten(const VectorXd &v, int cols)
{
  InnerPoints points(3, cols);
  Eigen::Map<VectorXd>(points.data(), v.size()) = v;
  return points;
}

MatrixXd controlResidual(const Trajectory &traj)
{
  const int pieces = traj.getPieceNum();
  const int rows = pieces * static_cast<int>(kNodes.size());
  MatrixXd r(rows, 3);
  r.setZero();
  const auto &C = traj.getCoefficients();
  const auto &T = traj.getDurations();
  for (int i = 0; i < pieces; ++i)
  {
    const double sqrt_T = std::sqrt(T(i));
    const auto coeff = C.block<Trajectory::COEFF_NUM, 3>(i * Trajectory::COEFF_NUM, 0);
    for (int k = 0; k < static_cast<int>(kNodes.size()); ++k)
    {
      const double u = kNodes[static_cast<std::size_t>(k)];
      const auto b4 = Trajectory::derivativeBasis(4, u * T(i));
      const Vector3d snap = coeff.transpose() * b4.transpose();
      r.row(i * static_cast<int>(kNodes.size()) + k) =
          (std::sqrt(kWeights[static_cast<std::size_t>(k)]) * sqrt_T * snap)
              .transpose();
    }
  }
  return r;
}

void testA1CoefficientJvp()
{
  VectorXd times(4);
  times << 0.85, 1.10, 0.72, 1.25;
  InnerPoints points(3, 3);
  points << 0.8, 1.9, 3.1, -0.1, 0.6, 0.9, 0.3, 0.5, 0.7;
  const Trajectory traj = makeTrajectory(points, times);

  std::mt19937 gen(17);
  std::normal_distribution<double> n(0.0, 1.0);
  InnerPoints dP(3, 3);
  VectorXd dT(4);
  for (int i = 0; i < dP.size(); ++i)
  {
    dP(i) = n(gen);
  }
  for (int i = 0; i < dT.size(); ++i)
  {
    dT(i) = 0.12 * n(gen);
  }

  CoeffMat dC;
  require(traj.propagateTangent(dP, dT, dC), "JVP failed");

  constexpr double eps = 2.0e-7;
  const Trajectory perturbed =
      makeTrajectory(points + eps * dP, times + eps * dT);
  const CoeffMat fd =
      (perturbed.getCoefficients() - traj.getCoefficients()) / eps;
  const double err =
      (fd - dC).norm() / std::max(1.0, fd.norm());
  require(err < 1.0e-5, "A1 coefficient JVP vs FD failed, e=" + std::to_string(err));
  std::cout << "  A1 coefficient JVP  e=" << err << "\n";
}

void testA2ControlResidualJvp()
{
  VectorXd times(4);
  times << 0.85, 1.10, 0.72, 1.25;
  InnerPoints points(3, 3);
  points << 0.8, 1.9, 3.1, -0.1, 0.6, 0.9, 0.3, 0.5, 0.7;
  const Trajectory traj = makeTrajectory(points, times);
  const MatrixXd r0 = controlResidual(traj);

  InnerPoints dP = InnerPoints::Zero(3, 3);
  dP(0, 1) = 0.35;
  dP(1, 0) = -0.22;
  VectorXd dT(4);
  dT << 0.08, -0.05, 0.04, -0.03;

  constexpr double eps = 1.0e-6;
  const Trajectory plus = makeTrajectory(points + eps * dP, times + eps * dT);
  const Trajectory minus = makeTrajectory(points - eps * dP, times - eps * dT);
  const MatrixXd Jr_fd = (controlResidual(plus) - controlResidual(minus)) / (2.0 * eps);

  CoeffMat dC;
  require(traj.propagateTangent(dP, dT, dC), "residual JVP tangent failed");
  MatrixXd Jr_an = MatrixXd::Zero(r0.rows(), 3);
  const auto &C = traj.getCoefficients();
  const auto &T = traj.getDurations();
  for (int i = 0; i < traj.getPieceNum(); ++i)
  {
    const double sqrt_T = std::sqrt(T(i));
    const auto coeff = C.block<Trajectory::COEFF_NUM, 3>(i * Trajectory::COEFF_NUM, 0);
    const auto dcoeff = dC.block<Trajectory::COEFF_NUM, 3>(i * Trajectory::COEFF_NUM, 0);
    for (int k = 0; k < static_cast<int>(kNodes.size()); ++k)
    {
      const double u = kNodes[static_cast<std::size_t>(k)];
      const double t = u * T(i);
      const auto b4 = Trajectory::derivativeBasis(4, t);
      const auto b5 = Trajectory::derivativeBasis(5, t);
      const Vector3d p4 = coeff.transpose() * b4.transpose();
      const Vector3d p5 = coeff.transpose() * b5.transpose();
      const Vector3d dp4 = dcoeff.transpose() * b4.transpose();
      const Vector3d dr =
          sqrt_T * dp4 + dT(i) * (0.5 / sqrt_T * p4 + sqrt_T * u * p5);
      Jr_an.row(i * static_cast<int>(kNodes.size()) + k) =
          (std::sqrt(kWeights[static_cast<std::size_t>(k)]) * dr).transpose();
    }
  }
  const double err = (Jr_an - Jr_fd).norm() / std::max(1.0, Jr_fd.norm());
  require(err < 1.0e-5, "A2 residual JVP vs FD failed, e=" + std::to_string(err));
  std::cout << "  A2 residual JVP     e=" << err << "\n";
}

void testA3A4A5Metric()
{
  VectorXd times(4);
  times << 0.70, 1.35, 0.55, 1.80;
  InnerPoints points(3, 3);
  points << 0.9, 2.1, 3.4, 0.05, -0.12, 0.18, 0.35, 0.5, 0.62;
  const Trajectory traj = makeTrajectory(points, times);

  minco::MincoMetricOptions opt;
  opt.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
  opt.regularization = 1.0e-10;
  opt.time_metric_weight = 1.0;
  opt.energy_weight = 1.0;
  minco::MincoMetric<3, 4> full;
  full.setOptions(opt);
  require(full.update(traj), "full space-time metric failed");
  const MatrixXd &G = full.spaceTimeMetric();
  require((G - G.transpose()).norm() < 1.0e-10, "A3 G is not symmetric");

  Eigen::SelfAdjointEigenSolver<MatrixXd> es(0.5 * (G + G.transpose()));
  require(es.info() == Eigen::Success, "A4 eigendecomposition failed");
  const double lmin = es.eigenvalues().minCoeff();
  require(lmin > 0.0, "A4 metric is not SPD");
  std::cout << "  A3 ||G-G^T||=" << (G - G.transpose()).norm()
            << "  A4 λmin=" << lmin
            << "  κ(G)=" << full.conditionNumber()
            << "  η=" << full.couplingEta() << "\n";

  opt.time_metric_weight = 0.0;
  opt.regularization = 0.0;
  minco::MincoMetric<3, 4> full_pp;
  full_pp.setOptions(opt);
  require(full_pp.update(traj), "full metric without time/rel failed");

  minco::MincoMetricOptions wp_opt;
  wp_opt.mode = minco::MincoMetricMode::kFrozenWaypoint;
  wp_opt.regularization = 0.0;
  wp_opt.energy_weight = 1.0;
  minco::MincoMetric<3, 4> wp;
  wp.setOptions(wp_opt);
  require(wp.update(traj), "waypoint MCE failed");

  const MatrixXd Gpp = full_pp.Gpp();
  const MatrixXd Gmce = wp.waypointMetric();
  const double e_pp = (Gpp - Gmce).norm() / std::max(1.0, Gmce.norm());
  require(e_pp < 1.0e-6, "A5 G_PP^{full} vs G_PP^{MCE} failed, e=" + std::to_string(e_pp));
  std::cout << "  A5 e_PP=" << e_pp
            << "  dim PP=" << Gpp.rows()
            << "  ||G_TP||=" << full.Gtp().norm() << "\n";
  require(full.Gtp().norm() > 1.0e-6, "full metric has no T-P coupling");
}

} // namespace

int main()
{
  try
  {
    std::cout << "[Level A] joint residual / full space-time metric\n";
    testA1CoefficientJvp();
    testA2ControlResidualJvp();
    testA3A4A5Metric();
    std::cout << "[minco_joint_metric_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[minco_joint_metric_self_test] FAIL: " << ex.what() << "\n";
    return 1;
  }
}
