#include "traj_opt/convex_hull/flatness_convex_hull_cost.hpp"
#include "traj_opt/costfunctional_manager/exp_convex_cost_manager.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"
#include "traj_opt/swarm_traj.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{

void require(bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
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

struct SyntheticTrajectory
{
  Eigen::VectorXd durations;
  Eigen::MatrixXd vel_controls;
  int leaves{1};

  Eigen::Vector3d getVel(double) const
  {
    return Eigen::Vector3d(2.0, 0.0, 0.0);
  }
  Eigen::Vector3d getAcc(double) const
  {
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  }
};

void checkSyntheticFlatnessGradient()
{
  using Cost = traj_opt::convex_hull::FlatnessConvexHullCost;
  Cost cost;
  Cost::Config config;
  config.mass = 1.64;
  config.gravity = 9.81;
  config.horizontal_drag = 0.35;
  config.vertical_drag = 0.35;
  config.parasitic_drag = 0.001;
  config.speed_smoothing = 1.0e-4;
  config.max_angular_rate = 1.5;
  config.max_tilt_angle = 0.55;
  config.min_thrust = 6.0;
  config.max_thrust = 30.0;
  config.smooth_epsilon = 1.0e-2;
  config.weights.angular_rate = 1.0e3;
  config.weights.thrust = 1.0e3;
  config.weights.tilt = 1.0e3;
  config.weights.force_projection = 1.0e3;
  config.weights.velocity_trust_region = 1.0e4;
  cost.configure(config);

  const int pieces = 2;
  const int vel_cp = 7;
  const int acc_cp = 6;
  const int jerk_cp = 5;
  Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(pieces * vel_cp, 3);
  Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(pieces * acc_cp, 3);
  Eigen::MatrixXd jerk = Eigen::MatrixXd::Zero(pieces * jerk_cp, 3);
  for (int i = 0; i < vel.rows(); ++i)
  {
    vel(i, 0) = 2.0 + 0.05 * static_cast<double>(i % vel_cp);
  }
  for (int i = 0; i < acc.rows(); ++i)
  {
    acc(i, 2) = 0.5;
  }
  for (int i = 0; i < jerk.rows(); ++i)
  {
    jerk(i, 1) = 0.2;
  }

  SyntheticTrajectory traj;
  traj.durations.resize(pieces);
  traj.durations.setConstant(0.5);
  traj.vel_controls = vel;
  cost.refreshReference(traj, pieces, /*leaves_per_segment=*/1, vel, vel_cp,
                        traj.durations);
  require(cost.referenceReady(), "Reference should be ready.");

  Eigen::MatrixXd g_vel = Eigen::MatrixXd::Zero(vel.rows(), 3);
  Eigen::MatrixXd g_acc = Eigen::MatrixXd::Zero(acc.rows(), 3);
  Eigen::MatrixXd g_jerk = Eigen::MatrixXd::Zero(jerk.rows(), 3);
  const double base = cost.accumulate(vel, acc, jerk, pieces, vel_cp, acc_cp,
                                      jerk_cp, g_vel, g_acc, g_jerk);
  require(std::isfinite(base), "Flatness cost must be finite.");
  require(g_vel.allFinite() && g_acc.allFinite() && g_jerk.allFinite(),
          "Flatness gradients must be finite.");

  const double eps = 1.0e-6;
  double max_rel = 0.0;
  for (int row = 0; row < vel.rows(); ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      Eigen::MatrixXd pert = vel;
      pert(row, col) += eps;
      Eigen::MatrixXd z_v = Eigen::MatrixXd::Zero(vel.rows(), 3);
      Eigen::MatrixXd z_a = Eigen::MatrixXd::Zero(acc.rows(), 3);
      Eigen::MatrixXd z_j = Eigen::MatrixXd::Zero(jerk.rows(), 3);
      const double plus =
          cost.accumulate(pert, acc, jerk, pieces, vel_cp, acc_cp, jerk_cp,
                          z_v, z_a, z_j);
      const double fd = (plus - base) / eps;
      const double analytic = g_vel(row, col);
      const double scale = std::max(1.0, std::abs(analytic));
      max_rel = std::max(max_rel, std::abs(fd - analytic) / scale);
    }
  }
  require(max_rel < 2.0e-3,
          "Velocity-control finite-difference gradient mismatch.");
  std::cout << "[flatness] synthetic FD max relative error = " << max_rel
            << "\n";
}

void checkManagerDisablesDenseOmgThr()
{
  using Optimizer =
      minco::MINCOOptimizer<3, 4, ExpTimeMap, IdentitySpatialMap>;
  Optimizer optimizer;
  optimizer.setEnergyWeight(0.0);
  optimizer.setSamplesPerPiece(2);
  std::vector<double> times{0.8, 0.8};
  Optimizer::WaypointsType waypoints(3, 3);
  waypoints << 0.0, 0.0, 1.0,
      1.5, 0.0, 1.0,
      3.0, 0.0, 1.0;
  Optimizer::BoundaryState head = Optimizer::BoundaryState::Zero();
  Optimizer::BoundaryState tail = Optimizer::BoundaryState::Zero();
  head.col(0) = waypoints.row(0).transpose();
  tail.col(0) = waypoints.row(2).transpose();
  require(optimizer.setInitState(times, waypoints, head, tail),
          "Failed to initialize optimizer.");

  general_utils::PolyhedraH corridors(1);
  corridors[0].resize(6, 4);
  corridors[0] << 1.0, 0.0, 0.0, -4.0, -1.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0,
      -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 1.0, -2.0, 0.0, 0.0, -1.0, 0.0;
  Eigen::VectorXi corridor_indices(2);
  corridor_indices.setZero();

  general_utils::VecDf bounds(6);
  bounds << 15.0, 20.0, 120.0, 3.5, 6.0 * 1.64, 22.0 * 1.64;
  general_utils::VecDf weights = general_utils::VecDf::Zero(7);
  weights << 1.0e4, 1.0e3, 1.0e3, 0.0, 0.0, 5.0e3, 2.0e3;
  flatness::FlatnessMap flatness_map;
  flatness_map.reset(1.64, 9.81, 0.35, 0.35, 0.001, 1.0e-4);
  traj_opt::SwarmPenaltyConfig swarm_config;
  traj_opt::SwarmTrajectoriesConstPtr swarm_trajectories;

  cost_functional_manager::ExpConvexCostManager manager;
  manager.configure(traj_opt::convex_hull::Basis::Bezier, 2, 2);
  traj_opt::convex_hull::FlatnessConvexHullCost::Config flat_cfg;
  flat_cfg.mass = 1.64;
  flat_cfg.gravity = 9.81;
  flat_cfg.horizontal_drag = 0.35;
  flat_cfg.vertical_drag = 0.35;
  flat_cfg.parasitic_drag = 0.001;
  flat_cfg.speed_smoothing = 1.0e-4;
  flat_cfg.max_angular_rate = 3.5;
  flat_cfg.max_tilt_angle = 1.05;
  flat_cfg.min_thrust = 6.0 * 1.64;
  flat_cfg.max_thrust = 22.0 * 1.64;
  flat_cfg.weights.angular_rate = weights(5);
  flat_cfg.weights.thrust = weights(6);
  flat_cfg.weights.tilt = weights(6);
  flat_cfg.weights.force_projection = weights(6);
  flat_cfg.weights.velocity_trust_region = 10.0 * weights(5);
  manager.configureFlatness(true, flat_cfg);
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
  require(manager.usesFlatnessHull(), "Flatness hull should be enabled.");
  require(!manager.usesDenseSampling(),
          "Flatness must zero residual omg/thr dense sampling.");

  Eigen::VectorXd x = optimizer.generateInitialGuess();
  require(optimizer.updateTrajectoryFromDecisionVector(x),
          "Failed to materialize trajectory.");
  manager.refreshFlatnessReference(optimizer.getTrajectory());

  typename Optimizer::TrajType::CoeffMat grad_c =
      Optimizer::TrajType::CoeffMat::Zero(
          optimizer.getTrajectory().getPieceNum() *
              Optimizer::TrajType::COEFF_NUM,
          3);
  Eigen::VectorXd grad_t =
      Eigen::VectorXd::Zero(optimizer.getTrajectory().getPieceNum());
  const double cost = manager.evaluateCoefficient(
      optimizer.getTrajectory(), grad_c, grad_t);
  require(std::isfinite(cost), "Coefficient cost must be finite.");
  require(grad_c.allFinite() && grad_t.allFinite(),
          "Coefficient gradients must be finite.");
  std::cout << "[flatness] manager coefficient cost = " << cost << "\n";
  std::cout << "[flatness] diagnostics max violation = "
            << manager.flatnessDiagnostics().maxViolation() << "\n";
}

} // namespace

int main()
{
  try
  {
    checkSyntheticFlatnessGradient();
    checkManagerDisablesDenseOmgThr();
    std::cout << "flatness_convex_hull_self_test passed\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "flatness_convex_hull_self_test failed: " << ex.what()
              << "\n";
    return 1;
  }
}
