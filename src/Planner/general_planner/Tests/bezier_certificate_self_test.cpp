#include "traj_opt/convex_hull/adaptive_bezier_forest.hpp"
#include "traj_opt/convex_hull/bezier_product.hpp"
#include "traj_opt/convex_hull/constraint_generator.hpp"
#include "traj_opt/convex_hull/continuous_certificate.hpp"
#include "traj_opt/convex_hull/scalar_bernstein.hpp"
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

void checkHalfSpaceBernsteinEnvelope()
{
  Eigen::MatrixXd controls(4, 3);
  controls << 0.0, 0.0, 0.0,
      1.0, 0.2, 0.0,
      2.0, -0.1, 0.0,
      3.0, 0.0, 0.0;
  const Eigen::Vector3d normal(0.0, 1.0, 0.0);
  const double offset = -0.05;
  const auto bound =
      traj_opt::convex_hull::halfSpaceBounds(controls, normal, offset);
  require(bound.lower <= bound.upper,
          "Half-space bound ordering failed.");

  for (int i = 0; i <= 40; ++i)
  {
    const double u = static_cast<double>(i) / 40.0;
    const Eigen::Vector3d value =
        traj_opt::convex_hull::evaluateBezier(controls, u);
    const double h = normal.dot(value) + offset;
    require(h + 1.0e-12 >= bound.lower && h - 1.0e-12 <= bound.upper,
            "Scalar Bernstein envelope does not cover the curve.");
  }
}

void checkDeCasteljauMonotoneTightening()
{
  Eigen::MatrixXd controls(4, 3);
  controls << -1.0, 0.0, 0.0,
      -0.2, 0.0, 0.0,
      0.2, 0.0, 0.0,
      1.0, 0.0, 0.0;
  const Eigen::Vector3d normal(1.0, 0.0, 0.0);
  const auto parent =
      traj_opt::convex_hull::halfSpaceBounds(controls, normal, 0.0);
  const auto children =
      traj_opt::convex_hull::deCasteljauSplit(controls, 0.5);
  const auto left =
      traj_opt::convex_hull::halfSpaceBounds(children.first, normal, 0.0);
  const auto right =
      traj_opt::convex_hull::halfSpaceBounds(children.second, normal, 0.0);
  require(left.upper <= parent.upper + 1.0e-12 &&
              right.upper <= parent.upper + 1.0e-12,
          "Child upper bound must not exceed parent.");
  require(left.lower >= parent.lower - 1.0e-12 &&
              right.lower >= parent.lower - 1.0e-12,
          "Child lower bound must not undercut parent.");
}

void checkSquaredNormBernsteinEnvelope()
{
  Eigen::MatrixXd controls(3, 3);
  controls << 1.0, 0.0, 0.0,
      0.0, 1.2, 0.0,
      -0.5, 0.0, 0.0;
  const double bound = 1.0;
  const auto coeffs =
      traj_opt::convex_hull::squaredNormBernstein(controls);
  require(coeffs.size() == 5, "Unexpected squared-norm Bernstein degree.");

  const auto residual_bounds =
      traj_opt::convex_hull::squaredNormBoundBounds(controls, bound);
  for (int i = 0; i <= 50; ++i)
  {
    const double u = static_cast<double>(i) / 50.0;
    const Eigen::Vector3d value =
        traj_opt::convex_hull::evaluateBezier(controls, u);
    const double residual = value.squaredNorm() - bound * bound;
    require(residual + 1.0e-10 >= residual_bounds.lower &&
                residual - 1.0e-10 <= residual_bounds.upper,
            "Squared-norm Bernstein envelope missed a sample.");
  }

  const auto hull =
      traj_opt::convex_hull::vectorHullNormBound(controls, bound);
  require(residual_bounds.upper <= hull.upper + 1.0e-12,
          "Squared-norm Bernstein should be at most as conservative as hull.");
}

void checkForestLeafWiseRefine()
{
  Eigen::MatrixXd controls(4, 3);
  controls << -1.0, 0.0, 0.0,
      -0.5, 0.0, 0.0,
      0.5, 0.0, 0.0,
      1.5, 0.0, 0.0;
  const Eigen::Vector3d normal(1.0, 0.0, 0.0);

  traj_opt::convex_hull::ForestOptions options;
  options.max_depth = 4;
  options.safe_margin = 0.0;
  options.enable_hysteresis = false;

  traj_opt::convex_hull::AdaptiveBezierForest forest;
  forest.configure(options);
  forest.seed(0, 0, 0, controls);

  std::vector<int> open{0};
  while (!open.empty())
  {
    const int id = open.back();
    open.pop_back();
    const auto status = forest.evaluateHalfSpace(id, normal, 0.0);
    if (status == traj_opt::convex_hull::LeafStatus::UNCERTAIN)
    {
      const int left = forest.split(id);
      require(left >= 0, "Expected de Casteljau split on uncertain leaf.");
      open.push_back(left);
      open.push_back(left + 1);
    }
  }

  int safe = 0;
  int violated = 0;
  int active = 0;
  for (const auto &leaf : forest.leaves())
  {
    if (!leaf.is_active)
    {
      continue;
    }
    ++active;
    if (leaf.status == traj_opt::convex_hull::LeafStatus::SAFE)
    {
      ++safe;
    }
    if (leaf.status == traj_opt::convex_hull::LeafStatus::VIOLATED)
    {
      ++violated;
    }
  }
  require(active >= 2, "Forest should refine into multiple active leaves.");
  require(safe >= 1 && violated >= 1,
          "Leaf-wise refine should isolate safe and violated children.");
  require(forest.maxDepthUsed() > 0, "Forest depth did not increase.");

  const auto candidates =
      traj_opt::convex_hull::extractViolatedCandidates(forest);
  require(!candidates.empty(),
          "Violated leaf did not produce constraint candidates.");
  require(candidates.front().source_segment == 0 &&
              candidates.front().derivative_order == 0,
          "Constraint candidate metadata incorrect.");
}

void checkEndToEndOracle()
{
  using Optimizer =
      minco::MINCOOptimizer<3, 4, ExpTimeMap, IdentitySpatialMap>;
  Optimizer optimizer;
  optimizer.setEnergyWeight(0.0);
  optimizer.setSamplesPerPiece(2);
  std::vector<double> times{1.0, 1.0};
  Optimizer::WaypointsType waypoints(3, 3);
  waypoints << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      2.0, 0.0, 0.0;
  Optimizer::BoundaryState head = Optimizer::BoundaryState::Zero();
  Optimizer::BoundaryState tail = Optimizer::BoundaryState::Zero();
  head.col(0) = waypoints.row(0).transpose();
  tail.col(0) = waypoints.row(2).transpose();
  require(optimizer.setInitState(times, waypoints, head, tail),
          "Failed to initialize oracle test optimizer.");

  general_utils::PolyhedraH corridors(1);
  corridors[0].resize(6, 4);
  corridors[0] <<
      1.0, 0.0, 0.0, -10.0,
      -1.0, 0.0, 0.0, -10.0,
      0.0, 1.0, 0.0, -10.0,
      0.0, -1.0, 0.0, -10.0,
      0.0, 0.0, 1.0, -10.0,
      0.0, 0.0, -1.0, -10.0;
  Eigen::VectorXi corridor_indices(2);
  corridor_indices.setZero();

  general_utils::VecDf bounds(6);
  bounds << 20.0, 40.0, 80.0, 20.0, 0.1, 100.0;
  general_utils::VecDf weights = general_utils::VecDf::Zero(7);
  weights(0) = 1.0;
  weights(1) = 1.0;
  flatness::FlatnessMap flatness_map;
  flatness_map.reset(1.0, 9.81, 0.0, 0.0, 0.0, 1.0e-4);
  traj_opt::SwarmPenaltyConfig swarm_config;
  traj_opt::SwarmTrajectoriesConstPtr swarm_trajectories;

  cost_functional_manager::ExpConvexCostManager manager;
  manager.configure(traj_opt::convex_hull::Basis::Bezier, 2, 2);
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
          "Failed to materialize trajectory for oracle.");
  const auto report =
      manager.computeAdaptiveContinuousCertificate(optimizer.getTrajectory());
  require(report.scalar_constraint_checks > 0,
          "Oracle produced no scalar checks.");
  require(report.continuous_feasible,
          "Wide corridor trajectory should be continuously feasible.");
  require(report.fully_resolved,
          "Wide corridor certificate should be fully resolved.");
  require(report.violated.empty(),
          "Feasible certificate should not emit violations.");

  corridors[0] <<
      1.0, 0.0, 0.0, -0.2,
      -1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, -0.2,
      0.0, -1.0, 0.0, -0.2,
      0.0, 0.0, 1.0, -0.2,
      0.0, 0.0, -1.0, -0.2;
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
  const auto violated_report =
      manager.computeAdaptiveContinuousCertificate(optimizer.getTrajectory());
  require(!violated_report.continuous_feasible,
          "Narrow corridor should fail continuous certificate.");
  require(!violated_report.violated.empty() ||
              violated_report.max_position_violation > 0.0,
          "Failed certificate should report violations or positive residual.");
}

} // namespace

int main()
{
  try
  {
    checkHalfSpaceBernsteinEnvelope();
    checkDeCasteljauMonotoneTightening();
    checkSquaredNormBernsteinEnvelope();
    checkForestLeafWiseRefine();
    checkEndToEndOracle();
    std::cout << "bezier_certificate_self_test passed" << std::endl;
  }
  catch (const std::exception &error)
  {
    std::cerr << "bezier_certificate_self_test failed: " << error.what()
              << std::endl;
    return 1;
  }
  return 0;
}
