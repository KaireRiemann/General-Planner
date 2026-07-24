#include "traj_opt/convex_hull/adaptive_bezier_forest.hpp"
#include "traj_opt/convex_hull/batched_residuals.hpp"
#include "traj_opt/convex_hull/bezier_product.hpp"
#include "traj_opt/convex_hull/constraint_pack.hpp"
#include "traj_opt/convex_hull/continuous_certificate.hpp"
#include "traj_opt/convex_hull/scalar_bernstein.hpp"
#include "traj_opt/costfunctional_manager/exp_convex_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/exp_packed_corrector_cost_manager.hpp"
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

void checkLeafSelectionMatrix()
{
  Eigen::MatrixXd controls(3, 3);
  controls << 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0,
      2.0, 0.0, 0.0;
  const auto children =
      traj_opt::convex_hull::deCasteljauSplit(controls, 0.5);
  const Eigen::MatrixXd S =
      traj_opt::convex_hull::leafSelectionMatrix(2, 1, 0);
  const Eigen::MatrixXd reconstructed = S * controls;
  require((reconstructed - children.first).norm() < 1.0e-12,
          "Left leaf selection matrix mismatch.");
  const Eigen::MatrixXd Sr =
      traj_opt::convex_hull::leafSelectionMatrix(2, 1, 1);
  require(((Sr * controls) - children.second).norm() < 1.0e-12,
          "Right leaf selection matrix mismatch.");
}

void checkPackTopKAndSignature()
{
  std::vector<traj_opt::ConstraintCandidate> candidates(5);
  for (int i = 0; i < 5; ++i)
  {
    candidates[static_cast<std::size_t>(i)].source_segment = 0;
    candidates[static_cast<std::size_t>(i)].derivative_order = 0;
    candidates[static_cast<std::size_t>(i)].depth = 1;
    candidates[static_cast<std::size_t>(i)].binary_index = i % 2;
    candidates[static_cast<std::size_t>(i)].control_or_bernstein_index = i;
    candidates[static_cast<std::size_t>(i)].plane_id = 0;
    candidates[static_cast<std::size_t>(i)].value = 1.0 + i;
  }
  const auto packed =
      traj_opt::convex_hull::packConstraintCandidates(candidates, 3);
  require(packed.constraints.size() == 3, "Top-K pack size incorrect.");
  require(packed.topology_signature != 0, "Topology signature missing.");
  require(packed.constraints.front().control_or_bernstein_index == 4,
          "Top-K did not keep the worst candidate first.");
}

void checkNondimensionalResidualUnits()
{
  // Constant velocity controls: every control is (v,0,0).
  const double speed = 4.0;
  const double bound = 2.0;
  Eigen::MatrixXd controls(4, 3);
  controls.setZero();
  controls.col(0).setConstant(speed);

  const Eigen::VectorXd raw =
      traj_opt::convex_hull::squaredNormBoundResiduals(controls, bound);
  const double expected_raw = speed * speed - bound * bound;
  require(std::abs(raw.maxCoeff() - expected_raw) < 1.0e-12,
          "Raw squared-norm residual mismatch.");

  traj_opt::convex_hull::AdaptiveBezierForest forest;
  traj_opt::convex_hull::ForestOptions forest_options;
  forest_options.max_depth = 2;
  forest_options.safe_margin = 0.05;
  forest.configure(forest_options);
  forest.seed(0, 1, -1, controls);
  const auto status = forest.evaluateNormBound(0, bound);
  require(status == traj_opt::convex_hull::LeafStatus::VIOLATED,
          "Over-speed leaf should be VIOLATED.");
  const auto &leaf = forest.leaves().front();
  const double expected_nondim = expected_raw / (bound * bound);
  require(std::abs(leaf.upper_bound - expected_nondim) < 1.0e-12,
          "Forest must store nondimensional (||v||^2/bound^2 - 1).");

  // Packed residual path must match the same nondim scale.
  traj_opt::convex_hull::PackedConstraintSet packed;
  traj_opt::convex_hull::PackedConstraint rec;
  rec.source_segment = 0;
  rec.derivative_order = 1;
  rec.depth = 0;
  rec.binary_index = 0;
  rec.control_or_bernstein_index = 0;
  packed.constraints.push_back(rec);

  std::array<Eigen::MatrixXd, 4> order_controls;
  order_controls[0] = Eigen::MatrixXd::Zero(1, 3);
  order_controls[1] = controls;
  order_controls[2] = Eigen::MatrixXd::Zero(1, 3);
  order_controls[3] = Eigen::MatrixXd::Zero(1, 3);
  Eigen::VectorXd durations(1);
  durations << 1.0;
  general_utils::PolyhedraH empty_polys;
  Eigen::VectorXi poly_idx(1);
  poly_idx.setZero();
  Eigen::VectorXd magnitude_bounds(3);
  magnitude_bounds << bound, 40.0, 80.0;
  Eigen::VectorXd multipliers = Eigen::VectorXd::Zero(1);
  std::array<Eigen::MatrixXd, 4> order_gradients;
  // controls_per_piece is the position control count; velocity uses cp-1 rows.
  const auto result = traj_opt::convex_hull::evaluatePackedResiduals(
      packed,
      order_controls,
      durations,
      /*controls_per_piece=*/5,
      empty_polys,
      poly_idx,
      magnitude_bounds,
      0.25,
      multipliers,
      1.0e3,
      order_gradients);
  require(result.values.size() == 1, "Packed residual size mismatch.");
  require(std::abs(result.values(0) - expected_nondim) < 1.0e-12,
          "Packed PHR residual must match nondimensional oracle scale.");
}

void checkFeasibleInfeasibleBoundary()
{
  Eigen::MatrixXd feasible(3, 3);
  feasible.setZero();
  feasible.col(0).setConstant(1.0);
  traj_opt::convex_hull::AdaptiveBezierForest forest;
  traj_opt::convex_hull::ForestOptions options;
  options.max_depth = 2;
  options.safe_margin = 0.05;
  forest.configure(options);
  forest.seed(0, 1, -1, feasible);
  require(forest.evaluateNormBound(0, 2.0) ==
              traj_opt::convex_hull::LeafStatus::SAFE,
          "Under-bound velocity should be SAFE.");

  Eigen::MatrixXd infeasible = feasible;
  infeasible.col(0).setConstant(3.0);
  forest.seed(0, 1, -1, infeasible);
  require(forest.evaluateNormBound(0, 2.0) ==
              traj_opt::convex_hull::LeafStatus::VIOLATED,
          "Over-bound velocity should be VIOLATED.");

  // Position half-space: a·Q+b > 0 is violation; certificate divides by scale.
  traj_opt::convex_hull::CertificateOptions cert_options;
  cert_options.position_scale = 0.25;
  cert_options.certify_jerk = false;
  cert_options.max_depth = 2;
  traj_opt::convex_hull::ContinuousCertificateOracle oracle;
  oracle.configure(cert_options);
  require(std::abs(oracle.options().position_scale - 0.25) < 1.0e-15,
          "Position scale configure failed.");
  require(!oracle.options().certify_jerk,
          "penna_jerk<=0 path must disable jerk certification.");
}

void checkPackedCorrectorEndToEnd()
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
          "Failed to initialize packed corrector optimizer.");

  general_utils::PolyhedraH corridors(1);
  corridors[0].resize(6, 4);
  corridors[0] <<
      1.0, 0.0, 0.0, -0.2,
      -1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, -0.2,
      0.0, -1.0, 0.0, -0.2,
      0.0, 0.0, 1.0, -0.2,
      0.0, 0.0, -1.0, -0.2;
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
          "Failed to materialize trajectory.");
  const auto certificate =
      manager.computeAdaptiveContinuousCertificate(optimizer.getTrajectory());
  require(!certificate.continuous_feasible,
          "Narrow corridor should fail certificate.");
  require(!certificate.violated.empty(),
          "Failed certificate should emit candidates.");

  auto packed = traj_opt::convex_hull::packConstraintCandidates(
      certificate.violated, 16);
  require(!packed.constraints.empty(), "Pack produced no constraints.");

  cost_functional_manager::ExpPackedCorrectorCostManager corrector;
  cost_functional_manager::ExpPackedCorrectorCostManager::Options options;
  options.position_scale = 0.25;
  options.max_constraints = 32;
  corrector.configure(options);
  corrector.reset(&corridors,
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
  Eigen::VectorXd multipliers =
      traj_opt::convex_hull::seedMultipliers(packed);
  require(corrector.initializeFromPacked(packed, multipliers, 1.0e3),
          "initializeFromPacked failed.");

  Optimizer::CoeffMat grad_coeff =
      Optimizer::CoeffMat::Zero(optimizer.getPieceNum() * 8, 3);
  Eigen::VectorXd grad_durations =
      Eigen::VectorXd::Zero(optimizer.getPieceNum());
  const double cost = corrector.evaluateCoefficient(
      optimizer.getTrajectory(), grad_coeff, grad_durations);
  require(std::isfinite(cost), "Packed PHR cost is not finite.");
  require(corrector.constraintValues().size() ==
              static_cast<Eigen::Index>(packed.constraints.size()),
          "Constraint value size mismatch.");
  require(corrector.constraintCount() <= 16,
          "Packed constraint count exceeded top-K.");
}

} // namespace

int main()
{
  try
  {
    checkLeafSelectionMatrix();
    checkPackTopKAndSignature();
    checkNondimensionalResidualUnits();
    checkFeasibleInfeasibleBoundary();
    checkPackedCorrectorEndToEnd();
    std::cout << "phase2_corrector_self_test passed" << std::endl;
  }
  catch (const std::exception &error)
  {
    std::cerr << "phase2_corrector_self_test failed: " << error.what()
              << std::endl;
    return 1;
  }
  return 0;
}
