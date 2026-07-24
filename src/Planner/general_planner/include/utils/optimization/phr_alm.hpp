#ifndef PHR_ALM_HPP
#define PHR_ALM_HPP

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>

namespace optimization
{
namespace phr_alm
{
/** Parameters for the Powell-Hestenes-Rockafellar inequality ALM.
 *
 * Constraints use g(x) <= 0 and the inner objective receives
 *
 *   sum_i max(0, lambda_i + rho g_i)^2 / (2 rho).
 */
struct Parameters
{
  int max_outer_iterations{4};
  double initial_penalty{1.0};
  double penalty_growth{5.0};
  double progress_ratio{0.5};
  double constraint_tolerance{1.0e-2};
  bool accept_initial_feasible{false};
};

enum class Status
{
  CONVERGED = 0,
  MAX_OUTER_ITERATIONS = 1,
  INVALID_PROBLEM = -1,
  INNER_SOLVER_FAILED = -2
};

enum class TopologyUpdate
{
  UNCHANGED = 0,
  APPEND = 1,
  REPLACE = 2
};

struct Report
{
  Status status{Status::INVALID_PROBLEM};
  int inner_status{0};
  int outer_iterations{0};
  int inner_solves{0};
  int topology_changes{0};
  std::size_t constraints{0};
  double penalty{0.0};
  double max_violation{std::numeric_limits<double>::infinity()};
  double primal_residual{0.0};
  double dual_residual{0.0};
  double complementarity_residual{0.0};
  double stationarity_residual{0.0};

  bool converged() const { return status == Status::CONVERGED; }
};

inline Parameters sanitize(const Parameters &input)
{
  Parameters result = input;
  result.max_outer_iterations = std::max(1, result.max_outer_iterations);
  result.initial_penalty = std::max(1.0e-12, result.initial_penalty);
  result.penalty_growth = std::max(1.0, result.penalty_growth);
  result.progress_ratio = std::clamp(result.progress_ratio, 0.0, 1.0);
  result.constraint_tolerance =
      std::max(0.0, result.constraint_tolerance);
  return result;
}

/**
 * Generic PHR-ALM outer solver.
 *
 * initialize(x, constraints) builds the initial frozen constraint topology.
 * apply_state(lambda, rho) exposes the current PHR state to the inner
 * objective. inner_solve(x, f) minimizes that fixed objective and returns a
 * negative value on failure. evaluate(x, constraints, topology_update)
 * certifies the inner solution and may append to or atomically replace the
 * topology between inner solves. Appends preserve existing multipliers;
 * replacements reset multipliers and rho.
 */
template <typename Vector,
          typename Initialize,
          typename ApplyState,
          typename InnerSolve,
          typename EvaluateConstraints>
Status solve(Vector &x,
             double &minimum,
             const Parameters &raw_parameters,
             Initialize &&initialize,
             ApplyState &&apply_state,
             InnerSolve &&inner_solve,
             EvaluateConstraints &&evaluate_constraints,
             Report &report)
{
  const Parameters parameters = sanitize(raw_parameters);
  report = Report{};
  Vector constraints;
  if (!initialize(x, constraints) || !constraints.allFinite())
  {
    report.status = Status::INVALID_PROBLEM;
    return report.status;
  }

  Vector multipliers = Vector::Zero(constraints.size());
  double penalty = parameters.initial_penalty;
  double previous_violation = std::numeric_limits<double>::infinity();
  report.constraints = static_cast<std::size_t>(constraints.size());
  report.max_violation =
      constraints.size() > 0
          ? std::max(0.0, constraints.maxCoeff())
          : 0.0;
  report.penalty = penalty;
  if (parameters.accept_initial_feasible &&
      report.max_violation <= parameters.constraint_tolerance)
  {
    report.status = Status::CONVERGED;
    return report.status;
  }

  int outer = 0;
  const int max_topology_changes =
      std::max<int>(64, static_cast<int>(constraints.size()));
  while (outer < parameters.max_outer_iterations)
  {
    ++report.inner_solves;
    apply_state(multipliers, penalty);
    report.inner_status = inner_solve(x, minimum);
    if (report.inner_status < 0)
    {
      report.status = Status::INNER_SOLVER_FAILED;
      return report.status;
    }

    TopologyUpdate topology_update = TopologyUpdate::UNCHANGED;
    if (!evaluate_constraints(x, constraints, topology_update) ||
        !constraints.allFinite())
    {
      report.status = Status::INVALID_PROBLEM;
      return report.status;
    }

    if (topology_update != TopologyUpdate::UNCHANGED ||
        constraints.size() != multipliers.size())
    {
      ++report.topology_changes;
      if (report.topology_changes > max_topology_changes)
      {
        report.status = Status::INVALID_PROBLEM;
        return report.status;
      }
      if (topology_update == TopologyUpdate::APPEND &&
          constraints.size() >= multipliers.size())
      {
        const Eigen::Index previous_size = multipliers.size();
        multipliers.conservativeResize(constraints.size());
        multipliers.tail(constraints.size() - previous_size).setZero();
      }
      else
      {
        multipliers.setZero(constraints.size());
        penalty = parameters.initial_penalty;
        previous_violation = std::numeric_limits<double>::infinity();
      }
      report.constraints = static_cast<std::size_t>(constraints.size());
      continue;
    }

    ++outer;
    report.outer_iterations = outer;
    report.constraints = static_cast<std::size_t>(constraints.size());
    report.max_violation =
        constraints.size() > 0
            ? std::max(0.0, constraints.maxCoeff())
            : 0.0;
    report.primal_residual = report.max_violation;
    report.dual_residual =
        multipliers.size() > 0
            ? std::max(0.0, (-multipliers).maxCoeff())
            : 0.0;
    report.complementarity_residual =
        multipliers.size() > 0
            ? (multipliers.cwiseProduct(constraints))
                  .cwiseAbs()
                  .maxCoeff()
            : 0.0;
    report.penalty = penalty;
    if (report.max_violation <= parameters.constraint_tolerance)
    {
      report.status = Status::CONVERGED;
      return report.status;
    }

    multipliers =
        (multipliers + penalty * constraints).cwiseMax(0.0);
    if (std::isfinite(previous_violation) &&
        report.max_violation >
            parameters.progress_ratio * previous_violation)
    {
      penalty *= parameters.penalty_growth;
    }
    previous_violation = report.max_violation;
  }

  report.status = Status::MAX_OUTER_ITERATIONS;
  report.penalty = penalty;
  return report.status;
}
} // namespace phr_alm
} // namespace optimization

#endif
