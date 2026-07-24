#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace traj_opt
{

/**
 * Phase-0/1 solver quality semantics.
 *
 * These flags deliberately separate:
 * - sampled feasibility (dense nodes / soft penalty logs),
 * - continuous Bezier certificates,
 * - approximate first-order stationarity / KKT residuals,
 * - optional strict polishing.
 *
 * Online fast L-BFGS may accept a trajectory that is not a strict stationary
 * point; callers must inspect these fields to know what was traded away.
 */
struct SolverQualityReport
{
  bool sampled_feasible{false};
  bool continuous_feasible{false};
  bool robustly_certified{false};
  bool approximately_kkt{false};
  bool strictly_polished{false};
  bool trajectory_stable{false};
  bool has_certified_incumbent{false};

  double max_sampled_violation{0.0};
  double max_position_violation{0.0};
  double max_derivative_violation{0.0};
  double max_normalized_violation{0.0};

  double min_position_margin{
      std::numeric_limits<double>::infinity()};
  double min_derivative_margin{
      std::numeric_limits<double>::infinity()};

  double relative_cost_change{0.0};
  double relative_decision_step{0.0};
  double relative_physical_time_change{0.0};
  double relative_waypoint_step{0.0};
  double scaled_gradient_inf{0.0};
  double min_accepted_step{0.0};

  double primal_residual{0.0};
  double dual_residual{0.0};
  double complementarity_residual{0.0};
  double stationarity_residual{0.0};

  std::size_t scalar_constraint_checks{0};
  std::size_t unresolved_leaves{0};
  int max_depth_used{0};

  std::string summary() const
  {
    std::string out = "sampled=";
    out += sampled_feasible ? "1" : "0";
    out += " continuous=";
    out += continuous_feasible ? "1" : "0";
    out += " robust=";
    out += robustly_certified ? "1" : "0";
    out += " approx_kkt=";
    out += approximately_kkt ? "1" : "0";
    out += " polished=";
    out += strictly_polished ? "1" : "0";
    return out;
  }
};

/**
 * Worst Bernstein / control-point constraint extracted from a failed leaf.
 * Phase 2 packs these into ALM/SQP correction sets.
 */
struct ConstraintCandidate
{
  int source_segment{-1};
  int derivative_order{-1};
  int leaf_id{-1};
  int depth{0};
  int binary_index{0};
  int control_or_bernstein_index{-1};
  int plane_id{-1};

  double value{0.0};
  double margin{0.0};
  double historical_multiplier{0.0};
};

struct ContinuousCertificateReport
{
  bool continuous_feasible{false};
  bool robustly_certified{false};
  bool fully_resolved{true};
  // Mirrors CertificateOptions / penalty weights: penna_jerk<=0 => false.
  bool jerk_certificate_enabled{false};
  bool velocity_certificate_enabled{false};
  bool acceleration_certificate_enabled{false};
  bool position_certificate_enabled{false};

  double max_position_violation{0.0};
  double max_velocity_violation{0.0};
  double max_acceleration_violation{0.0};
  double max_jerk_violation{0.0};
  double max_normalized_violation{0.0};

  double min_position_margin{
      std::numeric_limits<double>::infinity()};
  double min_velocity_margin{
      std::numeric_limits<double>::infinity()};
  double min_acceleration_margin{
      std::numeric_limits<double>::infinity()};
  double min_jerk_margin{
      std::numeric_limits<double>::infinity()};

  std::size_t scalar_constraint_checks{0};
  std::size_t active_leaves{0};
  std::size_t unresolved_leaves{0};
  int max_depth_used{0};

  std::vector<ConstraintCandidate> violated;
};

} // namespace traj_opt
