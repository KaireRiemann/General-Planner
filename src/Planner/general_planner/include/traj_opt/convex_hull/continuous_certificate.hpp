#pragma once

#include "traj_opt/convex_hull/adaptive_bezier_forest.hpp"
#include "traj_opt/convex_hull/constraint_generator.hpp"
#include "traj_opt/convex_hull/convex_hull.hpp"
#include "traj_opt/solver_quality_report.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace traj_opt
{
namespace convex_hull
{

struct CertificateOptions
{
  int max_depth{6};
  double safe_margin{0.05};
  double bound_gap_tol{1.0e-6};
  double refine_margin{0.05};
  double coarsen_margin{0.10};
  double position_scale{0.25};
  bool certify_position{true};
  bool certify_velocity{true};
  bool certify_acceleration{true};
  bool certify_jerk{true};
  bool enable_hysteresis{true};
  std::size_t max_leaves_per_stream{2048};
  int top_k_violations{8};
};

/**
 * Phase-1 adaptive Bezier continuous certificate oracle.
 *
 * Does not modify the live LBFGS penalty topology. For each source segment:
 *   position: scalar half-space Bernstein + de Casteljau BnB
 *   derivatives: Level A vector hull → Level B squared-norm Bernstein → Level C split
 */
class ContinuousCertificateOracle
{
public:
  using Hull = Representation<3>;

  void configure(const CertificateOptions &options)
  {
    options_ = options;
    options_.max_depth = std::clamp(options_.max_depth, 0, 12);
    options_.safe_margin = std::max(0.0, options_.safe_margin);
    options_.position_scale = std::max(1.0e-6, options_.position_scale);
    forest_options_.max_depth = options_.max_depth;
    forest_options_.safe_margin = options_.safe_margin;
    forest_options_.bound_gap_tol = options_.bound_gap_tol;
    forest_options_.refine_margin = options_.refine_margin;
    forest_options_.coarsen_margin = options_.coarsen_margin;
    forest_options_.enable_hysteresis = options_.enable_hysteresis;
    forest_options_.max_leaves = options_.max_leaves_per_stream;
  }

  const CertificateOptions &options() const { return options_; }

  template <typename Trajectory, typename Polyhedra>
  ContinuousCertificateReport certify(
      const Trajectory &trajectory,
      const Polyhedra &h_polys,
      const Eigen::VectorXi &h_poly_idx,
      const Eigen::VectorXd &magnitude_bounds) const
  {
    ContinuousCertificateReport report;
    report.position_certificate_enabled = options_.certify_position;
    report.velocity_certificate_enabled = options_.certify_velocity;
    report.acceleration_certificate_enabled = options_.certify_acceleration;
    report.jerk_certificate_enabled = options_.certify_jerk;
    const int segments = trajectory.getPieceNum();
    if (segments <= 0)
    {
      report.fully_resolved = true;
      report.continuous_feasible = true;
      report.robustly_certified = true;
      return report;
    }

    Hull position_hull;
    position_hull.resetTopology(segments,
                                Trajectory::COEFF_NUM,
                                Basis::Bezier,
                                0,
                                0);
    trajectory.updateConvexHull(position_hull);
    const Eigen::MatrixXd &position_controls = position_hull.controls();
    const Eigen::VectorXd &durations = trajectory.getDurations();
    const int controls_per_piece = position_hull.sourceDegree() + 1;

    std::array<Eigen::MatrixXd, 4> order_controls;
    order_controls[0] = position_controls;
    buildHodographs(position_controls,
                    durations,
                    controls_per_piece,
                    order_controls);

    ConstraintGeneratorOptions generator_options;
    generator_options.top_k_per_leaf = 1;
    generator_options.max_candidates =
        static_cast<std::size_t>(std::max(1, options_.top_k_violations));

    for (int segment = 0; segment < segments; ++segment)
    {
      if (options_.certify_position && segment < h_poly_idx.size())
      {
        const int poly_id = h_poly_idx(segment);
        if (poly_id >= 0 && poly_id < static_cast<int>(h_polys.size()))
        {
          certifyPositionSegment(segment,
                                 order_controls[0].middleRows(
                                     segment * controls_per_piece,
                                     controls_per_piece),
                                 h_polys[static_cast<std::size_t>(poly_id)],
                                 generator_options,
                                 report);
        }
      }

      for (int order = 1; order <= 3; ++order)
      {
        const bool enabled =
            (order == 1 && options_.certify_velocity) ||
            (order == 2 && options_.certify_acceleration) ||
            (order == 3 && options_.certify_jerk);
        if (!enabled || magnitude_bounds.size() < order)
        {
          continue;
        }
        const double bound = magnitude_bounds(order - 1);
        const int cp = controls_per_piece - order;
        if (cp <= 0)
        {
          continue;
        }
        certifyNormSegment(segment,
                           order,
                           order_controls[static_cast<std::size_t>(order)]
                               .middleRows(segment * cp, cp),
                           bound,
                           generator_options,
                           report);
      }
    }

    finalizeReport(report);
    return report;
  }

private:
  CertificateOptions options_{};
  ForestOptions forest_options_{};

  static void buildHodographs(
      const Eigen::MatrixXd &position_controls,
      const Eigen::VectorXd &durations,
      int controls_per_piece,
      std::array<Eigen::MatrixXd, 4> &order_controls)
  {
    const int segments = static_cast<int>(durations.size());
    order_controls[0] = position_controls;
    for (int order = 1; order <= 3; ++order)
    {
      const int previous_cp = controls_per_piece - (order - 1);
      const int current_cp = previous_cp - 1;
      order_controls[static_cast<std::size_t>(order)].resize(
          segments * current_cp, 3);
      const double degree_factor = static_cast<double>(previous_cp - 1);
      for (int segment = 0; segment < segments; ++segment)
      {
        const double scale = degree_factor / durations(segment);
        const int previous_row = segment * previous_cp;
        const int current_row = segment * current_cp;
        for (int control = 0; control < current_cp; ++control)
        {
          order_controls[static_cast<std::size_t>(order)].row(
              current_row + control) =
              scale *
              (order_controls[static_cast<std::size_t>(order - 1)].row(
                   previous_row + control + 1) -
               order_controls[static_cast<std::size_t>(order - 1)].row(
                   previous_row + control));
        }
      }
    }
  }

  void certifyPositionSegment(
      int segment,
      const Eigen::Ref<const Eigen::MatrixXd> &controls,
      const Eigen::MatrixXd &poly,
      const ConstraintGeneratorOptions &generator_options,
      ContinuousCertificateReport &report) const
  {
    for (int plane = 0; plane < poly.rows(); ++plane)
    {
      const Eigen::Vector3d normal =
          poly.block<1, 3>(plane, 0).transpose();
      const double offset = poly(plane, 3);

      AdaptiveBezierForest forest;
      forest.configure(forest_options_);
      forest.seed(segment, 0, plane, controls);

      std::queue<int> open;
      open.push(0);
      while (!open.empty())
      {
        const int leaf_id = open.front();
        open.pop();
        const LeafStatus status =
            forest.evaluateHalfSpace(leaf_id, normal, offset);
        const auto &leaf =
            forest.leaves()[static_cast<std::size_t>(leaf_id)];
        report.scalar_constraint_checks +=
            static_cast<std::size_t>(leaf.controls.rows());

        if (status == LeafStatus::UNCERTAIN)
        {
          const int left = forest.split(leaf_id);
          if (left >= 0)
          {
            open.push(left);
            open.push(left + 1);
          }
        }
      }

      absorbForest(forest, 0, generator_options, report);
    }
  }

  void certifyNormSegment(
      int segment,
      int order,
      const Eigen::Ref<const Eigen::MatrixXd> &controls,
      double bound,
      const ConstraintGeneratorOptions &generator_options,
      ContinuousCertificateReport &report) const
  {
    AdaptiveBezierForest forest;
    forest.configure(forest_options_);
    forest.seed(segment, order, -1, controls);

    std::queue<int> open;
    open.push(0);
    while (!open.empty())
    {
      const int leaf_id = open.front();
      open.pop();
      const LeafStatus status = forest.evaluateNormBound(leaf_id, bound);
      const auto &leaf =
          forest.leaves()[static_cast<std::size_t>(leaf_id)];
      // Level A vector scan + Level B Bernstein scan.
      report.scalar_constraint_checks +=
          static_cast<std::size_t>(2 * leaf.controls.rows());

      if (status == LeafStatus::UNCERTAIN)
      {
        const int left = forest.split(leaf_id);
        if (left >= 0)
        {
          open.push(left);
          open.push(left + 1);
        }
      }
    }

    absorbForest(forest, order, generator_options, report);
  }

  void absorbForest(const AdaptiveBezierForest &forest,
                    int order,
                    const ConstraintGeneratorOptions &generator_options,
                    ContinuousCertificateReport &report) const
  {
    report.max_depth_used =
        std::max(report.max_depth_used, forest.maxDepthUsed());
    for (const auto &leaf : forest.leaves())
    {
      if (!leaf.is_active)
      {
        continue;
      }
      ++report.active_leaves;
      // Position leaves store metric half-space residuals; derivative leaves
      // store nondimensional (||v||^2/bound^2 - 1). Normalize both into one
      // comparable score for continuous_feasible / top-K ranking.
      const double normalized_upper =
          order == 0 ? leaf.upper_bound / options_.position_scale
                     : leaf.upper_bound;
      const double violation = std::max(0.0, normalized_upper);
      const double margin = -normalized_upper;
      report.max_normalized_violation =
          std::max(report.max_normalized_violation, violation);

      if (order == 0)
      {
        report.max_position_violation = std::max(
            report.max_position_violation, std::max(0.0, leaf.upper_bound));
        report.min_position_margin =
            std::min(report.min_position_margin, -leaf.upper_bound);
      }
      else if (order == 1)
      {
        report.max_velocity_violation =
            std::max(report.max_velocity_violation, violation);
        report.min_velocity_margin =
            std::min(report.min_velocity_margin, margin);
      }
      else if (order == 2)
      {
        report.max_acceleration_violation =
            std::max(report.max_acceleration_violation, violation);
        report.min_acceleration_margin =
            std::min(report.min_acceleration_margin, margin);
      }
      else
      {
        report.max_jerk_violation =
            std::max(report.max_jerk_violation, violation);
        report.min_jerk_margin =
            std::min(report.min_jerk_margin, margin);
      }

      if (leaf.status == LeafStatus::MAX_DEPTH_REACHED ||
          leaf.status == LeafStatus::UNCERTAIN)
      {
        ++report.unresolved_leaves;
      }
    }

    auto candidates =
        extractViolatedCandidates(forest, generator_options);
    if (order == 0)
    {
      for (auto &candidate : candidates)
      {
        candidate.value /= options_.position_scale;
        candidate.margin = -candidate.value;
      }
    }
    report.violated.insert(report.violated.end(),
                           candidates.begin(),
                           candidates.end());
  }

  void finalizeReport(ContinuousCertificateReport &report) const
  {
    report.max_position_violation =
        std::max(0.0, report.max_position_violation);
    report.max_velocity_violation =
        std::max(0.0, report.max_velocity_violation);
    report.max_acceleration_violation =
        std::max(0.0, report.max_acceleration_violation);
    report.max_jerk_violation =
        std::max(0.0, report.max_jerk_violation);
    report.max_normalized_violation =
        std::max(0.0, report.max_normalized_violation);

    report.fully_resolved = report.unresolved_leaves == 0;
    // Every active leaf stores a Bernstein upper bound over its complete
    // interval.  Therefore upper_bound <= 0 is already a rigorous
    // continuous-time feasibility proof, even when the adaptive classifier
    // calls the leaf UNCERTAIN/MAX_DEPTH_REACHED because it has not reached
    // safe_margin.  Conflating that robust-interior classification with
    // basic feasibility made valid boundary-near trajectories fail forever:
    // max_normalized_violation was zero while continuous_feasible was false.
    //
    // Keep fully_resolved as a diagnostic for robust refinement, but let the
    // actual certificate be decided by the certified upper bounds.  The
    // stronger safe-margin requirement remains in robustly_certified below.
    report.continuous_feasible =
        report.max_normalized_violation <= 0.0;
    // Position robust margin stays metric; derivative margins are
    // nondimensional (||v||^2/bound^2 - 1). Disabled orders are ignored
    // (e.g. penna_jerk<=0 ⇒ certify_jerk=false ⇒ jerk not required).
    report.robustly_certified =
        report.continuous_feasible &&
        (!options_.certify_position ||
         report.min_position_margin >= options_.safe_margin) &&
        (!options_.certify_velocity ||
         report.min_velocity_margin >= options_.safe_margin) &&
        (!options_.certify_acceleration ||
         report.min_acceleration_margin >= options_.safe_margin) &&
        (!options_.certify_jerk ||
         report.min_jerk_margin >= options_.safe_margin);

    std::sort(report.violated.begin(),
              report.violated.end(),
              [](const ConstraintCandidate &a,
                 const ConstraintCandidate &b) {
                return a.value > b.value;
              });
    if (static_cast<int>(report.violated.size()) >
        options_.top_k_violations)
    {
      report.violated.resize(
          static_cast<std::size_t>(options_.top_k_violations));
    }
  }
};

} // namespace convex_hull
} // namespace traj_opt
