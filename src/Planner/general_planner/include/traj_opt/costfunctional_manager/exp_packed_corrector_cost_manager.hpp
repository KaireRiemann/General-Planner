#pragma once

#include "traj_opt/convex_hull/batched_residuals.hpp"
#include "traj_opt/convex_hull/constraint_pack.hpp"
#include "traj_opt/convex_hull/convex_hull.hpp"
#include "traj_opt/costfunctional_manager/exp_convex_cost_manager.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace cost_functional_manager
{

/**
 * Phase-2 packed active-leaf corrector.
 *
 * Topology is frozen for each inner LBFGS solve. Between outers the pack may
 * APPEND new oracle candidates; REPLACE is reserved for signature mismatch.
 */
class ExpPackedCorrectorCostManager
{
public:
  using Basis = traj_opt::convex_hull::Basis;
  using PackedConstraintSet = traj_opt::convex_hull::PackedConstraintSet;

  struct Options
  {
    double position_scale{0.25};
    int max_constraints{64};
    bool flatness_enabled{false};
    traj_opt::convex_hull::FlatnessConvexHullCost::Config flatness{};
  };

  struct UpdateReport
  {
    bool initialized{false};
    bool topology_changed{false};
    bool topology_append_only{false};
    bool certified{false};
    double max_normalized_violation{0.0};
    double primal_residual{0.0};
    double dual_residual{0.0};
    double complementarity_residual{0.0};
    double stationarity_residual{0.0};
    double penalty{0.0};
    std::size_t constraints{0};
    std::uint64_t topology_signature{0};
  };

  void configure(const Options &options)
  {
    options_ = options;
    options_.position_scale = std::max(1.0e-6, options_.position_scale);
    options_.max_constraints = std::max(1, options_.max_constraints);
  }

  void reset(const general_utils::PolyhedraH *h_polys,
             const Eigen::VectorXi *h_poly_idx,
             const general_utils::Mat3Df *waypoint_attractors,
             const general_utils::VecDf *waypoint_attractor_dead_d,
             double smooth_eps,
             const general_utils::VecDf &magnitude_bounds,
             const general_utils::VecDf &penalty_weights,
             flatness::FlatnessMap *quadrotor_flatness,
             const traj_opt::SwarmPenaltyConfig &swarm_config,
             const traj_opt::SwarmTrajectoriesConstPtr &swarm_trajs,
             double swarm_current_wall_time,
             const general_utils::vec_E<general_utils::Vec3f> *guide_path = nullptr,
             const std::vector<double> *guide_t = nullptr,
             double weight_guide_integral = 0.0,
             double guide_path_tube_radius = 0.0,
             double guide_path_z_tube_radius = 0.0,
             double guide_path_huber_delta = 0.0,
             bool guide_path_time_gradient_en = false,
             double weight_guide_z_lower = 0.0,
             double guide_z_lower_tolerance = 0.0)
  {
    h_polys_ = h_polys;
    h_poly_idx_ = h_poly_idx;
    magnitude_bounds_ = magnitude_bounds;

    general_utils::VecDf residual_weights = penalty_weights;
    const int polynomial_weight_count =
        std::min<int>(4, residual_weights.size());
    residual_weights.head(polynomial_weight_count).setZero();
    residual_manager_.configure(Basis::Bezier, 0);
    residual_manager_.reset(h_polys,
                            h_poly_idx,
                            waypoint_attractors,
                            waypoint_attractor_dead_d,
                            smooth_eps,
                            magnitude_bounds,
                            residual_weights,
                            quadrotor_flatness,
                            swarm_config,
                            swarm_trajs,
                            swarm_current_wall_time,
                            guide_path,
                            guide_t,
                            weight_guide_integral,
                            guide_path_tube_radius,
                            guide_path_z_tube_radius,
                            guide_path_huber_delta,
                            guide_path_time_gradient_en,
                            weight_guide_z_lower,
                            guide_z_lower_tolerance);

    packed_ = PackedConstraintSet{};
    multipliers_.resize(0);
    penalty_ = 1.0;
    constraint_values_.resize(0);
    initialized_ = false;
    last_report_ = UpdateReport{};
    last_scalar_checks_ = 0;
  }

  bool usesDenseSampling() const
  {
    return residual_manager_.usesDenseSampling();
  }

  bool usesSampleCost() const
  {
    return residual_manager_.usesSampleCost();
  }

  void beginEvaluation(const std::vector<double> *times)
  {
    residual_manager_.beginEvaluation(times);
  }

  bool initializeFromPacked(const PackedConstraintSet &packed,
                            const Eigen::VectorXd &multipliers,
                            double penalty)
  {
    if (!ready() || packed.constraints.empty())
    {
      return false;
    }
    packed_ = packed;
    multipliers_ = multipliers;
    if (multipliers_.size() !=
        static_cast<Eigen::Index>(packed_.constraints.size()))
    {
      multipliers_ = traj_opt::convex_hull::seedMultipliers(packed_);
    }
    penalty_ = std::max(1.0e-12, penalty);
    constraint_values_ = Eigen::VectorXd::Zero(multipliers_.size());
    initialized_ = true;
    last_report_.initialized = true;
    last_report_.constraints = packed_.constraints.size();
    last_report_.topology_signature = packed_.topology_signature;
    last_report_.penalty = penalty_;
    return true;
  }

  void setPhrState(const Eigen::VectorXd &multipliers, double penalty)
  {
    if (!initialized_ ||
        multipliers.size() !=
            static_cast<Eigen::Index>(packed_.constraints.size()) ||
        !multipliers.allFinite() || !std::isfinite(penalty) ||
        penalty <= 0.0)
    {
      return;
    }
    multipliers_ = multipliers;
    penalty_ = penalty;
  }

  const Eigen::VectorXd &constraintValues() const
  {
    return constraint_values_;
  }

  const Eigen::VectorXd &multipliers() const { return multipliers_; }

  const PackedConstraintSet &packedSet() const { return packed_; }

  const UpdateReport &lastUpdateReport() const { return last_report_; }

  std::size_t constraintCount() const { return packed_.constraints.size(); }

  bool hasFlatnessConstraints() const
  {
    for (const auto &constraint : packed_.constraints)
    {
      if (constraint.kind >=
              traj_opt::ConstraintKind::FlatnessVelocityTrust &&
          constraint.kind <=
              traj_opt::ConstraintKind::FlatnessAngularRate)
      {
        return true;
      }
    }
    return false;
  }

  bool shrinkFlatnessTrustRegions(double factor)
  {
    factor = std::clamp(factor, 0.1, 0.99);
    bool changed = false;
    for (auto &constraint : packed_.constraints)
    {
      if (constraint.kind < traj_opt::ConstraintKind::FlatnessVelocityTrust ||
          constraint.kind > traj_opt::ConstraintKind::FlatnessAngularRate)
      {
        continue;
      }
      auto &model = constraint.flatness;
      const double old_radius = model.trust_radius;
      const double new_radius =
          std::max(model.anchor_radius + 1.0e-6, factor * old_radius);
      if (new_radius + 1.0e-12 >= old_radius)
      {
        continue;
      }
      const double ratio = new_radius / old_radius;
      model.trust_radius = new_radius;
      model.drag_remainder *= ratio * ratio;
      model.force_remainder *= ratio * ratio;
      changed = true;
    }
    return changed;
  }

  std::size_t lastScalarChecks() const { return last_scalar_checks_; }

  std::size_t activeControlPointChecksPerEvaluation() const
  {
    return last_scalar_checks_;
  }

  template <typename Trajectory>
  UpdateReport inspectAndMaybeAppend(
      const Trajectory &trajectory,
      const std::vector<traj_opt::ConstraintCandidate> &fresh_candidates,
      bool allow_append)
  {
    last_report_ = UpdateReport{};
    last_report_.initialized = initialized_;
    last_report_.constraints = packed_.constraints.size();
    last_report_.topology_signature = packed_.topology_signature;
    last_report_.penalty = penalty_;
    if (!initialized_)
    {
      return last_report_;
    }

    refreshConstraintValues(trajectory);

    last_report_.max_normalized_violation =
        constraint_values_.size() > 0
            ? std::max(0.0, constraint_values_.maxCoeff())
            : 0.0;
    last_report_.primal_residual = last_report_.max_normalized_violation;
    last_report_.dual_residual =
        multipliers_.size() > 0 ? std::max(0.0, (-multipliers_).maxCoeff())
                                : 0.0;
    last_report_.complementarity_residual = 0.0;
    if (multipliers_.size() > 0 &&
        constraint_values_.size() == multipliers_.size())
    {
      last_report_.complementarity_residual =
          multipliers_.cwiseProduct(constraint_values_).cwiseAbs().maxCoeff();
    }

    if (allow_append &&
        traj_opt::convex_hull::appendPackedCandidates(
            packed_,
            multipliers_,
            fresh_candidates,
            options_.max_constraints))
    {
      const Eigen::Index before_guards = multipliers_.size();
      traj_opt::convex_hull::appendFlatnessTrustRegionGuards(
          packed_,
          /*velocity_controls_per_leaf=*/7,
          options_.max_constraints);
      if (static_cast<Eigen::Index>(packed_.constraints.size()) >
          before_guards)
      {
        Eigen::VectorXd grown = Eigen::VectorXd::Zero(
            static_cast<Eigen::Index>(packed_.constraints.size()));
        grown.head(before_guards) = multipliers_;
        multipliers_.swap(grown);
      }
      constraint_values_ = Eigen::VectorXd::Zero(
          static_cast<Eigen::Index>(packed_.constraints.size()));
      last_report_.topology_changed = true;
      last_report_.topology_append_only = true;
      last_report_.constraints = packed_.constraints.size();
      last_report_.topology_signature = packed_.topology_signature;
    }
    return last_report_;
  }

  const general_utils::VecDf &getPenaltyLog() const
  {
    combined_penalty_log_ = residual_manager_.getPenaltyLog();
    return combined_penalty_log_;
  }

  double guideIntegralViolation() const
  {
    return residual_manager_.guideIntegralViolation();
  }

  double guideZLowerViolation() const
  {
    return residual_manager_.guideZLowerViolation();
  }
  double guideCostLog() const { return residual_manager_.guideCostLog(); }
  double guideMaxAbsTimeGrad() const
  {
    return residual_manager_.guideMaxAbsTimeGrad();
  }
  int guideOutOfTimeRangeSamples() const
  {
    return residual_manager_.guideOutOfTimeRangeSamples();
  }

  double evaluateIntegral(int logical_idx,
                          double t_local,
                          double t_global,
                          int seg_idx,
                          int step_in_seg,
                          const Eigen::Vector3d &position,
                          const Eigen::Vector3d &velocity,
                          const Eigen::Vector3d &acceleration,
                          const Eigen::Vector3d &jerk,
                          Eigen::Vector3d &grad_position,
                          Eigen::Vector3d &grad_velocity,
                          Eigen::Vector3d &grad_acceleration,
                          Eigen::Vector3d &grad_jerk,
                          double &grad_time) const
  {
    return residual_manager_.evaluateIntegral(logical_idx,
                                              t_local,
                                              t_global,
                                              seg_idx,
                                              step_in_seg,
                                              position,
                                              velocity,
                                              acceleration,
                                              jerk,
                                              grad_position,
                                              grad_velocity,
                                              grad_acceleration,
                                              grad_jerk,
                                              grad_time);
  }

  template <typename SampleBuffer>
  double evaluateSample(const SampleBuffer &samples,
                        Eigen::Matrix<double, 3, Eigen::Dynamic> &grad_position,
                        Eigen::VectorXd &grad_time) const
  {
    return residual_manager_.evaluateSample(samples, grad_position, grad_time);
  }

  template <typename Trajectory>
  double evaluateCoefficient(
      const Trajectory &trajectory,
      typename Trajectory::CoeffMat &grad_coefficients,
      Eigen::VectorXd &grad_durations) const
  {
    if (!initialized_ || !ready() || packed_.constraints.empty())
    {
      return 0.0;
    }

    const int segments = trajectory.getPieceNum();
    hull_.resetTopology(segments,
                        Trajectory::COEFF_NUM,
                        Basis::Bezier,
                        0,
                        0);
    trajectory.updateConvexHull(hull_);
    const Eigen::MatrixXd &position_controls = hull_.controls();
    const Eigen::VectorXd &durations = trajectory.getDurations();
    const int controls_per_piece = hull_.sourceDegree() + 1;

    std::array<Eigen::MatrixXd, 4> order_controls;
    traj_opt::convex_hull::buildHodographControls(position_controls,
                                                  durations,
                                                  controls_per_piece,
                                                  order_controls);

    std::array<Eigen::MatrixXd, 4> order_gradients;
    const auto residual = traj_opt::convex_hull::evaluatePackedResiduals(
        packed_,
        order_controls,
        durations,
        controls_per_piece,
        *h_polys_,
        *h_poly_idx_,
        magnitude_bounds_,
        options_.position_scale,
        multipliers_,
        penalty_,
        options_.flatness_enabled ? &options_.flatness : nullptr,
        order_gradients);
    constraint_values_ = residual.values;
    last_scalar_checks_ = residual.scalar_checks;
    if (!residual.trust_region_feasible)
    {
      return std::numeric_limits<double>::infinity();
    }

    if (grad_durations.size() != durations.size())
    {
      grad_durations = Eigen::VectorXd::Zero(durations.size());
    }
    traj_opt::convex_hull::reverseHodographGradients(order_gradients,
                                                     order_controls,
                                                     durations,
                                                     controls_per_piece,
                                                     grad_durations);
    hull_.backwardAdd(order_gradients[0], grad_coefficients, grad_durations);
    return residual.phr_cost;
  }

private:
  bool ready() const
  {
    return h_polys_ != nullptr && h_poly_idx_ != nullptr &&
           magnitude_bounds_.size() >= 3;
  }

  template <typename Trajectory>
  void refreshConstraintValues(const Trajectory &trajectory) const
  {
    const int segments = trajectory.getPieceNum();
    hull_.resetTopology(segments,
                        Trajectory::COEFF_NUM,
                        Basis::Bezier,
                        0,
                        0);
    trajectory.updateConvexHull(hull_);
    const Eigen::MatrixXd &position_controls = hull_.controls();
    const Eigen::VectorXd &durations = trajectory.getDurations();
    const int controls_per_piece = hull_.sourceDegree() + 1;
    std::array<Eigen::MatrixXd, 4> order_controls;
    traj_opt::convex_hull::buildHodographControls(position_controls,
                                                  durations,
                                                  controls_per_piece,
                                                  order_controls);
    std::array<Eigen::MatrixXd, 4> order_gradients;
    const auto residual = traj_opt::convex_hull::evaluatePackedResiduals(
        packed_,
        order_controls,
        durations,
        controls_per_piece,
        *h_polys_,
        *h_poly_idx_,
        magnitude_bounds_,
        options_.position_scale,
        multipliers_,
        penalty_,
        options_.flatness_enabled ? &options_.flatness : nullptr,
        order_gradients);
    constraint_values_ = residual.values;
    last_scalar_checks_ = residual.scalar_checks;
  }

  Options options_{};
  const general_utils::PolyhedraH *h_polys_{nullptr};
  const Eigen::VectorXi *h_poly_idx_{nullptr};
  general_utils::VecDf magnitude_bounds_;
  ExpConvexCostManager residual_manager_;

  mutable traj_opt::convex_hull::Representation<3> hull_;
  PackedConstraintSet packed_{};
  Eigen::VectorXd multipliers_;
  double penalty_{1.0};
  mutable Eigen::VectorXd constraint_values_;
  mutable general_utils::VecDf combined_penalty_log_;
  mutable std::size_t last_scalar_checks_{0};
  bool initialized_{false};
  UpdateReport last_report_{};
};

} // namespace cost_functional_manager
