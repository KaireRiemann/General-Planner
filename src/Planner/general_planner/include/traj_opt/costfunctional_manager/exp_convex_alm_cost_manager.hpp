#ifndef EXP_CONVEX_ALM_COST_MANAGER_HPP
#define EXP_CONVEX_ALM_COST_MANAGER_HPP

#include "traj_opt/costfunctional_manager/exp_convex_cost_manager.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace cost_functional_manager
{
/**
 * Adaptive Bezier-hull inequality constraints solved with an augmented
 * Lagrangian. The adaptive depth and constraint layout are frozen during each
 * inner LBFGS solve. updateAlmState() is the only operation allowed to change
 * them, and is called between inner solves.
 */
class ExpConvexAlmCostManager
{
public:
  using Basis = traj_opt::convex_hull::Basis;

  struct Options
  {
    bool adaptive{true};
    bool active_set{false};
    bool refine_derivative_constraints{false};
    int max_depth{2};
    double position_refine_margin{0.05};
    double derivative_refine_margin{0.05};
    double position_scale{0.25};
    double active_set_margin{0.05};
  };

  struct UpdateReport
  {
    bool initialized{false};
    bool topology_changed{false};
    bool topology_append_only{false};
    bool certified{false};
    double max_normalized_violation{0.0};
    double max_position_violation{0.0};
    double max_derivative_violation{0.0};
    double primal_residual{0.0};
    double dual_residual{0.0};
    double complementarity_residual{0.0};
    double stationarity_residual{0.0};
    double penalty{0.0};
    std::size_t constraints{0};
    int coarse_segments{0};
    int fine_segments{0};
  };

  void fillKktResiduals(UpdateReport &report,
                        double stationarity_residual = 0.0) const
  {
    report.primal_residual = std::max(0.0, report.max_normalized_violation);
    report.dual_residual =
        multipliers_.size() > 0
            ? std::max(0.0, (-multipliers_).maxCoeff())
            : 0.0;
    report.complementarity_residual = 0.0;
    if (multipliers_.size() > 0 &&
        constraint_values_.size() == multipliers_.size())
    {
      report.complementarity_residual =
          multipliers_.cwiseProduct(constraint_values_)
              .cwiseAbs()
              .maxCoeff();
    }
    report.stationarity_residual = std::max(0.0, stationarity_residual);
  }

  void configure(const Options &options)
  {
    options_ = options;
    options_.max_depth = std::clamp(options_.max_depth, 0, 8);
    options_.position_refine_margin =
        std::max(0.0, options_.position_refine_margin);
    options_.derivative_refine_margin =
        std::max(0.0, options_.derivative_refine_margin);
    options_.position_scale = std::max(1.0e-6, options_.position_scale);
    options_.active_set_margin = std::max(0.0, options_.active_set_margin);
    levels_[0].depth = 0;
    levels_[1].depth = options_.max_depth;
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
             bool guide_path_time_gradient_en = false)
  {
    h_polys_ = h_polys;
    h_poly_idx_ = h_poly_idx;
    magnitude_bounds_ = magnitude_bounds;
    penalty_weights_ = penalty_weights;

    general_utils::VecDf residual_weights = penalty_weights;
    const int polynomial_weight_count =
        std::min<int>(4, residual_weights.size());
    residual_weights.head(polynomial_weight_count).setZero();
    residual_manager_.configure(Basis::Bezier, 0, 2);
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
                            guide_path_time_gradient_en);

    initialized_ = false;
    selected_depths_.clear();
    constraints_.clear();
    full_constraints_.clear();
    multipliers_.resize(0);
    penalty_ = 1.0;
    last_report_ = UpdateReport{};
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
    hull_violation_.setZero();
  }

  template <typename Trajectory>
  bool initializeAlm(const Trajectory &trajectory)
  {
    if (!ready())
    {
      return false;
    }
    updateLevels(trajectory);
    const int segments = trajectory.getPieceNum();
    selected_depths_.assign(static_cast<std::size_t>(segments),
                            options_.adaptive ? 0 : options_.max_depth);
    if (options_.adaptive && options_.max_depth > 0)
    {
      for (int segment = 0; segment < segments; ++segment)
      {
        if (segmentNeedsFine(segment))
        {
          selected_depths_[static_cast<std::size_t>(segment)] =
              options_.max_depth;
        }
      }
    }
    rebuildConstraintLayout();
    penalty_ = 1.0;
    initialized_ = true;
    last_report_ = inspectCurrentConstraints();
    last_report_.initialized = true;
    return true;
  }

  template <typename Trajectory>
  UpdateReport updateAlmState(const Trajectory &trajectory)
  {
    if (!initialized_)
    {
      initializeAlm(trajectory);
      return last_report_;
    }

    updateSelectedLevels(trajectory);
    bool refined = false;
    if (options_.adaptive && options_.max_depth > 0)
    {
      for (int segment = 0;
           segment < static_cast<int>(selected_depths_.size());
           ++segment)
      {
        int &depth = selected_depths_[static_cast<std::size_t>(segment)];
        if (depth == 0 && segmentNeedsFine(segment))
        {
          depth = options_.max_depth;
          refined = true;
        }
      }
    }

    if (refined)
    {
      updateLevel(trajectory, levels_[1]);
      rebuildConstraintLayout();
      last_report_ = inspectCurrentConstraints();
      last_report_.initialized = true;
      last_report_.topology_changed = true;
      return last_report_;
    }

    if (appendViolatedInactiveConstraints())
    {
      last_report_ = inspectCurrentConstraints();
      last_report_.initialized = true;
      last_report_.topology_changed = true;
      last_report_.topology_append_only = true;
      return last_report_;
    }

    last_report_ = inspectCurrentConstraints();
    last_report_.initialized = true;
    return last_report_;
  }

  void setPhrState(const Eigen::VectorXd &multipliers,
                   double penalty)
  {
    if (multipliers.size() !=
        static_cast<Eigen::Index>(constraints_.size()) ||
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

  const UpdateReport &lastUpdateReport() const
  {
    return last_report_;
  }

  std::size_t activeControlPointChecksPerEvaluation() const
  {
    if (!initialized_ || !levels_[0].hull.kernel())
    {
      return 0;
    }
    std::size_t checks = 0;
    for (int segment = 0;
         segment < static_cast<int>(selected_depths_.size());
         ++segment)
    {
      const int level_index = levelIndexForSegment(segment);
      const auto &level = levels_[level_index];
      const int leaves = level.hull.piecesPerSegment();
      for (int order = 0; order <= 3; ++order)
      {
        if (constraintOrderActive(order))
        {
          const int cp = controlsPerPiece(level, order);
          checks += static_cast<std::size_t>(leaves * (cp - 1) + 1);
        }
      }
    }
    return checks;
  }

  std::size_t constraintCount() const
  {
    return constraints_.size();
  }

  std::size_t fullConstraintCount() const
  {
    return full_constraints_.size();
  }

  int coarseSegmentCount() const
  {
    return static_cast<int>(std::count(selected_depths_.begin(),
                                       selected_depths_.end(), 0));
  }

  int fineSegmentCount() const
  {
    return static_cast<int>(selected_depths_.size()) - coarseSegmentCount();
  }

  const general_utils::VecDf &getPenaltyLog() const
  {
    combined_penalty_log_ = residual_manager_.getPenaltyLog();
    if (combined_penalty_log_.size() < hull_violation_.size())
    {
      combined_penalty_log_.conservativeResize(hull_violation_.size());
    }
    for (int i = 1; i <= 4; ++i)
    {
      combined_penalty_log_(i) =
          std::max(combined_penalty_log_(i), hull_violation_(i));
    }
    return combined_penalty_log_;
  }

  double guideIntegralViolation() const
  {
    return residual_manager_.guideIntegralViolation();
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
    if (!initialized_ || !ready())
    {
      return 0.0;
    }
    updateSelectedLevels(trajectory);
    const Eigen::VectorXd &durations = trajectory.getDurations();
    double cost = 0.0;

    for (std::size_t i = 0; i < constraints_.size(); ++i)
    {
      const auto &constraint = constraints_[i];
      auto &level = levels_[constraint.level];
      const Eigen::Vector3d value =
          level.controls[constraint.order].row(constraint.row).transpose();
      double normalized_violation = 0.0;
      Eigen::Vector3d normalized_gradient = Eigen::Vector3d::Zero();
      if (constraint.order == 0)
      {
        const auto &poly = (*h_polys_)[constraint.poly];
        const Eigen::Vector3d normal =
            poly.template block<1, 3>(constraint.plane, 0).transpose();
        const double raw =
            normal.dot(value) + poly(constraint.plane, 3);
        normalized_violation = raw / options_.position_scale;
        normalized_gradient = normal / options_.position_scale;
        hull_violation_(1) = std::max(hull_violation_(1), raw);
      }
      else
      {
        const double bound = magnitude_bounds_(constraint.order - 1);
        const double squared_bound = bound * bound;
        const double raw = value.squaredNorm() - squared_bound;
        normalized_violation = raw / squared_bound;
        normalized_gradient = 2.0 * value / squared_bound;
        hull_violation_(constraint.order + 1) =
            std::max(hull_violation_(constraint.order + 1), raw);
      }

      const double multiplier =
          multipliers_(static_cast<Eigen::Index>(i));
      const double shifted =
          multiplier +
          penalty_ * normalized_violation;
      // Keep the standard PHR constant. It does not change the gradient, but
      // it prevents the multiplier offset from corrupting an inner solver's
      // relative-objective stopping test.
      cost -= 0.5 * multiplier * multiplier / penalty_;
      if (shifted > 0.0)
      {
        cost += 0.5 * shifted * shifted / penalty_;
        level.gradients[constraint.order].row(constraint.row) +=
            (shifted * normalized_gradient).transpose();
      }
    }

    for (int level_index = 0; level_index < 2; ++level_index)
    {
      auto &level = levels_[level_index];
      if (!level.hull.kernel() ||
          !levelSelected(level_index))
      {
        continue;
      }
      backwardHodographs(level, durations, grad_durations);
      level.hull.backwardAdd(level.gradients[0],
                             grad_coefficients,
                             grad_durations);
    }
    return cost;
  }

private:
  using Hull = traj_opt::convex_hull::Representation<3>;
  using HullMatrix = typename Hull::Matrix;

  struct LevelWorkspace
  {
    int depth{0};
    mutable Hull hull;
    mutable std::array<HullMatrix, 4> controls;
    mutable std::array<HullMatrix, 4> gradients;
  };

  struct Constraint
  {
    int level{0};
    int order{0};
    int segment{0};
    int row{0};
    int poly{-1};
    int plane{-1};
  };

  static bool sameConstraint(const Constraint &lhs,
                             const Constraint &rhs)
  {
    return lhs.level == rhs.level && lhs.order == rhs.order &&
           lhs.segment == rhs.segment && lhs.row == rhs.row &&
           lhs.poly == rhs.poly && lhs.plane == rhs.plane;
  }

  bool ready() const
  {
    return h_polys_ != nullptr && h_poly_idx_ != nullptr &&
           magnitude_bounds_.size() >= 3 && penalty_weights_.size() >= 4;
  }

  bool constraintOrderActive(int order) const
  {
    return order >= 0 && order < penalty_weights_.size() &&
           penalty_weights_(order) > 0.0;
  }

  int controlsPerPiece(const LevelWorkspace &level, int order) const
  {
    return level.hull.sourceDegree() + 1 - order;
  }

  int levelIndexForSegment(int segment) const
  {
    return selected_depths_[static_cast<std::size_t>(segment)] == 0 ? 0 : 1;
  }

  template <typename Trajectory>
  void updateLevels(const Trajectory &trajectory) const
  {
    updateLevel(trajectory, levels_[0]);
    if (options_.max_depth > 0)
    {
      updateLevel(trajectory, levels_[1]);
    }
  }

  template <typename Trajectory>
  void updateSelectedLevels(const Trajectory &trajectory) const
  {
    if (levelSelected(0))
    {
      updateLevel(trajectory, levels_[0]);
    }
    if (options_.max_depth > 0 && levelSelected(1))
    {
      updateLevel(trajectory, levels_[1]);
    }
  }

  bool levelSelected(int level_index) const
  {
    const int selected_depth =
        level_index == 0 ? 0 : options_.max_depth;
    return std::find(selected_depths_.begin(),
                     selected_depths_.end(),
                     selected_depth) != selected_depths_.end();
  }

  template <typename Trajectory>
  void updateLevel(const Trajectory &trajectory,
                   LevelWorkspace &level) const
  {
    const bool topology_changed =
        !level.hull.kernel() ||
        level.hull.numSourceSegments() != trajectory.getPieceNum() ||
        level.hull.sourceNumCoeffs() != Trajectory::COEFF_NUM ||
        level.hull.subdivisionDepth() != level.depth;
    if (topology_changed)
    {
      level.hull.resetTopology(trajectory.getPieceNum(),
                               Trajectory::COEFF_NUM,
                               Basis::Bezier,
                               0,
                               level.depth);
      const int pieces = level.hull.numPieces();
      for (int order = 0; order <= 3; ++order)
      {
        const int cp = controlsPerPiece(level, order);
        level.controls[order].resize(pieces * cp, 3);
        level.gradients[order].resize(pieces * cp, 3);
      }
    }

    trajectory.updateConvexHull(level.hull);
    level.controls[0] = level.hull.controls();
    const Eigen::VectorXd &durations = trajectory.getDurations();
    const int leaves = level.hull.piecesPerSegment();
    for (int order = 1; order <= 3; ++order)
    {
      const int previous_cp = controlsPerPiece(level, order - 1);
      const int current_cp = previous_cp - 1;
      const double degree_factor = static_cast<double>(previous_cp - 1);
      for (int piece = 0; piece < level.hull.numPieces(); ++piece)
      {
        const int segment = level.hull.pieceInfo(piece).source_segment;
        const double scale =
            degree_factor * static_cast<double>(leaves) /
            durations(segment);
        const int previous_row = piece * previous_cp;
        const int current_row = piece * current_cp;
        for (int control = 0; control < current_cp; ++control)
        {
          level.controls[order].row(current_row + control) =
              scale *
              (level.controls[order - 1].row(previous_row + control + 1) -
               level.controls[order - 1].row(previous_row + control));
        }
      }
    }
    for (auto &gradient : level.gradients)
    {
      gradient.setZero();
    }
  }

  void backwardHodographs(LevelWorkspace &level,
                          const Eigen::VectorXd &durations,
                          Eigen::VectorXd &duration_gradients) const
  {
    const int leaves = level.hull.piecesPerSegment();
    for (int order = 3; order >= 1; --order)
    {
      const int current_cp = controlsPerPiece(level, order);
      const int previous_cp = current_cp + 1;
      const double degree_factor = static_cast<double>(previous_cp - 1);
      for (int piece = 0; piece < level.hull.numPieces(); ++piece)
      {
        const int segment = level.hull.pieceInfo(piece).source_segment;
        const double duration = durations(segment);
        const double scale =
            degree_factor * static_cast<double>(leaves) / duration;
        const int current_row = piece * current_cp;
        const int previous_row = piece * previous_cp;
        duration_gradients(segment) -=
            level.gradients[order]
                .middleRows(current_row, current_cp)
                .cwiseProduct(level.controls[order]
                                  .middleRows(current_row, current_cp))
                .sum() /
            duration;
        for (int control = 0; control < current_cp; ++control)
        {
          const auto gradient =
              level.gradients[order].row(current_row + control);
          level.gradients[order - 1].row(previous_row + control) -=
              scale * gradient;
          level.gradients[order - 1].row(previous_row + control + 1) +=
              scale * gradient;
        }
      }
    }
  }

  bool segmentNeedsFine(int segment) const
  {
    if (options_.max_depth <= 0)
    {
      return false;
    }
    const auto &coarse = levels_[0];
    if (constraintOrderActive(0) && segment < h_poly_idx_->size())
    {
      const int poly_id = (*h_poly_idx_)(segment);
      if (poly_id >= 0 && poly_id < static_cast<int>(h_polys_->size()))
      {
        const auto &poly = (*h_polys_)[poly_id];
        const int cp = controlsPerPiece(coarse, 0);
        const int first_row = segment * cp;
        for (int control = 0; control < cp; ++control)
        {
          const Eigen::Vector3d value =
              coarse.controls[0].row(first_row + control).transpose();
          for (int plane = 0; plane < poly.rows(); ++plane)
          {
            const double raw =
                poly.template block<1, 3>(plane, 0).dot(value) +
                poly(plane, 3);
            if (raw > -options_.position_refine_margin)
            {
              return true;
            }
          }
        }
      }
    }

    const double derivative_refine_threshold =
        options_.refine_derivative_constraints
            ? -options_.derivative_refine_margin
            : 0.0;
    for (int order = 1; order <= 3; ++order)
    {
      if (!constraintOrderActive(order))
      {
        continue;
      }
      const double bound = magnitude_bounds_(order - 1);
      if (!std::isfinite(bound) || bound <= 0.0)
      {
        continue;
      }
      const int cp = controlsPerPiece(coarse, order);
      const int first_row = segment * cp;
      for (int control = 0; control < cp; ++control)
      {
        const double normalized =
            coarse.controls[order].row(first_row + control).squaredNorm() /
                (bound * bound) -
            1.0;
        if (normalized > derivative_refine_threshold)
        {
          return true;
        }
      }
    }
    return false;
  }

  void rebuildConstraintLayout()
  {
    full_constraints_.clear();
    for (int segment = 0;
         segment < static_cast<int>(selected_depths_.size());
         ++segment)
    {
      const int level_index = levelIndexForSegment(segment);
      const auto &level = levels_[level_index];
      const int leaves = level.hull.piecesPerSegment();
      for (int order = 0; order <= 3; ++order)
      {
        if (!constraintOrderActive(order))
        {
          continue;
        }
        const int cp = controlsPerPiece(level, order);
        for (int leaf = 0; leaf < leaves; ++leaf)
        {
          const int piece = segment * leaves + leaf;
          const int first_control = leaf == 0 ? 0 : 1;
          for (int control = first_control; control < cp; ++control)
          {
            const int row = piece * cp + control;
            if (order == 0)
            {
              if (segment >= h_poly_idx_->size())
              {
                continue;
              }
              const int poly_id = (*h_poly_idx_)(segment);
              if (poly_id < 0 ||
                  poly_id >= static_cast<int>(h_polys_->size()))
              {
                continue;
              }
              for (int plane = 0; plane < (*h_polys_)[poly_id].rows(); ++plane)
              {
                full_constraints_.push_back(
                    Constraint{level_index, order, segment, row,
                               poly_id, plane});
              }
            }
            else
            {
              full_constraints_.push_back(
                  Constraint{level_index, order, segment, row, -1, -1});
            }
          }
        }
      }
    }
    constraints_.clear();
    if (options_.active_set)
    {
      for (const auto &constraint : full_constraints_)
      {
        if (normalizedConstraintValue(constraint) >
            -options_.active_set_margin)
        {
          constraints_.push_back(constraint);
        }
      }
    }
    else
    {
      constraints_ = full_constraints_;
    }
    multipliers_.setZero(static_cast<Eigen::Index>(constraints_.size()));
    constraint_values_.setZero(static_cast<Eigen::Index>(constraints_.size()));
  }

  bool appendViolatedInactiveConstraints()
  {
    if (!options_.active_set)
    {
      return false;
    }
    bool appended = false;
    for (const auto &constraint : full_constraints_)
    {
      if (normalizedConstraintValue(constraint) <= 0.0)
      {
        continue;
      }
      const bool already_active =
          std::any_of(constraints_.begin(),
                      constraints_.end(),
                      [&constraint](const Constraint &active) {
                        return sameConstraint(active, constraint);
                      });
      if (!already_active)
      {
        constraints_.push_back(constraint);
        appended = true;
      }
    }
    if (appended)
    {
      multipliers_.conservativeResize(
          static_cast<Eigen::Index>(constraints_.size()));
      constraint_values_.conservativeResize(
          static_cast<Eigen::Index>(constraints_.size()));
    }
    return appended;
  }

  double normalizedConstraintValue(const Constraint &constraint,
                                   double *raw_value = nullptr) const
  {
    const auto &level = levels_[constraint.level];
    const Eigen::Vector3d value =
        level.controls[constraint.order].row(constraint.row).transpose();
    if (constraint.order == 0)
    {
      const auto &poly = (*h_polys_)[constraint.poly];
      const double raw =
          poly.template block<1, 3>(constraint.plane, 0).dot(value) +
          poly(constraint.plane, 3);
      if (raw_value != nullptr)
      {
        *raw_value = raw;
      }
      return raw / options_.position_scale;
    }
    const double bound = magnitude_bounds_(constraint.order - 1);
    const double raw = value.squaredNorm() - bound * bound;
    if (raw_value != nullptr)
    {
      *raw_value = raw;
    }
    return raw / (bound * bound);
  }

  UpdateReport inspectCurrentConstraints()
  {
    UpdateReport report;
    report.initialized = initialized_;
    report.penalty = penalty_;
    report.constraints = constraints_.size();
    report.coarse_segments = coarseSegmentCount();
    report.fine_segments = fineSegmentCount();
    for (std::size_t i = 0; i < constraints_.size(); ++i)
    {
      constraint_values_(static_cast<Eigen::Index>(i)) =
          normalizedConstraintValue(constraints_[i]);
    }
    for (const auto &constraint : full_constraints_)
    {
      double raw = 0.0;
      const double normalized =
          normalizedConstraintValue(constraint, &raw);
      report.max_normalized_violation =
          std::max(report.max_normalized_violation, normalized);
      if (constraint.order == 0)
      {
        report.max_position_violation =
            std::max(report.max_position_violation, raw);
      }
      else
      {
        report.max_derivative_violation =
            std::max(report.max_derivative_violation, raw);
      }
    }
    report.max_normalized_violation =
        std::max(0.0, report.max_normalized_violation);
    report.max_position_violation =
        std::max(0.0, report.max_position_violation);
    report.max_derivative_violation =
        std::max(0.0, report.max_derivative_violation);
    report.certified = report.max_normalized_violation <= 0.0;
    fillKktResiduals(report);
    return report;
  }

  ExpConvexCostManager residual_manager_;
  const general_utils::PolyhedraH *h_polys_{nullptr};
  const Eigen::VectorXi *h_poly_idx_{nullptr};
  general_utils::VecDf magnitude_bounds_;
  general_utils::VecDf penalty_weights_;
  Options options_;
  mutable std::array<LevelWorkspace, 2> levels_;
  std::vector<int> selected_depths_;
  std::vector<Constraint> full_constraints_;
  std::vector<Constraint> constraints_;
  Eigen::VectorXd multipliers_;
  mutable Eigen::VectorXd constraint_values_;
  double penalty_{1.0};
  bool initialized_{false};
  UpdateReport last_report_;
  mutable general_utils::VecDf hull_violation_{
      general_utils::VecDf::Zero(9)};
  mutable general_utils::VecDf combined_penalty_log_{
      general_utils::VecDf::Zero(9)};
};
} // namespace cost_functional_manager

#endif
