#pragma once

#include "traj_opt/minco/minco_tangent.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace minco
{

/** The search geometries supported by the reduced MINCO manifold. */
enum class MincoMetricMode
{
  kDisabled = 0,
  kFrozenWaypoint = 1,
  kDynamicWaypoint = 2,
  kBlockSpaceTime = 3,
  kFullSpaceTimeGaussNewton = 4
};

struct MincoMetricOptions
{
  MincoMetricMode mode{MincoMetricMode::kDisabled};

  // A positive shift is necessary only for numerical robustness or for a
  // deliberately rank-deficient coordinate chart.  Zero preserves the exact
  // fixed-time control metric when that metric is already SPD.
  double regularization{1.0e-10};
  double regularization_scale_floor{1.0};

  // Weight of sum_i (dT_i / T_i)^2 in the block/GN space-time metrics.
  double time_metric_weight{1.0};

  // Objective energy weight.  Production V1 freezes
  //   G_0 = rho_E * G_MCE(T_0)
  // with G_MCE = 2 J_P^T Q J_P matching getEnergy() (no 1/2).
  double energy_weight{1.0};
};

/**
 * Pullback metric of the minimum-control trajectory manifold.
 *
 * This class deliberately operates in physical reduced coordinates:
 *
 *   waypoint  : vec(P_inner)
 *   space-time: [T, vec(P_inner)]
 *
 * Constraint maps and time maps are charts around this object and are lifted
 * by the caller.  This keeps the metric independent of a particular corridor
 * parameterization and makes the fixed-time construction directly testable.
 */
template <int DIM, int S>
class MincoMetric
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Trajectory = MINCOTrajectory<DIM, S>;
  using InnerPointsMat = typename Trajectory::InnerPointsMat;
  using CoeffMat = typename Trajectory::CoeffMat;
  using VectorD = typename Trajectory::VectorD;

  void setOptions(const MincoMetricOptions &options)
  {
    options_ = options;
  }

  const MincoMetricOptions &options() const { return options_; }

  void clear()
  {
    waypoint_metric_.resize(0, 0);
    scalar_waypoint_metric_.resize(0, 0);
    space_time_metric_.resize(0, 0);
    selected_metric_.resize(0, 0);
    selected_ldlt_ = Eigen::LDLT<Eigen::MatrixXd>{};
    ready_ = false;
    has_kronecker_structure_ = false;
    last_condition_number_ = std::numeric_limits<double>::infinity();
  }

  int waypointDim() const { return waypoint_dim_; }
  int spaceTimeDim() const { return space_time_dim_; }
  bool ready() const { return ready_; }
  bool isSpaceTimeMetric() const { return selected_is_space_time_; }
  bool hasKroneckerStructure() const { return has_kronecker_structure_; }
  double conditionNumber() const { return last_condition_number_; }
  int timeDim() const
  {
    return selected_is_space_time_ ? std::max(0, space_time_dim_ - waypoint_dim_)
                                   : 0;
  }

  Eigen::MatrixXd Gtt() const
  {
    const int m = timeDim();
    if (m <= 0 || space_time_metric_.rows() < m)
    {
      return Eigen::MatrixXd{};
    }
    return space_time_metric_.topLeftCorner(m, m);
  }

  Eigen::MatrixXd Gtp() const
  {
    const int m = timeDim();
    if (m <= 0 || waypoint_dim_ <= 0 ||
        space_time_metric_.rows() < m + waypoint_dim_)
    {
      return Eigen::MatrixXd{};
    }
    return space_time_metric_.topRightCorner(m, waypoint_dim_);
  }

  Eigen::MatrixXd Gpp() const
  {
    if (selected_is_space_time_)
    {
      if (waypoint_dim_ <= 0 ||
          space_time_metric_.rows() < timeDim() + waypoint_dim_)
      {
        return Eigen::MatrixXd{};
      }
      return space_time_metric_.bottomRightCorner(waypoint_dim_, waypoint_dim_);
    }
    return waypoint_metric_;
  }

  double couplingEta() const
  {
    const Eigen::MatrixXd tt = Gtt();
    const Eigen::MatrixXd tp = Gtp();
    const Eigen::MatrixXd pp = Gpp();
    if (tt.size() == 0 || tp.size() == 0 || pp.size() == 0)
    {
      return 0.0;
    }
    const double den = std::sqrt(tt.norm() * pp.norm());
    return den > 0.0 ? tp.norm() / den : 0.0;
  }

  const Eigen::MatrixXd &waypointMetric() const { return waypoint_metric_; }
  const Eigen::MatrixXd &scalarWaypointMetric() const
  {
    return scalar_waypoint_metric_;
  }
  const Eigen::MatrixXd &spaceTimeMetric() const { return space_time_metric_; }
  const Eigen::MatrixXd &selectedMetric() const { return selected_metric_; }

  /** Rebuild the metric represented by options(). */
  bool update(const Trajectory &trajectory)
  {
    switch (options_.mode)
    {
    case MincoMetricMode::kFrozenWaypoint:
    case MincoMetricMode::kDynamicWaypoint:
      return buildWaypointMetric(trajectory);
    case MincoMetricMode::kBlockSpaceTime:
      return buildBlockSpaceTimeMetric(trajectory);
    case MincoMetricMode::kFullSpaceTimeGaussNewton:
      return buildFullSpaceTimeGaussNewtonMetric(trajectory);
    case MincoMetricMode::kDisabled:
    default:
      clear();
      return false;
    }
  }

  /**
   * Exact fixed-time MCE metric matching getEnergy() (no 1/2):
   *
   *     G_MCE = 2 J_P^T Q J_P = H_{E,reduced},
   *     G_0   = rho_E G_MCE.
   *
   * For position MINCO this is the Kronecker product G_scalar ⊗ I_DIM.
   */
  bool buildWaypointMetric(const Trajectory &trajectory)
  {
    Eigen::MatrixXd raw;
    if (!computeWaypointMetric(trajectory, raw))
    {
      clear();
      return false;
    }
    waypoint_metric_ = std::move(raw);
    selected_is_space_time_ = false;
    return finalizeSelectedMetric(waypoint_metric_);
  }

  /** Block-diagonal physical metric with relative segment-time coordinates. */
  bool buildBlockSpaceTimeMetric(const Trajectory &trajectory)
  {
    Eigen::MatrixXd waypoint_raw;
    if (!computeWaypointMetric(trajectory, waypoint_raw))
    {
      clear();
      return false;
    }

    waypoint_metric_ = waypoint_raw;
    const int pieces = trajectory.getPieceNum();
    space_time_dim_ = pieces + waypoint_dim_;
    space_time_metric_ = Eigen::MatrixXd::Zero(space_time_dim_, space_time_dim_);
    const auto &durations = trajectory.getDurations();
    for (int i = 0; i < pieces; ++i)
    {
      const double T = durations(i);
      space_time_metric_(i, i) =
          std::max(0.0, options_.time_metric_weight) /
          std::max(1.0e-12, T * T);
    }
    if (waypoint_dim_ > 0)
    {
      space_time_metric_.bottomRightCorner(waypoint_dim_, waypoint_dim_) =
          waypoint_raw;
    }
    selected_is_space_time_ = true;
    return finalizeSelectedMetric(space_time_metric_);
  }

  /**
   * Space-time Gauss-Newton control-residual metric.  For S=4, four
   * Gauss-Legendre samples integrate the squared normalized snap exactly.
   */
  bool buildFullSpaceTimeGaussNewtonMetric(const Trajectory &trajectory)
  {
    const int pieces = trajectory.getPieceNum();
    if (pieces <= 0 || !trajectory.getDurations().allFinite())
    {
      clear();
      return false;
    }

    waypoint_dim_ = DIM * std::max(0, pieces - 1);
    space_time_dim_ = pieces + waypoint_dim_;
    std::vector<CoeffMat> coefficient_tangents;
    std::vector<Eigen::VectorXd> time_tangents;
    if (!buildSpaceTimeTangents(trajectory,
                                coefficient_tangents,
                                time_tangents))
    {
      clear();
      return false;
    }

    constexpr std::array<double, 4> kNodes{
        0.06943184420297371,
        0.33000947820757187,
        0.66999052179242813,
        0.93056815579702623};
    constexpr std::array<double, 4> kWeights{
        0.17392742256872693,
        0.32607257743127307,
        0.32607257743127307,
        0.17392742256872693};

    using ResidualMat = Eigen::Matrix<double, Eigen::Dynamic, DIM>;
    std::vector<ResidualMat> residual_tangents(
        static_cast<std::size_t>(space_time_dim_));
    const int residual_rows = pieces * static_cast<int>(kNodes.size());
    for (auto &residual : residual_tangents)
    {
      residual.setZero(residual_rows, DIM);
    }

    const auto &coefficients = trajectory.getCoefficients();
    const auto &durations = trajectory.getDurations();
    for (int i = 0; i < pieces; ++i)
    {
      const double T = durations(i);
      if (!std::isfinite(T) || T <= 0.0)
      {
        clear();
        return false;
      }
      const double sqrt_T = std::sqrt(T);
      const auto coeff_block = coefficients.template block<Trajectory::COEFF_NUM, DIM>(
          i * Trajectory::COEFF_NUM, 0);
      for (int k = 0; k < static_cast<int>(kNodes.size()); ++k)
      {
        const double u = kNodes[static_cast<std::size_t>(k)];
        const double t = u * T;
        const auto b4 = Trajectory::derivativeBasis(S, t);
        const auto b5 = Trajectory::derivativeBasis(S + 1, t);
        const VectorD p_control = coeff_block.transpose() * b4.transpose();
        const VectorD p_control_next =
            coeff_block.transpose() * b5.transpose();
        const double weighted_sqrt =
            std::sqrt(kWeights[static_cast<std::size_t>(k)]);
        const int row = i * static_cast<int>(kNodes.size()) + k;

        for (int a = 0; a < space_time_dim_; ++a)
        {
          const auto delta_block =
              coefficient_tangents[static_cast<std::size_t>(a)]
                  .template block<Trajectory::COEFF_NUM, DIM>(
                      i * Trajectory::COEFF_NUM, 0);
          const VectorD delta_control =
              delta_block.transpose() * b4.transpose();
          const double dT = time_tangents[static_cast<std::size_t>(a)](i);
          const VectorD delta_residual =
              sqrt_T * delta_control +
              dT * (0.5 / sqrt_T * p_control +
                    sqrt_T * u * p_control_next);
          residual_tangents[static_cast<std::size_t>(a)].row(row) =
              (weighted_sqrt * delta_residual).transpose();
        }
      }
    }

    space_time_metric_ = Eigen::MatrixXd::Zero(space_time_dim_, space_time_dim_);
    for (int a = 0; a < space_time_dim_; ++a)
    {
      for (int b = a; b < space_time_dim_; ++b)
      {
        const double value =
            (residual_tangents[static_cast<std::size_t>(a)].array() *
             residual_tangents[static_cast<std::size_t>(b)].array())
                .sum();
        space_time_metric_(a, b) = value;
        space_time_metric_(b, a) = value;
      }
    }

    // Residual r already includes sqrt(T) p^{(4)} and Gauss weights, so
    // ||r||^2 ≈ E_snap (no 1/2).  Matching getEnergy() therefore requires
    //   G_ctrl = 2 ρ_E J_r^T J_r
    // before the relative-time term is added.
    const double control_scale = 2.0 * std::max(0.0, options_.energy_weight);
    space_time_metric_ *= control_scale;

    for (int i = 0; i < pieces; ++i)
    {
      const double T = durations(i);
      space_time_metric_(i, i) +=
          std::max(0.0, options_.time_metric_weight) /
          std::max(1.0e-12, T * T);
    }

    selected_is_space_time_ = true;
    return finalizeSelectedMetric(space_time_metric_);
  }

  bool applyWaypointMetric(const Eigen::VectorXd &v, Eigen::VectorXd &Gv) const
  {
    if (waypoint_metric_.rows() == 0 || v.size() != waypoint_metric_.cols())
    {
      return false;
    }
    Gv.noalias() = waypoint_metric_ * v;
    return Gv.allFinite();
  }

  bool solveWaypointMetric(const Eigen::VectorXd &g, Eigen::VectorXd &d) const
  {
    if (waypoint_metric_.rows() == 0 || g.size() != waypoint_metric_.rows())
    {
      return false;
    }
    Eigen::LDLT<Eigen::MatrixXd> ldlt(waypoint_metric_);
    if (ldlt.info() != Eigen::Success)
    {
      return false;
    }
    d = ldlt.solve(g);
    return d.allFinite();
  }

  bool applySpaceTimeMetric(const Eigen::VectorXd &v, Eigen::VectorXd &Gv) const
  {
    if (space_time_metric_.rows() == 0 || v.size() != space_time_metric_.cols())
    {
      return false;
    }
    Gv.noalias() = space_time_metric_ * v;
    return Gv.allFinite();
  }

  bool solveSpaceTimeMetric(const Eigen::VectorXd &g, Eigen::VectorXd &d) const
  {
    if (space_time_metric_.rows() == 0 || g.size() != space_time_metric_.rows())
    {
      return false;
    }
    Eigen::LDLT<Eigen::MatrixXd> ldlt(space_time_metric_);
    if (ldlt.info() != Eigen::Success)
    {
      return false;
    }
    d = ldlt.solve(g);
    return d.allFinite();
  }

  /** Solve the metric selected by update(). */
  bool solve(const Eigen::VectorXd &g, Eigen::VectorXd &d) const
  {
    if (!ready_ || g.size() != selected_metric_.rows())
    {
      return false;
    }
    d = selected_ldlt_.solve(g);
    return selected_ldlt_.info() == Eigen::Success && d.allFinite();
  }

  /**
   * Restore a previously factorized waypoint metric without applying
   * regularization a second time.  Used by the cross-replan T-cache.
   */
  bool adoptReadyWaypointMetric(const Eigen::MatrixXd &metric,
                                const Eigen::MatrixXd &scalar_metric,
                                bool kronecker)
  {
    if (metric.rows() == 0 || metric.rows() != metric.cols() ||
        !metric.allFinite())
    {
      clear();
      return false;
    }
    const double saved_regularization = options_.regularization;
    options_.regularization = 0.0;
    waypoint_dim_ = static_cast<int>(metric.rows());
    has_kronecker_structure_ = kronecker &&
                               scalar_metric.rows() * DIM == metric.rows();
    scalar_waypoint_metric_ =
        has_kronecker_structure_ ? scalar_metric : Eigen::MatrixXd{};
    selected_is_space_time_ = false;
    const bool ok = finalizeSelectedMetric(metric);
    options_.regularization = saved_regularization;
    return ok;
  }

private:
  bool computeWaypointMetric(const Trajectory &trajectory,
                             Eigen::MatrixXd &metric)
  {
    Eigen::MatrixXd scalar;
    if (!computeScalarWaypointMetric(trajectory, scalar))
    {
      return false;
    }

    const double energy_weight = std::max(0.0, options_.energy_weight);
    scalar_waypoint_metric_ = (2.0 * energy_weight) * scalar;
    has_kronecker_structure_ = waypoint_dim_ > 0;
    if (waypoint_dim_ == 0)
    {
      metric.resize(0, 0);
      return true;
    }
    metric = expandKronecker(scalar_waypoint_metric_);
    return metric.allFinite();
  }

  bool computeScalarWaypointMetric(const Trajectory &trajectory,
                                   Eigen::MatrixXd &scalar)
  {
    const int pieces = trajectory.getPieceNum();
    if (pieces <= 0 || !trajectory.getDurations().allFinite())
    {
      return false;
    }

    const int inner = std::max(0, pieces - 1);
    waypoint_dim_ = DIM * inner;
    space_time_dim_ = pieces + waypoint_dim_;
    scalar = Eigen::MatrixXd::Zero(inner, inner);
    if (inner == 0)
    {
      return true;
    }

    std::vector<CoeffMat> coefficient_tangents;
    coefficient_tangents.reserve(static_cast<std::size_t>(inner));
    for (int i = 0; i < inner; ++i)
    {
      InnerPointsMat delta_points = InnerPointsMat::Zero(DIM, inner);
      delta_points(0, i) = 1.0;
      CoeffMat delta_coefficients;
      if (!trajectory.propagateTangentByPoints(delta_points,
                                                delta_coefficients))
      {
        return false;
      }
      coefficient_tangents.emplace_back(std::move(delta_coefficients));
    }

    for (int i = 0; i < pieces; ++i)
    {
      const auto Q = Trajectory::Traits::controlCostHessian(
          trajectory.getDurations()(i));
      for (int a = 0; a < inner; ++a)
      {
        const auto tangent_a =
            coefficient_tangents[static_cast<std::size_t>(a)]
                .template block<Trajectory::COEFF_NUM, DIM>(
                    i * Trajectory::COEFF_NUM, 0);
        for (int b = a; b < inner; ++b)
        {
          const auto tangent_b =
              coefficient_tangents[static_cast<std::size_t>(b)]
                  .template block<Trajectory::COEFF_NUM, DIM>(
                      i * Trajectory::COEFF_NUM, 0);
          const double value = (tangent_a.transpose() * Q * tangent_b).trace();
          scalar(a, b) += value;
          if (a != b)
          {
            scalar(b, a) += value;
          }
        }
      }
    }
    return scalar.allFinite();
  }

  Eigen::MatrixXd expandKronecker(const Eigen::MatrixXd &scalar) const
  {
    const int inner = static_cast<int>(scalar.rows());
    Eigen::MatrixXd metric = Eigen::MatrixXd::Zero(DIM * inner, DIM * inner);
    for (int a = 0; a < inner; ++a)
    {
      for (int b = 0; b < inner; ++b)
      {
        metric.template block<DIM, DIM>(DIM * a, DIM * b) =
            scalar(a, b) * Eigen::Matrix<double, DIM, DIM>::Identity();
      }
    }
    return metric;
  }

  bool buildSpaceTimeTangents(const Trajectory &trajectory,
                              std::vector<CoeffMat> &coefficient_tangents,
                              std::vector<Eigen::VectorXd> &time_tangents) const
  {
    const int pieces = trajectory.getPieceNum();
    const int waypoint_dim = DIM * std::max(0, pieces - 1);
    const int dimension = pieces + waypoint_dim;
    coefficient_tangents.clear();
    time_tangents.clear();
    coefficient_tangents.reserve(static_cast<std::size_t>(dimension));
    time_tangents.reserve(static_cast<std::size_t>(dimension));

    for (int a = 0; a < dimension; ++a)
    {
      InnerPointsMat delta_points =
          InnerPointsMat::Zero(DIM, std::max(0, pieces - 1));
      Eigen::VectorXd delta_times = Eigen::VectorXd::Zero(pieces);
      if (a < pieces)
      {
        delta_times(a) = 1.0;
      }
      else
      {
        const int waypoint_coordinate = a - pieces;
        delta_points(waypoint_coordinate % DIM,
                     waypoint_coordinate / DIM) = 1.0;
      }
      CoeffMat delta_coefficients;
      if (!trajectory.propagateTangent(delta_points,
                                       delta_times,
                                       delta_coefficients))
      {
        return false;
      }
      coefficient_tangents.emplace_back(std::move(delta_coefficients));
      time_tangents.emplace_back(std::move(delta_times));
    }
    return true;
  }

  bool finalizeSelectedMetric(const Eigen::MatrixXd &metric)
  {
    if (metric.rows() == 0 || metric.rows() != metric.cols() ||
        !metric.allFinite())
    {
      clear();
      return false;
    }

    selected_metric_ = 0.5 * (metric + metric.transpose());
    if (options_.regularization > 0.0)
    {
      const double mean_diagonal = selected_metric_.trace() /
                                   static_cast<double>(selected_metric_.rows());
      const double shift = options_.regularization *
                           std::max(options_.regularization_scale_floor,
                                    std::abs(mean_diagonal));
      if (!selected_is_space_time_ && has_kronecker_structure_ &&
          scalar_waypoint_metric_.rows() > 0)
      {
        scalar_waypoint_metric_ =
            0.5 * (scalar_waypoint_metric_ + scalar_waypoint_metric_.transpose());
        scalar_waypoint_metric_.diagonal().array() += shift;
        selected_metric_ = expandKronecker(scalar_waypoint_metric_);
      }
      else
      {
        selected_metric_.diagonal().array() += shift;
        has_kronecker_structure_ = false;
      }
    }

    // Keep public metric matrices equal to the factorized operator.
    if (selected_is_space_time_)
    {
      space_time_metric_ = selected_metric_;
      has_kronecker_structure_ = false;
    }
    else
    {
      waypoint_metric_ = selected_metric_;
    }

    selected_ldlt_.compute(selected_metric_);
    if (selected_ldlt_.info() != Eigen::Success ||
        !selected_ldlt_.isPositive())
    {
      clear();
      return false;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(selected_metric_);
    if (eigen_solver.info() == Eigen::Success)
    {
      const auto eigenvalues = eigen_solver.eigenvalues();
      const double min_eigenvalue = eigenvalues.minCoeff();
      const double max_eigenvalue = eigenvalues.maxCoeff();
      last_condition_number_ = min_eigenvalue > 0.0
                                   ? max_eigenvalue / min_eigenvalue
                                   : std::numeric_limits<double>::infinity();
    }
    else
    {
      last_condition_number_ = std::numeric_limits<double>::infinity();
    }

    ready_ = std::isfinite(last_condition_number_) &&
             selected_metric_.allFinite();
    return ready_;
  }

private:
  MincoMetricOptions options_;
  int waypoint_dim_{0};
  int space_time_dim_{0};
  bool ready_{false};
  bool selected_is_space_time_{false};
  bool has_kronecker_structure_{false};
  double last_condition_number_{std::numeric_limits<double>::infinity()};

  Eigen::MatrixXd waypoint_metric_;
  Eigen::MatrixXd scalar_waypoint_metric_;
  Eigen::MatrixXd space_time_metric_;
  Eigen::MatrixXd selected_metric_;
  Eigen::LDLT<Eigen::MatrixXd> selected_ldlt_;
};

} // namespace minco
