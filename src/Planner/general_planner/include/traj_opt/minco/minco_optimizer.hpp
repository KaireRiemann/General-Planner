#ifndef MINCO_OPTIMIZER_HPP
#define MINCO_OPTIMIZER_HPP

#include "traj_opt/minco/boundary_mapping.hpp"
#include "traj_opt/minco/minco_trajectory.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

namespace minco
{

namespace optimizer_traits
{
  template <typename...>
  using void_t = void;

  template <typename T, typename = void>
  struct HasTimeMapInterface : std::false_type
  {
  };

  template <typename T>
  struct HasTimeMapInterface<T, void_t<
                                   decltype(static_cast<double>(std::declval<T>().toTime(std::declval<double>()))),
                                   decltype(static_cast<double>(std::declval<T>().toTau(std::declval<double>()))),
                                   decltype(static_cast<double>(std::declval<T>().backward(std::declval<double>(), std::declval<double>(), std::declval<double>())))>> : std::true_type
  {
  };

  template <typename T, int DIM, typename = void>
  struct HasSpatialMapInterface : std::false_type
  {
  };

  template <typename T, int DIM>
  struct HasSpatialMapInterface<T, DIM, void_t<
                                           decltype(static_cast<int>(std::declval<T>().getUnconstrainedDim(std::declval<int>()))),
                                           decltype(std::declval<T>().toPhysical(std::declval<Eigen::VectorXd>(), std::declval<int>())),
                                           decltype(std::declval<T>().toUnconstrained(std::declval<Eigen::Matrix<double, DIM, 1>>(), std::declval<int>())),
                                           decltype(std::declval<T>().backwardGrad(std::declval<Eigen::VectorXd>(),
                                                                                   std::declval<Eigen::Matrix<double, DIM, 1>>(),
                                                                                   std::declval<int>())),
                                           decltype(std::declval<T>().addNormPenalty(std::declval<Eigen::VectorXd>(), std::declval<double &>(), std::declval<Eigen::VectorXd &>()))>> : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasTimeCostInterface : std::false_type
  {
  };

  template <typename T>
  struct HasTimeCostInterface<T, void_t<
                                     decltype(static_cast<double>(std::declval<T>()(
                                         std::declval<const std::vector<double> &>(),
                                         std::declval<Eigen::VectorXd &>())))>> : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasDiscreteSampleTimes : std::false_type
  {
  };

  template <typename T>
  struct HasDiscreteSampleTimes<T, void_t<
                                       decltype(std::declval<const T>().discreteSampleTimes())>> : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasBeginEvaluationWithTimes : std::false_type
  {
  };

  template <typename T>
  struct HasBeginEvaluationWithTimes<T, void_t<
                                            decltype(std::declval<T>().beginEvaluation(
                                                std::declval<const std::vector<double> *>()))>> : std::true_type
  {
  };

  template <typename T, typename = void>
  struct HasBeginEvaluationWithoutTimes : std::false_type
  {
  };

  template <typename T>
  struct HasBeginEvaluationWithoutTimes<T, void_t<
                                               decltype(std::declval<T>().beginEvaluation())>> : std::true_type
  {
  };

  /**
   * Optional dense-sampling gate. A cost manager returning false guarantees
   * that both its integral and sample callbacks are inactive for the current
   * configuration, allowing MINCO to avoid constructing any trajectory
   * samples or basis rows.
   */
  template <typename T, typename = void>
  struct HasDenseSamplingQuery : std::false_type
  {
  };

  template <typename T>
  struct HasDenseSamplingQuery<
      T,
      void_t<decltype(static_cast<bool>(
          std::declval<const T &>().usesDenseSampling()))>> : std::true_type
  {
  };

  /**
   * Optional post-integral sample-cost gate. This is intentionally separate
   * from usesDenseSampling(): a manager may need quadrature while having no
   * discrete evaluateSample() term. Returning false lets MINCO avoid retaining
   * samples and performing an all-zero sample-gradient backpropagation.
   */
  template <typename T, typename = void>
  struct HasSampleCostQuery : std::false_type
  {
  };

  template <typename T>
  struct HasSampleCostQuery<
      T,
      void_t<decltype(static_cast<bool>(
          std::declval<const T &>().usesSampleCost()))>> : std::true_type
  {
  };

  /**
   * Optional direct coefficient/time cost interface:
   *
   * double evaluateCoefficient(const Trajectory &trajectory,
   *                            Coefficients &grad_coefficients,
   *                            Eigen::VectorXd &grad_durations) const;
   *
   * The callback uses additive partial gradients and runs before the single
   * MINCO propagateGradFull() call.
   */
  template <typename T,
            typename Trajectory,
            typename Coefficients,
            typename = void>
  struct HasCoefficientCostInterface : std::false_type
  {
  };

  template <typename T, typename Trajectory, typename Coefficients>
  struct HasCoefficientCostInterface<
      T,
      Trajectory,
      Coefficients,
      void_t<decltype(static_cast<double>(
          std::declval<const T &>().evaluateCoefficient(
              std::declval<const Trajectory &>(),
              std::declval<Coefficients &>(),
              std::declval<Eigen::VectorXd &>())))>> : std::true_type
  {
  };
} // namespace optimizer_traits

template <int DIM, int S, typename TimeMap, typename SpatialMap>
class MINCOOptimizer
{
  static_assert(optimizer_traits::HasTimeMapInterface<TimeMap>::value,
                "TimeMap does not satisfy the required interface.");
  static_assert(optimizer_traits::HasSpatialMapInterface<SpatialMap, DIM>::value,
                "SpatialMap does not satisfy the required interface.");

public:
  using TrajType = MINCOTrajectory<DIM, S>;
  using VectorType = Eigen::Matrix<double, DIM, 1>;
  using BoundaryState = typename TrajType::BoundaryState;
  using WaypointsType = Eigen::Matrix<double, Eigen::Dynamic, DIM>;

  struct TimingStatistics
  {
    std::size_t evaluations{0};
    double evaluation_seconds{0.0};
    double dense_integral_seconds{0.0};
    double coefficient_seconds{0.0};

    double denseIntegralShareOfEvaluation() const
    {
      return evaluation_seconds > 0.0
                 ? dense_integral_seconds / evaluation_seconds
                 : 0.0;
    }

    double coefficientShareOfEvaluation() const
    {
      return evaluation_seconds > 0.0
                 ? coefficient_seconds / evaluation_seconds
                 : 0.0;
    }
  };
  using InnerPointsMat = Eigen::Matrix<double, DIM, Eigen::Dynamic>;
  using CoeffMat = Eigen::Matrix<double, Eigen::Dynamic, DIM>;

  struct Sample
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int seg_idx{0};
    int step_in_seg{0};
    int logical_idx{0};
    double alpha{0.0};
    double t_local{0.0};
    double t_global{0.0};
    double trap_weight{0.0};
    double dt{0.0};
    typename TrajType::BasisRow b_p;
    VectorType p{VectorType::Zero()};
    VectorType v{VectorType::Zero()};
  };

  using SampleBuffer = std::vector<Sample, Eigen::aligned_allocator<Sample>>;

  struct Workspace
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::VectorXd cache_T;
    InnerPointsMat cache_P_inner;
    BoundaryState head_state;
    BoundaryState tail_state;

    CoeffMat gdC_energy;
    Eigen::VectorXd gdT_energy;

    CoeffMat gdC_integral;
    Eigen::VectorXd gdT_integral;

    CoeffMat gdC_sample;
    Eigen::VectorXd gdT_sample;

    CoeffMat gdC_coefficient;
    Eigen::VectorXd gdT_coefficient;

    Eigen::VectorXd gdT_time;

    InnerPointsMat grad_by_points;
    Eigen::VectorXd grad_by_times;
    BoundaryState grad_by_head_state;
    BoundaryState grad_by_tail_state;

    SampleBuffer samples;
    SampleBuffer discrete_samples;
    Eigen::Matrix<double, DIM, Eigen::Dynamic> sample_grad_p;
    Eigen::VectorXd sample_grad_t_global;
    Eigen::VectorXd global_time_grad;

    void resize(int piece_num)
    {
      cache_T.resize(piece_num);
      cache_P_inner.resize(DIM, std::max(0, piece_num - 1));
      gdC_energy.resize(piece_num * TrajType::COEFF_NUM, DIM);
      gdT_energy.resize(piece_num);
      gdC_integral.resize(piece_num * TrajType::COEFF_NUM, DIM);
      gdT_integral.resize(piece_num);
      gdC_sample.resize(piece_num * TrajType::COEFF_NUM, DIM);
      gdT_sample.resize(piece_num);
      gdC_coefficient.resize(piece_num * TrajType::COEFF_NUM, DIM);
      gdT_coefficient.resize(piece_num);
      gdT_time.resize(piece_num);
      grad_by_points.resize(DIM, std::max(0, piece_num - 1));
      grad_by_times.resize(piece_num);
      global_time_grad.resize(piece_num);
      grad_by_head_state.setZero();
      grad_by_tail_state.setZero();
    }
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MINCOOptimizer()
      : active_time_map_(&default_time_map_),
        active_spatial_map_(&default_spatial_map_)
  {
    workspace_ = std::make_unique<Workspace>();
    setSamplesPerPiece(samples_per_piece_);
  }

  void setTimeMap(const TimeMap *time_map)
  {
    active_time_map_ = time_map != nullptr ? time_map : &default_time_map_;
  }

  void setSpatialMap(const SpatialMap *spatial_map)
  {
    active_spatial_map_ = spatial_map != nullptr ? spatial_map : &default_spatial_map_;
  }

  void setEnergyWeight(double rho_energy)
  {
    rho_energy_ = rho_energy;
  }

  void setSamplesPerPiece(int samples_per_piece)
  {
    samples_per_piece_ = std::max(1, samples_per_piece);
    sample_alphas_.resize(static_cast<std::size_t>(samples_per_piece_ + 1));
    sample_trap_weights_.resize(static_cast<std::size_t>(samples_per_piece_ + 1));
    const double inv_K = 1.0 / static_cast<double>(samples_per_piece_);
    for (int k = 0; k <= samples_per_piece_; ++k)
    {
      sample_alphas_[static_cast<std::size_t>(k)] =
          static_cast<double>(k) * inv_K;
      sample_trap_weights_[static_cast<std::size_t>(k)] =
          (k == 0 || k == samples_per_piece_) ? 0.5 : 1.0;
    }
  }

  void setUniformTimeMode(bool enabled)
  {
    uniform_time_mode_ = enabled;
  }

  bool setInitState(const std::vector<double> &time_segments,
                    const WaypointsType &waypoints,
                    const BoundaryState &boundary_head,
                    const BoundaryState &boundary_tail)
  {
    piece_num_ = static_cast<int>(time_segments.size());
    ref_times_ = time_segments;
    ref_waypoints_ = waypoints;

    workspace_->resize(piece_num_);
    nominal_head_state_ = boundary_head;
    nominal_tail_state_ = boundary_tail;
    workspace_->head_state = nominal_head_state_;
    workspace_->tail_state = nominal_tail_state_;
    return piece_num_ > 0;
  }

  void setWarmStartGuess(const Eigen::Ref<const Eigen::VectorXd> &x)
  {
    warm_start_guess_ = x;
    has_warm_start_guess_ = (x.size() > 0);
  }

  void clearWarmStartGuess()
  {
    warm_start_guess_.resize(0);
    has_warm_start_guess_ = false;
  }

  bool hasWarmStartGuess() const
  {
    return has_warm_start_guess_;
  }

  const Eigen::VectorXd &warmStartGuess() const
  {
    return warm_start_guess_;
  }

  Eigen::VectorXd encodeDecisionVector(
      const std::vector<double> &physical_times,
      const WaypointsType &physical_waypoints,
      const BoundaryStateMappingBase<DIM, S> *boundary_mapping = nullptr,
      const Eigen::VectorXd *extra_vars = nullptr) const
  {
    if (static_cast<int>(physical_times.size()) != piece_num_ ||
        physical_waypoints.rows() != piece_num_ + 1 ||
        physical_waypoints.cols() != DIM)
    {
      return Eigen::VectorXd{};
    }

    const int extra_dim =
        (boundary_mapping != nullptr && boundary_mapping->enabled())
            ? boundary_mapping->extraVariableDim()
            : 0;
    const int total_dim = getCoreDecisionDim() + extra_dim;

    Eigen::VectorXd x(total_dim);
    if (uniform_time_mode_)
    {
      double total_time = 0.0;
      for (int i = 0; i < piece_num_; ++i)
      {
        const double T = physical_times[static_cast<std::size_t>(i)];
        if (!std::isfinite(T) || T <= 0.0)
        {
          return Eigen::VectorXd{};
        }
        total_time += T;
      }
      if (total_time <= 0.0)
      {
        return Eigen::VectorXd{};
      }
      x(0) = active_time_map_->toTau(total_time);
    }
    else
    {
      for (int i = 0; i < piece_num_; ++i)
      {
        if (!std::isfinite(physical_times[static_cast<std::size_t>(i)]) ||
            physical_times[static_cast<std::size_t>(i)] <= 0.0)
        {
          return Eigen::VectorXd{};
        }
        x(i) = active_time_map_->toTau(physical_times[static_cast<std::size_t>(i)]);
      }
    }

    int offset = getTimeDecisionDim();
    for (int i = 1; i < piece_num_; ++i)
    {
      const int dof = active_spatial_map_->getUnconstrainedDim(i);
      const auto waypoint = physical_waypoints.row(i).transpose();
      if (!waypoint.allFinite())
      {
        return Eigen::VectorXd{};
      }
      x.segment(offset, dof) =
          active_spatial_map_->toUnconstrained(waypoint, i);
      offset += dof;
    }

    if (extra_dim > 0)
    {
      Eigen::Ref<Eigen::VectorXd> extra_segment =
          x.segment(offset, extra_dim);
      if (extra_vars != nullptr &&
          extra_vars->size() == extra_dim &&
          extra_vars->allFinite())
      {
        extra_segment = *extra_vars;
      }
      else if (boundary_mapping != nullptr)
      {
        boundary_mapping->setInitialExtraVariables(extra_segment);
      }
      else
      {
        extra_segment.setZero();
      }
    }

    return x;
  }

  Eigen::VectorXd generateInitialGuess() const
  {
    return generateInitialGuess(nullptr);
  }

  Eigen::VectorXd generateInitialGuess(const BoundaryStateMappingBase<DIM, S> *boundary_mapping) const
  {
    const int total_dim =
        getCoreDecisionDim() +
        ((boundary_mapping != nullptr && boundary_mapping->enabled())
             ? boundary_mapping->extraVariableDim()
             : 0);

    if (has_warm_start_guess_ &&
        warm_start_guess_.size() == total_dim &&
        warm_start_guess_.allFinite())
    {
      return warm_start_guess_;
    }

    return encodeDecisionVector(ref_times_, ref_waypoints_, boundary_mapping, nullptr);
  }

  template <typename TimeCostFunc, typename CostManager>
  double evaluate(const Eigen::Ref<const Eigen::VectorXd> &x,
                  Eigen::Ref<Eigen::VectorXd> grad_out,
                  TimeCostFunc &&time_cost_func,
                  CostManager &&cost_manager)
  {
    FixedBoundaryMapping<DIM, S> fixed_boundary_mapping;
    return evaluateWithBoundaryMapping(x,
                                       grad_out,
                                       std::forward<TimeCostFunc>(time_cost_func),
                                       std::forward<CostManager>(cost_manager),
                                       &fixed_boundary_mapping);
  }

  bool updateTrajectoryFromDecisionVector(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      const BoundaryStateMappingBase<DIM, S> *boundary_mapping = nullptr)
  {
    const int core_dim = getCoreDecisionDim();
    const int extra_dim =
        (boundary_mapping != nullptr && boundary_mapping->enabled())
            ? boundary_mapping->extraVariableDim()
            : 0;
    if (x.size() != core_dim + extra_dim)
    {
      return false;
    }

    Eigen::VectorXd dummy_grad = Eigen::VectorXd::Zero(x.size());
    double dummy_cost = 0.0;
    decodeDecisionVariables(x, dummy_grad, dummy_cost);
    workspace_->head_state = nominal_head_state_;
    workspace_->tail_state = nominal_tail_state_;
    if (boundary_mapping != nullptr && boundary_mapping->enabled())
    {
      const auto extra_vars = x.segment(core_dim, extra_dim);
      boundary_mapping->mapBoundaryStates(nominal_head_state_,
                                          nominal_tail_state_,
                                          workspace_->cache_T,
                                          extra_vars,
                                          workspace_->head_state,
                                          workspace_->tail_state);
    }

    if (uniform_time_mode_)
    {
      return traj_.generateUniform(workspace_->cache_P_inner,
                                   workspace_->head_state,
                                   workspace_->tail_state,
                                   workspace_->cache_T.sum());
    }
    return traj_.generate(workspace_->cache_P_inner,
                          workspace_->head_state,
                          workspace_->tail_state,
                          workspace_->cache_T);
  }

  template <typename TimeCostFunc, typename CostManager>
  double evaluateWithBoundaryMapping(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      Eigen::Ref<Eigen::VectorXd> grad_out,
      TimeCostFunc &&time_cost_func,
      CostManager &&cost_manager,
      const BoundaryStateMappingBase<DIM, S> *boundary_mapping)
  {
    static_assert(optimizer_traits::HasTimeCostInterface<typename std::decay<TimeCostFunc>::type>::value,
                  "TimeCostFunc does not satisfy the required interface.");

    grad_out.setZero();
    double total_cost = 0.0;
    const int core_dim = getCoreDecisionDim();
    const int extra_dim =
        (boundary_mapping != nullptr && boundary_mapping->enabled())
            ? boundary_mapping->extraVariableDim()
            : 0;

    if (x.size() != core_dim + extra_dim)
    {
      return std::numeric_limits<double>::infinity();
    }

    const auto evaluation_begin =
        timing_enabled_ ? std::chrono::steady_clock::now()
                        : std::chrono::steady_clock::time_point{};

    // Constraint elimination path:
    //   x = [tau, xi] -> (T, P_inner) through active time/spatial maps
    //   -> build MINCO trajectory in physical space
    //   -> accumulate costs/partials in physical variables
    //   -> backpropagate to the unconstrained decision vector x
    decodeDecisionVariables(x, grad_out, total_cost);
    workspace_->head_state = nominal_head_state_;
    workspace_->tail_state = nominal_tail_state_;
    if (boundary_mapping != nullptr && boundary_mapping->enabled())
    {
      const auto extra_vars = x.segment(core_dim, extra_dim);
      boundary_mapping->mapBoundaryStates(nominal_head_state_,
                                          nominal_tail_state_,
                                          workspace_->cache_T,
                                          extra_vars,
                                          workspace_->head_state,
                                          workspace_->tail_state);
      if (extra_dim > 0)
      {
        total_cost += boundary_mapping->addExtraVariableCost(
            extra_vars, grad_out.segment(core_dim, extra_dim));
      }
    }
    if (uniform_time_mode_)
    {
      traj_.generateUniform(workspace_->cache_P_inner,
                            workspace_->head_state,
                            workspace_->tail_state,
                            workspace_->cache_T.sum());
    }
    else
    {
      traj_.generate(workspace_->cache_P_inner,
                     workspace_->head_state,
                     workspace_->tail_state,
                     workspace_->cache_T);
    }

    double energy_cost = 0.0;
    if (rho_energy_ > 0.0)
    {
      traj_.getEnergyPartialGradByCoeffs(energy_cost, workspace_->gdC_energy);
      traj_.getEnergyPartialGradByTimes(workspace_->gdT_energy);
      total_cost += rho_energy_ * energy_cost;
    }
    else
    {
      workspace_->gdC_energy.setZero();
      workspace_->gdT_energy.setZero();
    }

    std::vector<double> T_vec(workspace_->cache_T.data(), workspace_->cache_T.data() + workspace_->cache_T.size());
    beginCostManagerEvaluation(cost_manager, T_vec);
    workspace_->gdT_time.setZero();
    const double time_cost = time_cost_func(T_vec, workspace_->gdT_time);
    total_cost += time_cost;

    double integral_cost = 0.0;
    const auto integral_begin =
        timing_enabled_ ? std::chrono::steady_clock::now()
                        : std::chrono::steady_clock::time_point{};
    accumulateIntegralCost(cost_manager, integral_cost);
    const auto integral_end =
        timing_enabled_ ? std::chrono::steady_clock::now()
                        : std::chrono::steady_clock::time_point{};
    total_cost += integral_cost;

    double sample_cost = 0.0;
    accumulateSampleCost(cost_manager, sample_cost);
    total_cost += sample_cost;

    double coefficient_cost = 0.0;
    using ActiveCostManager = typename std::decay<CostManager>::type;
    constexpr bool has_coefficient_cost =
        optimizer_traits::HasCoefficientCostInterface<
            ActiveCostManager, TrajType, CoeffMat>::value;
    const auto coefficient_begin =
        timing_enabled_ && has_coefficient_cost
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
    accumulateCoefficientCost(cost_manager, coefficient_cost);
    const auto coefficient_end =
        timing_enabled_ && has_coefficient_cost
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
    total_cost += coefficient_cost;

    last_energy_cost_ = rho_energy_ * energy_cost;
    last_time_cost_ = time_cost;
    last_integral_cost_ = integral_cost;
    last_sample_cost_ = sample_cost;
    last_coefficient_cost_ = coefficient_cost;

    const CoeffMat gdC_total =
        rho_energy_ * workspace_->gdC_energy +
        workspace_->gdC_integral +
        workspace_->gdC_sample +
        workspace_->gdC_coefficient;
    const Eigen::VectorXd gdT_direct_total =
        rho_energy_ * workspace_->gdT_energy +
        workspace_->gdT_time +
        workspace_->gdT_integral +
        workspace_->gdT_sample +
        workspace_->gdT_coefficient;

    traj_.propagateGradFull(gdC_total,
                            gdT_direct_total,
                            workspace_->grad_by_points,
                            workspace_->grad_by_times,
                            workspace_->grad_by_head_state,
                            workspace_->grad_by_tail_state);

    // Fixed-boundary MINCO already has dJ/dT with fixed head/tail states.
    // For boundary mappings where boundary_state = F(T, ...), add the mapping
    // chain-rule term before mapping dJ/dT back to tau.
    if (boundary_mapping != nullptr && boundary_mapping->enabled())
    {
      const auto extra_vars = x.segment(core_dim, extra_dim);
      boundary_mapping->backwardBoundaryTimeGradient(workspace_->grad_by_head_state,
                                                     workspace_->grad_by_tail_state,
                                                     workspace_->cache_T,
                                                     extra_vars,
                                                     workspace_->grad_by_times);
    }

    writeDecisionGradient(x, grad_out);

    if (boundary_mapping != nullptr && boundary_mapping->enabled())
    {
      const auto extra_vars = x.segment(core_dim, extra_dim);
      boundary_mapping->backwardBoundaryGradient(workspace_->grad_by_head_state,
                                                 workspace_->grad_by_tail_state,
                                                 workspace_->cache_T,
                                                 extra_vars,
                                                 grad_out);
    }
    if (timing_enabled_)
    {
      const auto evaluation_end = std::chrono::steady_clock::now();
      const double integral_seconds =
          std::chrono::duration<double>(integral_end - integral_begin).count();
      const double coefficient_seconds =
          has_coefficient_cost
              ? std::chrono::duration<double>(coefficient_end - coefficient_begin).count()
              : 0.0;
      const double evaluation_seconds =
          std::chrono::duration<double>(evaluation_end - evaluation_begin).count();
      last_timing_.evaluations = 1;
      last_timing_.evaluation_seconds = evaluation_seconds;
      last_timing_.dense_integral_seconds = integral_seconds;
      last_timing_.coefficient_seconds = coefficient_seconds;
      cumulative_timing_.evaluations += 1;
      cumulative_timing_.evaluation_seconds += evaluation_seconds;
      cumulative_timing_.dense_integral_seconds += integral_seconds;
      cumulative_timing_.coefficient_seconds += coefficient_seconds;
    }
    return total_cost;
  }

  const TrajType &getTrajectory() const
  {
    return traj_;
  }

  const Eigen::VectorXd &getCurrentTimes() const
  {
    return workspace_->cache_T;
  }

  int getPieceNum() const { return piece_num_; }
  int getSamplesPerPiece() const { return samples_per_piece_; }

  double lastEnergyCost() const { return last_energy_cost_; }
  double lastTimeCost() const { return last_time_cost_; }
  double lastIntegralCost() const { return last_integral_cost_; }
  double lastSampleCost() const { return last_sample_cost_; }
  double lastCoefficientCost() const { return last_coefficient_cost_; }

  void resetTimingStatistics()
  {
    last_timing_ = TimingStatistics{};
    cumulative_timing_ = TimingStatistics{};
  }

  void setTimingEnabled(bool enabled)
  {
    timing_enabled_ = enabled;
    resetTimingStatistics();
  }

  const TimingStatistics &lastTimingStatistics() const
  {
    return last_timing_;
  }

  const TimingStatistics &cumulativeTimingStatistics() const
  {
    return cumulative_timing_;
  }

private:
  int getTimeDecisionDim() const
  {
    return uniform_time_mode_ ? 1 : piece_num_;
  }

  int getCoreDecisionDim() const
  {
    int dim_P = 0;
    for (int i = 1; i < piece_num_; ++i)
    {
      dim_P += active_spatial_map_->getUnconstrainedDim(i);
    }
    return getTimeDecisionDim() + dim_P;
  }

  void decodeDecisionVariables(const Eigen::Ref<const Eigen::VectorXd> &x,
                               Eigen::Ref<Eigen::VectorXd> grad_out,
                               double &total_cost)
  {
    if (uniform_time_mode_)
    {
      const double total_time = active_time_map_->toTime(x(0));
      workspace_->cache_T.setConstant(total_time / static_cast<double>(std::max(1, piece_num_)));
    }
    else
    {
      for (int i = 0; i < piece_num_; ++i)
      {
        workspace_->cache_T(i) = active_time_map_->toTime(x(i));
      }
    }

    int offset = getTimeDecisionDim();
    for (int i = 1; i < piece_num_; ++i)
    {
      const int dof = active_spatial_map_->getUnconstrainedDim(i);
      const Eigen::VectorXd xi = x.segment(offset, dof);
      workspace_->cache_P_inner.col(i - 1) = active_spatial_map_->toPhysical(xi, i);

      Eigen::VectorXd grad_xi = Eigen::VectorXd::Zero(dof);
      active_spatial_map_->addNormPenalty(xi, total_cost, grad_xi);
      grad_out.segment(offset, dof) += grad_xi;
      offset += dof;
    }
  }

  template <typename CostManager>
  void beginCostManagerEvaluation(CostManager &&cost_manager,
                                  const std::vector<double> &times) const
  {
    using Manager = typename std::decay<CostManager>::type;
    if constexpr (optimizer_traits::HasBeginEvaluationWithTimes<Manager>::value)
    {
      cost_manager.beginEvaluation(&times);
    }
    else if constexpr (optimizer_traits::HasBeginEvaluationWithoutTimes<Manager>::value)
    {
      cost_manager.beginEvaluation();
    }
    else
    {
      (void)cost_manager;
      (void)times;
    }
  }

  template <typename CostManager>
  void accumulateIntegralCost(CostManager &&cost_manager, double &cost)
  {
    workspace_->gdC_integral.setZero();
    workspace_->gdT_integral.setZero();
    workspace_->samples.clear();

    using Manager = typename std::decay<CostManager>::type;
    if constexpr (optimizer_traits::HasDenseSamplingQuery<Manager>::value)
    {
      if (!cost_manager.usesDenseSampling())
      {
        return;
      }
    }

    bool retain_samples = true;
    if constexpr (optimizer_traits::HasSampleCostQuery<Manager>::value)
    {
      retain_samples = cost_manager.usesSampleCost();
    }
    if (retain_samples)
    {
      workspace_->samples.reserve(piece_num_ * samples_per_piece_ + 1);
    }

    const auto &coeffs = traj_.getCoefficients();
    workspace_->global_time_grad.setZero();

    double seg_start_time = 0.0;
    for (int i = 0; i < piece_num_; ++i)
    {
      const double T = workspace_->cache_T(i);
      const double inv_K = 1.0 / static_cast<double>(samples_per_piece_);
      const double dt = T * inv_K;
      const int base_row = i * TrajType::COEFF_NUM;
      const auto coeff_block = coeffs.template block<TrajType::COEFF_NUM, DIM>(base_row, 0);

      for (int k = 0; k <= samples_per_piece_; ++k)
      {
        const double alpha = sample_alphas_[static_cast<std::size_t>(k)];
        const double t = alpha * T;
        const double trap_weight =
            sample_trap_weights_[static_cast<std::size_t>(k)];
        const double common_weight = trap_weight * dt;
        const double t_global = seg_start_time + t;
        const int logical_idx = i * samples_per_piece_ + k;

        typename TrajType::BasisRow b_p, b_v, b_a, b_j, b_s;
        TrajType::computeBasisFunctions(t, b_p, b_v, b_a, b_j, b_s);

        VectorType p = VectorType::Zero();
        VectorType v = VectorType::Zero();
        VectorType a = VectorType::Zero();
        VectorType j = VectorType::Zero();
        VectorType s = VectorType::Zero();
        p.transpose().noalias() = b_p * coeff_block;
        v.transpose().noalias() = b_v * coeff_block;
        a.transpose().noalias() = b_a * coeff_block;
        j.transpose().noalias() = b_j * coeff_block;
        s.transpose().noalias() = b_s * coeff_block;

        VectorType gp = VectorType::Zero();
        VectorType gv = VectorType::Zero();
        VectorType ga = VectorType::Zero();
        VectorType gj = VectorType::Zero();
        double gt = 0.0;

        const double c_val = cost_manager.evaluateIntegral(
            logical_idx, t, t_global, i, k, p, v, a, j, gp, gv, ga, gj, gt);

        cost += c_val * common_weight;

        workspace_->gdC_integral.template block<TrajType::COEFF_NUM, DIM>(base_row, 0).noalias() +=
            (b_p.transpose() * gp.transpose() +
             b_v.transpose() * gv.transpose() +
             b_a.transpose() * ga.transpose() +
             b_j.transpose() * gj.transpose()) *
            common_weight;

        workspace_->gdT_integral(i) += c_val * trap_weight * inv_K;
        workspace_->gdT_integral(i) += (gp.dot(v) + gv.dot(a) + ga.dot(j) + gj.dot(s)) * alpha * common_weight;
        workspace_->gdT_integral(i) += gt * alpha * common_weight;
        workspace_->global_time_grad(i) += gt * common_weight;

        if (retain_samples && (k > 0 || i == 0))
        {
          Sample sample;
          sample.seg_idx = i;
          sample.step_in_seg = k;
          sample.logical_idx = logical_idx;
          sample.alpha = alpha;
          sample.t_local = t;
          sample.t_global = t_global;
          sample.trap_weight = trap_weight;
          sample.dt = dt;
          sample.b_p = b_p;
          sample.p = p;
          sample.v = v;
          workspace_->samples.push_back(sample);
        }
      }

      seg_start_time += T;
    }

    double accumulator = 0.0;
    for (int i = piece_num_ - 1; i > 0; --i)
    {
      accumulator += workspace_->global_time_grad(i);
      workspace_->gdT_integral(i - 1) += accumulator;
    }
  }

  template <typename CostManager>
  void accumulateSampleCost(CostManager &&cost_manager, double &cost)
  {
    workspace_->gdC_sample.setZero();
    workspace_->gdT_sample.setZero();

    using Manager = typename std::decay<CostManager>::type;
    if constexpr (optimizer_traits::HasSampleCostQuery<Manager>::value)
    {
      if (!cost_manager.usesSampleCost())
      {
        return;
      }
    }
    if constexpr (optimizer_traits::HasDenseSamplingQuery<Manager>::value)
    {
      if (!cost_manager.usesDenseSampling())
      {
        return;
      }
    }

    const SampleBuffer *active_samples = &workspace_->samples;
    constexpr bool uses_absolute_sample_times =
        optimizer_traits::HasDiscreteSampleTimes<typename std::decay<CostManager>::type>::value;
    if constexpr (uses_absolute_sample_times)
    {
      buildAbsoluteTimeSamples(cost_manager.discreteSampleTimes(),
                               workspace_->discrete_samples);
      active_samples = &workspace_->discrete_samples;
    }

    const Eigen::Index sample_count = static_cast<Eigen::Index>(active_samples->size());
    workspace_->sample_grad_p.resize(DIM, sample_count);
    workspace_->sample_grad_p.setZero();
    workspace_->sample_grad_t_global.resize(sample_count);
    workspace_->sample_grad_t_global.setZero();

    cost += cost_manager.evaluateSample(*active_samples,
                                        workspace_->sample_grad_p,
                                        workspace_->sample_grad_t_global);

    workspace_->global_time_grad.setZero();
    for (Eigen::Index sample_idx = 0; sample_idx < sample_count; ++sample_idx)
    {
      const auto &sample = (*active_samples)[sample_idx];
      const int base_row = sample.seg_idx * TrajType::COEFF_NUM;
      const VectorType grad_position = workspace_->sample_grad_p.col(sample_idx);

      workspace_->gdC_sample.template block<TrajType::COEFF_NUM, DIM>(base_row, 0).noalias() +=
          sample.b_p.transpose() * grad_position.transpose();

      if constexpr (uses_absolute_sample_times)
      {
        const double absolute_time_grad = grad_position.dot(sample.v);
        for (int time_idx = 0; time_idx < sample.seg_idx; ++time_idx)
        {
          workspace_->gdT_sample(time_idx) -= absolute_time_grad;
        }
      }
      else
      {
        workspace_->gdT_sample(sample.seg_idx) += grad_position.dot(sample.v) * sample.alpha;

        const double grad_time = workspace_->sample_grad_t_global(sample_idx);
        workspace_->gdT_sample(sample.seg_idx) += grad_time * sample.alpha;
        workspace_->global_time_grad(sample.seg_idx) += grad_time;
      }
    }

    if constexpr (!uses_absolute_sample_times)
    {
      double accumulator = 0.0;
      for (int i = piece_num_ - 1; i > 0; --i)
      {
        accumulator += workspace_->global_time_grad(i);
        workspace_->gdT_sample(i - 1) += accumulator;
      }
    }
  }

  template <typename CostManager>
  void accumulateCoefficientCost(CostManager &&cost_manager,
                                 double &cost)
  {
    workspace_->gdC_coefficient.setZero();
    workspace_->gdT_coefficient.setZero();

    using Manager = typename std::decay<CostManager>::type;
    if constexpr (
        optimizer_traits::HasCoefficientCostInterface<
            Manager, TrajType, CoeffMat>::value)
    {
      cost += cost_manager.evaluateCoefficient(
          traj_,
          workspace_->gdC_coefficient,
          workspace_->gdT_coefficient);
    }
    else
    {
      (void)cost_manager;
    }
  }

  void buildAbsoluteTimeSamples(const std::vector<double> &sample_times,
                                SampleBuffer &samples) const
  {
    samples.clear();
    if (piece_num_ <= 0 || sample_times.empty())
    {
      return;
    }

    double total_duration = 0.0;
    for (int i = 0; i < piece_num_; ++i)
    {
      total_duration += workspace_->cache_T(i);
    }

    const auto &coeffs = traj_.getCoefficients();
    samples.reserve(sample_times.size());
    for (std::size_t sample_id = 0; sample_id < sample_times.size(); ++sample_id)
    {
      const double raw_t = sample_times[sample_id];
      if (!std::isfinite(raw_t) || raw_t < -1.0e-6 || raw_t > total_duration + 1.0e-6)
      {
        continue;
      }

      const double t_global = std::clamp(raw_t, 0.0, total_duration);
      double seg_start_time = 0.0;
      for (int i = 0; i < piece_num_; ++i)
      {
        const double T = workspace_->cache_T(i);
        const bool in_segment =
            i == piece_num_ - 1 || t_global <= seg_start_time + T + 1.0e-9;
        if (!in_segment)
        {
          seg_start_time += T;
          continue;
        }

        const double t_local = std::clamp(t_global - seg_start_time, 0.0, T);
        const double alpha = T > 1.0e-9 ? t_local / T : 0.0;
        typename TrajType::BasisRow b_p, b_v, b_a, b_j, b_s;
        TrajType::computeBasisFunctions(t_local, b_p, b_v, b_a, b_j, b_s);

        const int base_row = i * TrajType::COEFF_NUM;
        const auto coeff_block = coeffs.template block<TrajType::COEFF_NUM, DIM>(base_row, 0);
        Sample sample;
        sample.seg_idx = i;
        sample.step_in_seg = -1;
        sample.logical_idx = static_cast<int>(sample_id);
        sample.alpha = alpha;
        sample.t_local = t_local;
        sample.t_global = t_global;
        sample.trap_weight = 1.0;
        sample.dt = 0.0;
        sample.b_p = b_p;
        sample.p.transpose().noalias() = b_p * coeff_block;
        sample.v.transpose().noalias() = b_v * coeff_block;
        samples.push_back(sample);
        break;
      }
    }
  }

  void writeDecisionGradient(const Eigen::Ref<const Eigen::VectorXd> &x,
                             Eigen::Ref<Eigen::VectorXd> grad_out) const
  {
    if (uniform_time_mode_)
    {
      grad_out(0) += active_time_map_->backward(
          x(0),
          workspace_->cache_T.sum(),
          workspace_->grad_by_times.sum() / static_cast<double>(std::max(1, piece_num_)));
    }
    else
    {
      for (int i = 0; i < piece_num_; ++i)
      {
        grad_out(i) += active_time_map_->backward(x(i), workspace_->cache_T(i), workspace_->grad_by_times(i));
      }
    }

    int offset = getTimeDecisionDim();
    for (int i = 1; i < piece_num_; ++i)
    {
      const int dof = active_spatial_map_->getUnconstrainedDim(i);
      const VectorType grad_p = workspace_->grad_by_points.col(i - 1);
      grad_out.segment(offset, dof) += active_spatial_map_->backwardGrad(x.segment(offset, dof), grad_p, i);
      offset += dof;
    }
  }

private:
  TrajType traj_;
  BoundaryState nominal_head_state_{BoundaryState::Zero()};
  BoundaryState nominal_tail_state_{BoundaryState::Zero()};
  int piece_num_{0};
  int samples_per_piece_{5};
  std::vector<double> sample_alphas_;
  std::vector<double> sample_trap_weights_;
  double rho_energy_{1.0};
  bool uniform_time_mode_{false};
  double last_energy_cost_{0.0};
  double last_time_cost_{0.0};
  double last_integral_cost_{0.0};
  double last_sample_cost_{0.0};
  double last_coefficient_cost_{0.0};
  TimingStatistics last_timing_;
  TimingStatistics cumulative_timing_;
  bool timing_enabled_{false};

  std::vector<double> ref_times_;
  WaypointsType ref_waypoints_;
  bool has_warm_start_guess_{false};
  Eigen::VectorXd warm_start_guess_;

  TimeMap default_time_map_;
  SpatialMap default_spatial_map_;
  const TimeMap *active_time_map_{nullptr};
  const SpatialMap *active_spatial_map_{nullptr};

  std::unique_ptr<Workspace> workspace_;
};

} // namespace minco

#endif
