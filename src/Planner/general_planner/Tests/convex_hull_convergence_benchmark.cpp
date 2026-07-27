#include "traj_opt/convex_hull/convex_hull.hpp"
#include "traj_opt/costfunctional_manager/exp_convex_cost_manager.hpp"
#include "traj_opt/costfunctional_manager/exp_integal_cost_manager.hpp"
#include "traj_opt/minco/minco_optimizer.hpp"
#include "utils/optimization/lbfgs.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{

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

struct LinearTimeCost
{
  double weight{1.0};

  double operator()(const std::vector<double> &times,
                    Eigen::VectorXd &gradient) const
  {
    gradient.setConstant(static_cast<Eigen::Index>(times.size()), weight);
    double total = 0.0;
    for (const double time : times)
    {
      total += time;
    }
    return weight * total;
  }
};

enum class Mode
{
  DENSE = 0,
  CONVEX = 1
};

const char *modeName(Mode mode)
{
  switch (mode)
  {
    case Mode::DENSE:
      return "dense";
    case Mode::CONVEX:
      return "depth2_stable";
  }
  return "unknown";
}

using Optimizer =
    minco::MINCOOptimizer<3, 4, ExpTimeMap, IdentitySpatialMap>;
using Hull = traj_opt::convex_hull::Representation<3>;

struct ContinuousMetrics
{
  double sampled_violation{0.0};
  double certificate_violation{0.0};
  double path_length{0.0};
  double duration{0.0};
  double max_speed{0.0};
  double max_acceleration{0.0};
};

struct RunResult
{
  Mode mode{Mode::DENSE};
  int case_id{0};
  int status{0};
  std::size_t evaluations{0};
  std::size_t iterations{0};
  std::size_t line_search_evaluations{0};
  int first_sampled_feasible_iteration{-1};
  int first_certified_iteration{-1};
  double initial_cost{0.0};
  double final_cost{0.0};
  double initial_gradient_inf{0.0};
  double final_gradient_inf{0.0};
  double sampled_violation{0.0};
  double certificate_violation{0.0};
  double evaluation_seconds{0.0};
  double dense_seconds{0.0};
  double coefficient_seconds{0.0};
  double solver_seconds{0.0};
  double path_length{0.0};
  double duration{0.0};
  double max_speed{0.0};
  double max_acceleration{0.0};
  bool fast_stop_satisfied{false};
  Eigen::VectorXd decision;
};

class PairedProblem
{
public:
  PairedProblem(int case_id, Mode mode)
      : case_id_(case_id), mode_(mode)
  {
    constexpr int pieces = 4;
    const double amplitude = 0.68 + 0.08 * static_cast<double>(case_id);
    const double corridor_half_width =
        0.32 + 0.025 * static_cast<double>(case_id % 3);

    times_ = {0.82, 0.76, 0.88, 0.80};
    waypoints_.resize(pieces + 1, 3);
    waypoints_ <<
        0.0, 0.0, 0.0,
        1.0, amplitude, 0.04,
        2.0, -0.92 * amplitude, -0.03,
        3.0, 0.82 * amplitude, 0.02,
        4.0, 0.0, 0.0;

    Optimizer::BoundaryState head = Optimizer::BoundaryState::Zero();
    Optimizer::BoundaryState tail = Optimizer::BoundaryState::Zero();
    head.col(0) = waypoints_.row(0).transpose();
    tail.col(0) = waypoints_.row(pieces).transpose();

    optimizer_.setEnergyWeight(0.02);
    optimizer_.setSamplesPerPiece(15);
    optimizer_.setTimingEnabled(true);
    optimizer_.setInitState(times_, waypoints_, head, tail);
    x0_ = optimizer_.generateInitialGuess();

    corridors_.resize(1);
    corridors_[0].resize(6, 4);
    corridors_[0] <<
        1.0, 0.0, 0.0, -10.0,
        -1.0, 0.0, 0.0, -10.0,
        0.0, 1.0, 0.0, -corridor_half_width,
        0.0, -1.0, 0.0, -corridor_half_width,
        0.0, 0.0, 1.0, -1.0,
        0.0, 0.0, -1.0, -1.0;
    corridor_indices_.setZero(pieces);

    magnitude_bounds_.resize(6);
    magnitude_bounds_ << 2.6, 6.0, 25.0, 50.0, 0.1, 100.0;
    penalty_weights_ = general_utils::VecDf::Zero(7);
    penalty_weights_(0) = 2.0e4;
    penalty_weights_(1) = 2.0e2;
    penalty_weights_(2) = 2.0e1;

    flatness_map_.reset(1.0, 9.81, 0.0, 0.0, 0.0, 1.0e-4);
    dense_manager_.reset(&corridors_,
                         &corridor_indices_,
                         nullptr,
                         nullptr,
                         1.0e-2,
                         magnitude_bounds_,
                         penalty_weights_,
                         &flatness_map_,
                         swarm_config_,
                         swarm_trajectories_,
                         0.0);
    convex_manager_.configure(
        traj_opt::convex_hull::Basis::Bezier,
        2);
    convex_manager_.reset(&corridors_,
                          &corridor_indices_,
                          nullptr,
                          nullptr,
                          1.0e-2,
                          magnitude_bounds_,
                          penalty_weights_,
                          &flatness_map_,
                          swarm_config_,
                          swarm_trajectories_,
                          0.0);
    corridor_half_width_ = corridor_half_width;
  }

  RunResult run(bool fast_stop = false,
                const Eigen::VectorXd *initial_decision = nullptr)
  {
    RunResult result;
    result.mode = mode_;
    result.case_id = case_id_;

    fast_stop_enabled_ = fast_stop;
    fast_stop_satisfied_ = false;
    accepted_cost_history_.clear();
    previous_accepted_x_.resize(0);
    previous_accepted_penalty_.resize(0);
    first_sampled_feasible_iteration_ = -1;
    first_certified_iteration_ = -1;
    monitor_seconds_ = 0.0;

    Eigen::VectorXd x =
        initial_decision == nullptr ? x0_ : *initial_decision;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(x.size());
    result.initial_cost = evaluateImpl(x, gradient, false);
    result.initial_gradient_inf = gradient.lpNorm<Eigen::Infinity>();
    const auto initial_metrics = continuousMetrics(x);
    if (initial_metrics.sampled_violation <= feasibility_tolerance_)
    {
      first_sampled_feasible_iteration_ = 0;
    }
    if (initial_metrics.certificate_violation <= feasibility_tolerance_)
    {
      first_certified_iteration_ = 0;
    }

    evaluations_ = 0;
    iterations_ = 0;
    line_search_evaluations_ = 0;
    optimizer_.resetTimingStatistics();

    math_utils::lbfgs::lbfgs_parameter_t parameters;
    parameters.mem_size =
        fast_stop
            ? std::min(32, std::max(3, static_cast<int>(x.size())))
            : 64;
    parameters.past = 3;
    parameters.g_epsilon = 0.0;
    parameters.delta = 5.0e-6;
    parameters.min_step = 1.0e-32;
    parameters.max_iterations = 600;

    double minimum = result.initial_cost;
    const auto solver_begin = std::chrono::steady_clock::now();
    result.status = math_utils::lbfgs::lbfgs_optimize(
        x,
        minimum,
        &PairedProblem::evaluateCallback,
        nullptr,
        &PairedProblem::progressCallback,
        this,
        parameters);
    result.solver_seconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - solver_begin)
            .count() -
        monitor_seconds_;

    gradient.setZero();
    result.final_cost = evaluateImpl(x, gradient, false);
    result.final_gradient_inf = gradient.lpNorm<Eigen::Infinity>();
    const auto final_metrics = continuousMetrics(x);
    result.sampled_violation = final_metrics.sampled_violation;
    result.certificate_violation = final_metrics.certificate_violation;
    result.path_length = final_metrics.path_length;
    result.duration = final_metrics.duration;
    result.max_speed = final_metrics.max_speed;
    result.max_acceleration = final_metrics.max_acceleration;
    result.fast_stop_satisfied = fast_stop_satisfied_;
    result.decision = x;
    result.evaluations = evaluations_;
    result.iterations = iterations_;
    result.line_search_evaluations = line_search_evaluations_;
    result.first_sampled_feasible_iteration =
        first_sampled_feasible_iteration_;
    result.first_certified_iteration = first_certified_iteration_;

    const auto timing = optimizer_.cumulativeTimingStatistics();
    result.evaluation_seconds = timing.evaluation_seconds;
    result.dense_seconds = timing.dense_integral_seconds;
    result.coefficient_seconds = timing.coefficient_seconds;
    return result;
  }

private:
  static double evaluateCallback(void *instance,
                                 const Eigen::VectorXd &x,
                                 Eigen::VectorXd &gradient)
  {
    auto *self = static_cast<PairedProblem *>(instance);
    return self->evaluateImpl(x, gradient, true);
  }

  static int progressCallback(void *instance,
                              const Eigen::VectorXd &x,
                              const Eigen::VectorXd &,
                              double cost,
                              double,
                              int,
                              int line_search_evaluations)
  {
    auto *self = static_cast<PairedProblem *>(instance);
    const auto monitor_begin = std::chrono::steady_clock::now();
    ++self->iterations_;
    self->line_search_evaluations_ +=
        static_cast<std::size_t>(std::max(0, line_search_evaluations));
    const auto metrics = self->continuousMetrics(x);
    if (self->first_sampled_feasible_iteration_ < 0 &&
        metrics.sampled_violation <= self->feasibility_tolerance_)
    {
      self->first_sampled_feasible_iteration_ =
          static_cast<int>(self->iterations_);
    }
    if (self->first_certified_iteration_ < 0 &&
        metrics.certificate_violation <= self->feasibility_tolerance_)
    {
      self->first_certified_iteration_ =
          static_cast<int>(self->iterations_);
    }
    self->monitor_seconds_ +=
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - monitor_begin)
            .count();

    if (!self->fast_stop_enabled_ ||
        !std::isfinite(cost) ||
        !x.allFinite())
    {
      return 0;
    }

    self->accepted_cost_history_.push_back(cost);
    constexpr std::size_t history_limit = 4;
    while (self->accepted_cost_history_.size() > history_limit)
    {
      self->accepted_cost_history_.erase(
          self->accepted_cost_history_.begin());
    }

    double relative_step = std::numeric_limits<double>::infinity();
    if (self->previous_accepted_x_.size() == x.size() &&
        self->previous_accepted_x_.allFinite())
    {
      relative_step =
          (x - self->previous_accepted_x_).lpNorm<Eigen::Infinity>() /
          std::max(1.0, x.lpNorm<Eigen::Infinity>());
    }

    const auto &penalty =
        self->mode_ == Mode::DENSE
            ? self->dense_manager_.getPenaltyLog()
            : self->convex_manager_.getPenaltyLog();
    double relative_penalty = std::numeric_limits<double>::infinity();
    if (self->previous_accepted_penalty_.size() == penalty.size() &&
        self->previous_accepted_penalty_.allFinite() &&
        penalty.allFinite())
    {
      relative_penalty =
          (penalty - self->previous_accepted_penalty_)
              .lpNorm<Eigen::Infinity>() /
          std::max({1.0,
                    penalty.lpNorm<Eigen::Infinity>(),
                    self->previous_accepted_penalty_
                        .lpNorm<Eigen::Infinity>()});
    }
    self->previous_accepted_x_ = x;
    self->previous_accepted_penalty_ = penalty;

    if (self->iterations_ >= 10 &&
        self->accepted_cost_history_.size() == history_limit)
    {
      const double relative_cost =
          std::abs(self->accepted_cost_history_.front() - cost) /
          std::max(1.0, std::abs(cost));
      if (relative_cost <= 1.0e-3 &&
          relative_step <= 2.0e-2 &&
          relative_penalty <= 5.0e-2)
      {
        self->fast_stop_satisfied_ = true;
        return 1;
      }
    }
    return 0;
  }

  double evaluateImpl(const Eigen::VectorXd &x,
                      Eigen::VectorXd &gradient,
                      bool count)
  {
    if (count)
    {
      ++evaluations_;
    }
    if (mode_ == Mode::DENSE)
    {
      return optimizer_.evaluate(x, gradient, time_cost_, dense_manager_);
    }
    return optimizer_.evaluate(x, gradient, time_cost_, convex_manager_);
  }

  ContinuousMetrics continuousMetrics(const Eigen::VectorXd &x)
  {
    ContinuousMetrics metrics;
    if (!optimizer_.updateTrajectoryFromDecisionVector(x))
    {
      metrics.sampled_violation = std::numeric_limits<double>::infinity();
      metrics.certificate_violation =
          std::numeric_limits<double>::infinity();
      return metrics;
    }

    const auto &trajectory = optimizer_.getTrajectory();
    const double total_time = trajectory.getTotalDuration();
    metrics.duration = total_time;
    constexpr int samples = 4096;
    Eigen::Vector3d previous_position = trajectory.getPos(0.0);
    for (int i = 0; i <= samples; ++i)
    {
      const double time =
          total_time * static_cast<double>(i) /
          static_cast<double>(samples);
      const Eigen::Vector3d position = trajectory.getPos(time);
      const Eigen::Vector3d velocity = trajectory.getVel(time);
      const Eigen::Vector3d acceleration = trajectory.getAcc(time);
      if (i > 0)
      {
        metrics.path_length += (position - previous_position).norm();
      }
      previous_position = position;
      metrics.max_speed = std::max(metrics.max_speed, velocity.norm());
      metrics.max_acceleration =
          std::max(metrics.max_acceleration, acceleration.norm());
      metrics.sampled_violation = std::max(
          metrics.sampled_violation,
          std::max({std::abs(position.y()) - corridor_half_width_,
                    position.z() - 1.0,
                    -position.z() - 1.0,
                    velocity.norm() - magnitude_bounds_(0),
                    acceleration.norm() - magnitude_bounds_(1)}));
    }
    metrics.sampled_violation = std::max(0.0, metrics.sampled_violation);

    constexpr int certificate_depth = 6;
    for (int derivative = 0; derivative <= 2; ++derivative)
    {
      Hull hull;
      hull.resetTopology(trajectory.getPieceNum(),
                         Optimizer::TrajType::COEFF_NUM,
                         traj_opt::convex_hull::Basis::Bezier,
                         derivative,
                         certificate_depth);
      trajectory.updateConvexHull(hull);
      if (derivative == 0)
      {
        for (Eigen::Index row = 0; row < hull.controls().rows(); ++row)
        {
          const auto control = hull.controls().row(row);
          metrics.certificate_violation = std::max(
              metrics.certificate_violation,
              std::max({std::abs(control.y()) - corridor_half_width_,
                        control.z() - 1.0,
                        -control.z() - 1.0}));
        }
      }
      else
      {
        const double bound = magnitude_bounds_(derivative - 1);
        for (Eigen::Index row = 0; row < hull.controls().rows(); ++row)
        {
          metrics.certificate_violation = std::max(
              metrics.certificate_violation,
              hull.controls().row(row).norm() - bound);
        }
      }
    }
    metrics.certificate_violation =
        std::max(0.0, metrics.certificate_violation);
    return metrics;
  }

public:
  double maxPositionDifference(const Eigen::VectorXd &lhs,
                               const Eigen::VectorXd &rhs)
  {
    if (!optimizer_.updateTrajectoryFromDecisionVector(lhs))
    {
      return std::numeric_limits<double>::infinity();
    }
    const auto lhs_trajectory = optimizer_.getTrajectory();
    if (!optimizer_.updateTrajectoryFromDecisionVector(rhs))
    {
      return std::numeric_limits<double>::infinity();
    }
    const auto rhs_trajectory = optimizer_.getTrajectory();
    constexpr int samples = 4096;
    double maximum = 0.0;
    for (int i = 0; i <= samples; ++i)
    {
      const double alpha =
          static_cast<double>(i) / static_cast<double>(samples);
      maximum = std::max(
          maximum,
          (lhs_trajectory.getPos(alpha * lhs_trajectory.getTotalDuration()) -
           rhs_trajectory.getPos(alpha * rhs_trajectory.getTotalDuration()))
              .norm());
    }
    return maximum;
  }

private:
  int case_id_{0};
  Mode mode_{Mode::DENSE};
  Optimizer optimizer_;
  std::vector<double> times_;
  Optimizer::WaypointsType waypoints_;
  Eigen::VectorXd x0_;
  general_utils::PolyhedraH corridors_;
  Eigen::VectorXi corridor_indices_;
  general_utils::VecDf magnitude_bounds_;
  general_utils::VecDf penalty_weights_;
  flatness::FlatnessMap flatness_map_;
  traj_opt::SwarmPenaltyConfig swarm_config_;
  traj_opt::SwarmTrajectoriesConstPtr swarm_trajectories_;
  cost_functional_manager::ExpIntegralCostManager dense_manager_;
  cost_functional_manager::ExpConvexCostManager convex_manager_;
  LinearTimeCost time_cost_;
  double corridor_half_width_{0.35};
  double feasibility_tolerance_{1.0e-3};
  std::size_t evaluations_{0};
  std::size_t iterations_{0};
  std::size_t line_search_evaluations_{0};
  int first_sampled_feasible_iteration_{-1};
  int first_certified_iteration_{-1};
  double monitor_seconds_{0.0};
  bool fast_stop_enabled_{false};
  bool fast_stop_satisfied_{false};
  std::vector<double> accepted_cost_history_;
  Eigen::VectorXd previous_accepted_x_;
  Eigen::VectorXd previous_accepted_penalty_;
};

struct Aggregate
{
  std::size_t runs{0};
  std::size_t evaluations{0};
  std::size_t iterations{0};
  std::size_t line_search_evaluations{0};
  std::size_t sampled_feasible_runs{0};
  std::size_t certified_runs{0};
  double evaluation_seconds{0.0};
  double dense_seconds{0.0};
  double coefficient_seconds{0.0};
  double solver_seconds{0.0};
  double final_sampled_violation{0.0};
  double final_certificate_violation{0.0};

  void add(const RunResult &result)
  {
    ++runs;
    evaluations += result.evaluations;
    iterations += result.iterations;
    line_search_evaluations += result.line_search_evaluations;
    sampled_feasible_runs += result.sampled_violation <= 1.0e-3;
    certified_runs += result.certificate_violation <= 1.0e-3;
    evaluation_seconds += result.evaluation_seconds;
    dense_seconds += result.dense_seconds;
    coefficient_seconds += result.coefficient_seconds;
    solver_seconds += result.solver_seconds;
    final_sampled_violation =
        std::max(final_sampled_violation, result.sampled_violation);
    final_certificate_violation =
        std::max(final_certificate_violation,
                 result.certificate_violation);
  }
};

} // namespace

int main()
{
  constexpr int cases = 6;
  const std::array<Mode, 2> modes{Mode::DENSE, Mode::CONVEX};
  std::array<Aggregate, 2> aggregates;

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "case,mode,status,evaluations,iterations,line_search,"
               "eval_per_iteration,first_sample_feasible,"
               "first_certified,initial_cost,final_cost,"
               "initial_grad_inf,final_grad_inf,sampled_violation,"
               "certificate_violation,evaluation_ms,dense_ms,"
               "coefficient_ms,solver_ms\n";

  for (int case_id = 0; case_id < cases; ++case_id)
  {
    std::array<RunResult, 2> case_results;
    for (int order = 0; order < 2; ++order)
    {
      const int mode_index = (case_id + order) % 2;
      const Mode mode = modes[mode_index];
      PairedProblem problem(case_id, mode);
      case_results[mode_index] = problem.run();
    }

    for (int mode_index = 0; mode_index < 2; ++mode_index)
    {
      const auto &result = case_results[mode_index];
      aggregates[mode_index].add(result);
      std::cout << result.case_id << ","
                << modeName(result.mode) << ","
                << result.status << ","
                << result.evaluations << ","
                << result.iterations << ","
                << result.line_search_evaluations << ","
                << (result.iterations > 0
                        ? static_cast<double>(
                              result.line_search_evaluations) /
                              static_cast<double>(result.iterations)
                        : 0.0)
                << ","
                << result.first_sampled_feasible_iteration << ","
                << result.first_certified_iteration << ","
                << result.initial_cost << ","
                << result.final_cost << ","
                << result.initial_gradient_inf << ","
                << result.final_gradient_inf << ","
                << result.sampled_violation << ","
                << result.certificate_violation << ","
                << result.evaluation_seconds * 1.0e3 << ","
                << result.dense_seconds * 1.0e3 << ","
                << result.coefficient_seconds * 1.0e3 << ","
                << result.solver_seconds * 1.0e3
                << "\n";
    }
  }

  std::cout << "aggregate,mode,runs,evaluations_per_run,"
               "iterations_per_run,line_search_per_iteration,"
               "evaluation_us,dense_us,coefficient_us,solver_ms_per_run,"
               "sampled_feasible_runs,certified_runs,"
               "max_sampled_violation,max_certificate_violation\n";
  for (int mode_index = 0; mode_index < 2; ++mode_index)
  {
    const auto &aggregate = aggregates[mode_index];
    std::cout << "aggregate,"
              << modeName(modes[mode_index]) << ","
              << aggregate.runs << ","
              << static_cast<double>(aggregate.evaluations) /
                     static_cast<double>(aggregate.runs)
              << ","
              << static_cast<double>(aggregate.iterations) /
                     static_cast<double>(aggregate.runs)
              << ","
              << (aggregate.iterations > 0
                      ? static_cast<double>(
                            aggregate.line_search_evaluations) /
                            static_cast<double>(aggregate.iterations)
                      : 0.0)
              << ","
              << aggregate.evaluation_seconds * 1.0e6 /
                     static_cast<double>(aggregate.evaluations)
              << ","
              << aggregate.dense_seconds * 1.0e6 /
                     static_cast<double>(aggregate.evaluations)
              << ","
              << aggregate.coefficient_seconds * 1.0e6 /
                     static_cast<double>(aggregate.evaluations)
              << ","
              << aggregate.solver_seconds * 1.0e3 /
                     static_cast<double>(aggregate.runs)
              << ","
              << aggregate.sampled_feasible_runs << ","
              << aggregate.certified_runs << ","
              << aggregate.final_sampled_violation << ","
              << aggregate.final_certificate_violation
              << "\n";
  }

  // Isolate the fast stopping rule from the convex-hull representation.
  // Every strict/fast pair starts from the identical dense objective and
  // decision vector. The polish solve starts at the fast result and uses the
  // original strict relative-cost tolerance, measuring how much useful
  // optimization the early stop left unfinished.
  std::cout << "fast_quality,case,strict_evaluations,fast_evaluations,"
               "polish_evaluations,strict_iterations,fast_iterations,"
               "polish_iterations,strict_cost,fast_cost,polished_cost,"
               "fast_cost_gap_rel,polish_gain_rel,strict_grad_inf,"
               "fast_grad_inf,polished_grad_inf,max_position_difference,"
               "duration_gap_rel,path_length_gap_rel,strict_max_speed,"
               "fast_max_speed,strict_max_acceleration,"
               "fast_max_acceleration,strict_sampled_violation,"
               "fast_sampled_violation,strict_certificate_violation,"
               "fast_certificate_violation,fast_stop_satisfied,"
               "strict_solver_ms,fast_solver_ms,polish_solver_ms\n";
  double sum_strict_evaluations = 0.0;
  double sum_fast_evaluations = 0.0;
  double sum_polish_evaluations = 0.0;
  double sum_strict_solver_ms = 0.0;
  double sum_fast_solver_ms = 0.0;
  double sum_polish_solver_ms = 0.0;
  double max_fast_cost_gap_rel = 0.0;
  double max_polish_gain_rel = 0.0;
  double max_position_difference = 0.0;
  double max_duration_gap_rel = 0.0;
  double max_path_length_gap_rel = 0.0;
  double max_sampled_violation = 0.0;
  double max_certificate_violation = 0.0;
  int fast_stop_count = 0;
  for (int case_id = 0; case_id < cases; ++case_id)
  {
    PairedProblem strict_problem(case_id, Mode::DENSE);
    const RunResult strict = strict_problem.run(false);
    PairedProblem fast_problem(case_id, Mode::DENSE);
    const RunResult fast = fast_problem.run(true);
    PairedProblem polish_problem(case_id, Mode::DENSE);
    const RunResult polished =
        polish_problem.run(false, &fast.decision);

    const double fast_cost_gap_rel =
        (fast.final_cost - strict.final_cost) /
        std::max(1.0, std::abs(strict.final_cost));
    const double polish_gain_rel =
        (fast.final_cost - polished.final_cost) /
        std::max(1.0, std::abs(fast.final_cost));
    const double position_difference =
        strict_problem.maxPositionDifference(
            strict.decision, fast.decision);
    const double duration_gap_rel =
        std::abs(fast.duration - strict.duration) /
        std::max(1.0, std::abs(strict.duration));
    const double path_length_gap_rel =
        std::abs(fast.path_length - strict.path_length) /
        std::max(1.0, std::abs(strict.path_length));

    sum_strict_evaluations += strict.evaluations;
    sum_fast_evaluations += fast.evaluations;
    sum_polish_evaluations += polished.evaluations;
    sum_strict_solver_ms += strict.solver_seconds * 1.0e3;
    sum_fast_solver_ms += fast.solver_seconds * 1.0e3;
    sum_polish_solver_ms += polished.solver_seconds * 1.0e3;
    max_fast_cost_gap_rel =
        std::max(max_fast_cost_gap_rel, fast_cost_gap_rel);
    max_polish_gain_rel =
        std::max(max_polish_gain_rel, polish_gain_rel);
    max_position_difference =
        std::max(max_position_difference, position_difference);
    max_duration_gap_rel =
        std::max(max_duration_gap_rel, duration_gap_rel);
    max_path_length_gap_rel =
        std::max(max_path_length_gap_rel, path_length_gap_rel);
    max_sampled_violation =
        std::max({max_sampled_violation,
                  strict.sampled_violation,
                  fast.sampled_violation});
    max_certificate_violation =
        std::max({max_certificate_violation,
                  strict.certificate_violation,
                  fast.certificate_violation});
    fast_stop_count += fast.fast_stop_satisfied ? 1 : 0;

    std::cout << "fast_quality,"
              << case_id << ","
              << strict.evaluations << ","
              << fast.evaluations << ","
              << polished.evaluations << ","
              << strict.iterations << ","
              << fast.iterations << ","
              << polished.iterations << ","
              << strict.final_cost << ","
              << fast.final_cost << ","
              << polished.final_cost << ","
              << fast_cost_gap_rel << ","
              << polish_gain_rel << ","
              << strict.final_gradient_inf << ","
              << fast.final_gradient_inf << ","
              << polished.final_gradient_inf << ","
              << position_difference << ","
              << duration_gap_rel << ","
              << path_length_gap_rel << ","
              << strict.max_speed << ","
              << fast.max_speed << ","
              << strict.max_acceleration << ","
              << fast.max_acceleration << ","
              << strict.sampled_violation << ","
              << fast.sampled_violation << ","
              << strict.certificate_violation << ","
              << fast.certificate_violation << ","
              << static_cast<int>(fast.fast_stop_satisfied) << ","
              << strict.solver_seconds * 1.0e3 << ","
              << fast.solver_seconds * 1.0e3 << ","
              << polished.solver_seconds * 1.0e3
              << "\n";
  }
  std::cout << "fast_quality_aggregate,cases=" << cases
            << ",fast_stops=" << fast_stop_count
            << ",strict_evaluations_per_case="
            << sum_strict_evaluations / static_cast<double>(cases)
            << ",fast_evaluations_per_case="
            << sum_fast_evaluations / static_cast<double>(cases)
            << ",polish_evaluations_per_case="
            << sum_polish_evaluations / static_cast<double>(cases)
            << ",strict_solver_ms_per_case="
            << sum_strict_solver_ms / static_cast<double>(cases)
            << ",fast_solver_ms_per_case="
            << sum_fast_solver_ms / static_cast<double>(cases)
            << ",polish_solver_ms_per_case="
            << sum_polish_solver_ms / static_cast<double>(cases)
            << ",max_fast_cost_gap_rel=" << max_fast_cost_gap_rel
            << ",max_polish_gain_rel=" << max_polish_gain_rel
            << ",max_position_difference=" << max_position_difference
            << ",max_duration_gap_rel=" << max_duration_gap_rel
            << ",max_path_length_gap_rel=" << max_path_length_gap_rel
            << ",max_sampled_violation=" << max_sampled_violation
            << ",max_certificate_violation="
            << max_certificate_violation
            << "\n";

  // A degree-two polynomial is also a valid degree-seven MINCO polynomial.
  // With 15 intervals (16 nodes), y(u)=4u(1-u) is sampled only at i/15.
  // The true peak at u=0.5 lies strictly between two nodes.
  constexpr double safety_bound = 0.998;
  double sampled_peak = 0.0;
  for (int i = 0; i <= 15; ++i)
  {
    const double u = static_cast<double>(i) / 15.0;
    sampled_peak = std::max(sampled_peak, 4.0 * u * (1.0 - u));
  }
  Hull::Matrix coefficients = Hull::Matrix::Zero(8, 3);
  coefficients(1, 1) = 4.0;
  coefficients(2, 1) = -4.0;
  Eigen::VectorXd duration(1);
  duration << 1.0;
  Hull safety_hull;
  safety_hull.resetTopology(
      1, 8, traj_opt::convex_hull::Basis::Bezier, 0, 2);
  safety_hull.update(coefficients, duration);
  const double hull_peak = safety_hull.controls().col(1).maxCoeff();
  std::cout << "safety_probe,dense_nodes=16,sampled_peak="
            << sampled_peak
            << ",true_peak=1.000000,depth2_hull_peak="
            << hull_peak
            << ",bound=" << safety_bound
            << ",dense_detects="
            << static_cast<int>(sampled_peak > safety_bound)
            << ",continuous_violation="
            << std::max(0.0, 1.0 - safety_bound)
            << ",hull_detects="
            << static_cast<int>(hull_peak > safety_bound)
            << "\n";
  return 0;
}
