#pragma once

/**
 * Fast L-BFGS driver: thin wrapper around math_utils::lbfgs that owns the
 * online early-stop / step-bound / optional numerical-fallback policy.
 *
 * The Quasi-Newton and line-search kernels stay in lbfgs.h; this class only
 * configures parameters and implements the progress / stepbound callbacks.
 */

#include "utils/optimization/lbfgs.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

namespace math_utils
{

class FastLbfgs
{
public:
  using Vector = Eigen::VectorXd;
  using Matrix = Eigen::MatrixXd;

  struct Options
  {
    /// When false, behave like ordinary L-BFGS (no early cancel, larger mem).
    bool early_stop_enabled{true};
    int mem_size{32};
    int fallback_mem_size{256};
    bool fallback_on_failure{true};
    bool step_bound_enabled{true};

    int window{5};
    int min_iterations{20};
    int consecutive{2};
    double rel_cost{1.0e-4};
    double rel_step{2.0e-3};
    double rel_penalty{5.0e-3};

    bool phase0_guards_en{false};
    double rel_time{2.0e-2};
    double rel_waypoint{2.0e-2};
    double scaled_grad{5.0e-2};
    double min_step_for_stall{1.0e-8};
    double penalty_tol{1.0e-2};
    int small_step_limit{3};

    int past{3};
    double delta{1.0e-5};
    double g_epsilon{0.0};
    double min_step{1.0e-32};
    int max_linesearch{64};
    int fallback_max_linesearch{128};
    double fallback_min_step{1.0e-20};
  };

  /**
   * Optional problem-specific quantities sampled once per accepted iterate.
   * Used only when early_stop_enabled (and phase0_guards for time/waypoint).
   */
  struct PhysicalSnapshot
  {
    /// Full penalty log; safety terms are indices [1..]. Empty skips penalty test.
    Vector penalty_log;
    /// Physical segment durations matching the time block of x. Empty skips.
    Vector durations;
    /// Inner waypoints, 3 x n. Empty skips waypoint test.
    Matrix waypoints;
    double corridor_scale{0.25};
  };

  using SnapshotCallback =
      PhysicalSnapshot (*)(void *user, const Vector &x);

  struct Report
  {
    int status{0};
    bool fast_stop_satisfied{false};
    bool fallback_used{false};
    std::size_t iterations{0};
    std::size_t line_search_evaluations{0};
    std::size_t max_line_search_evaluations{0};
    std::size_t fast_stop_iteration{0};
    double accepted_step_sum{0.0};
    double min_accepted_step{0.0};

    double relative_cost_change{std::numeric_limits<double>::infinity()};
    double relative_decision_step{std::numeric_limits<double>::infinity()};
    double relative_physical_time_change{0.0};
    double relative_waypoint_step{0.0};
    double scaled_gradient_inf{std::numeric_limits<double>::infinity()};
    double max_sampled_violation{0.0};
    bool trajectory_stable{false};

    // Per-condition observability. A check is counted only after the minimum
    // iteration and history-window requirements are met. Guard conditions are
    // still measured when phase0_guards_en=false, but they do not gate the
    // stable production fast stop.
    std::size_t stop_candidate_checks{0};
    std::size_t cost_passes{0};
    std::size_t decision_step_passes{0};
    std::size_t penalty_change_passes{0};
    std::size_t physical_time_passes{0};
    std::size_t waypoint_passes{0};
    std::size_t gradient_passes{0};
    std::size_t violation_passes{0};
    std::size_t nonstall_passes{0};
    std::size_t base_rule_passes{0};
    std::size_t guarded_rule_passes{0};
  };

  void setOptions(const Options &options) { options_ = options; }
  const Options &options() const { return options_; }
  const Report &report() const { return report_; }

  /// Clear counters and stop history. Call once at the start of a planner solve.
  void reset()
  {
    report_ = Report{};
    resetStopHistory();
    early_stop_active_ = options_.early_stop_enabled;
  }

  /// Clear early-stop window state only; keep cumulative iteration counters.
  void resetStopHistory()
  {
    cost_history_.clear();
    previous_x_.resize(0);
    previous_penalty_.resize(0);
    previous_durations_.resize(0);
    previous_waypoints_.resize(3, 0);
    fast_stop_streak_ = 0;
    small_step_streak_ = 0;
    report_.fast_stop_satisfied = false;
    report_.fast_stop_iteration = 0;
    report_.trajectory_stable = false;
    // Keep report_.fallback_used: it is sticky across inners / phase-2.
  }

  /**
   * Run one L-BFGS solve with the configured fast policy.
   * Does not clear cumulative iteration counters (caller uses reset()).
   * @param stepbound optional; ignored when options.step_bound_enabled is false
   * @param snapshot optional; supplies penalty/duration/waypoint for stop tests
   * @param allow_fallback recover with classical L-BFGS on numerical failure
   * @param fallback_restart if non-null, fallback restarts from this vector
   *        (legacy ExpTrajOpt restarts from the original initial decision)
   */
  int run(Vector &x,
          double &f,
          lbfgs::lbfgs_evaluate_t evaluate,
          lbfgs::lbfgs_stepbound_t stepbound,
          void *user,
          SnapshotCallback snapshot,
          bool allow_fallback = true,
          const Vector *fallback_restart = nullptr)
  {
    evaluate_ = evaluate;
    stepbound_ = stepbound;
    user_ = user;
    snapshot_ = snapshot;
    early_stop_active_ = options_.early_stop_enabled;

    const Vector restart =
        fallback_restart != nullptr ? *fallback_restart : x;
    // Primary mem_size is independent of early-stop (ALM disables cancel but
    // still wants the configured history length).
    lbfgs::lbfgs_parameter_t params = makeParams(
        options_.mem_size,
        options_.min_step,
        options_.max_linesearch,
        options_.delta);

    int status = lbfgs::lbfgs_optimize(
        x,
        f,
        &FastLbfgs::evaluateThunk,
        (options_.step_bound_enabled && early_stop_active_ &&
         stepbound_ != nullptr)
            ? &FastLbfgs::stepboundThunk
            : nullptr,
        &FastLbfgs::progressThunk,
        this,
        params);

    const bool accepted_fast_stop =
        status == lbfgs::LBFGS_CANCELED && report_.fast_stop_satisfied;
    if (allow_fallback && status < 0 && !accepted_fast_stop &&
        options_.fallback_on_failure && options_.early_stop_enabled)
    {
      report_.fallback_used = true;
      x = restart;
      early_stop_active_ = false;
      cost_history_.clear();
      previous_x_.resize(0);
      previous_penalty_.resize(0);
      previous_durations_.resize(0);
      previous_waypoints_.resize(3, 0);
      fast_stop_streak_ = 0;
      small_step_streak_ = 0;
      report_.fast_stop_satisfied = false;

      lbfgs::lbfgs_parameter_t fallback = makeParams(
          options_.fallback_mem_size,
          options_.fallback_min_step,
          options_.fallback_max_linesearch,
          options_.delta);
      status = lbfgs::lbfgs_optimize(x,
                                     f,
                                     &FastLbfgs::evaluateThunk,
                                     nullptr,
                                     &FastLbfgs::progressThunk,
                                     this,
                                     fallback);
    }

    report_.status = status;
    report_.trajectory_stable = report_.fast_stop_satisfied;
    return status;
  }

  /// True when the last optimize ended via intentional fast cancel.
  bool acceptedFastStop() const
  {
    return report_.status == lbfgs::LBFGS_CANCELED &&
           report_.fast_stop_satisfied;
  }

private:
  Options options_{};
  Report report_{};

  lbfgs::lbfgs_evaluate_t evaluate_{nullptr};
  lbfgs::lbfgs_stepbound_t stepbound_{nullptr};
  void *user_{nullptr};
  SnapshotCallback snapshot_{nullptr};

  bool early_stop_active_{false};
  std::deque<double> cost_history_;
  Vector previous_x_;
  Vector previous_penalty_;
  Vector previous_durations_;
  Matrix previous_waypoints_;
  int fast_stop_streak_{0};
  int small_step_streak_{0};

  lbfgs::lbfgs_parameter_t makeParams(int mem_size,
                                      double min_step,
                                      int max_linesearch,
                                      double delta) const
  {
    lbfgs::lbfgs_parameter_t params;
    params.mem_size = std::max(3, mem_size);
    params.past = options_.past;
    params.min_step = min_step;
    params.g_epsilon = options_.g_epsilon;
    params.delta = delta;
    params.max_linesearch = max_linesearch;
    return params;
  }

  static double evaluateThunk(void *instance,
                              const Vector &x,
                              Vector &g)
  {
    auto *self = static_cast<FastLbfgs *>(instance);
    return self->evaluate_(self->user_, x, g);
  }

  static double stepboundThunk(void *instance,
                               const Vector &x,
                               const Vector &direction)
  {
    auto *self = static_cast<FastLbfgs *>(instance);
    if (self->stepbound_ == nullptr)
    {
      return 1.0e20;
    }
    return self->stepbound_(self->user_, x, direction);
  }

  static int progressThunk(void *instance,
                           const Vector &x,
                           const Vector &g,
                           const double fx,
                           const double step,
                           const int /*k*/,
                           const int ls)
  {
    return static_cast<FastLbfgs *>(instance)->onProgress(x, g, fx, step, ls);
  }

  int onProgress(const Vector &x,
                 const Vector &g,
                 double cost,
                 double step,
                 int line_search_evaluations)
  {
    ++report_.iterations;
    const std::size_t line_search_count = static_cast<std::size_t>(
        std::max(0, line_search_evaluations));
    report_.line_search_evaluations += line_search_count;
    report_.max_line_search_evaluations =
        std::max(report_.max_line_search_evaluations, line_search_count);
    if (std::isfinite(step) && step >= 0.0)
    {
      report_.accepted_step_sum += step;
      report_.min_accepted_step =
          report_.iterations == 1
              ? step
              : std::min(report_.min_accepted_step, step);
    }

    if (!early_stop_active_ || !std::isfinite(cost) || !x.allFinite())
    {
      return 0;
    }

    cost_history_.push_back(cost);
    const std::size_t history_limit =
        static_cast<std::size_t>(options_.window + 1);
    while (cost_history_.size() > history_limit)
    {
      cost_history_.pop_front();
    }

    double relative_step = std::numeric_limits<double>::infinity();
    if (previous_x_.size() == x.size() && previous_x_.allFinite())
    {
      relative_step = (x - previous_x_).lpNorm<Eigen::Infinity>() /
                      std::max(1.0, x.lpNorm<Eigen::Infinity>());
    }

    PhysicalSnapshot snap;
    if (snapshot_ != nullptr)
    {
      snap = snapshot_(user_, x);
    }

    double relative_penalty = std::numeric_limits<double>::infinity();
    double max_penalty = 0.0;
    if (snap.penalty_log.size() > 1 && snap.penalty_log.allFinite())
    {
      max_penalty = snap.penalty_log.tail(snap.penalty_log.size() - 1)
                        .lpNorm<Eigen::Infinity>();
    }
    if (previous_penalty_.size() == snap.penalty_log.size() &&
        previous_penalty_.allFinite() && snap.penalty_log.allFinite())
    {
      const Eigen::Index safety_terms =
          std::max<Eigen::Index>(0, snap.penalty_log.size() - 1);
      if (safety_terms == 0)
      {
        relative_penalty = 0.0;
      }
      else
      {
        const auto current = snap.penalty_log.tail(safety_terms);
        const auto previous = previous_penalty_.tail(safety_terms);
        relative_penalty =
            (current - previous).lpNorm<Eigen::Infinity>() /
            std::max({1.0,
                      current.lpNorm<Eigen::Infinity>(),
                      previous.lpNorm<Eigen::Infinity>()});
      }
    }

    double relative_time = 0.0;
    if (previous_durations_.size() == snap.durations.size() &&
        previous_durations_.allFinite() && snap.durations.allFinite())
    {
      for (Eigen::Index i = 0; i < snap.durations.size(); ++i)
      {
        relative_time = std::max(
            relative_time,
            std::abs(snap.durations(i) - previous_durations_(i)) /
                std::max(previous_durations_(i), 1.0e-3));
      }
    }

    double relative_waypoint = 0.0;
    if (previous_waypoints_.cols() == snap.waypoints.cols() &&
        previous_waypoints_.rows() == snap.waypoints.rows() &&
        previous_waypoints_.allFinite() && snap.waypoints.allFinite())
    {
      const double corridor_scale = std::max(1.0e-3, snap.corridor_scale);
      for (Eigen::Index i = 0; i < snap.waypoints.cols(); ++i)
      {
        relative_waypoint = std::max(
            relative_waypoint,
            (snap.waypoints.col(i) - previous_waypoints_.col(i)).norm() /
                corridor_scale);
      }
    }

    double scaled_gradient_inf = std::numeric_limits<double>::infinity();
    if (g.size() == x.size() && g.allFinite() && x.allFinite())
    {
      scaled_gradient_inf = 0.0;
      for (Eigen::Index i = 0; i < x.size(); ++i)
      {
        const double scale = std::max(1.0, std::abs(x(i)));
        scaled_gradient_inf =
            std::max(scaled_gradient_inf, std::abs(g(i)) / scale);
      }
    }

    const bool small_step =
        std::isfinite(step) && step < options_.min_step_for_stall;
    small_step_streak_ = small_step ? small_step_streak_ + 1 : 0;
    const bool stall_blocks_fast_stop =
        small_step_streak_ >= options_.small_step_limit &&
        !(scaled_gradient_inf <= options_.scaled_grad);

    previous_x_ = x;
    previous_penalty_ = snap.penalty_log;
    previous_durations_ = snap.durations;
    previous_waypoints_ = snap.waypoints;

    report_.relative_decision_step = relative_step;
    report_.relative_physical_time_change = relative_time;
    report_.relative_waypoint_step = relative_waypoint;
    report_.scaled_gradient_inf = scaled_gradient_inf;
    report_.max_sampled_violation = max_penalty;

    bool stable = false;
    if (report_.iterations >=
            static_cast<std::size_t>(options_.min_iterations) &&
        cost_history_.size() == history_limit)
    {
      const double relative_cost =
          std::abs(cost_history_.front() - cost) /
          std::max(1.0, std::abs(cost));
      report_.relative_cost_change = relative_cost;
      const bool cost_ok = relative_cost <= options_.rel_cost;
      const bool decision_step_ok = relative_step <= options_.rel_step;
      const bool penalty_change_ok =
          relative_penalty <= options_.rel_penalty;
      const bool physical_time_ok = relative_time <= options_.rel_time;
      const bool waypoint_ok = relative_waypoint <= options_.rel_waypoint;
      const bool gradient_ok =
          scaled_gradient_inf <= options_.scaled_grad;
      const bool violation_ok = max_penalty <= options_.penalty_tol;
      const bool nonstall_ok = !stall_blocks_fast_stop;
      const bool base_rule_ok =
          cost_ok && decision_step_ok && penalty_change_ok;
      const bool guarded_rule_ok =
          base_rule_ok && physical_time_ok && waypoint_ok && gradient_ok &&
          violation_ok && nonstall_ok;

      ++report_.stop_candidate_checks;
      report_.cost_passes += static_cast<std::size_t>(cost_ok);
      report_.decision_step_passes +=
          static_cast<std::size_t>(decision_step_ok);
      report_.penalty_change_passes +=
          static_cast<std::size_t>(penalty_change_ok);
      report_.physical_time_passes +=
          static_cast<std::size_t>(physical_time_ok);
      report_.waypoint_passes += static_cast<std::size_t>(waypoint_ok);
      report_.gradient_passes += static_cast<std::size_t>(gradient_ok);
      report_.violation_passes += static_cast<std::size_t>(violation_ok);
      report_.nonstall_passes += static_cast<std::size_t>(nonstall_ok);
      report_.base_rule_passes += static_cast<std::size_t>(base_rule_ok);
      report_.guarded_rule_passes +=
          static_cast<std::size_t>(guarded_rule_ok);

      stable = base_rule_ok;
      if (options_.phase0_guards_en)
      {
        stable = guarded_rule_ok;
      }
    }

    fast_stop_streak_ = stable ? fast_stop_streak_ + 1 : 0;
    if (fast_stop_streak_ >= options_.consecutive)
    {
      report_.fast_stop_satisfied = true;
      report_.fast_stop_iteration = report_.iterations;
      report_.trajectory_stable = true;
      return 1;
    }
    return 0;
  }
};

} // namespace math_utils
