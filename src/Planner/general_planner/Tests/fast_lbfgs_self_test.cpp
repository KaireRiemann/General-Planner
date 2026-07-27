#include "utils/optimization/fast_lbfgs.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{

void require(bool condition, const char *message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

// Smooth strongly-convex quadratic: f(x) = 0.5 * ||x - x*||^2
struct Quadratic
{
  Eigen::VectorXd x_star;
  int evals{0};

  static double evaluate(void *ptr,
                         const Eigen::VectorXd &x,
                         Eigen::VectorXd &g)
  {
    auto *self = static_cast<Quadratic *>(ptr);
    ++self->evals;
    const Eigen::VectorXd r = x - self->x_star;
    g = r;
    return 0.5 * r.squaredNorm();
  }

  static math_utils::FastLbfgs::PhysicalSnapshot snapshot(
      void *, const Eigen::VectorXd &)
  {
    math_utils::FastLbfgs::PhysicalSnapshot snap;
    // Empty penalty/durations/waypoints => penalty test stays inactive
    // (relative_penalty = inf) so early-stop needs a finite penalty log
    // for the relative-penalty gate. Supply a trivial 2-vector so the
    // safety-tail relative change is defined (0).
    snap.penalty_log.resize(2);
    snap.penalty_log.setZero();
    return snap;
  }
};

void testClassicalSolve()
{
  Quadratic problem;
  problem.x_star = Eigen::VectorXd::LinSpaced(8, 1.0, 8.0);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(8);
  double f = 0.0;

  math_utils::FastLbfgs solver;
  math_utils::FastLbfgs::Options options;
  options.early_stop_enabled = false;
  options.mem_size = 16;
  options.delta = 1.0e-12;
  options.g_epsilon = 1.0e-12;
  solver.setOptions(options);
  solver.reset();

  const int status = solver.run(x,
                                f,
                                &Quadratic::evaluate,
                                nullptr,
                                &problem,
                                &Quadratic::snapshot,
                                /*allow_fallback=*/false);
  require(status >= 0, "classical LBFGS should succeed");
  require((x - problem.x_star).norm() < 1.0e-6, "should reach minimizer");
  require(!solver.report().fast_stop_satisfied,
          "classical path must not fast-stop");
}

void testFastStopCancels()
{
  Quadratic problem;
  problem.x_star = Eigen::VectorXd::LinSpaced(6, 0.5, 3.0);

  Eigen::VectorXd x = Eigen::VectorXd::Ones(6) * 10.0;
  double f = 0.0;

  math_utils::FastLbfgs solver;
  math_utils::FastLbfgs::Options options;
  options.early_stop_enabled = true;
  options.mem_size = 16;
  options.window = 3;
  options.min_iterations = 8;
  options.consecutive = 2;
  options.rel_cost = 1.0e-3;
  options.rel_step = 1.0e-2;
  options.rel_penalty = 1.0e-2;
  options.past = 0; // disable LBFGS built-in relative-cost stop
  options.delta = 1.0e-16;
  options.g_epsilon = 0.0;
  solver.setOptions(options);
  solver.reset();

  const int status = solver.run(x,
                                f,
                                &Quadratic::evaluate,
                                nullptr,
                                &problem,
                                &Quadratic::snapshot,
                                /*allow_fallback=*/false);
  require(status == math_utils::lbfgs::LBFGS_CANCELED,
          "fast path should cancel via progress");
  require(solver.acceptedFastStop(), "cancel must be accepted fast-stop");
  require(solver.report().iterations >=
              static_cast<std::size_t>(options.min_iterations),
          "fast-stop should respect min_iterations");
  require(solver.report().stop_candidate_checks > 0,
          "fast-stop should report eligible condition checks");
  require(solver.report().base_rule_passes >=
              static_cast<std::size_t>(options.consecutive),
          "base-rule pass counter should explain the accepted stop");
  require(solver.report().cost_passes > 0 &&
              solver.report().decision_step_passes > 0 &&
              solver.report().penalty_change_passes > 0,
          "base fast-stop conditions should be counted separately");
  require((x - problem.x_star).norm() < 1.0e-2,
          "fast-stop should still land near minimizer");
}

void testFallbackRestartsFromSeed()
{
  struct FailThenOk
  {
    Eigen::VectorXd seed;
    int calls{0};
    static double evaluate(void *ptr,
                           const Eigen::VectorXd &x,
                           Eigen::VectorXd &g)
    {
      auto *self = static_cast<FailThenOk *>(ptr);
      ++self->calls;
      // First solve: emit non-finite cost to force LBFGS failure.
      if (self->calls < 3)
      {
        g = Eigen::VectorXd::Constant(x.size(),
                                      std::numeric_limits<double>::quiet_NaN());
        return std::numeric_limits<double>::quiet_NaN();
      }
      g = x; // minimize ||x||^2 / 2
      return 0.5 * x.squaredNorm();
    }
  };

  FailThenOk problem;
  problem.seed = Eigen::VectorXd::Constant(4, 2.0);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(4, 5.0);
  double f = 0.0;

  math_utils::FastLbfgs solver;
  math_utils::FastLbfgs::Options options;
  options.early_stop_enabled = true;
  options.fallback_on_failure = true;
  options.mem_size = 8;
  options.fallback_mem_size = 32;
  options.delta = 1.0e-10;
  options.g_epsilon = 1.0e-10;
  // Disable early-stop gates so fallback path can fully converge.
  options.min_iterations = 100000;
  solver.setOptions(options);
  solver.reset();

  const int status = solver.run(x,
                                f,
                                &FailThenOk::evaluate,
                                nullptr,
                                &problem,
                                nullptr,
                                /*allow_fallback=*/true,
                                &problem.seed);
  require(solver.report().fallback_used, "fallback should trigger");
  require(status >= 0 || solver.acceptedFastStop(),
          "fallback should recover a usable status");
  require(x.norm() < 1.0e-4, "fallback should optimize from seed toward 0");
}

} // namespace

int main()
{
  try
  {
    testClassicalSolve();
    testFastStopCancels();
    testFallbackRestartsFromSeed();
    std::cout << "[fast_lbfgs_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[fast_lbfgs_self_test] FAIL: " << ex.what() << "\n";
    return 1;
  }
}
