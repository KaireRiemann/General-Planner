/**
 * Strategy-doc comparison: Euclidean vs old half-Hessian H0 vs
 * Hessian-consistent H0 vs frozen MCE whitening (production V1).
 *
 * G_MCE = 2 J_P^T Q J_P = H_E,  G_0 = rho_E G_MCE.
 */

#include "traj_opt/minco/minco_metric.hpp"
#include "traj_opt/minco/minco_whitening.hpp"
#include "utils/optimization/lbfgs.h"

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using Clock = std::chrono::steady_clock;
using Trajectory = minco::MINCO_S4<3>;
using InnerPoints = Trajectory::InnerPointsMat;
using BoundaryState = Trajectory::BoundaryState;

void require(bool ok, const std::string &message)
{
  if (!ok)
  {
    throw std::runtime_error(message);
  }
}

std::string sci(double v, int d = 4)
{
  std::ostringstream oss;
  oss << std::scientific << std::setprecision(d) << v;
  return oss.str();
}

Eigen::VectorXd flatten(const InnerPoints &points)
{
  return Eigen::Map<const Eigen::VectorXd>(points.data(), points.size());
}

InnerPoints unflatten(const Eigen::VectorXd &v, int cols)
{
  InnerPoints points(3, cols);
  Eigen::Map<Eigen::VectorXd>(points.data(), v.size()) = v;
  return points;
}

Trajectory makeTrajectory(const InnerPoints &points, const Eigen::VectorXd &times)
{
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  head.col(0) << 0.0, -0.2, 0.1;
  head.col(1) << 0.45, 0.1, 0.0;
  tail.col(0) << 4.2, 1.1, 0.8;
  tail.col(1) << 0.1, -0.15, 0.0;
  Trajectory trajectory;
  require(trajectory.generate(points, head, tail, times),
          "MINCO generate failed");
  return trajectory;
}

InnerPoints makeWaypoints(int pieces)
{
  InnerPoints points(3, pieces - 1);
  for (int i = 0; i < pieces - 1; ++i)
  {
    const double u = static_cast<double>(i + 1) / static_cast<double>(pieces);
    points.col(i) << u * 4.2,
        0.22 * std::sin(2.0 * M_PI * u),
        0.4 + 0.06 * std::cos(2.0 * M_PI * u);
  }
  return points;
}

struct Objective
{
  Eigen::VectorXd times;
  Eigen::VectorXd p_guide;
  double rho_e{1.0};
  double w_l2{0.0};
};

double evaluateObjective(void *ptr, const Eigen::VectorXd &p, Eigen::VectorXd &g)
{
  auto *obj = static_cast<Objective *>(ptr);
  const int cols = static_cast<int>(obj->times.size()) - 1;
  const Trajectory traj = makeTrajectory(unflatten(p, cols), obj->times);
  Trajectory::CoeffMat gdC;
  double energy = 0.0;
  traj.getEnergyPartialGradByCoeffs(energy, gdC);
  InnerPoints gP;
  Eigen::VectorXd gT;
  traj.propagateGrad(gdC, Eigen::VectorXd::Zero(traj.getPieceNum()), gP, gT);
  g = obj->rho_e * flatten(gP);
  double cost = obj->rho_e * energy;
  if (obj->w_l2 > 0.0 && obj->p_guide.size() == p.size())
  {
    const Eigen::VectorXd diff = p - obj->p_guide;
    cost += 0.5 * obj->w_l2 * diff.squaredNorm();
    g += obj->w_l2 * diff;
  }
  return cost;
}

struct SolverState
{
  Objective *obj{nullptr};
  minco::FrozenMceWhitening *whitening{nullptr};
  Eigen::MatrixXd G;
  bool use_h0{false};
  int iters{0};
  int evals{0};
};

double evalChart(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g)
{
  auto *st = static_cast<SolverState *>(ptr);
  ++st->evals;
  Eigen::VectorXd p = x;
  if (st->whitening != nullptr && st->whitening->ready())
  {
    require(st->whitening->toChart(x, p), "whitening decode failed");
  }
  Eigen::VectorXd g_p;
  const double cost = evaluateObjective(st->obj, p, g_p);
  if (st->whitening != nullptr && st->whitening->ready())
  {
    require(st->whitening->transformCovector(g_p, g), "g_z transform failed");
  }
  else
  {
    g = g_p;
  }
  return cost;
}

bool h0Apply(void *ptr, const Eigen::VectorXd &, const Eigen::VectorXd &q,
             Eigen::VectorXd &r)
{
  auto *st = static_cast<SolverState *>(ptr);
  if (!st->use_h0)
  {
    return false;
  }
  r = st->G.ldlt().solve(q);
  return r.allFinite();
}

int countIters(void *ptr, const Eigen::VectorXd &, const Eigen::VectorXd &,
               const double, const double, const int, const int)
{
  ++static_cast<SolverState *>(ptr)->iters;
  return 0;
}

struct RunResult
{
  std::string name;
  int status{0};
  int iters{0};
  int evals{0};
  double J{0.0};
  double t_s{0.0};
  double g_norm{0.0};
};

RunResult runSolve(const std::string &name,
                   Objective &obj,
                   const Eigen::VectorXd &p0,
                   const Eigen::MatrixXd *G_h0,
                   minco::FrozenMceWhitening *whitening)
{
  SolverState st;
  st.obj = &obj;
  st.whitening = whitening;
  st.use_h0 = G_h0 != nullptr;
  if (G_h0 != nullptr)
  {
    st.G = *G_h0;
  }

  Eigen::VectorXd x = p0;
  if (whitening != nullptr && whitening->ready())
  {
    require(whitening->encodeInPlace(x), "encode z=L^T(P-P0) failed");
  }

  math_utils::lbfgs::lbfgs_parameter_t param;
  param.mem_size = 16;
  param.g_epsilon = 1.0e-8;
  param.past = 3;
  param.delta = 1.0e-12;
  param.max_iterations = 200;
  param.max_linesearch = 64;
  double f = 0.0;
  const auto t0 = Clock::now();
  const int status = math_utils::lbfgs::lbfgs_optimize(
      x, f, &evalChart, nullptr, &countIters, &st, param,
      st.use_h0 ? &h0Apply : nullptr);
  RunResult result;
  result.name = name;
  result.status = status;
  result.iters = st.iters;
  result.evals = st.evals;
  result.J = f;
  result.t_s = std::chrono::duration<double>(Clock::now() - t0).count();
  Eigen::VectorXd g;
  evalChart(&st, x, g);
  result.g_norm = g.lpNorm<Eigen::Infinity>();
  --st.evals;
  return result;
}

void printResult(const RunResult &r, const RunResult *baseline)
{
  const double iter_ratio =
      baseline != nullptr && r.iters > 0
          ? static_cast<double>(baseline->iters) / static_cast<double>(r.iters)
          : 0.0;
  std::cout << "  " << std::left << std::setw(22) << r.name
            << "  J=" << sci(r.J, 6)
            << "  iters=" << std::setw(4) << r.iters
            << "  evals=" << std::setw(4) << r.evals
            << "  |g|_inf=" << sci(r.g_norm, 3)
            << "  t=" << sci(r.t_s, 3) << "s"
            << "  status=" << r.status;
  if (baseline != nullptr)
  {
    std::cout << "  speedup_iter=" << sci(iter_ratio, 2);
  }
  std::cout << "\n";
}

minco::MincoMetric<3, 4> makeMetric(const Trajectory &trajectory, double rho_e)
{
  minco::MincoMetricOptions options;
  options.mode = minco::MincoMetricMode::kFrozenWaypoint;
  options.regularization = 0.0;
  options.energy_weight = rho_e;
  minco::MincoMetric<3, 4> metric;
  metric.setOptions(options);
  require(metric.update(trajectory), "metric update failed");
  return metric;
}

void compareCase(const std::string &title,
                 int pieces,
                 const Eigen::VectorXd &times,
                 double rho_e,
                 double w_l2)
{
  std::cout << "\n== " << title << " ==\n";
  const InnerPoints points = makeWaypoints(pieces);
  const Trajectory trajectory = makeTrajectory(points, times);
  auto metric = makeMetric(trajectory, rho_e);
  const Eigen::MatrixXd &G = metric.waypointMetric();
  const Eigen::MatrixXd G_half = 0.5 * G;

  Objective obj;
  obj.times = times;
  obj.rho_e = rho_e;
  obj.w_l2 = w_l2;
  obj.p_guide = flatten(points);
  obj.p_guide.array() += 0.15;

  const Eigen::VectorXd p0 = flatten(points);
  minco::FrozenMceWhitening whitening;
  require(whitening.configureKronecker(0, 3, p0, metric.scalarWaypointMetric()),
          "whitening configure failed");

  const RunResult euclidean =
      runSolve("A Euclidean G=I", obj, p0, nullptr, nullptr);
  const RunResult h0_half =
      runSolve("B H0 old (G=H/2)", obj, p0, &G_half, nullptr);
  const RunResult h0_full =
      runSolve("C H0 consistent", obj, p0, &G, nullptr);
  const RunResult whitened =
      runSolve("D Frozen whitening", obj, p0, nullptr, &whitening);

  printResult(euclidean, nullptr);
  printResult(h0_half, &euclidean);
  printResult(h0_full, &euclidean);
  printResult(whitened, &euclidean);

  require(std::isfinite(euclidean.J) && std::isfinite(whitened.J),
          "non-finite terminal cost");
  require(whitened.J <= euclidean.J * 1.05 + 1.0e-9,
          "frozen whitening ended with a worse cost than Euclidean");
  if (w_l2 == 0.0)
  {
    require(whitened.iters <= euclidean.iters,
            "pure-MCE whitening used more iterations than Euclidean");
    require(h0_full.iters <= h0_half.iters + 2,
            "Hessian-consistent H0 should not be worse than half-Hessian H0");
  }
}

void testRhoEScalesMetric()
{
  Eigen::VectorXd times(4);
  times << 0.8, 1.1, 0.9, 1.2;
  const InnerPoints points = makeWaypoints(4);
  const Trajectory trajectory = makeTrajectory(points, times);
  auto g1 = makeMetric(trajectory, 1.0);
  auto g4 = makeMetric(trajectory, 4.0);
  const double ratio =
      g4.waypointMetric().norm() / std::max(1.0, g1.waypointMetric().norm());
  require(std::abs(ratio - 4.0) < 1.0e-10,
          "G_0 is not rho_E * G_MCE");
  std::cout << "  rho_E scale  ||G(4)||/||G(1)||=" << sci(ratio, 6) << "\n";
}

} // namespace

int main()
{
  try
  {
    std::cout << std::setprecision(6);
    std::cout << "Frozen MCE whitening vs Euclidean / H0 comparison\n";
    std::cout << "Document V1: G_MCE=2 J^T Q J, G_0=rho_E G_MCE, z=L^T(P-P0)\n";

    testRhoEScalesMetric();

    {
      Eigen::VectorXd times = Eigen::VectorXd::Ones(5);
      compareCase("Pure MCE, M=5 uniform", 5, times, 1.0, 0.0);
    }
    {
      Eigen::VectorXd times = Eigen::VectorXd::Ones(10);
      compareCase("Pure MCE, M=10 uniform", 10, times, 1.0, 0.0);
    }
    {
      Eigen::VectorXd times(5);
      times << 0.3, 0.6, 1.2, 1.8, 1.1;
      compareCase("Pure MCE, M=5 strong time ratio", 5, times, 1.0, 0.0);
    }
    {
      Eigen::VectorXd times = Eigen::VectorXd::Ones(5);
      compareCase("MCE + L2 tracking, M=5", 5, times, 1.0, 50.0);
    }
    {
      Eigen::VectorXd times = Eigen::VectorXd::Ones(5);
      compareCase("rho_E=0.25 pure MCE, M=5", 5, times, 0.25, 0.0);
    }

    std::cout << "\n[minco_frozen_whitening_comparison_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &exception)
  {
    std::cerr << "[minco_frozen_whitening_comparison_self_test] FAIL: "
              << exception.what() << '\n';
    return 1;
  }
}
