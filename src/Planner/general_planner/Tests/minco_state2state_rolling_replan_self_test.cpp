/**
 * Rolling-replan A/B on the production Frozen-MCE chart:
 * QuadInvTimeMap time + frozen Cholesky whitening of waypoints + Fast L-BFGS.
 *
 * Each snapshot is a remaining-horizon State2State-like problem. Euclidean and
 * Frozen MCE are solved on the identical sequence so latency is not confounded
 * by closed-loop path divergence. Closed-loop click_demo numbers come from
 * scripts/run_mce_vs_euclidean_state2state_ab.sh.
 */

#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/minco/minco_metric.hpp"
#include "traj_opt/minco/minco_whitening.hpp"
#include "utils/optimization/fast_lbfgs.hpp"

#include <Eigen/Dense>

#include <algorithm>
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
using CoeffMat = Trajectory::CoeffMat;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

void require(bool ok, const std::string &message)
{
  if (!ok)
  {
    throw std::runtime_error(message);
  }
}

VectorXd flatten(const InnerPoints &points)
{
  return Eigen::Map<const VectorXd>(points.data(), points.size());
}

InnerPoints unflatten(const VectorXd &v, int cols)
{
  InnerPoints points(3, cols);
  Eigen::Map<VectorXd>(points.data(), v.size()) = v;
  return points;
}

Vector3d nominalPos(double x)
{
  return Vector3d(x, 0.14 * std::sin(0.35 * x), 1.5);
}

struct Snapshot
{
  int pieces{8};
  VectorXd times;
  InnerPoints P0;
  InnerPoints P_guide;
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  double y_max{0.10};

  int inner() const { return std::max(0, pieces - 1); }
  int spatial() const { return 3 * inner(); }
};

struct Weights
{
  double energy{1.0};
  double time{20.0};
  double track{0.0};
  double vel{0.25};
  double acc{0.08};
  double corr{25.0};
  double vmax{15.0};
  double amax{20.0};
  int samples{8};
};

struct SolveResult
{
  int status{0};
  bool fast_stop{false};
  std::size_t iterations{0};
  std::size_t evaluations{0};
  double lbfgs_ms{0.0};
  double metric_ms{0.0};
  double cost{0.0};
  double duration{0.0};
  double max_viol{0.0};
  bool metric_ok{false};
};

Snapshot makeSnapshot(int index, int pieces)
{
  Snapshot s;
  s.pieces = pieces;
  const double x0 = 2.8 * static_cast<double>(index);
  const double xg = 70.0;
  const double remain = std::max(8.0, xg - x0);
  s.times = VectorXd::Constant(pieces, remain / (8.0 * pieces));
  s.P0.resize(3, s.inner());
  s.P_guide.resize(3, s.inner());
  for (int i = 0; i < s.inner(); ++i)
  {
    const double a = static_cast<double>(i + 1) / static_cast<double>(pieces);
    const double x = x0 + a * (xg - x0);
    Vector3d p = nominalPos(x);
    s.P_guide.col(i) = p;
    p.y() += 0.08 * std::sin(0.9 * x + 0.4 * index);
    s.P0.col(i) = p;
  }
  s.head.setZero();
  s.tail.setZero();
  s.head.col(0) = nominalPos(x0);
  s.head.col(1) << 8.0, 0.0, 0.0;
  s.tail.col(0) = nominalPos(xg);
  s.y_max = (index % 5 == 4) ? 0.07 : 0.10;
  return s;
}

Vector3d guidePos(const Snapshot &snap, double t_global)
{
  double t0 = 0.0;
  for (int i = 0; i < snap.pieces; ++i)
  {
    const double T = snap.times(i);
    const bool last = i == snap.pieces - 1;
    if (t_global <= t0 + T || last)
    {
      const double a =
          T > 1.0e-9 ? std::clamp((t_global - t0) / T, 0.0, 1.0) : 0.0;
      const Vector3d p0 =
          (i == 0) ? Vector3d(snap.head.col(0))
                   : Vector3d(snap.P_guide.col(i - 1));
      const Vector3d p1 =
          (i == snap.inner()) ? Vector3d(snap.tail.col(0))
                              : Vector3d(snap.P_guide.col(i));
      return (1.0 - a) * p0 + a * p1;
    }
    t0 += T;
  }
  return snap.tail.col(0);
}

struct EvalOut
{
  double cost{0.0};
  double max_viol{0.0};
  VectorXd gP;
  VectorXd gT;
};

EvalOut evaluatePhysical(const Snapshot &snap, const Weights &w,
                         const VectorXd &P, const VectorXd &T)
{
  Trajectory traj;
  require(traj.generate(unflatten(P, snap.inner()), snap.head, snap.tail, T),
          "MINCO generate failed");

  EvalOut out;
  out.gP = VectorXd::Zero(snap.spatial());
  out.gT = VectorXd::Zero(snap.pieces);

  CoeffMat gdC = CoeffMat::Zero(Trajectory::COEFF_NUM * snap.pieces, 3);
  VectorXd gdT = VectorXd::Zero(snap.pieces);

  double energy = 0.0;
  CoeffMat gdC_e;
  VectorXd gdT_e;
  traj.getEnergyPartialGradByCoeffs(energy, gdC_e);
  traj.getEnergyPartialGradByTimes(gdT_e);
  out.cost += w.energy * energy;
  gdC += w.energy * gdC_e;
  gdT += w.energy * gdT_e;

  out.cost += w.time * T.sum();
  gdT.array() += w.time;

  const auto &C = traj.getCoefficients();
  double t_global = 0.0;
  for (int i = 0; i < snap.pieces; ++i)
  {
    const double Ti = T(i);
    const double inv_K = 1.0 / static_cast<double>(w.samples);
    const double dt = Ti * inv_K;
    const auto coeff_block =
        C.block<Trajectory::COEFF_NUM, 3>(i * Trajectory::COEFF_NUM, 0);
    for (int k = 0; k <= w.samples; ++k)
    {
      const double alpha = static_cast<double>(k) * inv_K;
      const double t_local = alpha * Ti;
      const double trap = (k == 0 || k == w.samples) ? 0.5 : 1.0;
      const double common = trap * dt;
      typename Trajectory::BasisRow b_p, b_v, b_a, b_j, b_s;
      Trajectory::computeBasisFunctions(t_local, b_p, b_v, b_a, b_j, b_s);
      const Vector3d p = (b_p * coeff_block).transpose();
      const Vector3d v = (b_v * coeff_block).transpose();
      const Vector3d a = (b_a * coeff_block).transpose();
      const Vector3d j = (b_j * coeff_block).transpose();

      Vector3d gp = Vector3d::Zero();
      Vector3d gv = Vector3d::Zero();
      Vector3d ga = Vector3d::Zero();
      double c_val = 0.0;

      const Vector3d e = p - guidePos(snap, t_global + t_local);
      c_val += 0.5 * w.track * e.squaredNorm();
      gp += w.track * e;

      const double v2 = v.squaredNorm();
      c_val += 0.5 * w.vel * v2;
      gv += w.vel * v;
      const double v_over = v2 - w.vmax * w.vmax;
      if (v_over > 0.0)
      {
        c_val += 0.5 * 8.0 * w.vel * v_over * v_over;
        gv += (16.0 * w.vel * v_over) * v;
      }

      const double a2 = a.squaredNorm();
      c_val += 0.5 * w.acc * a2;
      ga += w.acc * a;
      const double a_over = a2 - w.amax * w.amax;
      if (a_over > 0.0)
      {
        c_val += 0.5 * 8.0 * w.acc * a_over * a_over;
        ga += (16.0 * w.acc * a_over) * a;
      }

      const double viol = std::abs(p.y()) - snap.y_max;
      out.max_viol = std::max(out.max_viol, viol);
      if (viol > 0.0)
      {
        c_val += 0.5 * w.corr * viol * viol;
        gp.y() += w.corr * viol * (p.y() >= 0.0 ? 1.0 : -1.0);
      }

      out.cost += c_val * common;
      gdC.block<Trajectory::COEFF_NUM, 3>(i * Trajectory::COEFF_NUM, 0)
          .noalias() +=
          (b_p.transpose() * gp.transpose() + b_v.transpose() * gv.transpose() +
           b_a.transpose() * ga.transpose()) *
          common;
      gdT(i) += c_val * trap * inv_K;
      gdT(i) += (gp.dot(v) + gv.dot(a) + ga.dot(j)) * alpha * common;
    }
    t_global += Ti;
  }

  InnerPoints gP;
  VectorXd gT;
  traj.propagateGrad(gdC, gdT, gP, gT);
  out.gP = flatten(gP);
  out.gT = gT;
  return out;
}

struct SolverState
{
  const Snapshot *snap{nullptr};
  Weights w;
  temporal_map::QuadInvTimeMap time_map;
  minco::FrozenMceWhitening *whitening{nullptr};
  std::size_t evaluations{0};
  std::size_t iterations{0};
};

void decode(const SolverState &st, const VectorXd &x, VectorXd &T, VectorXd &P)
{
  T.resize(st.snap->pieces);
  for (int i = 0; i < st.snap->pieces; ++i)
  {
    T(i) = st.time_map.toTime(x(i));
  }
  P = x.segment(st.snap->pieces, st.snap->spatial());
  if (st.whitening != nullptr && st.whitening->ready())
  {
    VectorXd chart;
    require(st.whitening->toChart(P, chart), "decode z->P failed");
    P = chart;
  }
}

double costFn(void *ptr, const VectorXd &x, VectorXd &g)
{
  auto *st = static_cast<SolverState *>(ptr);
  ++st->evaluations;
  VectorXd T, P;
  decode(*st, x, T, P);
  const EvalOut ev = evaluatePhysical(*st->snap, st->w, P, T);
  g.resize(x.size());
  for (int i = 0; i < st->snap->pieces; ++i)
  {
    g(i) = st->time_map.backward(x(i), T(i), ev.gT(i));
  }
  g.segment(st->snap->pieces, st->snap->spatial()) = ev.gP;
  if (st->whitening != nullptr && st->whitening->ready())
  {
    require(st->whitening->transformCovectorInPlace(g), "g_z failed");
  }
  return ev.cost;
}

math_utils::FastLbfgs::PhysicalSnapshot snapshotFn(void *ptr, const VectorXd &x)
{
  auto *st = static_cast<SolverState *>(ptr);
  VectorXd T, P;
  decode(*st, x, T, P);
  math_utils::FastLbfgs::PhysicalSnapshot snap;
  snap.durations = T;
  snap.waypoints = unflatten(P, st->snap->inner());
  return snap;
}

enum class StopMode
{
  ProductionFast,
  PhysicalGuards,
  FullConvergence
};

void configureFast(math_utils::FastLbfgs &solver, StopMode mode)
{
  math_utils::FastLbfgs::Options opt;
  opt.mem_size = 32;
  opt.step_bound_enabled = false;
  opt.rel_cost = 1.0e-3;
  opt.rel_step = 2.0e-2;
  opt.rel_penalty = 5.0e-2;
  opt.window = 3;
  opt.consecutive = 1;
  opt.min_iterations = 10;
  opt.rel_time = 2.0e-2;
  opt.rel_waypoint = 2.0e-2;
  opt.scaled_grad = 5.0e-2;
  if (mode == StopMode::FullConvergence)
  {
    opt.early_stop_enabled = false;
    opt.g_epsilon = 1.0e-5;
    opt.min_iterations = 0;
  }
  else
  {
    opt.early_stop_enabled = true;
    opt.g_epsilon = 0.0;
    opt.phase0_guards_en = (mode == StopMode::PhysicalGuards);
  }
  solver.setOptions(opt);
}

int countIters(void *ptr, const VectorXd &, const VectorXd &, const double,
               const double, const int k, const int)
{
  static_cast<SolverState *>(ptr)->iterations = static_cast<std::size_t>(k);
  return 0;
}

SolveResult solveSnapshot(const Snapshot &snap, const Weights &w, bool use_mce,
                          StopMode mode)
{
  SolverState st;
  st.snap = &snap;
  st.w = w;

  VectorXd x(snap.pieces + snap.spatial());
  for (int i = 0; i < snap.pieces; ++i)
  {
    x(i) = st.time_map.toTau(snap.times(i));
  }
  x.segment(snap.pieces, snap.spatial()) = flatten(snap.P0);

  minco::FrozenMceWhitening whitening;
  double metric_ms = 0.0;
  bool metric_ok = false;
  if (use_mce)
  {
    const auto t0 = Clock::now();
    Trajectory seed;
    require(seed.generate(snap.P0, snap.head, snap.tail, snap.times),
            "seed generate failed");
    minco::MincoMetric<3, 4> metric;
    minco::MincoMetricOptions options;
    options.mode = minco::MincoMetricMode::kFrozenWaypoint;
    options.regularization = 1.0e-10;
    options.energy_weight = w.energy;
    metric.setOptions(options);
    require(metric.update(seed), "G_MCE build failed");
    require(whitening.configureKronecker(snap.pieces, 3, flatten(snap.P0),
                                         metric.scalarWaypointMetric()),
            "whitening configure failed");
    st.whitening = &whitening;
    require(whitening.encodeInPlace(x), "encode z failed");
    metric_ms =
        1.0e3 * std::chrono::duration<double>(Clock::now() - t0).count();
    metric_ok = true;
  }

  double f = 0.0;
  int status = 0;
  std::size_t iterations = 0;
  bool fast_stop = false;
  const auto t_solve = Clock::now();
  if (mode == StopMode::FullConvergence)
  {
    st.evaluations = 0;
    st.iterations = 0;
    math_utils::lbfgs::lbfgs_parameter_t param;
    param.mem_size = 32;
    param.g_epsilon = 0.0;
    param.past = 3;
    param.delta = 1.0e-12;
    param.max_iterations = 400;
    param.max_linesearch = 64;
    status = math_utils::lbfgs::lbfgs_optimize(
        x, f, &costFn, nullptr, &countIters, &st, param);
    iterations = st.iterations;
  }
  else
  {
    math_utils::FastLbfgs solver;
    configureFast(solver, mode);
    solver.reset();
    status =
        solver.run(x, f, &costFn, nullptr, &st, &snapshotFn, true, &x, nullptr);
    iterations = solver.report().iterations;
    fast_stop = solver.acceptedFastStop();
  }
  const double lbfgs_ms =
      1.0e3 * std::chrono::duration<double>(Clock::now() - t_solve).count();

  VectorXd T, P;
  decode(st, x, T, P);
  const EvalOut ev = evaluatePhysical(snap, w, P, T);

  SolveResult r;
  r.status = status;
  r.fast_stop = fast_stop;
  r.iterations = iterations;
  r.evaluations = st.evaluations;
  r.lbfgs_ms = lbfgs_ms;
  r.metric_ms = metric_ms;
  r.cost = f;
  r.duration = T.sum();
  r.max_viol = ev.max_viol;
  r.metric_ok = metric_ok;
  return r;
}

std::string num(double v, int d = 2)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(d) << v;
  return oss.str();
}

struct Agg
{
  int n{0};
  double lbfgs_ms{0.0};
  double metric_ms{0.0};
  double iters{0.0};
  double evals{0.0};
  double cost{0.0};
  double viol{0.0};
  double duration{0.0};
  int fast{0};
  int worse{0};
};

void add(Agg &a, const SolveResult &r)
{
  a.n += 1;
  a.lbfgs_ms += r.lbfgs_ms;
  a.metric_ms += r.metric_ms;
  a.iters += static_cast<double>(r.iterations);
  a.evals += static_cast<double>(r.evaluations);
  a.cost += r.cost;
  a.viol += r.max_viol;
  a.duration += r.duration;
  a.fast += r.fast_stop ? 1 : 0;
}

double seedGradientError(bool use_mce)
{
  const Snapshot snap = makeSnapshot(0, 8);
  const Weights w;
  SolverState st;
  st.snap = &snap;
  st.w = w;
  VectorXd x(snap.pieces + snap.spatial());
  for (int i = 0; i < snap.pieces; ++i)
  {
    x(i) = st.time_map.toTau(snap.times(i));
  }
  x.segment(snap.pieces, snap.spatial()) = flatten(snap.P0);
  minco::FrozenMceWhitening whitening;
  if (use_mce)
  {
    Trajectory seed;
    require(seed.generate(snap.P0, snap.head, snap.tail, snap.times),
            "FD seed generate failed");
    minco::MincoMetric<3, 4> metric;
    minco::MincoMetricOptions options;
    options.mode = minco::MincoMetricMode::kFrozenWaypoint;
    options.energy_weight = w.energy;
    metric.setOptions(options);
    require(metric.update(seed), "FD G_MCE failed");
    require(whitening.configureKronecker(snap.pieces, 3, flatten(snap.P0),
                                         metric.scalarWaypointMetric()),
            "FD whitening failed");
    st.whitening = &whitening;
    require(whitening.encodeInPlace(x), "FD encode failed");
  }
  VectorXd g = VectorXd::Zero(x.size());
  const double f0 = costFn(&st, x, g);
  double max_rel = 0.0;
  for (int i = 0; i < x.size(); ++i)
  {
    const double h = 1.0e-6 * std::max(1.0, std::abs(x(i)));
    VectorXd xp = x;
    VectorXd xm = x;
    xp(i) += h;
    xm(i) -= h;
    VectorXd gp, gm;
    const double fp = costFn(&st, xp, gp);
    const double fm = costFn(&st, xm, gm);
    const double fd = (fp - fm) / (2.0 * h);
    const double rel =
        std::abs(fd - g(i)) / std::max({1.0, std::abs(fd), std::abs(g(i))});
    max_rel = std::max(max_rel, rel);
    (void)f0;
  }
  return max_rel;
}

} // namespace

void printAgg(const char *title, const Agg &ae, const Agg &am)
{
  const double inv = 1.0 / static_cast<double>(std::max(1, ae.n));
  const double e_lbfgs = ae.lbfgs_ms * inv;
  const double m_lbfgs = am.lbfgs_ms * inv;
  const double m_opt = (am.lbfgs_ms + am.metric_ms) * inv;
  std::cout << "\n=== " << title << " (n=" << ae.n << ") ===\n";
  std::cout << "                 Euclidean     Frozen MCE      ratio\n";
  std::cout << "L-BFGS ms/call   " << num(e_lbfgs, 3) << "         "
            << num(m_lbfgs, 3) << "         "
            << num(m_lbfgs / std::max(1.0e-9, e_lbfgs), 3) << "x\n";
  std::cout << "opt+metric ms    " << num(e_lbfgs, 3) << "         "
            << num(m_opt, 3) << "         "
            << num(m_opt / std::max(1.0e-9, e_lbfgs), 3) << "x\n";
  std::cout << "metric ms/call                   "
            << num(am.metric_ms * inv, 3) << "\n";
  std::cout << "iterations       " << num(ae.iters * inv, 1) << "          "
            << num(am.iters * inv, 1) << "          "
            << num((am.iters * inv) / std::max(1.0e-9, ae.iters * inv), 3)
            << "x\n";
  std::cout << "evaluations      " << num(ae.evals * inv, 1) << "          "
            << num(am.evals * inv, 1) << "          "
            << num((am.evals * inv) / std::max(1.0e-9, ae.evals * inv), 3)
            << "x\n";
  std::cout << "terminal J       " << num(ae.cost * inv, 3) << "         "
            << num(am.cost * inv, 3) << "         "
            << num((am.cost * inv) / std::max(1.0e-9, ae.cost * inv), 3)
            << "x\n";
  std::cout << "max viol         " << num(ae.viol * inv, 4) << "        "
            << num(am.viol * inv, 4) << "\n";
  std::cout << "duration s       " << num(ae.duration * inv, 3) << "         "
            << num(am.duration * inv, 3) << "\n";
  std::cout << "fast-stop rate   " << num(100.0 * ae.fast * inv, 1)
            << "%         " << num(100.0 * am.fast * inv, 1) << "%\n";
  std::cout << "L-BFGS sum ms    " << num(ae.lbfgs_ms, 1) << "         "
            << num(am.lbfgs_ms, 1) << "         speedup="
            << num(ae.lbfgs_ms / std::max(1.0e-9, am.lbfgs_ms), 2) << "x\n";
  std::cout << "quality regressions (cost+viol): " << am.worse << "/" << ae.n
            << "\n";
}

void runSuite(const char *title, StopMode mode, int replans, int pieces,
              bool require_matched_quality, bool require_faster)
{
  const Weights w;
  Agg ae, am;
  std::cout << "\n----- " << title << " -----\n";
  std::cout << std::setw(4) << "k" << std::setw(8) << "e_st" << std::setw(8)
            << "m_st" << std::setw(8) << "e_it" << std::setw(8) << "m_it"
            << std::setw(10) << "e_ms" << std::setw(10) << "m_ms" << std::setw(12)
            << "e_J" << std::setw(12) << "m_J" << std::setw(10) << "e_viol"
            << std::setw(10) << "m_viol"
            << "\n";
  for (int k = 0; k < replans; ++k)
  {
    const Snapshot snap = makeSnapshot(k, pieces);
    const SolveResult r0 = solveSnapshot(snap, w, false, mode);
    const SolveResult r1 = solveSnapshot(snap, w, true, mode);
    require(r1.metric_ok, "frozen MCE was not configured");
    require(std::isfinite(r0.cost) && std::isfinite(r1.cost),
            "non-finite terminal cost");
    add(ae, r0);
    add(am, r1);
    if (r1.cost > 1.15 * r0.cost && r1.max_viol > r0.max_viol + 0.02)
    {
      am.worse += 1;
    }
    std::cout << std::setw(4) << k << std::setw(8) << r0.status << std::setw(8)
              << r1.status << std::setw(10) << r0.iterations << std::setw(10)
              << r1.iterations << std::setw(10) << num(r0.lbfgs_ms)
              << std::setw(10) << num(r1.lbfgs_ms) << std::setw(12)
              << num(r0.cost, 3) << std::setw(12) << num(r1.cost, 3)
              << std::setw(10) << num(r0.max_viol, 3) << std::setw(10)
              << num(r1.max_viol, 3) << "\n";
  }
  printAgg(title, ae, am);
  if (require_matched_quality)
  {
    require(am.cost < 1.15 * ae.cost,
            "Frozen MCE mean cost exceeded Euclidean by more than 15%");
    require(am.viol < ae.viol + 0.02 * static_cast<double>(ae.n),
            "Frozen MCE mean corridor violation got worse");
    require(am.worse == 0, "Frozen MCE degraded cost and corridor together");
  }
  if (require_faster)
  {
    require(am.iters <= ae.iters,
            "Frozen MCE used more mean iterations than Euclidean");
    require(am.lbfgs_ms < ae.lbfgs_ms,
            "Frozen MCE L-BFGS wall time was not lower than Euclidean");
  }
}

int main()
{
  try
  {
    std::cout << "State2State-like rolling replan A/B\n";
    std::cout << "chart = QuadInvTimeMap + Frozen MCE whitening + Fast L-BFGS\n";
    std::cout << "objective = energy + time + vel/acc + corridor (free time)\n";

    const double fd_e = seedGradientError(false);
    const double fd_m = seedGradientError(true);
    std::cout << "seed FD gradient rel-error Euclidean=" << fd_e
              << " FrozenMCE=" << fd_m << "\n";
    require(fd_e < 5.0e-4, "Euclidean seed gradient failed finite differences");
    require(fd_m < 5.0e-4, "Frozen MCE seed gradient failed finite differences");

    // Warm the CPU so the first timed snapshot is not an outlier.
    {
      const Snapshot warm = makeSnapshot(0, 8);
      solveSnapshot(warm, Weights{}, false, StopMode::ProductionFast);
    }

    runSuite("Production Fast L-BFGS (phase0 guards off)",
             StopMode::ProductionFast, 16, 8, false, false);
    runSuite("Frozen MCE + physical waypoint/time guards",
             StopMode::PhysicalGuards, 16, 8, false, false);
    runSuite("Full convergence (classic L-BFGS, g_eps=0, delta=1e-12)",
             StopMode::FullConvergence, 8, 8, false, false);

    std::cout << "\n[minco_state2state_rolling_replan_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[minco_state2state_rolling_replan_self_test] FAIL: "
              << ex.what() << "\n";
    return 1;
  }
}
