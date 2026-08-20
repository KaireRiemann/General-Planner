/**
 * Level D/E/F-proxy: free-time L-BFGS on frozen joint charts.
 *
 * D0 Euclidean
 * D1 waypoint-only Frozen MCE (V1)
 * D2 block space-time frozen
 * D3 full joint frozen (control GN + G_T^{rel})
 * D4 D3 + active corridor GN
 *
 * Tight convergence first (Gate 5), then one metric refresh (Level E),
 * then production Fast L-BFGS physical-stop proxy (Level F).
 */

#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/minco/minco_joint_whitening.hpp"
#include "traj_opt/minco/minco_metric.hpp"
#include "traj_opt/minco/minco_whitening.hpp"
#include "utils/optimization/fast_lbfgs.hpp"
#include "utils/optimization/lbfgs.h"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
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

std::string sci(double v)
{
  std::ostringstream oss;
  oss << std::scientific << std::setprecision(3) << v;
  return oss.str();
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
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  double y_max{0.10};
  const char *label{"rolling"};
  int inner() const { return std::max(0, pieces - 1); }
  int spatial() const { return 3 * inner(); }
};

struct Weights
{
  double energy{1.0};
  double time{20.0};
  double vel{0.25};
  double acc{0.08};
  double corr{25.0};
  int samples{8};
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
  for (int i = 0; i < s.inner(); ++i)
  {
    const double a = static_cast<double>(i + 1) / static_cast<double>(pieces);
    const double x = x0 + a * (xg - x0);
    Vector3d p = nominalPos(x);
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

Snapshot makeUneven(int pieces, double ratio)
{
  Snapshot s = makeSnapshot(0, pieces);
  s.times = VectorXd::LinSpaced(pieces, 1.0, ratio);
  s.times *= (static_cast<double>(pieces) / s.times.sum()) * (70.0 / 8.0);
  s.label = "uneven";
  return s;
}

struct EvalOut
{
  double cost{0.0};
  double energy{0.0};
  double time_cost{0.0};
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
  CoeffMat gdC = CoeffMat::Zero(Trajectory::COEFF_NUM * snap.pieces, 3);
  VectorXd gdT = VectorXd::Zero(snap.pieces);
  double energy = 0.0;
  CoeffMat gdC_e;
  VectorXd gdT_e;
  traj.getEnergyPartialGradByCoeffs(energy, gdC_e);
  traj.getEnergyPartialGradByTimes(gdT_e);
  out.energy = w.energy * energy;
  out.time_cost = w.time * T.sum();
  out.cost = out.energy + out.time_cost;
  gdC = w.energy * gdC_e;
  gdT = w.energy * gdT_e;
  gdT.array() += w.time;

  const auto &C = traj.getCoefficients();
  for (int i = 0; i < snap.pieces; ++i)
  {
    const double Ti = T(i);
    const double inv_K = 1.0 / static_cast<double>(w.samples);
    const double dt = Ti * inv_K;
    const auto coeff =
        C.block<Trajectory::COEFF_NUM, 3>(i * Trajectory::COEFF_NUM, 0);
    for (int k = 0; k <= w.samples; ++k)
    {
      const double alpha = static_cast<double>(k) * inv_K;
      const double t_local = alpha * Ti;
      const double trap = (k == 0 || k == w.samples) ? 0.5 : 1.0;
      const double common = trap * dt;
      typename Trajectory::BasisRow b_p, b_v, b_a, b_j, b_s;
      Trajectory::computeBasisFunctions(t_local, b_p, b_v, b_a, b_j, b_s);
      const Vector3d p = (b_p * coeff).transpose();
      const Vector3d v = (b_v * coeff).transpose();
      const Vector3d a = (b_a * coeff).transpose();
      const Vector3d j = (b_j * coeff).transpose();
      Vector3d gp = Vector3d::Zero();
      Vector3d gv = Vector3d::Zero();
      Vector3d ga = Vector3d::Zero();
      double c_val = 0.0;
      c_val += 0.5 * w.vel * v.squaredNorm();
      gv += w.vel * v;
      c_val += 0.5 * w.acc * a.squaredNorm();
      ga += w.acc * a;
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
  }
  InnerPoints gP;
  VectorXd gT;
  traj.propagateGrad(gdC, gdT, gP, gT);
  out.gP = flatten(gP);
  out.gT = gT;
  return out;
}

enum class ChartKind
{
  Euclidean,
  WaypointOnly,
  BlockSpaceTime,
  FullJoint,
  FullJointGN
};

struct SolverState
{
  const Snapshot *snap{nullptr};
  Weights w;
  temporal_map::QuadInvTimeMap time_map;
  ChartKind kind{ChartKind::Euclidean};
  minco::FrozenMceWhitening *waypoint{nullptr};
  minco::FrozenJointWhitening *joint{nullptr};
  std::size_t evaluations{0};
  std::size_t iterations{0};
};

void decodePhysical(const SolverState &st, const VectorXd &x, VectorXd &T,
                    VectorXd &P)
{
  VectorXd chart = x;
  if (st.joint != nullptr && st.joint->ready())
  {
    require(st.joint->toChart(x, chart), "joint decode failed");
  }
  T.resize(st.snap->pieces);
  for (int i = 0; i < st.snap->pieces; ++i)
  {
    T(i) = st.time_map.toTime(chart(i));
  }
  P = chart.segment(st.snap->pieces, st.snap->spatial());
  if (st.waypoint != nullptr && st.waypoint->ready())
  {
    VectorXd phys;
    require(st.waypoint->toChart(P, phys), "waypoint decode failed");
    P = phys;
  }
}

double costFn(void *ptr, const VectorXd &x, VectorXd &g)
{
  auto *st = static_cast<SolverState *>(ptr);
  ++st->evaluations;
  VectorXd T, P;
  decodePhysical(*st, x, T, P);
  const EvalOut ev = evaluatePhysical(*st->snap, st->w, P, T);
  VectorXd gx = VectorXd::Zero(x.size());
  VectorXd chart = x;
  if (st->joint != nullptr && st->joint->ready())
  {
    require(st->joint->toChart(x, chart), "joint chart for grad failed");
  }
  for (int i = 0; i < st->snap->pieces; ++i)
  {
    gx(i) = st->time_map.backward(chart(i), T(i), ev.gT(i));
  }
  gx.segment(st->snap->pieces, st->snap->spatial()) = ev.gP;
  if (st->waypoint != nullptr && st->waypoint->ready())
  {
    require(st->waypoint->transformCovectorInPlace(gx), "g_z waypoint failed");
  }
  if (st->joint != nullptr && st->joint->ready())
  {
    if (!st->joint->transformCovector(gx, g) || !g.allFinite())
    {
      g.setZero();
      return std::numeric_limits<double>::infinity();
    }
  }
  else
  {
    g = gx;
  }
  if (!std::isfinite(ev.cost) || !g.allFinite())
  {
    g.setZero();
    return std::numeric_limits<double>::infinity();
  }
  return ev.cost;
}

int countIters(void *ptr, const VectorXd &, const VectorXd &, const double,
               const double, const int k, const int)
{
  static_cast<SolverState *>(ptr)->iterations = static_cast<std::size_t>(k);
  return 0;
}

math_utils::FastLbfgs::PhysicalSnapshot snapshotFn(void *ptr, const VectorXd &x)
{
  auto *st = static_cast<SolverState *>(ptr);
  VectorXd T, P;
  decodePhysical(*st, x, T, P);
  math_utils::FastLbfgs::PhysicalSnapshot snap;
  snap.durations = T;
  snap.waypoints = unflatten(P, st->snap->inner());
  return snap;
}

MatrixXd corridorGN(const Snapshot &snap, const Weights &w, const VectorXd &P,
                    const VectorXd &T)
{
  const int m = snap.pieces;
  const int pdim = snap.spatial();
  const int n = m + pdim;
  Trajectory traj;
  require(traj.generate(unflatten(P, snap.inner()), snap.head, snap.tail, T),
          "corridor generate failed");
  std::vector<CoeffMat> tangents(static_cast<std::size_t>(n));
  for (int a = 0; a < n; ++a)
  {
    InnerPoints dP = InnerPoints::Zero(3, snap.inner());
    VectorXd dT = VectorXd::Zero(m);
    if (a < m)
    {
      dT(a) = 1.0;
    }
    else
    {
      const int k = a - m;
      dP(k % 3, k / 3) = 1.0;
    }
    require(traj.propagateTangent(dP, dT, tangents[static_cast<std::size_t>(a)]),
            "corridor tangent failed");
  }
  MatrixXd G = MatrixXd::Zero(n, n);
  const auto &C = traj.getCoefficients();
  for (int i = 0; i < m; ++i)
  {
    const double Ti = T(i);
    const double inv_K = 1.0 / static_cast<double>(w.samples);
    const auto coeff =
        C.block<Trajectory::COEFF_NUM, 3>(i * Trajectory::COEFF_NUM, 0);
    for (int k = 0; k <= w.samples; ++k)
    {
      const double alpha = static_cast<double>(k) * inv_K;
      typename Trajectory::BasisRow b_p, b_v, b_a, b_j, b_s;
      Trajectory::computeBasisFunctions(alpha * Ti, b_p, b_v, b_a, b_j, b_s);
      const Vector3d p = (b_p * coeff).transpose();
      const double viol = std::abs(p.y()) - snap.y_max;
      if (viol <= 0.0)
      {
        continue;
      }
      const double sgn = p.y() >= 0.0 ? 1.0 : -1.0;
      VectorXd jr(n);
      for (int a = 0; a < n; ++a)
      {
        const auto dC =
            tangents[static_cast<std::size_t>(a)]
                .block<Trajectory::COEFF_NUM, 3>(i * Trajectory::COEFF_NUM, 0);
        Vector3d dp = (b_p * dC).transpose();
        if (a == i)
        {
          dp += alpha * Vector3d((b_v * coeff).transpose());
        }
        jr(a) = sgn * dp.y();
      }
      G.noalias() += w.corr * jr * jr.transpose();
    }
  }
  return G;
}

struct BuiltMetric
{
  MatrixXd Gx;
  double build_ms{0.0};
};

BuiltMetric buildSolverMetric(const Snapshot &snap, const Weights &w,
                              ChartKind kind, const VectorXd &T,
                              const VectorXd &P,
                              temporal_map::QuadInvTimeMap &tmap)
{
  BuiltMetric out;
  const auto t0 = Clock::now();
  const int m = snap.pieces;
  VectorXd tau(m);
  VectorXd dtau(m);
  for (int i = 0; i < m; ++i)
  {
    tau(i) = tmap.toTau(T(i));
    dtau(i) = tmap.backward(tau(i), T(i), 1.0);
  }
  Trajectory seed;
  require(seed.generate(unflatten(P, snap.inner()), snap.head, snap.tail, T),
          "metric seed failed");

  if (kind == ChartKind::Euclidean)
  {
    out.Gx = MatrixXd::Identity(m + snap.spatial(), m + snap.spatial());
  }
  else if (kind == ChartKind::WaypointOnly)
  {
    minco::MincoMetricOptions opt;
    opt.mode = minco::MincoMetricMode::kFrozenWaypoint;
    opt.regularization = 1.0e-10;
    opt.energy_weight = w.energy;
    minco::MincoMetric<3, 4> metric;
    metric.setOptions(opt);
    require(metric.update(seed), "G_PP failed");
    out.Gx = MatrixXd::Identity(m + snap.spatial(), m + snap.spatial());
    out.Gx.bottomRightCorner(snap.spatial(), snap.spatial()) =
        metric.waypointMetric();
  }
  else if (kind == ChartKind::BlockSpaceTime)
  {
    minco::MincoMetricOptions opt;
    opt.mode = minco::MincoMetricMode::kBlockSpaceTime;
    opt.regularization = 1.0e-10;
    opt.time_metric_weight = 1.0;
    opt.energy_weight = w.energy;
    minco::MincoMetric<3, 4> metric;
    metric.setOptions(opt);
    require(metric.update(seed), "block metric failed");
    require(minco::pullbackTimeMap(metric.spaceTimeMetric(), dtau, out.Gx),
            "block pullback failed");
  }
  else
  {
    minco::MincoMetricOptions opt;
    opt.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
    opt.regularization = 1.0e-10;
    opt.time_metric_weight = 1.0;
    opt.energy_weight = w.energy;
    minco::MincoMetric<3, 4> metric;
    metric.setOptions(opt);
    require(metric.update(seed), "full metric failed");
    require(minco::pullbackTimeMap(metric.spaceTimeMetric(), dtau, out.Gx),
            "full pullback failed");
    if (kind == ChartKind::FullJointGN)
    {
      MatrixXd Gc;
      require(minco::pullbackTimeMap(corridorGN(snap, w, P, T), dtau, Gc),
              "corr pullback failed");
      out.Gx += Gc;
    }
  }
  if (kind != ChartKind::Euclidean && out.Gx.size() > 0)
  {
    out.Gx.diagonal().array() +=
        1.0e-8 * std::max(1.0, std::abs(out.Gx.trace())) /
        static_cast<double>(std::max(1, static_cast<int>(out.Gx.rows())));
  }
  out.build_ms = 1.0e3 * std::chrono::duration<double>(Clock::now() - t0).count();
  return out;
}

bool configureJoint(minco::FrozenJointWhitening &joint, int time_dim,
                    const VectorXd &x, const MatrixXd &G)
{
  return joint.configureBlockSchur(time_dim, x, G) ||
         joint.configureDense(x, G);
}

struct SolveResult
{
  const char *name;
  int status{0};
  std::size_t iters{0};
  std::size_t evals{0};
  double J{0.0};
  double viol{0.0};
  double lbfgs_ms{0.0};
  double metric_ms{0.0};
  double duration{0.0};
  bool fast_stop{false};
};

SolveResult runTight(const Snapshot &snap, const Weights &w, ChartKind kind,
                     const char *name, int max_iters)
{
  SolverState st;
  st.snap = &snap;
  st.w = w;
  st.kind = kind;

  VectorXd x(snap.pieces + snap.spatial());
  for (int i = 0; i < snap.pieces; ++i)
  {
    x(i) = st.time_map.toTau(snap.times(i));
  }
  x.tail(snap.spatial()) = flatten(snap.P0);

  minco::FrozenMceWhitening waypoint;
  minco::FrozenJointWhitening joint;
  const BuiltMetric built =
      buildSolverMetric(snap, w, kind, snap.times, flatten(snap.P0), st.time_map);
  if (kind == ChartKind::WaypointOnly)
  {
    require(waypoint.configure(snap.pieces, flatten(snap.P0),
                               built.Gx.bottomRightCorner(snap.spatial(),
                                                          snap.spatial())),
            "waypoint configure failed");
    st.waypoint = &waypoint;
    require(waypoint.encodeInPlace(x), "waypoint encode failed");
  }
  else if (kind != ChartKind::Euclidean)
  {
    if (!joint.configureBlockSchur(snap.pieces, x, built.Gx) &&
        !joint.configureDense(x, built.Gx))
    {
      throw std::runtime_error(std::string(name) + " joint configure failed");
    }
    st.joint = &joint;
    require(joint.encodeInPlace(x), "joint encode failed");
  }

  st.evaluations = 0;
  st.iterations = 0;
  math_utils::lbfgs::lbfgs_parameter_t param;
  param.mem_size = 16;
  param.g_epsilon = 1.0e-6;
  param.past = 3;
  param.delta = 1.0e-12;
  param.max_iterations = max_iters;
  param.max_linesearch = 64;
  double f = 0.0;
  const auto t0 = Clock::now();
  const int status = math_utils::lbfgs::lbfgs_optimize(
      x, f, &costFn, nullptr, &countIters, &st, param);
  const double lbfgs_ms =
      1.0e3 * std::chrono::duration<double>(Clock::now() - t0).count();

  VectorXd T, P;
  decodePhysical(st, x, T, P);
  const EvalOut ev = evaluatePhysical(snap, w, P, T);
  SolveResult r;
  r.name = name;
  r.status = status;
  r.iters = st.iterations;
  r.evals = st.evaluations;
  r.J = ev.cost;
  r.viol = ev.max_viol;
  r.lbfgs_ms = lbfgs_ms;
  r.metric_ms = built.build_ms;
  r.duration = T.sum();
  return r;
}

SolveResult runRefresh(const Snapshot &snap, const Weights &w, int refreshes)
{
  SolverState st;
  st.snap = &snap;
  st.w = w;
  st.kind = ChartKind::FullJoint;

  VectorXd physical_tau(snap.pieces);
  for (int i = 0; i < snap.pieces; ++i)
  {
    physical_tau(i) = st.time_map.toTau(snap.times(i));
  }
  VectorXd physical_P = flatten(snap.P0);
  VectorXd T = snap.times;
  VectorXd P = physical_P;

  SolveResult r;
  r.name = refreshes == 0 ? "E0 no refresh" : (refreshes == 1 ? "E1 1 refresh" : "E2 2 refresh");
  const int inner_iters = refreshes == 0 ? 500 : 250;
  for (int outer = 0; outer <= refreshes; ++outer)
  {
    VectorXd x(snap.pieces + snap.spatial());
    x.head(snap.pieces) = physical_tau;
    x.tail(snap.spatial()) = physical_P;
    const BuiltMetric built =
        buildSolverMetric(snap, w, ChartKind::FullJoint, T, P, st.time_map);
    minco::FrozenJointWhitening joint;
    if (!configureJoint(joint, snap.pieces, x, built.Gx))
    {
      throw std::runtime_error("refresh configure failed");
    }
    st.joint = &joint;
    require(joint.encodeInPlace(x), "refresh encode failed");
    st.evaluations = 0;
    st.iterations = 0;
    math_utils::lbfgs::lbfgs_parameter_t param;
    param.mem_size = 16;
    param.g_epsilon = 1.0e-6;
    param.past = 3;
    param.delta = 1.0e-12;
    param.max_iterations = inner_iters;
    param.max_linesearch = 64;
    double f = 0.0;
    const auto t0 = Clock::now();
    r.status = math_utils::lbfgs::lbfgs_optimize(
        x, f, &costFn, nullptr, &countIters, &st, param);
    r.lbfgs_ms +=
        1.0e3 * std::chrono::duration<double>(Clock::now() - t0).count();
    r.metric_ms += built.build_ms;
    r.iters += st.iterations;
    r.evals += st.evaluations;
    decodePhysical(st, x, T, P);
    for (int i = 0; i < snap.pieces; ++i)
    {
      physical_tau(i) = st.time_map.toTau(T(i));
    }
    physical_P = P;
  }
  const EvalOut ev = evaluatePhysical(snap, w, P, T);
  r.J = ev.cost;
  r.viol = ev.max_viol;
  r.duration = T.sum();
  return r;
}

SolveResult runFast(const Snapshot &snap, const Weights &w, ChartKind kind,
                    const char *name)
{
  SolverState st;
  st.snap = &snap;
  st.w = w;
  st.kind = kind;
  VectorXd x(snap.pieces + snap.spatial());
  for (int i = 0; i < snap.pieces; ++i)
  {
    x(i) = st.time_map.toTau(snap.times(i));
  }
  x.tail(snap.spatial()) = flatten(snap.P0);
  minco::FrozenMceWhitening waypoint;
  minco::FrozenJointWhitening joint;
  const BuiltMetric built =
      buildSolverMetric(snap, w, kind, snap.times, flatten(snap.P0), st.time_map);
  if (kind == ChartKind::WaypointOnly)
  {
    require(waypoint.configure(snap.pieces, flatten(snap.P0),
                               built.Gx.bottomRightCorner(snap.spatial(),
                                                          snap.spatial())),
            "fast waypoint configure failed");
    st.waypoint = &waypoint;
    require(waypoint.encodeInPlace(x), "fast waypoint encode failed");
  }
  else if (kind != ChartKind::Euclidean)
  {
    require(configureJoint(joint, snap.pieces, x, built.Gx),
            "fast joint configure failed");
    st.joint = &joint;
    require(joint.encodeInPlace(x), "fast joint encode failed");
  }

  math_utils::FastLbfgs solver;
  math_utils::FastLbfgs::Options opt;
  opt.mem_size = 32;
  opt.step_bound_enabled = false;
  opt.early_stop_enabled = true;
  opt.g_epsilon = 0.0;
  opt.rel_cost = 1.0e-3;
  opt.rel_step = 0.0;
  opt.rel_time = 2.0e-2;
  opt.rel_waypoint = 2.0e-2;
  opt.min_iterations = 10;
  opt.phase0_guards_en = true;
  opt.window = 3;
  solver.setOptions(opt);
  double f = 0.0;
  const auto t0 = Clock::now();
  const int status =
      solver.run(x, f, &costFn, nullptr, &st, &snapshotFn, true, &x, nullptr);
  const double lbfgs_ms =
      1.0e3 * std::chrono::duration<double>(Clock::now() - t0).count();
  VectorXd T, P;
  decodePhysical(st, x, T, P);
  const EvalOut ev = evaluatePhysical(snap, w, P, T);
  SolveResult r;
  r.name = name;
  r.status = status;
  r.iters = solver.report().iterations;
  r.evals = st.evaluations;
  r.J = ev.cost;
  r.viol = ev.max_viol;
  r.lbfgs_ms = lbfgs_ms;
  r.metric_ms = built.build_ms;
  r.duration = T.sum();
  r.fast_stop = solver.acceptedFastStop();
  return r;
}

void printSolve(const SolveResult &r)
{
  std::cout << "  " << std::setw(22) << r.name
            << "  J=" << std::setw(10) << std::fixed << std::setprecision(2)
            << r.J
            << "  viol=" << sci(r.viol)
            << "  it=" << std::setw(4) << r.iters
            << "  ev=" << std::setw(4) << r.evals
            << "  lbfgs_ms=" << std::setw(7) << std::setprecision(2) << r.lbfgs_ms
            << "  met_ms=" << std::setw(6) << std::setprecision(3) << r.metric_ms
            << "  T=" << std::setprecision(3) << r.duration
            << "  st=" << r.status
            << (r.fast_stop ? "  fast" : "")
            << "\n";
}

} // namespace

int main()
{
  try
  {
    const Weights w;
    std::cout << "Level D/E/F-proxy  free-time frozen joint L-BFGS\n\n";

    auto runSuite = [&](const Snapshot &snap) {
      std::cout << "==== " << snap.label << " M=" << snap.pieces
                << " Tmax/Tmin="
                << sci(snap.times.maxCoeff() / snap.times.minCoeff())
                << " ====\n";
      std::cout << "tight L-BFGS  gε=1e-6  max=500\n";
      const SolveResult d0 =
          runTight(snap, w, ChartKind::Euclidean, "D0 Euclidean", 500);
      const SolveResult d1 =
          runTight(snap, w, ChartKind::WaypointOnly, "D1 waypoint V1", 500);
      const SolveResult d2 =
          runTight(snap, w, ChartKind::BlockSpaceTime, "D2 block ST", 500);
      const SolveResult d3 =
          runTight(snap, w, ChartKind::FullJoint, "D3 full joint", 500);
      const SolveResult d4 =
          runTight(snap, w, ChartKind::FullJointGN, "D4 full+GN", 500);
      printSolve(d0);
      printSolve(d1);
      printSolve(d2);
      printSolve(d3);
      printSolve(d4);
      const double den = std::max(1.0, std::abs(d0.J));
      const double rel_d3 = std::abs(d3.J - d0.J) / den;
      const double rel_d2 = std::abs(d2.J - d0.J) / den;
      if (rel_d3 > 1.0e-2 || rel_d2 > 1.0e-2)
      {
        std::cout << "  note: J* alignment loose  ΔD3=" << sci(rel_d3)
                  << "  ΔD2=" << sci(rel_d2) << "\n";
      }
      std::cout << "  ΔJ D3/D0=" << sci(std::abs(d3.J - d0.J) / den)
                << "  iter D3/D0=" << sci(static_cast<double>(d3.iters) /
                                          std::max<std::size_t>(1, d0.iters))
                << "  wall D3/D0=" << sci((d3.lbfgs_ms + d3.metric_ms) /
                                          std::max(1.0e-9, d0.lbfgs_ms))
                << "\n";

      std::cout << "metric refresh on D3 chart\n";
      printSolve(runRefresh(snap, w, 0));
      printSolve(runRefresh(snap, w, 1));
      printSolve(runRefresh(snap, w, 2));

      std::cout << "Fast L-BFGS physical stop (rel_step=0)\n";
      printSolve(runFast(snap, w, ChartKind::Euclidean, "F0 Euclidean"));
      printSolve(runFast(snap, w, ChartKind::WaypointOnly, "F1 waypoint V1"));
      printSolve(runFast(snap, w, ChartKind::FullJoint, "F2 full joint"));
      printSolve(runFast(snap, w, ChartKind::FullJointGN, "F3 full+GN"));
      std::cout << "\n";
    };

    const auto safeSuite = [&](const Snapshot &snap) {
      try
      {
        runSuite(snap);
      }
      catch (const std::exception &ex)
      {
        std::cout << "suite failed (" << snap.label << " M=" << snap.pieces
                  << "): " << ex.what() << "\n\n";
      }
    };
    safeSuite(makeSnapshot(0, 8));
    safeSuite(makeSnapshot(4, 8));
    safeSuite(makeUneven(8, 8.0));
    safeSuite(makeSnapshot(0, 12));

    std::cout << "==== C3 vs C4 solver switching (tight max=200) ====\n";
    auto switching = [&](const Snapshot &snap, const Weights &ww,
                         const char *tag) {
      std::cout << "-- " << tag << "  Tmax/Tmin="
                << sci(snap.times.maxCoeff() / snap.times.minCoeff())
                << "  w_c=" << sci(ww.corr) << "\n";
      const SolveResult c3 =
          runTight(snap, ww, ChartKind::FullJoint, "C3 full GN", 200);
      const SolveResult c4 =
          runTight(snap, ww, ChartKind::FullJointGN, "C4 +corr GN", 200);
      const SolveResult e1 = runRefresh(snap, ww, 1);
      printSolve(c3);
      printSolve(c4);
      printSolve(e1);
      const char *pick = "C3";
      double best = c3.J;
      if (c4.J < best * 0.995)
      {
        pick = "C4";
        best = c4.J;
      }
      if (e1.J < best * 0.995)
      {
        pick = "E1 refresh";
      }
      std::cout << "  recommend=" << pick << "\n";
    };
    {
      Weights w25 = w;
      w25.corr = 25.0;
      Weights w1e5 = w;
      w1e5.corr = 1.0e5;
      switching(makeSnapshot(0, 8), w25, "uniform M=8");
      switching(makeSnapshot(0, 8), w1e5, "uniform M=8 high w_c");
      switching(makeUneven(8, 8.0), w25, "uneven8 M=8");
      switching(makeUneven(8, 8.0), w1e5, "uneven8 M=8 high w_c");
    }

    std::cout << "\n==== Fast L-BFGS buckets (physical stop) ====\n";
    struct FastSample
    {
      double eu_ms{0.0};
      double jt_ms{0.0};
    };
    std::vector<FastSample> buckets;
    for (int pieces : {5, 8, 12})
    {
      for (double ratio : {1.0, 8.0})
      {
        const Snapshot snap =
            ratio <= 1.0 + 1.0e-12 ? makeSnapshot(0, pieces)
                                   : makeUneven(pieces, ratio);
        const SolveResult f0 =
            runFast(snap, w, ChartKind::Euclidean, "F0");
        const SolveResult f2 =
            runFast(snap, w, ChartKind::FullJoint, "F2");
        FastSample s;
        s.eu_ms = f0.lbfgs_ms + f0.metric_ms;
        s.jt_ms = f2.lbfgs_ms + f2.metric_ms;
        buckets.push_back(s);
        const double ratio_obs = snap.times.maxCoeff() / snap.times.minCoeff();
        std::cout << "  M=" << pieces
                  << "  ratio=" << sci(ratio_obs)
                  << "  F0 it=" << f0.iters << " ms=" << sci(s.eu_ms)
                  << " J=" << sci(f0.J)
                  << "  F2 it=" << f2.iters << " ms=" << sci(s.jt_ms)
                  << " J=" << sci(f2.J)
                  << "  wall F2/F0=" << sci(s.jt_ms / std::max(1.0e-9, s.eu_ms))
                  << "\n";
      }
    }
    auto percentile = [](std::vector<double> v, double p) {
      if (v.empty())
      {
        return std::numeric_limits<double>::quiet_NaN();
      }
      std::sort(v.begin(), v.end());
      const double idx = p * static_cast<double>(v.size() - 1);
      const int lo = static_cast<int>(std::floor(idx));
      const int hi = static_cast<int>(std::ceil(idx));
      const double a = v[static_cast<std::size_t>(lo)];
      const double b = v[static_cast<std::size_t>(hi)];
      const double t = idx - static_cast<double>(lo);
      return (1.0 - t) * a + t * b;
    };
    std::vector<double> ratios;
    for (const auto &s : buckets)
    {
      ratios.push_back(s.jt_ms / std::max(1.0e-9, s.eu_ms));
    }
    std::cout << "  wall F2/F0  P50=" << sci(percentile(ratios, 0.50))
              << "  P95=" << sci(percentile(ratios, 0.95))
              << "  n=" << buckets.size() << "\n";

    std::cout << "[minco_freetime_joint_whitening_comparison_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[minco_freetime_joint_whitening_comparison_self_test] FAIL: "
              << ex.what() << "\n";
    return 1;
  }
}
