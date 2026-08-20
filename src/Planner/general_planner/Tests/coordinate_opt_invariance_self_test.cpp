/**
 * Optimization-level coordinate invariance test.
 *
 * The basis-gradient test showed that one Euclidean step is coordinate
 * dependent. This file runs the same J from the same initial p(t) with:
 *   1. Euclidean L-BFGS in two coordinates
 *   2. L2 / MCE-preconditioned (whitened) L-BFGS in two coordinates
 *   3. Decision-space early stop (same rule as FastLbfgs rel_cost/rel_step)
 *
 * Expected:
 *   Euclidean paths differ. On this nonconvex J they can also finish at
 *   different local minima. Whitened / natural paths match at every iterate
 *   and therefore reach the same basin. Parameter-space early stop can freeze
 *   Euclidean runs at different p(t). On a more strongly convex MINCO L2
 *   problem, Euclidean finals match if run to a true critical point.
 */

#include "traj_opt/minco/minco_trajectory.hpp"
#include "utils/optimization/lbfgs.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Minco = minco::MINCOTrajectory<3, 4>;

constexpr double kPathTol = 1.0e-8;
constexpr double kFinalTol = 1.0e-6;
constexpr double kMismatchMin = 1.0e-3;
constexpr int kSampleN = 200;

void require(bool condition, const std::string &message)
{
  if (!condition)
  {
    throw std::runtime_error(message);
  }
}

std::string sci(double v, int digits = 6)
{
  std::ostringstream oss;
  oss << std::scientific << std::setprecision(digits) << v;
  return oss.str();
}

double binom(int n, int k)
{
  if (k < 0 || k > n)
  {
    return 0.0;
  }
  k = std::min(k, n - k);
  double out = 1.0;
  for (int i = 1; i <= k; ++i)
  {
    out *= static_cast<double>(n - k + i) / static_cast<double>(i);
  }
  return out;
}

MatrixXd powerToBernstein(int degree)
{
  const int n = degree + 1;
  MatrixXd T = MatrixXd::Zero(n, n);
  for (int j = 0; j < n; ++j)
  {
    for (int k = 0; k <= j; ++k)
    {
      T(j, k) = binom(j, k) / binom(degree, k);
    }
  }
  return T;
}

MatrixXd hilbertMass(int degree)
{
  const int n = degree + 1;
  MatrixXd G(n, n);
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      G(i, j) = 1.0 / static_cast<double>(i + j + 1);
    }
  }
  return G;
}

MatrixXd bernsteinMass(int degree)
{
  const int n = degree + 1;
  MatrixXd G(n, n);
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      G(i, j) = binom(degree, i) * binom(degree, j) /
                ((2.0 * degree + 1.0) * binom(2 * degree, i + j));
    }
  }
  return G;
}

double evalPower(const VectorXd &a, double t)
{
  double p = 0.0;
  double tk = 1.0;
  for (Eigen::Index k = 0; k < a.size(); ++k)
  {
    p += a(k) * tk;
    tk *= t;
  }
  return p;
}

double evalBernstein(const VectorXd &c, double t)
{
  const int degree = static_cast<int>(c.size()) - 1;
  double p = 0.0;
  for (int j = 0; j <= degree; ++j)
  {
    p += c(j) * binom(degree, j) * std::pow(t, j) *
         std::pow(1.0 - t, degree - j);
  }
  return p;
}

VectorXd bernsteinBasis(int degree, double t)
{
  VectorXd b(degree + 1);
  for (int j = 0; j <= degree; ++j)
  {
    b(j) = binom(degree, j) * std::pow(t, j) * std::pow(1.0 - t, degree - j);
  }
  return b;
}

VectorXd powerBasis(int degree, double t)
{
  VectorXd b(degree + 1);
  double tk = 1.0;
  for (int k = 0; k <= degree; ++k)
  {
    b(k) = tk;
    tk *= t;
  }
  return b;
}

MatrixXd spdSqrt(const MatrixXd &G)
{
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(G);
  require(es.info() == Eigen::Success, "SPD sqrt failed");
  const VectorXd s = es.eigenvalues().cwiseMax(0.0).cwiseSqrt();
  return es.eigenvectors() * s.asDiagonal() * es.eigenvectors().transpose();
}

MatrixXd spdInvSqrt(const MatrixXd &G)
{
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(G);
  require(es.info() == Eigen::Success, "SPD inv-sqrt failed");
  VectorXd s = es.eigenvalues();
  for (Eigen::Index i = 0; i < s.size(); ++i)
  {
    require(s(i) > 1.0e-14, "metric not SPD");
    s(i) = 1.0 / std::sqrt(s(i));
  }
  return es.eigenvectors() * s.asDiagonal() * es.eigenvectors().transpose();
}

double trajectoryRms(const VectorXd &a, const VectorXd &c, bool c_is_bernstein)
{
  double acc = 0.0;
  for (int k = 0; k <= kSampleN; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(kSampleN);
    const double pa = evalPower(a, t);
    const double pb = c_is_bernstein ? evalBernstein(c, t) : evalPower(c, t);
    const double d = pa - pb;
    acc += d * d;
  }
  return std::sqrt(acc / static_cast<double>(kSampleN + 1));
}

double polyObjective(const VectorXd &coeff, bool bernstein, double kappa,
                     double omega, VectorXd *grad)
{
  const int degree = static_cast<int>(coeff.size()) - 1;
  const MatrixXd G = bernstein ? bernsteinMass(degree) : hilbertMass(degree);
  double J = 0.5 * coeff.dot(G * coeff);
  if (grad != nullptr)
  {
    *grad = G * coeff;
  }
  for (int k = 0; k <= kSampleN; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(kSampleN);
    const double w =
        ((k == 0 || k == kSampleN) ? 0.5 : 1.0) / static_cast<double>(kSampleN);
    const double p =
        bernstein ? evalBernstein(coeff, t) : evalPower(coeff, t);
    J += kappa * w * std::cos(omega * p);
    if (grad != nullptr)
    {
      const double dp = -kappa * w * omega * std::sin(omega * p);
      *grad += dp * (bernstein ? bernsteinBasis(degree, t)
                               : powerBasis(degree, t));
    }
  }
  return J;
}

struct PolyProblem
{
  bool bernstein{false};
  bool whitened{false};
  bool early_stop{false};
  double kappa{0.0};
  double omega{6.0};
  double rel_cost{5.0e-2};
  double rel_step{8.0e-2};
  int min_iterations{3};
  int window{2};
  MatrixXd Winv;
  VectorXd previous_x;
  std::vector<VectorXd> iterates;
  std::vector<double> costs;

  VectorXd toPhysical(const VectorXd &x) const
  {
    return whitened ? (Winv * x) : x;
  }

  static double evaluate(void *ptr, const VectorXd &x, VectorXd &g)
  {
    auto *self = static_cast<PolyProblem *>(ptr);
    const VectorXd coeff = self->toPhysical(x);
    VectorXd g_coeff;
    const double J =
        polyObjective(coeff, self->bernstein, self->kappa, self->omega, &g_coeff);
    if (self->whitened)
    {
      g = self->Winv.transpose() * g_coeff;
    }
    else
    {
      g = g_coeff;
    }
    return J;
  }

  static int progress(void *ptr, const VectorXd &x, const VectorXd &,
                      const double fx, const double, const int k, const int)
  {
    auto *self = static_cast<PolyProblem *>(ptr);
    self->iterates.push_back(self->toPhysical(x));
    self->costs.push_back(fx);
    if (!self->early_stop)
    {
      self->previous_x = x;
      return 0;
    }

    double relative_step = std::numeric_limits<double>::infinity();
    if (self->previous_x.size() == x.size())
    {
      relative_step = (x - self->previous_x).lpNorm<Eigen::Infinity>() /
                      std::max(1.0, x.lpNorm<Eigen::Infinity>());
    }
    self->previous_x = x;

    const std::size_t history_limit =
        static_cast<std::size_t>(self->window + 1);
    if (k < self->min_iterations || self->costs.size() < history_limit)
    {
      return 0;
    }
    const double relative_cost =
        std::abs(self->costs[self->costs.size() - history_limit] - fx) /
        std::max(1.0, std::abs(fx));
    const bool stop = relative_cost <= self->rel_cost &&
                      relative_step <= self->rel_step;
    return stop ? 1 : 0;
  }
};

math_utils::lbfgs::lbfgs_parameter_t makeParams(int max_iters, bool full)
{
  math_utils::lbfgs::lbfgs_parameter_t p;
  p.mem_size = 8;
  p.max_iterations = max_iters;
  p.max_linesearch = 64;
  if (full)
  {
    p.g_epsilon = 1.0e-10;
    p.past = 3;
    p.delta = 1.0e-14;
  }
  else
  {
    p.g_epsilon = 0.0;
    p.past = 0;
    p.delta = 0.0;
  }
  return p;
}

struct RunRecord
{
  int status{0};
  int iters{0};
  double final_cost{0.0};
  std::vector<VectorXd> physical;
  std::vector<double> costs;
};

RunRecord runPolyLbfgs(const VectorXd &x0, bool bernstein, bool whitened,
                       const MatrixXd &G, double kappa, int max_iters,
                       bool full)
{
  PolyProblem problem;
  problem.bernstein = bernstein;
  problem.whitened = whitened;
  problem.kappa = kappa;
  if (whitened)
  {
    problem.Winv = spdInvSqrt(G);
  }
  VectorXd x = whitened ? (spdSqrt(G) * x0) : x0;
  problem.previous_x = x;
  problem.iterates.push_back(problem.toPhysical(x));
  VectorXd g_unused(x.size());
  problem.costs.push_back(PolyProblem::evaluate(&problem, x, g_unused));

  double f = 0.0;
  const int status = math_utils::lbfgs::lbfgs_optimize(
      x, f, &PolyProblem::evaluate, nullptr, &PolyProblem::progress, &problem,
      makeParams(max_iters, full));

  RunRecord rec;
  rec.status = status;
  rec.physical = problem.iterates;
  rec.costs = problem.costs;
  rec.iters = static_cast<int>(problem.iterates.size()) - 1;
  rec.final_cost = problem.costs.back();
  return rec;
}

double iterateRms(const RunRecord &a, const RunRecord &c, int k,
                  const MatrixXd &T)
{
  require(k >= 0, "iterate index");
  const int ka = std::min(k, static_cast<int>(a.physical.size()) - 1);
  const int kc = std::min(k, static_cast<int>(c.physical.size()) - 1);
  const VectorXd &ca = a.physical[static_cast<std::size_t>(ka)];
  const VectorXd &cc = c.physical[static_cast<std::size_t>(kc)];
  const VectorXd power_from_c = T.inverse() * cc;
  return trajectoryRms(ca, power_from_c, false);
}

VectorXd flattenPoints(const Minco::InnerPointsMat &P)
{
  return Eigen::Map<const VectorXd>(P.data(), P.size());
}

Minco::InnerPointsMat unflattenPoints(const VectorXd &v, int cols)
{
  Minco::InnerPointsMat P(3, cols);
  Eigen::Map<VectorXd>(P.data(), v.size()) = v;
  return P;
}

MatrixXd powerMassOnInterval(int coeff_num, double T)
{
  MatrixXd G(coeff_num, coeff_num);
  for (int i = 0; i < coeff_num; ++i)
  {
    for (int j = 0; j < coeff_num; ++j)
    {
      G(i, j) = std::pow(T, i + j + 1) / static_cast<double>(i + j + 1);
    }
  }
  return G;
}

struct MincoScene
{
  Minco::BoundaryState head = Minco::BoundaryState::Zero();
  Minco::BoundaryState tail = Minco::BoundaryState::Zero();
  Eigen::VectorXd durations;
  int inner_cols{2};
  double kappa{0.25};
  double omega{4.0};
};

Minco makeMinco(const MincoScene &scene, const VectorXd &Pvec)
{
  Minco traj;
  require(traj.generate(unflattenPoints(Pvec, scene.inner_cols), scene.head,
                        scene.tail, scene.durations),
          "MINCO generate failed");
  return traj;
}

double mincoObjective(const MincoScene &scene, const VectorXd &Pvec,
                      VectorXd *grad)
{
  const Minco traj = makeMinco(scene, Pvec);
  const auto &C = traj.getCoefficients();
  double J = 0.0;
  Minco::CoeffMat gdC(Minco::COEFF_NUM * traj.getPieceNum(), 3);
  gdC.setZero();
  for (int i = 0; i < traj.getPieceNum(); ++i)
  {
    const MatrixXd M = powerMassOnInterval(Minco::COEFF_NUM, traj.getDurations()(i));
    const auto Ci = C.block<Minco::COEFF_NUM, 3>(i * Minco::COEFF_NUM, 0);
    J += 0.5 * (Ci.transpose() * M * Ci).trace();
    gdC.block<Minco::COEFF_NUM, 3>(i * Minco::COEFF_NUM, 0) = M * Ci;
  }

  const double Ttot = traj.getTotalDuration();
  for (int k = 0; k <= kSampleN; ++k)
  {
    const double t = Ttot * static_cast<double>(k) / static_cast<double>(kSampleN);
    const double w = ((k == 0 || k == kSampleN) ? 0.5 : 1.0) * Ttot /
                     static_cast<double>(kSampleN);
    const Vector3d p = traj.getPos(t);
    J += scene.kappa * w * std::cos(scene.omega * p.x());
    const double dpx =
        -scene.kappa * w * scene.omega * std::sin(scene.omega * p.x());
    int piece = 0;
    double local = t;
    double acc = 0.0;
    for (int i = 0; i < traj.getPieceNum(); ++i)
    {
      if (t <= acc + traj.getDurations()(i) || i == traj.getPieceNum() - 1)
      {
        piece = i;
        local = std::min(std::max(t - acc, 0.0), traj.getDurations()(i));
        break;
      }
      acc += traj.getDurations()(i);
    }
    const auto basis = Minco::derivativeBasis(0, local);
    gdC.block<Minco::COEFF_NUM, 1>(piece * Minco::COEFF_NUM, 0) +=
        dpx * basis.transpose();
  }

  if (grad != nullptr)
  {
    Minco::InnerPointsMat gP;
    VectorXd gT;
    Minco::BoundaryState gHead;
    Minco::BoundaryState gTail;
    traj.propagateGradFull(gdC, VectorXd::Zero(traj.getPieceNum()), gP, gT,
                           gHead, gTail);
    *grad = flattenPoints(gP);
  }
  return J;
}

double mincoTrajRms(const MincoScene &scene, const VectorXd &Pa,
                    const VectorXd &Pb)
{
  const Minco ta = makeMinco(scene, Pa);
  const Minco tb = makeMinco(scene, Pb);
  const double Ttot = ta.getTotalDuration();
  double acc = 0.0;
  for (int k = 0; k <= kSampleN; ++k)
  {
    const double t = Ttot * static_cast<double>(k) / static_cast<double>(kSampleN);
    acc += (ta.getPos(t) - tb.getPos(t)).squaredNorm();
  }
  return std::sqrt(acc / static_cast<double>(kSampleN + 1));
}

MatrixXd assembleMincoL2Metric(const MincoScene &scene)
{
  const int m = 3 * scene.inner_cols;
  MatrixXd G = MatrixXd::Zero(m, m);
  Minco::BoundaryState zero = Minco::BoundaryState::Zero();
  std::vector<Minco> basis;
  basis.reserve(static_cast<std::size_t>(m));
  for (int i = 0; i < m; ++i)
  {
    VectorXd e = VectorXd::Zero(m);
    e(i) = 1.0;
    Minco traj;
    require(traj.generate(unflattenPoints(e, scene.inner_cols), zero, zero,
                          scene.durations),
            "MINCO JVP failed");
    basis.push_back(traj);
  }
  for (int i = 0; i < m; ++i)
  {
    for (int j = i; j < m; ++j)
    {
      double val = 0.0;
      const auto &Ci = basis[static_cast<std::size_t>(i)].getCoefficients();
      const auto &Cj = basis[static_cast<std::size_t>(j)].getCoefficients();
      for (int p = 0; p < basis[0].getPieceNum(); ++p)
      {
        const MatrixXd M =
            powerMassOnInterval(Minco::COEFF_NUM, scene.durations(p));
        const auto Cip = Ci.block<Minco::COEFF_NUM, 3>(p * Minco::COEFF_NUM, 0);
        const auto Cjp = Cj.block<Minco::COEFF_NUM, 3>(p * Minco::COEFF_NUM, 0);
        val += (Cip.transpose() * M * Cjp).trace();
      }
      G(i, j) = val;
      G(j, i) = val;
    }
  }
  return G;
}

struct MincoProblem
{
  const MincoScene *scene{nullptr};
  MatrixXd R;
  bool y_coords{false};
  bool whitened{false};
  MatrixXd Winv;
  std::vector<VectorXd> P_iterates;
  std::vector<double> costs;

  VectorXd toP(const VectorXd &x) const
  {
    VectorXd phys = whitened ? (Winv * x) : x;
    if (y_coords)
    {
      return R.inverse() * phys;
    }
    return phys;
  }

  static double evaluate(void *ptr, const VectorXd &x, VectorXd &g)
  {
    auto *self = static_cast<MincoProblem *>(ptr);
    const VectorXd P = self->toP(x);
    VectorXd gP;
    const double J = mincoObjective(*self->scene, P, &gP);
    VectorXd g_phys = self->y_coords ? self->R.inverse().transpose() * gP : gP;
    g = self->whitened ? self->Winv.transpose() * g_phys : g_phys;
    return J;
  }

  static int progress(void *ptr, const VectorXd &x, const VectorXd &,
                      const double fx, const double, const int, const int)
  {
    auto *self = static_cast<MincoProblem *>(ptr);
    self->P_iterates.push_back(self->toP(x));
    self->costs.push_back(fx);
    return 0;
  }
};

RunRecord runMincoLbfgs(const MincoScene &scene, const VectorXd &P0,
                        const MatrixXd &R, bool y_coords, bool whitened,
                        const MatrixXd &G_phys, int max_iters, bool full)
{
  MincoProblem problem;
  problem.scene = &scene;
  problem.R = R;
  problem.y_coords = y_coords;
  problem.whitened = whitened;
  if (whitened)
  {
    problem.Winv = spdInvSqrt(G_phys);
  }
  VectorXd x0 = y_coords ? (R * P0) : P0;
  VectorXd x = whitened ? (spdSqrt(G_phys) * x0) : x0;
  problem.P_iterates.push_back(problem.toP(x));
  VectorXd g_unused(x.size());
  problem.costs.push_back(MincoProblem::evaluate(&problem, x, g_unused));

  double f = 0.0;
  const int status = math_utils::lbfgs::lbfgs_optimize(
      x, f, &MincoProblem::evaluate, nullptr, &MincoProblem::progress, &problem,
      makeParams(max_iters, full));

  RunRecord rec;
  rec.status = status;
  rec.physical = problem.P_iterates;
  rec.costs = problem.costs;
  rec.iters = static_cast<int>(problem.P_iterates.size()) - 1;
  rec.final_cost = problem.costs.back();
  return rec;
}

struct Check
{
  std::string name;
  double value{0.0};
  bool pass{false};
  std::string expect;
};

Check makeCheck(const std::string &name, double value, double tol, bool small,
                const std::string &expect)
{
  Check c;
  c.name = name;
  c.value = value;
  c.expect = expect;
  c.pass = small ? (value < tol) : (value > tol);
  return c;
}

void printChecks(const std::string &title, const std::vector<Check> &checks,
                 const std::vector<std::pair<std::string, std::string>> &extras)
{
  std::cout << "\n" << std::string(78, '=') << "\n" << title << "\n"
            << std::string(78, '=') << "\n";
  for (const auto &kv : extras)
  {
    std::cout << "  " << std::left << std::setw(42) << kv.first << " = "
              << kv.second << "\n";
  }
  if (!extras.empty())
  {
    std::cout << "\n";
  }
  for (const auto &c : checks)
  {
    std::cout << "  " << std::left << std::setw(42) << c.name << " = "
              << sci(c.value) << "    [" << (c.pass ? "PASS" : "FAIL") << "]  "
              << c.expect << "\n";
  }
}

RunRecord runPolyEarlyStop(const VectorXd &x0, bool bernstein, bool whitened,
                           const MatrixXd &G, double kappa)
{
  PolyProblem problem;
  problem.bernstein = bernstein;
  problem.whitened = whitened;
  problem.early_stop = true;
  problem.kappa = kappa;
  if (whitened)
  {
    problem.Winv = spdInvSqrt(G);
  }
  VectorXd x = whitened ? (spdSqrt(G) * x0) : x0;
  problem.previous_x = x;
  problem.iterates.push_back(problem.toPhysical(x));
  VectorXd g_unused(x.size());
  problem.costs.push_back(PolyProblem::evaluate(&problem, x, g_unused));

  double f = 0.0;
  const int status = math_utils::lbfgs::lbfgs_optimize(
      x, f, &PolyProblem::evaluate, nullptr, &PolyProblem::progress, &problem,
      makeParams(30, false));

  RunRecord rec;
  rec.status = status;
  rec.physical = problem.iterates;
  rec.costs = problem.costs;
  rec.iters = static_cast<int>(problem.iterates.size()) - 1;
  rec.final_cost = problem.costs.back();
  return rec;
}

void writeCsv(const std::string &path, const std::vector<double> &t,
              const std::vector<double> &a, const std::vector<double> &b,
              const std::vector<double> &c, const std::vector<double> &d)
{
  std::ofstream out(path);
  require(out.good(), "failed to write " + path);
  out << "t,p_eucl_power,p_eucl_bernstein,p_nat_power,p_nat_bernstein\n";
  out << std::setprecision(16);
  for (std::size_t i = 0; i < t.size(); ++i)
  {
    out << t[i] << "," << a[i] << "," << b[i] << "," << c[i] << "," << d[i]
        << "\n";
  }
}

void appendCostTrace(std::ofstream &out, const std::string &name,
                     const RunRecord &rec)
{
  out << std::setprecision(16);
  for (std::size_t i = 0; i < rec.costs.size(); ++i)
  {
    out << name << "," << i << "," << rec.costs[i] << "\n";
  }
}

void printCostTrace(const std::string &name, const RunRecord &rec)
{
  std::cout << "  " << name << "  iters=" << rec.iters
            << "  J0=" << sci(rec.costs.front(), 8)
            << "  J*=" << sci(rec.final_cost, 8) << "\n    J[k] =";
  const int n = static_cast<int>(rec.costs.size());
  const int show = std::min(n, 12);
  for (int i = 0; i < show; ++i)
  {
    std::cout << " " << sci(rec.costs[static_cast<std::size_t>(i)], 4);
  }
  if (n > show)
  {
    std::cout << " ... " << sci(rec.costs.back(), 4);
  }
  std::cout << "\n";
}

int itersToReach(const RunRecord &rec, double target, double rel_tol)
{
  const double scale = std::max(1.0, std::abs(target));
  for (std::size_t i = 0; i < rec.costs.size(); ++i)
  {
    if (std::abs(rec.costs[i] - target) <= rel_tol * scale)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

} // namespace

int main()
{
  try
  {
    std::cout << std::setprecision(12);
    std::cout << "Coordinate L-BFGS / natural-gradient optimization test\n";
    std::cout << "Same initial p(t), same J; only coordinates and metric change.\n";

    std::vector<Check> all;
    auto take = [&](const std::vector<Check> &cs) {
      all.insert(all.end(), cs.begin(), cs.end());
    };

    const int degree = 2;
    const MatrixXd T = powerToBernstein(degree);
    const MatrixXd Ga = hilbertMass(degree);
    const MatrixXd Gb = bernsteinMass(degree);
    VectorXd a0(3);
    a0 << 1.0, -1.0, 0.5;
    const VectorXd c0 = T * a0;
    const double kappa = 0.25;
    const double omega = 6.0;

    VectorXd g_a;
    VectorXd g_c;
    const double J0a = polyObjective(a0, false, kappa, omega, &g_a);
    const double J0c = polyObjective(c0, true, kappa, omega, &g_c);
    require(std::abs(J0a - J0c) < 1.0e-12, "initial J mismatch");

    // ----- OPT-1 Euclidean L-BFGS -----
    const RunRecord eu1_a = runPolyLbfgs(a0, false, false, Ga, kappa, 1, false);
    const RunRecord eu1_c = runPolyLbfgs(c0, true, false, Gb, kappa, 1, false);
    const RunRecord eu5_a = runPolyLbfgs(a0, false, false, Ga, kappa, 5, false);
    const RunRecord eu5_c = runPolyLbfgs(c0, true, false, Gb, kappa, 5, false);
    const RunRecord euF_a = runPolyLbfgs(a0, false, false, Ga, kappa, 80, true);
    const RunRecord euF_c = runPolyLbfgs(c0, true, false, Gb, kappa, 80, true);

    const double e_eu1 = iterateRms(eu1_a, eu1_c, 1, T);
    const double e_eu5 = iterateRms(eu5_a, eu5_c, 5, T);
    const double e_euF = iterateRms(euF_a, euF_c, 1000, T);

    std::vector<Check> opt1;
    opt1.push_back(makeCheck("Euclidean L-BFGS iter-1 RMS", e_eu1, kMismatchMin,
                             false, ">> 0"));
    opt1.push_back(makeCheck("Euclidean L-BFGS iter-5 RMS", e_eu5, kMismatchMin,
                             false, ">> 0"));
    opt1.push_back(makeCheck("Euclidean L-BFGS full-final RMS", e_euF, kMismatchMin,
                             false, ">> 0 (nonconvex: different basins)"));
    opt1.push_back(makeCheck("|J*_power - J*_bernstein| full",
                             std::abs(euF_a.final_cost - euF_c.final_cost),
                             kMismatchMin, false, ">> 0 (different local min)"));
    printChecks("OPT-1  Power ↔ Bernstein  Euclidean L-BFGS", opt1,
                {{"J(p0)", sci(J0a, 12)},
                 {"Euclidean iter-1 J power", sci(eu1_a.final_cost, 12)},
                 {"Euclidean iter-1 J bernstein", sci(eu1_c.final_cost, 12)},
                 {"full iters power / bernstein",
                  std::to_string(euF_a.iters) + " / " +
                      std::to_string(euF_c.iters)},
                 {"full J* power / bernstein",
                  sci(euF_a.final_cost, 12) + " / " + sci(euF_c.final_cost, 12)}});
    take(opt1);

    // ----- OPT-2 L2-whitened (natural) L-BFGS -----
    const RunRecord n1_a = runPolyLbfgs(a0, false, true, Ga, kappa, 1, false);
    const RunRecord n1_c = runPolyLbfgs(c0, true, true, Gb, kappa, 1, false);
    const RunRecord n5_a = runPolyLbfgs(a0, false, true, Ga, kappa, 5, false);
    const RunRecord n5_c = runPolyLbfgs(c0, true, true, Gb, kappa, 5, false);
    const RunRecord nF_a = runPolyLbfgs(a0, false, true, Ga, kappa, 80, true);
    const RunRecord nF_c = runPolyLbfgs(c0, true, true, Gb, kappa, 80, true);

    std::vector<Check> opt2;
    opt2.push_back(makeCheck("Natural L-BFGS iter-1 RMS", iterateRms(n1_a, n1_c, 1, T),
                             kPathTol, true, "~ 0"));
    opt2.push_back(makeCheck("Natural L-BFGS iter-5 RMS", iterateRms(n5_a, n5_c, 5, T),
                             kPathTol, true, "~ 0"));
    opt2.push_back(makeCheck("Natural L-BFGS full-final RMS",
                             iterateRms(nF_a, nF_c, 1000, T), kFinalTol, true,
                             "~ 0"));
    printChecks("OPT-2  Power ↔ Bernstein  L2-whitened (natural) L-BFGS", opt2,
                {{"Natural iter-1 J power", sci(n1_a.final_cost, 12)},
                 {"Natural iter-1 J bernstein", sci(n1_c.final_cost, 12)},
                 {"full iters power / bernstein",
                  std::to_string(nF_a.iters) + " / " +
                      std::to_string(nF_c.iters)},
                 {"full J* power / bernstein",
                  sci(nF_a.final_cost, 12) + " / " + sci(nF_c.final_cost, 12)}});
    take(opt2);

    // ----- OPT-3 FastLbfgs parameter-space early stop -----
    const RunRecord fs_eu_a = runPolyEarlyStop(a0, false, false, Ga, kappa);
    const RunRecord fs_eu_c = runPolyEarlyStop(c0, true, false, Gb, kappa);
    const RunRecord fs_n_a = runPolyEarlyStop(a0, false, true, Ga, kappa);
    const RunRecord fs_n_c = runPolyEarlyStop(c0, true, true, Gb, kappa);
    const double e_fs_eu = trajectoryRms(fs_eu_a.physical.back(),
                                         T.inverse() * fs_eu_c.physical.back(),
                                         false);
    const double e_fs_n = trajectoryRms(fs_n_a.physical.back(),
                                        T.inverse() * fs_n_c.physical.back(),
                                        false);

    std::vector<Check> opt3;
    opt3.push_back(makeCheck("Early-stop Euclidean RMS", e_fs_eu,
                             kMismatchMin, false, ">> 0 (coord-dependent stop)"));
    opt3.push_back(makeCheck("Early-stop Natural RMS", e_fs_n, 5.0e-6,
                             true, "~ 0"));
    printChecks("OPT-3  Parameter-space early stop (rel_cost + rel_step on x)", opt3,
                {{"Euclidean stop iters power / bernstein",
                  std::to_string(fs_eu_a.iters) + " / " +
                      std::to_string(fs_eu_c.iters)},
                 {"Euclidean stopped J power / bernstein",
                  sci(fs_eu_a.final_cost, 8) + " / " +
                      sci(fs_eu_c.final_cost, 8)},
                 {"Natural stop iters power / bernstein",
                  std::to_string(fs_n_a.iters) + " / " +
                      std::to_string(fs_n_c.iters)},
                 {"Natural stopped J power / bernstein",
                  sci(fs_n_a.final_cost, 8) + " / " +
                      sci(fs_n_c.final_cost, 8)}});
    take(opt3);

    // ----- OPT-4 MINCO P vs y=RP -----
    MincoScene scene;
    scene.head.col(0) << 0.0, 0.0, 1.0;
    scene.tail.col(0) << 3.0, 1.0, 1.1;
    scene.durations.resize(3);
    scene.durations << 1.0, 1.0, 1.0;
    Minco::InnerPointsMat Pmat(3, 2);
    Pmat.col(0) << 1.0, 0.25, 1.05;
    Pmat.col(1) << 2.1, -0.35, 0.95;
    const VectorXd P0 = flattenPoints(Pmat);
    const int m = static_cast<int>(P0.size());
    std::mt19937 rng(42);
    std::normal_distribution<double> n01(0.0, 1.0);
    MatrixXd R(m, m);
    for (int i = 0; i < m; ++i)
    {
      for (int j = 0; j < m; ++j)
      {
        R(i, j) = n01(rng);
      }
    }
    R += 0.8 * MatrixXd::Identity(m, m);
    R(0, 1) += 1.4;
    const MatrixXd GP = assembleMincoL2Metric(scene);
    const MatrixXd Gy = R.inverse().transpose() * GP * R.inverse();

    const RunRecord meu1_p =
        runMincoLbfgs(scene, P0, R, false, false, GP, 1, false);
    const RunRecord meu1_y =
        runMincoLbfgs(scene, P0, R, true, false, Gy, 1, false);
    const RunRecord meu5_p =
        runMincoLbfgs(scene, P0, R, false, false, GP, 5, false);
    const RunRecord meu5_y =
        runMincoLbfgs(scene, P0, R, true, false, Gy, 5, false);
    const RunRecord meuF_p =
        runMincoLbfgs(scene, P0, R, false, false, GP, 80, true);
    const RunRecord meuF_y =
        runMincoLbfgs(scene, P0, R, true, false, Gy, 80, true);

    const RunRecord mn1_p =
        runMincoLbfgs(scene, P0, R, false, true, GP, 1, false);
    const RunRecord mn1_y =
        runMincoLbfgs(scene, P0, R, true, true, Gy, 1, false);
    const RunRecord mn5_p =
        runMincoLbfgs(scene, P0, R, false, true, GP, 5, false);
    const RunRecord mn5_y =
        runMincoLbfgs(scene, P0, R, true, true, Gy, 5, false);

    auto mincoIterRms = [&](const RunRecord &A, const RunRecord &B, int k) {
      const int ka = std::min(k, static_cast<int>(A.physical.size()) - 1);
      const int kb = std::min(k, static_cast<int>(B.physical.size()) - 1);
      return mincoTrajRms(scene, A.physical[static_cast<std::size_t>(ka)],
                          B.physical[static_cast<std::size_t>(kb)]);
    };

    std::vector<Check> opt4;
    opt4.push_back(makeCheck("MINCO Euclidean iter-1 RMS", mincoIterRms(meu1_p, meu1_y, 1),
                             kMismatchMin, false, ">> 0"));
    opt4.push_back(makeCheck("MINCO Euclidean iter-5 RMS", mincoIterRms(meu5_p, meu5_y, 5),
                             kMismatchMin, false, ">> 0"));
    opt4.push_back(makeCheck("MINCO Euclidean full-final RMS",
                             mincoIterRms(meuF_p, meuF_y, 1000), kFinalTol, true,
                             "~ 0"));
    opt4.push_back(makeCheck("MINCO Natural iter-1 RMS", mincoIterRms(mn1_p, mn1_y, 1),
                             1.0e-6, true, "~ 0"));
    opt4.push_back(makeCheck("MINCO Natural iter-5 RMS", mincoIterRms(mn5_p, mn5_y, 5),
                             1.0e-6, true, "~ 0"));
    const RunRecord mnF_p =
        runMincoLbfgs(scene, P0, R, false, true, GP, 80, true);
    const RunRecord mnF_y =
        runMincoLbfgs(scene, P0, R, true, true, Gy, 80, true);

    printChecks("OPT-4  MINCO-S4  P vs y=RP  Euclidean vs MCE-L2 whitened L-BFGS",
                opt4,
                {{"Euclidean iter-1 J(P) / J(y)",
                  sci(meu1_p.final_cost, 10) + " / " + sci(meu1_y.final_cost, 10)},
                 {"Natural iter-1 J(P) / J(y)",
                  sci(mn1_p.final_cost, 10) + " / " + sci(mn1_y.final_cost, 10)},
                 {"Euclidean full iters P / y",
                  std::to_string(meuF_p.iters) + " / " +
                      std::to_string(meuF_y.iters)},
                 {"Natural full iters P / y",
                  std::to_string(mnF_p.iters) + " / " +
                      std::to_string(mnF_y.iters)},
                 {"Euclidean full J* P / y",
                  sci(meuF_p.final_cost, 10) + " / " +
                      sci(meuF_y.final_cost, 10)},
                 {"Natural full J* P / y",
                  sci(mnF_p.final_cost, 10) + " / " +
                      sci(mnF_y.final_cost, 10)}});
    take(opt4);

    const RunRecord q_eu_a = runPolyLbfgs(a0, false, false, Ga, 0.0, 80, true);
    const RunRecord q_eu_c = runPolyLbfgs(c0, true, false, Gb, 0.0, 80, true);
    const RunRecord q_n_a = runPolyLbfgs(a0, false, true, Ga, 0.0, 80, true);
    const RunRecord q_n_c = runPolyLbfgs(c0, true, true, Gb, 0.0, 80, true);

    std::cout << "\n" << std::string(78, '=') << "\n";
    std::cout << "SPEED  J vs iteration (same solver, same stop, only metric/coords)\n";
    std::cout << std::string(78, '=') << "\n";
    printCostTrace("poly Euclidean Power", euF_a);
    printCostTrace("poly Euclidean Bernstein", euF_c);
    printCostTrace("poly Natural Power", nF_a);
    printCostTrace("poly Natural Bernstein", nF_c);
    printCostTrace("poly kappa=0 Euclidean Power", q_eu_a);
    printCostTrace("poly kappa=0 Euclidean Bernstein", q_eu_c);
    printCostTrace("poly kappa=0 Natural Power", q_n_a);
    printCostTrace("poly kappa=0 Natural Bernstein", q_n_c);
    printCostTrace("MINCO Euclidean P", meuF_p);
    printCostTrace("MINCO Euclidean y", meuF_y);
    printCostTrace("MINCO Natural P", mnF_p);
    printCostTrace("MINCO Natural y", mnF_y);

    const double poly_nat_star = nF_a.final_cost;
    const double minco_star = meuF_p.final_cost;
    std::cout << "\n  iters to |J-J_ref|/max(1,|J_ref|) < 1e-6\n";
    std::cout << "    poly Euclidean Power -> natural basin     = "
              << itersToReach(euF_a, poly_nat_star, 1.0e-6) << "\n";
    std::cout << "    poly Euclidean Bernstein -> natural basin = "
              << itersToReach(euF_c, poly_nat_star, 1.0e-6)
              << "  (own J*=" << sci(euF_c.final_cost, 6) << ")\n";
    std::cout << "    poly Natural Power                        = "
              << itersToReach(nF_a, poly_nat_star, 1.0e-6) << "\n";
    std::cout << "    poly Natural Bernstein                    = "
              << itersToReach(nF_c, poly_nat_star, 1.0e-6) << "\n";
    std::cout << "    MINCO Euclidean P                         = "
              << itersToReach(meuF_p, minco_star, 1.0e-6) << "\n";
    std::cout << "    MINCO Euclidean y                         = "
              << itersToReach(meuF_y, minco_star, 1.0e-6) << "\n";
    std::cout << "    MINCO Natural P                           = "
              << itersToReach(mnF_p, minco_star, 1.0e-6) << "\n";
    std::cout << "    MINCO Natural y                           = "
              << itersToReach(mnF_y, minco_star, 1.0e-6) << "\n";
    std::cout << "    kappa=0 Euclidean Power / Bernstein       = "
              << q_eu_a.iters << " / " << q_eu_c.iters << "\n";
    std::cout << "    kappa=0 Natural Power / Bernstein         = "
              << q_n_a.iters << " / " << q_n_c.iters << "\n";

#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif
    const std::string speed_csv =
        std::string(ROOT_DIR) + "Tests/coordinate_opt_speed_traces.csv";
    {
      std::ofstream out(speed_csv);
      require(out.good(), "failed to write speed csv");
      out << "name,iter,J\n";
      appendCostTrace(out, "poly_eucl_power", euF_a);
      appendCostTrace(out, "poly_eucl_bernstein", euF_c);
      appendCostTrace(out, "poly_nat_power", nF_a);
      appendCostTrace(out, "poly_nat_bernstein", nF_c);
      appendCostTrace(out, "poly_k0_eucl_power", q_eu_a);
      appendCostTrace(out, "poly_k0_eucl_bernstein", q_eu_c);
      appendCostTrace(out, "poly_k0_nat_power", q_n_a);
      appendCostTrace(out, "poly_k0_nat_bernstein", q_n_c);
      appendCostTrace(out, "minco_eucl_P", meuF_p);
      appendCostTrace(out, "minco_eucl_y", meuF_y);
      appendCostTrace(out, "minco_nat_P", mnF_p);
      appendCostTrace(out, "minco_nat_y", mnF_y);
    }
    std::cout << "  wrote " << speed_csv << "\n";

    std::vector<double> ts, p_eu_a, p_eu_c, p_n_a, p_n_c;
    for (int k = 0; k <= kSampleN; ++k)
    {
      const double t = static_cast<double>(k) / static_cast<double>(kSampleN);
      ts.push_back(t);
      p_eu_a.push_back(evalPower(eu1_a.physical.back(), t));
      p_eu_c.push_back(evalBernstein(eu1_c.physical.back(), t));
      p_n_a.push_back(evalPower(n1_a.physical.back(), t));
      p_n_c.push_back(evalBernstein(n1_c.physical.back(), t));
    }
#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif
    const std::string csv =
        std::string(ROOT_DIR) + "Tests/coordinate_opt_iter1_trajectories.csv";
    writeCsv(csv, ts, p_eu_a, p_eu_c, p_n_a, p_n_c);
    std::cout << "\n  wrote " << csv << "\n";

    std::cout << "\n" << std::string(78, '=') << "\nSUMMARY\n"
              << std::string(78, '=') << "\n";
    int n_pass = 0;
    int n_fail = 0;
    for (const auto &c : all)
    {
      std::cout << "  " << (c.pass ? "PASS" : "FAIL") << "  " << std::left
                << std::setw(42) << c.name << "  " << sci(c.value) << "\n";
      n_pass += static_cast<int>(c.pass);
      n_fail += static_cast<int>(!c.pass);
    }
    std::cout << "\n  checks passed = " << n_pass << " / " << (n_pass + n_fail)
              << "\n";
    require(n_fail == 0, "one or more optimization invariance checks failed");
    std::cout << "\n[coordinate_opt_invariance_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[coordinate_opt_invariance_self_test] FAIL: " << ex.what()
              << "\n";
    return 1;
  }
}
