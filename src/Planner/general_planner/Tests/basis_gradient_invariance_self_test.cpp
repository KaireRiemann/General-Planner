/**
 * Euclidean gradient basis-invariance validation.
 *
 * Evidence chain (document sections 8–32):
 *   trajectory equality
 *   -> objective equality
 *   -> correct covector transformation
 *   -> Euclidean physical-direction mismatch
 *   -> metric pullback
 *   -> natural physical-direction invariance
 *   -> same phenomenon on the MINCO/MCE reduced manifold
 *
 * Isolates representation + metric only: no L-BFGS, line search, obstacles,
 * penalties, or free-time optimization.
 */

#include "traj_opt/minco/minco_trajectory.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
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

constexpr double kEqTol = 1.0e-12;
constexpr double kGradTol = 1.0e-10;
constexpr double kMismatchMin = 1.0e-3;
constexpr int kSampleN = 400;

struct CheckResult
{
  std::string name;
  double value{0.0};
  bool pass{false};
  std::string expect;
};

struct LevelReport
{
  std::string title;
  std::vector<CheckResult> checks;
  std::vector<std::pair<std::string, std::string>> extras;
};

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

std::string vecStr(const VectorXd &v, int digits = 8)
{
  std::ostringstream oss;
  oss << std::setprecision(digits);
  oss << "[";
  for (Eigen::Index i = 0; i < v.size(); ++i)
  {
    if (i > 0)
    {
      oss << ", ";
    }
    oss << v(i);
  }
  oss << "]";
  return oss.str();
}

double relErr(const VectorXd &a, const VectorXd &b)
{
  return (a - b).norm() / std::max(1.0, a.norm());
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

MatrixXd kron(const MatrixXd &A, const MatrixXd &B)
{
  MatrixXd K(A.rows() * B.rows(), A.cols() * B.cols());
  for (int i = 0; i < A.rows(); ++i)
  {
    for (int j = 0; j < A.cols(); ++j)
    {
      K.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = A(i, j) * B;
    }
  }
  return K;
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

double maxTrajectoryError(const VectorXd &a, const VectorXd &c, int samples)
{
  double e = 0.0;
  for (int k = 0; k <= samples; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(samples);
    e = std::max(e, std::abs(evalPower(a, t) - evalBernstein(c, t)));
  }
  return e;
}

double rmsTangentError(const VectorXd &da, const VectorXd &dc_in_power_or_bern,
                       bool dc_is_bernstein, int samples)
{
  double acc = 0.0;
  for (int k = 0; k <= samples; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(samples);
    const double pa = evalPower(da, t);
    const double pb = dc_is_bernstein ? evalBernstein(dc_in_power_or_bern, t)
                                      : evalPower(dc_in_power_or_bern, t);
    const double d = pa - pb;
    acc += d * d;
  }
  return std::sqrt(acc / static_cast<double>(samples + 1));
}

double dirEnergy3(const std::vector<Vector3d> &a, const std::vector<Vector3d> &b,
                  double dt)
{
  require(a.size() == b.size() && !a.empty(), "dirEnergy3 size mismatch");
  double acc = 0.0;
  for (std::size_t k = 0; k < a.size(); ++k)
  {
    acc += (a[k] - b[k]).squaredNorm() * dt;
  }
  return std::sqrt(acc);
}

CheckResult makeCheck(const std::string &name, double value, double tol,
                      bool want_small, const std::string &expect)
{
  CheckResult c;
  c.name = name;
  c.value = value;
  c.expect = expect;
  c.pass = want_small ? (value < tol) : (value > tol);
  return c;
}

void printLevel(const LevelReport &report)
{
  std::cout << "\n" << std::string(78, '=') << "\n";
  std::cout << report.title << "\n";
  std::cout << std::string(78, '=') << "\n";
  for (const auto &kv : report.extras)
  {
    std::cout << "  " << std::left << std::setw(36) << kv.first << " = "
              << kv.second << "\n";
  }
  if (!report.extras.empty())
  {
    std::cout << "\n";
  }
  for (const auto &c : report.checks)
  {
    std::cout << "  " << std::left << std::setw(36) << c.name << " = "
              << sci(c.value) << "    [" << (c.pass ? "PASS" : "FAIL") << "]  "
              << c.expect << "\n";
  }
}

LevelReport runLevel1()
{
  constexpr int n = 6;
  MatrixXd A(n, n);
  A << 1.0, 2.0, 0.3, -0.4, 0.1, 0.7,
      0.2, 1.1, 1.4, 0.0, -0.6, 0.3,
      -0.5, 0.8, 2.2, 0.9, 0.2, -0.1,
      0.4, -0.3, 0.7, 1.5, 1.1, 0.0,
      0.9, 0.1, -0.8, 0.6, 1.8, 0.5,
      -0.2, 0.5, 0.0, -0.7, 0.4, 1.3;
  require(std::abs(A.determinant()) > 1.0e-6, "A must be invertible");
  require((A.transpose() * A - MatrixXd::Identity(n, n)).norm() > 0.5,
          "A must be non-orthogonal");

  MatrixXd H = MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i)
  {
    H(i, i) = 1.0 + 0.2 * i;
    if (i + 1 < n)
    {
      H(i, i + 1) = 0.15;
      H(i + 1, i) = 0.15;
    }
  }

  VectorXd x = VectorXd::LinSpaced(n, 0.4, 1.5);
  VectorXd y = A.inverse() * x;
  require((A * y - x).norm() < 1.0e-12, "x = A y reconstruction failed");

  const VectorXd gx = H * x;
  const VectorXd gy = A.transpose() * gx;
  const VectorXd dx_E = -gx;
  const VectorXd dy_E = -gy;
  const VectorXd mapped_E = A * dy_E;

  const MatrixXd Gy = A.transpose() * A;
  const VectorXd dy_N = -Gy.ldlt().solve(gy);
  const VectorXd mapped_N = A * dy_N;

  LevelReport r;
  r.title = "LEVEL 1  Pure linear algebra   x = A y,  J(x) = 1/2 x^T H x";
  r.extras.push_back({"n", std::to_string(n)});
  r.extras.push_back({"det(A)", sci(A.determinant())});
  r.extras.push_back({"||A^T A - I||", sci((A.transpose() * A - MatrixXd::Identity(n, n)).norm())});
  r.extras.push_back({"||x||", sci(x.norm())});
  r.checks.push_back(makeCheck("gradient chain-rule error", relErr(gy, A.transpose() * gx),
                               kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("Euclidean direction mismatch", relErr(dx_E, mapped_E),
                               kMismatchMin, false, ">> 1e-10"));
  r.checks.push_back(makeCheck("Natural direction mismatch", relErr(dx_E, mapped_N),
                               kGradTol, true, "< 1e-10"));
  return r;
}

struct PolyCase
{
  int degree{0};
  VectorXd a;
  VectorXd a_ref;
  bool tracking{false};
};

LevelReport runPowerBernstein(const PolyCase &pc, const std::string &title,
                              std::vector<double> *t_out,
                              std::vector<double> *dp_power_E,
                              std::vector<double> *dp_bern_E,
                              std::vector<double> *dp_bern_N)
{
  const int n = pc.degree + 1;
  const MatrixXd T = powerToBernstein(pc.degree);
  const MatrixXd Ga = hilbertMass(pc.degree);
  const MatrixXd Gb = bernsteinMass(pc.degree);
  const VectorXd c = T * pc.a;
  VectorXd c_ref = VectorXd::Zero(n);
  VectorXd a_shift = pc.a;
  VectorXd c_shift = c;
  if (pc.tracking)
  {
    c_ref = T * pc.a_ref;
    a_shift = pc.a - pc.a_ref;
    c_shift = c - c_ref;
  }

  const double Ja = 0.5 * a_shift.dot(Ga * a_shift);
  const double Jc = 0.5 * c_shift.dot(Gb * c_shift);
  const VectorXd ga = Ga * a_shift;
  const VectorXd gc = Gb * c_shift;

  const VectorXd da_E = -ga;
  const VectorXd dc_E = -gc;
  const VectorXd da_from_c_E = T.inverse() * dc_E;

  const MatrixXd Gc = T.inverse().transpose() * T.inverse();
  const VectorXd dc_N = -Gc.ldlt().solve(gc);
  const VectorXd da_from_c_N = T.inverse() * dc_N;

  const double e_p = maxTrajectoryError(pc.a, c, kSampleN);
  const double e_J = std::abs(Ja - Jc);
  const double e_g = relErr(ga, T.transpose() * gc);
  const double e_E = relErr(da_E, da_from_c_E);
  const double e_N = relErr(da_E, da_from_c_N);
  const double tan_E = rmsTangentError(da_E, dc_E, true, kSampleN);
  const double tan_N = rmsTangentError(da_E, dc_N, true, kSampleN);

  if (t_out != nullptr)
  {
    t_out->clear();
    dp_power_E->clear();
    dp_bern_E->clear();
    dp_bern_N->clear();
    for (int k = 0; k <= kSampleN; ++k)
    {
      const double t = static_cast<double>(k) / static_cast<double>(kSampleN);
      t_out->push_back(t);
      dp_power_E->push_back(evalPower(da_E, t));
      dp_bern_E->push_back(evalBernstein(dc_E, t));
      dp_bern_N->push_back(evalBernstein(dc_N, t));
    }
  }

  LevelReport r;
  r.title = title;
  r.extras.push_back({"degree", std::to_string(pc.degree)});
  r.extras.push_back({"a (Power)", vecStr(pc.a)});
  r.extras.push_back({"c = T a (Bernstein)", vecStr(c)});
  if (pc.tracking)
  {
    r.extras.push_back({"a_ref", vecStr(pc.a_ref)});
  }
  r.extras.push_back({"J_a", sci(Ja, 12)});
  r.extras.push_back({"J_c", sci(Jc, 12)});
  r.extras.push_back({"grad_a J", vecStr(ga)});
  r.extras.push_back({"grad_c J", vecStr(gc)});
  r.extras.push_back({"d_a^E = -grad_a J", vecStr(da_E)});
  r.extras.push_back({"T^{-1} d_c^E", vecStr(da_from_c_E)});
  r.extras.push_back({"T^{-1} d_c^N", vecStr(da_from_c_N)});
  r.extras.push_back({"||T^T T - I|| (non-orth.)",
                      sci((T.transpose() * T - MatrixXd::Identity(n, n)).norm())});

  r.checks.push_back(makeCheck("trajectory equality error", e_p, kEqTol, true, "< 1e-12"));
  r.checks.push_back(makeCheck("objective difference", e_J, kEqTol, true, "< 1e-12"));
  r.checks.push_back(makeCheck("gradient chain-rule error", e_g, kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("Euclidean direction mismatch", e_E, kMismatchMin, false,
                               ">> 1e-10"));
  r.checks.push_back(makeCheck("Natural direction mismatch", e_N, kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("tangent RMS e_E (sampled)", tan_E, kMismatchMin, false,
                               ">> 0"));
  r.checks.push_back(makeCheck("tangent RMS e_N (sampled)", tan_N, kGradTol, true,
                               "~ 0"));
  return r;
}

LevelReport runPowerBernstein3D()
{
  constexpr int degree = 7;
  constexpr int n = degree + 1;
  constexpr int dim = 3;
  const MatrixXd T = powerToBernstein(degree);
  const MatrixXd Tbar = kron(T, MatrixXd::Identity(dim, dim));
  const MatrixXd Ga = hilbertMass(degree);
  const MatrixXd Gabar = kron(Ga, MatrixXd::Identity(dim, dim));
  const MatrixXd Gb = bernsteinMass(degree);
  const MatrixXd Gbbar = kron(Gb, MatrixXd::Identity(dim, dim));

  VectorXd a(n * dim);
  a.setZero();
  const double ax[8] = {0.8, -0.4, 0.25, -0.12, 0.06, -0.03, 0.015, -0.007};
  const double ay[8] = {0.2, 0.5, -0.3, 0.08, -0.04, 0.02, -0.01, 0.004};
  const double az[8] = {1.0, -0.1, 0.05, 0.02, -0.01, 0.005, -0.002, 0.001};
  for (int k = 0; k < n; ++k)
  {
    a(k * dim + 0) = ax[k];
    a(k * dim + 1) = ay[k];
    a(k * dim + 2) = az[k];
  }
  const VectorXd c = Tbar * a;
  const double Ja = 0.5 * a.dot(Gabar * a);
  const double Jc = 0.5 * c.dot(Gbbar * c);
  const VectorXd ga = Gabar * a;
  const VectorXd gc = Gbbar * c;

  const VectorXd da_E = -ga;
  const VectorXd dc_E = -gc;
  const VectorXd da_from_c_E = Tbar.inverse() * dc_E;
  const MatrixXd Gc = Tbar.inverse().transpose() * Tbar.inverse();
  const VectorXd dc_N = -Gc.ldlt().solve(gc);
  const VectorXd da_from_c_N = Tbar.inverse() * dc_N;

  auto eval3 = [&](const VectorXd &coeff, bool bernstein, double t) {
    Vector3d p = Vector3d::Zero();
    for (int d = 0; d < dim; ++d)
    {
      VectorXd one(n);
      for (int k = 0; k < n; ++k)
      {
        one(k) = coeff(k * dim + d);
      }
      p(d) = bernstein ? evalBernstein(one, t) : evalPower(one, t);
    }
    return p;
  };

  double e_p = 0.0;
  std::vector<Vector3d> dA_E;
  std::vector<Vector3d> dB_E;
  std::vector<Vector3d> dB_N;
  dA_E.reserve(kSampleN + 1);
  dB_E.reserve(kSampleN + 1);
  dB_N.reserve(kSampleN + 1);
  for (int k = 0; k <= kSampleN; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(kSampleN);
    e_p = std::max(e_p, (eval3(a, false, t) - eval3(c, true, t)).norm());
    dA_E.push_back(eval3(da_E, false, t));
    dB_E.push_back(eval3(dc_E, true, t));
    dB_N.push_back(eval3(dc_N, true, t));
  }
  const double dt = 1.0 / static_cast<double>(kSampleN);
  const double E_dir_E = dirEnergy3(dA_E, dB_E, dt);
  const double E_dir_N = dirEnergy3(dA_E, dB_N, dt);

  LevelReport r;
  r.title = "LEVEL 2C  3D degree-7   T_bar = T ⊗ I_3    J = 1/2 ∫ ||p||^2 dt";
  r.extras.push_back({"dim(a)", std::to_string(static_cast<int>(a.size()))});
  r.extras.push_back({"J_a", sci(Ja, 12)});
  r.extras.push_back({"J_c", sci(Jc, 12)});
  r.extras.push_back({"||d_a^E||", sci(da_E.norm())});
  r.extras.push_back({"||T_bar^{-1} d_c^E||", sci(da_from_c_E.norm())});
  r.checks.push_back(makeCheck("trajectory equality error", e_p, kEqTol, true, "< 1e-12"));
  r.checks.push_back(makeCheck("objective difference", std::abs(Ja - Jc), kEqTol, true,
                               "< 1e-12"));
  r.checks.push_back(makeCheck("gradient chain-rule error", relErr(ga, Tbar.transpose() * gc),
                               kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("Euclidean direction mismatch", relErr(da_E, da_from_c_E),
                               kMismatchMin, false, ">> 1e-10"));
  r.checks.push_back(makeCheck("Natural direction mismatch", relErr(da_E, da_from_c_N),
                               kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("E_dir Euclidean (L2)", E_dir_E, kMismatchMin, false, ">> 0"));
  r.checks.push_back(makeCheck("E_dir Natural (L2)", E_dir_N, kGradTol, true, "~ 0"));
  return r;
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

using Minco = minco::MINCOTrajectory<3, 4>;

VectorXd flattenPoints(const Minco::InnerPointsMat &P)
{
  VectorXd v(P.size());
  Eigen::Map<const VectorXd> mapped(P.data(), P.size());
  v = mapped;
  return v;
}

Minco::InnerPointsMat unflattenPoints(const VectorXd &v, int cols)
{
  require(v.size() == 3 * cols, "unflatten size mismatch");
  Minco::InnerPointsMat P(3, cols);
  Eigen::Map<VectorXd>(P.data(), v.size()) = v;
  return P;
}

double trajectoryL2(const Minco &traj)
{
  const int pieces = traj.getPieceNum();
  const auto &C = traj.getCoefficients();
  double J = 0.0;
  for (int i = 0; i < pieces; ++i)
  {
    const MatrixXd G = powerMassOnInterval(Minco::COEFF_NUM, traj.getDurations()(i));
    const auto Ci = C.block<Minco::COEFF_NUM, 3>(i * Minco::COEFF_NUM, 0);
    J += 0.5 * (Ci.transpose() * G * Ci).trace();
  }
  return J;
}

Minco::CoeffMat l2PartialByCoeffs(const Minco &traj)
{
  Minco::CoeffMat gdC(Minco::COEFF_NUM * traj.getPieceNum(), 3);
  gdC.setZero();
  const auto &C = traj.getCoefficients();
  for (int i = 0; i < traj.getPieceNum(); ++i)
  {
    const MatrixXd G = powerMassOnInterval(Minco::COEFF_NUM, traj.getDurations()(i));
    gdC.block<Minco::COEFF_NUM, 3>(i * Minco::COEFF_NUM, 0) =
        G * C.block<Minco::COEFF_NUM, 3>(i * Minco::COEFF_NUM, 0);
  }
  return gdC;
}

VectorXd waypointGradient(const Minco &traj)
{
  Minco::InnerPointsMat gP;
  VectorXd gT;
  Minco::BoundaryState gHead;
  Minco::BoundaryState gTail;
  const VectorXd zerosT = VectorXd::Zero(traj.getPieceNum());
  traj.propagateGradFull(l2PartialByCoeffs(traj), zerosT, gP, gT, gHead, gTail);
  return flattenPoints(gP);
}

Minco jvpTrajectory(const Minco::InnerPointsMat &dP,
                    const Eigen::VectorXd &durations)
{
  Minco traj;
  Minco::BoundaryState zero = Minco::BoundaryState::Zero();
  require(traj.generate(dP, zero, zero, durations), "MINCO JVP generate failed");
  return traj;
}

std::vector<Vector3d> sampleTraj(const Minco &traj, int samples)
{
  std::vector<Vector3d> out;
  out.reserve(static_cast<std::size_t>(samples) + 1);
  const double T = traj.getTotalDuration();
  for (int k = 0; k <= samples; ++k)
  {
    const double t = T * static_cast<double>(k) / static_cast<double>(samples);
    out.push_back(traj.getPos(t));
  }
  return out;
}

MatrixXd assemblePullbackMetric(const Minco::InnerPointsMat &shape,
                                const Eigen::VectorXd &durations, bool snap)
{
  const int m = static_cast<int>(shape.size());
  const int cols = static_cast<int>(shape.cols());
  MatrixXd G = MatrixXd::Zero(m, m);
  std::vector<Minco> basis;
  basis.reserve(static_cast<std::size_t>(m));
  for (int i = 0; i < m; ++i)
  {
    VectorXd e = VectorXd::Zero(m);
    e(i) = 1.0;
    basis.push_back(jvpTrajectory(unflattenPoints(e, cols), durations));
  }
  for (int i = 0; i < m; ++i)
  {
    for (int j = i; j < m; ++j)
    {
      double val = 0.0;
      if (snap)
      {
        const auto &Ci = basis[static_cast<std::size_t>(i)].getCoefficients();
        const auto &Cj = basis[static_cast<std::size_t>(j)].getCoefficients();
        for (int p = 0; p < basis[0].getPieceNum(); ++p)
        {
          const auto Q =
              Minco::Traits::controlCostHessian(durations(p));
          const auto Ci_p = Ci.block<Minco::COEFF_NUM, 3>(p * Minco::COEFF_NUM, 0);
          const auto Cj_p = Cj.block<Minco::COEFF_NUM, 3>(p * Minco::COEFF_NUM, 0);
          val += (Ci_p.transpose() * Q * Cj_p).trace();
        }
      }
      else
      {
        const auto &Ci = basis[static_cast<std::size_t>(i)].getCoefficients();
        const auto &Cj = basis[static_cast<std::size_t>(j)].getCoefficients();
        for (int p = 0; p < basis[0].getPieceNum(); ++p)
        {
          const MatrixXd M = powerMassOnInterval(Minco::COEFF_NUM, durations(p));
          const auto Ci_p = Ci.block<Minco::COEFF_NUM, 3>(p * Minco::COEFF_NUM, 0);
          const auto Cj_p = Cj.block<Minco::COEFF_NUM, 3>(p * Minco::COEFF_NUM, 0);
          val += (Ci_p.transpose() * M * Cj_p).trace();
        }
      }
      G(i, j) = val;
      G(j, i) = val;
    }
  }
  return G;
}

LevelReport runMincoLevel(std::vector<double> *t_out,
                          std::vector<double> *dpP_E_x,
                          std::vector<double> *dpy_E_x,
                          std::vector<double> *dpy_N_x)
{
  Minco::BoundaryState head = Minco::BoundaryState::Zero();
  Minco::BoundaryState tail = Minco::BoundaryState::Zero();
  head.col(0) << 0.0, 0.0, 1.0;
  tail.col(0) << 3.0, 1.0, 1.1;
  Minco::InnerPointsMat P(3, 2);
  P.col(0) << 1.0, 0.25, 1.05;
  P.col(1) << 2.1, -0.35, 0.95;
  Eigen::VectorXd durations(3);
  durations << 1.0, 1.0, 1.0;

  Minco traj_P;
  require(traj_P.generate(P, head, tail, durations), "MINCO generate(P) failed");

  const int m = static_cast<int>(P.size());
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
  R(2, 4) -= 0.9;
  require(std::abs(R.determinant()) > 1.0e-4, "R must be invertible");
  require((R.transpose() * R - MatrixXd::Identity(m, m)).norm() > 0.5,
          "R must be non-orthogonal");

  const VectorXd Pvec = flattenPoints(P);
  const VectorXd y = R * Pvec;
  const VectorXd P_from_y = R.inverse() * y;
  const Minco::InnerPointsMat P_y = unflattenPoints(P_from_y, static_cast<int>(P.cols()));
  Minco traj_y;
  require(traj_y.generate(P_y, head, tail, durations), "MINCO generate(y) failed");

  const auto samples_P = sampleTraj(traj_P, kSampleN);
  const auto samples_y = sampleTraj(traj_y, kSampleN);
  double e_p = 0.0;
  for (std::size_t k = 0; k < samples_P.size(); ++k)
  {
    e_p = std::max(e_p, (samples_P[k] - samples_y[k]).norm());
  }

  const double JP = trajectoryL2(traj_P);
  const double Jy = trajectoryL2(traj_y);
  const VectorXd gP = waypointGradient(traj_P);
  const VectorXd gy = R.inverse().transpose() * gP;

  VectorXd gP_fd = VectorXd::Zero(m);
  const double eps = 1.0e-7;
  for (int i = 0; i < m; ++i)
  {
    VectorXd Pp = Pvec;
    VectorXd Pm = Pvec;
    Pp(i) += eps;
    Pm(i) -= eps;
    Minco tp;
    Minco tm;
    require(tp.generate(unflattenPoints(Pp, 2), head, tail, durations), "fd+ failed");
    require(tm.generate(unflattenPoints(Pm, 2), head, tail, durations), "fd- failed");
    gP_fd(i) = (trajectoryL2(tp) - trajectoryL2(tm)) / (2.0 * eps);
  }

  const VectorXd dP_E = -gP;
  const VectorXd dy_E = -gy;
  const VectorXd dP_from_y_E = R.inverse() * dy_E;

  const MatrixXd GP_l2 = assemblePullbackMetric(P, durations, false);
  const MatrixXd GP_snap = assemblePullbackMetric(P, durations, true);
  const MatrixXd Gy_l2 = R.inverse().transpose() * GP_l2 * R.inverse();
  const MatrixXd Gy_id = R.inverse().transpose() * R.inverse();

  const VectorXd dP_N_id = -gP;
  const VectorXd dy_N_id = -Gy_id.ldlt().solve(gy);
  const VectorXd dP_from_y_N_id = R.inverse() * dy_N_id;

  const VectorXd dP_N_l2 = -GP_l2.ldlt().solve(gP);
  const VectorXd dy_N_l2 = -Gy_l2.ldlt().solve(gy);
  const VectorXd dP_from_y_N_l2 = R.inverse() * dy_N_l2;

  const MatrixXd Gy_snap = R.inverse().transpose() * GP_snap * R.inverse();
  const VectorXd dP_N_snap = -GP_snap.ldlt().solve(gP);
  const VectorXd dy_N_snap = -Gy_snap.ldlt().solve(gy);
  const VectorXd dP_from_y_N_snap = R.inverse() * dy_N_snap;

  auto jvpSample = [&](const VectorXd &dPvec) {
    return sampleTraj(jvpTrajectory(unflattenPoints(dPvec, 2), durations), kSampleN);
  };
  const auto tan_P_E = jvpSample(dP_E);
  const auto tan_y_E = jvpSample(dP_from_y_E);
  const auto tan_y_N = jvpSample(dP_from_y_N_id);
  const auto tan_y_N_l2 = jvpSample(dP_from_y_N_l2);
  const auto tan_P_N_l2 = jvpSample(dP_N_l2);
  const double dt = durations.sum() / static_cast<double>(kSampleN);
  const double E_dir_E = dirEnergy3(tan_P_E, tan_y_E, dt);
  const double E_dir_N = dirEnergy3(tan_P_E, tan_y_N, dt);
  const double E_dir_N_l2 = dirEnergy3(tan_P_N_l2, tan_y_N_l2, dt);

  if (t_out != nullptr)
  {
    t_out->clear();
    dpP_E_x->clear();
    dpy_E_x->clear();
    dpy_N_x->clear();
    const double Ttot = durations.sum();
    for (int k = 0; k <= kSampleN; ++k)
    {
      t_out->push_back(Ttot * static_cast<double>(k) / static_cast<double>(kSampleN));
      dpP_E_x->push_back(tan_P_E[static_cast<std::size_t>(k)].x());
      dpy_E_x->push_back(tan_y_E[static_cast<std::size_t>(k)].x());
      dpy_N_x->push_back(tan_y_N[static_cast<std::size_t>(k)].x());
    }
  }

  LevelReport r;
  r.title = "LEVEL 3  MINCO-S4 reduced manifold   y = R P,  T fixed,  "
            "J = 1/2 ∫ ||p||^2 dt";
  r.extras.push_back({"pieces", "3"});
  r.extras.push_back({"inner waypoints", "2 (R^6)"});
  r.extras.push_back({"durations", "[1, 1, 1]"});
  r.extras.push_back({"det(R)", sci(R.determinant())});
  r.extras.push_back({"||R^T R - I||", sci((R.transpose() * R - MatrixXd::Identity(m, m)).norm())});
  r.extras.push_back({"J(P)", sci(JP, 12)});
  r.extras.push_back({"J(y)", sci(Jy, 12)});
  r.extras.push_back({"||grad_P J||", sci(gP.norm())});
  r.extras.push_back({"FD vs adjoint ||gP - gP_fd|| / ||gP||", sci(relErr(gP, gP_fd))});
  r.extras.push_back({"cond(G_P L2)", sci(GP_l2.norm() * GP_l2.inverse().norm())});
  r.extras.push_back({"cond(G_P snap/MCE)", sci(GP_snap.norm() * GP_snap.inverse().norm())});

  r.checks.push_back(makeCheck("trajectory equality error", e_p, kEqTol, true, "< 1e-12"));
  r.checks.push_back(makeCheck("objective difference", std::abs(JP - Jy), kEqTol, true,
                               "< 1e-12"));
  r.checks.push_back(makeCheck("gradient chain-rule error", relErr(gy, R.inverse().transpose() * gP),
                               kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("adjoint vs FD gradient", relErr(gP, gP_fd), 1.0e-5, true,
                               "< 1e-5"));
  r.checks.push_back(makeCheck("Euclidean reduced mismatch", relErr(dP_E, dP_from_y_E),
                               kMismatchMin, false, ">> 1e-10"));
  r.checks.push_back(makeCheck("Natural (G_P=I pullback) mismatch",
                               relErr(dP_N_id, dP_from_y_N_id), kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("MCE-L2 natural mismatch", relErr(dP_N_l2, dP_from_y_N_l2),
                               kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("MCE-snap natural mismatch", relErr(dP_N_snap, dP_from_y_N_snap),
                               kGradTol, true, "< 1e-10"));
  r.checks.push_back(makeCheck("JVP E_dir Euclidean", E_dir_E, kMismatchMin, false, ">> 0"));
  r.checks.push_back(makeCheck("JVP E_dir Natural (I)", E_dir_N, kGradTol, true, "~ 0"));
  r.checks.push_back(makeCheck("JVP E_dir Natural (L2/MCE)", E_dir_N_l2, kGradTol, true, "~ 0"));
  return r;
}

void writeCsv(const std::string &path,
              const std::vector<double> &t,
              const std::vector<double> &a,
              const std::vector<double> &b,
              const std::vector<double> &c,
              const char *ha, const char *hb, const char *hc)
{
  std::ofstream out(path);
  require(out.good(), "failed to write " + path);
  out << "t," << ha << "," << hb << "," << hc << "\n";
  out << std::setprecision(16);
  for (std::size_t i = 0; i < t.size(); ++i)
  {
    out << t[i] << "," << a[i] << "," << b[i] << "," << c[i] << "\n";
  }
}

void printSummary(const std::vector<LevelReport> &levels)
{
  std::cout << "\n" << std::string(78, '=') << "\n";
  std::cout << "SUMMARY\n";
  std::cout << std::string(78, '=') << "\n";
  int n_pass = 0;
  int n_fail = 0;
  for (const auto &lvl : levels)
  {
    std::cout << "\n" << lvl.title << "\n";
    for (const auto &c : lvl.checks)
    {
      std::cout << "  " << (c.pass ? "PASS" : "FAIL") << "  "
                << std::left << std::setw(40) << c.name << "  " << sci(c.value)
                << "\n";
      n_pass += static_cast<int>(c.pass);
      n_fail += static_cast<int>(!c.pass);
    }
  }
  std::cout << "\n  checks passed = " << n_pass << " / " << (n_pass + n_fail) << "\n";
  require(n_fail == 0, "one or more invariance checks failed");
}

} // namespace

int main()
{
  try
  {
    std::cout << std::setprecision(12);
    std::cout << "Euclidean Gradient Basis-Invariance Validation\n";
    std::cout << "Only coordinate representation and metric are varied.\n";

    std::vector<LevelReport> levels;
    levels.push_back(runLevel1());

    PolyCase quad;
    quad.degree = 2;
    quad.a.resize(3);
    quad.a << 1.0, -1.0, 0.5;
    std::vector<double> tq, pE, bE, bN;
    levels.push_back(runPowerBernstein(
        quad,
        "LEVEL 2A  Quadratic Power ↔ Bernstein    J = 1/2 ∫ p(t)^2 dt",
        &tq, &pE, &bE, &bN));

    PolyCase deg7;
    deg7.degree = 7;
    deg7.tracking = true;
    deg7.a.resize(8);
    deg7.a << 1.0, -0.6, 0.35, -0.18, 0.08, -0.03, 0.012, -0.004;
    deg7.a_ref.resize(8);
    deg7.a_ref << 0.7, -0.2, 0.1, -0.05, 0.02, -0.01, 0.004, -0.001;
    levels.push_back(runPowerBernstein(
        deg7,
        "LEVEL 2B  Degree-7 Power ↔ Bernstein    J = 1/2 ∫ (p-p_ref)^2 dt",
        nullptr, nullptr, nullptr, nullptr));

    levels.push_back(runPowerBernstein3D());

    std::vector<double> tm, mP, mE, mN;
    levels.push_back(runMincoLevel(&tm, &mP, &mE, &mN));

    for (const auto &lvl : levels)
    {
      printLevel(lvl);
    }

#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif
    const std::string quad_csv =
        std::string(ROOT_DIR) + "Tests/basis_invariance_quadratic_tangents.csv";
    const std::string minco_csv =
        std::string(ROOT_DIR) + "Tests/basis_invariance_minco_tangents.csv";
    writeCsv(quad_csv, tq, pE, bE, bN,
             "delta_p_power_E", "delta_p_bernstein_E", "delta_p_bernstein_N");
    writeCsv(minco_csv, tm, mP, mE, mN,
             "delta_p_P_euclidean_x", "delta_p_y_euclidean_x",
             "delta_p_y_natural_x");
    std::cout << "\n  wrote " << quad_csv << "\n";
    std::cout << "  wrote " << minco_csv << "\n";

    printSummary(levels);
    std::cout << "\n[basis_gradient_invariance_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[basis_gradient_invariance_self_test] FAIL: " << ex.what()
              << "\n";
    return 1;
  }
}
