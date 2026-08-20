/**
 * MCE / intrinsic-metric condition-number validation.
 *
 * After parameterization invariance, this test asks whether an intrinsic
 * metric actually improves Hessian conditioning and L-BFGS efficiency:
 *   kappa(H_J)  vs  kappa(G^{-1/2} H_J G^{-1/2})
 *
 * Sequence (framework §31):
 *   1. Pure MCE: H ≈ G_MCE, whitened kappa ≈ 1
 *   2. MCE + L2 tracking
 *   3. MCE + dynamics (vel/acc)
 *   4. MCE + corridor penalty (possibly indefinite)
 *   5. Constraint Gauss-Newton metric
 *   6. Frozen-metric L-BFGS: correlate kappa with iterations / line search
 *   7. Planner-like mixed objective (isolated State2State proxy)
 * Free-time and dynamic metric are intentionally omitted (later stages).
 */

#include "traj_opt/minco/minco_trajectory.hpp"
#include "utils/optimization/lbfgs.h"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Clock = std::chrono::steady_clock;
using Minco = minco::MINCOTrajectory<3, 4>;

constexpr double kLamEpsRel = 1.0e-10;

void require(bool ok, const std::string &msg)
{
  if (!ok)
  {
    throw std::runtime_error(msg);
  }
}

std::string sci(double v, int d = 4)
{
  std::ostringstream oss;
  oss << std::scientific << std::setprecision(d) << v;
  return oss.str();
}

double falling(int n, int r)
{
  if (r < 0 || r > n)
  {
    return 0.0;
  }
  double out = 1.0;
  for (int i = 0; i < r; ++i)
  {
    out *= static_cast<double>(n - i);
  }
  return out;
}

VectorXd flatten(const Minco::InnerPointsMat &P)
{
  return Eigen::Map<const VectorXd>(P.data(), P.size());
}

Minco::InnerPointsMat unflatten(const VectorXd &v, int cols)
{
  Minco::InnerPointsMat P(3, cols);
  Eigen::Map<VectorXd>(P.data(), v.size()) = v;
  return P;
}

struct Scene
{
  int pieces{3};
  Eigen::VectorXd durations;
  Minco::BoundaryState head = Minco::BoundaryState::Zero();
  Minco::BoundaryState tail = Minco::BoundaryState::Zero();
  Minco::InnerPointsMat P0;

  int innerCols() const { return std::max(0, pieces - 1); }
  int dim() const { return 3 * innerCols(); }
};

Scene makeScene(int pieces, const Eigen::VectorXd &durations)
{
  Scene s;
  s.pieces = pieces;
  s.durations = durations;
  s.head.col(0) << 0.0, 0.0, 1.0;
  s.tail.col(0) << static_cast<double>(pieces), 0.0, 1.0;
  s.P0.resize(3, s.innerCols());
  for (int i = 0; i < s.innerCols(); ++i)
  {
    const double u = static_cast<double>(i + 1) / static_cast<double>(pieces);
    s.P0.col(i) << u * s.tail(0, 0),
        0.18 * std::sin(2.0 * M_PI * u),
        1.0 + 0.04 * std::cos(2.0 * M_PI * u);
  }
  return s;
}

Scene makeUniform(int pieces, double t_each = 1.0)
{
  Eigen::VectorXd T(pieces);
  T.setConstant(t_each);
  return makeScene(pieces, T);
}

Minco generate(const Scene &s, const VectorXd &Pvec)
{
  Minco traj;
  require(traj.generate(unflatten(Pvec, s.innerCols()), s.head, s.tail,
                        s.durations),
          "MINCO generate failed");
  return traj;
}

Minco generateJvp(const Scene &s, const VectorXd &dP)
{
  Minco traj;
  Minco::BoundaryState z = Minco::BoundaryState::Zero();
  require(traj.generate(unflatten(dP, s.innerCols()), z, z, s.durations),
          "MINCO JVP failed");
  return traj;
}

MatrixXd derivativeMass(int deriv, double T)
{
  const int n = Minco::COEFF_NUM;
  MatrixXd Q = MatrixXd::Zero(n, n);
  for (int i = deriv; i < n; ++i)
  {
    const double ci = falling(i, deriv);
    for (int j = deriv; j < n; ++j)
    {
      const double cj = falling(j, deriv);
      const int power = i + j - 2 * deriv + 1;
      Q(i, j) = ci * cj * std::pow(T, static_cast<double>(power)) /
                static_cast<double>(power);
    }
  }
  return Q;
}

double coeffInner(const Minco &a, const Minco &b, int deriv)
{
  double v = 0.0;
  for (int p = 0; p < a.getPieceNum(); ++p)
  {
    const MatrixXd Q = (deriv == 4)
                           ? MatrixXd(Minco::Traits::controlCostHessian(
                                 a.getDurations()(p)))
                           : derivativeMass(deriv, a.getDurations()(p));
    const auto Ca = a.getCoefficients().block<Minco::COEFF_NUM, 3>(
        p * Minco::COEFF_NUM, 0);
    const auto Cb = b.getCoefficients().block<Minco::COEFF_NUM, 3>(
        p * Minco::COEFF_NUM, 0);
    v += (Ca.transpose() * Q * Cb).trace();
  }
  return v;
}

struct JvpBasis
{
  std::vector<Minco> basis;
  MatrixXd G_l2;
  MatrixXd G_h1;
  MatrixXd G_h2;
  MatrixXd G_snap;
  MatrixXd G_mce;
  double t_build_s{0.0};
};

JvpBasis buildBasis(const Scene &s)
{
  JvpBasis out;
  const int m = s.dim();
  const auto t0 = Clock::now();
  out.basis.reserve(static_cast<std::size_t>(m));
  for (int i = 0; i < m; ++i)
  {
    VectorXd e = VectorXd::Zero(m);
    e(i) = 1.0;
    out.basis.push_back(generateJvp(s, e));
  }
  auto fill = [&](MatrixXd &G, int deriv, double scale) {
    G = MatrixXd::Zero(m, m);
    for (int i = 0; i < m; ++i)
    {
      for (int j = i; j < m; ++j)
      {
        const double v =
            scale * coeffInner(out.basis[static_cast<std::size_t>(i)],
                               out.basis[static_cast<std::size_t>(j)], deriv);
        G(i, j) = v;
        G(j, i) = v;
      }
    }
  };
  fill(out.G_l2, 0, 1.0);
  fill(out.G_h1, 1, 1.0);
  fill(out.G_h2, 2, 1.0);
  fill(out.G_snap, 4, 1.0);
  // getEnergy() = C^T Q C, so Hessian_P = 2 J_P^T Q J_P.
  out.G_mce = 2.0 * out.G_snap;
  out.t_build_s = std::chrono::duration<double>(Clock::now() - t0).count();
  return out;
}

VectorXd energyGrad(const Minco &traj)
{
  Minco::CoeffMat gdC;
  double energy = 0.0;
  traj.getEnergyPartialGradByCoeffs(energy, gdC);
  Minco::InnerPointsMat gP;
  VectorXd gT;
  traj.propagateGrad(gdC, VectorXd::Zero(traj.getPieceNum()), gP, gT);
  return flatten(gP);
}

VectorXd l2TrackGrad(const Scene &s, const Minco &traj, const Minco &ref,
                     double lambda)
{
  Minco::CoeffMat gdC(Minco::COEFF_NUM * traj.getPieceNum(), 3);
  gdC.setZero();
  for (int p = 0; p < traj.getPieceNum(); ++p)
  {
    const MatrixXd M = derivativeMass(0, traj.getDurations()(p));
    const auto C = traj.getCoefficients().block<Minco::COEFF_NUM, 3>(
        p * Minco::COEFF_NUM, 0);
    const auto Cr = ref.getCoefficients().block<Minco::COEFF_NUM, 3>(
        p * Minco::COEFF_NUM, 0);
    gdC.block<Minco::COEFF_NUM, 3>(p * Minco::COEFF_NUM, 0) =
        lambda * M * (C - Cr);
  }
  Minco::InnerPointsMat gP;
  VectorXd gT;
  traj.propagateGrad(gdC, VectorXd::Zero(traj.getPieceNum()), gP, gT);
  return flatten(gP);
}

double l2TrackCost(const Minco &traj, const Minco &ref, double lambda)
{
  double J = 0.0;
  for (int p = 0; p < traj.getPieceNum(); ++p)
  {
    const MatrixXd M = derivativeMass(0, traj.getDurations()(p));
    const auto C = traj.getCoefficients().block<Minco::COEFF_NUM, 3>(
        p * Minco::COEFF_NUM, 0);
    const auto Cr = ref.getCoefficients().block<Minco::COEFF_NUM, 3>(
        p * Minco::COEFF_NUM, 0);
    const auto D = C - Cr;
    J += 0.5 * lambda * (D.transpose() * M * D).trace();
  }
  return J;
}

VectorXd derivQuadGrad(const Minco &traj, int deriv, double w)
{
  Minco::CoeffMat gdC(Minco::COEFF_NUM * traj.getPieceNum(), 3);
  gdC.setZero();
  for (int p = 0; p < traj.getPieceNum(); ++p)
  {
    const MatrixXd Q = derivativeMass(deriv, traj.getDurations()(p));
    const auto C = traj.getCoefficients().block<Minco::COEFF_NUM, 3>(
        p * Minco::COEFF_NUM, 0);
    gdC.block<Minco::COEFF_NUM, 3>(p * Minco::COEFF_NUM, 0) = w * Q * C;
  }
  Minco::InnerPointsMat gP;
  VectorXd gT;
  traj.propagateGrad(gdC, VectorXd::Zero(traj.getPieceNum()), gP, gT);
  return flatten(gP);
}

double derivQuadCost(const Minco &traj, int deriv, double w)
{
  double J = 0.0;
  for (int p = 0; p < traj.getPieceNum(); ++p)
  {
    const MatrixXd Q = derivativeMass(deriv, traj.getDurations()(p));
    const auto C = traj.getCoefficients().block<Minco::COEFF_NUM, 3>(
        p * Minco::COEFF_NUM, 0);
    J += 0.5 * w * (C.transpose() * Q * C).trace();
  }
  return J;
}

struct Corridor
{
  double y_max{0.08};
  int samples_per_piece{8};
  double weight{25.0};
};

double corridorCostGrad(const Scene &s, const Minco &traj, const JvpBasis &basis,
                        const Corridor &cor, VectorXd *grad, MatrixXd *gn)
{
  double J = 0.0;
  const int m = s.dim();
  if (grad != nullptr)
  {
    grad->setZero(m);
  }
  if (gn != nullptr)
  {
    *gn = MatrixXd::Zero(m, m);
  }
  for (int p = 0; p < traj.getPieceNum(); ++p)
  {
    const double T = traj.getDurations()(p);
    for (int k = 0; k <= cor.samples_per_piece; ++k)
    {
      const double tau = static_cast<double>(k) / static_cast<double>(cor.samples_per_piece);
      const double t_local = tau * T;
      const double w = ((k == 0 || k == cor.samples_per_piece) ? 0.5 : 1.0) * T /
                       static_cast<double>(cor.samples_per_piece);
      const Vector3d pos = traj.evaluate(
          (p == 0 ? 0.0 : traj.getDurations().head(p).sum()) + t_local, 0);
      const double viol = std::abs(pos.y()) - cor.y_max;
      if (viol <= 0.0)
      {
        continue;
      }
      J += cor.weight * 0.5 * w * viol * viol;
      const double d_d_py = cor.weight * w * viol * (pos.y() >= 0.0 ? 1.0 : -1.0);
      VectorXd jrow = VectorXd::Zero(m);
      const double t_abs =
          (p == 0 ? 0.0 : traj.getDurations().head(p).sum()) + t_local;
      for (int i = 0; i < m; ++i)
      {
        const Vector3d dp =
            basis.basis[static_cast<std::size_t>(i)].getPos(t_abs);
        jrow(i) = dp.y();
        if (grad != nullptr)
        {
          (*grad)(i) += d_d_py * dp.y();
        }
      }
      if (gn != nullptr)
      {
        const double s = (pos.y() >= 0.0 ? 1.0 : -1.0);
        *gn += (cor.weight * w) * (s * jrow) * (s * jrow).transpose();
      }
    }
  }
  return J;
}

double cosineCostGrad(const Minco &traj, const JvpBasis &basis, double kappa,
                      double omega, VectorXd *grad)
{
  double J = 0.0;
  const int m = static_cast<int>(basis.basis.size());
  if (grad != nullptr)
  {
    grad->setZero(m);
  }
  const double Ttot = traj.getTotalDuration();
  const int n = 80;
  for (int k = 0; k <= n; ++k)
  {
    const double t = Ttot * static_cast<double>(k) / static_cast<double>(n);
    const double w =
        ((k == 0 || k == n) ? 0.5 : 1.0) * Ttot / static_cast<double>(n);
    const Vector3d p = traj.getPos(t);
    J += kappa * w * std::cos(omega * p.x());
    if (grad != nullptr)
    {
      const double dpx = -kappa * w * omega * std::sin(omega * p.x());
      for (int i = 0; i < m; ++i)
      {
        (*grad)(i) += dpx * basis.basis[static_cast<std::size_t>(i)].getPos(t).x();
      }
    }
  }
  return J;
}

enum class ObjKind
{
  PureMce,
  MceL2,
  MceDyn,
  MceCorr,
  PlannerLike,
  MceCos
};

struct Weights
{
  double energy{1.0};
  double track{0.0};
  double vel{0.0};
  double acc{0.0};
  double corr{0.0};
  double cosk{0.0};
  double omega{5.0};
};

Weights weightsFor(ObjKind k)
{
  Weights w;
  switch (k)
  {
  case ObjKind::PureMce:
    break;
  case ObjKind::MceL2:
    w.track = 8.0;
    break;
  case ObjKind::MceDyn:
    w.vel = 0.4;
    w.acc = 0.15;
    break;
  case ObjKind::MceCorr:
    w.corr = 1.0;
    break;
  case ObjKind::PlannerLike:
    w.energy = 1.0;
    w.track = 6.0;
    w.vel = 0.25;
    w.acc = 0.08;
    w.corr = 1.0;
    break;
  case ObjKind::MceCos:
    w.cosk = 2.5;
    break;
  }
  return w;
}

const char *objName(ObjKind k)
{
  switch (k)
  {
  case ObjKind::PureMce:
    return "Pure-MCE";
  case ObjKind::MceL2:
    return "MCE+L2";
  case ObjKind::MceDyn:
    return "MCE+Dyn";
  case ObjKind::MceCorr:
    return "MCE+Corridor";
  case ObjKind::PlannerLike:
    return "Planner-like";
  case ObjKind::MceCos:
    return "MCE+Cosine";
  }
  return "?";
}

struct Problem
{
  Scene scene;
  JvpBasis basis;
  Minco ref;
  Corridor cor;
  Weights w;
  ObjKind kind{ObjKind::PureMce};
};

double evalJg(const Problem &pr, const VectorXd &P, VectorXd *g,
              MatrixXd *gn_corr)
{
  const Minco traj = generate(pr.scene, P);
  double J = 0.0;
  VectorXd grad = VectorXd::Zero(P.size());
  if (pr.w.energy != 0.0)
  {
    J += pr.w.energy * traj.getEnergy();
    if (g != nullptr)
    {
      grad += pr.w.energy * energyGrad(traj);
    }
  }
  if (pr.w.track != 0.0)
  {
    J += l2TrackCost(traj, pr.ref, pr.w.track);
    if (g != nullptr)
    {
      grad += l2TrackGrad(pr.scene, traj, pr.ref, pr.w.track);
    }
  }
  if (pr.w.vel != 0.0)
  {
    J += derivQuadCost(traj, 1, pr.w.vel);
    if (g != nullptr)
    {
      grad += derivQuadGrad(traj, 1, pr.w.vel);
    }
  }
  if (pr.w.acc != 0.0)
  {
    J += derivQuadCost(traj, 2, pr.w.acc);
    if (g != nullptr)
    {
      grad += derivQuadGrad(traj, 2, pr.w.acc);
    }
  }
  if (pr.w.corr != 0.0)
  {
    VectorXd gc;
    MatrixXd gn;
    J += corridorCostGrad(pr.scene, traj, pr.basis, pr.cor, &gc,
                          gn_corr != nullptr ? &gn : nullptr);
    if (g != nullptr)
    {
      grad += gc;
    }
    if (gn_corr != nullptr)
    {
      *gn_corr = gn;
    }
  }
  if (pr.w.cosk != 0.0)
  {
    VectorXd gc;
    J += cosineCostGrad(traj, pr.basis, pr.w.cosk, pr.w.omega, &gc);
    if (g != nullptr)
    {
      grad += gc;
    }
  }
  if (g != nullptr)
  {
    *g = grad;
  }
  return J;
}

MatrixXd fdHessian(const Problem &pr, const VectorXd &P, double eta)
{
  const int n = static_cast<int>(P.size());
  MatrixXd H(n, n);
  for (int j = 0; j < n; ++j)
  {
    const double h = eta * std::max(1.0, std::abs(P(j)));
    VectorXd ej = VectorXd::Zero(n);
    ej(j) = 1.0;
    VectorXd gp, gm;
    evalJg(pr, P + h * ej, &gp, nullptr);
    evalJg(pr, P - h * ej, &gm, nullptr);
    H.col(j) = (gp - gm) / (2.0 * h);
  }
  return 0.5 * (H + H.transpose());
}

MatrixXd regularizeSpd(const MatrixXd &G, double *eps_used)
{
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(G);
  require(es.info() == Eigen::Success, "metric eigendecomposition failed");
  const double lmax = es.eigenvalues().maxCoeff();
  const double lmin = es.eigenvalues().minCoeff();
  double eps = 0.0;
  if (lmin < 1.0e-12 * std::max(1.0, lmax))
  {
    eps = 1.0e-10 * std::max(1.0, lmax) - std::min(0.0, lmin);
  }
  *eps_used = eps;
  return G + eps * MatrixXd::Identity(G.rows(), G.cols());
}

MatrixXd whiteHessian(const MatrixXd &G, const MatrixXd &H)
{
  Eigen::LLT<MatrixXd> llt(G);
  require(llt.info() == Eigen::Success, "metric Cholesky failed");
  const MatrixXd L = llt.matrixL();
  const MatrixXd Y = L.triangularView<Eigen::Lower>().solve(H);
  const MatrixXd W =
      L.triangularView<Eigen::Lower>().solve(Y.transpose()).transpose();
  return 0.5 * (W + W.transpose());
}

struct Spectrum
{
  int n{0};
  int n_neg{0};
  int n_pos{0};
  int n_zero{0};
  double lmin{0.0};
  double lmax{0.0};
  double lmin_pos{0.0};
  double lmax_pos{0.0};
  double lmin_neg{0.0};
  double kappa{std::numeric_limits<double>::infinity()};
  double kappa_plus{std::numeric_limits<double>::infinity()};
  VectorXd eig;
};

Spectrum analyzeSpectrum(const MatrixXd &H)
{
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(H);
  require(es.info() == Eigen::Success, "Hessian eigendecomposition failed");
  Spectrum s;
  s.eig = es.eigenvalues();
  s.n = static_cast<int>(s.eig.size());
  s.lmin = s.eig.minCoeff();
  s.lmax = s.eig.maxCoeff();
  const double thr = kLamEpsRel * std::max(1.0, std::abs(s.lmax));
  s.lmin_pos = std::numeric_limits<double>::infinity();
  s.lmax_pos = 0.0;
  s.lmin_neg = 0.0;
  for (int i = 0; i < s.n; ++i)
  {
    const double l = s.eig(i);
    if (l > thr)
    {
      ++s.n_pos;
      s.lmin_pos = std::min(s.lmin_pos, l);
      s.lmax_pos = std::max(s.lmax_pos, l);
    }
    else if (l < -thr)
    {
      ++s.n_neg;
      s.lmin_neg = std::min(s.lmin_neg, l);
    }
    else
    {
      ++s.n_zero;
    }
  }
  if (s.n_neg == 0 && s.n_pos > 0 && s.lmin > 0.0)
  {
    s.kappa = s.lmax / s.lmin;
  }
  if (s.n_pos >= 2)
  {
    s.kappa_plus = s.lmax_pos / s.lmin_pos;
  }
  else if (s.n_pos == 1)
  {
    s.kappa_plus = 1.0;
  }
  return s;
}

double kappaUsed(const Spectrum &s)
{
  if (std::isfinite(s.kappa))
  {
    return s.kappa;
  }
  return s.kappa_plus;
}

struct CondRow
{
  std::string case_name;
  std::string metric_name;
  int n{0};
  double e_H{0.0};
  double kappa_E{0.0};
  double kappa_G{0.0};
  double R{0.0};
  int n_neg{0};
  double t_metric{0.0};
  int iters{0};
  int evals{0};
  double J_final{0.0};
  double t_opt{0.0};
  bool pass_white{true};
};

void printRow(const CondRow &r)
{
  std::cout << "  " << std::left << std::setw(16) << r.case_name << "  "
            << std::setw(10) << r.metric_name
            << "  n=" << std::setw(3) << r.n
            << "  kE=" << sci(r.kappa_E)
            << "  kG=" << sci(r.kappa_G)
            << "  R=" << sci(r.R, 3)
            << "  neg=" << r.n_neg
            << "  it=" << r.iters
            << "  ev=" << r.evals
            << "  J*=" << sci(r.J_final, 3)
            << "\n";
}

struct LbfgsState
{
  const Problem *pr{nullptr};
  MatrixXd L;
  bool whitened{false};
  int evals{0};
  int iters{0};
  int ls_sum{0};
  std::vector<double> costs;
};

VectorXd toPhys(const LbfgsState &st, const VectorXd &z)
{
  if (!st.whitened)
  {
    return z;
  }
  return st.L.transpose().triangularView<Eigen::Upper>().solve(z);
}

double lbfgsEval(void *ptr, const VectorXd &z, VectorXd &g)
{
  auto *st = static_cast<LbfgsState *>(ptr);
  ++st->evals;
  const VectorXd P = toPhys(*st, z);
  VectorXd gP;
  const double J = evalJg(*st->pr, P, &gP, nullptr);
  if (st->whitened)
  {
    g = st->L.triangularView<Eigen::Lower>().solve(gP);
  }
  else
  {
    g = gP;
  }
  return J;
}

int lbfgsProg(void *ptr, const VectorXd &, const VectorXd &, const double fx,
              const double, const int, const int ls)
{
  auto *st = static_cast<LbfgsState *>(ptr);
  ++st->iters;
  st->ls_sum += std::max(0, ls);
  st->costs.push_back(fx);
  return 0;
}

struct OptResult
{
  int iters{0};
  int evals{0};
  int ls{0};
  double J{0.0};
  double t_s{0.0};
  std::vector<double> costs;
};

OptResult runFrozenLbfgs(const Problem &pr, const VectorXd &P0, const MatrixXd &G,
                         bool whitened)
{
  LbfgsState st;
  st.pr = &pr;
  st.whitened = whitened;
  double eps = 0.0;
  const MatrixXd Gr = regularizeSpd(G, &eps);
  Eigen::LLT<MatrixXd> llt(Gr);
  require(llt.info() == Eigen::Success, "opt Cholesky failed");
  st.L = llt.matrixL();
  VectorXd x = whitened ? VectorXd(st.L.transpose() * P0) : P0;
  VectorXd g0(x.size());
  st.costs.push_back(lbfgsEval(&st, x, g0));
  math_utils::lbfgs::lbfgs_parameter_t p;
  p.mem_size = 16;
  p.g_epsilon = 1.0e-8;
  p.past = 3;
  p.delta = 1.0e-12;
  p.max_iterations = 200;
  p.max_linesearch = 64;
  double f = 0.0;
  const auto t0 = Clock::now();
  math_utils::lbfgs::lbfgs_optimize(x, f, &lbfgsEval, nullptr, &lbfgsProg, &st, p);
  OptResult r;
  r.iters = st.iters;
  r.evals = st.evals;
  r.ls = st.ls_sum;
  r.J = f;
  r.costs = st.costs;
  r.t_s = std::chrono::duration<double>(Clock::now() - t0).count();
  return r;
}

Problem makeProblem(const Scene &scene, ObjKind kind)
{
  Problem pr;
  pr.scene = scene;
  pr.basis = buildBasis(scene);
  pr.kind = kind;
  pr.w = weightsFor(kind);
  Scene ref_scene = scene;
  for (int i = 0; i < scene.innerCols(); ++i)
  {
    const double u = static_cast<double>(i + 1) / static_cast<double>(scene.pieces);
    ref_scene.P0.col(i) << u * scene.tail(0, 0), 0.0, 1.0;
  }
  pr.ref = generate(ref_scene, flatten(ref_scene.P0));
  return pr;
}

CondRow evalCase(const Problem &pr, const std::string &metric_name,
                 const MatrixXd &G_raw, const MatrixXd &H, bool run_opt,
                 const VectorXd &P0, std::vector<double> *cost_out)
{
  CondRow row;
  row.case_name = objName(pr.kind);
  row.metric_name = metric_name;
  row.n = static_cast<int>(P0.size());
  row.t_metric = pr.basis.t_build_s;
  const Spectrum spE = analyzeSpectrum(H);
  row.kappa_E = kappaUsed(spE);
  row.n_neg = spE.n_neg;
  double eps = 0.0;
  const MatrixXd G = regularizeSpd(G_raw, &eps);
  const MatrixXd Hw = whiteHessian(G, H);
  const Spectrum spG = analyzeSpectrum(Hw);
  row.kappa_G = kappaUsed(spG);
  row.R = row.kappa_G > 0.0 ? row.kappa_E / row.kappa_G : 0.0;
  if (metric_name == "MCE" && pr.kind == ObjKind::PureMce)
  {
    row.e_H = (H - pr.basis.G_mce).norm() / std::max(1.0, H.norm());
    row.pass_white = (row.e_H < 1.0e-3) && (row.kappa_G < 1.05);
  }
  if (run_opt)
  {
    const bool white = metric_name != "I";
    const MatrixXd Gopt =
        white ? G : MatrixXd::Identity(G.rows(), G.cols());
    const OptResult opt = runFrozenLbfgs(pr, P0, Gopt, white);
    row.iters = opt.iters;
    row.evals = opt.evals;
    row.J_final = opt.J;
    row.t_opt = opt.t_s;
    if (cost_out != nullptr)
    {
      *cost_out = opt.costs;
    }
  }
  return row;
}

void writeCsv(const std::string &path, const std::string &header,
              const std::vector<std::string> &lines)
{
  std::ofstream out(path);
  require(out.good(), "write failed: " + path);
  out << header << "\n";
  for (const auto &ln : lines)
  {
    out << ln << "\n";
  }
}

} // namespace

int main()
{
  try
  {
    std::cout << std::setprecision(6);
    std::cout << "MCE / intrinsic-metric condition-number validation\n";
    std::cout << "kappa(H) vs kappa(G^{-1/2} H G^{-1/2}), frozen-metric L-BFGS\n";

#ifndef ROOT_DIR
#define ROOT_DIR "./"
#endif
    const std::string root = ROOT_DIR;
    std::vector<CondRow> table;
    std::vector<std::string> spec_lines;
    std::vector<std::string> cost_lines;
    std::vector<std::string> scale_lines;

    int n_pass = 0;
    int n_fail = 0;
    auto record = [&](const CondRow &r, bool must_improve) {
      table.push_back(r);
      printRow(r);
      bool ok = std::isfinite(r.kappa_E) && std::isfinite(r.kappa_G);
      if (r.metric_name == "MCE" && r.case_name == std::string("Pure-MCE"))
      {
        ok = r.pass_white;
      }
      else if (must_improve &&
               (r.metric_name == "MCE" || r.metric_name == "Mix" ||
                r.metric_name == "MCE+GN"))
      {
        ok = ok && (r.kappa_G <= r.kappa_E * 1.05 + 1.0e-12);
      }
      n_pass += static_cast<int>(ok);
      n_fail += static_cast<int>(!ok);
      if (!ok)
      {
        std::cout << "    [FAIL] conditioning check\n";
      }
    };

    // ---- 1. Pure MCE, several M and time ratios ----
    std::cout << "\n== 1. Pure-MCE Hessian vs G_MCE ==\n";
    const int Ms[4] = {3, 5, 10, 20};
    for (int M : Ms)
    {
      const Scene sc = makeUniform(M);
      const Problem pr = makeProblem(sc, ObjKind::PureMce);
      const VectorXd P = flatten(sc.P0);
      const MatrixXd H = fdHessian(pr, P, 1.0e-6);
      const double eH =
          (H - pr.basis.G_mce).norm() / std::max(1.0, H.norm());
      std::cout << "  M=" << M << "  n=" << P.size() << "  e_H=" << sci(eH)
                << "  t_metric=" << sci(pr.basis.t_build_s, 3) << "s\n";
      const Spectrum sp = analyzeSpectrum(H);
      double eps_tmp = 0.0;
      const Spectrum spW = analyzeSpectrum(
          whiteHessian(regularizeSpd(pr.basis.G_mce, &eps_tmp), H));
      (void)spW;
      CondRow rI = evalCase(pr, "I", MatrixXd::Identity(P.size(), P.size()), H,
                            M == 5, P, nullptr);
      std::vector<double> c_mce;
      CondRow rM = evalCase(pr, "MCE", pr.basis.G_mce, H, M == 5, P, &c_mce);
      rM.e_H = eH;
      record(rI, false);
      record(rM, true);
      scale_lines.push_back(std::to_string(M) + ",uniform,I," +
                            sci(rI.kappa_E, 8) + "," + sci(rI.kappa_G, 8));
      scale_lines.push_back(std::to_string(M) + ",uniform,MCE," +
                            sci(rM.kappa_E, 8) + "," + sci(rM.kappa_G, 8));
      if (M == 5)
      {
        const Spectrum eigsE = analyzeSpectrum(H);
        const Spectrum eigsG =
            analyzeSpectrum(whiteHessian(pr.basis.G_mce, H));
        for (int i = 0; i < eigsE.n; ++i)
        {
          spec_lines.push_back("Pure-MCE,I," + std::to_string(i) + "," +
                               sci(eigsE.eig(i), 10));
          spec_lines.push_back("Pure-MCE,MCE," + std::to_string(i) + "," +
                               sci(eigsG.eig(i), 10));
        }
        for (std::size_t k = 0; k < c_mce.size(); ++k)
        {
          cost_lines.push_back("Pure-MCE,MCE," + std::to_string(k) + "," +
                               sci(c_mce[k], 12));
        }
      }
    }

    std::cout << "\n== 1b. Pure-MCE vs segment-time ratio (M=5) ==\n";
    const std::vector<std::pair<std::string, std::vector<double>>> ratios = {
        {"uniform", {1, 1, 1, 1, 1}},
        {"mild", {0.7, 1.0, 1.3, 0.9, 1.1}},
        {"strong", {0.3, 0.6, 1.2, 1.8, 1.1}},
        {"extreme", {0.2, 0.4, 1.5, 2.0, 0.9}},
    };
    for (const auto &rt : ratios)
    {
      Eigen::VectorXd T(5);
      for (int i = 0; i < 5; ++i)
      {
        T(i) = rt.second[static_cast<std::size_t>(i)];
      }
      const Scene sc = makeScene(5, T);
      const Problem pr = makeProblem(sc, ObjKind::PureMce);
      const VectorXd P = flatten(sc.P0);
      const MatrixXd H = fdHessian(pr, P, 1.0e-6);
      CondRow rI = evalCase(pr, "I", MatrixXd::Identity(P.size(), P.size()), H,
                            false, P, nullptr);
      CondRow rM = evalCase(pr, "MCE", pr.basis.G_mce, H, false, P, nullptr);
      rI.case_name = rt.first;
      rM.case_name = rt.first;
      record(rI, false);
      record(rM, true);
      scale_lines.push_back("5," + rt.first + ",I," + sci(rI.kappa_E, 8) + "," +
                            sci(rI.kappa_G, 8));
      scale_lines.push_back("5," + rt.first + ",MCE," + sci(rM.kappa_E, 8) + "," +
                            sci(rM.kappa_G, 8));
    }

    // ---- 2-7 mixed objectives on M=5 uniform ----
    const Scene sc5 = makeUniform(5);
    const VectorXd P5 = flatten(sc5.P0);
    const ObjKind kinds[5] = {ObjKind::MceL2, ObjKind::MceDyn, ObjKind::MceCorr,
                              ObjKind::PlannerLike, ObjKind::MceCos};
    std::cout << "\n== 2-7. Mixed objectives, metric comparison, frozen L-BFGS ==\n";
    for (ObjKind kind : kinds)
    {
      std::cout << "\n-- " << objName(kind) << " --\n";
      const Problem pr = makeProblem(sc5, kind);
      const MatrixXd H = fdHessian(pr, P5, 1.0e-6);
      const Spectrum spH = analyzeSpectrum(H);
      std::cout << "  lambda_min=" << sci(spH.lmin) << "  lambda_max="
                << sci(spH.lmax) << "  n_neg=" << spH.n_neg
                << "  k+=" << sci(spH.kappa_plus) << "\n";

      MatrixXd Ggn = MatrixXd::Zero(P5.size(), P5.size());
      if (kind == ObjKind::MceCorr || kind == ObjKind::PlannerLike)
      {
        evalJg(pr, P5, nullptr, &Ggn);
      }
      const MatrixXd Gmix = pr.basis.G_l2 + pr.basis.G_mce;
      const MatrixXd Gmce_gn = pr.basis.G_mce + Ggn;

      struct MetricPick
      {
        const char *name;
        const MatrixXd *G;
      };
      const MetricPick picks[] = {
          {"I", nullptr},
          {"L2", &pr.basis.G_l2},
          {"H2", &pr.basis.G_h2},
          {"MCE", &pr.basis.G_mce},
          {"Mix", &Gmix},
          {"MCE+GN", &Gmce_gn},
      };
      for (const auto &pk : picks)
      {
        if (std::string(pk.name) == "MCE+GN" && Ggn.norm() == 0.0)
        {
          continue;
        }
        std::vector<double> costs;
        const MatrixXd Guse = (pk.G == nullptr)
                                  ? MatrixXd::Identity(P5.size(), P5.size())
                                  : *pk.G;
        CondRow r = evalCase(pr, pk.name, Guse, H, true, P5, &costs);
        record(r, pk.name != std::string("I"));
        for (std::size_t k = 0; k < costs.size(); ++k)
        {
          cost_lines.push_back(std::string(objName(kind)) + "," + pk.name + "," +
                               std::to_string(k) + "," + sci(costs[k], 12));
        }
        if (std::string(pk.name) == "I" || std::string(pk.name) == "MCE" ||
            std::string(pk.name) == "Mix")
        {
          double eps_s = 0.0;
          const Spectrum eigs =
              analyzeSpectrum(std::string(pk.name) == "I"
                                  ? H
                                  : whiteHessian(regularizeSpd(Guse, &eps_s), H));
          for (int i = 0; i < eigs.n; ++i)
          {
            spec_lines.push_back(std::string(objName(kind)) + "," + pk.name +
                                 "," + std::to_string(i) + "," +
                                 sci(eigs.eig(i), 10));
          }
        }
      }
    }

    writeCsv(root + "Tests/mce_conditioning_table.csv",
             "case,metric,n,kappa_E,kappa_G,R,n_neg,iters,evals,J_final,t_opt,t_metric",
             [&]() {
               std::vector<std::string> ls;
               for (const auto &r : table)
               {
                 ls.push_back(r.case_name + "," + r.metric_name + "," +
                              std::to_string(r.n) + "," + sci(r.kappa_E, 8) +
                              "," + sci(r.kappa_G, 8) + "," + sci(r.R, 8) + "," +
                              std::to_string(r.n_neg) + "," +
                              std::to_string(r.iters) + "," +
                              std::to_string(r.evals) + "," +
                              sci(r.J_final, 10) + "," + sci(r.t_opt, 6) + "," +
                              sci(r.t_metric, 6));
               }
               return ls;
             }());
    writeCsv(root + "Tests/mce_conditioning_spectrum.csv",
             "case,metric,index,eigenvalue", spec_lines);
    writeCsv(root + "Tests/mce_conditioning_cost_traces.csv",
             "case,metric,iter,J", cost_lines);
    writeCsv(root + "Tests/mce_conditioning_scaling.csv",
             "M,time_pattern,metric,kappa_E,kappa_G", scale_lines);

    std::cout << "\n== SUMMARY ==\n";
    std::cout << "  rows=" << table.size() << "  checks passed=" << n_pass
              << " / " << (n_pass + n_fail) << "\n";
    require(n_fail == 0, "one or more conditioning checks failed");
    std::cout << "[minco_metric_conditioning_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[minco_metric_conditioning_self_test] FAIL: " << ex.what()
              << "\n";
    return 1;
  }
}
