/**
 * Level C: free-time joint Hessian conditioning ablation.
 *
 * C0 Euclidean G=I
 * C1 waypoint-only (Production V1 failure mode)
 * C2 block space-time  G_τ^{rel} ⊕ G_PP
 * C3 full space-time GN + G_T^{rel}
 * C4 C3 + active corridor Gauss-Newton
 *
 * Hessian is finite-differenced once in (τ,P), then congruenced by each G.
 */

#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/minco/minco_joint_whitening.hpp"
#include "traj_opt/minco/minco_metric.hpp"
#include "traj_opt/minco/minco_whitening.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

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
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  double y_max{0.10};
  const char *label{"snap"};
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
  s.label = "rolling";
  return s;
}

Snapshot makeUneven(int pieces, double ratio)
{
  Snapshot s = makeSnapshot(0, pieces);
  s.times = VectorXd::LinSpaced(pieces, 1.0, ratio);
  s.times *= (s.times.size() / s.times.sum()) * (70.0 / 8.0);
  s.label = "uneven";
  return s;
}

struct Grad
{
  double cost{0.0};
  VectorXd gT;
  VectorXd gP;
};

Grad evaluatePhysical(const Snapshot &snap, const Weights &w, const VectorXd &P,
                      const VectorXd &T)
{
  Trajectory traj;
  require(traj.generate(unflatten(P, snap.inner()), snap.head, snap.tail, T),
          "MINCO generate failed");
  Grad out;
  CoeffMat gdC = CoeffMat::Zero(Trajectory::COEFF_NUM * snap.pieces, 3);
  VectorXd gdT = VectorXd::Zero(snap.pieces);

  double energy = 0.0;
  CoeffMat gdC_e;
  VectorXd gdT_e;
  traj.getEnergyPartialGradByCoeffs(energy, gdC_e);
  traj.getEnergyPartialGradByTimes(gdT_e);
  out.cost = w.energy * energy + w.time * T.sum();
  gdC = w.energy * gdC_e;
  gdT = w.energy * gdT_e;
  gdT.array() += w.time;

  const auto &C = traj.getCoefficients();
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
      c_val += 0.5 * w.vel * v.squaredNorm();
      gv += w.vel * v;
      c_val += 0.5 * w.acc * a.squaredNorm();
      ga += w.acc * a;
      const double viol = std::abs(p.y()) - snap.y_max;
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

VectorXd packTauP(const VectorXd &tau, const VectorXd &P)
{
  VectorXd x(tau.size() + P.size());
  x << tau, P;
  return x;
}

void unpackTauP(const VectorXd &x, int m, VectorXd &tau, VectorXd &P)
{
  tau = x.head(m);
  P = x.tail(x.size() - m);
}

VectorXd solverGrad(const Snapshot &snap, const Weights &w, const VectorXd &x,
                    temporal_map::QuadInvTimeMap &tmap)
{
  const int m = snap.pieces;
  VectorXd tau, P;
  unpackTauP(x, m, tau, P);
  VectorXd T(m);
  for (int i = 0; i < m; ++i)
  {
    T(i) = tmap.toTime(tau(i));
  }
  const Grad ev = evaluatePhysical(snap, w, P, T);
  VectorXd g = VectorXd::Zero(x.size());
  for (int i = 0; i < m; ++i)
  {
    g(i) = tmap.backward(tau(i), T(i), ev.gT(i));
  }
  g.tail(snap.spatial()) = ev.gP;
  return g;
}

MatrixXd fdHessian(const Snapshot &snap, const Weights &w, const VectorXd &x,
                   temporal_map::QuadInvTimeMap &tmap)
{
  const int n = static_cast<int>(x.size());
  MatrixXd H = MatrixXd::Zero(n, n);
  const double eta = 1.0e-5;
  for (int j = 0; j < n; ++j)
  {
    const double h = eta * std::max(1.0, std::abs(x(j)));
    VectorXd xp = x;
    VectorXd xm = x;
    xp(j) += h;
    xm(j) -= h;
    H.col(j) = (solverGrad(snap, w, xp, tmap) - solverGrad(snap, w, xm, tmap)) /
               (2.0 * h);
  }
  return 0.5 * (H + H.transpose());
}

struct Spec
{
  double lmin{0.0};
  double lmax{0.0};
  double kappa{0.0};
  double kappa_plus{0.0};
  int n_neg{0};
};

Spec spectrum(const MatrixXd &A)
{
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(0.5 * (A + A.transpose()));
  require(es.info() == Eigen::Success, "eigendecomposition failed");
  Spec s;
  const auto &lam = es.eigenvalues();
  double amax = 0.0;
  double amin = 1.0e300;
  double pmin = 1.0e300;
  double pmax = 0.0;
  for (int i = 0; i < lam.size(); ++i)
  {
    if (lam(i) < -1.0e-8)
    {
      s.n_neg += 1;
    }
    const double a = std::abs(lam(i));
    amax = std::max(amax, a);
    if (a > 1.0e-12)
    {
      amin = std::min(amin, a);
    }
    if (lam(i) > 1.0e-12)
    {
      pmin = std::min(pmin, lam(i));
      pmax = std::max(pmax, lam(i));
    }
  }
  s.lmin = amin;
  s.lmax = amax;
  s.kappa = amax / std::max(1.0e-30, amin);
  s.kappa_plus = pmax / std::max(1.0e-30, pmin);
  return s;
}

std::string sci(double v)
{
  std::ostringstream oss;
  oss << std::scientific << std::setprecision(3) << v;
  return oss.str();
}

MatrixXd whiteHessian(const MatrixXd &G, const MatrixXd &H)
{
  Eigen::LLT<MatrixXd> llt(0.5 * (G + G.transpose()));
  require(llt.info() == Eigen::Success, "Cholesky of G failed");
  const MatrixXd L = llt.matrixL();
  const MatrixXd Y = L.triangularView<Eigen::Lower>().solve(H);
  const MatrixXd W =
      L.triangularView<Eigen::Lower>().solve(Y.transpose()).transpose();
  return 0.5 * (W + W.transpose());
}

VectorXd dTdtau(const VectorXd &tau, const VectorXd &T,
                temporal_map::QuadInvTimeMap &tmap)
{
  VectorXd d(tau.size());
  for (int i = 0; i < tau.size(); ++i)
  {
    d(i) = tmap.backward(tau(i), T(i), 1.0);
  }
  return d;
}

MatrixXd buildC1(const MatrixXd &Gpp, int m)
{
  MatrixXd G = MatrixXd::Identity(m + Gpp.rows(), m + Gpp.rows());
  G.bottomRightCorner(Gpp.rows(), Gpp.cols()) = Gpp;
  return G;
}

MatrixXd buildC2(const VectorXd &T, const VectorXd &dtau, const MatrixXd &Gpp,
                 double lambda_T)
{
  const int m = static_cast<int>(T.size());
  MatrixXd G = MatrixXd::Zero(m + Gpp.rows(), m + Gpp.rows());
  for (int i = 0; i < m; ++i)
  {
    const double gT = lambda_T / std::max(1.0e-12, T(i) * T(i));
    G(i, i) = gT * dtau(i) * dtau(i);
  }
  G.bottomRightCorner(Gpp.rows(), Gpp.cols()) = Gpp;
  return G;
}

MatrixXd corridorGN(const Snapshot &snap, const Weights &w, const VectorXd &P,
                    const VectorXd &T)
{
  const int m = snap.pieces;
  const int pdim = snap.spatial();
  const int n = m + pdim;
  Trajectory traj;
  require(traj.generate(unflatten(P, snap.inner()), snap.head, snap.tail, T),
          "corridor seed generate failed");
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
      const double t_local = alpha * Ti;
      typename Trajectory::BasisRow b_p, b_v, b_a, b_j, b_s;
      Trajectory::computeBasisFunctions(t_local, b_p, b_v, b_a, b_j, b_s);
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
          const Vector3d v = (b_v * coeff).transpose();
          dp += alpha * v;
        }
        jr(a) = sgn * dp.y();
      }
      G.noalias() += w.corr * jr * jr.transpose();
    }
  }
  return G;
}

struct Row
{
  const char *name;
  Spec joint;
  Spec hpp;
  Spec htt;
  Spec schur;
  double eta{0.0};
  bool chol_ok{true};
};

Row analyze(const char *name, const MatrixXd &H, const MatrixXd &G, int m)
{
  Row r;
  r.name = name;
  const int pdim = static_cast<int>(H.rows()) - m;
  Eigen::LLT<MatrixXd> llt(0.5 * (G + G.transpose()));
  r.chol_ok = llt.info() == Eigen::Success;
  require(r.chol_ok, std::string(name) + " Cholesky failed");
  const MatrixXd Hw = whiteHessian(G, H);
  r.joint = spectrum(Hw);
  r.hpp = spectrum(Hw.bottomRightCorner(pdim, pdim));
  r.htt = spectrum(Hw.topLeftCorner(m, m));
  const MatrixXd Htt = Hw.topLeftCorner(m, m);
  const MatrixXd Htp = Hw.topRightCorner(m, pdim);
  const MatrixXd Hpp = Hw.bottomRightCorner(pdim, pdim);
  Eigen::LDLT<MatrixXd> ldlt(Hpp);
  MatrixXd S = Htt;
  if (ldlt.info() == Eigen::Success)
  {
    S = Htt - Htp * ldlt.solve(Htp.transpose());
  }
  r.schur = spectrum(S);
  const double den = std::sqrt(Htt.norm() * Hpp.norm());
  r.eta = den > 0.0 ? Htp.norm() / den : 0.0;
  return r;
}

void printRow(const Row &r)
{
  std::cout << "  " << std::setw(28) << r.name
            << "  κ=" << sci(r.joint.kappa)
            << "  |λ|min=" << sci(r.joint.lmin)
            << "  |λ|max=" << sci(r.joint.lmax)
            << "  n-=" << r.joint.n_neg
            << "  κ+=" << sci(r.joint.kappa_plus)
            << "  κPP=" << sci(r.hpp.kappa)
            << "  κττ=" << sci(r.htt.kappa)
            << "  κS=" << sci(r.schur.kappa)
            << "  η=" << sci(r.eta) << "\n";
}

} // namespace

int main()
{
  try
  {
    temporal_map::QuadInvTimeMap tmap;
    Weights w;
    std::cout << "Level C  free-time joint Hessian conditioning\n";
    std::cout << "objective = energy + time + vel/acc + corridor\n\n";

    double geo_c1 = 1.0;
    double geo_c3 = 1.0;
    int n_cases = 0;
    bool gate4 = true;

    auto runCase = [&](const Snapshot &snap) {
      const int m = snap.pieces;
      VectorXd tau(m);
      for (int i = 0; i < m; ++i)
      {
        tau(i) = tmap.toTau(snap.times(i));
      }
      const VectorXd P0 = flatten(snap.P0);
      const VectorXd x = packTauP(tau, P0);
      const VectorXd dtau = dTdtau(tau, snap.times, tmap);

      Trajectory seed;
      require(seed.generate(snap.P0, snap.head, snap.tail, snap.times),
              "seed generate failed");

      minco::MincoMetricOptions wp_opt;
      wp_opt.mode = minco::MincoMetricMode::kFrozenWaypoint;
      wp_opt.regularization = 1.0e-10;
      wp_opt.energy_weight = w.energy;
      minco::MincoMetric<3, 4> wp;
      wp.setOptions(wp_opt);
      require(wp.update(seed), "G_MCE failed");

      minco::MincoMetricOptions full_opt;
      full_opt.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
      full_opt.regularization = 1.0e-10;
      full_opt.time_metric_weight = 1.0;
      full_opt.energy_weight = w.energy;
      minco::MincoMetric<3, 4> full;
      full.setOptions(full_opt);
      require(full.update(seed), "G_full failed");
      MatrixXd G_theta = full.spaceTimeMetric();
      MatrixXd Gx;
      require(minco::pullbackTimeMap(G_theta, dtau, Gx), "pullback failed");

      const MatrixXd Gc_theta = corridorGN(snap, w, P0, snap.times);
      MatrixXd Gc_x;
      require(minco::pullbackTimeMap(Gc_theta, dtau, Gc_x), "corr pullback failed");
      MatrixXd G4 = Gx + Gc_x;
      G4.diagonal().array() += 1.0e-10 * G4.trace() / static_cast<double>(G4.rows());

      const MatrixXd H = fdHessian(snap, w, x, tmap);
      const MatrixXd G0 = MatrixXd::Identity(H.rows(), H.rows());
      const MatrixXd G1 = buildC1(wp.waypointMetric(), m);
      const MatrixXd G2 = buildC2(snap.times, dtau, wp.waypointMetric(), 1.0);

      const Row c0 = analyze("C0 Euclidean", H, G0, m);
      const Row c1 = analyze("C1 waypoint-only V1", H, G1, m);
      const Row c2 = analyze("C2 block space-time", H, G2, m);
      const Row c3 = analyze("C3 full GN + G_Trel", H, Gx, m);
      const Row c4 = analyze("C4 full + corridor GN", H, G4, m);

      const double ratio = snap.times.maxCoeff() / snap.times.minCoeff();
      std::cout << "==== " << snap.label << " idx pieces=" << m
                << "  Tmax/Tmin=" << sci(ratio)
                << "  dim=" << H.rows() << " ====\n";
      printRow(c0);
      printRow(c1);
      printRow(c2);
      printRow(c3);
      printRow(c4);
      std::cout << "  metric η_F(G_θ)=" << sci(full.couplingEta())
                << "  κ(G_x)=" << sci(full.conditionNumber())
                << "  κ_C3/κ_C0=" << sci(c3.joint.kappa / c0.joint.kappa)
                << "  |λmin|_C3/C0=" << sci(c3.joint.lmin / c0.joint.lmin)
                << "\n\n";

      if (c2.joint.kappa > c1.joint.kappa * 1.05)
      {
        std::cout << "  note: C2 did not improve on C1 (block-diagonal time is not enough)\n";
      }
      if (!(c3.joint.kappa < c0.joint.kappa))
      {
        gate4 = false;
      }
      geo_c1 *= c1.joint.kappa / c0.joint.kappa;
      geo_c3 *= c3.joint.kappa / c0.joint.kappa;
      n_cases += 1;
    };

    for (int idx : {0, 4, 9})
    {
      runCase(makeSnapshot(idx, 8));
    }
    runCase(makeUneven(8, 8.0));
    runCase(makeSnapshot(0, 5));
    runCase(makeSnapshot(0, 12));

    geo_c1 = std::pow(geo_c1, 1.0 / static_cast<double>(n_cases));
    geo_c3 = std::pow(geo_c3, 1.0 / static_cast<double>(n_cases));
    std::cout << "geometric mean κ_C1/κ_C0 = " << sci(geo_c1) << "\n";
    std::cout << "geometric mean κ_C3/κ_C0 = " << sci(geo_c3) << "\n";
    std::cout << "Gate 4 (κ_C3 < κ_C0 on every case): " << (gate4 ? "PASS" : "FAIL")
              << "\n\n";

    std::cout << "---- λ_T sweep on rolling M=8 snap0, C3 ----\n";
    {
      const Snapshot snap = makeSnapshot(0, 8);
      VectorXd tau(snap.pieces);
      for (int i = 0; i < snap.pieces; ++i)
      {
        tau(i) = tmap.toTau(snap.times(i));
      }
      const VectorXd x = packTauP(tau, flatten(snap.P0));
      const VectorXd dtau = dTdtau(tau, snap.times, tmap);
      const MatrixXd H = fdHessian(snap, w, x, tmap);
      Trajectory seed;
      require(seed.generate(snap.P0, snap.head, snap.tail, snap.times),
              "sweep seed failed");
      const double lambdas[] = {1.0e-3, 1.0e-2, 1.0e-1, 1.0, 10.0, 100.0};
      for (double lam : lambdas)
      {
        minco::MincoMetricOptions opt;
        opt.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
        opt.regularization = 1.0e-10;
        opt.time_metric_weight = lam;
        opt.energy_weight = w.energy;
        minco::MincoMetric<3, 4> metric;
        metric.setOptions(opt);
        require(metric.update(seed), "λ_T metric failed");
        MatrixXd Gx;
        require(minco::pullbackTimeMap(metric.spaceTimeMetric(), dtau, Gx),
                "λ_T pullback failed");
        const Row row = analyze("C3", H, Gx, snap.pieces);
        std::cout << "  λ_T=" << sci(lam) << "  κ=" << sci(row.joint.kappa)
                  << "  |λ|min=" << sci(row.joint.lmin)
                  << "  n-=" << row.joint.n_neg << "\n";
      }
    }

    std::cout << "\n---- α regularization sweep, C3 ----\n";
    {
      const Snapshot snap = makeSnapshot(0, 8);
      VectorXd tau(snap.pieces);
      for (int i = 0; i < snap.pieces; ++i)
      {
        tau(i) = tmap.toTau(snap.times(i));
      }
      const VectorXd dtau = dTdtau(tau, snap.times, tmap);
      Trajectory seed;
      require(seed.generate(snap.P0, snap.head, snap.tail, snap.times),
              "alpha seed failed");
      minco::MincoMetricOptions opt;
      opt.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
      opt.regularization = 0.0;
      opt.time_metric_weight = 1.0;
      opt.energy_weight = w.energy;
      minco::MincoMetric<3, 4> metric;
      metric.setOptions(opt);
      require(metric.update(seed), "alpha metric failed");
      MatrixXd Gx;
      require(minco::pullbackTimeMap(metric.spaceTimeMetric(), dtau, Gx),
              "alpha pullback failed");
      const double alphas[] = {1.0e-12, 1.0e-10, 1.0e-8, 1.0e-6, 1.0e-4};
      for (double a : alphas)
      {
        MatrixXd G = Gx;
        const double shift = a * G.trace() / static_cast<double>(G.rows());
        G.diagonal().array() += shift;
        Eigen::LLT<MatrixXd> llt(0.5 * (G + G.transpose()));
        Eigen::SelfAdjointEigenSolver<MatrixXd> es(0.5 * (G + G.transpose()));
        const double lmin = es.eigenvalues().minCoeff();
        const double lmax = es.eigenvalues().maxCoeff();
        std::cout << "  α=" << sci(a)
                  << "  chol=" << (llt.info() == Eigen::Success ? "ok" : "FAIL")
                  << "  κ(G)=" << sci(lmax / std::max(1.0e-30, lmin))
                  << "  λmin(G)=" << sci(lmin) << "\n";
      }
    }

    std::cout << "\n---- corridor weight sweep, snap0 M=8 ----\n";
    {
      const Snapshot snap = makeSnapshot(0, 8);
      VectorXd tau(snap.pieces);
      for (int i = 0; i < snap.pieces; ++i)
      {
        tau(i) = tmap.toTau(snap.times(i));
      }
      const VectorXd P0 = flatten(snap.P0);
      const VectorXd x = packTauP(tau, P0);
      const VectorXd dtau = dTdtau(tau, snap.times, tmap);
      Trajectory seed;
      require(seed.generate(snap.P0, snap.head, snap.tail, snap.times),
              "wc seed failed");
      const double wcs[] = {10.0, 1.0e2, 1.0e3, 1.0e4, 1.0e5};
      for (double wc : wcs)
      {
        Weights ww = w;
        ww.corr = wc;
        const MatrixXd H = fdHessian(snap, ww, x, tmap);
        minco::MincoMetricOptions opt;
        opt.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
        opt.regularization = 1.0e-10;
        opt.time_metric_weight = 1.0;
        opt.energy_weight = ww.energy;
        minco::MincoMetric<3, 4> metric;
        metric.setOptions(opt);
        require(metric.update(seed), "wc metric failed");
        MatrixXd Gx;
        require(minco::pullbackTimeMap(metric.spaceTimeMetric(), dtau, Gx),
                "wc pullback failed");
        MatrixXd Gc;
        require(minco::pullbackTimeMap(corridorGN(snap, ww, P0, snap.times), dtau,
                                      Gc),
                "wc corr pullback failed");
        MatrixXd G4 = Gx + Gc;
        G4.diagonal().array() +=
            1.0e-10 * G4.trace() / static_cast<double>(G4.rows());
        const Row c0 = analyze("C0", H, MatrixXd::Identity(H.rows(), H.rows()),
                               snap.pieces);
        const Row c3 = analyze("C3", H, Gx, snap.pieces);
        const Row c4 = analyze("C4", H, G4, snap.pieces);
        std::cout << "  w_c=" << sci(wc)
                  << "  κ0=" << sci(c0.joint.kappa)
                  << "  κ3=" << sci(c3.joint.kappa)
                  << "  κ4=" << sci(c4.joint.kappa)
                  << "  η0=" << sci(c0.eta) << "\n";
      }
    }

    require(n_cases >= 3, "too few conditioning cases");
    std::cout << "\n[minco_freetime_joint_conditioning_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[minco_freetime_joint_conditioning_self_test] FAIL: "
              << ex.what() << "\n";
    return 1;
  }
}
