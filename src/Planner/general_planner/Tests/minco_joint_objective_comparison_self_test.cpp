/**
 * Joint-objective comparison: energy + L2 tracking + vel/acc + corridor.
 *
 * Compares Euclidean, frozen MCE whitening, and frozen MCE + active corridor GN.
 */

#include "traj_opt/minco/minco_metric.hpp"
#include "traj_opt/minco/minco_whitening.hpp"
#include "utils/optimization/lbfgs.h"

#include <Eigen/Dense>

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

std::string sci(double v, int d = 4)
{
  std::ostringstream oss;
  oss << std::scientific << std::setprecision(d) << v;
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

struct Scene
{
  int pieces{5};
  VectorXd times;
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  InnerPoints P0;
  InnerPoints P_guide;

  int inner() const { return std::max(0, pieces - 1); }
  int dim() const { return 3 * inner(); }
};

Scene makeScene(int pieces, const VectorXd &times)
{
  Scene s;
  s.pieces = pieces;
  s.times = times;
  s.head.col(0) << 0.0, 0.0, 1.0;
  s.head.col(1) << 0.6, 0.0, 0.0;
  s.tail.col(0) << static_cast<double>(pieces), 0.0, 1.0;
  s.tail.col(1) << 0.4, 0.0, 0.0;
  s.P0.resize(3, s.inner());
  s.P_guide.resize(3, s.inner());
  for (int i = 0; i < s.inner(); ++i)
  {
    const double u = static_cast<double>(i + 1) / static_cast<double>(pieces);
    s.P0.col(i) << u * s.tail(0, 0),
        0.18 * std::sin(2.0 * M_PI * u),
        1.0 + 0.05 * std::cos(2.0 * M_PI * u);
    s.P_guide.col(i) << u * s.tail(0, 0), 0.0, 1.0;
  }
  return s;
}

Trajectory generate(const Scene &s, const VectorXd &P)
{
  Trajectory traj;
  require(traj.generate(unflatten(P, s.inner()), s.head, s.tail, s.times),
          "MINCO generate failed");
  return traj;
}

Trajectory generateJvp(const Scene &s, const VectorXd &dP)
{
  Trajectory traj;
  BoundaryState z = BoundaryState::Zero();
  require(traj.generate(unflatten(dP, s.inner()), z, z, s.times),
          "MINCO JVP failed");
  return traj;
}

MatrixXd derivativeMass(int deriv, double T)
{
  const int n = Trajectory::COEFF_NUM;
  MatrixXd Q = MatrixXd::Zero(n, n);
  auto falling = [](int a, int r) {
    double out = 1.0;
    for (int i = 0; i < r; ++i)
    {
      out *= static_cast<double>(a - i);
    }
    return out;
  };
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

struct Weights
{
  double energy{1.0};
  double track{6.0};
  double vel{0.25};
  double acc{0.08};
  double corr{25.0};
  double y_max{0.08};
  int samples{8};
};

struct Breakdown
{
  double energy{0.0};
  double track{0.0};
  double vel{0.0};
  double acc{0.0};
  double corr{0.0};
  double total{0.0};
  double max_corr{0.0};
};

struct JointProblem
{
  Scene scene;
  Weights w;
  std::vector<Trajectory> jvp;
  MatrixXd G_mce;
  MatrixXd G_scalar;
};

double derivQuad(const Trajectory &traj, int deriv, double w, VectorXd *g)
{
  double J = 0.0;
  Trajectory::CoeffMat gdC(Trajectory::COEFF_NUM * traj.getPieceNum(), 3);
  gdC.setZero();
  for (int p = 0; p < traj.getPieceNum(); ++p)
  {
    const MatrixXd Q = derivativeMass(deriv, traj.getDurations()(p));
    const auto C = traj.getCoefficients().block<Trajectory::COEFF_NUM, 3>(
        p * Trajectory::COEFF_NUM, 0);
    J += 0.5 * w * (C.transpose() * Q * C).trace();
    if (g != nullptr)
    {
      gdC.block<Trajectory::COEFF_NUM, 3>(p * Trajectory::COEFF_NUM, 0) =
          w * Q * C;
    }
  }
  if (g != nullptr)
  {
    InnerPoints gP;
    VectorXd gT;
    traj.propagateGrad(gdC, VectorXd::Zero(traj.getPieceNum()), gP, gT);
    *g += flatten(gP);
  }
  return J;
}

double l2Track(const Trajectory &traj, const Trajectory &ref, double w,
               VectorXd *g)
{
  double J = 0.0;
  Trajectory::CoeffMat gdC(Trajectory::COEFF_NUM * traj.getPieceNum(), 3);
  gdC.setZero();
  for (int p = 0; p < traj.getPieceNum(); ++p)
  {
    const MatrixXd M = derivativeMass(0, traj.getDurations()(p));
    const auto C = traj.getCoefficients().block<Trajectory::COEFF_NUM, 3>(
        p * Trajectory::COEFF_NUM, 0);
    const auto Cr = ref.getCoefficients().block<Trajectory::COEFF_NUM, 3>(
        p * Trajectory::COEFF_NUM, 0);
    const auto D = C - Cr;
    J += 0.5 * w * (D.transpose() * M * D).trace();
    if (g != nullptr)
    {
      gdC.block<Trajectory::COEFF_NUM, 3>(p * Trajectory::COEFF_NUM, 0) =
          w * M * D;
    }
  }
  if (g != nullptr)
  {
    InnerPoints gP;
    VectorXd gT;
    traj.propagateGrad(gdC, VectorXd::Zero(traj.getPieceNum()), gP, gT);
    *g += flatten(gP);
  }
  return J;
}

double corridor(const JointProblem &pr, const Trajectory &traj, VectorXd *g,
                MatrixXd *gn, double *max_viol)
{
  double J = 0.0;
  const int m = pr.scene.dim();
  if (g != nullptr)
  {
    g->setZero(m);
  }
  if (gn != nullptr)
  {
    *gn = MatrixXd::Zero(m, m);
  }
  if (max_viol != nullptr)
  {
    *max_viol = 0.0;
  }
  for (int p = 0; p < traj.getPieceNum(); ++p)
  {
    const double T = traj.getDurations()(p);
    const double t0 = (p == 0 ? 0.0 : traj.getDurations().head(p).sum());
    for (int k = 0; k <= pr.w.samples; ++k)
    {
      const double tau = static_cast<double>(k) / static_cast<double>(pr.w.samples);
      const double t = t0 + tau * T;
      const double trap =
          ((k == 0 || k == pr.w.samples) ? 0.5 : 1.0) * T /
          static_cast<double>(pr.w.samples);
      const Vector3d pos = traj.getPos(t);
      const double viol = std::abs(pos.y()) - pr.w.y_max;
      if (max_viol != nullptr)
      {
        *max_viol = std::max(*max_viol, viol);
      }
      if (viol <= 0.0)
      {
        continue;
      }
      J += pr.w.corr * 0.5 * trap * viol * viol;
      const double s = pos.y() >= 0.0 ? 1.0 : -1.0;
      VectorXd jrow = VectorXd::Zero(m);
      for (int i = 0; i < m; ++i)
      {
        jrow(i) = pr.jvp[static_cast<std::size_t>(i)].getPos(t).y();
      }
      if (g != nullptr)
      {
        *g += (pr.w.corr * trap * viol * s) * jrow;
      }
      if (gn != nullptr)
      {
        *gn += (pr.w.corr * trap) * jrow * jrow.transpose();
      }
    }
  }
  return J;
}

Breakdown evaluate(const JointProblem &pr, const VectorXd &P, VectorXd *g)
{
  const Trajectory traj = generate(pr.scene, P);
  const Trajectory ref = generate(pr.scene, flatten(pr.scene.P_guide));
  Breakdown b;
  if (g != nullptr)
  {
    g->setZero(P.size());
  }
  if (pr.w.energy > 0.0)
  {
    Trajectory::CoeffMat gdC;
    double energy = 0.0;
    traj.getEnergyPartialGradByCoeffs(energy, gdC);
    b.energy = pr.w.energy * energy;
    if (g != nullptr)
    {
      InnerPoints gP;
      VectorXd gT;
      traj.propagateGrad(gdC, VectorXd::Zero(traj.getPieceNum()), gP, gT);
      *g += pr.w.energy * flatten(gP);
    }
  }
  if (pr.w.track > 0.0)
  {
    b.track = l2Track(traj, ref, pr.w.track, g);
  }
  if (pr.w.vel > 0.0)
  {
    b.vel = derivQuad(traj, 1, pr.w.vel, g);
  }
  if (pr.w.acc > 0.0)
  {
    b.acc = derivQuad(traj, 2, pr.w.acc, g);
  }
  if (pr.w.corr > 0.0)
  {
    VectorXd gc;
    b.corr = corridor(pr, traj, g != nullptr ? &gc : nullptr, nullptr,
                      &b.max_corr);
    if (g != nullptr)
    {
      *g += gc;
    }
  }
  b.total = b.energy + b.track + b.vel + b.acc + b.corr;
  return b;
}

JointProblem makeProblem(const Scene &scene, const Weights &w)
{
  JointProblem pr;
  pr.scene = scene;
  pr.w = w;
  pr.jvp.reserve(static_cast<std::size_t>(scene.dim()));
  for (int i = 0; i < scene.dim(); ++i)
  {
    VectorXd e = VectorXd::Zero(scene.dim());
    e(i) = 1.0;
    pr.jvp.push_back(generateJvp(scene, e));
  }
  const Trajectory traj = generate(scene, flatten(scene.P0));
  minco::MincoMetricOptions options;
  options.mode = minco::MincoMetricMode::kFrozenWaypoint;
  options.regularization = 0.0;
  options.energy_weight = w.energy;
  minco::MincoMetric<3, 4> metric;
  metric.setOptions(options);
  require(metric.update(traj), "G_MCE build failed");
  pr.G_mce = metric.waypointMetric();
  pr.G_scalar = metric.scalarWaypointMetric();
  return pr;
}

double conditionNumber(const MatrixXd &A)
{
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(0.5 * (A + A.transpose()));
  require(es.info() == Eigen::Success, "eigendecomposition failed");
  const double lmin = es.eigenvalues().minCoeff();
  const double lmax = es.eigenvalues().maxCoeff();
  if (lmin <= 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  return lmax / lmin;
}

MatrixXd whiteHessian(const MatrixXd &G, const MatrixXd &H)
{
  Eigen::LLT<MatrixXd> llt(0.5 * (G + G.transpose()));
  require(llt.info() == Eigen::Success, "Cholesky failed");
  const MatrixXd L = llt.matrixL();
  const MatrixXd Y = L.triangularView<Eigen::Lower>().solve(H);
  const MatrixXd W =
      L.triangularView<Eigen::Lower>().solve(Y.transpose()).transpose();
  return 0.5 * (W + W.transpose());
}

MatrixXd fdHessian(const JointProblem &pr, const VectorXd &P)
{
  const int n = static_cast<int>(P.size());
  MatrixXd H = MatrixXd::Zero(n, n);
  const double eta = 1.0e-6;
  for (int j = 0; j < n; ++j)
  {
    const double h = eta * std::max(1.0, std::abs(P(j)));
    VectorXd ej = VectorXd::Zero(n);
    ej(j) = 1.0;
    VectorXd gp, gm;
    evaluate(pr, P + h * ej, &gp);
    evaluate(pr, P - h * ej, &gm);
    H.col(j) = (gp - gm) / (2.0 * h);
  }
  return 0.5 * (H + H.transpose());
}

struct SolverState
{
  const JointProblem *pr{nullptr};
  minco::FrozenMceWhitening *whitening{nullptr};
  int iters{0};
  int evals{0};
};

double evalChart(void *ptr, const VectorXd &x, VectorXd &g)
{
  auto *st = static_cast<SolverState *>(ptr);
  ++st->evals;
  VectorXd P = x;
  if (st->whitening != nullptr && st->whitening->ready())
  {
    require(st->whitening->toChart(x, P), "decode failed");
  }
  VectorXd gP;
  const Breakdown b = evaluate(*st->pr, P, &gP);
  if (st->whitening != nullptr && st->whitening->ready())
  {
    require(st->whitening->transformCovector(gP, g), "g_z failed");
  }
  else
  {
    g = gP;
  }
  return b.total;
}

int countIters(void *ptr, const VectorXd &, const VectorXd &, const double,
               const double, const int, const int)
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
  double t_s{0.0};
  Breakdown start;
  Breakdown end;
  double kappa{0.0};
};

RunResult runSolve(const std::string &name, const JointProblem &pr,
                   const VectorXd &P0, minco::FrozenMceWhitening *whitening,
                   double kappa)
{
  SolverState st;
  st.pr = &pr;
  st.whitening = whitening;
  VectorXd x = P0;
  if (whitening != nullptr && whitening->ready())
  {
    require(whitening->encodeInPlace(x), "encode failed");
  }
  math_utils::lbfgs::lbfgs_parameter_t param;
  param.mem_size = 16;
  param.g_epsilon = 1.0e-7;
  param.past = 3;
  param.delta = 1.0e-12;
  param.max_iterations = 250;
  param.max_linesearch = 64;
  double f = 0.0;
  const auto t0 = Clock::now();
  const int status = math_utils::lbfgs::lbfgs_optimize(
      x, f, &evalChart, nullptr, &countIters, &st, param);
  VectorXd P_final = x;
  if (whitening != nullptr && whitening->ready())
  {
    require(whitening->toChart(x, P_final), "final decode failed");
  }
  RunResult r;
  r.name = name;
  r.status = status;
  r.iters = st.iters;
  r.evals = st.evals;
  r.t_s = std::chrono::duration<double>(Clock::now() - t0).count();
  r.start = evaluate(pr, P0, nullptr);
  r.end = evaluate(pr, P_final, nullptr);
  r.kappa = kappa;
  return r;
}

void printBreakdown(const char *tag, const Breakdown &b)
{
  std::cout << "    " << tag
            << "  J=" << sci(b.total, 4)
            << "  E=" << sci(b.energy, 3)
            << "  track=" << sci(b.track, 3)
            << "  vel=" << sci(b.vel, 3)
            << "  acc=" << sci(b.acc, 3)
            << "  corr=" << sci(b.corr, 3)
            << "  viol=" << sci(b.max_corr, 3) << "\n";
}

void printResult(const RunResult &r, const RunResult *baseline)
{
  const double speed =
      (baseline != nullptr && r.iters > 0)
          ? static_cast<double>(baseline->iters) / static_cast<double>(r.iters)
          : 0.0;
  std::cout << "  " << std::left << std::setw(18) << r.name
            << "  J=" << sci(r.end.total, 4)
            << "  iters=" << std::setw(4) << r.iters
            << "  evals=" << std::setw(4) << r.evals
            << "  viol=" << sci(r.end.max_corr, 3)
            << "  kappa=" << sci(r.kappa, 3)
            << "  t=" << sci(r.t_s, 3) << "s"
            << "  status=" << r.status;
  if (baseline != nullptr)
  {
    std::cout << "  x_iter=" << sci(speed, 2);
  }
  std::cout << "\n";
}

minco::FrozenMceWhitening makeWhitening(const VectorXd &P0, const MatrixXd &G)
{
  minco::FrozenMceWhitening w;
  MatrixXd Gs = 0.5 * (G + G.transpose());
  Eigen::SelfAdjointEigenSolver<MatrixXd> es(Gs);
  require(es.info() == Eigen::Success, "metric spectrum failed");
  const double lmin = es.eigenvalues().minCoeff();
  const double lmax = es.eigenvalues().maxCoeff();
  if (lmin <= 1.0e-12 * std::max(1.0, lmax))
  {
    Gs += (1.0e-10 * std::max(1.0, lmax) - std::min(0.0, lmin)) *
          MatrixXd::Identity(Gs.rows(), Gs.cols());
  }
  require(w.configure(0, P0, Gs), "whitening configure failed");
  return w;
}

void runCase(const std::string &title, const Scene &scene, const Weights &w)
{
  std::cout << "\n== " << title << " ==\n";
  const JointProblem pr = makeProblem(scene, w);
  const VectorXd P0 = flatten(scene.P0);
  const MatrixXd H = fdHessian(pr, P0);
  MatrixXd Ggn;
  corridor(pr, generate(scene, P0), nullptr, &Ggn, nullptr);
  const MatrixXd Gmce_gn = pr.G_mce + Ggn;
  const double kI = conditionNumber(H);
  const double kM = conditionNumber(whiteHessian(pr.G_mce, H));
  const double kG = conditionNumber(whiteHessian(Gmce_gn, H));
  std::cout << "  n=" << P0.size()
            << "  kappa_I=" << sci(kI)
            << "  kappa_MCE=" << sci(kM)
            << "  kappa_MCE+GN=" << sci(kG)
            << "  tr(Gc)/tr(GMCE)="
            << sci(Ggn.trace() / std::max(1.0, pr.G_mce.trace())) << "\n";

  minco::FrozenMceWhitening w_mce;
  require(w_mce.configureKronecker(0, 3, P0, pr.G_scalar),
          "MCE Kronecker whitening failed");
  minco::FrozenMceWhitening w_gn = makeWhitening(P0, Gmce_gn);

  const RunResult euclidean = runSolve("A Euclidean", pr, P0, nullptr, kI);
  const RunResult mce = runSolve("B Frozen MCE", pr, P0, &w_mce, kM);
  const RunResult gn = runSolve("C Frozen MCE+GN", pr, P0, &w_gn, kG);

  printBreakdown("start", euclidean.start);
  printResult(euclidean, nullptr);
  printResult(mce, &euclidean);
  printResult(gn, &euclidean);
  printBreakdown("end-I  ", euclidean.end);
  printBreakdown("end-MCE", mce.end);
  printBreakdown("end-GN ", gn.end);

  require(std::isfinite(euclidean.end.total) && std::isfinite(mce.end.total) &&
              std::isfinite(gn.end.total),
          "non-finite joint cost");
  require(mce.end.total <= 1.05 * euclidean.end.total + 1.0e-8,
          "frozen MCE ended worse than Euclidean");
  require(gn.end.total <= 1.05 * euclidean.end.total + 1.0e-8,
          "frozen MCE+GN ended worse than Euclidean");
}

} // namespace

int main()
{
  try
  {
    std::cout << std::setprecision(6);
    std::cout << "Joint objective: energy + L2 track + vel/acc + corridor\n";
    std::cout << "A Euclidean vs B Frozen MCE vs C Frozen MCE+GN\n";

    Weights w;
    {
      VectorXd T = VectorXd::Ones(5);
      runCase("Planner-like M=5 uniform", makeScene(5, T), w);
    }
    {
      VectorXd T = VectorXd::Ones(10);
      runCase("Planner-like M=10 uniform", makeScene(10, T), w);
    }
    {
      VectorXd T(5);
      T << 0.3, 0.6, 1.2, 1.8, 1.1;
      runCase("Planner-like M=5 strong time ratio", makeScene(5, T), w);
    }
    {
      Weights tight = w;
      tight.corr = 200.0;
      tight.y_max = 0.05;
      VectorXd T = VectorXd::Ones(5);
      runCase("Narrow corridor M=5 (GN should matter)", makeScene(5, T), tight);
    }

    std::cout << "\n[minco_joint_objective_comparison_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[minco_joint_objective_comparison_self_test] FAIL: " << ex.what()
              << '\n';
    return 1;
  }
}
