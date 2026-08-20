/**
 * Production-path identity for the chart wired into MINCOOptimizer /
 * ExpTrajOpt (energy + LinearTimeCost + QuadInvTimeMap + FrozenJointWhitening).
 *
 * G1 encode/decode round-trip on x=(τ,ξ)
 * G2 J(z)=J(x) and g_x^T dx = g_z^T dz
 * G3 step-bound uses physical τ, not mixed z
 * G4 refresh at a drifted chart: z'=0 and J is invariant
 */

#include "traj_opt/costfunctional/temporalcosts/linear_time_cost.hpp"
#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/minco/minco_joint_whitening.hpp"
#include "traj_opt/minco/minco_metric.hpp"
#include "traj_opt/minco/minco_trajectory.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using Trajectory = minco::MINCO_S4<3>;
using InnerPoints = Trajectory::InnerPointsMat;
using BoundaryState = Trajectory::BoundaryState;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void require(bool ok, const std::string &message)
{
  if (!ok)
  {
    throw std::runtime_error(message);
  }
}

struct Problem
{
  int pieces{4};
  VectorXd times;
  InnerPoints inner;
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  int spatial() const { return 3 * std::max(0, pieces - 1); }
};

Problem makeProblem()
{
  Problem p;
  p.times.resize(4);
  p.times << 0.55, 1.80, 0.40, 2.20;
  p.inner.resize(3, 3);
  p.inner << 1.1, 2.3, 3.6, 0.12, -0.08, 0.18, 1.48, 1.52, 1.47;
  p.head.col(0) << 0.0, 0.0, 1.5;
  p.head.col(1) << 2.4, 0.0, 0.0;
  p.tail.col(0) << 4.8, 0.02, 1.50;
  return p;
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

VectorXd pack(const VectorXd &tau, const VectorXd &P)
{
  VectorXd x(tau.size() + P.size());
  x << tau, P;
  return x;
}

void unpack(const VectorXd &x, int m, VectorXd &tau, VectorXd &P)
{
  tau = x.head(m);
  P = x.tail(x.size() - m);
}

struct Eval
{
  double cost{0.0};
  VectorXd g;
};

Eval evaluateChart(const Problem &prob, const VectorXd &x,
                   temporal_map::QuadInvTimeMap &tmap,
                   const cost_functional::LinearTimeCost &time_cost)
{
  const int m = prob.pieces;
  VectorXd tau, P;
  unpack(x, m, tau, P);
  VectorXd T(m);
  std::vector<double> Tvec(static_cast<std::size_t>(m));
  for (int i = 0; i < m; ++i)
  {
    T(i) = tmap.toTime(tau(i));
    Tvec[static_cast<std::size_t>(i)] = T(i);
  }
  Trajectory traj;
  require(traj.generate(unflatten(P, m - 1), prob.head, prob.tail, T),
          "generate failed");
  double energy = 0.0;
  Trajectory::CoeffMat gdC;
  VectorXd gdT;
  traj.getEnergyPartialGradByCoeffs(energy, gdC);
  traj.getEnergyPartialGradByTimes(gdT);
  VectorXd gdT_time = VectorXd::Zero(m);
  const double tcost = time_cost(Tvec, gdT_time);
  gdT += gdT_time;
  InnerPoints gP;
  VectorXd gT_prop;
  traj.propagateGrad(gdC, gdT, gP, gT_prop);
  Eval out;
  out.cost = energy + tcost;
  out.g = VectorXd::Zero(x.size());
  for (int i = 0; i < m; ++i)
  {
    out.g(i) = tmap.backward(tau(i), T(i), gT_prop(i));
  }
  out.g.tail(prob.spatial()) = flatten(gP);
  return out;
}

bool configureJoint(const Problem &prob, const VectorXd &chart,
                    temporal_map::QuadInvTimeMap &tmap,
                    minco::FrozenJointWhitening &joint)
{
  joint.clear();
  const int m = prob.pieces;
  VectorXd tau, P;
  unpack(chart, m, tau, P);
  VectorXd T(m);
  for (int i = 0; i < m; ++i)
  {
    T(i) = tmap.toTime(tau(i));
  }
  Trajectory seed;
  if (!seed.generate(unflatten(P, m - 1), prob.head, prob.tail, T))
  {
    return false;
  }
  minco::MincoMetricOptions opt;
  opt.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
  opt.regularization = 1.0e-10;
  opt.time_metric_weight = 1.0;
  opt.energy_weight = 1.0;
  minco::MincoMetric<3, 4> metric;
  metric.setOptions(opt);
  if (!metric.update(seed) || !metric.isSpaceTimeMetric())
  {
    return false;
  }
  VectorXd dT(m);
  for (int i = 0; i < m; ++i)
  {
    dT(i) = tmap.backward(tau(i), T(i), 1.0);
  }
  const MatrixXd J_psi = MatrixXd::Identity(prob.spatial(), prob.spatial());
  MatrixXd Gx;
  if (!minco::pullbackSolverChart(metric.spaceTimeMetric(), dT, J_psi, Gx))
  {
    return false;
  }
  Gx.diagonal().array() +=
      1.0e-8 * std::max(1.0, std::abs(Gx.trace())) /
      static_cast<double>(std::max(1, static_cast<int>(Gx.rows())));
  return joint.configureBlockSchur(m, chart, Gx) ||
         joint.configureDense(chart, Gx);
}

double physicalTauStepBound(const minco::FrozenJointWhitening &joint,
                            temporal_map::QuadInvTimeMap &tmap,
                            const VectorXd &z, const VectorXd &dz)
{
  VectorXd chart;
  VectorXd dchart;
  if (!joint.toChart(z, chart) || !joint.transformDirectionToChart(dz, dchart))
  {
    return 1.0e20;
  }
  double bound = 1.0e20;
  for (int i = 0; i < joint.timeDim(); ++i)
  {
    const double d = dchart(i);
    if (!std::isfinite(d) || std::abs(d) <= 1.0e-16)
    {
      continue;
    }
    const double duration = tmap.toTime(chart(i));
    if (d > 0.0)
    {
      const double upper = tmap.toTau(std::min(20.0, 8.0 * duration));
      if (upper > chart(i))
      {
        bound = std::min(bound, (upper - chart(i)) / d);
      }
    }
    else
    {
      const double lower = tmap.toTau(std::max(1.0e-2, 0.05 * duration));
      if (lower < chart(i))
      {
        bound = std::min(bound, (lower - chart(i)) / d);
      }
    }
  }
  return bound;
}

} // namespace

int main()
{
  try
  {
    const Problem prob = makeProblem();
    temporal_map::QuadInvTimeMap tmap;
    cost_functional::LinearTimeCost time_cost;
    time_cost.weight = 20.0;

    VectorXd tau(prob.pieces);
    for (int i = 0; i < prob.pieces; ++i)
    {
      tau(i) = tmap.toTau(prob.times(i));
    }
    const VectorXd x = pack(tau, flatten(prob.inner));
    const Eval ev_x = evaluateChart(prob, x, tmap, time_cost);
    require(std::isfinite(ev_x.cost) && ev_x.g.allFinite(), "chart evaluate");

    minco::FrozenJointWhitening joint;
    require(configureJoint(prob, x, tmap, joint), "joint configure");
    require(joint.ready() && joint.dim() == x.size(), "joint dim");

    VectorXd z = x;
    require(joint.encodeInPlace(z), "encode");
    VectorXd round = z;
    require(joint.decodeInPlace(round), "decode");
    const double round_err = (round - x).norm() / std::max(1.0, x.norm());
    require(round_err < 1.0e-10, "G1 encode/decode round-trip");

    VectorXd chart_from_z;
    require(joint.toChart(z, chart_from_z), "toChart");
    const Eval ev_z_chart = evaluateChart(prob, chart_from_z, tmap, time_cost);
    VectorXd g_z;
    require(joint.transformCovector(ev_z_chart.g, g_z), "covector");
    require(std::abs(ev_z_chart.cost - ev_x.cost) /
                    std::max(1.0, std::abs(ev_x.cost)) <
                1.0e-12,
            "G2 cost identity");

    VectorXd dz = VectorXd::LinSpaced(z.size(), -0.07, 0.11);
    VectorXd dx;
    require(joint.transformDirectionToChart(dz, dx), "direction");
    const double pair_x = ev_x.g.dot(dx);
    const double pair_z = g_z.dot(dz);
    require(std::abs(pair_x - pair_z) /
                    std::max(1.0, std::max(std::abs(pair_x), std::abs(pair_z))) <
                1.0e-9,
            "G2 pairing g_x^T dx = g_z^T dz");

    const double bound_physical = physicalTauStepBound(joint, tmap, z, dz);
    require(std::isfinite(bound_physical) && bound_physical > 0.0,
            "G3 physical step bound");
    double bound_naive = 1.0e20;
    for (int i = 0; i < joint.timeDim(); ++i)
    {
      const double d = dz(i);
      if (std::abs(d) <= 1.0e-16)
      {
        continue;
      }
      const double duration = tmap.toTime(z(i));
      if (d > 0.0)
      {
        const double upper = tmap.toTau(std::min(20.0, 8.0 * duration));
        if (upper > z(i))
        {
          bound_naive = std::min(bound_naive, (upper - z(i)) / d);
        }
      }
    }
    require(std::abs(bound_physical - bound_naive) >
                1.0e-8 * std::max(1.0, bound_physical),
            "G3 mixed z must not be used as τ");

    VectorXd drifted = x;
    for (int i = 0; i < prob.pieces; ++i)
    {
      drifted(i) += 0.35 * (i % 2 == 0 ? 1.0 : -0.6);
    }
    const Eval ev_mid = evaluateChart(prob, drifted, tmap, time_cost);
    minco::FrozenJointWhitening refreshed;
    require(configureJoint(prob, drifted, tmap, refreshed), "refresh configure");
    VectorXd z_refresh = drifted;
    require(refreshed.encodeInPlace(z_refresh), "refresh encode");
    require(z_refresh.norm() < 1.0e-12, "G4 refresh seed is z=0");
    VectorXd chart_r;
    require(refreshed.toChart(z_refresh, chart_r), "refresh toChart");
    const Eval ev_r = evaluateChart(prob, chart_r, tmap, time_cost);
    require(std::abs(ev_r.cost - ev_mid.cost) /
                    std::max(1.0, std::abs(ev_mid.cost)) <
                1.0e-12,
            "G4 refresh cost invariant");

    std::cout << "[minco_production_joint_path_self_test] OK\n";
    std::cout << "  dim=" << x.size() << "  J=" << ev_x.cost
              << "  round=" << round_err << "  pair_err="
              << std::abs(pair_x - pair_z) /
                     std::max(1.0, std::max(std::abs(pair_x), std::abs(pair_z)))
              << "  schur=" << (joint.usesBlockSchur() ? 1 : 0)
              << "  |Y|=" << joint.schurY().norm()
              << "  step_bound=" << bound_physical << "\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[minco_production_joint_path_self_test] FAIL: " << ex.what()
              << "\n";
    return 1;
  }
}
