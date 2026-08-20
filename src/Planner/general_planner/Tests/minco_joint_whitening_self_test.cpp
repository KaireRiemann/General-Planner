/**
 * Level B: TimeMap pullback and frozen joint / block-Schur whitening.
 *
 * B1 dx^T G_x dx = dθ^T G_θ dθ
 * B2 identity spatial chart (J_ψ = I)
 * B3 joint whitening quadratic form
 * B4 covector pairing g_x^T dx = g_z^T dz
 * B5 direction inverse dx = L^{-T} dz
 * B6 dense Cholesky vs block-Schur
 */

#include "traj_opt/costfunctional/temporalmap/quad_inv_time_map.hpp"
#include "traj_opt/minco/minco_joint_whitening.hpp"
#include "traj_opt/minco/minco_metric.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

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

struct Seed
{
  Trajectory traj;
  InnerPoints points;
};

Seed makeSeed()
{
  Seed seed;
  VectorXd times(4);
  times << 0.7, 1.4, 0.55, 1.9;
  seed.points.resize(3, 3);
  seed.points << 0.9, 2.0, 3.3, 0.08, -0.15, 0.2, 0.4, 0.52, 0.61;
  BoundaryState head = BoundaryState::Zero();
  BoundaryState tail = BoundaryState::Zero();
  head.col(0) << 0.0, -0.2, 0.1;
  head.col(1) << 0.45, 0.1, 0.0;
  tail.col(0) << 4.2, 1.1, 0.8;
  tail.col(1) << 0.1, -0.15, 0.0;
  require(seed.traj.generate(seed.points, head, tail, times), "generate failed");
  return seed;
}

} // namespace

int main()
{
  try
  {
    std::cout << "[Level B] TimeMap pullback / frozen joint whitening\n";
    const Seed seed = makeSeed();
    const Trajectory &traj = seed.traj;
    const int m = traj.getPieceNum();
    const int pdim = 3 * (m - 1);
    const int n = m + pdim;

    minco::MincoMetricOptions opt;
    opt.mode = minco::MincoMetricMode::kFullSpaceTimeGaussNewton;
    opt.regularization = 1.0e-10;
    opt.time_metric_weight = 1.0;
    opt.energy_weight = 1.0;
    minco::MincoMetric<3, 4> metric;
    metric.setOptions(opt);
    require(metric.update(traj), "full metric failed");
    const MatrixXd G_theta = metric.spaceTimeMetric();
    require(G_theta.rows() == n, "unexpected physical metric size");

    temporal_map::QuadInvTimeMap tmap;
    VectorXd dT_dtau(m);
    VectorXd tau(m);
    const auto &T = traj.getDurations();
    for (int i = 0; i < m; ++i)
    {
      tau(i) = tmap.toTau(T(i));
      dT_dtau(i) = tmap.backward(tau(i), T(i), 1.0);
    }

    MatrixXd G_x;
    require(minco::pullbackTimeMap(G_theta, dT_dtau, G_x), "pullback failed");

    VectorXd dtheta = VectorXd::LinSpaced(n, -0.4, 0.7);
    VectorXd dx = dtheta;
    for (int i = 0; i < m; ++i)
    {
      dx(i) = dtheta(i) / dT_dtau(i);
    }
    const double q_theta = dtheta.dot(G_theta * dtheta);
    const double q_x = dx.dot(G_x * dx);
    require(std::abs(q_theta - q_x) < 1.0e-10 * std::max(1.0, std::abs(q_theta)),
            "B1 pullback quadratic form mismatch");
    std::cout << "  B1 |q_θ - q_x| / |q| = "
              << std::abs(q_theta - q_x) / std::max(1.0, std::abs(q_theta))
              << "\n";

    require((G_x.bottomRightCorner(pdim, pdim) - G_theta.bottomRightCorner(pdim, pdim))
                    .norm() < 1.0e-12,
            "B2 identity spatial map must leave G_PP unchanged");
    std::cout << "  B2 identity spatial  G_PP unchanged\n";

    VectorXd x_seed(n);
    x_seed.head(m) = tau;
    x_seed.tail(pdim) = Eigen::Map<const VectorXd>(seed.points.data(), pdim);

    minco::FrozenJointWhitening dense;
    require(dense.configureDense(x_seed, G_x), "dense whitening failed");
    minco::FrozenJointWhitening schur;
    require(schur.configureBlockSchur(m, x_seed, G_x), "block-Schur failed");

    VectorXd z_dense;
    require(dense.toWhitened(x_seed + dx, z_dense), "dense encode failed");
    const double q_z = z_dense.squaredNorm();
    require(std::abs(q_z - q_x) < 1.0e-10 * std::max(1.0, std::abs(q_x)),
            "B3 dx^T G dx != dz^T dz");
    std::cout << "  B3 |q_x - ||z||^2| / |q| = "
              << std::abs(q_z - q_x) / std::max(1.0, std::abs(q_x)) << "\n";

    VectorXd gx = VectorXd::LinSpaced(n, 0.3, -0.8);
    VectorXd gz;
    require(dense.transformCovector(gx, gz), "covector failed");
    VectorXd recovered_dx;
    require(dense.transformDirectionToChart(z_dense, recovered_dx),
            "direction inverse failed");
    require((recovered_dx - dx).norm() < 1.0e-10,
            "B5 direction inverse failed");
    const double pair_x = gx.dot(dx);
    const double pair_z = gz.dot(z_dense);
    require(std::abs(pair_x - pair_z) < 1.0e-10 * std::max(1.0, std::abs(pair_x)),
            "B4 covector pairing failed");
    std::cout << "  B4 |g_x dx - g_z dz| / |g_x dx| = "
              << std::abs(pair_x - pair_z) / std::max(1.0, std::abs(pair_x))
              << "\n";
    std::cout << "  B5 ||L^{-T} z - dx|| = " << (recovered_dx - dx).norm()
              << "\n";

    VectorXd z_schur;
    require(schur.toWhitened(x_seed + dx, z_schur), "schur encode failed");
    VectorXd x_from_schur;
    require(schur.toChart(z_schur, x_from_schur), "schur decode failed");
    require((x_from_schur - (x_seed + dx)).norm() < 1.0e-10,
            "B6 schur decode mismatch");
    require(std::abs(z_schur.squaredNorm() - q_x) <
                1.0e-9 * std::max(1.0, std::abs(q_x)),
            "B6 schur quadratic form mismatch");
    VectorXd gz_s;
    require(schur.transformCovector(gx, gz_s), "schur covector failed");
    require(std::abs(gz_s.dot(z_schur) - pair_x) <
                1.0e-9 * std::max(1.0, std::abs(pair_x)),
            "B6 schur pairing mismatch");
    std::cout << "  B6 dense vs Schur  ||z||^2 match  Δ="
              << std::abs(z_schur.squaredNorm() - z_dense.squaredNorm())
              << "  Y_norm=" << schur.schurY().norm() << "\n";
    require(schur.schurY().norm() > 1.0e-8,
            "block-Schur Y vanished; no T-P coupling");

    std::cout << "[minco_joint_whitening_self_test] OK\n";
    return 0;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[minco_joint_whitening_self_test] FAIL: " << ex.what() << "\n";
    return 1;
  }
}
