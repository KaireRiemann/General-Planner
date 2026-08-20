#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <cmath>

namespace minco
{

/**
 * Pull a physical (T, P) metric back to solver coordinates (τ, ξ)
 *
 *   T = φ(τ),   P = ψ(ξ),
 *   G_x = J_{θ←x}^T G_θ J_{θ←x}.
 *
 * Identity spatial maps pass J_ψ = I.  dT_dtau(i) = ∂T_i/∂τ_i.
 */
inline bool pullbackTimeMap(const Eigen::MatrixXd &G_theta,
                            const Eigen::VectorXd &dT_dtau,
                            Eigen::MatrixXd &G_x)
{
  const int m = static_cast<int>(dT_dtau.size());
  if (m < 0 || G_theta.rows() != G_theta.cols() || G_theta.rows() < m ||
      !G_theta.allFinite() || !dT_dtau.allFinite())
  {
    return false;
  }
  const int n = static_cast<int>(G_theta.rows());
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(n, n);
  for (int i = 0; i < m; ++i)
  {
    J(i, i) = dT_dtau(i);
  }
  G_x = J.transpose() * G_theta * J;
  return G_x.allFinite();
}

/**
 * Pull physical (T, P) metric to solver (τ, ξ):
 *
 *   J = blkdiag(diag(dT/dτ), J_ψ),   P = ψ(ξ).
 *
 * Identity spatial charts pass J_ψ = I.
 */
inline bool pullbackSolverChart(const Eigen::MatrixXd &G_theta,
                                const Eigen::VectorXd &dT_dtau,
                                const Eigen::MatrixXd &J_psi,
                                Eigen::MatrixXd &G_x)
{
  const int m = static_cast<int>(dT_dtau.size());
  if (m < 0 || G_theta.rows() != G_theta.cols() || G_theta.rows() < m ||
      J_psi.rows() != G_theta.rows() - m || !G_theta.allFinite() ||
      !dT_dtau.allFinite() || !J_psi.allFinite())
  {
    return false;
  }
  const int pdim = static_cast<int>(G_theta.rows()) - m;
  const int sdim = static_cast<int>(J_psi.cols());
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(m + pdim, m + sdim);
  for (int i = 0; i < m; ++i)
  {
    J(i, i) = dT_dtau(i);
  }
  if (pdim > 0 && sdim > 0)
  {
    J.block(m, m, pdim, sdim) = J_psi;
  }
  G_x = J.transpose() * G_theta * J;
  return G_x.allFinite();
}

/**
 * Frozen full-joint whitening on solver coordinates x = (τ, ξ):
 *
 *   G_0 = L L^T,
 *   z   = L^T (x - x_seed),
 *   x   = x_seed + L^{-T} z,
 *   g_z = L^{-1} g_x.
 *
 * Dense Cholesky is the reference.  Block-Schur uses
 *
 *   z_T = L_T^T dτ,
 *   z_P = L_P^T (dξ + Y dτ),   Y = C^{-1} B^T,
 *
 * which is the missing time-space decorrelation term.
 */
class FrozenJointWhitening
{
public:
  void clear()
  {
    ready_ = false;
    block_schur_ = false;
    time_dim_ = 0;
    spatial_dim_ = 0;
    x_seed_.resize(0);
    L_.resize(0, 0);
    L_T_.resize(0, 0);
    L_P_.resize(0, 0);
    Y_.resize(0, 0);
  }

  bool ready() const { return ready_; }
  bool usesBlockSchur() const { return block_schur_; }
  int timeDim() const { return time_dim_; }
  int spatialDim() const { return spatial_dim_; }
  int dim() const { return time_dim_ + spatial_dim_; }
  const Eigen::VectorXd &seed() const { return x_seed_; }
  const Eigen::MatrixXd &matrixL() const { return L_; }
  const Eigen::MatrixXd &schurY() const { return Y_; }

  bool configure(const Eigen::VectorXd &x_seed, const Eigen::MatrixXd &G)
  {
    return configureDense(x_seed, G);
  }

  bool configureDense(const Eigen::VectorXd &x_seed, const Eigen::MatrixXd &G)
  {
    clear();
    if (x_seed.size() == 0 || G.rows() != x_seed.size() ||
        G.cols() != x_seed.size() || !x_seed.allFinite() || !G.allFinite())
    {
      return false;
    }
    const Eigen::MatrixXd symmetric = 0.5 * (G + G.transpose());
    Eigen::LLT<Eigen::MatrixXd> llt(symmetric);
    if (llt.info() != Eigen::Success)
    {
      return false;
    }
    time_dim_ = 0;
    spatial_dim_ = static_cast<int>(x_seed.size());
    x_seed_ = x_seed;
    L_ = llt.matrixL();
    block_schur_ = false;
    ready_ = L_.allFinite();
    return ready_;
  }

  bool configureBlockSchur(int time_dim,
                           const Eigen::VectorXd &x_seed,
                           const Eigen::MatrixXd &G)
  {
    clear();
    const int n = static_cast<int>(x_seed.size());
    if (time_dim < 0 || time_dim > n || n == 0 || G.rows() != n ||
        G.cols() != n || !x_seed.allFinite() || !G.allFinite())
    {
      return false;
    }
    const int spatial = n - time_dim;
    const Eigen::MatrixXd symmetric = 0.5 * (G + G.transpose());
    if (time_dim == 0 || spatial == 0)
    {
      return configureDense(x_seed, symmetric);
    }

    const Eigen::MatrixXd A = symmetric.topLeftCorner(time_dim, time_dim);
    const Eigen::MatrixXd B = symmetric.topRightCorner(time_dim, spatial);
    const Eigen::MatrixXd C = symmetric.bottomRightCorner(spatial, spatial);

    Eigen::LLT<Eigen::MatrixXd> llt_p(C);
    if (llt_p.info() != Eigen::Success)
    {
      return false;
    }
    L_P_ = llt_p.matrixL();
    Y_ = C.ldlt().solve(B.transpose());
    if (!Y_.allFinite())
    {
      return false;
    }
    const Eigen::MatrixXd S = A - B * Y_;
    Eigen::LLT<Eigen::MatrixXd> llt_t(0.5 * (S + S.transpose()));
    if (llt_t.info() != Eigen::Success)
    {
      return false;
    }
    L_T_ = llt_t.matrixL();

    time_dim_ = time_dim;
    spatial_dim_ = spatial;
    x_seed_ = x_seed;
    block_schur_ = true;
    ready_ = L_T_.allFinite() && L_P_.allFinite() && Y_.allFinite();
    return ready_;
  }

  bool toWhitened(const Eigen::VectorXd &x, Eigen::VectorXd &z) const
  {
    if (!ready_ || x.size() != x_seed_.size())
    {
      return false;
    }
    const Eigen::VectorXd dx = x - x_seed_;
    if (block_schur_)
    {
      const Eigen::VectorXd dtau = dx.head(time_dim_);
      const Eigen::VectorXd dxi = dx.tail(spatial_dim_);
      z.resize(dx.size());
      z.head(time_dim_).noalias() = L_T_.transpose() * dtau;
      z.tail(spatial_dim_).noalias() = L_P_.transpose() * (dxi + Y_ * dtau);
      return z.allFinite();
    }
    z.noalias() = L_.transpose() * dx;
    return z.allFinite();
  }

  bool toChart(const Eigen::VectorXd &z, Eigen::VectorXd &x) const
  {
    Eigen::VectorXd dx;
    if (!transformDirectionToChart(z, dx))
    {
      return false;
    }
    x = x_seed_ + dx;
    return x.allFinite();
  }

  bool transformCovector(const Eigen::VectorXd &gx, Eigen::VectorXd &gz) const
  {
    if (!ready_ || gx.size() != x_seed_.size())
    {
      return false;
    }
    if (block_schur_)
    {
      const Eigen::VectorXd g_tau = gx.head(time_dim_);
      const Eigen::VectorXd g_xi = gx.tail(spatial_dim_);
      gz.resize(gx.size());
      gz.head(time_dim_) =
          L_T_.triangularView<Eigen::Lower>().solve(g_tau - Y_.transpose() * g_xi);
      gz.tail(spatial_dim_) =
          L_P_.triangularView<Eigen::Lower>().solve(g_xi);
      return gz.allFinite();
    }
    gz = L_.triangularView<Eigen::Lower>().solve(gx);
    return gz.allFinite();
  }

  bool transformDirectionToChart(const Eigen::VectorXd &dz,
                                 Eigen::VectorXd &dx) const
  {
    if (!ready_ || dz.size() != x_seed_.size())
    {
      return false;
    }
    if (block_schur_)
    {
      const Eigen::VectorXd dtau =
          L_T_.transpose().triangularView<Eigen::Upper>().solve(dz.head(time_dim_));
      const Eigen::VectorXd dxi_plus =
          L_P_.transpose().triangularView<Eigen::Upper>().solve(dz.tail(spatial_dim_));
      dx.resize(dz.size());
      dx.head(time_dim_) = dtau;
      dx.tail(spatial_dim_) = dxi_plus - Y_ * dtau;
      return dx.allFinite();
    }
    dx = L_.transpose().triangularView<Eigen::Upper>().solve(dz);
    return dx.allFinite();
  }

  bool encodeInPlace(Eigen::VectorXd &x) const
  {
    if (!ready_ || x.size() < dim())
    {
      return false;
    }
    const Eigen::VectorXd core = x.head(dim());
    Eigen::VectorXd z;
    if (!toWhitened(core, z))
    {
      return false;
    }
    x.head(dim()) = z;
    return true;
  }

  bool decodeInPlace(Eigen::VectorXd &z) const
  {
    if (!ready_ || z.size() < dim())
    {
      return false;
    }
    const Eigen::VectorXd core = z.head(dim());
    Eigen::VectorXd x;
    if (!toChart(core, x))
    {
      return false;
    }
    z.head(dim()) = x;
    return true;
  }

  bool transformCovectorInPlace(Eigen::Ref<Eigen::VectorXd> grad) const
  {
    if (!ready_ || grad.size() < dim())
    {
      return false;
    }
    const Eigen::VectorXd gx = grad.head(dim());
    Eigen::VectorXd gz;
    if (!transformCovector(gx, gz))
    {
      return false;
    }
    grad.head(dim()) = gz;
    return true;
  }

private:
  bool ready_{false};
  bool block_schur_{false};
  int time_dim_{0};
  int spatial_dim_{0};
  Eigen::VectorXd x_seed_;
  Eigen::MatrixXd L_;
  Eigen::MatrixXd L_T_;
  Eigen::MatrixXd L_P_;
  Eigen::MatrixXd Y_;
};

} // namespace minco
