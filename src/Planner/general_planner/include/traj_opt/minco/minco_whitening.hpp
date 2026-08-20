#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Dense>

namespace minco
{

/**
 * Frozen MCE whitening chart from the production strategy:
 *
 *   G_0 = L L^T,
 *   z = L^T (y - y_seed),
 *   y = y_seed + L^{-T} z,
 *   g_z = L^{-1} g_y.
 *
 * L-BFGS then runs Euclidean algebra in z.  Time coordinates are left
 * untouched.  When G_0 = G_s ⊗ I_DIM the Cholesky factor is stored only on
 * the scalar waypoint block.
 */
class FrozenMceWhitening
{
public:
  void clear()
  {
    ready_ = false;
    kronecker_ = false;
    time_dim_ = 0;
    spatial_dim_ = 0;
    axis_dim_ = 3;
    inner_points_ = 0;
    y_seed_.resize(0);
    L_.resize(0, 0);
    L_scalar_.resize(0, 0);
  }

  bool ready() const { return ready_; }
  bool usesKronecker() const { return kronecker_; }
  int timeDim() const { return time_dim_; }
  int spatialDim() const { return spatial_dim_; }
  const Eigen::VectorXd &seed() const { return y_seed_; }

  bool configure(int time_dim,
                 const Eigen::VectorXd &y_seed,
                 const Eigen::MatrixXd &metric)
  {
    clear();
    if (time_dim < 0 || y_seed.size() == 0 ||
        metric.rows() != y_seed.size() || metric.cols() != y_seed.size() ||
        !y_seed.allFinite() || !metric.allFinite())
    {
      return false;
    }

    const Eigen::MatrixXd symmetric = 0.5 * (metric + metric.transpose());
    Eigen::LLT<Eigen::MatrixXd> llt(symmetric);
    if (llt.info() != Eigen::Success)
    {
      return false;
    }

    time_dim_ = time_dim;
    spatial_dim_ = static_cast<int>(y_seed.size());
    y_seed_ = y_seed;
    L_ = llt.matrixL();
    kronecker_ = false;
    ready_ = L_.allFinite();
    return ready_;
  }

  bool configureKronecker(int time_dim,
                          int axis_dim,
                          const Eigen::VectorXd &y_seed,
                          const Eigen::MatrixXd &scalar_metric)
  {
    clear();
    if (time_dim < 0 || axis_dim <= 0 || y_seed.size() == 0 ||
        y_seed.size() % axis_dim != 0 ||
        scalar_metric.rows() != y_seed.size() / axis_dim ||
        scalar_metric.cols() != scalar_metric.rows() ||
        !y_seed.allFinite() || !scalar_metric.allFinite())
    {
      return false;
    }

    const Eigen::MatrixXd symmetric =
        0.5 * (scalar_metric + scalar_metric.transpose());
    Eigen::LLT<Eigen::MatrixXd> llt(symmetric);
    if (llt.info() != Eigen::Success)
    {
      return false;
    }

    time_dim_ = time_dim;
    spatial_dim_ = static_cast<int>(y_seed.size());
    axis_dim_ = axis_dim;
    inner_points_ = spatial_dim_ / axis_dim_;
    y_seed_ = y_seed;
    L_scalar_ = llt.matrixL();
    kronecker_ = true;
    ready_ = L_scalar_.allFinite();
    return ready_;
  }

  bool encodeInPlace(Eigen::VectorXd &x) const
  {
    Eigen::VectorXd chart;
    if (!extractSpatial(x, chart))
    {
      return false;
    }
    Eigen::VectorXd whitened;
    if (!toWhitened(chart, whitened))
    {
      return false;
    }
    x.segment(time_dim_, spatial_dim_) = whitened;
    return true;
  }

  bool decodeInPlace(Eigen::VectorXd &x) const
  {
    Eigen::VectorXd whitened;
    if (!extractSpatial(x, whitened))
    {
      return false;
    }
    Eigen::VectorXd chart;
    if (!toChart(whitened, chart))
    {
      return false;
    }
    x.segment(time_dim_, spatial_dim_) = chart;
    return true;
  }

  bool transformCovectorInPlace(Eigen::Ref<Eigen::VectorXd> grad) const
  {
    if (!ready_ || grad.size() < time_dim_ + spatial_dim_)
    {
      return false;
    }
    const Eigen::VectorXd chart_grad = grad.segment(time_dim_, spatial_dim_);
    Eigen::VectorXd whitened_grad;
    if (!transformCovector(chart_grad, whitened_grad))
    {
      return false;
    }
    grad.segment(time_dim_, spatial_dim_) = whitened_grad;
    return true;
  }

  bool toWhitened(const Eigen::VectorXd &chart, Eigen::VectorXd &whitened) const
  {
    if (!ready_ || chart.size() != spatial_dim_)
    {
      return false;
    }
    const Eigen::VectorXd delta = chart - y_seed_;
    return applyLt(delta, whitened);
  }

  bool toChart(const Eigen::VectorXd &whitened, Eigen::VectorXd &chart) const
  {
    if (!ready_ || whitened.size() != spatial_dim_)
    {
      return false;
    }
    Eigen::VectorXd delta;
    if (!applyInvLt(whitened, delta))
    {
      return false;
    }
    chart = y_seed_ + delta;
    return chart.allFinite();
  }

  bool transformCovector(const Eigen::VectorXd &chart_grad,
                         Eigen::VectorXd &whitened_grad) const
  {
    if (!ready_ || chart_grad.size() != spatial_dim_)
    {
      return false;
    }
    return applyInvL(chart_grad, whitened_grad);
  }

  bool transformDirectionToChart(const Eigen::VectorXd &whitened_dir,
                                 Eigen::VectorXd &chart_dir) const
  {
    if (!ready_ || whitened_dir.size() != spatial_dim_)
    {
      return false;
    }
    return applyInvLt(whitened_dir, chart_dir);
  }

private:
  bool extractSpatial(const Eigen::VectorXd &x, Eigen::VectorXd &spatial) const
  {
    if (!ready_ || x.size() < time_dim_ + spatial_dim_)
    {
      return false;
    }
    spatial = x.segment(time_dim_, spatial_dim_);
    return spatial.allFinite();
  }

  bool applyLt(const Eigen::VectorXd &v, Eigen::VectorXd &out) const
  {
    if (kronecker_)
    {
      return applyKroneckerLt(v, out);
    }
    out.noalias() = L_.transpose() * v;
    return out.allFinite();
  }

  bool applyInvLt(const Eigen::VectorXd &v, Eigen::VectorXd &out) const
  {
    if (kronecker_)
    {
      return applyKroneckerInvLt(v, out);
    }
    out = L_.transpose().triangularView<Eigen::Upper>().solve(v);
    return out.allFinite();
  }

  bool applyInvL(const Eigen::VectorXd &v, Eigen::VectorXd &out) const
  {
    if (kronecker_)
    {
      return applyKroneckerInvL(v, out);
    }
    out = L_.triangularView<Eigen::Lower>().solve(v);
    return out.allFinite();
  }

  Eigen::Map<const Eigen::MatrixXd> asPointMat(const Eigen::VectorXd &v) const
  {
    return Eigen::Map<const Eigen::MatrixXd>(v.data(), axis_dim_, inner_points_);
  }

  bool applyKroneckerLt(const Eigen::VectorXd &v, Eigen::VectorXd &out) const
  {
    // z = (L_s^T ⊗ I) v  <=>  Z = P * L_s
    out.resize(spatial_dim_);
    Eigen::Map<Eigen::MatrixXd> z_mat(out.data(), axis_dim_, inner_points_);
    z_mat.noalias() = asPointMat(v) * L_scalar_;
    return out.allFinite();
  }

  bool applyKroneckerInvLt(const Eigen::VectorXd &v, Eigen::VectorXd &out) const
  {
    // P = Z * L_s^{-T}
    out.resize(spatial_dim_);
    Eigen::Map<Eigen::MatrixXd> p_mat(out.data(), axis_dim_, inner_points_);
    p_mat = L_scalar_.transpose()
                .triangularView<Eigen::Upper>()
                .solve(asPointMat(v).transpose())
                .transpose();
    return out.allFinite();
  }

  bool applyKroneckerInvL(const Eigen::VectorXd &v, Eigen::VectorXd &out) const
  {
    // g_z = (L_s^{-1} ⊗ I) g_y  <=>  G_z = G_y * L_s^{-T}
    out.resize(spatial_dim_);
    Eigen::Map<Eigen::MatrixXd> g_z(out.data(), axis_dim_, inner_points_);
    g_z = L_scalar_.triangularView<Eigen::Lower>()
              .solve(asPointMat(v).transpose())
              .transpose();
    return out.allFinite();
  }

  bool ready_{false};
  bool kronecker_{false};
  int time_dim_{0};
  int spatial_dim_{0};
  int axis_dim_{3};
  int inner_points_{0};
  Eigen::VectorXd y_seed_;
  Eigen::MatrixXd L_;
  Eigen::MatrixXd L_scalar_;
};

} // namespace minco
