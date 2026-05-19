#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

namespace traj_opt {

struct SE3FlatnessOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d z_b{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d omega{Eigen::Vector3d::Zero()};
  double thrust{0.0};
  bool valid{false};
};

class SE3FlatnessMap {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setYawMode(bool use_yaw, bool heading_to_velocity) {
    use_yaw_ = use_yaw;
    heading_to_velocity_ = heading_to_velocity;
  }

  bool forward(const Eigen::Vector3d &vel,
               const Eigen::Vector3d &acc,
               const Eigen::Vector3d &jerk,
               const Eigen::Vector3d &snap,
               double yaw,
               double yaw_rate,
               double gravity,
               SE3FlatnessOutput &out) const {
    (void)snap;
    out = SE3FlatnessOutput();

    const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d u = acc + gravity * e3;
    const double thrust = u.norm();
    if (!std::isfinite(thrust) || thrust < eps_) {
      return false;
    }

    const Eigen::Vector3d z_b = u / thrust;
    Eigen::Vector3d x_c = Eigen::Vector3d::UnitX();
    const bool yaw_enabled = use_yaw_ && std::isfinite(yaw);
    if (yaw_enabled) {
      x_c = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
    } else if (heading_to_velocity_) {
      Eigen::Vector3d heading(vel.x(), vel.y(), 0.0);
      if (heading.norm() > eps_) {
        x_c = heading.normalized();
      }
    }

    Eigen::Vector3d y_b = z_b.cross(x_c);
    if (y_b.norm() < eps_) {
      Eigen::Vector3d fallback = std::abs(z_b.z()) < 0.9
                                     ? Eigen::Vector3d::UnitZ()
                                     : Eigen::Vector3d::UnitY();
      y_b = z_b.cross(fallback);
    }
    if (y_b.norm() < eps_) {
      return false;
    }
    y_b.normalize();
    const Eigen::Vector3d x_b = y_b.cross(z_b).normalized();

    out.R.col(0) = x_b;
    out.R.col(1) = y_b;
    out.R.col(2) = z_b;
    out.z_b = z_b;
    out.thrust = thrust;

    const Eigen::Vector3d b = out.R.transpose() * jerk / thrust;
    out.omega = Eigen::Vector3d(-b.y(), b.x(), yaw_enabled ? yaw_rate : 0.0);
    out.valid = out.R.allFinite() && out.omega.allFinite() && std::isfinite(out.thrust);
    return out.valid;
  }

  double thrustSquared(const Eigen::Vector3d &acc, double gravity) const {
    const Eigen::Vector3d u = acc + gravity * Eigen::Vector3d::UnitZ();
    return u.squaredNorm();
  }

  double bodyRateSquared(const Eigen::Vector3d &vel,
                         const Eigen::Vector3d &acc,
                         const Eigen::Vector3d &jerk,
                         const Eigen::Vector3d &snap,
                         double yaw,
                         double yaw_rate,
                         double gravity) const {
    SE3FlatnessOutput out;
    if (!forward(vel, acc, jerk, snap, yaw, yaw_rate, gravity, out)) {
      return std::numeric_limits<double>::infinity();
    }
    return out.omega.squaredNorm();
  }

private:
  bool use_yaw_{false};
  bool heading_to_velocity_{true};
  double eps_{1.0e-8};
};

} // namespace traj_opt
