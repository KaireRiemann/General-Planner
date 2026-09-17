#pragma once

#include <cmath>
#include <limits>
#include <Eigen/Geometry>

namespace geometry_utils {

// Same thrust/heading frame as the Unity command bridge. Unlike yaw alone,
// acceleration and jerk rotate a body-fixed camera even during a straight chase.
struct TrackingAttitude {
    Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d heading, heading_perp, x, y, z;
    double thrust_norm{0.0}, cross_norm{0.0};
    bool valid{false};

    TrackingAttitude(const Eigen::Vector3d &acceleration, double yaw) {
        const Eigen::Vector3d thrust = acceleration + Eigen::Vector3d(0, 0, 9.81);
        thrust_norm = thrust.norm();
        if (!thrust.allFinite() || !std::isfinite(yaw) ||
            !std::isfinite(thrust_norm) || thrust_norm < 1.e-6) return;
        z = thrust / thrust_norm;
        heading = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0);
        heading_perp = Eigen::Vector3d(-std::sin(yaw), std::cos(yaw), 0);
        const Eigen::Vector3d cross = z.cross(heading);
        cross_norm = cross.norm();
        if (!std::isfinite(cross_norm) || cross_norm < 1.e-6) return;
        y = cross / cross_norm;
        x = y.cross(z);
        rotation.col(0) = x; rotation.col(1) = y; rotation.col(2) = z;
        valid = true;
    }

    void backward(Eigen::Vector3d gx, Eigen::Vector3d gy, Eigen::Vector3d gz,
                  Eigen::Vector3d gh, double gn, double gt,
                  Eigen::Vector3d &grad_acceleration, double &grad_yaw) const {
        gy += z.cross(gx);
        gz += gx.cross(y);
        const Eigen::Vector3d gq = (gy - y * y.dot(gy)) / cross_norm + gn * y;
        gz += heading.cross(gq);
        gh += gq.cross(z);
        grad_acceleration += (gz - z * z.dot(gz)) / thrust_norm + gt * z;
        grad_yaw += gh.dot(heading_perp);
    }

    void rotationGradient(const Eigen::Matrix3d &gradient,
                          Eigen::Vector3d &grad_acceleration, double &grad_yaw) const {
        backward(gradient.col(0), gradient.col(1), gradient.col(2),
                 Eigen::Vector3d::Zero(), 0.0, 0.0, grad_acceleration, grad_yaw);
    }

    // ||omega||^2 = ||z_dot||^2 + spin_about_z^2; gradients are optional.
    double bodyRateSquared(const Eigen::Vector3d &jerk, double yaw_rate,
                           Eigen::Vector3d *grad_acceleration = nullptr,
                           Eigen::Vector3d *grad_jerk = nullptr,
                           double *grad_yaw = nullptr, double *grad_yaw_rate = nullptr) const {
        if (!valid || !jerk.allFinite() || !std::isfinite(yaw_rate))
            return std::numeric_limits<double>::infinity();
        const double projection = z.dot(jerk);
        const Eigen::Vector3d zd = (jerk - z * projection) / thrust_norm;
        const Eigen::Vector3d hd = yaw_rate * heading_perp;
        const Eigen::Vector3d qd = zd.cross(heading) + z.cross(hd);
        const double spin = -x.dot(qd) / cross_norm;
        if (grad_acceleration && grad_jerk && grad_yaw && grad_yaw_rate) {
            const Eigen::Vector3d gx = -2.0 * spin * qd / cross_norm;
            const Eigen::Vector3d gqd = -2.0 * spin * x / cross_norm;
            const double gn = 2.0 * spin * x.dot(qd) / (cross_norm * cross_norm);
            const Eigen::Vector3d gzd = 2.0 * zd + heading.cross(gqd);
            Eigen::Vector3d gz = hd.cross(gqd);
            const Eigen::Vector3d gh = gqd.cross(zd);
            const Eigen::Vector3d ghd = gqd.cross(z);
            *grad_yaw_rate += ghd.dot(heading_perp);
            *grad_yaw -= yaw_rate * ghd.dot(heading);
            const double gprojection = -gzd.dot(z) / thrust_norm;
            gz += -projection * gzd / thrust_norm + gprojection * jerk;
            *grad_jerk += gzd / thrust_norm + gprojection * z;
            const double gt = -gzd.dot(zd) / thrust_norm;
            backward(gx, Eigen::Vector3d::Zero(), gz, gh, gn, gt,
                     *grad_acceleration, *grad_yaw);
        }
        return zd.squaredNorm() + spin * spin;
    }
};

} // namespace geometry_utils
